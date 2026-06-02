"""A small, self-contained PPO for the residual-gait env — no SB3, no RLlib.

Just enough PPO to answer one question honestly: can a learned closed-loop
policy break the ~3 s lateral ceiling that every hand-tuned / model-based gait in
this lab hits? The actor-critic is two small MLPs (torch, runs on GPU); the
collected rollouts come from :class:`gait_lab.rl_env.G1WalkEnv` on CPU.

The trained actor is exported to plain numpy arrays (``export_numpy``) so that
:class:`~gait_lab.controllers.RLResidualWalk` can run it at inference with no
torch dependency — the same dependency-free-inference convention the linear
``learned-feedback`` policy already follows.
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn


class RunningNorm:
    """Welford running mean/variance for observation normalisation."""

    def __init__(self, dim: int):
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4

    def update(self, x: np.ndarray) -> None:
        x = np.atleast_2d(x)
        b_mean = x.mean(0)
        b_var = x.var(0)
        b_n = x.shape[0]
        delta = b_mean - self.mean
        tot = self.count + b_n
        self.mean += delta * b_n / tot
        m_a = self.var * self.count
        m_b = b_var * b_n
        self.var = (m_a + m_b + delta ** 2 * self.count * b_n / tot) / tot
        self.count = tot

    def normalize(self, x: np.ndarray) -> np.ndarray:
        return (x - self.mean) / np.sqrt(self.var + 1e-8)


def _mlp(sizes, act=nn.Tanh, out_act=nn.Identity):
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        layers.append(act() if i < len(sizes) - 2 else out_act())
    return nn.Sequential(*layers)


def _orthogonal_init(seq, gains):
    """Orthogonal init per Linear layer with the given output gains (PPO default)."""
    linears = [m for m in seq if isinstance(m, nn.Linear)]
    for lin, g in zip(linears, gains):
        nn.init.orthogonal_(lin.weight, g)
        nn.init.zeros_(lin.bias)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 128):
        super().__init__()
        self.actor = _mlp([obs_dim, hidden, hidden, act_dim])
        self.critic = _mlp([obs_dim, hidden, hidden, 1])
        # Crucial for *residual* learning: the final actor layer is initialised
        # near zero (gain 0.01) so the policy starts emitting ~0 residual — i.e.
        # it begins AT the clean BalancedCPG feedforward (which already survives
        # ~3 s) and improves from there, rather than from a degraded random
        # offset. Hidden layers use the sqrt(2) ReLU/Tanh gain; the critic is
        # standard.
        _orthogonal_init(self.actor, [np.sqrt(2), np.sqrt(2), 0.01])
        _orthogonal_init(self.critic, [np.sqrt(2), np.sqrt(2), 1.0])
        # Start exploration gentle so early noise does not topple the gait.
        self.log_std = nn.Parameter(-1.8 * torch.ones(act_dim))

    def dist(self, obs):
        mean = self.actor(obs)
        std = torch.exp(self.log_std)
        return torch.distributions.Normal(mean, std)

    def act(self, obs):
        d = self.dist(obs)
        a = d.sample()
        return a, d.log_prob(a).sum(-1), self.critic(obs).squeeze(-1)

    def evaluate(self, obs, act):
        d = self.dist(obs)
        return (
            d.log_prob(act).sum(-1),
            d.entropy().sum(-1),
            self.critic(obs).squeeze(-1),
        )


def compute_gae(rewards, values, terminals, last_value, gamma=0.99, lam=0.95):
    """GAE-lambda. ``terminals[t]`` true = true terminal (no bootstrap)."""
    n = len(rewards)
    adv = np.zeros(n)
    gae = 0.0
    for t in reversed(range(n)):
        next_v = last_value if t == n - 1 else values[t + 1]
        nonterminal = 0.0 if terminals[t] else 1.0
        delta = rewards[t] + gamma * next_v * nonterminal - values[t]
        gae = delta + gamma * lam * nonterminal * gae
        adv[t] = gae
    return adv, adv + values


def ppo_update(net, opt, batch, *, clip=0.2, epochs=10, minibatch=512,
               vf_coef=0.5, ent_coef=0.0, device="cpu"):
    obs = torch.as_tensor(batch["obs"], dtype=torch.float32, device=device)
    act = torch.as_tensor(batch["act"], dtype=torch.float32, device=device)
    old_lp = torch.as_tensor(batch["logp"], dtype=torch.float32, device=device)
    ret = torch.as_tensor(batch["ret"], dtype=torch.float32, device=device)
    adv = torch.as_tensor(batch["adv"], dtype=torch.float32, device=device)
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)

    n = obs.shape[0]
    idx = np.arange(n)
    for _ in range(epochs):
        np.random.shuffle(idx)
        for s in range(0, n, minibatch):
            mb = idx[s:s + minibatch]
            lp, ent, val = net.evaluate(obs[mb], act[mb])
            ratio = torch.exp(lp - old_lp[mb])
            a = adv[mb]
            pg = -torch.min(ratio * a, torch.clamp(ratio, 1 - clip, 1 + clip) * a).mean()
            vloss = ((val - ret[mb]) ** 2).mean()
            loss = pg + vf_coef * vloss - ent_coef * ent.mean()
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(net.parameters(), 0.5)
            opt.step()


def export_numpy(net: ActorCritic, norm: RunningNorm) -> dict:
    """Export the actor + obs-normaliser to plain numpy arrays for inference."""
    sd = net.state_dict()
    layers = [k for k in sd if k.startswith("actor.") and k.endswith(".weight")]
    out = {"n_layers": np.array([len(layers)])}
    for i, k in enumerate(sorted(layers, key=lambda s: int(s.split(".")[1]))):
        out[f"W{i}"] = sd[k].cpu().numpy()
        out[f"b{i}"] = sd[k.replace("weight", "bias")].cpu().numpy()
    out["obs_mean"] = norm.mean.copy()
    out["obs_std"] = np.sqrt(norm.var + 1e-8)
    return out
