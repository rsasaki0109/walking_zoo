"""Train the residual-gait policy with the self-contained PPO.

    python3 train_rl.py --iters 400 --steps 4096 --workers 12

Collects rollouts from :class:`gait_lab.rl_env.G1WalkEnv` (BalancedCPG rhythm +
learned leg residual), runs PPO (:mod:`gait_lab.ppo`), and saves the best actor
as plain numpy to ``gait_lab/rl_policy.npz`` for :class:`RLResidualWalk`.

The question it answers: every hand-tuned / model-based gait here tops out near
the ~3 s lateral ceiling (``stability_ceiling.py``). Does a *learned* closed-loop
residual break it? Training survival climbing past 3 s toward the 8 s horizon is
the answer.

Rollout collection is CPU-bound on ``mj_step``; ``--workers N`` runs N persistent
worker processes, each stepping its own G1 with a numpy copy of the current
policy, so a single GPU learner is fed in parallel (near-linear speedup).
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
from pathlib import Path

import numpy as np
import torch

from gait_lab.ppo import ActorCritic, RunningNorm, compute_gae, export_numpy, ppo_update
from gait_lab.rl_env import ACT_DIM, OBS_DIM, G1WalkEnv

POLICY_PATH = Path(__file__).parent / "gait_lab" / "rl_policy.npz"


# -- numpy policy used inside the (torch-free) worker processes ----------------
def _mlp_np(Ws, bs, x):
    h = x
    for i, (W, b) in enumerate(zip(Ws, bs)):
        h = W @ h + b
        if i < len(Ws) - 1:
            h = np.tanh(h)
    return h


def to_numpy(net: ActorCritic):
    sd = net.state_dict()

    def layers(prefix):
        ks = sorted([k for k in sd if k.startswith(prefix) and k.endswith(".weight")],
                    key=lambda s: int(s.split(".")[1]))
        return ([sd[k].cpu().numpy() for k in ks],
                [sd[k.replace("weight", "bias")].cpu().numpy() for k in ks])

    aW, ab = layers("actor.")
    cW, cb = layers("critic.")
    return aW, ab, cW, cb, net.log_std.detach().cpu().numpy()


# -- worker process ------------------------------------------------------------
def _worker(conn, env_kwargs, seed):
    env = G1WalkEnv(**env_kwargs)
    rng = np.random.default_rng(seed)
    o = env.reset(seed=int(rng.integers(1 << 30)))
    cur_ret = 0.0
    while True:
        msg = conn.recv()
        if msg is None:
            break
        aW, ab, cW, cb, log_std, om, os_, nsteps = msg
        std = np.exp(log_std)
        OB, AC, LP, RW, VL, TM = [], [], [], [], [], []
        surv, rets = [], []
        for _ in range(nsteps):
            on = (o - om) / os_
            mean = _mlp_np(aW, ab, on)
            val = float(_mlp_np(cW, cb, on)[0])
            noise = rng.standard_normal(mean.shape[0])
            a = mean + std * noise
            logp = float(np.sum(-0.5 * noise ** 2 - log_std - 0.5 * np.log(2 * np.pi)))
            res = env.step(a)
            OB.append(o.copy()); AC.append(a); LP.append(logp)
            RW.append(res.reward); VL.append(val); TM.append(res.info["fell"])
            cur_ret += res.reward
            o = res.obs
            if res.done:
                surv.append(res.info["t"]); rets.append(cur_ret); cur_ret = 0.0
                o = env.reset(seed=int(rng.integers(1 << 30)))
        last_v = float(_mlp_np(cW, cb, (o - om) / os_)[0])
        conn.send((np.array(OB), np.array(AC), np.array(LP), np.array(RW),
                   np.array(VL), np.array(TM), last_v, surv, rets))
    env_kwargs.clear()
    conn.close()


class VecCollector:
    """Persistent worker pool feeding one GPU learner."""

    def __init__(self, n_workers, env_kwargs, seed):
        ctx = mp.get_context("fork")
        self.parents = []
        self.procs = []
        for i in range(n_workers):
            p_conn, c_conn = ctx.Pipe()
            proc = ctx.Process(target=_worker, args=(c_conn, dict(env_kwargs), seed + i))
            proc.daemon = True
            proc.start()
            self.parents.append(p_conn)
            self.procs.append(proc)

    def collect(self, net, norm, steps_per_worker):
        aW, ab, cW, cb, log_std = to_numpy(net)
        msg = (aW, ab, cW, cb, log_std, norm.mean, np.sqrt(norm.var + 1e-8), steps_per_worker)
        for c in self.parents:
            c.send(msg)
        obs, act, adv, ret = [], [], [], []
        ep_surv, ep_ret = [], []
        for c in self.parents:
            OB, AC, LP, RW, VL, TM, last_v, surv, rets = c.recv()
            a, r = compute_gae(RW, VL, TM, last_v)
            obs.append(OB); act.append(AC)
            adv.append(a); ret.append(r)
            # logp recomputed by the learner against normalised obs at update time
            ep_surv += surv; ep_ret += rets
        OBS = np.concatenate(obs)
        norm.update(OBS)  # for the NEXT iteration's normalisation
        batch = {
            "obs_raw": OBS, "act": np.concatenate(act),
            "adv": np.concatenate(adv), "ret": np.concatenate(ret),
        }
        return batch, ep_surv, ep_ret

    def close(self):
        for c in self.parents:
            try:
                c.send(None)
            except Exception:
                pass
        for p in self.procs:
            p.join(timeout=2)
            if p.is_alive():
                p.terminate()


def evaluate(net, norm, eval_env, device,
             perturb_seeds=(2001, 2002, 2003, 2004, 2005)):
    """Deterministic (mean-action) rollouts from the nominal start *and* several
    perturbed ones. Returns (robust, nominal) survival, where robust = the worst
    case. Saving on the robust score (not a single nominal rollout) is the chaos
    lesson from ``learned-feedback``: a lone lucky 8 s rollout is a fluke. Using
    *several* perturbed seeds (not one or two) matters just as much — a policy
    that handles only two fixed seeds has overfit to them; worst-of-many forces
    genuine robustness. These seeds are disjoint from the final eval (0..7) so
    the reported number is not leaked from the save metric.
    """
    std = np.sqrt(norm.var + 1e-8)

    def run(seed):
        o = eval_env.reset(seed=seed)
        done = False
        while not done:
            on = (o - norm.mean) / std
            with torch.no_grad():
                mean = net.actor(torch.as_tensor(on, dtype=torch.float32, device=device))
            res = eval_env.step(mean.cpu().numpy())
            o = res.obs
            done = res.done
        return res.info["t"]

    nominal = run(None)
    worst = min([nominal] + [run(s) for s in perturb_seeds])
    return worst, nominal


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--steps", type=int, default=4096, help="total control steps per iter")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--perturb", type=float, default=0.0,
                    help="domain-randomisation perturbation scale (0 = nominal start)")
    ap.add_argument("--push-interval", type=float, default=0.0,
                    help="mean seconds between mid-episode shoves (0 = no pushes)")
    ap.add_argument("--push-speed", type=float, default=0.0,
                    help="velocity-kick magnitude per shove (m/s)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3,
                    help="iters to populate the obs normaliser before any PPO update "
                         "(prevents a destructive first update on un-normalised obs)")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--out", default=str(POLICY_PATH))
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env_kwargs = dict(
        horizon=args.horizon, perturb_scale=args.perturb,
        push_interval=args.push_interval, push_speed=args.push_speed)
    net = ActorCritic(OBS_DIM, ACT_DIM).to(args.device)
    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    norm = RunningNorm(OBS_DIM)

    n_workers = max(1, args.workers)
    per_worker = max(1, args.steps // n_workers)
    vec = VecCollector(n_workers, env_kwargs, args.seed + 1)
    # Eval env always perturbs (regardless of the training perturb scale) so the
    # robust eval below actually exercises perturbed starts.
    eval_kwargs = dict(env_kwargs); eval_kwargs["perturb_scale"] = 0.015
    eval_env = G1WalkEnv(**eval_kwargs)

    best_eval = 0.0
    print(f"device={args.device}  workers={n_workers}x{per_worker}steps  "
          f"horizon={args.horizon}s  ceiling-to-beat~3.1s")
    try:
        for it in range(1, args.iters + 1):
            # Snapshot the normaliser the workers will use, so the learner
            # normalises identically (collect() updates norm afterwards, for the
            # next iter). Keeps old_logp consistent with the behaviour policy.
            pre_mean = norm.mean.copy()
            pre_std = np.sqrt(norm.var + 1e-8)
            batch, ep_surv, ep_ret = vec.collect(net, norm, per_worker)
            if it <= args.warmup:
                # Only populate the normaliser (done inside collect); skip the
                # update so the good near-CPG init is not wrecked by a gradient
                # step on still-un-normalised observations.
                continue
            obs_n = (batch["obs_raw"] - pre_mean) / pre_std
            with torch.no_grad():
                o_t = torch.as_tensor(obs_n, dtype=torch.float32, device=args.device)
                a_t = torch.as_tensor(batch["act"], dtype=torch.float32, device=args.device)
                lp, _, _ = net.evaluate(o_t, a_t)
            up = {"obs": obs_n, "act": batch["act"], "logp": lp.cpu().numpy(),
                  "adv": batch["adv"], "ret": batch["ret"]}
            ppo_update(net, opt, up, device=args.device)
            if it % 5 == 0 or it == 1:
                # Robust deterministic eval = what RLResidualWalk actually does,
                # scored as worst-case over perturbed starts. Save on this so the
                # *latest, most robust* policy wins (not the first lucky 8 s one).
                robust_t, nominal_t = evaluate(net, norm, eval_env, args.device)
                if robust_t > best_eval:
                    best_eval = robust_t
                    np.savez(args.out, **export_numpy(net, norm))
                msurv = float(np.mean(ep_surv)) if ep_surv else 0.0
                print(f"it {it:4d}  eval nom={nominal_t:4.2f}s robust={robust_t:4.2f}s  "
                      f"train mean={msurv:4.2f}  best_robust={best_eval:4.2f}", flush=True)
    finally:
        vec.close()
    print(f"\nbest robust survival {best_eval:.2f}s  ->  saved {args.out}")


if __name__ == "__main__":
    main()
