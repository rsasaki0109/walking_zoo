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
from gait_lab.rl_env import ACT_DIM, OBS_DIM, STEER_OBS_DIM, G1WalkEnv
from gait_lab.controllers import Command

POLICY_PATH = Path(__file__).parent / "gait_lab" / "rl_policy.npz"
STEER_POLICY_PATH = Path(__file__).parent / "gait_lab" / "rl_policy_steer.npz"
STEER_FS_POLICY_PATH = Path(__file__).parent / "gait_lab" / "rl_policy_steer_fs.npz"


# -- numpy policy used inside the (torch-free) worker processes ----------------
def _mlp_np(Ws, bs, x):
    h = x
    for i, (W, b) in enumerate(zip(Ws, bs)):
        h = W @ h + b
        if i < len(Ws) - 1:
            h = np.tanh(h)
    return h


def load_actor_from_npz(net: ActorCritic, norm: RunningNorm, path: str):
    """Warm-start the actor (+ obs normaliser) from an exported policy npz.

    The headline use is *cross-dimension* curriculum: load the proven straight
    ``rl_policy.npz`` (34-dim obs, walks the full horizon) into the 36-dim
    steerable network. The two extra inputs are the command ``(forward_speed,
    yaw_rate)``; their first-layer weight columns are initialised to ZERO, so the
    warm-started policy starts out *exactly* the straight 8 s walker (the command
    has no effect yet) and learns to act on the command as a perturbation from a
    skill that already walks — instead of discovering walking and steering at once
    from scratch (which does not converge here). The normaliser is restored and
    padded for the new command dims. The export stores only the actor, so the
    critic restarts (it re-fits quickly).
    """
    import torch.nn as nn

    d = np.load(path)
    n = int(d["n_layers"][0])
    linears = [m for m in net.actor if isinstance(m, nn.Linear)]
    if len(linears) != n:
        raise ValueError(f"actor layer count {len(linears)} != checkpoint {n}")
    ck_in = int(d["W0"].shape[1])
    net_in = linears[0].weight.shape[1]
    pad = net_in - ck_in
    if pad < 0:
        raise ValueError(f"checkpoint obs dim {ck_in} > net obs dim {net_in}")
    with torch.no_grad():
        for i, lin in enumerate(linears):
            w = np.asarray(d[f"W{i}"], dtype=np.float32)
            b = np.asarray(d[f"b{i}"], dtype=np.float32)
            if i == 0 and pad > 0:
                # Pad the extra input columns (the command) with zeros so they
                # have no effect at warm-start.
                w = np.concatenate([w, np.zeros((w.shape[0], pad), np.float32)], axis=1)
            if w.shape != tuple(lin.weight.shape) or b.shape != tuple(lin.bias.shape):
                raise ValueError(
                    f"layer {i} shape mismatch: ckpt {w.shape}/{b.shape} vs net "
                    f"{tuple(lin.weight.shape)}/{tuple(lin.bias.shape)}")
            lin.weight.copy_(torch.as_tensor(w))
            lin.bias.copy_(torch.as_tensor(b))
    mean = d["obs_mean"].astype(np.float64)
    std = d["obs_std"].astype(np.float64)
    if pad > 0:
        # Pad normaliser for the command dims: a neutral mid-range mean and a
        # unit-ish std so the (initially-ignored) command inputs are sanely scaled
        # once their weights start to grow.
        mean = np.concatenate([mean, np.zeros(pad)])
        std = np.concatenate([std, np.ones(pad)])
    norm.mean = mean
    # The exported std already baked in the +1e-8 of sqrt(var+1e-8); subtract it
    # back out so a re-export reproduces the SAME std bit-for-bit (otherwise the
    # epsilon is applied twice, perturbing normalisation of small-variance dims —
    # enough to turn the chaotic walker's +0.7 m walk into in-place stepping).
    norm.var = np.maximum(std ** 2 - 1e-8, 0.0)
    # Make the restored normaliser STICKY: the checkpoint's normaliser came from a
    # long training run, and the warm-started actor's behaviour depends on being
    # fed obs normalised exactly that way. A small count lets the first few
    # iterations' fresh statistics (tens of thousands of samples) swamp it, which
    # silently shifts normalisation and breaks the inherited walk (vx collapses to
    # ~0). A large count keeps it essentially fixed while the policy fine-tunes.
    norm.count = 1.0e6
    print(f"warm-started actor (+{pad} zero-padded command inputs) + normaliser "
          f"from {path}")


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


# Command grid the steerable eval must survive AND track: stand, straight at a
# few speeds, and turning each way while walking.
STEER_EVAL_CMDS = [
    Command(0.18, 0.0),    # brisk straight walk
    Command(0.15, 0.3),    # arc left
    Command(0.15, -0.3),   # arc right
    Command(0.0, 0.3),     # turn in place, left
    Command(0.0, -0.3),    # turn in place, right
]


def evaluate_steerable(net, norm, eval_env, device, cmds=STEER_EVAL_CMDS):
    """Steerable eval: roll out over a command grid and measure survival AND
    tracking. Returns (score, mean_survival, vx_err, wz_err) where score folds
    tracking into survival so the saved policy both stays up under every command
    and follows it. Tracking is averaged over the upright portion of each rollout.
    """
    std = np.sqrt(norm.var + 1e-8)

    def run(cmd, seed):
        o = eval_env.reset(seed=seed, cmd=cmd)
        done = False
        vx_e, wz_e, n = 0.0, 0.0, 0
        surv = 0.0
        while not done:
            on = (o - norm.mean) / std
            with torch.no_grad():
                mean = net.actor(torch.as_tensor(on, dtype=torch.float32, device=device))
            res = eval_env.step(mean.cpu().numpy())
            o = res.obs
            done = res.done
            surv = res.info["t"]
            if res.info["t"] > 0.5:    # ignore the initial settle
                vx_e += abs(res.info["vx"] - cmd.forward_speed)
                wz_e += abs(res.info["wz"] - cmd.yaw_rate)
                n += 1
        return surv, (vx_e / max(n, 1)), (wz_e / max(n, 1))

    survs, vxs, wzs = [], [], []
    for cmd in cmds:
        s, vx_e, wz_e = run(cmd, None)
        survs.append(s); vxs.append(vx_e); wzs.append(wz_e)
    msurv = float(np.mean(survs))
    vx_err = float(np.mean(vxs))
    wz_err = float(np.mean(wzs))
    # Survival dominates (seconds, ~8 scale). Yaw tracking is the finely-tracked
    # command, so it is weighted more than the forward-speed error (forward is
    # GO/HOLD, not a finely tracked speed, so a brisk-walk vx gap is expected and
    # shouldn't dominate the save metric).
    score = msurv - 0.5 * vx_err - 2.0 * wz_err
    return score, msurv, vx_err, wz_err


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
    ap.add_argument("--steerable", action="store_true",
                    help="train a command-conditioned (velocity + yaw) policy "
                         "instead of the fixed straight walker")
    ap.add_argument("--speed-max", type=float, default=0.25,
                    help="max sampled forward_speed for steerable training (m/s)")
    ap.add_argument("--yaw-max", type=float, default=0.4,
                    help="max |sampled yaw_rate| for steerable training (rad/s); "
                         "0 trains a speed-only (no-turn) policy")
    ap.add_argument("--hold-prob", type=float, default=0.25,
                    help="fraction of steerable episodes commanded to hold (vx=0)")
    ap.add_argument("--footstep", action="store_true",
                    help="train the steerable residual on the FOOTSTEP substrate "
                         "(SteerableFootstepGait, which actually steers) instead of "
                         "the CPG; learns to stabilise the steering footstep base")
    ap.add_argument("--init-policy", default=None,
                    help="warm-start the actor + normaliser from this exported npz "
                         "(curriculum: e.g. forward-only -> add turning)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3,
                    help="iters to populate the obs normaliser before any PPO update "
                         "(prevents a destructive first update on un-normalised obs)")
    ap.add_argument("--critic-warmup", type=int, default=0,
                    help="iters of value-only (actor-frozen) updates after the norm "
                         "warmup; protects a --init-policy warm-started actor from a "
                         "random critic's garbage early advantages")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--out", default=str(POLICY_PATH))
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env_kwargs = dict(
        horizon=args.horizon, perturb_scale=args.perturb,
        push_interval=args.push_interval, push_speed=args.push_speed,
        steerable=args.steerable,
        footstep=args.footstep,
        speed_range=(0.0, args.speed_max),
        yaw_range=(-args.yaw_max, args.yaw_max),
        hold_prob=args.hold_prob)
    obs_dim = STEER_OBS_DIM if args.steerable else OBS_DIM
    net = ActorCritic(obs_dim, ACT_DIM).to(args.device)
    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    norm = RunningNorm(obs_dim)
    if args.init_policy:
        load_actor_from_npz(net, norm, args.init_policy)

    # Default the output to the steerable policy file when training steerable,
    # unless the user passed an explicit --out.
    out_path = args.out
    if args.steerable and out_path == str(POLICY_PATH):
        out_path = str(STEER_FS_POLICY_PATH if args.footstep else STEER_POLICY_PATH)

    n_workers = max(1, args.workers)
    per_worker = max(1, args.steps // n_workers)
    vec = VecCollector(n_workers, env_kwargs, args.seed + 1)
    # Eval env always perturbs (regardless of the training perturb scale) so the
    # robust eval below actually exercises perturbed starts.
    eval_kwargs = dict(env_kwargs); eval_kwargs["perturb_scale"] = 0.015
    eval_env = G1WalkEnv(**eval_kwargs)

    best_eval = -1e9
    print(f"device={args.device}  workers={n_workers}x{per_worker}steps  "
          f"horizon={args.horizon}s  "
          f"{'steerable (track vx+yaw)' if args.steerable else 'ceiling-to-beat~3.1s'}")
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
            # Value-only updates while the critic warms up, so a warm-started actor
            # is not destroyed by a random critic's garbage advantages.
            pg_coef = 0.0 if it <= args.warmup + args.critic_warmup else 1.0
            ppo_update(net, opt, up, pg_coef=pg_coef, device=args.device)
            if it % 5 == 0 or it == 1:
                msurv = float(np.mean(ep_surv)) if ep_surv else 0.0
                if args.steerable:
                    # Steerable: score over the command grid (survival + tracking),
                    # save the best so the policy both stays up under every command
                    # and follows it.
                    score, ms, vxe, wze = evaluate_steerable(
                        net, norm, eval_env, args.device)
                    if score > best_eval:
                        best_eval = score
                        np.savez(out_path, **export_numpy(net, norm))
                    print(f"it {it:4d}  eval surv={ms:4.2f}s  vx_err={vxe:.3f} "
                          f"wz_err={wze:.3f}  score={score:5.2f}  "
                          f"train mean={msurv:4.2f}  best={best_eval:5.2f}", flush=True)
                else:
                    # Robust deterministic eval = what RLResidualWalk actually does,
                    # scored as worst-case over perturbed starts. Save on this so the
                    # *latest, most robust* policy wins (not the first lucky 8 s one).
                    robust_t, nominal_t = evaluate(net, norm, eval_env, args.device)
                    if robust_t > best_eval:
                        best_eval = robust_t
                        np.savez(out_path, **export_numpy(net, norm))
                    print(f"it {it:4d}  eval nom={nominal_t:4.2f}s robust={robust_t:4.2f}s  "
                          f"train mean={msurv:4.2f}  best_robust={best_eval:4.2f}", flush=True)
    finally:
        vec.close()
    print(f"\nbest eval score {best_eval:.2f}  ->  saved {out_path}")


if __name__ == "__main__":
    main()
