"""Robustly evaluate the trained RL residual policy.

Training saves the policy with the best *single* deterministic rollout — and a
falling humanoid is chaotic, so a single lucky 8 s rollout can be a fluke (the
same trap documented for ``learned-feedback`` in the README). This script is the
honest check: it rolls ``RLResidualWalk`` out from the nominal start *and* from
several perturbed initial states (small base tilt + joint jitter), and reports
the spread. A genuinely robust policy survives them all; a fluke does not.

    python3 eval_policy.py --seeds 8
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model, GaitHarness, RLResidualWalk
from gait_lab.controllers import BalancedCPG


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8, help="number of perturbed starts")
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--perturb", type=float, default=0.015)
    ap.add_argument("--push-speeds", type=float, nargs="*",
                    default=[0.3, 0.5, 0.7],
                    help="velocity-kick magnitudes (m/s) for the push-recovery sweep")
    ap.add_argument("--push-trials", type=int, default=5,
                    help="rollouts per push magnitude (different shove schedules)")
    ap.add_argument("--policy", default=None,
                    help="path to an rl_policy.npz (default: the shipped one)")
    args = ap.parse_args()

    def make():
        return RLResidualWalk(args.policy)

    model = G1Model()
    harness = GaitHarness(model, horizon=args.horizon)

    nominal, _ = harness.rollout(make(), render=False)
    cpg, _ = harness.rollout(BalancedCPG(), render=False)
    print(f"horizon {args.horizon:.0f}s   (balanced-cpg baseline: "
          f"survive {cpg.survival_time:.2f}s, kinematic ceiling ~3 s)\n")
    print(f"  nominal start   survive {nominal.survival_time:4.2f}s  "
          f"forward {nominal.forward_distance:+.2f}m  fell={nominal.fell}")

    survs, fwds = [nominal.survival_time], [nominal.forward_distance]
    for seed in range(args.seeds):
        m, _ = harness.rollout(make(), render=False,
                               perturb_seed=seed, perturb_scale=args.perturb)
        survs.append(m.survival_time); fwds.append(m.forward_distance)
        print(f"  perturb seed {seed:2d}  survive {m.survival_time:4.2f}s  "
              f"forward {m.forward_distance:+.2f}m  fell={m.fell}")

    survs = np.array(survs)
    print(f"\n  survival  mean {survs.mean():4.2f}s  min {survs.min():4.2f}s  "
          f"max {survs.max():4.2f}s  ({(survs >= args.horizon - 0.1).sum()}/"
          f"{len(survs)} reached the full horizon)")
    print(f"  forward   mean {np.mean(fwds):+.2f}m")
    verdict = "ROBUST" if survs.min() > 3.1 else "FLAKY (a fluke, like the chaos caveat)"
    print(f"  verdict: {verdict} — worst-case survival {survs.min():.2f}s "
          f"vs kinematic ceiling ~3 s")

    # Push-recovery sweep: shove the robot mid-walk and see how long it stays up.
    if args.push_speeds:
        print("\n  push-recovery (mid-walk velocity kicks):")
        for speed in args.push_speeds:
            ps = []
            for trial in range(args.push_trials):
                m, _ = harness.rollout(
                    make(), render=False, perturb_seed=trial,
                    perturb_scale=args.perturb, push_speed=speed,
                    push_interval=1.5, push_seed=trial)
                ps.append(m.survival_time)
            ps = np.array(ps)
            print(f"    shove {speed:.1f} m/s  survive mean {ps.mean():4.2f}s "
                  f"min {ps.min():4.2f}s  ({(ps >= args.horizon - 0.1).sum()}/"
                  f"{len(ps)} full horizon)")


if __name__ == "__main__":
    main()
