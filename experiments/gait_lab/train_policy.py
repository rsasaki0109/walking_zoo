#!/usr/bin/env python3
"""Train the LearnedFeedbackWalk linear feedback policy with the Cross-Entropy
Method — *robustly*.

The lesson baked into this script: a falling humanoid is chaotic, so scoring a
candidate on a single nominal rollout overfits. An early, naive version of this
search found a policy that "survived 3.4 s" — which collapsed to 1.8 s the moment
its weights were rounded to four decimals. The fix is to score each candidate on
the **worst** of several perturbed initial states (`--seeds`), so only genuinely
robust feedback wins.

    python3 train_policy.py                       # robust CEM, prints best weights
    python3 train_policy.py --seeds 5 --iters 16  # more robustness / more search

Paste the printed weights into controllers.LEARNED_FEEDBACK_WEIGHTS. Deterministic
for fixed --seed (CEM sampling) and --seeds (perturbations).
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from gait_lab import GaitHarness, G1Model
from gait_lab.controllers import LearnedFeedbackWalk


def robust_score(harness, weights, perturb_seeds, horizon):
    """Worst-case sustained-walking score over several perturbed starts."""
    scores, survs, fwds = [], [], []
    for ps in perturb_seeds:
        m, _ = harness.rollout(
            LearnedFeedbackWalk(list(weights)), render=False, perturb_seed=ps
        )
        scores.append(m.forward_distance * (m.survival_time / horizon)
                      + 0.05 * m.survival_time)
        survs.append(m.survival_time)
        fwds.append(m.forward_distance)
    return min(scores), (float(np.mean(survs)), float(np.mean(fwds)), min(survs))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--iters", type=int, default=14)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--elite", type=int, default=6)
    ap.add_argument("--seeds", type=int, default=3, help="perturbed starts per candidate")
    ap.add_argument("--seed", type=int, default=7, help="CEM sampling seed")
    ap.add_argument("--sigma", type=float, default=0.20)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=args.horizon)
    perturb_seeds = list(range(args.seeds))
    dim = LearnedFeedbackWalk.OUT_DIM * LearnedFeedbackWalk.OBS_DIM

    # Warm-start at balanced-cpg-style ankle feedback (rest zero).
    mean = np.zeros(dim)
    mean[1] = 0.15   # ankle_pitch <- pitch
    mean[3] = 0.05   # ankle_pitch <- pitch_rate
    mean[6] = 0.30   # ankle_roll  <- roll
    mean[8] = 0.05   # ankle_roll  <- roll_rate
    std = np.full(dim, args.sigma)

    rng = np.random.default_rng(args.seed)
    base_s, base_stat = robust_score(harness, mean, perturb_seeds, args.horizon)
    print(f"warm-start (hand-style feedback): robust score={base_s:.3f} "
          f"mean_surv={base_stat[0]:.2f}s mean_fwd={base_stat[1]:+.3f}m\n")
    best = (base_s, mean.copy(), base_stat)

    for it in range(args.iters):
        pop = np.clip(rng.normal(mean, std, size=(args.pop, dim)), -4.0, 4.0)
        scored = []
        for w in pop:
            score, stat = robust_score(harness, w, perturb_seeds, args.horizon)
            scored.append((score, w))
            if score > best[0]:
                best = (score, w.copy(), stat)
        scored.sort(key=lambda r: r[0], reverse=True)
        elite = np.array([w for _, w in scored[: args.elite]])
        mean = elite.mean(axis=0)
        std = elite.std(axis=0) + 0.02
        s = best[2]
        print(f"gen {it + 1:2d}/{args.iters}: robust score={best[0]:.3f} "
              f"mean_surv={s[0]:.2f}s mean_fwd={s[1]:+.3f}m worst_surv={s[2]:.2f}s")

    weights = [round(float(v), 4) for v in best[1]]
    print("\nbest weights (paste into LEARNED_FEEDBACK_WEIGHTS):")
    print(json.dumps(weights))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(weights, f)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
