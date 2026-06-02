#!/usr/bin/env python3
"""Optimise a gait controller's parameters with the Cross-Entropy Method.

This is the "optimisation-based gait" in the testbed: instead of hand-tuning the
capture-point walker's constants, search them against a physics rollout score.
The result is just another `GaitController` (same interface) whose parameters
were found by optimisation rather than by hand — so we can measure directly
whether optimisation closes the "farthest walker vs. most stable" gap.

    python3 optimize.py                       # optimise CapturePointWalk
    python3 optimize.py --iters 12 --pop 24   # bigger search

Needs scipy-free numpy only. Deterministic for a fixed --seed. Prints the best
parameter dict (copy it into controllers.OPTIMIZED_CAPTURE_POINT_PARAMS).
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from gait_lab import CapturePointWalk, GaitHarness, G1Model


def score_params(harness: GaitHarness, controller_cls, names, vec) -> tuple[float, object]:
    params = dict(zip(names, vec))
    metrics, _ = harness.rollout(controller_cls(params), render=False)
    # Maximise distance walked over the horizon; a tiny survival term breaks ties
    # toward staying upright (a faller stops accumulating distance, so distance
    # already rewards not falling within the horizon).
    fitness = metrics.forward_distance + 0.05 * metrics.survival_time
    return fitness, metrics


def cem(controller_cls, model, *, horizon, iters, pop, elite_frac, seed):
    names = list(controller_cls.TUNABLES)
    lo = np.array([controller_cls.TUNABLES[n][0] for n in names])
    hi = np.array([controller_cls.TUNABLES[n][1] for n in names])
    rng = np.random.default_rng(seed)
    harness = GaitHarness(model, horizon=horizon)

    # Warm-start the search distribution at the hand-tuned defaults.
    mean = np.array([float(getattr(controller_cls, n)) for n in names])
    std = (hi - lo) * 0.30
    n_elite = max(2, int(round(pop * elite_frac)))

    best_fit, best_vec, best_metrics = -1e9, mean.copy(), None
    for it in range(iters):
        samples = np.clip(rng.normal(mean, std, size=(pop, len(names))), lo, hi)
        scored = []
        for s in samples:
            fit, met = score_params(harness, controller_cls, names, s)
            scored.append((fit, s, met))
            if fit > best_fit:
                best_fit, best_vec, best_metrics = fit, s.copy(), met
        scored.sort(key=lambda r: r[0], reverse=True)
        elite = np.array([s for _, s, _ in scored[:n_elite]])
        mean = elite.mean(axis=0)
        std = elite.std(axis=0) + 1e-3
        top = best_metrics
        print(
            f"gen {it + 1:2d}/{iters}: best fit={best_fit:+.3f} "
            f"(fwd={top.forward_distance:+.3f}m surv={top.survival_time:.2f}s)"
        )
    return names, best_vec, best_metrics


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--horizon", type=float, default=6.0)
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--pop", type=int, default=18)
    ap.add_argument("--elite-frac", type=float, default=0.25)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--json", default=None, help="write best params here")
    args = ap.parse_args()

    model = G1Model(args.menagerie)
    cls = CapturePointWalk

    base, _ = GaitHarness(model, horizon=args.horizon).rollout(cls(), render=False)
    print(f"hand-tuned baseline: fwd={base.forward_distance:+.3f}m "
          f"surv={base.survival_time:.2f}s\n")

    names, best_vec, best = cem(
        cls, model, horizon=args.horizon, iters=args.iters, pop=args.pop,
        elite_frac=args.elite_frac, seed=args.seed,
    )
    params = {n: round(float(v), 4) for n, v in zip(names, best_vec)}
    print("\nbest params:")
    print(json.dumps(params, indent=2))
    print(f"\noptimised:  fwd={best.forward_distance:+.3f}m  surv={best.survival_time:.2f}s")
    print(f"hand-tuned: fwd={base.forward_distance:+.3f}m  surv={base.survival_time:.2f}s")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(params, f, indent=2)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
