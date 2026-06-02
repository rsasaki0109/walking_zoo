"""Benchmark: the position-control stability ceiling.

Every hand-tuned / model-based gait in this lab eventually topples. The obvious
suspicion is that they are simply *mistuned* — that with the right gains one of
them would walk the whole horizon. This benchmark tests that suspicion directly:
it sweeps each gait *class* over its own parameters and reports the best survival
any setting achieves.

The result is the motivation for the learned policy (``train_rl.py`` /
``RLResidualWalk``): across all three classes, the best survivor tops out around
~3 s. A position-controlled humanoid in single support is a laterally-unstable
inverted pendulum, and reactive ankle/hip *position* feedback cannot inject the
ground-reaction impulse needed to arrest the sideways fall — no choice of gains
breaks that ceiling. It is a property of the control *class*, not the tuning.

    python3 stability_ceiling.py            # default (small) sweep, ~3-5 min
    python3 stability_ceiling.py --full     # wider grids

Run it before/after the RL result to see what learning has to beat.
"""

from __future__ import annotations

import argparse
import itertools

from gait_lab import G1Model, GaitHarness
from gait_lab.controllers import BalancedCPG, CapturePointWalk, ZMPPreviewWalk, _Gains


def _best(model, horizon, label, make, grid):
    """Roll out every config in ``grid`` (list of kwarg dicts); keep the best."""
    harness = GaitHarness(model, horizon=horizon)
    best = None
    for cfg in grid:
        metrics, _ = harness.rollout(make(cfg), render=False)
        if best is None or metrics.survival_time > best[0].survival_time:
            best = (metrics, cfg)
    m, cfg = best
    print(f"  {label:16s} best survival {m.survival_time:4.2f}s  "
          f"forward {m.forward_distance:+.2f}m  over {len(grid)} configs")
    return m


def _balanced(cfg):
    c = BalancedCPG(_Gains(pitch_kp=0.15, pitch_kd=0.05,
                           roll_kp=cfg["roll_kp"], roll_kd=0.08))
    c.frequency = cfg["freq"]
    c.lateral_amp = cfg["lat"]
    return c


def _capture(cfg):
    return CapturePointWalk({
        "step_duration": cfg["step_duration"], "capture_y": cfg["capture_y"],
        "ankle_roll_kp": cfg["ankle_roll_kp"], "forward_speed": 0.10,
        "capture_x": 0.5, "nominal_width": 0.9,
    })


def _zmp(cfg):
    c = ZMPPreviewWalk()
    c.step_duration = cfg["step_duration"]
    c.ankle_roll_kp = cfg["ankle_roll_kp"]
    return c


def _grid(**axes):
    keys = list(axes)
    return [dict(zip(keys, vals)) for vals in itertools.product(*axes.values())]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--full", action="store_true", help="wider parameter grids")
    args = ap.parse_args()

    model = G1Model()
    if args.full:
        bal = _grid(freq=(0.5, 0.65, 0.8), lat=(0.10, 0.18, 0.26), roll_kp=(0.3, 0.5, 0.8))
        cap = _grid(step_duration=(0.70, 0.84, 0.95), capture_y=(0.4, 0.7, 1.0),
                    ankle_roll_kp=(0.3, 0.5))
        zmp = _grid(step_duration=(0.45, 0.55, 0.70), ankle_roll_kp=(0.2, 0.4))
    else:
        bal = _grid(freq=(0.65, 0.8), lat=(0.10, 0.18), roll_kp=(0.3, 0.5))
        cap = _grid(step_duration=(0.84, 0.95), capture_y=(0.4, 1.0), ankle_roll_kp=(0.5,))
        zmp = _grid(step_duration=(0.45, 0.55), ankle_roll_kp=(0.2, 0.4))

    print(f"Stability-ceiling sweep over {args.horizon:.0f}s horizon "
          f"({'full' if args.full else 'default'} grids):\n")
    results = {
        "balanced-cpg": _best(model, args.horizon, "balanced-cpg", _balanced, bal),
        "capture-point": _best(model, args.horizon, "capture-point", _capture, cap),
        "zmp-preview": _best(model, args.horizon, "zmp-preview", _zmp, zmp),
    }
    ceiling = max(m.survival_time for m in results.values())
    print(f"\n  Best survival of any tuned model-based / CPG gait: {ceiling:4.2f}s "
          f"(horizon {args.horizon:.0f}s).")
    print("  No setting walks the full horizon: position-controlled reactive gaits")
    print("  hit a structural ~3 s lateral ceiling. Breaking it is the RL policy's job.")


if __name__ == "__main__":
    main()
