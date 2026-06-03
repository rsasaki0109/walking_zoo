#!/usr/bin/env python3
"""The **push-robustness frontier** — the showdown's thesis, as a hard number.

`render_showdown.py` shows three controllers taking *one* forward shove: the stiff
stand topples, the contact-QP certifies "must step", the capture step recovers. That
is one slice through one direction at one magnitude. This measures the *whole map*:

For each controller and each push direction theta around the circle, binary-search
the **largest base velocity kick it survives for the full horizon**. The result is a
*robustness polygon* in velocity space — a polar radius r(theta) = max survivable
shove (m/s) from that angle. Its shape is the controller's recovery anisotropy; its
**worst direction** (min radius) is a single honest scalar: "what shove are you
guaranteed to survive, no matter where it comes from?"

    python3 push_frontier.py                 # 16 directions, 3 controllers
    python3 push_frontier.py --dirs 24 --hi 2.0

Honesty notes baked in:
* A shove is a one-shot horizontal base-velocity kick (m/s) applied after settling,
  exactly the disturbance `run_*_push` already uses — so these numbers line up with
  the showdown and the tests.
* The contact-QP "survives" only when it reports ``held``; an ``infeasible`` return
  is the QP *certifying you must step*, which here counts as not-surviving-in-place
  (the whole point — it tells you to do the thing the capture step does).
* The binary search assumes survival is monotone in shove magnitude. Locomotion is
  chaotic, so that can have small islands; ``--verify`` re-checks each frontier
  radius against a fine linear sweep and reports any non-monotonicity it finds.
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np


# --- controller adapters: fn(model, v, theta, H, fall) -> (survived, detail) ----
# ``survived`` == held the *full* horizon under a shove of speed ``v`` at angle
# ``theta``. ``detail`` is a short human string for logging.

def _stand(model, v, theta, H, fall):
    from capture_step import run_stand
    t = run_stand(model, v, theta, H, fall)
    return t >= H - 1e-6, f"{t:.2f}s"


def _capture(model, v, theta, H, fall):
    from capture_step import run_capture_step
    t = run_capture_step(model, v, theta, H, fall)
    return t >= H - 1e-6, f"{t:.2f}s"


def _qp(model, v, theta, H, fall):
    from wbc_qp import run_qp_stand_push
    direction = (math.cos(theta), math.sin(theta))
    t, reason = run_qp_stand_push(model, H, fall, v, direction=direction)
    return reason == "held", f"{t:.2f}s/{reason}"


FRONTIER_CONTROLLERS = {
    "stiff-stand": (_stand, "500-gain position stand, in place"),
    "contact-qp": (_qp, "contact-QP WBC, in place (infeasible = must step)"),
    "capture-step": (_capture, "step to the capture point"),
}


def max_survivable(fn, model, theta, H, fall, *, lo=0.0, hi=1.6, tol=0.05):
    """Largest shove speed (m/s) at angle ``theta`` the controller survives for the
    full horizon. Binary search assuming survival is monotone in magnitude; returns
    the conservative lower edge of the bracket (it *did* survive ``lo``)."""
    if not fn(model, lo, theta, H, fall)[0]:
        return 0.0                      # cannot even survive the smallest shove
    if fn(model, hi, theta, H, fall)[0]:
        return hi                       # survives the whole range; report the cap
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if fn(model, mid, theta, H, fall)[0] else (lo, mid)
    return lo


def _verify_monotone(fn, model, theta, H, fall, r, hi, step=0.05):
    """Re-check: every shove <= r should survive and r+step should not. Returns a
    list of (v, survived) pairs that violate monotonicity (empty if clean)."""
    bad = []
    grid = list(np.arange(0.0, min(r + 3 * step, hi) + 1e-9, step))
    for v in grid:
        survived = fn(model, float(v), theta, H, fall)[0]
        expected = v <= r + 1e-9
        if survived != expected:
            bad.append((round(float(v), 3), survived))
    return bad


def polygon_area(radii, thetas):
    """Area of the closed polar polygon r(theta) — the capturable-velocity region in
    (vx, vy) space, (m/s)^2. A single scalar for total push robustness."""
    pts = [(r * math.cos(t), r * math.sin(t)) for r, t in zip(radii, thetas)]
    area = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        area += x0 * y1 - x1 * y0
    return abs(area) * 0.5


def compute_frontier(model, *, dirs=16, H=3.0, fall=0.5, hi=1.6, tol=0.05,
                     controllers=None, verify=False, log=print):
    """Return ``{name: {"thetas", "radii", "worst", "mean", "area", ...}}``."""
    thetas = [2.0 * math.pi * k / dirs for k in range(dirs)]
    names = controllers or list(FRONTIER_CONTROLLERS)
    out = {}
    for name in names:
        fn, gloss = FRONTIER_CONTROLLERS[name]
        radii, anomalies = [], []
        for th in thetas:
            r = max_survivable(fn, model, th, H, fall, hi=hi, tol=tol)
            radii.append(round(r, 3))
            if verify:
                bad = _verify_monotone(fn, model, th, H, fall, r, hi)
                if bad:
                    anomalies.append({"theta": round(th, 3), "violations": bad})
            log(f"  {name:13s} {math.degrees(th):5.0f}deg -> {r:.2f} m/s")
        radii_arr = np.array(radii)
        out[name] = {
            "gloss": gloss,
            "thetas": [round(t, 4) for t in thetas],
            "radii": radii,
            "worst": float(radii_arr.min()),
            "worst_dir_deg": float(math.degrees(thetas[int(radii_arr.argmin())])),
            "best": float(radii_arr.max()),
            "mean": float(radii_arr.mean()),
            "area": round(polygon_area(radii, thetas), 4),
            "anomalies": anomalies,
        }
        log(f"  -> {name}: worst {out[name]['worst']:.2f} m/s "
            f"(@{out[name]['worst_dir_deg']:.0f}deg), mean {out[name]['mean']:.2f}, "
            f"area {out[name]['area']:.3f} (m/s)^2\n")
    return out


def _leaderboard(frontier):
    rows = sorted(frontier.items(), key=lambda kv: (-kv[1]["worst"], -kv[1]["area"]))
    lines = ["push-robustness leaderboard (sorted by worst-direction survival)",
             f"  {'controller':14s} {'worst':>7} {'mean':>7} {'best':>7} "
             f"{'area (m/s)^2':>13}",
             "  " + "-" * 52]
    for name, d in rows:
        lines.append(f"  {name:14s} {d['worst']:6.2f}  {d['mean']:6.2f}  "
                     f"{d['best']:6.2f}  {d['area']:12.3f}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--dirs", type=int, default=16, help="push directions around circle")
    ap.add_argument("--horizon", type=float, default=3.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    ap.add_argument("--hi", type=float, default=1.6, help="max shove speed searched (m/s)")
    ap.add_argument("--tol", type=float, default=0.05, help="bisection tolerance (m/s)")
    ap.add_argument("--only", nargs="*", default=None, help="subset of controllers")
    ap.add_argument("--verify", action="store_true",
                    help="re-check each radius against a fine sweep (monotonicity)")
    ap.add_argument("--out", default="out/push_frontier.json")
    args = ap.parse_args()

    from gait_lab import G1Model
    model = G1Model(args.menagerie)
    print(f"mapping the push-robustness frontier: {args.dirs} directions, "
          f"horizon {args.horizon}s, shove up to {args.hi} m/s\n")
    frontier = compute_frontier(
        model, dirs=args.dirs, H=args.horizon, fall=args.fall_height,
        hi=args.hi, tol=args.tol, controllers=args.only, verify=args.verify)

    print(_leaderboard(frontier))
    any_anom = any(d["anomalies"] for d in frontier.values())
    if args.verify:
        print("\n  monotonicity:", "clean" if not any_anom else "ANOMALIES FOUND")
        for name, d in frontier.items():
            for a in d["anomalies"]:
                print(f"    {name} @{math.degrees(a['theta']):.0f}deg: {a['violations']}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"horizon": args.horizon, "hi": args.hi, "dirs": args.dirs,
                   "frontier": frontier}, f, indent=2)
    print(f"\nwrote {args.out}  (feed it to render_frontier.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
