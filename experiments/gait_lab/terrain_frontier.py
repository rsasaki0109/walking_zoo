#!/usr/bin/env python3
"""The terrain frontier: does the flat-ground capturability theory predict a slope?

The push frontier (`push_frontier.py`) and the fall-time theory (`fall_time_theory.py`)
were all measured on flat ground. The cleanest test of a theory is a *new* regime it
was not fitted to. A slope is exactly that, and it costs no model surgery: walking on
an incline of angle ``alpha`` is equivalent to tilting **gravity** by ``alpha`` (a
constant downhill component ``g sin alpha`` plus a reduced normal ``g cos alpha``), so
this just rotates ``model.opt.gravity`` and re-runs the same shove experiments.

The flat capturability rule ``v* = d*omega`` predicts what a slope should do. The
tilted gravity biases the inverted pendulum downhill by a static lean ``z*tan(alpha)``,
so the **downhill** support margin shrinks and the **uphill** one grows:

    downhill recoverable kick  ~ (d_fwd  - z*tan a) * omega_eff   (-> 0 at the limit)
    uphill   recoverable kick  ~ (d_back + z*tan a) * omega_eff
    omega_eff = sqrt(g cos a / z)

So the prediction is a frontier that *shifts uphill* as the slope steepens, and a
**critical slope** ``a*`` at which the downhill margin vanishes and the stand can no
longer hold *even without a push*.

What the measurement says (honest, including where the clean theory bends):

* The frontier does shift uphill, exactly as predicted in direction: downhill
  capturability collapses toward zero with slope while the uphill kick grows.
* The **critical slope is torque-limited, not geometry-limited** — the same forward
  asymmetry the fall-time theory found. Geometry says the stand should self-hold to
  ``arctan(d_fwd/z) ~ 9.6 deg``; it actually lets go near ~4-5 deg, because the ankle
  cannot drive the CoP to the toe to hold the downhill lean (the torque budget again).
* The uphill kick grows *more* than the static margin predicts, because on a downhill
  slope gravity also actively *decelerates* an uphill shove — a dynamic assist the
  static capture-point bound does not capture. So ``v*=d*omega`` predicts the uphill
  *direction* of the shift but under-predicts its size: an honest partial validation.
* And the lab's through-line holds on terrain too: **stepping extends the limit.** A
  capture step raises the no-push critical slope (~4.5 deg -> ~7 deg) and roughly
  doubles the downhill recoverable kick — but still ceilings out (~8 deg), because once
  the slope is steep enough even a step lands on ground that keeps falling away.

    python3 terrain_frontier.py        # -> out/terrain_frontier.json
                                       # then: python3 render_terrain_frontier.py

Reuses the stand / capture-step rollouts from ``capture_step.py``; only gravity changes.
"""

from __future__ import annotations

import argparse
import json
import math
import os

G = 9.81
# direction unit vectors in the slope plane: downhill is +x (gravity tilts toward +x).
DIRS = {"downhill": (1.0, 0.0), "uphill": (-1.0, 0.0), "lateral": (0.0, 1.0)}


def _set_slope(model, alpha_rad):
    model.model.opt.gravity[:] = [G * math.sin(alpha_rad), 0.0,
                                  -G * math.cos(alpha_rad)]


def _restore(model):
    model.model.opt.gravity[:] = [0.0, 0.0, -G]


def _survives(model, controller, alpha_rad, kick, direction, *, H, fall):
    """True iff ``controller`` holds the full horizon on slope ``alpha_rad`` under a
    base-velocity ``kick`` along ``direction``. ``controller`` is "stand" or
    "capture-step" (the recovery rollouts from capture_step.py)."""
    from capture_step import run_capture_step, run_stand
    theta = math.atan2(direction[1], direction[0])
    _set_slope(model, alpha_rad)
    try:
        if controller == "stand":
            t = run_stand(model, kick, theta, H, fall)
        else:
            t = run_capture_step(model, kick, theta, H, fall)
    finally:
        _restore(model)
    return t >= H - 1e-6


def max_kick(model, controller, alpha_deg, direction, *, H=3.0, fall=0.5,
             hi=0.8, tol=0.05):
    """Largest base-velocity kick (m/s) ``controller`` recovers on the slope, by
    binary search (monotone-in-magnitude assumption, as in push_frontier)."""
    a = math.radians(alpha_deg)
    if not _survives(model, controller, a, 0.0, direction, H=H, fall=fall):
        return 0.0
    if _survives(model, controller, a, hi, direction, H=H, fall=fall):
        return hi
    lo = 0.0
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if _survives(model, controller, a, mid, direction, H=H, fall=fall):
            lo = mid
        else:
            hi = mid
    return round(lo, 3)


def critical_slope(model, controller, *, H=3.0, fall=0.5, hi=15.0, tol=0.25):
    """Largest slope (deg) ``controller`` self-holds for the full horizon with NO
    push. Binary search on the slope angle."""
    if not _survives(model, controller, 0.0, 0.0, DIRS["downhill"], H=H, fall=fall):
        return 0.0
    lo, hi_a = 0.0, hi
    if _survives(model, controller, math.radians(hi_a), 0.0, DIRS["downhill"],
                 H=H, fall=fall):
        return hi_a
    while hi_a - lo > tol:
        mid = 0.5 * (lo + hi_a)
        if _survives(model, controller, math.radians(mid), 0.0, DIRS["downhill"],
                     H=H, fall=fall):
            lo = mid
        else:
            hi_a = mid
    return round(lo, 2)


def geometric_critical_slope(model):
    """The slope at which the static CoM projection reaches the forward support
    edge: ``arctan(d_fwd / z)`` — the geometry bound the torque limit falls short of."""
    from fall_time_theory import lipm_capturability
    cap = lipm_capturability(model)
    return round(math.degrees(math.atan(cap["margins"]["fwd"] / cap["z_com"])), 2), cap


def analyse(model, *, slopes=(0.0, 1.5, 3.0, 4.5), down_slopes=(0.0, 3.0, 4.5, 6.0),
            H=3.0, fall=0.5, log=print):
    geo_deg, cap = geometric_critical_slope(model)
    a_stand = critical_slope(model, "stand", H=H, fall=fall)
    a_step = critical_slope(model, "capture-step", H=H, fall=fall)

    log(f"\nLIPM: omega {cap['omega']:.3f}, z {cap['z_com']:.3f} m, "
        f"fwd margin {cap['margins']['fwd']:.3f} m\n")
    log("Critical slope (self-hold, no push):")
    log(f"  geometry  arctan(d_fwd/z) = {geo_deg:.1f} deg   (the bound)")
    log(f"  stiff stand (measured)    = {a_stand:.1f} deg   "
        f"-> torque-limited, well below geometry")
    log(f"  capture step (measured)   = {a_step:.1f} deg   "
        f"-> stepping extends the limit\n")

    # frontier shift: recoverable kick per direction vs slope (stiff stand)
    log("Stiff-stand recoverable kick (m/s) vs slope:")
    log(f"  {'slope':>6} {'downhill':>9} {'uphill':>8} {'lateral':>8}")
    stand_curve = {}
    for a in slopes:
        row = {d: max_kick(model, "stand", a, DIRS[d], H=H, fall=fall) for d in DIRS}
        stand_curve[a] = row
        log(f"  {a:5.1f}d {row['downhill']:8.2f} {row['uphill']:7.2f} "
            f"{row['lateral']:7.2f}")

    # stepping payoff: downhill recoverable kick, stand vs capture step
    log("\nDownhill recoverable kick (m/s), stand vs capture step:")
    log(f"  {'slope':>6} {'stand':>7} {'capture-step':>13}")
    down_curve = {}
    for a in down_slopes:
        s = max_kick(model, "stand", a, DIRS["downhill"], H=H, fall=fall)
        c = max_kick(model, "capture-step", a, DIRS["downhill"], H=H, fall=fall)
        down_curve[a] = {"stand": s, "capture-step": c}
        log(f"  {a:5.1f}d {s:6.2f} {c:12.2f}")

    log(f"\n  The slope biases the frontier uphill (downhill capturability -> 0 at the\n"
        f"  critical slope, uphill grows); the critical slope is torque-limited "
        f"({a_stand:.1f} vs\n  geometry {geo_deg:.1f} deg); and stepping extends it "
        f"({a_stand:.1f} -> {a_step:.1f} deg) — the\n  lab's through-line, now on terrain.")
    return {
        "omega": cap["omega"], "z_com": cap["z_com"],
        "fwd_margin": cap["margins"]["fwd"],
        "critical_slope": {"geometry": geo_deg, "stand": a_stand,
                           "capture-step": a_step},
        "stand_curve": {str(k): v for k, v in stand_curve.items()},
        "down_curve": {str(k): v for k, v in down_curve.items()},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--horizon", type=float, default=3.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    ap.add_argument("--out", default="out/terrain_frontier.json")
    args = ap.parse_args()

    from gait_lab import G1Model
    model = G1Model(args.menagerie)
    print("Terrain frontier: tilting gravity to test the capturability theory on a slope")
    result = analyse(model, H=args.horizon, fall=args.fall_height)
    result["horizon"] = args.horizon

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {args.out}  (feed it to render_terrain_frontier.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
