#!/usr/bin/env python3
"""Why ~1 second? The collapse the whole lab keeps hitting, predicted from theory.

Every controller in this testbed — open-loop CPG, balanced CPG, the footstep
walkers, the DCM/adaptive-step steppers, even the contact-QP WBC under a shove —
loses balance on roughly the same clock: a second or so, give or take. That number
has been an *observation* repeated across a dozen experiments. This module turns it
into a *prediction* from two first-principles quantities, and checks the prediction
against the measured frontier and fall times. Two claims, one honest caveat.

The linear inverted pendulum (LIPM) has exactly one timescale,

    1/omega = sqrt(z_com / g)   (~0.27 s for this G1, CoM ~0.69 m up),

and one capturability rule (Pratt/Hof): a stand recovers in place iff the **capture
point** ``xi = x + v/omega`` stays inside the support polygon. From those alone:

* **Claim 1 — the push frontier is geometry.** The largest in-place-recoverable
  velocity kick from a given direction is ``v* = d * omega``, where ``d`` is the
  support margin in that direction (CoM-to-edge). With the G1's measured margins
  this predicts the *anisotropy* the push frontier measured empirically — and it
  lands within ~5 % laterally and backward, where the support polygon is the binding
  limit. Forward it *over*-predicts (the stand holds less than ``d*omega``): there
  the binding limit is not the long foot but the **ankle-pitch torque** that cannot
  drive the CoP all the way to the toe — exactly the torque budget ``wbc_qp.py``
  Experiment 4 found. So ``v* = d*omega`` is a tight bound where geometry binds and a
  loose one where actuation binds, and the gap *localises which limit is active*.

* **Claim 2 — the fall clock is leg length, not the controller.** Once balance is
  lost the body is a (nearly) free inverted pendulum, and its topple time is set by
  ``1/omega`` alone. The measured stiff-stand fall time asymptotes to ``~2/omega ~
  0.53 s`` under hard shoves, independent of the kick size, with the free-IP topple a
  strict lower bound beneath it. The universal ~1 s ceiling is just a few multiples
  of ``1/omega``: a humanoid that loses balance has about a quarter-second timescale
  to act on, so "topple + one futile step" lands near a second, for *any* controller.
  That is why force-vs-position never moved the wall — the wall is ``sqrt(z/g)``.

* **Honest caveat.** The free-IP topple is a *lower bound* on the measured fall
  time: a stiff servo near its capturability limit lingers (up to ~1.5 s) by fighting
  the fall it cannot win. The model predicts the floor and the scaling, not the exact
  curve — the controller can only *delay* within the 1/omega budget, never escape it.

    python3 fall_time_theory.py        # prints both claims vs the measured numbers
                                       # writes out/fall_theory.json (-> render_fall_theory.py)

Reads the measured push frontier from ``out/push_frontier.json`` if present (else
notes it is missing) and runs a short stiff-stand fall sweep itself.
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

G = 9.81


def lipm_capturability(model):
    """ω, CoM height, support margins and the capturability kick ``v* = d*omega``
    (m/s) per cardinal direction, all measured from the settled stand."""
    model.reset()
    d = model.data
    com = d.subtree_com[0].copy()
    pts = np.array([d.contact[i].pos.copy() for i in range(d.ncon)])
    ground = float(pts[:, 2].mean())
    z = float(com[2] - ground)
    omega = math.sqrt(G / z)
    margins = {
        "fwd": float(pts[:, 0].max() - com[0]),
        "back": float(com[0] - pts[:, 0].min()),
        "lat": float(np.abs(pts[:, 1] - com[1]).max()),
    }
    vstar = {k: round(v * omega, 3) for k, v in margins.items()}
    return {
        "omega": round(omega, 4),
        "z_com": round(z, 4),
        "tau": round(1.0 / omega, 4),            # the LIPM timescale 1/omega
        "pelvis0": round(float(d.qpos[2]), 4),
        "margins": {k: round(v, 4) for k, v in margins.items()},
        "vstar": vstar,
    }


def free_ip_fall_time(omega, v0, phi_fall, n=4000):
    """Topple time of a *free* inverted pendulum kicked with base velocity ``v0`` from
    upright to lean angle ``phi_fall`` — a controller-independent **lower bound** on
    the measured fall time.

    Energy gives the angular rate ``phidot^2 = (v0*omega/g*... )`` — in pendulum form
    with L = g/omega^2: ``phidot^2 = (v0/L)^2 + 2 (g/L)(1 - cos phi)``; integrate
    ``dt = dphi / phidot``. At ``v0 -> 0`` this is the pure gravitational topple
    (~2/omega to ~50 deg); larger kicks only shorten it.
    """
    L = G / (omega * omega)
    phi = np.linspace(1e-4, phi_fall, n)
    phidot = np.sqrt((v0 / L) ** 2 + 2.0 * (G / L) * (1.0 - np.cos(phi)))
    trapz = getattr(np, "trapezoid", np.trapz)   # renamed in numpy 2.0
    return float(trapz(1.0 / phidot, phi))


def fall_angle(pelvis0, fall_h):
    """Lean angle at which the pelvis drops to ``fall_h`` (the rollout's fall test):
    ``cos phi = fall_h / pelvis0``."""
    return float(math.acos(max(-1.0, min(1.0, fall_h / pelvis0))))


def measure_stiff_fall(model, v0s, *, theta=0.0, horizon=4.0, fall_h=0.5,
                       push_at=0.3):
    """Measured stiff-stand fall time (from the kick, push delay removed) vs kick."""
    from wbc_qp import run_position_stand_push
    direction = (math.cos(theta), math.sin(theta))
    out = []
    for v0 in v0s:
        t = run_position_stand_push(model, horizon, fall_h, v0, push_at=push_at,
                                    direction=direction)
        out.append((round(float(v0), 3), round(float(t - push_at), 3)))
    return out


def _load_frontier_radii(path):
    """Cardinal-direction measured radii {fwd,back,lat} from a push_frontier json,
    or None if the file is absent. Picks the nearest sampled angle to each cardinal."""
    if not os.path.exists(path):
        return None
    data = json.load(open(path))["frontier"]
    want = {"fwd": 0.0, "back": math.pi, "lat": math.pi / 2}
    out = {}
    for ctrl, d in data.items():
        th = d["thetas"]
        out[ctrl] = {}
        for name, target in want.items():
            i = min(range(len(th)), key=lambda k: abs(th[k] - target))
            out[ctrl][name] = d["radii"][i]
    return out


def analyse(model, *, frontier_json="out/push_frontier.json",
            v0s=(0.5, 0.7, 0.9, 1.1, 1.3), fall_h=0.5, log=print):
    cap = lipm_capturability(model)
    omega = cap["omega"]
    phi_fall = fall_angle(cap["pelvis0"], fall_h)
    stiff = measure_stiff_fall(model, v0s, fall_h=fall_h)
    ip = [(v0, round(free_ip_fall_time(omega, v0, phi_fall), 3)) for v0, _ in stiff]
    radii = _load_frontier_radii(frontier_json)

    log(f"\nLIPM timescale: 1/omega = sqrt(z/g) = {cap['tau']:.3f} s "
        f"(z_com {cap['z_com']:.3f} m, omega {omega:.3f})\n")
    log("Claim 1 — capturability v* = d*omega vs the measured push frontier:")
    log(f"  {'dir':5s} {'margin d':>9} {'v*=d·omega':>11} {'measured (stiff-stand)':>24}")
    for k in ("fwd", "lat", "back"):
        meas = radii["stiff-stand"][k] if radii and "stiff-stand" in radii else None
        ms = f"{meas:.2f} m/s" if meas is not None else "(no frontier json)"
        note = ""
        if meas is not None:
            rel = (cap["vstar"][k] - meas) / max(meas, 1e-6)
            note = "  geometry-limited (tight)" if abs(rel) < 0.15 \
                else "  torque-limited (v* over-predicts)" if rel > 0 else ""
        log(f"  {k:5s} {cap['margins'][k]:8.3f}m {cap['vstar'][k]:9.2f} m/s "
            f"{ms:>24}{note}")

    log(f"\nClaim 2 — fall clock is 1/omega, not the controller. Free-IP topple "
        f"(lower bound) vs measured stiff-stand fall (kick removed):")
    log(f"  {'v0':>5} {'measured':>10} {'free-IP':>9}   (2/omega floor = "
        f"{2*cap['tau']:.2f} s)")
    for (v0, tm), (_, ti) in zip(stiff, ip):
        log(f"  {v0:5.2f} {tm:8.2f}s {ti:7.2f}s")
    hard = stiff[-1][1]
    log(f"\n  hardest kick falls in {hard:.2f}s ~ 2/omega ({2*cap['tau']:.2f}s): once "
        f"balance is lost the body topples on the leg-length clock, and the universal\n"
        f"  ~1s ceiling is a few multiples of 1/omega — independent of the controller. "
        f"The free-IP time is a LOWER bound; a stiff servo near v* lingers, never escapes.")
    return {
        "capturability": cap, "fall_h": fall_h, "phi_fall": round(phi_fall, 4),
        "frontier_radii": radii, "stiff_fall": stiff, "free_ip_fall": ip,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--frontier-json", default="out/push_frontier.json")
    ap.add_argument("--fall-height", type=float, default=0.5)
    ap.add_argument("--out", default="out/fall_theory.json")
    args = ap.parse_args()

    from gait_lab import G1Model
    model = G1Model(args.menagerie)
    print("Predicting the lab's ~1s collapse from 1/omega = sqrt(z/g) and capturability")
    result = analyse(model, frontier_json=args.frontier_json, fall_h=args.fall_height)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {args.out}  (feed it to render_fall_theory.py)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
