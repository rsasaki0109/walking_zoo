#!/usr/bin/env python3
"""Render the **fall-time theory** — the lab's ~1s collapse, predicted from 1/omega.

Reads ``out/fall_theory.json`` (from ``fall_time_theory.py``) and draws the two
claims side by side:

* **left — capturability is geometry.** Predicted ``v* = d*omega`` (the largest
  in-place-recoverable kick, set by the support margin) against the measured push
  frontier per direction. Lateral and backward land on the prediction (the support
  polygon is the binding limit); forward the bar over-shoots the measurement — there
  the binding limit is ankle-pitch torque, not the long foot.
* **right — the fall clock is leg length.** Measured stiff-stand fall time vs kick,
  the free inverted-pendulum topple (a controller-independent *lower bound*), and the
  ``2/omega`` floor. Hard kicks fall at ~2/omega; the universal ~1s ceiling is a few
  multiples of the single LIPM timescale ``1/omega = sqrt(z/g)``.

    python3 render_fall_theory.py        # -> assets/fall_time_theory.png

Matplotlib only (no MuJoCo) — it plots what fall_time_theory.py measured/derived.
"""

from __future__ import annotations

import argparse
import json
import os


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="out/fall_theory.json")
    ap.add_argument("--out", default="assets/fall_time_theory.png")
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    data = json.load(open(args.json))
    cap = data["capturability"]
    omega, tau = cap["omega"], cap["tau"]
    radii = data.get("frontier_radii") or {}
    stiff = data["stiff_fall"]
    ip = data["free_ip_fall"]

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11.6, 5.0), facecolor="#0f1116")
    for ax in (axL, axR):
        ax.set_facecolor("#15171c")
        ax.tick_params(colors="#9aa0ab", labelsize=9)
        ax.grid(color="#21252e", lw=0.6)
        for s in ax.spines.values():
            s.set_color("#2a2e38")

    # -- left: capturability v* = d*omega vs measured frontier -----------------
    dirs = ["fwd", "lat", "back"]
    labels = {"fwd": "forward", "lat": "lateral", "back": "backward"}
    pred = [cap["vstar"][k] for k in dirs]
    meas = [radii.get("stiff-stand", {}).get(k) for k in dirs]
    x = np.arange(len(dirs))
    w = 0.36
    axL.bar(x - w / 2, pred, w, color="#7cc4ff", label="predicted  v* = d·ω")
    mvals = [m if m is not None else 0.0 for m in meas]
    axL.bar(x + w / 2, mvals, w, color="#5aa05f",
            label="measured push frontier (stiff stand)")
    for xi, k, p, m in zip(x, dirs, pred, meas):
        if m is None:
            continue
        rel = (p - m) / max(m, 1e-6)
        tag = "geometry-limited" if abs(rel) < 0.15 else "torque-limited"
        col = "#9aa0ab" if abs(rel) < 0.15 else "#e6aa28"
        axL.annotate(tag, (xi, max(p, m) + 0.015), ha="center", color=col,
                     fontsize=8.5, fontweight="bold")
    axL.set_xticks(x)
    axL.set_xticklabels([labels[k] for k in dirs], color="#c8ccd4", fontsize=10)
    axL.set_ylabel("max recoverable kick (m/s)", color="#c8ccd4", fontsize=10)
    axL.set_ylim(0, max(pred + mvals) * 1.22)
    axL.set_title("capturability is geometry:  v* = d·ω", color="#c8ccd4",
                  fontsize=11.5, loc="left", pad=6)
    axL.legend(facecolor="#15171c", edgecolor="#2a2e38", labelcolor="#e8e9ee",
               fontsize=8.8, loc="upper right")

    # -- right: fall time vs kick — measured, free-IP lower bound, 2/omega floor
    v0 = [p[0] for p in stiff]
    tm = [p[1] for p in stiff]
    ti = [p[1] for p in ip]
    axR.axhline(2 * tau, color="#9aa0ab", lw=1.0, ls=(0, (5, 4)), zorder=1)
    axR.text(v0[0], 2 * tau + 0.02, f"2/ω = {2*tau:.2f} s  (free topple floor)",
             color="#c8ccd4", fontsize=8.6, va="bottom")
    axR.plot(v0, tm, "-o", color="#e04642", ms=6, lw=2.2, zorder=4,
             label="measured stiff-stand fall (kick removed)")
    axR.plot(v0, ti, "--s", color="#7cc4ff", ms=5, lw=1.8, zorder=3,
             label="free inverted-pendulum topple (lower bound)")
    axR.fill_between(v0, ti, tm, color="#e04642", alpha=0.10, zorder=2)
    axR.set_xlabel("forward kick v0 (m/s)", color="#c8ccd4", fontsize=10)
    axR.set_ylabel("time to fall (s)", color="#c8ccd4", fontsize=10)
    axR.set_ylim(0, max(tm) * 1.12)
    axR.set_title("the fall clock is leg length, not the controller", color="#c8ccd4",
                  fontsize=11.5, loc="left", pad=6)
    axR.legend(facecolor="#15171c", edgecolor="#2a2e38", labelcolor="#e8e9ee",
               fontsize=8.8, loc="upper right")

    fig.suptitle("locomotion_ros2 · why ~1 second?  the collapse predicted from "
                 f"1/ω = √(z/g) = {tau:.2f} s", color="#7cc4ff", fontsize=14.5,
                 fontweight="bold", x=0.5, y=0.99)
    fig.text(0.5, 0.925, "the support polygon sets which shove you recover (left); "
             "the leg-length clock sets how fast you fall when you don't (right)",
             color="#c8ccd4", fontsize=10, ha="center")
    fig.tight_layout(rect=(0, 0, 1, 0.9))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"wrote {args.out}  {os.path.getsize(args.out) / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
