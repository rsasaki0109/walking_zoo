#!/usr/bin/env python3
"""Render the **push-robustness frontier** as a polar hero image.

Reads ``out/push_frontier.json`` (from ``push_frontier.py``) and draws each
controller's robustness polygon: radius r(theta) = the largest base-velocity shove
(m/s) it survives for the full horizon, coming from direction theta. The further the
curve reaches in a direction, the harder a shove from there it can take. The shape is
the recovery anisotropy; the dashed ring marks each controller's *worst* direction —
the shove it is guaranteed to survive from any angle.

    python3 render_frontier.py                       # -> assets/push_frontier.png
    python3 render_frontier.py --json out/push_frontier.json --out assets/push_frontier.png

Matplotlib only (no MuJoCo) — it just plots the numbers the benchmark produced.
"""

from __future__ import annotations

import argparse
import json
import math
import os


# Brand-consistent palette (matches the GIF status chips: green/amber/red family).
COLORS = {
    "capture-step": ("#5aa05f", "steps to the capture point"),
    "qp-capture-step": ("#7cc4ff", "force-aware QP balance, then capture step"),
    "contact-qp": ("#e6aa28", "contact-QP WBC, in place"),
    "stiff-stand": ("#e04642", "500-gain position stand"),
}
ORDER = ["capture-step", "qp-capture-step", "contact-qp", "stiff-stand"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="out/push_frontier.json")
    ap.add_argument("--out", default="assets/push_frontier.png")
    ap.add_argument("--dpi", type=int, default=130)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    with open(args.json) as f:
        data = json.load(f)
    frontier = data["frontier"]
    horizon = data.get("horizon", "?")

    fig = plt.figure(figsize=(7.4, 7.4), facecolor="#0f1116")
    ax = fig.add_subplot(111, projection="polar", facecolor="#15171c")
    # Top-down compass: 0deg = forward (+x) at the top, robot-left (+y, 90deg) to the
    # left, back at the bottom — counterclockwise, so the image reads like looking down
    # on the robot from above.
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(1)

    names = [n for n in ORDER if n in frontier] + \
            [n for n in frontier if n not in ORDER]
    rmax = 0.0
    for name in names:
        d = frontier[name]
        color, _ = COLORS.get(name, ("#7cc4ff", ""))
        if d["best"] < 1e-6:
            # Degenerate frontier (the contact-QP): it never balances in place under a
            # shove — it certifies "must step" at any magnitude, so r(theta)=0 every
            # direction. Don't draw an invisible polygon; mark the origin and say so.
            ax.plot([0], [0], marker="o", ms=11, color=color, zorder=5,
                    label=f"{name}  (r≈0: certifies “must step” under any shove)")
            continue
        th = np.array(d["thetas"] + [d["thetas"][0]])
        r = np.array(d["radii"] + [d["radii"][0]])
        rmax = max(rmax, float(r.max()))
        ax.plot(th, r, color=color, lw=2.4, solid_joinstyle="round",
                label=f"{name}  (worst {d['worst']:.2f}, area {d['area']:.2f})")
        ax.fill(th, r, color=color, alpha=0.13)
        # dashed ring at the worst-direction radius — the any-angle guarantee.
        ring = np.linspace(0, 2 * math.pi, 180)
        ax.plot(ring, np.full_like(ring, d["worst"]), color=color, lw=0.9,
                ls=(0, (4, 4)), alpha=0.55)

    ax.set_ylim(0, math.ceil(rmax * 10) / 10 + 0.05)
    ax.set_rlabel_position(157.5)
    ax.tick_params(colors="#9aa0ab", labelsize=9)
    ax.grid(color="#2a2e38", lw=0.7)
    for spine in ax.spines.values():
        spine.set_color("#2a2e38")
    # Compass labels at the cardinal shove directions.
    ax.set_xticks([0, math.pi / 2, math.pi, 3 * math.pi / 2])
    ax.set_xticklabels(["shove\nFORWARD", "LEFT", "BACK", "RIGHT"],
                       color="#c8ccd4", fontsize=10)

    fig.suptitle("walking_zoo · push-robustness frontier", color="#7cc4ff",
                 x=0.5, y=1.005, fontsize=16, fontweight="bold")
    fig.text(0.5, 0.952, f"max base-velocity shove survived for {horizon}s, by "
             "direction (m/s) — bigger polygon = more recoverable",
             color="#c8ccd4", fontsize=10.5, ha="center")
    leg = ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06),
                    facecolor="#15171c", edgecolor="#2a2e38", fontsize=9.5,
                    labelcolor="#e8e9ee", ncol=1, framealpha=0.9)
    leg.set_title("worst direction = the any-angle guarantee",
                  prop={"size": 9})
    leg.get_title().set_color("#9aa0ab")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    mb = os.path.getsize(args.out) / 1e6
    print(f"wrote {args.out}  {mb:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
