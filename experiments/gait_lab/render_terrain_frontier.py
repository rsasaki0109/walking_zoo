#!/usr/bin/env python3
"""Render the **terrain frontier**: the capturability theory tested on a slope.

Reads ``out/terrain_frontier.json`` (from ``terrain_frontier.py``) and draws:

* **left — the frontier shifts uphill.** Stiff-stand recoverable kick vs slope, per
  direction. Downhill capturability collapses toward zero at the critical slope while
  the uphill kick grows: tilting gravity biases the inverted pendulum downhill, exactly
  as ``v* = d*omega`` predicts in direction. The measured critical slope (where the
  stand lets go with no push) is marked, well below the geometric ``arctan(d_fwd/z)``
  bound — the limit is ankle torque, not the foot length (the fall-time theory's
  forward asymmetry, on terrain).
* **right — stepping extends the limit.** Downhill recoverable kick, stand vs a capture
  step. Stepping raises the critical slope and roughly doubles the downhill kick — the
  lab's through-line (the recovery is the *step*) holds on slopes too, until even a
  step lands on ground that keeps falling away.

    python3 render_terrain_frontier.py        # -> assets/terrain_frontier.png

Matplotlib only — it plots what terrain_frontier.py measured (locomotion is chaotic,
so small-slope values wiggle; the trend and the limits are the point).
"""

from __future__ import annotations

import argparse
import json
import os


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="out/terrain_frontier.json")
    ap.add_argument("--out", default="assets/terrain_frontier.png")
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = json.load(open(args.json))
    crit = data["critical_slope"]
    stand_curve = data["stand_curve"]
    down_curve = data["down_curve"]

    def xs(curve):
        return sorted(float(k) for k in curve)

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11.6, 5.0), facecolor="#0f1116")
    for ax in (axL, axR):
        ax.set_facecolor("#15171c")
        ax.tick_params(colors="#9aa0ab", labelsize=9)
        ax.grid(color="#21252e", lw=0.6)
        for s in ax.spines.values():
            s.set_color("#2a2e38")

    # -- left: frontier shift, per direction -----------------------------------
    a = xs(stand_curve)
    colors = {"downhill": "#e04642", "uphill": "#5aa05f", "lateral": "#7cc4ff"}
    for d in ("downhill", "uphill", "lateral"):
        y = [stand_curve[f"{ai:.1f}" if f"{ai:.1f}" in stand_curve else str(ai)][d]
             for ai in a]
        axL.plot(a, y, "-o", color=colors[d], ms=5, lw=2.2, label=d)
    axL.axvline(crit["geometry"], color="#9aa0ab", lw=1.0, ls=(0, (4, 4)))
    axL.text(crit["geometry"], axL.get_ylim()[1] * 0.96, f" geometry {crit['geometry']:.1f}°",
             color="#9aa0ab", fontsize=8.4, va="top", ha="left", rotation=90)
    axL.axvline(crit["stand"], color="#e6aa28", lw=1.4, ls=(0, (5, 3)))
    axL.text(crit["stand"], axL.get_ylim()[1] * 0.55, f" critical {crit['stand']:.1f}°",
             color="#e6aa28", fontsize=8.8, va="center", ha="left", rotation=90,
             fontweight="bold")
    axL.set_xlabel("slope (deg)", color="#c8ccd4", fontsize=10)
    axL.set_ylabel("recoverable kick (m/s)", color="#c8ccd4", fontsize=10)
    axL.set_title("the frontier shifts uphill (stiff stand)", color="#c8ccd4",
                  fontsize=11.5, loc="left", pad=6)
    axL.legend(facecolor="#15171c", edgecolor="#2a2e38", labelcolor="#e8e9ee",
               fontsize=9, loc="upper right")

    # -- right: stepping payoff ------------------------------------------------
    a2 = xs(down_curve)
    ys = [down_curve[str(ai) if str(ai) in down_curve else f"{ai:.1f}"]["stand"]
          for ai in a2]
    yc = [down_curve[str(ai) if str(ai) in down_curve else f"{ai:.1f}"]["capture-step"]
          for ai in a2]
    axR.plot(a2, ys, "-o", color="#e04642", ms=6, lw=2.2, label="stiff stand")
    axR.plot(a2, yc, "-o", color="#5aa05f", ms=6, lw=2.2, label="capture step")
    axR.fill_between(a2, ys, yc, color="#5aa05f", alpha=0.10)
    axR.axvline(crit["stand"], color="#e04642", lw=1.0, ls=(0, (4, 4)))
    axR.axvline(crit["capture-step"], color="#5aa05f", lw=1.0, ls=(0, (4, 4)))
    axR.text(crit["capture-step"], axR.get_ylim()[1] * 0.5,
             f" stepping holds to {crit['capture-step']:.1f}°", color="#5aa05f",
             fontsize=8.6, va="center", ha="left", rotation=90, fontweight="bold")
    axR.set_xlabel("slope (deg)", color="#c8ccd4", fontsize=10)
    axR.set_ylabel("downhill recoverable kick (m/s)", color="#c8ccd4", fontsize=10)
    axR.set_title("stepping extends the limit", color="#c8ccd4", fontsize=11.5,
                  loc="left", pad=6)
    axR.legend(facecolor="#15171c", edgecolor="#2a2e38", labelcolor="#e8e9ee",
               fontsize=9, loc="upper right")

    fig.suptitle("locomotion_ros2 · terrain frontier — capturability on a slope",
                 color="#7cc4ff", fontsize=14.5, fontweight="bold", x=0.5, y=0.99)
    fig.text(0.5, 0.925, "tilting gravity by α: the downhill margin shrinks and the "
             "critical slope is torque-limited (≪ geometry); stepping pushes both back",
             color="#c8ccd4", fontsize=9.8, ha="center")
    fig.tight_layout(rect=(0, 0, 1, 0.9))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"wrote {args.out}  {os.path.getsize(args.out) / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
