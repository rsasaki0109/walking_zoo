#!/usr/bin/env python3
"""Render the **survival-time curve** the binary frontier flattens away.

``render_frontier.py`` draws the push-robustness *polygon*: for each direction, the
largest shove a controller survives for the **full horizon**. That is a binary —
"did it reach the ceiling?" — so two very different failures collapse onto the same
``r=0``: a controller that topples instantly and one that survives most of the
horizon both read as "did not survive". The contact-QP and the force+step synthesis
(``qp-capture-step``) both sit at ``r=0`` on the polygon, which hides that the latter
survives **~2x longer**.

This plot un-flattens it. For one shove direction it sweeps the raw time-to-fall vs
shove magnitude (``push_frontier.py --curve``), with the horizon drawn as a *recovery
ceiling*. The story it makes legible:

* **recovering** — ``stiff-stand`` and the immediate ``capture-step`` ride the ceiling
  up to a magnitude limit (the capture step's is the highest: it steps the support
  back under the falling CoM), then drop off;
* **only delaying** — ``contact-qp`` and ``qp-capture-step`` never reach the ceiling
  for any nonzero shove. The bare QP goes infeasible at once (~0.55 s plateau:
  "must step"); feeding that certificate to a capture STEP roughly doubles the time
  (~1.2 s plateau) — real, measurable value the polygon scored as a flat zero — but
  the late step from a drifted, compliant-balance state still cannot recover the
  horizon. Force authority delays the fall; it does not widen the frontier.

    python3 render_survival_curve.py            # -> assets/survival_curve.png

Matplotlib only (no MuJoCo) — it plots the numbers ``push_frontier.py --curve`` made.
"""

from __future__ import annotations

import argparse
import json
import os


# Same palette as render_frontier.py.
STYLE = {
    "capture-step": ("#5aa05f", "capture step — recovers (highest ceiling)"),
    "qp-capture-step": ("#7cc4ff", "QP balance, then capture step — only delays (~2x bare QP)"),
    "contact-qp": ("#e6aa28", "contact-QP WBC — infeasible at once (must step)"),
    "stiff-stand": ("#e04642", "500-gain position stand — recovers, then topples"),
}
ORDER = ["capture-step", "stiff-stand", "qp-capture-step", "contact-qp"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="out/survival_curve.json")
    ap.add_argument("--out", default="assets/survival_curve.png")
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(args.json) as f:
        data = json.load(f)
    curve = data["curve"]
    horizon = float(data.get("horizon", 3.0))
    deg = data.get("theta_deg", 0.0)
    direction = {0: "forward", 90: "left", 180: "back", 270: "right"}.get(
        int(deg), f"{deg:.0f}deg")

    fig, ax = plt.subplots(figsize=(8.6, 5.2), facecolor="#0f1116")
    ax.set_facecolor("#15171c")

    # The recovery ceiling: surviving the full horizon = recovered. A band at the top
    # makes "rode the ceiling" read at a glance.
    ax.axhspan(horizon - 0.04, horizon + 0.2, color="#2a3340", alpha=0.6, zorder=0)
    ax.axhline(horizon, color="#9aa0ab", lw=1.0, ls=(0, (5, 4)), zorder=1)
    ax.text(0.0, horizon + 0.05, "recovery ceiling (survived the full horizon)",
            color="#c8ccd4", fontsize=9, va="bottom")

    names = [n for n in ORDER if n in curve] + [n for n in curve if n not in ORDER]
    for name in names:
        d = curve[name]
        color, label = STYLE.get(name, ("#7cc4ff", name))
        mags, times = d["mags"], d["times"]
        ax.plot(mags, times, "-o", color=color, ms=5, lw=2.2, zorder=4, label=label)
        # mark where it leaves the ceiling (its recovery edge).
        cm = d.get("ceiling_mag", 0.0)
        if cm > 0:
            ax.plot([cm], [horizon], marker="v", color=color, ms=10, zorder=6)
            ax.annotate(f"{cm:.2f} m/s", (cm, horizon), textcoords="offset points",
                        xytext=(4, -14), color=color, fontsize=9, fontweight="bold")

    ax.set_xlim(left=0.0)
    ax.set_ylim(0.0, horizon + 0.35)
    ax.set_xlabel(f"shove magnitude — {direction} base-velocity kick (m/s)",
                  color="#c8ccd4", fontsize=10.5)
    ax.set_ylabel("time to fall (s)", color="#c8ccd4", fontsize=10.5)
    ax.tick_params(colors="#9aa0ab", labelsize=9)
    ax.grid(color="#21252e", lw=0.6)
    for s in ax.spines.values():
        s.set_color("#2a2e38")

    fig.suptitle("locomotion_ros2 · push recovery — delaying a fall vs recovering",
                 color="#7cc4ff", fontsize=15, fontweight="bold", x=0.5, y=0.99)
    fig.text(0.5, 0.918, "time-to-fall the binary frontier flattens to r=0: force "
             "authority (QP) delays the fall; only stepping reaches the ceiling",
             color="#c8ccd4", fontsize=9.8, ha="center")
    leg = ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13),
                    facecolor="#15171c", edgecolor="#2a2e38", labelcolor="#e8e9ee",
                    fontsize=8.8, framealpha=0.92, ncol=2)
    leg.set_zorder(10)
    fig.tight_layout(rect=(0, 0.02, 1, 0.9))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"wrote {args.out}  {os.path.getsize(args.out) / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
