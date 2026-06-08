#!/usr/bin/env python3
"""Render the **adaptive step-duration** result on restricted footholds — why
varying *when* you step, not just *where*, keeps a biped balanced on stepping stones.

Two top-down panels plan the *same* irregular stepping stones (long/short forward gaps
that no single stride+cadence can reach while staying balanced):

* **adaptive timing** (the paper) — the DCM-at-touchdown (×, joined) stays tucked
  beside each foothold: the gait is viable.
* **fixed cadence** — same footholds, but the DCM shoots off the strip: forced to keep
  a nominal step time over irregular gaps, the divergent component runs away and the
  robot would topple.

Both *hit* the stones; only adaptive timing stays balanced. This is the core claim of
"Adaptive Step Duration for Accurate Foot Placement" (arXiv:2403.17136, 2024), which
ships no public code — implemented here from the paper in `adaptive_step.py`.

    python3 render_stepping_stones.py --out assets/adaptive_step_stones.png

Matplotlib only — it plots what the planner (`compare_timing_on_stones`) produces.
"""

from __future__ import annotations

import argparse
import os


def _panel(ax, stones, plan, par, *, title, color, viab, ylim, mpatches, np):
    # stepping stones — fixed-size square markers (so the wide aspect doesn't distort
    # them into bars); these are the only legal footholds.
    sx = [c[0] for c, _ in stones]
    sy = [c[1] for c, _ in stones]
    ax.scatter(sx, sy, marker="s", s=320, facecolor="#2c3340", edgecolor="#5a6472",
               linewidths=1.0, zorder=1, label="stepping stone")
    u, xi = plan.u, plan.xi
    # planted feet on the stones, annotated with the (possibly adapted) step time
    ax.plot(u[:, 0], u[:, 1], "o", color=color, ms=9, zorder=4)
    for k in range(len(plan.T)):
        ax.annotate(f"{plan.T[k]:.2f}s", (u[k, 0], u[k, 1]),
                    textcoords="offset points", xytext=(0, 11), ha="center",
                    color=color, fontsize=8.5, fontweight="bold")
    # DCM at each touch-down, joined — the thing that must stay near the feet.
    inb = np.abs(xi[:, 1]) <= ylim          # points still on the strip
    ax.plot(xi[inb, 0], xi[inb, 1], "x--", color="#e8e9ee", ms=8, lw=1.3, zorder=3,
            label="DCM at touch-down")
    # if the DCM genuinely runs away (fixed cadence), show it leaving with an arrow.
    # A single transient excursion (adaptive) is not "diverging", so gate on the mean.
    for k in range(1, len(xi)) if viab > 1.0 else []:
        if abs(xi[k, 1]) > ylim and abs(xi[k - 1, 1]) <= ylim:
            yedge = np.sign(xi[k, 1]) * ylim
            ax.annotate("DCM diverges", xy=(xi[k - 1, 0], yedge),
                        xytext=(xi[k - 1, 0] + 0.04, np.sign(xi[k, 1]) * ylim * 0.78),
                        color="#e8a0a0", fontsize=9, fontweight="bold",
                        arrowprops=dict(arrowstyle="-|>", color="#e8a0a0", lw=1.6))
            break
    ax.set_title(f"{title}   (DCM viability err: mean {viab:.2f})",
                 color="#c8ccd4", fontsize=11, loc="left", pad=6)
    ax.axhline(0, color="#2a2e38", lw=0.8, zorder=0)
    ax.set_facecolor("#15171c")
    ax.tick_params(colors="#9aa0ab", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#2a2e38")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="assets/adaptive_step_stones.png")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt
    import numpy as np

    from adaptive_step import GaitParams, compare_timing_on_stones

    par = GaitParams()
    stones, adaptive, fixed, s = compare_timing_on_stones(par, n=args.n)

    ylim = 0.45
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.2, 5.4), facecolor="#0f1116")
    _panel(ax1, stones, adaptive, par, title="adaptive step timing  (paper)",
           color="#5aa05f", viab=s["adaptive_viab_mean"], ylim=ylim,
           mpatches=mpatches, np=np)
    _panel(ax2, stones, fixed, par, title="fixed cadence  (baseline)",
           color="#e04642", viab=s["fixed_viab_mean"], ylim=ylim,
           mpatches=mpatches, np=np)

    # Keep both panels on the same forward axis; clamp lateral so the diverging
    # fixed-cadence DCM is visibly "running off" rather than rescaling everything.
    xmax = stones[-1][0][0] + 0.15
    for ax in (ax1, ax2):
        ax.set_xlim(-0.1, xmax)
        ax.set_ylim(-ylim - 0.05, ylim + 0.05)
        ax.set_ylabel("lateral y (m)", color="#9aa0ab", fontsize=9)
        ax.grid(color="#21252e", lw=0.6)
    ax2.set_xlabel("forward x (m)", color="#9aa0ab", fontsize=9)
    ax1.legend(loc="upper left", facecolor="#15171c", edgecolor="#2a2e38",
               labelcolor="#e8e9ee", fontsize=8.5)

    fig.suptitle("locomotion_ros2 · adaptive step duration on restricted footholds",
                 color="#7cc4ff", fontsize=14, fontweight="bold", x=0.5, y=0.985)
    fig.text(0.5, 0.925, "irregular stepping stones (arXiv:2403.17136) — both hit the "
             "stones; only adaptive timing keeps the DCM viable",
             color="#c8ccd4", fontsize=10, ha="center")
    fig.tight_layout(rect=(0, 0, 1, 0.91))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"wrote {args.out}  {os.path.getsize(args.out)/1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
