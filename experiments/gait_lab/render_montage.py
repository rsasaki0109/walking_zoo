#!/usr/bin/env python3
"""Render a side-by-side comparison montage of every gait algorithm.

One row per algorithm, columns sampled across a fixed horizon, so you can see at
a glance which gaits walk, which creep, and which topple. Each row carries a
distinct colour swatch on the left (the README legend maps colour -> algorithm,
since the PNG writer carries no font).

    MUJOCO_GL=egl python3 render_montage.py --out assets/gait_comparison.png

Needs a GL backend for MuJoCo rendering (``MUJOCO_GL=egl`` is headless-friendly).
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import CONTROLLERS, Command, GaitHarness, G1Model
from gait_lab.pngio import save_png

# Row legend (matches CONTROLLERS order). RGB swatch -> algorithm name.
SWATCHES = {
    "stand-hold": (120, 120, 120),
    "open-loop-cpg": (214, 69, 65),
    "balanced-cpg": (66, 133, 244),
    "capture-point": (244, 180, 0),
    "optimized-cp": (52, 168, 83),
    "zmp-preview": (162, 94, 224),
    "learned-feedback": (0, 191, 196),
}


def sample(frames: list, cols: int) -> list:
    if not frames:
        return []
    idx = np.linspace(0, len(frames) - 1, cols).round().astype(int)
    return [frames[i] for i in idx]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--out", default="assets/gait_comparison.png")
    ap.add_argument("--horizon", type=float, default=4.0)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--width", type=int, default=220)
    ap.add_argument("--height", type=int, default=170)
    ap.add_argument("--camera-distance", type=float, default=1.9)
    args = ap.parse_args()

    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=args.horizon)
    cmd = Command()

    gap = 3           # white separator between cells
    bar = 10          # colour swatch width
    white = np.full((1, 1, 3), 255, np.uint8)
    rows = []
    for controller in CONTROLLERS():
        try:
            _, frames = harness.rollout(
                controller, cmd=cmd, render=True,
                width=args.width, height=args.height,
                camera_distance=args.camera_distance,
            )
        except ImportError as exc:
            print(f"{controller.name}: skipped ({exc})")
            continue
        cells = sample(frames, args.cols)
        if not cells:
            continue
        # Glue the sampled frames left-to-right with thin white gaps.
        strip = []
        for j, cell in enumerate(cells):
            if j:
                strip.append(np.broadcast_to(white, (args.height, gap, 3)))
            strip.append(cell)
        row = np.concatenate(strip, axis=1)
        swatch = np.broadcast_to(
            np.array(SWATCHES.get(controller.name, (0, 0, 0)), np.uint8),
            (args.height, bar, 3),
        )
        rows.append(np.concatenate([swatch, row], axis=1))
        print(f"{controller.name}: {len(frames)} frames -> {len(cells)} columns")

    width = max(r.shape[1] for r in rows)
    sep = np.full((gap, width, 3), 255, np.uint8)
    stacked = []
    for i, r in enumerate(rows):
        if r.shape[1] < width:  # pad ragged rows on the right
            pad = np.full((r.shape[0], width - r.shape[1], 3), 255, np.uint8)
            r = np.concatenate([r, pad], axis=1)
        if i:
            stacked.append(sep)
        stacked.append(r)
    montage = np.concatenate(stacked, axis=0)
    save_png(args.out, montage)
    print(f"\nwrote {args.out}  ({montage.shape[1]}x{montage.shape[0]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
