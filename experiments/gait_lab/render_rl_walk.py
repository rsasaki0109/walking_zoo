#!/usr/bin/env python3
"""Render a filmstrip of the RL ``rl-residual`` gait walking the full horizon.

This is the gait the ``locomotion_ros2_gait_lab_sil`` adapter runs in the runtime, so
the strip doubles as the SIL integration's hero image: the only gait_lab gait
that stays up and walks forward across the whole horizon (every model-based /
hand-tuned gait is on the ground by ~3 s — see the comparison montage).

    MUJOCO_GL=egl python3 render_rl_walk.py --out <pkg>/assets/rl_residual_walk.png

Dependency-free PNG via gait_lab.pngio (no imageio/Pillow needed).
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import Command, G1Model, GaitHarness, RLResidualWalk
from gait_lab.pngio import save_png


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--out", default="assets/rl_residual_walk.png")
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--cols", type=int, default=10)
    ap.add_argument("--width", type=int, default=240)
    ap.add_argument("--height", type=int, default=200)
    ap.add_argument("--camera-distance", type=float, default=2.4)
    args = ap.parse_args()

    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=args.horizon)
    metrics, frames = harness.rollout(
        RLResidualWalk(), cmd=Command(forward_speed=0.4), render=True,
        width=args.width, height=args.height, camera_distance=args.camera_distance,
    )
    if not frames:
        print("no frames rendered (need a GL backend, e.g. MUJOCO_GL=egl)")
        return 1

    idx = np.linspace(0, len(frames) - 1, args.cols).round().astype(int)
    gap = 3
    white = np.full((args.height, gap, 3), 255, np.uint8)
    strip = []
    for j, i in enumerate(idx):
        if j:
            strip.append(white)
        strip.append(frames[i])
    # An orange ribbon on top (the rl-residual colour from the comparison montage).
    row = np.concatenate(strip, axis=1)
    ribbon = np.broadcast_to(np.array([255, 87, 34], np.uint8), (6, row.shape[1], 3))
    image = np.concatenate([ribbon, row], axis=0)
    save_png(args.out, image)
    print(f"wrote {args.out}  ({image.shape[1]}x{image.shape[0]})  "
          f"survived {metrics.survival_time:.2f}s, forward {metrics.forward_distance:+.2f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
