#!/usr/bin/env python3
"""Render the GitHub **social-preview card** (1280x640) — the image every shared
link to the repo shows, anywhere.

A static, legible-at-thumbnail composition: a title band over a single frame of the
gait zoo at the moment the story is readable (several controllers already down in red,
the survivors still green). Set it once under repo Settings -> Social preview; then
every link to walking_zoo — search results, Slack, X, anywhere — carries the honest
benchmark instead of a generic card.

    MUJOCO_GL=egl python3 render_og_card.py --out assets/social_preview.png
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from render_zoo_gif import GLOSS, _font, _label_tile  # GLOSS now includes dcm-walk


def _title_band(width, height, fonts_big):
    from PIL import Image, ImageDraw

    f_brand, f_tag, f_sub = fonts_big
    band = Image.new("RGB", (width, height), (16, 17, 22))
    d = ImageDraw.Draw(band)
    d.text((22, 14), "walking_zoo", font=f_brand, fill=(124, 196, 255))
    d.text((24, 58), "honest physics benchmarks for walking robots — bad gaits "
                     "actually fall over", font=f_tag, fill=(232, 233, 238))
    d.text((24, 84), "9 controllers · one MuJoCo Unitree G1 · live fall detection",
           font=f_sub, fill=(150, 154, 165))
    return np.asarray(band)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--out", default="assets/social_preview.png")
    ap.add_argument("--at", type=float, default=2.6, help="time (s) of the frame to use")
    ap.add_argument("--cols", type=int, default=3)     # 9 controllers -> clean 3x3
    ap.add_argument("--tile-w", type=int, default=320)
    ap.add_argument("--tile-h", type=int, default=130)  # 3 rows + title band -> ~640
    ap.add_argument("--camera-distance", type=float, default=2.2)
    args = ap.parse_args()

    os.environ.setdefault("MUJOCO_GL", "egl")
    from PIL import Image
    from gait_lab import CONTROLLERS, Command, GaitHarness, G1Model

    fps = 12
    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=max(args.at + 0.4, 3.0))
    cmd = Command()
    fonts = (_font(15), _font(12))
    k = int(round(args.at * fps))

    cells = []
    for controller in CONTROLLERS():
        try:
            metrics, frames = harness.rollout(
                controller, cmd=cmd, render=True, fps=fps,
                width=args.tile_w, height=args.tile_h,
                camera_distance=args.camera_distance)
        except (ImportError, FileNotFoundError) as exc:
            print(f"{controller.name}: skipped ({exc})")
            continue
        if not frames:
            continue
        frame = frames[min(k, len(frames) - 1)]
        fell = metrics.fell and args.at >= metrics.survival_time
        if fell:
            status, rgb = f"FELL @{metrics.survival_time:.1f}s", (224, 70, 66)
        elif controller.name == "stand-hold":
            status, rgb = "STANDING", (90, 160, 95)
        else:
            status, rgb = f"WALKING {args.at:0.1f}s", (90, 160, 95)
        cells.append(_label_tile(frame, controller.name,
                                 GLOSS.get(controller.name, ""), status, rgb, fonts))
        print(f"{controller.name:16s} {status}")

    cols = args.cols
    rows = (len(cells) + cols - 1) // cols
    ch, cw = cells[0].shape[:2]
    while len(cells) < rows * cols:
        cells.append(np.full((ch, cw, 3), 24, np.uint8))
    grid = np.vstack([np.hstack(cells[r * cols:(r + 1) * cols]) for r in range(rows)])

    width = grid.shape[1]
    band = _title_band(width, 640 - grid.shape[0],
                       (_font(34), _font(17), _font(13)))
    card = np.vstack([band, grid])
    # exactly 1280x640 for GitHub's social preview
    img = Image.fromarray(card).resize((1280, 640), Image.LANCZOS)
    img.quantize(colors=220, method=Image.MAXCOVERAGE).save(args.out, optimize=True)
    mb = os.path.getsize(args.out) / 1e6
    print(f"\nwrote {args.out}  1280x640  {mb:.2f} MB\n"
          f"Set it under GitHub repo Settings -> Social preview (one upload).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
