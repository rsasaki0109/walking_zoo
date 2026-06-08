#!/usr/bin/env python3
"""Render the **animated gait zoo**: every controller walking side by side, live.

The static `render_montage.py` strip answers "which gaits walk" at a glance; this
turns the same roster into the repo's hero artifact — an animated grid where each
tile is one controller running the *same* horizon under the *same* command, with a
name label and a live status that flips red the instant that gait falls. Watching
the open-loop CPG topple while the RL-residual walks the full horizon, all in one
loop, is the honest benchmark the lab exists to produce, made shareable.

    MUJOCO_GL=egl python3 render_zoo_gif.py --out assets/gait_zoo.gif

Needs a GL backend for MuJoCo (`MUJOCO_GL=egl` is headless-friendly) plus Pillow +
imageio (already in the lab's venv). With `--push` it renders the push-recovery
cut: the same gaits under a recurring shove, so the toppling/standing contrast is
the story rather than forward distance.
"""

from __future__ import annotations

import argparse
import os

import numpy as np

# A readable one-line gloss per controller, shown under each tile's title so the
# GIF tells the story without the README next to it.
GLOSS = {
    "stand-hold": "hold the keyframe",
    "open-loop-cpg": "no feedback",
    "balanced-cpg": "weight-shift + attitude",
    "capture-point": "LIPM footstep + IK",
    "optimized-cp": "capture-point, CEM-tuned",
    "dcm-walk": "DCM step adjustment",
    "zmp-preview": "ZMP preview control",
    "learned-feedback": "CPG + learned feedback",
    "rl-residual": "CPG + RL residual (PPO)",
}


def _font(size):
    from PIL import ImageFont
    try:
        from matplotlib import font_manager
        path = font_manager.findfont("DejaVu Sans", fallback_to_default=True)
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()


def _label_tile(frame, title, gloss, status, status_rgb, fonts):
    """Composite a rendered RGB frame into a labelled tile: a title/gloss band on
    top and a live status chip in the lower-left corner."""
    from PIL import Image, ImageDraw

    f_title, f_small = fonts
    h, w = frame.shape[:2]
    band = 30
    tile = Image.new("RGB", (w, h + band), (24, 26, 32))
    tile.paste(Image.fromarray(frame), (0, band))
    draw = ImageDraw.Draw(tile)
    draw.text((6, 2), title, font=f_title, fill=(238, 238, 240))
    draw.text((6, 16), gloss, font=f_small, fill=(150, 154, 165))
    # status chip
    chip = f" {status} "
    x0, y0 = 6, band + h - 20
    tw = draw.textlength(chip, font=f_small)
    draw.rectangle([x0, y0, x0 + tw, y0 + 16], fill=status_rgb)
    draw.text((x0, y0 + 1), chip, font=f_small, fill=(15, 16, 20))
    return np.asarray(tile)


def _header(width, fonts, left, right):
    """A branded title band so the GIF self-identifies wherever it's posted."""
    from PIL import Image, ImageDraw

    f_title, f_small = fonts
    band = Image.new("RGB", (width, 26), (16, 17, 22))
    draw = ImageDraw.Draw(band)
    draw.text((8, 5), left, font=f_title, fill=(124, 196, 255))
    rw = draw.textlength(right, font=f_small)
    draw.text((width - rw - 8, 7), right, font=f_small, fill=(150, 154, 165))
    return np.asarray(band)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--out", default="assets/gait_zoo.gif")
    ap.add_argument("--horizon", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=200)
    ap.add_argument("--height", type=int, default=180)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--gif-fps", type=int, default=14)
    ap.add_argument("--camera-distance", type=float, default=2.2)
    ap.add_argument("--push", type=float, default=0.0,
                    help="recurring shove speed (m/s); 0 = clean forward walk")
    args = ap.parse_args()

    os.environ.setdefault("MUJOCO_GL", "egl")
    import imageio.v2 as imageio
    from gait_lab import CONTROLLERS, Command, GaitHarness, G1Model

    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=args.horizon)
    cmd = Command()
    fonts = (_font(13), _font(11))

    tiles = []   # (title, gloss, frames, survival_time, fell)
    for controller in CONTROLLERS():
        try:
            metrics, frames = harness.rollout(
                controller, cmd=cmd, render=True, fps=args.fps,
                width=args.width, height=args.height,
                camera_distance=args.camera_distance,
                push_speed=args.push, push_interval=1.5,
            )
        except (ImportError, FileNotFoundError) as exc:
            print(f"{controller.name}: skipped ({exc})")
            continue
        if not frames:
            continue
        tiles.append((controller.name, GLOSS.get(controller.name, ""),
                      frames, metrics.survival_time, metrics.fell))
        verdict = (f"fell @{metrics.survival_time:.1f}s" if metrics.fell
                   else f"held {args.horizon:.0f}s, {metrics.forward_distance:+.2f}m")
        print(f"{controller.name:16s} {len(frames):3d} frames  {verdict}")

    if not tiles:
        print("no tiles rendered (need MUJOCO_GL + menagerie); nothing written")
        return 1

    n_frames = min(len(t[2]) for t in tiles)
    cols = args.cols
    rows = (len(tiles) + cols - 1) // cols
    dt = 1.0 / args.fps

    gif = []
    header = None
    for k in range(n_frames):
        t = k * dt
        cells = []
        for title, gloss, frames, surv, fell in tiles:
            if fell and t >= surv:
                status, rgb = f"FELL @{surv:.1f}s", (224, 70, 66)
            elif title == "stand-hold":
                status, rgb = "STANDING", (90, 160, 95)
            else:
                status, rgb = f"WALKING {t:0.1f}s", (90, 160, 95)
            cells.append(_label_tile(frames[k], title, gloss, status, rgb, fonts))
        # pad the grid to a full rectangle with empty dark cells
        ch, cw = cells[0].shape[:2]
        while len(cells) < rows * cols:
            cells.append(np.full((ch, cw, 3), 24, np.uint8))
        grid = np.vstack([np.hstack(cells[r * cols:(r + 1) * cols])
                          for r in range(rows)])
        if header is None:
            tagline = ("recurring shove · live fall detection" if args.push > 0
                       else "one command · live fall detection")
            header = _header(grid.shape[1], fonts, "locomotion_ros2 · gait zoo",
                             f"{len(tiles)} controllers · {tagline}")
        gif.append(np.vstack([header, grid]))

    # Quantize to one shared adaptive palette (flicker-free, far smaller): the
    # MuJoCo renders use a narrow colour range, so ~128 colours is lossless to the eye.
    from PIL import Image
    pal = Image.fromarray(gif[0]).quantize(colors=128, method=Image.MAXCOVERAGE)
    seq = [Image.fromarray(f).quantize(palette=pal, dither=Image.NONE) for f in gif]
    seq[0].save(args.out, save_all=True, append_images=seq[1:], loop=0,
                duration=int(1000 / args.gif_fps), optimize=True, disposal=2)
    mb = os.path.getsize(args.out) / 1e6
    print(f"\nwrote {args.out}  {gif[0].shape[1]}x{gif[0].shape[0]}  "
          f"{len(gif)} frames  {mb:.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
