#!/usr/bin/env python3
"""Render the **push-recovery showdown** — the lab's thesis in one cinematic loop.

The gait-zoo push cut shows every stand-and-walk controller toppling under a shove;
this answers the question it raises. Three controllers take the *same* forward shove
side by side:

* **STIFF POSITION STAND** — the 500-gain servo that wins every other comparison.
  It topples: a rigid inverted pendulum about the ankle.
* **CONTACT-QP WBC (TSID)** — the textbook force-aware controller, solving
  friction-cone ground-reaction forces per step. It goes *infeasible* the instant the
  capture point leaves the support polygon — not a crash, a **certificate**: "no force
  can save this, you must step."
* **CAPTURE STEP** — listens to exactly that certificate and steps the falling-side
  foot to the capture point. It **recovers** and holds the full horizon.

So the one move that survives a real push is not a stiffer servo or a cleverer QP —
it is *stepping*, taken exactly when the QP says you must. That is the through-line of
the whole lab, rendered in a single shot.

    MUJOCO_GL=egl python3 render_showdown.py --out assets/push_recovery_showdown.gif

Implementation: each tile runs the *unmodified, tested* rollout
(`run_position_stand_push`, `run_qp_stand_push`, `run_capture_step`) with `model.step`
wrapped to capture frames — so what you see is exactly the benchmarked behaviour, not
a re-implementation. Shorter runs (the topplers) are padded with their last frame.
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from render_zoo_gif import _font, _header, _label_tile


def record_run(make_model, fn, *, width, height, cam_dist, fps):
    """Run a tested rollout with ``model.step`` wrapped to grab frames; return
    ``(result, frames)``. Zero changes to the control code — full fidelity."""
    import mujoco

    model = make_model()
    renderer = mujoco.Renderer(model.model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.distance = cam_dist
    camera.elevation = -18.0
    camera.azimuth = 120.0
    frames: list = []
    frame_every = max(1, int(round((1.0 / fps) / model.timestep)))
    orig_step = model.step
    counter = {"i": 0}

    def step_rec():
        if counter["i"] % frame_every == 0:
            camera.lookat[:] = [model.data.qpos[0], model.data.qpos[1], 0.6]
            renderer.update_scene(model.data, camera=camera)
            frames.append(renderer.render().copy())
        counter["i"] += 1
        orig_step()

    model.step = step_rec
    try:
        result = fn(model)
    finally:
        model.step = orig_step
        renderer.close()
    return result, frames


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None)
    ap.add_argument("--out", default="assets/push_recovery_showdown.gif")
    ap.add_argument("--horizon", type=float, default=5.0)
    ap.add_argument("--push", type=float, default=0.3)
    ap.add_argument("--width", type=int, default=240)
    ap.add_argument("--height", type=int, default=200)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--camera-distance", type=float, default=2.3)
    args = ap.parse_args()

    os.environ.setdefault("MUJOCO_GL", "egl")
    from PIL import Image
    from gait_lab import G1Model
    from wbc_qp import run_position_stand_push, run_qp_stand_push
    from capture_step import run_capture_step

    H, P, fall = args.horizon, args.push, 0.5
    mk = lambda: G1Model(args.menagerie)
    rk = dict(width=args.width, height=args.height,
              cam_dist=args.camera_distance, fps=args.fps)

    # Each tile: title, gloss, kind, the tested rollout, and how to read its result.
    specs = [
        ("STIFF POSITION STAND", "500-gain servo, no stepping", "fall",
         lambda m: run_position_stand_push(m, H, fall, P, direction=(1.0, 0.0)),
         lambda r: float(r)),
        ("CONTACT-QP WBC (TSID)", "solves friction-cone GRF", "certify",
         lambda m: run_qp_stand_push(m, H, fall, P, direction=(1.0, 0.0)),
         lambda r: float(r[0])),
        ("CAPTURE STEP", "step to the capture point", "recover",
         lambda m: run_capture_step(m, P, 0.0, H, fall),
         lambda r: float(r[0] if isinstance(r, tuple) else r)),
    ]

    tiles = []
    for title, gloss, kind, fn, read in specs:
        result, frames = record_run(mk, fn, **rk)
        surv = read(result)
        tiles.append((title, gloss, kind, surv, frames))
        print(f"{title:24s} {len(frames):3d} frames  surv={surv:.2f}s")

    n = max(len(t[4]) for t in tiles)        # pad topplers with their last frame
    fonts = (_font(13), _font(11))
    dt = 1.0 / args.fps
    GREEN, RED, AMBER = (90, 160, 95), (224, 70, 66), (230, 170, 40)

    def status(kind, surv, t):
        if kind == "fall":
            return (f"HOLDING {t:0.1f}s", GREEN) if t < surv else \
                   (f"FELL @{surv:.1f}s", RED)
        if kind == "certify":
            return ("BALANCING", GREEN) if t < surv else \
                   ("QP INFEASIBLE -> MUST STEP", AMBER)
        return (f"STEPS, RECOVERS  {t:0.1f}s", GREEN)   # capture step never falls here

    gif, header = [], None
    for k in range(n):
        t = k * dt
        cells = []
        for title, gloss, kind, surv, frames in tiles:
            frame = frames[min(k, len(frames) - 1)]
            st, rgb = status(kind, surv, t)
            cells.append(_label_tile(frame, title, gloss, st, rgb, fonts))
        row = np.hstack(cells)
        if header is None:
            header = _header(row.shape[1], fonts, "walking_zoo · push-recovery showdown",
                             f"forward {P:g} m/s shove · who survives?")
        gif.append(np.vstack([header, row]))

    pal = Image.fromarray(gif[0]).quantize(colors=128, method=Image.MAXCOVERAGE)
    seq = [Image.fromarray(f).quantize(palette=pal, dither=Image.NONE) for f in gif]
    seq[0].save(args.out, save_all=True, append_images=seq[1:], loop=0,
                duration=int(1000 / args.fps), optimize=True, disposal=2)
    mb = os.path.getsize(args.out) / 1e6
    print(f"\nwrote {args.out}  {gif[0].shape[1]}x{gif[0].shape[0]}  "
          f"{len(gif)} frames  {mb:.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
