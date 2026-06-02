#!/usr/bin/env python3
"""Compare gait algorithms on the same MuJoCo G1 under physics.

    python3 run_compare.py                      # table only
    python3 run_compare.py --gif out/           # also render a GIF per algorithm
    python3 run_compare.py --json out/cmp.json  # dump metrics as JSON

Every registered controller (gait_lab.controllers.CONTROLLERS) is rolled out for
the same horizon with the same forward-speed command, then ranked by how far it
walked without falling.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from gait_lab import CONTROLLERS, Command, GaitHarness, G1Model
from gait_lab.metrics import HEADER


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--menagerie", default=None, help="mujoco_menagerie checkout path")
    ap.add_argument("--horizon", type=float, default=5.0, help="rollout length (s)")
    ap.add_argument("--speed", type=float, default=0.4, help="commanded forward speed (m/s)")
    ap.add_argument("--gif", default=None, help="directory to write <name>.gif per algorithm")
    ap.add_argument("--json", default=None, help="path to write the metrics table as JSON")
    args = ap.parse_args()

    model = G1Model(args.menagerie)
    harness = GaitHarness(model, horizon=args.horizon)
    cmd = Command(forward_speed=args.speed)

    gif_dir = None
    if args.gif:
        gif_dir = Path(args.gif)
        gif_dir.mkdir(parents=True, exist_ok=True)

    results = []
    print(HEADER)
    for controller in CONTROLLERS():
        try:
            metrics, frames = harness.rollout(
                controller, cmd=cmd, render=gif_dir is not None
            )
        except ImportError as exc:
            # e.g. zmp-preview needs scipy; skip rather than abort the comparison.
            print(f"{controller.name:18s} skipped ({exc})")
            continue
        print(metrics.as_row())
        results.append(metrics)
        if gif_dir is not None and frames:
            _save_gif(gif_dir / f"{controller.name}.gif", frames)

    walker = max(results, key=lambda r: r.forward_distance)
    sturdy = max(results, key=lambda r: r.survival_time)
    print(f"\nfarthest walker: {walker.name} "
          f"({walker.forward_distance:+.3f} m, survived {walker.survival_time:.2f}s)")
    print(f"most stable:     {sturdy.name} (survived {sturdy.survival_time:.2f}s)")

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps([r.as_dict() for r in results], indent=2))
        print(f"wrote {args.json}")
    return 0


def _save_gif(path: Path, frames: list, fps: int = 30) -> None:
    # Prefer imageio, fall back to Pillow; both are optional eye-candy deps.
    try:
        import imageio.v2 as imageio

        imageio.mimsave(path, frames, duration=1.0 / fps, loop=0)
        print(f"  wrote {path}")
        return
    except ImportError:
        pass
    try:
        from PIL import Image

        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save(
            path, save_all=True, append_images=imgs[1:],
            duration=int(1000 / fps), loop=0,
        )
        print(f"  wrote {path}")
    except ImportError:
        print(f"  (install imageio or pillow to encode {path.name})")


if __name__ == "__main__":
    raise SystemExit(main())
