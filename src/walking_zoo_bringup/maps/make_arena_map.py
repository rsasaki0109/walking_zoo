#!/usr/bin/env python3
"""Generate the SIL Nav2 demo arena map (occupancy grid PGM + YAML).

A deliberately simple world: a rectangular free arena ringed by a wall, with one
short interior wall the planner has to route around. That is enough to exercise
the *full* Nav2 stack (global planner + controller + behaviour tree) driving the
gait_lab SIL G1 to a goal, without depending on perception — localisation is
perfect in sim (the map->odom transform is identity; the MuJoCo base pose is the
odometry).

    python3 make_arena_map.py   # writes arena.pgm + arena.yaml next to this file
"""

from __future__ import annotations

from pathlib import Path

RES = 0.05          # m / pixel
W_M, H_M = 12.0, 8.0  # arena size (m)
ORIGIN = (-2.0, -4.0, 0.0)  # world coords of the map's lower-left pixel

FREE, OCC = 254, 0


def main():
    w = int(round(W_M / RES))
    h = int(round(H_M / RES))
    grid = bytearray([FREE]) * (w * h)

    def set_px(ix, iy, val):
        if 0 <= ix < w and 0 <= iy < h:
            grid[iy * w + ix] = val

    # Border wall (3 px thick).
    for ix in range(w):
        for t in range(3):
            set_px(ix, t, OCC)
            set_px(ix, h - 1 - t, OCC)
    for iy in range(h):
        for t in range(3):
            set_px(t, iy, OCC)
            set_px(w - 1 - t, iy, OCC)

    # One interior wall sticking up from the bottom around x=4 m, forcing the
    # planner to steer the robot around it on the way to a goal ahead.
    wall_x = int(round((4.0 - ORIGIN[0]) / RES))
    for iy in range(0, int(round(h * 0.55))):
        for t in range(-2, 3):
            set_px(wall_x + t, iy, OCC)

    here = Path(__file__).parent
    pgm = here / "arena.pgm"
    # PGM is stored top row first; our grid is bottom row first (image y-up), so
    # flip vertically on write.
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for iy in range(h - 1, -1, -1):
            f.write(bytes(grid[iy * w:(iy + 1) * w]))

    yaml = here / "arena.yaml"
    yaml.write_text(
        f"image: arena.pgm\n"
        f"mode: trinary\n"
        f"resolution: {RES}\n"
        f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, {ORIGIN[2]}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    print(f"wrote {pgm} ({w}x{h}) and {yaml}")


if __name__ == "__main__":
    main()
