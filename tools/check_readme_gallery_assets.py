#!/usr/bin/env python3
"""Validate the README MuJoCo G1 gallery GIF metadata.

This checks the committed gallery assets without requiring Pillow or MuJoCo, so it
is safe to run in CI. It parses the GIF header directly to confirm the signature,
logical screen dimensions, animated frame count, and a minimum byte size.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "assets" / "readme"

# (filename, expected (width, height), min frames, min bytes)
GALLERIES = [
    ("mujoco_unitree_g1_gait_gallery.gif", (960, 1350), 24, 200_000),
    ("mujoco_unitree_g1_body_pose_gallery.gif", (960, 540), 24, 200_000),
]


def _u16le(data, offset):
    return data[offset] | (data[offset + 1] << 8)


def check_gallery(filename, expected_size, min_frames, min_bytes):
    path = ASSETS / filename
    rel = path.relative_to(ROOT)
    if not path.exists():
        raise SystemExit(f"missing asset: {rel}")

    data = path.read_bytes()
    if data[:6] != b"GIF89a":
        raise SystemExit(f"unexpected file signature: {rel}")
    if len(data) < min_bytes:
        raise SystemExit(f"asset too small: {rel} ({len(data)} bytes)")

    size = (_u16le(data, 6), _u16le(data, 8))
    if size != expected_size:
        raise SystemExit(
            f"unexpected dimensions: {rel} ({size[0]}x{size[1]}, "
            f"expected {expected_size[0]}x{expected_size[1]})"
        )

    # Each animated frame is preceded by a Graphic Control Extension block.
    frames = data.count(b"\x21\xf9\x04")
    if frames < min_frames:
        raise SystemExit(
            f"too few frames: {rel} ({frames} frames, expected >= {min_frames})"
        )

    print(
        f"gallery README asset looks valid: {rel} "
        f"({size[0]}x{size[1]}, {frames} frames, {len(data)} bytes)"
    )


def main():
    for filename, expected_size, min_frames, min_bytes in GALLERIES:
        check_gallery(filename, expected_size, min_frames, min_bytes)


if __name__ == "__main__":
    main()
