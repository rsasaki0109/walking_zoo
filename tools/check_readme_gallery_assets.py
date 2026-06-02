#!/usr/bin/env python3
"""Validate the README MuJoCo G1 gait gallery GIF metadata.

This checks the committed gallery asset without requiring Pillow or MuJoCo, so it
is safe to run in CI. It parses the GIF header directly to confirm the signature,
logical screen dimensions, animated frame count, and a minimum byte size.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GIF = ROOT / "docs" / "assets" / "readme" / "mujoco_unitree_g1_gait_gallery.gif"

EXPECTED_SIZE = (960, 810)
MIN_FRAMES = 24
MIN_BYTES = 200_000


def _u16le(data, offset):
    return data[offset] | (data[offset + 1] << 8)


def main():
    if not GIF.exists():
        raise SystemExit(f"missing asset: {GIF.relative_to(ROOT)}")

    data = GIF.read_bytes()
    rel = GIF.relative_to(ROOT)

    if data[:6] != b"GIF89a":
        raise SystemExit(f"unexpected file signature: {rel}")
    if len(data) < MIN_BYTES:
        raise SystemExit(f"asset too small: {rel} ({len(data)} bytes)")

    width = _u16le(data, 6)
    height = _u16le(data, 8)
    if (width, height) != EXPECTED_SIZE:
        raise SystemExit(
            f"unexpected dimensions: {rel} ({width}x{height}, "
            f"expected {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]})"
        )

    # Each animated frame is preceded by a Graphic Control Extension block.
    frames = data.count(b"\x21\xf9\x04")
    if frames < MIN_FRAMES:
        raise SystemExit(
            f"too few frames: {rel} ({frames} frames, expected >= {MIN_FRAMES})"
        )

    print(
        f"gait gallery README asset looks valid: {rel} "
        f"({width}x{height}, {frames} frames, {len(data)} bytes)"
    )


if __name__ == "__main__":
    main()
