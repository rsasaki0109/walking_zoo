#!/usr/bin/env python3
"""Check the README MuJoCo G1 showcase assets metadata.

This validates the committed hero GIF and preview PNG without requiring Pillow or
MuJoCo, so it is safe to run in CI. Headers are parsed directly to confirm the
signature, logical dimensions, animated frame count, and a minimum byte size.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "assets" / "readme"
GIF = ASSETS / "mujoco_unitree_g1_showcase.gif"
PNG = ASSETS / "mujoco_unitree_g1_showcase_preview.png"

EXPECTED_SIZE = (960, 540)
GIF_MIN_FRAMES = 48
GIF_MIN_BYTES = 100_000
PNG_MIN_BYTES = 50_000


def _u16le(data, offset):
    return data[offset] | (data[offset + 1] << 8)


def _u32be(data, offset):
    return int.from_bytes(data[offset:offset + 4], "big")


def check_gif(path):
    rel = path.relative_to(ROOT)
    if not path.exists():
        raise SystemExit(f"missing asset: {rel}")
    data = path.read_bytes()
    if data[:6] != b"GIF89a":
        raise SystemExit(f"unexpected file signature: {rel}")
    if len(data) < GIF_MIN_BYTES:
        raise SystemExit(f"asset too small: {rel} ({len(data)} bytes)")
    size = (_u16le(data, 6), _u16le(data, 8))
    if size != EXPECTED_SIZE:
        raise SystemExit(
            f"unexpected dimensions: {rel} ({size[0]}x{size[1]}, "
            f"expected {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]})"
        )
    frames = data.count(b"\x21\xf9\x04")
    if frames < GIF_MIN_FRAMES:
        raise SystemExit(
            f"too few frames: {rel} ({frames} frames, expected >= {GIF_MIN_FRAMES})"
        )
    print(f"showcase GIF valid: {rel} ({size[0]}x{size[1]}, {frames} frames, {len(data)} bytes)")


def check_png(path):
    rel = path.relative_to(ROOT)
    if not path.exists():
        raise SystemExit(f"missing asset: {rel}")
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise SystemExit(f"unexpected file signature: {rel}")
    if len(data) < PNG_MIN_BYTES:
        raise SystemExit(f"asset too small: {rel} ({len(data)} bytes)")
    # IHDR width/height are the first chunk, big-endian at byte offsets 16 and 20.
    size = (_u32be(data, 16), _u32be(data, 20))
    if size != EXPECTED_SIZE:
        raise SystemExit(
            f"unexpected dimensions: {rel} ({size[0]}x{size[1]}, "
            f"expected {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]})"
        )
    print(f"showcase PNG valid: {rel} ({size[0]}x{size[1]}, {len(data)} bytes)")


def main():
    check_gif(GIF)
    check_png(PNG)
    print("MuJoCo G1 showcase README assets look valid")


if __name__ == "__main__":
    main()
