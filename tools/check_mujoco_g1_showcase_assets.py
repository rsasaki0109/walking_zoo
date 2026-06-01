#!/usr/bin/env python3
"""Check that the README MuJoCo G1 showcase assets exist and are non-empty."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "assets" / "readme"
GIF = ASSETS / "mujoco_unitree_g1_showcase.gif"
PNG = ASSETS / "mujoco_unitree_g1_showcase_preview.png"


def check_file(path, signature, min_size):
    if not path.exists():
        raise SystemExit(f"missing asset: {path.relative_to(ROOT)}")
    size = path.stat().st_size
    if size < min_size:
        raise SystemExit(f"asset too small: {path.relative_to(ROOT)} ({size} bytes)")
    with path.open("rb") as handle:
        header = handle.read(len(signature))
    if header != signature:
        raise SystemExit(f"unexpected file signature: {path.relative_to(ROOT)}")


def main():
    check_file(GIF, b"GIF89a", 100_000)
    check_file(PNG, b"\x89PNG\r\n\x1a\n", 50_000)
    print("MuJoCo G1 showcase README assets look valid")


if __name__ == "__main__":
    main()
