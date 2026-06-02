"""Write an RGB numpy array to a PNG using only the standard library.

The gait_lab venvs have MuJoCo but no image-encoding package (imageio / Pillow /
cv2). Rather than add a dependency just to save a comparison picture, this writes
a valid PNG with stdlib ``zlib`` + ``struct`` — enough to turn rendered frames
into a committable README asset anywhere MuJoCo runs.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np


def _chunk(tag: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + tag
        + data
        + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    )


def save_png(path: str, image: np.ndarray, *, level: int = 6) -> None:
    """Write an ``(H, W, 3)`` uint8 RGB array as a PNG (truecolour, 8-bit)."""
    img = np.ascontiguousarray(image, dtype=np.uint8)
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("save_png expects an (H, W, 3) RGB array")
    h, w, _ = img.shape

    # Each scanline is prefixed with filter-type 0 (None).
    rows = np.concatenate(
        [np.zeros((h, 1), np.uint8), img.reshape(h, w * 3)], axis=1
    )
    raw = rows.tobytes()

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit, colour type 2
    idat = zlib.compress(raw, level)
    with open(path, "wb") as f:
        f.write(sig)
        f.write(_chunk(b"IHDR", ihdr))
        f.write(_chunk(b"IDAT", idat))
        f.write(_chunk(b"IEND", b""))
