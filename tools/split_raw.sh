#!/bin/bash
set -euo pipefail
RAW=imx385_1080p.raw
FRAME_BYTES=$((2560*1080))
OUT_DIR=frames
mkdir -p "$OUT_DIR"

python - "$RAW" "$FRAME_BYTES" "$OUT_DIR" <<'PY'
import sys
from pathlib import Path

raw_path = Path(sys.argv[1])
frame_bytes = int(sys.argv[2])
out_dir = Path(sys.argv[3])

WIDTH = 1920
HEIGHT = 1080
STRIDE = 2560
ROW_BYTES = WIDTH * 10 // 8

raw = raw_path.read_bytes()
if len(raw) % frame_bytes:
    raise SystemExit(f"raw size {len(raw)} is not a multiple of {frame_bytes}")
frames = len(raw) // frame_bytes

for idx in range(frames):
    frame_chunk = raw[idx * frame_bytes:(idx + 1) * frame_bytes]
    out_pixels = bytearray()

    for row in range(HEIGHT):
        row_start = row * STRIDE
        row_data = frame_chunk[row_start:row_start + ROW_BYTES]
        buf = 0
        bits_in_buf = 0
        pixels = []

        for byte in row_data:
            buf |= byte << bits_in_buf
            bits_in_buf += 8
            while bits_in_buf >= 10 and len(pixels) < WIDTH:
                pixels.append(buf & 0x3FF)
                buf >>= 10
                bits_in_buf -= 10

        if len(pixels) != WIDTH:
            raise SystemExit(f"row decoded {len(pixels)} pixels instead of {WIDTH}")

        for px in pixels:
            out_pixels.extend(((px & 0x3FF) << 6).to_bytes(2, "little"))

    out_file = out_dir / f"frame_{idx:03d}_16bit.raw"
    out_file.write_bytes(out_pixels)
    print(f"wrote {out_file} ({len(out_pixels)} bytes)")
PY
