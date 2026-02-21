import numpy as np
from pathlib import Path
width, height, stride = 1920, 1080, 2560
frame_bytes = stride * height
raw = Path('imx385_1080p.raw').read_bytes()
if len(raw) % frame_bytes:
    raise SystemExit('raw size not multiple of per-frame stride*height')
frames = len(raw) // frame_bytes

def decode_row(row):
    pixels = []
    bit_buf = 0
    bits = 0
    for b in row:
        bit_buf |= b << bits
        bits += 8
        while bits >= 10 and len(pixels) < width:
            pixels.append(bit_buf & 0x3FF)
            bit_buf >>= 10
            bits -= 10
    return pixels

out = bytearray()
for fi in range(frames):
    base = fi * frame_bytes
    for r in range(height):
        row_start = base + r * stride
        row_data = raw[row_start:row_start + (width * 10 // 8)]
        pixels = decode_row(row_data)
        if len(pixels) != width:
            raise SystemExit(f'row {r} only decoded {len(pixels)} pixels')
        for px in pixels:
            out.extend(((px & 0x3FF) << 6).to_bytes(2, 'little'))
Path('imx385_1080p_16bit.raw').write_bytes(out)
