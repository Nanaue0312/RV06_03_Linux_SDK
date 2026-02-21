#!/bin/bash

# Convert all 16-bit raw frames in the frames directory to JPG images.
# Usage: run the script from the workspace root. It assumes a 1920x1080 Bayer RGGB format
# and saves output files alongside the input with a .jpg extension.

for raw in frames/*_16bit.raw; do
    [ -e "$raw" ] || continue          # skip if no matches
    base=$(basename "$raw" .raw)
    out="frames/${base}.jpg"
    echo "Converting $raw -> $out"
    ffmpeg -y -f rawvideo \
        -pixel_format bayer_rggb16le \
        -video_size 1920x1080 \
        -i "$raw" \
        -vf "normalize,format=rgb24" \
        -frames:v 1 \
        -update 1 "$out"
done
