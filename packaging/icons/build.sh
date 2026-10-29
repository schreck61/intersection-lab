#!/usr/bin/env bash
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Regenerate the application icons (macOS only: uses sips and iconutil). Outputs are
# committed, so builds don't need to run this.
#   packaging/macos/AppIcon.icns, packaging/windows/app.ico,
#   packaging/linux/intersection-lab-{256,512}.png, assets/icon_128.rgba (window icon)
set -euo pipefail
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cargo run --release --example make_icons -- "$work"
# The renderer writes uncompressed PNGs; let sips recompress them.
for f in "$work"/icon_*.png; do sips -s format png "$f" --out "$f" > /dev/null; done

iconset="$work/AppIcon.iconset"
mkdir -p "$iconset"
for s in 16 32 128 256 512; do
    cp "$work/icon_$s.png" "$iconset/icon_${s}x${s}.png"
    cp "$work/icon_$((s * 2)).png" "$iconset/icon_${s}x${s}@2x.png"
done
iconutil -c icns "$iconset" -o packaging/macos/AppIcon.icns

# Windows .ico with PNG-compressed entries (Vista and later).
python3 - "$work" packaging/windows/app.ico <<'PY'
import struct, sys
src, out = sys.argv[1], sys.argv[2]
sizes = [16, 24, 32, 48, 64, 128, 256]
blobs = [open(f"{src}/icon_{s}.png", "rb").read() for s in sizes]
head = struct.pack("<HHH", 0, 1, len(sizes))
offset = 6 + 16 * len(sizes)
dirs = b""
for s, b in zip(sizes, blobs):
    dim = 0 if s >= 256 else s
    dirs += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(b), offset)
    offset += len(b)
open(out, "wb").write(head + dirs + b"".join(blobs))
PY

cp "$work/icon_256.png" packaging/linux/intersection-lab-256.png
cp "$work/icon_512.png" packaging/linux/intersection-lab-512.png
mkdir -p assets
cp "$work/icon_128.rgba" assets/icon_128.rgba
cp "$work/icon_1024.png" packaging/icons/icon-1024.png
echo "icons updated"
