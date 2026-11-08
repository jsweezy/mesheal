#!/usr/bin/env bash
# Regenerates assets/icons/mesheal.{png,jpeg,icns} from a square source PNG (2048x2048 recommended).
# Usage: scripts/make-icons.sh path/to/mesheal_logo.png
set -euo pipefail
src="${1:?usage: make-icons.sh SOURCE.png}"
out="$(cd "$(dirname "$0")/.." && pwd)/assets/icons"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
iconset="$work/mesheal.iconset"
mkdir -p "$iconset"
for s in 16 32 128 256 512; do
    sips -z "$s" "$s" "$src" --out "$iconset/icon_${s}x${s}.png" >/dev/null
    sips -z $((s * 2)) $((s * 2)) "$src" --out "$iconset/icon_${s}x${s}@2x.png" >/dev/null
done
iconutil -c icns "$iconset" -o "$out/mesheal.icns"
sips -z 1024 1024 "$src" --out "$out/mesheal.png" >/dev/null
sips -s format jpeg "$src" --out "$out/mesheal.jpeg" >/dev/null
