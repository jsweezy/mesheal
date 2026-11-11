#!/usr/bin/env bash
# Regenerates assets/icons/mesheal.{png,jpeg,icns} from the master logo, assets/icons/mesheal_source.png
# (2048x2048), or from another square PNG given as the first argument. Also writes mesheal_512.png.
# Usage: scripts/make-icons.sh [path/to/logo.png]
set -euo pipefail
out="$(cd "$(dirname "$0")/.." && pwd)/assets/icons"
src="${1:-$out/mesheal_source.png}"
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
# Linux desktop icon (freedesktop icon themes and AppImage accept at most 512x512)
sips -z 512 512 "$src" --out "$out/mesheal_512.png" >/dev/null
sips -s format jpeg "$src" --out "$out/mesheal.jpeg" >/dev/null
