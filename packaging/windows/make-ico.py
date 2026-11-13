#!/usr/bin/env python3
"""Write assets/icons/mesheal.ico (the Windows app and installer icon) from assets/icons/mesheal.png.

    packaging/windows/make-ico.py

Needs macOS `sips` to resize. Each size is stored as a PNG inside the .ico, which Windows Vista
and later read. Run it again after changing mesheal.png, and commit the new mesheal.ico.
"""

import os
import struct
import subprocess
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SOURCE = os.path.join(ROOT, "assets", "icons", "mesheal.png")
TARGET = os.path.join(ROOT, "assets", "icons", "mesheal.ico")
SIZES = (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)


def main() -> None:
    images = []
    with tempfile.TemporaryDirectory() as tmp:
        for size in SIZES:
            png = os.path.join(tmp, f"{size}.png")
            subprocess.run(["sips", "-z", str(size), str(size), SOURCE, "--out", png],
                           check=True, capture_output=True)
            with open(png, "rb") as f:
                images.append((size, f.read()))
    header = struct.pack("<HHH", 0, 1, len(images))
    offset = len(header) + 16 * len(images)
    entries, data = b"", b""
    for size, png in images:
        dim = 0 if size >= 256 else size  # 0 means 256 in an icon directory entry
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(png), offset + len(data))
        data += png
    with open(TARGET, "wb") as f:
        f.write(header + entries + data)
    print(f"Wrote {TARGET} ({len(header + entries + data)} bytes, sizes {', '.join(map(str, SIZES))})")


if __name__ == "__main__":
    main()
