"""Runs MeshFix (pymeshfix) on one STL file: MeshFix.repair() with library
defaults, which keep only the largest shell.

Runs in its own Python environment with pymeshfix installed:

    python3.14 -m venv ~/.cache/mesheal/meshfix-venv
    ~/.cache/mesheal/meshfix-venv/bin/pip install pymeshfix  # the latest; each result records the version
    ~/.cache/mesheal/meshfix-venv/bin/python benchmarks/meshfix_run.py in.stl out.stl

The STL is read with its identical corner points merged into shared vertices (as STL loaders do), and the
result is written as binary STL. Prints one JSON line starting with "MESHFIX_RESULT ".
"""

import json
import struct
import sys
import time

import numpy as np
import pymeshfix


def read_stl(path: str):
    data = open(path, "rb").read()
    tris = None
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
            tris = rec["v"].astype(np.float64)
    if tris is None:  # ASCII
        pts = [list(map(float, ln.split()[1:4])) for ln in data.decode("utf-8", "replace").splitlines() if ln.strip().startswith("vertex")]
        tris = np.asarray(pts, dtype=np.float64).reshape(-1, 3, 3)
    v, inv = np.unique(tris.reshape(-1, 3), axis=0, return_inverse=True)
    return v, inv.reshape(-1, 3).astype(np.int32)


def write_stl(path: str, v: np.ndarray, f: np.ndarray) -> None:
    tris = v[f].astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(f), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tris
    with open(path, "wb") as out:
        out.write(b"\0" * 80 + struct.pack("<I", len(f)) + rec.tobytes())


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    t0 = time.perf_counter()
    v, f = read_stl(src)
    t1 = time.perf_counter()
    mf = pymeshfix.MeshFix(v, f)
    mf.repair()
    t2 = time.perf_counter()
    if len(mf.faces) == 0:
        raise SystemExit("MeshFix removed every face")
    write_stl(dst, np.asarray(mf.points), np.asarray(mf.faces))
    print("MESHFIX_RESULT " + json.dumps({"pymeshfix_version": pymeshfix.__version__, "faces_after": int(len(mf.faces)),
                                          "read_s": t1 - t0, "repair_s": t2 - t1}), flush=True)


main()
