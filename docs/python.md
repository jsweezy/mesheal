# Python module

`src/python/` builds `_mesheal`, a native Python module ([nanobind](https://github.com/wjakob/nanobind))
that exposes Mesheal's checks and repairs. Meshes go in and come out as NumPy arrays, and long jobs
can run in the background and be cancelled. It powers the [Blender extension](blender.md).

!!! note "Ready-made modules"
    Every [release](https://github.com/jsweezy/mesheal/releases/latest) includes the module for
    Python 3.11 and 3.13 on Windows, macOS and Linux, inside the Blender extension zips
    (`Mesheal-blender-<platform>.zip`). There is no `pip install` package yet; for any other Python,
    build the module as below.

## Build

The module builds with CMake against any Python 3, on Windows, macOS and Linux, using the same
[prerequisites](install.md#1-prerequisites) as Mesheal itself. Its tests need NumPy, so start from
a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows PowerShell: .venv\Scripts\Activate.ps1
pip install numpy

cmake -B build -DCMAKE_BUILD_TYPE=Release -DMESHEAL_BUILD_GUI=OFF \
      -DMESHEAL_BUILD_PYTHON=ON -DPython_EXECUTABLE="$(which python)"
cmake --build build -j --target _mesheal
ctest --test-dir build -R python --output-on-failure
```

On Windows, configure with the same vcpkg options as the
[Windows build](install.md#2-build), plus `-DMESHEAL_BUILD_GUI=OFF -DMESHEAL_BUILD_PYTHON=ON`
and `-DPython_EXECUTABLE=<path to python.exe>`.

The module is `build/src/python/_mesheal.cpython-<version>-<platform>.so` (`.pyd` on Windows).
Put its folder on `PYTHONPATH`, or copy it next to your script, and import it:

```bash
PYTHONPATH=build/src/python python -c "import _mesheal; print(_mesheal.version())"
```

### For Blender development on macOS

```bash
scripts/build-blender-module.sh
```

This builds the module against Blender's own Python (it looks in `/Applications/Blender.app`) in
`build_python/`, and copies it into `blender/mesheal_tools/`, so Blender loads the add-on straight
from the repository. Set `BLENDER_PYTHON` to use another Python, and `PYTHON_INCLUDE_DIR` if that
Python has no `Python.h`. See [Blender extension](blender.md#install-from-source).

## API

```python
import numpy as np
import _mesheal as m

vertices = np.asarray(..., dtype=np.float64)  # (N, 3)
faces = np.asarray(..., dtype=np.int32)       # (M, 3)

# Check: one result per Mesh Health row
for row in m.check(vertices, faces):
    print(row.name, row.count)            # e.g. "Holes (open edges)" 3
    bad_faces = row.faces                 # NumPy array of face indices

# Only some rows
rows = m.check(vertices, faces, categories=[m.Category.OPEN_EDGES, m.Category.THIN_FACES])

# Repair: returns the new mesh
r = m.repair(vertices, faces, m.Method.FIX_ALL)
if r.success:
    vertices, faces = r.vertices, r.faces   # float64 (N, 3), int64 (M, 3)
    print(r.status, r.report)

# Repair only part of the mesh (one uint8 per face, non-zero = in scope)
mask = np.zeros(len(faces), dtype=np.uint8); mask[:100] = 1
r = m.repair(vertices, faces, m.Method.FIX_ALL, face_in_scope=mask)

# Background job with progress and cancel
job = m.start_repair(vertices, faces, m.Method.VOXEL)
while not job.done:
    print(job.progress, job.status_text)
    job.wait(timeout=0.1)
result = job.result()
```

| Name | Description |
|---|---|
| `version()` | Mesheal version string |
| `check(vertices, faces, params=CheckParams(), categories=None)` | Run checks; returns a list of `CategoryResult` |
| `repair(vertices, faces, method, params=GlobalRepairParams(), face_in_scope=None)` | Run a repair; returns a `RepairResult` |
| `start_check(...)`, `start_repair(...)` | Same arguments, return a `Job` that runs in the background |
| `Category` | `SHELLS`, `OPEN_EDGES`, `NON_MANIFOLD_EDGES`, `NON_MANIFOLD_VERTICES`, `FLIPPED_NORMALS`, `INVERTED_SHELLS`, `INTERSECTIONS`, `THIN_FACES`, `DEGENERATE`, `DUPLICATES` |
| `Method` | `FIX_ALL`, `VOXEL`, `ISOSURFACE`, `ALPHA_WRAP`, `DECIMATE`, `MERGE` |
| `CategoryResult` | `category`, `name`, `count`, `info_count`, `faces`, `warnings`, `cancelled`, `elapsed_ms` |
| `RepairResult` | `success`, `cancelled`, `status`, `vertices`, `faces`, `changed_faces`, `removed_faces`, `report` |
| `Job` | `progress`, `status_text`, `done`, `cancel()`, `wait(timeout=-1)`, `result()` |

## Scripting the command-line tool

The `benchmarks` package also has a small wrapper around the [command-line tool](CLI.md), which
needs no native module:

```python
from benchmarks import repair, diagnostics

diag = diagnostics("bracket.stl")
print(diag.metrics.volume, diag.metrics.watertight)

result = repair("bracket.stl", method="fix-all")
if result.is_success:
    print(result.output_file, f"{result.metrics.volume_change_pct:+.2f}%")
```

See [Benchmarks](BENCHMARKS.md).
