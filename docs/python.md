# Python module

`src/python/` builds `_mesheal`, a native Python module ([nanobind](https://github.com/wjakob/nanobind))
that exposes Mesheal's checks and repairs. Meshes go in and come out as NumPy arrays, and long jobs
can run in the background and be cancelled. It powers the [Blender extension](blender.md).

!!! warning "Development build"
    The module is currently built from source for macOS against Blender's bundled Python. There is no
    wheel on PyPI yet.

## Build

```bash
scripts/build-blender-module.sh
```

This configures `build_python/` (Release, no GUI), builds the `_mesheal` target and copies
`_mesheal.cpython-*.so` into `blender/mesheal_tools/`. Set `BLENDER_PYTHON` to build for another
Python, and `PYTHON_INCLUDE_DIR` if that Python has no `Python.h`.

Run its tests with:

```bash
ctest --test-dir build_python -R "python|blender" --output-on-failure --timeout 300
```

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
