# Command line

The `mesheal` executable is both the desktop app and a headless command-line tool. With an action
flag it runs without a window, writes its result and exits. Use it for batch repairs, CI pipelines
and scripts.

```text
mesheal [<input.stl>]                      # open the desktop app (optionally with a file)
mesheal <action> [options] <input.stl>     # run headless and exit
```

Options can come before or after the file name. `mesheal --help` prints the full list.

## Examples

```bash
# Report mesh health; never changes or writes anything
mesheal part.stl --diagnostics

# Fix All, writing part_fixed_1.stl next to the input (or _2, _3, ... if taken)
mesheal part.stl --fix-all

# Fix All with an explicit output file and more passes
mesheal part.stl --fix-all --passes 5 -o part_clean.stl

# Remesh a scan with the fast voxel method and a 0.5-unit target edge length
mesheal scan.stl --remesh-voxel --edge-length 0.5 -o scan_remeshed.stl

# Precision remesh for a finite-element mesh
mesheal bracket.stl --remesh-iso --edge-length 1.0 -o bracket_fea.stl

# Machine-readable before/after metrics
mesheal part.stl --fix-all --json > metrics.json
```

## Actions

Pick one. Without an action (and without `--json` or `-o`) Mesheal opens the desktop app.

| Action | What it does |
|---|---|
| `--diagnostics`, `--inspect` | Runs every Mesh Health check and prints the counts. Never changes or writes the mesh. |
| `--fix-all` | The multi-pass [Fix All](gui/repairs.md#everything-fix-all) pipeline: weld cracks, remove collapsed and duplicate faces, separate touching solids, fix normals, heal thin faces, fill holes, repair intersecting islands, merge overlapping shells. |
| `--remesh-voxel` | Voxel Heal & Remesh (fast): winding-number voxelization, Taubin smoothing and isotropic remeshing. Keeps cavities. |
| `--remesh-iso` | Precision Isosurface (FEA grade): Delaunay refinement of the winding-number surface with tangential relaxation. |
| `--rebuild-alpha` | Shrink-Wrap outer envelope (CGAL 3D Alpha Wrap). Always watertight; fills cavities. |
| `--rebuild-union` | Self-union: cut along every crossing and keep the outer surface (CGAL autorefinement). |
| `--rebuild-winding` | Generalized winding-number marching cubes rebuild. |

See [Repairing](gui/repairs.md#whole-surface-rebuilds) for how to choose between the rebuilds.

## Options

| Option | Default | Applies to | Description |
|---|---|---|---|
| `--passes <N>` | 3 | `--fix-all` | Maximum number of passes. Stops earlier when the mesh is clean. |
| `--time-budget <sec>` | 0 (no limit) | `--fix-all` | Stop after this many seconds. |
| `--edge-length <len>` | auto | remesh, rebuild | Target triangle edge length, in file units. |
| `--smoothing <N>` | 3 | `--remesh-voxel` | Smoothing passes. |
| `--chord-ratio <ratio>` | 0.10 | `--remesh-iso` | How closely the new surface follows curves (smaller is closer). |
| `--alpha <val>` | auto | `--rebuild-alpha` | Size of the wrapping probe. Smaller follows more detail. |
| `--offset <val>` | auto | `--rebuild-alpha` | Distance of the wrap from the input surface. |
| `--resolution <N>` | 128 (min 16) | `--rebuild-winding` | Grid resolution. |
| `-o`, `--output <file>` | auto-named | repairs | Where to write the result. |
| `--json` | off | all | Print one JSON object to stdout instead of text. |
| `--quiet` | off | all | Suppress informational output. Errors are still printed. |
| `-v`, `--version` | | | Print version and build information. |
| `-h`, `--help` | | | Print the help. |

## The input file is never overwritten

- If `-o` names the input file, Mesheal stops with an error.
- Without `-o`, Mesheal writes `<stem>_fixed_1.stl` next to the input, or the next free number:

    ```text
    input:  models/bracket.stl
    output: models/bracket_fixed_1.stl   (or _2, _3, ... if those exist)
    ```

## Output

### Text

```text
$ mesheal examples/self_intersecting_box.stl --diagnostics
Mesh Diagnostics for: examples/self_intersecting_box.stl
  Faces: 12 | Shells: 1
  Signed Volume: 250 mm³ | Surface Area: 640.062 mm²
  Watertight: Yes
  Open Edges: 0
  Non-manifold Edges: 0
  Non-manifold Vertices: 0
  Self-intersecting Faces: 6
  Thin Faces: 0
  Flipped Faces: 0
  Elapsed: 0.055 ms
```

Units are shown as `mm`; the command line does not reinterpret units.

### JSON: diagnostics

```json
{
  "status": "success",
  "action": "diagnostics",
  "input": "examples/self_intersecting_box.stl",
  "elapsed_ms": 0.32,
  "metrics": {
    "volume": 250, "surface_area": 640.062, "faces": 12, "shells": 1, "watertight": true,
    "open_edges": 0, "non_manifold_edges": 0, "non_manifold_vertices": 0,
    "flipped_faces": 0, "inverted_shells": 0, "self_intersecting_faces": 6,
    "thin_faces": 0, "degenerate_faces": 0, "duplicate_faces": 0
  }
}
```

### JSON: repairs

Every repair action reports the same fields, each with a `_before` and `_after` value:

```json
{
  "status": "success",
  "action": "fix_all",
  "input": "examples/self_intersecting_box.stl",
  "output": "examples/self_intersecting_box_fixed_1.stl",
  "elapsed_ms": 13.1,
  "metrics": {
    "volume_before": 250, "volume_after": 314.815, "volume_change_pct": 25.93,
    "area_before": 640.062, "area_after": 516.052, "area_change_pct": -19.37,
    "max_deviation": 2.0e-15, "max_lost": 5.27,
    "faces_before": 12, "faces_after": 16,
    "shells_before": 1, "shells_after": 1,
    "watertight_before": true, "watertight_after": true,
    "open_edges_before": 0, "open_edges_after": 0,
    "non_manifold_edges_before": 0, "non_manifold_edges_after": 0,
    "non_manifold_vertices_before": 0, "non_manifold_vertices_after": 0,
    "self_intersecting_faces_before": 6, "self_intersecting_faces_after": 0,
    "thin_faces_before": 0, "thin_faces_after": 0,
    "flipped_faces_before": 0, "flipped_faces_after": 0,
    "inverted_shells_before": 0, "inverted_shells_after": 0,
    "degenerate_faces_before": 0, "degenerate_faces_after": 0,
    "duplicate_faces_before": 0, "duplicate_faces_after": 0
  }
}
```

| Field | Meaning |
|---|---|
| `action` | `diagnostics`, `fix_all`, `remesh_voxel`, `remesh_iso`, `rebuild_alpha`, `rebuild_union` or `rebuild_winding` |
| `volume_*`, `area_*` | Signed volume and surface area, in file units cubed / squared |
| `*_change_pct` | Percentage change from before to after |
| `max_deviation` | Largest distance from the result to the original surface |
| `max_lost` | Largest distance from the original surface to the result (how far removed parts were) |

On failure, the object is `{"status": "error", "error": "<message>"}`, with `action` when the
failure happened during a repair.

## Exit status

| Code | Meaning |
|---|---|
| `0` | Success |
| `1` | Error: no input file, the file could not be read, the repair failed, or the output could not be written |
