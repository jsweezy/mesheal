# Ogun Command-Line Interface (CLI)

Ogun provides a headless command-line interface for batch mesh repair, remeshing, diagnostics inspection, and benchmarking without initializing or launching the 3D GUI window.

## Usage

```bash
ogun [options] <input.stl>
```

When invoked with any action flag (`--fix-all`, `--remesh-voxel`, `--remesh-iso`, `--rebuild-alpha`, `--rebuild-union`, `--rebuild-winding`, `--diagnostics`, `--inspect`) or `--json`, Ogun executes headlessly in the console.

## Safe File Output & Collision-Free Auto-Naming

- **Overwriting Guard**: Ogun strictly prohibits overwriting the input mesh. If an explicit output path equals the input file path, Ogun aborts with an error.
- **Auto-Naming**: If `-o` or `--output` is omitted, Ogun automatically increments an available filename:
  ```
  input:  models/bracket.stl
  output: models/bracket_fixed_1.stl  (or _2.stl, _3.stl if prior files exist)
  ```

## Repair & Rebuild Actions

| Option | Description |
|---|---|
| `--fix-all` | Runs the automated multi-pass Fix All pipeline (crack closing, sliver healing, non-manifold repair, shell merging). |
| `--remesh-voxel` | Pipeline A: Fast generalized winding number voxelization with Taubin smoothing and isotropic remeshing. |
| `--remesh-iso` | Pipeline B: Precision FEA-grade level set meshing via Delaunay domain refinement and tangential relaxation. |
| `--rebuild-alpha` | Reconstructs a watertight 2-manifold surface via CGAL 3D Alpha Wrap. |
| `--rebuild-union` | Self-union outer envelope via CGAL autorefinement. |
| `--rebuild-winding` | Generalized winding number marching cubes rebuild. |
| `--diagnostics`, `--inspect` | Evaluates mesh health and defects without altering geometry. |

## Tuning Options

| Option | Default | Description |
|---|---|---|
| `--passes <N>` | 3 | Maximum repair passes for `--fix-all`. |
| `--time-budget <sec>` | 0 (none) | Time budget in seconds for `--fix-all`. |
| `--edge-length <len>` | auto | Target triangle edge length for remeshing routines. |
| `--smoothing <N>` | 3 | Smoothing passes for remeshing routines. |
| `--chord-ratio <ratio>` | 0.10 | Chord ratio for precision isosurface remeshing. |
| `--alpha <val>` | auto | Alpha parameter for alpha wrap. |
| `--offset <val>` | auto | Offset parameter for alpha wrap. |
| `--resolution <N>` | 128 | Grid resolution for winding rebuild. |

## Output Formats

- `-o, --output <file>`: Sets explicit destination path for repaired mesh.
- `--quiet`: Suppresses informational console text.
- `--json`: Outputs machine-readable JSON metrics on `stdout`.

### JSON Repair Output Example

```json
{
  "status": "success",
  "action": "fix_all",
  "input": "examples/self_intersecting_box.stl",
  "output": "self_intersecting_box_fixed_1.stl",
  "elapsed_ms": 19.2,
  "metrics": {
    "volume_before": 250.0,
    "volume_after": 314.815,
    "volume_change_pct": 25.9259,
    "area_before": 640.062,
    "area_after": 516.052,
    "area_change_pct": -19.3747,
    "max_deviation": 0.0,
    "max_lost": 5.27,
    "faces_before": 12,
    "faces_after": 16,
    "shells_before": 1,
    "shells_after": 1,
    "watertight_before": true,
    "watertight_after": true,
    "open_edges_before": 0,
    "open_edges_after": 0,
    "non_manifold_edges_before": 0,
    "non_manifold_edges_after": 0,
    "self_intersecting_faces_before": 6,
    "self_intersecting_faces_after": 0,
    "thin_faces_before": 0,
    "thin_faces_after": 0
  }
}
```

