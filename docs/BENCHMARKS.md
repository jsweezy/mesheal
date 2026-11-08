# Mesheal Benchmarks & FixSTL Evaluation Suite

Mesheal includes a Python-driven benchmarking framework modeled after the [FixSTL 3D Repair Benchmark](https://fixstl.ai/benchmarks). It automates pulling models from benchmark datasets (such as Thingi10K), caching them locally without tracking them in git, running Mesheal repairs (Fix All, Voxel Remesh, Precision Isosurface, Shrink-Wrap), and evaluating results in standardized JSON format.

## Key Features

- **Decoupled Local Caching**: Models are downloaded on demand and cached in `~/.cache/mesheal/benchmarks/models/` (or a custom path via `MESHEAL_BENCHMARK_CACHE` or `--cache-dir`). No benchmark STLs are checked into git.
- **Deduplication & Integrity**: Verifies SHA-256 digests against dataset specifications.
- **Thingiverse Zip Extraction & Mirror Fallbacks**: Supports extracting target STLs from Thingiverse project zip archives (`https://www.thingiverse.com/thing:<id>/zip`) or direct Thingi10K raw repository mirrors.
- **Volume & Area Tracking**: Directly tracks volume before/after, volume change percentage ($\Delta V\%$), surface area change percentage ($\Delta A\%$), and full defect counts.
- **FixSTL Criteria**: Evaluates `solid`, `shapeKept`, `repaired`, and `broken` definitions matching the FixSTL benchmark specification.

---

## Python Benchmark API

You can script Mesheal repairs directly in Python:

```python
from benchmarks import repair, diagnostics

# Run mesh health diagnostics
diag = diagnostics("models/sample.stl")
print(f"Volume: {diag.metrics.volume} mm³ | Watertight: {diag.metrics.watertight}")

# Run Fix All repair
result = repair("models/sample.stl", method="fix-all", output_file="models/sample_fixed.stl")
if result.is_success:
    m = result.metrics
    print(f"Volume change: {m.volume_change_pct:+.2f}%")
    print(f"Watertight: {m.watertight_after}")
```

Available `method` options:
- `"fix-all"`: Multi-pass Fix All repair pipeline
- `"remesh-voxel"`: Pipeline A — Voxel Heal & Remesh (Taubin smoothed)
- `"remesh-iso"`: Pipeline B — Precision Isosurface Remesh (FEA grade)
- `"rebuild-alpha"`: Shrink-wrap outer envelope (CGAL 3D Alpha Wrap)
- `"rebuild-union"`: Self-union outer surface (CGAL autorefinement)
- `"rebuild-winding"`: Winding number marching cubes

---

## Running the FixSTL Benchmark Suite

Run benchmarks against FixSTL's Thingi10K 200-model dataset (or any custom JSON benchmark specification):

```bash
# Run Fix All on the first 10 models as a test
python3 -m benchmarks.run_benchmark --limit 10 --method fix-all

# Run across all 200 models
python3 -m benchmarks.run_benchmark --method fix-all

# Test with Precision Isosurface Remeshing
python3 -m benchmarks.run_benchmark --limit 5 --method remesh-iso

# Dry run: download and cache models only without executing repairs
python3 -m benchmarks.run_benchmark --limit 20 --dry-run
```

### Command-Line Arguments

| Argument | Default | Description |
|---|---|---|
| `--benchmark-json` | `https://fixstl.ai/benchmarks/thingi10k-200-2026-10-27.json` | Path or URL to benchmark specification. |
| `--method` | `fix-all` | Repair method (`fix-all`, `remesh-voxel`, `remesh-iso`, etc.). |
| `--limit` | 0 (all) | Maximum number of models to process. |
| `--cache-dir` | `~/.cache/mesheal/benchmarks/models/` | Directory where downloaded STLs are stored. |
| `-o, --output` | `benchmarks/results/mesheal-<method>-<N>-<date>.json` | Destination path for benchmark results JSON. |
| `--dry-run` | false | Downloads and caches models without running repair passes. |

---

## Output JSON Format

Results are output in the format matching FixSTL's JSON benchmark specifications:

```json
{
  "id": "thingi-44000",
  "source_id": "44000",
  "thing_url": "https://www.thingiverse.com/thing:13497",
  "input_sha256": "68d0a591bc0dd6366b7151b301dc682917de8669818bb53413338d99398a787a",
  "defective": true,
  "results": {
    "mesheal": {
      "status": "output",
      "solid": true,
      "shapeKept": true,
      "crossCheckSolid": true,
      "repaired": true,
      "broken": false,
      "volume_before": 1234.5,
      "volume_after": 1230.2,
      "volume_change_pct": -0.35,
      "area_before": 520.1,
      "area_after": 518.4,
      "area_change_pct": -0.33,
      "faces_before": 1500,
      "faces_after": 1492,
      "elapsed_ms": 142.5
    }
  }
}
```

---

## Running Automated Benchmark Tests

Execute the unit and end-to-end integration tests:

```bash
python3 -m unittest discover -s benchmarks/tests
```

