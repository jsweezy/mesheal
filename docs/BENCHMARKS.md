# Mesheal Benchmarks & FixSTL Evaluation Suite

Mesheal includes a Python-driven benchmarking framework modeled after the [FixSTL 3D Repair Benchmark](https://fixstl.ai/benchmarks). It automates pulling models from benchmark datasets (such as Thingi10K), caching them locally without tracking them in git, running Mesheal repairs (Fix All, Voxel Remesh, Precision Isosurface, Shrink-Wrap), and evaluating results in standardized JSON format.

## Results: Thingi10K 200-file set (2026-11-21)

The test set, the definitions, and every non-Mesheal number below come from the
**[FixSTL STL repair benchmark](https://fixstl.ai/benchmarks)** (run dated October 27, 2026).
Many thanks to the FixSTL team for publishing the per-file results and the file list. That is what makes
a comparison like this possible. Mesheal's row is our own run of **Fix All** on the same 200 files.

| Tool | Repaired (of 131 broken) | Healthy files broken (of 69) | Solid, shape changed | Median time |
|---|---:|---:|---:|---:|
| **Mesheal 6** (`v6`), Fix All | **92** | **0** | 1 | 0.37 s |
| FixSTL, engine 0.6.0 (a7ea280) | 57 | 0 | 2 | 0.46 s |
| Blender 3D Print Toolbox (Blender 5.2.1, toolbox 1.5.0) | 41 | 1 | 3 | 0.77 s |
| MeshFix (pymeshfix 0.18.1) | 40 | 11 | 66 | 0.99 s |
| ADMesh 0.98.5 | 33 | 2 | 2 | 0.06 s |
| PrusaSlicer 2.9.6 | 30 | 4 | 2 | 0.75 s |
| trimesh 5.2.0 | 26 | 2 | 2 | 0.48 s |
| MeshLab (PyMeshLab 2025.7.post1) | 25 | 2 | 0 | 0.46 s |

**Download the per-file results:** [`mesheal-fix-all-200-2026-11-21.json`](assets/benchmarks/mesheal-fix-all-200-2026-11-21.json) (520 KB).
It is FixSTL's published per-file file ([`thingi10k-200-2026-10-27.json`](https://fixstl.ai/benchmarks/thingi10k-200-2026-10-27.json))
with a `mesheal` entry added to each file. The entry holds volume and area before and after, face counts, and the run time.
The other tools' entries are FixSTL's, unchanged. Earlier runs are kept as
[`mesheal-fix-all-200-2026-11-19.json`](assets/benchmarks/mesheal-fix-all-200-2026-11-19.json),
[`mesheal-fix-all-200-2026-11-16.json`](assets/benchmarks/mesheal-fix-all-200-2026-11-16.json) and
[`mesheal-fix-all-200-2026-11-15.json`](assets/benchmarks/mesheal-fix-all-200-2026-11-15.json).

### By problem type

Broken files repaired, grouped by the problems each file had. A file can have more than one problem, so the rows overlap.

| Tool | Holes (76) | Non-manifold (82) | Flipped / inside-out (29) | Self-intersections (88) | Duplicate or zero-area faces (90) |
|---|---:|---:|---:|---:|---:|
| **Mesheal** | **45** | **47** | **13** | **50** | **55** |
| FixSTL | 29 | 26 | 5 | 20 | 30 |
| Blender 3D Print Toolbox | 23 | 17 | 4 | 5 | 17 |
| MeshFix (pymeshfix) | 23 | 17 | 3 | 16 | 19 |
| ADMesh | 20 | 12 | 1 | 1 | 12 |
| PrusaSlicer | 17 | 12 | 1 | 1 | 12 |
| trimesh | 11 | 14 | 2 | 2 | 14 |
| MeshLab (PyMeshLab) | 11 | 14 | 2 | 2 | 14 |

### Other numbers from this run

- **Solid output:** 162 of 200 files.
- **By group:** 30 of 37 broken files repaired in FixSTL's 50-file development group, and 62 of 94 in the 150-file holdout group.
- **Compared with FixSTL:** Mesheal repaired 36 broken files that FixSTL did not. FixSTL repaired 1 that Mesheal did not (`thingi-260540`).
- **Files no other tool repaired:** 64 broken files. Mesheal repaired 27 of them.
- **Crashes:** none. Each file had a 30-minute limit, and none reached it.
- **Slow files:** the mean time is 19.5 s, much higher than the median, because a few large files take minutes.
  Two files ran past FixSTL's 300-second limit (`thingi-1368052`, 868 s, and `thingi-87674`, 311 s).
  Neither was repaired, so the 300-second limit would not change the totals.

### Changes since the 2026-11-19 run

- **87 → 92 repaired, none lost.** Two new repairs did most of it:
    - **Folded faces.** Where two faces sharing an edge fold back onto each other, Fix All flips the shared edge or
      collapses one face's tip onto it. This repaired `thingi-688370`, `thingi-96639` and `thingi-1717686`.
    - **Models cut flat.** *Cap Cut Plane* caps the openings a flat cut leaves, and leaves a ring's through-hole open.
      This repaired `thingi-117959`, a model cut at z = 0 whose base was missing (volume change −0.01%).
  `thingi-45811` is also repaired now.
- **Smaller volume changes on files that are still not repaired.** `thingi-1423014` went from −10% to 0%
  (and from 534 s to 160 s), and `thingi-48950` from +214% to +105%.
- **`thingi-1368052` is slower** (757 s → 868 s) and still not repaired.
- **No crashes or time-outs,** and no healthy file was broken.

### How to read this comparison

The two sets of numbers were not measured the same way, so this is not an exact head-to-head:

- **Different checker.** FixSTL checks every tool with its own `stl-repair diagnose` and `assess-shape` commands,
  which are not public. Mesheal's row is checked with Mesheal's own diagnostics (see `benchmarks/evaluator.py`).
  The **solid** check covers the same list of problems: closed, manifold edges and vertices, consistent winding,
  outward shells, and no self-intersections, duplicate faces, or zero-area faces.
- **Looser shape check.** FixSTL's **shape kept** test needs the same number of shells, surface area within ±5%,
  and every sampled point within 5% of the bounding-box diagonal of the other surface. Mesheal's evaluator only checks
  that surface area **or** volume changed by no more than 5%. It also allows for area lost by deleting duplicate faces.
  It does not yet check the shell count or the surface distance, so some of Mesheal's repairs might fail FixSTL's stricter test.
- **No cross-check.** FixSTL also cross-checks results with trimesh and MeshLab. Mesheal's run does not do that yet,
  so the `crossCheckSolid` field in Mesheal's entries just copies `solid`. That column is left out of the table above.
- **Different hardware.** FixSTL timed its run on an Apple M5 with 32 GB, five jobs in parallel. Mesheal ran one file at a time
  on an Apple A18 Pro with 8 GB. Both timings include process startup. Treat them as rough orders of magnitude only.
- **Not a blind test for either tool.** FixSTL notes that it used these files while developing its engine.
  Mesheal has also been tuned against files from this set.

To confirm Mesheal's numbers independently, run the repaired files through the trimesh and MeshLab cross-check
that the FixSTL page describes.

---

## Key Features

- **Decoupled Local Caching**: Models are downloaded on demand and cached in `~/.cache/mesheal/benchmarks/models/<source_id>/<source_id>.stl` (or a custom path via `MESHEAL_BENCHMARK_CACHE` or `--cache-dir`), alongside a `README.md` linking to the original Thingiverse page. No benchmark STLs are checked into git.
- **Deduplication & Integrity**: Verifies SHA-256 digests against dataset specifications.
- **Thingiverse Zip Extraction & Mirror Fallbacks**: Supports extracting target STLs from Thingiverse project zip archives (`https://www.thingiverse.com/thing:<id>/zip`) or direct Thingi10K raw repository mirrors.
- **Volume & Area Tracking**: Directly tracks volume before/after, volume change percentage ($\Delta V\%$), surface area change percentage ($\Delta A\%$), and full defect counts.
- **FixSTL Criteria**: Evaluates `solid`, `shapeKept`, `repaired`, and `broken` using FixSTL's definitions. The shape check is simpler than FixSTL's (see [How to read this comparison](#how-to-read-this-comparison)).

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

### Fast set

`benchmarks/specs/mesheal-fast.json` is a second, separate list for quick checks while developing. It does not
replace the 200-file list above, and its results are not compared with FixSTL's.

- **What is in it:** Thingi10K files whose Fix All run took under 3 seconds when they were added. It started on
  2026-11-21 with the 137 files of the 200-file set that ran under 3 seconds (82 broken, 55 healthy), and 10 new
  Thingi10K files were added the same day (seed 1: 8 broken, 2 healthy).
- **The list is fixed.** A file stays in it even if a later Fix All takes longer. New Thingi10K files are added only
  after a timed run under 3 seconds.
- **Each entry** has a `fast_set` record: the date it was added, where it came from, its Fix All time then, and its
  face count. Files from the 200-file set keep FixSTL's per-file results and labels. New Thingi10K files have no
  FixSTL results: Mesheal's own check labels them broken or healthy and lists their problems
  (`"labelled_by": "mesheal"`), and the record also says whether they were timed on battery or mains power.
- **A full run** takes about a minute and a half.

```bash
python3 -m benchmarks.run_benchmark --benchmark-json benchmarks/specs/mesheal-fast.json
```

To add new Thingi10K files, `benchmarks/add_fast_files.py` draws candidates at random from Thingi10K's metadata
(version 1.5.0) with a seed you choose, leaving out files already in any list and files over 50,000 faces. It keeps
about a fifth of the added files healthy. Each candidate is downloaded into the benchmark cache and repaired with
Fix All, and is added only if it ran under 3 seconds. On battery power (macOS) the limit is multiplied by
`--battery-factor` (default 1.05, measured on the development laptop: 3 s × 1.05 = 3.15 s), since the same file runs
slower there. `--dry-run` times the candidates without changing the list.

```bash
python3 -m benchmarks.add_fast_files --count 10 --seed 2
```

### Other tools on the same files

Four more methods run another tool instead of Mesheal and judge its output with Mesheal's own checker, the same
way as Fix All. They use each tool's latest version and default repair, and each result records the version
that ran. Results are stored under their own key (`blender_print3d_local`, `fixstl_web_local`, `admesh_local`,
`pymeshfix_local`),
next to FixSTL's published entries, so new Thingi10K files get a comparison too.

- **`--method blender-print3d`** runs Blender's 3D Print Toolbox, *Make Manifold* with default settings, as FixSTL
  did. Blender runs in the background, one file per process, with a separate profile so your own Blender setup is
  not touched. The time is the whole Blender process, start-up included. Setup, once:
  download `add-on-print3d-toolbox-v1.5.0.zip` from extensions.blender.org, then
  `BLENDER_USER_RESOURCES=~/.cache/mesheal/blender-print3d/profile Blender -b --factory-startup --command extension install-file -r user_default -e add-on-print3d-toolbox-v1.5.0.zip`.
- **`--method fixstl-web`** runs the [FixSTL](https://fixstl.ai) web tool in a headless browser: it chooses the
  file, applies the conservative repair and saves the download. The model never leaves the machine; usage
  analytics requests are blocked so test runs are not counted. The time is the check plus the repair, without
  loading the page. Each result records whether FixSTL repaired the file or kept the input, and the engine file
  the page loaded. It needs Playwright in its own environment; see `benchmarks/fixstl_web.py` for the setup.
- **`--method admesh`** runs `admesh -b out.stl in.stl`. With no fix options ADMesh runs all of its fixes: exact
  and nearby merge, remove unconnected facets, fill holes, fix normals. Install it with `brew install admesh` (or
  set `ADMESH_BIN`).
- **`--method pymeshfix`** runs MeshFix's `repair()` with its defaults, which keep only the largest shell, through
  `benchmarks/meshfix_run.py` in its own Python environment (see that file for the setup). A file MeshFix empties
  completely counts as not repaired.

All four time the whole run of the tool, one file per process (the FixSTL web tool: the check and repair only).

On the 137 fast-set files that FixSTL published results for, these runs agree with FixSTL's own verdicts on 134
files (ADMesh), 132 (FixSTL), 130 (Blender) and 117 (MeshFix); the rest differ because the two checkers disagree,
not the tools. MeshFix differs most because it drops every shell but the largest: FixSTL's shape check also
compares the number of shells, Mesheal's does not.

### Command-Line Arguments

| Argument | Default | Description |
|---|---|---|
| `--benchmark-json` | `benchmarks/specs/thingi10k-200-2026-10-27.json` | Path or URL to benchmark specification (archived from `https://fixstl.ai/benchmarks/thingi10k-200-2026-10-27.json`). |
| `--method` | `fix-all` | Repair method (`fix-all`, `remesh-voxel`, `remesh-iso`, etc.). |
| `--limit` | 0 (all) | Maximum number of models to process. |
| `--cache-dir` | `~/.cache/mesheal/benchmarks/models/` | Directory where downloaded STLs are stored. |
| `-o, --output` | `benchmarks/results/mesheal-<method>-<N>-<date>.json` | Destination path for benchmark results JSON. |
| `--timeout` | 1800 | Stops a repair that runs longer than this many seconds (wall clock) and records it as timed out. 0 = no limit. |
| `--blender-bin`, `--blender-profile` | Blender's usual install, `~/.cache/mesheal/blender-print3d/profile` | Blender and the profile with the 3D Print Toolbox, for `blender-print3d`. |
| `--meshfix-python` | `~/.cache/mesheal/meshfix-venv/bin/python` | Python with pymeshfix, for `pymeshfix`. |
| `--fixstl-python` | `~/.cache/mesheal/fixstl-venv/bin/python` | Python with Playwright, for `fixstl-web`. |
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

