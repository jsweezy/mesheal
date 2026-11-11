# Mesheal — STL Repair Tool for Simulation Meshes

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE.md)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B20)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](benchmarks/)

**Mesheal** (formerly Ogun) is an open-source geometry library, headless CLI, and interactive 3D desktop application for inspecting, diagnosing, and repairing STL surface meshes for simulation and engineering analysis (e.g., Monte Carlo particle transport, FEA, CFD, and ray tracing) where watertight 2-manifold surfaces are mandatory.

Unlike 3D printing tools that silently degrade mesh fidelity or complex 3D modeling suites like Blender, Mesheal provides a deterministic, precision-oriented workflow: **multi-scale edge-aware welding, strict units verification, self-intersection resolution, thin/sliver face healing, volume change tracking, and universal multi-step undo.**

---

## Quickstart

**Download:** ready-to-run versions for Windows, macOS (Apple Silicon) and Linux are on the
[releases page](https://github.com/jsweezy/mesheal/releases/latest). See the
[install guide](docs/install.md#download) for the one-time step to open them (they are not code-signed).

### 1. Build from Source

Install prerequisites:
- **macOS** (Homebrew): `brew install cmake cgal boost`
- **Ubuntu 24.04 / Debian**: `sudo apt-get update && sudo apt-get install -y cmake build-essential git libboost-dev libgmp-dev libmpfr-dev libtbb-dev libgl1-mesa-dev xorg-dev libwayland-dev libxkbcommon-dev`
  (CMake downloads CGAL 6.2 automatically; the distro's `libcgal-dev` is too old.)

Build the optimized release binary (with parallel compilation `-j 2`):
```bash
cmake -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release -j 2
```

Run test suite:
```bash
ctest --test-dir build_release -j 4
```

Install desktop app (optional):
- **macOS** (installs `Mesheal.app` into `/Applications` & creates `~/.local/bin/mesheal`):
  ```bash
  ./packaging/macos/install-app.sh
  ```
- **Linux** (installs desktop entry, MIME association, and `~/.local/bin/mesheal`):
  ```bash
  ./packaging/linux/install-user-desktop.sh
  ```

Or build and run the command-line tool in Docker on any OS (including Windows): see [docs/DOCKER.md](docs/DOCKER.md).

---

## Three Ways to Use Mesheal

### 1. Interactive 3D Desktop Viewer
Launch the interactive viewer with Blender-style navigation, live repair previews, and real-time Mesh Health inspection:
```bash
./build_release/src/app/mesheal [path/to/model.stl]
```
- **Blender Camera Controls**: Orbit (`Right Mouse` or `Alt + Left Mouse`), Pan (`Middle Mouse` or `Shift + Right Mouse`), Focus on Selection (`.`), and Toggle Normal View (`..` / `Shift + .`).
- **Universal Undo**: Every repair, remesh, or geometry edit can be reverted step-by-step with `Ctrl+Z` / `Cmd+Z`.
- See [docs/NAVIGATION.md](docs/NAVIGATION.md) for full keyboard and mouse mappings.

---

### 2. Headless Command-Line Interface (CLI)
Automate batch mesh repairs or diagnostics in pipelines and scripts without launching a window:

```bash
# Check mesh health and print volume/defects without modifying geometry
./build_release/src/app/mesheal model.stl --diagnostics

# Automatically repair mesh using multi-pass Fix All
# (Never overwrites input: writes to model_fixed_1.stl, model_fixed_2.stl, etc.)
./build_release/src/app/mesheal model.stl --fix-all

# Explicit output destination
./build_release/src/app/mesheal model.stl --fix-all -o repaired.stl

# Remesh via Fast Voxel Marching Cubes with Taubin smoothing
./build_release/src/app/mesheal model.stl --remesh-voxel --edge-length 1.5 -o remeshed.stl

# Precision FEA-grade level set remeshing
./build_release/src/app/mesheal model.stl --remesh-iso --edge-length 1.0 -o fea_mesh.stl

# Stream structured JSON metrics (volume change %, area change %, defects) to stdout
./build_release/src/app/mesheal model.stl --fix-all --json
```

**Safe Auto-Naming Guarantee**: Mesheal strictly prohibits overwriting input STLs. If `-o` is omitted, Mesheal automatically increments available filenames (`desk_fixed_1.stl`, `desk_fixed_2.stl`, ...).

Full CLI flags and options are documented in **[docs/CLI.md](docs/CLI.md)**.

---

### 3. Python Interface & Benchmark Suite
Mesheal includes a Python API and benchmark harness modeled after the [FixSTL 3D Benchmark](https://fixstl.ai/benchmarks).

#### Python API Example
```python
from benchmarks import repair, diagnostics

# 1. Inspect mesh health
diag = diagnostics("bracket.stl")
print(f"Volume: {diag.metrics.volume:.2f} mm³ | Watertight: {diag.metrics.watertight}")
print(f"Self-intersections: {diag.metrics.self_intersecting_faces}")

# 2. Repair mesh with Fix All
result = repair("bracket.stl", method="fix-all")
if result.is_success:
    m = result.metrics
    print(f"Repaired to: {result.output_file}")
    print(f"Volume change: {m.volume_change_pct:+.2f}% (from {m.volume_before:.2f} to {m.volume_after:.2f})")
    print(f"Watertight: {m.watertight_after}")
```

#### Running FixSTL Benchmarks (with Local Caching)
Run evaluations against standardized datasets (e.g., Thingi10K) without bloating your git repository:
```bash
# Download, cache, and test 10 models with Fix All
python3 -m benchmarks.run_benchmark --limit 10 --method fix-all

# Evaluate using Precision Isosurface Remeshing
python3 -m benchmarks.run_benchmark --limit 5 --method remesh-iso

# Dry run: download and verify cache only
python3 -m benchmarks.run_benchmark --limit 20 --dry-run
```
Models are cached locally in `~/.cache/mesheal/benchmarks/models/` and verified with SHA-256 digests. Results are exported to standard FixSTL JSON schemas.

Full Python API and dataset instructions are documented in **[docs/BENCHMARKS.md](docs/BENCHMARKS.md)**.

---

## Core Capabilities Summary

| Capability | Description |
|---|---|
| **Edge-Aware Vertex Welding** | Welds unindexed triangle soup using coordinate-relative tolerances ($10^{-9}$ relative, $10^{-9}$ absolute floor) while protecting zero-area and sliver faces from premature collapse. |
| **Units Verification** | Change File Units (`mm`, `cm`, `m`, `in`, `ft`) without mutating vertex coordinates; separate explicit scaling action avoids scaling mistakes. |
| **Mesh Health Diagnostics** | Real-time background checking for open holes, non-manifold edges/vertices, flipped normals, inverted shells, self-intersections, thin/sliver faces, degenerate faces, and duplicates. |
| **Automated "Fix All"** | Multi-pass engine that closes cracks, heals slivers, splits non-manifold contacts, repairs normal orientation, and merges overlapping shells while preserving cavities. |
| **Surgical Local Repair** | Single-face Euler repairs (*Flip Longest Edge*, *Collapse Shortest Edge*, *Snap Collinear*, *Remove Flap*) with 3D 2-ring neighborhood preview. |
| **Watertight Remeshing** | Whole-surface reconstruction: *Shrink-Wrap (Outer Envelope)* via CGAL Alpha Wrap, *Voxel Heal & Remesh (Fast)* via Taubin de-terracing, and *Precision Isosurface (FEA Grade)* via Delaunay level-set refinement. |
| **Universal Multi-Step Undo** | Revert every repair, remesh, transform, or face deletion with `Ctrl+Z` / `Cmd+Z`. |

For detailed algorithmic descriptions of all repairs and diagnostics, see **[docs/FEATURES.md](docs/FEATURES.md)**.

---

## Python Module (Blender)

`src/python/` builds `_mesheal`, a nanobind module that exposes the checks and repairs to Python
(numpy in, numpy out, with cancellable background jobs). It powers the Blender extension in
`blender/`. Dev build on macOS against Blender's bundled Python:

```bash
scripts/build-blender-module.sh                       # builds build_python/, copies the .so into blender/mesheal_tools/
ctest --test-dir build_python -R "python|blender" --output-on-failure --timeout 300
```

See [docs/blender.md](docs/blender.md) for the extension and its limitations.

## Documentation

**Full documentation: <https://jsweezy.github.io/mesheal/>** (built from [`docs/`](docs/)).

- **[Getting started](docs/getting-started.md)**: a five-step walk-through of the desktop app.
- **[Why Mesheal](docs/why.md)**: the simulation problem, multi-scale welding, and design principles.
- **Desktop app**: [window tour](docs/gui/overview.md), [Mesh Health](docs/gui/mesh-health.md), [repairing](docs/gui/repairs.md), [shells](docs/gui/shells.md), [units & preferences](docs/gui/units-and-preferences.md), [navigation & shortcuts](docs/NAVIGATION.md), [desktop integration](docs/DESKTOP_INTEGRATION.md).
- **[Command line](docs/CLI.md)**: headless actions, options, safe output naming, JSON output and exit codes.
- **[Python module](docs/python.md)**, **[Blender extension](docs/blender.md)** and **[benchmarks](docs/BENCHMARKS.md)**.
- **[Feature reference](docs/FEATURES.md)**: welding tolerances, checks, and repair algorithms.
- **[Contributing](CONTRIBUTING.md)** and **[release process](RELEASE_PROCESS.md)**.

In the app, see **Help > Getting Started**, **Help > Navigation & Shortcuts** (`F1`), and
**Help > Online Documentation**. On the command line, run `mesheal --help`.

---

## Tech Stack & Dependencies

- **Language Standard**: C++20 (`namespace mesheal`) & Python 3.10+
- **Build & Test**: CMake (3.20+), CTest, Catch2 v3, and Python `unittest`
- **Visualization & UI**: Polyscope, Dear ImGui, and portable-file-dialogs
- **Geometry & Mesh Kernels**: CGAL (6.2+), Eigen3, libigl, and Manifold

---

## License

Mesheal is licensed under the **[GNU General Public License v3.0 or later (GPL-3.0-or-later)](LICENSE.md)** in accordance with the inclusion of CGAL's `Polygon_mesh_processing` package.
