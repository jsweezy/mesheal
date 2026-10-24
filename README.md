# Ogun — STL Repair Tool for Simulation Meshes

**Ogun** (named after the Orisha of iron, metalwork, and forging) is an open-source C++ desktop application and library for inspecting, diagnosing, and repairing STL surface meshes for Monte Carlo particle transport (built as a companion to **Legba**, as well as MCNP, OpenMC, DAGMC, Geant4, and Serpent) and computational physics workflows.

Unlike 3D printing repair tools—which often voxelize or alter geometry in ways that hurt ray-tracing accuracy—or complex 3D modeling suites like Blender, **Ogun** provides a focused workflow: **load an STL with multi-scale edge-aware welding, verify physical units without accidental scaling, check manifold/geometric integrity, click issues to highlight offending facets in 3D (with `.` to zoom to selection), and apply targeted repairs.**

## Key Documentation

- **[VISION.md](VISION.md)**: Motivation, user workflow, multi-scale tolerance design, architectural philosophy, and open-source licensing strategy.
- **[PLAN.md](PLAN.md)**: Incremental development roadmap starting with the Minimal Viable Tool (Milestone 1), module layout, and TDD strategy with Catch2.

## Tech Stack

- **Language & Namespace**: C++20 (`namespace ogun`)
- **Build & Test System**: CMake (3.20+, `FetchContent`), CTest, and **[Catch2](https://github.com/catchorg/Catch2)** (Test-Driven Development with co-located `Test/` directories inside every module)
- **3D Visualization & UI**: [Polyscope](https://polyscope.run/) + [Dear ImGui](https://github.com/ocornut/imgui) + [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs)
- **Geometry Processing**: [libigl](https://libigl.github.io/) + [Eigen](https://eigen.tuxfamily.org/) + [Manifold](https://github.com/elalish/manifold) (with planned evaluation of [CGAL](https://www.cgal.org/) for advanced repairs)

## Repository Layout

Each functional subdirectory under `src/` contains its own `Test/` directory so modules and their Catch2 unit tests remain self-contained and portable:

```text
STL_Repair_Tool/
├── CMakeLists.txt
├── README.md
├── VISION.md
├── PLAN.md
├── cmake/
│   └── Dependencies.cmake
└── src/
    ├── mesh/           # Indexed Face-Vertex Mesh, Units & ToleranceConfig (+ src/mesh/Test/)
    ├── io/             # STL reading, local edge-aware welding & sample meshes (+ src/io/Test/)
    ├── diagnostics/    # Manifold & thin/sliver face detection (+ src/diagnostics/Test/)
    └── app/            # Headless AppController (+ src/app/Test/) & Polyscope/ImGui viewer
```

## Building and Running Tests (Milestone 1)

```bash
# Configure the project (fetches Catch2, libigl, Manifold, Polyscope via FetchContent)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build libraries, Catch2 unit tests, and GUI application
cmake --build build -j

# Run all Catch2 unit tests via CTest
ctest --test-dir build --output-on-failure

# Launch the Ogun viewer (with optional STL path)
./build/src/app/ogun_app path/to/model.stl
```

## License

Developed under a permissive **BSD-3-Clause** license using MIT/Apache-2.0/MPL-2.0/BSL-1.0 dependencies (`Catch2`, `Polyscope`, `Dear ImGui`, `libigl`, `Eigen`, `Manifold`, `portable-file-dialogs`). If GPLv3+ components from `CGAL` are integrated in future milestones, the project license will be updated accordingly (see [VISION.md](VISION.md)).
