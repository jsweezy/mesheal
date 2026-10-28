# Ogun — STL Repair Tool for Simulation Meshes

**Ogun** (named after the Orisha of iron, metalwork, and forging) is an open-source C++ desktop application and geometry library for inspecting, diagnosing, and repairing STL surface meshes for engineering analysis, specifically targeting ray-tracing applications, such as Monte Carlo particle transport that need manifold (water-tight) meshes.

Unlike 3D printing repair tools—which often voxelize or aggressively alter geometry in ways that compromise ray-tracing accuracy—or complex 3D modeling suites like Blender, **Ogun** provides a focused, deterministic workflow: **load an STL with multi-scale edge-aware welding, verify physical units without accidental scaling, diagnose manifold and sliver integrity, inspect defects in 3D with Blender-style navigation, and apply targeted repairs with live previews and universal undo.**

---

## Key Features & Capabilities

- **Edge-Aware Multi-Scale Vertex Welding on Load**: Converts raw unindexed STL triangle soup into a shared-vertex mesh using local coordinate-relative tolerances ($10^{-9}$ relative, $10^{-9}\text{ cm}$ atomic floor). An **edge-aware guard** guarantees that zero-area and sliver triangles are never collapsed on load, preserving defects for inspection. The weld is then **topology-aware**: solids that only touch along an edge or at a vertex (which a position weld cannot tell from a real connection) keep their own copies of the shared vertices, so they never show up as non-manifold edges or vertices.
- **Physical Units Management (Without Accidental Scaling)**: Switch active File Units (`mm`, `cm`, `m`, `in`, `ft`) to interpret coordinates and tolerances without altering vertex positions. Rescaling geometry coordinates is handled through an explicit, separate action.
- **Mesh Health Panel**: A live status list (like Blender's 3D-Print Toolbox) of holes (open edges), non-manifold edges and vertices, flipped normals, shells, intersecting faces, thin / sliver faces, degenerate and duplicate faces. Checks run in the background after loading (F5 re-runs them), Auto rows re-check after every edit, and a status-bar badge shows whether the mesh is watertight.
- **Self-Intersection Detection**: Finds faces that pass through (or overlap coplanar with) other faces, the way Blender's 3D-Print Toolbox does: a multi-threaded AABB-tree self-overlap followed by Möller's fast triangle-triangle test. Its Auto checkbox is off by default because it is the slowest check (about 45 ms on the 69k-triangle bunny in a release build). Try *File > Load Sample > Self-intersecting box* (`examples/self_intersecting_box.stl`).
- **Intersection Repair, Global or Surgical**: *Repair All Intersections* (the Tool's green button or the Repair menu) computes whole-mesh repairs side by side: *Patch each island*, *Self-union*, *Winding-number rebuild* and *Alpha wrap*. A table shows each result's volume and area change, deviation, whether it is watertight, and the crossings left. Show the result, the original or both (optionally colored by deviation), then Apply the one that suits the model. Clicking an intersecting face opens its island with four local repairs (Remove & Fill Flat / Smooth, CGAL Local Repair, Cut & Merge Walls), a live preview and a scorecard. Ogun offers coarse and surgical fixes alike and lets you decide. Try *File > Load Sample > Folded cube (intersection islands)*.
- **Hole Repair, Global or Surgical**: Open edges are grouped into holes, one row each in the *Holes (open edges)* tool, and classified as *Flat* (a missing CAD face or end cap), *Curved*, *Crack* (a gap much narrower than its faces, e.g. a T-junction between unwelded CAD patches) or *Open chain* (not a loop). Focusing a hole (its row, Next / Prev, or a click on its rim) opens four repairs: *Fill Flat* (CGAL `triangulate_hole`, planar rims triangulated in their plane), *Fill & Refine* (matches the mesh density), *Fill Smooth* (refined and faired into the surrounding curvature) and *Close Crack* (snaps the rim across the gap and stitches it, adding no faces). Each has a live preview and a scorecard (faces and vertices added, area, smallest angle, whether it closes the hole, and whether the patch crosses any other face, which disables Commit). *Repair All Holes* compares *Fill each hole* with the winding-number rebuild and alpha wrap. *Mesh > Alpha Wrap...* also wraps the whole mesh (or the shells in scope) on its own, with alpha and offset controls and the same preview. Fills keep every existing face and vertex exactly. Try *File > Load Sample > Holes* or the Stanford bunny (five holes in its base).
- **Decimate, Weld / Merge Vertices and Delete, on the Whole Mesh or Selected Faces**: *Mesh > Decimate...* (ratio 0.01 to 1, default 0.5; hole rims and the edge of a selection stay fixed) and *Mesh > Weld / Merge Vertices...* (distance slider 0.0001 to 10 in file coordinates, default 0.05, unit-independent; removes the faces that collapse; *Keep surfaces apart* only joins vertices along an edge or across a crack, so touching surfaces are not pinched), each with a preview and before / after numbers. Select faces Blender-style: `Shift + click`, `B` box select, `C` circle (paint) select, `A` / `Alt + A`; hidden shells are left out and Ghost (`G`) selects through. `Del` (*Mesh > Delete Selected Faces*) removes the selected faces; their color is set in *Display > Base mesh*. All of these live in the Mesh menu's *Modify* section.
- **Transform**: *Mesh > Transform > Set Origin to Selection...* moves the whole mesh so the area-weighted centroid of the selected faces (a single facet: its centroid) lands on the origin, after a confirmation showing the centroid. *Mesh > Transform > Translate / Rotate...* takes typed X / Y / Z translations and rotations in degrees about the world X, Y and Z axes (translation first, then X, Y, Z rotation: p' = Rz Ry Rx (p + t)) with a live preview, then applies them as one undoable step. The face selection is kept.
- **Non-manifold Edge & Vertex Splitting**: *Fix All* / *Fix Selected* on the non-manifold rows separate solids that only touch along an edge (*Touching* edges) or at a vertex (bow-ties) by giving each its own copies of the shared vertices at the same positions, so no coordinate moves. Around a touching edge the faces are sorted radially and paired by winding into closed solids; fins, internal walls and other edges splitting cannot fix are listed as *Fin / other* and left alone. Opening an STL already does this split, so these fixes matter only for contacts that an edit creates. Try *File > Load Sample > Touching cubes and a fin*.
- **Merge Shells**: select shells in the Shells panel, right-click > *Merge...*. A checklist built from Mesh Health shows whether each shell is closed, consistently oriented, free of fins and free of self-crossings, then Manifold's exact boolean union joins the shells that overlap or touch, in the background and as one undoable step. Shells that only come close are not joined yet (closing small gaps is planned).
- **Deterministic Sliver Diagnostics**: Identifies degenerate, near-zero-area, and high-aspect-ratio sliver triangles via configurable Minimum Altitude ($h_{\min} = \frac{2A}{\ell_{\max}}$), Minimum Area ($A$), and Aspect Ratio thresholds, bucketed as Zero-area / Needle / Cap / Small.
- **Scales to Huge Defect Counts**: Above a list cap (100) a defect type becomes a 3D highlight plus a clickable worst-first histogram, Next / Prev (`N` / `Shift+N`) and 3D picking instead of an unusable list.
- **Selection-Centric 3D Viewport Navigation**: Blender-inspired camera controls:
  - Focus on selection with **`.`** (toggle between `+Normal` and `-Normal` views with `..` or `Shift+.`).
  - Automatic orbit pivot centered on selected facet centroids.
  - Dynamic 3D curve and point overlays ensuring sub-pixel slivers remain visible at any zoom level.
  - Ghost / semi-transparent mesh toggling and interactive clipping plane.
- **Synchronized Dual Mesh Architecture**: Combines `ogun::Mesh` (contiguous `Eigen` buffers for high-performance Polyscope rendering) with **CGAL** (`CGAL::Surface_mesh<EPICK::Point_3>`) for $O(1)$ halfedge queries and exact geometric predicates.
- **Interactive Single-Face Live Repair Preview**: Classifies defective facets into **Caps**, **Needles**, **Microscopic Triangles**, and **Dangling Flaps**. Generates a live 3D preview of the 2-ring neighborhood across targeted Euler operators (*Flip Longest Edge*, *Collapse Shortest Edge*, *Snap Collinear Vertex*, *Remove Degenerate Flap*) before committing.
- **Batch Sliver Healing**: **Fix All** (or **Fix Selected** for a hand-picked set) combining iterative Cap edge-flips, link-safe Needle edge-collapses, and CGAL `remove_almost_degenerate_faces` while protecting sharp feature creases.
- **Universal Multi-Step Undo (`Ctrl+Z` / `Cmd+Z`)**: Every operation modifying the mesh in memory (single-face repair commits, hole fills, batch repairs, non-manifold splits, coordinate scaling) pushes a snapshot to Ogun's undo history stack.
- **Open Recent (File > Open Recent)**: The last 10 STLs opened are remembered by absolute path in `~/.config/ogun/recent_files.txt` (or `$XDG_CONFIG_HOME/ogun`), so they open again whatever directory Ogun is launched from. Files that have moved are greyed out; *Clear Recent* empties the list.
- **STL Export (File > Save / Save As)**: Save repaired meshes in **Binary STL** (default) or **ASCII STL** formats.
- **Menu-Bar GUI**: `File Edit View Mesh Repair Help` menus, docked Display / Navigate / Structures panels on the left, Mesh Health and a per-defect Tool panel on the right, and a status bar ([UI_DESIGN.md](UI_DESIGN.md)).
- **Saved Preferences**: Everything in *Edit > Preferences* (weld floor, relative tolerance, the Thin / sliver faces defaults, Auto defaults, UI scale, caps) and every *Display* panel style (colors, edge widths, point radius, preview mode and opacities, base mesh style, background) is saved as soon as it changes to `~/.config/ogun/preferences.conf` (`$XDG_CONFIG_HOME/ogun/` when set, on macOS and Linux) and reused at the next start. The file is plain `key = value` text; *Preferences > Reset to defaults* restores the built-in values.

---

## Documentation

- **[docs/NAVIGATION.md](docs/NAVIGATION.md)**: 3D viewport mouse controls, keyboard shortcuts (`.`, `Shift+.`, `N`, `R`, `G`, `Esc`, `F5`, `Home`/`H`, `?`/`F1`), selection mechanics, and clipping plane usage.
- **[VISION.md](VISION.md)**: Physical simulation motivation, multi-scale tolerance philosophy, decoupled architecture, and licensing rationale.
- **[UI_DESIGN.md](UI_DESIGN.md)**: GUI design (P1-P3 built): menu bar, Mesh Health panel, per-defect Tool panels, operator panels, large-defect-count handling, implementation notes, and the remaining phases.
- **[PLAN.md](PLAN.md)**: Incremental development roadmap, module structure, and Test-Driven Development (TDD) harness with Catch2.
- **[TODO.md](TODO.md)**: Punch list tracking completed and upcoming features.
- **[RELEASES.md](RELEASES.md)**: Release notes for each version (simple integers, tagged `v1`, `v2`, ...).
- **[RELEASE_PROCESS.md](RELEASE_PROCESS.md)**: How to cut a release: where the version number and date are set in `CMakeLists.txt`, and the tag and push steps.
- **[LICENSE.md](LICENSE.md)**: Full text of the GNU General Public License v3.0.

---

## Tech Stack

- **Language & Namespace**: C++20 (`namespace ogun`)
- **Build & Test System**: CMake (3.20+), CTest, and **[Catch2](https://github.com/catchorg/Catch2)** (v3 TDD with co-located `Test/` suites in every module)
- **3D Visualization & UI**: [Polyscope](https://polyscope.run/) + [Dear ImGui](https://github.com/ocornut/imgui) + [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs)
- **Geometry Processing & Mesh Repair**:
  - **[CGAL](https://www.cgal.org/)** (v5.6+ / v6.x): `EPICK` exact predicates kernel, `Surface_mesh`, `Euler` local flips/collapses, and `Polygon_mesh_processing`
  - **[Eigen3](https://eigen.tuxfamily.org/)** & **[libigl](https://libigl.github.io/)**: Dense matrix storage (`V`, `F`, `N`) and STL I/O
  - **[Manifold](https://github.com/elalish/manifold)**: Watertight manifold mesh kernel & `MeshGL` interoperability
  - **[Boost](https://www.boost.org/)**: Graph concepts and data structures required by CGAL

---

## Repository Layout

Each functional directory under `src/` contains its own `Test/` suite to keep modules self-contained:

```text
ogun_stl_repair_tool/
├── CMakeLists.txt
├── README.md
├── LICENSE.md
├── VISION.md
├── PLAN.md
├── TODO.md
├── UI_DESIGN.md
├── assets/
│   └── icons/          # Application & toolbar icons (ogun.jpeg, ogun.png, ogun.icns)
├── docs/
│   └── NAVIGATION.md   # 3D viewport navigation & keyboard shortcuts guide
├── cmake/
│   └── Dependencies.cmake
└── src/
    ├── core/           # Mathematical constants & centralized default settings (+ src/core/Test/)
    ├── mesh/           # Indexed Face-Vertex Mesh, CGAL Interop, Units & ToleranceConfig (+ src/mesh/Test/)
    ├── io/             # STL reading, edge-aware welding, STL export, sample meshes & the config directory (+ src/io/Test/)
    ├── diagnostics/    # Mesh Health checks: topology, self-intersections, thin/sliver, degenerate, duplicates (+ src/diagnostics/Test/)
    ├── repair/         # Sliver healing, hole filling, intersection repair (islands, self-union, rebuilds, metrics) (+ src/repair/Test/)
    ├── app/            # Headless AppController (+ src/app/Test/) & Polyscope/ImGui viewer
    └── mockup/         # Historical clickable UI mockup of UI_DESIGN.md (fake defect data)
```

---

## Prerequisites & Building

### 1. Install Prerequisites

CGAL and Boost are required external dependencies. Other libraries (`Catch2`, `libigl`, `Eigen3`, `Manifold`, `Polyscope`) are fetched and built automatically via CMake `FetchContent`.

- **macOS** (Homebrew):
  ```bash
  brew install cmake cgal boost
  ```

- **Ubuntu / Debian**:
  ```bash
  sudo apt-get update
  sudo apt-get install -y cmake build-essential libcgal-dev libboost-all-dev
  ```

### 2. Configure, Build, and Test

```bash
# Configure the project
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build libraries, Catch2 unit tests, and the Ogun GUI application
cmake --build build -j 2

# Run all unit tests via CTest
ctest --test-dir build --output-on-failure -j 4

# Launch the Ogun viewer (optionally passing an STL file path)
./build/src/app/ogun [path/to/model.stl]

# Historical first-draft mockup of UI_DESIGN.md (fake defect data)
./build/src/mockup/ogun_ui_mockup [path/to/model.stl]
```

---

## License

Ogun is licensed under the **[GNU General Public License v3.0 or later (GPL-3.0-or-later)](LICENSE.md)**.

This license is adopted in accordance with the integration of **[CGAL](https://www.cgal.org/)**'s `Polygon_mesh_processing` package, which is licensed under **GPLv3+**.

### Third-Party Dependencies

All third-party libraries used in Ogun are licensed under compatible open-source licenses:

| Dependency | License | Role |
| :--- | :--- | :--- |
| **[CGAL](https://www.cgal.org/)** (`Polygon_mesh_processing`) | **GPL-3.0-or-later** | Mesh repair, degeneracy removal, normal computation |
| **[CGAL](https://www.cgal.org/)** (Kernel & Basic Library) | **LGPL-3.0-or-later** | Halfedge `Surface_mesh` data structure, Euler operators |
| **[libigl](https://libigl.github.io/)** | **MPL-2.0** | Matrix-based mesh geometry & STL parsing |
| **[Eigen3](https://eigen.tuxfamily.org/)** | **MPL-2.0** | Linear algebra, contiguous `V` and `F` matrix storage |
| **[Manifold](https://github.com/elalish/manifold)** | **Apache-2.0** | Watertight geometric manifold representations & `MeshGL` interop |
| **[Polyscope](https://polyscope.run/)** | **MIT** | 3D scientific visualization & viewport rendering |
| **[Dear ImGui](https://github.com/ocornut/imgui)** | **MIT** | Immediate-mode user interface panels |
| **[portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs)** | **WTFPL** | Native cross-platform file open/save dialogs n|
| **[Catch2](https://github.com/catchorg/Catch2)** | **BSL-1.0** | Unit testing framework |
| **[Boost](https://www.boost.org/)** | **BSL-1.0** | Graph algorithms & C++ utility libraries required by CGAL |
