# Mesheal Releases

Release notes, newest first. Versions are simple integers (v1, v2, ...) tagged in git as `v#`. See [the release process](https://github.com/jsweezy/mesheal/blob/main/RELEASE_PROCESS.md) for how to cut a release.

## v6 — 2026-11-21

Fix All repairs 92 of 131 broken files in the FixSTL 200-file benchmark (v5: 86), still without damaging any of the 69 healthy files, and every file gets the same result on every run. See the [benchmark results](https://jsweezy.github.io/mesheal/BENCHMARKS/).

- Fix All cuts a closed part that passes through itself along every crossing and keeps its outer surface. Cavities, open parts and other parts are left alone. Turn it off with `--no-unite-shells`, the Fix All options in the app, or the Blender panel.
- Fix All unfolds faces that fold back onto each other along a shared edge.
- New hole fill, Cap Cut Plane: closes the base of a model that was cut flat, and leaves a ring's through-hole open. Fix All uses it on its own when several openings share one plane; it is also in the app's Holes list.
- Heal thin faces no longer moves the edge of a flat opening, so it can still be capped.
- Hole filling gives the same result on every run.
- When Fix All cannot fill a hole, its report says why.
- `--trace` on the command line, and `fix_all.trace_steps` in Python, measure the whole mesh after every Fix All step. Python also gains `measure()`, `crossing_pairs()` and `shell_ids()`.
- View > Reload (Cmd/Ctrl+R) reloads the STL file from disk.

## v5 — 2026-11-16

Fix All repairs more broken files and no longer crashes or hangs on any file in the FixSTL 200-file benchmark: 86 of 131 broken files repaired, none of the 69 healthy files damaged. See the [benchmark results](https://jsweezy.github.io/mesheal/BENCHMARKS/).

- Fix All merges parts that cross each other on its own, so a batch of files needs no manual choice. A solid wholly inside another is kept. On the command line, `--merge-shells` and `--no-merge-shells` override the choice, and `--report` lists every step Fix All ran.
- Thin faces are repaired on meshes with non-manifold edges or pinched vertices elsewhere; those faces are left as they are and counted in the report.
- Fixed crashes and endless runs in Fix All on several large or badly broken meshes, and a stall on very large files.
- The intersection repair no longer accepts a result that shrinks the part.
- `--dedup` on the command line only welds nearby vertices and removes duplicate and collapsed faces.
- Rotate the view with the arrow keys.
- New eel app icon and logo, and two eel example models.
- Benchmark scripts for the FixSTL test set, with a time limit per file.

## v4 — 2026-11-13

First public release, with ready-to-run downloads and a Blender extension.

- Downloads for Windows (an installer with a Start menu entry, "Open with" for `.stl` files and an uninstaller, or a zip to run without installing), macOS (Apple Silicon) and Linux (AppImage).
- Blender extension for Blender 4.2 and later on Windows, macOS and Linux: check and repair meshes inside Blender, with each repair checked again afterwards. Install it with Edit > Preferences > Get Extensions > Install from Disk.
- Precision Isosurface uses every processor core on Windows and in Blender 5.1 and later, as it already did on macOS and Linux.
- Docker image and a command-line-only build for Linux.
- Online documentation with a Getting Started guide; Help menu links to it, and `--help` lists every option.
- Edge width sliders snap to standard line widths; the face selection shows over Custom back faces.
- Cleaner app icon.

## v3 — 2026-11-07

First release as Mesheal (formerly Ogun).

- Renamed the project, app (`Mesheal.app`), executable, namespaces and bundle identifier to Mesheal.
- New application icon.
- Saved preferences and recent files move automatically from `~/.config/ogun` to `~/.config/mesheal`.

## v2 — 2026-11-07

Final release under the name Ogun; the project continues as Mesheal.

- Grow and shrink the hole rim selection with `+` and `-` before Delete.
- Box and circle selection no longer pick faces behind or partly behind the repair patch.
- Darker and Custom back faces stay visible when faces are selected.
- Selected hole-rim faces use the selected faces color.
- UI theme with shared widget styling and saved preferences.

## v1 — 2026-10-28

First release.

- Loads binary and ASCII STL with edge-aware, topology-aware multi-scale vertex welding; slivers and zero-area faces are preserved for inspection.
- Informational file units (`mm`, `cm`, `m`, `in`, `ft`) that never rescale geometry; explicit Scale Mesh Coordinates action.
- Mesh Health panel: holes, non-manifold edges and vertices, flipped normals, shells, thin / sliver faces, and self-intersections, with histograms and Next / Prev browsing for huge defect counts.
- Blender-style selection-centric 3D navigation, rectangle face selection, clipping plane, and "." focus.
- Sliver repair (caps, needles, microscopic triangles, dangling flaps) for single faces or all at once, with live preview.
- Hole filling, non-manifold edge / vertex splitting, and intersection repair, each global or surgical.
- Mesh > Decimate, Weld / Merge Vertices, Delete Selected Faces (`X`), Alpha Wrap clean-up, and Merge Shells.
- Mesh > Transform: Set Origin to Selection and Translate / Rotate with preview.
- Universal multi-step undo (`Ctrl+Z` / `Cmd+Z`) for every mesh modification.
- File > Open Recent (last 10 STLs), Load Sample meshes, and Save / Save As in binary or ASCII STL.
- Preferences and display styles saved to `~/.config/mesheal/preferences.conf`.
