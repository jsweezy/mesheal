# Mesheal Releases

Release notes, newest first. Versions are simple integers (v1, v2, ...) tagged in git as `v#`. See [RELEASE_PROCESS.md](RELEASE_PROCESS.md) for how to cut a release.

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
