# Ogun Feature Reference & Repair Algorithms

This document provides detailed algorithmic explanations for Ogun's mesh diagnostics, welding tolerances, repair operators, and surface rebuild pipelines.

---

## 1. Edge-Aware Multi-Scale Vertex Welding

Raw STL files are unindexed soups of detached triangle facets ($3|F|$ coordinates). On file loading, Ogun welds duplicate vertex coordinates into a shared-vertex indexed mesh using local coordinate-relative tolerances:
- **Relative Tolerance**: $10^{-9}$ of the local coordinate magnitude.
- **Absolute Floor**: $10^{-9}\text{ cm}$ atomic coordinate floor to prevent collapse near the origin.
- **Edge-Aware Guard**: Two corners belonging to the same triangle are never merged into the same vertex index, strictly preserving zero-area and thin sliver faces for user inspection.
- **Topology-Aware Contact Separation**: Solids that only touch along an edge or at a vertex (which geometric position welding cannot distinguish from a structural connection) are separated by allocating distinct vertex indices, avoiding false non-manifold edge or vertex diagnostics.

---

## 2. Mesh Health Diagnostics & Defects

The **Mesh Health** panel provides real-time diagnostics across ten defect categories:

1. **Holes (Open Edges)**: Boundary halfedges without an opposite face. Grouped into hole loops and classified as *Flat*, *Curved*, *Crack*, or *Open Chain*.
2. **Non-Manifold Edges**: Edges shared by 3 or more faces (e.g. internal partitions or touching solid contacts).
3. **Non-Manifold Vertices**: Vertices whose incident triangles form multiple distinct umbrella fans (pinch points / bow-ties).
4. **Flipped Normals**: Triangles wound against the orientation majority of their connected shell.
5. **Inverted Shells**: Outward shells wound inside out (negative volume without a container). Cavities inside solids are distinguished via Rule O nesting analysis.
6. **Shells**: Connected topological components, ordered largest first with signed volumes and bounding box dimensions.
7. **Self-Intersections**: Triangles intersecting or overlapping coplanar faces within an intersection tolerance. Evaluated using a fast AABB bounding tree and Möller's triangle-triangle intersection algorithm.
8. **Thin / Sliver Faces**: Triangles exceeding aspect ratio limits ($\ell_{\max} / h_{\min}$, where $h_{\min} = 2A / \ell_{\max}$, default 100:1), classified into *Needles*, *Caps*, *Zero-Area*, and *Small*.
9. **Degenerate Faces**: Triangles with repeated vertex indices or height below numerical precision ($10^{-10}\ell_{\max}$ or $2^{-40}$ times coordinate span).
10. **Duplicate Faces**: Coincident triangles sharing the same 3 vertices.

---

## 3. Repair Strategies: Surgical vs. Global

### 3.1 Fix All Pipeline
The **Fix All** operator executes an automated multi-pass repair engine:
1. Weld open cracks up to tolerance.
2. Remove collapsed zero-area and duplicate faces.
3. Split non-manifold touching edges and vertices.
4. Prune tiny disconnected debris shells and dangling fins.
5. Unify face normal orientation and reverse inverted shells.
6. Heal thin/sliver needles and caps via iterative edge flips and safe collapses.
7. Fill open boundary holes.
8. Heal internal self-intersection islands.
9. Merge overlapping outer shells via exact CSG boolean unions while preserving internal cavities.

### 3.2 Single-Face Surgical Repairs
Targeted Euler operators for localized defects:
- **Flip Longest Edge**: Flips crease caps and shallow dihedral angles without moving vertices.
- **Collapse Shortest Edge**: Collapses sliver altitudes to heal needles, with link condition checks to prevent non-manifold pinch points.
- **Snap Collinear Vertex**: Merges inline boundary vertices along straight edges.
- **Remove Degenerate Flap**: Excises zero-area and dangling boundary flaps.

### 3.3 Hole Filling
- **Fill Flat**: CGAL minimum-weight triangulation across planar rims.
- **Fill & Refine**: Triangulates and refines patch density to match adjacent wall faces.
- **Fill Smooth**: Refines and fairs the patch to match local surface curvature.
- **Close Crack**: Snaps opposing boundary edges together across narrow gaps without introducing new faces.

### 3.4 Remesh & Watertight Rebuild Pipelines
For severely corrupted meshes, Ogun provides whole-surface reconstruction pipelines guaranteed to produce watertight, 2-manifold shells:
- **Shrink-Wrap (Outer Envelope)**: Evaluates `CGAL::alpha_wrap_3` with customizable detail ($\alpha$) and offset parameters.
- **Pipeline A — Voxel Heal & Remesh (Fast)**: Evaluates generalized winding numbers ($W = 0.5$) on a 3D grid, extracts the Marching Cubes isosurface, applies volume-preserving Taubin ($\lambda = 0.50, \mu = -0.53$) smoothing, and runs isotropic remeshing.
- **Pipeline B — Precision Isosurface (FEA Grade)**: Traces exact winding-number level sets via `CGAL::make_mesh_3` with Delaunay refinement ($\ge 30^\circ$ angles) adapting to curvature via chord tolerance, followed by tangential relaxation.
- **Self-Union**: Autorefines intersecting triangles and extracts the outer boundary via winding number evaluation.

---

## 4. Selection & Blender-Style Transformation

- **Blender Selection**: Box select (`B`), circle select (`C`), select all (`A`), clear selection (`Alt+A`), invert selection (`Ctrl+I`).
- **Origin to Selection**: Sets mesh origin to the area-weighted centroid of active facet selections.
- **Translate / Rotate**: Precise numeric translation ($X, Y, Z$) and Euler rotation in degrees ($R_z R_y R_x$) with interactive 3D preview and universal undo.
