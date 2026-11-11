# Repairing

Mesheal offers repairs at three levels: everything at once, one kind of problem, or one defect.
All of them follow the same rules:

- **Preview first.** Nothing changes until you press **Apply** (or **Commit Repair** in the
  Inspector).
- **Numbers before and after.** Volume, surface area, how far the surface moved, whether the
  result is watertight, and which defects remain.
- **One undo step per Apply.** `Cmd/Ctrl+Z` takes it back. Try another method and compare.
- **Scope follows the selection.** Repairs work on the selected faces, otherwise on the selected
  shells, otherwise on the whole mesh. Hidden shells are never changed.

Most repair panels have a **Preview** switch: *Result* (the repaired mesh), *Original*, or *Both*
(the faces the repair removes are ghosted behind the result). *Color by deviation* colors the result
by its distance from the original surface.

## Everything: Fix All

**Repair > Fix All...** (`Shift+F5`, or the green **Fix All...** button in Mesh Health) opens a plan
of every step. Opening it runs nothing.

Each pass runs these steps in order:

| # | Step | Can you turn it off? |
|---|---|---|
| 0 | Weld vertices that are closer than a tiny tolerance (closes hairline cracks) | yes |
| 1 | Remove collapsed and duplicate faces | always runs |
| 2 | Separate touching solids | always runs |
| 3 | Delete tiny floating shells | yes |
| 4 | Remove fins and internal walls | yes |
| 5 | Fix normals (cavities stay inward) | always runs |
| 6 | Close cracks | yes |
| 7 | Heal thin faces | yes |
| 8 | Fill holes | always runs |
| 9 | Repair intersecting islands within a shell | always runs |
| 10 | Merge shells that overlap (a union; cavities are kept) | yes; ticked automatically when shells overlap |
| 11 | Clean up the merged result | runs only after a merge |

Each row shows what Mesh Health finds now and has a tooltip saying what the step can change.
**Passes** (default 3, set in Preferences > Fix All) is how many times the sequence may repeat. It
stops early when the mesh is clean or a pass changes nothing.

- **Preview Fix All** runs every pass on a copy and shows the result with before/after numbers.
- **Preview one pass** (or **Repair > Fix All, One Pass...**, `Alt+Shift+F5`) runs the sequence once,
  so you can watch each step's effect.
- **Compare with rebuilds** runs Fix All and the whole-surface rebuilds side by side, so you can
  choose between *repair* (keep my geometry) and *rebuild* (new watertight surface).

Warnings in orange (for example, a shell that might be inside out) come with a **Show them** button
that selects and frames the shells concerned.

## One kind of problem

Every Mesh Health row's tool has a green **Fix All** button, also under the **Repair** menu.

| Row | What *Fix All* does |
|---|---|
| Thin / sliver faces | Flips or collapses long, thin triangles, keeping the shape. Rejects a heal that changes the volume by more than 0.5 % or moves the surface more than 1 % of the model's size. |
| Non-manifold edges / vertices | Splits them by copying vertices in place. The geometry does not move. |
| Flipped normals | Flips the faces wound against the majority of their shell. Never reverses a whole shell. |
| Inverted shells | Reverses the shells that are inside out. Never touches a cavity or a nested solid. |
| Degenerate faces | Removes faces that repeat a vertex, then heals zero-area faces by an edge flip or collapse (never by deleting them, which would open a crack). |
| Duplicate faces | Keeps one copy; for opposite windings keeps the one its neighbours agree with. |

**Holes** and **Intersecting faces** have **Repair All...** instead, which compares methods side by
side before you apply one:

- **Repair All Holes:** fill each hole with its recommended method, a winding-number rebuild, or
  Shrink-Wrap.
- **Repair All Intersections:** patch each intersecting island, self-union (cut along every crossing
  and keep the outer surface), a winding-number rebuild, or Shrink-Wrap. **Compare all** computes
  every method and lists them with volume, area, deviation, watertightness and remaining crossings.

**Fix Selected** applies the row's repair to the selected defects only.

## One defect at a time: the Inspector

Focus one defect (click it in 3D or in the list, or press `N`) and the **Inspector** opens at the
bottom of the tool, with the repairs that apply to it and a live preview. **Commit Repair** (`R`)
applies the chosen one.

=== "Thin / sliver faces"

    The Inspector names the face's type and recommends a strategy:

    - **Flip Longest Edge**: best for flat caps; no vertex moves.
    - **Collapse Shortest Edge**: best for needles.
    - **Snap Collinear Vertex**: for a cap on a feature crease.
    - **Remove Degenerate Flap**: for an isolated or redundant face.

=== "Holes"

    - **Fill Flat**: missing flat CAD faces.
    - **Fill & Refine**: fill, then match the surrounding triangle size.
    - **Fill Smooth**: curved surfaces and scans.
    - **Close Crack**: unwelded seams and T-junctions; moves rim vertices together without adding
      faces.

    A scorecard shows each fill's area, volume change and whether it crosses other faces.

=== "Intersecting faces"

    The focused face's *island* (the connected patch of crossing faces) can be repaired with:

    - **Remove & Fill Flat**: CAD folds.
    - **Remove & Fill Smooth**: curved surfaces.
    - **CGAL Local Repair**: grazing kinks.
    - **Cut & Merge Walls**: one wall pushed through another.

## Whole-surface rebuilds

**Repair > Remesh & Watertight Rebuild** builds a new surface. The result is watertight and
manifold, but every triangle is new.

| Method | Best for | Watch out for |
|---|---|---|
| **Shrink-Wrap (Outer Envelope)** | Wrapping an outer casing around messy geometry and thin, zero-volume sheets | Slightly inflates the model and fills every internal cavity |
| **Voxel Heal & Remesh (Fast)** | Organic models, 3D scans, quick manifold conversion. Keeps cavities. | Very sharp edges may be slightly softened |
| **Precision Isosurface (FEA Grade)** | Engineering parts, CAD exports, finite-element meshes. Triangles with angles of at least 30° that follow curves and fillets. | Slower than the voxel method |

See [Feature reference](../FEATURES.md#34-remesh-watertight-rebuild-pipelines) for the algorithms.

## Other mesh operations

Under **Mesh > Modify**, each with a preview and one undo step:

- **Decimate**: reduce the face count (ratio 0.01 to 1).
- **Merge by Distance**: merge vertices closer than a distance, removing faces that collapse. Keeps
  open edges and cracks by default.
- **Weld**: join open boundary vertices across seams and cracks without touching interior faces.
- **Delete Selected Faces** (`X` asks first; `Del` / `Backspace` delete at once).
- **Remove Small Shells**: selects shells under a face count in the Shells panel, so you can review
  them before pressing Delete.

Under **Mesh > Transform**: **Set Origin to Selection** (moves the mesh so the selected faces'
centroid is at the origin) and **Translate / Rotate** (typed values, translation first, then
rotation about X, Y and Z, with a preview).
