# Mesh Health

The Mesh Health panel (top right) says what is wrong with the mesh. Each row is one kind of problem,
with a count.

## Rows

| Row | Counts | What it means |
|---|---|---|
| **Holes (open edges)** | holes | Loops of edges used by only one triangle, so the mesh is not watertight. Each hole is classed by shape: *Flat*, *Curved*, *Crack* (a narrow gap) or *Open chain*. |
| **Non-manifold edges** | edges | Edges shared by three or more triangles: solids touching along an edge (*Touching*), fins and internal walls (*Fin / other*). |
| **Non-manifold vertices** | vertices | Two or more fans of triangles that meet at a single vertex (bow-ties). |
| **Flipped normals** | faces | Triangles wound opposite to their neighbours. |
| **Inverted shells** | shells | Closed shells wound inside out: facing inward with nothing around them and nothing inside them. A hollow inside a solid (a *cavity*) faces inward on purpose; it is correct and not counted. |
| **Shells** | shells | Separate connected pieces. Several can be intended, so this row is information only. |
| **Intersecting faces** | faces | Triangles that pass through other triangles, or overlap them in the same plane (*Crossing*, *Coplanar overlap*). |
| **Thin / sliver faces** | faces | Triangles stretched more than the aspect limit (default 100 : 1), or with no reliable normal. Classed as *Zero-area*, *Needle* (one very short edge), *Cap* (one angle near 180°) or *Small*. |
| **Degenerate faces** | faces | Triangles that repeat a vertex, or have no reliable normal (flat, or below rounding at their position). |
| **Duplicate faces** | faces | Triangles with the same three vertices as another triangle (*Same winding* or *Opposite winding*). |

!!! info "Holes vs. non-manifold edges"
    These are separate checks. A *hole* edge has **one** triangle; a *non-manifold* edge has
    **three or more**. A watertight, 2-manifold mesh has exactly **two** triangles on every edge. The
    status bar's **Watertight** badge is green only when both rows are 0.

When an STL is opened, solids that only touch along an edge or at a corner are kept apart. So
*Touching* edges and non-manifold vertices normally appear only after an edit creates them.

## Reading a row

| Look | Meaning |
|---|---|
| Hollow grey dot, `-` | Not checked yet |
| Spinning dot, `...` | Checking now (progress and Cancel are in the status bar) |
| Green `0` | No problems of this kind |
| Orange count | Problems found |
| Greyed count with `?` | The mesh changed since this row last ran (*stale*) |

Hover a row for a one-line explanation.

## Checking

- **Check Mesh** (`F5`, or the button at the top of the panel) runs every row in the background. The
  window stays responsive, and each row fills in as it finishes.
- Each row has an **Auto** box. With Auto on, the row re-runs after every edit (repair, Fix All, Mesh
  operation, Undo). These checks take milliseconds even on large meshes.
- *Intersecting faces* has Auto **off** by default, because on big meshes it takes most of the
  checking time. After an edit it turns stale until you press **Check Mesh** or **Re-check** in its
  tool.
- The defaults are in **Preferences > Diagnostics**, including *Run Check Mesh after loading a file*.

## The row's tool

Click a row to select it. Its defects are tinted in 3D, and its tool opens below Mesh Health. Click
the row again to close it. The tool has, from top to bottom:

1. **A description** of the check, and a warning when the row is stale.
2. **Settings:** the check's thresholds, for example the aspect limit for thin faces. Changing them
   re-runs the check.
3. **A filter** by sub-type (for example *Zero-area / Needle / Cap / Small*). The filter applies to
   the list, the 3D tint, Next / Previous and *Fix Selected*.
4. **Actions:** the green **Fix All** (or **Repair All...** for holes and intersections),
   **Fix Selected**, and **Re-check** (re-runs this row only).
5. **The defect list**, or a summary when there are more than 100 (the *list cap*, set in
   Preferences). Above the cap, defects are still tinted in 3D; reach individual ones by clicking
   them, with `N` / `Shift+N`, or through the histogram, which opens a list for one bin.
6. **The Inspector**, when one defect is focused: its details, the repairs that apply to it, and a
   live 3D preview. See [Repairing](repairs.md#one-defect-at-a-time-the-inspector).

## Mesh Info

**Mesh Info** (below the rows, or **Mesh > Mesh Info**) lists vertices, triangles, size, bounding
box, surface area and volume, and how they changed since the file was loaded.

## Scope: which part of the mesh is checked

When the mesh has several shells and some are selected in the [Shells](shells.md) panel, Mesh Health
covers only those shells, and a blue *Scope* line says so. Hidden shells are never checked, listed or
edited, but they are still saved.
