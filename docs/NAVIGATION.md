# Ogun 3D Viewport Navigation & Shortcuts Guide

Ogun uses **Blender-inspired, selection-centric 3D camera navigation** tailored for inspecting both whole STL meshes and microscopic sliver/degenerate facets ($10^{-4}\text{ mm}$ and smaller).

---

## 1. Quick Reference: Mouse & Keyboard Controls

### Mouse Controls (3D Viewport)

| Action | Input | Behavior |
| :--- | :--- | :--- |
| **Orbit / Rotate** | `Left-Click Drag` | Orbits the camera around the active **Orbit Pivot** (respecting the active Polyscope camera style: *Turntable* or *Free*). |
| **Pan Camera & Pivot** | `Shift + Left-Click Drag`, `Right-Click Drag`, or `Middle-Click Drag` | Translates both the camera and the **Orbit Pivot** in the view plane. Pan speed scales automatically with the distance to the Orbit Pivot so the surface stays locked under the cursor at any zoom depth. |
| **Zoom (Dolly)** | `Scroll Wheel` or `Ctrl + Shift + Left-Click Drag` | Dollies the camera toward/away from the active **Orbit Pivot**. Zoom step size scales proportionally with the distance to the pivot so you never overshoot microscopic facets. |
| **Adjust Near Clip Plane** | `Shift + Scroll Wheel` | Manually shifts the camera near clipping plane (in addition to Ogun's automatic zoom-proportional near clip adjustment). |
| **Select Face / Clear Pick** | `Left-Click` / `Right-Click` | Left-clicking a face without dragging: if it is a defect of the selected **Mesh Health** row, it becomes the **focus** (highlighted, the **Orbit Pivot** moves to its centroid, and for thin / sliver faces the **Inspector** opens with a live 3D repair preview). This works even when the row has more defects than the list cap and no list is shown. With no row selected, clicking a defective face opens the row that contains it (thin / sliver faces first). Clicking a face of another type, or a clean face, only shows a status-bar message (a clean face also moves the Orbit Pivot). Right-clicking clears the Polyscope pick. |

---

### Keyboard Shortcuts (3D Viewport)

`Ctrl` is `Cmd` on macOS. Every button tooltip shows its key; the **Navigate** panel (left column) has a button for each camera key, so experts can close it.

| Shortcut | UI Equivalent | Behavior |
| :--- | :--- | :--- |
| **`.`** or **`Numpad .`** | Navigate > `+Normal`, View > Camera > Frame focused defect | **Frame Focused Facet (`+Normal` / Toggle):** Zooms in on the focused facet's centroid looking straight down its **outward positive normal (`+N`)**. Pressing **`.`** again while aligned with `+N` flips the camera to the **negative normal (`-N`)** side. Without a single focused face it frames the selected / visible defects (or the whole mesh). |
| **`Shift + .`** | Navigate > `-Normal`, Inspector > `Back side` | **Frame Focused Facet (`-Normal`):** Directly frames the focused facet from its **inward negative normal (`-N`)** side. |
| **`N`** / **`Shift + N`** | Navigate > `Next >` / `< Prev`, Edit > Select | **Next / Previous Defect:** Focuses the next (previous) defect of the selected Mesh Health row, worst first and within the active filter (sub-type or histogram bin), and flies the camera to it. Wraps around. |
| **`Home`** or **`H`** | Navigate > `Home`, View > Camera > Home | **Frame All:** Flies the camera back to the full-mesh home view and resets the **Orbit Pivot** to the mesh center. The Home tooltip shows the current Orbit Pivot. |
| **`G`** | View > Ghost Mesh, Display > Transparency | **Ghost:** Toggles the base mesh between opaque and the last transparency. |
| **`Esc`** | Edit > Select > None | **Staged clear:** The first press clears the defect selection and focus; the next press closes the Tool panel and deselects the row (or cancels an open Mesh operation). |
| **`F5`** | Mesh Health > `Check Mesh`, Repair > Check Mesh | **Check Mesh:** Runs every Mesh Health row in the background, including rows whose **Auto** is off. Progress and **Cancel** are in the status bar. |
| **`Ctrl + Z`** | Edit > Undo *label*, Inspector > `Undo` | **Undo Mesh Modification:** Reverts the last mesh modification (single-face repair, *Fix All*, *Fix Selected*, or *Apply Scale*) one step at a time, restoring the Mesh Health results, the selected row, the focused face, chosen repair strategy and live preview. |
| **`Ctrl + O`** / **`Ctrl + S`** / **`Ctrl + Shift + S`** | File > Open / Save / Save As | Open an STL; save over the current file (asks the first time); save as a new Binary or ASCII STL. |
| **`Ctrl + ,`** | Edit > Preferences | Opens the Preferences window. |
| **`?`** (`Shift + /`) or **`F1`** | Help > Navigation & Shortcuts | Opens or closes the in-app **3D Viewport Navigation & Shortcuts** window. |
| **`Ctrl + C`** / **`Ctrl + V`** | | Copy / paste the camera pose (Polyscope built-in). |

### Mouse in the Tool panel's defect list

| Input | Behavior |
| :--- | :--- |
| `Click` | Selects only that defect and focuses it. |
| `Ctrl/Cmd + Click` | Adds or removes a defect from the selection. |
| `Shift + Click` | Selects the range from the last clicked row. |
| `Double-Click` | Focuses the defect and flies the camera to it. |
| Click a column header | Sorts by that column (third click restores worst-first order). |
| Click a histogram bar (`Shift` for a range) or the `<` `>` arrows | Filters to that bin and selects its defects; its list opens once it holds no more than the list cap. |

---

## 2. Core Navigation Concepts (Blender Conventions)

### Auto-Orbit Around Selection Centroid
When you focus a defect (click its row in the Tool panel's list, click it in 3D, or press `N`):
- Ogun highlights the triangle's surface, edges, and vertices in 3D.
- The 3D **Orbit Pivot** (center of rotation) immediately moves to the triangle's true geometric vertex centroid:
  $$\mathbf{c} = \frac{\mathbf{v}_0 + \mathbf{v}_1 + \mathbf{v}_2}{3}$$
- Even before pressing `.` to zoom in, left-click dragging in the viewport orbits smoothly around the selected facet rather than swinging the facet out of view around the global mesh center.
- When a Mesh Health row is selected without a focus, the Orbit Pivot is placed at the center of the bounding box of its selected (or else all visible) defects. When the selection is cleared, it returns to the full mesh center.

### Inspecting Front (`+Normal`) vs. Back (`-Normal`) of a Facet
In STL repair, inspecting a defective facet from both its outward (`+N`) and inward (`-N`) sides helps determine whether neighboring faces are folded, inverted, or self-intersecting:
1. **First `.` press:** Flies the camera to $\mathbf{c} + r\,\hat{\mathbf{n}}$, looking along $-\hat{\mathbf{n}}$ directly at the front (`+Normal`) of the triangle.
2. **Second `.` press (or `..`):** Because the camera is already aligned with `+Normal` on that facet, pressing `.` again flips the camera to $\mathbf{c} - r\,\hat{\mathbf{n}}$, looking along $+\hat{\mathbf{n}}$ at the back (`-Normal`) of the triangle.
3. **Rotating away resets the toggle:** If you orbit the camera manually with the mouse to inspect an oblique angle, pressing `.` again returns you cleanly to the canonical `+Normal` view first.
4. **Direct `-Normal` access:** Pressing `Shift + .` always jumps straight to the `-Normal` side regardless of current camera angle.
5. **Collinear zero-area triangles:** If a defective triangle has three strictly collinear vertices ($\|(\mathbf{v}_1 - \mathbf{v}_0) \times (\mathbf{v}_2 - \mathbf{v}_0)\| = 0$), its geometric normal is undefined. Ogun automatically computes a viewing axis **perpendicular to the triangle's longest edge** so `.` and `Shift + .` still frame the degenerate segment broadside from opposite directions.

### Selecting & Repairing a Face in the Viewport
1. Select the **Thin / sliver faces** row in **Mesh Health**, then **left-click** a highlighted face in the 3D viewport, click its row in the Tool panel's list, or press `N`. Zero-area faces have no visible surface to click, so pick those from the list (the *Zero-area* filter) or with `N`.
2. The Tool panel's **Inspector** shows the face's detected type, maximum angle, and min/max edge ratio, and pre-selects the recommended strategy.
3. The preview display options live in the left column under **Display > Repair preview**. With **Show preview patch** enabled, the repaired 2-ring neighborhood is drawn in green with black edges (colors adjustable). Unsafe strategies show a red warning instead. **Mode** chooses how the patch is combined with the original surface:
   - **Cutout + Original Edges** (default): the replaced base faces are hidden, the opaque patch fills the gap without Z-fighting flicker, and the pre-repair edges are drawn as thin lines over it. Selecting it switches Polyscope to *Pretty* transparency. The pre-repair edges are thinner (*Original Edge Width (px)*, default `0.5`) and semi-transparent (*Original Edge Opacity*, default `0.5`) than the patch edges. An edge that survives the repair shows the patch edge color through and around it; an edge the repair removes shows only the pre-repair color.
   - **Translucent Base**: the whole base mesh is dimmed (*Base Mesh Opacity*, default `0.40`) so the old surface shows through where the repair changes the shape. Selecting it switches Polyscope to *Simple* transparency, which avoids flicker.
   - **Translucent Patch**: the base mesh stays opaque and only the patch is see-through (*Patch Opacity*).
4. **Commit Repair** applies the repair to the full mesh; the Auto rows of Mesh Health re-run. To repair several faces at once, select them (Ctrl/Shift+click, or a histogram bin) and press **Fix Selected**, which applies each face's recommended strategy as one undo step. Press **`Ctrl + Z`** / **`Cmd + Z`** (or **Undo**) to restore the face and try a different strategy.

### Depth-Scaled Zoom, Pan & Dynamic Highlight Sizing
- **Proportional Zoom & Pan:** Dolly and pan speeds scale with $\|\mathbf{p}_{\text{cam}} - \mathbf{p}_{\text{pivot}}\|$ (the distance from the camera to the Orbit Pivot). Whether you are viewing a $100\text{ mm}$ housing or a $10^{-4}\text{ mm}$ sliver triangle, scroll-zooming and panning move at a consistent screen-space rate.
- **Pivot-Tracking Pan:** Panning shifts the Orbit Pivot in world space by the exact camera translation vector so subsequent rotations always pivot around the center of your screen.
- **Near Clip Scaling:** As you zoom closer to a facet, Ogun reduces the camera's near clipping ratio so microscopic triangles are never clipped.
- **Constant Screen-Size Overlays:** Defect vertex spheres (Display > Defect highlight > **Point radius**, default `3 px`) and defect / pre-repair edge lines (**Edge width**, default `1 px`) keep the same on-screen size at any zoom depth.
- **3D Overlay Cap:** Selecting a row tints all of its defects. Edge tubes and vertex spheres are drawn for all of them only up to the overlay cap (default 5,000 faces, Preferences > Performance); above it they are drawn just for the selected / focused defects, which keeps 20,000-defect rows responsive.
- **Styles:** The **Display** panel holds the base-mesh colour, edges, transparency, back-face mode (*Custom* makes flipped normals easy to spot) and shading, the defect highlight colours, and the repair-preview styles. The **Structures (advanced)** panel still shows Polyscope's own per-structure controls; Ogun remembers each edge width when you toggle **Edges** off and on.

