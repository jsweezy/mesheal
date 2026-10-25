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
| **Select Face / Clear Pick** | `Left-Click` / `Right-Click` | Left-clicking a face of the STL mesh without dragging selects that face exactly like clicking it in the Diagnostics list: it is highlighted, the **Orbit Pivot** moves to its centroid, and the **Single-Face Repair Inspector** opens with a live 3D repair preview. Clicking a vertex or edge only shows Polyscope's pick info. Right-clicking clears the Polyscope pick. |

---

### Keyboard Shortcuts (3D Viewport)

| Shortcut | UI Button Equivalent | Behavior |
| :--- | :--- | :--- |
| **`.`** or **`Numpad .`** | `Zoom +Normal (.)` | **Frame Selected Facet (`+Normal` / Toggle):** Zooms in on the selected facet's centroid looking straight down its **outward positive normal (`+N`)**. Pressing **`.`** a second time (or double-tapping **`..`**) while aligned with `+N` flips the camera to the **negative normal (`-N`)** side. |
| **`Shift + .`** | `Zoom -Normal (Shift+.)` | **Frame Selected Facet (`-Normal`):** Directly frames the selected facet from its **inward negative normal (`-N`)** side. |
| **`Home`** or **`H`** | `Reset View (Home)` | **Frame All:** Flies the camera back to the full-mesh home view and resets the **Orbit Pivot** to the full mesh bounding-box center. |
| **`Ctrl + Z`** or **`Cmd + Z`** | `Undo (Ctrl+Z / Cmd+Z)` | **Undo Mesh Modification:** Reverts the last mesh modification (single-face repair commit, *Fix All Thin / Sliver Faces*, or *Scale Mesh Coordinates*) one step at a time, restoring the previous diagnostics report, face selection, chosen repair strategy, and live preview. |
| **`?`** (`Shift + /`) or **`F1`** | `? Navigation Help` | Opens or closes the in-app **3D Viewport Navigation & Shortcuts** floating reference window. |

---

## 2. Core Navigation Concepts (Blender Conventions)

### Auto-Orbit Around Selection Centroid
When you click a defective facet in the **Diagnostics** list:
- Ogun highlights the triangle's surface, edges, and vertices in 3D.
- The 3D **Orbit Pivot** (center of rotation) immediately moves to the triangle's true geometric vertex centroid:
  $$\mathbf{c} = \frac{\mathbf{v}_0 + \mathbf{v}_1 + \mathbf{v}_2}{3}$$
- Even before pressing `.` to zoom in, left-click dragging in the viewport orbits smoothly around the selected facet rather than swinging the facet out of view around the global mesh center.
- When the entire defect category (*Thin / Near-Zero-Area Faces*) is selected, the Orbit Pivot is placed at the center of the combined selection bounding box. When selection is cleared, it returns to the full mesh center.

### Inspecting Front (`+Normal`) vs. Back (`-Normal`) of a Facet
In STL repair, inspecting a defective facet from both its outward (`+N`) and inward (`-N`) sides helps determine whether neighboring faces are folded, inverted, or self-intersecting:
1. **First `.` press:** Flies the camera to $\mathbf{c} + r\,\hat{\mathbf{n}}$, looking along $-\hat{\mathbf{n}}$ directly at the front (`+Normal`) of the triangle.
2. **Second `.` press (or `..`):** Because the camera is already aligned with `+Normal` on that facet, pressing `.` again flips the camera to $\mathbf{c} - r\,\hat{\mathbf{n}}$, looking along $+\hat{\mathbf{n}}$ at the back (`-Normal`) of the triangle.
3. **Rotating away resets the toggle:** If you orbit the camera manually with the mouse to inspect an oblique angle, pressing `.` again returns you cleanly to the canonical `+Normal` view first.
4. **Direct `-Normal` access:** Pressing `Shift + .` always jumps straight to the `-Normal` side regardless of current camera angle.
5. **Collinear zero-area triangles:** If a defective triangle has three strictly collinear vertices ($\|(\mathbf{v}_1 - \mathbf{v}_0) \times (\mathbf{v}_2 - \mathbf{v}_0)\| = 0$), its geometric normal is undefined. Ogun automatically computes a viewing axis **perpendicular to the triangle's longest edge** so `.` and `Shift + .` still frame the degenerate segment broadside from opposite directions.

### Selecting & Repairing a Face in the Viewport
1. **Left-click** a face in the 3D viewport (or click its row in the Diagnostics list). Zero-area faces have no visible surface to click, so select those from the list.
2. The **Single-Face Repair Inspector** shows the face's detected type, maximum angle, and min/max edge ratio, and pre-selects the recommended strategy.
3. With **Show 3D Repair Preview Patch** enabled, the repaired 2-ring neighborhood is drawn in green with black edges (colors adjustable). Unsafe strategies show a red warning instead. **Preview Display** chooses how the patch is combined with the original surface:
   - **Cutout + Original Edges** (default): the replaced base faces are hidden, the opaque patch fills the gap without Z-fighting flicker, and the pre-repair edges are drawn as thin lines over it. Selecting it switches Polyscope to *Pretty* transparency. The pre-repair edges are thinner (*Original Edge Width (px)*, default `0.5`) and semi-transparent (*Original Edge Opacity*, default `0.5`) than the patch edges. An edge that survives the repair shows the patch edge color through and around it; an edge the repair removes shows only the pre-repair color.
   - **Translucent Base**: the whole base mesh is dimmed (*Base Mesh Opacity*, default `0.40`) so the old surface shows through where the repair changes the shape. Selecting it switches Polyscope to *Simple* transparency, which avoids flicker.
   - **Translucent Patch**: the base mesh stays opaque and only the patch is see-through (*Patch Opacity*).
4. **Commit Repair** applies the repair to the full mesh and re-runs diagnostics. Press **`Ctrl + Z`** / **`Cmd + Z`** (or **Undo**) to restore the face and try a different strategy.

### Depth-Scaled Zoom, Pan & Dynamic Highlight Sizing
- **Proportional Zoom & Pan:** Dolly and pan speeds scale with $\|\mathbf{p}_{\text{cam}} - \mathbf{p}_{\text{pivot}}\|$ (the distance from the camera to the Orbit Pivot). Whether you are viewing a $100\text{ mm}$ housing or a $10^{-4}\text{ mm}$ sliver triangle, scroll-zooming and panning move at a consistent screen-space rate.
- **Pivot-Tracking Pan:** Panning shifts the Orbit Pivot in world space by the exact camera translation vector so subsequent rotations always pivot around the center of your screen.
- **Near Clip Scaling:** As you zoom closer to a facet, Ogun reduces the camera's near clipping ratio so microscopic triangles are never clipped.
- **Constant Screen-Size Overlays:** Defect vertex spheres (**Defect Point Radius (px)**, default `3 px`) and defect / pre-repair edge lines (**Defect Edge Width (px)**, default `1 px`) keep the same on-screen size at any zoom depth.
- **Overlay Styles in Polyscope's Structures Panel:** *Selected Defect Faces* and *Repair Preview Patch* are surface meshes with their own **Color**, **Edges**, **Edge Color**, and **Width** (default `1 px`). Ogun remembers each width, including the STL Surface Mesh's, when you toggle **Edges** off and on.

