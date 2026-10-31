# Ogun 3D Viewport Navigation & Shortcuts Guide

Ogun uses **Blender-inspired, selection-centric 3D camera navigation** tailored for inspecting both whole STL meshes and microscopic sliver/degenerate facets ($10^{-4}\text{ mm}$ and smaller).

---

## 1. Quick Reference: Mouse & Keyboard Controls

### Mouse Controls (3D Viewport)

These are the bindings of the default **Ogun** mouse scheme. *Edit > Preferences > Mouse* switches to a Blender, OpenSCAD, Maya, Fusion 360 or SolidWorks scheme, or rebinds any action (see [Mouse Schemes](#mouse-schemes-edit--preferences--mouse) below). The in-app **3D Viewport Navigation & Shortcuts** window always lists the active bindings.

| Action | Input | Behavior |
| :--- | :--- | :--- |
| **Orbit / Rotate** | `Left-Click Drag` | Orbits the camera around the active **Orbit Pivot** (respecting the active Polyscope camera style: *Turntable* or *Free*). |
| **Pan Camera & Pivot** | `Shift + Left-Click Drag`, `Right-Click Drag`, or `Middle-Click Drag` | Translates both the camera and the **Orbit Pivot** in the view plane. Pan speed scales automatically with the distance to the Orbit Pivot so the surface stays locked under the cursor at any zoom depth. |
| **Zoom (Dolly)** | `Scroll Wheel` or `Ctrl + Shift + Left-Click Drag` | Dollies the camera toward/away from the active **Orbit Pivot**. Zoom step size scales proportionally with the distance to the pivot so you never overshoot microscopic facets. |
| **Adjust Near Clip Plane** | `Shift + Scroll Wheel` | Manually shifts the camera near clipping plane (in addition to Ogun's automatic zoom-proportional near clip adjustment). |
| **Select Face / Clear Pick** | `Left-Click` / `Right-Click` | Left-clicking a face without dragging: if it is a defect of the selected **Mesh Health** row, it becomes the **focus** (highlighted, the **Orbit Pivot** moves to its centroid, and for thin / sliver faces the **Inspector** opens with a live 3D repair preview). This works even when the row has more defects than the list cap and no list is shown. With no row selected, clicking a defective face opens the row that contains it (thin / sliver faces first). Clicking a face of another type, or a clean face, only shows a status-bar message (a clean face also moves the Orbit Pivot). Right-clicking clears the Polyscope pick. |
| **Toggle Face in the Face Selection** | `Shift + Left-Click` (no drag) | Adds the face under the cursor to the **face selection** (magenta), or removes it. Faces cut away by a slice plane are clicked through. *Decimate*, *Merge by Distance* and *Weld* work on the selected faces when there are any. |
| **Box / Circle Select Faces** | `B` then drag / `C` then paint | See the keyboard table. In these modes the left mouse button selects instead of orbiting. |
| **Select Shell** | `Alt + Left-Click` (`Alt + Ctrl/Cmd + Left-Click` toggles) | Multi-shell meshes: selects the whole shell under the cursor in the **Shells** panel (or adds / removes it). A plain click never changes the shell selection, so working on a set of shells is never undone by clicking a defect. |

### Mouse Schemes (Edit > Preferences > Mouse)

The **Mouse** tab of Preferences has a **Scheme** combo and a table with one row per action. Each input is a small button: click it to change it (press the mouse button, or turn the wheel, over the capture box while holding the modifier keys, or pick the button and the Shift / Ctrl / Alt boxes by hand) or remove it; **+** adds another input (up to three per action). Picking a scheme replaces every binding; editing any binding switches the scheme to **Custom**. *Reset to defaults* at the bottom of Preferences also restores the Ogun scheme.

| Action | Ogun (default) | Blender | OpenSCAD | Maya | Fusion 360 | SolidWorks |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Orbit | `Left drag` | `Middle drag` | `Left drag`, `Shift + Left drag` | `Alt + Left drag` | `Shift + Middle drag` | `Middle drag` |
| Pan | `Shift + Left`, `Right` or `Middle drag` | `Shift + Middle drag` | `Right drag` | `Alt + Middle drag` | `Middle drag` | `Ctrl + Middle drag` |
| Zoom (drag) | `Ctrl + Shift + Left drag` | `Ctrl + Middle drag` | `Middle drag`, `Shift + Right drag` | `Alt + Right drag` | (none) | `Shift + Middle drag` |
| Zoom (wheel) | `Wheel` | `Wheel` | `Wheel` | `Wheel` | `Wheel` | `Wheel` |
| Near clip plane | `Shift + Wheel` | `Shift + Wheel` | `Shift + Wheel` | `Shift + Wheel` | `Shift + Wheel` | `Shift + Wheel` |
| Select face / defect | `Left click` | `Left click` | `Left click` | `Left click` | `Left click` | `Left click` |
| Toggle face in selection | `Shift + Left click` | same | same | same | same | same |
| Select shell | `Alt + Left click` | same | same | same | same | same |
| Clear pick | `Right click` | same | same | same | same | same |

How inputs are matched:
- A **click** is a press and release without moving the mouse, so a drag and a click on the same button never conflict (Ogun: `Left drag` orbits, `Left click` selects; Maya: `Alt + Left drag` tumbles, `Alt + Left click` selects a shell). The press must be in the 3D view: a press on a panel (e.g. dragging a slider) released over the view is not a click.
- **Drags and the wheel:** modifiers that no binding of that button uses are ignored, and the others must match exactly. In the Ogun scheme `Alt + Left drag` still orbits, `Shift + Alt + Left drag` pans, and `Ctrl + Left drag` does nothing; in Blender `Left drag` does nothing, so left-click selection never fights the camera.
- **Clicks:** an exact match first; otherwise the binding with the most modifiers that are all held (*Toggle face* needs its exact modifiers). So `Ctrl + Shift + Left click` selects and `Shift + Alt + Left click` selects the shell. Adding `Ctrl` (when the Select shell binding does not already use it) or `Cmd` to a Select shell click adds or removes the shell.
- **Conflicts:** two actions of the same kind on the same input are drawn red in the table with a warning below it; the action higher in the table wins.
- **Box (`B`) and Circle (`C`) select** draw with the first button of *Select face*: Left in every built-in scheme. Right-click cancels them unless *Select face* is on the right button; middle-drag erases in Circle select unless *Select face* is on the middle button.
- **Ctrl** is the Control key, also on macOS (keyboard shortcuts still use Cmd).
- **Zoom options:** *Invert wheel zoom* (off: wheel up zooms in) and *Zoom speed* (0.1x to 5x, default 1x) multiply every wheel and drag zoom step.
- The scheme is saved with the other preferences (`mouse.preset`, `mouse.invert_wheel_zoom`, `mouse.zoom_speed` and one `mouse.<action>` line per action, e.g. `mouse.pan = shift+left, right, middle`). A built-in scheme always uses its own bindings; only *Custom* reads the binding lines. An unknown scheme name gives the Ogun scheme, and an invalid binding line keeps the Ogun binding of that action.

---

### Keyboard Shortcuts (3D Viewport)

`Ctrl` is `Cmd` on macOS. Every button tooltip shows its key; the **Navigate** panel (left column) has a button for each camera key, so experts can close it.

| Shortcut | UI Equivalent | Behavior |
| :--- | :--- | :--- |
| **`.`** or **`Numpad .`** | Navigate > `+Normal`, View > Camera > Frame selection, Shells right-click > Frame | **Frame Focused Facet (`+Normal` / Toggle):** Zooms in on the focused facet's centroid looking straight down its **outward positive normal (`+N`)**. Pressing **`.`** again while aligned with `+N` flips the camera to the **negative normal (`-N`)** side. A focused hole (Holes row) frames the **whole hole**, looking in along the hole's outward normal, with the same toggle. After a face selection (`Shift + Click`, `B`, `C`) it frames the **selected faces**: square on along their area-weighted normal when they mostly face one way (`.` again flips to the back), else from the current direction; clicking a defect or shell gives the focus back to it. Without a single focused face or hole it frames the selected defects, else the selected shells, else the visible defects or shells (or the whole mesh). |
| **`Shift + .`** | Navigate > `-Normal`, Inspector > `Back side` | **Frame Focused Facet (`-Normal`):** Directly frames the focused facet (or hole) from its **inward negative normal (`-N`)** side. |
| **`N`** / **`Shift + N`** | Navigate > `Next >` / `< Prev`, Edit > Select | **Next / Previous Defect:** Focuses the next (previous) defect of the selected Mesh Health row, worst first and within the active filter (sub-type or histogram bin), and flies the camera to it. Wraps around. |
| **`R`** | Inspector > `Commit Repair (R)` | **Commit Repair:** Applies the Inspector's chosen repair to the focused defect (thin face, intersecting island or hole) as one undoable step, as the button does. When the repair is not safe, or no defect is focused, nothing changes and the status bar says why. |
| **`Home`** or **`H`** | Navigate > `Home`, View > Camera > Home | **Frame All:** Flies the camera back to the full-mesh home view and resets the **Orbit Pivot** to the mesh center. The Home tooltip shows the current Orbit Pivot. |
| **`G`** | View > Ghost Mesh, Display > Transparency | **Ghost:** Toggles the base mesh between opaque and the last transparency. |
| **`B`** | Edit > Select Faces > Box Select | **Box Select Faces (Blender):** Drag a rectangle: the faces whose centre is inside and that you can see are added to the face selection; `Shift + drag` removes them instead. One box per `B`; `Esc` or right-click cancels. With *Select Through (X-ray)* on, hidden faces count too. Faces cut away by an enabled slice plane (View > Polyscope > Slice Planes) are never selected, and do not hide the faces behind them; a face the plane crosses counts as shown while its centre is on the kept side. |
| **`C`** | Edit > Select Faces > Circle Select | **Circle Select Faces (Blender):** A brush circle follows the cursor; drag to paint faces into the selection, `Shift + drag` or middle-drag to paint them out, the scroll wheel resizes the brush. `Esc`, `Enter` or right-click finishes. Like `B`, only the faces you see: never those a slice plane cuts away. |
| **`A`** / **`Alt + A`** | Edit > Select Faces > Select All / Deselect All | Selects every shown face (not hidden shells, not faces a slice plane cuts away) / clears the face selection. |
| **`Esc`** | Edit > Select Defects > None | **Staged clear:** The first press leaves box / circle select, or clears the defect selection and focus; the next press closes the Tool panel and deselects the row (or cancels an open Mesh operation); then it clears the face selection, and last the shell selection. |
| **`I`** | Shells right-click > Isolate, Mesh > Shells > Isolate | **Isolate Shells:** Shows only the selected shells, so Mesh Health, the Tool list, Next / Prev and *Fix All* cover only them. Press again to bring back the view from before the first Isolate. With a different shell selected, `I` switches the isolation to it. View only: no undo step. |
| **`X`** | Mesh > Delete Selected Faces | **Delete Selected Faces (confirmed):** With faces selected, opens a small "Delete N selected faces?" popup at the cursor (Blender style). `Enter` or the *Delete* button confirms and removes them as one undoable step; `Esc`, *Cancel* or a click outside cancels. With nothing selected it only shows "No faces selected." in the status bar. |
| **`Delete`** / **`Backspace`** | Mesh > Delete Selected Faces; Shells right-click > Delete, Mesh > Shells > Delete | **Delete Selected Faces / Shells:** Removes the selected faces (Shift+click, B, C) immediately, with no confirmation (use `X` to be asked first), as one undoable step (`Ctrl + Z`); with no faces selected, removes the selected shells. Refuses to delete everything. |
| **`F5`** | Mesh Health > `Check Mesh`, Repair > Check Mesh | **Check Mesh:** Runs every Mesh Health row in the background, including rows whose **Auto** is off. Progress and **Cancel** are in the status bar. |
| **`Shift + F5`** | Mesh Health > `Fix All...`, Repair > Fix All... | **Fix All...:** Opens *Operation: Fix All* with the saved number of passes (3 unless changed in Preferences > Fix All). It follows the selection (selected faces, else selected shells, else the whole mesh) and the panel says which. Choose the steps, then **Preview Fix All**: nothing changes until **Apply**. |
| **`Alt + Shift + F5`** | Mesh Health > `One Pass...`, Repair > Fix All, One Pass... | **Fix All, One Pass:** The same panel with **Passes** = 1, to step the repair by hand and look at each pass. On macOS Alt is the Option key. |
| **`Ctrl + Z`** | Edit > Undo *label*, Inspector > `Undo` | **Undo Mesh Modification:** Reverts the last mesh modification (single-face repair, *Fix All*, *Fix Selected*, *Apply Scale*, *Set Origin to Selection*, *Translate / Rotate*, *Decimate*, *Merge by Distance*, *Weld*, or *Delete Faces / Shells*) one step at a time, restoring the Mesh Health results, the selected row, the focused face, chosen repair strategy and live preview. Undoing a whole-mesh move (*Apply Scale*, *Set Origin to Selection*, *Translate / Rotate*) also re-frames the camera to Home, as the edit did. |
| **`Ctrl + O`** / **`Ctrl + S`** / **`Ctrl + Shift + S`** | File > Open / Save / Save As | Open an STL; save over the current file (asks the first time); save as a new Binary or ASCII STL. |
| **`Ctrl + ,`** | Edit > Preferences | Opens the Preferences window. |
| **`?`** (`Shift + /`) or **`F1`** | Help > Navigation & Shortcuts | Opens or closes the in-app **3D Viewport Navigation & Shortcuts** window. |
| **`Ctrl + C`** / **`Ctrl + V`** | | Copy / paste the camera pose (Polyscope built-in). |

### Mouse in the Tool panel's defect list

| Input | Behavior |
| :--- | :--- |
| `Click` | Selects only that defect, focuses it and flies the camera to it (as **`.`**, so a repair needs no keyboard). |
| `Ctrl/Cmd + Click` | Adds or removes a defect from the selection (the camera stays). |
| `Shift + Click` | Selects the range from the last clicked row (the camera stays). |
| `Double-Click` | Flies to the defect, also after a Ctrl/Cmd or Shift click. |
| Click a column header | Sorts by that column (third click restores worst-first order). |
| Click a histogram bar (`Shift` for a range) or the `<` `>` arrows | Filters to that bin and selects its defects; its list opens once it holds no more than the list cap. |

### Mouse in the Shells panel (multi-shell meshes)

**View > Compass** draws X / Y / Z axes (red / green / blue) in the bottom-left corner of the 3D view; they turn with the camera. It is off by default and remembered. **View > Polyscope** opens Polyscope's Slice Planes, Appearance (transparency, ground plane, background), Camera & Clipping and Render & Debug sections as floating windows. Each opens in its own place side by side along the top of the free 3D view (in rows when the view is narrow), so several open at once do not overlap. Ogun starts empty when no STL is given on the command line.

The **Shells** panel (left column, between Navigate and Structures) appears only when the mesh has more than one shell. Mesh Health covers the **selected** shells, or the **visible** shells when none is selected; a *Scope* line in Mesh Health says so, with an *All shells* button. A crossing counts only when both of its shells are in scope.

| Input | Behavior |
| :--- | :--- |
| `Click` / `Ctrl/Cmd + Click` / `Shift + Click` | Selects one shell / adds or removes one / selects a range, as in the defect list. |
| `Double-Click` | Selects the shell and frames it. |
| `Right-Click` | Frame, Isolate / Un-isolate, Hide, Hide All, Show All, Delete, Save As (one STL, or one STL per shell). *Merge...* stays greyed out until Merge Shells exists (Milestone 4). |
| Vis checkbox (header: all) | Shows or hides a shell (or all). Hidden shells are cut from the 3D view and never edited, but are still saved. |
| `Select...` | All, None, Invert, Visible shells, Unhealthy, Nested inside another shell, Smaller than N faces. |

---

## 2. Core Navigation Concepts (Blender Conventions)

### Auto-Orbit Around Selection Centroid
When you focus a defect (click its row in the Tool panel's list, click it in 3D, or press `N`; a row click and `N` also fly the camera to it):
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
4. **Commit Repair** (or **`R`**) applies the repair to the full mesh; the Auto rows of Mesh Health re-run. To repair several faces at once, select them (Ctrl/Shift+click, or a histogram bin) and press **Fix Selected**, which applies each face's recommended strategy as one undo step. Press **`Ctrl + Z`** / **`Cmd + Z`** (or **Undo**) to restore the face and try a different strategy.

### Depth-Scaled Zoom, Pan & Dynamic Highlight Sizing
- **Proportional Zoom & Pan:** Dolly and pan speeds scale with $\|\mathbf{p}_{\text{cam}} - \mathbf{p}_{\text{pivot}}\|$ (the distance from the camera to the Orbit Pivot). Whether you are viewing a $100\text{ mm}$ housing or a $10^{-4}\text{ mm}$ sliver triangle, scroll-zooming and panning move at a consistent screen-space rate.
- **Pivot-Tracking Pan:** Panning shifts the Orbit Pivot in world space by the exact camera translation vector so subsequent rotations always pivot around the center of your screen.
- **Near Clip Scaling:** As you zoom closer to a facet, Ogun reduces the camera's near clipping ratio so microscopic triangles are never clipped.
- **Constant Screen-Size Overlays:** Defect vertex spheres (Display > Defect highlight > **Point radius**, default `3 px`) and defect / pre-repair edge lines (**Edge width**, default `1 px`) keep the same on-screen size at any zoom depth. Each sphere and tube end is sized for its own distance from the camera, so defects near the camera stay small even when the focused defect is far away.
- **3D Overlay Cap:** Selecting a row tints all of its defects. Edge tubes and vertex spheres are drawn for all of them only up to the overlay cap (default 5,000 faces, Preferences > Performance); above it they are drawn just for the selected / focused defects, which keeps 20,000-defect rows responsive.
- **Styles:** The **Display** panel holds the base-mesh color, edges, transparency, back-face mode (*Custom* makes flipped normals easy to spot) and shading, the defect highlight colors, and the repair-preview styles. The **Structures (advanced)** panel still shows Polyscope's own per-structure controls; Ogun remembers each edge width when you toggle **Edges** off and on.

