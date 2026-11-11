# Units & Preferences

## File units never move the geometry

STL files store bare numbers with no units. Mesheal lets you say what those numbers mean, without
changing them.

- **File units** (`mm`, `cm`, `m`, `in`, `ft`; default `mm`) only label the numbers, so sizes, areas
  and volumes are shown with the right unit. No vertex moves and no undo step is recorded. Set them
  in **Mesh > Units & Scale...** or by clicking the unit in the status bar.
- **Scale** is the separate, explicit way to change the geometry. In **Mesh > Units & Scale...**,
  pick a target unit under *Scale to*. Mesheal shows the coordinate multiplier (for example 0.1 for
  mm to cm). **Apply Scale** multiplies every vertex coordinate. That is one undo step, and the
  automatic checks re-run.

Tolerances such as the weld floor and the thin-face limits are also plain numbers in file
coordinates. Changing the file units relabels them; it does not convert them.

## Preferences

**Edit > Preferences...** (`Cmd/Ctrl+,`). Changes are saved at once to
`~/.config/mesheal/preferences.conf` (or `$XDG_CONFIG_HOME/mesheal/`) and apply to every later
session. **Reset to defaults** at the bottom restores everything, including the Display panel styles.

| Tab | Settings |
|---|---|
| **Units & Tolerance** | *Weld floor* and *Relative tolerance* used when loading STL files (both $10^{-9}$ by default). Defaults for the Thin / sliver faces check: aspect limit, minimum height and minimum area. Editing a default also sets the current check; changing the check later does not change the default. |
| **Fix All** | Starting values of the Fix All panel: number of passes, time budget, its own weld (separate from the load weld), and which optional steps are on (remove fins and internal walls, remove tiny floating shells and their size, close cracks, heal thin faces). |
| **Diagnostics** | *Run Check Mesh after loading a file*, and the default **Auto** box of each Mesh Health row for new meshes. |
| **Interface** | UI scale and color theme. |
| **Mouse** | Mouse scheme (Mesheal, Blender, OpenSCAD, Maya, Fusion 360, SolidWorks or Custom) and per-action bindings. See [Navigation & Shortcuts](../NAVIGATION.md#mouse-schemes-edit-preferences-mouse). |
| **Performance** | *List cap* (default 100): above this many defects a tool shows a summary instead of a list. *3D overlay cap*: above this many highlighted faces only the face tint is drawn, which keeps large meshes fast. |

The Display panel's styles (colors, edge widths, opacities, back-face style, preview mode) are saved
with the preferences too.

Recent files are stored next to the preferences, in `recent_files.txt`. On first start after the
rename from Ogun, settings in `~/.config/ogun` are moved over automatically.
