# Mesheal for Blender

`mesheal_tools/` is a Blender extension (4.2+) that puts mesheal's checks and repairs in the
3D Viewport sidebar (N-panel > **Mesheal**) and in a **Mesheal** submenu of the Object and Edit Mesh menus.

**Status: UI prototype.** The panels and settings are real, but every operator is a stub that only
reports "not hooked up yet". **Check Mesh** fills the results with placeholder values so the
results layout can be designed.

## Develop

Symlink the package into Blender's user extensions repository, replacing `<ver>` with your Blender
version (e.g. `4.2`, `5.0`):

```sh
mkdir -p "$HOME/Library/Application Support/Blender/<ver>/extensions/user_default"
ln -s "$PWD/blender/mesheal_tools" \
  "$HOME/Library/Application Support/Blender/<ver>/extensions/user_default/mesheal_tools"
```

Then enable **Mesheal** in Preferences > Get Extensions / Add-ons (module
`bl_ext.user_default.mesheal_tools`). After editing the Python, run **Reload Scripts** (F3 > Reload Scripts).

## Build a zip

```sh
blender --command extension validate blender/mesheal_tools
blender --command extension build --source-dir blender/mesheal_tools --output-dir <out>
```

## Layout

| File | Contents |
|---|---|
| `props.py` | Scene settings (`scene.mesheal`); defaults mirror `src/core/Settings.hpp` |
| `operators.py` | Check, Select Result, Clear, Fix All, Remesh, Decimate, Merge (stubs) |
| `report.py` | In-memory check results (placeholder rows for now) |
| `ui.py` | Sidebar panels and the Mesheal menu |
