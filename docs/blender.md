# Blender extension

`blender/mesheal_tools/` is a Blender extension (Blender 4.2 and newer) that puts Mesheal's checks
and repairs in the 3D Viewport sidebar (**N-panel > Mesheal**) and in a **Mesheal** submenu of the
Object and Edit Mesh menus. It uses the [Python module](python.md).

- **Check Mesh** lists holes, non-manifold geometry, flipped normals, self-intersections and more.
  In Edit Mode, the select button on a row selects those faces.
- **Fix All**, **Remesh** (Voxel, Isosurface, Shrink-Wrap), **Decimate** and **Merge by Distance**
  call Mesheal. Fix All, Decimate and Merge can work on the selected faces only.
- Repairs run on a background thread: progress shows in the status bar, **Esc** cancels, and Blender
  stays responsive. Each repair is one undo step, and the mesh is checked again afterwards.

## Install

Download the zip for your system. It works in Blender 4.2 and newer:

| System | Download |
|--------|----------|
| Windows (64-bit) | [Mesheal-blender-windows-x64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-blender-windows-x64.zip) |
| macOS (Apple Silicon) | [Mesheal-blender-macos-arm64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-blender-macos-arm64.zip) |
| Linux (x86-64) | [Mesheal-blender-linux-x64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-blender-linux-x64.zip) |

1. In Blender, open **Edit > Preferences > Get Extensions**.
2. Open the menu at the top right (**v**) and choose **Install from Disk...**, then pick the zip
   (don't unzip it). You can also drag the zip into Blender's window.
3. Mesheal is enabled straight away: press **N** in the 3D Viewport and open the **Mesheal** tab.

To update, install the newer zip the same way. To remove it, find Mesheal in **Preferences >
Add-ons** and choose **Uninstall** from its menu.

If the Check panel says "Mesheal has no native module for this Blender", your Blender is newer than
this Mesheal download: get the latest one.

## Install from source

For development on macOS: the add-on runs straight from the source tree.

1. Build the native module against Blender's Python:

    ```bash
    scripts/build-blender-module.sh
    ```

2. Link the package into Blender's user extensions folder. Replace `<ver>` with your Blender version
   (for example `4.2` or `5.0`):

    ```bash
    mkdir -p "$HOME/Library/Application Support/Blender/<ver>/extensions/user_default"
    ln -s "$PWD/blender/mesheal_tools" \
      "$HOME/Library/Application Support/Blender/<ver>/extensions/user_default/mesheal_tools"
    ```

3. Enable **Mesheal** in Blender's **Preferences > Add-ons**.

Without the native module the panels still show, and the Check panel says
"mesheal native module not built: run scripts/build-blender-module.sh".

## Limitations

- Works in the object's local coordinates and ignores modifiers.
- A repair replaces the mesh with a plain triangle mesh: UVs, materials and custom attributes are
  lost.
- Check results are kept in memory (not saved in the `.blend`) and go stale when the face count
  changes.

Developer notes (tests, building a zip, file layout) are in
[`blender/README.md`](https://github.com/jsweezy/mesheal/blob/main/blender/README.md).
