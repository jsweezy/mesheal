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

!!! warning "Development build only"
    There is no packaged release yet. The extension runs on macOS from a source build and needs
    Homebrew GMP and MPFR at runtime.

## Install from source

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
