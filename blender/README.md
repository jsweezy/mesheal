# Mesheal for Blender

`mesheal_tools/` is a Blender extension (4.2+) that puts mesheal's checks and repairs in the
3D Viewport sidebar (N-panel > **Mesheal**) and in a **Mesheal** submenu of the Object and Edit Mesh menus.

- **Check Mesh** lists holes, non-manifold geometry, flipped normals, self-intersections and more.
  In Edit Mode, the select button on a row selects those faces.
- **Fix All**, **Remesh** (Voxel, Isosurface, Shrink-Wrap), **Decimate** and **Merge by Distance** call
  mesheal. Fix All, Decimate and Merge can work on the selected faces only.
- Repairs run on a background thread: progress shows in the status bar, **Esc** cancels, and Blender
  stays responsive. Each repair is one undo step.

## Build the native module

The operators need the compiled `_mesheal` module, built against Blender's bundled Python:

```sh
scripts/build-blender-module.sh
```

This configures `build_python/` (Release, no GUI, no TBB), builds the `_mesheal` target and copies
`_mesheal.cpython-*.so` into `blender/mesheal_tools/`. Set `BLENDER_PYTHON` to use another Python,
and `PYTHON_INCLUDE_DIR` if Blender's Python has no `Python.h` (the script then borrows the headers
of a same-version CPython, e.g. one installed by `uv`).

Without the module the panels still show, and the Check panel says:
"mesheal native module not built: run scripts/build-blender-module.sh".

## Limitations

- The development build (above) links Homebrew's GMP and MPFR. The release zips don't: they link
  them in statically (see "Release zips" below).
- Works in the object's local coordinates and ignores modifiers.
- A repair replaces the mesh with a plain triangle mesh: UVs, materials and custom attributes are lost.
- Check results are kept in memory (not saved in the .blend) and go stale when the face count changes.

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

## Tests

```sh
# Python unit tests of the module + headless Blender smoke test (needs scripts/build-blender-module.sh first,
# because the smoke test imports the copied .so)
ctest --test-dir build_python -R "python|blender" --output-on-failure --timeout 300

# or by hand
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python blender/tests/smoke_test.py
```

## Release zips

`.github/workflows/release.yml` builds one zip per platform (`Mesheal-blender-<platform>.zip`). Each
holds two modules, `_mesheal.*311*` for Blender 4.2-5.0 (Python 3.11) and `_mesheal.*313*` for
Blender 5.1 and later (Python 3.13); Python loads the one matching its version. The libraries come
from vcpkg (`packaging/vcpkg/`) and are linked in statically, so the module depends only on the
operating system; GMP and MPFR licenses go in `licenses/`. TBB is off for now.

- `packaging/blender/build-extension.py` assembles a zip from built modules (and refuses modules that
  still need other libraries).
- `packaging/blender/test-extension.py <blender> <zip>` installs a zip into a throwaway Blender
  profile, as "Install from Disk" does, and runs `tests/smoke_test.py` against it. CI runs it in
  Blender 4.2, 5.0, 5.1 and the newest release.

## Layout

| File | Contents |
|---|---|
| `props.py` | Scene settings (`scene.mesheal`); defaults mirror `src/core/Settings.hpp` |
| `bridge.py` | Blender mesh <-> numpy, settings -> native parameter structs |
| `operators.py` | Check, Select Result, Clear, Fix All, Remesh, Decimate, Merge (sync + modal) |
| `report.py` | In-memory check results |
| `ui.py` | Sidebar panels and the Mesheal menu |
| `tests/smoke_test.py` | Headless Blender smoke test |
