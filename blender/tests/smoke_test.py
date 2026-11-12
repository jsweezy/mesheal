# SPDX-License-Identifier: GPL-3.0-or-later
"""Headless Blender smoke test of the Mesheal extension.

Run: blender --background --factory-startup --python blender/tests/smoke_test.py
Needs the native module: run scripts/build-blender-module.sh first.

To test an installed extension instead of the source tree, set MESHEAL_EXTENSION_DIR to the folder
that contains mesheal_tools (packaging/blender/test-extension.py does this).
"""

import os
import sys
import traceback

import bmesh
import bpy

sys.path.insert(0, os.environ.get("MESHEAL_EXTENSION_DIR")
                or os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import mesheal_tools  # noqa: E402
from mesheal_tools import bridge, report  # noqa: E402


def loaded_tbb_libraries():
    """Paths of the TBB libraries (libtbb, tbb12.dll, ... not tbbmalloc) loaded in this process."""
    paths = []
    if sys.platform.startswith("linux"):
        with open("/proc/self/maps") as maps:
            paths = [line.split()[-1] for line in maps if "/" in line]
    elif sys.platform == "darwin":
        import ctypes

        dyld = ctypes.CDLL(None)
        dyld._dyld_get_image_name.restype = ctypes.c_char_p
        paths = [dyld._dyld_get_image_name(i).decode() for i in range(dyld._dyld_image_count())]
    elif sys.platform == "win32":
        import ctypes
        from ctypes import wintypes

        psapi, kernel32 = ctypes.WinDLL("psapi"), ctypes.WinDLL("kernel32")
        # Declare the types: handles are 64-bit and would not fit ctypes' default (int)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.GetCurrentProcess.argtypes = []
        psapi.EnumProcessModules.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.HMODULE),
                                             wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
        psapi.EnumProcessModules.restype = wintypes.BOOL
        psapi.GetModuleFileNameExW.argtypes = [wintypes.HANDLE, wintypes.HMODULE, wintypes.LPWSTR, wintypes.DWORD]
        psapi.GetModuleFileNameExW.restype = wintypes.DWORD
        process = kernel32.GetCurrentProcess()
        modules, needed = (wintypes.HMODULE * 2048)(), wintypes.DWORD()
        if not psapi.EnumProcessModules(process, modules, ctypes.sizeof(modules), ctypes.byref(needed)):
            raise OSError("EnumProcessModules failed")
        name = ctypes.create_unicode_buffer(1024)
        for module in modules[: needed.value // ctypes.sizeof(wintypes.HMODULE)]:
            if psapi.GetModuleFileNameExW(process, module, name, len(name)):
                paths.append(name.value)
    names = [p for p in paths if os.path.basename(p).lower().startswith(("libtbb.", "tbb12"))]
    return sorted(set(os.path.realpath(p) for p in names))


def check_tbb():
    """The module must use Blender's own TBB: one copy, inside Blender's installation."""
    import bpy

    native = bridge.native()
    loaded = loaded_tbb_libraries()
    print(f"module tbb={native.tbb}; TBB loaded: {loaded}")
    # Blender.app/Contents (macOS) or the folder with the blender executable
    blender_dir = os.path.dirname(os.path.realpath(bpy.app.binary_path))
    if sys.platform == "darwin":
        blender_dir = os.path.dirname(blender_dir)
    assert len(loaded) <= 1, f"more than one TBB loaded: {loaded}"
    assert all(os.path.normcase(p).startswith(os.path.normcase(blender_dir)) for p in loaded), \
        f"TBB loaded from outside Blender ({blender_dir}): {loaded}"
    # Release zips: the Blender 5.1+ module (Python 3.13) runs CGAL in parallel on Blender's TBB
    if os.environ.get("MESHEAL_RELEASE_ZIP") and sys.version_info >= (3, 13):
        assert native.tbb, "the Python 3.13 module was built without TBB"


def counts():
    return {i.name: i.count for i in report.get()}


def make_open_cube():
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.quads_convert_to_tris()
    bpy.ops.mesh.select_all(action="DESELECT")
    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bmesh.ops.delete(bm, geom=[bm.faces[0]], context="FACES_ONLY")
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")
    return obj


def main():
    assert bridge.available(), bridge.unavailable_reason()
    print(f"Python {sys.version.split()[0]}, module {bridge.native().__file__}")
    check_tbb()
    mesheal_tools.register()
    obj = make_open_cube()
    assert len(obj.data.polygons) == 11, len(obj.data.polygons)

    assert bpy.ops.mesh.mesheal_check() == {"FINISHED"}
    assert counts()["Holes"] == 1, counts()

    assert bpy.ops.mesh.mesheal_fix_all() == {"FINISHED"}
    # Fix All checks its result, so the results are back without pressing Check Mesh
    assert counts().get("Holes") == 0, counts()
    assert not report.stale(obj, len(obj.data.polygons)), "results after Fix All are stale"
    assert bpy.ops.mesh.mesheal_check() == {"FINISHED"}
    assert counts()["Holes"] == 0, counts()
    # The cube was flat shaded; writing the result back must not turn on smooth shading
    assert not any(p.use_smooth for p in obj.data.polygons), "Fix All turned on smooth shading"

    # A smooth-shaded mesh stays smooth
    smooth_cube = make_open_cube()
    smooth_cube.data.polygons.foreach_set("use_smooth", [True] * len(smooth_cube.data.polygons))
    assert bpy.ops.mesh.mesheal_fix_all() == {"FINISHED"}
    assert all(p.use_smooth for p in smooth_cube.data.polygons), "Fix All turned off smooth shading"

    # Select Result in Edit Mode (on a fresh open cube)
    obj2 = make_open_cube()
    bpy.ops.mesh.mesheal_check()
    bpy.ops.object.mode_set(mode="EDIT")
    assert bpy.ops.mesh.mesheal_select_report(index=1) == {"FINISHED"}
    assert any(f.select for f in bmesh.from_edit_mesh(obj2.data).faces)
    bpy.ops.object.mode_set(mode="OBJECT")

    # Selected-faces scope
    obj2.data.polygons.foreach_set("select", [True] * len(obj2.data.polygons))
    bpy.context.scene.mesheal.scope = "SELECTED"
    assert bpy.ops.mesh.mesheal_fix_all() == {"FINISHED"}
    bpy.context.scene.mesheal.scope = "WHOLE"

    # Decimate a subdivided sphere
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    sphere = bpy.context.active_object
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.quads_convert_to_tris()
    bpy.ops.object.mode_set(mode="OBJECT")
    before = len(sphere.data.polygons)
    assert bpy.ops.mesh.mesheal_decimate() == {"FINISHED"}
    after = len(sphere.data.polygons)
    assert after < before, (before, after)
    assert counts(), "Decimate did not check its result"
    assert not report.stale(sphere, after), "results after Decimate are stale"
    print(f"decimate: {before} -> {after}")

    # Merge: a no-op-ish run must still finish
    assert bpy.ops.mesh.mesheal_merge() == {"FINISHED"}

    # Precision Isosurface: CGAL's mesher, which runs in parallel when the module has TBB
    make_open_cube()
    bpy.context.scene.mesheal.remesh_method = "ISO"
    bpy.context.scene.mesheal.remesh_target_size = 0.1  # the cube is 2 wide: a few thousand faces
    assert bpy.ops.mesh.mesheal_remesh() == {"FINISHED"}
    remeshed = bpy.context.active_object
    assert len(remeshed.data.polygons) > 12, len(remeshed.data.polygons)
    print(f"isosurface: 11 -> {len(remeshed.data.polygons)} faces")
    check_tbb()

    mesheal_tools.unregister()
    print("MESHEAL SMOKE TEST PASSED")


try:
    main()
except Exception:
    traceback.print_exc()
    sys.exit(1)
