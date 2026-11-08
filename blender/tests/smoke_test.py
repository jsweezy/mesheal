# SPDX-License-Identifier: GPL-3.0-or-later
"""Headless Blender smoke test of the Mesheal extension.

Run: blender --background --factory-startup --python blender/tests/smoke_test.py
Needs the native module: run scripts/build-blender-module.sh first.
"""

import os
import sys
import traceback

import bmesh
import bpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import mesheal_tools  # noqa: E402
from mesheal_tools import bridge, report  # noqa: E402


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
    assert bridge.available(), bridge.BUILD_HINT
    mesheal_tools.register()
    obj = make_open_cube()
    assert len(obj.data.polygons) == 11, len(obj.data.polygons)

    assert bpy.ops.mesh.mesheal_check() == {"FINISHED"}
    assert counts()["Holes"] == 1, counts()

    assert bpy.ops.mesh.mesheal_fix_all() == {"FINISHED"}
    assert bpy.ops.mesh.mesheal_check() == {"FINISHED"}
    assert counts()["Holes"] == 0, counts()

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
    print(f"decimate: {before} -> {after}")

    # Merge: a no-op-ish run must still finish
    assert bpy.ops.mesh.mesheal_merge() == {"FINISHED"}

    mesheal_tools.unregister()
    print("MESHEAL SMOKE TEST PASSED")


try:
    main()
except Exception:
    traceback.print_exc()
    sys.exit(1)
