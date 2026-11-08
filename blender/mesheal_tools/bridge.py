# SPDX-License-Identifier: GPL-3.0-or-later
"""Glue between Blender (bpy) and the native _mesheal module: mesh <-> numpy and settings mapping."""

import numpy as np

BUILD_HINT = "mesheal native module not built: run scripts/build-blender-module.sh"

_native = None
_tried = False


def native():
    """The compiled _mesheal module, or None if it is missing or fails to import."""
    global _native, _tried
    if not _tried:
        _tried = True
        try:
            from . import _mesheal

            _native = _mesheal
        except ImportError as exc:
            print(f"Mesheal: could not import the native module ({exc})")
            _native = None
    return _native


def available():
    return native() is not None


def polygon_count(obj):
    """Face count of the mesh as it is now (Edit Mode edits included)."""
    if obj.mode == "EDIT":
        obj.update_from_editmode()
    return len(obj.data.polygons)


def read_mesh(obj):
    """Local-space triangulated mesh of `obj` (modifiers ignored).

    Returns (V float64 (N,3), F int32 (M,3), tri_to_poly int32 (M,), polygon count).
    """
    if obj.mode == "EDIT":
        obj.update_from_editmode()
    mesh = obj.data
    mesh.calc_loop_triangles()

    nv = len(mesh.vertices)
    nt = len(mesh.loop_triangles)
    co = np.empty(nv * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", co)
    verts = np.ascontiguousarray(co.reshape(-1, 3), dtype=np.float64)

    tris = np.empty(nt * 3, dtype=np.int32)
    mesh.loop_triangles.foreach_get("vertices", tris)
    faces = np.ascontiguousarray(tris.reshape(-1, 3))

    tri_to_poly = np.empty(nt, dtype=np.int32)
    mesh.loop_triangles.foreach_get("polygon_index", tri_to_poly)
    return verts, faces, tri_to_poly, len(mesh.polygons)


def selection_mask(mesh, tri_to_poly):
    """uint8 mask over the triangles: 1 where the owning polygon is selected."""
    sel = np.empty(len(mesh.polygons), dtype=np.bool_)
    mesh.polygons.foreach_get("select", sel)
    return sel[tri_to_poly].astype(np.uint8)


def write_mesh(obj, verts, faces):
    """Replace the geometry of `obj` with a triangle mesh.

    Known limitation: UVs, materials and custom attributes are lost (the result is a new triangle soup).
    """
    import bpy

    was_edit = obj.mode == "EDIT"
    if was_edit:
        bpy.ops.object.mode_set(mode="OBJECT")

    mesh = obj.data
    nv = len(verts)
    nf = len(faces)
    mesh.clear_geometry()
    mesh.vertices.add(nv)
    mesh.vertices.foreach_set("co", np.ascontiguousarray(verts, dtype=np.float32).ravel())
    mesh.loops.add(nf * 3)
    mesh.loops.foreach_set("vertex_index", np.ascontiguousarray(faces, dtype=np.int32).ravel())
    mesh.polygons.add(nf)
    mesh.polygons.foreach_set("loop_start", np.arange(0, nf * 3, 3, dtype=np.int32))
    mesh.update(calc_edges=True)

    if was_edit:
        bpy.ops.object.mode_set(mode="EDIT")


def check_params(props):
    m = native()
    p = m.CheckParams()
    p.intersection_tolerance = props.intersection_tolerance
    p.thin_max_aspect = props.thin_max_aspect
    return p


def repair_params(props):
    m = native()
    p = m.RepairParams()
    p.check = check_params(props)

    fa = p.fix_all
    fa.passes = props.fixall_passes
    fa.time_budget = props.fixall_time_budget
    fa.weld = props.weld
    fa.weld_tolerance = props.weld_tolerance
    fa.remove_tiny_shells = props.remove_tiny_shells
    fa.tiny_shell_fraction = props.tiny_shell_fraction
    fa.remove_fins = props.remove_fins
    fa.close_cracks = props.close_cracks
    fa.heal_thin_faces = props.heal_thin_faces
    fa.merge_shells = props.merge_shells

    p.remesh_target_size = props.remesh_target_size
    p.voxel_smoothing_passes = props.remesh_smoothing
    p.isosurface_smoothing_passes = props.iso_smoothing
    p.isosurface_chord_ratio = props.iso_chord_ratio
    p.alpha_wrap_detail = props.alpha_detail
    p.alpha_wrap_offset_ratio = props.alpha_offset_ratio
    p.simplify_rebuild = props.simplify_rebuild
    p.decimate_ratio = props.decimate_ratio
    p.merge_distance = props.merge_distance
    p.merge_keep_surfaces_apart = props.merge_keep_surfaces_apart
    p.merge_preserve_open_edges = props.merge_preserve_open_edges
    return p


def remesh_method(props):
    m = native()
    return {
        "VOXEL": m.Method.VOXEL,
        "ISO": m.Method.ISOSURFACE,
        "ALPHA": m.Method.ALPHA_WRAP,
    }[props.remesh_method]
