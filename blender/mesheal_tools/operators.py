# SPDX-License-Identifier: GPL-3.0-or-later
"""Operators. These are stubs for UI design; none of them change the mesh yet."""

from bpy.props import IntProperty
from bpy.types import Operator

from . import report


def _has_mesh(context):
    obj = context.active_object
    return obj is not None and obj.type == "MESH"


class _MeshOp:
    @classmethod
    def poll(cls, context):
        return _has_mesh(context)

    def stub(self, what):
        self.report({"INFO"}, f"Mesheal: {what} is not hooked up yet")
        return {"FINISHED"}


class MESH_OT_mesheal_check(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_check"
    bl_label = "Check Mesh"
    bl_description = "Find defects: holes, non-manifold geometry, flipped normals, self-intersections, thin faces"

    def execute(self, context):
        report.update(report.placeholder_results())
        self.report({"INFO"}, "Mesheal: showing placeholder results (not hooked up yet)")
        return {"FINISHED"}


class MESH_OT_mesheal_select_report(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_select_report"
    bl_label = "Select Result"
    bl_description = "Select the faces of this result"
    bl_options = {"INTERNAL"}

    index: IntProperty()

    def execute(self, context):
        items = report.get()
        if not 0 <= self.index < len(items):
            self.report({"WARNING"}, "Result is out of date, run Check Mesh again")
            return {"CANCELLED"}
        return self.stub(f"Select {items[self.index].name}")


class WM_OT_mesheal_report_clear(Operator):
    bl_idname = "wm.mesheal_report_clear"
    bl_label = "Clear Results"
    bl_description = "Clear the check results"
    bl_options = {"INTERNAL"}

    def execute(self, context):
        report.clear()
        return {"FINISHED"}


class MESH_OT_mesheal_fix_all(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_fix_all"
    bl_label = "Fix All"
    bl_description = "Repair every defect while keeping your geometry"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        return self.stub("Fix All")


class MESH_OT_mesheal_remesh(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_remesh"
    bl_label = "Remesh"
    bl_description = "Rebuild the mesh as a new watertight surface"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        method = context.scene.mesheal.bl_rna.properties["remesh_method"]
        label = method.enum_items[context.scene.mesheal.remesh_method].name
        return self.stub(f"Remesh ({label})")


class MESH_OT_mesheal_decimate(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_decimate"
    bl_label = "Decimate"
    bl_description = "Reduce the face count while keeping the shape"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        return self.stub("Decimate")


class MESH_OT_mesheal_merge(_MeshOp, Operator):
    bl_idname = "mesh.mesheal_merge"
    bl_label = "Merge by Distance"
    bl_description = "Merge vertices that are closer than the distance"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        return self.stub("Merge by Distance")


classes = (
    MESH_OT_mesheal_check,
    MESH_OT_mesheal_select_report,
    WM_OT_mesheal_report_clear,
    MESH_OT_mesheal_fix_all,
    MESH_OT_mesheal_remesh,
    MESH_OT_mesheal_decimate,
    MESH_OT_mesheal_merge,
)
