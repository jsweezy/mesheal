# SPDX-License-Identifier: GPL-3.0-or-later
"""Sidebar panels (N-panel > Mesheal) and the Mesheal menu."""

from bpy.types import Menu, Panel

from . import bridge, report


class _Sidebar:
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Mesheal"

    @classmethod
    def poll(cls, context):
        return context.mode in {"OBJECT", "EDIT_MESH"}


def _big_button(layout, idname, icon):
    row = layout.row()
    row.scale_y = 1.4
    row.operator(idname, icon=icon)


class VIEW3D_PT_mesheal_check(_Sidebar, Panel):
    bl_label = "Check"

    def draw(self, context):
        layout = self.layout
        props = context.scene.mesheal

        if not bridge.available():
            layout.label(text=bridge.BUILD_HINT, icon="ERROR")
        _big_button(layout, "mesh.mesheal_check", "VIEWZOOM")

        items = report.get()
        if items:
            box = layout.box()
            row = box.row()
            row.label(text="Results")
            row.operator("wm.mesheal_report_clear", text="", icon="X", emboss=False)

            is_edit = context.mode == "EDIT_MESH"
            col = box.column(align=True)
            for i, item in enumerate(items):
                row = col.row(align=True)
                row.label(text=item.name, icon=item.icon)
                count = str(item.count)
                if item.info_only:
                    row.label(text=count)
                elif item.count == 0:
                    row.label(text="", icon="CHECKMARK")
                elif is_edit and item.indices:
                    row.operator("mesh.mesheal_select_report", text=count, icon="RESTRICT_SELECT_OFF").index = i
                else:
                    row.label(text=count, icon="ERROR")
            if not is_edit:
                box.label(text="Enter Edit Mode to select results", icon="INFO")

        header, body = layout.panel("mesheal_check_tolerances", default_closed=True)
        header.label(text="Tolerances")
        if body:
            body.use_property_split = True
            body.use_property_decorate = False
            body.prop(props, "intersection_tolerance", text="Intersection")
            body.prop(props, "thin_max_aspect", text="Thin Face Aspect")


class VIEW3D_PT_mesheal_fix_all(_Sidebar, Panel):
    bl_label = "Fix All"

    def draw(self, context):
        layout = self.layout
        props = context.scene.mesheal

        layout.row().prop(props, "scope", expand=True)
        layout.prop(props, "fixall_passes")
        _big_button(layout, "mesh.mesheal_fix_all", "TOOL_SETTINGS")

        header, body = layout.panel("mesheal_fixall_steps", default_closed=True)
        header.label(text="Steps")
        if body:
            col = body.column()
            _toggle_with_value(col, props, "weld", "weld_tolerance")
            _toggle_with_value(col, props, "remove_tiny_shells", "tiny_shell_fraction")
            col.prop(props, "remove_fins")
            col.prop(props, "close_cracks")
            col.prop(props, "heal_thin_faces")
            col.prop(props, "merge_shells")
            col.separator()
            col.prop(props, "fixall_time_budget")


def _toggle_with_value(layout, props, toggle, value):
    row = layout.row(align=True)
    row.prop(props, toggle)
    sub = row.row(align=True)
    sub.active = getattr(props, toggle)
    sub.prop(props, value, text="")


class VIEW3D_PT_mesheal_remesh(_Sidebar, Panel):
    bl_label = "Remesh"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        props = context.scene.mesheal
        layout.use_property_split = True
        layout.use_property_decorate = False

        layout.prop(props, "remesh_method")
        col = layout.column()
        if props.remesh_method in {"VOXEL", "ISO"}:
            col.prop(props, "remesh_target_size", text="Edge Length")
            if props.remesh_method == "ISO":
                col.prop(props, "iso_smoothing", text="Smoothing")
                col.prop(props, "iso_chord_ratio")
            else:
                col.prop(props, "remesh_smoothing", text="Smoothing")
        else:
            col.prop(props, "alpha_detail")
            col.prop(props, "alpha_offset_ratio")
        col.prop(props, "simplify_rebuild", text="Simplify")

        _big_button(layout, "mesh.mesheal_remesh", "MOD_REMESH")


class VIEW3D_PT_mesheal_modify(_Sidebar, Panel):
    bl_label = "Decimate & Merge"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        props = context.scene.mesheal

        layout.row().prop(props, "scope", expand=True)

        box = layout.box()
        box.label(text="Decimate", icon="MOD_DECIM")
        box.prop(props, "decimate_ratio", slider=True)
        box.operator("mesh.mesheal_decimate")

        box = layout.box()
        box.label(text="Merge by Distance", icon="AUTOMERGE_OFF")
        box.prop(props, "merge_distance")
        box.prop(props, "merge_keep_surfaces_apart")
        box.prop(props, "merge_preserve_open_edges")
        box.operator("mesh.mesheal_merge")


class VIEW3D_MT_mesheal(Menu):
    bl_label = "Mesheal"

    def draw(self, context):
        layout = self.layout
        layout.operator("mesh.mesheal_check", icon="VIEWZOOM")
        layout.operator("mesh.mesheal_fix_all", icon="TOOL_SETTINGS")
        layout.separator()
        layout.operator("mesh.mesheal_remesh", icon="MOD_REMESH")
        layout.operator("mesh.mesheal_decimate", icon="MOD_DECIM")
        layout.operator("mesh.mesheal_merge", icon="AUTOMERGE_OFF")


def draw_mesheal_menu(self, context):
    layout = self.layout
    layout.separator()
    layout.menu("VIEW3D_MT_mesheal")


classes = (
    VIEW3D_PT_mesheal_check,
    VIEW3D_PT_mesheal_fix_all,
    VIEW3D_PT_mesheal_remesh,
    VIEW3D_PT_mesheal_modify,
    VIEW3D_MT_mesheal,
)
