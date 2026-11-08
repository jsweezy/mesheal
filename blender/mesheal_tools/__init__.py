# SPDX-License-Identifier: GPL-3.0-or-later
"""Mesheal tools for Blender: mesh checks and repairs (UI prototype, operators are stubs)."""

if "bpy" in locals():
    import importlib

    importlib.reload(props)  # noqa: F821
    importlib.reload(report)  # noqa: F821
    importlib.reload(operators)  # noqa: F821
    importlib.reload(ui)  # noqa: F821
else:
    from . import props, report, operators, ui

import bpy
from bpy.props import PointerProperty

classes = (
    props.MeshealSceneProps,
    *operators.classes,
    *ui.classes,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.mesheal = PointerProperty(type=props.MeshealSceneProps)
    bpy.types.VIEW3D_MT_object.append(ui.draw_mesheal_menu)
    bpy.types.VIEW3D_MT_edit_mesh.append(ui.draw_mesheal_menu)


def unregister():
    bpy.types.VIEW3D_MT_edit_mesh.remove(ui.draw_mesheal_menu)
    bpy.types.VIEW3D_MT_object.remove(ui.draw_mesheal_menu)
    del bpy.types.Scene.mesheal
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    report.clear()
