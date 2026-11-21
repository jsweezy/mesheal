"""Runs Blender's 3D Print Toolbox "Make Manifold" on one STL file, as the FixSTL benchmark did.

Runs inside Blender, in the background, one file per process:

    BLENDER_USER_RESOURCES=<profile with print3d_toolbox installed> \\
    Blender -b --factory-startup --python-exit-code 1 --python benchmarks/blender_print3d.py -- in.stl out.stl

The file is imported and exported with Blender's default STL settings, and Make Manifold
(mesh.print3d_clean_non_manifold) runs with its default settings. Prints one JSON line starting with
"PRINT3D_RESULT " with the toolbox version and the time spent in each step.
"""

import json
import sys
import time

import addon_utils
import bpy

MODULE = "bl_ext.user_default.print3d_toolbox"


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    if len(argv) != 2:
        raise SystemExit("usage: ... -- input.stl output.stl")
    src, dst = argv

    _, loaded = addon_utils.enable(MODULE, default_set=True, handle_error=None), None
    version = None
    for mod in addon_utils.modules():
        if mod.__name__ == MODULE:
            version = ".".join(str(v) for v in addon_utils.module_bl_info(mod).get("version", ()))
    if not hasattr(bpy.ops.mesh, "print3d_clean_non_manifold") or MODULE not in bpy.context.preferences.addons:
        raise SystemExit(f"3D Print Toolbox is not installed ({MODULE}); see benchmarks/blender_print3d.py")

    # Start from an empty scene
    bpy.ops.wm.read_factory_settings(use_empty=True)
    addon_utils.enable(MODULE, default_set=True)

    t0 = time.perf_counter()
    bpy.ops.wm.stl_import(filepath=src)
    t1 = time.perf_counter()
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not meshes:
        raise SystemExit("The STL file has no mesh")
    obj = meshes[0]
    bpy.context.view_layer.objects.active = obj
    for o in bpy.context.scene.objects:
        o.select_set(o == obj)

    bpy.ops.mesh.print3d_clean_non_manifold()
    t2 = time.perf_counter()

    bpy.ops.wm.stl_export(filepath=dst, export_selected_objects=True)
    t3 = time.perf_counter()

    print("PRINT3D_RESULT " + json.dumps({
        "toolbox_version": version,
        "blender_version": bpy.app.version_string,
        "faces_after": len(obj.data.polygons),
        "import_s": t1 - t0,
        "make_manifold_s": t2 - t1,
        "export_s": t3 - t2,
    }), flush=True)


main()
