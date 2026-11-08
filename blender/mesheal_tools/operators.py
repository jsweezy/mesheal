# SPDX-License-Identifier: GPL-3.0-or-later
"""Operators. Check, Fix All, Remesh, Decimate and Merge call the native _mesheal module.

Each one works two ways: `execute()` runs synchronously (scripts, F3 search, redo), and `invoke()`
runs the job on a C++ thread while a modal timer shows progress; Esc cancels.
"""

import bpy
import numpy as np
from bpy.props import IntProperty
from bpy.types import Operator

from . import bridge, report

_active = None  # the running job operator, if any

_PASS_THROUGH = {
    "MIDDLEMOUSE",
    "WHEELUPMOUSE",
    "WHEELDOWNMOUSE",
    "WHEELINMOUSE",
    "WHEELOUTMOUSE",
    "TRACKPADPAN",
    "TRACKPADZOOM",
    "MOUSEMOVE",
    "INBETWEEN_MOUSEMOVE",
    "NDOF_MOTION",
    "NDOF_BUTTON_MENU",
    "NDOF_BUTTON_FIT",
}


def _has_mesh(context):
    obj = context.active_object
    return obj is not None and obj.type == "MESH"


class _Task:
    """What a job operator needs to run and to apply its result."""

    def __init__(self, obj, verts, faces, tri_to_poly, polygons, mask=None, method=None, params=None):
        self.obj = obj
        self.verts = verts
        self.faces = faces
        self.tri_to_poly = tri_to_poly
        self.polygons = polygons
        self.mask = mask
        self.method = method  # None: a check
        self.params = params


class _JobOperator:
    """Mixin: run a check or repair synchronously (execute) or as a cancellable modal job (invoke)."""

    use_scope = False

    @classmethod
    def poll(cls, context):
        return _has_mesh(context)

    # -- subclass hooks -----------------------------------------------------
    def method(self, context):
        """The native Method for a repair, or None for a check."""
        raise NotImplementedError

    def params(self, context):
        return bridge.repair_params(context.scene.mesheal)

    def apply(self, context, task, result):
        raise NotImplementedError

    # -- shared -------------------------------------------------------------
    def prepare(self, context):
        if not bridge.available():
            self.report({"ERROR"}, bridge.BUILD_HINT)
            return None
        obj = context.active_object
        props = context.scene.mesheal
        verts, faces, tri_to_poly, polygons = bridge.read_mesh(obj)
        if len(faces) == 0:
            self.report({"WARNING"}, "Mesheal: the mesh has no faces")
            return None
        mask = None
        if self.use_scope and props.scope == "SELECTED":
            mask = bridge.selection_mask(obj.data, tri_to_poly)
            if not mask.any():
                self.report({"ERROR"}, "Mesheal: no faces are selected")
                return None
        return _Task(obj, verts, faces, tri_to_poly, polygons, mask, self.method(context), self.params(context))

    def _start(self, task):
        m = bridge.native()
        if task.method is None:
            return m.start_check(task.verts, task.faces, task.params.check)
        return m.start_repair(task.verts, task.faces, task.method, task.params, task.mask)

    def _run_sync(self, task):
        m = bridge.native()
        if task.method is None:
            return m.check(task.verts, task.faces, task.params.check)
        return m.repair(task.verts, task.faces, task.method, task.params, task.mask)

    def _finish(self, context, task, result):
        obj = bpy.data.objects.get(task.obj.name)
        if obj is None or bridge.polygon_count(obj) != task.polygons:
            self.report({"WARNING"}, "Mesheal: the mesh changed while the job ran; result discarded")
            return {"CANCELLED"}
        return self.apply(context, task, result)

    def execute(self, context):
        task = self.prepare(context)
        if task is None:
            return {"CANCELLED"}
        try:
            result = self._run_sync(task)
        except Exception as exc:  # noqa: BLE001 - surface any native error to the user
            self.report({"ERROR"}, f"Mesheal: {exc}")
            return {"CANCELLED"}
        return self._finish(context, task, result)

    def invoke(self, context, event):
        global _active
        if _active is not None:
            self.report({"WARNING"}, "Mesheal: another job is already running")
            return {"CANCELLED"}
        if context.window is None:
            return self.execute(context)
        task = self.prepare(context)
        if task is None:
            return {"CANCELLED"}
        try:
            self._job = self._start(task)
        except Exception as exc:  # noqa: BLE001
            self.report({"ERROR"}, f"Mesheal: {exc}")
            return {"CANCELLED"}
        self._task = task
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.progress_begin(0, 100)
        wm.modal_handler_add(self)
        _active = self
        return {"RUNNING_MODAL"}

    def _cleanup(self, context):
        global _active
        wm = context.window_manager
        if getattr(self, "_timer", None) is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        wm.progress_end()
        try:
            context.workspace.status_text_set(None)
        except AttributeError:
            pass
        _active = None

    def modal(self, context, event):
        job = self._job
        if event.type == "ESC" and event.value == "PRESS":
            job.cancel()
            return {"RUNNING_MODAL"}

        if event.type == "TIMER" and not job.done:
            pct = int(job.progress * 100)
            context.window_manager.progress_update(pct)
            context.workspace.status_text_set(f"Mesheal: {job.status_text} {pct}%  (Esc to cancel)")
            return {"RUNNING_MODAL"}

        if job.done:
            self._cleanup(context)
            try:
                result = job.result()
            except Exception as exc:  # noqa: BLE001
                self.report({"ERROR"}, f"Mesheal: {exc}")
                return {"CANCELLED"}
            if _was_cancelled(result):
                self.report({"INFO"}, "Mesheal: cancelled")
                return {"CANCELLED"}
            return self._finish(context, self._task, result)

        if event.type in _PASS_THROUGH or event.type.startswith("NDOF_"):
            return {"PASS_THROUGH"}
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        job = getattr(self, "_job", None)
        if job is not None:
            job.cancel()
        self._cleanup(context)


def _was_cancelled(result):
    if isinstance(result, list):
        return any(r.cancelled for r in result)
    return bool(result.cancelled)


class _RepairOperator(_JobOperator):
    bl_options = {"REGISTER", "UNDO"}

    def apply(self, context, task, result):
        if not result.success:
            self.report({"ERROR"}, f"Mesheal: {result.status or 'repair failed'}")
            return {"CANCELLED"}
        before = len(task.faces)
        bridge.write_mesh(task.obj, result.vertices, result.faces)
        for line in result.report:
            print(f"Mesheal: {line}")
        report.clear()
        status = result.status or "done"
        self.report({"INFO"}, f"Mesheal: {status} ({before} -> {len(result.faces)} triangles)")
        return {"FINISHED"}


class MESH_OT_mesheal_check(_JobOperator, Operator):
    bl_idname = "mesh.mesheal_check"
    bl_label = "Check Mesh"
    bl_description = "Find defects: holes, non-manifold geometry, flipped normals, self-intersections, thin faces"

    def method(self, context):
        return None

    def params(self, context):
        return _CheckOnly(bridge.check_params(context.scene.mesheal))

    def apply(self, context, task, results):
        items = []
        for r in results:
            label, icon = report.CATEGORIES[r.category.name]
            info = r.category.name == "SHELLS"
            polys = ()
            if len(r.faces):
                polys = tuple(int(i) for i in np.unique(task.tri_to_poly[r.faces]))
            items.append(report.Item(label, r.info_count if info else r.count, icon, info_only=info, indices=polys))
        report.update(items, task.obj, task.polygons)
        defects = sum(i.count for i in items if not i.info_only)
        self.report({"INFO"}, f"Mesheal: {defects} defects found" if defects else "Mesheal: no defects found")
        return {"FINISHED"}


class _CheckOnly:
    """Task params of a check: only `.check` is used."""

    def __init__(self, check):
        self.check = check


class MESH_OT_mesheal_select_report(Operator):
    bl_idname = "mesh.mesheal_select_report"
    bl_label = "Select Result"
    bl_description = "Select the faces of this result"
    bl_options = {"INTERNAL"}

    index: IntProperty()

    @classmethod
    def poll(cls, context):
        return _has_mesh(context) and context.mode == "EDIT_MESH"

    def execute(self, context):
        import bmesh

        items = report.get()
        obj = context.edit_object
        if not 0 <= self.index < len(items):
            self.report({"WARNING"}, "Result is out of date, run Check Mesh again")
            return {"CANCELLED"}
        bm = bmesh.from_edit_mesh(obj.data)
        if report.stale(obj, len(bm.faces)):
            self.report({"WARNING"}, "Result is out of date, run Check Mesh again")
            return {"CANCELLED"}
        context.tool_settings.mesh_select_mode = (False, False, True)
        bm.select_mode = {"FACE"}
        for f in bm.faces:
            f.select_set(False)
        bm.faces.ensure_lookup_table()
        for i in items[self.index].indices:
            bm.faces[i].select_set(True)
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
        return {"FINISHED"}


class WM_OT_mesheal_report_clear(Operator):
    bl_idname = "wm.mesheal_report_clear"
    bl_label = "Clear Results"
    bl_description = "Clear the check results"
    bl_options = {"INTERNAL"}

    def execute(self, context):
        report.clear()
        return {"FINISHED"}


class MESH_OT_mesheal_fix_all(_RepairOperator, Operator):
    bl_idname = "mesh.mesheal_fix_all"
    bl_label = "Fix All"
    bl_description = "Repair every defect while keeping your geometry"
    use_scope = True

    def method(self, context):
        return bridge.native().Method.FIX_ALL


class MESH_OT_mesheal_remesh(_RepairOperator, Operator):
    bl_idname = "mesh.mesheal_remesh"
    bl_label = "Remesh"
    bl_description = "Rebuild the mesh as a new watertight surface"

    def method(self, context):
        return bridge.remesh_method(context.scene.mesheal)


class MESH_OT_mesheal_decimate(_RepairOperator, Operator):
    bl_idname = "mesh.mesheal_decimate"
    bl_label = "Decimate"
    bl_description = "Reduce the face count while keeping the shape"
    use_scope = True

    def method(self, context):
        return bridge.native().Method.DECIMATE


class MESH_OT_mesheal_merge(_RepairOperator, Operator):
    bl_idname = "mesh.mesheal_merge"
    bl_label = "Merge by Distance"
    bl_description = "Merge vertices that are closer than the distance"
    use_scope = True

    def method(self, context):
        return bridge.native().Method.MERGE


classes = (
    MESH_OT_mesheal_check,
    MESH_OT_mesheal_select_report,
    WM_OT_mesheal_report_clear,
    MESH_OT_mesheal_fix_all,
    MESH_OT_mesheal_remesh,
    MESH_OT_mesheal_decimate,
    MESH_OT_mesheal_merge,
)
