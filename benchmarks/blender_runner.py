"""Runs Blender's 3D Print Toolbox on a file and measures the input and output with Mesheal's own check.

Blender runs in the background with a separate profile (default ~/.cache/mesheal/blender-print3d/profile)
that has the 3D Print Toolbox installed, so the user's own Blender setup is not touched. Install it once:

    BLENDER_USER_RESOURCES=~/.cache/mesheal/blender-print3d/profile \\
    Blender -b --factory-startup --command extension install-file -r user_default -e add-on-print3d-toolbox-v1.5.0.zip

The time reported is wall clock for the whole Blender process, start-up included, as FixSTL measured it.
"""

from __future__ import annotations

import json
import os
import platform
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional, Union

from .runner import MeshMetrics, RepairResult, diagnostics

SCRIPT = Path(__file__).resolve().parent / "blender_print3d.py"
DEFAULT_PROFILE = Path.home() / ".cache" / "mesheal" / "blender-print3d" / "profile"


def find_blender() -> Path:
    """Blender from $BLENDER_BIN, the usual install place, or PATH."""
    env = os.getenv("BLENDER_BIN")
    if env and Path(env).is_file():
        return Path(env)
    if platform.system() == "Darwin":
        app = Path("/Applications/Blender.app/Contents/MacOS/Blender")
        if app.is_file():
            return app
    found = shutil.which("blender")
    if found:
        return Path(found)
    raise FileNotFoundError("Could not find Blender: set BLENDER_BIN")


def _pct(before: float, after: float) -> float:
    return 100.0 * (after - before) / before if before != 0.0 else 0.0


def combine(before: MeshMetrics, after: MeshMetrics) -> MeshMetrics:
    """One set of before/after metrics from Mesheal's check of the input and of the output."""
    m = MeshMetrics()
    fields = ("faces", "shells", "watertight", "open_edges", "non_manifold_edges", "non_manifold_vertices",
              "self_intersecting_faces", "thin_faces", "flipped_faces", "inverted_shells", "degenerate_faces",
              "duplicate_faces")
    for f in fields:
        setattr(m, f"{f}_before", getattr(before, f))
        setattr(m, f"{f}_after", getattr(after, f))
    m.volume_before = float(before.raw.get("volume", 0.0))
    m.volume_after = float(after.raw.get("volume", 0.0))
    m.area_before = float(before.raw.get("surface_area", 0.0))
    m.area_after = float(after.raw.get("surface_area", 0.0))
    m.volume_change_pct = _pct(m.volume_before, m.volume_after)
    m.area_change_pct = _pct(m.area_before, m.area_after)
    m.raw = {"before": before.raw, "after": after.raw}
    return m


def repair_with_print3d(
    input_file: Union[str, Path],
    output_file: Union[str, Path],
    mesheal_bin: Optional[Union[str, Path]] = None,
    blender_bin: Optional[Union[str, Path]] = None,
    profile: Optional[Union[str, Path]] = None,
    timeout_sec: Optional[float] = None,
) -> RepairResult:
    """Make Manifold on `input_file`, written to `output_file`, with Mesheal's metrics before and after."""
    action = "blender-print3d"
    src = Path(input_file).resolve()
    dst = Path(output_file).resolve()
    env = dict(os.environ, BLENDER_USER_RESOURCES=str(Path(profile) if profile else DEFAULT_PROFILE))
    cmd = [str(Path(blender_bin) if blender_bin else find_blender()), "-b", "--factory-startup",
           "--python-exit-code", "1", "--python", str(SCRIPT), "--", str(src), str(dst)]

    start = time.perf_counter()
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace", env=env,
                              timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        return RepairResult(status="error", action=action, input_file=str(src),
                            elapsed_ms=(time.perf_counter() - start) * 1000.0,
                            error=f"Blender timed out after {timeout_sec:g} s", exit_code=-int(signal.SIGKILL), timed_out=True)
    wall_ms = (time.perf_counter() - start) * 1000.0

    line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("PRINT3D_RESULT ")), None)
    if proc.returncode != 0 or line is None or not dst.is_file():
        detail = (proc.stderr.strip() or proc.stdout.strip())[-500:]
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=wall_ms,
                            error=f"Blender exited with code {proc.returncode}: {detail}", exit_code=proc.returncode or 1)

    before = diagnostics(src, mesheal_bin)
    after = diagnostics(dst, mesheal_bin)
    if not before.is_success or not after.is_success or before.metrics is None or after.metrics is None:
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=wall_ms,
                            error=f"Mesheal could not check the files: {before.error or after.error}", exit_code=1)
    metrics = combine(before.metrics, after.metrics)
    metrics.raw["blender"] = json.loads(line[len("PRINT3D_RESULT "):])
    return RepairResult(status="success", action=action, input_file=str(src), output_file=str(dst),
                        elapsed_ms=wall_ms, metrics=metrics, exit_code=0)
