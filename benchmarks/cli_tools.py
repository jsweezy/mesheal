"""Runs ADMesh and MeshFix (pymeshfix) on a file and measures the input and output with Mesheal's own check.

Each file runs in its own process, and the time is the whole process, start-up included.

- ADMesh: `admesh -b out.stl in.stl`. With no fix options ADMesh runs all of its fixes: exact and nearby
  merge, remove unconnected facets, fill holes, fix normal directions and values. Install: `brew install
  admesh` (or your package manager), or set ADMESH_BIN.
- MeshFix: benchmarks/meshfix_run.py in its own Python environment (default ~/.cache/mesheal/meshfix-venv);
  see that file for the setup.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional, Union

from .blender_runner import combine
from .runner import RepairResult, diagnostics

MESHFIX_SCRIPT = Path(__file__).resolve().parent / "meshfix_run.py"
DEFAULT_MESHFIX_PYTHON = Path.home() / ".cache" / "mesheal" / "meshfix-venv" / "bin" / "python"


def find_admesh() -> Path:
    env = os.getenv("ADMESH_BIN")
    if env and Path(env).is_file():
        return Path(env)
    found = shutil.which("admesh")
    if found:
        return Path(found)
    raise FileNotFoundError("Could not find admesh: install it (brew install admesh) or set ADMESH_BIN")


def admesh_version(admesh: Path) -> str:
    out = subprocess.run([str(admesh), "--version"], capture_output=True, text=True).stdout.strip()
    return out.split("version")[-1].strip() if "version" in out else out


def _run_and_check(action: str, cmd: List[str], src: Path, dst: Path, mesheal_bin, timeout_sec, tag: Optional[str],
                   info: dict) -> RepairResult:
    start = time.perf_counter()
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace",
                              timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        return RepairResult(status="error", action=action, input_file=str(src),
                            elapsed_ms=(time.perf_counter() - start) * 1000.0,
                            error=f"{action} timed out after {timeout_sec:g} s", exit_code=-int(signal.SIGKILL), timed_out=True)
    wall_ms = (time.perf_counter() - start) * 1000.0
    line = next((ln for ln in proc.stdout.splitlines() if tag and ln.startswith(tag + " ")), None)
    if proc.returncode != 0 or not dst.is_file() or (tag and line is None):
        detail = (proc.stderr.strip() or proc.stdout.strip())[-500:]
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=wall_ms,
                            error=f"{action} exited with code {proc.returncode}: {detail}", exit_code=proc.returncode or 1)
    before = diagnostics(src, mesheal_bin)
    after = diagnostics(dst, mesheal_bin)
    if not before.is_success or not after.is_success or before.metrics is None or after.metrics is None:
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=wall_ms,
                            error=f"Mesheal could not check the files: {before.error or after.error}", exit_code=1)
    metrics = combine(before.metrics, after.metrics)
    if line is not None:
        info = {**info, **json.loads(line[len(tag) + 1:])}
    metrics.raw["tool"] = info
    return RepairResult(status="success", action=action, input_file=str(src), output_file=str(dst),
                        elapsed_ms=wall_ms, metrics=metrics, exit_code=0)


def repair_with_admesh(input_file: Union[str, Path], output_file: Union[str, Path],
                       mesheal_bin: Optional[Union[str, Path]] = None, timeout_sec: Optional[float] = None) -> RepairResult:
    admesh = find_admesh()
    src, dst = Path(input_file).resolve(), Path(output_file).resolve()
    return _run_and_check("admesh", [str(admesh), "-b", str(dst), str(src)], src, dst, mesheal_bin, timeout_sec, None,
                          {"admesh_version": admesh_version(admesh)})


def repair_with_meshfix(input_file: Union[str, Path], output_file: Union[str, Path],
                        mesheal_bin: Optional[Union[str, Path]] = None, python: Optional[Union[str, Path]] = None,
                        timeout_sec: Optional[float] = None) -> RepairResult:
    py = Path(python) if python else DEFAULT_MESHFIX_PYTHON
    if not py.is_file():
        raise FileNotFoundError(f"No pymeshfix environment at {py}: see benchmarks/meshfix_run.py")
    src, dst = Path(input_file).resolve(), Path(output_file).resolve()
    return _run_and_check("pymeshfix", [str(py), "-I", str(MESHFIX_SCRIPT), str(src), str(dst)], src, dst, mesheal_bin,
                          timeout_sec, "MESHFIX_RESULT", {})
