"""Subprocess wrapper and Python interface for the Mesheal STL repair binary."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Union


@dataclass
class MeshMetrics:
    # Single-state / inspection fields
    volume: float = 0.0
    surface_area: float = 0.0
    faces: int = 0
    shells: int = 0
    watertight: bool = False
    open_edges: int = 0
    non_manifold_edges: int = 0
    non_manifold_vertices: int = 0
    self_intersecting_faces: int = 0
    thin_faces: int = 0
    flipped_faces: int = 0
    inverted_shells: int = 0
    degenerate_faces: int = 0
    duplicate_faces: int = 0

    # Before / After repair fields
    volume_before: float = 0.0
    volume_after: float = 0.0
    volume_change_pct: float = 0.0
    area_before: float = 0.0
    area_after: float = 0.0
    area_change_pct: float = 0.0
    max_deviation: float = 0.0
    max_lost: float = 0.0
    faces_before: int = 0
    faces_after: int = 0
    shells_before: int = 0
    shells_after: int = 0
    watertight_before: bool = False
    watertight_after: bool = False
    open_edges_before: int = 0
    open_edges_after: int = 0
    non_manifold_edges_before: int = 0
    non_manifold_edges_after: int = 0
    non_manifold_vertices_before: int = 0
    non_manifold_vertices_after: int = 0
    self_intersecting_faces_before: int = 0
    self_intersecting_faces_after: int = 0
    thin_faces_before: int = 0
    thin_faces_after: int = 0
    flipped_faces_before: int = 0
    flipped_faces_after: int = 0
    inverted_shells_before: int = 0
    inverted_shells_after: int = 0
    degenerate_faces_before: int = 0
    degenerate_faces_after: int = 0
    duplicate_faces_before: int = 0
    duplicate_faces_after: int = 0
    raw: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> MeshMetrics:
        kwargs: Dict[str, Any] = {}
        for k, v in data.items():
            if hasattr(cls, k):
                kwargs[k] = v
            # If single inspection metric, populate corresponding _before field
            before_key = f"{k}_before"
            if before_key in cls.__annotations__ and before_key not in data:
                kwargs[before_key] = v
        kwargs["raw"] = data
        return cls(**kwargs)


@dataclass
class RepairResult:
    status: str
    action: str
    input_file: str
    output_file: Optional[str] = None
    elapsed_ms: float = 0.0
    metrics: Optional[MeshMetrics] = None
    error: Optional[str] = None
    exit_code: int = 0

    @property
    def is_success(self) -> bool:
        return self.status == "success" and self.exit_code == 0


def find_mesheal_binary() -> Path:
    """Finds the Mesheal binary path in repository build trees or system PATH."""
    env_bin = os.getenv("MESHEAL_BIN")
    if env_bin:
        p = Path(env_bin)
        if p.is_file() and os.access(p, os.X_OK):
            return p

    repo_root = Path(__file__).resolve().parent.parent
    candidates = [
        repo_root / "build_release" / "src" / "app" / "mesheal",
        repo_root / "build_release" / "mesheal",
        repo_root / "build" / "src" / "app" / "mesheal",
        repo_root / "build" / "mesheal",
        repo_root / "build_release" / "Mesheal.app" / "Contents" / "MacOS" / "Mesheal",
        repo_root / "build" / "Mesheal.app" / "Contents" / "MacOS" / "Mesheal",
    ]

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate

    system_bin = shutil.which("mesheal")
    if system_bin:
        return Path(system_bin)

    raise FileNotFoundError(
        "Could not locate the mesheal binary. Please compile with 'cmake --build build_release -j 2' "
        "or set the MESHEAL_BIN environment variable."
    )


def diagnostics(
    input_file: Union[str, Path],
    mesheal_bin: Optional[Union[str, Path]] = None,
) -> RepairResult:
    """Runs headless mesh health inspection on an STL file."""
    bin_path = Path(mesheal_bin) if mesheal_bin else find_mesheal_binary()
    input_path = Path(input_file).resolve()

    cmd = [str(bin_path), str(input_path), "--diagnostics", "--json"]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    try:
        data = json.loads(proc.stdout.strip())
        if data.get("status") == "success":
            return RepairResult(
                status="success",
                action="diagnostics",
                input_file=str(input_path),
                elapsed_ms=data.get("elapsed_ms", 0.0),
                metrics=MeshMetrics.from_dict(data.get("metrics", {})),
                exit_code=proc.returncode,
            )
        else:
            return RepairResult(
                status="error",
                action="diagnostics",
                input_file=str(input_path),
                error=data.get("error", proc.stderr.strip()),
                exit_code=proc.returncode,
            )
    except json.JSONDecodeError:
        return RepairResult(
            status="error",
            action="diagnostics",
            input_file=str(input_path),
            error=proc.stderr.strip() or proc.stdout.strip() or "Invalid JSON output from mesheal",
            exit_code=proc.returncode,
        )


def repair(
    input_file: Union[str, Path],
    method: str = "fix-all",
    output_file: Optional[Union[str, Path]] = None,
    passes: int = 3,
    time_budget_sec: float = 0.0,
    edge_length: Optional[float] = None,
    smoothing_passes: int = 3,
    chord_ratio: float = 0.10,
    alpha: Optional[float] = None,
    offset: Optional[float] = None,
    resolution: int = 128,
    mesheal_bin: Optional[Union[str, Path]] = None,
) -> RepairResult:
    """Repairs an STL file using Mesheal and returns detailed before/after volume metrics."""
    bin_path = Path(mesheal_bin) if mesheal_bin else find_mesheal_binary()
    input_path = Path(input_file).resolve()

    method_flag_map = {
        "fix-all": "--fix-all",
        "fix_all": "--fix-all",
        "remesh-voxel": "--remesh-voxel",
        "remesh_voxel": "--remesh-voxel",
        "remesh-iso": "--remesh-iso",
        "remesh_iso": "--remesh-iso",
        "rebuild-alpha": "--rebuild-alpha",
        "rebuild_alpha": "--rebuild-alpha",
        "rebuild-union": "--rebuild-union",
        "rebuild_union": "--rebuild-union",
        "rebuild-winding": "--rebuild-winding",
        "rebuild_winding": "--rebuild-winding",
    }

    flag = method_flag_map.get(method.lower(), "--fix-all")
    cmd = [str(bin_path), str(input_path), flag, "--json"]

    if output_file:
        cmd.extend(["-o", str(Path(output_file).resolve())])

    if passes != 3:
        cmd.extend(["--passes", str(passes)])
    if time_budget_sec > 0:
        cmd.extend(["--time-budget", str(time_budget_sec)])
    if edge_length is not None:
        cmd.extend(["--edge-length", str(edge_length)])
    if smoothing_passes != 3:
        cmd.extend(["--smoothing", str(smoothing_passes)])
    if chord_ratio != 0.10:
        cmd.extend(["--chord-ratio", str(chord_ratio)])
    if alpha is not None:
        cmd.extend(["--alpha", str(alpha)])
    if offset is not None:
        cmd.extend(["--offset", str(offset)])
    if resolution != 128:
        cmd.extend(["--resolution", str(resolution)])

    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    try:
        data = json.loads(proc.stdout.strip())
        if data.get("status") == "success":
            return RepairResult(
                status="success",
                action=data.get("action", method),
                input_file=data.get("input", str(input_path)),
                output_file=data.get("output"),
                elapsed_ms=data.get("elapsed_ms", 0.0),
                metrics=MeshMetrics.from_dict(data.get("metrics", {})),
                exit_code=proc.returncode,
            )
        else:
            return RepairResult(
                status="error",
                action=method,
                input_file=str(input_path),
                error=data.get("error", proc.stderr.strip()),
                exit_code=proc.returncode,
            )
    except json.JSONDecodeError:
        return RepairResult(
            status="error",
            action=method,
            input_file=str(input_path),
            error=proc.stderr.strip() or proc.stdout.strip() or "Invalid JSON output from mesheal",
            exit_code=proc.returncode,
        )
