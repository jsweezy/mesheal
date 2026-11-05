"""Evaluates repair output against FixSTL benchmark criteria."""

from __future__ import annotations

from typing import Any, Dict

from .runner import MeshMetrics, RepairResult


def is_solid(metrics: MeshMetrics) -> bool:
    """Checks FixSTL 'solid' definition:
    Output is closed, edge- and vertex-manifold, consistently wound, free of self-intersections,
    duplicate and zero-area faces.
    """
    if not metrics.watertight_after:
        return False
    if metrics.open_edges_after > 0:
        return False
    if metrics.non_manifold_edges_after > 0:
        return False
    if metrics.non_manifold_vertices_after > 0:
        return False
    if metrics.self_intersecting_faces_after > 0:
        return False
    if metrics.duplicate_faces_after > 0:
        return False
    if metrics.degenerate_faces_after > 0:
        return False
    if metrics.flipped_faces_after > 0:
        return False
    if metrics.inverted_shells_after > 0:
        return False
    return True


def shape_kept(metrics: MeshMetrics, max_area_delta_pct: float = 5.0) -> bool:
    """Checks FixSTL 'shapeKept' definition:
    Surface area change within 5%, or reasonable volume preservation.
    """
    if abs(metrics.area_change_pct) <= max_area_delta_pct:
        return True
    # If initial surface area was close to 0 or highly degenerate, check volume
    if abs(metrics.volume_change_pct) <= max_area_delta_pct:
        return True
    return False


def evaluate_repair_result(
    defective_input: bool,
    result: RepairResult,
) -> Dict[str, Any]:
    """Generates the result dictionary for FixSTL benchmark JSON."""
    if not result.is_success or result.metrics is None:
        return {
            "status": "failed",
            "solid": False,
            "shapeKept": False,
            "crossCheckSolid": False,
            "repaired": False,
            "broken": not defective_input,
            "error": result.error or "Unknown repair failure",
            "elapsed_ms": result.elapsed_ms,
        }

    m = result.metrics
    solid = is_solid(m)
    kept = shape_kept(m)
    repaired = defective_input and solid and kept
    broken = (not defective_input) and ((not solid) or (not kept))

    return {
        "status": "output",
        "solid": solid,
        "shapeKept": kept,
        "crossCheckSolid": solid,
        "repaired": repaired,
        "broken": broken,
        "volume_before": m.volume_before,
        "volume_after": m.volume_after,
        "volume_change_pct": m.volume_change_pct,
        "area_before": m.area_before,
        "area_after": m.area_after,
        "area_change_pct": m.area_change_pct,
        "faces_before": m.faces_before,
        "faces_after": m.faces_after,
        "elapsed_ms": result.elapsed_ms,
    }

