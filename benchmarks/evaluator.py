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


def has_volume_sign_flip(metrics: MeshMetrics) -> bool:
    """Returns True if signed volume flipped between before and after repair."""
    return (metrics.volume_before < 0.0 < metrics.volume_after) or (
        metrics.volume_before > 0.0 > metrics.volume_after
    )


def volume_magnitude_change_pct(metrics: MeshMetrics) -> float:
    """Returns percentage change in |volume|, avoiding +200% artifacts on sign flips."""
    if metrics.volume_before == 0.0:
        return metrics.volume_change_pct
    return 100.0 * (abs(metrics.volume_after) - abs(metrics.volume_before)) / abs(metrics.volume_before)


def shape_kept(metrics: MeshMetrics, max_area_delta_pct: float = 5.0) -> bool:
    """Checks FixSTL 'shapeKept' definition:
    Surface area change within 5%, or reasonable volume preservation.
    When duplicate faces were removed, compares remaining area against the
    expected non-duplicate fraction so deduplication is not penalized.
    """
    if abs(metrics.area_change_pct) <= max_area_delta_pct:
        return True
    # If initial surface area was close to 0 or highly degenerate, check volume
    vol_delta = volume_magnitude_change_pct(metrics) if has_volume_sign_flip(metrics) else metrics.volume_change_pct
    if abs(vol_delta) <= max_area_delta_pct:
        return True

    dupes_removed = max(0, metrics.duplicate_faces_before - metrics.duplicate_faces_after)
    if dupes_removed > 0 and metrics.faces_before > dupes_removed and metrics.area_before > 0.0:
        expected_ratio = (metrics.faces_before - dupes_removed) / metrics.faces_before
        actual_ratio = metrics.area_after / metrics.area_before
        if expected_ratio > 0.0:
            deduped_area_delta_pct = 100.0 * (actual_ratio - expected_ratio) / expected_ratio
            if abs(deduped_area_delta_pct) <= max_area_delta_pct:
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
            "crashed": result.crashed,
            "timed_out": result.timed_out,
            "exit_code": result.exit_code,
            "elapsed_ms": result.elapsed_ms,
        }

    m = result.metrics
    solid = is_solid(m)
    kept = shape_kept(m)
    repaired = defective_input and solid and kept
    broken = (not defective_input) and ((not solid) or (not kept))
    sign_flipped = has_volume_sign_flip(m)
    vol_mag_chg = volume_magnitude_change_pct(m)
    dupes_removed = max(0, m.duplicate_faces_before - m.duplicate_faces_after)

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
        "volume_magnitude_change_pct": vol_mag_chg,
        "volume_sign_flipped": sign_flipped,
        "duplicate_faces_removed": dupes_removed,
        "area_before": m.area_before,
        "area_after": m.area_after,
        "area_change_pct": m.area_change_pct,
        "faces_before": m.faces_before,
        "faces_after": m.faces_after,
        "elapsed_ms": result.elapsed_ms,
    }


def needs_non_dedup_check(eval_res: Dict[str, Any], threshold_pct: float = 1.0) -> bool:
    """Returns True when duplicate faces were removed and volume or area changed by > threshold_pct."""
    if eval_res.get("status") != "output":
        return False
    if eval_res.get("duplicate_faces_removed", 0) <= 0:
        return False
    vol_pct = (
        eval_res.get("volume_magnitude_change_pct", 0.0)
        if eval_res.get("volume_sign_flipped")
        else eval_res.get("volume_change_pct", 0.0)
    )
    area_pct = eval_res.get("area_change_pct", 0.0)
    return abs(vol_pct) > threshold_pct or abs(area_pct) > threshold_pct


def attach_non_dedup_metrics(
    eval_res: Dict[str, Any],
    post_dedup_result: RepairResult,
    defective_input: bool,
) -> None:
    """Attaches non-dedup volume/area deltas from running the repair on a pre-deduplicated mesh."""
    if not post_dedup_result.is_success or post_dedup_result.metrics is None:
        return
    m = post_dedup_result.metrics
    sign_flipped = has_volume_sign_flip(m)
    vol_pct = volume_magnitude_change_pct(m) if sign_flipped else m.volume_change_pct
    eval_res["non_dedup_volume_change_pct"] = vol_pct
    eval_res["non_dedup_area_change_pct"] = m.area_change_pct
    eval_res["non_dedup_volume_sign_flipped"] = sign_flipped
    if not eval_res.get("shapeKept") and shape_kept(m):
        eval_res["shapeKept"] = True
        eval_res["repaired"] = defective_input and eval_res.get("solid", False)
        eval_res["broken"] = (not defective_input) and (not eval_res.get("solid", False))

