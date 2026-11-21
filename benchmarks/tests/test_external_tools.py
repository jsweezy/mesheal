"""Unit tests for judging another tool's output with Mesheal's metrics (no Blender or browser needed)."""

import unittest

from benchmarks.blender_runner import combine
from benchmarks.evaluator import evaluate_repair_result
from benchmarks.runner import MeshMetrics, RepairResult


def check(**kw):
    """Metrics as `mesheal --diagnostics --json` reports them for one file."""
    data = {"volume": 10.0, "surface_area": 30.0, "faces": 12, "shells": 1, "watertight": True, "open_edges": 0,
            "non_manifold_edges": 0, "non_manifold_vertices": 0, "flipped_faces": 0, "inverted_shells": 0,
            "self_intersecting_faces": 0, "thin_faces": 0, "degenerate_faces": 0, "duplicate_faces": 0}
    data.update(kw)
    return MeshMetrics.from_dict(data)


class TestExternalTools(unittest.TestCase):
    def test_before_and_after_come_from_the_two_checks(self):
        m = combine(check(open_edges=4, watertight=False, faces=10, surface_area=28.0), check(volume=11.0))
        self.assertEqual((m.open_edges_before, m.open_edges_after), (4, 0))
        self.assertEqual((m.faces_before, m.faces_after), (10, 12))
        self.assertFalse(m.watertight_before)
        self.assertTrue(m.watertight_after)
        self.assertAlmostEqual(m.volume_change_pct, 10.0)
        self.assertAlmostEqual(m.area_change_pct, 100.0 * 2.0 / 28.0)

    def test_a_closed_output_with_the_same_shape_counts_as_repaired(self):
        m = combine(check(open_edges=4, watertight=False), check())
        res = evaluate_repair_result(True, RepairResult(status="success", action="x", input_file="a", metrics=m))
        self.assertTrue(res["repaired"])

    def test_a_kept_broken_input_is_not_repaired(self):
        m = combine(check(self_intersecting_faces=3), check(self_intersecting_faces=3))
        res = evaluate_repair_result(True, RepairResult(status="success", action="x", input_file="a", metrics=m))
        self.assertFalse(res["repaired"])
        self.assertFalse(res["solid"])


if __name__ == "__main__":
    unittest.main()
