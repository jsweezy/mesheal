"""Unit tests for FixSTL criteria evaluator."""

import unittest

from benchmarks.evaluator import (
    attach_non_dedup_metrics,
    evaluate_repair_result,
    is_solid,
    needs_non_dedup_check,
    shape_kept,
)
from benchmarks.runner import MeshMetrics, RepairResult


class TestEvaluator(unittest.TestCase):
    def test_solid_clean_mesh(self):
        m = MeshMetrics(
            watertight_after=True,
            open_edges_after=0,
            non_manifold_edges_after=0,
            non_manifold_vertices_after=0,
            self_intersecting_faces_after=0,
            thin_faces_after=0,
            degenerate_faces_after=0,
            duplicate_faces_after=0,
        )
        self.assertTrue(is_solid(m))

    def test_solid_defective_mesh(self):
        m = MeshMetrics(
            watertight_after=True,
            self_intersecting_faces_after=2,
        )
        self.assertFalse(is_solid(m))

    def test_shape_kept(self):
        m1 = MeshMetrics(area_change_pct=1.2, volume_change_pct=-0.5)
        self.assertTrue(shape_kept(m1))

        m2 = MeshMetrics(area_change_pct=12.5, volume_change_pct=20.0)
        self.assertFalse(shape_kept(m2))

    def test_evaluate_repaired(self):
        m = MeshMetrics(
            watertight_after=True,
            open_edges_after=0,
            non_manifold_edges_after=0,
            non_manifold_vertices_after=0,
            self_intersecting_faces_after=0,
            degenerate_faces_after=0,
            duplicate_faces_after=0,
            area_change_pct=0.2,
            volume_change_pct=-0.1,
            volume_before=100.0,
            volume_after=99.9,
        )
        res = RepairResult(
            status="success",
            action="fix_all",
            input_file="test.stl",
            elapsed_ms=50.0,
            metrics=m,
        )
        ev = evaluate_repair_result(defective_input=True, result=res)
        self.assertTrue(ev["repaired"])
        self.assertFalse(ev["broken"])
        self.assertFalse(ev["volume_sign_flipped"])
        self.assertEqual(ev["volume_before"], 100.0)
        self.assertEqual(ev["volume_after"], 99.9)

    def test_volume_sign_flip_flagged(self):
        m = MeshMetrics(
            watertight_after=True,
            area_change_pct=0.0,
            volume_change_pct=200.0,
            volume_before=-9564.06,
            volume_after=9564.06,
        )
        res = RepairResult(
            status="success",
            action="fix_all",
            input_file="inverted.stl",
            elapsed_ms=1.2,
            metrics=m,
        )
        ev = evaluate_repair_result(defective_input=True, result=res)
        self.assertTrue(ev["repaired"])
        self.assertTrue(ev["volume_sign_flipped"])
        self.assertAlmostEqual(ev["volume_magnitude_change_pct"], 0.0, places=6)

    def test_duplicate_faces_removed_preserves_shape_kept(self):
        # 138197-style 3x stacked duplicate mesh: 2/3 of faces are duplicates
        m = MeshMetrics(
            watertight_after=True,
            faces_before=5856,
            faces_after=1904,
            duplicate_faces_before=3904,
            duplicate_faces_after=0,
            area_before=29380.9,
            area_after=9795.05,
            area_change_pct=-66.6618,
            volume_before=32777.3,
            volume_after=10780.4,
            volume_change_pct=-67.1102,
        )
        res = RepairResult(
            status="success",
            action="fix_all",
            input_file="138197.stl",
            elapsed_ms=49.2,
            metrics=m,
        )
        ev = evaluate_repair_result(defective_input=True, result=res)
        self.assertTrue(ev["shapeKept"])
        self.assertTrue(ev["repaired"])
        self.assertEqual(ev["duplicate_faces_removed"], 3904)

    def test_duplicate_faces_still_fails_if_real_geometry_lost(self):
        # Only 10 duplicates removed out of 1000, yet 50% of area was lost
        m = MeshMetrics(
            watertight_after=True,
            faces_before=1000,
            faces_after=495,
            duplicate_faces_before=10,
            duplicate_faces_after=0,
            area_before=1000.0,
            area_after=500.0,
            area_change_pct=-50.0,
            volume_before=1000.0,
            volume_after=500.0,
            volume_change_pct=-50.0,
        )
        self.assertFalse(shape_kept(m))

    def test_non_dedup_check_and_attach(self):
        m = MeshMetrics(
            watertight_after=True,
            faces_before=5856,
            faces_after=1904,
            duplicate_faces_before=3904,
            duplicate_faces_after=0,
            area_before=29380.9,
            area_after=9795.05,
            area_change_pct=-66.6618,
            volume_before=32777.3,
            volume_after=10780.4,
            volume_change_pct=-67.1102,
        )
        ev = evaluate_repair_result(
            defective_input=True,
            result=RepairResult(status="success", action="fix_all", input_file="138197.stl", elapsed_ms=49.2, metrics=m),
        )
        self.assertTrue(needs_non_dedup_check(ev))

        post_dedup_metrics = MeshMetrics(
            watertight_after=True,
            area_before=9793.63,
            area_after=9795.05,
            area_change_pct=0.0145,
            volume_before=10925.8,
            volume_after=10780.4,
            volume_change_pct=-1.3308,
        )
        attach_non_dedup_metrics(
            ev,
            RepairResult(status="success", action="fix_all", input_file="dedup.stl", elapsed_ms=10.0, metrics=post_dedup_metrics),
            defective_input=True,
        )
        self.assertAlmostEqual(ev["non_dedup_volume_change_pct"], -1.3308, places=4)
        self.assertAlmostEqual(ev["non_dedup_area_change_pct"], 0.0145, places=4)
        self.assertFalse(ev["non_dedup_volume_sign_flipped"])


if __name__ == "__main__":
    unittest.main()

