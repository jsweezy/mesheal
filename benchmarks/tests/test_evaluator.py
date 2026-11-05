"""Unit tests for FixSTL criteria evaluator."""

import unittest

from benchmarks.evaluator import evaluate_repair_result, is_solid, shape_kept
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
        self.assertEqual(ev["volume_before"], 100.0)
        self.assertEqual(ev["volume_after"], 99.9)


if __name__ == "__main__":
    unittest.main()
