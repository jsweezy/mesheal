"""Unit tests for the Ogun Python runner."""

from pathlib import Path
import tempfile
import unittest

from benchmarks.runner import diagnostics, find_ogun_binary, repair


class TestOgunRunner(unittest.TestCase):
    def setUp(self):
        self.ogun_bin = find_ogun_binary()
        self.repo_root = Path(__file__).resolve().parent.parent.parent
        self.example_box = self.repo_root / "examples" / "self_intersecting_box.stl"

    def test_find_ogun_binary(self):
        self.assertTrue(self.ogun_bin.is_file())

    def test_diagnostics_on_box(self):
        res = diagnostics(self.example_box, ogun_bin=self.ogun_bin)
        self.assertTrue(res.is_success)
        self.assertIsNotNone(res.metrics)
        self.assertEqual(res.metrics.self_intersecting_faces_before, 6)
        self.assertEqual(res.metrics.volume_before, 250.0)

    def test_repair_fix_all_on_box(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            out_file = Path(tmpdir) / "repaired.stl"
            res = repair(self.example_box, method="fix-all", output_file=out_file, ogun_bin=self.ogun_bin)
            self.assertTrue(res.is_success)
            self.assertIsNotNone(res.metrics)
            self.assertEqual(res.metrics.self_intersecting_faces_after, 0)
            self.assertTrue(res.metrics.watertight_after)
            self.assertGreater(res.metrics.volume_after, 0.0)
            self.assertNotEqual(res.metrics.volume_change_pct, 0.0)
            self.assertTrue(out_file.is_file())


if __name__ == "__main__":
    unittest.main()
