"""Unit tests for the Mesheal Python runner."""

from pathlib import Path
import stat
import tempfile
import unittest

from benchmarks.evaluator import evaluate_repair_result
from benchmarks.runner import diagnostics, find_mesheal_binary, repair


class TestMeshealRunner(unittest.TestCase):
    def setUp(self):
        self.mesheal_bin = find_mesheal_binary()
        self.repo_root = Path(__file__).resolve().parent.parent.parent
        self.example_box = self.repo_root / "examples" / "self_intersecting_box.stl"

    def test_find_mesheal_binary(self):
        self.assertTrue(self.mesheal_bin.is_file())

    def test_diagnostics_on_box(self):
        res = diagnostics(self.example_box, mesheal_bin=self.mesheal_bin)
        self.assertTrue(res.is_success)
        self.assertIsNotNone(res.metrics)
        self.assertEqual(res.metrics.self_intersecting_faces_before, 6)
        self.assertEqual(res.metrics.volume_before, 250.0)

    def test_repair_fix_all_on_box(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            out_file = Path(tmpdir) / "repaired.stl"
            res = repair(self.example_box, method="fix-all", output_file=out_file, mesheal_bin=self.mesheal_bin)
            self.assertTrue(res.is_success)
            self.assertIsNotNone(res.metrics)
            self.assertEqual(res.metrics.self_intersecting_faces_after, 0)
            self.assertTrue(res.metrics.watertight_after)
            self.assertGreater(res.metrics.volume_after, 0.0)
            self.assertNotEqual(res.metrics.volume_change_pct, 0.0)
            self.assertTrue(out_file.is_file())


class TestCrashReporting(unittest.TestCase):
    """A mesheal process that dies on a signal must be reported as a crash, not a 0 ms failure."""

    def _fake_bin(self, tmpdir: str, body: str) -> Path:
        path = Path(tmpdir) / "fake_mesheal"
        path.write_text("#!/bin/sh\n" + body + "\n")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)
        return path

    def test_signal_death_is_reported_as_crash(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            fake = self._fake_bin(tmpdir, "echo 'partial log' >&2\nkill -ABRT $$")
            res = repair(Path(tmpdir) / "in.stl", mesheal_bin=fake)
        self.assertFalse(res.is_success)
        self.assertTrue(res.crashed)
        self.assertIn("SIGABRT", res.error)
        self.assertIn("partial log", res.error)
        self.assertGreater(res.elapsed_ms, 0.0)

        ev = evaluate_repair_result(True, res)
        self.assertEqual(ev["status"], "failed")
        self.assertTrue(ev["crashed"])
        self.assertEqual(ev["exit_code"], -6)

    def test_slow_run_is_stopped_and_reported_as_timed_out(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            fake = self._fake_bin(tmpdir, "exec sleep 30")
            res = repair(Path(tmpdir) / "in.stl", mesheal_bin=fake, timeout_sec=0.5)
        self.assertFalse(res.is_success)
        self.assertTrue(res.timed_out)
        self.assertFalse(res.crashed)
        self.assertIn("timed out", res.error)
        self.assertLess(res.elapsed_ms, 10000.0)

        ev = evaluate_repair_result(True, res)
        self.assertEqual(ev["status"], "failed")
        self.assertTrue(ev["timed_out"])
        self.assertFalse(ev["crashed"])

    def test_nonzero_exit_without_json_is_error_not_crash(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            fake = self._fake_bin(tmpdir, "echo 'bad input' >&2\nexit 2")
            res = diagnostics(Path(tmpdir) / "in.stl", mesheal_bin=fake)
        self.assertFalse(res.crashed)
        self.assertIn("exited with code 2", res.error)
        self.assertIn("bad input", res.error)


if __name__ == "__main__":
    unittest.main()
