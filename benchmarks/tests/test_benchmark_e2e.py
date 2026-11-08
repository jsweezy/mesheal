"""End-to-end integration test for FixSTL benchmark runner."""

import json
import shutil
import tempfile
import unittest
from pathlib import Path

from benchmarks.cache import ModelCache, compute_sha256
from benchmarks.run_benchmark import load_benchmark_spec
from benchmarks.runner import repair
from benchmarks.evaluator import evaluate_repair_result


class TestBenchmarkE2E(unittest.TestCase):
    def test_e2e_benchmark_flow(self):
        repo_root = Path(__file__).resolve().parent.parent.parent
        example_box = repo_root / "examples" / "self_intersecting_box.stl"
        fixture_json = repo_root / "benchmarks" / "tests" / "fixtures" / "sample_benchmark.json"

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            cache_dir = tmp_path / "cache"
            cache = ModelCache(cache_dir)

            # Pre-populate cache so no network access is needed
            cached_stl = cache_dir / "self_intersecting_box.stl"
            shutil.copyfile(example_box, cached_stl)
            file_hash = compute_sha256(cached_stl)

            spec = load_benchmark_spec(str(fixture_json))
            spec["files"][0]["input_sha256"] = file_hash

            out_fixed = tmp_path / "box_repaired.stl"
            res = repair(cached_stl, method="fix-all", output_file=out_fixed)
            self.assertTrue(res.is_success)

            ev = evaluate_repair_result(defective_input=True, result=res)
            spec["files"][0]["results"]["mesheal"] = ev

            out_json = tmp_path / "result.json"
            with open(out_json, "w") as f:
                json.dump(spec, f, indent=2)

            # Verify saved output JSON
            with open(out_json, "r") as f:
                saved_data = json.load(f)

            mesheal_res = saved_data["files"][0]["results"]["mesheal"]
            self.assertEqual(mesheal_res["status"], "output")
            self.assertTrue(mesheal_res["solid"])
            # Self-intersecting box has 19% area change when unfolded, so shapeKept is False under strict 5% limit
            self.assertFalse(mesheal_res["shapeKept"])
            self.assertFalse(mesheal_res["repaired"])
            self.assertEqual(mesheal_res["volume_before"], 250.0)
            self.assertGreater(mesheal_res["volume_after"], 250.0)
            self.assertIn("volume_change_pct", mesheal_res)


if __name__ == "__main__":
    unittest.main()
