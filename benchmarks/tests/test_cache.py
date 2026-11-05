"""Unit tests for the model cache and SHA-256 verification."""

import tempfile
import unittest
from pathlib import Path

from benchmarks.cache import ModelCache, compute_sha256


class TestCache(unittest.TestCase):
    def test_cache_and_sha256(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            cache = ModelCache(Path(tmpdir))
            test_file = cache.cache_dir / "1234.stl"
            test_file.write_bytes(b"solid test\nendsolid test\n")

            expected_hash = compute_sha256(test_file)
            self.assertIsNotNone(expected_hash)

            found = cache.get_cached_path("1234", expected_hash)
            self.assertEqual(found, test_file)

            # Wrong hash should return None
            mismatch = cache.get_cached_path("1234", "0000000000000000000000000000000000000000000000000000000000000000")
            self.assertIsNone(mismatch)


if __name__ == "__main__":
    unittest.main()
