"""Unit tests for the model cache and SHA-256 verification."""

import hashlib
import io
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from benchmarks.cache import ModelCache, compute_sha256


def _make_zip_bytes(entries: dict[str, bytes]) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in entries.items():
            zf.writestr(name, data)
    return buf.getvalue()


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

    def test_thingi10k_mirrors_tried_before_thingiverse(self):
        stl_bytes = b"solid mirror\nendsolid mirror\n"
        expected_hash = hashlib.sha256(stl_bytes).hexdigest()
        requested_urls: list[str] = []

        def fake_download(url: str) -> bytes:
            requested_urls.append(url)
            if "raw.githubusercontent.com/Thingi10K/Thingi10K" in url:
                return stl_bytes
            raise AssertionError(f"Unexpected fallback URL: {url}")

        with tempfile.TemporaryDirectory() as tmpdir:
            cache = ModelCache(Path(tmpdir))
            with patch.object(cache, "_download_url_data", side_effect=fake_download):
                path = cache.get_or_download_model(
                    "44000",
                    thing_url="https://www.thingiverse.com/thing:13497",
                    expected_sha256=expected_hash,
                )
            self.assertEqual(path, Path(tmpdir).resolve() / "44000" / "44000.stl")
            self.assertEqual(path.read_bytes(), stl_bytes)
            readme = path.parent / "README.md"
            self.assertTrue(readme.is_file())
            self.assertIn("https://www.thingiverse.com/thing:13497", readme.read_text(encoding="utf-8"))
            self.assertEqual(len(requested_urls), 1)
            self.assertIn("Thingi10K", requested_urls[0])

    def test_thingiverse_fallback_rejects_sha256_mismatch(self):
        zip_bytes = _make_zip_bytes({"files/wrong.stl": b"solid wrong\nendsolid wrong\n"})
        wrong_hash = "0" * 64

        def fake_download(url: str) -> bytes:
            if "thingiverse.com" in url:
                return zip_bytes
            raise RuntimeError("mirror unavailable")

        with tempfile.TemporaryDirectory() as tmpdir:
            cache = ModelCache(Path(tmpdir))
            with patch.object(cache, "_download_url_data", side_effect=fake_download):
                with self.assertRaises(RuntimeError):
                    cache.get_or_download_model(
                        "44000",
                        thing_url="https://www.thingiverse.com/thing:13497",
                        expected_sha256=wrong_hash,
                    )


if __name__ == "__main__":
    unittest.main()

