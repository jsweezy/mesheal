"""Model cache and downloader for Thingiverse and Thingi10K benchmark datasets."""

from __future__ import annotations

import hashlib
import io
import os
import re
import urllib.error
import urllib.request
import zipfile
from pathlib import Path
from typing import Optional


def compute_sha256(file_path: Path) -> str:
    """Computes the SHA-256 hexadecimal digest of a file."""
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()


class ModelCache:
    """Manages downloading, extracting, and caching STL benchmark models outside git."""

    def __init__(self, cache_dir: Optional[Path] = None):
        if cache_dir:
            self.cache_dir = Path(cache_dir).resolve()
        else:
            env_dir = os.getenv("OGUN_BENCHMARK_CACHE")
            if env_dir:
                self.cache_dir = Path(env_dir).resolve()
            else:
                self.cache_dir = Path.home() / ".cache" / "ogun" / "benchmarks" / "models"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def get_cached_path(self, source_id: str, expected_sha256: Optional[str] = None) -> Optional[Path]:
        """Checks if a model is already in cache and optionally verifies SHA-256."""
        candidate = self.cache_dir / f"{source_id}.stl"
        if candidate.is_file():
            if expected_sha256:
                if compute_sha256(candidate).lower() == expected_sha256.lower():
                    return candidate
            else:
                return candidate

        # Also check if saved with hash suffix
        if expected_sha256:
            candidate_hash = self.cache_dir / f"{source_id}_{expected_sha256[:8]}.stl"
            if candidate_hash.is_file() and compute_sha256(candidate_hash).lower() == expected_sha256.lower():
                return candidate_hash

        return None

    def get_or_download_model(
        self,
        source_id: str,
        thing_url: Optional[str] = None,
        expected_sha256: Optional[str] = None,
    ) -> Path:
        """Retrieves model from local cache, or downloads and verifies it."""
        cached = self.get_cached_path(source_id, expected_sha256)
        if cached:
            return cached

        target_file = self.cache_dir / f"{source_id}.stl"

        # Strategy 1: Thingiverse zip download
        if thing_url:
            m = re.search(r"thing:(\d+)", thing_url)
            if m:
                thing_num = m.group(1)
                try:
                    stl_data = self._download_from_thingiverse_zip(thing_num, expected_sha256)
                    if stl_data:
                        with open(target_file, "wb") as f:
                            f.write(stl_data)
                        return target_file
                except Exception:
                    pass

        # Strategy 2: Thingi10K raw repository / mirrors
        mirror_urls = [
            f"https://raw.githubusercontent.com/Thingi10K/Thingi10K/master/raw_meshes/{source_id}.stl",
            f"https://huggingface.co/datasets/Thingi10K/Thingi10K/resolve/main/raw_meshes/{source_id}.stl",
        ]

        for url in mirror_urls:
            try:
                stl_data = self._download_url_data(url)
                if stl_data:
                    if expected_sha256:
                        actual_hash = hashlib.sha256(stl_data).hexdigest()
                        if actual_hash.lower() != expected_sha256.lower():
                            continue
                    with open(target_file, "wb") as f:
                        f.write(stl_data)
                    return target_file
            except Exception:
                continue

        raise RuntimeError(
            f"Failed to download STL for source_id={source_id} (thing_url={thing_url}). "
            f"Could not retrieve model matching expected SHA-256."
        )

    def _download_url_data(self, url: str) -> bytes:
        headers = {"User-Agent": "Ogun-Benchmark-Tool/1.0"}
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req, timeout=30) as resp:
            return resp.read()

    def _download_from_thingiverse_zip(self, thing_id: str, expected_sha256: Optional[str]) -> Optional[bytes]:
        zip_url = f"https://www.thingiverse.com/thing:{thing_id}/zip"
        zip_bytes = self._download_url_data(zip_url)

        with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
            stls = [n for n in zf.namelist() if n.lower().endswith(".stl")]
            if not stls:
                return None

            # First, check if any file inside matches expected_sha256
            if expected_sha256:
                for name in stls:
                    content = zf.read(name)
                    if hashlib.sha256(content).hexdigest().lower() == expected_sha256.lower():
                        return content

            # Second, prioritize files under 'files/'
            files_stls = [n for n in stls if "files/" in n.lower()]
            chosen = files_stls[0] if files_stls else stls[0]
            return zf.read(chosen)
