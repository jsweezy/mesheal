#!/usr/bin/env python3
"""Runner script to execute Ogun repairs across FixSTL benchmark datasets."""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import tempfile
import urllib.request
from pathlib import Path
from typing import Any, Dict, List

from .cache import ModelCache
from .evaluator import evaluate_repair_result
from .runner import find_ogun_binary, repair


def load_benchmark_spec(spec_source: str) -> Dict[str, Any]:
    """Loads benchmark JSON from local file path or remote HTTP/HTTPS URL."""
    p = Path(spec_source)
    if p.is_file():
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)

    headers = {"User-Agent": "Ogun-Benchmark-Tool/1.0"}
    req = urllib.request.Request(spec_source, headers=headers)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Ogun benchmark tests matching FixSTL format")
    parser.add_argument(
        "--benchmark-json",
        default="https://fixstl.ai/benchmarks/thingi10k-200-2026-10-27.json",
        help="Path or URL to input benchmark spec (default: FixSTL 200 model benchmark)",
    )
    parser.add_argument(
        "--method",
        default="fix-all",
        choices=[
            "fix-all",
            "remesh-voxel",
            "remesh-iso",
            "rebuild-alpha",
            "rebuild-union",
            "rebuild-winding",
        ],
        help="Repair method to test (default: fix-all)",
    )
    parser.add_argument("--limit", type=int, default=0, help="Limit number of models to process (0 = all)")
    parser.add_argument("--cache-dir", type=str, default=None, help="Directory to cache downloaded models")
    parser.add_argument("-o", "--output", type=str, default=None, help="Output benchmark JSON path")
    parser.add_argument("--ogun-bin", type=str, default=None, help="Explicit path to ogun binary")
    parser.add_argument("--dry-run", action="store_true", help="Download and verify cache only without running repair")

    args = parser.parse_args()

    # Verify ogun binary
    if not args.dry_run:
        try:
            bin_path = find_ogun_binary() if not args.ogun_bin else Path(args.ogun_bin)
            print(f"Using Ogun binary: {bin_path}")
        except FileNotFoundError as e:
            print(f"Error: {e}", file=sys.stderr)
            return 1

    print(f"Fetching benchmark definition from: {args.benchmark_json}")
    try:
        data = load_benchmark_spec(args.benchmark_json)
    except Exception as e:
        print(f"Failed to load benchmark spec: {e}", file=sys.stderr)
        return 1

    files: List[Dict[str, Any]] = data.get("files", [])
    if args.limit > 0:
        files = files[: args.limit]

    print(f"Loaded {len(files)} models to evaluate with method '{args.method}'")
    cache = ModelCache(Path(args.cache_dir) if args.cache_dir else None)
    print(f"Model cache directory: {cache.cache_dir}")

    total_count = len(files)
    repaired_count = 0
    solid_count = 0
    broken_count = 0
    total_elapsed_ms = 0.0

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)

        for i, entry in enumerate(files, 1):
            source_id = str(entry.get("source_id", ""))
            thing_url = entry.get("thing_url")
            expected_sha256 = entry.get("input_sha256")
            defective = entry.get("defective", True)
            entry_id = entry.get("id", f"thingi-{source_id}")

            print(f"[{i}/{total_count}] {entry_id} (source_id={source_id})...", end=" ", flush=True)

            try:
                model_path = cache.get_or_download_model(source_id, thing_url, expected_sha256)
            except Exception as e:
                print(f"DOWNLOAD FAILED: {e}")
                continue

            if args.dry_run:
                print(f"CACHED ({model_path.stat().st_size} bytes)")
                continue

            out_fixed = tmp_path / f"{source_id}_fixed.stl"
            result = repair(
                input_file=model_path,
                method=args.method,
                output_file=out_fixed,
                ogun_bin=args.ogun_bin,
            )

            eval_res = evaluate_repair_result(defective, result)
            entry.setdefault("results", {})["ogun"] = eval_res

            if eval_res.get("repaired"):
                repaired_count += 1
            if eval_res.get("solid"):
                solid_count += 1
            if eval_res.get("broken"):
                broken_count += 1
            total_elapsed_ms += eval_res.get("elapsed_ms", 0.0)

            v_chg = eval_res.get("volume_change_pct", 0.0)
            status_str = "REPAIRED" if eval_res.get("repaired") else ("SOLID" if eval_res.get("solid") else "FAILED")
            print(f"{status_str} in {eval_res.get('elapsed_ms', 0):.1f}ms (vol change: {v_chg:+.2f}%)")

    if args.dry_run:
        print("\nDry run complete.")
        return 0

    # Save output benchmark JSON
    date_str = datetime.date.today().isoformat()
    if args.output:
        out_path = Path(args.output).resolve()
    else:
        out_dir = Path("benchmarks") / "results"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"ogun-{args.method}-{len(files)}-{date_str}.json"

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print("\n" + "=" * 60)
    print("BENCHMARK SUMMARY")
    print("=" * 60)
    print(f"Total Models Tested:  {total_count}")
    print(f"Solid Output Meshes:  {solid_count} / {total_count} ({solid_count / max(1, total_count) * 100:.1f}%)")
    print(f"Repaired Defective:   {repaired_count} / {total_count} ({repaired_count / max(1, total_count) * 100:.1f}%)")
    print(f"Broken Healthy:       {broken_count}")
    print(f"Average Time / Mesh:  {total_elapsed_ms / max(1, total_count):.1f} ms")
    print(f"Results Written To:   {out_path}")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())

