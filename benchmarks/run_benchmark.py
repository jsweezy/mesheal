#!/usr/bin/env python3
"""Runner script to execute Mesheal repairs across FixSTL benchmark datasets."""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import tempfile
import urllib.request
from pathlib import Path
from typing import Any, Dict, List

from .blender_runner import repair_with_print3d
from .cache import ModelCache
from .cli_tools import repair_with_admesh, repair_with_meshfix
from .fixstl_runner import repair_with_fixstl_web
from .evaluator import attach_non_dedup_metrics, evaluate_repair_result, needs_non_dedup_check
from .runner import find_mesheal_binary, repair


def load_benchmark_spec(spec_source: str) -> Dict[str, Any]:
    """Loads benchmark JSON from local file path or remote HTTP/HTTPS URL."""
    p = Path(spec_source)
    if p.is_file():
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)

    headers = {"User-Agent": "Mesheal-Benchmark-Tool/1.0"}
    req = urllib.request.Request(spec_source, headers=headers)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    DEFAULT_BENCHMARK_JSON = str(Path(__file__).resolve().parent / "specs" / "thingi10k-200-2026-10-27.json")

    parser = argparse.ArgumentParser(description="Run Mesheal benchmark tests matching FixSTL format")
    parser.add_argument(
        "--benchmark-json",
        default=DEFAULT_BENCHMARK_JSON,
        help="Path or URL to input benchmark spec (default: archived FixSTL 200-model benchmark)",
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
            "blender-print3d",
            "fixstl-web",
            "admesh",
            "pymeshfix",
        ],
        help="Repair method to test (default: fix-all). blender-print3d runs Blender's 3D Print Toolbox "
        "(Make Manifold) instead of Mesheal, fixstl-web the FixSTL web tool in a headless browser, admesh ADMesh "
        "and pymeshfix MeshFix; all are checked with Mesheal",
    )
    parser.add_argument("--limit", type=int, default=0, help="Limit number of models to process (0 = all)")
    parser.add_argument("--cache-dir", type=str, default=None, help="Directory to cache downloaded models")
    parser.add_argument("-o", "--output", type=str, default=None, help="Output benchmark JSON path")
    parser.add_argument("--mesheal-bin", type=str, default=None, help="Explicit path to mesheal binary")
    parser.add_argument(
        "--timeout",
        type=float,
        default=1800.0,
        help="Stop a repair that runs longer than this many seconds, wall clock (default: 1800; 0 = no limit)",
    )
    parser.add_argument("--blender-bin", type=str, default=None, help="Blender to run for blender-print3d")
    parser.add_argument("--blender-profile", type=str, default=None,
                        help="Blender profile with the 3D Print Toolbox (default: ~/.cache/mesheal/blender-print3d/profile)")
    parser.add_argument("--fixstl-python", type=str, default=None,
                        help="Python with Playwright for fixstl-web (default: ~/.cache/mesheal/fixstl-venv/bin/python)")
    parser.add_argument("--meshfix-python", type=str, default=None,
                        help="Python with pymeshfix for pymeshfix (default: ~/.cache/mesheal/meshfix-venv/bin/python)")
    parser.add_argument("--dry-run", action="store_true", help="Download and verify cache only without running repair")

    args = parser.parse_args()

    # Verify mesheal binary
    if not args.dry_run:
        try:
            bin_path = find_mesheal_binary() if not args.mesheal_bin else Path(args.mesheal_bin)
            print(f"Using Mesheal binary: {bin_path}")
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
    defective_count = 0
    healthy_count = 0
    crashed_ids: List[str] = []
    timed_out_ids: List[str] = []
    total_elapsed_ms = 0.0
    blender = args.method == "blender-print3d"
    fixstl = args.method == "fixstl-web"
    # Other tools' results get their own key, so FixSTL's published entries and Mesheal's stay as they are
    other_keys = {"blender-print3d": "blender_print3d_local", "fixstl-web": "fixstl_web_local",
                  "admesh": "admesh_local", "pymeshfix": "pymeshfix_local"}
    other = args.method in other_keys
    result_key = other_keys.get(args.method, "mesheal")

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
            if args.method == "admesh":
                result = repair_with_admesh(model_path, out_fixed, mesheal_bin=args.mesheal_bin, timeout_sec=args.timeout or None)
            elif args.method == "pymeshfix":
                result = repair_with_meshfix(model_path, out_fixed, mesheal_bin=args.mesheal_bin, python=args.meshfix_python,
                                             timeout_sec=args.timeout or None)
            elif fixstl:
                result = repair_with_fixstl_web(model_path, out_fixed, mesheal_bin=args.mesheal_bin,
                                                python=args.fixstl_python, timeout_sec=args.timeout or None)
            elif blender:
                result = repair_with_print3d(model_path, out_fixed, mesheal_bin=args.mesheal_bin,
                                             blender_bin=args.blender_bin, profile=args.blender_profile,
                                             timeout_sec=args.timeout or None)
            else:
                result = repair(
                    input_file=model_path,
                    method=args.method,
                    output_file=out_fixed,
                    mesheal_bin=args.mesheal_bin,
                    timeout_sec=args.timeout or None,
                )

            eval_res = evaluate_repair_result(defective, result)
            if blender and result.metrics is not None:
                eval_res["blender"] = result.metrics.raw.get("blender", {})
            if fixstl and result.metrics is not None:
                eval_res["fixstl"] = result.metrics.raw.get("fixstl", {})
            if other and result.metrics is not None and "tool" in result.metrics.raw:
                eval_res["tool"] = result.metrics.raw["tool"]
            if not other and args.method != "dedup" and needs_non_dedup_check(eval_res):
                out_dedup = tmp_path / f"{source_id}_dedup.stl"
                dedup_res = repair(
                    input_file=model_path,
                    method="dedup",
                    output_file=out_dedup,
                    mesheal_bin=args.mesheal_bin,
                    timeout_sec=args.timeout or None,
                )
                if dedup_res.is_success and out_dedup.is_file():
                    out_post_dedup = tmp_path / f"{source_id}_post_dedup_fixed.stl"
                    post_dedup_res = repair(
                        input_file=out_dedup,
                        method=args.method,
                        output_file=out_post_dedup,
                        mesheal_bin=args.mesheal_bin,
                        timeout_sec=args.timeout or None,
                    )
                    attach_non_dedup_metrics(eval_res, post_dedup_res, defective)

            entry.setdefault("results", {})[result_key] = eval_res

            if defective:
                defective_count += 1
            else:
                healthy_count += 1

            if eval_res.get("repaired"):
                repaired_count += 1
            if eval_res.get("solid"):
                solid_count += 1
            if eval_res.get("broken"):
                broken_count += 1
            if eval_res.get("crashed"):
                crashed_ids.append(entry_id)
            if eval_res.get("timed_out"):
                timed_out_ids.append(entry_id)
            total_elapsed_ms += eval_res.get("elapsed_ms", 0.0)

            a_chg = eval_res.get("area_change_pct", 0.0)
            if eval_res.get("volume_sign_flipped"):
                v_chg = eval_res.get("volume_magnitude_change_pct", 0.0)
                flip_dir = "- -> +" if eval_res.get("volume_before", 0.0) < 0 else "+ -> -"
                vol_str = f"vol: {v_chg:+.2f}% [sign flipped: {flip_dir}]"
            else:
                v_chg = eval_res.get("volume_change_pct", 0.0)
                vol_str = f"vol: {v_chg:+.2f}%"
            if eval_res.get("status") == "failed":
                # No metrics came back: the process crashed or reported an error.
                kind = "CRASHED" if eval_res.get("crashed") else ("TIMED OUT" if eval_res.get("timed_out") else "ERROR")
                if not defective:
                    kind = f"HEALTHY ({kind.lower()})"
                err = str(eval_res.get("error", "")).splitlines()
                err_str = err[0] if err else "unknown error"
                print(f"{kind} after {eval_res.get('elapsed_ms', 0):.1f}ms: {err_str}")
                continue
            if not defective:
                status_str = "HEALTHY (broken)" if eval_res.get("broken") else "HEALTHY (unchanged)"
            elif eval_res.get("repaired"):
                status_str = "REPAIRED"
            elif eval_res.get("solid"):
                status_str = "SOLID (SHAPE LOST)"
            else:
                status_str = "FAILED"
            dupes_removed = eval_res.get("duplicate_faces_removed", 0)
            dedup_str = f", deduped: {dupes_removed} faces" if dupes_removed > 0 else ""
            non_dedup_str = ""
            if "non_dedup_volume_change_pct" in eval_res:
                nd_vol = eval_res["non_dedup_volume_change_pct"]
                nd_area = eval_res.get("non_dedup_area_change_pct", 0.0)
                nd_flip = " [sign flipped]" if eval_res.get("non_dedup_volume_sign_flipped") else ""
                non_dedup_str = f" | non-dedup vol: {nd_vol:+.2f}%{nd_flip}, area: {nd_area:+.2f}%"
            print(f"{status_str} in {eval_res.get('elapsed_ms', 0):.1f}ms ({vol_str}, area: {a_chg:+.2f}%{dedup_str}{non_dedup_str})")

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
        out_path = out_dir / f"mesheal-{args.method}-{len(files)}-{date_str}.json"

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print("\n" + "=" * 60)
    print("BENCHMARK SUMMARY")
    print("=" * 60)
    print(f"Total Models Tested:              {total_count}")
    print(f"Solid Output Meshes:              {solid_count} / {total_count} ({solid_count / max(1, total_count) * 100:.1f}%)")
    print(f"Repaired -- Originally Defective: {repaired_count} / {defective_count} ({repaired_count / max(1, defective_count) * 100:.1f}%)")
    print(f"Broken -- Originally Healthy:     {broken_count} / {healthy_count} ({broken_count / max(1, healthy_count) * 100:.1f}%)")
    print(f"Crashed:                          {len(crashed_ids)}" + (f" ({', '.join(crashed_ids)})" if crashed_ids else ""))
    print(f"Timed Out:                        {len(timed_out_ids)}" + (f" ({', '.join(timed_out_ids)})" if timed_out_ids else ""))
    print(f"Average Time / Mesh:              {total_elapsed_ms / max(1, total_count):.1f} ms")
    print(f"Results Written To:               {out_path}")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())

