#!/usr/bin/env python3
"""Adds new Thingi10K files to the fast benchmark set (benchmarks/specs/mesheal-fast.json).

Candidates are drawn at random, with a fixed seed, from Thingi10K's own metadata: files that are in no
list yet and are under a face limit. About a fifth of the files added are healthy, the rest broken
(by Thingi10K's metadata for the draw; by Mesheal's own check for the label that is stored). Each
candidate is downloaded into the benchmark cache and repaired with Fix All, and is added only if it
ran under the time limit. On battery power the limit is multiplied by --battery-factor, since the same
file runs slower there (3 s x 1.05 = 3.15 s on the machine the factor was measured on).

The list is only ever added to: files already in it are never removed or retimed.

    python3 -m benchmarks.add_fast_files --count 10 --seed 1
"""

from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import io
import json
import platform
import random
import subprocess
import sys
import tempfile
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from .cache import ModelCache
from .runner import MeshMetrics, find_mesheal_binary, repair

THINGI10K_REVISION = "v1.5.0"
METADATA_URL = f"https://huggingface.co/datasets/Thingi10K/Thingi10K/resolve/{THINGI10K_REVISION}/metadata"
CORRUPT_FILE_IDS = {49911, 74463, 286163, 77942}  # Listed as corrupt by the thingi10k package

SPECS = Path(__file__).resolve().parent / "specs"
DEFAULT_FAST_SPEC = SPECS / "mesheal-fast.json"


@dataclass
class Candidate:
    file_id: int
    thing_id: str
    faces: int
    healthy: bool  # By Thingi10K's metadata
    license: str
    author: str


def _truthy(value: str) -> bool:
    return value.strip().upper() in ("TRUE", "1")


def healthy_by_metadata(summary: Dict[str, str], geometry: Dict[str, str]) -> bool:
    """Thingi10K's own checks: closed, manifold, no duplicate or degenerate faces, no self-intersections."""
    flags = ("Closed", "Edge manifold", "Vertex manifold", "No duplicated faces", "No degenerate faces")
    if not all(_truthy(summary.get(f, "")) for f in flags):
        return False
    try:
        return int(geometry.get("num_self_intersections", "1") or 1) == 0
    except ValueError:
        return False


def load_candidates(summary_csv: str, geometry_csv: str, contextual_csv: str, max_faces: int, exclude: Set[str]) -> List[Candidate]:
    """Every Thingi10K file under `max_faces` faces that is not in `exclude` (source ids), in file id order."""
    geometry = {row["file_id"]: row for row in csv.DictReader(io.StringIO(geometry_csv))}
    authors = {row["Thing ID"]: row.get("Author", "") for row in csv.DictReader(io.StringIO(contextual_csv))}
    out: List[Candidate] = []
    for row in csv.DictReader(io.StringIO(summary_csv)):
        fid = row["ID"].strip()
        g = geometry.get(fid)
        if g is None or fid in exclude or int(fid) in CORRUPT_FILE_IDS:
            continue
        if not row.get("Link", "").lower().endswith(".stl"):
            continue  # The benchmark reads STL only
        try:
            faces = int(g["num_faces"])
        except (KeyError, ValueError):
            continue
        if faces <= 0 or faces > max_faces:
            continue
        thing = row["Thing ID"].strip()
        out.append(Candidate(int(fid), thing, faces, healthy_by_metadata(row, g), row.get("License", ""), authors.get(thing, "")))
    out.sort(key=lambda c: c.file_id)
    return out


def draw_order(candidates: List[Candidate], seed: int, healthy_share: float) -> List[Candidate]:
    """The candidates interleaved so that, taken in order, about `healthy_share` of them are healthy."""
    rng = random.Random(seed)
    healthy = [c for c in candidates if c.healthy]
    broken = [c for c in candidates if not c.healthy]
    rng.shuffle(healthy)
    rng.shuffle(broken)
    out: List[Candidate] = []
    taken_healthy = 0
    while healthy or broken:
        want_healthy = healthy and (not broken or taken_healthy < healthy_share * (len(out) + 1))
        out.append(healthy.pop() if want_healthy else broken.pop())
        taken_healthy += 1 if want_healthy else 0
    return out


def problems_of(m: MeshMetrics) -> List[str]:
    """FixSTL's problem labels, from Mesheal's check of the input."""
    problems = []
    if m.open_edges_before > 0 or not m.watertight_before:
        problems.append("holes")
    if m.non_manifold_edges_before > 0 or m.non_manifold_vertices_before > 0:
        problems.append("non-manifold")
    if m.flipped_faces_before > 0 or m.inverted_shells_before > 0:
        problems.append("flipped")
    if m.self_intersecting_faces_before > 0:
        problems.append("self-intersections")
    if m.duplicate_faces_before > 0 or m.degenerate_faces_before > 0:
        problems.append("duplicate-or-degenerate")
    return problems


def on_battery() -> Optional[bool]:
    """True on battery, False on mains power, None when it cannot be told (not macOS)."""
    if platform.system() != "Darwin":
        return None
    try:
        out = subprocess.run(["pmset", "-g", "batt"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    if "Battery Power" in out:
        return True
    if "AC Power" in out:
        return False
    return None


def _fetch(url: str) -> str:
    req = urllib.request.Request(url, headers={"User-Agent": "Mesheal-Benchmark-Tool/1.0"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read().decode("utf-8")


def metadata(cache_dir: Path) -> Tuple[str, str, str]:
    """Thingi10K's metadata tables, downloaded once into the cache."""
    folder = cache_dir / "thingi10k-metadata" / THINGI10K_REVISION
    folder.mkdir(parents=True, exist_ok=True)
    texts = []
    for name in ("input_summary", "geometry_data", "contextual_data"):
        path = folder / f"{name}.csv"
        if not path.is_file():
            path.write_text(_fetch(f"{METADATA_URL}/{name}.csv"), encoding="utf-8")
        texts.append(path.read_text(encoding="utf-8"))
    return texts[0], texts[1], texts[2]


def listed_ids(paths: Iterable[Path]) -> Set[str]:
    ids: Set[str] = set()
    for p in paths:
        with open(p, encoding="utf-8") as f:
            ids.update(str(e.get("source_id", "")) for e in json.load(f).get("files", []))
    return ids


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Add new Thingi10K files to the fast benchmark set")
    parser.add_argument("--count", type=int, default=10, help="Files to add (default: 10)")
    parser.add_argument("--seed", type=int, required=True, help="Random seed for the draw (record it to redraw the same files)")
    parser.add_argument("--healthy-share", type=float, default=0.2, help="Share of healthy files among those added (default: 0.2)")
    parser.add_argument("--max-seconds", type=float, default=3.0, help="Fix All time limit on mains power (default: 3)")
    parser.add_argument("--battery-factor", type=float, default=1.05,
                        help="How much slower this machine is on battery; the limit is multiplied by it (default: 1.05)")
    parser.add_argument("--max-faces", type=int, default=50000, help="Skip files with more faces than this (default: 50000)")
    parser.add_argument("--max-tries", type=int, default=0, help="Stop after this many candidates (default: 5 x count)")
    parser.add_argument("--spec", type=str, default=str(DEFAULT_FAST_SPEC), help="Fast set to add to")
    parser.add_argument("--cache-dir", type=str, default=None, help="Model cache directory")
    parser.add_argument("--mesheal-bin", type=str, default=None, help="Explicit path to the mesheal binary")
    parser.add_argument("--dry-run", action="store_true", help="Time the candidates but do not change the list")
    args = parser.parse_args(argv)

    spec_path = Path(args.spec)
    with open(spec_path, encoding="utf-8") as f:
        spec = json.load(f)
    bin_path = Path(args.mesheal_bin) if args.mesheal_bin else find_mesheal_binary()
    cache = ModelCache(Path(args.cache_dir) if args.cache_dir else None)

    battery = on_battery()
    factor = args.battery_factor if battery else 1.0
    limit = args.max_seconds * factor
    power = {True: "battery", False: "mains", None: "unknown"}[battery]
    print(f"Mesheal: {bin_path}")
    print(f"Power: {power}; time limit {limit:.2f} s ({args.max_seconds:g} s on mains power x {factor:g})")

    exclude = listed_ids(sorted(SPECS.glob("*.json")) + [spec_path])
    summary_csv, geometry_csv, contextual_csv = metadata(cache.cache_dir.parent)
    candidates = draw_order(load_candidates(summary_csv, geometry_csv, contextual_csv, args.max_faces, exclude),
                            args.seed, args.healthy_share)
    want_healthy = round(args.count * args.healthy_share)
    want_broken = args.count - want_healthy
    max_tries = args.max_tries or 5 * args.count
    print(f"{len(candidates)} candidates under {args.max_faces} faces; adding {want_broken} broken and {want_healthy} healthy\n")

    added: List[Dict[str, Any]] = []
    got = {True: 0, False: 0}  # defective -> count
    today = datetime.date.today().isoformat()
    tries = 0
    with tempfile.TemporaryDirectory() as tmp:
        for c in candidates:
            if got[True] >= want_broken and got[False] >= want_healthy:
                break
            if tries >= max_tries:
                print(f"Stopped after {tries} candidates.")
                break
            # Skip a candidate whose kind (by Thingi10K's metadata) is already full
            if (c.healthy and got[False] >= want_healthy) or (not c.healthy and got[True] >= want_broken):
                continue
            tries += 1
            sid = str(c.file_id)
            print(f"thingi-{sid} ({c.faces} faces)...", end=" ", flush=True)
            try:
                # No Thingiverse fallback: a thing's zip can hold several STL files
                path = cache.get_or_download_model(sid, None, None)
            except Exception as e:  # noqa: BLE001 - any download failure skips the file
                print(f"skipped (download failed: {e})")
                continue
            result = repair(input_file=path, method="fix-all", output_file=Path(tmp) / f"{sid}.stl",
                            mesheal_bin=str(bin_path), timeout_sec=max(60.0, 20 * limit))
            if not result.is_success or result.metrics is None:
                print(f"skipped (Fix All failed: {(result.error or 'no result').splitlines()[0]})")
                continue
            seconds = result.elapsed_ms / 1000.0
            m = result.metrics
            problems = problems_of(m)
            defective = bool(problems)
            kind = "broken" if defective else "healthy"
            if seconds > limit:
                print(f"skipped ({kind}, {seconds:.2f} s: over {limit:.2f} s)")
                continue
            if got[defective] >= (want_broken if defective else want_healthy):
                print(f"skipped ({kind}, {seconds:.2f} s: enough {kind} files; Thingi10K's metadata called it "
                      f"{'healthy' if c.healthy else 'broken'})")
                continue
            got[defective] += 1
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            added.append({
                "id": f"thingi-{sid}",
                "group": "thingi10k-added",
                "source_id": sid,
                "thing_url": f"https://www.thingiverse.com/thing:{c.thing_id}",
                "license": c.license,
                "author": c.author,
                "input_sha256": sha,
                "problems": problems,
                "defective": defective,
                "results": {},
                "fast_set": {
                    "added": today,
                    "source": f"thingi10k-{THINGI10K_REVISION}, seed {args.seed}",
                    "fix_all_seconds": round(seconds, 3),
                    "power": power,
                    "faces": m.faces_before,
                    "labelled_by": "mesheal",
                },
            })
            print(f"ADDED ({kind}{': ' + ', '.join(problems) if problems else ''}; {seconds:.2f} s)")

    print(f"\n{len(added)} files added ({got[True]} broken, {got[False]} healthy) after {tries} candidates.")
    if args.dry_run or not added:
        if args.dry_run:
            print("Dry run: the list was not changed.")
        return 0 if len(added) == args.count else 1
    spec["files"].extend(added)
    spec.setdefault("selection", {}).setdefault("sources", {})[f"thingi10k-{THINGI10K_REVISION}"] = (
        f"{METADATA_URL} (files labelled broken or healthy by Mesheal's own check; no FixSTL results)")
    with open(spec_path, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2)
        f.write("\n")
    print(f"{spec_path} now has {len(spec['files'])} files.")
    return 0 if len(added) == args.count else 1


if __name__ == "__main__":
    sys.exit(main())
