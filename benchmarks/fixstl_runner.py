"""Runs the FixSTL web tool (benchmarks/fixstl_web.py) on files and measures them with Mesheal's own check.

One headless browser serves the whole run: fixstl_web.py runs as a child process in its own Python
environment (default ~/.cache/mesheal/fixstl-venv, see that file for the setup) and is sent one file
at a time. The time reported is the tool's own work, from choosing the file to the download being
offered (the check plus the repair), without loading the page.
"""

from __future__ import annotations

import atexit
import json
import os
import subprocess
from pathlib import Path
from typing import Optional, Union

from .blender_runner import combine
from .runner import RepairResult, diagnostics

SCRIPT = Path(__file__).resolve().parent / "fixstl_web.py"
DEFAULT_PYTHON = Path.home() / ".cache" / "mesheal" / "fixstl-venv" / "bin" / "python"
DEFAULT_BROWSERS = Path.home() / ".cache" / "mesheal" / "playwright-browsers"


class FixStlSession:
    """The browser process, started on first use and stopped at exit."""

    def __init__(self, python: Optional[Union[str, Path]] = None):
        self.python = Path(python) if python else DEFAULT_PYTHON
        self.proc: Optional[subprocess.Popen] = None

    def _start(self) -> None:
        if not self.python.is_file():
            raise FileNotFoundError(f"No Playwright environment at {self.python}: see benchmarks/fixstl_web.py")
        env = dict(os.environ)
        env.setdefault("PLAYWRIGHT_BROWSERS_PATH", str(DEFAULT_BROWSERS))
        self.proc = subprocess.Popen([str(self.python), "-I", str(SCRIPT)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env)
        atexit.register(self.close)

    def repair(self, src: Path, dst: Path, timeout_sec: float) -> dict:
        if self.proc is None or self.proc.poll() is not None:
            self._start()
        assert self.proc and self.proc.stdin and self.proc.stdout
        self.proc.stdin.write(json.dumps({"input": str(src), "output": str(dst), "timeout": timeout_sec}) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            self.close()
            return {"status": "error", "error": "The browser process stopped"}
        return json.loads(line)

    def close(self) -> None:
        if self.proc and self.proc.poll() is None:
            if self.proc.stdin:
                self.proc.stdin.close()
            try:
                self.proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None


_session: Optional[FixStlSession] = None


def repair_with_fixstl_web(
    input_file: Union[str, Path],
    output_file: Union[str, Path],
    mesheal_bin: Optional[Union[str, Path]] = None,
    python: Optional[Union[str, Path]] = None,
    timeout_sec: Optional[float] = None,
) -> RepairResult:
    """FixSTL's conservative repair of `input_file`, saved to `output_file`, with Mesheal's metrics before and after."""
    global _session
    if _session is None:
        _session = FixStlSession(python)
    action = "fixstl-web"
    src = Path(input_file).resolve()
    dst = Path(output_file).resolve()
    res = _session.repair(src, dst, min(timeout_sec or 300.0, 300.0))
    elapsed_ms = 1000.0 * float(res.get("elapsed_s", res.get("check_s", 0.0) + res.get("repair_s", 0.0)))
    if res.get("status") != "success" or not dst.is_file():
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=elapsed_ms,
                            error=res.get("error", "No output"), exit_code=1, timed_out=bool(res.get("timed_out")))
    before = diagnostics(src, mesheal_bin)
    after = diagnostics(dst, mesheal_bin)
    if not before.is_success or not after.is_success or before.metrics is None or after.metrics is None:
        return RepairResult(status="error", action=action, input_file=str(src), elapsed_ms=elapsed_ms,
                            error=f"Mesheal could not check the files: {before.error or after.error}", exit_code=1)
    metrics = combine(before.metrics, after.metrics)
    metrics.raw["fixstl"] = {k: v for k, v in res.items() if k != "status"}
    return RepairResult(status="success", action=action, input_file=str(src), output_file=str(dst),
                        elapsed_ms=elapsed_ms, metrics=metrics, exit_code=0)

