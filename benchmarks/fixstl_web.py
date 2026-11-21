"""Repairs STL files with the FixSTL web tool (https://fixstl.ai) in a headless browser.

FixSTL repairs files in the browser; the model never leaves this machine. This script drives the page
as a person would: choose the file, wait for the check, click "Apply conservative repair", and save the
download ("Download result" when it repaired the file, "Download original" when it kept the input).
Usage analytics requests (PostHog, Cloudflare Insights) and every non-GET request are blocked, so test
runs are not counted in FixSTL's statistics; FixSTL's privacy policy says the tool works the same.

Runs with Playwright, which lives in its own environment (not the project's Python):

    python3.14 -m venv ~/.cache/mesheal/fixstl-venv
    ~/.cache/mesheal/fixstl-venv/bin/pip install playwright==1.65.2
    PLAYWRIGHT_BROWSERS_PATH=~/.cache/mesheal/playwright-browsers \\
        ~/.cache/mesheal/fixstl-venv/bin/python -m playwright install chromium

As a server for the benchmark (one browser for the whole run), it reads one JSON request per line on
stdin, {"input": "...", "output": "...", "timeout": 120}, and answers each with one JSON line on stdout.
"""

import json
import re
import sys
import time

from playwright.sync_api import sync_playwright

URL = "https://fixstl.ai/"
REPAIR = "Apply conservative repair"
BLOCKED_HOSTS = ("celados.com", "cloudflareinsights.com")


def _blocked(request) -> bool:
    return request.method != "GET" or any(h in request.url for h in BLOCKED_HOSTS)


def _controls(page):
    return [t.strip().split("\n")[0] for t in page.locator("#repair button, #repair a").all_inner_texts() if t.strip()]


def _wait(page, done, timeout: float):
    """Polls the tool's controls until `done(controls)` is true; returns them, or None on time-out."""
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        controls = _controls(page)
        if done(controls):
            return controls
        page.wait_for_timeout(50)
    return None


class FixStlWeb:
    def __init__(self, playwright):
        self.browser = playwright.chromium.launch()
        self.context = self.browser.new_context(accept_downloads=True)
        self.engine = None

        def route(r):
            if _blocked(r.request):
                return r.abort()
            m = re.search(r"/assets/(stl_repair_engine[^/?]*\.wasm)", r.request.url)
            if m:
                self.engine = m.group(1)
            return r.continue_()

        self.context.route("**/*", route)

    def repair(self, src: str, dst: str, timeout: float = 120.0) -> dict:
        page = self.context.new_page()
        try:
            page.goto(URL, wait_until="networkidle", timeout=60000)
            t0 = time.perf_counter()
            page.locator("input[type=file]").first.set_input_files(src)
            checked = _wait(page, lambda c: any(REPAIR in x for x in c) or any("Download" in x for x in c), timeout)
            t1 = time.perf_counter()
            if checked is None:
                return {"status": "error", "error": "No check result: " + page.locator("#repair").inner_text()[:300],
                        "timed_out": True, "check_s": t1 - t0}
            if any(REPAIR in x for x in checked):
                page.get_by_role("button", name=REPAIR).click()
            done = _wait(page, lambda c: any("Download" in x for x in c), timeout)
            t2 = time.perf_counter()
            if done is None:
                return {"status": "error", "error": "No repair result: " + page.locator("#repair").inner_text()[:300],
                        "timed_out": True, "check_s": t1 - t0, "repair_s": t2 - t1}
            label = next(x for x in done if "Download" in x)
            with page.expect_download(timeout=60000) as d:
                page.get_by_text(label, exact=True).first.click()
            d.value.save_as(dst)
            text = page.locator("#repair").inner_text()
            return {
                "status": "success",
                "outcome": "repaired" if "result" in label.lower() else "kept-input",
                "download": label.rstrip(" ↓"),
                "check_s": t1 - t0,
                "repair_s": t2 - t1,
                "elapsed_s": t2 - t0,
                "message": next((ln for ln in text.split("\n") if ln.startswith(("Repaired", "No safe", "Automatic"))), ""),
                "engine": self.engine,
            }
        except Exception as e:  # noqa: BLE001 - reported to the caller as an error for this file
            return {"status": "error", "error": f"{type(e).__name__}: {e}"[:500]}
        finally:
            page.close()

    def close(self):
        self.browser.close()


def main() -> int:
    with sync_playwright() as p:
        tool = FixStlWeb(p)
        try:
            if len(sys.argv) == 3:
                print(json.dumps(tool.repair(sys.argv[1], sys.argv[2])), flush=True)
                return 0
            for line in sys.stdin:
                if not line.strip():
                    continue
                req = json.loads(line)
                print(json.dumps(tool.repair(req["input"], req["output"], float(req.get("timeout", 120)))), flush=True)
        finally:
            tool.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
