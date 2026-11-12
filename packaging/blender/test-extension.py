#!/usr/bin/env python3
"""Install a packaged Mesheal extension into a throwaway Blender profile and run the smoke test.

    packaging/blender/test-extension.py /path/to/blender Mesheal-blender-linux-x64.zip

Uses `blender --command extension install-file`, as a user's "Install from Disk" does (it also
checks the manifest), then runs blender/tests/smoke_test.py against the installed copy. Your own
Blender settings and extensions are not touched. Used by .github/workflows/release.yml.
"""

import glob
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    blender, package = sys.argv[1], os.path.abspath(sys.argv[2])
    with tempfile.TemporaryDirectory() as profile:
        env = dict(os.environ, BLENDER_USER_RESOURCES=profile)
        subprocess.run([blender, "--version"], check=True, env=env)
        subprocess.run([blender, "--background", "--command", "extension", "install-file",
                        "--repo", "user_default", package], check=True, env=env)
        manifests = glob.glob(os.path.join(profile, "extensions", "*", "mesheal_tools", "blender_manifest.toml"))
        if len(manifests) != 1:
            sys.exit(f"error: expected one installed mesheal_tools under {profile}, found {manifests}")
        env["MESHEAL_EXTENSION_DIR"] = os.path.dirname(os.path.dirname(manifests[0]))
        env["MESHEAL_RELEASE_ZIP"] = "1"  # the smoke test then also requires TBB in the 5.1+ module
        result = subprocess.run([blender, "--background", "--factory-startup",
                                 "--python", os.path.join(ROOT, "blender", "tests", "smoke_test.py")],
                                env=env, capture_output=True, text=True)
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        # Blender exits 0 even when a --python script fails on some versions: check the marker too
        if result.returncode != 0 or "MESHEAL SMOKE TEST PASSED" not in result.stdout:
            sys.exit("error: the smoke test failed")


if __name__ == "__main__":
    main()
