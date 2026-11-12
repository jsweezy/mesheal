#!/usr/bin/env python3
"""Package the Blender extension for one platform: blender/mesheal_tools plus compiled modules.

    packaging/blender/build-extension.py --platform linux-x64 \\
        --licenses build/vcpkg_installed/x64-linux-static-release/share \\
        --out Mesheal-blender-linux-x64.zip \\
        build-py311/src/python/_mesheal.cpython-311-x86_64-linux-gnu.so \\
        build-py313/src/python/_mesheal.cpython-313-x86_64-linux-gnu.so

One module per Python version goes next to the add-on's Python files; Python picks the one whose
name matches its own version (Blender 4.2-5.0 use Python 3.11, 5.1 and later 3.13). Each module
must depend only on the operating system and, for Python 3.13, Blender's own oneTBB: GMP and MPFR
are linked in statically, so their licenses are added under licenses/. The manifest gets the platform, and the zip has the manifest at its
root, as `blender --command extension build` makes it. Used by .github/workflows/release.yml.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SOURCE = os.path.join(ROOT, "blender", "mesheal_tools")
PLATFORMS = ("windows-x64", "macos-arm64", "linux-x64")
# The Blender 5.1+ module uses the oneTBB that Blender ships and has already loaded; only these
# names may refer to it (macOS: found through @executable_path/../Resources/lib, i.e. Blender.app)
BLENDER_TBB = {
    "linux-x64": ("libtbb.so.12", "libtbbmalloc.so.2"),
    "macos-arm64": ("@rpath/libtbb.dylib", "@rpath/libtbbmalloc.dylib"),
    "windows-x64": ("tbb12.dll", "tbbmalloc.dll"),
}
STATIC_LIBRARIES = ("gmp", "mpfr")

LICENSE_README = """\
The Mesheal module (_mesheal) includes GMP and MPFR, linked statically. Both are licensed under the
GNU LGPL v3; their license texts are in this folder.

Mesheal itself is licensed under the GNU GPL v3 (LICENSE.md). Its complete source code, with the
build recipe, is at https://github.com/jsweezy/mesheal, so you can rebuild the module against your
own GMP or MPFR.
"""


def run(*args: str) -> str:
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def dependencies(platform: str, module: str) -> list[str]:
    """Libraries the module loads that are not part of the operating system."""
    if platform == "linux-x64":
        needed = [line.split("[", 1)[1].rstrip("]") for line in run("readelf", "-d", module).splitlines()
                  if "(NEEDED)" in line]
        system = ("libc.so", "libm.so", "libdl.so", "libpthread.so", "librt.so", "ld-linux",
                  "libstdc++.so", "libgcc_s.so")
        return [n for n in needed if not n.startswith(system)]
    if platform == "macos-arm64":
        lines = run("otool", "-L", module).splitlines()[1:]
        names = [line.strip().split(" (compatibility")[0] for line in lines if line.strip()]
        return [n for n in names if not n.startswith(("/usr/lib/", "/System/"))]
    # Windows: everything must come with Windows (System32) or be Blender's Python DLL
    lines = run("dumpbin", "/dependents", module).splitlines()
    names = [line.strip() for line in lines if line.strip().lower().endswith(".dll")]
    system32 = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32")
    return [n for n in names
            if not n.lower().startswith(("python3", "api-ms-win-"))
            and not os.path.exists(os.path.join(system32, n))]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--platform", required=True, choices=PLATFORMS)
    parser.add_argument("--licenses", required=True,
                        help="vcpkg's share folder (vcpkg_installed/<triplet>/share) with <lib>/copyright")
    parser.add_argument("--out", required=True, help="zip file to write")
    parser.add_argument("modules", nargs="+", help="compiled _mesheal modules, one per Python version")
    args = parser.parse_args()

    problems = []
    for module in args.modules:
        if not os.path.basename(module).startswith("_mesheal."):
            problems.append(f"{module}: not a _mesheal module")
            continue
        problems += [f"{module}: needs {dep}" for dep in dependencies(args.platform, module)
                     if dep.lower() not in [n.lower() for n in BLENDER_TBB[args.platform]]]
    if problems:
        sys.exit("error: the modules are not self-contained:\n  " + "\n  ".join(problems))

    with tempfile.TemporaryDirectory() as tmp:
        stage = os.path.join(tmp, "mesheal_tools")
        shutil.copytree(SOURCE, stage, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "_mesheal.*"))
        for module in args.modules:
            shutil.copy2(module, stage)

        licenses = os.path.join(stage, "licenses")
        os.makedirs(licenses)
        shutil.copy2(os.path.join(ROOT, "LICENSE.md"), licenses)
        for lib in STATIC_LIBRARIES:
            copyright_file = os.path.join(args.licenses, lib, "copyright")
            if not os.path.isfile(copyright_file):
                sys.exit(f"error: missing license file {copyright_file}")
            shutil.copy2(copyright_file, os.path.join(licenses, f"{lib}-copyright.txt"))
        with open(os.path.join(licenses, "README.txt"), "w", newline="\n") as f:
            f.write(LICENSE_README)

        # The manifest lists the one platform this zip is for
        manifest_path = os.path.join(stage, "blender_manifest.toml")
        with open(manifest_path) as f:
            manifest = f.read()
        if "\nplatforms" in manifest or "\n[build]" not in manifest:
            sys.exit("error: blender_manifest.toml layout changed; update build-extension.py")
        manifest = manifest.replace("\n[build]", f'\nplatforms = ["{args.platform}"]\n\n[build]', 1)
        with open(manifest_path, "w", newline="\n") as f:
            f.write(manifest)

        if os.path.exists(args.out):
            os.remove(args.out)
        with zipfile.ZipFile(args.out, "w", zipfile.ZIP_DEFLATED) as zf:
            for folder, dirs, files in os.walk(stage):
                dirs.sort()
                for name in sorted(files):
                    path = os.path.join(folder, name)
                    zf.write(path, os.path.relpath(path, stage).replace(os.sep, "/"))
        with zipfile.ZipFile(args.out) as zf:
            print(f"Wrote {args.out}:")
            for info in zf.infolist():
                print(f"  {info.file_size:>10}  {info.filename}")


if __name__ == "__main__":
    main()
