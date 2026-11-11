#!/usr/bin/env python3
"""Make Mesheal.app self-contained: copy every non-system library it uses into the bundle.

    packaging/macos/bundle-libs.py build_release/src/app/Mesheal.app

Each library the executable needs from outside macOS (Homebrew's GMP, MPFR, TBB, ...) and the
libraries those need in turn are copied into Contents/Frameworks. Every reference is rewritten to
@executable_path/../Frameworks/<name>, search paths (LC_RPATH) that point outside the bundle are
removed, and the bundle is re-signed ad hoc (install_name_tool invalidates signatures). Fails if
anything still points outside the bundle afterwards. Used by .github/workflows/release.yml.
"""

import os
import shutil
import subprocess
import sys

BUNDLE_PREFIX = "@executable_path/../Frameworks/"


def run(*args: str) -> str:
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def linked_libraries(path: str) -> list[str]:
    """Install names this file links against (its own id excluded)."""
    own_id = run("otool", "-D", path).splitlines()[1:]  # empty for executables
    lines = run("otool", "-L", path).splitlines()[1:]
    names = [line.strip().split(" (compatibility")[0] for line in lines if line.strip()]
    return [n for n in names if n not in own_id]


def rpaths(path: str) -> list[str]:
    out, lines = [], run("otool", "-l", path).splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "cmd LC_RPATH":
            # "cmdsize N" then "path <value> (offset N)"
            out.append(lines[i + 2].strip().split(" ", 1)[1].rsplit(" (offset", 1)[0])
    return out


def is_system(name: str) -> bool:
    return name.startswith(("/usr/lib/", "/System/")) or name.startswith(BUNDLE_PREFIX)


def resolve(name: str, referrer_original: str, referrer_rpaths: list[str]) -> str:
    """Real file for an install name, as seen from the referring file's original location."""
    origin = os.path.dirname(referrer_original)
    if name.startswith("@loader_path/"):
        return os.path.realpath(os.path.join(origin, name[len("@loader_path/"):]))
    if name.startswith("@rpath/"):
        rest = name[len("@rpath/"):]
        for rp in referrer_rpaths:
            base = rp.replace("@loader_path", origin).replace("@executable_path", origin)
            candidate = os.path.join(base, rest)
            if os.path.exists(candidate):
                return os.path.realpath(candidate)
        sys.exit(f"error: cannot resolve {name} from {referrer_original} (rpaths: {referrer_rpaths})")
    return os.path.realpath(name)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    app = sys.argv[1].rstrip("/")
    exe = os.path.join(app, "Contents", "MacOS", "Mesheal")
    frameworks = os.path.join(app, "Contents", "Frameworks")
    os.makedirs(frameworks, exist_ok=True)

    # (file inside the bundle, where it originally came from) still to process
    queue = [(exe, os.path.realpath(exe))]
    bundled: dict[str, str] = {}  # library file name -> original path
    processed = set()
    while queue:
        path, original = queue.pop()
        if path in processed:
            continue
        processed.add(path)
        own_rpaths = rpaths(path)
        for name in linked_libraries(path):
            if is_system(name):
                continue
            source = resolve(name, original, own_rpaths)
            file_name = os.path.basename(name)
            target = os.path.join(frameworks, file_name)
            if file_name not in bundled:
                print(f"  bundling {source} as {file_name}")
                shutil.copyfile(source, target)  # follows symlinks
                os.chmod(target, 0o755)
                run("install_name_tool", "-id", BUNDLE_PREFIX + file_name, target)
                bundled[file_name] = source
                queue.append((target, source))
            run("install_name_tool", "-change", name, BUNDLE_PREFIX + file_name, path)
        # Every reference now points into the bundle: search paths outside it are no longer needed,
        # and duplicates make dyld refuse to load the file
        seen = set()
        for rp in own_rpaths:
            if rp in seen or not rp.startswith(("@executable_path", "@loader_path")):
                run("install_name_tool", "-delete_rpath", rp, path)
            seen.add(rp)

    # Re-sign: libraries first, then the app
    for file_name in bundled:
        run("codesign", "--force", "--sign", "-", os.path.join(frameworks, file_name))
    run("codesign", "--force", "--deep", "--sign", "-", app)

    # Verify nothing points outside the bundle
    problems = []
    for path in [exe] + [os.path.join(frameworks, f) for f in bundled]:
        problems += [f"{path}: links {n}" for n in linked_libraries(path) if not is_system(n)]
        rps = rpaths(path)
        problems += [f"{path}: rpath {r}" for r in rps if not r.startswith(("@executable_path", "@loader_path"))]
        if len(rps) != len(set(rps)):
            problems.append(f"{path}: duplicate rpaths {rps}")
    if problems:
        sys.exit("error: the bundle is not self-contained:\n  " + "\n  ".join(problems))
    print(f"Bundled {len(bundled)} libraries into {frameworks}")


if __name__ == "__main__":
    main()
