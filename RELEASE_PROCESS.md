# Release Process

Mesheal versions are simple integers: `1`, `2`, `3`, ... Each release is tagged in git as `v#` (for example `v1`).

## Where the version lives

One place: the top-level `CMakeLists.txt`.

```cmake
project(
    Mesheal
    VERSION 1            # <- release number
    ...
)
set(MESHEAL_VERSION_DATE "2026-10-28")  # <- release date, ISO YYYY-MM-DD
```

At configure time `cmake/Version.cmake` turns these (plus compiler, flags, platform, build type, build timestamp, and dependency versions) into `build/generated/mesheal/Version.hpp`. Code uses it with `#include "mesheal/Version.hpp"` (`mesheal::kVersion`, `mesheal::kVersionString`, `mesheal::kVersionDate`, `mesheal::kBuildTimestamp`, `mesheal::kCompilerId`, `mesheal::kCxxFlags`, `mesheal::kDependencies`, ...). Never edit the generated header. The build timestamp is the configure time, so re-run `cmake` to refresh it.

Dependency versions come from `cmake/Dependencies.cmake`; when you change a dependency tag there, update the matching `mesheal_record_dependency(...)` line.

## Steps to cut a release

1. Make sure `main` is clean and up to date.
2. In `CMakeLists.txt`, change `VERSION` in `project(...)` to the new integer and set `MESHEAL_VERSION_DATE` to today's date.
3. In `RELEASES.md`, add a new `## v<N> — <date>` section at the top with a short bullet summary of what changed since the last release.
4. Rebuild and run the tests: `cmake --build build -j 2 && ctest --test-dir build -j 4` (cmake re-configures automatically when `CMakeLists.txt` changes).
5. Commit: `git commit -am "Release v<N>"`.
6. Tag: `git tag -a v<N> -m "Mesheal v<N>"`.
7. Push: `git push origin main --tags`.
8. Pushing the tag starts the **Release binaries** workflow (`.github/workflows/release.yml`). It builds
   and tests Mesheal on Linux, macOS and Windows and the Blender extension for each (installed and
   tested in Blender 4.2 to 5.2), then publishes a GitHub Release with
   `Mesheal-linux-x86_64.AppImage`, `Mesheal-macos-arm64.zip`, `Mesheal-windows-x64-setup.exe`,
   `Mesheal-windows-x64.zip`, `Mesheal-blender-linux-x64.zip`, `Mesheal-blender-macos-arm64.zip`,
   `Mesheal-blender-windows-x64.zip` and `SHA256SUMS.txt`, using this version's section of
   `RELEASES.md` as the notes. A full run takes about an hour and a half (longer if the vcpkg
   library caches have expired). Watch it under
   **Actions**; if a platform fails, nothing is published. Fix the problem, delete the tag
   (`git push origin :refs/tags/v<N>` and `git tag -d v<N>`), and tag again.

The download links in `docs/install.md` point at `releases/latest/download/<file>`, so they follow the
newest release automatically as long as the file names stay the same.

To test the release builds without publishing anything, run the workflow by hand
(**Actions > Release binaries > Run workflow**, or `gh workflow run release.yml -f platforms=windows`;
the platforms are `linux`, `macos`, `windows` and `blender`);
the packages are attached to that run as artifacts.

The builds are not code-signed. Signing can be added to the workflow later (an Apple Developer ID
with notarization for macOS; a code-signing certificate such as Azure Trusted Signing for Windows).
