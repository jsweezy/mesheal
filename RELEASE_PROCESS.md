# Release Process

Ogun versions are simple integers: `1`, `2`, `3`, ... Each release is tagged in git as `v#` (for example `v1`).

## Where the version lives

One place: the top-level `CMakeLists.txt`.

```cmake
project(
    Ogun
    VERSION 1            # <- release number
    ...
)
set(OGUN_VERSION_DATE "2026-10-28")  # <- release date, ISO YYYY-MM-DD
```

At configure time `cmake/Version.cmake` turns these (plus compiler, flags, platform, build type, build timestamp, and dependency versions) into `build/generated/ogun/Version.hpp`. Code uses it with `#include "ogun/Version.hpp"` (`ogun::kVersion`, `ogun::kVersionString`, `ogun::kVersionDate`, `ogun::kBuildTimestamp`, `ogun::kCompilerId`, `ogun::kCxxFlags`, `ogun::kDependencies`, ...). Never edit the generated header. The build timestamp is the configure time, so re-run `cmake` to refresh it.

Dependency versions come from `cmake/Dependencies.cmake`; when you change a dependency tag there, update the matching `ogun_record_dependency(...)` line.

## Steps to cut a release

1. Make sure `main` is clean and up to date.
2. In `CMakeLists.txt`, change `VERSION` in `project(...)` to the new integer and set `OGUN_VERSION_DATE` to today's date.
3. In `RELEASES.md`, add a new `## v<N> — <date>` section at the top with a short bullet summary of what changed since the last release.
4. Rebuild and run the tests: `cmake --build build -j 2 && ctest --test-dir build -j 4` (cmake re-configures automatically when `CMakeLists.txt` changes).
5. Commit: `git commit -am "Release v<N>"`.
6. Tag: `git tag -a v<N> -m "Ogun v<N>"`.
7. Push: `git push origin main --tags`.
