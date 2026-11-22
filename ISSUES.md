# Known Issues

Problems we know about but have not fixed at the source: workarounds in place, behaviour we chose to
leave, and limits of the tools around Mesheal. Each entry says what happens, why, what we do about it,
and when the workaround can go. Close an entry by moving it to **Resolved** with the date and the fix.

## Open

### Windows builds need MSVC toolset 14.51: 14.52 breaks CGAL's self-intersection repair

- **Found:** 2026-11-21, when the v6 release build failed on Windows.
- **What happens:** built with MSVC 19.52 (toolset 14.52), CGAL 6.2.1's
  `Polygon_mesh_processing::experimental::remove_self_intersections` crashes (access violation) in its
  smoothing step, `remove_self_intersections_with_smoothing`, at `repair_self_intersections.h:534`: the
  first `is_degenerate_triangle_face` on the local copy of the patch reads an invalid face. Mesheal's
  CGAL Local Repair calls it, so six Windows tests crash (island repairs, Fix All on samples, the Island
  Inspector). The same code built with MSVC 19.51, Apple Clang and GCC works; the v5 Windows build used 19.51.
- **Why it looked like our change:** GitHub's `windows-latest` image moved from 19.51 to 19.52 after v5,
  so every Windows build since then, including test builds of the older commits, used the new compiler.
  Compiling out the exact union, turning CGAL's Delaunay hole search back on, an 8 MB stack and an
  unsorted crossing list all crashed the same way.
- **Workaround:** the release workflow installs the 14.51 toolset with the Visual Studio installer
  (`Microsoft.VisualStudio.Component.VC.14.51.x86.x64`) and builds with it, and the vcpkg triplet
  (`packaging/vcpkg/triplets/x64-windows-static-md-release.cmake`) sets `VCPKG_PLATFORM_TOOLSET_VERSION`
  to 14.51 so the libraries match. Windows builds take a few minutes longer, and results stay the same on
  every platform.
- **To remove it:** when a newer MSVC or CGAL builds it correctly, drop the install step and the
  `toolset` input in `.github/workflows/release.yml` and the triplet line, then run the Windows build.
  Microsoft will eventually stop offering 14.51, so this has to happen before then.
- **To do:** report it to CGAL (and Microsoft if it turns out to be a compiler bug) with the stack trace.

### A cap where a model touches its cut plane reloads with non-manifold edges

- **Found:** 2026-11-19, on `thingi-117959` (a model cut at z = 0 whose valleys reach down to the cut).
- **What happens:** Cap Cut Plane gives the cap its own copies of the vertices where the model only touches
  the plane along its own edges, so the repaired mesh is closed and manifold in memory. STL stores no
  vertex sharing, so loading the saved file merges the copies again: those edges then have four faces
  (2 non-manifold edges on 117959).
- **Decision:** left as it is (2026-11-21). The solid is correct and every ray still crosses it an even
  number of times; most slicers accept an edge where two parts touch. The alternative is moving those
  vertices about a micron off the plane.

### Benchmark: Mesheal's shape check does not compare shell counts

- **What happens:** FixSTL's *shape kept* test also needs the same number of shells; Mesheal's evaluator
  (`benchmarks/evaluator.py`) checks only area and volume. Tools that delete shells, MeshFix above all
  (it keeps only the largest), are judged differently: on the 137 fast-set files with published results
  MeshFix's totals match FixSTL's, but 20 per-file verdicts differ.

### Benchmark: the FixSTL web tool did not take `thingi-56117`

- **Found:** 2026-11-21, in the 400-file fast-set run.
- **What happens:** after the script chose the file, the page still showed "Drop STL or 3MF files here"
  after 300 s, so the file counts as not repaired. Not yet investigated (the page may reject the file
  without a message, or the script may need to wait for something).

## Resolved

None yet.
