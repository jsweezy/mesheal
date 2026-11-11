# Agent instructions

## Communication & Answering Questions

- **Always answer user questions directly and immediately**: When the user asks a question, stop other activities, do not trigger new build or test jobs, and provide a clear, direct answer first before proceeding with any code edits or commands.

## Commits

- If the private `.docs/` submodule is checked out, follow `.docs/COMMIT_RULES.md` for the commit identity.
- Never add `Co-Authored-By: Claude ...` (or any Claude / AI attribution) trailers to commit messages or pull request descriptions.

## Documentation

- User documentation lives in `docs/` and is published with MkDocs Material (`mkdocs.yml`). Run `mkdocs build --strict` after editing it.
- Internal design notes (vision, roadmap, UI spec, punch list) live in the private `.docs/` submodule. Update them when you have it; never copy them into the public tree.

## Building & Testing

- Limit CPU usage when building with `-j 2`. Testing can use `-j 4`.
- Two build trees exist:
  - **`build/`**: Debug build (`CMAKE_BUILD_TYPE=Debug`) for targeted unit tests and debugging.
  - **`build_release/`**: Release build (`CMAKE_BUILD_TYPE=Release`) for running the app on large meshes, performance profiling, and running full test suites.
- **Always run tests for the Release build (`build_release/`) unless actively debugging**:
  - In `build_release/`, optimized `-O3` binaries run the entire 465-test suite in ~13–20 seconds (`ctest --test-dir build_release -j 4` or `./build_release/src/<path>/mesheal_*_tests`).
  - Running tests in `build/` (Debug) takes 5+ minutes due to unoptimized execution of heavy mesh operations (e.g. Turkey 259k faces, bunny). Only run tests in `build/` when actively debugging an issue that requires debug symbols or unoptimized assertions.
- Build release with: `cmake --build build_release -j 2`
- When running tests, establish a baseline timing, target specific test tags/names during iteration when helpful (e.g. `mesheal_app_tests "[app][weld]"`), and use an explicit timeout with a buffer to ensure tests do not hang in an infinite loop.

## Test Files and STL Assets

- **NEVER reference files outside the repository in tests**: Tests must NEVER reference or probe paths in `~/Downloads`, `~/Desktop`, `/tmp`, or any other directory outside the repository.
- **Prefer self-contained in-memory geometry**: Tests should construct synthetic meshes in memory (using `SampleMeshes`, parametric generators, or local vertex/face matrices) or use existing files already committed under `examples/`.
- **Licensing for new STL assets**: Never add an external STL file to the repository without copying in its license and attribution. Avoid adding external STLs if the behavior can be verified with synthetic geometry.
