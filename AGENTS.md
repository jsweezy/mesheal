# Agent instructions

## Commits

- ALWAYS make commits as jsweezy (`Jeremy Sweezy <19966001+jsweezy@users.noreply.github.com>`). NEVER as Claude.
- Never add `Co-Authored-By: Claude ...` (or any Claude / AI attribution) trailers to commit messages or pull request descriptions.
- Before committing, confirm `git config user.email` is the noreply address above.

## Building & Testing

- Limit CPU usage when building with `-j 2`. Testing can use `-j 4`.
- Two build trees exist:
  - **`build/`**: Debug build (`CMAKE_BUILD_TYPE=Debug`) for targeted unit tests and debugging.
  - **`build_release/`**: Release build (`CMAKE_BUILD_TYPE=Release`) for running the app on large meshes, performance profiling, and running full test suites.
- Build release with: `cmake --build build_release -j 2`
- Running the full 460-test suite in `build/` (Debug) takes 5+ minutes due to unoptimized execution of heavy mesh operations (e.g. Turkey 259k faces, bunny). In `build_release/`, optimized `-O3` binaries run the entire suite in ~20–30 seconds.
- When running tests, establish a baseline timing, target specific test tags/names during iteration (e.g. `ogun_app_tests "[app][weld]"`), and use an explicit timeout with a buffer to ensure tests do not hang in an infinite loop.
