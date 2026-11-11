# Contributing to Mesheal

Thanks for your interest. Build instructions, code layout, test and documentation guidelines are in
[docs/contributing.md](docs/contributing.md), also published at
<https://jsweezy.github.io/mesheal/contributing/>.

Short version:

- Clone **without** `--recursive`: `.docs/` is a private submodule of internal design notes that the
  build does not need.
- Build with `cmake -B build_release -DCMAKE_BUILD_TYPE=Release && cmake --build build_release -j 2`
  and run `ctest --test-dir build_release -j 4` before sending a pull request.
- Add Catch2 tests for new behaviour, and update `docs/` for user-visible changes.
