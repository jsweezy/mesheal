# Contributing

Bug reports, test meshes and pull requests are welcome. Please open an
[issue](https://github.com/jsweezy/mesheal/issues) first for larger changes.

## Build and test

```bash
git clone https://github.com/jsweezy/mesheal.git   # no --recursive: .docs/ is a private submodule
cd mesheal

# Release build: use it to run the app and the full test suite
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build -j

# Debug build: only for debugging (the full suite takes minutes in Debug)
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j
```

Each module has its own Catch2 test binary, for example
`./build/src/app/Test/mesheal_app_tests "[cli]"` runs only the tests tagged `[cli]`.

## Code layout

| Directory | Contents |
|---|---|
| `src/core/` | Default settings and constants |
| `src/mesh/` | The indexed mesh, units, transforms, the CGAL bridge |
| `src/io/` | STL reading (with welding) and writing, sample meshes, the config directory |
| `src/diagnostics/` | Mesh Health checks |
| `src/repair/` | Repairs, Fix All, remeshing and rebuilds |
| `src/app/` | `AppController` (all app logic, no GUI calls, unit-tested), the Polyscope / Dear ImGui GUI, and the command-line runner |
| `src/mockup/` | An early clickable prototype of the GUI layout (historical) |
| `src/python/` | The `_mesheal` Python module |
| `blender/` | The Blender extension |
| `benchmarks/` | FixSTL benchmark harness (Python) |
| `docs/` | This documentation site |
| `examples/` | Test meshes, each with its license |

## Guidelines

- **Tests.** New behaviour comes with Catch2 tests in the module's `Test/` folder. Build test meshes
  in memory (`SampleMeshes` or small vertex / face arrays) or use a file already in `examples/`.
  Tests must never read files outside the repository.
- **Undo.** Every operation that changes the mesh records an undo step in `AppController`, with a
  test of `undoLastModification()`.
- **Documentation.** Update the pages in `docs/` with any user-visible change. Keyboard and mouse
  changes go in both `docs/NAVIGATION.md` and the in-app *Navigation & Shortcuts* window
  (`PolyscopeGui::drawNavigationHelpWindow()`). New command-line flags go in `usageText()` in
  `src/app/CliRunner.cpp` (a test checks that every flag is listed) and in `docs/CLI.md`.
- **Test meshes.** Never add an STL from elsewhere without its license and attribution (see the
  `*_license.md` files in `examples/`). Prefer generated geometry when it shows the same problem.

## Documentation site

The site is built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) and published
to GitHub Pages on every push to `main`.

```bash
python3 -m pip install -r docs/requirements.txt
mkdocs serve            # live preview at http://127.0.0.1:8000
mkdocs build --strict   # what CI runs; fails on broken links
```

## Releases

See [`RELEASE_PROCESS.md`](https://github.com/jsweezy/mesheal/blob/main/RELEASE_PROCESS.md).

## License

By contributing you agree that your contribution is licensed under the
[GPL-3.0-or-later](https://github.com/jsweezy/mesheal/blob/main/LICENSE.md).
