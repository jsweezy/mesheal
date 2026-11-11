# Install

Mesheal is built from source with CMake. It runs on macOS and Linux.

## 1. Prerequisites

=== "macOS (Homebrew)"

    ```bash
    xcode-select --install   # compilers, if not already installed
    brew install cmake cgal boost
    ```

=== "Ubuntu / Debian"

    ```bash
    sudo apt-get update
    sudo apt-get install -y cmake build-essential libcgal-dev libboost-all-dev
    ```

The other dependencies (Eigen, libigl, Manifold, Polyscope, Dear ImGui, Catch2 and others) are
downloaded and built automatically by CMake on the first configure. You need a C++20 compiler and
CMake 3.20 or newer.

## 2. Build

```bash
git clone https://github.com/jsweezy/mesheal.git
cd mesheal
cmake -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release -j 2
```

!!! note
    Clone without `--recursive`. The repository has one private submodule (`.docs/`, internal
    design notes) that the build does not need.

The first build takes a while because it compiles the dependencies. Raise `-j` if you have the cores
and memory to spare.

The executable is `build_release/src/app/mesheal`. On macOS the build also produces the app bundle
`build_release/src/app/Mesheal.app`.

## 3. Check the build (optional)

```bash
ctest --test-dir build_release -j 4
```

## 4. Install the desktop app (optional)

=== "macOS"

    ```bash
    ./packaging/macos/install-app.sh
    ```

    Copies `Mesheal.app` into `/Applications`, registers it for `.stl` files, and links the
    `mesheal` command into `~/.local/bin`. Use `--dest ~/Applications` to install without
    administrator rights, and `--uninstall` to remove it.

=== "Linux"

    ```bash
    ./packaging/linux/install-user-desktop.sh
    ```

    Installs a desktop entry and the STL file association for your user, and links the `mesheal`
    command into `~/.local/bin`.

Make sure `~/.local/bin` is on your `PATH`. See [Desktop integration](DESKTOP_INTEGRATION.md) for
details and for making Mesheal the default app for STL files.

## Where Mesheal keeps its settings

Preferences and the recent-files list are saved in `~/.config/mesheal/` (or
`$XDG_CONFIG_HOME/mesheal/`). Delete that folder to start from the defaults.
