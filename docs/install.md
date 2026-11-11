# Install

Mesheal is built from source with CMake. It runs on macOS and Linux.

To run the command-line tool without installing a compiler (including on Windows), use the
[Docker image](DOCKER.md) instead.

## 1. Prerequisites

=== "macOS (Homebrew)"

    ```bash
    xcode-select --install   # compilers, if not already installed
    brew install cmake cgal boost
    ```

=== "Ubuntu 24.04 / Debian"

    ```bash
    sudo apt-get update
    sudo apt-get install -y cmake build-essential git libboost-dev libgmp-dev libmpfr-dev \
        libtbb-dev libgl1-mesa-dev xorg-dev libwayland-dev libxkbcommon-dev
    ```

    Mesheal needs CGAL 6.2 or newer. Ubuntu's `libcgal-dev` package is older, so CMake
    downloads CGAL 6.2.1 automatically on the first configure. CGAL is header-only, so this adds
    no compile time.

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
`build_release/src/app/Mesheal.app`. The build also produces `mesheal-cli`, a command-line-only
version of the same program that needs no graphics libraries.

### Command-line-only build

For servers and other machines without a display, build without the desktop app:

```bash
cmake -B build_release -DCMAKE_BUILD_TYPE=Release -DMESHEAL_BUILD_GUI=OFF
cmake --build build_release -j 2
```

This produces `build_release/src/app/mesheal` with every [command-line](CLI.md) action but no
window. It does not need Polyscope or the OpenGL / X11 packages, so on Ubuntu you can leave out
`libgl1-mesa-dev xorg-dev libwayland-dev libxkbcommon-dev`.

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
