# Install

Mesheal runs on Windows, macOS and Linux. Download a ready-to-run version below, or
[build it from source](#build-from-source). To run only the command-line tool in a container,
use the [Docker image](DOCKER.md).

## Download

| System | Download |
|--------|----------|
| Windows 10 / 11 (64-bit) | [Mesheal-windows-x64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-windows-x64.zip) |
| macOS 15 or newer (Apple Silicon: M1 and later) | [Mesheal-macos-arm64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-macos-arm64.zip) |
| Linux (x86-64) | [Mesheal-linux-x86_64.AppImage](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-linux-x86_64.AppImage) |

Older versions and checksums are on the [releases page](https://github.com/jsweezy/mesheal/releases).

These builds are not code-signed, so the first time you open Mesheal your system warns that it
does not recognise the developer. This is expected; the one-time steps to open it are below.

=== "Windows"

    1. Unzip the whole `Mesheal` folder somewhere permanent (for example `Documents`). Keep the
       `.dll` files next to `mesheal.exe`.
    2. Double-click `mesheal.exe`. If Windows shows **Windows protected your PC**, click
       **More info**, then **Run anyway**.

    For the [command line](CLI.md), open PowerShell in that folder and run, for example,
    `.\mesheal.exe part.stl --fix-all`.

=== "macOS"

    1. Unzip the download and drag `Mesheal.app` into **Applications**.
    2. Open it. When macOS says it cannot verify the app, click **Done**, then open
       **System Settings → Privacy & Security**, scroll down and click **Open Anyway** next to
       the Mesheal message.

    Or, in Terminal: `xattr -dr com.apple.quarantine /Applications/Mesheal.app`

    The command-line tool is inside the app:
    `/Applications/Mesheal.app/Contents/MacOS/Mesheal part.stl --fix-all`. To type just
    `mesheal`, link it into your path:
    `ln -s /Applications/Mesheal.app/Contents/MacOS/Mesheal ~/.local/bin/mesheal`.

=== "Linux"

    ```bash
    chmod +x Mesheal-linux-x86_64.AppImage
    ./Mesheal-linux-x86_64.AppImage                    # desktop app
    ./Mesheal-linux-x86_64.AppImage part.stl --fix-all  # command line
    ```

    AppImages need FUSE 2. If it does not start, install it (Ubuntu 24.04:
    `sudo apt install libfuse2t64`; Ubuntu 22.04: `sudo apt install libfuse2`), or run it with
    `--appimage-extract-and-run` as the first argument.

## Build from source

Mesheal builds with CMake on macOS and Linux.

### 1. Prerequisites

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

### 2. Build

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

#### Command-line-only build

For servers and other machines without a display, build without the desktop app:

```bash
cmake -B build_release -DCMAKE_BUILD_TYPE=Release -DMESHEAL_BUILD_GUI=OFF
cmake --build build_release -j 2
```

This produces `build_release/src/app/mesheal` with every [command-line](CLI.md) action but no
window. It does not need Polyscope or the OpenGL / X11 packages, so on Ubuntu you can leave out
`libgl1-mesa-dev xorg-dev libwayland-dev libxkbcommon-dev`.

### 3. Check the build (optional)

```bash
ctest --test-dir build_release -j 4
```

### 4. Install the desktop app (optional)

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

Preferences and the recent-files list are saved in `~/.config/mesheal/` on macOS and Linux, and
in `%APPDATA%\mesheal\` on Windows (or `$XDG_CONFIG_HOME/mesheal/` when that variable is set).
Delete that folder to start from the defaults.
