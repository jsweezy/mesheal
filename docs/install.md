# Install

Mesheal runs on Windows, macOS and Linux. Download a ready-to-run version below, or
[build it from source](#build-from-source). To run only the command-line tool in a container,
use the [Docker image](DOCKER.md).

## Download

| System | Download |
|--------|----------|
| Windows 10 / 11 (64-bit) | [Mesheal-windows-x64-setup.exe](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-windows-x64-setup.exe) (installer), or [Mesheal-windows-x64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-windows-x64.zip) to run without installing |
| macOS 15 or newer (Apple Silicon: M1 and later) | [Mesheal-macos-arm64.zip](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-macos-arm64.zip) |
| Linux (x86-64) | [Mesheal-linux-x86_64.AppImage](https://github.com/jsweezy/mesheal/releases/latest/download/Mesheal-linux-x86_64.AppImage) |

Older versions and checksums are on the [releases page](https://github.com/jsweezy/mesheal/releases).
For the Blender add-on, see [Blender extension](blender.md#install).

These builds are not code-signed, so the first time you open Mesheal your system warns that it
does not recognise the developer. This is expected; the one-time steps to open it are below.

=== "Windows"

    **Installer** (recommended):

    1. Run `Mesheal-windows-x64-setup.exe`. If Windows shows **Windows protected your PC**,
       click **More info**, then **Run anyway**.
    2. Follow the steps. Mesheal installs for your user only, so no administrator password is
       needed, and adds itself to the Start menu (tick the box for a desktop icon too).
    3. To open `.stl` files with Mesheal by double-clicking them: right-click an `.stl` file,
       choose **Open with → Choose another app**, pick **Mesheal** and click **Always**. Windows
       does not let an installer make this choice for you.

    Installing a newer version over an older one updates it. Remove Mesheal from
    **Settings → Apps → Installed apps**.

    **Zip** (no install): unzip the whole `Mesheal` folder somewhere permanent (for example
    `Documents`), keep the `.dll` files next to `mesheal.exe`, and double-click `mesheal.exe`.
    The `README.txt` inside explains how to pin it to Start and open `.stl` files with it.

    For the [command line](CLI.md), open PowerShell in the Mesheal folder and run, for example,
    `.\mesheal.exe part.stl --fix-all`. The installer puts Mesheal in
    `%LOCALAPPDATA%\Programs\Mesheal`.

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

Mesheal builds with CMake on macOS, Linux and Windows. To build only the command-line tool without
installing a compiler, use [Docker](#build-with-docker-command-line-only) on any of them.

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

=== "Windows"

    - [Visual Studio 2022 or newer](https://visualstudio.microsoft.com/downloads/) (Community is
      free) with the **Desktop development with C++** workload. It includes the compiler, CMake and
      Ninja.
    - [Git for Windows](https://git-scm.com/download/win).
    - [vcpkg](https://vcpkg.io), which builds CGAL, Boost, GMP and MPFR. In PowerShell:

    ```powershell
    git clone https://github.com/microsoft/vcpkg C:\vcpkg
    C:\vcpkg\bootstrap-vcpkg.bat -disableMetrics
    ```

The other dependencies (Eigen, libigl, Manifold, Polyscope, Dear ImGui, Catch2 and others) are
downloaded and built automatically by CMake on the first configure. You need a C++20 compiler and
CMake 3.20 or newer.

### 2. Build

=== "macOS / Linux"

    ```bash
    git clone https://github.com/jsweezy/mesheal.git
    cd mesheal
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
    ```

=== "Windows"

    Open **Developer PowerShell for VS** from the Start menu (it sets up the compiler), then:

    ```powershell
    git clone https://github.com/jsweezy/mesheal.git
    cd mesheal
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release `
      -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
      -DVCPKG_MANIFEST_DIR="$PWD/packaging/vcpkg" `
      -DVCPKG_OVERLAY_TRIPLETS="$PWD/packaging/vcpkg/triplets" `
      -DVCPKG_TARGET_TRIPLET=x64-windows-static-md-release `
      -DVCPKG_HOST_TRIPLET=x64-windows-static-md-release
    cmake --build build -j
    ```

    The first configure builds the vcpkg libraries and takes about an hour; vcpkg keeps them, so
    later configures are quick. These are the settings the release builds use: the libraries are
    linked into `mesheal.exe`, so it needs no extra DLLs.

    !!! tip "Faster Precision Isosurface (optional)"
        Without oneTBB, Precision Isosurface runs on one core. To use every core, download
        `oneapi-tbb-<version>-win.zip` from the [oneTBB releases](https://github.com/uxlfoundation/oneTBB/releases),
        unzip it, add `-DTBB_DIR=<oneTBB folder>/lib/cmake/tbb` to the first `cmake` command, and
        copy `tbb12.dll` and `tbbmalloc.dll` from its `redist\intel64\vc14` folder next to
        `mesheal.exe` (and the test programs, to run the tests).

!!! note
    Clone without `--recursive`. The repository has one private submodule (`.docs/`, internal
    design notes) that the build does not need.

The first build takes a while because it compiles the dependencies. `-j` compiles on every core.
Some files need over a gigabyte of memory each, so if the build runs out of memory, give it a
number, for example `-j 4`.

The executable is `build/src/app/mesheal` (`mesheal.exe` on Windows). On macOS the build also produces the app bundle
`build/src/app/Mesheal.app`. The build also produces `mesheal-cli`, a command-line-only
version of the same program that needs no graphics libraries.

#### Command-line-only build

For servers and other machines without a display, build without the desktop app:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DMESHEAL_BUILD_GUI=OFF
cmake --build build -j
```

This produces `build/src/app/mesheal` with every [command-line](CLI.md) action but no
window. It does not need Polyscope or the OpenGL / X11 packages, so on Ubuntu you can leave out
`libgl1-mesa-dev xorg-dev libwayland-dev libxkbcommon-dev`.

#### Build with Docker (command line only)

The repository's `Dockerfile` builds Mesheal on Ubuntu 24.04, runs the tests, and packages the
`mesheal` command-line tool in an image. You need only [Docker](https://www.docker.com/products/docker-desktop/),
on Windows, macOS or Linux:

```bash
git clone https://github.com/jsweezy/mesheal.git
cd mesheal
docker build -t mesheal .
docker run --rm -v "$PWD:/data" mesheal part.stl --diagnostics
```

The first build takes 15 to 30 minutes. See [Docker](DOCKER.md) for the Windows PowerShell and
Command Prompt versions of `docker run`, and for faster rebuilds while developing.

### 3. Check the build (optional)

```bash
ctest --test-dir build -j
```

CTest older than 3.29 (Ubuntu 24.04 has 3.28) needs a number after `-j`, for example `-j 4`.

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

=== "Windows"

    Run `build\src\app\mesheal.exe` directly, or install a release with the
    [installer](#download). To make an installer from your own build, see
    `packaging/windows/mesheal.iss` (Inno Setup) and [Desktop integration](DESKTOP_INTEGRATION.md#windows).

On macOS and Linux, make sure `~/.local/bin` is on your `PATH`. See [Desktop integration](DESKTOP_INTEGRATION.md) for
details and for making Mesheal the default app for STL files.

## Where Mesheal keeps its settings

Preferences and the recent-files list are saved in `~/.config/mesheal/` on macOS and Linux, and
in `%APPDATA%\mesheal\` on Windows (or `$XDG_CONFIG_HOME/mesheal/` when that variable is set).
Delete that folder to start from the defaults.
