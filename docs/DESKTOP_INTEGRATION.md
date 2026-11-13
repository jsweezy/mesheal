# Desktop Integration & File Association Guide

Mesheal is configured as a native desktop application on **macOS**, **Linux** and **Windows**, allowing you to associate `.stl` files with Mesheal so that double-clicking any STL in Finder, Explorer or your Linux file browser opens it directly.

---

## macOS

### 1. App Bundle Details
- **Bundle Target**: `build_release/src/app/Mesheal.app` (or `build/src/app/Mesheal.app`)
- **Bundle Identifier**: `io.github.jsweezy.mesheal`
- **Supported File Types**: `.stl`, `.STL` (`public.standard-tesselated-geometry-format`, `model/stl`, `application/sla`)
- **Application Icon**: High-resolution `assets/icons/mesheal.icns` embedded directly in `Mesheal.app/Contents/Resources`.

### 2. Installing to `/Applications` & CLI Setup
To install the native `Mesheal.app` bundle into `/Applications` and register the `mesheal` command in your terminal:
```bash
./packaging/macos/install-app.sh
```
This script:
1. Locates the prebuilt `Mesheal.app` bundle in `build_release/src/app/Mesheal.app` (or runs `cmake --build build_release` to build it if needed).
2. Copies `Mesheal.app` into `/Applications` using Apple's `ditto`, safely preserving code signatures, resource forks, and icons.
3. Strips quarantine attributes if present.
4. Updates macOS `LaunchServices` via `lsregister` so Finder immediately detects the `.stl` file association and high-resolution application icon without needing a reboot.
5. Creates a convenient CLI symlink in `~/.local/bin/mesheal` (configurable via `--cli-dir`) so you can run `mesheal` directly from any terminal window.

Supported options:
- `./packaging/macos/install-app.sh --build`: Rebuilds Mesheal before installing.
- `./packaging/macos/install-app.sh --dest ~/Applications`: Installs to user-local Applications folder without requiring administrator privileges.
- `./packaging/macos/install-app.sh --uninstall`: Cleanly uninstalls `Mesheal.app` from `/Applications` and removes the CLI symlink.

### 3. How to Associate STL Files with Mesheal in Finder
1. In Finder, locate any `.stl` file.
2. Right-click (or `Ctrl+click`) the file and select **Get Info** (or press `Cmd+I`).
3. In the Info window, expand the **Open with:** section.
4. Select **Mesheal** from the dropdown menu.
   - If Mesheal is not in the list, choose **Other...**, navigate to `/Applications/Mesheal.app` (or your build directory `build_release/src/app/Mesheal.app`), and click **Add**.
5. Click the **Change All...** button beneath the dropdown and confirm.

Now, double-clicking any `.stl` file anywhere in Finder will automatically open it in Mesheal!

### 4. Multi-Instance Launching
- When you double-click an STL in Finder and Mesheal is **not yet running**, Mesheal launches and displays the mesh immediately.
- When an Mesheal window is **already open with a model** and you double-click another STL in Finder, Mesheal automatically spawns a **second independent Mesheal instance** (`NSWorkspaceLaunchNewInstance`). Both models run in their own process and window side-by-side.
- If an Mesheal window is open but empty (no mesh loaded), double-clicking an STL loads the model into that existing window.

### 5. Drag & Drop
- Dragging an `.stl` file from Finder or the Desktop directly into the Mesheal 3D viewport immediately replaces the mesh in that specific window.

### 6. Command-Line Usage
You can still run Mesheal from the terminal:
```bash
# Launch installed app bundle directly
open -a /Applications/Mesheal.app path/to/model.stl

# Launch via CLI symlink (~/.local/bin/mesheal)
mesheal path/to/model.stl

# Launch from build tree
./build_release/src/app/mesheal path/to/model.stl
```

---

## Linux

### 1. Desktop Entry & MIME Specification
- **Desktop Entry**: `packaging/linux/mesheal.desktop`
  - Registered MIME types: `model/stl;application/sla;model/x.stl-ascii;model/x.stl-binary;`
  - Execution command: `Exec=mesheal %f`
- **MIME Definition**: `packaging/linux/mesheal-mime.xml`
- **Icon**: `assets/icons/mesheal.png`

### 2. Installing Desktop Integration (Current User)
To install Mesheal desktop integration and register file associations for your user account without needing root/sudo:
```bash
./packaging/linux/install-user-desktop.sh
```
This script:
1. Links the `mesheal` executable into `~/.local/bin/mesheal`.
2. Installs the high-resolution PNG icon to `~/.local/share/icons/hicolor/512x512/apps/mesheal.png`.
3. Installs the `.desktop` launcher to `~/.local/share/applications/mesheal.desktop`.
4. Installs the shared-mime-info XML to `~/.local/share/mime/packages/mesheal.xml`.
5. Updates MIME and desktop databases via `update-mime-database` and `update-desktop-database`.
6. Sets Mesheal as the default handler for `model/stl` via `xdg-mime default mesheal.desktop model/stl`.

### 3. Multi-Instance Behavior
On Linux, double-clicking multiple STL files in Nautilus (GNOME Files), Dolphin (KDE), or Thunar automatically launches each file in a separate independent Mesheal instance.

### 4. Drag & Drop
Dragging an `.stl` file from your Linux file manager directly into an open Mesheal 3D viewport immediately loads that model into the window.


---

## Windows

### 1. Installer
`Mesheal-windows-x64-setup.exe` (built by `.github/workflows/release.yml` from `packaging/windows/mesheal.iss` with Inno Setup):
1. Installs for the current user into `%LOCALAPPDATA%\Programs\Mesheal`, with no administrator prompt. Choosing *Install for all users* installs into `Program Files` instead and asks for administrator rights.
2. Adds a Start menu entry and, if ticked, a desktop icon.
3. Registers Mesheal under **Open with** for `.stl` files (`Mesheal.stl` under `Software\Classes`). Windows 10 and 11 do not let an installer become the default app: right-click an `.stl` file, choose **Open with → Choose another app**, pick **Mesheal** and click **Always**.
4. Adds an uninstaller to **Settings → Apps → Installed apps**, which also removes the `.stl` registration. Running a newer installer updates an existing install.

Silent install for scripted setups: `Mesheal-windows-x64-setup.exe /VERYSILENT /CURRENTUSER` (or `/ALLUSERS`).

### 2. Icon and console
`mesheal.exe` carries the icon `assets/icons/mesheal.ico` (made from `mesheal.png` by `packaging/windows/make-ico.py`). It is a console program so the command line prints to PowerShell; opened from the Start menu, Explorer or an `.stl` file it closes the console window Windows gave it.
