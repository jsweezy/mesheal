# Desktop Integration & File Association Guide

Ogun is configured as a native desktop application on both **macOS** and **Linux**, allowing you to associate `.stl` files with Ogun so that double-clicking any STL in Finder or your Linux file browser opens it directly.

---

## macOS

### 1. App Bundle Details
- **Bundle Target**: `build_release/src/app/Ogun.app` (or `build/src/app/Ogun.app`)
- **Bundle Identifier**: `io.github.jsweezy.ogun`
- **Supported File Types**: `.stl`, `.STL` (`public.standard-tesselated-geometry-format`, `model/stl`, `application/sla`)
- **Application Icon**: High-resolution `assets/icons/ogun.icns` embedded directly in `Ogun.app/Contents/Resources`.

### 2. How to Associate STL Files with Ogun in Finder
1. In Finder, locate any `.stl` file.
2. Right-click (or `Ctrl+click`) the file and select **Get Info** (or press `Cmd+I`).
3. In the Info window, expand the **Open with:** section.
4. Select **Ogun** from the dropdown menu.
   - If Ogun is not in the list, choose **Other...**, navigate to your Ogun build directory (e.g. `build_release/src/app/Ogun.app`) or `/Applications/Ogun.app`, and click **Add**.
5. Click the **Change All...** button beneath the dropdown and confirm.

Now, double-clicking any `.stl` file anywhere in Finder will automatically open it in Ogun!

### 3. Multi-Instance Launching
- When you double-click an STL in Finder and Ogun is **not yet running**, Ogun launches and displays the mesh immediately.
- When an Ogun window is **already open with a model** and you double-click another STL in Finder, Ogun automatically spawns a **second independent Ogun instance** (`NSWorkspaceLaunchNewInstance`). Both models run in their own process and window side-by-side.
- If an Ogun window is open but empty (no mesh loaded), double-clicking an STL loads the model into that existing window.

### 4. Drag & Drop
- Dragging an `.stl` file from Finder or the Desktop directly into the Ogun 3D viewport immediately replaces the mesh in that specific window.

### 5. Command-Line Usage
You can still run Ogun from the terminal:
```bash
# Direct bundle launch (opens a new instance)
open -a build_release/src/app/Ogun.app path/to/model.stl

# Direct CLI launch (using the symlink created by the build)
./build_release/src/app/ogun path/to/model.stl
```

---

## Linux

### 1. Desktop Entry & MIME Specification
- **Desktop Entry**: `packaging/linux/ogun.desktop`
  - Registered MIME types: `model/stl;application/sla;model/x.stl-ascii;model/x.stl-binary;`
  - Execution command: `Exec=ogun %f`
- **MIME Definition**: `packaging/linux/ogun-mime.xml`
- **Icon**: `assets/icons/ogun.png`

### 2. Installing Desktop Integration (Current User)
To install Ogun desktop integration and register file associations for your user account without needing root/sudo:
```bash
./packaging/linux/install-user-desktop.sh
```
This script:
1. Links the `ogun` executable into `~/.local/bin/ogun`.
2. Installs the high-resolution PNG icon to `~/.local/share/icons/hicolor/512x512/apps/ogun.png`.
3. Installs the `.desktop` launcher to `~/.local/share/applications/ogun.desktop`.
4. Installs the shared-mime-info XML to `~/.local/share/mime/packages/ogun.xml`.
5. Updates MIME and desktop databases via `update-mime-database` and `update-desktop-database`.
6. Sets Ogun as the default handler for `model/stl` via `xdg-mime default ogun.desktop model/stl`.

### 3. Multi-Instance Behavior
On Linux, double-clicking multiple STL files in Nautilus (GNOME Files), Dolphin (KDE), or Thunar automatically launches each file in a separate independent Ogun instance.

### 4. Drag & Drop
Dragging an `.stl` file from your Linux file manager directly into an open Ogun 3D viewport immediately loads that model into the window.
