# Agent Guidelines for Ogun (STL Repair Tool)

## 1. Keep Documentation, Punch List & In-App Help Up-to-Date

Whenever you add, modify, or remove features, workflows, UI controls, diagnostics, or viewport interactions, you **must** update all relevant documentation and in-app help before completing your task:

1. **Task Tracking (`TODO.md` & `PLAN.md`)**
   - Update `TODO.md` (marking completed punch-list items with `[x]` or adding newly identified `[ ]` items) and `PLAN.md` as work is finished so both accurately reflect the current state of the project.

2. **Navigation & Shortcuts Documentation (`docs/NAVIGATION.md`)**
   - Keep `docs/NAVIGATION.md` synchronized with any changes to 3D viewport mouse controls, keyboard shortcuts, camera framing behavior, orbit pivot mechanics, clipping plane behavior, or highlight sizing behavior.

3. **In-App Help Window ("3D Viewport Navigation & Shortcuts")**
   - Keep the in-app help window in `src/app/PolyscopeGuiShell.cpp` (`PolyscopeGui::drawNavigationHelpWindow()`, titled `"3D Viewport Navigation & Shortcuts"`) and the key tooltips in the Navigate panel (`PolyscopeGui::drawNavigatePanel()`, `src/app/PolyscopeGuiPanels.cpp`) synchronized with `docs/NAVIGATION.md` and the actual implementation in `PolyscopeGui` / `AppController`.
   - Any new or modified mouse interaction, keyboard shortcut, or selection/camera behavior must appear accurately in both `docs/NAVIGATION.md` and the `"3D Viewport Navigation & Shortcuts"` ImGui window.

4. **Project & Architecture Documentation (`README.md`, `VISION.md`, and `docs/*`)**
   - Update `README.md`, `VISION.md`, and any files under `docs/` whenever project status, build instructions, architectural decisions, diagnostics/repair capabilities, or user-facing workflows change.

## 2. All Mesh Modification Operations Must Support Undo (`Ctrl+Z` / `Cmd+Z`)

- **Every operation that modifies the mesh in memory** (including single-face repair commits, batch `"Fix All Thin / Sliver Faces"` repairs, explicit coordinate scaling `"Scale Mesh Coordinates"`, and any future mesh repair, welding, hole-patching, or normal-flipping operations) **must** record an undo snapshot on `AppController`'s undo history stack before mutating the mesh.
- Both a UI **"Undo"** button (displaying the name of the last action when available) and the standard **`Ctrl+Z` / `Cmd+Z`** keyboard shortcut in the 3D viewport must be supported to revert mesh modifications step-by-step and refresh active diagnostics/overlays.
- Unit tests in `src/app/Test/AppControllerTests.cpp` must verify `undoLastModification()` for every newly added mesh-modifying action.

## 3. Ask for User Review & Present a 1-Click "Accept & Commit" Button in Chat

- At the conclusion of every completed task (after all unit tests pass and documentation/`TODO.md`/`PLAN.md` updates are in place), **do not** stage or commit changes to `git` automatically.
- Leave all modified and newly created files uncommitted in the working tree so the user can easily inspect the diffs in VS Code's **Source Control** view.
- Summarize the completed work and call the `ask_question` tool to display a clickable 1-click button in chat:
  - **`(Recommended) Accept & Commit to git`**
  - **`Hold off on committing`**
- Once the user clicks **`(Recommended) Accept & Commit to git`**, stage the modified/created files and commit them to `git` with a clear, descriptive commit message.

## 4. C++ Coding Conventions

- **Prefer `std::array` over `std::vector` when the element count is fixed at compile time** (e.g., the 3 vertices of a triangle). This avoids heap allocations in hot paths.
- Use `reserve()` on a `std::vector` whenever the final size is known or can be estimated up front.
- Keep code comments short, and only comment complex code such as what an algorithm is doing.

## 5. Building

- Limit CPU usage when building on this Mac Neo with "-j 2".   Testing can use "-j 4".  
- There are two build trees:
  - **`build/`**: Debug build (`CMAKE_BUILD_TYPE=Debug`), for day-to-day development and tests.
  - **`build_release/`**: Release build (`CMAKE_BUILD_TYPE=Release`), for running the app on big meshes and timing performance.
- `build_release/` reuses the dependency sources already downloaded into `build/_deps/` (`FETCHCONTENT_SOURCE_DIR_<NAME>` for Catch2, Eigen, libigl, Manifold, Polyscope and portable-file-dialogs), so it needs no network access. `build/` must therefore stay in place.

- **Driving the GUI**: `cliclick` (Homebrew, `/opt/homebrew/bin/cliclick`) clicks and types in the running app, e.g. `cliclick c:400,300` (click), `dc:` (double-click), `t:"text"` (type), `kp:return` (key). Take screenshots with `screencapture -x <file>.png` and look at them. Screen points, not pixels: on a Retina display divide screenshot pixel coordinates by 2. cliclick needs Accessibility permission for the terminal running the agent (System Settings > Privacy & Security > Accessibility); without it, it warns "Accessibility privileges not enabled".

## 6. Constants and Default Settings Management

- **Do not hardcode magic decimal values or re-declare mathematical constants in individual files**:
  - Always use standard C++20 `<numbers>` (`std::numbers::pi`, `std::numbers::phi`, `std::numbers::sqrt2`, etc.) and `constexpr` arithmetic rather than manual floating-point approximations (e.g., use `kAlphaY = 1.0 / std::numbers::phi;`, `kAlphaZ = std::numbers::sqrt2 - 1.0;`).
- **File Organization**:
  - **Mathematical / Geometric Constants**: Place system-wide mathematical, trigonometric, and numerical constants in `src/core/Constants.hpp` under `namespace ogun`.
  - **Application Defaults & Tunable Settings**: Place viewport defaults, camera FOV bounds, diagnostic thresholds, healing tolerances, and vertex welding defaults in `src/core/Settings.hpp` under `namespace ogun::settings`.
- **Common Format & Style**:
  - Use `inline constexpr` at namespace scope.
  - Prefix constant names with `k` in camelCase (e.g., `kPi`, `kDegToRad`, `kDefaultHighlightPointRadius`, `kDefaultNeedleEdgeRatio`).
  - Place a clear, descriptive one-line comment beside every setting in `src/core/Settings.hpp` indicating its purpose, units, and geometric significance so settings are easy to find and modify.

