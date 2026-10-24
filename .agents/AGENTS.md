# Agent Guidelines for Ogun (STL Repair Tool)

## 1. Keep Documentation, Punch List & In-App Help Up-to-Date

Whenever you add, modify, or remove features, workflows, UI controls, diagnostics, or viewport interactions, you **must** update all relevant documentation and in-app help before completing your task:

1. **Task Tracking (`TODO.md` & `PLAN.md`)**
   - Update `TODO.md` (marking completed punch-list items with `[x]` or adding newly identified `[ ]` items) and `PLAN.md` as work is finished so both accurately reflect the current state of the project.

2. **Navigation & Shortcuts Documentation (`docs/NAVIGATION.md`)**
   - Keep `docs/NAVIGATION.md` synchronized with any changes to 3D viewport mouse controls, keyboard shortcuts, camera framing behavior, orbit pivot mechanics, clipping plane behavior, or highlight sizing behavior.

3. **In-App Help Window ("3D Viewport Navigation & Shortcuts")**
   - Keep the in-app help window in `src/app/PolyscopeGui.cpp` (`PolyscopeGui::drawNavigationHelpWindow()`, titled `"3D Viewport Navigation & Shortcuts"`) and any related tooltip/status tips in `PolyscopeGui::drawHighlightAndNavigationSection()` synchronized with `docs/NAVIGATION.md` and the actual implementation in `PolyscopeGui` / `AppController`.
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
