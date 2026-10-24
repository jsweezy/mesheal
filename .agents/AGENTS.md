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

## 2. Commit Changes to Git at the End of Each Task

- At the conclusion of every completed task (after all unit tests pass and documentation/`TODO.md`/`PLAN.md` updates are in place), stage the modified/created files and commit the changes to `git` with a clear, descriptive commit message.
