#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>
#include <polyscope/polyscope.h>
#include <polyscope/screenshot.h>
#include <polyscope/view.h>
#include <portable-file-dialogs.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <string>
#include <utility>

namespace ogun {

using gui::modShortcut;
using gui::tooltip;
using gui::withThousands;

namespace {

constexpr ImGuiWindowFlags kDockedFlags = ImGuiWindowFlags_NoMove;

/// Menu item for a feature planned in a later phase: drawn disabled with a tooltip.
void plannedMenuItem(const char* label, const char* when) {
    ImGui::MenuItem(label, nullptr, false, false);
    tooltip(when);
}

} // namespace

// -----------------------------------------------------------------------------
// Docked layout
// -----------------------------------------------------------------------------

void PolyscopeGui::resetLayout() {
    showStructures_ = false;
    showShells_ = true;
    rightColumnWidth_ = settings::kRightColumnDefaultPx;
    layoutResetPending_ = true;
}

void PolyscopeGui::drawLeftColumn(float top, float bottom, float rightColumnLeft) {
    const float width = settings::kLeftColumnWidthPx;
    const float margin = settings::kPanelMarginPx;

    // Narrow windows: collapse the left column first so the 3D view keeps at least kMinViewportWidthPx
    const bool narrow = rightColumnLeft - (width + 2.0f * margin) < settings::kMinViewportWidthPx;
    const bool collapseNow = narrow && !leftColumnAutoCollapsed_;
    leftColumnAutoCollapsed_ = narrow;

    float y = top;
    const auto stackedPanel = [&](const char* title, bool* open, bool startCollapsed, auto&& body) {
        if ((open && !*open) || y >= bottom) {
            return;
        }
        ImGui::SetNextWindowPos(ImVec2(margin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0.0f), ImVec2(width, bottom - y));
        if (collapseNow) {
            ImGui::SetNextWindowCollapsed(true, ImGuiCond_Always);
        } else if (layoutResetPending_) {
            ImGui::SetNextWindowCollapsed(startCollapsed, ImGuiCond_Always);
        } else {
            ImGui::SetNextWindowCollapsed(startCollapsed, ImGuiCond_FirstUseEver);
        }
        const bool expanded = ImGui::Begin(title, open, kDockedFlags | ImGuiWindowFlags_AlwaysAutoResize);
        if (open == &showStructures_) {
            structuresCollapsed_ = ImGui::IsWindowCollapsed();
        }
        if (expanded) {
            body();
        }
        y += ImGui::GetWindowHeight() + margin;
        ImGui::End();
    };

    stackedPanel("Display", nullptr, false, [this] { drawDisplayPanel(); });
    stackedPanel("Navigate", nullptr, false, [this] { drawNavigatePanel(); });

    // Shells (multi-shell meshes only) fills the column down to Structures, a title bar while collapsed
    const ShellBrowser& shells = controller_.shells();
    if (showShells_ && shells.multiShell() && y < bottom) {
        const bool structures = showStructures_;
        const float structuresHeight = !structures ? 0.0f
                                       : structuresCollapsed_ ? ImGui::GetFrameHeight()
                                                              : (bottom - y) * settings::kStructuresShareOfShellsColumn;
        const float height = std::max(bottom - y - (structures ? structuresHeight + margin : 0.0f),
                                      settings::kShellsPanelMinHeightPx);
        ImGui::SetNextWindowPos(ImVec2(margin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
        if (collapseNow) {
            ImGui::SetNextWindowCollapsed(true, ImGuiCond_Always);
        } else if (layoutResetPending_) {
            ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
        }
        const std::string title = "Shells (" + gui::withThousands(shells.size()) + ")###Shells";
        if (ImGui::Begin(title.c_str(), &showShells_, kDockedFlags | ImGuiWindowFlags_NoResize)) {
            drawShellsPanel();
        }
        y += ImGui::GetWindowHeight() + margin;
        ImGui::End();
    }

    stackedPanel("Structures (advanced)", &showStructures_, true, [this] { drawStructuresPanel(); });
}

void PolyscopeGui::drawRightColumn(float top, float bottom) {
    const float displayWidth = ImGui::GetIO().DisplaySize.x;
    const float margin = settings::kPanelMarginPx;
    rightColumnWidth_ = std::clamp(rightColumnWidth_, settings::kRightColumnMinPx, settings::kRightColumnMaxPx);
    const float width = rightColumnWidth_;
    const float x = displayWidth - width - margin;
    float y = top;

    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0.0f), ImVec2(width, (bottom - top) * 0.6f));
    if (layoutResetPending_) {
        ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
    }
    if (ImGui::Begin("Mesh Health", nullptr, kDockedFlags | ImGuiWindowFlags_AlwaysAutoResize)) {
        drawHealthPanel();
    }
    y += ImGui::GetWindowHeight() + margin;
    ImGui::End();

    // Progressive disclosure: the Tool slot exists only once a row or a Mesh operation is picked
    const ToolSlot slot = controller_.activeTool();
    if (slot == ToolSlot::None || y >= bottom - 60.0f) {
        return;
    }
    std::string title = slot == ToolSlot::UnitsAndScale            ? std::string("Operation: Units & Scale")
                        : slot == ToolSlot::RepairAllIntersections ? std::string("Operation: Repair All Intersections")
                        : slot == ToolSlot::RepairAllHoles         ? std::string("Operation: Repair All Holes")
                        : slot == ToolSlot::AlphaWrap              ? std::string("Operation: Alpha Wrap")
                        : slot == ToolSlot::Decimate               ? std::string("Operation: Decimate")
                        : slot == ToolSlot::MergeVertices          ? std::string("Operation: Merge Vertices")
                        : slot == ToolSlot::MergeShells            ? std::string("Operation: Merge Shells")
                        : std::string("Tool: ") + categoryInfo(*controller_.selectedCategory()).name;
    title += "###ToolSlot";

    // Width is draggable from the left edge; height always fills the column
    const float height = bottom - y;
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height), layoutResetPending_ ? ImGuiCond_Always : ImGuiCond_Appearing);
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(settings::kRightColumnMinPx, height),
        ImVec2(settings::kRightColumnMaxPx, height));
    bool open = true;
    if (ImGui::Begin(title.c_str(), &open, kDockedFlags | ImGuiWindowFlags_NoCollapse)) {
        if (slot == ToolSlot::UnitsAndScale) {
            drawUnitsAndScaleOperator();
        } else if (controller_.repairAllOpen()) {
            drawRepairAllOperator();
        } else if (slot == ToolSlot::MergeShells) {
            drawMergeShellsOperator();
        } else {
            ImGui::PushTextWrapPos(0.0f); // long lines wrap at the panel edge
            drawToolPanel();
            ImGui::PopTextWrapPos();
        }
    }
    rightColumnWidth_ = ImGui::GetWindowWidth();
    ImGui::End();
    if (!open) {
        controller_.closeTool();
        requestSync();
    }
}

// -----------------------------------------------------------------------------
// Menu bar
// -----------------------------------------------------------------------------

float PolyscopeGui::drawMenuBar() {
    float height = ImGui::GetFrameHeight();
    if (!ImGui::BeginMainMenuBar()) {
        return height;
    }
    drawFileMenu();
    drawEditMenu();
    drawViewMenu();
    drawMeshMenu();
    drawRepairMenu();
    drawHelpMenu();
    height = ImGui::GetWindowHeight();
    ImGui::EndMainMenuBar();
    return height;
}

void PolyscopeGui::drawFileMenu() {
    if (!ImGui::BeginMenu("File")) {
        return;
    }
    if (ImGui::MenuItem("Open...", modShortcut("O").c_str())) {
        openFileDialog();
    }
    plannedMenuItem("Open Recent", "Planned (P5).");
    if (ImGui::BeginMenu("Load Sample")) {
        if (ImGui::MenuItem("Defective test cube")) {
            loadSample(nullptr);
        }
        if (ImGui::MenuItem("Folded cube (intersection islands)")) {
            controller_.setMesh(SampleMeshes::createFoldedCubeMesh(controller_.fileUnit()), "Folded cube sample");
            controller_.postStatusMessage("Loaded the folded cube: one local fold and one pass-through in one shell.");
            overwriteConfirmedFor_.clear();
            syncMeshStructureToPolyscope(true);
        }
        tooltip("A gridded cube with a folded-over top vertex and a bottom vertex pushed through the top.");
        if (ImGui::MenuItem("Touching cubes and a fin")) {
            // Loaded the way an STL file is, so the touching contacts are kept apart
            controller_.setMesh(separateTouchingContacts(SampleMeshes::createTouchingCubesMesh(controller_.fileUnit())),
                                "Touching cubes sample");
            controller_.postStatusMessage("Loaded the touching cubes: two touch along an edge, one at a corner, plus a fin.");
            overwriteConfirmedFor_.clear();
            syncMeshStructureToPolyscope(true);
        }
        tooltip("Three cubes: two touch along an edge and one at a corner, which loading keeps apart (three shells, "
                "no non-manifold vertices), and a fin triangle on a top edge, which stays a non-manifold edge.");
        if (ImGui::MenuItem("Holes (flat, curved, crack)")) {
            controller_.setMesh(separateTouchingContacts(SampleMeshes::createHolesSampleMesh(controller_.fileUnit())),
                                "Holes sample");
            controller_.postStatusMessage("Loaded the holes sample: a crack, a square hole, a sphere cap and a cylinder end cap.");
            overwriteConfirmedFor_.clear();
            syncMeshStructureToPolyscope(true);
        }
        tooltip("Four shells with one hole each: a box with a T-junction crack in its top, a box missing a square "
                "of its top, a sphere missing its cap and a cylinder missing an end cap. Try the hole fills on them.");
        constexpr std::array<std::pair<const char*, const char*>, 3> kExampleFiles = {{
            {"Self-intersecting box", "self_intersecting_box.stl"},
            {"Stanford bunny", "stanford_bunny.stl"},
            {"Turkey (28 shells)", "Turkey.stl"},
        }};
        for (const auto& [label, file] : kExampleFiles) {
            const bool found = !AppController::resolveExamplePath(file).empty();
            if (ImGui::MenuItem(label, nullptr, false, found)) {
                loadSample(file);
            }
        }
        ImGui::EndMenu();
    }
    ImGui::Separator();
    const bool hasMesh = controller_.hasMesh();
    if (ImGui::MenuItem("Save", modShortcut("S").c_str(), false, hasMesh)) {
        saveCurrentFile();
    }
    if (ImGui::BeginMenu("Save As", hasMesh)) {
        if (ImGui::MenuItem("Binary STL...", modShortcut("Shift+S").c_str())) {
            saveAsDialog(StlFormat::Binary);
        }
        if (ImGui::MenuItem("ASCII STL...")) {
            saveAsDialog(StlFormat::Ascii);
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Export Selected Shells", controller_.shells().selectedCount() > 0)) {
        if (ImGui::MenuItem("To one STL...")) {
            saveSelectedShellsDialog(false);
        }
        if (ImGui::MenuItem("Each to its own STL...")) {
            saveSelectedShellsDialog(true);
        }
        ImGui::EndMenu();
    }
    if (controller_.shells().selectedCount() == 0) {
        tooltip("Select shells in the Shells panel first (multi-shell meshes).");
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Quit")) {
        polyscope::unshow();
    }
    ImGui::EndMenu();
}

void PolyscopeGui::drawEditMenu() {
    if (!ImGui::BeginMenu("Edit")) {
        return;
    }
    const std::string undoLabel = controller_.canUndo()
        ? "Undo " + controller_.lastUndoActionLabel()
        : std::string("Undo");
    if (ImGui::MenuItem(undoLabel.c_str(), modShortcut("Z").c_str(), false, controller_.canUndo())) {
        undoLastModification();
    }
    plannedMenuItem("Redo", "Planned.");
    plannedMenuItem("Undo History...", "Planned.");
    ImGui::Separator();
    if (ImGui::BeginMenu("Select Defects", controller_.browsableResult())) {
        if (ImGui::MenuItem("All of this type")) {
            controller_.selectAllVisibleDefects();
            requestSync();
        }
        if (ImGui::MenuItem("None", "Esc")) {
            controller_.selectNoDefects();
            requestSync();
        }
        if (ImGui::MenuItem("Invert")) {
            controller_.invertDefectSelection();
            requestSync();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Next defect", "N")) {
            stepDefectFocus(+1);
        }
        if (ImGui::MenuItem("Previous defect", "Shift+N")) {
            stepDefectFocus(-1);
        }
        ImGui::EndMenu();
    }
    drawSelectFacesMenu();
    ImGui::Separator();
    if (ImGui::MenuItem("Preferences...", modShortcut(",").c_str())) {
        showPreferences_ = true;
    }
    ImGui::EndMenu();
}

void PolyscopeGui::drawViewMenu() {
    if (!ImGui::BeginMenu("View")) {
        return;
    }
    ImGui::MenuItem("Structures (advanced)", nullptr, &showStructures_);
    ImGui::Separator();
    if (ImGui::BeginMenu("Camera", controller_.hasMesh())) {
        if (ImGui::MenuItem("Home (frame all)", "Home / H")) {
            resetCameraAndPivotToHome();
        }
        if (ImGui::MenuItem("Frame selection", ".")) {
            focusCameraOnSelection(FocusNormalSide::AutoToggleNormal);
        }
        ImGui::Separator();
        static constexpr std::array<const char*, 6> kViews = {"Front", "Back", "Right", "Left", "Top", "Bottom"};
        for (std::size_t v = 0; v < kViews.size(); ++v) {
            if (ImGui::MenuItem(kViews[v])) {
                flyToStandardView(static_cast<int>(v));
            }
        }
        ImGui::Separator();
        const bool ortho = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
        if (ImGui::MenuItem("Orthographic", nullptr, ortho)) {
            toggleOrthographic();
        }
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Ghost Mesh", "G", controller_.ghostMeshEnabled())) {
        toggleGhost();
    }
    bool edges = controller_.overlayAppearance().baseMesh.showEdges;
    if (ImGui::MenuItem("Wireframe Edges", nullptr, &edges)) {
        OverlayAppearance appearance = controller_.overlayAppearance();
        appearance.baseMesh.showEdges = edges;
        controller_.setOverlayAppearance(appearance);
        applyBaseMeshStyleToPolyscope();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Screenshot")) {
        polyscope::screenshot(false);
        controller_.postStatusMessage("Saved screenshot_*.png in the working directory.");
    }
    if (ImGui::MenuItem("Reset Layout")) {
        resetLayout();
    }
    ImGui::EndMenu();
}

void PolyscopeGui::drawMeshMenu() {
    if (!ImGui::BeginMenu("Mesh")) {
        return;
    }
    if (ImGui::MenuItem("Units & Scale...", nullptr, false, controller_.hasMesh())) {
        controller_.openUnitsAndScale();
        pendingScaleTargetUnit_ = controller_.fileUnit() == Unit::Centimeter ? Unit::Millimeter : Unit::Centimeter;
        requestSync();
    }
    if (ImGui::BeginMenu("Transform")) {
        plannedMenuItem("Center at Origin", "Planned.");
        plannedMenuItem("Translate / Rotate / Mirror", "Planned.");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Shells", controller_.shells().multiShell())) {
        drawShellActionItems();
        ImGui::EndMenu();
    }

    // Modify: every operation that changes the faces, in one place. Decimate, Weld and Delete work on
    // the selected faces (Shift+click, B, C), else the shells in scope, else the whole mesh
    ImGui::SeparatorText("Modify");
    const bool hasMesh = controller_.hasMesh();
    if (ImGui::MenuItem("Decimate...", nullptr, false, hasMesh)) {
        controller_.openDecimate();
        requestSync();
    }
    tooltip("Reduce the face count of the selected faces, or the whole mesh (ratio 0.01 to 1, default 0.5). "
            "Preview, then apply.");
    if (ImGui::MenuItem("Weld / Merge Vertices...", nullptr, false, hasMesh)) {
        controller_.openMergeVertices();
        requestSync();
    }
    tooltip("Merge vertices closer than a distance (default 0.05 mm) in the selected faces, or the whole mesh, "
            "removing the faces that collapse. Preview, then apply.");
    if (ImGui::MenuItem("Alpha Wrap...", nullptr, false, hasMesh)) {
        controller_.openAlphaWrap();
        requestSync();
    }
    tooltip("Shrink-wrap the mesh (or the shells in scope) into a new watertight, manifold surface without "
            "self-intersections (CGAL 3D Alpha Wrapping). Set alpha and offset, preview, then apply.");
    if (ImGui::MenuItem("Delete Selected Faces", "Del", false, controller_.selectedFaceCount() > 0)) {
        deleteSelection();
    }
    tooltip("Remove the selected faces (Shift+click, B or C) from the mesh. Undo with Cmd+Z.");
    if (ImGui::MenuItem("Remove Small Shells...", nullptr, false, controller_.shells().multiShell())) {
        // Selecting them in the Shells panel lets the user review before pressing Delete
        const auto limit = static_cast<std::size_t>(smallShellFaces_);
        controller_.selectShellsWhere([&](std::size_t s) { return controller_.shells().shell(s).faces.size() < limit; });
        showShells_ = true;
        controller_.postStatusMessage(withThousands(controller_.shells().selectedCount()) + " shell(s) under " +
                                      withThousands(limit) + " faces selected: press Delete to remove them.");
        requestSync();
    }
    tooltip("Selects the shells smaller than N faces in the Shells panel (Select... sets N).");
    plannedMenuItem("Remove Degenerate Faces...", "Planned (P5).");
    plannedMenuItem("Remove Duplicate Faces", "Planned (P5).");
    plannedMenuItem("Remesh...", "Planned.");
    ImGui::Separator();
    if (ImGui::MenuItem("Mesh Info", nullptr, showMeshInfo_)) {
        showMeshInfo_ = !showMeshInfo_;
    }
    ImGui::EndMenu();
}

void PolyscopeGui::drawRepairMenu() {
    if (!ImGui::BeginMenu("Repair")) {
        return;
    }
    if (ImGui::MenuItem("Check Mesh", "F5", false, controller_.hasMesh())) {
        checkMesh();
    }
    ImGui::Separator();
    for (const DefectCategory category : kAllDefectCategories) {
        const CategoryInfo& info = categoryInfo(category);
        if (info.infoOnly) {
            continue;
        }
        const CategoryHealth& row = controller_.categoryHealth(category);
        const std::size_t count = row.status == HealthStatus::Defects ? controller_.rowCount(category) : 0;
        const bool repairAll = category == DefectCategory::Intersections || category == DefectCategory::OpenEdges;
        const std::string label = std::string(repairAll ? "Repair All " : "Fix All ") + info.menuName +
                                  (count > 0 ? " (" + withThousands(count) + ")" : std::string()) +
                                  (repairAll ? "..." : "");
        // Repair All opens a comparison panel, so it is offered even before the row is checked
        if (ImGui::MenuItem(label.c_str(), nullptr, false, info.hasFixAll && (count > 0 || (repairAll && controller_.hasMesh())))) {
            fixAll(category);
        }
        if (!info.hasFixAll) {
            tooltip("Planned (P4).");
        }
    }
    ImGui::Separator();
    plannedMenuItem("Auto-Repair in Safe Order", "Planned.");
    ImGui::EndMenu();
}

void PolyscopeGui::drawHelpMenu() {
    if (!ImGui::BeginMenu("Help")) {
        return;
    }
    if (ImGui::MenuItem("Navigation & Shortcuts", "F1 / ?", showNavigationHelpWindow_)) {
        showNavigationHelpWindow_ = !showNavigationHelpWindow_;
    }
    if (ImGui::MenuItem("About Ogun")) {
        requestAbout_ = true;
    }
    ImGui::EndMenu();
}

// -----------------------------------------------------------------------------
// File actions
// -----------------------------------------------------------------------------

void PolyscopeGui::openFileDialog() {
    auto selection = pfd::open_file(
        "Open STL File",
        "",
        {"STL Mesh (*.stl)", "*.stl *.STL", "All Files", "*"},
        pfd::opt::none).result();
    if (!selection.empty() && controller_.loadStlFile(std::filesystem::path(selection.front()))) {
        overwriteConfirmedFor_.clear();
        syncMeshStructureToPolyscope(true);
    }
}

void PolyscopeGui::loadSample(const char* exampleFile) {
    if (exampleFile != nullptr) {
        if (!controller_.loadSampleFile(exampleFile)) {
            return;
        }
    } else {
        controller_.loadSampleDefectiveMesh();
        controller_.postStatusMessage("Loaded the sample defective cube.");
    }
    overwriteConfirmedFor_.clear();
    syncMeshStructureToPolyscope(true);
}

void PolyscopeGui::saveCurrentFile() {
    if (!controller_.hasMesh()) {
        return;
    }
    const std::filesystem::path& path = controller_.currentFilePath();
    if (path.empty()) {
        saveAsDialog(saveFormat_);
        return;
    }
    if (overwriteConfirmedFor_ != path.string()) {
        requestSaveConfirm_ = true;
        return;
    }
    saveTo(path.string(), saveFormat_);
}

void PolyscopeGui::saveAsDialog(StlFormat format) {
    if (!controller_.hasMesh()) {
        return;
    }
    const std::string suggestion = controller_.currentFilePath().empty()
        ? std::string("repaired.stl")
        : controller_.currentFilePath().filename().string();
    auto destination = pfd::save_file(
        format == StlFormat::Binary ? "Save Binary STL" : "Save ASCII STL",
        suggestion,
        {"STL Mesh (*.stl)", "*.stl *.STL", "All Files", "*"},
        pfd::opt::none).result();
    if (destination.empty()) {
        return;
    }
    std::filesystem::path outPath(destination);
    if (!outPath.has_extension()) {
        outPath.replace_extension(".stl");
    }
    saveFormat_ = format;
    saveTo(outPath.string(), format);
}

void PolyscopeGui::saveTo(const std::string& path, StlFormat format) {
    if (controller_.saveStlFile(std::filesystem::path(path), format)) {
        overwriteConfirmedFor_ = path;
    }
}

// -----------------------------------------------------------------------------
// Status bar
// -----------------------------------------------------------------------------

void PolyscopeGui::drawStatusBar(float top, float height) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(0.0f, top), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(display.x, height), ImGuiCond_Always);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (!ImGui::Begin("##StatusBar", nullptr, flags)) {
        ImGui::End();
        return;
    }
    const auto divider = [] {
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
    };

    ImGui::AlignTextToFramePadding();
    const bool modified = controller_.isModified();
    ImGui::Text("%s%s", controller_.loadedSourceLabel().c_str(), modified ? " *" : "");
    tooltip(modified ? "Modified since it was loaded or saved." : "Unmodified.");
    divider();

    const Unit unit = controller_.fileUnit();
    const std::string unitButton = std::string(unitSymbol(unit)) + "##UnitButton";
    if (ImGui::SmallButton(unitButton.c_str())) {
        ImGui::OpenPopup("FileUnitsPopup");
    }
    tooltip("File units: click to change. They re-interpret the numbers in the file and never move "
            "vertices. Mesh > Units & Scale has the same setting and coordinate scaling.");
    if (ImGui::BeginPopup("FileUnitsPopup")) {
        ImGui::TextDisabled("File units (re-interpret, no scaling)");
        for (const Unit u : kAllUnits) {
            if (ImGui::Selectable(std::string(unitDisplayName(u)).c_str(), u == unit)) {
                controller_.setFileUnit(u);
                controller_.postStatusMessage("File units set to " + std::string(unitDisplayName(u)) +
                                              "; coordinates unchanged.");
                requestSync();
            }
        }
        ImGui::Separator();
        if (ImGui::Selectable("Scale coordinates instead...")) {
            controller_.openUnitsAndScale();
            requestSync();
        }
        ImGui::EndPopup();
    }

    if (controller_.hasMesh()) {
        const Mesh& mesh = controller_.mesh();
        divider();
        ImGui::Text("V %s   T %s",
                    withThousands(static_cast<std::size_t>(mesh.numVertices())).c_str(),
                    withThousands(static_cast<std::size_t>(mesh.numFaces())).c_str());
        divider();
        const std::string sym(unitSymbol(unit));
        const BoundingBox bbox = mesh.boundingBox();
        const Eigen::Vector3d ext = bbox.extents();
        ImGui::Text("%.4g x %.4g x %.4g %s", ext.x(), ext.y(), ext.z(), sym.c_str());
        if (ImGui::BeginItemTooltip()) {
            ImGui::Text("Bounding box (%s)", sym.c_str());
            ImGui::Text("Min: (%.5g, %.5g, %.5g)", bbox.min.x(), bbox.min.y(), bbox.min.z());
            ImGui::Text("Max: (%.5g, %.5g, %.5g)", bbox.max.x(), bbox.max.y(), bbox.max.z());
            ImGui::Text("Diagonal: %.5g", bbox.diagonal());
            ImGui::EndTooltip();
        }
        divider();
        switch (controller_.watertightState()) {
        case WatertightState::Unknown:
            ImGui::TextDisabled("Watertight: ?");
            tooltip("Run Check Mesh (F5): open and non-manifold edges both need checking.");
            break;
        case WatertightState::Watertight:
            ImGui::TextColored(gui::kOkColor, "Watertight");
            tooltip("No open edges and no non-manifold edges.");
            break;
        case WatertightState::NotWatertight:
            ImGui::TextColored(gui::kDefectColor, "Not watertight");
            tooltip("Holes (open edges) or non-manifold edges were found. See Mesh Health.");
            break;
        }
        if (controller_.selectedFaceCount() > 0) {
            divider();
            ImGui::Text("%s faces selected", withThousands(controller_.selectedFaceCount()).c_str());
            tooltip("Faces selected with B (box), C (circle) or Shift+click. Decimate and Weld / Merge Vertices work "
                    "on them. Alt+A deselects.");
        }
    }
    divider();

    if (controller_.statusMessageSerial() != shownMessageSerial_) {
        shownMessageSerial_ = controller_.statusMessageSerial();
        messageTime_ = std::chrono::steady_clock::now();
    }
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - messageTime_).count();
    if (age < settings::kStatusMessageSeconds && !controller_.statusMessage().empty()) {
        if (controller_.statusMessageIsError()) {
            ImGui::TextColored(gui::kErrorColor, "%s", controller_.statusMessage().c_str());
        } else {
            ImGui::TextUnformatted(controller_.statusMessage().c_str());
        }
    } else {
        ImGui::TextDisabled("Ready");
    }

    if (controller_.mergeRunning()) {
        // Manifold reports no progress: the bar only shows that the merge is running
        const float barWidth = 200.0f;
        const float cancelWidth = ImGui::CalcTextSize("Cancel").x + 2.0f * ImGui::GetStyle().FramePadding.x;
        ImGui::SameLine(display.x - barWidth - cancelWidth - 3.0f * settings::kPanelMarginPx);
        const auto loop = static_cast<float>(std::fmod(ImGui::GetTime() * 0.5, 1.0));
        ImGui::ProgressBar(loop, ImVec2(barWidth, 0.0f), "Merging shells...");
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel##merge")) {
            controller_.cancelMerge();
        }
    } else if (controller_.globalRepairRunning()) {
        const float barWidth = 200.0f;
        const float cancelWidth = ImGui::CalcTextSize("Cancel").x + 2.0f * ImGui::GetStyle().FramePadding.x;
        ImGui::SameLine(display.x - barWidth - cancelWidth - 3.0f * settings::kPanelMarginPx);
        const std::optional<GlobalRepairMethod> current = controller_.globalRepairCurrentMethod();
        const std::string overlay = current ? std::string(globalRepairMethodLabel(*current)) + "..." : std::string("Repairing...");
        ImGui::ProgressBar(controller_.globalRepairProgress(), ImVec2(barWidth, 0.0f), overlay.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel##repair")) {
            controller_.cancelGlobalRepair();
            controller_.postStatusMessage("Repair All preview cancelled.");
        }
    } else if (controller_.checksRunning()) {
        const float barWidth = 200.0f;
        const float cancelWidth = ImGui::CalcTextSize("Cancel").x + 2.0f * ImGui::GetStyle().FramePadding.x;
        ImGui::SameLine(display.x - barWidth - cancelWidth - 3.0f * settings::kPanelMarginPx);
        const std::optional<DefectCategory> current = controller_.currentCheckCategory();
        const std::string overlay = current ? std::string(categoryInfo(*current).name) + "..." : std::string("Checking...");
        ImGui::ProgressBar(controller_.checkProgress(), ImVec2(barWidth, 0.0f), overlay.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) {
            controller_.cancelChecks();
            controller_.postStatusMessage("Check cancelled.");
            requestSync();
        }
    }
    ImGui::End();
}

// -----------------------------------------------------------------------------
// Floating windows
// -----------------------------------------------------------------------------

void PolyscopeGui::drawPreferencesWindow() {
    const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(460.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::Begin("Preferences", &showPreferences_, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("PreferencesTabs")) {
        if (ImGui::BeginTabItem("Units & Tolerance")) {
            ToleranceConfig tol = controller_.toleranceConfig();
            bool changed = false;
            ImGui::PushItemWidth(110.0f);
            if (ImGui::InputDouble("Weld floor (cm)", &tol.absFloorCm, 0.0, 0.0, "%.0e")) {
                tol.absFloorCm = std::max(0.0, tol.absFloorCm);
                changed = true;
            }
            gui::helpMarker("Absolute vertex-welding tolerance floor used when loading STL files.");
            if (ImGui::InputDouble("Relative tolerance", &tol.relTol, 0.0, 0.0, "%.0e")) {
                tol.relTol = std::max(0.0, tol.relTol);
                changed = true;
            }
            ImGui::PopItemWidth();
            if (changed) {
                controller_.setToleranceConfig(tol);
            }
            const std::string sym(unitSymbol(controller_.fileUnit()));
            ImGui::TextDisabled("Active floor: %.0e %s (applies to the next load)",
                                controller_.toleranceConfig().absFloorInUnit(controller_.fileUnit()), sym.c_str());
            if (ImGui::Button("Reset default tolerances")) {
                controller_.setToleranceConfig(ToleranceConfig{});
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Diagnostics")) {
            bool afterLoad = controller_.checkMeshAfterLoad();
            if (ImGui::Checkbox("Run Check Mesh after loading a file", &afterLoad)) {
                controller_.setCheckMeshAfterLoad(afterLoad);
            }
            ImGui::SeparatorText("Default Auto per row (new meshes)");
            for (const DefectCategory category : kAllDefectCategories) {
                bool enabled = controller_.autoCheckDefault(category);
                if (ImGui::Checkbox(categoryInfo(category).name, &enabled)) {
                    controller_.setAutoCheckDefault(category, enabled);
                }
            }
            ImGui::TextDisabled("Thresholds live in each row's Tool > Settings.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Interface")) {
            ImGui::SetNextItemWidth(160.0f);
            ImGui::SliderFloat("UI scale", &uiScale_, settings::kMinUiScale, settings::kMaxUiScale, "%.2f");
            int listCap = static_cast<int>(controller_.listCap());
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputInt("List cap", &listCap)) {
                controller_.setListCap(static_cast<std::size_t>(std::max(1, listCap)));
            }
            gui::helpMarker("Above this many defects the Tool panel shows a summary and histogram instead of a list.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Performance")) {
            int overlayCap = static_cast<int>(controller_.overlayCap());
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputInt("3D overlay cap", &overlayCap, 100, 1000)) {
                controller_.setOverlayCap(static_cast<std::size_t>(std::max(0, overlayCap)));
                requestSync();
            }
            gui::helpMarker("Above this many highlighted faces only the face tint is drawn; edge tubes and vertex "
                            "spheres are kept for the selected defects.");
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::Separator();
    ImGui::TextDisabled("Settings last for this session. Saving defaults is planned (P5).");
    ImGui::End();
}

void PolyscopeGui::drawModalPopups() {
    if (requestAbout_) {
        ImGui::OpenPopup("About Ogun");
        requestAbout_ = false;
    }
    if (ImGui::BeginPopupModal("About Ogun", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Ogun - STL Mesh Inspection & Repair Tool");
        ImGui::TextDisabled("Prepares STL meshes for simulation. GPL-3.0-or-later.");
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (requestSaveConfirm_) {
        ImGui::OpenPopup("Overwrite file?");
        requestSaveConfirm_ = false;
    }
    if (ImGui::BeginPopupModal("Overwrite file?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const std::string path = controller_.currentFilePath().string();
        ImGui::Text("Overwrite %s?", controller_.currentFilePath().filename().string().c_str());
        ImGui::TextDisabled("%s", path.c_str());
        ImGui::TextDisabled("Format: %s STL. You will not be asked again for this file.",
                            saveFormat_ == StlFormat::Binary ? "Binary" : "ASCII");
        if (ImGui::Button("Overwrite")) {
            saveTo(path, saveFormat_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Save As...")) {
            ImGui::CloseCurrentPopup();
            saveAsDialog(saveFormat_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void PolyscopeGui::drawNavigationHelpWindow() {
    const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("3D Viewport Navigation & Shortcuts", &showNavigationHelpWindow_)) {
        ImGui::End();
        return;
    }
    ImGui::TextWrapped(
        "Ogun uses Blender-inspired selection-centric camera navigation so inspecting microscopic sliver "
        "facets feels smooth at any zoom depth. Every button tooltip shows its key.");
    ImGui::Separator();

    const auto table = [](const char* id, const char* left, std::initializer_list<std::pair<const char*, const char*>> rows) {
        if (ImGui::BeginTable(id, 2, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn(left, ImGuiTableColumnFlags_WidthFixed, 175.0f);
            ImGui::TableSetupColumn("Behavior", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (const auto& [key, behavior] : rows) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(key);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextWrapped("%s", behavior);
            }
            ImGui::EndTable();
        }
    };

    ImGui::TextUnformatted("Mouse Controls (3D Viewport)");
    table("MouseNavTable", "Action", {
        {"Orbit / Rotate", "Left-Click Drag (orbits around the active Orbit Pivot)"},
        {"Pan Camera & Pivot", "Shift + Left-Click Drag, Right-Click Drag, or Middle-Click Drag (shifts the Orbit Pivot with the camera, scaled by depth)"},
        {"Zoom (Dolly)", "Scroll Wheel or Ctrl + Shift + Left-Click Drag (speed scales with distance to the Orbit Pivot)"},
        {"Select Face", "Left-Click a face: if it is a defect of the selected Mesh Health row it becomes the focus (Inspector, Orbit Pivot at its centroid); with no row selected, the row containing it opens; a clean face only moves the Orbit Pivot. Right-Click clears the Polyscope pick."},
        {"Select Shell", "Alt + Left-Click a face selects its whole shell in the Shells panel (multi-shell meshes); Alt + Ctrl/Cmd + Click adds or removes it. A plain click never changes the shell selection."},
        {"Toggle Face", "Shift + Left-Click a face adds it to or removes it from the face selection (Decimate and Weld / Merge Vertices then work on the selected faces)"},
    });

    ImGui::Spacing();
    ImGui::TextUnformatted("Keyboard Shortcuts");
    table("KeyNavTable", "Shortcut", {
        {"Click defect in list", "Focuses it, flies the camera to it (as '.') and sets the Orbit Pivot to its centroid; Ctrl/Cmd+click toggles, Shift+click selects a range (the camera stays)"},
        {"Click shell in Shells panel", "Selects it (Ctrl/Cmd+click toggles, Shift+click selects a range, double-click frames it); right-click for Frame, Isolate, Hide, Delete, Save As"},
        {"N  /  Shift+N", "Next / previous defect of the selected row (worst first, within the active filter) and fly to it"},
        {".  or  Numpad .", "Frames the focused facet looking down +Normal (press '.' again to toggle to -Normal); a focused hole frames the whole hole, looking in along the hole's outward normal; after Shift+click / B / C, frames the selected faces (square on when they face one way, '.' again flips); with no focused defect, frames the selected shells"},
        {"Shift + .", "Frames the focused facet (or hole) from the -Normal (back / inside) side"},
        {"R", "Commit Repair: applies the Inspector's chosen repair to the focused defect (thin face, intersecting island or hole); one undo step. Refused, with the reason in the status bar, when the preview is not safe"},
        {"Home  or  H", "Frames the entire mesh and resets the Orbit Pivot to the mesh center"},
        {"G", "Toggles Ghost (base-mesh transparency)"},
        {"B", "Box select faces: drag a rectangle (Shift+drag deselects); one box per B. Esc or right-click cancels. Only the faces you see, unless Select Through or Ghost (G) is on; hidden shells never"},
        {"C", "Circle select faces: paint with the brush (Shift or middle drag deselects, wheel resizes); Esc, Enter or right-click finishes"},
        {"A  /  Alt+A", "Select all shown faces / deselect all faces"},
        {"Esc", "Leaves box / circle select; clears the defect selection; then closes the Tool and deselects the row, then clears the face selection, then the shell selection"},
        {"I", "Shells: isolate the selected shells (Mesh Health then covers only them); press again to bring the others back. With another shell selected, switches the isolation to it"},
        {"Delete  /  Backspace", "Deletes the selected faces (Mesh > Delete Selected Faces); with no faces selected, the selected shells. Undoable"},
        {"F5", "Check Mesh: runs every Mesh Health row in the background"},
        {"Ctrl/Cmd + Z", "Undoes the last mesh modification (repair, Fix All / Fix Selected, Scale, Decimate, Weld, Delete Faces / Shells), one step at a time"},
        {"Ctrl/Cmd + O / S / Shift+S", "Open, Save (overwrite, asks the first time), Save As"},
        {"Ctrl/Cmd + ,", "Preferences"},
        {"?  or  F1", "Toggles this Navigation & Shortcuts window"},
        {"Ctrl+C / Ctrl+V", "Copy / paste the camera pose (Polyscope built-in)"},
    });
    ImGui::End();
}

} // namespace ogun
