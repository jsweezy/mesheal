#include "app/AboutInfo.hpp"
#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"
#include "ogun/Version.hpp"

#include <imgui.h>
#include <polyscope/polyscope.h>
#include <polyscope/render/engine.h>
#include <polyscope/screenshot.h>
#include <polyscope/view.h>
#include <portable-file-dialogs.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ogun {

using gui::modShortcut;
using gui::tooltip;
using gui::withThousands;

namespace {

constexpr ImGuiWindowFlags kDockedFlags = ImGuiWindowFlags_NoMove;

/// A built-in sample under File > Load Sample, named by its shape.
struct ShapeSample {
    const char* label;
    const char* status;  ///< Status bar text after loading
    const char* tooltip;
    Mesh (*make)(Unit unit);
};

constexpr std::array<ShapeSample, 17> kShapeSamples = {{
    {"Box with cavity, hole, slivers, and intersections",
     "Loaded a gridded box with a cavity, a hole, slivers, a fold and a peg through its side.",
     "A gridded box with a hollow inside (a cavity), a missing square (a hole), three thin faces, a top vertex "
     "folded over its neighbours and a peg box passing through its side: 3 shells with something for every repair.",
     [](Unit unit) { return SampleMeshes::createCavityHoleSliversIntersectionsMesh(unit); }},
    {"Box with cavity", "Loaded a box with a cavity: 2 shells, #2 inside #1 and wound inward.",
     "A box holding a closed hollow: the inner box is wound inward, so it is a cavity, not material.",
     [](Unit unit) { return SampleMeshes::createCavityBoxMesh(false, unit); }},
    {"Box with cavity, inside out", "Loaded a box with a cavity, every face wound the wrong way.",
     "The box with a cavity written wholly inside out: the outer box faces inward, the inner box outward.",
     [](Unit unit) { return SampleMeshes::createCavityBoxMesh(true, unit); }},
    {"Overlapping boxes with cavity", "Loaded two overlapping boxes, the first holding a cavity.",
     "Two boxes that overlap; the first holds a cavity well clear of the second: 3 shells.",
     [](Unit unit) { return SampleMeshes::createOverlapPlusCavityMesh(unit); }},
    {"Overlapping boxes with cavity in the overlap", "Loaded two overlapping boxes whose cavity the second box reaches into.",
     "Two boxes that overlap; the first box's cavity lies across the second box's wall, which passes through it.",
     [](Unit unit) { return SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::CrossesWall, unit); }},
    {"Box with nested box and cavity", "Loaded a box holding a cavity and a smaller solid box.",
     "A box holding a cavity (wound inward) and a smaller solid box (wound outward) that touches nothing: 3 shells.",
     [](Unit unit) { return SampleMeshes::createNestedSolidAndCavityMesh(false, unit); }},
    {"Boxes touching along an edge", "Loaded two boxes that touch along one edge.",
     "Two boxes that meet along one edge only, so they cannot become one proper solid.",
     [](Unit unit) { return SampleMeshes::createEdgeTouchingBoxesMesh(unit); }},
    {"Boxes touching across a face", "Loaded two boxes that share a patch of one face.",
     "A small box pressed against a side of a larger one: they share a 1 x 1 patch of face.",
     [](Unit unit) { return SampleMeshes::createFaceTouchingBoxesMesh(unit); }},
    {"Turkey-like assembly", "Loaded a body box with six parts through it, one hidden inside and one apart: 9 shells.",
     "A body box, six thin boxes poking through its top, a small box hidden inside it touching nothing and one "
     "box apart from the rest: 9 shells.",
     [](Unit unit) { return SampleMeshes::createTurkeyLikeMesh(unit); }},
    {"Cube with doubled and collapsed faces",
     "Loaded a cube with two faces stored twice, a face that repeats a corner and a flat triangle on an edge.",
     "A closed cube plus: a face stored twice the same way, a face stored twice facing both ways, a triangle that "
     "repeats a corner and a flat triangle lying along the bottom front edge (closing a T-junction). 2 degenerate "
     "and 2 duplicate faces.",
     [](Unit unit) { return SampleMeshes::createDoubledFacesCubeMesh(unit); }},
    {"Cube with tiny floating pieces",
     "Loaded a cube with a small cube apart, a tiny cavity inside, a tiny tetrahedron on top and one far away: 5 shells.",
     "A cube, a small cube beside it, a tiny cavity inside it, a tiny tetrahedron resting on its top and a tiny "
     "tetrahedron far from everything (0.06 % of the model's size, touching nothing): 5 shells.",
     [](Unit unit) { return SampleMeshes::createTinyFloatingShellsMesh(unit); }},
    {"Cube with a hairline seam",
     "Loaded a cube cut in two open halves with a 0.000005 gap between them: 2 shells, 8 open edges.",
     "A cube [10, 20]^3 cut across at x = 15 into two open halves, the right half 0.000005 further right: a crack "
     "too wide for the weld done when loading (1.5e-8 there) but within Fix All's weld (1.5e-5 there).",
     [](Unit unit) { return SampleMeshes::createWeldSeamCubeMesh(5.0e-6, Eigen::Vector3d::Zero(), 1.0, unit); }},
    {"Cube with a visible crack",
     "Loaded a 4 mm cube with a visible 0.04 mm crack: 1 shell, 6 open edges, 0 other defects.",
     "A 4 mm cube with a clearly visible 0.04 mm crack centered on its top face: 1 shell, 6 open edges, and "
     "zero other defects. Try Mesh > Modify > Weld (default distance 0.05 mm heals it completely) or "
     "Mesh > Modify > Merge by Distance (preserves the crack when checked).",
     [](Unit unit) { return SampleMeshes::createVisibleCrackCubeMesh(0.04, unit); }},
    {"Cup with a post",
     "Loaded a cup with an open top and a post rising inside it to just below the opening: 1 shell, 1 hole.",
     "A round cup whose lip slopes down into a round opening (a 32-edge hole), with a post pushed up from its "
     "bottom to just below the opening. Fill Smooth follows the lip down and runs into the post; a flat fill "
     "crosses nothing.",
     [](Unit unit) { return SampleMeshes::createCupWithPostMesh(SampleMeshes::CupPost::BelowOpening, unit); }},
    {"Cracked cube with flipped faces",
     "Loaded a cube with two hairline cracks, two faces wound backwards and a missing triangle: 1 shell.",
     "A gridded cube read like a file: two corners are 0.0000001 off in one triangle each (cracks the weld done "
     "when loading leaves and Fix All's weld closes), two triangles face inward and one triangle is missing.",
     [](Unit unit) { return SampleMeshes::createCrackedFlippedCubeSoup(unit); }},
    {"Box holding a plate through an open box",
     "Loaded a box holding a thin plate and an open box the plate passes through: 3 shells, 1 hole.",
     "A box holding a thin plate and a smaller box with one side missing; the plate passes through the small "
     "box's walls and its open side, so any patch over the opening crosses the plate. With Merge shells on, "
     "the merge unites them.",
     [](Unit unit) {
         return SampleMeshes::createFillCrossesShellMesh(SampleMeshes::FillCrossesVariant::PlateThroughOpenBox, unit);
     }},
    {"Overlapping boxes, one open",
     "Loaded three boxes crossing the first, the third with one side missing: 3 shells, 1 hole.",
     "Two boxes that overlap and a third, open box poking into the first. An open shell cannot be merged: the "
     "merge unites the two closed ones and names the open one.",
     [](Unit unit) { return SampleMeshes::createOpenShellAmongCubesMesh(unit); }},
}};

/// Menu item for a feature planned in a later phase: drawn disabled with a tooltip.
void plannedMenuItem(const char* label, const char* when) {
    ImGui::MenuItem(label, nullptr, false, false);
    tooltip(when);
}

/// What an action does, for the Mouse tab's tooltips.
const char* mouseActionHelp(MouseAction action) {
    switch (action) {
    case MouseAction::Orbit: return "Orbits the camera around the Orbit Pivot.";
    case MouseAction::Pan: return "Moves the camera and the Orbit Pivot in the view plane.";
    case MouseAction::Zoom: return "Dollies toward the Orbit Pivot (drag up) or away from it (drag down).";
    case MouseAction::WheelZoom: return "Dollies toward or away from the Orbit Pivot.";
    case MouseAction::NearClip: return "Shifts the camera's near clip plane.";
    case MouseAction::Select:
        return "Clicks a face: focuses a defect of the selected Mesh Health row, or opens the row that holds it. "
               "Its button also draws Box (B) and paints Circle (C) select.";
    case MouseAction::ToggleFace: return "Adds the face under the cursor to the face selection, or removes it.";
    case MouseAction::SelectShell:
        return "Selects the shell under the cursor (multi-shell meshes). Add Ctrl or Cmd to add or remove it.";
    case MouseAction::ClearPick: return "Clears Polyscope's pick.";
    }
    return "";
}

} // namespace

void PolyscopeGui::drawMousePreferencesTab() {
    MouseMapping& mapping = mouseMapping_;
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo("Scheme", mousePresetName(mapping.preset))) {
        for (const MousePreset preset : kAllMousePresets) {
            if (ImGui::Selectable(mousePresetName(preset), preset == mapping.preset)) {
                selectMousePreset(mapping, preset);
            }
            tooltip(mousePresetDescription(preset));
        }
        ImGui::EndCombo();
    }
    gui::helpMarker("Which mouse button and modifier keys orbit, pan, zoom and select in the 3D view. Picking a "
                    "scheme replaces every binding below; editing a binding switches to Custom.");
    ImGui::TextWrapped("%s", mousePresetDescription(mapping.preset));

    const std::vector<MouseConflict> conflicts = findMouseConflicts(mapping);
    const auto inConflict = [&conflicts](MouseAction action, const MouseBinding& binding) {
        return std::any_of(conflicts.begin(), conflicts.end(), [&](const MouseConflict& c) {
            return c.binding == binding && (c.first == action || c.second == action);
        });
    };

    bool openEditor = false;
    if (ImGui::BeginTable("MouseBindings", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 170.0f);
        ImGui::TableSetupColumn("Inputs (click one to change it, + to add one)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (const MouseAction action : kAllMouseActions) {
            ImGui::PushID(static_cast<int>(action));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(mouseActionName(action));
            tooltip(mouseActionHelp(action));
            ImGui::TableSetColumnIndex(1);
            const MouseActionBindings& bindings = mapping.of(action);
            std::size_t slot = 0;
            for (; slot < bindings.size() && bindings[slot].bound(); ++slot) {
                if (slot > 0) {
                    ImGui::SameLine();
                }
                const bool conflict = inConflict(action, bindings[slot]);
                if (conflict) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.60f, 0.15f, 0.15f, 1.0f));
                }
                ImGui::PushID(static_cast<int>(slot));
                if (ImGui::SmallButton(formatMouseBinding(bindings[slot], mouseActionKind(action)).c_str())) {
                    editingMouseAction_ = action;
                    editingMouseSlot_ = slot;
                    openEditor = true;
                }
                ImGui::PopID();
                if (conflict) {
                    ImGui::PopStyleColor();
                }
                tooltip(conflict ? "Also bound to another action (see the warning below). Click to change it."
                                 : "Click to change or remove this input.");
            }
            if (slot < bindings.size()) {
                if (slot > 0) {
                    ImGui::SameLine();
                }
                if (ImGui::SmallButton("+")) {
                    editingMouseAction_ = action;
                    editingMouseSlot_ = slot;
                    openEditor = true;
                }
                tooltip(slot == 0 ? "Not bound: click to add an input." : "Add another input for this action.");
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (openEditor) {
        ImGui::OpenPopup("MouseBindingEditor");
    }
    drawMouseBindingEditor();

    for (const MouseConflict& c : conflicts) {
        ImGui::PushStyleColor(ImGuiCol_Text, gui::kErrorColor);
        ImGui::TextWrapped("%s is bound to both %s and %s: only %s acts on it.",
                           formatMouseBinding(c.binding, mouseActionKind(c.first)).c_str(), mouseActionName(c.first),
                           mouseActionName(c.second), mouseActionName(c.first));
        ImGui::PopStyleColor();
    }
    ImGui::TextDisabled("A click is a press and release without moving, so a drag and a click on the same");
    ImGui::TextDisabled("button never conflict. Ctrl is the Control key, also on macOS.");

    ImGui::SeparatorText("Zoom");
    ImGui::Checkbox("Invert wheel zoom", &mapping.invertWheelZoom);
    tooltip("Off: wheel up (away from you) zooms in. On: wheel up zooms out.");
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Zoom speed", &mapping.zoomSpeed, settings::kMinMouseZoomSpeed, settings::kMaxMouseZoomSpeed,
                       "%.2fx", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
    tooltip("Multiplies each wheel and drag zoom step (1.00x: Ogun's default speed).");
}

void PolyscopeGui::drawMouseBindingEditor() {
    if (!ImGui::BeginPopup("MouseBindingEditor")) {
        return;
    }
    const MouseAction action = editingMouseAction_;
    const MouseActionKind kind = mouseActionKind(action);
    const std::size_t slot = std::min(editingMouseSlot_, kMaxMouseBindingsPerAction - 1);
    const MouseBinding current = mouseMapping_.of(action)[slot];
    ImGui::TextUnformatted(mouseActionName(action));
    ImGui::TextDisabled("Input %zu: %s", slot + 1, formatMouseBinding(current, kind).c_str());

    // Click to capture: press the button (or turn the wheel) over the box with the modifiers held
    const ImVec2 size(320.0f, 3.0f * ImGui::GetFrameHeight());
    ImGui::Dummy(ImVec2(size.x, 0.0f)); // fixes the popup width so the prompt wraps inside it
    const ImVec2 corner = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("capture", size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 far(corner.x + size.x, corner.y + size.y);
    draw->AddRectFilled(corner, far, ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), 4.0f);
    draw->AddRect(corner, far, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
    const char* prompt = kind == MouseActionKind::Wheel ? "Hold the modifier keys and scroll here"
                                                         : "Hold the modifier keys and press a mouse button here";
    const float padding = ImGui::GetStyle().FramePadding.x;
    ImGui::PushClipRect(corner, far, true);
    ImGui::SetCursorScreenPos(ImVec2(corner.x + padding, corner.y + padding));
    ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + size.x - 2.0f * padding);
    ImGui::TextDisabled("%s", prompt);
    ImGui::PopTextWrapPos();
    ImGui::PopClipRect();
    ImGui::SetCursorScreenPos(ImVec2(corner.x, far.y + ImGui::GetStyle().ItemSpacing.y));

    std::optional<MouseBinding> edited;
    if (hovered) {
        const ImGuiIO& io = ImGui::GetIO();
        if (kind == MouseActionKind::Wheel) {
            if (io.MouseWheel != 0.0f || io.MouseWheelH != 0.0f) {
                edited = MouseBinding{MouseInput::Wheel, gui::heldMouseModifiers()};
            }
        } else {
            for (const auto& [button, input] : {std::pair{ImGuiMouseButton_Left, MouseInput::Left},
                                                std::pair{ImGuiMouseButton_Middle, MouseInput::Middle},
                                                std::pair{ImGuiMouseButton_Right, MouseInput::Right}}) {
                if (ImGui::IsMouseClicked(button)) {
                    edited = MouseBinding{input, gui::heldMouseModifiers()};
                }
            }
        }
    }

    // Or set it by hand
    MouseBinding manual = current;
    if (kind == MouseActionKind::Wheel) {
        bool wheel = manual.bound();
        if (ImGui::Checkbox("Wheel", &wheel)) {
            manual.input = wheel ? MouseInput::Wheel : MouseInput::None;
        }
    } else {
        constexpr std::array<std::pair<MouseInput, const char*>, 3> kButtons = {{
            {MouseInput::Left, "Left"}, {MouseInput::Middle, "Middle"}, {MouseInput::Right, "Right"},
        }};
        for (const auto& [input, label] : kButtons) {
            if (ImGui::RadioButton(label, manual.input == input)) {
                manual.input = input;
            }
            ImGui::SameLine();
        }
        ImGui::TextDisabled(kind == MouseActionKind::Click ? "click" : "drag");
    }
    ImGui::BeginDisabled(!manual.bound());
    for (const auto& [bit, label] : {std::pair{kMouseModCtrl, "Ctrl"}, std::pair{kMouseModShift, "Shift"},
                                     std::pair{kMouseModAlt, "Alt"}}) {
        bool on = (manual.modifiers & bit) != 0;
        if (ImGui::Checkbox(label, &on)) {
            manual.modifiers = static_cast<MouseModifiers>(on ? (manual.modifiers | bit) : (manual.modifiers & ~bit));
        }
        ImGui::SameLine();
    }
    ImGui::NewLine();
    ImGui::EndDisabled();
    if (!manual.bound()) {
        manual.modifiers = kMouseModNone;
    }
    if (!edited && !(manual == current)) {
        edited = manual;
    }

    bool close = false;
    if (current.bound() && ImGui::Button("Remove")) {
        edited = MouseBinding{};
    }
    if (current.bound()) {
        ImGui::SameLine();
    }
    if (ImGui::Button("Done")) {
        close = true;
    }
    if (edited && setMouseBinding(mouseMapping_, action, slot, *edited) && !edited->bound()) {
        close = true; // the remaining inputs moved up: nothing left to edit in this slot
    }
    if (close) {
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

// -----------------------------------------------------------------------------
// Docked layout
// -----------------------------------------------------------------------------

void PolyscopeGui::resetLayout() {
    showStructures_ = false;
    showPolySlicePlanes_ = showPolyAppearance_ = showPolyView_ = showPolyRender_ = false;
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
                        : slot == ToolSlot::AlphaWrap              ? std::string("Operation: Shrink-Wrap (Outer Envelope)")
                        : slot == ToolSlot::VoxelHealRemesh        ? std::string("Operation: Voxel Heal & Remesh (Fast)")
                        : slot == ToolSlot::PrecisionIsosurface    ? std::string("Operation: Precision Isosurface (FEA Grade)")
                        : slot == ToolSlot::Decimate               ? std::string("Operation: Decimate")
                        : slot == ToolSlot::MergeVertices          ? std::string("Operation: Merge by Distance")
                        : slot == ToolSlot::Weld                   ? std::string("Operation: Weld")
                        : slot == ToolSlot::MergeShells            ? std::string("Operation: Merge Shells")
                        : slot == ToolSlot::Transform              ? std::string("Operation: Translate / Rotate")
                        : slot == ToolSlot::FixAll                 ? std::string("Operation: Fix All")
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
        } else if (slot == ToolSlot::FixAll) {
            drawFixAllOperator();
        } else if (controller_.repairAllOpen()) {
            drawRepairAllOperator();
        } else if (slot == ToolSlot::MergeShells) {
            drawMergeShellsOperator();
        } else if (slot == ToolSlot::Transform) {
            drawTransformOperator();
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
    drawOpenRecentMenu();
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
        ImGui::Separator();
        for (const ShapeSample& sample : kShapeSamples) {
            if (ImGui::MenuItem(sample.label)) {
                // Loaded the way an STL file is, so shells that touch stay apart
                controller_.setMesh(separateTouchingContacts(sample.make(controller_.fileUnit())), sample.label);
                controller_.postStatusMessage(sample.status);
                overwriteConfirmedFor_.clear();
                syncMeshStructureToPolyscope(true);
            }
            tooltip(sample.tooltip);
        }
        ImGui::Separator();
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
    ImGui::MenuItem("Compass", nullptr, &showCompass_);
    tooltip("Draw X / Y / Z axes in the corner of the 3D view, turning with the camera.");
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
    if (ImGui::BeginMenu("Polyscope")) {
        ImGui::MenuItem("Appearance", nullptr, &showPolyAppearance_);
        tooltip("Background color, transparency mode (None / Simple / Pretty), ground plane, tone mapping, "
                "anti-aliasing, materials.");
        ImGui::MenuItem("Slice Planes", nullptr, &showPolySlicePlanes_);
        tooltip("Polyscope's slice planes: add, move and remove cutting planes. Also under Navigate > View.");
        ImGui::MenuItem("Structures", nullptr, &showStructures_);
        tooltip("Polyscope's own per-structure controls.");
        ImGui::MenuItem("Camera & Clipping", nullptr, &showPolyView_);
        tooltip("Navigation style, up and front direction, field of view, clip planes, slice planes. Also under "
                "Navigate > View.");
        ImGui::MenuItem("Render & Debug", nullptr, &showPolyRender_);
        tooltip("Frame rate, max fps, vsync, force refresh, pick buffer.");
        ImGui::EndMenu();
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
        const std::optional<Eigen::Vector3d> centroid = controller_.selectionCentroid();
        if (ImGui::MenuItem("Set Origin to Selection...", nullptr, false, centroid.has_value())) {
            pendingOrigin_ = *centroid;
            requestSetOrigin_ = true;
        }
        tooltip(centroid ? "Move the whole mesh so the centroid of the selected faces (area weighted) lands on the "
                           "origin. Asks first. One undo step."
                         : "Select faces first");
        if (ImGui::MenuItem("Translate / Rotate...", nullptr, false, controller_.hasMesh())) {
            controller_.openTransform();
            requestSync();
        }
        tooltip("Type a translation and rotations about X, Y and Z (degrees). Preview, then apply. Translation "
                "happens before rotation.");
        plannedMenuItem("Center at Origin", "Planned.");
        plannedMenuItem("Mirror", "Planned.");
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
    if (ImGui::MenuItem("Merge by Distance...", nullptr, false, hasMesh)) {
        controller_.openMergeVertices();
        requestSync();
    }
    tooltip("Merge vertices closer than a distance (default 0.05) in the selected faces, or the whole mesh, "
            "removing the faces that collapse. Preserves open edges and cracks by default. Preview, then apply.");
    if (ImGui::MenuItem("Weld...", nullptr, false, hasMesh)) {
        controller_.openWeld();
        requestSync();
    }
    tooltip("Weld open boundary vertices together across seams and cracks within a distance (default 0.05) "
            "without modifying interior faces. Preview, then apply.");
    if (ImGui::MenuItem("Delete Selected Faces", "X / Del", false, controller_.selectedFaceCount() > 0)) {
        deleteSelection();
    }
    tooltip("Remove the selected faces (Shift+click, B or C) from the mesh. X asks first; Del / Backspace delete at once. Undo with Cmd+Z.");
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
    plannedMenuItem("Remove Degenerate Faces...", "Planned (P5). Fix All on the Degenerate faces row (Mesh Health) does this now.");
    plannedMenuItem("Remove Duplicate Faces", "Planned (P5). Fix All on the Duplicate faces row (Mesh Health) does this now.");
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
    const bool hasMesh = controller_.hasMesh();
    if (ImGui::MenuItem("Check Mesh", "F5", false, hasMesh)) {
        checkMesh();
    }
    if (ImGui::MenuItem("Fix All...", "Shift+F5", false, hasMesh)) {
        openFixAllPanel(true);
    }
    tooltip("Repair every problem in the scope (the selection, or the whole mesh), repeating the sequence up to the number of "
            "passes you set, with a preview and before / after numbers. Nothing changes until Apply.");
    if (ImGui::MenuItem("Fix All, One Pass...", "Alt+Shift+F5", false, hasMesh)) {
        openFixAllPanel(false);
    }
    tooltip("Run the Fix All sequence once and preview it, so you can see each step's effect. Run it again for the next pass.");
    ImGui::SeparatorText("Remesh & Watertight Rebuild");
    if (ImGui::MenuItem("Shrink-Wrap (Outer Envelope)...", nullptr, false, hasMesh)) {
        controller_.openAlphaWrap();
        requestSync();
    }
    tooltip("Best for 3D printing external shells and enveloping thin, zero-volume sheets. Creates a guaranteed "
            "watertight outer casing around the geometry. Warning: This will slightly inflate the model and "
            "completely fill in any enclosed internal voids or hollow cavities.");
    if (ImGui::MenuItem("Voxel Heal & Remesh (Fast)...", nullptr, false, hasMesh)) {
        controller_.openVoxelHealRemesh();
        requestSync();
    }
    tooltip("Best for organic models, 3D scans, and rapid manifold conversion. Rebuilds the model using a uniform "
            "3D grid to quickly resolve all self-intersections, stitch holes, and generate even, uniform triangles. "
            "Preserves internal hollow cavities. Note: Very sharp mechanical edges may be slightly softened.");
    if (ImGui::MenuItem("Precision Isosurface (FEA Grade)...", nullptr, false, hasMesh)) {
        controller_.openPrecisionIsosurface();
        requestSync();
    }
    tooltip("Best for engineering parts, CAD exports, and finite element simulation. Traces the exact mathematical "
            "boundary of the volume using Delaunay refinement. Generates high-quality, simulation-ready triangles "
            "(guaranteed >= 30 deg angles) that adaptively shrink to hug tight curves and mechanical fillets without "
            "stair-stepping. Slower than Voxel Remesh.");
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
        if (ImGui::MenuItem(label.c_str(), nullptr, false, info.hasFixAll && (count > 0 || (repairAll && hasMesh)))) {
            fixAll(category);
        }
        if (!info.hasFixAll) {
            tooltip("Planned (P4).");
        }
        if (category == DefectCategory::OpenEdges) {
            if (ImGui::MenuItem("Weld...", nullptr, false, hasMesh)) {
                controller_.openWeld();
                requestSync();
            }
            tooltip("Weld open boundary vertices together across seams and cracks within a distance (default 0.05) "
                    "without modifying interior faces. Preview, then apply.");
        }
    }
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
        showAboutWindow_ = true;
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
    if (!selection.empty()) {
        openStlFile(std::filesystem::path(selection.front()));
    }
}

void PolyscopeGui::openStlFile(const std::filesystem::path& path) {
    if (!controller_.loadStlFile(path)) {
        return;
    }
    recentFiles_.add(path);
    if (std::string error; !recentFiles_.save(&error)) {
        controller_.postStatusMessage("Could not save the recent files list: " + error, true);
    }
    overwriteConfirmedFor_.clear();
    syncMeshStructureToPolyscope(true);
}

// A missing file is shown greyed out (not removed on click), so the list stays predictable and the
// entry comes back if the drive is remounted. The full path is the tooltip.
void PolyscopeGui::drawOpenRecentMenu() {
    if (!ImGui::BeginMenu("Open Recent")) {
        return;
    }
    if (recentFiles_.empty()) {
        ImGui::MenuItem("(none)", nullptr, false, false);
    }
    // Copy: opening a file reorders the list while it is drawn
    const std::vector<std::filesystem::path> entries = recentFiles_.entries();
    std::error_code ec;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const std::string label = entries[i].filename().string() + "##recent" + std::to_string(i);
        const bool exists = std::filesystem::exists(entries[i], ec);
        if (ImGui::MenuItem(label.c_str(), nullptr, false, exists)) {
            openStlFile(entries[i]);
        }
        tooltip((entries[i].string() + (exists ? "" : "\n(file not found)")).c_str());
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Clear Recent", nullptr, false, !recentFiles_.empty())) {
        recentFiles_.clear();
        if (std::string error; !recentFiles_.save(&error)) {
            controller_.postStatusMessage("Could not save the recent files list: " + error, true);
        }
    }
    ImGui::EndMenu();
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
    tooltip("File units: click to change. They only relabel the numbers; no value or vertex changes. Mesh > Units & Scale has the same setting and coordinate scaling.");
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
        const BoundingBox bbox = controller_.meshBounds();
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
            tooltip("Faces selected with B (box), C (circle) or Shift+click. Decimate, Merge by Distance and Weld work "
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
        const bool fixAll = controller_.fixAllRunning();
        const std::string overlay = fixAll ? (controller_.fixAllCancelling() ? std::string("Cancelling...") : std::string("Fix All..."))
                                    : controller_.globalRepairCancelling() ? std::string("Cancelling...")
                                    : current                              ? std::string(globalRepairMethodLabel(*current)) + "..."
                                                                           : std::string("Repairing...");
        ImGui::ProgressBar(controller_.globalRepairProgress(), ImVec2(barWidth, 0.0f), overlay.c_str());
        if (fixAll) {
            ImGui::SetItemTooltip("%s", controller_.fixAllStatusText().c_str());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel##repair")) {
            controller_.cancelGlobalRepair(); // posts its own status message
        }
    } else if (controller_.checksRunning()) {
        const float barWidth = 200.0f;
        const float cancelWidth = ImGui::CalcTextSize("Cancel").x + 2.0f * ImGui::GetStyle().FramePadding.x;
        ImGui::SameLine(display.x - barWidth - cancelWidth - 3.0f * settings::kPanelMarginPx);
        const std::optional<DefectCategory> current = controller_.currentCheckCategory();
        const std::string overlay = current ? "Check: " + std::string(categoryInfo(*current).name) + "..." : std::string("Checking...");
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

void PolyscopeGui::drawPreferencesWindow(float top, float bottom, float rightColumnLeft) {
    // Open centred in the free 3D view between the two columns so the Tool panel stays visible
    const float margin = settings::kPanelMarginPx;
    const float freeLeft = settings::kLeftColumnWidthPx + 2.0f * margin;
    const float freeRight = rightColumnLeft - margin;
    const float freeWidth = std::max(freeRight - freeLeft, 200.0f);
    const float freeHeight = std::max(bottom - top, 200.0f);
    const ImVec2 size(std::min(settings::kPreferencesWindowWidthPx, freeWidth),
                      std::min(settings::kPreferencesWindowHeightPx, freeHeight));
    const ImVec2 center(freeLeft + freeWidth * 0.5f, top + freeHeight * 0.5f);
    const ImGuiCond cond = layoutResetPending_ ? ImGuiCond_Always : ImGuiCond_Appearing;
    ImGui::SetNextWindowPos(center, cond, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size, cond);
    if (!ImGui::Begin("Preferences", &showPreferences_)) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("PreferencesTabs")) {
        if (ImGui::BeginTabItem("Units & Tolerance")) {
            const ToleranceConfig storedTol = controller_.toleranceConfig();
            ToleranceConfig tol = storedTol;
            bool changed = false;
            ImGui::SeparatorText("Loading");
            ImGui::PushItemWidth(110.0f);
            if (ImGui::InputDouble("Weld floor", &tol.absFloor, 0.0, 0.0, "%.6g")) {
                tol.absFloor = std::max(0.0, tol.absFloor);
                changed = true;
            }
            gui::helpMarker("Absolute vertex-welding tolerance floor used when loading STL files, in file coordinates (the file unit does not change it). Fix All has its own weld (Fix All tab).");
            if (ImGui::InputDouble("Relative tolerance", &tol.relTol, 0.0, 0.0, "%.6g")) {
                tol.relTol = std::max(0.0, tol.relTol);
                changed = true;
            }
            ImGui::PopItemWidth();
            if (changed && (tol.absFloor != storedTol.absFloor || tol.relTol != storedTol.relTol)) {
                controller_.setToleranceConfig(tol);
            }
            const std::string sym(unitSymbol(controller_.fileUnit()));
            ImGui::SeparatorText("Thin / sliver faces defaults");
            const ThinFaceThresholds stored = controller_.preferenceDefaults().thinFace;
            ThinFaceThresholds def = stored;
            if (gui::thinFaceThresholdFields(def, controller_.fileUnit(), 70.0f) &&
                (def.minAltitude != stored.minAltitude || def.minArea != stored.minArea ||
                 def.maxAspectRatio != stored.maxAspectRatio)) {
                controller_.setThinFaceDefaults(def);
                requestSync();
            }
            ImGui::TextDisabled("Press Enter to apply.");
            gui::helpMarker("Editing a default also sets the Thin / sliver faces setting. Changing that setting "
                            "later does not change the default.");
            ImGui::TextDisabled("Active floor: %.0e %s (applies to the next load)",
                                controller_.toleranceConfig().absFloor, sym.c_str());
            if (ImGui::Button("Reset default tolerances")) {
                controller_.setToleranceConfig(ToleranceConfig{});
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Fix All")) {
            drawFixAllPreferencesTab();
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
        if (ImGui::BeginTabItem("Mouse")) {
            drawMousePreferencesTab();
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
    if (ImGui::Button("Reset to defaults")) {
        applyPreferences(UserPreferences{});
        controller_.postStatusMessage("Preferences and Display styles reset to the built-in defaults.");
    }
    tooltip("Restore every preference on these tabs (the Ogun mouse scheme included) and every Display panel style "
            "(colors, widths, preview, background) to its built-in value, and save.");
    if (preferencesFile_.empty()) {
        ImGui::TextDisabled("No home directory: preferences last for this session only.");
    } else {
        ImGui::TextDisabled("Changes are saved automatically to %s", preferencesFile_.string().c_str());
        if (!preferencesSaveError_.empty()) {
            ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.2f, 1.0f), "Save failed: %s", preferencesSaveError_.c_str());
        }
    }
    ImGui::End();
}

void PolyscopeGui::drawFixAllPreferencesTab() {
    const FixAllPreferences stored = controller_.preferenceDefaults().fixAll;
    FixAllPreferences fix = stored;
    ImGui::TextWrapped("Starting values of the Fix All panel. They are separate from the weld done when the file was loaded.");
    ImGui::SeparatorText("Run");
    ImGui::PushItemWidth(110.0f);
    ImGui::InputInt("Passes", &fix.passes);
    gui::helpMarker("Passes of a full Fix All run (1 to 10). It stops earlier when the mesh is clean or a pass changes nothing. "
                    "Fix All, One Pass always runs 1.");
    ImGui::InputDouble("Time budget (s)", &fix.timeBudgetSeconds, 0.0, 0.0, "%.6g");
    gui::helpMarker("No new pass starts once a pass has ended past this many seconds. 0 = no budget.");
    ImGui::SeparatorText("Fix All weld");
    ImGui::Checkbox("Weld by distance first", &fix.weld);
    gui::helpMarker("Merges vertices that are almost on top of each other (cracks) at the start of each pass. Separate from "
                    "the weld done when the file was loaded (Units & Tolerance > Loading).");
    ImGui::InputDouble("Fix All weld floor", &fix.weldTolerance.absFloor, 0.0, 0.0, "%.6g");
    gui::helpMarker("Absolute per-axis tolerance floor of the Fix All weld, in file coordinates. "
                    "Allowed range 1e-9 to 1e-3. Separate from the weld done when the file was loaded.");
    ImGui::InputDouble("Fix All relative tolerance", &fix.weldTolerance.relTol, 0.0, 0.0, "%.6g");
    gui::helpMarker("Per-axis tolerance relative to the larger coordinate on that axis. Allowed range 1e-9 to 1e-3; 0 is "
                    "not allowed. Separate from the weld done when the file was loaded.");
    ImGui::SeparatorText("Steps");
    ImGui::Checkbox("Remove fins / internal walls", &fix.removeFins);
    ImGui::Checkbox("Remove tiny floating shells", &fix.removeTinyShells);
    double percent = fix.tinyShellFraction * 100.0;
    if (ImGui::InputDouble("Tiny shell size (%)", &percent, 0.0, 0.0, "%.6g")) {
        fix.tinyShellFraction = percent / 100.0;
    }
    gui::helpMarker("A floating shell touching nothing is tiny when its bounding-box diagonal is below this share of the "
                    "whole model's bounding-box diagonal (0.1 % by default).");
    ImGui::PopItemWidth();
    ImGui::Checkbox("Close cracks", &fix.closeCracks);
    ImGui::Checkbox("Heal thin faces", &fix.healThinFaces);
    gui::helpMarker("Uses the aspect-ratio limit of the Thin / sliver faces settings.");
    ImGui::TextDisabled("Merge shells is decided for each mesh in the Fix All panel; it is not saved.");
    if (ImGui::Button("Reset Fix All settings")) {
        fix = FixAllPreferences{};
    }
    const FixAllPreferences& s = stored;
    if (fix.passes != s.passes || fix.timeBudgetSeconds != s.timeBudgetSeconds || fix.weld != s.weld ||
        fix.weldTolerance.absFloor != s.weldTolerance.absFloor || fix.weldTolerance.relTol != s.weldTolerance.relTol ||
        fix.removeTinyShells != s.removeTinyShells || fix.tinyShellFraction != s.tinyShellFraction ||
        fix.removeFins != s.removeFins || fix.closeCracks != s.closeCracks || fix.healThinFaces != s.healThinFaces) {
        if (fix.weldTolerance.relTol <= 0.0) {
            fix.weldTolerance.relTol = s.weldTolerance.relTol; // 0 is refused
        }
        controller_.setFixAllDefaults(fix);
    }
}

UserPreferences PolyscopeGui::currentPreferences() const {
    UserPreferences prefs = controller_.userPreferences();
    prefs.uiScale = uiScale_;
    prefs.backgroundColor = polyscope::view::bgColor;
    prefs.showCompass = showCompass_;
    prefs.mouse = mouseMapping_;
    return prefs;
}

void PolyscopeGui::applyPreferences(const UserPreferences& prefs) {
    controller_.applyUserPreferences(prefs);
    uiScale_ = prefs.uiScale;
    polyscope::view::bgColor = prefs.backgroundColor;
    showCompass_ = prefs.showCompass;
    mouseMapping_ = prefs.mouse;
    applyTransparencyModeForPreview();
    applyBaseMeshStyleToPolyscope();
    requestSync();
}

void PolyscopeGui::savePreferencesIfChanged(bool force) {
    // While a slider is dragged or a field typed in, wait for the release
    if (preferencesFile_.empty() || (!force && ImGui::IsAnyItemActive())) {
        return;
    }
    const UserPreferences prefs = currentPreferences();
    if (sameUserPreferences(prefs, savedPreferences_)) {
        return;
    }
    savedPreferences_ = prefs; // also on failure, so a read-only file is not retried every frame
    std::string error;
    if (saveUserPreferences(prefs, preferencesFile_, &error)) {
        preferencesSaveError_.clear();
    } else {
        preferencesSaveError_ = error;
        controller_.postStatusMessage("Could not save preferences: " + error, true);
    }
}

void PolyscopeGui::drawModalPopups() {
    if (requestDeleteConfirm_) {
        requestDeleteConfirm_ = false;
        ImGui::SetNextWindowPos(ImGui::GetMousePos(), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::OpenPopup("DeleteFacesConfirm");
    }
    if (ImGui::BeginPopup("DeleteFacesConfirm")) {
        const std::size_t count = controller_.selectedFaceCount();
        ImGui::Text("Delete %s selected face%s?", gui::withThousands(count).c_str(), count == 1 ? "" : "s");
        if (ImGui::IsWindowAppearing()) {
            ImGui::SetKeyboardFocusHere();
        }
        if (count == 0) {
            ImGui::CloseCurrentPopup(); // selection vanished: nothing left to confirm
        } else if (ImGui::Button("Delete") || ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
                   ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            deleteFacesRequested_ = true; // faces only, never the shell-delete path
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (requestSetOrigin_) {
        ImGui::OpenPopup("Set Origin to Selection");
        requestSetOrigin_ = false;
    }
    if (ImGui::BeginPopupModal("Set Origin to Selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const Eigen::Vector3d& c = pendingOrigin_;
        const std::string unit(unitSymbol(controller_.fileUnit()));
        // + 0.0 turns -0 into 0
        ImGui::Text("Move the selection centroid (%.6g, %.6g, %.6g) %s to the origin?", c.x() + 0.0, c.y() + 0.0,
                    c.z() + 0.0, unit.c_str());
        ImGui::Text("This translates the whole mesh by (%.6g, %.6g, %.6g).", -c.x() + 0.0, -c.y() + 0.0, -c.z() + 0.0);
        ImGui::TextDisabled("One undo step. Hidden shells move too. Enter: OK, Esc: Cancel.");
        // Keyboard navigation is off, so Enter / Esc are read here (the app's shortcuts are paused
        // while a popup is open). Not on the first frame, so the key that opened it cannot answer it.
        const bool appearing = ImGui::IsWindowAppearing();
        const bool enter = !appearing && (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
                                          ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false));
        const bool escape = !appearing && ImGui::IsKeyPressed(ImGuiKey_Escape, false);
        ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor); // the default button
        const bool ok = ImGui::Button("OK");
        ImGui::PopStyleColor();
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
        const bool cancel = ImGui::Button("Cancel") || escape;
        if (cancel) {
            ImGui::CloseCurrentPopup();
        } else if (ok || enter) {
            confirmSetOriginToSelection();
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

void PolyscopeGui::drawAboutWindow() {
    const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("About Ogun", &showAboutWindow_)) {
        ImGui::End();
        return;
    }

    if (ImGui::BeginTabBar("AboutTabs")) {
        if (ImGui::BeginTabItem("Version")) {
            constexpr float kIconSize = 224.0f;
            if (appIconTexture_ != nullptr) {
                const float indent = (ImGui::GetContentRegionAvail().x - kIconSize) * 0.5f;
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, indent));
                ImGui::Image(appIconTexture_->getNativeHandle(), ImVec2(kIconSize, kIconSize));
            }
            const auto centredText = [](const std::string& text) {
                const float indent = (ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(text.c_str()).x) * 0.5f;
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, indent));
                ImGui::TextUnformatted(text.c_str());
            };
            ImGui::Spacing();
            centredText("Ogun");
            centredText("Version " + std::string(kVersionString) + " (" + std::string(kVersionDate) + ")");
            centredText("By Jeremy Sweezy");
            ImGui::Spacing();
            centredText("Licensed under the GNU General Public License v3.0 (GPLv3)");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Credits")) {
            ImGui::TextUnformatted("Author: Jeremy Sweezy");
            ImGui::Spacing();
            ImGui::TextUnformatted("Built on these open-source projects:");
            constexpr std::array<std::pair<const char*, const char*>, 9> kCredits = {{
                {"CGAL", "computational geometry: alpha wrap, hole filling and intersection repair"},
                {"Manifold", "robust mesh booleans and merging of shells"},
                {"libigl", "mesh processing and geometry queries"},
                {"Eigen", "linear algebra"},
                {"Polyscope", "the 3D viewport and application window"},
                {"Dear ImGui", "the user interface widgets"},
                {"Boost", "support libraries used by CGAL"},
                {"portable-file-dialogs", "native open and save dialogs"},
                {"Catch2", "the unit-test framework"},
            }};
            for (const auto& [name, use] : kCredits) {
                ImGui::BulletText("%s: %s", name, use);
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Dependencies")) {
            if (ImGui::BeginTable("AboutDependencies", 2,
                                  ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Version");
                ImGui::TableHeadersRow();
                for (const DependencyInfo& dep : kDependencies) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(dep.name.data(), dep.name.data() + dep.name.size());
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(dep.version.data(), dep.version.data() + dep.version.size());
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Copyright")) {
            const std::string_view notice = copyrightNotice();
            if (ImGui::Button("Copy")) {
                ImGui::SetClipboardText(std::string(notice).c_str());
            }
            ImGui::BeginChild("CopyrightText", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Border,
                              ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::TextUnformatted(notice.data(), notice.data() + notice.size());
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Build Info")) {
            if (ImGui::Button("Copy build info")) {
                ImGui::SetClipboardText(buildInfoText().c_str());
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(for bug reports)");
            ImGui::BeginChild("BuildInfoText", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Border);
            const auto row = [](const char* label, const std::string& value) {
                ImGui::TextDisabled("%s", label);
                ImGui::TextWrapped("%s", value.c_str());
                ImGui::Spacing();
            };
            row("Build date", std::string(kBuildTimestamp));
            row("Build type", std::string(kBuildType));
            row("Compiler", std::string(kCompilerId) + " " + std::string(kCompilerVersion));
            row("C++ standard", std::string(kCxxStandard));
            row("Platform", std::string(kPlatform));
            row("Compiler flags", std::string(kCxxFlags));
            row("Linker flags", kLinkerFlags.empty() ? std::string("(none)") : std::string(kLinkerFlags));
            row("Libraries", librariesText());
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
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

    const auto table = [](const char* id, const char* left, const std::vector<std::pair<std::string, std::string>>& rows) {
        if (ImGui::BeginTable(id, 2, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn(left, ImGuiTableColumnFlags_WidthFixed, 175.0f);
            ImGui::TableSetupColumn("Behavior", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (const auto& [key, behavior] : rows) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(key.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextWrapped("%s", behavior.c_str());
            }
            ImGui::EndTable();
        }
    };

    // The mouse rows show the active scheme (Preferences > Mouse)
    const auto inputs = [this](MouseAction action) { return describeMouseBindings(mouseMapping_, action); };
    ImGui::TextUnformatted("Mouse Controls (3D Viewport)");
    ImGui::TextDisabled("Scheme: %s. Change it in Edit > Preferences > Mouse.", mousePresetName(mouseMapping_.preset));
    table("MouseNavTable", "Action", {
        {"Orbit / Rotate", inputs(MouseAction::Orbit) + " (orbits around the active Orbit Pivot)"},
        {"Pan Camera & Pivot", inputs(MouseAction::Pan) + " (shifts the Orbit Pivot with the camera, scaled by depth)"},
        {"Zoom (Dolly)", inputs(MouseAction::WheelZoom) + "; drag: " + inputs(MouseAction::Zoom) +
                             " (speed scales with distance to the Orbit Pivot)"},
        {"Near Clip Plane", inputs(MouseAction::NearClip) + " shifts the near clip plane (Ogun also adjusts it as you zoom)"},
        {"Select Face", inputs(MouseAction::Select) + " a face: if it is a defect of the selected Mesh Health row it becomes the focus (Inspector, Orbit Pivot at its centroid); with no row selected, the row containing it opens; a clean face only moves the Orbit Pivot. " +
                            inputs(MouseAction::ClearPick) + " clears the Polyscope pick."},
        {"Select Shell", inputs(MouseAction::SelectShell) + " a face selects its whole shell in the Shells panel (multi-shell meshes); with Ctrl/Cmd added it adds or removes it. A plain click never changes the shell selection."},
        {"Toggle Face", inputs(MouseAction::ToggleFace) + " a face adds it to or removes it from the face selection (Decimate, Merge by Distance and Weld then work on the selected faces). Faces a slice plane cuts away are clicked through"},
        {"Drag & Drop STL", "Drag an .stl file from Finder / file manager directly into the 3D viewport to load and inspect it immediately"},
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
        {"View > Compass", "Draws X (red), Y (green), Z (blue) axes in the bottom-left of the 3D view, turning with the camera. Off by default"},
        {"View > Polyscope", "Opens Polyscope's Appearance, Slice Planes, Structures, Camera & Clipping and Render & Debug sections"},
        {"B", "Box select faces: drag a rectangle (Shift+drag deselects); one box per B. Esc or right-click cancels. Only the faces you see, unless Select Through or Ghost (G) is on; hidden shells and faces cut away by a slice plane never"},
        {"C", "Circle select faces: paint with the brush (Shift or middle drag deselects, wheel resizes); Esc, Enter or right-click finishes. As B, never faces cut away by a slice plane"},
        {"A  /  Alt+A", "Select all shown faces (not hidden shells or faces a slice plane cuts away) / deselect all faces"},
        {"Esc", "Leaves box / circle select; clears the defect selection; then closes the Tool and deselects the row, then clears the face selection, then the shell selection"},
        {"I", "Shells: isolate the selected shells (Mesh Health then covers only them); press again to bring the others back. With another shell selected, switches the isolation to it"},
        {"X", "Asks \"Delete N selected faces?\" next to the cursor (Enter or the Delete button confirms; Esc, Cancel or a click outside cancels), then deletes the selected faces as Delete does"},
        {"Delete  /  Backspace", "Immediately (no confirmation) deletes the selected faces (Mesh > Delete Selected Faces); with no faces selected, the selected shells. Undoable"},
        {"F5", "Check Mesh: runs every Mesh Health row in the background"},
        {"Shift + F5", "Fix All...: opens Operation: Fix All with the saved number of passes (3 unless changed in Preferences > Fix All). Follows the selection (faces, else shells, else the whole mesh). Preview first; nothing changes until Apply"},
        {"Alt + Shift + F5", "Fix All, One Pass...: the same panel with Passes = 1 (Option+Shift+F5 on macOS), to step the repair by hand"},
        {"Repair > Remesh & Watertight Rebuild", "Rebuilds a watertight surface when local repairs cannot fix severe defects: Shrink-Wrap (Outer Envelope), Voxel Heal & Remesh (Fast), and Precision Isosurface (FEA Grade). Fix All also includes a Rebuild switch"},
        {"Ctrl/Cmd + Z", "Undoes the last mesh modification (repair, Fix All / Fix Selected, Scale, Transform, Decimate, Merge by Distance, Weld, Delete Faces / Shells), one step at a time. "
                         "Undoing a Scale or Transform re-frames the camera (Home)"},
        {"Ctrl/Cmd + O / S / Shift+S", "Open, Save (overwrite, asks the first time), Save As"},
        {"Ctrl/Cmd + ,", "Preferences"},
        {"?  or  F1", "Toggles this Navigation & Shortcuts window"},
        {"Ctrl+C / Ctrl+V", "Copy / paste the camera pose (Polyscope built-in)"},
    });
    ImGui::End();
}

} // namespace ogun
