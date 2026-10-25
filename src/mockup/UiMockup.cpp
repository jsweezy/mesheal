#include "mockup/UiMockup.hpp"

#include <imgui.h>
#include <polyscope/pick.h>
#include <polyscope/polyscope.h>
#include <polyscope/screenshot.h>
#include <polyscope/structure.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <random>
#include <utility>

namespace ogun::mockup {
namespace {

constexpr const char* kMeshName = "STL Surface Mesh";
constexpr const char* kHighlightName = "Health Highlight";
constexpr float kMargin = 8.0f;
constexpr float kLeftColumnWidth = 290.0f;
constexpr float kDefaultRightColumnWidth = 430.0f;
constexpr float kMinRightColumnWidth = 340.0f;
constexpr float kMaxRightColumnWidth = 900.0f;
constexpr double kFastCheckSeconds = 0.25;
constexpr double kSlowCheckSeconds = 2.5;
constexpr double kStatusMessageSeconds = 8.0;
constexpr float kClickDragThresholdSq = 9.0f;

#ifdef __APPLE__
constexpr const char* kMod = "Cmd";
#else
constexpr const char* kMod = "Ctrl";
#endif

const ImVec4 kOkColor(0.35f, 0.80f, 0.40f, 1.0f);
const ImVec4 kDefectColor(1.00f, 0.62f, 0.20f, 1.0f);
const ImVec4 kRunningColor(0.35f, 0.65f, 1.00f, 1.0f);
const ImVec4 kMutedColor(0.55f, 0.55f, 0.55f, 1.0f);

// Order matches the rows built in the UiMockup constructor
enum CategoryId : std::size_t {
    kOpenEdges,
    kNonManifoldEdges,
    kNonManifoldVertices,
    kFlippedNormals,
    kShells,
    kIntersections,
    kSlivers,
    kDegenerate,
    kDuplicates,
    kCategoryCount,
};

constexpr std::array<const char*, 3> kSliverSubtypes = {"Zero-area", "Needle", "Cap"};
constexpr std::array<const char*, 4> kSliverStrategies = {
    "Flip Longest Edge (Best for Coplanar Caps)",
    "Collapse Shortest Edge (Best for Needles)",
    "Snap Collinear Vertex (Cap on Feature Crease)",
    "Remove Degenerate Flap (Isolated / Redundant Face)",
};
constexpr std::array<const char*, 3> kPreviewModes = {"Cutout + Original Edges", "Translucent Base", "Translucent Patch"};

glm::vec3 toGlm(const Eigen::Vector3d& v) {
    return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

std::string withThousands(std::size_t value) {
    std::string digits = std::to_string(value);
    for (auto pos = static_cast<std::ptrdiff_t>(digits.size()) - 3; pos > 0; pos -= 3) {
        digits.insert(static_cast<std::size_t>(pos), ",");
    }
    return digits;
}

void helpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void tooltip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

void statusDot(HealthStatus status) {
    const float radius = ImGui::GetTextLineHeight() * 0.3f;
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 center(cursor.x + radius + 2.0f, cursor.y + ImGui::GetTextLineHeight() * 0.5f);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    switch (status) {
    case HealthStatus::NotRun:
        draw->AddCircle(center, radius, ImGui::GetColorU32(kMutedColor), 12, 1.5f);
        break;
    case HealthStatus::Running: {
        const auto t = static_cast<float>(ImGui::GetTime());
        const float start = std::fmod(t * 6.0f, 6.2832f);
        draw->PathArcTo(center, radius, start, start + 4.2f, 12);
        draw->PathStroke(ImGui::GetColorU32(kRunningColor), 0, 2.0f);
        break;
    }
    case HealthStatus::Ok:
        draw->AddCircleFilled(center, radius, ImGui::GetColorU32(kOkColor), 12);
        break;
    case HealthStatus::Defects:
        draw->AddCircleFilled(center, radius, ImGui::GetColorU32(kDefectColor), 12);
        break;
    case HealthStatus::Stale:
        draw->AddCircleFilled(center, radius, ImGui::GetColorU32(kMutedColor), 12);
        draw->AddCircle(center, radius + 1.5f, ImGui::GetColorU32(kDefectColor), 12, 1.0f);
        break;
    }
    ImGui::Dummy(ImVec2(radius * 2.0f + 4.0f, ImGui::GetTextLineHeight()));
}

const char* statusLabel(HealthStatus status) {
    switch (status) {
    case HealthStatus::NotRun: return "Not checked yet";
    case HealthStatus::Running: return "Checking...";
    case HealthStatus::Ok: return "No problems found";
    case HealthStatus::Defects: return "Problems found";
    case HealthStatus::Stale: return "Stale: the mesh changed since this slow check last ran";
    }
    return "";
}

void sortWorstFirst(std::vector<MockDefect>& defects) {
    std::stable_sort(defects.begin(), defects.end(), [](const MockDefect& a, const MockDefect& b) {
        return a.metric < b.metric;
    });
}

} // namespace

UiMockup::UiMockup(Mesh mesh, std::string sourceLabel)
    : mesh_(std::move(mesh)), sourceLabel_(std::move(sourceLabel)) {
    if (mesh_.N.rows() != mesh_.F.rows()) {
        mesh_.recomputeNormals();
    }
    bbox_ = mesh_.boundingBox();
    fileUnit_ = mesh_.fileUnit;
    rightColumnWidth_ = kDefaultRightColumnWidth;

    categories_.resize(kCategoryCount);
    const auto define = [this](CategoryId id, const char* name, const char* group, const char* metric,
                               bool slow, const char* tip) {
        HealthCategory& c = categories_[id];
        c.name = name;
        c.group = group;
        c.metricLabel = metric;
        c.slowCheck = slow;
        c.tooltip = tip;
    };
    define(kOpenEdges, "Open edges (holes)", "Topology", "Loop edges", false,
           "Edges used by only one triangle. They form the rims of holes, so the mesh is not watertight.");
    define(kNonManifoldEdges, "Non-manifold edges", "Topology", "Faces on edge", false,
           "Edges shared by three or more triangles (fins, T-junctions). Ray tracers cannot tell inside from outside here.");
    define(kNonManifoldVertices, "Non-manifold vertices", "Topology", "Fans", false,
           "Vertices where two separate fans of triangles touch at a single point (bow-ties).");
    define(kFlippedNormals, "Flipped normals", "Topology", "Bad edges", false,
           "Triangles wound opposite to their neighbours, so their normal points inward.");
    define(kShells, "Shells", "Topology", "Faces", false,
           "Separate connected pieces. Information only: several shells can be intended.");
    define(kIntersections, "Intersecting faces", "Geometry", "Overlap", true,
           "Triangles that pass through other triangles. Slow to check on large meshes, so it is marked stale after edits instead of re-running.");
    define(kSlivers, "Thin / sliver faces", "Geometry", "h_min", false,
           "Triangles with tiny altitude, tiny area or extreme aspect ratio (zero-area, needles and caps).");
    define(kDegenerate, "Degenerate faces", "Geometry", "Area", false,
           "Triangles with exactly zero area or a repeated vertex.");
    define(kDuplicates, "Duplicate faces", "Geometry", "Copies", false,
           "Triangles that use the same three vertices as another triangle.");
    categories_[kShells].infoOnly = true;
    categories_[kShells].infoValue = 1;

    generateFakeDefects();
}

void UiMockup::generateFakeDefects() {
    const Eigen::Index nFaces = mesh_.numFaces();
    if (nFaces == 0) {
        return;
    }
    std::mt19937 rng(20261024u);
    std::vector<Eigen::Index> order(static_cast<std::size_t>(nFaces));
    std::iota(order.begin(), order.end(), Eigen::Index{0});
    std::shuffle(order.begin(), order.end(), rng);
    std::uniform_real_distribution<double> logMetric(-9.0, -3.0);
    std::uniform_int_distribution<int> subtype(0, 2);

    // Disjoint face sets so each category tints its own faces
    std::size_t next = 0;
    const auto take = [&](CategoryId id, std::size_t wanted) {
        const std::size_t count = std::min(wanted, order.size() - next);
        auto& pending = categories_[id].pending;
        pending.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            MockDefect d;
            d.faceIndex = order[next++];
            d.metric = std::pow(10.0, logMetric(rng));
            d.area = d.metric * d.metric * 10.0;
            d.subtype = subtype(rng);
            pending.push_back(d);
        }
        sortWorstFirst(pending);
    };
    take(kOpenEdges, 41);
    take(kNonManifoldEdges, 3);
    take(kNonManifoldVertices, 0);
    take(kFlippedNormals, 0);
    take(kIntersections, std::min<std::size_t>(20000, static_cast<std::size_t>(nFaces) / 3));
    take(kSlivers, 312);
    take(kDegenerate, 12);
    take(kDuplicates, 0);
}

void UiMockup::run() {
    polyscope::options::programName = "Ogun - UI Mockup (fake defect data)";
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::options::buildDefaultGuiPanels = false;
    polyscope::options::openImGuiWindowForUserCallback = false;
    polyscope::init();

    registerMesh();
    polyscope::view::resetCameraToHomeView();
    setStatusMessage("Mockup: defect counts are fake. Press F5 or click 'Check All' to start.");

    polyscope::state::userCallback = [this]() { drawFrame(); };
    polyscope::show();
}

// -----------------------------------------------------------------------------
// Frame & layout
// -----------------------------------------------------------------------------

void UiMockup::drawFrame() {
    handleKeyboardShortcuts();
    handleViewportClick();
    advanceCheckJob();

    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float menuHeight = drawMenuBar();
    const float statusHeight = ImGui::GetFrameHeight() + 2.0f * ImGui::GetStyle().WindowPadding.y;
    const float top = menuHeight + kMargin;
    const float bottom = display.y - statusHeight - kMargin;

    drawLeftColumn(top, bottom);
    drawRightColumn(top, bottom);
    drawStatusBar(display.y - statusHeight, statusHeight);

    if (showPreferences_) {
        drawPreferencesWindow();
    }
    if (showHelp_) {
        drawHelpWindow();
    }
    if (showAbout_) {
        ImGui::OpenPopup("About Ogun");
        showAbout_ = false;
    }
    if (ImGui::BeginPopupModal("About Ogun", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Ogun - STL Mesh Inspection & Repair Tool");
        ImGui::TextDisabled("UI mockup build. See UI_DESIGN.md.");
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void UiMockup::drawLeftColumn(float top, float bottom) {
    float y = top;
    const ImGuiWindowFlags autoFlags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove;

    const auto stackedPanel = [&](const char* title, bool* open, auto&& body) {
        if (!*open || y >= bottom) {
            return;
        }
        ImGui::SetNextWindowPos(ImVec2(kMargin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSizeConstraints(ImVec2(kLeftColumnWidth, 0.0f), ImVec2(kLeftColumnWidth, bottom - y));
        if (ImGui::Begin(title, open, autoFlags)) {
            body();
        }
        y += ImGui::GetWindowHeight() + kMargin;
        ImGui::End();
    };

    stackedPanel("Display", &showDisplay_, [this] { drawDisplayPanelContents(); });
    stackedPanel("Navigate", &showNavigation_, [this] { drawNavigationPanelContents(); });

    // Last panel fills the rest of the column and scrolls
    if (showStructures_ && y < bottom) {
        ImGui::SetNextWindowPos(ImVec2(kMargin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(kLeftColumnWidth, bottom - y), ImGuiCond_Always);
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Structures (advanced)", &showStructures_, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
            drawStructuresPanelContents();
        }
        ImGui::End();
    }
}

void UiMockup::drawRightColumn(float top, float bottom) {
    const float displayWidth = ImGui::GetIO().DisplaySize.x;
    const float width = std::clamp(rightColumnWidth_, kMinRightColumnWidth, kMaxRightColumnWidth);
    const float x = displayWidth - width - kMargin;
    float y = top;

    if (showHealth_) {
        ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
        ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0.0f), ImVec2(width, (bottom - top) * 0.6f));
        if (ImGui::Begin("Mesh Health", &showHealth_, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
            drawHealthPanelContents();
        }
        y += ImGui::GetWindowHeight() + kMargin;
        ImGui::End();
    }

    // Progressive disclosure: the Tool slot only exists once something is picked
    if (!showTool_ || toolSlot_ == ToolSlot::None || y >= bottom - 60.0f) {
        return;
    }

    std::string title;
    switch (toolSlot_) {
    case ToolSlot::DefectCategory: title = std::string("Tool: ") + categories_[static_cast<std::size_t>(selectedCategory_)].name; break;
    case ToolSlot::UnitsAndScale: title = "Operation: Units & Scale"; break;
    case ToolSlot::Decimate: title = "Operation: Decimate"; break;
    case ToolSlot::WeldVertices: title = "Operation: Weld / Merge Vertices"; break;
    case ToolSlot::RemoveDegenerate: title = "Operation: Remove Degenerate Faces"; break;
    case ToolSlot::None: break;
    }
    title += "###ToolSlot";

    // Width is user-resizable from the left edge; height always fills the column
    const float height = bottom - y;
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Appearing);
    ImGui::SetNextWindowSizeConstraints(ImVec2(kMinRightColumnWidth, height), ImVec2(kMaxRightColumnWidth, height));
    bool open = true;
    if (ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse)) {
        if (toolSlot_ == ToolSlot::DefectCategory) {
            drawToolPanelContents();
        } else {
            drawOperatorTool();
        }
    }
    rightColumnWidth_ = ImGui::GetWindowWidth();
    ImGui::End();
    if (!open) {
        toolSlot_ = ToolSlot::None;
        selectedCategory_ = -1;
        refreshFaceColors();
    }
}

float UiMockup::drawMenuBar() {
    float height = ImGui::GetFrameHeight();
    if (!ImGui::BeginMainMenuBar()) {
        return height;
    }
    const std::string undoShortcut = std::string(kMod) + "+Z";
    const std::string openShortcut = std::string(kMod) + "+O";
    const std::string saveShortcut = std::string(kMod) + "+S";
    const std::string saveAsShortcut = std::string(kMod) + "+Shift+S";
    const std::string prefsShortcut = std::string(kMod) + "+,";

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open...", openShortcut.c_str())) mockAction("Open...");
        if (ImGui::BeginMenu("Open Recent")) {
            ImGui::MenuItem(sourceLabel_.c_str(), nullptr, false, false);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Load Sample")) {
            if (ImGui::MenuItem("Defective test cube")) mockAction("Load sample cube");
            if (ImGui::MenuItem("Stanford bunny")) mockAction("Load bunny");
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Save", saveShortcut.c_str())) mockAction("Save");
        if (ImGui::BeginMenu("Save As")) {
            if (ImGui::MenuItem("Binary STL...", saveAsShortcut.c_str())) mockAction("Save As binary STL");
            if (ImGui::MenuItem("ASCII STL...")) mockAction("Save As ASCII STL");
            ImGui::EndMenu();
        }
        ImGui::MenuItem("Export Health Report... (future)", nullptr, false, false);
        ImGui::Separator();
        if (ImGui::MenuItem("Quit")) polyscope::unshow();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
        const std::string undoLabel = undoStack_.empty() ? std::string("Undo") : "Undo " + undoStack_.back().label;
        if (ImGui::MenuItem(undoLabel.c_str(), undoShortcut.c_str(), false, !undoStack_.empty())) undo();
        ImGui::MenuItem("Redo (future)", nullptr, false, false);
        ImGui::MenuItem("Undo History... (future)", nullptr, false, false);
        ImGui::Separator();
        HealthCategory* cat = selectedCategory();
        if (ImGui::BeginMenu("Select", cat != nullptr)) {
            if (ImGui::MenuItem("All of this type")) {
                for (auto& d : cat->defects) d.selected = true;
                refreshFaceColors();
            }
            if (ImGui::MenuItem("None", "Esc")) {
                for (auto& d : cat->defects) d.selected = false;
                focusedRow_ = -1;
                refreshFaceColors();
            }
            if (ImGui::MenuItem("Invert")) {
                for (auto& d : cat->defects) d.selected = !d.selected;
                refreshFaceColors();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Next defect", "N")) stepFocus(+1);
            if (ImGui::MenuItem("Previous defect", "Shift+N")) stepFocus(-1);
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Preferences...", prefsShortcut.c_str())) showPreferences_ = true;
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        ImGui::TextDisabled("Panels");
        ImGui::MenuItem("Mesh Health", nullptr, &showHealth_);
        ImGui::MenuItem("Tool / Operation", nullptr, &showTool_);
        ImGui::MenuItem("Display", nullptr, &showDisplay_);
        ImGui::MenuItem("Navigate", nullptr, &showNavigation_);
        ImGui::MenuItem("Structures (advanced)", nullptr, &showStructures_);
        ImGui::Separator();
        if (ImGui::BeginMenu("Camera")) {
            if (ImGui::MenuItem("Home (frame all)", "Home / H")) polyscope::view::flyToHomeView();
            if (ImGui::MenuItem("Frame focused defect", ".")) {
                if (HealthCategory* c = selectedCategory(); c && focusedRow_ >= 0) {
                    flyToFace(c->defects[static_cast<std::size_t>(focusedRow_)].faceIndex, 1.0);
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Front (+Z)")) flyToAxisView(Eigen::Vector3d::UnitZ());
            if (ImGui::MenuItem("Back (-Z)")) flyToAxisView(-Eigen::Vector3d::UnitZ());
            if (ImGui::MenuItem("Right (+X)")) flyToAxisView(Eigen::Vector3d::UnitX());
            if (ImGui::MenuItem("Left (-X)")) flyToAxisView(-Eigen::Vector3d::UnitX());
            if (ImGui::MenuItem("Top (+Y)")) flyToAxisView(Eigen::Vector3d::UnitY());
            if (ImGui::MenuItem("Bottom (-Y)")) flyToAxisView(-Eigen::Vector3d::UnitY());
            ImGui::Separator();
            const bool ortho = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
            if (ImGui::MenuItem("Orthographic", nullptr, ortho)) {
                polyscope::view::projectionMode = ortho ? polyscope::ProjectionMode::Perspective
                                                        : polyscope::ProjectionMode::Orthographic;
            }
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Ghost Mesh", "G", &ghost_)) applyBaseStyle();
        if (ImGui::MenuItem("Wireframe Edges", nullptr, &showEdges_)) applyBaseStyle();
        ImGui::Separator();
        if (ImGui::MenuItem("Screenshot")) {
            polyscope::screenshot(false);
            setStatusMessage("Saved screenshot_*.png in the working directory.");
        }
        if (ImGui::MenuItem("Reset Layout")) resetLayout();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Mesh")) {
        if (ImGui::MenuItem("Units & Scale...")) openOperator(ToolSlot::UnitsAndScale);
        if (ImGui::BeginMenu("Transform")) {
            ImGui::MenuItem("Center at Origin (future)", nullptr, false, false);
            ImGui::MenuItem("Translate / Rotate / Mirror (future)", nullptr, false, false);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Clean Up")) {
            if (ImGui::MenuItem("Weld / Merge Vertices...")) openOperator(ToolSlot::WeldVertices);
            if (ImGui::MenuItem("Remove Degenerate Faces...")) openOperator(ToolSlot::RemoveDegenerate);
            ImGui::MenuItem("Remove Duplicate Faces (future)", nullptr, false, false);
            ImGui::MenuItem("Remove Small Shells (future)", nullptr, false, false);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simplify")) {
            if (ImGui::MenuItem("Decimate...")) openOperator(ToolSlot::Decimate);
            ImGui::MenuItem("Remesh (future)", nullptr, false, false);
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Mesh Info")) {
            showHealth_ = true;
            showHealthInfo_ = true;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Repair")) {
        if (ImGui::MenuItem("Check All", "F5", false, !jobRunning_)) startCheckAll();
        ImGui::Separator();
        for (std::size_t i = 0; i < categories_.size(); ++i) {
            const HealthCategory& c = categories_[i];
            if (c.infoOnly) {
                continue;
            }
            const std::string label = "Fix All " + std::string(c.name) +
                (c.defects.empty() ? std::string() : " (" + withThousands(c.defects.size()) + ")");
            if (ImGui::MenuItem(label.c_str(), nullptr, false, !c.defects.empty() && !jobRunning_)) {
                fixAll(i);
            }
        }
        ImGui::Separator();
        ImGui::MenuItem("Auto-Repair in Safe Order (future)", nullptr, false, false);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("Navigation & Shortcuts", "F1")) showHelp_ = !showHelp_;
        if (ImGui::MenuItem("About Ogun")) showAbout_ = true;
        ImGui::EndMenu();
    }

    height = ImGui::GetWindowHeight();
    ImGui::EndMainMenuBar();
    return height;
}

void UiMockup::drawStatusBar(float top, float height) {
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
    ImGui::Text("%s%s", sourceLabel_.c_str(), modified_ ? " *" : "");
    tooltip(modified_ ? "Modified since it was loaded (unsaved)." : "Loaded file.");
    divider();

    const std::string unitButton = std::string(unitSymbol(fileUnit_)) + "##UnitButton";
    if (ImGui::SmallButton(unitButton.c_str())) {
        ImGui::OpenPopup("FileUnitsPopup");
    }
    tooltip("File Units: how the numbers in the file are read. Changing this never moves vertices.");
    if (ImGui::BeginPopup("FileUnitsPopup")) {
        ImGui::TextDisabled("File Units (re-interpret, no scaling)");
        for (const Unit u : kAllUnits) {
            if (ImGui::Selectable(std::string(unitDisplayName(u)).c_str(), u == fileUnit_)) {
                fileUnit_ = u;
                setStatusMessage("File units set to " + std::string(unitDisplayName(u)) + "; coordinates unchanged.");
            }
        }
        ImGui::Separator();
        if (ImGui::Selectable("Scale coordinates instead...")) {
            openOperator(ToolSlot::UnitsAndScale);
        }
        ImGui::EndPopup();
    }
    divider();

    ImGui::Text("V %s   T %s",
                withThousands(static_cast<std::size_t>(mesh_.numVertices())).c_str(),
                withThousands(static_cast<std::size_t>(mesh_.numFaces())).c_str());
    divider();

    const std::string sym(unitSymbol(fileUnit_));
    const Eigen::Vector3d ext = bbox_.extents();
    ImGui::Text("%.4g x %.4g x %.4g %s", ext.x(), ext.y(), ext.z(), sym.c_str());
    if (ImGui::BeginItemTooltip()) {
        ImGui::Text("Bounding box (%s)", sym.c_str());
        ImGui::Text("Min: (%.5g, %.5g, %.5g)", bbox_.min.x(), bbox_.min.y(), bbox_.min.z());
        ImGui::Text("Max: (%.5g, %.5g, %.5g)", bbox_.max.x(), bbox_.max.y(), bbox_.max.z());
        ImGui::Text("Diagonal: %.5g", bbox_.diagonal());
        ImGui::EndTooltip();
    }
    divider();

    if (!watertightKnown()) {
        ImGui::TextDisabled("Watertight: ?");
    } else if (isWatertight()) {
        ImGui::TextColored(kOkColor, "Watertight");
    } else {
        ImGui::TextColored(kDefectColor, "Not watertight");
    }
    divider();

    const bool fresh = std::chrono::duration<double>(std::chrono::steady_clock::now() - statusMessageTime_).count() <
                       kStatusMessageSeconds;
    if (fresh && !statusMessage_.empty()) {
        ImGui::TextUnformatted(statusMessage_.c_str());
    } else {
        ImGui::TextDisabled("Ready");
    }

    if (jobRunning_) {
        const float barWidth = 200.0f;
        const float cancelWidth = ImGui::CalcTextSize("Cancel").x + 2.0f * ImGui::GetStyle().FramePadding.x;
        ImGui::SameLine(display.x - barWidth - cancelWidth - 3.0f * kMargin);
        const std::string overlay = std::string(categories_[jobCategory_].name) + "...";
        ImGui::ProgressBar(jobProgress_, ImVec2(barWidth, 0.0f), overlay.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) {
            cancelCheckJob();
        }
    }
    ImGui::End();
}

// -----------------------------------------------------------------------------
// Left column panels
// -----------------------------------------------------------------------------

void UiMockup::drawDisplayPanelContents() {
    ImGui::PushItemWidth(150.0f);
    if (ImGui::CollapsingHeader("Base mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::ColorEdit3("Color", baseColor_, ImGuiColorEditFlags_NoInputs)) refreshFaceColors();
        ImGui::SameLine();
        if (ImGui::Checkbox("Edges", &showEdges_)) applyBaseStyle();
        if (ImGui::Checkbox("Ghost (G)", &ghost_)) applyBaseStyle();
        if (ghost_) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::SliderFloat("##GhostOpacity", &ghostOpacity_, 0.05f, 0.9f, "%.2f")) applyBaseStyle();
        }
    }
    if (ImGui::CollapsingHeader("Defect highlight")) {
        if (ImGui::ColorEdit3("Category", categoryColor_, ImGuiColorEditFlags_NoInputs)) refreshFaceColors();
        ImGui::SameLine();
        if (ImGui::ColorEdit3("Selected", selectedColor_, ImGuiColorEditFlags_NoInputs)) refreshFaceColors();
        static float pointRadius = 3.0f;
        static float edgeWidth = 1.0f;
        ImGui::SliderFloat("Point radius (px)", &pointRadius, 0.5f, 20.0f, "%.1f");
        ImGui::SliderFloat("Edge width (px)", &edgeWidth, 0.25f, 10.0f, "%.2f");
        ImGui::TextDisabled("Tubes/spheres above %d faces: off", overlayCap_);
    }
    if (ImGui::CollapsingHeader("Repair preview")) {
        static bool showPreview = true;
        ImGui::Checkbox("Show 3D repair preview", &showPreview);
        ImGui::Combo("Mode", &previewMode_, kPreviewModes.data(), static_cast<int>(kPreviewModes.size()));
        static float opacity = 0.5f;
        ImGui::SliderFloat(previewMode_ == 0 ? "Orig. edge opacity" : "Opacity", &opacity, 0.05f, 1.0f, "%.2f");
    }
    ImGui::PopItemWidth();
    if (ImGui::SmallButton("Reset styles")) {
        mockAction("Reset styles");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Save as default")) {
        mockAction("Save display defaults");
    }
}

void UiMockup::drawNavigationPanelContents() {
    const float w = (kLeftColumnWidth - 4.0f * kMargin) / 3.0f;
    const ImVec2 size(w, 0.0f);
    if (ImGui::Button("Front", size)) flyToAxisView(Eigen::Vector3d::UnitZ());
    tooltip("Look along -Z.");
    ImGui::SameLine();
    if (ImGui::Button("Top", size)) flyToAxisView(Eigen::Vector3d::UnitY());
    ImGui::SameLine();
    if (ImGui::Button("Right", size)) flyToAxisView(Eigen::Vector3d::UnitX());
    if (ImGui::Button("Back", size)) flyToAxisView(-Eigen::Vector3d::UnitZ());
    ImGui::SameLine();
    if (ImGui::Button("Bottom", size)) flyToAxisView(-Eigen::Vector3d::UnitY());
    ImGui::SameLine();
    if (ImGui::Button("Left", size)) flyToAxisView(-Eigen::Vector3d::UnitX());

    ImGui::Separator();
    if (ImGui::Button("Home", size)) polyscope::view::flyToHomeView();
    tooltip("Frame the whole mesh.  Key: Home or H");
    ImGui::SameLine();
    HealthCategory* cat = selectedCategory();
    const bool hasFocus = cat != nullptr && focusedRow_ >= 0;
    ImGui::BeginDisabled(!hasFocus);
    if (ImGui::Button("+Normal", size) && hasFocus) {
        flyToFace(cat->defects[static_cast<std::size_t>(focusedRow_)].faceIndex, 1.0);
    }
    tooltip("Frame the focused defect from its front.  Key: .  (press again for the back)");
    ImGui::SameLine();
    if (ImGui::Button("-Normal", size) && hasFocus) {
        flyToFace(cat->defects[static_cast<std::size_t>(focusedRow_)].faceIndex, -1.0);
    }
    tooltip("Frame the focused defect from behind.  Key: Shift+.");
    ImGui::EndDisabled();

    ImGui::BeginDisabled(cat == nullptr || cat->defects.empty());
    if (ImGui::Button("< Prev", size)) stepFocus(-1);
    tooltip("Fly to the previous defect of the selected type.  Key: Shift+N");
    ImGui::SameLine();
    if (ImGui::Button("Next >", size)) stepFocus(+1);
    tooltip("Fly to the next defect of the selected type (worst first).  Key: N");
    ImGui::EndDisabled();
    ImGui::SameLine();
    const bool ortho = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
    if (ImGui::Button(ortho ? "Ortho" : "Persp", size)) {
        polyscope::view::projectionMode = ortho ? polyscope::ProjectionMode::Perspective
                                                : polyscope::ProjectionMode::Orthographic;
    }
    tooltip("Toggle perspective / orthographic projection.");
    ImGui::TextDisabled("Hover a button to learn its key.");
}

void UiMockup::drawStructuresPanelContents() {
    ImGui::TextDisabled("Polyscope's own per-structure controls.");
    for (auto& [typeName, structureMap] : polyscope::state::structures) {
        if (structureMap.empty()) {
            continue;
        }
        ImGui::PushID(typeName.c_str());
        const std::string header = typeName + " (" + std::to_string(structureMap.size()) + ")";
        if (ImGui::CollapsingHeader(header.c_str())) {
            for (auto& entry : structureMap) {
                entry.second->buildUI();
            }
        }
        ImGui::PopID();
    }
}

// -----------------------------------------------------------------------------
// Right column: Mesh Health
// -----------------------------------------------------------------------------

void UiMockup::drawHealthPanelContents() {
    ImGui::BeginDisabled(jobRunning_);
    if (ImGui::Button("Check All")) {
        startCheckAll();
    }
    ImGui::EndDisabled();
    tooltip("Run every check in the background.  Key: F5");

    std::size_t issues = 0;
    bool anyRun = false;
    for (const HealthCategory& c : categories_) {
        if (!c.infoOnly) {
            issues += c.defects.size();
            anyRun |= c.status != HealthStatus::NotRun;
        }
    }
    ImGui::SameLine();
    if (jobRunning_) {
        ImGui::TextColored(kRunningColor, "Checking %s... %d%%", categories_[jobCategory_].name,
                           static_cast<int>(jobProgress_ * 100.0f));
    } else if (!anyRun) {
        ImGui::TextDisabled("Not checked yet");
    } else if (issues == 0) {
        ImGui::TextColored(kOkColor, "No problems found");
    } else {
        ImGui::TextColored(kDefectColor, "%s problems", withThousands(issues).c_str());
    }

    const ImGuiTableFlags tableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("HealthTable", 4, tableFlags)) {
        ImGui::TableSetupColumn("##status", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##count", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("##check", ImGuiTableColumnFlags_WidthFixed);
        const char* lastGroup = "";
        for (std::size_t i = 0; i < categories_.size(); ++i) {
            if (std::string(categories_[i].group) != lastGroup) {
                lastGroup = categories_[i].group;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("%s", lastGroup);
            }
            drawHealthRow(i);
        }
        ImGui::EndTable();
    }

    if (selectedCategory_ < 0) {
        ImGui::TextDisabled("Click a row to highlight it and open its tool.");
    }

    ImGui::SetNextItemOpen(showHealthInfo_);
    showHealthInfo_ = ImGui::CollapsingHeader("Mesh Info");
    if (showHealthInfo_) {
        const std::string sym(unitSymbol(fileUnit_));
        const Eigen::Vector3d ext = bbox_.extents();
        ImGui::Text("Vertices %s   Triangles %s",
                    withThousands(static_cast<std::size_t>(mesh_.numVertices())).c_str(),
                    withThousands(static_cast<std::size_t>(mesh_.numFaces())).c_str());
        ImGui::Text("Size: %.4g x %.4g x %.4g %s  (diag %.4g)", ext.x(), ext.y(), ext.z(), sym.c_str(),
                    bbox_.diagonal());
        ImGui::Text("Min: (%.4g, %.4g, %.4g)", bbox_.min.x(), bbox_.min.y(), bbox_.min.z());
        ImGui::Text("Max: (%.4g, %.4g, %.4g)", bbox_.max.x(), bbox_.max.y(), bbox_.max.z());
        ImGui::TextDisabled("Area / volume / Euler characteristic: future");
    }
}

void UiMockup::drawHealthRow(std::size_t i) {
    HealthCategory& c = categories_[i];
    ImGui::TableNextRow();
    ImGui::PushID(static_cast<int>(i));

    ImGui::TableSetColumnIndex(0);
    statusDot(c.status);

    ImGui::TableSetColumnIndex(1);
    const bool selected = selectedCategory_ == static_cast<int>(i);
    if (ImGui::Selectable(c.name, selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
        selectCategory(selected ? -1 : static_cast<int>(i));
    }
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
        ImGui::TextUnformatted(c.tooltip);
        ImGui::TextDisabled("%s", statusLabel(c.status));
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }

    ImGui::TableSetColumnIndex(2);
    std::string count;
    ImVec4 color = kMutedColor;
    switch (c.status) {
    case HealthStatus::NotRun: count = "-"; break;
    case HealthStatus::Running: count = "..."; color = kRunningColor; break;
    case HealthStatus::Ok:
    case HealthStatus::Defects:
        count = withThousands(c.infoOnly ? c.infoValue : c.defects.size());
        color = c.infoOnly ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : (c.defects.empty() ? kOkColor : kDefectColor);
        break;
    case HealthStatus::Stale: count = withThousands(c.defects.size()) + "?"; break;
    }
    const float textWidth = ImGui::CalcTextSize(count.c_str()).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - textWidth);
    ImGui::TextColored(color, "%s", count.c_str());

    ImGui::TableSetColumnIndex(3);
    ImGui::BeginDisabled(jobRunning_);
    if (ImGui::SmallButton("Check")) {
        startCheckOne(i);
    }
    ImGui::EndDisabled();
    tooltip(c.slowCheck ? "Re-run this check (slow on big meshes)." : "Re-run this check.");
    ImGui::PopID();
}

// -----------------------------------------------------------------------------
// Right column: Tool slot
// -----------------------------------------------------------------------------

void UiMockup::drawToolPanelContents() {
    HealthCategory* cat = selectedCategory();
    if (cat != nullptr) {
        drawCategoryTool(*cat);
    }
}

void UiMockup::drawCategoryTool(HealthCategory& category) {
    const auto categoryIndex = static_cast<std::size_t>(selectedCategory_);
    ImGui::TextWrapped("%s", category.tooltip);

    if (category.status == HealthStatus::NotRun) {
        ImGui::TextDisabled("This check has not run yet.");
    } else if (category.status == HealthStatus::Stale) {
        ImGui::TextColored(kDefectColor, "Stale: the mesh changed since this slow check ran.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Recheck now")) startCheckOne(categoryIndex);
    }

    if (ImGui::CollapsingHeader("Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        drawCategorySettings(categoryIndex);
    }

    if (category.infoOnly) {
        ImGui::Text("%zu shell(s).", category.infoValue);
        if (ImGui::Button("Remove shells smaller than... (future)")) mockAction("Remove small shells");
        return;
    }

    // Subtype filter chips (slivers only); a filter can bring the count under the list cap
    if (categoryIndex == kSlivers && !category.defects.empty()) {
        std::array<std::size_t, 3> perType{};
        for (const auto& d : category.defects) ++perType[static_cast<std::size_t>(d.subtype)];
        const std::string all = "All (" + withThousands(category.defects.size()) + ")";
        if (ImGui::RadioButton(all.c_str(), subtypeFilter_ < 0)) subtypeFilter_ = -1;
        for (std::size_t t = 0; t < kSliverSubtypes.size(); ++t) {
            ImGui::SameLine();
            const std::string label = std::string(kSliverSubtypes[t]) + " (" + std::to_string(perType[t]) + ")";
            if (ImGui::RadioButton(label.c_str(), subtypeFilter_ == static_cast<int>(t))) {
                subtypeFilter_ = static_cast<int>(t);
            }
        }
    }

    std::vector<std::size_t> visibleRows;
    visibleRows.reserve(category.defects.size());
    for (std::size_t r = 0; r < category.defects.size(); ++r) {
        if (subtypeFilter_ < 0 || category.defects[r].subtype == subtypeFilter_) {
            visibleRows.push_back(r);
        }
    }
    const std::size_t selectedCount = selectedRowCount(category);

    ImGui::Separator();
    ImGui::BeginDisabled(category.defects.empty() || jobRunning_);
    const std::string fixAllLabel = "Fix All " + withThousands(category.defects.size());
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.25f, 1.0f));
    if (ImGui::Button(fixAllLabel.c_str())) fixAll(categoryIndex);
    ImGui::PopStyleColor();
    tooltip("Repair every defect of this type in one undoable step.");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(selectedCount == 0 || jobRunning_);
    const std::string fixSelLabel = "Fix Selected (" + withThousands(selectedCount) + ")";
    if (ImGui::Button(fixSelLabel.c_str())) fixSelected(categoryIndex);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(jobRunning_);
    if (ImGui::Button("Check")) startCheckOne(categoryIndex);
    ImGui::EndDisabled();

    if (category.defects.empty()) {
        if (category.status == HealthStatus::Ok) {
            ImGui::TextColored(kOkColor, "No %s.", category.name);
        }
        return;
    }

    if (static_cast<int>(visibleRows.size()) <= listCap_) {
        drawDefectTable(category, visibleRows);
    } else {
        drawDefectSummary(category, visibleRows);
    }

    if (static_cast<int>(category.defects.size()) > overlayCap_) {
        ImGui::TextDisabled("3D: face tint only (%s faces > overlay cap %d).",
                            withThousands(category.defects.size()).c_str(), overlayCap_);
    }

    if (focusedRow_ >= 0 && static_cast<std::size_t>(focusedRow_) < category.defects.size()) {
        drawInspector(category);
    }
}

void UiMockup::drawCategorySettings(std::size_t categoryIndex) {
    const std::string sym(unitSymbol(fileUnit_));
    ImGui::PushItemWidth(110.0f);
    switch (categoryIndex) {
    case kSlivers:
        ImGui::InputDouble(("Min altitude h_min (" + sym + ")").c_str(), &sliverMinAltitude_, 0.0, 0.0, "%.0e");
        ImGui::InputDouble(("Min area (" + sym + "^2)").c_str(), &sliverMinArea_, 0.0, 0.0, "%.0e");
        ImGui::InputDouble("Max aspect ratio", &sliverMaxAspect_, 0.0, 0.0, "%.0f");
        break;
    case kIntersections:
        ImGui::Checkbox("Ignore faces that only touch along a shared edge", &intersectIgnoreAdjacent_);
        ImGui::InputDouble(("Tolerance (" + sym + ")").c_str(), &intersectTolerance_, 0.0, 0.0, "%.0e");
        break;
    case kOpenEdges: {
        ImGui::InputInt("Max hole size to fill (edges)", &holeMaxEdges_);
        constexpr std::array<const char*, 3> methods = {"Flat (triangulate)", "Smooth (refine)", "Fair (curvature)"};
        ImGui::Combo("Fill method", &holeFillMethod_, methods.data(), static_cast<int>(methods.size()));
        break;
    }
    case kNonManifoldEdges:
    case kNonManifoldVertices: {
        constexpr std::array<const char*, 2> strategies = {"Split vertices", "Remove smallest fin faces"};
        ImGui::Combo("Strategy", &nonManifoldStrategy_, strategies.data(), static_cast<int>(strategies.size()));
        break;
    }
    case kDegenerate:
        ImGui::InputDouble(("Zero-area epsilon (" + sym + "^2)").c_str(), &degenerateAreaEps_, 0.0, 0.0, "%.0e");
        break;
    case kFlippedNormals:
        ImGui::TextDisabled("Orients every shell consistently, outward.");
        break;
    default:
        ImGui::TextDisabled("No settings.");
        break;
    }
    ImGui::PopItemWidth();
}

void UiMockup::drawDefectTable(HealthCategory& category, const std::vector<std::size_t>& visibleRows) {
    const ImGuiIO& io = ImGui::GetIO();
    const bool showSubtype = selectedCategory_ == static_cast<int>(kSlivers);
    const float inspectorReserve = focusedRow_ >= 0 ? 230.0f : 40.0f;
    const float tableHeight = std::max(120.0f, ImGui::GetContentRegionAvail().y - inspectorReserve);
    const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                                  ImGuiTableFlags_Sortable | ImGuiTableFlags_Resizable;
    const int columns = showSubtype ? 4 : 3;
    if (!ImGui::BeginTable("DefectTable", columns, flags, ImVec2(0.0f, tableHeight))) {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Face", ImGuiTableColumnFlags_WidthFixed, 0.0f, 0);
    ImGui::TableSetupColumn(category.metricLabel, ImGuiTableColumnFlags_DefaultSort, 0.0f, 1);
    ImGui::TableSetupColumn("Area", ImGuiTableColumnFlags_None, 0.0f, 2);
    if (showSubtype) {
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_None, 0.0f, 3);
    }
    ImGui::TableHeadersRow();

    if (ImGuiTableSortSpecs* specs = ImGui::TableGetSortSpecs(); specs && specs->SpecsDirty && specs->SpecsCount > 0) {
        const ImGuiTableColumnSortSpecs& s = specs->Specs[0];
        const bool asc = s.SortDirection == ImGuiSortDirection_Ascending;
        const Eigen::Index focusedFace =
            focusedRow_ >= 0 ? category.defects[static_cast<std::size_t>(focusedRow_)].faceIndex : -1;
        std::stable_sort(category.defects.begin(), category.defects.end(),
                         [&](const MockDefect& a, const MockDefect& b) {
                             double ka = 0.0;
                             double kb = 0.0;
                             switch (s.ColumnUserID) {
                             case 0: ka = static_cast<double>(a.faceIndex); kb = static_cast<double>(b.faceIndex); break;
                             case 1: ka = a.metric; kb = b.metric; break;
                             case 2: ka = a.area; kb = b.area; break;
                             default: ka = a.subtype; kb = b.subtype; break;
                             }
                             return asc ? ka < kb : ka > kb;
                         });
        if (focusedFace >= 0) {
            const auto it = std::find_if(category.defects.begin(), category.defects.end(),
                                         [focusedFace](const MockDefect& d) { return d.faceIndex == focusedFace; });
            focusedRow_ = it - category.defects.begin();
        }
        specs->SpecsDirty = false;
        // Rows were reordered under the visible-row list; redraw next frame
        ImGui::EndTable();
        return;
    }

    bool selectionChanged = false;
    for (std::size_t pos = 0; pos < visibleRows.size(); ++pos) {
        const std::size_t row = visibleRows[pos];
        MockDefect& d = category.defects[row];
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushID(static_cast<int>(row));
        char label[32];
        std::snprintf(label, sizeof(label), "#%lld", static_cast<long long>(d.faceIndex));
        if (ImGui::Selectable(label, d.selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
            const bool toggle = io.KeyCtrl || io.KeySuper;
            if (io.KeyShift && selectionAnchor_ >= 0) {
                const auto anchorIt = std::find(visibleRows.begin(), visibleRows.end(), static_cast<std::size_t>(selectionAnchor_));
                const auto anchorPos = static_cast<std::size_t>(anchorIt - visibleRows.begin());
                const std::size_t lo = std::min(anchorPos, pos);
                const std::size_t hi = std::min(std::max(anchorPos, pos), visibleRows.size() - 1);
                if (!toggle) {
                    for (auto& other : category.defects) other.selected = false;
                }
                for (std::size_t p = lo; p <= hi; ++p) category.defects[visibleRows[p]].selected = true;
            } else if (toggle) {
                d.selected = !d.selected;
                selectionAnchor_ = static_cast<std::ptrdiff_t>(row);
            } else {
                for (auto& other : category.defects) other.selected = false;
                d.selected = true;
                selectionAnchor_ = static_cast<std::ptrdiff_t>(row);
            }
            focusedRow_ = static_cast<std::ptrdiff_t>(row);
            selectionChanged = true;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                flyToFace(d.faceIndex, 1.0);
            }
        }
        if (scrollToFocus_ && static_cast<std::ptrdiff_t>(row) == focusedRow_) {
            ImGui::SetScrollHereY(0.5f);
            scrollToFocus_ = false;
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.2e", d.metric);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.2e", d.area);
        if (showSubtype) {
            ImGui::TableSetColumnIndex(3);
            ImGui::TextUnformatted(kSliverSubtypes[static_cast<std::size_t>(d.subtype)]);
        }
        ImGui::PopID();
    }
    ImGui::EndTable();
    if (selectionChanged) {
        refreshFaceColors();
    }

    ImGui::TextDisabled("%zu listed. Click, %s+click, Shift+click; double-click to fly.", visibleRows.size(), kMod);
}

void UiMockup::drawDefectSummary(HealthCategory& category, const std::vector<std::size_t>& visibleRows) {
    ImGui::Spacing();
    ImGui::TextWrapped(
        "%s defects are highlighted in 3D. That is more than the list cap (%d), so no list is shown. "
        "Use Fix All, click a highlighted face in the 3D view, or step through them worst first with "
        "Next / Prev (N / Shift+N). The list appears once fewer than %d remain.",
        withThousands(visibleRows.size()).c_str(), listCap_, listCap_);

    // Distribution of the metric so the user can judge the problem without a list
    std::array<float, 24> histogram{};
    for (const std::size_t row : visibleRows) {
        const double logValue = std::log10(std::max(category.defects[row].metric, 1.0e-12));
        const double t = std::clamp((logValue + 9.0) / 6.0, 0.0, 0.999);
        histogram[static_cast<std::size_t>(t * static_cast<double>(histogram.size()))] += 1.0f;
    }
    const std::string histLabel = std::string("log10 ") + category.metricLabel + ": -9 .. -3";
    ImGui::PlotHistogram("##MetricHistogram", histogram.data(), static_cast<int>(histogram.size()), 0,
                         histLabel.c_str(), 0.0f, FLT_MAX, ImVec2(-1.0f, 70.0f));
    if (!visibleRows.empty()) {
        ImGui::Text("Worst %s: %.2e   Best: %.2e", category.metricLabel,
                    category.defects[visibleRows.front()].metric, category.defects[visibleRows.back()].metric);
    }

    if (ImGui::Button("< Prev")) stepFocus(-1);
    ImGui::SameLine();
    if (ImGui::Button("Next >")) stepFocus(+1);
    ImGui::SameLine();
    if (focusedRow_ >= 0) {
        ImGui::Text("Defect %s of %s (worst first)", withThousands(static_cast<std::size_t>(focusedRow_) + 1).c_str(),
                    withThousands(category.defects.size()).c_str());
    } else {
        ImGui::TextDisabled("None focused");
    }
}

void UiMockup::drawInspector(const HealthCategory& category) {
    const MockDefect& d = category.defects[static_cast<std::size_t>(focusedRow_)];
    const std::string header = "Inspector: Face #" + std::to_string(d.faceIndex) + "###Inspector";
    if (!ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::Text("%s = %.3e   Area = %.3e", category.metricLabel, d.metric, d.area);
    if (selectedCategory_ == static_cast<int>(kSlivers)) {
        ImGui::Text("Detected type: %s", kSliverSubtypes[static_cast<std::size_t>(d.subtype)]);
        for (std::size_t s = 0; s < kSliverStrategies.size(); ++s) {
            ImGui::RadioButton(kSliverStrategies[s], &sliverStrategy_, static_cast<int>(s));
        }
    } else {
        ImGui::TextDisabled("Per-defect repair strategies for this type go here.");
    }
    if (ImGui::Button("Commit Repair")) {
        fixSelected(static_cast<std::size_t>(selectedCategory_));
    }
    ImGui::SameLine();
    if (ImGui::Button("Frame (.)")) flyToFace(d.faceIndex, 1.0);
    ImGui::SameLine();
    if (ImGui::Button("Back side (Shift+.)")) flyToFace(d.faceIndex, -1.0);
}

void UiMockup::drawOperatorTool() {
    const std::string sym(unitSymbol(fileUnit_));
    ImGui::PushItemWidth(150.0f);
    switch (toolSlot_) {
    case ToolSlot::UnitsAndScale: {
        ImGui::TextWrapped("File Units only re-interpret the numbers. Scaling multiplies every vertex coordinate.");
        ImGui::Text("Current file units: %s", std::string(unitDisplayName(fileUnit_)).c_str());
        std::array<const char*, kAllUnits.size()> names{};
        std::array<std::string, kAllUnits.size()> storage;
        for (std::size_t i = 0; i < kAllUnits.size(); ++i) {
            storage[i] = std::string(unitDisplayName(kAllUnits[i]));
            names[i] = storage[i].c_str();
        }
        ImGui::Combo("Scale to", &scaleTargetUnit_, names.data(), static_cast<int>(names.size()));
        const double factor = unitConversionFactor(fileUnit_, kAllUnits[static_cast<std::size_t>(scaleTargetUnit_)]);
        ImGui::Text("Coordinate multiplier: %.6g", factor);
        break;
    }
    case ToolSlot::Decimate: {
        ImGui::TextWrapped("Reduce the triangle count while keeping the shape. Not a repair: it changes geometry.");
        ImGui::SliderFloat("Keep ratio", &decimateRatio_, 0.01f, 1.0f, "%.2f");
        ImGui::Text("Target: %s triangles",
                    withThousands(static_cast<std::size_t>(static_cast<double>(mesh_.numFaces()) * decimateRatio_)).c_str());
        ImGui::Checkbox("Preserve open boundaries", &decimatePreserveBoundary_);
        ImGui::Checkbox("Preserve sharp features", &decimatePreserveFeatures_);
        break;
    }
    case ToolSlot::WeldVertices:
        ImGui::TextWrapped("Merge vertices closer than the tolerance (manifold-style; never collapses a triangle onto itself).");
        ImGui::InputDouble(("Tolerance (" + sym + ")").c_str(), &weldTolerance_, 0.0, 0.0, "%.0e");
        break;
    case ToolSlot::RemoveDegenerate:
        ImGui::TextWrapped("Delete zero-area triangles and close the gap they leave.");
        ImGui::InputDouble(("Zero-area epsilon (" + sym + "^2)").c_str(), &degenerateAreaEps_, 0.0, 0.0, "%.0e");
        break;
    default:
        break;
    }
    ImGui::PopItemWidth();

    ImGui::Separator();
    static bool livePreview = true;
    ImGui::Checkbox("Live preview", &livePreview);
    if (ImGui::Button("Apply")) {
        modified_ = true;
        pushUndo("Mesh operation");
        mockAction("Apply operation (undoable)");
        toolSlot_ = ToolSlot::None;
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        toolSlot_ = ToolSlot::None;
    }
    ImGui::TextDisabled("Every Apply is one undo step (%s+Z).", kMod);
}

// -----------------------------------------------------------------------------
// Floating windows
// -----------------------------------------------------------------------------

void UiMockup::drawPreferencesWindow() {
    ImGui::SetNextWindowSize(ImVec2(460.0f, 320.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Preferences", &showPreferences_)) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("PrefsTabs")) {
        if (ImGui::BeginTabItem("Units & Tolerance")) {
            static double floorCm = 1.0e-9;
            static double relTol = 1.0e-9;
            ImGui::InputDouble("Atomic floor (cm)", &floorCm, 0.0, 0.0, "%.0e");
            ImGui::InputDouble("Relative tolerance", &relTol, 0.0, 0.0, "%.0e");
            ImGui::TextDisabled("Default file units for new files: mm");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Diagnostics")) {
            ImGui::TextDisabled("Default thresholds for every check (same fields as each tool's Settings).");
            ImGui::InputDouble("Sliver h_min", &sliverMinAltitude_, 0.0, 0.0, "%.0e");
            static bool autoCheckOnLoad = false;
            ImGui::Checkbox("Run Check All after loading a file", &autoCheckOnLoad);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Display")) {
            ImGui::ColorEdit3("Category highlight", categoryColor_);
            ImGui::ColorEdit3("Selected highlight", selectedColor_);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Interface")) {
            ImGui::SliderFloat("UI scale", &ImGui::GetIO().FontGlobalScale, 0.75f, 2.0f, "%.2f");
            ImGui::InputInt("Defect list cap", &listCap_);
            listCap_ = std::clamp(listCap_, 5, 1000);
            helpMarker("A defect type with more defects than this shows a summary and 3D highlight instead of a list.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Performance")) {
            ImGui::InputInt("3D overlay cap", &overlayCap_);
            overlayCap_ = std::max(overlayCap_, 0);
            helpMarker("Above this many faces, highlights use face tint only (no edge tubes or vertex spheres).");
            static int threads = 4;
            ImGui::SliderInt("Background check threads", &threads, 1, 8);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::Separator();
    if (ImGui::Button("Save as Defaults")) mockAction("Save preferences file");
    ImGui::SameLine();
    if (ImGui::Button("Restore Factory Defaults")) mockAction("Restore defaults");
    ImGui::End();
}

void UiMockup::drawHelpWindow() {
    ImGui::SetNextWindowSize(ImVec2(440.0f, 380.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Navigation & Shortcuts", &showHelp_)) {
        ImGui::End();
        return;
    }
    const std::array<std::pair<const char*, const char*>, 16> rows = {{
        {"Left drag", "Orbit"},
        {"Right / Shift+Left drag", "Pan"},
        {"Scroll", "Zoom"},
        {"Left click face", "Focus that defect (if it belongs to the selected type)"},
        {".", "Frame focused defect (+Normal; again for -Normal)"},
        {"Shift+.", "Frame focused defect from behind"},
        {"Home / H", "Frame whole mesh"},
        {"N / Shift+N", "Next / previous defect (worst first)"},
        {"F5", "Check All"},
        {"G", "Toggle ghost mesh"},
        {"Esc", "Clear selection / close tool"},
        {"Ctrl/Cmd+Z", "Undo"},
        {"Ctrl/Cmd+O / S", "Open / Save"},
        {"Ctrl/Cmd+,", "Preferences"},
        {"F1 or ?", "This window"},
        {"Ctrl+C / Ctrl+V", "Copy / paste camera pose (Polyscope)"},
    }};
    if (ImGui::BeginTable("Keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        for (const auto& [key, action] : rows) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(key);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(action);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// -----------------------------------------------------------------------------
// Input
// -----------------------------------------------------------------------------

void UiMockup::handleKeyboardShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) {
        return;
    }
    const bool mod = io.KeyCtrl || io.KeySuper;
    if (mod && ImGui::IsKeyPressed(ImGuiKey_Z, false)) undo();
    if (mod && ImGui::IsKeyPressed(ImGuiKey_O, false)) mockAction("Open...");
    if (mod && ImGui::IsKeyPressed(ImGuiKey_S, false)) mockAction(io.KeyShift ? "Save As..." : "Save");
    if (mod && ImGui::IsKeyPressed(ImGuiKey_Comma, false)) showPreferences_ = true;
    if (mod) {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false) && !jobRunning_) startCheckAll();
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false) || (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Slash, false))) {
        showHelp_ = !showHelp_;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_N, false)) stepFocus(io.KeyShift ? -1 : +1);
    if (ImGui::IsKeyPressed(ImGuiKey_G, false)) {
        ghost_ = !ghost_;
        applyBaseStyle();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false) || ImGui::IsKeyPressed(ImGuiKey_H, false)) {
        polyscope::view::flyToHomeView();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Period, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false)) {
        if (HealthCategory* c = selectedCategory(); c && focusedRow_ >= 0) {
            flyToFace(c->defects[static_cast<std::size_t>(focusedRow_)].faceIndex, io.KeyShift ? -1.0 : 1.0);
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (toolSlot_ != ToolSlot::DefectCategory && toolSlot_ != ToolSlot::None) {
            toolSlot_ = ToolSlot::None;
        } else if (HealthCategory* c = selectedCategory(); c && (focusedRow_ >= 0 || selectedRowCount(*c) > 0)) {
            for (auto& d : c->defects) d.selected = false;
            focusedRow_ = -1;
            refreshFaceColors();
        } else {
            selectCategory(-1);
        }
    }
}

void UiMockup::handleViewportClick() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse || !ImGui::IsMouseReleased(ImGuiMouseButton_Left) ||
        io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] > kClickDragThresholdSq) {
        return;
    }
    HealthCategory* cat = selectedCategory();
    if (cat == nullptr) {
        return;
    }
    const ImVec2 p = ImGui::GetMousePos();
    const polyscope::PickResult pick = polyscope::pickAtScreenCoords(glm::vec2{p.x, p.y});
    if (!pick.isHit || pick.structureName != kMeshName) {
        return;
    }
    const auto nVerts = static_cast<std::uint64_t>(mesh_.numVertices());
    if (pick.localIndex < nVerts || pick.localIndex >= nVerts + static_cast<std::uint64_t>(mesh_.numFaces())) {
        return;
    }
    const auto face = static_cast<Eigen::Index>(pick.localIndex - nVerts);
    const auto it = std::find_if(cat->defects.begin(), cat->defects.end(),
                                 [face](const MockDefect& d) { return d.faceIndex == face; });
    if (it == cat->defects.end()) {
        setStatusMessage("Face #" + std::to_string(face) + " is not one of the " + cat->name + ".");
        return;
    }
    for (auto& d : cat->defects) d.selected = false;
    focusDefectRow(static_cast<std::size_t>(it - cat->defects.begin()), false);
}

// -----------------------------------------------------------------------------
// Simulated checks, fixes and undo
// -----------------------------------------------------------------------------

void UiMockup::startCheckAll() {
    jobRunning_ = true;
    jobSingle_ = false;
    jobCategory_ = 0;
    jobProgress_ = 0.0f;
    jobCategoryStart_ = std::chrono::steady_clock::now();
    categories_[0].status = HealthStatus::Running;
}

void UiMockup::startCheckOne(std::size_t categoryIndex) {
    jobRunning_ = true;
    jobSingle_ = true;
    jobCategory_ = categoryIndex;
    jobProgress_ = 0.0f;
    jobCategoryStart_ = std::chrono::steady_clock::now();
    categories_[categoryIndex].status = HealthStatus::Running;
}

void UiMockup::advanceCheckJob() {
    if (!jobRunning_) {
        return;
    }
    HealthCategory& c = categories_[jobCategory_];
    const double duration = c.slowCheck ? kSlowCheckSeconds : kFastCheckSeconds;
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - jobCategoryStart_).count();
    const float within = static_cast<float>(std::min(elapsed / duration, 1.0));
    jobProgress_ = jobSingle_ ? within
                              : (static_cast<float>(jobCategory_) + within) / static_cast<float>(categories_.size());
    if (elapsed < duration) {
        return;
    }

    // The first check "discovers" the pre-generated defects; later checks keep what fixes left
    if (!c.pending.empty()) {
        c.defects = std::move(c.pending);
        c.pending.clear();
    }
    c.status = (c.infoOnly || c.defects.empty()) ? HealthStatus::Ok : HealthStatus::Defects;
    if (static_cast<int>(jobCategory_) == selectedCategory_) {
        refreshFaceColors();
    }

    if (jobSingle_ || jobCategory_ + 1 >= categories_.size()) {
        jobRunning_ = false;
        setStatusMessage(jobSingle_ ? std::string(c.name) + " checked." : "Check All finished.");
        return;
    }
    ++jobCategory_;
    categories_[jobCategory_].status = HealthStatus::Running;
    jobCategoryStart_ = std::chrono::steady_clock::now();
}

void UiMockup::cancelCheckJob() {
    if (!jobRunning_) {
        return;
    }
    HealthCategory& c = categories_[jobCategory_];
    c.status = c.defects.empty() && c.pending.empty() && c.status == HealthStatus::Running && !c.slowCheck
        ? HealthStatus::NotRun
        : HealthStatus::Stale;
    if (!c.pending.empty()) {
        c.status = HealthStatus::NotRun;
    }
    jobRunning_ = false;
    setStatusMessage(std::string("Cancelled while checking ") + c.name + ".");
}

void UiMockup::selectCategory(int categoryIndex) {
    selectedCategory_ = categoryIndex;
    toolSlot_ = categoryIndex >= 0 ? ToolSlot::DefectCategory : ToolSlot::None;
    focusedRow_ = -1;
    selectionAnchor_ = -1;
    subtypeFilter_ = -1;
    refreshFaceColors();
}

void UiMockup::fixAll(std::size_t categoryIndex) {
    HealthCategory& c = categories_[categoryIndex];
    const std::size_t count = c.defects.size();
    pushUndo(std::string("Fix All ") + c.name);
    c.defects.clear();
    c.status = HealthStatus::Ok;
    modified_ = true;
    bool markedStale = false;
    for (std::size_t i = 0; i < categories_.size(); ++i) {
        if (i != categoryIndex && categories_[i].slowCheck && categories_[i].status != HealthStatus::NotRun) {
            categories_[i].status = HealthStatus::Stale;
            markedStale = true;
        }
    }
    focusedRow_ = -1;
    refreshFaceColors();
    setStatusMessage("Fixed " + withThousands(count) + " " + c.name + " (mockup). Fast checks re-ran" +
                     (markedStale ? "; slow checks marked stale." : "."));
}

void UiMockup::fixSelected(std::size_t categoryIndex) {
    HealthCategory& c = categories_[categoryIndex];
    if (focusedRow_ >= 0 && selectedRowCount(c) == 0) {
        c.defects[static_cast<std::size_t>(focusedRow_)].selected = true;
    }
    const std::size_t before = c.defects.size();
    pushUndo(std::string("Fix Selected ") + c.name);
    std::erase_if(c.defects, [](const MockDefect& d) { return d.selected; });
    if (c.defects.empty()) {
        c.status = HealthStatus::Ok;
    }
    modified_ = true;
    for (HealthCategory& other : categories_) {
        if (&other != &c && other.slowCheck && other.status != HealthStatus::NotRun) {
            other.status = HealthStatus::Stale;
        }
    }
    focusedRow_ = -1;
    selectionAnchor_ = -1;
    refreshFaceColors();
    setStatusMessage("Repaired " + std::to_string(before - c.defects.size()) + " face(s) (mockup).");
}

void UiMockup::pushUndo(const std::string& label) {
    UndoSnapshot snap;
    snap.label = label;
    snap.defects.reserve(categories_.size());
    snap.statuses.reserve(categories_.size());
    for (const HealthCategory& c : categories_) {
        snap.defects.push_back(c.defects);
        snap.statuses.push_back(c.status);
    }
    undoStack_.push_back(std::move(snap));
}

void UiMockup::undo() {
    if (undoStack_.empty()) {
        return;
    }
    UndoSnapshot snap = std::move(undoStack_.back());
    undoStack_.pop_back();
    for (std::size_t i = 0; i < categories_.size(); ++i) {
        categories_[i].defects = std::move(snap.defects[i]);
        categories_[i].status = snap.statuses[i];
    }
    modified_ = !undoStack_.empty();
    focusedRow_ = -1;
    refreshFaceColors();
    setStatusMessage("Undid: " + snap.label);
}

void UiMockup::stepFocus(int direction) {
    HealthCategory* cat = selectedCategory();
    if (cat == nullptr || cat->defects.empty()) {
        return;
    }
    const auto n = static_cast<std::ptrdiff_t>(cat->defects.size());
    std::ptrdiff_t row = focusedRow_ < 0 ? (direction > 0 ? 0 : n - 1) : (focusedRow_ + direction + n) % n;
    // Respect the subtype filter while stepping
    for (std::ptrdiff_t guard = 0; guard < n && subtypeFilter_ >= 0 &&
                                   cat->defects[static_cast<std::size_t>(row)].subtype != subtypeFilter_;
         ++guard) {
        row = (row + direction + n) % n;
    }
    for (auto& d : cat->defects) d.selected = false;
    focusDefectRow(static_cast<std::size_t>(row), true);
}

void UiMockup::focusDefectRow(std::size_t row, bool flyCamera) {
    HealthCategory* cat = selectedCategory();
    if (cat == nullptr || row >= cat->defects.size()) {
        return;
    }
    cat->defects[row].selected = true;
    focusedRow_ = static_cast<std::ptrdiff_t>(row);
    selectionAnchor_ = focusedRow_;
    scrollToFocus_ = true;
    refreshFaceColors();
    if (flyCamera) {
        flyToFace(cat->defects[row].faceIndex, 1.0);
    }
}

void UiMockup::openOperator(ToolSlot slot) {
    selectedCategory_ = -1;
    toolSlot_ = slot;
    showTool_ = true;
    refreshFaceColors();
}

void UiMockup::setStatusMessage(std::string message) {
    statusMessage_ = std::move(message);
    statusMessageTime_ = std::chrono::steady_clock::now();
}

void UiMockup::mockAction(const char* what) {
    setStatusMessage(std::string(what) + " (mockup: not wired)");
}

// -----------------------------------------------------------------------------
// Polyscope binding
// -----------------------------------------------------------------------------

void UiMockup::registerMesh() {
    auto* psMesh = polyscope::registerSurfaceMesh(kMeshName, mesh_.V, mesh_.F);
    psMesh->setSmoothShade(false);
    psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Different);
    applyBaseStyle();
    refreshFaceColors();
}

void UiMockup::refreshFaceColors() {
    if (!polyscope::hasSurfaceMesh(kMeshName)) {
        return;
    }
    const Eigen::RowVector3d base(baseColor_[0], baseColor_[1], baseColor_[2]);
    const Eigen::RowVector3d category(categoryColor_[0], categoryColor_[1], categoryColor_[2]);
    const Eigen::RowVector3d selected(selectedColor_[0], selectedColor_[1], selectedColor_[2]);
    Eigen::MatrixXd colors = base.replicate(mesh_.numFaces(), 1);
    if (const HealthCategory* cat = selectedCategory()) {
        for (const MockDefect& d : cat->defects) {
            colors.row(d.faceIndex) = d.selected ? selected : category;
        }
    }
    polyscope::getSurfaceMesh(kMeshName)->addFaceColorQuantity(kHighlightName, colors)->setEnabled(true);
}

void UiMockup::applyBaseStyle() {
    if (!polyscope::hasSurfaceMesh(kMeshName)) {
        return;
    }
    auto* psMesh = polyscope::getSurfaceMesh(kMeshName);
    psMesh->setEdgeWidth(showEdges_ ? 1.0 : 0.0);
    psMesh->setEdgeColor(glm::vec3{0.18f, 0.20f, 0.24f});
    if (ghost_ && polyscope::options::transparencyMode == polyscope::TransparencyMode::None) {
        polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
    }
    psMesh->setTransparency(ghost_ ? ghostOpacity_ : 1.0f);
}

void UiMockup::flyToFace(Eigen::Index faceIndex, double normalSign) {
    if (faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return;
    }
    const Eigen::Vector3d centroid =
        (mesh_.V.row(mesh_.F(faceIndex, 0)) + mesh_.V.row(mesh_.F(faceIndex, 1)) + mesh_.V.row(mesh_.F(faceIndex, 2)))
            .transpose() / 3.0;
    Eigen::Vector3d normal = mesh_.N.row(faceIndex).transpose();
    if (normal.squaredNorm() < 0.5) {
        normal = Eigen::Vector3d::UnitZ();
    }
    const double distance = std::max(bbox_.diagonal(), 1.0e-9) * 0.06;
    const Eigen::Vector3d camera = centroid + normalSign * distance * normal;
    const Eigen::Vector3d up = std::abs(normal.y()) > 0.95 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    polyscope::view::lookAt(toGlm(camera), toGlm(centroid), toGlm(up), true);
}

void UiMockup::flyToAxisView(const Eigen::Vector3d& direction) {
    const Eigen::Vector3d center = bbox_.center();
    const Eigen::Vector3d camera = center + direction * std::max(bbox_.diagonal(), 1.0e-9) * 1.6;
    const Eigen::Vector3d up = std::abs(direction.y()) > 0.9 ? Eigen::Vector3d(0.0, 0.0, -direction.y())
                                                              : Eigen::Vector3d::UnitY();
    polyscope::view::lookAt(toGlm(camera), toGlm(center), toGlm(up), true);
}

void UiMockup::resetLayout() {
    showDisplay_ = showNavigation_ = showStructures_ = showHealth_ = showTool_ = true;
    rightColumnWidth_ = kDefaultRightColumnWidth;
    ImGui::GetIO().FontGlobalScale = 1.0f;
    setStatusMessage("Layout reset.");
}

// -----------------------------------------------------------------------------
// Queries
// -----------------------------------------------------------------------------

HealthCategory* UiMockup::selectedCategory() {
    if (selectedCategory_ < 0 || toolSlot_ != ToolSlot::DefectCategory) {
        return nullptr;
    }
    return &categories_[static_cast<std::size_t>(selectedCategory_)];
}

std::size_t UiMockup::selectedRowCount(const HealthCategory& category) const {
    return static_cast<std::size_t>(std::count_if(category.defects.begin(), category.defects.end(),
                                                  [](const MockDefect& d) { return d.selected; }));
}

bool UiMockup::watertightKnown() const {
    const auto known = [](const HealthCategory& c) {
        return c.status == HealthStatus::Ok || c.status == HealthStatus::Defects;
    };
    return known(categories_[kOpenEdges]) && known(categories_[kNonManifoldEdges]);
}

bool UiMockup::isWatertight() const {
    return categories_[kOpenEdges].defects.empty() && categories_[kNonManifoldEdges].defects.empty();
}

} // namespace ogun::mockup
