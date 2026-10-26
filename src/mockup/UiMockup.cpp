#include "mockup/UiMockup.hpp"

#include "core/Constants.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "io/StlWriter.hpp"

#include <Eigen/Geometry>

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
#include <filesystem>
#include <limits>
#include <numeric>
#include <random>
#include <span>
#include <tuple>
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
static_assert(kCategoryCount == kDefectCategoryCount, "mockup rows follow kAllDefectCategories");
static_assert(kCategoryCount <= std::tuple_size_v<decltype(ShellHealth::counts)>);

constexpr int kWithinShell = 0;
constexpr int kBetweenShells = 1;
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
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) {
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

/// `yOffset`: FramePadding.y when the line was aligned with AlignTextToFramePadding().
void dot(const ImVec4& color, float yOffset = 0.0f) {
    const float radius = ImGui::GetTextLineHeight() * 0.3f;
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 center(cursor.x + radius + 2.0f, cursor.y + yOffset + ImGui::GetTextLineHeight() * 0.5f);
    ImGui::GetWindowDrawList()->AddCircleFilled(center, radius, ImGui::GetColorU32(color), 12);
    ImGui::Dummy(ImVec2(radius * 2.0f + 4.0f, ImGui::GetTextLineHeight()));
}

// Möller-Trumbore: does the ray origin + t * dir (t > 0) hit triangle abc?
bool rayHitsTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, const Eigen::Vector3d& a,
                     const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
    const Eigen::Vector3d e1 = b - a;
    const Eigen::Vector3d e2 = c - a;
    const Eigen::Vector3d p = dir.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) <= 1.0e-14 * e1.norm() * e2.norm()) {
        return false;
    }
    const double inv = 1.0 / det;
    const Eigen::Vector3d s = origin - a;
    const double u = s.dot(p) * inv;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Eigen::Vector3d q = s.cross(e1);
    const double v = dir.dot(q) * inv;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    return e2.dot(q) * inv > 0.0;
}

bool boxInside(const BoundingBox& inner, const BoundingBox& outer) {
    return (inner.min.array() > outer.min.array()).all() && (inner.max.array() < outer.max.array()).all();
}

BoundingBox mergeBoxes(const BoundingBox& a, const BoundingBox& b) {
    return {a.min.cwiseMin(b.min), a.max.cwiseMax(b.max)};
}

} // namespace

UiMockup::UiMockup(Mesh mesh, std::string sourceLabel, bool fakeDefects)
    : mesh_(std::move(mesh)), sourceLabel_(std::move(sourceLabel)), fakeDefects_(fakeDefects) {
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
    const auto realSubtypes = [this](CategoryId id) {
        for (const char* label : categoryInfo(kAllDefectCategories[id]).subtypes) {
            if (label != nullptr) {
                categories_[id].subtypes.push_back(label);
            }
        }
    };
    realSubtypes(kSlivers);
    realSubtypes(kDegenerate);
    realSubtypes(kDuplicates);
    // The real check splits Crossing / Coplanar; the shell workflow needs Within / Between shells
    categories_[kIntersections].subtypes = {"Within shell", "Between shells"};

    buildShells();
    categories_[kShells].infoValue = shells_.size();
    if (fakeDefects_) {
        generateFakeDefects();
        findNestedShells();
    }
}

void UiMockup::buildShells() {
    const CategoryResult result = checkShells(mesh_);
    std::vector<std::size_t> order(result.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return result.facesOf(a).size() > result.facesOf(b).size();
    });

    faceShell_.assign(static_cast<std::size_t>(mesh_.numFaces()), -1);
    shells_.clear();
    shells_.reserve(order.size());
    for (const std::size_t element : order) {
        const std::span<const Eigen::Index> faces = result.facesOf(element);
        MockShell shell;
        shell.faces.assign(faces.begin(), faces.end());
        shell.bbox.min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        shell.bbox.max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
        for (const Eigen::Index f : faces) {
            faceShell_[static_cast<std::size_t>(f)] = static_cast<int>(shells_.size());
            for (int corner = 0; corner < 3; ++corner) {
                const Eigen::Vector3d p = mesh_.V.row(mesh_.F(f, corner)).transpose();
                shell.bbox.min = shell.bbox.min.cwiseMin(p);
                shell.bbox.max = shell.bbox.max.cwiseMax(p);
            }
        }
        shells_.push_back(std::move(shell));
    }
    shellFlags_.assign(shells_.size(), ShellFlags{});
}

void UiMockup::startRealChecks() {
    // A copy of the mesh, so the checks cannot race with anything the GUI does
    realChecks_ = std::async(std::launch::async, [mesh = mesh_]() {
        std::vector<CategoryResult> results(kDefectCategoryCount);
        const CheckParams params;
        for (const DefectCategory category : kAllDefectCategories) {
            if (category != DefectCategory::Shells) {
                results[categoryIndex(category)] = runCategoryCheck(category, mesh, params);
            }
        }
        return results;
    });
}

void UiMockup::applyRealResults(std::vector<CategoryResult> results) {
    crossingPairs_.clear();
    for (std::size_t i = 0; i < categories_.size(); ++i) {
        if (i == kShells) {
            continue;
        }
        const CategoryResult& result = results[i];
        auto& pending = categories_[i].pending;
        pending.clear();
        pending.reserve(result.size());
        for (std::size_t e = 0; e < result.size(); ++e) {
            const std::span<const Eigen::Index> faces = result.facesOf(e);
            if (faces.empty()) {
                continue;
            }
            const DefectElement& element = result.elements[e];
            MockDefect d;
            d.faceIndex = faces.front();
            d.metric = element.metric;
            d.area = element.columns[1];
            d.subtype = std::max(element.subtype, 0);
            d.shell = faceShell_[static_cast<std::size_t>(d.faceIndex)];
            if (i == kIntersections) {
                for (const Eigen::Index partner : result.relatedOf(e)) {
                    const int other = faceShell_[static_cast<std::size_t>(partner)];
                    if (other < 0 || d.shell < 0 || other == d.shell) {
                        d.selfCrossing = true;
                    } else if (std::find(d.partnerShells.begin(), d.partnerShells.end(), other) == d.partnerShells.end()) {
                        d.partnerShells.push_back(other);
                        crossingPairs_.emplace(std::min(other, d.shell), std::max(other, d.shell));
                    }
                }
                d.subtype = d.selfCrossing ? kWithinShell : kBetweenShells;
            }
            // Shells deleted while the checks ran stay deleted
            if (d.shell >= 0 && shellFlags_[static_cast<std::size_t>(rootShell(d.shell))].deleted) {
                continue;
            }
            pending.push_back(d);
        }
    }
    findNestedShells();
    realChecksApplied_ = true;
}

void UiMockup::findNestedShells() {
    // A shell is nested in k when its box lies inside k's box, it does not cross k, and one of its
    // vertices is inside k (odd number of ray hits). The skewed direction avoids hitting edges.
    const Eigen::Vector3d dir = Eigen::Vector3d(kAlphaX, kAlphaY, kAlphaZ).normalized();
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        MockShell& shell = shells_[s];
        shell.nestedIn = -1;
        const Eigen::Vector3d probe = mesh_.V.row(mesh_.F(shell.faces.front(), 0)).transpose();
        // Smallest container wins, so a shell inside a cavity reports the cavity
        for (std::size_t k = shells_.size(); k-- > 0;) {
            const auto si = static_cast<int>(s);
            const auto ki = static_cast<int>(k);
            if (k == s || !boxInside(shell.bbox, shells_[k].bbox) ||
                crossingPairs_.contains({std::min(si, ki), std::max(si, ki)})) {
                continue;
            }
            std::size_t hits = 0;
            for (const Eigen::Index f : shells_[k].faces) {
                hits += rayHitsTriangle(probe, dir, mesh_.V.row(mesh_.F(f, 0)).transpose(),
                                        mesh_.V.row(mesh_.F(f, 1)).transpose(), mesh_.V.row(mesh_.F(f, 2)).transpose())
                    ? 1
                    : 0;
            }
            if (hits % 2 == 1) {
                shell.nestedIn = ki;
                break;
            }
        }
    }
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
            d.subtype = id == kIntersections ? kWithinShell : subtype(rng);
            d.selfCrossing = id == kIntersections;
            d.shell = faceShell_[static_cast<std::size_t>(d.faceIndex)];
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
    polyscope::options::programName = "Ogun - UI Mockup (simulated fixes)";
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::options::buildDefaultGuiPanels = false;
    polyscope::options::openImGuiWindowForUserCallback = false;
    polyscope::init();

    registerMesh();
    polyscope::view::resetCameraToHomeView();
    if (!fakeDefects_) {
        startRealChecks();
    }
    startCheckAll();
    setStatusMessage(fakeDefects_ ? "Mockup: defect counts are fake (--fake)."
                                  : "Mockup: shells and defects are real; fixes, deletes and merges are simulated.");

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

    // Shells fills the rest of the column (only for multi-shell meshes); Structures sits below it,
    // a title bar while collapsed
    const bool structures = showStructures_ && y < bottom;
    const float structuresHeight = !structures ? 0.0f
                                   : structuresCollapsed_ ? ImGui::GetFrameHeight()
                                                          : (bottom - y) * 0.4f;
    if (showShells_ && hasShellPanel() && y < bottom) {
        const float height = std::max(bottom - y - (structures ? structuresHeight + kMargin : 0.0f), 120.0f);
        ImGui::SetNextWindowPos(ImVec2(kMargin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(kLeftColumnWidth, height), ImGuiCond_Always);
        const std::string title = "Shells (" + withThousands(liveShellCount()) + ")###Shells";
        if (ImGui::Begin(title.c_str(), &showShells_, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
            drawShellsPanelContents();
        }
        y += ImGui::GetWindowHeight() + kMargin;
        ImGui::End();
    }

    // Last panel fills the rest of the column and scrolls
    if (structures && y < bottom) {
        ImGui::SetNextWindowPos(ImVec2(kMargin, y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(kLeftColumnWidth, bottom - y), ImGuiCond_Always);
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_FirstUseEver);
        const bool open = ImGui::Begin("Structures (advanced)", &showStructures_, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
        structuresCollapsed_ = ImGui::IsWindowCollapsed();
        if (open) {
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
    case ToolSlot::MergeShells: title = "Operation: Merge Shells"; break;
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
        if (ImGui::BeginMenu("Export Selected Shells", !selectedShells().empty())) {
            drawShellSavePopup();
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
                for (std::size_t r = 0; r < cat->defects.size(); ++r) cat->defects[r].selected = rowVisible(*cat, r);
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
        ImGui::MenuItem("Shells", nullptr, &showShells_, hasShellPanel());
        ImGui::MenuItem("Structures (advanced)", nullptr, &showStructures_);
        ImGui::Separator();
        if (ImGui::BeginMenu("Camera")) {
            if (ImGui::MenuItem("Home (frame all)", "Home / H")) polyscope::view::flyToHomeView();
            if (ImGui::MenuItem("Frame focused defect", ".")) {
                if (HealthCategory* c = selectedCategory(); c && focusedRow_ >= 0) {
                    flyToFace(c->defects[static_cast<std::size_t>(focusedRow_)].faceIndex, 1.0);
                }
            }
            if (ImGui::MenuItem("Frame selected shells", ".", false, !selectedShells().empty())) frameSelectedShells();
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
            if (ImGui::MenuItem("Remove Small Shells...", nullptr, false, hasShellPanel())) {
                showShells_ = true;
                const std::vector<std::size_t> faces = shellFaceCounts();
                selectShellsWhere([&](int s) { return faces[static_cast<std::size_t>(s)] < static_cast<std::size_t>(smallShellFaces_); });
                setStatusMessage(withThousands(selectedShells().size()) + " shell(s) under " +
                                 withThousands(static_cast<std::size_t>(smallShellFaces_)) +
                                 " faces selected (change N in Shells > Select). Press Delete to remove them.");
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Shells", hasShellPanel())) {
            ImGui::MenuItem("Show Shells Panel", nullptr, &showShells_);
            drawShellActionItems();
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
            const std::size_t n = fixAllCount(i);
            const std::string label = "Fix All " + std::string(c.name) + (n == 0 ? std::string() : " (" + withThousands(n) + ")");
            if (ImGui::MenuItem(label.c_str(), nullptr, false, n > 0 && !jobRunning_)) {
                fixAll(i);
            }
        }
        ImGui::Separator();
        ImGui::MenuItem("Auto-Repair in Safe Order (future)", nullptr, false, false);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Mockup")) {
        ImGui::TextDisabled("Scope model: which shells Mesh Health covers");
        if (ImGui::MenuItem("Visible shells", nullptr, scopeModel_ == ScopeModel::VisibleShells)) {
            scopeModel_ = ScopeModel::VisibleShells;
            pruneOutOfScope();
        }
        if (ImGui::MenuItem("Selected shells (chosen; visible if none selected)", nullptr, scopeModel_ == ScopeModel::SelectedShells)) {
            scopeModel_ = ScopeModel::SelectedShells;
            pruneOutOfScope();
        }
        if (ImGui::MenuItem("Whole mesh (no scope)", nullptr, scopeModel_ == ScopeModel::WholeMesh)) {
            scopeModel_ = ScopeModel::WholeMesh;
            pruneOutOfScope();
        }
        ImGui::Separator();
        ImGui::TextDisabled(fakeDefects_ ? "Defects: fake (--fake)" : "Defects: real checks; fixes simulated");
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
            issues += scopedCount(c);
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

    // Scope line: only while the counts cover part of the mesh
    if (hasShellPanel() && scopeModel_ != ScopeModel::WholeMesh) {
        const std::vector<int> live = liveShells();
        const std::size_t shown = static_cast<std::size_t>(std::count_if(live.begin(), live.end(), [this](int s) {
            return shellFlags_[static_cast<std::size_t>(s)].visible;
        }));
        const std::size_t selected = selectedShells().size();
        if (scopeModel_ == ScopeModel::SelectedShells && selected > 0) {
            ImGui::TextColored(kRunningColor, "Scope: %s selected shell%s", withThousands(selected).c_str(),
                               selected == 1 ? "" : "s");
            tooltip("Counts, lists, Next/Prev and Fix All cover only the selected shells.");
            ImGui::SameLine();
            if (ImGui::SmallButton("All shells")) {
                selectShellsWhere([](int) { return false; });
            }
        } else if (shown < live.size()) {
            ImGui::TextColored(kRunningColor, "Scope: %s of %s shells", withThousands(shown).c_str(),
                               withThousands(live.size()).c_str());
            tooltip("Hidden shells are not counted, listed or fixed. They are still saved with the mesh.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Show all")) {
                showAllShells();
            }
        }
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
    // The dot follows the scoped count, so an isolated clean shell shows green rows
    statusDot(c.status == HealthStatus::Defects && scopedCount(c) == 0 ? HealthStatus::Ok : c.status);

    ImGui::TableSetColumnIndex(1);
    const bool selected = selectedCategory_ == static_cast<int>(i);
    if (ImGui::Selectable(c.name, selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
        if (i == kShells) {
            // Shells are managed in their own panel, not in the Tool slot
            if (hasShellPanel()) {
                showShells_ = true;
                colorByShell_ = true;
                refreshFaceColors();
            } else {
                setStatusMessage("One shell: nothing to manage.");
            }
        } else {
            selectCategory(selected ? -1 : static_cast<int>(i));
        }
    }
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
        ImGui::TextUnformatted(c.tooltip);
        if (i == kShells && hasShellPanel()) {
            ImGui::TextDisabled("Click to open the Shells panel and color each shell.");
        }
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
        count = withThousands(c.infoOnly ? liveShellCount() : scopedCount(c));
        color = c.infoOnly ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : (scopedCount(c) == 0 ? kOkColor : kDefectColor);
        break;
    case HealthStatus::Stale: count = withThousands(scopedCount(c)) + "?"; break;
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
        ImGui::Text("%s shell(s). Manage them in the Shells panel.", withThousands(liveShellCount()).c_str());
        return;
    }

    // Subtype filter chips; a filter can bring the count under the list cap
    const std::size_t inScope = scopedCount(category);
    if (!category.subtypes.empty() && inScope > 0) {
        std::vector<std::size_t> perType(category.subtypes.size(), 0);
        for (const auto& d : category.defects) {
            if (defectInScope(d) && static_cast<std::size_t>(d.subtype) < perType.size()) {
                ++perType[static_cast<std::size_t>(d.subtype)];
            }
        }
        const std::string all = "All (" + withThousands(inScope) + ")";
        if (ImGui::RadioButton(all.c_str(), subtypeFilter_ < 0)) subtypeFilter_ = -1;
        for (std::size_t t = 0; t < category.subtypes.size(); ++t) {
            const std::string label = std::string(category.subtypes[t]) + " (" + withThousands(perType[t]) + ")";
            const float nextRight = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + ImGui::GetFrameHeight() +
                                    ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(label.c_str()).x;
            if (nextRight < ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) {
                ImGui::SameLine();
            }
            if (ImGui::RadioButton(label.c_str(), subtypeFilter_ == static_cast<int>(t))) {
                subtypeFilter_ = static_cast<int>(t);
            }
        }
    }
    if (categoryIndex == kIntersections && hasShellPanel()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("Crossings between shells are removed by Merge Shells, not by Fix All.");
        ImGui::PopStyleColor();
    }

    std::vector<std::size_t> visibleRows;
    visibleRows.reserve(category.defects.size());
    for (std::size_t r = 0; r < category.defects.size(); ++r) {
        if (rowVisible(category, r)) {
            visibleRows.push_back(r);
        }
    }
    const std::size_t selectedCount = selectedRowCount(category);
    const std::size_t fixable = fixAllCount(categoryIndex);

    ImGui::Separator();
    ImGui::BeginDisabled(fixable == 0 || jobRunning_);
    const std::string fixAllLabel = "Fix All " + withThousands(fixable);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.25f, 1.0f));
    if (ImGui::Button(fixAllLabel.c_str())) fixAll(categoryIndex);
    ImGui::PopStyleColor();
    tooltip(hasShellPanel() && scopeModel_ != ScopeModel::WholeMesh
                ? "Repair every defect of this type in the scoped shells, in one undoable step."
                : "Repair every defect of this type in one undoable step.");
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

    if (inScope == 0) {
        if (category.status == HealthStatus::Ok || category.status == HealthStatus::Defects) {
            ImGui::TextColored(kOkColor, "No %s%s.", category.name,
                               category.defects.empty() ? "" : " in the scoped shells");
        }
        return;
    }

    if (static_cast<int>(visibleRows.size()) <= listCap_) {
        drawDefectTable(category, visibleRows);
    } else {
        drawDefectSummary(category, visibleRows);
    }

    if (static_cast<int>(visibleRows.size()) > overlayCap_) {
        ImGui::TextDisabled("3D: face tint only (%s faces > overlay cap %d).",
                            withThousands(visibleRows.size()).c_str(), overlayCap_);
    }

    if (focusedRow_ >= 0 && static_cast<std::size_t>(focusedRow_) < category.defects.size() &&
        rowVisible(category, static_cast<std::size_t>(focusedRow_))) {
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
    const bool showSubtype = !category.subtypes.empty();
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
            ImGui::TextUnformatted(static_cast<std::size_t>(d.subtype) < category.subtypes.size()
                                       ? category.subtypes[static_cast<std::size_t>(d.subtype)]
                                       : "-");
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
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    const auto logMetric = [&](std::size_t row) { return std::log10(std::max(category.defects[row].metric, 1.0e-12)); };
    for (const std::size_t row : visibleRows) {
        lo = std::min(lo, logMetric(row));
        hi = std::max(hi, logMetric(row));
    }
    const double span = std::max(hi - lo, 1.0e-9);
    for (const std::size_t row : visibleRows) {
        const double t = std::clamp((logMetric(row) - lo) / span, 0.0, 0.999);
        histogram[static_cast<std::size_t>(t * static_cast<double>(histogram.size()))] += 1.0f;
    }
    char histLabel[96];
    std::snprintf(histLabel, sizeof(histLabel), "log10 %s: %.1f .. %.1f", category.metricLabel, lo, hi);
    ImGui::PlotHistogram("##MetricHistogram", histogram.data(), static_cast<int>(histogram.size()), 0,
                         histLabel, 0.0f, FLT_MAX, ImVec2(-1.0f, 70.0f));
    if (!visibleRows.empty()) {
        ImGui::Text("Worst %s: %.2e   Best: %.2e", category.metricLabel,
                    category.defects[visibleRows.front()].metric, category.defects[visibleRows.back()].metric);
    }

    if (ImGui::Button("< Prev")) stepFocus(-1);
    ImGui::SameLine();
    if (ImGui::Button("Next >")) stepFocus(+1);
    ImGui::SameLine();
    const auto focusedPos = std::find(visibleRows.begin(), visibleRows.end(), static_cast<std::size_t>(focusedRow_));
    if (focusedRow_ >= 0 && focusedPos != visibleRows.end()) {
        ImGui::Text("Defect %s of %s (worst first)",
                    withThousands(static_cast<std::size_t>(focusedPos - visibleRows.begin()) + 1).c_str(),
                    withThousands(visibleRows.size()).c_str());
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
    if (d.shell >= 0 && hasShellPanel()) {
        ImGui::Text("In shell %s", shellLabel(rootShell(d.shell)).c_str());
        std::string others;
        for (const int p : d.partnerShells) {
            if (rootShell(p) != rootShell(d.shell)) others += (others.empty() ? "" : ", ") + shellLabel(rootShell(p));
        }
        if (!others.empty()) {
            ImGui::TextDisabled("Also crosses shell %s (Merge Shells removes those crossings)", others.c_str());
        }
    }
    if (selectedCategory_ == static_cast<int>(kSlivers)) {
        ImGui::Text("Detected type: %s", static_cast<std::size_t>(d.subtype) < category.subtypes.size()
                                             ? category.subtypes[static_cast<std::size_t>(d.subtype)]
                                             : "-");
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
    if (toolSlot_ == ToolSlot::MergeShells) {
        drawMergeTool();
        return;
    }
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
    const std::array<std::pair<const char*, const char*>, 19> rows = {{
        {"Left drag", "Orbit"},
        {"Right / Shift+Left drag", "Pan"},
        {"Scroll", "Zoom"},
        {"Left click face", "Focus that defect (if it belongs to the selected type)"},
        {"Alt+click face", "Select its shell (Ctrl/Cmd: toggle, Shift: add)"},
        {"I", "Isolate the selected shells (again to undo)"},
        {"Delete / Backspace", "Delete the selected shells (undoable)"},
        {".", "Frame focused defect (+Normal; again for -Normal), else the selected shells"},
        {"Shift+.", "Frame focused defect from behind"},
        {"Home / H", "Frame whole mesh"},
        {"N / Shift+N", "Next / previous defect (worst first)"},
        {"F5", "Check All"},
        {"G", "Toggle ghost mesh"},
        {"Esc", "Clear selection / close tool / clear shells"},
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
    if (ImGui::IsKeyPressed(ImGuiKey_I, false) && hasShellPanel()) toggleIsolate();
    if ((ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) &&
        !selectedShells().empty()) {
        deleteSelectedShells();
    }
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
        } else if (!selectedShells().empty()) {
            frameSelectedShells();
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (toolSlot_ != ToolSlot::DefectCategory && toolSlot_ != ToolSlot::None) {
            toolSlot_ = ToolSlot::None;
        } else if (HealthCategory* c = selectedCategory(); c && (focusedRow_ >= 0 || selectedRowCount(*c) > 0)) {
            for (auto& d : c->defects) d.selected = false;
            focusedRow_ = -1;
            refreshFaceColors();
        } else if (selectedCategory_ >= 0) {
            selectCategory(-1);
        } else if (!selectedShells().empty()) {
            selectShellsWhere([](int) { return false; });
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
    const bool shellPick = hasShellPanel() && io.KeyAlt;
    if (cat == nullptr && !shellPick) {
        return;
    }
    const ImVec2 p = ImGui::GetMousePos();
    const polyscope::PickResult pick = polyscope::pickAtScreenCoords(glm::vec2{p.x, p.y});
    Eigen::Index face = -1;
    const auto nVerts = static_cast<std::uint64_t>(mesh_.numVertices());
    if (pick.isHit && pick.structureName == kMeshName && pick.localIndex >= nVerts &&
        pick.localIndex < nVerts + static_cast<std::uint64_t>(displayFaces_.size())) {
        face = displayFaces_[static_cast<std::size_t>(pick.localIndex - nVerts)];
    }

    if (shellPick) {
        const bool toggle = io.KeyCtrl || io.KeySuper;
        if (face < 0 || faceShell_[static_cast<std::size_t>(face)] < 0) {
            return;
        }
        const int shell = rootShell(faceShell_[static_cast<std::size_t>(face)]);
        if (io.KeyShift) {
            shellFlags_[static_cast<std::size_t>(shell)].selected = true;
            shellAnchor_ = shell;
            onShellSelectionChanged();
        } else {
            clickShell(shell, toggle, false, {});
        }
        setStatusMessage("Shell " + shellLabel(shell) + ": " + withThousands(shellFaceCounts()[static_cast<std::size_t>(shell)]) +
                         " faces.");
        return;
    }
    if (face < 0) {
        return;
    }
    const auto it = std::find_if(cat->defects.begin(), cat->defects.end(),
                                 [this, face](const MockDefect& d) { return d.faceIndex == face && defectInScope(d); });
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
    // Hold the row on Running until the real checks have finished in the background
    if (!fakeDefects_ && !realChecksApplied_) {
        if (!realChecks_.valid() || realChecks_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;
        }
        applyRealResults(realChecks_.get());
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
        std::string done = jobSingle_ ? std::string(c.name) + " checked." : "Check All finished.";
        const std::vector<int> live = liveShells();
        const auto nested = std::count_if(live.begin(), live.end(), [this](int s) {
            const int in = shells_[static_cast<std::size_t>(s)].nestedIn;
            return in >= 0 && shellLive(rootShell(in)) && rootShell(in) != s;
        });
        if (!jobSingle_ && nested > 0) {
            done += " " + std::to_string(nested) + " shell(s) lie inside another shell (Shells > Select... > Nested).";
        }
        setStatusMessage(done);
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
    const std::size_t count = fixAllCount(categoryIndex);
    if (count == 0) {
        return;
    }
    pushUndo(std::string("Fix All ") + c.name);
    std::erase_if(c.defects, [&](const MockDefect& d) { return defectInScope(d) && fixableByFixAll(categoryIndex, d); });
    for (MockDefect& d : c.defects) d.selected = false;
    c.status = c.defects.empty() ? HealthStatus::Ok : HealthStatus::Defects;
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
    const std::vector<int> live = liveShells();
    const bool scoped = std::any_of(live.begin(), live.end(), [this](int s) { return !shellInScope(s); });
    const bool betweenLeft = categoryIndex == kIntersections &&
        std::any_of(c.defects.begin(), c.defects.end(), [this](const MockDefect& d) { return defectInScope(d); });
    setStatusMessage("Fixed " + withThousands(count) + " " + c.name + (scoped ? " in the scoped shells" : "") +
                     " (mockup). Fast checks re-ran" + (markedStale ? "; slow checks marked stale." : ".") +
                     (betweenLeft ? " Crossings between shells are left for Merge Shells." : ""));
}

void UiMockup::fixSelected(std::size_t categoryIndex) {
    HealthCategory& c = categories_[categoryIndex];
    if (focusedRow_ >= 0 && selectedRowCount(c) == 0) {
        c.defects[static_cast<std::size_t>(focusedRow_)].selected = true;
    }
    const std::size_t before = c.defects.size();
    pushUndo(std::string("Fix Selected ") + c.name);
    std::erase_if(c.defects, [this](const MockDefect& d) { return d.selected && defectInScope(d); });
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
    snap.shellFlags = shellFlags_;
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
    const bool visibilityChanged = std::any_of(shellFlags_.begin(), shellFlags_.end(), [&](const ShellFlags& f) {
        const ShellFlags& old = snap.shellFlags[static_cast<std::size_t>(&f - shellFlags_.data())];
        return f.visible != old.visible || f.deleted != old.deleted || f.mergedInto != old.mergedInto;
    });
    shellFlags_ = std::move(snap.shellFlags);
    modified_ = !undoStack_.empty();
    focusedRow_ = -1;
    if (visibilityChanged) {
        registerMesh();
    }
    pruneOutOfScope();
    setStatusMessage("Undid: " + snap.label);
}

void UiMockup::stepFocus(int direction) {
    HealthCategory* cat = selectedCategory();
    if (cat == nullptr || cat->defects.empty()) {
        return;
    }
    const auto n = static_cast<std::ptrdiff_t>(cat->defects.size());
    std::ptrdiff_t row = focusedRow_ < 0 ? (direction > 0 ? 0 : n - 1) : (focusedRow_ + direction + n) % n;
    // Respect the scope and the subtype filter while stepping
    std::ptrdiff_t guard = 0;
    for (; guard < n && !rowVisible(*cat, static_cast<std::size_t>(row)); ++guard) {
        row = (row + direction + n) % n;
    }
    if (guard == n) {
        return;
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
    // Hidden and deleted shells are cut out; displayFaces_ maps picks back to mesh faces
    displayFaces_.clear();
    displayFaces_.reserve(static_cast<std::size_t>(mesh_.numFaces()));
    faceToRow_.assign(static_cast<std::size_t>(mesh_.numFaces()), -1);
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        if (faceShown(f)) {
            faceToRow_[static_cast<std::size_t>(f)] = static_cast<Eigen::Index>(displayFaces_.size());
            displayFaces_.push_back(f);
        }
    }
    // Polyscope aborts when drawing a structure without faces
    if (displayFaces_.empty()) {
        polyscope::removeSurfaceMesh(kMeshName, false);
        return;
    }
    Eigen::MatrixXi faces(static_cast<Eigen::Index>(displayFaces_.size()), 3);
    for (std::size_t row = 0; row < displayFaces_.size(); ++row) {
        faces.row(static_cast<Eigen::Index>(row)) = mesh_.F.row(displayFaces_[row]);
    }
    auto* psMesh = polyscope::registerSurfaceMesh(kMeshName, mesh_.V, faces);
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
    const Eigen::RowVector3d shellSelected(shellSelectedColor_[0], shellSelectedColor_[1], shellSelectedColor_[2]);
    const bool shellsShown = hasShellPanel();

    // Layers: shell color, then the selected-shell tint, then defect tints
    Eigen::MatrixXd colors(static_cast<Eigen::Index>(displayFaces_.size()), 3);
    for (std::size_t row = 0; row < displayFaces_.size(); ++row) {
        const int shell = shellsShown ? faceShell_[static_cast<std::size_t>(displayFaces_[row])] : -1;
        const int root = shell >= 0 ? rootShell(shell) : -1;
        Eigen::RowVector3d c = root >= 0 && colorByShell_ ? shellColor(root) : base;
        if (root >= 0 && shellFlags_[static_cast<std::size_t>(root)].selected) {
            c = 0.35 * c + 0.65 * shellSelected;
        }
        colors.row(static_cast<Eigen::Index>(row)) = c;
    }
    if (const HealthCategory* cat = selectedCategory()) {
        for (std::size_t r = 0; r < cat->defects.size(); ++r) {
            const MockDefect& d = cat->defects[r];
            const Eigen::Index row = faceToRow_[static_cast<std::size_t>(d.faceIndex)];
            if (row >= 0 && rowVisible(*cat, r)) {
                colors.row(row) = d.selected ? selected : category;
            }
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

void UiMockup::flyToBox(const BoundingBox& box) {
    const Eigen::Vector3d center = box.center();
    const glm::vec3 eye = polyscope::view::getCameraWorldPosition();
    Eigen::Vector3d dir = Eigen::Vector3d(eye.x, eye.y, eye.z) - center;
    if (dir.squaredNorm() < 1.0e-30) {
        dir = Eigen::Vector3d::UnitZ();
    }
    dir.normalize();
    const double distance = std::max(box.diagonal(), bbox_.diagonal() * 1.0e-3) * 1.6;
    const Eigen::Vector3d up = std::abs(dir.y()) > 0.95 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    polyscope::view::lookAt(toGlm(center + distance * dir), toGlm(center), toGlm(up), true);
}

void UiMockup::resetLayout() {
    showDisplay_ = showNavigation_ = showStructures_ = showHealth_ = showTool_ = showShells_ = true;
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


// -----------------------------------------------------------------------------
// Shells: queries
// -----------------------------------------------------------------------------

int UiMockup::rootShell(int shell) const {
    while (shell >= 0 && shellFlags_[static_cast<std::size_t>(shell)].mergedInto >= 0) {
        shell = shellFlags_[static_cast<std::size_t>(shell)].mergedInto;
    }
    return shell;
}

bool UiMockup::shellLive(int shell) const {
    const ShellFlags& flags = shellFlags_[static_cast<std::size_t>(shell)];
    return !flags.deleted && flags.mergedInto < 0;
}

bool UiMockup::faceShown(Eigen::Index face) const {
    const int shell = faceShell_[static_cast<std::size_t>(face)];
    if (shell < 0) {
        return true;
    }
    const ShellFlags& flags = shellFlags_[static_cast<std::size_t>(rootShell(shell))];
    return !flags.deleted && flags.visible;
}

bool UiMockup::shellInScope(int root) const {
    const ShellFlags& flags = shellFlags_[static_cast<std::size_t>(root)];
    switch (scopeModel_) {
    case ScopeModel::WholeMesh:
        return true;
    case ScopeModel::SelectedShells:
        return selectedShellCount_ > 0 ? flags.selected : flags.visible;
    case ScopeModel::VisibleShells:
        return flags.visible;
    }
    return true;
}

bool UiMockup::crossesOwnShell(const MockDefect& defect) const {
    const int own = rootShell(defect.shell);
    return defect.selfCrossing || std::any_of(defect.partnerShells.begin(), defect.partnerShells.end(),
                                              [&](int p) { return rootShell(p) == own; });
}

bool UiMockup::defectInScope(const MockDefect& defect) const {
    if (defect.shell < 0) {
        return true;
    }
    if (!shellInScope(rootShell(defect.shell))) {
        return false;
    }
    if (defect.partnerShells.empty() || crossesOwnShell(defect)) {
        return true;
    }
    // A face that only crosses other shells counts when one of those shells is in scope too
    return std::any_of(defect.partnerShells.begin(), defect.partnerShells.end(),
                       [this](int p) { return shellInScope(rootShell(p)); });
}

bool UiMockup::rowVisible(const HealthCategory& category, std::size_t row) const {
    const MockDefect& d = category.defects[row];
    return defectInScope(d) && (subtypeFilter_ < 0 || d.subtype == subtypeFilter_);
}

bool UiMockup::fixableByFixAll(std::size_t categoryIndex, const MockDefect& defect) const {
    // Crossings between two shells are resolved by Merge Shells (a union), not by a per-shell repair
    return !(categoryIndex == kIntersections && !defect.partnerShells.empty() && !crossesOwnShell(defect));
}

std::size_t UiMockup::scopedCount(const HealthCategory& category) const {
    return static_cast<std::size_t>(std::count_if(category.defects.begin(), category.defects.end(),
                                                  [this](const MockDefect& d) { return defectInScope(d); }));
}

std::size_t UiMockup::fixAllCount(std::size_t categoryIndex) const {
    const auto& defects = categories_[categoryIndex].defects;
    return static_cast<std::size_t>(std::count_if(defects.begin(), defects.end(), [&](const MockDefect& d) {
        return defectInScope(d) && fixableByFixAll(categoryIndex, d);
    }));
}

bool UiMockup::hasShellPanel() const {
    // Stays once the file had several shells, so a merge or delete down to one keeps the panel
    return shells_.size() > 1;
}

std::size_t UiMockup::liveShellCount() const {
    return liveShells().size();
}

std::vector<int> UiMockup::liveShells() const {
    std::vector<int> live;
    live.reserve(shells_.size());
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (shellLive(static_cast<int>(s))) {
            live.push_back(static_cast<int>(s));
        }
    }
    return live;
}

std::vector<int> UiMockup::selectedShells() const {
    std::vector<int> selected;
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (shellLive(static_cast<int>(s)) && shellFlags_[s].selected) {
            selected.push_back(static_cast<int>(s));
        }
    }
    return selected;
}

std::vector<std::size_t> UiMockup::shellFaceCounts() const {
    std::vector<std::size_t> counts(shells_.size(), 0);
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        counts[static_cast<std::size_t>(rootShell(static_cast<int>(s)))] += shells_[s].faces.size();
    }
    return counts;
}

BoundingBox UiMockup::shellBox(int root) const {
    std::optional<BoundingBox> box;
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        if (rootShell(static_cast<int>(s)) == root) {
            box = box ? mergeBoxes(*box, shells_[s].bbox) : shells_[s].bbox;
        }
    }
    return box.value_or(bbox_);
}

std::vector<ShellHealth> UiMockup::shellHealth() const {
    std::vector<ShellHealth> health(shells_.size());
    for (std::size_t i = 0; i < categories_.size(); ++i) {
        if (categories_[i].infoOnly) {
            continue;
        }
        for (const MockDefect& d : categories_[i].defects) {
            if (d.shell < 0) {
                continue;
            }
            ShellHealth& h = health[static_cast<std::size_t>(rootShell(d.shell))];
            if (fixableByFixAll(i, d)) {
                ++h.counts[i];
                ++h.total;
            } else {
                ++h.between;
            }
        }
    }
    return health;
}

bool UiMockup::healthKnown() const {
    return std::any_of(categories_.begin(), categories_.end(), [](const HealthCategory& c) {
        return !c.infoOnly && (c.status == HealthStatus::Ok || c.status == HealthStatus::Defects ||
                               c.status == HealthStatus::Stale);
    });
}

std::string UiMockup::shellLabel(int shell) const {
    return "#" + std::to_string(shell + 1);
}

Eigen::RowVector3d UiMockup::shellColor(int root) const {
    // Golden-ratio hue steps inside green..purple: no red, orange or yellow, which mean defects
    const float t = std::fmod(static_cast<float>(root) * 0.618034f, 1.0f);
    const float hue = 0.28f + 0.55f * t;
    const float value = root % 2 == 0 ? 0.82f : 0.70f;
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    ImGui::ColorConvertHSVtoRGB(hue, 0.38f, value, r, g, b);
    return {r, g, b};
}

Mesh UiMockup::extractShells(const std::vector<int>& roots) const {
    std::vector<Eigen::Index> faces;
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        const int shell = faceShell_[static_cast<std::size_t>(f)];
        if (shell >= 0 && std::find(roots.begin(), roots.end(), rootShell(shell)) != roots.end()) {
            faces.push_back(f);
        }
    }
    std::vector<Eigen::Index> remap(static_cast<std::size_t>(mesh_.numVertices()), -1);
    Mesh out;
    out.fileUnit = mesh_.fileUnit;
    out.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    std::vector<Eigen::Index> usedVertices;
    for (std::size_t row = 0; row < faces.size(); ++row) {
        for (int corner = 0; corner < 3; ++corner) {
            const auto v = static_cast<std::size_t>(mesh_.F(faces[row], corner));
            if (remap[v] < 0) {
                remap[v] = static_cast<Eigen::Index>(usedVertices.size());
                usedVertices.push_back(static_cast<Eigen::Index>(v));
            }
            out.F(static_cast<Eigen::Index>(row), corner) = static_cast<int>(remap[v]);
        }
    }
    out.V.resize(static_cast<Eigen::Index>(usedVertices.size()), 3);
    for (std::size_t i = 0; i < usedVertices.size(); ++i) {
        out.V.row(static_cast<Eigen::Index>(i)) = mesh_.V.row(usedVertices[i]);
    }
    out.recomputeNormals();
    return out;
}

// -----------------------------------------------------------------------------
// Shells: selection, visibility and edits
// -----------------------------------------------------------------------------

void UiMockup::recountSelection() {
    selectedShellCount_ = selectedShells().size();
}

void UiMockup::clickShell(int shell, bool toggle, bool range, const std::vector<int>& order) {
    if (range && shellAnchor_ >= 0) {
        const auto a = std::find(order.begin(), order.end(), shellAnchor_);
        const auto b = std::find(order.begin(), order.end(), shell);
        if (a != order.end() && b != order.end()) {
            if (!toggle) {
                for (ShellFlags& f : shellFlags_) f.selected = false;
            }
            for (auto it = std::min(a, b); it <= std::max(a, b); ++it) {
                shellFlags_[static_cast<std::size_t>(*it)].selected = true;
            }
            onShellSelectionChanged();
            return;
        }
    }
    ShellFlags& flags = shellFlags_[static_cast<std::size_t>(shell)];
    if (toggle) {
        flags.selected = !flags.selected;
    } else {
        for (ShellFlags& f : shellFlags_) f.selected = false;
        flags.selected = true;
    }
    shellAnchor_ = shell;
    onShellSelectionChanged();
}

void UiMockup::selectShellsWhere(const std::function<bool(int)>& predicate) {
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        shellFlags_[s].selected = shellLive(static_cast<int>(s)) && predicate(static_cast<int>(s));
    }
    onShellSelectionChanged();
}

void UiMockup::toggleIsolate() {
    const std::vector<int> selected = selectedShells();
    // Un-isolate when nothing new is selected; a different selection re-isolates onto it
    if (isolateRestore_ && (selected.empty() || selectionMatchesVisible())) {
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            shellFlags_[s].visible = (*isolateRestore_)[s];
        }
        isolateRestore_.reset();
        onShellVisibilityChanged();
        setStatusMessage("Isolate off: the other shells are back.");
        return;
    }
    if (selected.empty()) {
        setStatusMessage("Select one or more shells to isolate (click a row, or Alt+click in 3D).");
        return;
    }
    // Keep the view from before the first Isolate, so Un-isolate always returns there
    if (!isolateRestore_) {
        std::vector<bool> restore(shells_.size());
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            restore[s] = shellFlags_[s].visible;
        }
        isolateRestore_ = std::move(restore);
    }
    for (ShellFlags& f : shellFlags_) {
        f.visible = f.selected;
    }
    onShellVisibilityChanged();
    setStatusMessage("Isolated " + withThousands(selected.size()) + " shell(s): Mesh Health now covers only them. Press I to go back.");
}

bool UiMockup::selectionMatchesVisible() const {
    for (const int s : liveShells()) {
        const ShellFlags& f = shellFlags_[static_cast<std::size_t>(s)];
        if (f.selected != f.visible) {
            return false;
        }
    }
    return true;
}

void UiMockup::hideAllShells() {
    for (ShellFlags& f : shellFlags_) {
        f.visible = false;
        f.selected = false;
    }
    isolateRestore_.reset();
    onShellVisibilityChanged();
    setStatusMessage("All shells hidden. Tick Vis to show the ones you want to work on.");
}

void UiMockup::frameSelectedShells() {
    const std::vector<int> selected = selectedShells();
    if (selected.empty()) {
        return;
    }
    BoundingBox box = shellBox(selected.front());
    for (const int s : selected) box = mergeBoxes(box, shellBox(s));
    flyToBox(box);
}

void UiMockup::hideSelectedShells() {
    for (const int s : selectedShells()) {
        shellFlags_[static_cast<std::size_t>(s)].visible = false;
        shellFlags_[static_cast<std::size_t>(s)].selected = false;
    }
    onShellVisibilityChanged();
}

void UiMockup::showAllShells() {
    for (ShellFlags& f : shellFlags_) f.visible = true;
    isolateRestore_.reset();
    onShellVisibilityChanged();
}

void UiMockup::deleteSelectedShells() {
    const std::vector<int> selected = selectedShells();
    if (selected.empty()) {
        return;
    }
    const std::vector<std::size_t> faces = shellFaceCounts();
    std::size_t faceCount = 0;
    for (const int s : selected) faceCount += faces[static_cast<std::size_t>(s)];

    pushUndo("Delete " + withThousands(selected.size()) + " shell(s)");
    for (const int s : selected) {
        shellFlags_[static_cast<std::size_t>(s)].deleted = true;
        shellFlags_[static_cast<std::size_t>(s)].selected = false;
    }
    const auto deleted = [this](int shell) {
        return shell >= 0 && shellFlags_[static_cast<std::size_t>(rootShell(shell))].deleted;
    };
    const auto gone = [&](MockDefect& d) {
        const bool hadPartners = !d.partnerShells.empty();
        std::erase_if(d.partnerShells, deleted);
        return deleted(d.shell) || (hadPartners && d.partnerShells.empty() && !d.selfCrossing);
    };
    for (HealthCategory& c : categories_) {
        std::erase_if(c.defects, gone);
        std::erase_if(c.pending, gone);
        if (c.status == HealthStatus::Defects && c.defects.empty()) {
            c.status = HealthStatus::Ok;
        }
    }
    focusedRow_ = -1;
    modified_ = true;
    onShellVisibilityChanged();
    setStatusMessage("Deleted " + withThousands(selected.size()) + " shell(s), " + withThousands(faceCount) +
                     " faces (mockup). Undo: " + kMod + "+Z.");
}

void UiMockup::saveSelectedShells(bool eachOwnFile) {
    const std::vector<int> selected = selectedShells();
    if (selected.empty()) {
        return;
    }
    const std::string stem = std::filesystem::path(sourceLabel_).stem().string();
    const std::filesystem::path dir = std::filesystem::current_path();
    std::string error;
    if (!eachOwnFile) {
        const std::filesystem::path path = dir / (stem + "_selected_shells.stl");
        if (saveStl(path, extractShells(selected), StlFormat::Binary, &error)) {
            setStatusMessage("Saved " + withThousands(selected.size()) + " shell(s) to " + path.string());
        } else {
            setStatusMessage("Save failed: " + error);
        }
        return;
    }
    std::size_t saved = 0;
    for (const int s : selected) {
        char name[32];
        std::snprintf(name, sizeof(name), "_shell_%02d.stl", s + 1);
        if (saveStl(dir / (stem + name), extractShells({s}), StlFormat::Binary, &error)) {
            ++saved;
        }
    }
    setStatusMessage(saved == selected.size()
                         ? "Saved " + withThousands(saved) + " file(s) " + stem + "_shell_NN.stl in " + dir.string()
                         : "Saved " + withThousands(saved) + " of " + withThousands(selected.size()) + " files: " + error);
}

void UiMockup::applyMerge(const std::vector<int>& roots) {
    if (roots.size() < 2) {
        return;
    }
    const int target = *std::min_element(roots.begin(), roots.end());
    pushUndo("Merge " + withThousands(roots.size()) + " shells");
    for (const int s : roots) {
        if (s != target) {
            shellFlags_[static_cast<std::size_t>(s)].mergedInto = target;
            shellFlags_[static_cast<std::size_t>(s)].selected = false;
        }
    }
    shellFlags_[static_cast<std::size_t>(target)].selected = true;
    // The union removes the crossings between the merged shells
    HealthCategory& crossings = categories_[kIntersections];
    const std::size_t before = crossings.defects.size();
    std::erase_if(crossings.defects, [this](MockDefect& d) {
        if (d.shell < 0 || d.partnerShells.empty()) {
            return false;
        }
        const int own = rootShell(d.shell);
        std::erase_if(d.partnerShells, [&](int p) { return rootShell(p) == own; });
        return d.partnerShells.empty() && !d.selfCrossing;
    });
    if (crossings.status == HealthStatus::Defects && crossings.defects.empty()) {
        crossings.status = HealthStatus::Ok;
    }
    focusedRow_ = -1;
    modified_ = true;
    toolSlot_ = ToolSlot::None;
    onShellVisibilityChanged();
    setStatusMessage("Merged " + withThousands(roots.size()) + " shells into " + shellLabel(target) + "; " +
                     withThousands(before - crossings.defects.size()) + " crossing faces removed (mockup).");
}

void UiMockup::onShellVisibilityChanged() {
    registerMesh();
    pruneOutOfScope();
}

void UiMockup::onShellSelectionChanged() {
    if (scopeModel_ == ScopeModel::SelectedShells) {
        pruneOutOfScope();
    } else {
        recountSelection();
        refreshFaceColors();
    }
}

void UiMockup::pruneOutOfScope() {
    // FB-13: a defect that leaves the scope also loses its selection and focus
    recountSelection();
    for (HealthCategory& c : categories_) {
        for (MockDefect& d : c.defects) {
            d.selected = d.selected && defectInScope(d);
        }
    }
    if (const HealthCategory* cat = selectedCategory();
        cat != nullptr && focusedRow_ >= 0 &&
        (static_cast<std::size_t>(focusedRow_) >= cat->defects.size() ||
         !defectInScope(cat->defects[static_cast<std::size_t>(focusedRow_)]))) {
        focusedRow_ = -1;
    }
    refreshFaceColors();
}

// -----------------------------------------------------------------------------
// Shells panel
// -----------------------------------------------------------------------------

void UiMockup::drawShellsPanelContents() {
    const std::vector<ShellHealth> health = shellHealth();
    if (ImGui::Checkbox("Color by shell", &colorByShell_)) {
        refreshFaceColors();
    }
    tooltip("Give each shell its own muted color (no reds or oranges, so defect tints stay clear).");
    const float selectWidth = ImGui::CalcTextSize("Select...").x + 2.0f * ImGui::GetStyle().FramePadding.x;
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - selectWidth);
    if (ImGui::Button("Select...")) {
        ImGui::OpenPopup("ShellSelect");
    }
    tooltip("Select all, none, invert, unhealthy, nested, or small shells.");
    drawShellSelectPopup(health);

    // Footer: a summary only; actions live in the right-click menu (and Mesh > Shells)
    const float footer = 2.0f * ImGui::GetTextLineHeightWithSpacing();
    drawShellTable(health, std::max(80.0f, ImGui::GetContentRegionAvail().y - footer));

    const std::vector<int> selected = selectedShells();
    const std::vector<std::size_t> faces = shellFaceCounts();
    if (selected.empty()) {
        ImGui::TextDisabled("Click a shell, or Alt+click it in 3D.");
    } else {
        std::size_t faceCount = 0;
        for (const int s : selected) faceCount += faces[static_cast<std::size_t>(s)];
        ImGui::Text("%s selected - %s faces", withThousands(selected.size()).c_str(), withThousands(faceCount).c_str());
    }
    ImGui::TextDisabled("Right-click a shell for actions.");
    tooltip("Frame (.), Isolate (I), Hide, Hide All / Show All, Delete (Delete key), Save As, Merge.");
}

void UiMockup::drawShellTable(const std::vector<ShellHealth>& health, float height) {
    const ImGuiIO& io = ImGui::GetIO();
    const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                                  ImGuiTableFlags_Sortable | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("ShellTable", 4, flags, ImVec2(0.0f, height))) {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Vis", ImGuiTableColumnFlags_NoSort, 0.0f, 0);
    ImGui::TableSetupColumn("Shell", ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_WidthStretch, 0.0f, 1);
    ImGui::TableSetupColumn("Faces", ImGuiTableColumnFlags_None, 0.0f, 2);
    ImGui::TableSetupColumn("Health", ImGuiTableColumnFlags_None, 0.0f, 3);
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    ImGui::TableSetColumnIndex(0);
    {
        const std::vector<int> live = liveShells();
        bool allVisible = std::all_of(live.begin(), live.end(), [this](int s) { return shellFlags_[static_cast<std::size_t>(s)].visible; });
        if (ImGui::Checkbox("##AllVisible", &allVisible)) {
            allVisible ? showAllShells() : hideAllShells();
        }
        tooltip("Show or hide every shell.");
    }
    for (int column = 1; column < 4; ++column) {
        ImGui::TableSetColumnIndex(column);
        ImGui::TableHeader(ImGui::TableGetColumnName(column));
    }

    const std::vector<std::size_t> faces = shellFaceCounts();
    std::vector<int> order = liveShells();
    if (const ImGuiTableSortSpecs* specs = ImGui::TableGetSortSpecs(); specs && specs->SpecsCount > 0) {
        const ImGuiTableColumnSortSpecs& s = specs->Specs[0];
        const bool asc = s.SortDirection == ImGuiSortDirection_Ascending;
        const auto key = [&](int shell) -> double {
            switch (s.ColumnUserID) {
            case 2: return static_cast<double>(faces[static_cast<std::size_t>(shell)]);
            case 3: return static_cast<double>(health[static_cast<std::size_t>(shell)].total);
            default: return static_cast<double>(shell);
            }
        };
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return asc ? key(a) < key(b) : key(a) > key(b); });
    }

    const bool known = healthKnown();
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(order.size()));
    while (clipper.Step()) {
        for (int pos = clipper.DisplayStart; pos < clipper.DisplayEnd; ++pos) {
            const int s = order[static_cast<std::size_t>(pos)];
            ShellFlags& shellFlags = shellFlags_[static_cast<std::size_t>(s)];
            ImGui::TableNextRow();
            ImGui::PushID(s);

            // The name cell is submitted first so the visibility checkbox (column 0) stays clickable
            ImGui::TableSetColumnIndex(1);
            ImGui::AlignTextToFramePadding();
            if (colorByShell_) {
                const Eigen::RowVector3d c = shellColor(s);
                const float side = ImGui::GetTextLineHeight() * 0.8f;
                const ImVec2 p = ImGui::GetCursorScreenPos();
                const float pad = (ImGui::GetTextLineHeight() - side) * 0.5f + ImGui::GetStyle().FramePadding.y;
                ImGui::GetWindowDrawList()->AddRectFilled(
                    ImVec2(p.x, p.y + pad), ImVec2(p.x + side, p.y + pad + side),
                    ImGui::GetColorU32(ImVec4(static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]), 1.0f)));
                ImGui::Dummy(ImVec2(side, ImGui::GetTextLineHeight()));
                ImGui::SameLine();
            }
            // Hidden shells are dimmed rather than labelled, so the narrow column never truncates
            if (!shellFlags.visible) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            }
            const std::string label = shellLabel(s);
            const bool clicked = ImGui::Selectable(label.c_str(), shellFlags.selected,
                                                   ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap |
                                                       ImGuiSelectableFlags_AllowDoubleClick);
            if (!shellFlags.visible) {
                ImGui::PopStyleColor();
            }
            if (clicked) {
                clickShell(s, io.KeyCtrl || io.KeySuper, io.KeyShift, order);
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    flyToBox(shellBox(s));
                }
            }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && ImGui::BeginTooltip()) {
                drawShellRowTooltip(s, health[static_cast<std::size_t>(s)]);
                ImGui::EndTooltip();
            }
            if (ImGui::BeginPopupContextItem("ShellContext")) {
                if (!shellFlags.selected) {
                    clickShell(s, false, false, order);
                }
                drawShellActionItems();
                ImGui::EndPopup();
            }

            ImGui::TableSetColumnIndex(0);
            bool visible = shellFlags.visible;
            if (ImGui::Checkbox("##vis", &visible)) {
                shellFlags.visible = visible;
                onShellVisibilityChanged();
            }

            ImGui::TableSetColumnIndex(2);
            ImGui::AlignTextToFramePadding();
            const std::string count = withThousands(faces[static_cast<std::size_t>(s)]);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(count.c_str()).x);
            if (shellFlags.visible) {
                ImGui::TextUnformatted(count.c_str());
            } else {
                ImGui::TextDisabled("%s", count.c_str());
            }

            ImGui::TableSetColumnIndex(3);
            ImGui::AlignTextToFramePadding();
            const ShellHealth& h = health[static_cast<std::size_t>(s)];
            if (!known) {
                ImGui::TextDisabled("-");
            } else {
                dot(h.total == 0 ? kOkColor : kDefectColor, ImGui::GetStyle().FramePadding.y);
                ImGui::SameLine(0.0f, 2.0f);
                ImGui::TextColored(h.total == 0 ? kOkColor : kDefectColor, "%s", withThousands(h.total).c_str());
            }
            const int nestedIn = shells_[static_cast<std::size_t>(s)].nestedIn;
            if (nestedIn >= 0 && shellLive(rootShell(nestedIn)) && rootShell(nestedIn) != s) {
                ImGui::SameLine();
                ImGui::TextColored(kRunningColor, "inside %s", shellLabel(rootShell(nestedIn)).c_str());
            }
            ImGui::PopID();
        }
    }
    ImGui::EndTable();
}

void UiMockup::drawShellRowTooltip(int shell, const ShellHealth& health) {
    const std::vector<std::size_t> faces = shellFaceCounts();
    std::size_t parts = 0;
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        parts += rootShell(static_cast<int>(s)) == shell ? 1 : 0;
    }
    ImGui::Text("Shell %s: %s faces%s", shellLabel(shell).c_str(), withThousands(faces[static_cast<std::size_t>(shell)]).c_str(),
                parts > 1 ? (" (merged from " + std::to_string(parts) + " shells)").c_str() : "");
    if (!healthKnown()) {
        ImGui::TextDisabled("Not checked yet.");
    } else if (health.total == 0) {
        ImGui::TextColored(kOkColor, "Clean: nothing to fix inside this shell.");
    } else {
        for (std::size_t i = 0; i < categories_.size(); ++i) {
            if (health.counts[i] > 0) {
                ImGui::TextColored(kDefectColor, "%s: %s", categories_[i].name, withThousands(health.counts[i]).c_str());
            }
        }
    }

    std::set<int> partners;
    for (const auto& [a, b] : crossingPairs_) {
        const int ra = rootShell(a);
        const int rb = rootShell(b);
        if (ra != rb && (ra == shell || rb == shell)) {
            const int other = ra == shell ? rb : ra;
            if (shellLive(other)) partners.insert(other);
        }
    }
    if (health.between > 0 || !partners.empty()) {
        std::string list;
        for (const int p : partners) list += (list.empty() ? "" : ", ") + shellLabel(p);
        ImGui::TextDisabled("Crosses shell%s %s (%s faces): Merge Shells removes these.", partners.size() == 1 ? "" : "s",
                            list.c_str(), withThousands(health.between).c_str());
    }
    const int nestedIn = shells_[static_cast<std::size_t>(shell)].nestedIn;
    if (nestedIn >= 0 && shellLive(rootShell(nestedIn)) && rootShell(nestedIn) != shell) {
        ImGui::TextColored(kRunningColor, "Lies inside shell %s without touching it: often a hidden internal part.",
                           shellLabel(rootShell(nestedIn)).c_str());
    }
    ImGui::TextDisabled("Double-click to frame. Right-click for actions.");
}

void UiMockup::drawShellActionItems() {
    const std::vector<int> selected = selectedShells();
    const bool none = selected.empty();
    if (ImGui::MenuItem("Frame", ".", false, !none)) frameSelectedShells();
    const bool unisolate = isolateRestore_ && (none || selectionMatchesVisible());
    if (ImGui::MenuItem(unisolate ? "Un-isolate" : "Isolate", "I", false, isolateRestore_ || !none)) toggleIsolate();
    if (ImGui::MenuItem("Hide", nullptr, false, !none)) hideSelectedShells();
    if (ImGui::MenuItem("Hide All")) hideAllShells();
    if (ImGui::MenuItem("Show All")) showAllShells();
    ImGui::Separator();
    if (ImGui::MenuItem("Delete", "Delete", false, !none)) deleteSelectedShells();
    if (ImGui::BeginMenu("Save As", !none)) {
        drawShellSavePopup();
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Merge...", nullptr, false, selected.size() >= 2)) openOperator(ToolSlot::MergeShells);
    if (selected.size() < 2) {
        tooltip("Select two or more shells to merge: Cmd/Ctrl+click or Shift+click rows, or Select... > Visible shells.");
    }
    ImGui::MenuItem("Flip Normals (future)", nullptr, false, false);
}

void UiMockup::drawShellSelectPopup(const std::vector<ShellHealth>& health) {
    if (!ImGui::BeginPopup("ShellSelect")) {
        return;
    }
    if (ImGui::MenuItem("All")) selectShellsWhere([](int) { return true; });
    if (ImGui::MenuItem("None", "Esc")) selectShellsWhere([](int) { return false; });
    if (ImGui::MenuItem("Invert")) {
        selectShellsWhere([this](int s) { return !shellFlags_[static_cast<std::size_t>(s)].selected; });
    }
    if (ImGui::MenuItem("Visible shells")) {
        selectShellsWhere([this](int s) { return shellFlags_[static_cast<std::size_t>(s)].visible; });
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Unhealthy (orange)", nullptr, false, healthKnown())) {
        selectShellsWhere([&](int s) { return health[static_cast<std::size_t>(s)].total > 0; });
    }
    if (ImGui::MenuItem("Nested inside another shell")) {
        selectShellsWhere([this](int s) {
            const int in = shells_[static_cast<std::size_t>(s)].nestedIn;
            return in >= 0 && shellLive(rootShell(in)) && rootShell(in) != s;
        });
    }
    ImGui::Separator();
    const std::string small = "Smaller than " + withThousands(static_cast<std::size_t>(smallShellFaces_)) + " faces";
    if (ImGui::MenuItem(small.c_str())) {
        const std::vector<std::size_t> faces = shellFaceCounts();
        selectShellsWhere([&](int s) { return faces[static_cast<std::size_t>(s)] < static_cast<std::size_t>(smallShellFaces_); });
    }
    ImGui::SetNextItemWidth(100.0f);
    ImGui::InputInt("N faces", &smallShellFaces_, 0);
    smallShellFaces_ = std::max(smallShellFaces_, 1);
    ImGui::EndPopup();
}

void UiMockup::drawShellSavePopup() {
    if (ImGui::MenuItem("Selected shells to one STL...")) saveSelectedShells(false);
    if (ImGui::MenuItem("Each selected shell to its own STL...")) saveSelectedShells(true);
    ImGui::TextDisabled("Mockup: binary STL in the working directory.");
}

// -----------------------------------------------------------------------------
// Operation: Merge Shells
// -----------------------------------------------------------------------------

void UiMockup::drawMergeTool() {
    const std::vector<int> roots = selectedShells();
    ImGui::TextWrapped("Boolean union of the selected shells into one shell. Faces where they cross are removed, "
                       "and gaps smaller than the gap tolerance are closed first.");
    if (roots.size() < 2) {
        ImGui::TextDisabled("Select two or more shells in the Shells panel (Select... > All for everything).");
        if (ImGui::Button("Cancel")) toolSlot_ = ToolSlot::None;
        return;
    }
    const std::vector<ShellHealth> health = shellHealth();
    const std::vector<std::size_t> faces = shellFaceCounts();
    std::size_t faceCount = 0;
    for (const int s : roots) faceCount += faces[static_cast<std::size_t>(s)];
    ImGui::Text("%s shells selected (%s faces).", withThousands(roots.size()).c_str(), withThousands(faceCount).c_str());
    const bool known = healthKnown();
    if (!known) {
        ImGui::TextColored(kDefectColor, "Run Check All (F5) first: the checklist comes from Mesh Health.");
    }

    struct Rule {
        const char* label;
        std::array<std::size_t, 2> rows;
    };
    const std::array<Rule, 4> rules = {{
        {"closed (no open edges)", {kOpenEdges, kOpenEdges}},
        {"manifold edges and vertices", {kNonManifoldEdges, kNonManifoldVertices}},
        {"consistently oriented", {kFlippedNormals, kFlippedNormals}},
        {"free of self-crossings", {kIntersections, kIntersections}},
    }};
    ImGui::Separator();
    ImGui::TextUnformatted("Before merging, every shell must be:");
    bool allPass = known;
    if (ImGui::BeginTable("MergeRules", 3, ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("##dot", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##rule", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##count", ImGuiTableColumnFlags_WidthFixed);
        for (std::size_t r = 0; r < rules.size(); ++r) {
            const Rule& rule = rules[r];
            std::vector<int> failing;
            for (const int s : roots) {
                const ShellHealth& h = health[static_cast<std::size_t>(s)];
                const std::size_t n = h.counts[rule.rows[0]] + (rule.rows[1] != rule.rows[0] ? h.counts[rule.rows[1]] : 0);
                if (n > 0) failing.push_back(s);
            }
            allPass = allPass && failing.empty();
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(r));
            ImGui::TableSetColumnIndex(0);
            if (known) {
                dot(failing.empty() ? kOkColor : kDefectColor);
            } else {
                statusDot(HealthStatus::NotRun);
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(rule.label);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%s/%s", withThousands(roots.size() - failing.size()).c_str(), withThousands(roots.size()).c_str());
            if (known && !failing.empty()) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Show")) {
                    selectShellsWhere([&](int s) { return std::find(failing.begin(), failing.end(), s) != failing.end(); });
                    setStatusMessage("Selected the " + withThousands(failing.size()) + " shell(s) that are not " + rule.label +
                                     ". Isolate (I) and fix them, then select all again.");
                }
                tooltip("Select only the shells that fail this check.");
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    std::size_t between = 0;
    for (const MockDefect& d : categories_[kIntersections].defects) {
        const auto inGroup = [&](int s) { return std::find(roots.begin(), roots.end(), s) != roots.end(); };
        if (d.shell >= 0 && inGroup(rootShell(d.shell))) {
            const int a = rootShell(d.shell);
            between += std::any_of(d.partnerShells.begin(), d.partnerShells.end(), [&](int p) {
                return rootShell(p) != a && inGroup(rootShell(p));
            }) ? 1 : 0;
        }
    }
    ImGui::TextDisabled("%s faces cross between these shells; the union removes them.", withThousands(between).c_str());
    ImGui::TextDisabled("Thin, degenerate and duplicate faces do not block a merge.");
    ImGui::PushItemWidth(110.0f);
    ImGui::InputDouble("Gap tolerance (x size)", &mergeGapTolerance_, 0.0, 0.0, "%.0e");
    ImGui::PopItemWidth();
    helpMarker("Gaps between shells smaller than this fraction of the model size are closed before the union. "
               "Larger gaps stay open (PLAN.md Milestone 4).");

    ImGui::Separator();
    ImGui::BeginDisabled(!allPass);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.45f, 0.25f, 1.0f));
    if (ImGui::Button("Apply Merge")) applyMerge(roots);
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (!allPass && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Every selected shell must pass the checklist above. Use Show to find the ones that fail.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) toolSlot_ = ToolSlot::None;
    ImGui::TextDisabled("Apply is one undo step (%s+Z). Mockup: simulated.", kMod);
}

} // namespace ogun::mockup
