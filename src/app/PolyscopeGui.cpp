#include "app/PolyscopeGui.hpp"
#include "core/Constants.hpp"
#include "core/Settings.hpp"

#ifdef __APPLE__
#include "app/MacAppIcon.hpp"
#endif

#include <imgui.h>
#include <polyscope/curve_network.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/render/engine.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>
#include <portable-file-dialogs.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <queue>
#include <string>
#include <vector>

extern "C" {
unsigned char* stbi_load(const char* filename, int* x, int* y, int* channels_in_file, int desired_channels);
void stbi_image_free(void* retval_from_stbi_load);
}

namespace ogun {

namespace {

/// When loading a 3-channel JPEG icon that has a white/shadow background outside the rounded
/// metallic frame, flood-fills the outer margin to transparent alpha with 2px edge feathering.
void makeOuterBackgroundTransparent(std::vector<unsigned char>& rgba, int w, int h) {
    if (w <= 0 || h <= 0 || rgba.size() < static_cast<std::size_t>(w * h * 4)) {
        return;
    }

    auto isBgCandidate = [&](int idx) noexcept {
        const int r = rgba[static_cast<std::size_t>(idx) * 4 + 0];
        const int g = rgba[static_cast<std::size_t>(idx) * 4 + 1];
        const int b = rgba[static_cast<std::size_t>(idx) * 4 + 2];
        const int maxCh = std::max({r, g, b});
        const int minCh = std::min({r, g, b});
        const int diff = maxCh - minCh;
        return (minCh >= 230 && diff <= 12) || (minCh >= 175 && diff <= 6);
    };

    std::vector<std::uint8_t> visited(static_cast<std::size_t>(w * h), 0);
    std::queue<int> q;

    for (int x = 0; x < w; ++x) {
        const int topIdx = x;
        const int botIdx = (h - 1) * w + x;
        if (isBgCandidate(topIdx)) {
            visited[static_cast<std::size_t>(topIdx)] = 1;
            q.push(topIdx);
        }
        if (isBgCandidate(botIdx)) {
            visited[static_cast<std::size_t>(botIdx)] = 1;
            q.push(botIdx);
        }
    }
    for (int y = 0; y < h; ++y) {
        const int leftIdx = y * w;
        const int rightIdx = y * w + (w - 1);
        if (!visited[static_cast<std::size_t>(leftIdx)] && isBgCandidate(leftIdx)) {
            visited[static_cast<std::size_t>(leftIdx)] = 1;
            q.push(leftIdx);
        }
        if (!visited[static_cast<std::size_t>(rightIdx)] && isBgCandidate(rightIdx)) {
            visited[static_cast<std::size_t>(rightIdx)] = 1;
            q.push(rightIdx);
        }
    }

    constexpr int kDx[4] = {1, -1, 0, 0};
    constexpr int kDy[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        const int cur = q.front();
        q.pop();
        const int cx = cur % w;
        const int cy = cur / w;
        for (int d = 0; d < 4; ++d) {
            const int nx = cx + kDx[d];
            const int ny = cy + kDy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
                continue;
            }
            const int nidx = ny * w + nx;
            if (!visited[static_cast<std::size_t>(nidx)] && isBgCandidate(nidx)) {
                visited[static_cast<std::size_t>(nidx)] = 1;
                q.push(nidx);
            }
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int idx = y * w + x;
            if (visited[static_cast<std::size_t>(idx)]) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 0;
                continue;
            }
            bool nearBg1 = false;
            bool nearBg2 = false;
            for (int oy = -2; oy <= 2; ++oy) {
                for (int ox = -2; ox <= 2; ++ox) {
                    const int nx = std::clamp(x + ox, 0, w - 1);
                    const int ny = std::clamp(y + oy, 0, h - 1);
                    if (visited[static_cast<std::size_t>(ny * w + nx)]) {
                        if (std::abs(ox) <= 1 && std::abs(oy) <= 1) {
                            nearBg1 = true;
                        } else {
                            nearBg2 = true;
                        }
                    }
                }
            }
            if (nearBg1) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 128;
            } else if (nearBg2) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 220;
            } else {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 255;
            }
        }
    }
}

constexpr std::array<PreviewDisplayMode, 3> kPreviewDisplayModes = {
    PreviewDisplayMode::CutoutOriginalEdges,
    PreviewDisplayMode::TranslucentBase,
    PreviewDisplayMode::TranslucentPatch,
};
constexpr double kColorReadbackTolerance = 1.0e-4;
const ImVec4 kWarningTextColor(1.0f, 0.35f, 0.35f, 1.0f);
const ImVec4 kSuccessTextColor(0.28f, 0.90f, 0.42f, 1.0f);

struct HealActionOption {
    HealAction action;
    const char* label;
};

constexpr std::array<HealActionOption, 4> kHealActionOptions = {{
    {HealAction::FlipLongestEdge, "Flip Longest Edge (Best for Coplanar Caps)"},
    {HealAction::CollapseShortestEdge, "Collapse Shortest Edge (Best for Needles)"},
    {HealAction::SnapCollinearCollapse, "Snap Collinear Vertex (Cap on Feature Crease)"},
    {HealAction::RemoveDegenerateFace, "Remove Degenerate Flap (Isolated / Redundant Face)"},
}};

glm::vec3 toGlm(const Eigen::Vector3d& v) {
    return glm::vec3(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

Eigen::Vector3d fromGlm(const glm::vec3& v) {
    return Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
}

/// Adopts `polyscopeValue` into `target` when the user changed it in Polyscope's UI.
bool adoptColor(const glm::vec3& polyscopeValue, Eigen::Vector3d& target) {
    const Eigen::Vector3d value = fromGlm(polyscopeValue);
    if ((value - target).cwiseAbs().maxCoeff() <= kColorReadbackTolerance) {
        return false;
    }
    target = value;
    return true;
}

/// Re-sizing is skipped unless the target radius drifted noticeably from the last applied one.
bool radiusIsStale(double radius, double lastApplied, bool forceUpdate) {
    return radius > 0.0 &&
           (forceUpdate || lastApplied <= 0.0 || std::abs(radius - lastApplied) > 2.0e-3 * lastApplied);
}

void removeSurfaceMeshIfPresent(const char* name) {
    if (polyscope::hasSurfaceMesh(name)) {
        polyscope::removeSurfaceMesh(name, false);
    }
}

void removeCurveNetworkIfPresent(const char* name) {
    if (polyscope::hasCurveNetwork(name)) {
        polyscope::removeCurveNetwork(name, false);
    }
}

void removePointCloudIfPresent(const char* name) {
    if (polyscope::hasPointCloud(name)) {
        polyscope::removePointCloud(name, false);
    }
}

} // namespace

PolyscopeGui::PolyscopeGui(AppController& controller)
    : controller_(controller) {}

void PolyscopeGui::run() {
    polyscope::options::programName = "Ogun - STL Mesh Inspection & Repair Tool";
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::options::openImGuiWindowForUserCallback = false;

    polyscope::init();
    polyscope::state::doDefaultMouseInteraction = false;
    applyTransparencyModeForPreview();
    loadAndApplyApplicationIcon();

    if (controller_.hasMesh()) {
        syncMeshStructureToPolyscope(true);
    }

    polyscope::state::userCallback = [this]() {
        drawUiFrame();
    };

    polyscope::show();
    appIconTexture_.reset();
}

void PolyscopeGui::loadAndApplyApplicationIcon() {
    const std::filesystem::path iconPath = AppController::resolveApplicationIconPath();
    if (iconPath.empty()) {
        return;
    }

    int width = 0;
    int height = 0;
    int channelsInFile = 0;
    unsigned char* rawPixels = stbi_load(iconPath.string().c_str(), &width, &height, &channelsInFile, 4);
    if (rawPixels == nullptr || width <= 0 || height <= 0) {
        if (rawPixels != nullptr) {
            stbi_image_free(rawPixels);
        }
        return;
    }

    const std::size_t byteCount =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u;
    std::vector<unsigned char> rgba(rawPixels, rawPixels + byteCount);
    stbi_image_free(rawPixels);

    if (channelsInFile == 3) {
        makeOuterBackgroundTransparent(rgba, width, height);
    }

#ifdef __APPLE__
    setMacApplicationIconFromRgba(rgba.data(), width, height);
#endif

    if (polyscope::render::engine != nullptr) {
        appIconTexture_ = polyscope::render::engine->generateTextureBuffer(
            polyscope::TextureFormat::RGBA8,
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            rgba.data());
        if (appIconTexture_ != nullptr) {
            appIconTexture_->setFilterMode(polyscope::FilterMode::Linear);
        }
    }
}

void PolyscopeGui::drawUiFrame() {
    syncStylesFromPolyscopeUi();
    handleViewportMouseNavigation();
    handleKeyboardShortcuts();

    const float winWidth = static_cast<float>(polyscope::view::windowWidth);
    const float winHeight = static_cast<float>(polyscope::view::windowHeight);
    const float panelWidth = 460.0f;
    const float panelHeight = std::min(780.0f, std::max(480.0f, winHeight - 32.0f));
    const float defaultRightX = std::max(330.0f, winWidth - panelWidth - 16.0f);

    if (!positionedMainWindowOnce_ && winWidth > 0.0f) {
        ImGui::SetNextWindowPos(ImVec2(defaultRightX, 16.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_FirstUseEver);
        positionedMainWindowOnce_ = true;
    }

    if (ImGui::Begin("Ogun - STL Diagnostics & Inspection")) {
        drawFileBarSection();
        ImGui::Separator();
        drawUnitsAndScaleSection();
        ImGui::Separator();
        drawDiagnosticsSection();
        ImGui::Separator();
        drawMeshRepairSection();
        ImGui::Separator();
        drawHighlightAndNavigationSection();
    }
    ImGui::End();

    if (showNavigationHelpWindow_) {
        drawNavigationHelpWindow();
    }

    applyEffectiveHighlightSizingToPolyscope(false);
}

void PolyscopeGui::drawFileBarSection() {
    if (appIconTexture_ != nullptr) {
        ImGui::Image(
            reinterpret_cast<ImTextureID>(appIconTexture_->getNativeHandle()),
            ImVec2(40.0f, 40.0f));
        ImGui::SameLine();
        ImGui::BeginGroup();
    }

    ImGui::TextUnformatted("Mesh Source");

    if (ImGui::Button("Open STL...")) {
        auto selection = pfd::open_file(
            "Open STL File",
            "",
            {"STL Mesh (*.stl)", "*.stl *.STL", "All Files", "*"},
            pfd::opt::none).result();
        if (!selection.empty()) {
            if (controller_.loadStlFile(std::filesystem::path(selection.front()))) {
                saveStatusMessage_.clear();
                syncMeshStructureToPolyscope(true);
            }
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Load Sample Test Mesh")) {
        controller_.loadSampleDefectiveMesh();
        saveStatusMessage_.clear();
        syncMeshStructureToPolyscope(true);
    }

    ImGui::BeginDisabled(!controller_.hasMesh());
    if (ImGui::Button("Save STL...")) {
        auto destination = pfd::save_file(
            "Save STL File",
            "repaired.stl",
            {"STL Mesh (*.stl)", "*.stl *.STL", "All Files", "*"},
            pfd::opt::none).result();
        if (!destination.empty()) {
            std::filesystem::path outPath(destination);
            if (!outPath.has_extension()) {
                outPath.replace_extension(".stl");
            }
            saveStatusMessage_ = controller_.saveStlFile(outPath, exportFormat_)
                ? "Saved " + outPath.filename().string()
                : std::string{};
        }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    const char* formatLabel = (exportFormat_ == StlFormat::Binary) ? "Binary STL" : "ASCII STL";
    if (ImGui::BeginCombo("##ExportFormat", formatLabel)) {
        if (ImGui::Selectable("Binary STL", exportFormat_ == StlFormat::Binary)) {
            exportFormat_ = StlFormat::Binary;
        }
        if (ImGui::Selectable("ASCII STL", exportFormat_ == StlFormat::Ascii)) {
            exportFormat_ = StlFormat::Ascii;
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    drawUndoButton("FileBar");

    if (appIconTexture_ != nullptr) {
        ImGui::EndGroup();
    }

    ImGui::Text("Loaded: %s", controller_.loadedSourceLabel().c_str());
    if (controller_.hasMesh()) {
        ImGui::Text(
            "Vertices: %lld   |   Triangles: %lld",
            static_cast<long long>(controller_.mesh().numVertices()),
            static_cast<long long>(controller_.mesh().numFaces()));
    } else {
        ImGui::TextDisabled("No mesh loaded. Click 'Open STL...' or 'Load Sample Test Mesh'.");
    }

    if (!controller_.lastErrorMessage().empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kWarningTextColor);
        ImGui::TextWrapped("Error: %s", controller_.lastErrorMessage().c_str());
        ImGui::PopStyleColor();
    } else if (!saveStatusMessage_.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kSuccessTextColor);
        ImGui::TextWrapped("%s", saveStatusMessage_.c_str());
        ImGui::PopStyleColor();
    }
}

void PolyscopeGui::drawUndoButton(const char* idSuffix) {
    const std::string undoLabel = controller_.canUndo()
        ? "Undo: " + controller_.lastUndoActionLabel() + " (Ctrl+Z / Cmd+Z)##" + idSuffix
        : std::string("Undo (Ctrl+Z / Cmd+Z)##") + idSuffix;
    ImGui::BeginDisabled(!controller_.canUndo());
    if (ImGui::Button(undoLabel.c_str())) {
        undoLastModification();
    }
    ImGui::EndDisabled();
}

void PolyscopeGui::undoLastModification() {
    if (controller_.undoLastModification()) {
        syncModifiedMeshToPolyscope();
    }
}

void PolyscopeGui::drawUnitsAndScaleSection() {
    ImGui::TextUnformatted("Units & Coordinate Scale");

    const Unit currentUnit = controller_.fileUnit();
    const std::string currentLabel(unitDisplayName(currentUnit));
    ImGui::SetNextItemWidth(175.0f);
    if (ImGui::BeginCombo("File Units", currentLabel.c_str())) {
        for (const Unit candidate : kAllUnits) {
            const bool isSelected = (candidate == currentUnit);
            const std::string itemLabel(unitDisplayName(candidate));
            if (ImGui::Selectable(itemLabel.c_str(), isSelected)) {
                controller_.setFileUnit(candidate);
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Note: Changing File Units re-interprets units without altering vertex coordinates.");
    ImGui::PopTextWrapPos();

    const std::string sym(unitSymbol(controller_.fileUnit()));
    if (controller_.hasMesh()) {
        const BoundingBox bbox = controller_.mesh().boundingBox();
        const Eigen::Vector3d ext = bbox.extents();
        ImGui::Text(
            "Bounding Box: %.4g x %.4g x %.4g %s (diag: %.4g %s)",
            ext.x(), ext.y(), ext.z(), sym.c_str(),
            bbox.diagonal(), sym.c_str());
        ImGui::Text(
            "  Min: (%.4g, %.4g, %.4g) %s",
            bbox.min.x(), bbox.min.y(), bbox.min.z(), sym.c_str());
        ImGui::Text(
            "  Max: (%.4g, %.4g, %.4g) %s",
            bbox.max.x(), bbox.max.y(), bbox.max.z(), sym.c_str());
    }

    if (ImGui::TreeNode("Scale Mesh Coordinates...")) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.78f, 0.28f, 1.0f));
        ImGui::TextWrapped(
            "Warning: This action multiplies vertex coordinates V to convert geometry from %s into the target unit.",
            sym.c_str());
        ImGui::PopStyleColor();

        const std::string targetLabel(unitDisplayName(pendingScaleTargetUnit_));
        ImGui::SetNextItemWidth(175.0f);
        if (ImGui::BeginCombo("Target Unit", targetLabel.c_str())) {
            for (const Unit candidate : kAllUnits) {
                const bool isSelected = (candidate == pendingScaleTargetUnit_);
                const std::string itemLabel(unitDisplayName(candidate));
                if (ImGui::Selectable(itemLabel.c_str(), isSelected)) {
                    pendingScaleTargetUnit_ = candidate;
                }
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        const double factor = unitConversionFactor(controller_.fileUnit(), pendingScaleTargetUnit_);
        ImGui::Text("Coordinate multiplier: %.6g", factor);

        ImGui::BeginDisabled(!controller_.hasMesh() || pendingScaleTargetUnit_ == controller_.fileUnit());
        if (ImGui::Button("Scale Mesh Coordinates Now")) {
            controller_.scaleMeshCoordinatesToUnit(pendingScaleTargetUnit_);
            polyscope::resetSelection();
            syncMeshStructureToPolyscope(true);
        }
        ImGui::EndDisabled();

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Preferences / Welding & Tolerance")) {
        ToleranceConfig tol = controller_.toleranceConfig();
        bool changed = false;
        ImGui::PushItemWidth(110.0f);
        if (ImGui::InputDouble("Atomic Floor (cm)", &tol.absFloorCm, 0.0, 0.0, "%.0e")) {
            tol.absFloorCm = std::max(0.0, tol.absFloorCm);
            changed = true;
        }
        if (ImGui::InputDouble("Relative Tolerance", &tol.relTol, 0.0, 0.0, "%.0e")) {
            tol.relTol = std::max(0.0, tol.relTol);
            changed = true;
        }
        ImGui::PopItemWidth();
        if (changed) {
            controller_.setToleranceConfig(tol);
        }
        ImGui::Text(
            "Active floor in %s: %.0e %s",
            sym.c_str(),
            controller_.toleranceConfig().absFloorInUnit(controller_.fileUnit()),
            sym.c_str());
        if (ImGui::Button("Reset Default Tolerances")) {
            controller_.setToleranceConfig(ToleranceConfig{});
        }
        ImGui::TreePop();
    }
}

void PolyscopeGui::drawDiagnosticsSection() {
    ImGui::TextUnformatted("Diagnostics -- Check Manifold (Thin / Sliver Faces)");

    const std::string sym(unitSymbol(controller_.fileUnit()));
    ThinFaceThresholds thresh = controller_.thresholds();
    bool threshChanged = false;

    ImGui::PushItemWidth(110.0f);
    const std::string minAltLabel = "Min Altitude h_min (" + sym + ")";
    if (ImGui::InputDouble(minAltLabel.c_str(), &thresh.minAltitude, 0.0, 0.0, "%.0e")) {
        thresh.minAltitude = std::max(0.0, thresh.minAltitude);
        threshChanged = true;
    }

    const std::string minAreaLabel = "Min Area A (" + sym + "^2)";
    if (ImGui::InputDouble(minAreaLabel.c_str(), &thresh.minArea, 0.0, 0.0, "%.0e")) {
        thresh.minArea = std::max(0.0, thresh.minArea);
        threshChanged = true;
    }

    if (ImGui::InputDouble("Max Aspect Ratio (l_max / h_min)", &thresh.maxAspectRatio, 0.0, 0.0, "%.0f")) {
        thresh.maxAspectRatio = std::max(1.0, thresh.maxAspectRatio);
        threshChanged = true;
    }
    ImGui::PopItemWidth();

    if (threshChanged) {
        controller_.setThresholds(thresh);
    }

    ImGui::BeginDisabled(!controller_.hasMesh());
    if (ImGui::Button("Check Manifold")) {
        controller_.runCheckManifold();
        syncHighlightOverlaysToPolyscope();
    }
    ImGui::EndDisabled();

    if (!controller_.hasDiagnosticsReport()) {
        ImGui::TextDisabled("Click 'Check Manifold' to inspect mesh facets.");
        return;
    }

    const ThinFaceReport& report = controller_.thinFaceReport();
    if (report.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kSuccessTextColor);
        ImGui::Text(
            "0 thin/sliver faces detected across %lld triangles.",
            static_cast<long long>(report.totalFacesChecked));
        ImGui::PopStyleColor();
        return;
    }

    char categoryHeader[128];
    std::snprintf(
        categoryHeader,
        sizeof(categoryHeader),
        "Thin / Near-Zero-Area Faces (%zu)  [Click to Highlight All]",
        report.numDefects());

    const bool categorySelected = (controller_.selectionMode() == SelectionMode::AllThinFaces);
    if (ImGui::Selectable(categoryHeader, categorySelected)) {
        controller_.selectAllThinFaces();
        syncHighlightOverlaysToPolyscope();
    }

    ImGui::BeginChild("DefectListScrollRegion", ImVec2(0.0f, 175.0f), true);
    for (const ThinFaceDefect& defect : report.defects) {
        char rowLabel[256];
        if (std::isfinite(defect.aspectRatio)) {
            std::snprintf(
                rowLabel,
                sizeof(rowLabel),
                "Face #%lld | h_min=%.2e %s | A=%.2e %s^2 | AR=%.1f",
                static_cast<long long>(defect.faceIndex),
                defect.minAltitude,
                sym.c_str(),
                defect.area,
                sym.c_str(),
                defect.aspectRatio);
        } else {
            std::snprintf(
                rowLabel,
                sizeof(rowLabel),
                "Face #%lld | h_min=%.2e %s | A=%.2e %s^2 | AR=INF",
                static_cast<long long>(defect.faceIndex),
                defect.minAltitude,
                sym.c_str(),
                defect.area,
                sym.c_str());
        }

        const bool rowSelected =
            (controller_.selectionMode() == SelectionMode::SingleFace) &&
            (controller_.selectedFaceIndex() == defect.faceIndex);

        if (ImGui::Selectable(rowLabel, rowSelected)) {
            controller_.selectSingleFace(defect.faceIndex);
            syncHighlightOverlaysToPolyscope();
        }
    }
    ImGui::EndChild();
}

void PolyscopeGui::drawMeshRepairSection() {
    ImGui::TextUnformatted("Mesh Repair -- Thin / Sliver Faces");

    const bool hasDefects = controller_.hasDiagnosticsReport() && controller_.thinFaceReport().hasDefects();
    ImGui::BeginDisabled(!hasDefects);
    if (ImGui::Button("Fix All Thin / Sliver Faces")) {
        if (controller_.fixAllThinFaces()) {
            syncModifiedMeshToPolyscope();
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    drawUndoButton("RepairSection");

    if (!controller_.lastRepairStatusMessage().empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", controller_.lastRepairStatusMessage().c_str());
        ImGui::PopTextWrapPos();
    }

    if (controller_.selectionMode() == SelectionMode::SingleFace) {
        drawSingleFaceRepairInspector();
    } else {
        ImGui::TextDisabled("Select a face in the defect list or click it in the 3D viewport to inspect it.");
    }
}

void PolyscopeGui::drawSingleFaceRepairInspector() {
    const SliverAnalysis& analysis = controller_.selectedFaceAnalysis();
    const SingleFaceRepairPreview& preview = controller_.repairPreview();

    ImGui::Spacing();
    ImGui::Text("Selected Face #%lld", static_cast<long long>(controller_.selectedFaceIndex()));
    ImGui::Text("Detected Type: %s", sliverTypeLabel(analysis.type));
    ImGui::Text(
        "Max Angle: %.3f deg   |   Min/Max Edge Ratio: %.3e",
        analysis.maxAngleDeg,
        analysis.edgeRatio);
    if (analysis.recommendedAction != HealAction::None) {
        ImGui::TextDisabled("Recommended: %s", healActionLabel(analysis.recommendedAction));
    }

    for (const HealActionOption& option : kHealActionOptions) {
        if (ImGui::RadioButton(option.label, controller_.selectedHealAction() == option.action)) {
            controller_.setSelectedHealAction(option.action);
            syncHighlightOverlaysToPolyscope();
        }
    }

    bool showPreview = controller_.repairPreviewVisible();
    if (ImGui::Checkbox("Show 3D Repair Preview Patch", &showPreview)) {
        controller_.setRepairPreviewVisible(showPreview);
        syncHighlightOverlaysToPolyscope();
    }

    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo("Preview Display", previewDisplayModeLabel(controller_.previewDisplayMode()))) {
        for (const PreviewDisplayMode mode : kPreviewDisplayModes) {
            const bool isSelected = (mode == controller_.previewDisplayMode());
            if (ImGui::Selectable(previewDisplayModeLabel(mode), isSelected)) {
                controller_.setPreviewDisplayMode(mode);
                applyTransparencyModeForPreview();
                syncHighlightOverlaysToPolyscope();
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::SetNextItemWidth(200.0f);
    if (controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentBase) {
        auto opacity = static_cast<float>(controller_.previewBaseOpacity());
        if (ImGui::SliderFloat("Base Mesh Opacity", &opacity, 0.05f, 1.0f, "%.2f")) {
            controller_.setPreviewBaseOpacity(static_cast<double>(opacity));
            syncMeshTransparencyToPolyscope();
        }
    } else if (controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentPatch) {
        auto opacity = static_cast<float>(controller_.previewPatchOpacity());
        if (ImGui::SliderFloat("Patch Opacity", &opacity, 0.05f, 1.0f, "%.2f")) {
            controller_.setPreviewPatchOpacity(static_cast<double>(opacity));
            syncMeshTransparencyToPolyscope();
        }
    } else if (controller_.previewDisplayMode() == PreviewDisplayMode::CutoutOriginalEdges) {
        OverlayAppearance appearance = controller_.overlayAppearance();
        bool appearanceChanged = false;
        auto widthPx = static_cast<float>(appearance.originalEdgeWidthPx);
        if (ImGui::SliderFloat(
                "Original Edge Width (px)",
                &widthPx,
                static_cast<float>(settings::kMinOverlayEdgeWidthPx),
                static_cast<float>(settings::kMaxOverlayEdgeWidthPx),
                "%.2f")) {
            appearance.originalEdgeWidthPx = static_cast<double>(widthPx);
            appearanceChanged = true;
        }
        ImGui::SetNextItemWidth(200.0f);
        auto opacity = static_cast<float>(appearance.originalEdgeOpacity);
        if (ImGui::SliderFloat("Original Edge Opacity", &opacity, 0.05f, 1.0f, "%.2f")) {
            appearance.originalEdgeOpacity = static_cast<double>(opacity);
            appearanceChanged = true;
        }
        if (appearanceChanged) {
            controller_.setOverlayAppearance(appearance);
            applyOverlayAppearanceToPolyscope();
        }
    }

    if (!preview.statusMessage.empty()) {
        const bool unsafe = !preview.valid && controller_.selectedHealAction() != HealAction::None;
        ImGui::PushStyleColor(ImGuiCol_Text, unsafe ? kWarningTextColor : kSuccessTextColor);
        ImGui::TextWrapped("%s%s", unsafe ? "Warning: " : "", preview.statusMessage.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::BeginDisabled(controller_.selectedHealAction() == HealAction::None);
    if (ImGui::Button("Commit Repair")) {
        if (controller_.commitSelectedFaceRepair()) {
            syncModifiedMeshToPolyscope();
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        if (controller_.hasDiagnosticsReport() && controller_.thinFaceReport().hasDefects()) {
            controller_.selectAllThinFaces();
        } else {
            controller_.clearSelection();
        }
        syncHighlightOverlaysToPolyscope();
    }
    ImGui::SameLine();
    drawUndoButton("Inspector");
}

void PolyscopeGui::drawHighlightAndNavigationSection() {
    ImGui::TextUnformatted("3D Highlight & Camera Navigation");

    OverlayAppearance appearance = controller_.overlayAppearance();
    bool appearanceChanged = false;
    ImGui::PushItemWidth(160.0f);
    auto pointRadiusPx = static_cast<float>(appearance.pointRadiusPx);
    if (ImGui::SliderFloat(
            "Defect Point Radius (px)",
            &pointRadiusPx,
            static_cast<float>(settings::kMinHighlightPointRadiusPx),
            static_cast<float>(settings::kMaxHighlightPointRadiusPx),
            "%.1f")) {
        appearance.pointRadiusPx = static_cast<double>(pointRadiusPx);
        appearanceChanged = true;
    }
    auto edgeWidthPx = static_cast<float>(appearance.selectedFaces.edgeWidthPx);
    if (ImGui::SliderFloat(
            "Defect Edge Width (px)",
            &edgeWidthPx,
            static_cast<float>(settings::kMinOverlayEdgeWidthPx),
            static_cast<float>(settings::kMaxOverlayEdgeWidthPx),
            "%.2f")) {
        appearance.selectedFaces.edgeWidthPx = static_cast<double>(edgeWidthPx);
        appearance.selectedFaces.showEdges = true;
        appearanceChanged = true;
    }
    ImGui::PopItemWidth();
    if (appearanceChanged) {
        controller_.setOverlayAppearance(appearance);
        applyOverlayAppearanceToPolyscope();
    }

    if (ImGui::Button("Reset Default Highlight Styles")) {
        controller_.resetOverlayAppearanceToDefaults();
        syncHighlightOverlaysToPolyscope();
    }
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled(
        "Color, Edges, Edge Color and Width of 'Selected Defect Faces' and 'Repair Preview Patch' are in Polyscope's Structures panel.");
    ImGui::PopTextWrapPos();

    bool ghost = controller_.ghostMeshEnabled();
    if (ImGui::Checkbox("Ghost / Transparent Mesh", &ghost)) {
        controller_.setGhostMeshEnabled(ghost);
        syncMeshTransparencyToPolyscope();
    }

    if (controller_.ghostMeshEnabled()) {
        ImGui::SetNextItemWidth(160.0f);
        auto opacity = static_cast<float>(controller_.ghostMeshOpacity());
        if (ImGui::SliderFloat("Ghost Mesh Opacity", &opacity, 0.08f, 0.90f, "%.2f")) {
            controller_.setGhostMeshOpacity(static_cast<double>(opacity));
            syncMeshTransparencyToPolyscope();
        }
    }

    ImGui::Checkbox("Show Polyscope Structures Sidebar", &polyscope::options::buildDefaultGuiPanels);

    if (controller_.hasMesh()) {
        const std::string sym(unitSymbol(controller_.fileUnit()));
        const Eigen::Vector3d& pivot = controller_.orbitPivot();
        if (controller_.selectionMode() == SelectionMode::SingleFace) {
            const int normSign = controller_.lastFocusedNormalSign();
            const char* sideTag = (normSign > 0) ? " [+Normal]" : ((normSign < 0) ? " [-Normal]" : "");
            ImGui::TextDisabled(
                "Orbit Pivot (Face #%lld Centroid%s): (%.4g, %.4g, %.4g) %s",
                static_cast<long long>(controller_.selectedFaceIndex()),
                sideTag,
                pivot.x(), pivot.y(), pivot.z(),
                sym.c_str());
        } else if (controller_.selectionMode() == SelectionMode::AllThinFaces) {
            ImGui::TextDisabled(
                "Orbit Pivot (Selection): (%.4g, %.4g, %.4g) %s",
                pivot.x(), pivot.y(), pivot.z(),
                sym.c_str());
        } else {
            ImGui::TextDisabled(
                "Orbit Pivot (Mesh Center): (%.4g, %.4g, %.4g) %s",
                pivot.x(), pivot.y(), pivot.z(),
                sym.c_str());
        }
    }

    ImGui::BeginDisabled(!controller_.hasMesh());
    if (ImGui::Button("Zoom +Normal (.)")) {
        focusCameraOnSelection(FocusNormalSide::PositiveNormal);
    }
    ImGui::SameLine();
    if (ImGui::Button("Zoom -Normal (Shift+.)")) {
        focusCameraOnSelection(FocusNormalSide::NegativeNormal);
    }

    if (ImGui::Button("Reset View (Home)")) {
        resetCameraAndPivotToHome();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Selection")) {
        controller_.clearSelection();
        syncHighlightOverlaysToPolyscope();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("? Navigation Help")) {
        showNavigationHelpWindow_ = !showNavigationHelpWindow_;
    }

    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled(
        "Tip: Press '.' for +Normal (press '.' again / '..' to toggle -Normal), 'Shift+.' for -Normal, 'Home' to reset.");
    ImGui::PopTextWrapPos();
}

void PolyscopeGui::drawNavigationHelpWindow() {
    ImGui::SetNextWindowSize(ImVec2(500.0f, 360.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("3D Viewport Navigation & Shortcuts", &showNavigationHelpWindow_)) {
        ImGui::TextWrapped(
            "Ogun uses Blender-inspired selection-centric camera navigation so inspecting microscopic sliver facets feels smooth at any zoom depth.");
        ImGui::Separator();

        ImGui::TextUnformatted("Mouse Controls (3D Viewport)");
        if (ImGui::BeginTable("MouseNavTable", 2, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 175.0f);
            ImGui::TableSetupColumn("Input / Behavior", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Orbit / Rotate");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Left-Click Drag (orbits around active Orbit Pivot)");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Pan Camera & Pivot");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Shift + Left-Click Drag, Right-Click Drag, or Middle-Click Drag (shifts Orbit Pivot with camera, scaled by depth)");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Zoom (Dolly)");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Scroll Wheel or Ctrl + Shift + Left-Click Drag (speed scales with distance to Orbit Pivot)");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Select Face / Clear Pick");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Left-Click a mesh face to select it, set the Orbit Pivot to its centroid, and open the Single-Face Repair Inspector with a live preview; Right-Click to clear the Polyscope pick");

            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::TextUnformatted("Keyboard Shortcuts & Orbit Pivot");
        if (ImGui::BeginTable("KeyNavTable", 2, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Shortcut", ImGuiTableColumnFlags_WidthFixed, 175.0f);
            ImGui::TableSetupColumn("Behavior", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Click Defect in List");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Highlights facet and immediately sets Orbit Pivot to its centroid (v0 + v1 + v2) / 3");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(".  or  Numpad .");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Frames selected facet looking down +Normal. Press '.' again (or '..') to toggle to -Normal!");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Shift + .");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Frames selected facet directly from the -Normal (back/inward) side");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Home  or  H");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Frames entire mesh and resets Orbit Pivot to the full mesh center");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Ctrl+Z  or  Cmd+Z");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Undoes the last mesh modification (single-face repair, Fix All Thin / Sliver Faces, or Scale Mesh Coordinates), one step at a time");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("?  or  F1");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("Toggles this Navigation & Shortcuts help window");

            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void PolyscopeGui::handleViewportMouseNavigation() {
    ImGuiIO& io = ImGui::GetIO();
    if (polyscope::view::windowWidth <= 0 || polyscope::view::windowHeight <= 0) {
        return;
    }

    bool widgetCapturedMouse = false;
    for (polyscope::WeakHandle<polyscope::Widget>& wHandle : polyscope::state::widgets) {
        if (wHandle.isValid()) {
            polyscope::Widget& w = wHandle.get();
            widgetCapturedMouse = w.interact();
            if (widgetCapturedMouse) {
                break;
            }
        }
    }

    if (io.WantCaptureMouse || widgetCapturedMouse) {
        return;
    }

    const Eigen::Vector3d camPos = currentCameraWorldPosition();
    const Eigen::Vector3d pivot = controller_.orbitPivot();
    const double rawDist = (camPos - pivot).norm();
    const double meshDiag = controller_.hasMesh()
        ? std::max(controller_.mesh().boundingBox().diagonal(), 1.0e-6)
        : 1.0;
    const double minNavDist = 1.0e-6 * meshDiag;
    const double effectiveDist = (std::isfinite(rawDist) && rawDist > minNavDist)
        ? rawDist
        : std::max(minNavDist, 1.0e-3 * meshDiag);

    // 1. Scroll wheel zoom (or Shift+Scroll clip plane shift)
    const double xoffset = io.MouseWheelH;
    const double yoffset = io.MouseWheel;
    if (xoffset != 0.0 || yoffset != 0.0) {
        const double maxScroll = (std::abs(yoffset) > std::abs(xoffset)) ? yoffset : xoffset;
        if (maxScroll != 0.0) {
            if (io.KeyShift) {
                polyscope::view::processClipPlaneShift(maxScroll);
            } else {
                processPivotZoom(maxScroll, effectiveDist);
            }
        }
    }

    // 2. Mouse drags (Orbit, Pan, or Drag-Zoom)
    const bool dragLeft = ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    const bool dragRight = !dragLeft && ImGui::IsMouseDragging(ImGuiMouseButton_Right);
    const bool dragMiddle = !dragLeft && !dragRight && ImGui::IsMouseDragging(ImGuiMouseButton_Middle);
    if (dragLeft || dragRight || dragMiddle) {
        const glm::vec2 dragDelta{
            io.MouseDelta.x / static_cast<float>(polyscope::view::windowWidth),
            -io.MouseDelta.y / static_cast<float>(polyscope::view::windowHeight)};
        dragDistSinceLastRelease_ += std::abs(dragDelta.x) + std::abs(dragDelta.y);

        const bool isDragZoom = dragLeft && io.KeyShift && io.KeyCtrl;
        const bool isTranslate = (dragLeft && io.KeyShift && !io.KeyCtrl) || dragRight || dragMiddle;
        const bool isRotate = dragLeft && !io.KeyShift && !io.KeyCtrl;

        if (isDragZoom) {
            processPivotZoom(static_cast<double>(dragDelta.y) * 5.0, effectiveDist);
        } else if (isRotate) {
            processPivotRotate(dragDelta.x, dragDelta.y);
        } else if (isTranslate) {
            processPivotTranslate(dragDelta.x, dragDelta.y, effectiveDist);
        }
    }

    // 3. Click picks on release when not dragging
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (dragDistSinceLastRelease_ < settings::kDragIgnoreThreshold) {
            const ImVec2 p = ImGui::GetMousePos();
            const polyscope::PickResult pickResult = polyscope::pickAtScreenCoords(glm::vec2{p.x, p.y});
            polyscope::setSelection(pickResult);
            handleViewportFacePick(pickResult);
        }
        dragDistSinceLastRelease_ = 0.0f;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        if (dragDistSinceLastRelease_ < settings::kDragIgnoreThreshold) {
            polyscope::resetSelection();
        }
        dragDistSinceLastRelease_ = 0.0f;
    }
}

void PolyscopeGui::handleViewportFacePick(const polyscope::PickResult& pickResult) {
    if (!pickResult.isHit || !controller_.hasMesh()) {
        return;
    }

    // Both pickable surfaces may be drawn with a subset of faces; map the local face back to the mesh
    const std::vector<Eigen::Index>* faceMap = nullptr;
    std::uint64_t nVerts = 0;
    std::uint64_t nFaces = 0;
    if (pickResult.structureName == kSurfaceMeshName) {
        nVerts = static_cast<std::uint64_t>(controller_.mesh().numVertices());
        nFaces = baseIsCut_
            ? static_cast<std::uint64_t>(baseFaceIndices_.size())
            : static_cast<std::uint64_t>(controller_.mesh().numFaces());
        faceMap = baseIsCut_ ? &baseFaceIndices_ : nullptr;
    } else if (pickResult.structureName == kDefectFacesName && polyscope::hasSurfaceMesh(kDefectFacesName)) {
        nVerts = static_cast<std::uint64_t>(polyscope::getSurfaceMesh(kDefectFacesName)->nVertices());
        nFaces = static_cast<std::uint64_t>(defectFaceIndices_.size());
        faceMap = &defectFaceIndices_;
    } else {
        return;
    }

    // Polyscope's combined pick index lists vertices [0, nVerts) before faces [nVerts, nVerts + nFaces)
    const std::uint64_t elementId = pickResult.localIndex;
    if (elementId < nVerts || elementId >= nVerts + nFaces) {
        return;
    }

    const auto localFace = static_cast<std::size_t>(elementId - nVerts);
    controller_.selectSingleFace(faceMap ? (*faceMap)[localFace] : static_cast<Eigen::Index>(localFace));
    syncHighlightOverlaysToPolyscope();
}

void PolyscopeGui::processPivotRotate(float dragDeltaX, float dragDeltaY) {
    if (dragDeltaX == 0.0f && dragDeltaY == 0.0f) {
        return;
    }
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None ||
        polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::Planar) {
        return;
    }

    glm::vec3 frameLookDir(0.0f, 0.0f, -1.0f);
    glm::vec3 frameUpDir(0.0f, 1.0f, 0.0f);
    glm::vec3 frameRightDir(1.0f, 0.0f, 0.0f);
    polyscope::view::getCameraFrame(frameLookDir, frameUpDir, frameRightDir);

    const Eigen::Vector3d& pivotEig = controller_.orbitPivot();
    const glm::vec3 pivot(
        static_cast<float>(pivotEig.x()),
        static_cast<float>(pivotEig.y()),
        static_cast<float>(pivotEig.z()));

    const float moveScale = static_cast<float>(polyscope::view::moveScale);
    const float delTheta = 2.0f * dragDeltaX * moveScale;
    const float delPhi = 2.0f * dragDeltaY * moveScale;

    switch (polyscope::view::getNavigateStyle()) {
    case polyscope::NavigateStyle::Turntable: {
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, pivot);
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, frameRightDir);
        polyscope::view::viewMat = polyscope::view::viewMat * phiCamR;
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, polyscope::view::getUpVec());
        polyscope::view::viewMat = polyscope::view::viewMat * thetaCamR;
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, -pivot);
        break;
    }
    case polyscope::NavigateStyle::Free:
    case polyscope::NavigateStyle::Arcball: {
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, pivot);
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, frameUpDir);
        polyscope::view::viewMat = polyscope::view::viewMat * thetaCamR;
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, frameRightDir);
        polyscope::view::viewMat = polyscope::view::viewMat * phiCamR;
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, -pivot);
        break;
    }
    case polyscope::NavigateStyle::FirstPerson: {
        const glm::vec3 rotAx = glm::mat3(polyscope::view::viewMat) * polyscope::view::getUpVec();
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, rotAx);
        polyscope::view::viewMat = thetaCamR * polyscope::view::viewMat;
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, glm::vec3(1.0f, 0.0f, 0.0f));
        polyscope::view::viewMat = phiCamR * polyscope::view::viewMat;
        break;
    }
    case polyscope::NavigateStyle::Planar:
    case polyscope::NavigateStyle::None:
        break;
    }

    controller_.notifyCameraRotated();
    polyscope::view::immediatelyEndFlight();
    polyscope::requestRedraw();
}

void PolyscopeGui::processPivotTranslate(float dragDeltaX, float dragDeltaY, double effectiveDist) {
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None) {
        return;
    }
    if (dragDeltaX == 0.0f && dragDeltaY == 0.0f) {
        return;
    }

    const double clampedFovDeg = std::clamp(
        static_cast<double>(polyscope::view::fov),
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double fovRadHalf = (clampedFovDeg * 0.5) * kDegToRad;
    const double viewHeightAtPivot = 2.0 * effectiveDist * std::tan(fovRadHalf);
    const auto movementScale = static_cast<float>(viewHeightAtPivot * polyscope::view::moveScale);

    const Eigen::Vector3d oldCamPos = currentCameraWorldPosition();
    const glm::mat4x4 camSpaceT = glm::translate(
        glm::mat4x4(1.0f),
        movementScale * glm::vec3(dragDeltaX, dragDeltaY, 0.0f));
    polyscope::view::viewMat = camSpaceT * polyscope::view::viewMat;
    const Eigen::Vector3d newCamPos = currentCameraWorldPosition();

    // Shift the orbit pivot in world space along with the camera so subsequent rotations stay centered
    controller_.translateOrbitPivot(newCamPos - oldCamPos);

    polyscope::view::immediatelyEndFlight();
    polyscope::requestRedraw();
}

void PolyscopeGui::processPivotZoom(double amount, double effectiveDist) {
    if (amount == 0.0) {
        return;
    }
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None ||
        polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::FirstPerson) {
        return;
    }

    if (polyscope::view::projectionMode == polyscope::ProjectionMode::Perspective) {
        const auto movementScale = static_cast<float>(effectiveDist * 0.12 * polyscope::view::moveScale);
        const glm::mat4x4 camSpaceT = glm::translate(
            glm::mat4x4(1.0f),
            glm::vec3(0.0f, 0.0f, movementScale * static_cast<float>(amount)));
        polyscope::view::viewMat = camSpaceT * polyscope::view::viewMat;
        polyscope::view::immediatelyEndFlight();
        polyscope::requestRedraw();
    } else {
        polyscope::view::processZoom(amount);
    }
}

void PolyscopeGui::handleKeyboardShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) {
        return;
    }

    if ((ImGui::IsKeyPressed(ImGuiKey_Slash, false) && io.KeyShift) ||
        ImGui::IsKeyPressed(ImGuiKey_F1, false)) {
        showNavigationHelpWindow_ = !showNavigationHelpWindow_;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && (io.KeyCtrl || io.KeySuper)) {
        undoLastModification();
    }

    if (!controller_.hasMesh()) {
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Period, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false)) {
        if (io.KeyShift) {
            focusCameraOnSelection(FocusNormalSide::NegativeNormal);
        } else {
            focusCameraOnSelection(FocusNormalSide::AutoToggleNormal);
        }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Home, false) ||
        ImGui::IsKeyPressed(ImGuiKey_H, false)) {
        resetCameraAndPivotToHome();
    }
}

void PolyscopeGui::syncMeshStructureToPolyscope(bool resetCameraToHome) {
    if (!controller_.hasMesh()) {
        removeSurfaceMeshIfPresent(kSurfaceMeshName);
        removeSurfaceMeshIfPresent(kDefectFacesName);
        removeSurfaceMeshIfPresent(kRepairPreviewName);
        removeCurveNetworkIfPresent(kOverlayCurveName);
        removeCurveNetworkIfPresent(kOriginalEdgesName);
        removePointCloudIfPresent(kOverlayPointName);
        baseIsCut_ = false;
        baseFaceIndices_.clear();
        defectFaceIndices_.clear();
        lastAppliedDefectEdgeRadius_ = -1.0;
        lastAppliedOriginalEdgeRadius_ = -1.0;
        lastAppliedPointRadius_ = -1.0;
        return;
    }

    syncHighlightOverlaysToPolyscope();

    if (resetCameraToHome) {
        controller_.resetOrbitPivotToSelection();
        polyscope::view::resetCameraToHomeView();
    }
}

void PolyscopeGui::syncHighlightOverlaysToPolyscope() {
    if (!controller_.hasMesh()) {
        return;
    }

    const HighlightOverlayGeometry overlay = controller_.buildHighlightOverlay();
    registerBaseMeshToPolyscope(overlay);
    syncDefectOverlaysToPolyscope(overlay);
    syncRepairPreviewToPolyscope();
    applyOverlayAppearanceToPolyscope();
    syncMeshTransparencyToPolyscope();
}

void PolyscopeGui::registerBaseMeshToPolyscope(const HighlightOverlayGeometry& overlay) {
    // Re-registering replaces the structure, so carry over the user's Structures-panel settings
    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        auto* previous = polyscope::getSurfaceMesh(kSurfaceMeshName);
        baseStyle_.color = fromGlm(previous->getSurfaceColor());
        baseStyle_.edgeColor = fromGlm(previous->getEdgeColor());
        baseStyle_.adoptPolyscopeEdgeWidth(previous->getEdgeWidth());
        baseMaterial_ = previous->getMaterial();
        baseEnabled_ = previous->isEnabled();
        baseStyleCaptured_ = true;
    }

    const Mesh& mesh = controller_.mesh();
    BaseMeshDisplay display = controller_.buildBaseMeshDisplay();
    baseIsCut_ = display.isCut;
    baseFaceIndices_ = std::move(display.faceIndices);
    if (baseIsCut_ && display.F.rows() == 0) {
        removeSurfaceMeshIfPresent(kSurfaceMeshName);
        return;
    }

    auto* psMesh = polyscope::registerSurfaceMesh(kSurfaceMeshName, mesh.V, baseIsCut_ ? display.F : mesh.F);
    if (baseStyleCaptured_) {
        psMesh->setSurfaceColor(toGlm(baseStyle_.color));
        psMesh->setMaterial(baseMaterial_);
        psMesh->setEnabled(baseEnabled_);
    }
    psMesh->setEdgeColor(toGlm(baseStyle_.edgeColor));
    psMesh->setEdgeWidth(baseStyle_.effectiveEdgeWidth());
    psMesh->setSmoothShade(false);
    psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Different);

    std::vector<Eigen::Vector3d> faceColors;
    if (baseIsCut_) {
        faceColors.reserve(baseFaceIndices_.size());
        for (const Eigen::Index f : baseFaceIndices_) {
            faceColors.push_back(overlay.faceColors[static_cast<std::size_t>(f)]);
        }
    } else {
        faceColors = overlay.faceColors;
    }
    psMesh->addFaceColorQuantity(kFaceColorQuantityName, faceColors)->setEnabled(true);
}

void PolyscopeGui::syncDefectOverlaysToPolyscope(const HighlightOverlayGeometry& overlay) {
    defectFaceIndices_ = overlay.overlayFaceIndices;
    lastAppliedDefectEdgeRadius_ = -1.0;
    lastAppliedPointRadius_ = -1.0;
    if (!overlay.hasSelectionOverlay()) {
        removeSurfaceMeshIfPresent(kDefectFacesName);
        removeCurveNetworkIfPresent(kOverlayCurveName);
        removePointCloudIfPresent(kOverlayPointName);
        return;
    }

    auto* psFaces = polyscope::registerSurfaceMesh(kDefectFacesName, overlay.overlayPoints, overlay.overlayFaces);
    psFaces->setSmoothShade(false);
    psFaces->setBackFacePolicy(polyscope::BackFacePolicy::Identical);

    // Edge tubes and vertex spheres keep zero-area and sub-pixel slivers visible at any zoom
    polyscope::registerCurveNetwork(kOverlayCurveName, overlay.overlayPoints, overlay.overlayEdges);
    polyscope::registerPointCloud(kOverlayPointName, overlay.overlayPoints);
}

void PolyscopeGui::syncMeshTransparencyToPolyscope() {
    float alpha = controller_.ghostMeshEnabled()
        ? static_cast<float>(controller_.ghostMeshOpacity())
        : 1.0f;
    const bool previewShown = controller_.repairPreviewShown();
    if (previewShown && controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentBase) {
        alpha = std::min(alpha, static_cast<float>(controller_.previewBaseOpacity()));
    }
    const bool translucentPatch = previewShown &&
                                  controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentPatch;

    // Polyscope switches to Pretty on its own when it first sees transparency; pick the mode's preference first
    if ((alpha < 1.0f || translucentPatch) &&
        polyscope::options::transparencyMode == polyscope::TransparencyMode::None) {
        applyTransparencyModeForPreview();
    }

    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        polyscope::getSurfaceMesh(kSurfaceMeshName)->setTransparency(alpha);
    }
    if (polyscope::hasSurfaceMesh(kRepairPreviewName)) {
        polyscope::getSurfaceMesh(kRepairPreviewName)->setTransparency(
            translucentPatch ? static_cast<float>(controller_.previewPatchOpacity()) : 1.0f);
    }
}

void PolyscopeGui::applyTransparencyModeForPreview() {
    // Translucent Base looks best with Simple blending; the other modes with Pretty depth peeling
    polyscope::options::transparencyMode =
        controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentBase
            ? polyscope::TransparencyMode::Simple
            : polyscope::TransparencyMode::Pretty;
}

void PolyscopeGui::syncRepairPreviewToPolyscope() {
    const Mesh& patch = controller_.repairPreview().previewPatchMesh;
    if (controller_.repairPreviewShown() && !patch.empty()) {
        auto* psPatch = polyscope::registerSurfaceMesh(kRepairPreviewName, patch.V, patch.F);
        psPatch->setSmoothShade(false);
        psPatch->setBackFacePolicy(polyscope::BackFacePolicy::Identical);
    } else {
        removeSurfaceMeshIfPresent(kRepairPreviewName);
    }

    lastAppliedOriginalEdgeRadius_ = -1.0;
    const HighlightOverlayGeometry originalEdges = controller_.buildRepairPreviewOriginalEdges();
    if (originalEdges.hasSelectionOverlay()) {
        polyscope::registerCurveNetwork(kOriginalEdgesName, originalEdges.overlayPoints, originalEdges.overlayEdges);
    } else {
        removeCurveNetworkIfPresent(kOriginalEdgesName);
    }
}

void PolyscopeGui::applyOverlayAppearanceToPolyscope() {
    const OverlayAppearance& appearance = controller_.overlayAppearance();
    const auto applySurfaceStyle = [](const char* name, const SurfaceStyle& style) {
        if (polyscope::hasSurfaceMesh(name)) {
            auto* psMesh = polyscope::getSurfaceMesh(name);
            psMesh->setSurfaceColor(toGlm(style.color));
            psMesh->setEdgeColor(toGlm(style.edgeColor));
            psMesh->setEdgeWidth(style.effectiveEdgeWidth());
        }
    };
    applySurfaceStyle(kDefectFacesName, appearance.selectedFaces);
    applySurfaceStyle(kRepairPreviewName, appearance.previewPatch);

    if (polyscope::hasCurveNetwork(kOverlayCurveName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOverlayCurveName);
        curveNet->setColor(toGlm(appearance.selectedFaces.edgeColor));
        curveNet->setEnabled(appearance.selectedFaces.showEdges);
    }
    if (polyscope::hasPointCloud(kOverlayPointName)) {
        polyscope::getPointCloud(kOverlayPointName)->setPointColor(toGlm(appearance.pointColor));
    }
    if (polyscope::hasCurveNetwork(kOriginalEdgesName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOriginalEdgesName);
        curveNet->setColor(toGlm(appearance.originalEdgeColor));
        curveNet->setTransparency(static_cast<float>(appearance.originalEdgeOpacity));
    }
    applyEffectiveHighlightSizingToPolyscope(true);
}

void PolyscopeGui::syncStylesFromPolyscopeUi() {
    if (!controller_.hasMesh()) {
        return;
    }

    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        auto* psMesh = polyscope::getSurfaceMesh(kSurfaceMeshName);
        if (baseStyle_.adoptPolyscopeEdgeWidth(psMesh->getEdgeWidth())) {
            psMesh->setEdgeWidth(baseStyle_.effectiveEdgeWidth());
        }
    }

    OverlayAppearance appearance = controller_.overlayAppearance();
    bool changed = false;
    const auto adoptSurfaceStyle = [&changed](const char* name, SurfaceStyle& style) {
        if (polyscope::hasSurfaceMesh(name)) {
            auto* psMesh = polyscope::getSurfaceMesh(name);
            changed |= adoptColor(psMesh->getSurfaceColor(), style.color);
            changed |= adoptColor(psMesh->getEdgeColor(), style.edgeColor);
            changed |= style.adoptPolyscopeEdgeWidth(psMesh->getEdgeWidth());
        }
    };
    adoptSurfaceStyle(kDefectFacesName, appearance.selectedFaces);
    adoptSurfaceStyle(kRepairPreviewName, appearance.previewPatch);

    if (polyscope::hasCurveNetwork(kOverlayCurveName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOverlayCurveName);
        changed |= adoptColor(curveNet->getColor(), appearance.selectedFaces.edgeColor);
        changed |= adoptOverlayRadius(
            static_cast<double>(curveNet->getRadius()), lastAppliedDefectEdgeRadius_, appearance.selectedFaces.edgeWidthPx);
    }
    if (polyscope::hasCurveNetwork(kOriginalEdgesName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOriginalEdgesName);
        changed |= adoptColor(curveNet->getColor(), appearance.originalEdgeColor);
        changed |= adoptOverlayRadius(
            static_cast<double>(curveNet->getRadius()), lastAppliedOriginalEdgeRadius_, appearance.originalEdgeWidthPx);
        const auto alpha = static_cast<double>(curveNet->getTransparency());
        if (std::abs(alpha - appearance.originalEdgeOpacity) > 1.0e-4) {
            appearance.originalEdgeOpacity = alpha;
            changed = true;
        }
    }
    if (polyscope::hasPointCloud(kOverlayPointName)) {
        auto* ptCloud = polyscope::getPointCloud(kOverlayPointName);
        changed |= adoptColor(ptCloud->getPointColor(), appearance.pointColor);
        changed |= adoptOverlayRadius(ptCloud->getPointRadius(), lastAppliedPointRadius_, appearance.pointRadiusPx);
    }

    if (changed) {
        controller_.setOverlayAppearance(appearance);
        applyOverlayAppearanceToPolyscope();
    }
}

bool PolyscopeGui::adoptOverlayRadius(double polyscopeRadius, double& lastApplied, double& valuePx) {
    if (lastApplied <= 0.0 || std::abs(polyscopeRadius - lastApplied) <= 1.0e-4 * lastApplied) {
        return false;
    }
    const double worldPerPx = controller_.worldLengthAtSelection(
        1.0,
        currentCameraWorldPosition(),
        polyscope::view::fov,
        static_cast<double>(polyscope::view::windowHeight));
    if (worldPerPx <= 0.0) {
        return false;
    }
    valuePx = polyscopeRadius / worldPerPx;
    lastApplied = polyscopeRadius;
    return true;
}

void PolyscopeGui::syncModifiedMeshToPolyscope() {
    polyscope::resetSelection();
    syncMeshStructureToPolyscope(false);
}

Eigen::Vector3d PolyscopeGui::currentCameraWorldPosition() const noexcept {
    const glm::vec3 camPos = polyscope::view::getCameraWorldPosition();
    return Eigen::Vector3d(
        static_cast<double>(camPos.x),
        static_cast<double>(camPos.y),
        static_cast<double>(camPos.z));
}

void PolyscopeGui::applyEffectiveHighlightSizingToPolyscope(bool forceUpdate) {
    if (!controller_.hasMesh()) {
        return;
    }

    const Eigen::Vector3d camPos = currentCameraWorldPosition();
    const double fovDeg = polyscope::view::fov;
    const double zScale = controller_.computeHighlightZoomScale(camPos, fovDeg);

    // Keep the near clip plane proportional when zoomed in close so microscopic defects are never clipped
    const double targetNearClip = std::clamp(
        polyscope::view::defaultNearClipRatio * zScale,
        1.0e-6,
        polyscope::view::defaultNearClipRatio);
    if (std::abs(polyscope::view::nearClipRatio - targetNearClip) > 1.0e-7) {
        polyscope::view::nearClipRatio = targetNearClip;
    }

    // Tubes and spheres are re-sized as the camera moves so they keep a constant on-screen size
    const OverlayAppearance& appearance = controller_.overlayAppearance();
    const auto toWorld = [&](double pixels) {
        return controller_.worldLengthAtSelection(
            pixels, camPos, fovDeg, static_cast<double>(polyscope::view::windowHeight));
    };

    const double defectEdgeRadius = toWorld(appearance.selectedFaces.edgeWidthPx);
    if (polyscope::hasCurveNetwork(kOverlayCurveName) &&
        radiusIsStale(defectEdgeRadius, lastAppliedDefectEdgeRadius_, forceUpdate)) {
        polyscope::getCurveNetwork(kOverlayCurveName)->setRadius(static_cast<float>(defectEdgeRadius), false);
        lastAppliedDefectEdgeRadius_ = defectEdgeRadius;
    }

    const double originalEdgeRadius = toWorld(appearance.originalEdgeWidthPx);
    if (polyscope::hasCurveNetwork(kOriginalEdgesName) &&
        radiusIsStale(originalEdgeRadius, lastAppliedOriginalEdgeRadius_, forceUpdate)) {
        polyscope::getCurveNetwork(kOriginalEdgesName)->setRadius(static_cast<float>(originalEdgeRadius), false);
        lastAppliedOriginalEdgeRadius_ = originalEdgeRadius;
    }

    const double pointRadius = toWorld(appearance.pointRadiusPx);
    if (polyscope::hasPointCloud(kOverlayPointName) &&
        radiusIsStale(pointRadius, lastAppliedPointRadius_, forceUpdate)) {
        polyscope::getPointCloud(kOverlayPointName)->setPointRadius(pointRadius, false);
        lastAppliedPointRadius_ = pointRadius;
    }
}

void PolyscopeGui::focusCameraOnSelection(FocusNormalSide normalSide) {
    if (!controller_.hasMesh()) {
        return;
    }

    glm::vec3 lookDir(0.0f, 0.0f, -1.0f);
    glm::vec3 upDir(0.0f, 1.0f, 0.0f);
    glm::vec3 rightDir(1.0f, 0.0f, 0.0f);
    polyscope::view::getCameraFrame(lookDir, upDir, rightDir);

    const Eigen::Vector3d currentViewDir(
        static_cast<double>(lookDir.x),
        static_cast<double>(lookDir.y),
        static_cast<double>(lookDir.z));

    const glm::vec3 sceneUp = polyscope::view::getUpVec();
    const Eigen::Vector3d preferredUpDir(
        static_cast<double>(sceneUp.x),
        static_cast<double>(sceneUp.y),
        static_cast<double>(sceneUp.z));

    const CameraFocusTarget focus = controller_.focusOnSelection(
        normalSide,
        currentViewDir,
        preferredUpDir);
    if (!focus.valid) {
        return;
    }

    const glm::vec3 camPos(
        static_cast<float>(focus.cameraPosition.x()),
        static_cast<float>(focus.cameraPosition.y()),
        static_cast<float>(focus.cameraPosition.z()));
    const glm::vec3 targetPos(
        static_cast<float>(focus.targetCenter.x()),
        static_cast<float>(focus.targetCenter.y()),
        static_cast<float>(focus.targetCenter.z()));
    const glm::vec3 camUp(
        static_cast<float>(focus.cameraUpDirection.x()),
        static_cast<float>(focus.cameraUpDirection.y()),
        static_cast<float>(focus.cameraUpDirection.z()));

    polyscope::view::lookAt(camPos, targetPos, camUp, true);
}

void PolyscopeGui::resetCameraAndPivotToHome() {
    if (!controller_.hasMesh()) {
        return;
    }
    controller_.resetOrbitPivotToMeshCenter();
    polyscope::view::flyToHomeView();
}

} // namespace ogun
