#include "app/AppController.hpp"
#include "core/Constants.hpp"
#include "core/Settings.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <system_error>
#include <unordered_map>
#include <utility>

namespace ogun {

const char* previewDisplayModeLabel(PreviewDisplayMode mode) noexcept {
    switch (mode) {
    case PreviewDisplayMode::TranslucentBase:
        return "Translucent Base";
    case PreviewDisplayMode::TranslucentPatch:
        return "Translucent Patch";
    case PreviewDisplayMode::CutoutOriginalEdges:
        return "Cutout + Original Edges";
    }
    return "Unknown";
}

AppController::AppController()
    : thresholds_(ThinFaceThresholds::defaultsForUnit(Unit::Millimeter)) {
    mesh_.fileUnit = Unit::Millimeter;
}

std::filesystem::path AppController::resolveApplicationIconPath() noexcept {
    const std::array<std::filesystem::path, 4> candidates = {
        std::filesystem::path("assets/icons/ogun.png"),
        std::filesystem::path("assets/icons/ogun.jpeg"),
#ifdef OGUN_SOURCE_ICON_PNG_PATH
        std::filesystem::path(OGUN_SOURCE_ICON_PNG_PATH),
#else
        std::filesystem::path{},
#endif
#ifdef OGUN_SOURCE_ICON_JPEG_PATH
        std::filesystem::path(OGUN_SOURCE_ICON_JPEG_PATH),
#else
        std::filesystem::path{},
#endif
    };

    std::error_code ec;
    for (const auto& candidate : candidates) {
        if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

bool AppController::loadStlFile(const std::filesystem::path& filepath) {
    Mesh loadedMesh;
    std::string err;
    if (!ogun::loadStl(filepath, loadedMesh, mesh_.fileUnit, toleranceConfig_, &err)) {
        lastErrorMessage_ = std::move(err);
        return false;
    }

    lastErrorMessage_.clear();
    const std::string label = filepath.filename().empty()
        ? filepath.string()
        : filepath.filename().string();
    setMesh(std::move(loadedMesh), label);
    return true;
}

void AppController::loadSampleDefectiveMesh() {
    lastErrorMessage_.clear();
    setMesh(SampleMeshes::createSampleDefectiveMesh(mesh_.fileUnit), "Sample Defective Mesh");
}

void AppController::setMesh(Mesh mesh, std::string sourceLabel) {
    loadedSourceLabel_ = std::move(sourceLabel);
    undoStack_.clear();
    lastRepairStatusMessage_.clear();
    replaceMeshPreservingHistory(std::move(mesh));
}

void AppController::replaceMeshPreservingHistory(Mesh mesh) {
    mesh_ = std::move(mesh);
    rebuildCgalMesh();
    hasDiagnosticsReport_ = false;
    thinFaceReport_ = ThinFaceReport{};
    clearSelection();
}

void AppController::rebuildCgalMesh() {
    cgalMeshValid_ = toCgalSurfaceMesh(mesh_, cgalMesh_);
}

bool AppController::saveStlFile(const std::filesystem::path& filepath, StlFormat format) {
    std::string err;
    if (!ogun::saveStl(filepath, mesh_, format, &err)) {
        lastErrorMessage_ = err.empty() ? "Failed to save STL file." : std::move(err);
        return false;
    }
    lastErrorMessage_.clear();
    return true;
}

void AppController::setFileUnit(Unit newUnit) noexcept {
    const Unit oldUnit = mesh_.fileUnit;
    if (oldUnit == newUnit) {
        return;
    }

    ogun::setFileUnit(mesh_, newUnit);
    thresholds_ = thresholds_.convertedToUnit(oldUnit, newUnit);

    if (hasDiagnosticsReport_) {
        thinFaceReport_.unit = newUnit;
        thinFaceReport_.thresholds = thresholds_;
    }
}

void AppController::scaleMeshCoordinatesToUnit(Unit targetUnit) {
    const Unit oldUnit = mesh_.fileUnit;
    if (oldUnit == targetUnit) {
        return;
    }

    pushUndoEntry(
        "Scale Mesh Coordinates (" + std::string(unitSymbol(oldUnit)) + " -> " +
        std::string(unitSymbol(targetUnit)) + ")");
    ogun::scaleMeshToUnit(mesh_, targetUnit);
    thresholds_ = thresholds_.convertedToUnit(oldUnit, targetUnit);
    rebuildCgalMesh();

    if (hasDiagnosticsReport_) {
        const SelectionMode prevMode = selectionMode_;
        const Eigen::Index prevFace = selectedFaceIndex_;
        thinFaceReport_ = ThinFaceChecker::check(mesh_, thresholds_);
        if (prevMode == SelectionMode::SingleFace && prevFace >= 0 && prevFace < mesh_.numFaces()) {
            selectSingleFace(prevFace);
        } else if (prevMode == SelectionMode::AllThinFaces && thinFaceReport_.hasDefects()) {
            selectAllThinFaces();
        } else if (thinFaceReport_.hasDefects()) {
            selectAllThinFaces();
        } else {
            clearSelection();
        }
    } else {
        resetOrbitPivotToSelection();
    }
}

const ThinFaceReport& AppController::runCheckManifold() {
    thinFaceReport_ = ThinFaceChecker::check(mesh_, thresholds_);
    hasDiagnosticsReport_ = true;

    if (thinFaceReport_.hasDefects()) {
        selectAllThinFaces();
    } else {
        clearSelection();
    }

    return thinFaceReport_;
}

void AppController::selectAllThinFaces() {
    if (!hasDiagnosticsReport_ || thinFaceReport_.empty()) {
        clearSelection();
        return;
    }
    selectionMode_ = SelectionMode::AllThinFaces;
    selectedFaceIndex_ = -1;
    highlightedFaceIndices_ = thinFaceReport_.faceIndices;
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    resetSingleFaceRepairState();
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::selectSingleFace(Eigen::Index faceIndex) {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        clearSelection();
        return;
    }
    const bool newSelection = selectedFaceIndex_ != faceIndex || selectionMode_ != SelectionMode::SingleFace;
    if (newSelection) {
        lastFocusedFaceIndex_ = -1;
        lastFocusedNormalSign_ = 0;
    }
    selectionMode_ = SelectionMode::SingleFace;
    selectedFaceIndex_ = faceIndex;
    highlightedFaceIndices_ = {faceIndex};
    orbitPivot_ = computeSelectionCentroid();

    selectedFaceAnalysis_ = cgalMeshValid_
        ? SliverHealer::analyzeFace(
              cgalMesh_,
              CgalFaceIndex(static_cast<CgalSurfaceMesh::size_type>(faceIndex)),
              thresholds_)
        : SliverAnalysis{};
    if (newSelection) {
        selectedHealAction_ = selectedFaceAnalysis_.recommendedAction;
    }
    refreshRepairPreview();
}

void AppController::clearSelection() noexcept {
    selectionMode_ = SelectionMode::None;
    selectedFaceIndex_ = -1;
    highlightedFaceIndices_.clear();
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    resetSingleFaceRepairState();
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::resetSingleFaceRepairState() noexcept {
    selectedFaceAnalysis_ = SliverAnalysis{};
    selectedHealAction_ = HealAction::None;
    repairPreview_ = SingleFaceRepairPreview{};
}

void AppController::refreshRepairPreview() {
    if (selectionMode_ != SelectionMode::SingleFace || !cgalMeshValid_) {
        repairPreview_ = SingleFaceRepairPreview{};
        return;
    }
    repairPreview_ = SliverHealer::previewSingleFaceRepair(
        cgalMesh_,
        CgalFaceIndex(static_cast<CgalSurfaceMesh::size_type>(selectedFaceIndex_)),
        selectedHealAction_,
        mesh_.fileUnit,
        thresholds_);
}

void AppController::setPreviewBaseOpacity(double opacity) noexcept {
    if (std::isfinite(opacity)) {
        previewBaseOpacity_ = std::clamp(opacity, settings::kMinPreviewOpacity, 1.0);
    }
}

void AppController::setPreviewPatchOpacity(double opacity) noexcept {
    if (std::isfinite(opacity)) {
        previewPatchOpacity_ = std::clamp(opacity, settings::kMinPreviewOpacity, 1.0);
    }
}

bool AppController::previewUsesCutout() const noexcept {
    return repairPreviewShown() &&
           !repairPreview_.sourceFaceIndices.empty() &&
           previewDisplayMode_ == PreviewDisplayMode::CutoutOriginalEdges;
}

HighlightOverlayGeometry AppController::buildRepairPreviewOriginalEdges() const {
    HighlightOverlayGeometry overlay;
    if (previewUsesCutout()) {
        appendFaceGeometry(repairPreview_.sourceFaceIndices, overlay);
    }
    return overlay;
}

BaseMeshDisplay AppController::buildBaseMeshDisplay() const {
    BaseMeshDisplay display;
    const bool cutPreview = previewUsesCutout();
    if (mesh_.empty() || (highlightedFaceIndices_.empty() && !cutPreview)) {
        return display;
    }

    std::vector<bool> hidden(static_cast<std::size_t>(mesh_.numFaces()), false);
    const auto hide = [&](const std::vector<Eigen::Index>& faces) {
        for (const Eigen::Index f : faces) {
            if (f >= 0 && f < mesh_.numFaces()) {
                hidden[static_cast<std::size_t>(f)] = true;
            }
        }
    };
    hide(highlightedFaceIndices_);
    if (cutPreview) {
        hide(repairPreview_.sourceFaceIndices);
    }

    display.faceIndices.reserve(hidden.size());
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        if (!hidden[static_cast<std::size_t>(f)]) {
            display.faceIndices.push_back(f);
        }
    }
    display.F.resize(static_cast<Eigen::Index>(display.faceIndices.size()), 3);
    for (Eigen::Index row = 0; row < display.F.rows(); ++row) {
        display.F.row(row) = mesh_.F.row(display.faceIndices[static_cast<std::size_t>(row)]);
    }
    display.isCut = true;
    return display;
}

void AppController::setSelectedHealAction(HealAction action) {
    if (selectionMode_ != SelectionMode::SingleFace) {
        return;
    }
    selectedHealAction_ = action;
    refreshRepairPreview();
}

bool AppController::commitSelectedFaceRepair() {
    if (selectionMode_ != SelectionMode::SingleFace || selectedHealAction_ == HealAction::None) {
        lastRepairStatusMessage_ = "Select a face and a repair strategy first.";
        return false;
    }

    const Eigen::Index faceIndex = selectedFaceIndex_;
    const HealAction action = selectedHealAction_;
    const std::string faceLabel = "Face #" + std::to_string(faceIndex);
    RepairResults results = SliverHealer::healSingleFace(mesh_, faceIndex, action, thresholds_);
    if (!results.success) {
        lastRepairStatusMessage_ = faceLabel + ": " + results.statusMessage;
        return false;
    }

    pushUndoEntry("Single-Face Repair (" + faceLabel + ")");
    replaceMeshPreservingHistory(std::move(results.mesh));
    runCheckManifold();
    lastRepairStatusMessage_ = std::string(healActionLabel(action)) + " committed on " + faceLabel + " (" +
                               std::to_string(thinFaceReport_.numDefects()) + " thin faces remaining).";
    return true;
}

bool AppController::fixAllThinFaces() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }

    RepairResults results = SliverHealer::healAllThinFaces(mesh_, thresholds_);
    if (results.initialThinFaces == 0) {
        lastRepairStatusMessage_ = "No thin / sliver faces to heal.";
        return false;
    }
    if (!results.success) {
        lastRepairStatusMessage_ = results.statusMessage;
        return false;
    }

    pushUndoEntry("Fix All Thin / Sliver Faces");
    replaceMeshPreservingHistory(std::move(results.mesh));
    runCheckManifold();
    lastRepairStatusMessage_ = std::move(results.statusMessage);
    return true;
}

void AppController::pushUndoEntry(std::string actionLabel) {
    undoStack_.push_back(UndoEntry{
        mesh_,
        thresholds_,
        hasDiagnosticsReport_,
        selectionMode_,
        selectedFaceIndex_,
        selectedHealAction_,
        std::move(actionLabel)});
    if (undoStack_.size() > settings::kMaxUndoHistoryDepth) {
        undoStack_.erase(undoStack_.begin());
    }
}

std::string AppController::lastUndoActionLabel() const {
    return undoStack_.empty() ? std::string{} : undoStack_.back().actionLabel;
}

bool AppController::undoLastModification() {
    if (undoStack_.empty()) {
        return false;
    }
    UndoEntry entry = std::move(undoStack_.back());
    undoStack_.pop_back();

    replaceMeshPreservingHistory(std::move(entry.mesh));
    thresholds_ = entry.thresholds;
    if (entry.hadDiagnosticsReport) {
        thinFaceReport_ = ThinFaceChecker::check(mesh_, thresholds_);
        hasDiagnosticsReport_ = true;
    }

    if (entry.selectionMode == SelectionMode::SingleFace &&
        entry.selectedFaceIndex >= 0 && entry.selectedFaceIndex < mesh_.numFaces()) {
        selectSingleFace(entry.selectedFaceIndex);
        if (entry.selectedHealAction != HealAction::None && entry.selectedHealAction != selectedHealAction_) {
            setSelectedHealAction(entry.selectedHealAction);
        }
    } else if (entry.selectionMode == SelectionMode::AllThinFaces) {
        selectAllThinFaces();
    } else {
        clearSelection();
    }

    lastRepairStatusMessage_ = "Undid: " + entry.actionLabel;
    return true;
}

HighlightOverlayGeometry AppController::buildHighlightOverlay() const {
    HighlightOverlayGeometry overlay;
    if (mesh_.empty()) {
        return overlay;
    }

    const Eigen::Vector3d kCleanFaceColor(0.70, 0.75, 0.80);
    const Eigen::Vector3d kSecondaryDefectColor(0.95, 0.58, 0.16);
    const Eigen::Vector3d& activeHighlightColor = overlayAppearance_.selectedFaces.color;

    const auto numFaces = static_cast<std::size_t>(mesh_.numFaces());
    overlay.faceColors.assign(numFaces, kCleanFaceColor);

    if (hasDiagnosticsReport_) {
        for (const Eigen::Index defectFaceIdx : thinFaceReport_.faceIndices) {
            if (defectFaceIdx >= 0 && defectFaceIdx < mesh_.numFaces()) {
                overlay.faceColors[static_cast<std::size_t>(defectFaceIdx)] = kSecondaryDefectColor;
            }
        }
    }

    for (const Eigen::Index faceIdx : highlightedFaceIndices_) {
        if (faceIdx >= 0 && faceIdx < mesh_.numFaces()) {
            overlay.faceColors[static_cast<std::size_t>(faceIdx)] = activeHighlightColor;
        }
    }
    appendFaceGeometry(highlightedFaceIndices_, overlay);
    return overlay;
}

void AppController::appendFaceGeometry(
    const std::vector<Eigen::Index>& faces,
    HighlightOverlayGeometry& overlay) const {
    std::unordered_map<int, std::size_t> globalVertToOverlayNode;
    std::vector< std::pair<std::size_t, std::size_t> > seenUndirectedEdges;

    for (const Eigen::Index faceIdx : faces) {
        if (faceIdx < 0 || faceIdx >= mesh_.numFaces()) {
            continue;
        }

        std::array<std::size_t, 3> localNodes{};
        for (int c = 0; c < 3; ++c) {
            const int vIdx = mesh_.F(faceIdx, c);
            auto [it, inserted] = globalVertToOverlayNode.try_emplace(vIdx, overlay.overlayPoints.size());
            if (inserted) {
                overlay.overlayPoints.emplace_back(mesh_.V.row(vIdx).transpose());
            }
            localNodes[static_cast<std::size_t>(c)] = it->second;
        }
        overlay.overlayFaces.push_back(localNodes);
        overlay.overlayFaceIndices.push_back(faceIdx);

        for (std::size_t c = 0; c < 3; ++c) {
            const std::size_t u = localNodes[c];
            const std::size_t v = localNodes[(c + 1) % 3];
            const auto canonicalEdge = std::minmax(u, v);
            if (std::find(seenUndirectedEdges.begin(), seenUndirectedEdges.end(), canonicalEdge) ==
                seenUndirectedEdges.end()) {
                seenUndirectedEdges.push_back(canonicalEdge);
                overlay.overlayEdges.push_back({u, v});
            }
        }
    }
}

void AppController::setGhostMeshOpacity(double opacity) noexcept {
    ghostMeshOpacity_ = std::clamp(opacity, 0.05, 1.0);
}

bool SurfaceStyle::adoptPolyscopeEdgeWidth(double polyscopeWidth) noexcept {
    if (!(polyscopeWidth > 0.0)) {
        const bool changed = showEdges;
        showEdges = false;
        return changed;
    }
    if (!showEdges) {
        // Polyscope's "Edges" checkbox re-enables edges at exactly 1.0; keep the remembered width instead
        showEdges = true;
        if (polyscopeWidth != 1.0) {
            edgeWidthPx = polyscopeWidth;
        }
        return true;
    }
    if (std::abs(polyscopeWidth - edgeWidthPx) > 1.0e-4) {
        edgeWidthPx = polyscopeWidth;
        return true;
    }
    return false;
}

void AppController::setOverlayAppearance(const OverlayAppearance& appearance) noexcept {
    const auto clampColor = [](const Eigen::Vector3d& c, const Eigen::Vector3d& fallback) {
        return c.allFinite() ? c.cwiseMax(0.0).cwiseMin(1.0).eval() : fallback;
    };
    const auto clampWidth = [](double width, double fallback) {
        return std::isfinite(width)
            ? std::clamp(width, settings::kMinOverlayEdgeWidthPx, settings::kMaxOverlayEdgeWidthPx)
            : fallback;
    };
    const auto clampStyle = [&](const SurfaceStyle& in, SurfaceStyle& out) {
        out.color = clampColor(in.color, out.color);
        out.edgeColor = clampColor(in.edgeColor, out.edgeColor);
        out.edgeWidthPx = clampWidth(in.edgeWidthPx, out.edgeWidthPx);
        out.showEdges = in.showEdges;
    };
    clampStyle(appearance.selectedFaces, overlayAppearance_.selectedFaces);
    clampStyle(appearance.previewPatch, overlayAppearance_.previewPatch);
    overlayAppearance_.originalEdgeColor = clampColor(appearance.originalEdgeColor, overlayAppearance_.originalEdgeColor);
    overlayAppearance_.originalEdgeWidthPx = clampWidth(appearance.originalEdgeWidthPx, overlayAppearance_.originalEdgeWidthPx);
    if (std::isfinite(appearance.originalEdgeOpacity)) {
        overlayAppearance_.originalEdgeOpacity = std::clamp(appearance.originalEdgeOpacity, settings::kMinPreviewOpacity, 1.0);
    }
    overlayAppearance_.pointColor = clampColor(appearance.pointColor, overlayAppearance_.pointColor);
    if (std::isfinite(appearance.pointRadiusPx)) {
        overlayAppearance_.pointRadiusPx = std::clamp(
            appearance.pointRadiusPx,
            settings::kMinHighlightPointRadiusPx,
            settings::kMaxHighlightPointRadiusPx);
    }
}

double AppController::worldLengthPerPixel(
    double viewDistance,
    double fovDegrees,
    double viewportHeightPx) noexcept {
    if (!std::isfinite(viewDistance) || viewDistance <= 0.0 || !(viewportHeightPx > 0.0)) {
        return 0.0;
    }
    const double clampedFovDeg = std::clamp(
        fovDegrees,
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double viewHeight = 2.0 * viewDistance * std::tan((clampedFovDeg * 0.5) * kDegToRad);
    return viewHeight / viewportHeightPx;
}

double AppController::worldLengthAtSelection(
    double pixels,
    const Eigen::Vector3d& cameraPosition,
    double fovDegrees,
    double viewportHeightPx) const noexcept {
    if (mesh_.empty()) {
        return 0.0;
    }
    const double dist = (cameraPosition - computeSelectionCentroid()).norm();
    return pixels * worldLengthPerPixel(dist, fovDegrees, viewportHeightPx);
}

double AppController::referenceFullMeshViewDistance() const noexcept {
    if (mesh_.empty()) {
        return 1.0;
    }
    const double meshDiag = mesh_.boundingBox().diagonal();
    return std::max(1.0e-9, kReferenceFullMeshDistanceFactor * std::max(meshDiag, 1.0e-9));
}

double AppController::computeHighlightZoomScale(
    const Eigen::Vector3d& cameraPosition,
    double fovDegrees) const noexcept {
    if (mesh_.empty()) {
        return 1.0;
    }

    const Eigen::Vector3d targetCenter = computeSelectionCentroid();
    const double camDist = (cameraPosition - targetCenter).norm();
    const double refDist = referenceFullMeshViewDistance();
    if (!std::isfinite(camDist) || refDist <= 0.0) {
        return 1.0;
    }

    constexpr double kDefaultFovRadHalf = (settings::kDefaultCameraFovDegrees * 0.5) * kDegToRad;
    const double clampedFovDeg = std::clamp(
        fovDegrees,
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double fovRadHalf = (clampedFovDeg * 0.5) * kDegToRad;
    const double fovFactor = std::tan(fovRadHalf) / std::tan(kDefaultFovRadHalf);

    const double rawScale = (camDist / refDist) * fovFactor;
    return std::clamp(rawScale, kMinHighlightZoomScale, 1.0);
}


Eigen::Vector3d AppController::computeFaceCentroid(Eigen::Index faceIndex) const noexcept {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return Eigen::Vector3d::Zero();
    }
    const int i0 = mesh_.F(faceIndex, 0);
    const int i1 = mesh_.F(faceIndex, 1);
    const int i2 = mesh_.F(faceIndex, 2);
    if (i0 < 0 || i0 >= mesh_.numVertices() ||
        i1 < 0 || i1 >= mesh_.numVertices() ||
        i2 < 0 || i2 >= mesh_.numVertices()) {
        return Eigen::Vector3d::Zero();
    }
    const Eigen::Vector3d v0 = mesh_.V.row(i0).transpose();
    const Eigen::Vector3d v1 = mesh_.V.row(i1).transpose();
    const Eigen::Vector3d v2 = mesh_.V.row(i2).transpose();
    return (v0 + v1 + v2) / 3.0;
}

Eigen::Vector3d AppController::computeSelectionCentroid() const noexcept {
    if (mesh_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    if (selectionMode_ == SelectionMode::SingleFace &&
        selectedFaceIndex_ >= 0 &&
        selectedFaceIndex_ < mesh_.numFaces()) {
        return computeFaceCentroid(selectedFaceIndex_);
    }
    return computeActiveSelectionOrMeshBounds().center();
}

Eigen::Vector3d AppController::computeFaceInspectionNormal(Eigen::Index faceIndex) const noexcept {
    if (mesh_.empty() || faceIndex < 0 || faceIndex >= mesh_.numFaces()) {
        return Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    if (mesh_.N.rows() == mesh_.numFaces() && mesh_.N.cols() >= 3) {
        const Eigen::Vector3d cachedN = mesh_.N.row(faceIndex).transpose();
        const double cachedNorm = cachedN.norm();
        if (cachedNorm > 1.0e-12 && std::isfinite(cachedNorm)) {
            return cachedN / cachedNorm;
        }
    }

    const int i0 = mesh_.F(faceIndex, 0);
    const int i1 = mesh_.F(faceIndex, 1);
    const int i2 = mesh_.F(faceIndex, 2);
    if (i0 >= 0 && i0 < mesh_.numVertices() &&
        i1 >= 0 && i1 < mesh_.numVertices() &&
        i2 >= 0 && i2 < mesh_.numVertices()) {
        const Eigen::Vector3d v0 = mesh_.V.row(i0).transpose();
        const Eigen::Vector3d v1 = mesh_.V.row(i1).transpose();
        const Eigen::Vector3d v2 = mesh_.V.row(i2).transpose();

        const Eigen::Vector3d cross = (v1 - v0).cross(v2 - v0);
        const double crossNorm = cross.norm();
        if (crossNorm > 1.0e-15 && std::isfinite(crossNorm)) {
            return cross / crossNorm;
        }

        // Zero-area collinear triangle fallback: pick a unit vector perpendicular to the longest edge
        const Eigen::Vector3d e0 = v1 - v0;
        const Eigen::Vector3d e1 = v2 - v1;
        const Eigen::Vector3d e2 = v0 - v2;
        Eigen::Vector3d longestEdge = e0;
        if (e1.squaredNorm() > longestEdge.squaredNorm()) {
            longestEdge = e1;
        }
        if (e2.squaredNorm() > longestEdge.squaredNorm()) {
            longestEdge = e2;
        }
        const double edgeLen = longestEdge.norm();
        if (edgeLen > 1.0e-15 && std::isfinite(edgeLen)) {
            const Eigen::Vector3d edgeDir = longestEdge / edgeLen;
            const Eigen::Vector3d refAxis = (std::abs(edgeDir.z()) < 0.9)
                ? Eigen::Vector3d(0.0, 0.0, 1.0)
                : Eigen::Vector3d(0.0, 1.0, 0.0);
            const Eigen::Vector3d perp = edgeDir.cross(refAxis);
            const double perpNorm = perp.norm();
            if (perpNorm > 1.0e-12) {
                return perp / perpNorm;
            }
        }
    }

    return Eigen::Vector3d(0.0, 0.0, 1.0);
}

void AppController::setOrbitPivot(const Eigen::Vector3d& pivot) noexcept {
    if (pivot.allFinite()) {
        orbitPivot_ = pivot;
    }
}

void AppController::translateOrbitPivot(const Eigen::Vector3d& worldDelta) noexcept {
    if (worldDelta.allFinite()) {
        orbitPivot_ += worldDelta;
    }
}

void AppController::resetOrbitPivotToSelection() noexcept {
    orbitPivot_ = computeSelectionCentroid();
}

void AppController::resetOrbitPivotToMeshCenter() noexcept {
    lastFocusedFaceIndex_ = -1;
    lastFocusedNormalSign_ = 0;
    if (mesh_.empty()) {
        orbitPivot_ = Eigen::Vector3d::Zero();
    } else {
        orbitPivot_ = mesh_.boundingBox().center();
    }
}

Eigen::Vector3d AppController::computeOrthogonalUpVector(
    const Eigen::Vector3d& lookDirection,
    const Eigen::Vector3d& preferredUp) noexcept {
    Eigen::Vector3d lookDir = lookDirection;
    if (lookDir.norm() <= 1.0e-12 || !lookDir.allFinite()) {
        lookDir = Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
        lookDir.normalize();
    }

    Eigen::Vector3d up = preferredUp;
    if (up.norm() <= 1.0e-12 || !up.allFinite()) {
        up = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
        up.normalize();
    }

    Eigen::Vector3d projUp = up - up.dot(lookDir) * lookDir;
    if (projUp.norm() > 1.0e-3) {
        return projUp.normalized();
    }

    // `preferredUp` is nearly collinear with `lookDir`: pick a stable orthogonal fallback axis
    const Eigen::Vector3d fallbackAxis = (std::abs(lookDir.z()) < 0.9)
        ? Eigen::Vector3d(0.0, 0.0, 1.0)
        : Eigen::Vector3d(1.0, 0.0, 0.0);
    projUp = fallbackAxis - fallbackAxis.dot(lookDir) * lookDir;
    return projUp.normalized();
}

BoundingBox AppController::computeActiveSelectionOrMeshBounds() const noexcept {
    bool initializedBounds = false;
    BoundingBox bounds;

    for (const Eigen::Index faceIdx : highlightedFaceIndices_) {
        if (faceIdx < 0 || faceIdx >= mesh_.numFaces()) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            const int vIdx = mesh_.F(faceIdx, c);
            if (vIdx < 0 || vIdx >= mesh_.numVertices()) {
                continue;
            }
            const Eigen::Vector3d p = mesh_.V.row(vIdx).transpose();
            if (!initializedBounds) {
                bounds.min = p;
                bounds.max = p;
                initializedBounds = true;
            } else {
                bounds.min = bounds.min.cwiseMin(p);
                bounds.max = bounds.max.cwiseMax(p);
            }
        }
    }

    if (!initializedBounds && !mesh_.empty()) {
        bounds = mesh_.boundingBox();
    }
    return bounds;
}

CameraFocusTarget AppController::computeCameraFocusTarget(
    const Eigen::Vector3d& currentViewDirection,
    FocusNormalSide normalSide,
    const Eigen::Vector3d& preferredUpDirection) const {
    CameraFocusTarget focus;
    if (mesh_.empty()) {
        return focus;
    }

    const BoundingBox bounds = computeActiveSelectionOrMeshBounds();
    focus.selectionBounds = bounds;
    focus.targetCenter = computeSelectionCentroid();

    const double selDiag = bounds.diagonal();
    const double meshDiag = std::max(mesh_.boundingBox().diagonal(), 1.0e-6);
    const double minRadius = 1.0e-3 * meshDiag;
    const double rawRadius = (selDiag > 0.0)
        ? (1.75 * selDiag)
        : (0.1 * meshDiag);
    focus.focusRadius = std::max(rawRadius, minRadius);

    Eigen::Vector3d viewDir = currentViewDirection;
    int resolvedNormalSign = 0;

    if (selectionMode_ == SelectionMode::SingleFace &&
        selectedFaceIndex_ >= 0 &&
        selectedFaceIndex_ < mesh_.numFaces() &&
        normalSide != FocusNormalSide::PreserveViewDirection) {
        if (normalSide == FocusNormalSide::NegativeNormal) {
            resolvedNormalSign = -1;
        } else if (normalSide == FocusNormalSide::PositiveNormal) {
            resolvedNormalSign = +1;
        } else if (normalSide == FocusNormalSide::AutoToggleNormal) {
            resolvedNormalSign = (lastFocusedFaceIndex_ == selectedFaceIndex_ && lastFocusedNormalSign_ == +1)
                ? -1
                : +1;
        }

        const Eigen::Vector3d faceNormal = computeFaceInspectionNormal(selectedFaceIndex_);
        // Looking at the face from the +N side means the camera sits at +N and looks along -N
        viewDir = -static_cast<double>(resolvedNormalSign) * faceNormal;
    }

    if (viewDir.norm() <= 1.0e-12 || !viewDir.allFinite()) {
        viewDir = Eigen::Vector3d(-1.0, -1.0, -1.0);
    }
    viewDir.normalize();

    focus.normalSideSign = resolvedNormalSign;
    focus.cameraPosition = focus.targetCenter - viewDir * focus.focusRadius;
    focus.cameraUpDirection = computeOrthogonalUpVector(viewDir, preferredUpDirection);
    focus.valid = true;
    return focus;
}

CameraFocusTarget AppController::focusOnSelection(
    FocusNormalSide normalSide,
    const Eigen::Vector3d& currentViewDirection,
    const Eigen::Vector3d& preferredUpDirection) {
    const CameraFocusTarget focus = computeCameraFocusTarget(
        currentViewDirection,
        normalSide,
        preferredUpDirection);
    if (!focus.valid) {
        return focus;
    }

    orbitPivot_ = focus.targetCenter;
    if (selectionMode_ == SelectionMode::SingleFace && focus.normalSideSign != 0) {
        lastFocusedFaceIndex_ = selectedFaceIndex_;
        lastFocusedNormalSign_ = focus.normalSideSign;
    } else {
        lastFocusedFaceIndex_ = -1;
        lastFocusedNormalSign_ = 0;
    }
    return focus;
}

} // namespace ogun
