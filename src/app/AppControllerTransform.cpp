// AppController: Mesh > Transform (Set Origin to Selection and the Translate / Rotate tool).

#include "app/AppController.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

namespace ogun {

namespace {

[[nodiscard]] std::string vectorText(const Eigen::Vector3d& v) {
    char buffer[96];
    // + 0.0 turns -0 into 0
    std::snprintf(buffer, sizeof(buffer), "(%.6g, %.6g, %.6g)", v.x() + 0.0, v.y() + 0.0, v.z() + 0.0);
    return buffer;
}

[[nodiscard]] Eigen::Vector3d finiteOrZero(Eigen::Vector3d v) noexcept {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(v[i])) {
            v[i] = 0.0;
        }
    }
    return v;
}

} // namespace

std::optional<Eigen::Vector3d> AppController::selectionCentroid() const {
    if (faceSelectedCount_ == 0 || mesh_.empty()) {
        return std::nullopt;
    }
    const std::vector<std::uint8_t> shown = shownFaceMask();
    std::vector<Eigen::Index> faces;
    faces.reserve(faceSelectedCount_);
    for (std::size_t f = 0; f < faceSelected_.size(); ++f) {
        if (faceSelected_[f] != 0 && (shown.empty() || shown[f] != 0)) {
            faces.push_back(static_cast<Eigen::Index>(f));
        }
    }
    return facesCentroid(mesh_, faces);
}

bool AppController::setOriginToSelection() {
    const std::optional<Eigen::Vector3d> centroid = selectionCentroid();
    if (!centroid) {
        postStatusMessage("Select faces first (Shift+click, B or C).", true);
        return false;
    }
    if (centroid->isZero(0.0)) {
        postStatusMessage("The selection centroid is already at the origin.");
        return false;
    }
    const Eigen::Vector3d c = *centroid;
    if (!transformWholeMesh(MeshTransform{-c, Eigen::Vector3d::Zero()}, "Set Origin to Selection")) {
        return false;
    }
    lastRepairStatusMessage_ = "Moved the selection centroid " + vectorText(c) + " " +
                               std::string(unitSymbol(mesh_.fileUnit)) + " to the origin.";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::transformWholeMesh(const MeshTransform& transform, std::string actionLabel) {
    if (mesh_.empty() || transform.isIdentity()) {
        return false;
    }
    Mesh moved = mesh_;
    transformMesh(moved, transform);
    // Same faces, so the shells and their view state carry over exactly (only their boxes move)
    ShellBrowser shells = shells_;
    shells.updateBoxes(moved);
    std::vector<std::uint8_t> selection = faceSelected_;
    const std::size_t selectedCount = faceSelectedCount_;

    pushUndoEntry(std::move(actionLabel), true);
    finishMeshEdit(std::move(moved), std::move(shells));
    faceSelected_ = std::move(selection);
    faceSelectedCount_ = selectedCount;
    if (selectedCount > 0) {
        onFaceSelectionChanged();
    }
    return true;
}

void AppController::openTransform() {
    clearSelection();
    activeTool_ = ToolSlot::Transform;
}

void AppController::resetTransformTool() noexcept {
    transform_ = MeshTransform{};
    transformPreviewV_.resize(0, 3);
    transformPreviewBounds_ = BoundingBox{};
}

void AppController::previewTransform(const MeshTransform& transform) {
    transform_ = MeshTransform{finiteOrZero(transform.translation), finiteOrZero(transform.rotationDeg)};
    if (mesh_.empty() || transform_.isIdentity()) {
        transformPreviewV_.resize(0, 3);
        transformPreviewBounds_ = BoundingBox{};
        return;
    }
    // Only the vertices move, so the preview keeps just them (the faces are `mesh_.F`)
    Mesh moved;
    moved.V = mesh_.V;
    transformMesh(moved, transform_);
    transformPreviewBounds_ = moved.boundingBox();
    transformPreviewV_ = std::move(moved.V);
}

bool AppController::applyTransform() {
    const MeshTransform transform = transform_;
    if (!transformWholeMesh(transform, "Translate / Rotate")) {
        postStatusMessage("Enter a translation or a rotation first.");
        return false;
    }
    lastRepairStatusMessage_ = "Translated by " + vectorText(transform.translation) + " " +
                               std::string(unitSymbol(mesh_.fileUnit)) + ", then rotated by " +
                               vectorText(transform.rotationDeg) + " degrees about X, Y, Z.";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::transformPreviewShown() const noexcept {
    return activeTool_ == ToolSlot::Transform && repairPreviewVisible_ && transformPreviewV_.rows() > 0 &&
           transformPreviewV_.rows() == mesh_.numVertices() && transformPreviewShow_ != GlobalPreviewShow::Original;
}

} // namespace ogun
