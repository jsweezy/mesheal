// AppController: the face selection (viewport Shift+click, box and circle) and the scope of the
// Decimate / Merge Vertices operators.

#include "app/AppController.hpp"

#include <algorithm>
#include <cstdio>

namespace ogun {

std::size_t AppController::selectFaces(std::span<const Eigen::Index> faces, bool select) {
    faceSelected_.resize(static_cast<std::size_t>(mesh_.numFaces()), 0);
    std::size_t changed = 0;
    for (const Eigen::Index f : faces) {
        if (f < 0 || f >= mesh_.numFaces()) {
            continue;
        }
        std::uint8_t& flag = faceSelected_[static_cast<std::size_t>(f)];
        if ((flag != 0) != select) {
            flag = select ? 1 : 0;
            ++changed;
        }
    }
    faceSelectedCount_ = select ? faceSelectedCount_ + changed : faceSelectedCount_ - changed;
    return changed;
}

void AppController::toggleFaceSelected(Eigen::Index face) {
    const Eigen::Index one[] = {face};
    selectFaces(one, !faceSelected(face));
}

void AppController::selectAllFaces() {
    faceSelected_.assign(static_cast<std::size_t>(mesh_.numFaces()), 1);
    faceSelectedCount_ = faceSelected_.size();
}

bool AppController::clearFaceSelection() {
    const bool had = faceSelectedCount_ > 0;
    faceSelected_.clear();
    faceSelectedCount_ = 0;
    return had;
}

std::size_t AppController::selectFacesInRectangle(const ScreenView& view, const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                                                  bool select) {
    const FaceVisibility* visibility = nullptr;
    if (!selectThrough_) {
        if (!faceVisibility_.built() || faceVisibilityGeneration_ != meshGeneration_) {
            faceVisibility_.build(mesh_);
            faceVisibilityGeneration_ = meshGeneration_;
        }
        visibility = &faceVisibility_;
    }
    return selectFaces(facesInRectangle(mesh_, view, a, b, visibility), select);
}

std::size_t AppController::selectFacesInCircle(const ScreenView& view, const Eigen::Vector2d& center, double radius, bool select) {
    const FaceVisibility* visibility = nullptr;
    if (!selectThrough_) {
        if (!faceVisibility_.built() || faceVisibilityGeneration_ != meshGeneration_) {
            faceVisibility_.build(mesh_);
            faceVisibilityGeneration_ = meshGeneration_;
        }
        visibility = &faceVisibility_;
    }
    return selectFaces(facesInCircle(mesh_, view, center, radius, visibility), select);
}

std::vector<std::uint8_t> AppController::operatorScopeMask() const {
    const bool selectionScope = activeTool_ == ToolSlot::Decimate || activeTool_ == ToolSlot::MergeVertices;
    if (selectionScope && faceSelectedCount_ > 0) {
        return faceSelected_;
    }
    return repairScopeMask();
}

std::size_t AppController::operatorScopeFaceCount() const {
    const std::vector<std::uint8_t> mask = operatorScopeMask();
    return mask.empty() ? static_cast<std::size_t>(mesh_.numFaces())
                        : static_cast<std::size_t>(std::count(mask.begin(), mask.end(), std::uint8_t{1}));
}

std::string AppController::operatorScopeText() const {
    if (faceSelectedCount_ > 0) {
        return "the " + std::to_string(faceSelectedCount_) + " selected faces";
    }
    return shells_.scoped() ? "the shells in scope" : "the whole mesh";
}

double AppController::meanEdgeLength() const {
    if (mesh_.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            sum += (mesh_.V.row(mesh_.F(f, c)) - mesh_.V.row(mesh_.F(f, (c + 1) % 3))).norm();
        }
    }
    return sum / (3.0 * static_cast<double>(mesh_.numFaces()));
}

} // namespace ogun
