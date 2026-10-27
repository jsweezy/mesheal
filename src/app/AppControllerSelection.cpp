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
    if (changed > 0) {
        onFaceSelectionChanged();
    }
    return changed;
}

void AppController::toggleFaceSelected(Eigen::Index face) {
    const Eigen::Index one[] = {face};
    selectFaces(one, !faceSelected(face));
}

void AppController::selectAllFaces() {
    // Only the shown faces, as in Blender (hidden shells stay out)
    faceSelected_ = shownFaceMask();
    if (faceSelected_.empty()) {
        faceSelected_.assign(static_cast<std::size_t>(mesh_.numFaces()), 1);
    }
    faceSelectedCount_ = static_cast<std::size_t>(std::count(faceSelected_.begin(), faceSelected_.end(), std::uint8_t{1}));
    onFaceSelectionChanged();
}

bool AppController::deleteSelectedFaces() {
    std::vector<std::uint8_t> drop = faceSelected_;
    const std::vector<std::uint8_t> shown = shownFaceMask();
    for (std::size_t f = 0; f < shown.size() && f < drop.size(); ++f) {
        drop[f] = static_cast<std::uint8_t>(drop[f] & shown[f]);
    }
    const auto count = static_cast<std::size_t>(std::count(drop.begin(), drop.end(), std::uint8_t{1}));
    if (count == 0) {
        postStatusMessage("Select the faces to delete first (Shift+click, B or C).");
        return false;
    }
    if (count == static_cast<std::size_t>(mesh_.numFaces())) {
        postStatusMessage("Every face is selected: deleting them all would leave no mesh.", true);
        return false;
    }
    std::vector<Eigen::Index> keep;
    keep.reserve(static_cast<std::size_t>(mesh_.numFaces()) - count);
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        if (static_cast<std::size_t>(f) >= drop.size() || drop[static_cast<std::size_t>(f)] == 0) {
            keep.push_back(f);
        }
    }
    const std::string label = "Delete " + std::to_string(count) + (count == 1 ? " Face" : " Faces");
    pushUndoEntry(label);
    finishMeshEdit(extractFaces(mesh_, keep)); // also clears the selection: face numbers change
    postStatusMessage(label + ".");
    return true;
}

bool AppController::clearFaceSelection() {
    const bool had = faceSelectedCount_ > 0;
    faceSelected_.clear();
    faceSelectedCount_ = 0;
    if (had) {
        onFaceSelectionChanged();
    }
    return had;
}

void AppController::onFaceSelectionChanged() {
    // A Decimate / Merge preview was computed on the old selection
    if (activeTool_ == ToolSlot::Decimate || activeTool_ == ToolSlot::MergeVertices) {
        discardGlobalCandidates();
    }
}

std::vector<std::uint8_t> AppController::shownFaceMask() const {
    if (shells_.visibleCount() == shells_.size()) {
        return {};
    }
    std::vector<std::uint8_t> shown(static_cast<std::size_t>(mesh_.numFaces()), 1);
    for (const ShellInfo& shell : shells_.shells()) {
        if (!shell.visible) {
            for (const Eigen::Index f : shell.faces) {
                shown[static_cast<std::size_t>(f)] = 0;
            }
        }
    }
    return shown;
}

const FaceVisibility& AppController::faceVisibility() const {
    std::vector<std::uint8_t> shown = shownFaceMask();
    if (!faceVisibility_.built() || faceVisibilityGeneration_ != meshGeneration_ || shown != faceVisibilityShown_) {
        faceVisibility_.build(mesh_, shown);
        faceVisibilityGeneration_ = meshGeneration_;
        faceVisibilityShown_ = std::move(shown);
    }
    return faceVisibility_;
}

std::size_t AppController::selectFacesInRectangle(const ScreenView& view, const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                                                  bool select) {
    const FaceVisibility* visibility = selectsThrough() ? nullptr : &faceVisibility();
    return selectFaces(facesInRectangle(mesh_, view, a, b, visibility, shownFaceMask()), select);
}

std::size_t AppController::selectFacesInStroke(const ScreenView& view, const Eigen::Vector2d& from, const Eigen::Vector2d& to,
                                               double radius, bool select) {
    const FaceVisibility* visibility = selectsThrough() ? nullptr : &faceVisibility();
    return selectFaces(facesInStroke(mesh_, view, from, to, radius, visibility, shownFaceMask()), select);
}

std::optional<Eigen::Index> AppController::faceAtScreenPoint(const ScreenView& view, const Eigen::Vector2d& point) const {
    return mesh_.empty() ? std::nullopt : faceVisibility().faceAt(mesh_, view, point);
}

std::vector<std::uint8_t> AppController::operatorScopeMask() const {
    const bool selectionScope = activeTool_ == ToolSlot::Decimate || activeTool_ == ToolSlot::MergeVertices;
    if (selectionScope && faceSelectedCount_ > 0) {
        // Faces in hidden shells are never edited, even when selected before the hide
        std::vector<std::uint8_t> mask = faceSelected_;
        const std::vector<std::uint8_t> shown = shownFaceMask();
        for (std::size_t f = 0; f < shown.size(); ++f) {
            mask[f] = static_cast<std::uint8_t>(mask[f] & shown[f]);
        }
        return mask;
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
        const std::size_t count = operatorScopeFaceCount();
        return "the " + std::to_string(count) + (count == 1 ? " selected face" : " selected faces");
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
