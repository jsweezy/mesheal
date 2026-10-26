// AppController: hole filling, per hole (Inspector) and for the selected open edges (Fix Selected).

#include "app/AppController.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

namespace ogun {

const std::vector<HoleLoop>& AppController::holeLoops() {
    if (!holesValid_) {
        holes_ = findHoleLoops(mesh_, holeSettings_.shape);
        holesValid_ = true;
    }
    return holes_;
}

std::size_t AppController::holeCountInScope() {
    const std::vector<HoleLoop>& holes = holeLoops();
    if (!shells_.scoped()) {
        return holes.size();
    }
    return static_cast<std::size_t>(std::count_if(holes.begin(), holes.end(), [&](const HoleLoop& hole) {
        return std::any_of(hole.faces.begin(), hole.faces.end(), [&](Eigen::Index f) { return shells_.inScope(shells_.shellOfFace(f)); });
    }));
}

const HoleLoop* AppController::focusedHole() const noexcept {
    return focusedHole_ && *focusedHole_ < holes_.size() ? &holes_[*focusedHole_] : nullptr;
}

std::span<const Eigen::Index> AppController::focusedHoleFaces() const noexcept {
    const HoleLoop* hole = focusedHole();
    return hole != nullptr ? std::span<const Eigen::Index>(hole->faces) : std::span<const Eigen::Index>{};
}

void AppController::resetHoleRepairState() noexcept {
    focusedHole_.reset();
    selectedHoleMethod_ = HoleFillMethod::None;
    holePreview_ = HoleFillPreview{};
}

void AppController::refreshHoleFocus(bool openEdgeFocus) {
    const CategoryResult* result = browser_.result();
    if (!openEdgeFocus || result == nullptr) {
        resetHoleRepairState();
        return;
    }
    const DefectElement& element = result->elements[browser_.focusedElement()];
    const std::vector<HoleLoop>& holes = holeLoops();
    const int index = findHoleOfEdge(holes, element.edge[0], element.edge[1]);
    if (index < 0) {
        resetHoleRepairState();
        return;
    }
    if (focusedHole_ == static_cast<std::size_t>(index)) {
        return; // same hole: keep the chosen method and its preview
    }
    focusedHole_ = static_cast<std::size_t>(index);
    selectedHoleMethod_ = recommendedHoleFill(holes[*focusedHole_].shape);
    refreshHolePreview();
}

void AppController::refreshHolePreview() {
    const HoleLoop* hole = focusedHole();
    if (hole == nullptr || selectedHoleMethod_ == HoleFillMethod::None) {
        holePreview_ = HoleFillPreview{};
        return;
    }
    holePreview_ = previewHoleFill(mesh_, *hole, selectedHoleMethod_, checkParams(), holeSettings_);
}

void AppController::setSelectedHoleMethod(HoleFillMethod method) {
    if (!focusedHole_) {
        return;
    }
    selectedHoleMethod_ = method;
    refreshHolePreview();
}

void AppController::setHoleFillSettings(const HoleFillSettings& settings) {
    holeSettings_ = settings;
    holeSettings_.patchRings = std::clamp(holeSettings_.patchRings, 1, settings::kMaxHolePatchRings);
    holeSettings_.maxHoleEdges = std::max(holeSettings_.maxHoleEdges, 0);
    holeSettings_.maxPatchFaces = std::max<std::size_t>(holeSettings_.maxPatchFaces, 1);
    holeSettings_.crackSnapFactor = std::max(holeSettings_.crackSnapFactor, 0.0);
    refreshHolePreview();
}

bool AppController::commitHoleFill() {
    const HoleLoop* focused = focusedHole();
    if (focused == nullptr || selectedHoleMethod_ == HoleFillMethod::None) {
        lastRepairStatusMessage_ = "Focus an open edge and pick a fill method first.";
        return false;
    }
    const HoleFillMethod method = selectedHoleMethod_;
    const std::size_t number = *focusedHole_ + 1;
    HoleFillResult result = fillHole(mesh_, *focused, method, checkParams(), holeSettings_);
    if (!result.success) {
        lastRepairStatusMessage_ = result.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    if (result.newCrossings > 0) {
        lastRepairStatusMessage_ = std::string(holeFillMethodLabel(method)) + " would cross " +
                                   std::to_string(result.newCrossings) + " faces. Try another method.";
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    pushUndoEntry(std::string("Hole Fill (") + holeFillMethodLabel(method) + ", hole " + std::to_string(number) + ")");
    finishMeshEdit(std::move(result.mesh));
    lastRepairStatusMessage_ = "Hole " + std::to_string(number) + ": " + result.statusMessage;
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixSelectedHoles() {
    const CategoryResult* result = browser_.result();
    if (result == nullptr || result->category != DefectCategory::OpenEdges || browser_.selectedCount() == 0) {
        lastRepairStatusMessage_ = "Select open edges first.";
        return false;
    }
    const std::vector<HoleLoop>& holes = holeLoops();
    std::unordered_set<int> chosen;
    for (const std::size_t element : browser_.selectedElements()) {
        const DefectElement& e = result->elements[element];
        if (const int h = findHoleOfEdge(holes, e.edge[0], e.edge[1]); h >= 0) {
            chosen.insert(h);
        }
    }
    std::vector<HoleLoop> selected;
    for (std::size_t h = 0; h < holes.size(); ++h) {
        if (chosen.contains(static_cast<int>(h))) {
            selected.push_back(holes[h]);
        }
    }
    HoleBatchResult batch = fillHoles(mesh_, selected, HoleFillMethod::None, holeSettings_);
    if (!batch.success) {
        lastRepairStatusMessage_ = batch.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    pushUndoEntry("Fix Selected Holes (" + std::to_string(selected.size()) + (selected.size() == 1 ? " hole)" : " holes)"));
    finishMeshEdit(std::move(batch.mesh));
    lastRepairStatusMessage_ = std::move(batch.statusMessage);
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

} // namespace ogun
