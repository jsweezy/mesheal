// AppController: intersecting-face repair, per island (Inspector) and for the whole mesh (Repair All),
// and the Repair All operator it shares with holes.

#include "app/AppController.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace ogun {

// -----------------------------------------------------------------------------
// Islands
// -----------------------------------------------------------------------------

std::vector<int> AppController::faceShellIds() const {
    std::vector<int> faceShell(static_cast<std::size_t>(mesh_.numFaces()));
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        faceShell[static_cast<std::size_t>(f)] = shells_.shellOfFace(f);
    }
    return faceShell;
}

std::vector<std::uint8_t> AppController::repairScopeMask() const {
    if (!shells_.scoped()) {
        return {};
    }
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh_.numFaces()));
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        mask[static_cast<std::size_t>(f)] = shells_.inScope(shells_.shellOfFace(f)) ? 1 : 0;
    }
    return mask;
}

const std::vector<IntersectionIsland>& AppController::intersectionIslands() {
    if (!islandsValid_) {
        islands_.clear();
        if (const CategoryResult* result = currentResult(DefectCategory::Intersections); result != nullptr) {
            islands_ = clusterIntersectionIslands(mesh_, *result, faceShellIds(), repairScopeMask());
        }
        islandsValid_ = true;
    }
    return islands_;
}

std::size_t AppController::intersectionIslandCount() {
    return intersectionIslands().size();
}

const IntersectionIsland* AppController::focusedIsland() const noexcept {
    return focusedIsland_ && *focusedIsland_ < islands_.size() ? &islands_[*focusedIsland_] : nullptr;
}

void AppController::resetIslandRepairState() noexcept {
    focusedIsland_.reset();
    focusedIslandAnalysis_ = IslandAnalysis{};
    selectedIslandMethod_ = IntersectionRepairMethod::None;
    islandPreview_ = IslandRepairPreview{};
}

void AppController::refreshIslandFocus(Eigen::Index face) {
    if (face < 0) {
        resetIslandRepairState();
        return;
    }
    const bool recomputed = !islandsValid_;
    const std::vector<IntersectionIsland>& islands = intersectionIslands();
    const auto it = std::find_if(islands.begin(), islands.end(), [face](const IntersectionIsland& island) {
        return std::binary_search(island.faces.begin(), island.faces.end(), face);
    });
    if (it == islands.end()) {
        resetIslandRepairState(); // crosses other shells only: Merge Shells / Repair All unions those
        return;
    }
    const auto index = static_cast<std::size_t>(it - islands.begin());
    if (!recomputed && focusedIsland_ == index) {
        return; // same island: keep the chosen method and its preview
    }
    focusedIsland_ = index;
    focusedIslandAnalysis_ = analyzeIsland(mesh_, *it, islandSettings_);
    selectedIslandMethod_ = focusedIslandAnalysis_.recommended;
    refreshIslandPreview();
}

void AppController::refreshIslandPreview() {
    const IntersectionIsland* island = focusedIsland();
    if (island == nullptr || selectedIslandMethod_ == IntersectionRepairMethod::None) {
        islandPreview_ = IslandRepairPreview{};
        return;
    }
    islandPreview_ = previewIslandRepair(mesh_, *island, selectedIslandMethod_, checkParams(), islandSettings_, faceShellIds());
}

void AppController::setSelectedIslandMethod(IntersectionRepairMethod method) {
    if (!focusedIsland_) {
        return;
    }
    selectedIslandMethod_ = method;
    refreshIslandPreview();
}

void AppController::setIslandRepairSettings(const IntersectionRepairSettings& settings) {
    islandSettings_ = settings;
    islandSettings_.dilationRings = std::clamp(islandSettings_.dilationRings, 0, settings::kMaxIntersectionDilationRings);
    islandSettings_.cgalMaxSteps = std::max(islandSettings_.cgalMaxSteps, 1);
    if (focusedIsland_) {
        focusedIslandAnalysis_ = analyzeIsland(mesh_, islands_[*focusedIsland_], islandSettings_);
        refreshIslandPreview();
    }
}

bool AppController::commitIslandRepair() {
    const IntersectionIsland* focused = focusedIsland();
    if (focused == nullptr || selectedIslandMethod_ == IntersectionRepairMethod::None) {
        lastRepairStatusMessage_ = "Focus an intersecting face and pick a repair method first.";
        return false;
    }
    const IntersectionIsland island = *focused;
    const IntersectionRepairMethod method = selectedIslandMethod_;
    IslandRepairResult result = repairIsland(mesh_, island, method, checkParams(), islandSettings_, faceShellIds());
    if (!result.success) {
        lastRepairStatusMessage_ = result.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    if (result.newCrossings > 0) {
        lastRepairStatusMessage_ = std::string(intersectionRepairMethodLabel(method)) + " would create " +
                                   std::to_string(result.newCrossings) + " new intersecting faces. Try another method.";
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }

    pushUndoEntry(std::string("Island Repair (") + intersectionRepairMethodLabel(method) + ", " +
                  std::to_string(island.faces.size()) + " faces)");
    finishMeshEdit(std::move(result.mesh));
    lastRepairStatusMessage_ = std::string(intersectionRepairMethodLabel(method)) + ": intersecting faces in the shell " +
                               std::to_string(result.crossingsBefore) + " -> " + std::to_string(result.crossingsAfter) + ".";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixSelectedIntersections() {
    const CategoryResult* result = browser_.result();
    if (result == nullptr || result->category != DefectCategory::Intersections || browser_.selectedCount() == 0) {
        lastRepairStatusMessage_ = "Select intersecting faces first.";
        return false;
    }
    std::vector<Eigen::Index> seeds;
    seeds.reserve(browser_.selectedCount());
    for (const std::size_t element : browser_.selectedElements()) {
        const auto faces = result->facesOf(element);
        seeds.insert(seeds.end(), faces.begin(), faces.end());
    }
    IslandBatchResult batch = healIntersectionIslands(mesh_, checkParams(), repairScopeMask(), seeds, islandSettings_);
    if (!batch.success) {
        lastRepairStatusMessage_ = batch.statusMessage;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    pushUndoEntry("Fix Selected Intersections (" + std::to_string(batch.islandsBefore) + " islands)");
    finishMeshEdit(std::move(batch.mesh));
    recheckCategory(DefectCategory::Intersections);
    lastRepairStatusMessage_ = std::move(batch.statusMessage);
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

// -----------------------------------------------------------------------------
// Repair All operator
// -----------------------------------------------------------------------------

void AppController::openRepairAll(ToolSlot slot, GlobalRepairMethod method) {
    clearSelection();
    activeTool_ = slot;
    globalMethod_ = method;
    globalParams_.checkParams = checkParams();
    globalParams_.islandSettings = islandSettings_;
    globalParams_.holeSettings = holeSettings_;
}

void AppController::openRepairAllIntersections() {
    openRepairAll(ToolSlot::RepairAllIntersections, GlobalRepairMethod::PatchIslands);
}

void AppController::openRepairAllHoles() {
    openRepairAll(ToolSlot::RepairAllHoles, GlobalRepairMethod::FillHoles);
}

void AppController::openAlphaWrap() {
    openRepairAll(ToolSlot::AlphaWrap, GlobalRepairMethod::AlphaWrap);
}

void AppController::openDecimate() {
    openRepairAll(ToolSlot::Decimate, GlobalRepairMethod::Decimate);
}

void AppController::openMergeVertices() {
    openRepairAll(ToolSlot::MergeVertices, GlobalRepairMethod::MergeVertices);
}

void AppController::setGlobalRepairMethod(GlobalRepairMethod method) {
    globalMethod_ = method;
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [&](const RepairCandidate& c) { return c.method == method; });
    if (it != globalCandidates_.end()) {
        shownCandidate_ = method;
    }
}

void AppController::setGlobalRepairParams(const GlobalRepairParams& params) {
    globalParams_ = params;
    globalParams_.alphaWrapDetail =
        std::clamp(globalParams_.alphaWrapDetail, settings::kMinAlphaWrapDetail, settings::kMaxAlphaWrapDetail);
    globalParams_.alphaWrapOffset = std::max(globalParams_.alphaWrapOffset, 0.0);
    globalParams_.decimateRatio = std::clamp(globalParams_.decimateRatio, settings::kMinDecimateRatio, settings::kMaxDecimateRatio);
    globalParams_.mergeDistanceMm = std::max(globalParams_.mergeDistanceMm, 0.0);
    globalParams_.windingResolution = std::clamp(globalParams_.windingResolution, settings::kMinWindingGridResolution,
                                                 settings::kMaxWindingGridResolution);
}

void AppController::previewGlobalRepair() {
    queueGlobalRepairs({globalMethod_});
}

void AppController::compareAllGlobalRepairs() {
    const std::span<const GlobalRepairMethod> methods = globalRepairMethods();
    queueGlobalRepairs(std::vector<GlobalRepairMethod>(methods.begin(), methods.end()));
}

void AppController::queueGlobalRepairs(std::vector<GlobalRepairMethod> methods) {
    if (mesh_.empty() || methods.empty()) {
        return;
    }
    globalParams_.checkParams = checkParams();
    globalParams_.islandSettings = islandSettings_;
    globalParams_.holeSettings = holeSettings_;
    RepairRequest request{meshGeneration_, std::make_shared<const Mesh>(mesh_), operatorScopeMask(), std::move(methods), globalParams_};
    if (backgroundChecks_) {
        repairRunner_.start(std::move(request));
        postStatusMessage("Computing Repair All preview...");
        return;
    }
    for (const GlobalRepairMethod method : request.methods) {
        storeCandidate(runGlobalRepair(mesh_, request.faceInScope, method, request.params));
    }
}

void AppController::cancelGlobalRepair() {
    repairRunner_.cancel();
}

void AppController::waitForGlobalRepair() {
    repairRunner_.waitIdle();
    pollRepairJobs();
}

void AppController::storeCandidate(RepairCandidate candidate) {
    const GlobalRepairMethod method = candidate.method;
    if (candidate.success) {
        postStatusMessage(std::string(globalRepairMethodLabel(method)) + " preview ready.");
    } else {
        postStatusMessage(std::string(globalRepairMethodLabel(method)) + ": " + candidate.statusMessage, true);
    }
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [&](const RepairCandidate& c) { return c.method == method; });
    if (it != globalCandidates_.end()) {
        *it = std::move(candidate);
    } else {
        globalCandidates_.push_back(std::move(candidate));
        std::sort(globalCandidates_.begin(), globalCandidates_.end(),
                  [](const RepairCandidate& a, const RepairCandidate& b) { return a.method < b.method; });
    }
    if (method == globalMethod_ || !shownCandidate_) {
        shownCandidate_ = method;
    }
}

bool AppController::pollRepairJobs() {
    std::vector<FinishedCandidate> finished = repairRunner_.takeFinished();
    bool changed = false;
    for (FinishedCandidate& f : finished) {
        if (f.meshGeneration == meshGeneration_ && repairAllOpen()) {
            storeCandidate(std::move(f.candidate));
            changed = true;
        }
    }
    return changed;
}

void AppController::discardGlobalCandidates() {
    repairRunner_.cancel();
    (void)repairRunner_.takeFinished();
    globalCandidates_.clear();
    shownCandidate_.reset();
}

const RepairCandidate* AppController::shownGlobalCandidate() const noexcept {
    if (!shownCandidate_) {
        return nullptr;
    }
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [&](const RepairCandidate& c) { return c.method == *shownCandidate_; });
    return it != globalCandidates_.end() ? &*it : nullptr;
}

void AppController::showGlobalCandidate(GlobalRepairMethod method) {
    globalMethod_ = method;
    setGlobalRepairMethod(method);
}

bool AppController::globalPreviewShown() const noexcept {
    const RepairCandidate* candidate = shownGlobalCandidate();
    return repairAllOpen() && repairPreviewVisible_ && candidate != nullptr &&
           candidate->success && globalPreviewShow_ != GlobalPreviewShow::Original;
}

bool AppController::applyGlobalRepair() {
    const RepairCandidate* candidate = shownGlobalCandidate();
    if (candidate == nullptr || !candidate->success) {
        lastRepairStatusMessage_ = "Preview a Repair All method first.";
        return false;
    }
    const std::string label = globalRepairMethodLabel(candidate->method);
    Mesh repaired = candidate->mesh;
    pushUndoEntry(activeTool_ == ToolSlot::AlphaWrap       ? std::string("Alpha Wrap")
                  : activeTool_ == ToolSlot::Decimate      ? std::string("Decimate")
                  : activeTool_ == ToolSlot::MergeVertices ? std::string("Merge Vertices")
                  : activeTool_ == ToolSlot::RepairAllHoles ? "Repair All Holes (" + label + ")"
                                                            : "Repair All Intersections (" + label + ")");
    finishMeshEdit(std::move(repaired));
    recheckCategory(DefectCategory::Intersections);
    lastRepairStatusMessage_ = "Applied " + label + ".";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

} // namespace ogun
