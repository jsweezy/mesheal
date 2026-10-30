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

void AppController::refreshGlobalParams() {
    globalParams_.checkParams = checkParams();
    globalParams_.islandSettings = islandSettings_;
    globalParams_.holeSettings = holeSettings_;
}

void AppController::openRepairAll(ToolSlot slot, GlobalRepairMethod method) {
    clearSelection();
    activeTool_ = slot;
    globalMethod_ = method;
    refreshGlobalParams();
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
    globalParams_.alphaWrapOffsetRatio = std::clamp(globalParams_.alphaWrapOffsetRatio, settings::kMinAlphaWrapOffsetRatio,
                                                    settings::kMaxAlphaWrapOffsetRatio);
    globalParams_.decimateRatio = std::clamp(globalParams_.decimateRatio, settings::kMinDecimateRatio, settings::kMaxDecimateRatio);
    globalParams_.mergeDistance =
        std::clamp(globalParams_.mergeDistance, settings::kMinMergeDistance, settings::kMaxMergeDistance);
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

void AppController::queueGlobalRepairs(std::vector<GlobalRepairMethod> methods, bool fixAllOnePass) {
    if (mesh_.empty() || methods.empty()) {
        return;
    }
    refreshGlobalParams();
    RepairRequest request;
    request.meshGeneration = meshGeneration_;
    request.mesh = std::make_shared<const Mesh>(mesh_);
    request.methods = std::move(methods);
    request.params = globalParams_;
    const bool fixAll = std::find(request.methods.begin(), request.methods.end(), GlobalRepairMethod::FixAll) != request.methods.end();
    if (activeTool_ == ToolSlot::FixAll) {
        request.faceInScope = fixAllScopeMask(request.fixAllScopeKind);
        request.fixAllHooks = fixAllHooks_;
        if (fixAllOnePass) {
            request.params.fixAll.passes = 1;
        }
    } else {
        request.faceInScope = operatorScopeMask();
    }
    if (backgroundChecks_) {
        repairRunner_.start(std::move(request));
        repairJobActive_ = true;
        repairJobCancelled_ = false;
        repairJobJustStored_.clear();
        fixAllRebuildJob_ = false;
        fixAllJobRunning_ = fixAll;
        fixAllCancelling_ = false;
        postStatusMessage(fixAll ? "Fix All: checking the mesh..." : "Computing Repair All preview...");
        return;
    }
    for (const GlobalRepairMethod method : request.methods) {
        const FixAllRunInput input{request.fixAllScopeKind, nullptr, request.fixAllHooks.get()};
        storeCandidate(runGlobalRepair(mesh_, request.faceInScope, method, request.params, {}, nullptr, &input));
    }
}

void AppController::cancelGlobalRepair() {
    if (fixAllJobRunning_) {
        cancelFixAll();
        return;
    }
    if (fixAllRebuildJob_) {
        cancelFixAllRebuild();
        return;
    }
    if (!repairJobActive_ || repairJobCancelled_) {
        return;
    }
    stopRepairJobForCancel();
    postStatusMessage("Preview cancelled: nothing was changed.");
}

void AppController::stopRepairJobForCancel() {
    // Does not wait: an alpha wrap stops at its next step, a union finishes first. `requestStop` also drops
    // a result that is finished but not collected yet, and the poll drops anything the job hands over later.
    repairRunner_.requestStop();
    repairJobCancelled_ = true;
    // Collected by this frame's poll, before the click was handled: the user never saw it
    for (const auto& [rebuildRow, method] : repairJobJustStored_) {
        if (rebuildRow) {
            std::erase_if(fixAllRebuilds_, [&](const RepairCandidate& c) { return c.method == method; });
            if (shownRebuild_ == method) {
                shownRebuild_.reset();
            }
        } else {
            std::erase_if(globalCandidates_, [&](const RepairCandidate& c) { return c.method == method; });
            if (shownCandidate_ == method) {
                shownCandidate_.reset();
            }
        }
    }
    repairJobJustStored_.clear();
    if (!shownCandidate_ && !globalCandidates_.empty()) {
        shownCandidate_ = globalCandidates_.front().method;
    }
}

bool AppController::candidateIsCurrent(const RepairCandidate& candidate) const noexcept {
    if (candidate.method == GlobalRepairMethod::FixAll && candidate.fixAll != nullptr && candidate.fixAll->onePassRequested) {
        GlobalRepairParams params = candidate.params;
        params.fixAll.passes = globalParams_.fixAll.passes;
        return params == globalParams_;
    }
    return candidate.params == globalParams_;
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
    // Read before collecting: a job that has ended has handed over everything it computed
    const bool ended = !repairRunner_.busy();
    std::vector<FinishedCandidate> finished = repairRunner_.takeFinished();
    repairJobJustStored_.clear();
    bool changed = false;
    for (FinishedCandidate& f : finished) {
        if (f.meshGeneration != meshGeneration_ || !repairAllOpen() || repairJobCancelled_) {
            continue;
        }
        const GlobalRepairMethod method = f.candidate.method;
        if (f.rebuildRow) {
            storeFixAllRebuild(std::move(f.candidate));
        } else {
            storeCandidate(std::move(f.candidate));
        }
        repairJobJustStored_.emplace_back(f.rebuildRow, method);
        changed = true;
    }
    if (ended && repairJobActive_) {
        repairJobActive_ = false;
        repairJobCancelled_ = false;
        changed = true;
    }
    return changed;
}

void AppController::discardGlobalCandidates() {
    if (fixAllJobRunning_ || fixAllRebuildJob_ || repairRunner_.stopping()) {
        // A Fix All run (or a rebuild row's merge) may be inside a Manifold union: do not wait for it,
        // just drop what it finds
        repairRunner_.requestStop();
        fixAllJobRunning_ = false;
        fixAllCancelling_ = false;
    } else {
        repairRunner_.cancel();
    }
    repairJobActive_ = false;
    repairJobCancelled_ = false;
    repairJobJustStored_.clear();
    fixAllRebuildJob_ = false;
    fixAllPreviewQueued_.reset();
    (void)repairRunner_.takeFinished();
    globalCandidates_.clear();
    shownCandidate_.reset();
    fixAllRebuilds_.clear();
    fixAllRebuildShell_.reset();
    shownRebuild_.reset();
}

const RepairCandidate* AppController::shownGlobalCandidate() const noexcept {
    if (shownRebuild_) {
        const auto it = std::find_if(fixAllRebuilds_.begin(), fixAllRebuilds_.end(),
                                     [&](const RepairCandidate& c) { return c.method == *shownRebuild_; });
        if (it != fixAllRebuilds_.end()) {
            return &*it;
        }
    }
    if (!shownCandidate_) {
        return nullptr;
    }
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [&](const RepairCandidate& c) { return c.method == *shownCandidate_; });
    return it != globalCandidates_.end() ? &*it : nullptr;
}

void AppController::showGlobalCandidate(GlobalRepairMethod method) {
    shownRebuild_.reset();
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
    if (candidate->fixAll != nullptr && candidate->fixAll->rebuild) {
        return applyFixAllRebuild(candidate->method);
    }
    if (candidate->method == GlobalRepairMethod::FixAll) {
        return applyFixAll();
    }
    const std::string label = globalRepairMethodLabel(candidate->method);
    Mesh repaired = candidate->mesh;
    pushUndoEntry(activeTool_ == ToolSlot::AlphaWrap       ? std::string("Alpha Wrap")
                  : activeTool_ == ToolSlot::Decimate      ? std::string("Decimate")
                  : activeTool_ == ToolSlot::MergeVertices ? std::string("Merge Vertices")
                  : activeTool_ == ToolSlot::FixAll        ? "Rebuild (" + label + ")"
                  : activeTool_ == ToolSlot::RepairAllHoles ? "Repair All Holes (" + label + ")"
                                                            : "Repair All Intersections (" + label + ")");
    finishMeshEdit(std::move(repaired));
    recheckCategory(DefectCategory::Intersections);
    lastRepairStatusMessage_ = "Applied " + label + ".";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

} // namespace ogun
