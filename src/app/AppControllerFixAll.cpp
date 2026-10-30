// AppController: Fix All in the app. The panel's plan-time check (which shells overlap), the run as a
// background Repair All job with progress and Cancel, Apply as one undo step with the run's final check
// installed as the Mesh Health report, the selection after Apply, and the report's shell numbers.

#include "app/AppController.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace ogun {

namespace {

[[nodiscard]] std::string withThousands(std::size_t value) {
    std::string digits = std::to_string(value);
    for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(digits.size()) - 3; i > 0; i -= 3) {
        digits.insert(static_cast<std::size_t>(i), ",");
    }
    return digits;
}

[[nodiscard]] std::string counted(std::size_t n, const char* one, const char* many) {
    return withThousands(n) + " " + (n == 1 ? one : many);
}

[[nodiscard]] bool sameThresholds(const CheckParams& a, const CheckParams& b) noexcept {
    return a.thinThresholds.minAltitude == b.thinThresholds.minAltitude && a.thinThresholds.minArea == b.thinThresholds.minArea &&
           a.thinThresholds.maxAspectRatio == b.thinThresholds.maxAspectRatio && a.needleEdgeRatio == b.needleEdgeRatio &&
           a.capAngleDeg == b.capAngleDeg && a.intersectionTolerance == b.intersectionTolerance;
}

} // namespace

std::string renumberShellText(std::string_view text, std::span<const int> numbers) {
    struct Token {
        std::size_t pos = 0;
        std::size_t length = 0;
    };
    std::vector<Token> tokens;
    for (std::size_t i = 0; i + 1 < text.size(); ++i) {
        if (text[i] != '#' || std::isdigit(static_cast<unsigned char>(text[i + 1])) == 0) {
            continue;
        }
        std::size_t end = i + 1;
        while (end < text.size() && std::isdigit(static_cast<unsigned char>(text[end])) != 0) {
            ++end;
        }
        tokens.push_back({i, end - i});
        i = end - 1;
    }
    if (tokens.size() != numbers.size()) {
        return std::string(text);
    }
    std::string out;
    out.reserve(text.size());
    std::size_t at = 0;
    for (std::size_t t = 0; t < tokens.size(); ++t) {
        out.append(text.substr(at, tokens[t].pos - at));
        if (numbers[t] > 0) {
            out += "#" + std::to_string(numbers[t]);
        } else {
            out.append(text.substr(tokens[t].pos, tokens[t].length));
        }
        at = tokens[t].pos + tokens[t].length;
    }
    out.append(text.substr(at));
    return out;
}

// -----------------------------------------------------------------------------
// Scope
// -----------------------------------------------------------------------------

std::vector<std::uint8_t> AppController::fixAllScopeMask(FixAllScopeKind& kind) const {
    // After a whole-mesh Apply the merged shells are selected; the next pass still covers the whole mesh
    if (fixAllApplied_ && fixAllApplied_->scopeKind == FixAllScopeKind::Whole && fixAllApplied_->meshGeneration == meshGeneration_ &&
        fixAllApplied_->shellsRevision == shells_.revision() &&
        fixAllApplied_->faceSelectionGeneration == faceSelectionGeneration_) {
        kind = FixAllScopeKind::Whole;
        return {};
    }
    const auto faces = static_cast<std::size_t>(mesh_.numFaces());
    const std::vector<std::uint8_t> shown = shownFaceMask();
    const auto visible = [&](std::size_t f) { return shown.empty() || shown[f] != 0; };
    if (faceSelectedCount_ > 0) {
        kind = FixAllScopeKind::Faces;
        std::vector<std::uint8_t> mask = faceSelected_;
        mask.resize(faces, 0);
        for (std::size_t f = 0; f < faces; ++f) {
            mask[f] = static_cast<std::uint8_t>(mask[f] != 0 && visible(f));
        }
        return mask;
    }
    if (shells_.scoped()) {
        kind = FixAllScopeKind::Shells;
        std::vector<std::uint8_t> mask(faces, 0);
        for (std::size_t f = 0; f < faces; ++f) {
            mask[f] = static_cast<std::uint8_t>(shells_.inScope(shells_.shellOfFace(static_cast<Eigen::Index>(f))) && visible(f));
        }
        return mask;
    }
    kind = FixAllScopeKind::Whole;
    return {};
}

FixAllScopeInfo AppController::fixAllScope() const {
    FixAllScopeInfo info;
    const std::vector<std::uint8_t> mask = fixAllScopeMask(info.kind);
    info.totalShells = shells_.size();
    info.hiddenShells = shells_.size() - shells_.visibleCount();
    std::vector<std::uint8_t> touched(shells_.size(), 0);
    for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
        if (!mask.empty() && mask[static_cast<std::size_t>(f)] == 0) {
            continue;
        }
        ++info.faces;
        const int s = shells_.shellOfFace(f);
        if (s >= 0) {
            touched[static_cast<std::size_t>(s)] = 1;
        }
    }
    info.shells = static_cast<std::size_t>(std::count(touched.begin(), touched.end(), std::uint8_t{1}));
    switch (info.kind) {
    case FixAllScopeKind::Whole:
        info.text = "whole mesh, " + counted(info.totalShells, "shell", "shells") +
                    (fixAllApplied_ && shells_.selectedCount() > 0 ? " (as in the last run)" : " (nothing selected)");
        break;
    case FixAllScopeKind::Shells:
        info.text = withThousands(info.shells) + " of " + counted(info.totalShells, "shell", "shells") +
                    (shells_.selectedCount() > 0 ? " (selected)" : " (the visible ones)");
        break;
    case FixAllScopeKind::Faces:
        info.text = counted(info.faces, "selected face", "selected faces") + " (in " + counted(info.shells, "shell", "shells") + ")";
        break;
    }
    if (info.hiddenShells > 0) {
        info.text += info.hiddenShells == 1 ? "; 1 hidden shell is left as it is"
                                            : "; " + withThousands(info.hiddenShells) + " hidden shells are left as they are";
    }
    return info;
}

void AppController::onFixAllScopeChanged() {
    ++fixAllScopeSerial_;
    fixAllPlan_.reset();
    fixAllPlanDue_ = true;
    // A candidate (or a run) for the old scope or mesh is dropped
    if (fixAllJobRunning_ || fixAllPreviewQueued_ || !globalCandidates_.empty()) {
        discardGlobalCandidates();
    }
}

// -----------------------------------------------------------------------------
// Opening, settings and the plan-time check
// -----------------------------------------------------------------------------

void AppController::resetFixAllState() {
    fixAllPlanRunner_.cancel();
    fixAllPlan_.reset();
    fixAllPlanDue_ = false;
    fixAllMergeUserSet_ = false;
    fixAllPreviewQueued_.reset();
    fixAllApplied_.reset();
    fixAllCancelling_ = false;
}

void AppController::applyFixAllDefaults(FixAllParams& params, const FixAllPreferences& d) {
    params.passes = d.passes;
    params.timeBudgetSeconds = d.timeBudgetSeconds;
    params.weld = d.weld;
    params.weldTolerance = d.weldTolerance;
    params.removeTinyShells = d.removeTinyShells;
    params.tinyShellFraction = d.tinyShellFraction;
    params.removeFins = d.removeFins;
    params.closeCracks = d.closeCracks;
    params.healThinFaces = d.healThinFaces;
}

void AppController::openFixAll(int passes) {
    openRepairAll(ToolSlot::FixAll, GlobalRepairMethod::FixAll);
    resetFixAllState();
    applyFixAllDefaults(globalParams_.fixAll, preferenceDefaults_.fixAll);
    globalParams_.fixAll.passes = std::clamp(passes, settings::kFixAllMinPasses, settings::kFixAllMaxPasses);
    globalParams_.fixAll.mergeShells = settings::kDefaultFixAllMergeShells;
    onFixAllScopeChanged();
    ensureFixAllPlan();
}

void AppController::setFixAllParams(const FixAllParams& params) {
    const bool mergeClicked = params.mergeShells != globalParams_.fixAll.mergeShells;
    FixAllParams& p = globalParams_.fixAll;
    p = params;
    p.passes = std::clamp(p.passes, settings::kFixAllMinPasses, settings::kFixAllMaxPasses);
    p.timeBudgetSeconds = std::max(p.timeBudgetSeconds, 0.0);
    p.tinyShellFraction = std::max(p.tinyShellFraction, 0.0);
    p.weldTolerance.absFloor =
        std::clamp(p.weldTolerance.absFloor, settings::kFixAllWeldToleranceMin, settings::kFixAllWeldToleranceMax);
    p.weldTolerance.relTol = std::clamp(p.weldTolerance.relTol, settings::kFixAllWeldToleranceMin, settings::kFixAllWeldToleranceMax);
    if (mergeClicked) {
        fixAllMergeUserSet_ = true;
    }
}

void AppController::setFixAllPasses(int passes) {
    globalParams_.fixAll.passes = std::clamp(passes, settings::kFixAllMinPasses, settings::kFixAllMaxPasses);
}

void AppController::setFixAllMergeShells(bool ticked) {
    globalParams_.fixAll.mergeShells = ticked;
    fixAllMergeUserSet_ = true;
}

bool AppController::fixAllPlanChecking() const {
    return fixAllOpen() && (fixAllPlanDue_ || fixAllPlanRunner_.pending());
}

const MergePlan* AppController::fixAllMergePlan() const noexcept {
    return fixAllPlan_ ? &fixAllPlan_->merge : nullptr;
}

const FixAllWeldPlan* AppController::fixAllWeldPlan() const noexcept {
    return fixAllPlan_ ? &fixAllPlan_->weld : nullptr;
}

void AppController::ensureFixAllPlan() {
    if (!fixAllOpen() || !fixAllPlanDue_ || mesh_.empty()) {
        return;
    }
    fixAllPlanDue_ = false;
    FixAllPlanRequest request;
    request.meshGeneration = meshGeneration_;
    request.scopeSerial = fixAllScopeSerial_;
    FixAllScopeKind kind = FixAllScopeKind::Whole;
    const std::vector<std::uint8_t> mask = fixAllScopeMask(kind);
    request.scope = FixAllScope::fromFaceMask(mesh_, kind, mask);
    request.mergeParams.intersectionTolerance = intersectionTolerance_;
    request.weldTolerance = globalParams_.fixAll.weldTolerance;
    if (backgroundChecks_) {
        request.mesh = std::make_shared<const Mesh>(mesh_);
        fixAllPlanRunner_.start(std::move(request));
        return;
    }
    request.mesh = std::shared_ptr<const Mesh>(&mesh_, [](const Mesh*) {}); // inline: no copy
    acceptFixAllPlan(computeFixAllPlan(request));
}

void AppController::acceptFixAllPlan(FinishedFixAllPlan finished) {
    if (!fixAllOpen() || finished.meshGeneration != meshGeneration_ || finished.scopeSerial != fixAllScopeSerial_ ||
        !finished.merge.complete) {
        return;
    }
    if (!fixAllMergeUserSet_) {
        globalParams_.fixAll.mergeShells = finished.merge.tickedByDefault;
    }
    fixAllPlan_ = std::move(finished);
}

void AppController::waitForFixAllPlan() {
    if (!fixAllOpen()) {
        return;
    }
    ensureFixAllPlan();
    fixAllPlanRunner_.waitIdle();
    if (std::optional<FinishedFixAllPlan> finished = fixAllPlanRunner_.takeFinished()) {
        acceptFixAllPlan(std::move(*finished));
    }
    if (fixAllPreviewQueued_ && !fixAllPlanChecking()) {
        const bool onePass = *fixAllPreviewQueued_;
        fixAllPreviewQueued_.reset();
        startFixAllRun(onePass);
    }
}

// -----------------------------------------------------------------------------
// Running, cancel and progress
// -----------------------------------------------------------------------------

void AppController::previewFixAll(bool onePass) {
    if (!fixAllOpen() || mesh_.empty()) {
        return;
    }
    ensureFixAllPlan();
    if (!fixAllMergeUserSet_ && fixAllPlanChecking()) {
        // The merge box's default is not known yet: run once the check is done
        fixAllPreviewQueued_ = onePass;
        postStatusMessage("Fix All: checking which shells overlap...");
        return;
    }
    startFixAllRun(onePass);
}

void AppController::runFixAllOneMorePass() {
    previewFixAll(true);
}

void AppController::startFixAllRun(bool onePass) {
    // The new run replaces the shown Fix All candidate
    globalCandidates_.erase(std::remove_if(globalCandidates_.begin(), globalCandidates_.end(),
                                           [](const RepairCandidate& c) { return c.method == GlobalRepairMethod::FixAll; }),
                            globalCandidates_.end());
    if (shownCandidate_ == GlobalRepairMethod::FixAll) {
        shownCandidate_.reset();
    }
    globalMethod_ = GlobalRepairMethod::FixAll;
    queueGlobalRepairs({GlobalRepairMethod::FixAll}, onePass);
}

void AppController::cancelFixAll() {
    if (fixAllPreviewQueued_) {
        fixAllPreviewQueued_.reset();
        postStatusMessage("Fix All cancelled: nothing was changed.");
        return;
    }
    if (!fixAllJobRunning_ || !repairRunner_.busy() || fixAllCancelling_) {
        return;
    }
    const std::string text = repairRunner_.progressText();
    repairRunner_.requestStop();
    fixAllCancelling_ = true;
    fixAllCancelText_ = text.find("merging") != std::string::npos ? "Cancelling after the current merge..." : "Cancelling Fix All...";
    postStatusMessage(fixAllCancelText_);
}

bool AppController::fixAllRunning() const noexcept {
    return fixAllPreviewQueued_.has_value() || (fixAllJobRunning_ && repairRunner_.busy());
}

float AppController::fixAllProgress() const noexcept {
    return fixAllJobRunning_ && repairRunner_.busy() ? repairRunner_.progress() : 0.0f;
}

std::string AppController::fixAllStatusText() const {
    if (fixAllPreviewQueued_) {
        return "Fix All: checking which shells overlap...";
    }
    if (!fixAllRunning()) {
        return {};
    }
    if (fixAllCancelling_) {
        return fixAllCancelText_;
    }
    std::string text = repairRunner_.progressText();
    if (text.empty()) {
        const std::optional<GlobalRepairMethod> current = repairRunner_.currentMethod();
        text = current && *current != GlobalRepairMethod::FixAll ? std::string(globalRepairMethodLabel(*current)) + "..."
                                                                 : std::string("checking the mesh...");
    }
    const int percent = static_cast<int>(std::lround(100.0 * static_cast<double>(fixAllProgress())));
    return "Fix All: " + text + " " + std::to_string(percent) + " %";
}

bool AppController::pollFixAll() {
    if (!fixAllOpen()) {
        return false;
    }
    bool changed = false;
    if (backgroundChecks_) {
        ensureFixAllPlan();
    }
    if (std::optional<FinishedFixAllPlan> finished = fixAllPlanRunner_.takeFinished()) {
        acceptFixAllPlan(std::move(*finished));
        changed = true;
    }
    if (fixAllPreviewQueued_ && !fixAllPlanChecking()) {
        const bool onePass = *fixAllPreviewQueued_;
        fixAllPreviewQueued_.reset();
        startFixAllRun(onePass);
        changed = true;
    }
    if (fixAllJobRunning_ && !repairRunner_.busy()) {
        fixAllJobRunning_ = false;
        if (fixAllCancelling_) {
            fixAllCancelling_ = false;
            postStatusMessage("Fix All cancelled: nothing was changed.");
        }
        changed = true;
    }
    return changed;
}

void AppController::waitForFixAll() {
    waitForFixAllPlan();
    repairRunner_.waitIdle();
    pollRepairJobs();
    pollFixAll();
}

const RepairCandidate* AppController::fixAllCandidate() const noexcept {
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [](const RepairCandidate& c) { return c.method == GlobalRepairMethod::FixAll; });
    return it != globalCandidates_.end() ? &*it : nullptr;
}

const FixAllCandidate* AppController::fixAllShown() const noexcept {
    if (const RepairCandidate* candidate = fixAllCandidate(); candidate != nullptr && candidate->fixAll != nullptr) {
        return candidate->fixAll.get();
    }
    return fixAllApplied_ ? fixAllApplied_->run.get() : nullptr;
}

void AppController::backToFixAllPlan() {
    if (!fixAllOpen() || fixAllRunning()) {
        return;
    }
    discardGlobalCandidates();
}

FixAllPanelState AppController::fixAllState() const noexcept {
    if (!fixAllOpen()) {
        return FixAllPanelState::Plan;
    }
    if (fixAllRunning()) {
        return FixAllPanelState::Running;
    }
    if (fixAllCandidate() != nullptr) {
        return FixAllPanelState::Result;
    }
    return fixAllApplied_ ? FixAllPanelState::Report : FixAllPanelState::Plan;
}

// -----------------------------------------------------------------------------
// Apply, health, selection, undo
// -----------------------------------------------------------------------------

void AppController::installHealthReport(std::vector<CategoryResult> results) {
    checkRunner_.cancel();
    std::array<bool, kDefectCategoryCount> installed{};
    for (CategoryResult& result : results) {
        if (result.cancelled) {
            continue;
        }
        const DefectCategory category = result.category;
        result.meshGeneration = meshGeneration_;
        health_[category].status = HealthStatus::Running;
        applyCheckResult(std::move(result));
        installed[categoryIndex(category)] = health_[category].status != HealthStatus::Running;
    }
    // Rows the run did not check (none after a normal run) re-run when Auto, else turn Stale
    for (const DefectCategory category : kAllDefectCategories) {
        CategoryHealth& row = health_[category];
        if (installed[categoryIndex(category)] || row.status == HealthStatus::NotRun) {
            continue;
        }
        row.status = row.autoCheck || row.status == HealthStatus::Running ? HealthStatus::Running : HealthStatus::Stale;
    }
    syncBrowserResult();
    launchQueuedChecks();
}

bool AppController::applyFixAll() {
    const auto it = std::find_if(globalCandidates_.begin(), globalCandidates_.end(),
                                 [](const RepairCandidate& c) { return c.method == GlobalRepairMethod::FixAll; });
    if (it == globalCandidates_.end() || !it->success || it->fixAll == nullptr) {
        lastRepairStatusMessage_ = "Preview Fix All first: nothing changes until Apply.";
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    const std::shared_ptr<const FixAllCandidate> run = it->fixAll;
    const FixAllResult& result = run->result;
    const std::string label = result.onePass ? "Fix All (one pass)" : "Fix All";
    // The run's final check is this mesh's report, unless the thresholds changed since the preview
    const bool sameChecks = sameThresholds(it->params.checkParams, checkParams());
    if (!result.changed) {
        // Nothing to undo: keep the mesh, take the run's check as its report
        globalCandidates_.erase(it);
        shownCandidate_.reset();
        if (sameChecks) {
            installHealthReport(result.report.health);
        }
        fixAllApplied_ = FixAllApplied{run, std::string(), 0, meshGeneration_, run->scopeKind, shells_.revision(),
                                       faceSelectionGeneration_};
        lastRepairStatusMessage_ = "Fix All changed nothing: " + result.report.passesLine + ". " + result.report.leftLine;
        postStatusMessage(lastRepairStatusMessage_);
        return true;
    }
    Mesh repaired = std::move(it->mesh);

    pushUndoEntry(label);
    replaceMeshPreservingHistory(std::move(repaired)); // also drops the candidates
    modified_ = true;
    installHealthReport(sameChecks ? result.report.health : std::vector<CategoryResult>{});

    // Selection: the next pass works on the same part
    const std::vector<std::uint8_t>& inScope = result.scope.faceInScope;
    const auto faceInScope = [&](Eigen::Index f) {
        return static_cast<std::size_t>(f) < inScope.size() && inScope[static_cast<std::size_t>(f)] != 0;
    };
    switch (run->scopeKind) {
    case FixAllScopeKind::Faces: {
        std::vector<Eigen::Index> faces;
        for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
            if (faceInScope(f)) {
                faces.push_back(f);
            }
        }
        selectFaces(faces, true); // an exception to "an edit clears the face selection"
        break;
    }
    case FixAllScopeKind::Shells:
        selectShellsWhere([&](std::size_t s) {
            const std::vector<Eigen::Index>& faces = shells_.shell(s).faces;
            return std::any_of(faces.begin(), faces.end(), faceInScope);
        });
        break;
    case FixAllScopeKind::Whole: {
        // The merged shells, as the Merge Shells operator selects them
        std::vector<std::uint8_t> merged(shells_.size(), 0);
        for (const FixAllStepReport& step : result.steps) {
            if (step.stage != FixAllStageId::MergeShells || !step.accepted() || !step.notesInResult || step.notes.empty()) {
                continue;
            }
            for (const std::vector<Eigen::Index>& faces : step.notes.front().shells) {
                for (const Eigen::Index f : faces) {
                    const int s = shells_.shellOfFace(f);
                    if (s >= 0) {
                        merged[static_cast<std::size_t>(s)] = 1;
                    }
                }
            }
        }
        if (std::find(merged.begin(), merged.end(), std::uint8_t{1}) != merged.end()) {
            selectShellsWhere([&](std::size_t s) { return merged[s] != 0; });
        }
        break;
    }
    }

    fixAllApplied_ = FixAllApplied{run,       label, undoStack_.size(), meshGeneration_, run->scopeKind, shells_.revision(),
                                   faceSelectionGeneration_};
    lastRepairStatusMessage_ = "Applied " + label + ": " + result.report.passesLine + ". " + result.report.shellsLine + ".";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixAllUndoAvailable() const noexcept {
    return fixAllApplied_ && !fixAllApplied_->undoLabel.empty() && !undoStack_.empty() &&
           undoStack_.size() == fixAllApplied_->undoDepth && undoStack_.back().actionLabel == fixAllApplied_->undoLabel;
}

bool AppController::undoFixAll() {
    return fixAllUndoAvailable() && undoLastModification();
}

// -----------------------------------------------------------------------------
// Shell numbers of the report and its notes
// -----------------------------------------------------------------------------

std::vector<int> AppController::fixAllNoteShellNumbers(const FixAllStepReport& step, const FixAllShellNote& note) const {
    const FixAllCandidate* run = fixAllShown();
    if (run == nullptr || !step.notesInResult || note.faceSet) {
        return {};
    }
    std::vector<int> numbers;
    numbers.reserve(note.shells.size());
    for (const std::vector<Eigen::Index>& faces : note.shells) {
        int number = 0;
        for (const Eigen::Index f : faces) {
            if (f >= 0 && static_cast<std::size_t>(f) < run->faceShell.size() && run->faceShell[static_cast<std::size_t>(f)] >= 0) {
                number = fixAllShellNumber(run->faceShell[static_cast<std::size_t>(f)]);
                break;
            }
        }
        numbers.push_back(number);
    }
    return numbers;
}

std::string AppController::fixAllNoteText(const FixAllStepReport& step, const FixAllShellNote& note) const {
    if (!step.notesInResult || note.faceSet) {
        return note.text;
    }
    const std::vector<int> numbers = fixAllNoteShellNumbers(step, note);
    return numbers.empty() && !note.shells.empty() ? note.text : renumberShellText(note.text, numbers);
}

std::vector<int> AppController::fixAllShellsInCurrentMesh(std::span<const int> reportShells) const {
    const RepairCandidate* candidate = fixAllCandidate();
    const FixAllCandidate* run = nullptr;
    const Mesh* resultMesh = nullptr;
    if (candidate != nullptr && candidate->fixAll != nullptr) {
        run = candidate->fixAll.get();
        resultMesh = &candidate->mesh;
    } else if (fixAllApplied_ && fixAllApplied_->meshGeneration == meshGeneration_) {
        run = fixAllApplied_->run.get();
        resultMesh = &mesh_;
    } else {
        return {};
    }
    const std::vector<std::vector<Eigen::Index>>& shells = run->result.report.shells;
    std::vector<int> out;
    const auto valid = [&](int k) { return k >= 0 && static_cast<std::size_t>(k) < shells.size(); };
    if (resultMesh == &mesh_) {
        for (const int k : reportShells) {
            if (valid(k) && !shells[static_cast<std::size_t>(k)].empty()) {
                out.push_back(shells_.shellOfFace(shells[static_cast<std::size_t>(k)].front()));
            }
        }
    } else {
        // The preview: match the result's faces to the current mesh's by their corners
        std::unordered_map<FaceGeometryKey, Eigen::Index, FaceGeometryKeyHash> current;
        current.reserve(static_cast<std::size_t>(mesh_.numFaces()));
        for (Eigen::Index f = 0; f < mesh_.numFaces(); ++f) {
            current.emplace(faceGeometryKey(mesh_, f), f);
        }
        for (const int k : reportShells) {
            if (!valid(k)) {
                continue;
            }
            for (const Eigen::Index f : shells[static_cast<std::size_t>(k)]) {
                const auto found = current.find(faceGeometryKey(*resultMesh, f));
                if (found != current.end()) {
                    out.push_back(shells_.shellOfFace(found->second));
                }
            }
        }
    }
    out.erase(std::remove(out.begin(), out.end(), -1), out.end());
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

void AppController::fixAllShowShells(std::span<const int> reportShells) {
    const std::vector<int> shells = fixAllShellsInCurrentMesh(reportShells);
    if (!shells.empty()) {
        showShells(shells);
    }
}

bool AppController::fixAllFlipShells(std::span<const int> reportShells) {
    const std::vector<int> shells = fixAllShellsInCurrentMesh(reportShells);
    return !shells.empty() && flipShells(shells);
}

} // namespace ogun
