#include "repair/FixAll.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/TopologyCleanup.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <optional>
#include <unordered_set>
#include <utility>

namespace ogun {

namespace {

using Clock = std::chrono::steady_clock;

[[nodiscard]] double msSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

[[nodiscard]] std::string counted(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

[[nodiscard]] std::string formatLength(double value, Unit unit) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.3g", value);
    return std::string(text.data()) + " " + std::string(unitSymbol(unit));
}

[[nodiscard]] bool indicesValid(const Mesh& mesh) {
    if (mesh.F.rows() == 0) {
        return true;
    }
    return mesh.F.cols() == 3 && mesh.V.cols() == 3 && mesh.F.minCoeff() >= 0 && mesh.F.maxCoeff() < mesh.V.rows();
}

[[nodiscard]] bool sameMesh(const Mesh& a, const Mesh& b) {
    return a.V.rows() == b.V.rows() && a.V.cols() == b.V.cols() && a.F.rows() == b.F.rows() &&
           a.F.cols() == b.F.cols() && a.V == b.V && a.F == b.F;
}

[[nodiscard]] bool elementTouchesScope(const CategoryResult& result, std::size_t e, const FixAllScope& scope,
                                       ElementKind kind) {
    const auto faces = result.facesOf(e);
    if (!faces.empty()) {
        return std::any_of(faces.begin(), faces.end(), [&](Eigen::Index f) { return scope.inScope(f); });
    }
    return kind != ElementKind::Face || scope.inScope(result.elements[e].index);
}

[[nodiscard]] std::size_t countInScope(const CategoryResult& result, const FixAllScope& scope, bool everyFace) {
    if (result.category == DefectCategory::OpenEdges) {
        if (everyFace) {
            return result.infoCount;
        }
        std::size_t edges = 0;
        for (std::size_t e = 0; e < result.size(); ++e) {
            if (elementTouchesScope(result, e, scope, ElementKind::Hole)) {
                edges += result.edgesOf(e).size();
            }
        }
        return edges;
    }
    if (everyFace) {
        return result.size();
    }
    const ElementKind kind = categoryInfo(result.category).kind;
    std::size_t count = 0;
    for (std::size_t e = 0; e < result.size(); ++e) {
        count += elementTouchesScope(result, e, scope, kind) ? 1 : 0;
    }
    return count;
}

// ---------------------------------------------------------------------------------------------
// Phase 1 stages
// ---------------------------------------------------------------------------------------------

[[nodiscard]] FixAllStageOutcome weldStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    MergeByDistanceResult weld =
        mergeByDistance(mesh, context.params.weldTolerance, context.scope.faceInScope, true, context.scope.pinned);
    if (!weld.success) {
        outcome.failure = weld.statusMessage;
        return outcome;
    }
    outcome.targetBefore = weld.verticesMerged + weld.degenerateRemoved + weld.duplicatesRemoved;
    if (outcome.targetBefore == 0) {
        return outcome; // the weld would only drop unused vertices
    }
    outcome.changed = true;
    outcome.mesh = std::move(weld.mesh);
    outcome.faceMap = std::move(weld.faceMap);
    outcome.maxMove = weld.maxMove;
    outcome.detail = "Welded " + counted(weld.verticesMerged, "vertex", "vertices") + " (max move " +
                     formatLength(weld.maxMove, mesh.fileUnit) + "; " +
                     counted(weld.degenerateRemoved, "collapsed face", "collapsed faces") + " removed";
    if (weld.duplicatesRemoved > 0) {
        outcome.detail += ", " + counted(weld.duplicatesRemoved, "copy", "copies");
    }
    outcome.detail += ")";
    return outcome;
}

[[nodiscard]] FixAllStageOutcome dropRepeatedStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    RepeatedVertexFacesResult dropped = removeRepeatedVertexFaces(mesh, context.scope.faceInScope);
    outcome.targetBefore = dropped.removed;
    if (dropped.removed == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(dropped.mesh);
    outcome.faceMap = std::move(dropped.faceMap);
    outcome.detail = "Removed " + counted(dropped.removed, "collapsed face", "collapsed faces");
    return outcome;
}

[[nodiscard]] FixAllStageOutcome dedupeStage(const Mesh& mesh, const FixAllStageContext& context) {
    FixAllStageOutcome outcome;
    DuplicateFacesResult deduped = removeDuplicateFaces(mesh, context.scope.faceInScope);
    outcome.targetBefore = deduped.removed();
    if (deduped.removed() == 0) {
        return outcome;
    }
    outcome.changed = true;
    outcome.mesh = std::move(deduped.mesh);
    outcome.faceMap = std::move(deduped.faceMap);
    outcome.detail = "Removed " + counted(deduped.removed(), "duplicate face", "duplicate faces") + " (" +
                     std::to_string(deduped.removedSameWinding) + " same, " +
                     std::to_string(deduped.removedOppositeWinding) + " opposite winding)";
    if (deduped.groupsSkipped > 0) {
        outcome.detail += "; " + counted(deduped.groupsSkipped, "set of copies was", "sets of copies were") +
                          " left: a copy outside the scope would have to go";
    }
    return outcome;
}

[[nodiscard]] FixAllStageOutcome splitTouchingStage(const Mesh& mesh, const FixAllStageContext& context) {
    // As the load does (`separateTouchingContacts`): with every touching edge paired, each vertex is
    // split into its fans. Only vertices whose faces are all in scope are split.
    FixAllStageOutcome outcome;
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = touchingEdges(mesh, topo);
    const std::vector<std::uint32_t> fans = cornerFans(mesh, topo, touching);

    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint32_t> firstFan(nv, kNoFan);
    std::vector<std::uint8_t> manyFans(nv, 0);
    std::vector<std::uint8_t> outside(nv, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const bool in = context.scope.inScope(f);
        for (int c = 0; c < 3; ++c) {
            const auto v = static_cast<std::size_t>(mesh.F(f, c));
            outside[v] = static_cast<std::uint8_t>(outside[v] | (in ? 0 : 1));
            const std::uint32_t fan = fans[static_cast<std::size_t>(f) * 3 + static_cast<std::size_t>(c)];
            if (fan == kNoFan) {
                continue;
            }
            if (firstFan[v] == kNoFan) {
                firstFan[v] = fan;
            } else if (firstFan[v] != fan) {
                manyFans[v] = 1;
            }
        }
    }
    std::vector<std::uint8_t> target(nv, 0);
    for (std::size_t v = 0; v < nv; ++v) {
        target[v] = static_cast<std::uint8_t>(manyFans[v] != 0 && outside[v] == 0 ? 1 : 0);
    }
    outcome.targetBefore = static_cast<std::size_t>(std::count(target.begin(), target.end(), std::uint8_t{1}));
    if (outcome.targetBefore == 0) {
        return outcome;
    }
    std::size_t edgesSplit = 0;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (touching[e] != 0 && target[static_cast<std::size_t>(topo.edges[e][0])] != 0 &&
            target[static_cast<std::size_t>(topo.edges[e][1])] != 0) {
            ++edgesSplit;
        }
    }

    VertexSplit split = splitVerticesByFan(mesh, fans, target);
    const std::size_t shellsBefore = shellsLargestFirst(mesh).size();
    const std::size_t shellsAfter = shellsLargestFirst(split.mesh).size();
    outcome.changed = true;
    outcome.mesh = std::move(split.mesh);
    outcome.detail = "Split " + counted(edgesSplit, "touching edge", "touching edges") + " and " +
                     counted(split.verticesSplit, "pinched vertex", "pinched vertices") + "; shells " +
                     std::to_string(shellsBefore) + " to " + std::to_string(shellsAfter);
    return outcome;
}

[[nodiscard]] std::string weldSkipReason(const FixAllParams& params, const FixAllScope&) {
    return params.weld ? std::string() : std::string("Turned off in the options.");
}

// ---------------------------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------------------------

/// Why a stage's result cannot be kept as it is, or an empty string.
[[nodiscard]] std::string invalidResultReason(const Mesh& before, const FixAllStageOutcome& outcome) {
    const Mesh& after = outcome.mesh;
    if (!indicesValid(after) || (after.F.rows() > 0 && after.F.cols() != 3)) {
        return "It returned faces with invalid vertex indices.";
    }
    const auto newFaces = after.numFaces();
    if (outcome.faceMap.empty()) {
        return newFaces >= before.numFaces() ? std::string() : std::string("It removed faces without a face map.");
    }
    if (outcome.faceMap.size() != static_cast<std::size_t>(before.numFaces())) {
        return "Its face map does not match the mesh.";
    }
    std::vector<std::uint8_t> hit(static_cast<std::size_t>(newFaces), 0);
    for (const Eigen::Index target : outcome.faceMap) {
        if (target == -1) {
            continue;
        }
        if (target < 0 || target >= newFaces || hit[static_cast<std::size_t>(target)] != 0) {
            return "Its face map does not match the mesh.";
        }
        hit[static_cast<std::size_t>(target)] = 1;
    }
    return {};
}

[[nodiscard]] std::size_t removedFaceCount(const FixAllStageOutcome& outcome) {
    return static_cast<std::size_t>(std::count(outcome.faceMap.begin(), outcome.faceMap.end(), Eigen::Index{-1}));
}

struct BestState {
    Mesh mesh;
    FixAllScope scope;
    FixAllCounts counts;
    FixAllPriority priority;
    int pass = 0;
};

class FixAllRun {
public:
    FixAllRun(const Mesh& input, const FixAllParams& params, std::stop_token stopToken, FixAllProgress* progress,
              const FixAllTestHooks* hooks)
        : input_(input), params_(params), stop_(std::move(stopToken)), progress_(progress), hooks_(hooks),
          passes_(std::clamp(params.passes, settings::kFixAllMinPasses, settings::kFixAllMaxPasses)),
          start_(Clock::now()) {}

    FixAllResult run(FixAllScope scope) {
        result_.onePass = passes_ == 1;
        inputScope_ = scope;
        result_.weldPlan = planFixAllWeld(input_, scope, params_.weldTolerance);
        if (!params_.weld) {
            result_.weldPlan.noteNeeded = false;
        }

        // Step 0: the input is pass 0
        report(0.0f, "checking the mesh...");
        const FixAllCounts baseline = countFixAllDefects(input_, scope, params_.checkParams, stop_);
        if (!baseline.complete || stop_.stop_requested()) {
            return cancelled();
        }
        result_.countsBefore = baseline;
        result_.priorityBefore = baseline.priority();
        report(settings::kFixAllBaselineProgressShare, "checking the mesh...");

        mesh_ = input_;
        scope_ = std::move(scope);
        BestState best{input_, scope_, baseline, result_.priorityBefore, 0};
        FixAllCounts lastCounts = baseline;
        std::unordered_set<std::uint64_t> seen{meshGeometryHash(input_)};
        int passesWithoutProgress = 0;
        result_.stopReason = FixAllStopReason::PassLimit;

        for (int pass = 1; pass <= passes_; ++pass) {
            const auto passStart = Clock::now();
            FixAllPassReport passReport;
            passReport.pass = pass;
            passReport.before = lastCounts.priority();
            passReport.firstStep = result_.steps.size();

            if (!runPass(pass)) {
                return cancelled();
            }
            report(passFraction(pass, checkStart_), passText(pass, "checking the result..."));
            const FixAllCounts counts = countFixAllDefects(mesh_, scope_, params_.checkParams, stop_);
            if (!counts.complete || stop_.stop_requested()) {
                return cancelled();
            }
            lastCounts = counts;
            result_.passesRun = pass;
            passReport.after = counts.priority();
            passReport.stepCount = result_.steps.size() - passReport.firstStep;
            if (passReport.after < best.priority) {
                best = BestState{mesh_, scope_, counts, passReport.after, pass};
                passReport.newBest = true;
                passesWithoutProgress = 0;
            } else {
                ++passesWithoutProgress;
            }
            passReport.repeated = !seen.insert(meshGeometryHash(mesh_)).second;
            passReport.elapsedMs = msSince(passStart);
            result_.passes.push_back(passReport);
            report(passFraction(pass, 1.0), passText(pass, "checking the result..."));

            if (passReport.after.clean()) {
                result_.stopReason = FixAllStopReason::Clean;
                break;
            }
            if (passReport.repeated) {
                result_.stopReason = FixAllStopReason::RepeatedState;
                break;
            }
            if (passesWithoutProgress >= 2) {
                result_.stopReason = FixAllStopReason::NoProgress;
                break;
            }
            if (params_.timeBudgetSeconds > 0.0 && msSince(start_) > params_.timeBudgetSeconds * 1000.0) {
                result_.stopReason = FixAllStopReason::TimeBudget;
                break;
            }
        }

        if (result_.onePass) {
            // Stepping by hand: the pass is returned even when it is worse (a later pass repairs it)
            result_.mesh = std::move(mesh_);
            result_.scope = std::move(scope_);
            result_.countsAfter = lastCounts;
            result_.bestPass = result_.passesRun;
        } else {
            result_.mesh = std::move(best.mesh);
            result_.scope = std::move(best.scope);
            result_.countsAfter = best.counts;
            result_.bestPass = best.pass;
        }
        result_.priorityAfter = result_.countsAfter.priority();
        result_.temporarilyWorse = result_.onePass && result_.priorityBefore < result_.priorityAfter;
        result_.changed = !sameMesh(result_.mesh, input_);
        result_.success = true;
        result_.elapsedMs = msSince(start_);
        report(1.0f, "done");
        return std::move(result_);
    }

private:
    [[nodiscard]] FixAllResult cancelled() {
        result_.success = false;
        result_.cancelled = true;
        result_.stopReason = FixAllStopReason::Cancelled;
        result_.error = "Cancelled.";
        result_.mesh = input_;
        result_.scope = std::move(inputScope_);
        result_.changed = false;
        result_.elapsedMs = msSince(start_);
        return std::move(result_);
    }

    void report(float value, std::string text) {
        if (progress_ != nullptr) {
            progress_->update(value, std::move(text));
        }
    }

    /// Overall fraction at `local` (0..1) through pass `pass`.
    [[nodiscard]] float passFraction(int pass, double local) const {
        const double share = static_cast<double>(settings::kFixAllPassesProgressShare) / passes_;
        return static_cast<float>(settings::kFixAllBaselineProgressShare + share * (pass - 1 + std::clamp(local, 0.0, 1.0)));
    }

    [[nodiscard]] std::string passText(int pass, const std::string& what) const {
        return "pass " + std::to_string(pass) + " of " + std::to_string(passes_) + ", " + what;
    }

    [[nodiscard]] bool mergeRuns() const noexcept {
        return params_.mergeShells && scope_.kind != FixAllScopeKind::Faces;
    }

    /// Runs one pass on `mesh_`; false when cancelled.
    bool runPass(int pass) {
        std::vector<FixAllStage> stages = fixAllRepairStages();
        if (hooks_ != nullptr && hooks_->adjustStages) {
            hooks_->adjustStages(pass, FixAllPhase::Repair, stages);
        }

        // Progress layout of the pass: phase 1 stages, phase 2 and 3 (merge on), the closing check
        double total = settings::kFixAllPassCheckProgressWeight;
        for (const FixAllStage& stage : stages) {
            total += std::max(0.0, stage.progressWeight);
        }
        const double mergeWeight = mergeRuns() ? settings::kFixAllMergeProgressWeight + settings::kFixAllCleanupProgressWeight : 0.0;
        total += mergeWeight;
        double done = 0.0;

        // ---- Phase 1: repair each shell ----
        for (const FixAllStage& stage : stages) {
            if (stop_.stop_requested()) {
                return false;
            }
            report(passFraction(pass, done / total), passText(pass, std::string("phase 1, ") + stage.activity + "..."));
            if (!applyStep(stage, pass, FixAllPhase::Repair)) {
                return false;
            }
            done += std::max(0.0, stage.progressWeight);
        }

        // ---- Phase 2: merge the shells (not built yet: nothing is merged, so phase 3 never runs) ----
        if (stop_.stop_requested()) {
            return false;
        }
        done += mergeWeight;
        checkStart_ = done / total;
        return true;
    }

    /// Runs `stage` on a copy of `mesh_`, checks the guards, and keeps or discards the result. False
    /// when cancelled.
    bool applyStep(const FixAllStage& stage, int pass, FixAllPhase phase) {
        FixAllStepReport step;
        step.stage = stage.id;
        step.label = stage.label;
        step.pass = pass;
        step.phase = phase;

        const std::string skip = stage.skipReason ? stage.skipReason(params_, scope_) : std::string();
        if (!skip.empty() || !stage.run) {
            step.status = FixAllStepStatus::Skipped;
            step.reason = skip.empty() ? std::string("Nothing to run.") : skip;
            result_.steps.push_back(std::move(step));
            return true;
        }

        const auto stepStart = Clock::now();
        FixAllStageOutcome outcome;
        try {
            outcome = stage.run(mesh_, FixAllStageContext{params_, scope_, pass, phase, stop_});
        } catch (const std::exception& error) {
            outcome = FixAllStageOutcome{};
            outcome.failure = std::string("It failed: ") + error.what();
        } catch (...) {
            outcome = FixAllStageOutcome{};
            outcome.failure = "It failed.";
        }
        step.elapsedMs = msSince(stepStart);
        if (outcome.cancelled || stop_.stop_requested()) {
            return false;
        }
        step.countBefore = outcome.targetBefore;
        step.countAfter = outcome.changed ? outcome.targetAfter : outcome.targetBefore;
        step.detail = outcome.detail;

        if (!outcome.failure.empty()) {
            step.status = FixAllStepStatus::Refused;
            step.reason = outcome.failure;
            step.countAfter = step.countBefore;
        } else if (!outcome.changed) {
            step.status = FixAllStepStatus::Unchanged;
        } else if (std::string invalid = invalidResultReason(mesh_, outcome); !invalid.empty()) {
            // Hard guard: valid result
            step.status = FixAllStepStatus::Refused;
            step.reason = std::move(invalid);
            step.countAfter = step.countBefore;
        } else if (outcome.targetAfter >= outcome.targetBefore) {
            // Hard guard: no churn (a step that does not lower its own target changes nothing)
            step.status = FixAllStepStatus::Refused;
            step.reason = "It did not reduce what it repairs (" + std::to_string(outcome.targetBefore) + " -> " +
                          std::to_string(outcome.targetAfter) + "), so nothing was changed.";
            step.countAfter = step.countBefore;
        } else {
            step.status = FixAllStepStatus::Applied;
            step.facesRemoved = removedFaceCount(outcome);
            const auto kept = static_cast<std::size_t>(mesh_.numFaces()) - step.facesRemoved;
            step.facesAdded = static_cast<std::size_t>(outcome.mesh.numFaces()) - kept;
            step.maxMove = outcome.maxMove;
            scope_.carry(outcome.faceMap, outcome.mesh);
            mesh_ = std::move(outcome.mesh);
        }
        result_.steps.push_back(std::move(step));
        return true;
    }

    const Mesh& input_;
    const FixAllParams& params_;
    std::stop_token stop_;
    FixAllProgress* progress_;
    const FixAllTestHooks* hooks_;
    int passes_;
    Clock::time_point start_;
    Mesh mesh_;
    FixAllScope scope_;
    FixAllScope inputScope_;
    double checkStart_ = 0.0; ///< Where the closing check starts within the pass (0..1)
    FixAllResult result_;
};

} // namespace

// ---------------------------------------------------------------------------------------------
// Scope
// ---------------------------------------------------------------------------------------------

FixAllScope FixAllScope::whole(const Mesh& mesh) {
    return fromFaceMask(mesh, FixAllScopeKind::Whole, {});
}

FixAllScope FixAllScope::fromFaceMask(const Mesh& mesh, FixAllScopeKind kind, std::span<const std::uint8_t> faceInScope) {
    FixAllScope scope;
    scope.kind = kind;
    const auto nf = static_cast<std::size_t>(mesh.numFaces());
    if (faceInScope.empty()) {
        scope.faceInScope.assign(nf, 1);
    } else if (faceInScope.size() == nf) {
        scope.faceInScope.resize(nf);
        std::transform(faceInScope.begin(), faceInScope.end(), scope.faceInScope.begin(),
                       [](std::uint8_t flag) { return static_cast<std::uint8_t>(flag != 0 ? 1 : 0); });
    } else {
        return scope; // does not match the mesh
    }
    scope.updatePins(mesh);
    return scope;
}

bool FixAllScope::matches(const Mesh& mesh) const noexcept {
    return faceInScope.size() == static_cast<std::size_t>(mesh.numFaces()) &&
           pinned.size() == static_cast<std::size_t>(mesh.numVertices());
}

bool FixAllScope::inScope(Eigen::Index face) const noexcept {
    return face >= 0 && static_cast<std::size_t>(face) < faceInScope.size() &&
           faceInScope[static_cast<std::size_t>(face)] != 0;
}

std::size_t FixAllScope::faceCount() const noexcept {
    return static_cast<std::size_t>(std::count(faceInScope.begin(), faceInScope.end(), std::uint8_t{1}));
}

bool FixAllScope::everyFace() const noexcept {
    return std::all_of(faceInScope.begin(), faceInScope.end(), [](std::uint8_t flag) { return flag != 0; });
}

void FixAllScope::carry(std::span<const Eigen::Index> faceMap, const Mesh& result) {
    const auto nf = static_cast<std::size_t>(result.numFaces());
    std::vector<std::uint8_t> carried(nf, 1); // faces no old face maps to are new: in scope
    if (faceMap.empty()) {
        const std::size_t kept = std::min(nf, faceInScope.size());
        std::copy_n(faceInScope.begin(), kept, carried.begin());
    } else {
        const std::size_t old = std::min(faceMap.size(), faceInScope.size());
        for (std::size_t f = 0; f < old; ++f) {
            const Eigen::Index target = faceMap[f];
            if (target >= 0 && static_cast<std::size_t>(target) < nf) {
                carried[static_cast<std::size_t>(target)] = faceInScope[f];
            }
        }
    }
    faceInScope = std::move(carried);
    updatePins(result);
}

void FixAllScope::updatePins(const Mesh& mesh) {
    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint8_t> usedIn(nv, 0);
    std::vector<std::uint8_t> usedOut(nv, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        std::vector<std::uint8_t>& used = inScope(f) ? usedIn : usedOut;
        for (int c = 0; c < 3; ++c) {
            const int v = mesh.F(f, c);
            if (v >= 0 && static_cast<std::size_t>(v) < nv) {
                used[static_cast<std::size_t>(v)] = 1;
            }
        }
    }
    pinned.assign(nv, 0);
    for (std::size_t v = 0; v < nv; ++v) {
        pinned[v] = static_cast<std::uint8_t>(usedIn[v] != 0 && usedOut[v] != 0 ? 1 : 0);
    }
}

// ---------------------------------------------------------------------------------------------
// Score, hash, weld plan
// ---------------------------------------------------------------------------------------------

FixAllPriority FixAllCounts::priority() const noexcept {
    FixAllPriority p;
    p.closedManifold = openEdges + nonManifoldEdges + nonManifoldVertices;
    p.oriented = flippedFaces + invertedShells;
    p.intersecting = intersectingFaces;
    p.junk = degenerateFaces + duplicateFaces;
    p.thin = thinFaces;
    return p;
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                std::stop_token stopToken) {
    struct Item {
        DefectCategory category;
        std::size_t FixAllCounts::*field;
    };
    static constexpr std::array<Item, 9> kItems = {{
        {DefectCategory::OpenEdges, &FixAllCounts::openEdges},
        {DefectCategory::NonManifoldEdges, &FixAllCounts::nonManifoldEdges},
        {DefectCategory::NonManifoldVertices, &FixAllCounts::nonManifoldVertices},
        {DefectCategory::FlippedNormals, &FixAllCounts::flippedFaces},
        {DefectCategory::InvertedShells, &FixAllCounts::invertedShells},
        {DefectCategory::Degenerate, &FixAllCounts::degenerateFaces},
        {DefectCategory::Duplicates, &FixAllCounts::duplicateFaces},
        {DefectCategory::ThinFaces, &FixAllCounts::thinFaces},
        {DefectCategory::Intersections, &FixAllCounts::intersectingFaces},
    }};

    FixAllCounts counts;
    const bool everyFace = scope.everyFace();
    for (const Item& item : kItems) {
        if (stopToken.stop_requested()) {
            counts.complete = false;
            return counts;
        }
        const CategoryResult result = runCategoryCheck(item.category, mesh, params, stopToken);
        if (result.cancelled) {
            counts.complete = false;
            return counts;
        }
        counts.*item.field = countInScope(result, scope, everyFace);
    }
    return counts;
}

std::uint64_t meshGeometryHash(const Mesh& mesh) {
    std::vector<std::uint64_t> keys;
    keys.reserve(static_cast<std::size_t>(mesh.numFaces()));
    const FaceGeometryKeyHash hasher;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        keys.push_back(static_cast<std::uint64_t>(hasher(faceGeometryKey(mesh, f))));
    }
    std::sort(keys.begin(), keys.end());
    std::uint64_t h = keys.size();
    for (const std::uint64_t key : keys) {
        h ^= key + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return h;
}

double FixAllWeldPlan::diagonalShare() const noexcept {
    return scopeDiagonal > 0.0 ? maxTolerance / scopeDiagonal : 0.0;
}

FixAllWeldPlan planFixAllWeld(const Mesh& mesh, const FixAllScope& scope, const ToleranceConfig& tolerance) {
    FixAllWeldPlan plan;
    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint8_t> used(nv, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (!scope.inScope(f)) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            const int v = mesh.F(f, c);
            if (v >= 0 && static_cast<std::size_t>(v) < nv) {
                used[static_cast<std::size_t>(v)] = 1;
            }
        }
    }

    double largest = 0.0;
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo;
    bool any = false;
    for (std::size_t v = 0; v < nv; ++v) {
        const Eigen::Vector3d p = mesh.V.row(static_cast<Eigen::Index>(v)).transpose();
        if (used[v] == 0 || !p.allFinite()) {
            continue;
        }
        any = true;
        largest = std::max(largest, p.cwiseAbs().maxCoeff());
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    plan.maxTolerance = tolerance.coordinateTolerance(largest, 0.0);
    plan.scopeDiagonal = any ? (hi - lo).norm() : 0.0;
    plan.noteNeeded = any && plan.maxTolerance > settings::kFixAllWeldNoteDiagonalShare * plan.scopeDiagonal;

    // Edges of faces in scope whose two ends match: a weld with surfaces kept apart collapses them
    const EdgeTopology topo = buildEdgeTopology(mesh);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        bool inScope = false;
        for (std::uint32_t i = topo.faceOffsets[e]; i < topo.faceOffsets[e + 1] && !inScope; ++i) {
            inScope = scope.inScope(static_cast<Eigen::Index>(topo.incidentFaces[i]));
        }
        if (!inScope) {
            continue;
        }
        const Eigen::Vector3d p = mesh.V.row(topo.edges[e][0]).transpose();
        const Eigen::Vector3d q = mesh.V.row(topo.edges[e][1]).transpose();
        if (p.allFinite() && q.allFinite() && tolerance.verticesMatchWithinTolerance(p, q)) {
            ++plan.collapsingEdges;
        }
    }
    return plan;
}

// ---------------------------------------------------------------------------------------------
// Stages, progress, run
// ---------------------------------------------------------------------------------------------

std::vector<FixAllStage> fixAllRepairStages() {
    std::vector<FixAllStage> stages;
    stages.reserve(4);
    stages.push_back({FixAllStageId::Weld, "Weld by distance", "welding", settings::kFixAllQuickStageProgressWeight,
                      weldSkipReason, weldStage});
    stages.push_back({FixAllStageId::DropRepeated, "Remove collapsed faces", "removing collapsed faces",
                      settings::kFixAllQuickStageProgressWeight, {}, dropRepeatedStage});
    stages.push_back({FixAllStageId::Dedupe, "Remove duplicate faces", "removing duplicate faces",
                      settings::kFixAllQuickStageProgressWeight, {}, dedupeStage});
    stages.push_back({FixAllStageId::SplitTouching, "Split touching contacts", "splitting touching contacts",
                      settings::kFixAllQuickStageProgressWeight, {}, splitTouchingStage});
    return stages;
}

const char* fixAllStopReasonLabel(FixAllStopReason reason) noexcept {
    switch (reason) {
    case FixAllStopReason::Clean:
        return "clean";
    case FixAllStopReason::RepeatedState:
        return "repeated state";
    case FixAllStopReason::NoProgress:
        return "no further progress";
    case FixAllStopReason::PassLimit:
        return "pass limit";
    case FixAllStopReason::TimeBudget:
        return "time budget";
    case FixAllStopReason::Cancelled:
        return "cancelled";
    case FixAllStopReason::InvalidInput:
        return "invalid input";
    }
    return "";
}

void FixAllProgress::update(float value, std::string text) {
    // Never backwards
    float current = fraction.load();
    while (value > current && !fraction.compare_exchange_weak(current, value)) {
    }
    const float shown = std::max(value, current);
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        text_ = text;
    }
    if (observer) {
        observer(shown, text);
    }
}

std::string FixAllProgress::text() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return text_;
}

FixAllResult runFixAll(const Mesh& mesh, const FixAllScope& scope, const FixAllParams& params,
                       std::stop_token stopToken, FixAllProgress* progress, const FixAllTestHooks* hooks) {
    if (mesh.empty() || !indicesValid(mesh) || !scope.matches(mesh)) {
        FixAllResult invalid;
        invalid.mesh = mesh;
        invalid.stopReason = FixAllStopReason::InvalidInput;
        invalid.error = mesh.empty()           ? "No mesh."
                        : !indicesValid(mesh) ? "The mesh has faces with invalid vertex indices."
                                              : "The scope does not match the mesh.";
        return invalid;
    }
    if (stopToken.stop_requested()) {
        FixAllResult stopped;
        stopped.mesh = mesh;
        stopped.scope = scope;
        stopped.cancelled = true;
        stopped.stopReason = FixAllStopReason::Cancelled;
        stopped.error = "Cancelled.";
        return stopped;
    }
    FixAllRun run(mesh, params, std::move(stopToken), progress, hooks);
    return run.run(scope);
}

} // namespace ogun
