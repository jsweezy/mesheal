#include "repair/FixAll.hpp"
#include "repair/FixAllDetail.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/TopologyCleanup.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <initializer_list>
#include <limits>
#include <optional>
#include <span>
#include <unordered_set>
#include <utility>

namespace ogun {

namespace {

using Clock = std::chrono::steady_clock;

[[nodiscard]] double msSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
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
// Hard guards
// ---------------------------------------------------------------------------------------------

/// The stage's face map, or the identity when it kept every row (and only appended).
[[nodiscard]] std::vector<Eigen::Index> fullFaceMap(const Mesh& before, const FixAllStageOutcome& outcome) {
    if (!outcome.faceMap.empty()) {
        return outcome.faceMap;
    }
    std::vector<Eigen::Index> map(static_cast<std::size_t>(before.numFaces()));
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        map[static_cast<std::size_t>(f)] = f;
    }
    return map;
}

/// Scope guard: every face outside the scope is still there with the same corners.
[[nodiscard]] bool outsideScopeKept(const Mesh& before, const Mesh& after, std::span<const Eigen::Index> map,
                                    const FixAllScope& scope) {
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        if (scope.inScope(f)) {
            continue;
        }
        const Eigen::Index to = map[static_cast<std::size_t>(f)];
        if (to < 0 || to >= after.numFaces() || faceGeometryKey(before, f) != faceGeometryKey(after, to)) {
            return false;
        }
    }
    return true;
}

/// A closed, consistently wound shell with a negative volume: a cavity, an inside-out shell or one rule O
/// cannot judge. The cavity guard protects every one of them (the orient stage names the inside-out
/// shells it reverses, and the tiny-shell stage the debris it deletes).
struct InwardShell {
    std::vector<Eigen::Index> faces;
    double volume = 0.0;
    int number = 0; ///< Index in `shellsLargestFirst`
};

[[nodiscard]] std::vector<InwardShell> inwardShells(const Mesh& mesh) {
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    const std::vector<ShellFacts> facts = describeShells(mesh, shells);
    std::vector<InwardShell> inward;
    for (std::size_t s = 0; s < facts.size(); ++s) {
        if (facts[s].closed && facts[s].oriented && facts[s].signedVolume < 0.0) {
            inward.push_back({shells[s], facts[s].signedVolume, static_cast<int>(s)});
        }
    }
    return inward;
}

/// Cavity guard: every inward shell of `before` (not named in `exempt`) is still its own closed,
/// consistently wound shell with a negative volume, joined to no other shell's faces and not shrunk.
[[nodiscard]] std::string cavityLoss(const Mesh& before, std::span<const InwardShell> inward, const Mesh& after,
                                     std::span<const Eigen::Index> map, std::span<const Eigen::Index> exempt) {
    if (inward.empty()) {
        return {};
    }
    std::vector<int> owner(static_cast<std::size_t>(before.numFaces()), -1);
    for (std::size_t i = 0; i < inward.size(); ++i) {
        for (const Eigen::Index f : inward[i].faces) {
            owner[static_cast<std::size_t>(f)] = static_cast<int>(i);
        }
    }
    std::vector<std::uint8_t> exempted(inward.size(), 0);
    for (const Eigen::Index f : exempt) {
        if (f >= 0 && f < before.numFaces() && owner[static_cast<std::size_t>(f)] >= 0) {
            exempted[static_cast<std::size_t>(owner[static_cast<std::size_t>(f)])] = 1;
        }
    }
    std::vector<Eigen::Index> origin(static_cast<std::size_t>(after.numFaces()), -1);
    for (std::size_t f = 0; f < map.size(); ++f) {
        if (map[f] >= 0 && map[f] < after.numFaces()) {
            origin[static_cast<std::size_t>(map[f])] = static_cast<Eigen::Index>(f);
        }
    }
    const std::vector<std::vector<Eigen::Index>> afterShells = shellsLargestFirst(after);
    const std::vector<int> shellOf = fixall_detail::shellOfFaces(after.numFaces(), afterShells);
    std::optional<std::vector<ShellFacts>> facts;

    for (std::size_t i = 0; i < inward.size(); ++i) {
        if (exempted[i] != 0) {
            continue;
        }
        const InwardShell& shell = inward[i];
        const auto refuse = [&](const char* what) {
            return "It would " + std::string(what) + " #" + std::to_string(shell.number + 1) +
                   ", which is wound inward (a cavity, or a shell rule O cannot judge), so nothing was changed.";
        };
        int target = -1;
        bool split = false;
        for (const Eigen::Index f : shell.faces) {
            const Eigen::Index to = map[static_cast<std::size_t>(f)];
            if (to < 0) {
                continue;
            }
            const int s = shellOf[static_cast<std::size_t>(to)];
            split = split || s < 0 || (target >= 0 && s != target);
            target = s;
        }
        if (target < 0 && !split) {
            return refuse("remove");
        }
        if (split) {
            return refuse("split");
        }
        if (!facts) {
            facts = describeShells(after, afterShells);
        }
        const ShellFacts& now = (*facts)[static_cast<std::size_t>(target)];
        if (!now.closed || !now.oriented) {
            return refuse("open");
        }
        if (now.signedVolume >= 0.0) {
            return refuse("turn outward");
        }
        for (const Eigen::Index f : afterShells[static_cast<std::size_t>(target)]) {
            const Eigen::Index from = origin[static_cast<std::size_t>(f)];
            if (from >= 0 && owner[static_cast<std::size_t>(from)] != static_cast<int>(i)) {
                return refuse("join another shell to");
            }
        }
        if (std::abs(now.signedVolume) < std::abs(shell.volume) * (1.0 - settings::kFixAllCavityShrinkShare)) {
            return refuse("shrink");
        }
    }
    return {};
}

/// Faces `query` pairs that cross (not only touch), between different shells.
[[nodiscard]] std::optional<std::vector<FacePairHit>> crossingPairs(const Mesh& mesh, const CheckParams& params,
                                                                   std::span<const int> group, std::size_t groups,
                                                                   std::stop_token stop) {
    std::vector<std::uint8_t> interest(groups, 0);
    interest[0] = 1;
    FacePairQuery query;
    query.faceGroup = group;
    query.groupOfInterest = interest;
    const double tol = params.intersectionTolerance * mesh.boundingBox().diagonal();
    std::optional<std::vector<FacePairHit>> hits = findFacePairs(mesh, tol, query, std::move(stop));
    if (hits) {
        std::erase_if(*hits, [](const FacePairHit& hit) { return !hit.hit || hit.touching; });
    }
    return hits;
}

/// Dead-end guard: the faces a step moved or made (`faceDiff`) cross a face of another shell that none
/// of the faces it replaced crossed, and the merge cannot take that pair: merge off, a face scope, a
/// shell not wholly in scope, or a shell that is not closed, consistently wound and outward.
[[nodiscard]] std::string deadEnd(const Mesh& before, const Mesh& after, const FixAllScope& scopeAfter,
                                  const FixAllParams& params, std::stop_token stop) {
    const FaceDiff diff = faceDiff(before, after);
    if (diff.added.empty()) {
        return {};
    }
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(after);
    if (shells.size() < 2) {
        return {};
    }
    const std::vector<int> shellOf = fixall_detail::shellOfFaces(after.numFaces(), shells);
    std::vector<int> group(static_cast<std::size_t>(after.numFaces()));
    for (std::size_t f = 0; f < group.size(); ++f) {
        group[f] = shellOf[f] < 0 ? -1 : shellOf[f] + 1;
    }
    for (const Eigen::Index f : diff.added) {
        group[static_cast<std::size_t>(f)] = 0;
    }
    const auto hits = crossingPairs(after, params.checkParams, group, shells.size() + 1, stop);
    if (!hits || hits->empty()) {
        return {};
    }

    const std::vector<ShellFacts> facts = describeShells(after, shells);
    const bool mergeOn = params.mergeShells && scopeAfter.kind != FixAllScopeKind::Faces;
    const auto whyNot = [&](int s) -> std::string {
        const std::string name = "#" + std::to_string(s + 1);
        const auto& faces = shells[static_cast<std::size_t>(s)];
        if (!std::all_of(faces.begin(), faces.end(), [&](Eigen::Index f) { return scopeAfter.inScope(f); })) {
            return name + " is not selected";
        }
        const ShellFacts& fact = facts[static_cast<std::size_t>(s)];
        if (!fact.closed || !fact.oriented) {
            return name + " is not closed";
        }
        return fact.signedVolume > 0.0 ? std::string() : name + " is wound inward";
    };
    struct Candidate {
        int moved;
        int other;
        FaceGeometryKey key;
        std::string why;
    };
    std::vector<Candidate> candidates;
    for (const FacePairHit& hit : *hits) {
        const bool aMoved = group[hit.a] == 0;
        const std::uint32_t moved = aMoved ? hit.a : hit.b;
        const std::uint32_t other = aMoved ? hit.b : hit.a;
        const int sm = shellOf[moved];
        const int so = shellOf[other];
        if (sm < 0 || so < 0 || sm == so || group[other] == 0) {
            continue;
        }
        std::string why = !params.mergeShells                       ? std::string("the merge is off")
                          : scopeAfter.kind == FixAllScopeKind::Faces ? std::string("the merge needs whole shells")
                                                                     : std::string();
        if (why.empty() && mergeOn) {
            why = whyNot(sm);
            if (why.empty()) {
                why = whyNot(so);
            }
        }
        if (!why.empty()) {
            candidates.push_back({sm, so, faceGeometryKey(after, static_cast<Eigen::Index>(other)), std::move(why)});
        }
    }
    if (candidates.empty()) {
        return {};
    }

    // Crossings the replaced faces already had with other shells are not new
    std::unordered_set<FaceGeometryKey, FaceGeometryKeyHash> crossedBefore;
    if (!diff.removed.empty()) {
        const std::vector<std::vector<Eigen::Index>> beforeShells = shellsLargestFirst(before);
        const std::vector<int> beforeShellOf = fixall_detail::shellOfFaces(before.numFaces(), beforeShells);
        std::vector<int> beforeGroup(static_cast<std::size_t>(before.numFaces()));
        for (std::size_t f = 0; f < beforeGroup.size(); ++f) {
            beforeGroup[f] = beforeShellOf[f] < 0 ? -1 : beforeShellOf[f] + 1;
        }
        for (const Eigen::Index f : diff.removed) {
            beforeGroup[static_cast<std::size_t>(f)] = 0;
        }
        const auto beforeHits = crossingPairs(before, params.checkParams, beforeGroup, beforeShells.size() + 1, stop);
        if (beforeHits) {
            for (const FacePairHit& hit : *beforeHits) {
                const bool aMoved = beforeGroup[hit.a] == 0;
                const std::uint32_t moved = aMoved ? hit.a : hit.b;
                const std::uint32_t other = aMoved ? hit.b : hit.a;
                if (beforeShellOf[moved] != beforeShellOf[other] && beforeGroup[other] != 0) {
                    crossedBefore.insert(faceGeometryKey(before, static_cast<Eigen::Index>(other)));
                }
            }
        }
    }
    for (const Candidate& candidate : candidates) {
        if (crossedBefore.count(candidate.key) == 0) {
            return "It would move faces of #" + std::to_string(candidate.moved + 1) + " through #" +
                   std::to_string(candidate.other + 1) + ", and the merge cannot take them (" + candidate.why +
                   "), so nothing was changed.";
        }
    }
    return {};
}

/// True when a stage after `index` has one of `ids`.
[[nodiscard]] bool laterStage(std::span<const FixAllStage> stages, std::size_t index, std::initializer_list<FixAllStageId> ids) {
    for (std::size_t i = index + 1; i < stages.size(); ++i) {
        if (std::find(ids.begin(), ids.end(), stages[i].id) != ids.end()) {
            return true;
        }
    }
    return false;
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
        const FixAllCounts baseline = countFixAllDefects(input_, scope, params_.checkParams, tinyFraction(), stop_);
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
            const FixAllCounts counts = countFixAllDefects(mesh_, scope_, params_.checkParams, tinyFraction(), stop_);
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

    /// Tiny floating shells count as junk while the step that deletes them is on.
    [[nodiscard]] double tinyFraction() const noexcept {
        return params_.removeTinyShells ? params_.tinyShellFraction : 0.0;
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
        for (std::size_t i = 0; i < stages.size(); ++i) {
            if (stop_.stop_requested()) {
                return false;
            }
            report(passFraction(pass, done / total), passText(pass, std::string("phase 1, ") + stages[i].activity + "..."));
            if (!applyStep(stages, i, pass, FixAllPhase::Repair)) {
                return false;
            }
            done += std::max(0.0, stages[i].progressWeight);
        }

        // ---- Phase 2: merge the shells (not built yet: nothing is merged, so phase 3 never runs) ----
        if (stop_.stop_requested()) {
            return false;
        }
        done += mergeWeight;
        checkStart_ = done / total;
        return true;
    }

    /// Runs `stages[index]` on a copy of `mesh_`, checks the hard guards, and keeps or discards the
    /// result. False when cancelled.
    bool applyStep(std::span<const FixAllStage> stages, std::size_t index, int pass, FixAllPhase phase) {
        const FixAllStage& stage = stages[index];
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
        if (outcome.cancelled || stop_.stop_requested()) {
            return false;
        }
        step.countBefore = outcome.targetBefore;
        step.countAfter = outcome.changed ? outcome.targetAfter : outcome.targetBefore;
        step.detail = outcome.detail;
        step.notes = std::move(outcome.notes);

        const auto refuse = [&](FixAllGuard guard, std::string reason) {
            step.status = FixAllStepStatus::Refused;
            step.guard = guard;
            step.reason = std::move(reason);
            step.countAfter = step.countBefore;
            step.notes.clear(); // they named shells of the mesh that was not kept
        };
        if (!outcome.failure.empty()) {
            refuse(FixAllGuard::Failed, outcome.failure);
        } else if (!outcome.skipped.empty()) {
            step.status = FixAllStepStatus::Skipped;
            step.reason = outcome.skipped;
        } else if (!outcome.changed) {
            step.status = FixAllStepStatus::Unchanged;
        } else if (std::string invalid = invalidResultReason(mesh_, outcome); !invalid.empty()) {
            refuse(FixAllGuard::ValidResult, std::move(invalid));
        } else if (outcome.targetAfter >= outcome.targetBefore) {
            // A step that does not lower its own target changes nothing
            refuse(FixAllGuard::NoChurn, "It did not reduce what it repairs (" + std::to_string(outcome.targetBefore) +
                                             " -> " + std::to_string(outcome.targetAfter) + "), so nothing was changed.");
        } else {
            keepIfGuardsPass(stages, index, outcome, step, refuse);
        }
        step.elapsedMs = msSince(stepStart);
        if (stop_.stop_requested()) {
            return false;
        }
        result_.steps.push_back(std::move(step));
        return true;
    }

    /// The scope, cavity and dead-end guards; keeps the stage's mesh when they pass.
    template <typename Refuse>
    void keepIfGuardsPass(std::span<const FixAllStage> stages, std::size_t index, FixAllStageOutcome& outcome,
                          FixAllStepReport& step, const Refuse& refuse) {
        const FixAllStage& stage = stages[index];
        const std::vector<Eigen::Index> map = fullFaceMap(mesh_, outcome);
        if (!scope_.everyFace() && !outsideScopeKept(mesh_, outcome.mesh, map, scope_)) {
            refuse(FixAllGuard::Scope, "It would change faces outside the scope, so nothing was changed.");
            return;
        }
        if (!inward_) {
            inward_ = inwardShells(mesh_);
        }
        if (std::string lost = cavityLoss(mesh_, *inward_, outcome.mesh, map, outcome.notCavityFaces); !lost.empty()) {
            refuse(FixAllGuard::Cavity, std::move(lost));
            return;
        }
        FixAllScope carried = scope_;
        carried.carry(outcome.faceMap, outcome.mesh);
        if (stage.movesVertices) {
            if (std::string dead = deadEnd(mesh_, outcome.mesh, carried, params_, stop_); !dead.empty()) {
                refuse(FixAllGuard::DeadEnd, std::move(dead));
                return;
            }
        }

        // Kept: note what it leaves for later steps (the deferral table)
        if (!edges_) {
            edges_ = fixall_detail::countEdgeDefects(mesh_);
        }
        const fixall_detail::EdgeDefectCounts after = fixall_detail::countEdgeDefects(outcome.mesh);
        step.deferred = std::move(outcome.deferred);
        if (after.open > edges_->open) {
            const std::size_t more = after.open - edges_->open;
            const bool fillLater = laterStage(stages, index, {FixAllStageId::FillHoles});
            step.deferred.push_back({FixAllDeferredKind::OpenEdges, more,
                                     fixall_detail::counted(more, "more open edge", "more open edges") +
                                         (fillLater ? ": left for the hole fill" : ": left for the next pass's hole fill")});
        }
        if (after.nonManifold > edges_->nonManifold) {
            const std::size_t more = after.nonManifold - edges_->nonManifold;
            const bool splitLater = laterStage(stages, index, {FixAllStageId::SplitTouching, FixAllStageId::Fins});
            step.deferred.push_back({FixAllDeferredKind::NonManifoldEdges, more,
                                     fixall_detail::counted(more, "more non-manifold edge", "more non-manifold edges") +
                                         (splitLater ? ": left for the split and fin steps"
                                                     : ": left for the next pass's split and fin steps")});
        }

        step.status = FixAllStepStatus::Applied;
        step.facesRemoved = removedFaceCount(outcome);
        const auto kept = static_cast<std::size_t>(mesh_.numFaces()) - step.facesRemoved;
        step.facesAdded = static_cast<std::size_t>(outcome.mesh.numFaces()) - kept;
        step.maxMove = outcome.maxMove;
        scope_ = std::move(carried);
        mesh_ = std::move(outcome.mesh);
        edges_ = after;
        inward_.reset();
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
    std::optional<std::vector<InwardShell>> inward_;            ///< Of `mesh_`, worked out when a guard needs it
    std::optional<fixall_detail::EdgeDefectCounts> edges_;      ///< Of `mesh_`
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
    p.junk = degenerateFaces + duplicateFaces + tinyShells;
    p.thin = thinFaces;
    return p;
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                std::stop_token stopToken) {
    return countFixAllDefects(mesh, scope, params, 0.0, std::move(stopToken));
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                double tinyShellFraction, std::stop_token stopToken) {
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
    if (tinyShellFraction > 0.0 && scope.kind != FixAllScopeKind::Faces) {
        TinyShellSearch search;
        search.fraction = tinyShellFraction;
        search.intersectionTolerance = params.intersectionTolerance;
        const TinyFloatingShellsResult tiny = findTinyFloatingShells(mesh, search, stopToken);
        if (!tiny.complete) {
            counts.complete = false;
            return counts;
        }
        for (const TinyFloatingShell& shell : tiny.shells) {
            counts.tinyShells += std::all_of(shell.faces.begin(), shell.faces.end(), [&](Eigen::Index f) { return scope.inScope(f); }) ? 1 : 0;
        }
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
