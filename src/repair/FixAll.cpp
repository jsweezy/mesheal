#include "repair/FixAll.hpp"
#include "repair/FixAllDetail.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "repair/MergePlan.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/TopologyCleanup.hpp"

#include <algorithm>
#include <array>
#include <cassert>
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

/// Why the merge would not unite shells `a` and `b` of `plan` (`shellsLargestFirst` of the mesh it was
/// worked out on), or an empty string when they are in one merge group.
[[nodiscard]] std::string notMergeable(const MergePlan& plan, int a, int b) {
    for (const MergePlanGroup& group : plan.groups) {
        const bool hasA = std::find(group.shells.begin(), group.shells.end(), a) != group.shells.end();
        const bool hasB = std::find(group.shells.begin(), group.shells.end(), b) != group.shells.end();
        if (hasA && hasB) {
            return {};
        }
    }
    const auto name = [](int s) { return "#" + std::to_string(s + 1); };
    for (const int s : {a, b}) {
        if (static_cast<std::size_t>(s) >= plan.shellInScope.size() || plan.shellInScope[static_cast<std::size_t>(s)] == 0) {
            return name(s) + " is not selected";
        }
    }
    for (const MergePlanRefusedGroup& group : plan.refusedGroups) {
        const bool hasA = std::find(group.shells.begin(), group.shells.end(), a) != group.shells.end();
        const bool hasB = std::find(group.shells.begin(), group.shells.end(), b) != group.shells.end();
        if ((hasA || hasB) && !group.hollows.empty()) {
            return "their group reaches into cavity " + name(group.hollows.front());
        }
    }
    for (const std::vector<int>& stopped : plan.stoppedGroups) {
        for (const int s : {a, b}) {
            if (std::find(stopped.begin(), stopped.end(), s) != stopped.end()) {
                return name(s) + " looks inside out with its inner shells";
            }
        }
    }
    for (const MergePlanSkipped& skipped : plan.skipped) {
        if (skipped.shell != a && skipped.shell != b) {
            continue;
        }
        const std::string s = name(skipped.shell);
        switch (skipped.why) {
        case MergeSkipReason::Open:
            return s + " is not closed";
        case MergeSkipReason::FinEdges:
            return s + " has Fin / other edges";
        case MergeSkipReason::Flipped:
            return s + "'s faces are not wound consistently";
        case MergeSkipReason::InsideOut:
            return s + " is inside out";
        case MergeSkipReason::Unclassified:
            return "rule O cannot tell whether " + s + " is a cavity";
        case MergeSkipReason::NotManifold:
            return "Manifold cannot read " + s;
        case MergeSkipReason::NoVolume:
            return s + " has no volume";
        case MergeSkipReason::ContainerNotMergeable:
            return s + " lies inside a shell that cannot be merged";
        }
    }
    for (const int s : {a, b}) {
        const ShellRole role = static_cast<std::size_t>(s) < plan.roles.shells.size()
                                   ? plan.roles.shells[static_cast<std::size_t>(s)].role
                                   : ShellRole::Unclassified;
        if (role == ShellRole::Cavity || static_cast<std::size_t>(s) >= plan.relations.shells.size() ||
            plan.relations.shells[static_cast<std::size_t>(s)].signedVolume < 0.0) {
            return name(s) + " is wound inward";
        }
    }
    return name(a) + " and " + name(b) + " would not be merged";
}

/// Dead-end guard: the faces a step moved or made (`faceDiff`) cross a face of another shell that none
/// of the faces it replaced crossed, and the merge cannot take that pair: merge off, a face scope, or the
/// merge plan of the step's result (`planMerge`, worked out only when such a crossing appears) does not
/// put the two shells in one merge group (a shell not selected, skipped, stopped, wound inward, or in a
/// group that reaches into a cavity).
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

    const bool mergeOn = params.mergeShells && scopeAfter.kind != FixAllScopeKind::Faces;
    std::optional<MergePlan> plan; // worked out on the step's result when a crossing needs it
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
            if (!plan) {
                MergePlanParams planParams;
                planParams.intersectionTolerance = params.checkParams.intersectionTolerance;
                plan = planMerge(after, scopeAfter, planParams, stop);
            }
            if (!plan->complete) {
                return {}; // stopped: the engine drops the run
            }
            why = notMergeable(*plan, sm, so);
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

/// Nested solids the merge would delete (rule O NestedSolid touching and holding nothing, it and its
/// container in scope). Relations are worked out only when some shell's box lies inside another's.
/// std::nullopt when stopped.
[[nodiscard]] std::optional<std::size_t> countIsolatedNestedSolids(const Mesh& mesh, const FixAllScope& scope,
                                                                    const CheckParams& params, std::stop_token stop) {
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    if (shells.size() < 2) {
        return std::size_t{0};
    }
    std::vector<BoundingBox> boxes;
    boxes.reserve(shells.size());
    for (const std::vector<Eigen::Index>& faces : shells) {
        BoundingBox box{Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
                        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity())};
        for (const Eigen::Index f : faces) {
            for (int c = 0; c < 3; ++c) {
                const Eigen::Vector3d p = mesh.V.row(mesh.F(f, c)).transpose();
                box.min = box.min.cwiseMin(p);
                box.max = box.max.cwiseMax(p);
            }
        }
        boxes.push_back(box);
    }
    bool anyInside = false;
    for (std::size_t a = 0; a < boxes.size() && !anyInside; ++a) {
        for (std::size_t b = 0; b < boxes.size() && !anyInside; ++b) {
            anyInside = a != b && (boxes[a].min.array() >= boxes[b].min.array()).all() &&
                        (boxes[a].max.array() <= boxes[b].max.array()).all();
        }
    }
    if (!anyInside) {
        return std::size_t{0};
    }
    ShellRelationParams relationParams;
    relationParams.intersectionTolerance = params.intersectionTolerance;
    const ShellRelations relations = computeShellRelations(mesh, shells, relationParams, stop);
    if (!relations.complete || stop.stop_requested()) {
        return std::nullopt;
    }
    const ShellRoles roles = classifyShellRoles(mesh, relations);
    std::vector<std::uint8_t> busy(shells.size(), 0); // meets another shell, or holds one
    for (const ShellPairRelation& pair : relations.pairs) {
        if (pair.crossings > 0 || pair.sharedAreaPairs > 0 || pair.contacts > 0) {
            busy[static_cast<std::size_t>(pair.a)] = 1;
            busy[static_cast<std::size_t>(pair.b)] = 1;
        }
    }
    for (const ShellFacts& facts : relations.shells) {
        for (const int c : facts.containers) {
            busy[static_cast<std::size_t>(c)] = 1;
        }
    }
    const std::vector<std::uint8_t> in = shellsInScope(scope, shells);
    std::size_t count = 0;
    for (std::size_t s = 0; s < shells.size() && s < roles.shells.size(); ++s) {
        const int c = relations.shells[s].nestedIn;
        count += roles.shells[s].role == ShellRole::NestedSolid && busy[s] == 0 && in[s] != 0 && c >= 0 &&
                         in[static_cast<std::size_t>(c)] != 0
                     ? 1
                     : 0;
    }
    return count;
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
    int merges = 0;
    std::vector<Eigen::Index> mergeRemoved; ///< Input faces phase 2 removed up to this state
    std::vector<CategoryResult> health;     ///< The nine checks of this state (whole mesh)
    std::vector<int> fillOf;                ///< Fill record of each face (-1: not made by a fill)
    std::size_t stepCount = 0;              ///< Steps up to this state
    std::vector<std::vector<FixAllShellNote>> notes; ///< Their notes, in this state's numbering
};

/// The deleting steps, whose removed input faces the report lists.
[[nodiscard]] bool deletingStage(FixAllStageId id) noexcept {
    return id == FixAllStageId::TinyShells || id == FixAllStageId::Fins || id == FixAllStageId::DeleteNestedSolids;
}

/// Maps every face of `notes` through `map` (old face -> new row, -1 = removed: dropped).
void mapNoteFaces(std::vector<FixAllShellNote>& notes, std::span<const Eigen::Index> map) {
    for (FixAllShellNote& note : notes) {
        for (std::vector<Eigen::Index>& faces : note.shells) {
            std::size_t kept = 0;
            for (const Eigen::Index f : faces) {
                const Eigen::Index to = f >= 0 && static_cast<std::size_t>(f) < map.size() ? map[static_cast<std::size_t>(f)] : -1;
                if (to >= 0) {
                    faces[kept++] = to;
                }
            }
            faces.resize(kept);
        }
    }
}

/// The identity face map of `faceCount` faces.
[[nodiscard]] std::vector<Eigen::Index> identityMap(Eigen::Index faceCount) {
    std::vector<Eigen::Index> map(static_cast<std::size_t>(std::max<Eigen::Index>(faceCount, 0)));
    for (std::size_t f = 0; f < map.size(); ++f) {
        map[f] = static_cast<Eigen::Index>(f);
    }
    return map;
}

[[nodiscard]] std::vector<Eigen::Index> sortedUnique(std::vector<Eigen::Index> faces) {
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    return faces;
}

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
        std::vector<CategoryResult> lastHealth;
        const FixAllCounts baseline =
            countFixAllDefects(input_, scope, params_.checkParams, tinyFraction(), params_.mergeShells, stop_, &lastHealth);
        if (!baseline.complete || stop_.stop_requested()) {
            return cancelled();
        }
        result_.countsBefore = baseline;
        result_.priorityBefore = baseline.priority();
        report(settings::kFixAllBaselineProgressShare, "checking the mesh...");

        mesh_ = input_;
        scope_ = std::move(scope);
        origin_ = identityMap(input_.numFaces());
        fillOf_.assign(static_cast<std::size_t>(input_.numFaces()), -1);
        BestState best{input_, scope_, baseline, result_.priorityBefore, 0, 0, {}, lastHealth, fillOf_, 0, {}};
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

            if (!runPass(pass, passReport)) {
                return cancelled();
            }
            merges_ += passReport.merged() ? 1 : 0;
            report(passFraction(pass, checkStart_), passText(pass, "checking the result..."));
            const FixAllCounts counts = countFixAllDefects(mesh_, scope_, params_.checkParams, tinyFraction(),
                                                           params_.mergeShells, stop_, &lastHealth);
            if (!counts.complete || stop_.stop_requested()) {
                return cancelled();
            }
            lastCounts = counts;
            result_.passesRun = pass;
            passReport.after = counts.priority();
            passReport.stepCount = result_.steps.size() - passReport.firstStep;
            if (passReport.after < best.priority) {
                std::vector<std::vector<FixAllShellNote>> notes;
                notes.reserve(result_.steps.size());
                for (const FixAllStepReport& step : result_.steps) {
                    notes.push_back(step.notes);
                }
                best = BestState{mesh_,         scope_,     counts,  passReport.after,     pass,           merges_,
                                 mergeRemoved_, lastHealth, fillOf_, result_.steps.size(), std::move(notes)};
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

        std::vector<CategoryResult> health;
        std::vector<int> fillOf;
        std::size_t returnedSteps = result_.steps.size();
        if (result_.onePass) {
            // Stepping by hand: the pass is returned even when it is worse (a later pass repairs it)
            result_.mesh = std::move(mesh_);
            result_.scope = std::move(scope_);
            result_.countsAfter = lastCounts;
            result_.bestPass = result_.passesRun;
            result_.merges = merges_;
            result_.mergeRemovedFaces = sortedUnique(std::move(mergeRemoved_));
            health = std::move(lastHealth);
            fillOf = std::move(fillOf_);
        } else {
            result_.mesh = std::move(best.mesh);
            result_.scope = std::move(best.scope);
            result_.countsAfter = best.counts;
            result_.bestPass = best.pass;
            result_.merges = best.merges;
            result_.mergeRemovedFaces = sortedUnique(std::move(best.mergeRemoved));
            health = std::move(best.health);
            fillOf = std::move(best.fillOf);
            // Notes of the steps up to the state returned, in its numbering; later steps describe a
            // state that was not kept
            returnedSteps = best.stepCount;
            for (std::size_t i = 0; i < result_.steps.size(); ++i) {
                if (i < best.stepCount) {
                    result_.steps[i].notes = std::move(best.notes[i]);
                } else {
                    result_.steps[i].notesInResult = false;
                }
            }
        }
        result_.priorityAfter = result_.countsAfter.priority();
        result_.temporarilyWorse = result_.onePass && result_.priorityBefore < result_.priorityAfter;
        result_.changed = !sameMesh(result_.mesh, input_);

        // Steps 18 and 19: what is left, by shell and by crossing pair
        report(settings::kFixAllBaselineProgressShare + settings::kFixAllPassesProgressShare, "checking what is left...");
        fixall_detail::FinalReportInput finalInput{input_, inputScope_, params_, std::move(health), fillOf, fills_, returnedSteps};
        if (!fixall_detail::buildFinalReport(result_, finalInput, stop_) || stop_.stop_requested()) {
            return cancelled();
        }
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

    /// Lets a test edit a phase's stage list.
    void adjust(int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) const {
        if (hooks_ != nullptr && hooks_->adjustStages) {
            hooks_->adjustStages(pass, phase, stages);
        }
    }

    /// Runs `stages` of `phase` in order, sharing `weight` of the pass's progress (phase 1: each stage's
    /// own weight; phase 2: in proportion to the stages' weights, i.e. their face counts; phase 3:
    /// equally). Sets `changed` when a step was kept. False when cancelled.
    bool runPhase(std::vector<FixAllStage>& stages, int pass, FixAllPhase phase, double total, double& done,
                  double weight, bool& changed) {
        const auto phaseName = "phase " + std::to_string(static_cast<int>(phase)) + ", ";
        double weights = 0.0;
        for (const FixAllStage& stage : stages) {
            weights += std::max(0.0, stage.progressWeight);
        }
        for (std::size_t i = 0; i < stages.size(); ++i) {
            if (stop_.stop_requested()) {
                return false;
            }
            report(passFraction(pass, done / total), passText(pass, phaseName + stages[i].activity + "..."));
            if (!applyStep(stages, i, pass, phase)) {
                return false;
            }
            changed = changed || result_.steps.back().status == FixAllStepStatus::Applied;
            const double own = std::max(0.0, stages[i].progressWeight);
            done += phase == FixAllPhase::Repair                  ? own
                    : phase == FixAllPhase::Merge && weights > 0.0 ? weight * own / weights
                                                                   : weight / static_cast<double>(stages.size());
        }
        if (phase != FixAllPhase::Repair && (stages.empty() || (phase == FixAllPhase::Merge && weights <= 0.0))) {
            done += weight;
        }
        return true;
    }

    /// Records a step that did not run (the merge under a face scope).
    void skippedStep(FixAllStageId id, std::string label, int pass, FixAllPhase phase, std::string reason) {
        FixAllStepReport step;
        step.stage = id;
        step.label = std::move(label);
        step.pass = pass;
        step.phase = phase;
        step.status = FixAllStepStatus::Skipped;
        step.reason = std::move(reason);
        result_.steps.push_back(std::move(step));
    }

    /// Records the merge plan as a report-only step: its lines, with the shells they name.
    void planStep(const MergePlan& plan, int pass) {
        FixAllStepReport step;
        step.stage = FixAllStageId::PlanMerge;
        step.label = "Plan the merge";
        step.pass = pass;
        step.phase = FixAllPhase::Merge;
        step.status = FixAllStepStatus::Unchanged;
        for (const MergePlanLine& line : plan.lines) {
            FixAllShellNote note;
            note.text = line.text;
            note.warning = line.kind == MergePlanLineKind::Warning;
            for (const int s : line.shells) {
                if (s >= 0 && static_cast<std::size_t>(s) < plan.shells.size()) {
                    note.shells.push_back(plan.shells[static_cast<std::size_t>(s)]);
                }
            }
            if (line.kind == MergePlanLineKind::Info) {
                step.detail += (step.detail.empty() ? "" : " ") + line.text;
            }
            step.notes.push_back(std::move(note));
        }
        step.countBefore = plan.operandCount();
        step.countAfter = plan.operandCount();
        step.elapsedMs = plan.elapsedMs;
        result_.steps.push_back(std::move(step));
    }

    /// Runs one pass on `mesh_`; false when cancelled.
    bool runPass(int pass, FixAllPassReport& passReport) {
        std::vector<FixAllStage> repair = fixAllRepairStages();
        adjust(pass, FixAllPhase::Repair, repair);

        // Progress layout of the pass: phase 1 stages, phase 2 and 3 (merge on, or a test that may add a
        // merge), the closing check
        double total = settings::kFixAllPassCheckProgressWeight;
        for (const FixAllStage& stage : repair) {
            total += std::max(0.0, stage.progressWeight);
        }
        const bool mergeShare = params_.mergeShells || (hooks_ != nullptr && hooks_->adjustStages);
        const double mergeWeight = mergeShare ? settings::kFixAllMergeProgressWeight : 0.0;
        const double cleanupWeight = mergeShare ? settings::kFixAllCleanupProgressWeight : 0.0;
        total += mergeWeight + cleanupWeight;
        double done = 0.0;

        // ---- Phase 1: repair each shell ----
        bool repaired = false;
        if (!runPhase(repair, pass, FixAllPhase::Repair, total, done, 0.0, repaired)) {
            return false;
        }

        // ---- Phase 2: merge the shells (whole shells only), planned once per pass on phase 1's result ----
        if (stop_.stop_requested()) {
            return false;
        }
        std::vector<FixAllStage> merge;
        double stagesWeight = mergeWeight;
        if (mergeRuns()) {
            report(passFraction(pass, done / total), passText(pass, "phase 2, planning the merge..."));
            MergePlanParams planParams;
            planParams.intersectionTolerance = params_.checkParams.intersectionTolerance;
            MergePlan plan = planMerge(mesh_, scope_, planParams, stop_);
            if (!plan.complete || stop_.stop_requested()) {
                return false;
            }
            if (hooks_ != nullptr && hooks_->adjustPlan) {
                hooks_->adjustPlan(pass, plan);
            }
            planStep(plan, pass);
            merge = fixAllMergeStages(plan);
            done += mergeWeight * settings::kFixAllMergePlanProgressShare;
            stagesWeight = mergeWeight * (1.0 - settings::kFixAllMergePlanProgressShare);
        } else if (params_.mergeShells) {
            skippedStep(FixAllStageId::PlanMerge, "Plan the merge", pass, FixAllPhase::Merge,
                        "Merge needs whole shells: select shells, or clear the selection.");
        }
        adjust(pass, FixAllPhase::Merge, merge);
        const Mesh preMerge = merge.empty() ? Mesh{} : mesh_;
        planFaceMap_ = identityMap(mesh_.numFaces());
        bool merged = false;
        if (scope_.kind == FixAllScopeKind::Faces) {
            for (const FixAllStage& stage : merge) {
                skippedStep(stage.id, stage.label, pass, FixAllPhase::Merge,
                            "Merge needs whole shells: select shells, or clear the selection.");
            }
            done += stagesWeight;
        } else {
            const std::size_t firstMergeStep = result_.steps.size();
            if (!runPhase(merge, pass, FixAllPhase::Merge, total, done, stagesWeight, merged)) {
                return false;
            }
            for (std::size_t i = firstMergeStep; i < result_.steps.size(); ++i) {
                const FixAllStepReport& step = result_.steps[i];
                if (step.accepted() && step.stage == FixAllStageId::MergeShells) {
                    ++passReport.groupsMerged;
                } else if (step.accepted() && step.stage == FixAllStageId::DeleteNestedSolids) {
                    passReport.nestedSolidsDeleted += step.countBefore - step.countAfter;
                }
            }
        }
        planFaceMap_.clear();

        // ---- Phase 3: clean up the merged result (only when phase 2 changed the mesh) ----
        if (merged) {
            std::vector<FixAllStage> cleanup = fixAllCleanupStages();
            adjust(pass, FixAllPhase::Cleanup, cleanup);
            preMerge_ = &preMerge;
            bool cleaned = false;
            const bool finished = runPhase(cleanup, pass, FixAllPhase::Cleanup, total, done, cleanupWeight, cleaned);
            preMerge_ = nullptr;
            if (!finished) {
                return false;
            }
        } else {
            done += cleanupWeight;
        }
        if (stop_.stop_requested()) {
            return false;
        }
        checkStart_ = done / total;
        return true;
    }

    /// Runs `stages[index]` on a copy of `mesh_`, checks the hard guards, and keeps or discards the
    /// result. False when cancelled.
    bool applyStep(std::span<const FixAllStage> stages, std::size_t index, int pass, FixAllPhase phase) {
        const FixAllStage& stage = stages[index];
        assert(scope_.matches(mesh_)); // a stage that returns a wrong face map would make the scope drift
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
            const std::span<const Eigen::Index> planMap =
                phase == FixAllPhase::Merge ? std::span<const Eigen::Index>(planFaceMap_) : std::span<const Eigen::Index>();
            outcome = stage.run(mesh_, FixAllStageContext{params_, scope_, pass, phase, stop_, preMerge_, planMap});
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
        } else if (!outcome.refusal.empty()) {
            refuse(FixAllGuard::Merge, outcome.refusal);
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
        const auto guardStart = Clock::now();
        struct GuardTimer {
            FixAllStepReport& step;
            Clock::time_point start;
            ~GuardTimer() { step.guardMs = msSince(start); }
        } timer{step, guardStart};
        const std::vector<Eigen::Index> map = fullFaceMap(mesh_, outcome);
        if (!scope_.everyFace() && !fixall_detail::outsideScopeKept(mesh_, outcome.mesh, map, scope_)) {
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
            const bool fillLater = step.phase == FixAllPhase::Merge || laterStage(stages, index, {FixAllStageId::FillHoles});
            step.deferred.push_back({FixAllDeferredKind::OpenEdges, more,
                                     fixall_detail::counted(more, "more open edge", "more open edges") +
                                         (fillLater ? ": left for the hole fill" : ": left for the next pass's hole fill")});
        }
        if (after.nonManifold > edges_->nonManifold) {
            const std::size_t more = after.nonManifold - edges_->nonManifold;
            const bool splitLater = step.phase == FixAllPhase::Merge ||
                                    laterStage(stages, index, {FixAllStageId::SplitTouching, FixAllStageId::Fins});
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

        // Carry each face's input face (for what phase 2 and the deleting steps removed), the fill that
        // made it, and phase 2's plan rows
        const auto newFaces = static_cast<std::size_t>(outcome.mesh.numFaces());
        std::vector<Eigen::Index> origin(newFaces, -1);
        std::vector<int> fillOf(newFaces, -1);
        const bool deleting = deletingStage(stage.id);
        for (std::size_t f = 0; f < map.size() && f < origin_.size(); ++f) {
            if (map[f] >= 0) {
                origin[static_cast<std::size_t>(map[f])] = origin_[f];
                fillOf[static_cast<std::size_t>(map[f])] = f < fillOf_.size() ? fillOf_[f] : -1;
            } else if (origin_[f] >= 0) {
                if (step.phase == FixAllPhase::Merge) {
                    mergeRemoved_.push_back(origin_[f]);
                }
                if (deleting) {
                    step.removedInputFaces.push_back(origin_[f]);
                }
            }
        }
        std::sort(step.removedInputFaces.begin(), step.removedInputFaces.end());
        for (const FixAllFillPatch& patch : outcome.fills) {
            const int record = static_cast<int>(fills_.size());
            fills_.push_back({step.pass, patch.hole});
            for (const Eigen::Index f : patch.faces) {
                if (f >= 0 && static_cast<std::size_t>(f) < newFaces) {
                    fillOf[static_cast<std::size_t>(f)] = record;
                }
            }
        }
        origin_ = std::move(origin);
        fillOf_ = std::move(fillOf);
        // Notes of the earlier steps follow their faces to the new mesh
        for (FixAllStepReport& earlier : result_.steps) {
            mapNoteFaces(earlier.notes, map);
        }
        if (step.phase == FixAllPhase::Merge) {
            for (Eigen::Index& row : planFaceMap_) {
                row = row >= 0 && static_cast<std::size_t>(row) < map.size() ? map[static_cast<std::size_t>(row)] : -1;
            }
        }
        scope_ = std::move(carried);
        mesh_ = std::move(outcome.mesh);
        edges_ = after;
        inward_.reset();
        assert(scope_.matches(mesh_));
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
    const Mesh* preMerge_ = nullptr; ///< Phase 3: the mesh before the merge
    std::vector<Eigen::Index> origin_;       ///< Input face of each face of `mesh_` (-1: made by the run)
    std::vector<Eigen::Index> planFaceMap_;  ///< Phase 2: face of the planned mesh -> row of `mesh_`
    std::vector<Eigen::Index> mergeRemoved_; ///< Input faces phase 2 removed (unsorted, may repeat)
    std::vector<int> fillOf_;                ///< Of each face of `mesh_`: its record in `fills_` (-1: not made by a fill)
    std::vector<fixall_detail::FillRecord> fills_; ///< Every kept hole fill: pass and hole
    int merges_ = 0;                         ///< Passes whose phase 2 changed the mesh
    std::optional<std::vector<InwardShell>> inward_;            ///< Of `mesh_`, worked out when a guard needs it
    std::optional<fixall_detail::EdgeDefectCounts> edges_;      ///< Of `mesh_`
    FixAllResult result_;
};

} // namespace

std::size_t fixall_detail::countInScope(const CategoryResult& result, const FixAllScope& scope) {
    return ogun::countInScope(result, scope, scope.everyFace());
}

bool fixall_detail::outsideScopeKept(const Mesh& before, const Mesh& after, std::span<const Eigen::Index> map,
                                     const FixAllScope& scope) {
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        if (scope.inScope(f)) {
            continue;
        }
        const Eigen::Index to = map.empty() ? f : map[static_cast<std::size_t>(f)];
        if (to < 0 || to >= after.numFaces() || faceGeometryKey(before, f) != faceGeometryKey(after, to)) {
            return false;
        }
    }
    return true;
}

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
    p.junk = degenerateFaces + duplicateFaces + tinyShells + nestedSolids;
    p.thin = thinFaces;
    return p;
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                std::stop_token stopToken) {
    return countFixAllDefects(mesh, scope, params, 0.0, std::move(stopToken));
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                double tinyShellFraction, std::stop_token stopToken) {
    return countFixAllDefects(mesh, scope, params, tinyShellFraction, false, std::move(stopToken));
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                double tinyShellFraction, bool nestedSolids, std::stop_token stopToken) {
    return countFixAllDefects(mesh, scope, params, tinyShellFraction, nestedSolids, std::move(stopToken), nullptr);
}

FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                double tinyShellFraction, bool nestedSolids, std::stop_token stopToken,
                                std::vector<CategoryResult>* results) {
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
    if (results != nullptr) {
        results->clear();
        results->reserve(kItems.size());
    }
    for (const Item& item : kItems) {
        if (stopToken.stop_requested()) {
            counts.complete = false;
            return counts;
        }
        CategoryResult result = runCategoryCheck(item.category, mesh, params, stopToken);
        if (result.cancelled) {
            counts.complete = false;
            return counts;
        }
        counts.*item.field = countInScope(result, scope, everyFace);
        if (results != nullptr) {
            results->push_back(std::move(result));
        }
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
    if (nestedSolids && scope.kind != FixAllScopeKind::Faces) {
        const std::optional<std::size_t> nested = countIsolatedNestedSolids(mesh, scope, params, stopToken);
        if (!nested) {
            counts.complete = false;
            return counts;
        }
        counts.nestedSolids = *nested;
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
