// Fix All steps 18 and 19: the final check of the mesh a run returns, what is left in each shell with a
// suggestion (alpha wrap, tick the merge, or a report only), every crossing left by shell pair with the
// fill that made it, and the whole-run summary lines. Nothing here changes the mesh.

#include "repair/FixAll.hpp"
#include "repair/FixAllDetail.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "repair/MergePlan.hpp"
#include "repair/RepairMetrics.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ogun {

namespace {

using fixall_detail::counted;
using fixall_detail::shellList;

[[nodiscard]] std::string label(int s) {
    return "#" + std::to_string(s + 1);
}

/// "a", "a and b", "a, b and c".
[[nodiscard]] std::string joinAnd(const std::vector<std::string>& parts) {
    std::string text;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        text += (i == 0 ? "" : i + 1 == parts.size() ? " and " : ", ") + parts[i];
    }
    return text;
}

/// A sentence as a clause: first letter lower case, final period dropped.
[[nodiscard]] std::string asClause(std::string text) {
    if (!text.empty() && text.back() == '.') {
        text.pop_back();
    }
    if (!text.empty() && text.front() >= 'A' && text.front() <= 'Z' && !(text.size() > 1 && text[1] >= 'A' && text[1] <= 'Z')) {
        text.front() = static_cast<char>(text.front() - 'A' + 'a');
    }
    return text;
}

[[nodiscard]] bool contains(const std::vector<int>& list, int value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

/// What the report knows about the shells of the returned mesh, and why two of them were not merged.
class ShellFactsOfResult {
public:
    ShellFactsOfResult(const FixAllResult& result, const FixAllParams& params, std::size_t returnedSteps)
        : result_(result), params_(params) {
        faceScope_ = result.scope.kind == FixAllScopeKind::Faces;
        mergeOn_ = params.mergeShells && !faceScope_;
        for (std::size_t i = 0; i < returnedSteps && i < result.steps.size(); ++i) {
            const FixAllStepReport& step = result.steps[i];
            if (step.stage == FixAllStageId::MergeShells && step.status == FixAllStepStatus::Refused) {
                lastMergeRefusal_ = step.reason;
            }
        }
    }

    /// Works out the shells, relations (every pair, contacts on), roles and, for whole shells, the merge
    /// plan of the returned mesh. False when stopped.
    [[nodiscard]] bool compute(std::stop_token stop) {
        const Mesh& mesh = result_.mesh;
        if (!faceScope_) {
            MergePlanParams planParams;
            planParams.intersectionTolerance = params_.checkParams.intersectionTolerance;
            plan_ = planMerge(mesh, result_.scope, planParams, stop);
            if (!plan_->complete) {
                return false;
            }
            shells_ = plan_->shells;
            relations_ = plan_->relations;
            roles_ = plan_->roles;
            wholeIn_ = plan_->shellInScope;
        } else {
            shells_ = shellsLargestFirst(mesh);
            ShellRelationParams relationParams;
            relationParams.intersectionTolerance = params_.checkParams.intersectionTolerance;
            relations_ = computeShellRelations(mesh, shells_, relationParams, stop);
            if (!relations_.complete) {
                return false;
            }
            roles_ = classifyShellRoles(mesh, relations_);
            wholeIn_ = shellsInScope(result_.scope, shells_);
        }
        anyIn_.assign(shells_.size(), 0);
        for (std::size_t s = 0; s < shells_.size(); ++s) {
            anyIn_[s] = std::any_of(shells_[s].begin(), shells_[s].end(), [&](Eigen::Index f) { return result_.scope.inScope(f); }) ? 1 : 0;
        }
        return !stop.stop_requested();
    }

    [[nodiscard]] const std::vector<std::vector<Eigen::Index>>& shells() const noexcept { return shells_; }
    [[nodiscard]] const ShellRelations& relations() const noexcept { return relations_; }
    [[nodiscard]] const ShellRoles& roles() const noexcept { return roles_; }
    [[nodiscard]] const std::vector<std::uint8_t>& anyInScope() const noexcept { return anyIn_; }
    [[nodiscard]] const MergePlan* plan() const noexcept { return plan_ ? &*plan_ : nullptr; }

    [[nodiscard]] ShellRole role(int s) const noexcept {
        return static_cast<std::size_t>(s) < roles_.shells.size() ? roles_.shells[static_cast<std::size_t>(s)].role
                                                                   : ShellRole::Unclassified;
    }

    /// The stopped group (rule O, Q23) that holds `s`, or nullptr.
    [[nodiscard]] const std::vector<int>* stoppedGroupOf(int s) const {
        for (const std::vector<int>& group : roles_.stoppedGroups) {
            if (contains(group, s)) {
                return &group;
            }
        }
        return nullptr;
    }

    /// Why shells `a` and `b` (they cross) are still apart, and whether ticking the merge would unite them.
    [[nodiscard]] std::pair<std::string, bool> whyNotMerged(int a, int b) const {
        if (faceScope_) {
            return {"the merge needs whole shells (select shells, or clear the selection)", false};
        }
        for (const int s : {a, b}) {
            if (wholeIn_[static_cast<std::size_t>(s)] == 0) {
                return {label(s) + " is not selected", false};
            }
        }
        const MergePlan& plan = *plan_;
        for (const MergePlanRefusedGroup& group : plan.refusedGroups) {
            if (!(contains(group.shells, a) || contains(group.hollows, a) || contains(group.shells, b) ||
                  contains(group.hollows, b)) ||
                group.hollows.empty()) {
                continue;
            }
            const int h = group.hollows.front();
            const int c = relations_.shells[static_cast<std::size_t>(h)].nestedIn;
            return {c >= 0 ? "their group reaches into cavity " + label(h) + " of " + label(c) + ", so it is not merged (merge them by hand)"
                           : "their group reaches into " + label(h) + ", which is wound inward, so it is not merged",
                    false};
        }
        for (const int s : {a, b}) {
            if (stoppedGroupOf(s) != nullptr) {
                return {label(s) + " looks inside out with its inner shells (flip them by hand)", false};
            }
        }
        for (const MergePlanSkipped& skipped : plan.skipped) {
            if (skipped.shell == a || skipped.shell == b) {
                if (skipped.why == MergeSkipReason::Unclassified) {
                    return {"rule O cannot tell whether " + label(skipped.shell) + " is a cavity", false};
                }
                return {label(skipped.shell) + " cannot be merged: " + skipped.reason, false};
            }
        }
        for (const int s : {a, b}) {
            const ShellRole r = role(s);
            if (r == ShellRole::Cavity || (r == ShellRole::Unclassified && relations_.shells[static_cast<std::size_t>(s)].signedVolume < 0.0)) {
                return {label(s) + " is wound inward (a cavity, or rule O cannot tell)", false};
            }
            if (r == ShellRole::InsideOut) {
                return {label(s) + " is inside out (see Inverted shells)", false};
            }
        }
        const bool grouped = std::any_of(plan.groups.begin(), plan.groups.end(), [&](const MergePlanGroup& g) {
            return contains(g.shells, a) && contains(g.shells, b);
        });
        if (grouped && !mergeOn_) {
            return {"the merge is off", true};
        }
        if (grouped) {
            return {lastMergeRefusal_.empty() ? std::string("the merge did not unite them")
                                              : "the merge did not unite them (" + asClause(lastMergeRefusal_) + ")",
                    false};
        }
        return {mergeOn_ ? std::string("the merge would not unite them") : std::string("the merge is off"), false};
    }

private:
    const FixAllResult& result_;
    const FixAllParams& params_;
    bool faceScope_ = false;
    bool mergeOn_ = false;
    std::string lastMergeRefusal_;
    std::optional<MergePlan> plan_;
    std::vector<std::vector<Eigen::Index>> shells_;
    ShellRelations relations_;
    ShellRoles roles_;
    std::vector<std::uint8_t> wholeIn_;
    std::vector<std::uint8_t> anyIn_;
};

/// One clause per defect still in the counts ("4 open edges"), for the "Left after" line.
[[nodiscard]] std::vector<std::string> defectClauses(const FixAllCounts& c, std::size_t within, std::size_t between) {
    std::vector<std::string> parts;
    const auto add = [&](std::size_t n, const char* one, const char* many) {
        if (n > 0) {
            parts.push_back(counted(n, one, many));
        }
    };
    add(c.openEdges, "open edge", "open edges");
    add(c.nonManifoldEdges, "non-manifold edge", "non-manifold edges");
    add(c.nonManifoldVertices, "non-manifold vertex", "non-manifold vertices");
    add(c.flippedFaces, "flipped face", "flipped faces");
    add(c.invertedShells, "inside-out shell", "inside-out shells");
    if (c.intersectingFaces > 0) {
        parts.push_back(counted(c.intersectingFaces, "crossing face", "crossing faces") + " (" + std::to_string(within) +
                        " within shells, " + std::to_string(between) + " between shells)");
    }
    add(c.degenerateFaces, "degenerate face", "degenerate faces");
    add(c.duplicateFaces, "duplicate face", "duplicate faces");
    add(c.tinyShells, "tiny floating shell", "tiny floating shells");
    add(c.nestedSolids, "nested solid to delete", "nested solids to delete");
    add(c.thinFaces, "thin face", "thin faces");
    return parts;
}

/// "This pass repaired 223 open edges; it left more crossing faces (0 -> 6). ..." for One pass.
[[nodiscard]] std::string temporarilyWorseText(const FixAllCounts& before, const FixAllCounts& after) {
    struct Item {
        std::size_t FixAllCounts::*field;
        const char* name;
    };
    static constexpr Item kItems[] = {
        {&FixAllCounts::openEdges, "open edges"},         {&FixAllCounts::nonManifoldEdges, "non-manifold edges"},
        {&FixAllCounts::nonManifoldVertices, "non-manifold vertices"}, {&FixAllCounts::flippedFaces, "flipped faces"},
        {&FixAllCounts::invertedShells, "inside-out shells"}, {&FixAllCounts::intersectingFaces, "crossing faces"},
        {&FixAllCounts::degenerateFaces, "degenerate faces"}, {&FixAllCounts::duplicateFaces, "duplicate faces"},
        {&FixAllCounts::tinyShells, "tiny floating shells"}, {&FixAllCounts::nestedSolids, "nested solids"},
        {&FixAllCounts::thinFaces, "thin faces"},
    };
    std::vector<std::string> better;
    std::vector<std::string> worse;
    for (const Item& item : kItems) {
        const std::size_t b = before.*item.field;
        const std::size_t a = after.*item.field;
        if (a < b) {
            better.push_back(std::to_string(b - a) + " " + item.name);
        } else if (a > b) {
            worse.push_back(std::string(item.name) + " (" + std::to_string(b) + " -> " + std::to_string(a) + ")");
        }
    }
    std::string text = better.empty() ? "This pass left more " + joinAnd(worse)
                                      : "This pass repaired " + joinAnd(better) + "; it left more " + joinAnd(worse);
    return text + ". The next pass should repair them: run one more pass.";
}

} // namespace

const char* fixAllSuggestionLabel(FixAllSuggestion suggestion) noexcept {
    switch (suggestion) {
    case FixAllSuggestion::None:
        return "none";
    case FixAllSuggestion::AlphaWrap:
        return "alpha wrap";
    case FixAllSuggestion::TickMerge:
        return "tick Merge shells";
    case FixAllSuggestion::ReportOnly:
        return "report";
    }
    return "";
}

bool fixall_detail::buildFinalReport(FixAllResult& result, FinalReportInput input, std::stop_token stopToken) {
    const auto start = std::chrono::steady_clock::now();
    FixAllFinalReport& report = result.report;
    report = FixAllFinalReport{};
    const Mesh& mesh = result.mesh;
    const FixAllScope& scope = result.scope;
    const auto nf = static_cast<std::size_t>(mesh.numFaces());

    // Step 18: every Mesh Health row of the returned mesh (the pass-end check's nine, plus Shells)
    CategoryResult shellsRow = runCategoryCheck(DefectCategory::Shells, mesh, input.params.checkParams, stopToken);
    if (shellsRow.cancelled) {
        return false;
    }
    input.health.push_back(std::move(shellsRow));
    for (const DefectCategory category : kAllDefectCategories) {
        const auto it = std::find_if(input.health.begin(), input.health.end(),
                                     [&](const CategoryResult& row) { return row.category == category; });
        if (it != input.health.end()) {
            report.health.push_back(std::move(*it));
        }
    }
    const auto row = [&](DefectCategory category) -> const CategoryResult* {
        for (const CategoryResult& r : report.health) {
            if (r.category == category) {
                return &r;
            }
        }
        return nullptr;
    };

    ShellFactsOfResult facts(result, input.params, input.returnedSteps);
    if (!facts.compute(stopToken)) {
        return false;
    }
    const auto& shells = facts.shells();
    const ShellRelations& relations = facts.relations();
    const std::vector<std::uint8_t>& anyIn = facts.anyInScope();
    const std::vector<int> shellOf = shellOfFaces(mesh.numFaces(), shells);
    report.shells = shells;
    report.shellInScope = anyIn;
    const std::size_t n = shells.size();

    // Per shell: open and non-manifold edges, crossings within and between shells, flipped and thin faces
    std::vector<FixAllShellResidue> perShell(n);
    for (std::size_t s = 0; s < n; ++s) {
        perShell[s].shell = static_cast<int>(s);
        perShell[s].faces = shells[s].size();
        perShell[s].volume = s < relations.shells.size() ? relations.shells[s].signedVolume : 0.0;
    }
    const EdgeTopology topo = buildEdgeTopology(mesh);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        const std::uint32_t count = topo.faceCount(e);
        if (count == 2) {
            continue;
        }
        std::vector<int> owners;
        for (std::uint32_t i = topo.faceOffsets[e]; i < topo.faceOffsets[e + 1]; ++i) {
            const int s = shellOf[topo.incidentFaces[i]];
            if (s >= 0 && !contains(owners, s)) {
                owners.push_back(s);
            }
        }
        for (const int s : owners) {
            (count == 1 ? perShell[static_cast<std::size_t>(s)].openEdges : perShell[static_cast<std::size_t>(s)].nonManifoldEdges) += 1;
        }
    }
    // Fill faces crossing another shell: (fill record, other shell) -> face pairs; the shell each fill is in
    std::map<std::pair<int, int>, std::size_t> fillPairs;
    std::vector<int> fillShell(input.fills.size(), -1);
    for (std::size_t f = 0; f < input.fillOf.size() && f < nf; ++f) {
        const int fill = input.fillOf[f];
        if (fill >= 0 && static_cast<std::size_t>(fill) < fillShell.size() && fillShell[static_cast<std::size_t>(fill)] < 0) {
            fillShell[static_cast<std::size_t>(fill)] = shellOf[f];
        }
    }
    if (const CategoryResult* crossings = row(DefectCategory::Intersections)) {
        for (std::size_t e = 0; e < crossings->size(); ++e) {
            const Eigen::Index f = crossings->elements[e].index;
            if (f < 0 || static_cast<std::size_t>(f) >= nf) {
                continue;
            }
            const int s = shellOf[static_cast<std::size_t>(f)];
            bool within = false;
            bool between = false;
            for (const Eigen::Index g : crossings->relatedOf(e)) {
                const int o = g >= 0 && static_cast<std::size_t>(g) < nf ? shellOf[static_cast<std::size_t>(g)] : -1;
                within = within || o == s;
                between = between || o != s;
                const int fill = static_cast<std::size_t>(f) < input.fillOf.size() ? input.fillOf[static_cast<std::size_t>(f)] : -1;
                if (o != s && o >= 0 && fill >= 0) {
                    ++fillPairs[{fill, o}];
                }
            }
            if (s >= 0 && within) {
                ++perShell[static_cast<std::size_t>(s)].crossingFacesWithin;
            }
            if (scope.inScope(f)) {
                report.crossingFacesWithin += within ? 1 : 0;
                report.crossingFacesBetween += between ? 1 : 0;
            }
        }
    }
    const auto countFaces = [&](DefectCategory category, std::size_t FixAllShellResidue::*field) {
        if (const CategoryResult* r = row(category)) {
            for (std::size_t e = 0; e < r->size(); ++e) {
                const Eigen::Index f = r->elements[e].index;
                if (f >= 0 && static_cast<std::size_t>(f) < nf && shellOf[static_cast<std::size_t>(f)] >= 0) {
                    perShell[static_cast<std::size_t>(shellOf[static_cast<std::size_t>(f)])].*field += 1;
                }
            }
        }
    };
    countFaces(DefectCategory::FlippedNormals, &FixAllShellResidue::flippedFaces);
    countFaces(DefectCategory::ThinFaces, &FixAllShellResidue::thinFaces);
    for (const ShellPairRelation& pair : relations.pairs) {
        if (pair.crosses() || pair.sharesFaceArea()) {
            perShell[static_cast<std::size_t>(pair.a)].crosses.push_back(pair.b);
            perShell[static_cast<std::size_t>(pair.b)].crosses.push_back(pair.a);
        } else if (pair.edgeOrPointContactOnly()) {
            perShell[static_cast<std::size_t>(pair.a)].touches.push_back(pair.b);
            perShell[static_cast<std::size_t>(pair.b)].touches.push_back(pair.a);
        }
    }
    if (stopToken.stop_requested()) {
        return false;
    }

    // Shell counts, watertight shells, cavities kept
    const FixAllScope& inputScope = input.inputScope;
    for (const std::vector<Eigen::Index>& shell : shellsLargestFirst(input.input)) {
        report.shellsBefore += std::any_of(shell.begin(), shell.end(), [&](Eigen::Index f) { return inputScope.inScope(f); }) ? 1 : 0;
    }
    std::size_t cavitiesAfter = 0;
    for (std::size_t s = 0; s < n; ++s) {
        const int c = s < relations.shells.size() ? relations.shells[s].nestedIn : -1;
        if (facts.role(static_cast<int>(s)) == ShellRole::Cavity &&
            (anyIn[s] != 0 || (c >= 0 && anyIn[static_cast<std::size_t>(c)] != 0))) {
            report.cavitiesKept.push_back(static_cast<int>(s));
        }
        if (anyIn[s] == 0) {
            continue;
        }
        ++report.shellsAfter;
        report.closedShells += perShell[s].openEdges == 0 && perShell[s].nonManifoldEdges == 0 ? 1 : 0;
        cavitiesAfter += facts.role(static_cast<int>(s)) == ShellRole::Cavity ? 1 : 0;
    }

    // Groups not merged: refused by a cavity (Q20) or stopped by rule O (Q23)
    const MergePlan* plan = facts.plan();
    if (plan != nullptr) {
        for (const MergePlanRefusedGroup& group : plan->refusedGroups) {
            std::vector<int> named = group.shells;
            named.insert(named.end(), group.hollows.begin(), group.hollows.end());
            report.groupsNotMerged.push_back({group.reason, std::move(named), true});
        }
    }
    for (const std::vector<int>& group : facts.roles().stoppedGroups) {
        if (group.size() < 2 || std::none_of(group.begin(), group.end(), [&](int s) { return anyIn[static_cast<std::size_t>(s)] != 0; })) {
            continue;
        }
        const std::vector<int> inner(group.begin() + 1, group.end());
        report.groupsNotMerged.push_back({label(group.front()) + " and its inner " + (inner.size() == 1 ? "shell " : "shells ") +
                                              shellList(inner) +
                                              " look inside out together: left as they are. Flip them by hand (Flip all or "
                                              "Flip outer only, under Inverted shells).",
                                          group, true});
    }

    // Step 19: what is left in each shell in scope, and the suggestion
    for (std::size_t s = 0; s < n; ++s) {
        if (anyIn[s] == 0) {
            continue;
        }
        FixAllShellResidue r = std::move(perShell[s]);
        const int si = static_cast<int>(s);
        const std::string name = label(si);
        r.named = {si};
        const ShellRole role = facts.role(si);
        if (r.unresolved()) {
            std::vector<std::string> left;
            if (r.openEdges > 0) {
                left.push_back(counted(r.openEdges, "open edge", "open edges"));
            }
            if (r.nonManifoldEdges > 0) {
                left.push_back(counted(r.nonManifoldEdges, "non-manifold edge", "non-manifold edges"));
            }
            if (r.crossingFacesWithin > 0) {
                left.push_back(counted(r.crossingFacesWithin, "face crossing its own shell", "faces crossing their own shell"));
            }
            r.suggestion = FixAllSuggestion::AlphaWrap;
            r.reason = name + " still has " + joinAnd(left) + ".";
            if (!r.crosses.empty()) {
                r.reason += " It still crosses " + shellList(r.crosses) + ".";
                r.named.insert(r.named.end(), r.crosses.begin(), r.crosses.end());
            }
            r.reason += " Suggested: alpha wrap it.";
        } else if (const std::vector<int>* group = facts.stoppedGroupOf(si)) {
            r.suggestion = FixAllSuggestion::ReportOnly;
            r.reason = name + " looks inside out together with " +
                       shellList(std::vector<int>(group->begin() + (group->front() == si ? 1 : 0), group->end())) +
                       ": left as it is. Flip them by hand (see Inverted shells).";
            r.named = *group;
        } else if (role == ShellRole::Unclassified) {
            r.suggestion = FixAllSuggestion::ReportOnly;
            r.reason = name + " " + formatShellRoleReason(facts.roles().shells[s], label);
        } else if (role == ShellRole::InsideOut) {
            r.suggestion = FixAllSuggestion::ReportOnly;
            r.reason = name + " is still inside out (see Inverted shells): Fix All could not reverse it.";
        } else if (r.flippedFaces > 0) {
            r.suggestion = FixAllSuggestion::ReportOnly;
            r.reason = name + " still has " + counted(r.flippedFaces, "face", "faces") +
                       " wound against their neighbours (a twist, like a Mobius strip): left as it is.";
        } else if (!r.crosses.empty()) {
            // Group the crossed shells by why they were not merged
            std::vector<std::pair<std::string, std::vector<int>>> byWhy;
            bool tick = false;
            for (const int o : r.crosses) {
                auto [why, ticking] = facts.whyNotMerged(si, o);
                tick = tick || ticking;
                auto it = std::find_if(byWhy.begin(), byWhy.end(), [&](const auto& entry) { return entry.first == why; });
                if (it == byWhy.end()) {
                    byWhy.push_back({std::move(why), {o}});
                } else {
                    it->second.push_back(o);
                }
            }
            std::vector<std::string> clauses;
            for (const auto& [why, others] : byWhy) {
                clauses.push_back(shellList(others) + ": " + why);
            }
            r.suggestion = tick ? FixAllSuggestion::TickMerge : FixAllSuggestion::ReportOnly;
            r.reason = name + " still crosses " + joinAnd(clauses) + "." +
                       (tick ? " Tick Merge shells and preview again." : "");
            r.named.insert(r.named.end(), r.crosses.begin(), r.crosses.end());
        } else if (!r.touches.empty()) {
            r.suggestion = FixAllSuggestion::ReportOnly;
            r.reason = name + " touches " + shellList(r.touches) +
                       " along an edge or at a point only and cannot be one solid with it: left as it is.";
            r.named.insert(r.named.end(), r.touches.begin(), r.touches.end());
        } else if (r.thinFaces > 0) {
            r.suggestion = FixAllSuggestion::None;
            r.reason = name + ": " + counted(r.thinFaces, "thin face", "thin faces") +
                       " left. The healer found no flip or collapse that keeps the surface in place (for example on a "
                       "sharp crease).";
        } else {
            continue;
        }
        report.residue.push_back(std::move(r));
    }

    // Every crossing left, by shell pair, with the fills that made it (Q27)
    const bool manyPasses = result.passesRun > 1;
    for (const ShellPairRelation& pair : relations.pairs) {
        const std::size_t facePairs = pair.crossings + pair.sharedAreaPairs;
        if (facePairs == 0 || (anyIn[static_cast<std::size_t>(pair.a)] == 0 && anyIn[static_cast<std::size_t>(pair.b)] == 0)) {
            continue;
        }
        FixAllCrossingLeft left;
        left.a = pair.a;
        left.b = pair.b;
        left.facePairs = facePairs;
        left.whyNotMerged = facts.whyNotMerged(pair.a, pair.b).first;
        std::vector<std::string> fillTexts;
        for (const auto& [key, count] : fillPairs) {
            const auto [fill, other] = key;
            const FillRecord& record = input.fills[static_cast<std::size_t>(fill)];
            // The fill's shell is the other one of the pair
            const int holeShell = fillShell[static_cast<std::size_t>(fill)];
            if (!((holeShell == pair.a && other == pair.b) || (holeShell == pair.b && other == pair.a))) {
                continue;
            }
            left.fills.push_back({record.pass, record.hole, holeShell, count});
            fillTexts.push_back(label(holeShell) + "'s fill of hole " + std::to_string(record.hole) +
                                (manyPasses ? " (pass " + std::to_string(record.pass) + ")" : std::string()) + " crosses " +
                                label(other) + " in " + counted(count, "place", "places"));
        }
        if (fillTexts.empty()) {
            left.text = label(pair.a) + " and " + label(pair.b) + " still cross (" +
                        counted(facePairs, "crossing face pair", "crossing face pairs") + "): " + left.whyNotMerged + ".";
        } else {
            left.text = joinAnd(fillTexts) + ": " + left.whyNotMerged + ". The hole stays filled.";
        }
        report.crossingsLeft.push_back(std::move(left));
    }
    if (stopToken.stop_requested()) {
        return false;
    }

    // Deletions and merges of the steps that led to the returned mesh
    std::size_t merged = 0;
    std::size_t unions = 0;
    std::size_t tiny = 0;
    std::size_t nested = 0;
    for (std::size_t i = 0; i < input.returnedSteps && i < result.steps.size(); ++i) {
        const FixAllStepReport& step = result.steps[i];
        if (!step.accepted()) {
            continue;
        }
        if (step.stage == FixAllStageId::MergeShells) {
            merged += step.countBefore;
            unions += step.countAfter;
        } else if (step.stage == FixAllStageId::TinyShells || step.stage == FixAllStageId::Fins ||
                   step.stage == FixAllStageId::DeleteNestedSolids) {
            report.deletions.push_back({step.stage, step.pass, step.detail, step.removedInputFaces});
            const std::size_t gone = step.countBefore - std::min(step.countBefore, step.countAfter);
            tiny += step.stage == FixAllStageId::TinyShells ? gone : 0;
            nested += step.stage == FixAllStageId::DeleteNestedSolids ? gone : 0;
        }
    }

    // Summary lines
    const int asked = std::clamp(input.params.passes, settings::kFixAllMinPasses, settings::kFixAllMaxPasses);
    const std::string passWord = counted(static_cast<std::size_t>(result.passesRun), "pass", "passes");
    if (result.onePass) {
        report.passesLine = "Passes: 1 (One pass)";
    } else {
        report.passesLine = "Passes: " + std::to_string(result.passesRun) + " of " + std::to_string(asked) + " (stopped: " +
                            fixAllStopReasonLabel(result.stopReason) + ")";
        // Said only when a later pass ended worse than the state returned (it was discarded)
        const bool discarded = !result.passes.empty() && result.priorityAfter < result.passes.back().after;
        if (discarded && result.bestPass == 0) {
            report.passesLine += "; no pass was better than the input, so it is returned unchanged";
        } else if (discarded) {
            report.passesLine += "; pass " + std::to_string(result.bestPass) + " was the best and is returned";
        }
    }
    std::vector<int> leftIn;
    for (const FixAllShellResidue& r : report.residue) {
        if (r.suggestion != FixAllSuggestion::None) {
            leftIn.push_back(r.shell);
        }
    }
    if (result.priorityAfter.clean()) {
        report.leftLine = "Clean after " + passWord + ".";
        if (result.countsAfter.thinFaces > 0) {
            report.leftLine += " " + counted(result.countsAfter.thinFaces, "thin face", "thin faces") + " left.";
        }
    } else {
        report.leftLine = "Left after " + passWord + ": " +
                          joinAnd(defectClauses(result.countsAfter, report.crossingFacesWithin, report.crossingFacesBetween));
        if (!leftIn.empty()) {
            if (leftIn.size() > settings::kFixAllReportShellsNamed) {
                std::string first;
                for (std::size_t i = 0; i < settings::kFixAllReportShellsNamed; ++i) {
                    first += label(leftIn[i]) + ", ";
                }
                report.leftLine += "; in " + first + "and " + std::to_string(leftIn.size() - settings::kFixAllReportShellsNamed) + " more";
            } else {
                report.leftLine += "; in " + shellList(leftIn);
            }
        }
        report.leftLine += ".";
    }
    if (result.onePass && result.temporarilyWorse) {
        report.temporarilyWorseText = temporarilyWorseText(result.countsBefore, result.countsAfter);
    }

    std::vector<std::string> shellParts;
    if (merged > 0) {
        shellParts.push_back("merged " + std::to_string(merged) + " into " + std::to_string(unions));
    }
    if (tiny > 0) {
        shellParts.push_back("deleted " + counted(tiny, "tiny floating shell", "tiny floating shells"));
    }
    if (nested > 0) {
        shellParts.push_back("deleted " + counted(nested, "nested solid", "nested solids"));
    }
    if (!report.cavitiesKept.empty()) {
        shellParts.push_back(counted(report.cavitiesKept.size(), "cavity", "cavities") + " kept");
    }
    if (!report.groupsNotMerged.empty()) {
        shellParts.push_back(counted(report.groupsNotMerged.size(), "group", "groups") + " not merged");
    }
    if (plan != nullptr && !plan->skipped.empty()) {
        std::vector<int> skipped;
        for (const MergePlanSkipped& sk : plan->skipped) {
            skipped.push_back(sk.shell);
        }
        shellParts.push_back("skipped " + shellList(skipped));
    }
    report.shellsLine = "Shells: " + std::to_string(report.shellsBefore) + " -> " + std::to_string(report.shellsAfter);
    if (!shellParts.empty()) {
        std::string joined;
        for (const std::string& part : shellParts) {
            joined += (joined.empty() ? "" : "; ") + part;
        }
        report.shellsLine += " (" + joined + ")";
    }

    if (!report.cavitiesKept.empty()) {
        std::unordered_set<FaceGeometryKey, FaceGeometryKeyHash> inputKeys;
        inputKeys.reserve(static_cast<std::size_t>(input.input.numFaces()));
        for (Eigen::Index f = 0; f < input.input.numFaces(); ++f) {
            inputKeys.insert(faceGeometryKey(input.input, f));
        }
        std::vector<std::string> parts;
        for (const int c : report.cavitiesKept) {
            const auto cs = static_cast<std::size_t>(c);
            const bool unchanged = std::all_of(shells[cs].begin(), shells[cs].end(),
                                               [&](Eigen::Index f) { return inputKeys.count(faceGeometryKey(mesh, f)) > 0; });
            const int container = relations.shells[cs].nestedIn;
            parts.push_back(label(c) + (container >= 0 ? " inside " + label(container) : std::string()) + ", " +
                            formatVolume(relations.shells[cs].signedVolume, mesh.fileUnit) +
                            (unchanged ? ", unchanged" : ", changed"));
        }
        std::string joined;
        for (const std::string& part : parts) {
            joined += (joined.empty() ? "" : "; ") + part;
        }
        report.cavitiesLine = "Cavities kept: " + std::to_string(report.cavitiesKept.size()) + " (" + joined + ")";
    }
    report.closedLine = "Watertight: " + std::to_string(report.closedShells) + " of " +
                        counted(report.shellsAfter, "shell", "shells");
    if (cavitiesAfter > 0) {
        report.closedLine += " (" + counted(report.shellsAfter - cavitiesAfter, "solid", "solids") + " + " +
                             counted(cavitiesAfter, "cavity", "cavities") + ")";
    }

    // The whole report in reading order
    std::vector<FixAllReportLine>& lines = report.lines;
    lines.push_back({report.passesLine, {}, false});
    lines.push_back({report.leftLine, leftIn, !result.priorityAfter.clean()});
    if (!report.temporarilyWorseText.empty()) {
        lines.push_back({report.temporarilyWorseText, {}, true});
    }
    lines.push_back({report.shellsLine, {}, false});
    if (!report.cavitiesLine.empty()) {
        lines.push_back({report.cavitiesLine, report.cavitiesKept, false});
    }
    lines.push_back({report.closedLine, {}, report.closedShells < report.shellsAfter});
    for (const FixAllReportLine& line : report.groupsNotMerged) {
        lines.push_back(line);
    }
    for (const FixAllShellResidue& r : report.residue) {
        lines.push_back({r.reason, r.named, r.suggestion != FixAllSuggestion::None});
    }
    for (const FixAllCrossingLeft& left : report.crossingsLeft) {
        lines.push_back({left.text, {left.a, left.b}, true});
    }
    for (const FixAllDeletion& deletion : report.deletions) {
        lines.push_back({deletion.text + (manyPasses ? " (pass " + std::to_string(deletion.pass) + ")" : std::string()), {}, false});
    }
    report.complete = true;
    report.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return true;
}

} // namespace ogun
