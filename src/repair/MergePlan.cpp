// Fix All phase 2, the plan: roles (rule O), links, merge groups, refused groups, nested solids to
// delete, skipped shells and what is only reported. The merge itself (mergeShells) runs elsewhere.

#include "repair/MergePlan.hpp"

#include "repair/FixAllDetail.hpp"
#include "repair/ShellMerge.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace mesheal {

namespace {

/// Union-find over shell indices.
class ShellGroups {
public:
    explicit ShellGroups(std::size_t n) : parent_(n) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    [[nodiscard]] int root(int s) {
        while (parent_[static_cast<std::size_t>(s)] != s) {
            parent_[static_cast<std::size_t>(s)] = parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(s)])];
            s = parent_[static_cast<std::size_t>(s)];
        }
        return s;
    }

    void link(int a, int b) {
        a = root(a);
        b = root(b);
        if (a != b) {
            parent_[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
        }
    }

private:
    std::vector<int> parent_;
};

[[nodiscard]] std::string shellLabel(int s) {
    return "#" + std::to_string(s + 1);
}

[[nodiscard]] bool overlaps(const ShellPairRelation& pair) noexcept {
    return pair.crosses() || pair.sharesFaceArea();
}

[[nodiscard]] bool meetsAtAll(const ShellPairRelation& pair) noexcept {
    return pair.crossings > 0 || pair.sharedAreaPairs > 0 || pair.contacts > 0;
}

void addUnique(std::vector<int>& list, int value) {
    if (std::find(list.begin(), list.end(), value) == list.end()) {
        list.push_back(value);
    }
}

[[nodiscard]] std::string skipReasonText(MergeSkipReason why, std::size_t count, int container) {
    switch (why) {
    case MergeSkipReason::Open:
        return "it is still open (" + fixall_detail::counted(count, "open edge", "open edges") + ")";
    case MergeSkipReason::FinEdges:
        return "it has " + fixall_detail::counted(count, "Fin / other edge", "Fin / other edges") +
               " (three or more faces meet there)";
    case MergeSkipReason::Flipped:
        return "its faces are not wound consistently (flipped faces)";
    case MergeSkipReason::InsideOut:
        return "it is inside out (see Inverted shells)";
    case MergeSkipReason::NotManifold:
        return "Manifold cannot read it as a solid";
    case MergeSkipReason::NoVolume:
        return "it has no volume";
    case MergeSkipReason::ContainerNotMergeable:
        return "it lies inside " + shellLabel(container) + ", which cannot be merged";
    case MergeSkipReason::Unclassified:
        break;
    }
    return {};
}

/// "#6 and #9 reach into cavity #28 of #12, so their group of 27 shells is not merged: ..."
[[nodiscard]] std::string refusedGroupText(const MergePlan& plan, const MergePlanRefusedGroup& group) {
    std::string text;
    std::vector<int> hollows;
    for (const MergePlanReach& reach : group.reaches) {
        addUnique(hollows, reach.hollow);
    }
    for (std::size_t i = 0; i < hollows.size(); ++i) {
        const int h = hollows[i];
        std::vector<int> reachers;
        for (const MergePlanReach& reach : group.reaches) {
            if (reach.hollow == h) {
                addUnique(reachers, reach.shell);
            }
        }
        std::sort(reachers.begin(), reachers.end());
        text += (i == 0 ? "" : "; ") + fixall_detail::shellList(reachers) +
                (reachers.size() > 1 ? " reach into " : " reaches into ");
        const int container = plan.relations.shells[static_cast<std::size_t>(h)].nestedIn;
        if (container >= 0) { // a cavity, or an inward shell inside another that rule O cannot judge
            text += "cavity " + shellLabel(h) + " of " + shellLabel(container);
        } else {
            text += shellLabel(h) + ", which is wound inward (a cavity or inside out?)";
        }
    }
    text += group.shells.size() == 1 ? ", so it is not merged"
                                     : ", so their group of " + std::to_string(group.shells.size()) + " shells is not merged";
    text += ": Fix All cannot tell whether the hollow should cut them or they should fill it. Merge them by hand.";
    return text;
}

void buildLines(MergePlan& plan) {
    using enum MergePlanLineKind;
    std::vector<MergePlanLine>& lines = plan.lines;
    if (!plan.groups.empty()) {
        std::vector<int> operands;
        std::size_t passThrough = 0;
        for (const MergePlanGroup& g : plan.groups) {
            operands.insert(operands.end(), g.shells.begin(), g.shells.end());
            passThrough += g.cavities.size();
        }
        std::sort(operands.begin(), operands.end());
        std::string text = "Merge " + fixall_detail::counted(operands.size(), "shell", "shells") + " that overlap into " +
                           fixall_detail::counted(plan.groups.size(), "solid", "solids");
        if (passThrough > 0) {
            text += " (" + fixall_detail::counted(passThrough, "cavity", "cavities") + " kept)";
        }
        lines.push_back({Info, text + ".", operands});
    } else if (!plan.tickedByDefault) {
        lines.push_back({Info, "No shells overlap: nothing to merge.", {}});
    } else if (plan.isolatedNestedSolids.empty()) {
        lines.push_back({Info, "Nothing can be merged: every overlap is refused or skipped (below).", {}});
    }
    for (const MergePlanNestedSolid& n : plan.isolatedNestedSolids) {
        lines.push_back({Info,
                         "Delete nested solid " + shellLabel(n.shell) + " (wholly inside " + shellLabel(n.container) +
                             ", touching nothing).",
                         {n.shell, n.container}});
    }
    if (!plan.cavitiesKept.empty()) {
        std::vector<int> cavities;
        for (const MergePlanCavity& c : plan.cavitiesKept) {
            cavities.push_back(c.shell);
        }
        lines.push_back({Info,
                         fixall_detail::counted(cavities.size(), "cavity", "cavities") + " kept: " +
                             fixall_detail::shellList(cavities) + ".",
                         cavities});
    }
    for (const MergePlanRefusedGroup& g : plan.refusedGroups) {
        std::vector<int> shells = g.shells;
        shells.insert(shells.end(), g.hollows.begin(), g.hollows.end());
        lines.push_back({Warning, g.reason, std::move(shells)});
    }
    for (const std::vector<int>& g : plan.stoppedGroups) {
        if (g.size() < 2) {
            continue;
        }
        const std::span<const int> inner(g.data() + 1, g.size() - 1);
        lines.push_back({Warning,
                         shellLabel(g.front()) + " and its inner " + (inner.size() == 1 ? "shell " : "shells ") +
                             fixall_detail::shellList(inner) +
                             " look inside out together: not merged. Flip them by hand (see Inverted shells).",
                         g});
    }
    for (const MergePlanSkipped& s : plan.skipped) {
        std::vector<int> shells{s.shell};
        shells.insert(shells.end(), s.overlaps.begin(), s.overlaps.end());
        if (s.why == MergeSkipReason::Unclassified) {
            lines.push_back({Warning, shellLabel(s.shell) + " " + s.reason, std::move(shells)});
            continue;
        }
        lines.push_back({Warning,
                         shellLabel(s.shell) + " cannot be merged yet: " + s.reason + "; it overlaps " +
                             fixall_detail::shellList(s.overlaps) + ".",
                         std::move(shells)});
    }
    for (const MergePlanOutsideLink& link : plan.outsideScopeLinks) {
        std::string text;
        switch (link.kind) {
        case MergeOutsideLinkKind::Crosses:
            text = shellLabel(link.shell) + " still crosses " + shellLabel(link.outside) +
                   ", which is not selected: select both to merge them.";
            break;
        case MergeOutsideLinkKind::InsideOf:
            text = shellLabel(link.shell) + " lies inside " + shellLabel(link.outside) +
                   ", which is not selected: not deleted (select " + shellLabel(link.outside) + " too).";
            break;
        case MergeOutsideLinkKind::Holds:
            text = shellLabel(link.shell) + " holds " + shellLabel(link.outside) +
                   ", which is not selected: left as it is.";
            break;
        }
        lines.push_back({Warning, std::move(text), {link.shell, link.outside}});
    }
    for (const MergePlanContact& c : plan.edgeOrPointContacts) {
        lines.push_back({Info,
                         shellLabel(c.a) + " and " + shellLabel(c.b) +
                             " touch along an edge or at a point only and cannot be one solid: left as they are.",
                         {c.a, c.b}});
    }
}

} // namespace

std::size_t MergePlan::operandCount() const noexcept {
    std::size_t n = 0;
    for (const MergePlanGroup& g : groups) {
        n += g.shells.size();
    }
    return n;
}

bool MergePlan::hasWork() const noexcept {
    return !groups.empty() || !isolatedNestedSolids.empty();
}

std::vector<std::uint8_t> shellsInScope(const FixAllScope& scope, const std::vector<std::vector<Eigen::Index>>& shells) {
    std::vector<std::uint8_t> in(shells.size(), 0);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        in[s] = !shells[s].empty() &&
                        std::all_of(shells[s].begin(), shells[s].end(), [&](Eigen::Index f) { return scope.inScope(f); })
                    ? 1
                    : 0;
    }
    return in;
}

bool mergeTickedByDefault(const FixAllScope& scope, const ShellRelations& relations, const ShellRoles& roles,
                          const std::vector<std::uint8_t>& shellInScope) {
    if (scope.kind == FixAllScopeKind::Faces || !relations.complete) {
        return false;
    }
    const auto in = [&](int s) { return s >= 0 && static_cast<std::size_t>(s) < shellInScope.size() && shellInScope[static_cast<std::size_t>(s)] != 0; };
    for (const ShellPairRelation& pair : relations.pairs) {
        if (in(pair.a) && in(pair.b) && overlaps(pair)) {
            return true;
        }
    }
    for (std::size_t s = 0; s < roles.shells.size() && s < relations.shells.size(); ++s) {
        if (roles.shells[s].role == ShellRole::NestedSolid && in(static_cast<int>(s)) &&
            in(relations.shells[s].nestedIn)) {
            return true;
        }
    }
    return false;
}

std::optional<bool> mergeTickedByDefault(const Mesh& mesh, const FixAllScope& scope, const MergePlanParams& params,
                                         std::stop_token stopToken) {
    if (scope.kind == FixAllScopeKind::Faces) {
        return false;
    }
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    ShellRelationParams relationParams;
    relationParams.intersectionTolerance = params.intersectionTolerance;
    const ShellRelations relations = computeShellRelations(mesh, shells, relationParams, stopToken);
    if (!relations.complete || stopToken.stop_requested()) {
        return std::nullopt;
    }
    return mergeTickedByDefault(scope, relations, classifyShellRoles(mesh, relations), shellsInScope(scope, shells));
}

MergePlan planMerge(const Mesh& mesh, const FixAllScope& scope, const MergePlanParams& params, std::stop_token stopToken) {
    const auto start = std::chrono::steady_clock::now();
    MergePlan plan;
    const auto finish = [&]() -> MergePlan {
        plan.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return std::move(plan);
    };
    if (scope.kind == FixAllScopeKind::Faces) {
        plan.needsWholeShells = true;
        plan.lines.push_back({MergePlanLineKind::Info, "Merge needs whole shells: select shells, or clear the selection.", {}});
        return finish();
    }

    plan.shells = shellsLargestFirst(mesh);
    plan.shellInScope = shellsInScope(scope, plan.shells);
    ShellRelationParams relationParams;
    relationParams.intersectionTolerance = params.intersectionTolerance;
    plan.relations = computeShellRelations(mesh, plan.shells, relationParams, stopToken);
    if (!plan.relations.complete || stopToken.stop_requested()) {
        plan.complete = false;
        return finish();
    }
    plan.roles = classifyShellRoles(mesh, plan.relations);
    plan.tickedByDefault = mergeTickedByDefault(scope, plan.relations, plan.roles, plan.shellInScope);

    const int n = static_cast<int>(plan.shells.size());
    const auto at = [](auto& v, int i) -> decltype(auto) { return v[static_cast<std::size_t>(i)]; };
    const auto in = [&](int s) { return s >= 0 && s < n && at(plan.shellInScope, s) != 0; };
    const auto role = [&](int s) { return at(plan.roles.shells, s).role; };
    const auto facts = [&](int s) -> const ShellFacts& { return at(plan.relations.shells, s); };

    // Hollows: cavities and inward shells rule O cannot judge. Never operands; reaching one stops a group
    std::vector<std::uint8_t> hollow(static_cast<std::size_t>(n), 0);
    std::vector<std::uint8_t> stopped(static_cast<std::size_t>(n), 0);
    for (int s = 0; s < n; ++s) {
        const ShellFacts& f = facts(s);
        at(hollow, s) = role(s) == ShellRole::Cavity ||
                        (role(s) == ShellRole::Unclassified && f.closed && f.oriented && f.signedVolume < 0.0);
    }
    for (const std::vector<int>& group : plan.roles.stoppedGroups) {
        bool anyInScope = false;
        for (const int s : group) {
            at(stopped, s) = 1;
            anyInScope = anyInScope || in(s);
        }
        if (anyInScope) {
            plan.stoppedGroups.push_back(group);
        }
    }
    std::vector<std::uint8_t> touches(static_cast<std::size_t>(n), 0); // meets any shell
    for (const ShellPairRelation& pair : plan.relations.pairs) {
        if (meetsAtAll(pair)) {
            at(touches, pair.a) = 1;
            at(touches, pair.b) = 1;
        }
    }
    std::vector<std::uint8_t> holdsShell(static_cast<std::size_t>(n), 0);
    for (int s = 0; s < n; ++s) {
        for (const int c : facts(s).containers) {
            at(holdsShell, c) = 1;
        }
    }

    // Eligibility: rule O first, then Manifold for the shells that could be operands
    const auto roleOk = [&](int s) {
        const ShellRole r = role(s);
        return in(s) && at(hollow, s) == 0 && at(stopped, s) == 0 &&
               (r == ShellRole::Solid || r == ShellRole::NestedSolid || r == ShellRole::Island);
    };
    std::vector<std::uint8_t> needCheck(static_cast<std::size_t>(n), 0);
    for (const ShellPairRelation& pair : plan.relations.pairs) {
        if (overlaps(pair) && in(pair.a) && in(pair.b)) {
            for (const int s : {pair.a, pair.b}) {
                if (roleOk(s)) {
                    at(needCheck, s) = 1;
                }
            }
        }
    }
    for (int s = 0; s < n; ++s) {
        if (roleOk(s) && role(s) == ShellRole::NestedSolid) {
            at(needCheck, s) = 1;
            if (roleOk(facts(s).nestedIn)) {
                at(needCheck, facts(s).nestedIn) = 1;
            }
        }
    }
    std::vector<std::uint8_t> eligible(static_cast<std::size_t>(n), 0);
    std::vector<std::optional<MergePlanSkipped>> skip(static_cast<std::size_t>(n));
    const auto setSkip = [&](int s, MergeSkipReason why, std::string reason) {
        at(skip, s) = MergePlanSkipped{s, why, std::move(reason), {}};
        at(eligible, s) = 0;
    };
    for (int s = 0; s < n; ++s) {
        if (stopToken.stop_requested()) {
            plan.complete = false;
            return finish();
        }
        if (at(needCheck, s) != 0) {
            const ManifoldAcceptance acceptance = manifoldAccepts(mesh, at(plan.shells, s));
            if (!acceptance.accepted) {
                setSkip(s, MergeSkipReason::NotManifold, skipReasonText(MergeSkipReason::NotManifold, 0, -1));
            } else if (!(acceptance.volume > 0.0)) {
                setSkip(s, MergeSkipReason::NoVolume, skipReasonText(MergeSkipReason::NoVolume, 0, -1));
            } else {
                at(eligible, s) = 1;
            }
        } else if (in(s) && at(hollow, s) == 0 && at(stopped, s) == 0 && !roleOk(s)) {
            if (role(s) == ShellRole::InsideOut) {
                setSkip(s, MergeSkipReason::InsideOut, skipReasonText(MergeSkipReason::InsideOut, 0, -1));
            } else if (role(s) == ShellRole::Unclassified) {
                setSkip(s, MergeSkipReason::Unclassified, formatShellRoleReason(at(plan.roles.shells, s), shellLabel));
            } else { // NotClosed: Fin / other edges, open, or wound against itself
                const fixall_detail::EdgeDefectCounts edges =
                    fixall_detail::countEdgeDefects(extractFaces(mesh, at(plan.shells, s)));
                const MergeSkipReason why = edges.nonManifold > 0 ? MergeSkipReason::FinEdges
                                            : edges.open > 0      ? MergeSkipReason::Open
                                                                  : MergeSkipReason::Flipped;
                setSkip(s, why, skipReasonText(why, edges.nonManifold > 0 ? edges.nonManifold : edges.open, -1));
            }
        }
    }

    // Nested solids: linked to their container when they meet anything, deleted when they touch nothing
    ShellGroups links(static_cast<std::size_t>(n));
    std::vector<MergePlanNestedSolid> isolated;
    for (int s = 0; s < n; ++s) {
        if (role(s) != ShellRole::NestedSolid) {
            continue;
        }
        const int c = facts(s).nestedIn;
        if (c < 0) {
            continue;
        }
        if (!in(s)) {
            if (in(c) && at(hollow, c) == 0) {
                plan.outsideScopeLinks.push_back({c, s, MergeOutsideLinkKind::Holds});
            }
            continue;
        }
        if (at(eligible, s) == 0) {
            continue;
        }
        if (!in(c)) {
            plan.outsideScopeLinks.push_back({s, c, MergeOutsideLinkKind::InsideOf});
            at(eligible, s) = 0;
            continue;
        }
        if (at(eligible, c) == 0) {
            setSkip(s, MergeSkipReason::ContainerNotMergeable, skipReasonText(MergeSkipReason::ContainerNotMergeable, 0, c));
            continue;
        }
        if (at(touches, s) != 0 || at(holdsShell, s) != 0) {
            links.link(s, c);
        } else {
            isolated.push_back({s, c});
        }
    }

    // Links: crossings and shared face areas between eligible shells in scope
    for (const ShellPairRelation& pair : plan.relations.pairs) {
        if (!overlaps(pair)) {
            continue;
        }
        const int a = pair.a;
        const int b = pair.b;
        if (in(a) && in(b)) {
            if (at(eligible, a) != 0 && at(eligible, b) != 0) {
                links.link(a, b);
            }
            if (at(skip, a) && at(hollow, b) == 0) {
                addUnique(at(skip, a)->overlaps, b);
            }
            if (at(skip, b) && at(hollow, a) == 0) {
                addUnique(at(skip, b)->overlaps, a);
            }
        } else if (in(a) != in(b)) {
            const int s = in(a) ? a : b;
            const int o = in(a) ? b : a;
            if (at(hollow, s) == 0 && at(hollow, o) == 0) {
                plan.outsideScopeLinks.push_back({s, o, MergeOutsideLinkKind::Crosses});
            }
        }
    }
    for (int s = 0; s < n; ++s) {
        const int c = facts(s).nestedIn;
        if (at(skip, s) && in(c) && at(hollow, c) == 0) {
            addUnique(at(skip, s)->overlaps, c);
        }
    }

    // Merge groups: two or more linked shells
    std::vector<int> size(static_cast<std::size_t>(n), 0);
    for (int s = 0; s < n; ++s) {
        if (at(eligible, s) != 0) {
            ++at(size, links.root(s));
        }
    }
    std::vector<std::uint8_t> grouped(static_cast<std::size_t>(n), 0);
    std::vector<int> operands;
    for (int s = 0; s < n; ++s) {
        if (at(eligible, s) != 0 && at(size, links.root(s)) >= 2) {
            at(grouped, s) = 1;
            operands.push_back(s);
        }
    }

    // Refusals: a group whose shell reaches into a hollow, joined through the hollow to its container's group
    std::vector<int> hollows;
    std::vector<int> containers;
    for (int s = 0; s < n; ++s) {
        if (at(hollow, s) != 0) {
            hollows.push_back(s);
            containers.push_back(facts(s).nestedIn);
        }
    }
    const std::vector<CavityReach> reaches = findCavityReaches(plan.relations, operands, hollows, containers);
    ShellGroups joined = links;
    for (const CavityReach& reach : reaches) {
        joined.link(reach.operand, reach.hollow);
    }
    for (std::size_t i = 0; i < hollows.size(); ++i) {
        if (containers[i] >= 0 && at(grouped, containers[i]) != 0) {
            joined.link(hollows[i], containers[i]);
        }
    }
    std::vector<int> refusedRoots;
    for (const CavityReach& reach : reaches) {
        addUnique(refusedRoots, joined.root(reach.hollow));
    }
    const auto refusedRoot = [&](int s) {
        return std::find(refusedRoots.begin(), refusedRoots.end(), joined.root(s)) != refusedRoots.end();
    };
    std::vector<int> groupRoots;
    for (const int s : operands) {
        addUnique(groupRoots, joined.root(s));
    }
    std::sort(groupRoots.begin(), groupRoots.end());
    for (const int r : groupRoots) {
        if (std::find(refusedRoots.begin(), refusedRoots.end(), r) != refusedRoots.end()) {
            MergePlanRefusedGroup group;
            for (const int s : operands) {
                if (joined.root(s) == r) {
                    group.shells.push_back(s);
                }
            }
            for (const int h : hollows) {
                if (joined.root(h) == r) {
                    group.hollows.push_back(h);
                }
            }
            for (const CavityReach& reach : reaches) {
                if (joined.root(reach.hollow) == r) {
                    group.reaches.push_back({reach.operand, reach.hollow});
                }
            }
            plan.refusedGroups.push_back(std::move(group));
            continue;
        }
        MergePlanGroup group;
        for (const int s : operands) {
            if (joined.root(s) == r) {
                group.shells.push_back(s);
            }
        }
        plan.groups.push_back(std::move(group));
    }

    // Nested solids to delete, unless their container's group is refused (then they stay with it)
    for (const MergePlanNestedSolid& nested : isolated) {
        if (at(grouped, nested.container) != 0 && refusedRoot(nested.container)) {
            for (MergePlanRefusedGroup& group : plan.refusedGroups) {
                if (joined.root(group.shells.front()) == joined.root(nested.container)) {
                    group.shells.push_back(nested.shell);
                    std::sort(group.shells.begin(), group.shells.end());
                }
            }
            continue;
        }
        plan.isolatedNestedSolids.push_back(nested);
    }
    for (MergePlanRefusedGroup& group : plan.refusedGroups) {
        group.reason = refusedGroupText(plan, group);
    }

    // Cavities kept: in scope or of a container in scope, in no refused group
    for (int s = 0; s < n; ++s) {
        if (role(s) != ShellRole::Cavity) {
            continue;
        }
        const int c = facts(s).nestedIn;
        if (!(in(s) || in(c)) || refusedRoot(s)) {
            continue;
        }
        const bool passThrough = c >= 0 && at(grouped, c) != 0 && !refusedRoot(c);
        plan.cavitiesKept.push_back({s, c, passThrough});
        if (passThrough) {
            for (MergePlanGroup& group : plan.groups) {
                if (std::find(group.shells.begin(), group.shells.end(), c) != group.shells.end()) {
                    group.cavities.push_back(s);
                }
            }
        }
    }

    // Edge or point contacts between shells that do not end up in one group
    for (const ShellPairRelation& pair : plan.relations.pairs) {
        if (!pair.edgeOrPointContactOnly() || !in(pair.a) || !in(pair.b) || at(hollow, pair.a) != 0 ||
            at(hollow, pair.b) != 0) {
            continue;
        }
        if (at(grouped, pair.a) != 0 && at(grouped, pair.b) != 0 && joined.root(pair.a) == joined.root(pair.b)) {
            continue;
        }
        plan.edgeOrPointContacts.push_back({pair.a, pair.b});
    }

    for (int s = 0; s < n; ++s) {
        if (at(skip, s) && !at(skip, s)->overlaps.empty()) {
            std::sort(at(skip, s)->overlaps.begin(), at(skip, s)->overlaps.end());
            plan.skipped.push_back(std::move(*at(skip, s)));
        }
    }
    buildLines(plan);
    return finish();
}

} // namespace mesheal
