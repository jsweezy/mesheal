#include "app/AppController.hpp"

#include "repair/RepairMetrics.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <utility>

namespace mesheal {

namespace {

[[nodiscard]] std::string formatValue(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.6g", value);
    return buffer;
}

/// Shells-panel numbering: shell k is #(k + 1).
[[nodiscard]] std::string shellLabel(std::size_t s) {
    return "#" + std::to_string(s + 1);
}

/// "#2", "#2 and #5", "#2, #5 and #7".
template <typename Range, typename Label>
[[nodiscard]] std::string joinLabels(const Range& shells, Label label) {
    std::string out;
    std::size_t i = 0;
    for (const auto s : shells) {
        out += (i == 0 ? "" : (i + 1 == shells.size() ? " and " : ", ")) + label(s);
        ++i;
    }
    return out;
}

void sortUnique(std::vector<std::size_t>& values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

} // namespace

bool MergeChecklist::canApply() const noexcept {
    return shells.size() >= 2 && operands.size() >= 2 &&
           std::all_of(checks.begin(), checks.end(), [](const MergeCheck& check) {
               return !check.blocking || (check.known && check.failing.empty());
           });
}

const AppController::MergeRoles& AppController::mergeRoles() const {
    if (mergeRolesCache_ && mergeRolesCache_->generation == meshGeneration_ &&
        mergeRolesCache_->tolerance == intersectionTolerance_) {
        return *mergeRolesCache_;
    }
    MergeRoles cache;
    cache.generation = meshGeneration_;
    cache.tolerance = intersectionTolerance_;
    std::vector<std::vector<Eigen::Index>> faceLists;
    faceLists.reserve(shells_.size());
    for (std::size_t s = 0; s < shells_.size(); ++s) {
        faceLists.push_back(shells_.shell(s).faces);
    }
    const std::vector<int> needed = shellsRuleONeeds(describeShells(mesh_, faceLists));
    if (!needed.empty()) {
        ShellRelationParams params;
        params.intersectionTolerance = intersectionTolerance_;
        params.involving = needed;
        cache.relations = computeShellRelations(mesh_, faceLists, params);
        cache.roles = classifyShellRoles(mesh_, cache.relations);
    }
    mergeRolesCache_ = std::move(cache);
    return *mergeRolesCache_;
}

void AppController::openMergeShells() {
    clearSelection();
    activeTool_ = ToolSlot::MergeShells;
}

MergeChecklist AppController::mergeChecklist() const {
    MergeChecklist list;
    list.shells = shells_.selectedShells();
    std::vector<std::uint8_t> selected(shells_.size(), 0);
    for (const std::size_t s : list.shells) {
        selected[s] = 1;
        list.faces += shells_.shell(s).faces.size();
    }
    const auto selectedShellOf = [&](Eigen::Index f) {
        const int s = shells_.shellOfFace(f);
        return s >= 0 && selected[static_cast<std::size_t>(s)] != 0 ? s : -1;
    };

    // Each check: the selected shells owning an element of `category` that `counts` accepts
    const auto fill = [&](MergeCheck& check, DefectCategory category, auto&& counts) {
        const CategoryResult* result = currentResult(category);
        check.known = result != nullptr;
        if (result == nullptr) {
            return;
        }
        for (std::size_t e = 0; e < result->size(); ++e) {
            const auto faces = result->facesOf(e);
            const int s = faces.empty() ? -1 : selectedShellOf(faces.front());
            if (s >= 0 && counts(*result, e, s)) {
                check.failing.push_back(static_cast<std::size_t>(s));
            }
        }
        std::sort(check.failing.begin(), check.failing.end());
        check.failing.erase(std::unique(check.failing.begin(), check.failing.end()), check.failing.end());
    };
    const auto any = [](const CategoryResult&, std::size_t, int) { return true; };
    list.checks[MergeChecklist::kClosed].label = "closed (no open edges)";
    fill(list.checks[MergeChecklist::kClosed], DefectCategory::OpenEdges, any);
    list.checks[MergeChecklist::kOriented].label = "consistently oriented (no flipped normals)";
    fill(list.checks[MergeChecklist::kOriented], DefectCategory::FlippedNormals, any);
    list.checks[MergeChecklist::kNoFins].label = "no fins or internal walls";
    fill(list.checks[MergeChecklist::kNoFins], DefectCategory::NonManifoldEdges,
         [](const CategoryResult& r, std::size_t e, int) { return r.elements[e].subtype != 0; });
    list.checks[MergeChecklist::kNoSelfCrossings].label = "free of self-crossings";
    list.checks[MergeChecklist::kNoSelfCrossings].blocking = false;
    fill(list.checks[MergeChecklist::kNoSelfCrossings], DefectCategory::Intersections, [&](const CategoryResult& r, std::size_t e, int s) {
        const auto related = r.relatedOf(e);
        return std::any_of(related.begin(), related.end(), [&](Eigen::Index p) { return shells_.shellOfFace(p) == s; });
    });

    if (const CategoryResult* crossings = currentResult(DefectCategory::Intersections)) {
        for (std::size_t e = 0; e < crossings->size(); ++e) {
            const auto faces = crossings->facesOf(e);
            const int s = faces.empty() ? -1 : selectedShellOf(faces.front());
            const auto related = crossings->relatedOf(e);
            if (s >= 0 && std::any_of(related.begin(), related.end(), [&](Eigen::Index p) {
                    const int other = selectedShellOf(p);
                    return other >= 0 && other != s;
                })) {
                ++list.crossingsBetween;
            }
        }
    }
    if (list.shells.size() >= 2) {
        checkRoles(list, selected);
    }
    return list;
}

void AppController::checkRoles(MergeChecklist& list, const std::vector<std::uint8_t>& selected) const {
    MergeCheck& check = list.checks[MergeChecklist::kOutwardOrCavity];
    check.label = "outward, or a cavity nothing reaches into";
    check.known = true;
    const MergeRoles& merge = mergeRoles();
    if (merge.roles.shells.size() != shells_.size()) {
        list.operands = list.shells; // no closed shell is wound inward
        return;
    }
    const std::vector<ShellRoleInfo>& roles = merge.roles.shells;
    const std::vector<ShellFacts>& facts = merge.relations.shells;
    const auto isSelected = [&](int s) { return s >= 0 && selected[static_cast<std::size_t>(s)] != 0; };
    const auto label = [](int s) { return shellLabel(static_cast<std::size_t>(s)); };
    const auto fail = [&](std::string problem, std::initializer_list<int> shells) {
        list.problems.push_back(std::move(problem));
        for (const int s : shells) {
            if (isSelected(s)) {
                check.failing.push_back(static_cast<std::size_t>(s));
            }
        }
    };

    // Each selected shell by its role: outward ones are operands, cavities pass through, the rest block
    std::vector<int> operands;
    for (const std::size_t s : list.shells) {
        const ShellRoleInfo& info = roles[s];
        const bool inward = facts[s].signedVolume < 0.0;
        if (info.role == ShellRole::InsideOut) {
            fail(shellLabel(s) + " is inside out: fix orientation first (see Inverted shells).", {static_cast<int>(s)});
        } else if (info.role == ShellRole::Unclassified && inward) {
            fail(shellLabel(s) + " " + formatShellRoleReason(info, label), {static_cast<int>(s)});
        } else if (info.role != ShellRole::Cavity && info.role != ShellRole::InsideOutWithInner) {
            operands.push_back(static_cast<int>(s));
        }
    }
    for (const std::vector<int>& group : merge.roles.stoppedGroups) {
        if (std::none_of(group.begin(), group.end(), isSelected)) {
            continue;
        }
        const std::vector<int> inner(group.begin() + 1, group.end());
        list.problems.push_back(label(group.front()) + " and its inner shell" + (inner.size() > 1 ? "s " : " ") +
                                joinLabels(inner, label) + " look inside out together: fix orientation first.");
        for (const int s : group) {
            if (isSelected(s)) {
                check.failing.push_back(static_cast<std::size_t>(s));
            }
        }
    }

    // Hollows (cavities, and inward shells rule O could not judge): does a selected operand reach one?
    std::vector<int> hollows;
    std::vector<int> containers;
    for (std::size_t s = 0; s < roles.size(); ++s) {
        if (roles[s].role == ShellRole::Cavity || (roles[s].role == ShellRole::Unclassified && facts[s].signedVolume < 0.0)) {
            hollows.push_back(static_cast<int>(s));
            containers.push_back(facts[s].nestedIn);
        }
    }
    const std::vector<CavityReach> reaches = findCavityReaches(merge.relations, operands, hollows, containers);
    for (std::size_t i = 0; i < hollows.size(); ++i) {
        const int h = hollows[i];
        const int container = containers[i];
        std::vector<int> reaching;
        for (const CavityReach& reach : reaches) {
            if (reach.hollow == h) {
                reaching.push_back(reach.operand);
            }
        }
        if (!reaching.empty()) {
            fail(joinLabels(reaching, label) + (reaching.size() > 1 ? " reach" : " reaches") + " into cavity " + label(h) +
                     (container >= 0 ? " of " + label(container) : std::string()) + ": merge them by hand.",
                 {h, container});
            for (const int o : reaching) {
                check.failing.push_back(static_cast<std::size_t>(o));
            }
        } else if (roles[static_cast<std::size_t>(h)].role == ShellRole::Cavity && (isSelected(h) || isSelected(container))) {
            list.cavities.push_back(static_cast<std::size_t>(h));
        }
    }
    sortUnique(check.failing);

    // An island floats in a kept cavity: the union would swallow it, so it is left as it is
    for (const int o : operands) {
        const int cavity = facts[static_cast<std::size_t>(o)].nestedIn;
        if (roles[static_cast<std::size_t>(o)].role == ShellRole::Island && cavity >= 0 &&
            std::binary_search(list.cavities.begin(), list.cavities.end(), static_cast<std::size_t>(cavity)) &&
            isSelected(facts[static_cast<std::size_t>(cavity)].nestedIn)) {
            list.islands.push_back(static_cast<std::size_t>(o));
        } else {
            list.operands.push_back(static_cast<std::size_t>(o));
        }
    }
}

bool AppController::mergeSelectedShells() {
    const MergeChecklist list = mergeChecklist();
    if (list.shells.size() < 2) {
        postStatusMessage("Select two or more shells to merge.", true);
        return false;
    }
    if (!list.canApply()) {
        postStatusMessage("Some selected shells cannot be merged yet: see the checklist in Merge Shells.", true);
        return false;
    }
    if (mergeRunner_.busy()) {
        return false;
    }

    MergeRequest request;
    request.meshGeneration = meshGeneration_;
    request.mesh = std::make_shared<const Mesh>(mesh_);
    request.input.intersectionTolerance = intersectionTolerance_;
    request.operandIds = list.operands;
    for (const std::size_t s : list.operands) {
        request.input.operands.push_back(shells_.shell(s).faces);
    }
    const MergeRoles& merge = mergeRoles();
    for (const std::size_t c : list.cavities) {
        const int container = merge.relations.shells[c].nestedIn;
        const auto it = std::find(list.operands.begin(), list.operands.end(), static_cast<std::size_t>(container));
        request.cavityIds.push_back(c);
        request.input.cavities.push_back(
            {shells_.shell(c).faces, it == list.operands.end() ? -1 : static_cast<int>(it - list.operands.begin())});
    }
    if (backgroundChecks_) {
        postStatusMessage("Merging " + std::to_string(list.operands.size()) + " shells with Manifold...");
        mergeRunner_.start(std::move(request));
        return true;
    }
    ShellMergeResult result = mergeShells(*request.mesh, request.input);
    return applyMergeResult({request.meshGeneration, std::move(request.input), std::move(request.operandIds),
                             std::move(request.cavityIds), std::move(result)});
}

bool AppController::applyMergeResult(FinishedMerge finished) {
    if (finished.meshGeneration != meshGeneration_) {
        postStatusMessage("The mesh changed while the shells were merging, so the merge was dropped.", true);
        return false;
    }
    ShellMergeResult& r = finished.result;
    const auto operandLabel = [&](int o) { return shellLabel(finished.operandIds[static_cast<std::size_t>(o)]); };
    const auto cavityLabel = [&](int c) { return shellLabel(finished.cavityIds[static_cast<std::size_t>(c)]); };
    // Shells the merge left as they were, and why
    std::string leftAlone;
    for (const RefusedMergeGroup& group : r.refusedGroups) {
        leftAlone += " " + describeRefusedGroup(finished.input, group, operandLabel, cavityLabel);
    }
    for (const RefusedMergeShell& refused : r.refusedShells) {
        leftAlone += " " + operandLabel(refused.operand) + ": " + refused.reason;
    }
    if (!r.success || r.operandsMerged == 0) {
        lastRepairStatusMessage_ = "Merge failed. " + r.message + leftAlone;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }

    // Faces not united keep their order at the front: where each kept cavity's first face lands
    std::vector<std::uint8_t> united(static_cast<std::size_t>(mesh_.numFaces()), 0);
    for (std::size_t o = 0; o < finished.input.operands.size(); ++o) {
        const int index = static_cast<int>(o);
        const bool left = std::any_of(r.refusedShells.begin(), r.refusedShells.end(), [&](const RefusedMergeShell& x) { return x.operand == index; }) ||
                          std::any_of(r.refusedGroups.begin(), r.refusedGroups.end(), [&](const RefusedMergeGroup& g) {
                              return std::find(g.operands.begin(), g.operands.end(), index) != g.operands.end();
                          });
        if (!left) {
            for (const Eigen::Index f : finished.input.operands[o]) {
                united[static_cast<std::size_t>(f)] = 1;
            }
        }
    }
    std::vector<Eigen::Index> newIndex(united.size(), -1);
    Eigen::Index next = 0;
    for (std::size_t f = 0; f < united.size(); ++f) {
        if (united[f] == 0) {
            newIndex[f] = next++;
        }
    }

    const Eigen::Index facesBefore = mesh_.numFaces();
    const std::string unit(unitSymbol(mesh_.fileUnit));
    pushUndoEntry("Merge " + std::to_string(finished.operandIds.size()) + " Shells");
    const Eigen::Index firstMerged = r.firstMergedFace;
    finishMeshEdit(std::move(r.mesh));
    if (shells_.multiShell()) {
        selectShellsWhere([&](std::size_t s) { return shells_.shell(s).faces.front() >= firstMerged; });
    }

    std::string cavities;
    if (r.cavitiesKept > 0) {
        std::vector<std::size_t> kept;
        for (std::size_t c = 0; c < finished.input.cavities.size(); ++c) {
            const Eigen::Index first = newIndex[static_cast<std::size_t>(finished.input.cavities[c].faces.front())];
            const int now = first >= 0 ? shells_.shellOfFace(first) : -1;
            if (now >= 0 && std::none_of(r.refusedGroups.begin(), r.refusedGroups.end(), [&](const RefusedMergeGroup& g) {
                    return std::find(g.cavities.begin(), g.cavities.end(), static_cast<int>(c)) != g.cavities.end();
                })) {
                kept.push_back(static_cast<std::size_t>(now));
            }
        }
        std::sort(kept.begin(), kept.end());
        cavities = " " + std::to_string(r.cavitiesKept) + (r.cavitiesKept == 1 ? " cavity" : " cavities") + " kept as " +
                   (r.cavitiesKept == 1 ? "it was: " : "they were: ") + joinLabels(kept, shellLabel) + " (" +
                   formatValue(r.cavityVolume) + " " + unit + "^3).";
    }
    lastRepairStatusMessage_ =
        "Merged " + std::to_string(r.operandsMerged) + " shells into " + std::to_string(r.unionShells) + " (" +
        formatValue(r.elapsedMs / 1000.0) + " s): faces " + std::to_string(facesBefore) + " -> " +
        std::to_string(mesh_.numFaces()) + ", volume " + formatValue(r.volumeBefore) + " -> " + formatValue(r.unionVolume) +
        " " + unit + "^3 (overlaps counted once), " + formatValue(100.0 * (1.0 - r.unchangedFraction)) + "% of their faces changed." +
        cavities + (leftAlone.empty() ? std::string() : " Left as they were:" + leftAlone) +
        " The cuts can add thin faces: see Mesh Health.";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::pollMergeJob() {
    std::optional<FinishedMerge> finished = mergeRunner_.takeFinished();
    if (!finished) {
        return false;
    }
    applyMergeResult(std::move(*finished));
    return true;
}

void AppController::cancelMerge() {
    mergeRunner_.discard();
    postStatusMessage("Merge cancelled.");
}

void AppController::waitForMerge() {
    mergeRunner_.waitIdle();
    pollMergeJob();
}

} // namespace mesheal
