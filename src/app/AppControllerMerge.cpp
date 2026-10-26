#include "app/AppController.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <utility>

namespace ogun {

namespace {

[[nodiscard]] std::string formatValue(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.6g", value);
    return buffer;
}

} // namespace

bool MergeChecklist::canApply() const noexcept {
    return shells.size() >= 2 && std::all_of(checks.begin(), checks.end(), [](const MergeCheck& check) {
        return !check.blocking || (check.known && check.failing.empty());
    });
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
    list.checks[0].label = "closed (no open edges)";
    fill(list.checks[0], DefectCategory::OpenEdges, any);
    list.checks[1].label = "consistently oriented (no flipped normals)";
    fill(list.checks[1], DefectCategory::FlippedNormals, any);
    list.checks[2].label = "no fins or internal walls";
    fill(list.checks[2], DefectCategory::NonManifoldEdges,
         [](const CategoryResult& r, std::size_t e, int) { return r.elements[e].subtype != 0; });
    list.checks[3].label = "free of self-crossings";
    list.checks[3].blocking = false;
    fill(list.checks[3], DefectCategory::Intersections, [&](const CategoryResult& r, std::size_t e, int s) {
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
    return list;
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
    request.shellIds = list.shells;
    for (const std::size_t s : list.shells) {
        request.shells.push_back(shells_.shell(s).faces);
    }
    if (backgroundChecks_) {
        postStatusMessage("Merging " + std::to_string(list.shells.size()) + " shells with Manifold...");
        mergeRunner_.start(std::move(request));
        return true;
    }
    ShellMergeResult result = mergeShells(*request.mesh, request.shells);
    return applyMergeResult({request.meshGeneration, std::move(request.shellIds), std::move(result)});
}

bool AppController::applyMergeResult(FinishedMerge finished) {
    if (finished.meshGeneration != meshGeneration_) {
        postStatusMessage("The mesh changed while the shells were merging, so the merge was dropped.", true);
        return false;
    }
    ShellMergeResult& r = finished.result;
    if (!r.success) {
        const std::string shell = r.rejectedShell >= 0
            ? "Shell #" + std::to_string(finished.shellIds[static_cast<std::size_t>(r.rejectedShell)] + 1) + ": "
            : std::string();
        lastRepairStatusMessage_ = "Merge failed. " + shell + r.message;
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }

    const Eigen::Index facesBefore = mesh_.numFaces();
    const std::string unit(unitSymbol(mesh_.fileUnit));
    pushUndoEntry("Merge " + std::to_string(r.shellsIn) + " Shells");
    const Eigen::Index firstMerged = r.firstMergedFace;
    finishMeshEdit(std::move(r.mesh));
    selectShellsWhere([&](std::size_t s) { return shells_.shell(s).faces.front() >= firstMerged; });

    lastRepairStatusMessage_ =
        "Merged " + std::to_string(r.shellsIn) + " shells into " + std::to_string(r.shellsOut) + " (" +
        formatValue(r.elapsedMs / 1000.0) + " s): faces " + std::to_string(facesBefore) + " -> " +
        std::to_string(mesh_.numFaces()) + ", volume " + formatValue(r.volumeBefore) + " -> " + formatValue(r.volumeAfter) +
        " " + unit + "^3 (overlaps counted once), " + formatValue(100.0 * r.unchangedFraction) +
        "% of their faces unchanged. The cuts can add thin faces: see Mesh Health.";
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

} // namespace ogun
