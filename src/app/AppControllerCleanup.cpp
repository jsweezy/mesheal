#include "app/AppController.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "repair/TopologyCleanup.hpp"

#include <algorithm>
#include <utility>

namespace ogun {

namespace {

[[nodiscard]] std::string countOf(std::size_t n, const char* singular, const char* plural) {
    return std::to_string(n) + " " + (n == 1 ? singular : plural);
}

/// "1 edge flip, 2 edge collapses and 1 dangling flap removed".
[[nodiscard]] std::string healActionsText(const RepairResults& results) {
    std::vector<std::string> parts;
    if (results.edgeFlips > 0) {
        parts.push_back(countOf(static_cast<std::size_t>(results.edgeFlips), "edge flip", "edge flips"));
    }
    if (results.edgeCollapses > 0) {
        parts.push_back(countOf(static_cast<std::size_t>(results.edgeCollapses), "edge collapse", "edge collapses"));
    }
    if (results.facesRemoved > 0) {
        parts.push_back(countOf(static_cast<std::size_t>(results.facesRemoved), "dangling flap", "dangling flaps") +
                        " removed");
    }
    std::string text;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        text += (i == 0 ? "" : (i + 1 == parts.size() ? " and " : ", ")) + parts[i];
    }
    return text;
}

} // namespace

bool AppController::fixAllDegenerateFaces() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }
    const bool scoped = shells_.scoped();
    const std::vector<std::uint8_t> scope = repairScopeMask();

    // 1. Faces that repeat a vertex bound nothing: removing them changes no edge
    RepeatedVertexFacesResult dropped = removeRepeatedVertexFaces(mesh_, scope);
    Mesh mesh = std::move(dropped.mesh);
    std::vector<std::uint8_t> keptScope;
    if (!scope.empty()) {
        keptScope.assign(static_cast<std::size_t>(mesh.numFaces()), 0);
        for (std::size_t f = 0; f < dropped.faceMap.size(); ++f) {
            if (dropped.faceMap[f] >= 0) {
                keptScope[static_cast<std::size_t>(dropped.faceMap[f])] = scope[f];
            }
        }
    }

    // 2. Zero-area faces with three distinct vertices: healed in place, never deleted
    const CategoryResult degenerate = checkDegenerateFaces(mesh);
    std::vector<Eigen::Index> zeroArea;
    bool collapsedLeft = false;
    for (const DefectElement& element : degenerate.elements) {
        if (element.subtype == 0) {
            collapsedLeft = true; // outside the scope
        } else if (keptScope.empty() || keptScope[static_cast<std::size_t>(element.index)] != 0) {
            zeroArea.push_back(element.index);
        }
    }
    std::sort(zeroArea.begin(), zeroArea.end());

    std::string healText;
    std::size_t healed = 0;
    if (!zeroArea.empty()) {
        const std::string left = countOf(zeroArea.size(), "zero-area face was", "zero-area faces were") + " left";
        if (checkDuplicateFaces(mesh).size() > 0) {
            healText = left + ": the mesh has duplicate faces, which the healer cannot read without tearing them "
                              "apart. Fix the Duplicate faces row first, then run this again.";
        } else if (collapsedLeft) {
            healText = left + ": shells outside the scope have faces with a repeated vertex, which the healer "
                              "cannot read. Fix them first (clear the shell selection), then run this again.";
        } else {
            RepairResults results = SliverHealer::healFaces(mesh, zeroArea, thresholds_);
            healed = results.success ? static_cast<std::size_t>(results.edgeFlips + results.edgeCollapses +
                                                                results.facesRemoved)
                                     : 0;
            if (healed > 0) {
                mesh = std::move(results.mesh);
                healText = "Healed " + std::to_string(healed) + " of " +
                           countOf(zeroArea.size(), "zero-area face", "zero-area faces") + " (" +
                           healActionsText(results) + ").";
            }
            if (healed < zeroArea.size()) {
                const std::size_t unhealed = zeroArea.size() - healed;
                healText += std::string(healText.empty() ? "" : " ") +
                            countOf(unhealed, "zero-area face", "zero-area faces") +
                            " could not be healed safely (a flip or collapse would fold or tear the surface) and " +
                            (unhealed == 1 ? "was" : "were") + " left as " + (unhealed == 1 ? "it is" : "they are") +
                            ": deleting " + (unhealed == 1 ? "it" : "them") + " would open a crack.";
            }
        }
    }

    if (dropped.removed == 0 && healed == 0) {
        lastRepairStatusMessage_ = !healText.empty() ? healText
                                   : scoped          ? "No degenerate faces in the shells in scope."
                                                     : "No degenerate faces to fix.";
        postStatusMessage(lastRepairStatusMessage_, !healText.empty());
        return false;
    }

    std::string label = "Fix All Degenerate Faces";
    if (scoped) {
        label += " (" + std::to_string(shells_.scopeCount()) + " shells)";
    }
    pushUndoEntry(std::move(label));
    finishMeshEdit(std::move(mesh));
    lastRepairStatusMessage_.clear();
    if (dropped.removed > 0) {
        lastRepairStatusMessage_ = "Removed " +
                                   countOf(dropped.removed, "collapsed face", "collapsed faces") +
                                   " (a repeated vertex).";
    }
    if (!healText.empty()) {
        lastRepairStatusMessage_ += (lastRepairStatusMessage_.empty() ? "" : " ") + healText;
    }
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixAllDuplicateFaces() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }
    const bool scoped = shells_.scoped();
    DuplicateFacesResult result = removeDuplicateFaces(mesh_, repairScopeMask());
    const std::string skipped = result.groupsSkipped == 0 ? std::string()
        : " " + countOf(result.groupsSkipped, "set of copies was", "sets of copies were") +
          " left as they were: a copy outside the scope would have to go.";
    if (result.removed() == 0) {
        lastRepairStatusMessage_ = (scoped ? "No duplicate faces to remove in the shells in scope."
                                           : "No duplicate faces to remove.") + skipped;
        postStatusMessage(lastRepairStatusMessage_, !skipped.empty());
        return false;
    }

    std::string label = "Fix All Duplicate Faces";
    if (scoped) {
        label += " (" + std::to_string(shells_.scopeCount()) + " shells)";
    }
    const std::size_t shellsBefore = shells_.size();
    pushUndoEntry(std::move(label));
    finishMeshEdit(std::move(result.mesh));
    lastRepairStatusMessage_ = "Removed " + countOf(result.removed(), "duplicate face", "duplicate faces") + " (" +
                               std::to_string(result.removedSameWinding) + " same, " +
                               std::to_string(result.removedOppositeWinding) + " opposite winding).";
    if (shells_.size() != shellsBefore) {
        lastRepairStatusMessage_ += " Shells: " + std::to_string(shellsBefore) + " -> " + std::to_string(shells_.size()) + ".";
    }
    lastRepairStatusMessage_ += skipped;
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

} // namespace ogun
