#include "app/AppController.hpp"

#include "repair/Orientation.hpp"

#include <algorithm>
#include <utility>

namespace mesheal {

namespace {

[[nodiscard]] std::string countOf(std::size_t n, const char* singular, const char* plural) {
    return std::to_string(n) + " " + (n == 1 ? singular : plural);
}

/// "#2", "#2 and #5", "#2, #5 and #7".
[[nodiscard]] std::string joinLabels(const std::vector<std::string>& labels) {
    std::string out;
    for (std::size_t i = 0; i < labels.size(); ++i) {
        out += (i == 0 ? "" : (i + 1 == labels.size() ? " and " : ", ")) + labels[i];
    }
    return out;
}

} // namespace

bool AppController::fixAllFlippedNormals() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }
    const bool scoped = shells_.scoped();
    OrientShellsOptions options;
    options.reverseInsideOut = false;
    OrientShellsResult result = orientShells(mesh_, repairScopeMask(), options);

    // Shells-panel numbers of the result's shells
    const auto label = [&](int s) {
        const std::vector<Eigen::Index>& faces = result.shells[static_cast<std::size_t>(s)];
        return "#" + std::to_string(faces.empty() ? s + 1 : shells_.shellOfFace(faces.front()) + 1);
    };
    std::string twisted;
    if (!result.nonOrientable.empty()) {
        std::vector<std::string> labels;
        for (const int s : result.nonOrientable) {
            labels.push_back(label(s));
        }
        twisted = std::string(labels.size() == 1 ? " Shell " : " Shells ") + joinLabels(labels) +
                  (labels.size() == 1 ? " has" : " have") +
                  " a twist (like a Mobius strip), so no winding agrees everywhere: left as " +
                  (labels.size() == 1 ? "it is." : "they are.");
    }
    if (result.flippedFaces == 0) {
        lastRepairStatusMessage_ = (scoped ? "No flipped faces in the shells in scope." : "No flipped faces to fix.") + twisted;
        postStatusMessage(lastRepairStatusMessage_, !twisted.empty());
        return false;
    }

    // A closed shell whose majority faced inward now faces inward as a whole: rule O judges it
    std::vector<std::vector<Eigen::Index>> touched;
    for (const int s : result.shellsWithFlippedFaces) {
        touched.push_back(result.shells[static_cast<std::size_t>(s)]);
    }
    const std::vector<ShellFacts> facts = describeShells(result.mesh, touched);
    std::vector<std::string> inward;
    for (std::size_t i = 0; i < facts.size(); ++i) {
        if (facts[i].oriented && facts[i].signedVolume < 0.0) {
            inward.push_back(label(result.shellsWithFlippedFaces[i]));
        }
    }

    std::string actionLabel = "Fix All Flipped Normals";
    if (scoped) {
        actionLabel += " (" + std::to_string(shells_.scopeCount()) + " shells)";
    }
    pushUndoEntry(std::move(actionLabel));
    lastRepairStatusMessage_ = "Flipped " + countOf(result.flippedFaces, "face", "faces") + " in " +
                               countOf(result.shellsWithFlippedFaces.size(), "shell", "shells") +
                               " to agree with their neighbours.";
    finishMeshEdit(std::move(result.mesh));
    if (!inward.empty()) {
        lastRepairStatusMessage_ += " " + joinLabels(inward) + (inward.size() == 1 ? " now faces" : " now face") +
                                    " inward as a whole: Inverted shells tells a cavity from an inside-out shell.";
    }
    lastRepairStatusMessage_ += twisted;
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::fixAllInvertedShells() {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }
    const bool scoped = shells_.scoped();
    OrientShellsOptions options;
    options.flipFaces = false;
    options.intersectionTolerance = intersectionTolerance_;
    OrientShellsResult result = orientShells(mesh_, repairScopeMask(), options);
    const auto label = [&](int s) {
        const std::vector<Eigen::Index>& faces = result.shells[static_cast<std::size_t>(s)];
        return "#" + std::to_string(faces.empty() ? s + 1 : shells_.shellOfFace(faces.front()) + 1);
    };
    const auto labelsOf = [&](const std::vector<int>& shells) {
        std::vector<std::string> labels;
        for (const int s : shells) {
            labels.push_back(label(s));
        }
        return joinLabels(labels);
    };

    // What was left, and why (labels are taken before the mesh changes; shell numbers do not change)
    std::string notes;
    if (!result.cavitiesKept.empty()) {
        notes += " Kept " + countOf(result.cavitiesKept.size(), "cavity", "cavities") + " inward: " +
                 labelsOf(result.cavitiesKept) + ".";
    }
    for (const std::vector<int>& group : result.stopped) {
        const std::vector<int> inner(group.begin() + 1, group.end());
        notes += " " + label(group.front()) + " and its inner shell" + (inner.size() > 1 ? "s " : " ") +
                 labelsOf(inner) + " look inside out together: left for you (Flip all or Flip outer only, under "
                                   "Inverted shells).";
    }
    if (!result.selfCrossing.empty()) {
        notes += " " + labelsOf(result.selfCrossing) + (result.selfCrossing.size() == 1 ? " crosses itself" : " cross themselves") +
                 ", so inside and outside are not clear: left as " + (result.selfCrossing.size() == 1 ? "it is." : "they are.");
    }
    if (!result.unclassified.empty()) {
        notes += " " + labelsOf(result.unclassified) + (result.unclassified.size() == 1 ? " is" : " are") +
                 " wound inward where a cavity and an inside-out shell look alike: left as " +
                 (result.unclassified.size() == 1 ? "it is" : "they are") + " (see the warnings under Inverted shells).";
    }
    if (!result.outOfScope.empty()) {
        notes += " " + countOf(result.outOfScope.size(), "inside-out shell", "inside-out shells") +
                 " outside the scope " + (result.outOfScope.size() == 1 ? "was" : "were") + " left.";
    }

    if (result.reversedShells.empty()) {
        lastRepairStatusMessage_ = (scoped ? "No inside-out shells to reverse in the shells in scope."
                                           : "No inside-out shells to reverse.") + notes;
        postStatusMessage(lastRepairStatusMessage_, result.stopped.size() + result.selfCrossing.size() > 0);
        return false;
    }
    std::string actionLabel = "Fix All Inverted Shells";
    if (scoped) {
        actionLabel += " (" + std::to_string(shells_.scopeCount()) + " shells)";
    }
    lastRepairStatusMessage_ = "Reversed " + countOf(result.reversedShells.size(), "inside-out shell", "inside-out shells") +
                               ": " + labelsOf(result.reversedShells) + "." + notes;
    pushUndoEntry(std::move(actionLabel));
    finishMeshEdit(std::move(result.mesh));
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

bool AppController::flipShells(std::span<const int> shells) {
    if (mesh_.empty()) {
        lastRepairStatusMessage_ = "No mesh loaded.";
        return false;
    }
    std::vector<int> chosen;
    for (const int s : shells) {
        if (s >= 0 && static_cast<std::size_t>(s) < shells_.size()) {
            chosen.push_back(s);
        }
    }
    std::sort(chosen.begin(), chosen.end());
    chosen.erase(std::unique(chosen.begin(), chosen.end()), chosen.end());
    if (chosen.empty()) {
        lastRepairStatusMessage_ = "No shells to flip.";
        postStatusMessage(lastRepairStatusMessage_, true);
        return false;
    }
    std::vector<std::vector<Eigen::Index>> faceSets;
    std::vector<std::string> labels;
    for (const int s : chosen) {
        faceSets.push_back(shells_.shell(static_cast<std::size_t>(s)).faces);
        labels.push_back("#" + std::to_string(s + 1));
    }
    FlipShellsResult result = mesheal::flipShells(mesh_, faceSets);
    pushUndoEntry(std::string(chosen.size() == 1 ? "Flip Shell " : "Flip Shells ") + joinLabels(labels));
    finishMeshEdit(std::move(result.mesh));
    lastRepairStatusMessage_ = "Flipped " + joinLabels(labels) + " (" + countOf(result.flippedFaces, "face", "faces") +
                               "): " + (chosen.size() == 1 ? "it now faces" : "they now face") + " the other way.";
    postStatusMessage(lastRepairStatusMessage_);
    return true;
}

} // namespace mesheal
