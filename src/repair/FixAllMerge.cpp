// Fix All phase 2, the merge itself: delete the nested solids that touch nothing, then unite each merge
// group of the pass's plan (`planMerge`) with the cavity-safe `mergeShells`, checking the merge guards.
// The engine (FixAll.cpp) runs these stages and checks the other hard guards; phase 3 cleans up after.

#include "repair/MergePlan.hpp"

#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "repair/FixAllDetail.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/ShellMerge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <utility>

namespace ogun {

namespace {

using fixall_detail::counted;
using fixall_detail::shellList;

[[nodiscard]] std::string label(int shell) {
    return "#" + std::to_string(shell + 1);
}

[[nodiscard]] std::string number(double value) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.6g", value);
    return text.data();
}

/// Rows in the stage's mesh of `faces` (faces of the planned mesh); false when one is gone.
[[nodiscard]] bool currentFaces(std::span<const Eigen::Index> faces, std::span<const Eigen::Index> planFaceMap,
                                Eigen::Index faceCount, std::vector<Eigen::Index>& out) {
    out.clear();
    out.reserve(faces.size());
    for (const Eigen::Index f : faces) {
        Eigen::Index row = f;
        if (!planFaceMap.empty()) {
            row = f >= 0 && static_cast<std::size_t>(f) < planFaceMap.size() ? planFaceMap[static_cast<std::size_t>(f)] : -1;
        }
        if (row < 0 || row >= faceCount) {
            return false;
        }
        out.push_back(row);
    }
    return !out.empty();
}

/// Shells-panel numbers (0-based, `shellsLargestFirst`) of a mesh's faces.
class ShellNumbers {
public:
    explicit ShellNumbers(const Mesh& mesh)
        : shells_(shellsLargestFirst(mesh)), of_(fixall_detail::shellOfFaces(mesh.numFaces(), shells_)) {}

    /// The shell of the first of `faces`.
    [[nodiscard]] int of(std::span<const Eigen::Index> faces) const {
        return faces.empty() ? -1 : of_[static_cast<std::size_t>(faces.front())];
    }

    [[nodiscard]] int ofFace(Eigen::Index face) const {
        return of_[static_cast<std::size_t>(face)];
    }

    [[nodiscard]] const std::vector<Eigen::Index>& faces(int shell) const {
        return shells_[static_cast<std::size_t>(shell)];
    }

private:
    std::vector<std::vector<Eigen::Index>> shells_;
    std::vector<int> of_;
};

/// `faces` (rows of the stage's input) mapped through `faceMap`; faces removed are left out.
[[nodiscard]] std::vector<Eigen::Index> mapped(std::span<const Eigen::Index> faces, std::span<const Eigen::Index> faceMap) {
    std::vector<Eigen::Index> out;
    out.reserve(faces.size());
    for (const Eigen::Index f : faces) {
        if (faceMap[static_cast<std::size_t>(f)] >= 0) {
            out.push_back(faceMap[static_cast<std::size_t>(f)]);
        }
    }
    return out;
}

[[nodiscard]] std::size_t inScopeCount(DefectCategory category, const Mesh& mesh, const FixAllScope& scope,
                                       const CheckParams& params, std::stop_token stop, bool& cancelled) {
    const CategoryResult result = runCategoryCheck(category, mesh, params, std::move(stop));
    cancelled = cancelled || result.cancelled;
    return result.cancelled ? 0 : fixall_detail::countInScope(result, scope);
}

// ---------------------------------------------------------------------------------------------
// Delete the isolated nested solids
// ---------------------------------------------------------------------------------------------

struct NestedSolidJob {
    std::vector<Eigen::Index> faces;     ///< In the planned mesh
    std::vector<Eigen::Index> container; ///< In the planned mesh
};

[[nodiscard]] FixAllStageOutcome deleteNestedSolids(const Mesh& mesh, const FixAllStageContext& context,
                                                    const std::vector<NestedSolidJob>& jobs) {
    FixAllStageOutcome outcome;
    outcome.targetBefore = jobs.size();
    const ShellNumbers numbers(mesh);
    std::vector<std::uint8_t> gone(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::vector<int> solids;
    std::vector<int> containers;
    std::vector<std::vector<Eigen::Index>> containerFaces;
    std::vector<Eigen::Index> faces;
    for (const NestedSolidJob& job : jobs) {
        std::vector<Eigen::Index> container;
        if (!currentFaces(job.faces, context.planFaceMap, mesh.numFaces(), faces) ||
            !currentFaces(job.container, context.planFaceMap, mesh.numFaces(), container)) {
            outcome.failure = "Its shells changed after the merge was planned.";
            return outcome;
        }
        for (const Eigen::Index f : faces) {
            gone[static_cast<std::size_t>(f)] = 1;
        }
        solids.push_back(numbers.of(faces));
        containers.push_back(numbers.of(container));
        containerFaces.push_back(std::move(container));
    }

    std::vector<Eigen::Index> kept;
    kept.reserve(gone.size());
    outcome.faceMap.assign(gone.size(), -1);
    for (std::size_t f = 0; f < gone.size(); ++f) {
        if (gone[f] == 0) {
            outcome.faceMap[f] = static_cast<Eigen::Index>(kept.size());
            kept.push_back(static_cast<Eigen::Index>(f));
        }
    }
    outcome.mesh = extractFaces(mesh, kept);
    outcome.changed = true;
    outcome.targetAfter = 0;

    std::vector<int> holders = containers;
    std::sort(holders.begin(), holders.end());
    holders.erase(std::unique(holders.begin(), holders.end()), holders.end());
    std::vector<int> deleted = solids;
    std::sort(deleted.begin(), deleted.end());
    outcome.detail = std::string("Deleted ") + (deleted.size() == 1 ? "nested solid " : "nested solids ") +
                     shellList(deleted) + " (wholly inside " + shellList(holders) + ", touching nothing)";
    for (std::size_t i = 0; i < solids.size(); ++i) {
        outcome.notes.push_back({"Deleted nested solid " + label(solids[i]) + " (wholly inside " + label(containers[i]) +
                                     ", touching nothing)",
                                 {{}, mapped(containerFaces[i], outcome.faceMap)}});
    }
    return outcome;
}

[[nodiscard]] FixAllStage deleteNestedSolidsStage(std::vector<NestedSolidJob> jobs) {
    std::size_t faces = 0;
    for (const NestedSolidJob& job : jobs) {
        faces += job.faces.size();
    }
    FixAllStage stage;
    stage.id = FixAllStageId::DeleteNestedSolids;
    stage.label = "Delete nested solids";
    stage.activity = jobs.size() == 1 ? "deleting a nested solid" : "deleting nested solids";
    stage.progressWeight = static_cast<double>(faces);
    stage.run = [jobs = std::move(jobs)](const Mesh& mesh, const FixAllStageContext& context) {
        return deleteNestedSolids(mesh, context, jobs);
    };
    return stage;
}

// ---------------------------------------------------------------------------------------------
// Unite one merge group
// ---------------------------------------------------------------------------------------------

struct MergeGroupJob {
    std::vector<std::vector<Eigen::Index>> operands; ///< In the planned mesh
    std::vector<std::vector<Eigen::Index>> cavities; ///< In the planned mesh: kept bit-identical
    std::vector<int> cavityContainers;               ///< Index into `operands` (-1: none)
};

/// Old face -> new row for `mergeShells`' result: faces of no operand keep their order before the union;
/// an operand face the union kept bit-identical maps to that union face; the others are removed.
[[nodiscard]] std::vector<Eigen::Index> mergeFaceMap(const Mesh& before, const ShellMergeResult& merged,
                                                     const std::vector<std::vector<Eigen::Index>>& operands) {
    std::vector<std::uint8_t> operand(static_cast<std::size_t>(before.numFaces()), 0);
    for (const auto& faces : operands) {
        for (const Eigen::Index f : faces) {
            operand[static_cast<std::size_t>(f)] = 1;
        }
    }
    std::unordered_map<FaceGeometryKey, std::vector<Eigen::Index>, FaceGeometryKeyHash> unionRows;
    for (Eigen::Index f = merged.mesh.numFaces() - 1; f >= merged.firstMergedFace; --f) {
        unionRows[faceGeometryKey(merged.mesh, f)].push_back(f); // popped from the back: lowest row first
    }
    std::vector<Eigen::Index> map(operand.size(), -1);
    Eigen::Index row = 0;
    for (std::size_t f = 0; f < operand.size(); ++f) {
        if (operand[f] == 0) {
            map[f] = row++;
            continue;
        }
        const auto it = unionRows.find(faceGeometryKey(before, static_cast<Eigen::Index>(f)));
        if (it != unionRows.end() && !it->second.empty()) {
            map[f] = it->second.back();
            it->second.pop_back();
        }
    }
    if (row != merged.firstMergedFace) {
        return {}; // not the layout mergeShells promises
    }
    return map;
}

[[nodiscard]] FixAllStageOutcome mergeGroup(const Mesh& mesh, const FixAllStageContext& context, const MergeGroupJob& job) {
    const auto start = std::chrono::steady_clock::now();
    FixAllStageOutcome outcome;
    outcome.targetBefore = job.operands.size();
    ShellMergeInput input;
    input.intersectionTolerance = context.params.checkParams.intersectionTolerance;
    std::vector<Eigen::Index> faces;
    for (const auto& operand : job.operands) {
        if (!currentFaces(operand, context.planFaceMap, mesh.numFaces(), faces)) {
            outcome.failure = "Its shells changed after the merge was planned.";
            return outcome;
        }
        input.operands.push_back(faces);
    }
    for (std::size_t c = 0; c < job.cavities.size(); ++c) {
        if (!currentFaces(job.cavities[c], context.planFaceMap, mesh.numFaces(), faces)) {
            outcome.failure = "Its shells changed after the merge was planned.";
            return outcome;
        }
        input.cavities.push_back({faces, job.cavityContainers[c]});
    }
    const ShellNumbers numbers(mesh);
    const auto operandLabel = [&](int o) { return label(numbers.of(input.operands[static_cast<std::size_t>(o)])); };
    const auto cavityLabel = [&](int c) { return label(numbers.of(input.cavities[static_cast<std::size_t>(c)].faces)); };

    const CheckParams& params = context.params.checkParams;
    bool cancelled = false;
    const std::size_t crossingBefore =
        inScopeCount(DefectCategory::Intersections, mesh, context.scope, params, context.stopToken, cancelled);
    const std::size_t thinBefore = inScopeCount(DefectCategory::ThinFaces, mesh, context.scope, params, context.stopToken, cancelled);
    if (cancelled) {
        outcome.cancelled = true;
        return outcome;
    }

    // Manifold cannot be interrupted: a Cancel takes effect when it returns
    ShellMergeResult merged = mergeShells(mesh, input);
    if (context.stopToken.stop_requested()) {
        outcome.cancelled = true;
        return outcome;
    }

    // ---- Merge guards ----
    if (!merged.success) {
        outcome.refusal = "Manifold could not unite them (" + merged.message + "), so nothing was merged.";
        return outcome;
    }
    if (!merged.refusedGroups.empty()) {
        outcome.refusal = describeRefusedGroup(input, merged.refusedGroups.front(), operandLabel, cavityLabel);
        return outcome;
    }
    if (!merged.refusedShells.empty()) {
        const RefusedMergeShell& refused = merged.refusedShells.front();
        outcome.refusal = operandLabel(refused.operand) + " was left out of the union: " + refused.reason +
                          " Nothing was merged.";
        return outcome;
    }
    if (merged.operandsMerged != input.operands.size()) {
        outcome.refusal = "Not every shell of the group could be united, so nothing was merged.";
        return outcome;
    }
    std::vector<Eigen::Index> unionRows;
    unionRows.reserve(static_cast<std::size_t>(merged.mesh.numFaces() - merged.firstMergedFace));
    for (Eigen::Index f = merged.firstMergedFace; f < merged.mesh.numFaces(); ++f) {
        unionRows.push_back(f);
    }
    const Mesh unionMesh = extractFaces(merged.mesh, unionRows);
    const std::vector<std::vector<Eigen::Index>> pieces = shellsLargestFirst(unionMesh);
    const std::vector<ShellFacts> pieceFacts = describeShells(unionMesh, pieces);
    if (pieces.empty() || std::any_of(pieceFacts.begin(), pieceFacts.end(),
                                      [](const ShellFacts& f) { return !f.closed || !f.oriented; })) {
        outcome.refusal = "The union is not closed and consistently wound, so nothing was merged.";
        return outcome;
    }
    std::vector<double> solidVolumes;
    for (const auto& operand : input.operands) {
        solidVolumes.push_back(signedVolume(extractFaces(mesh, operand)));
    }
    std::vector<double> cavityVolumes;
    double signedTotal = signedVolume(unionMesh);
    for (const ShellMergeCavity& cavity : input.cavities) {
        cavityVolumes.push_back(signedVolume(extractFaces(mesh, cavity.faces)));
        signedTotal += cavityVolumes.back();
    }
    if (std::string problem = mergeVolumeProblem(solidVolumes, merged.unionVolume, cavityVolumes, signedTotal);
        !problem.empty()) {
        outcome.refusal = std::move(problem);
        return outcome;
    }
    std::vector<Eigen::Index> faceMap = mergeFaceMap(mesh, merged, input.operands);
    if (faceMap.empty()) {
        outcome.failure = "The union's faces are not where they were expected.";
        return outcome;
    }
    FixAllScope carried = context.scope;
    carried.carry(faceMap, merged.mesh);
    const std::size_t crossingAfter =
        inScopeCount(DefectCategory::Intersections, merged.mesh, carried, params, context.stopToken, cancelled);
    const std::size_t thinAfter = inScopeCount(DefectCategory::ThinFaces, merged.mesh, carried, params, context.stopToken, cancelled);
    if (cancelled) {
        outcome.cancelled = true;
        return outcome;
    }
    if (crossingAfter > crossingBefore) {
        outcome.refusal = "It would raise the crossing faces from " + std::to_string(crossingBefore) + " to " +
                          std::to_string(crossingAfter) + ", so nothing was merged.";
        return outcome;
    }

    // ---- Kept: the report ----
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::array<char, 32> time{};
    std::snprintf(time.data(), time.size(), "%.2g s", seconds);
    outcome.detail = "Merged " + counted(input.operands.size(), "shell", "shells") + " into " +
                     std::to_string(merged.unionShells) + " (" + time.data() + "): crossing faces " +
                     std::to_string(crossingBefore) + " -> " + std::to_string(crossingAfter) + ", faces " +
                     std::to_string(mesh.numFaces()) + " -> " + std::to_string(merged.mesh.numFaces()) + ", volume " +
                     fixall_detail::formatVolume(merged.volumeBefore, mesh.fileUnit) + " -> " +
                     fixall_detail::formatVolume(merged.unionVolume, mesh.fileUnit);
    if (thinAfter > thinBefore) {
        outcome.detail += "; thin faces " + std::to_string(thinBefore) + " -> " + std::to_string(thinAfter) +
                          ", left for the clean-up";
    }
    const ShellNumbers after(merged.mesh);
    std::vector<int> unionShells;
    for (Eigen::Index f = merged.firstMergedFace; f < merged.mesh.numFaces(); ++f) {
        const int s = after.ofFace(f);
        if (s >= 0 && std::find(unionShells.begin(), unionShells.end(), s) == unionShells.end()) {
            unionShells.push_back(s);
        }
    }
    std::sort(unionShells.begin(), unionShells.end());
    FixAllShellNote unionNote{"Merged " + counted(input.operands.size(), "shell", "shells") + " into " + shellList(unionShells), {}};
    for (const int s : unionShells) {
        unionNote.shells.push_back(after.faces(s));
    }
    outcome.notes.push_back(std::move(unionNote));
    if (!input.cavities.empty()) {
        std::vector<int> kept;
        FixAllShellNote cavityNote;
        for (const ShellMergeCavity& cavity : input.cavities) {
            std::vector<Eigen::Index> rows = mapped(cavity.faces, faceMap);
            kept.push_back(after.of(rows));
            cavityNote.shells.push_back(std::move(rows));
        }
        cavityNote.text = std::string(kept.size() == 1 ? "Kept cavity " : "Kept cavities ") + shellList(kept) +
                          (kept.size() == 1 ? " as it was" : " as they were");
        outcome.notes.push_back(std::move(cavityNote));
        std::vector<int> before;
        for (int c = 0; c < static_cast<int>(input.cavities.size()); ++c) {
            before.push_back(numbers.of(input.cavities[static_cast<std::size_t>(c)].faces));
        }
        std::sort(before.begin(), before.end());
        outcome.detail += "; " + counted(before.size(), "cavity", "cavities") +
                          (before.size() == 1 ? " kept as it was: " : " kept as they were: ") + shellList(before);
    }

    outcome.changed = true;
    outcome.targetAfter = merged.unionShells;
    outcome.mesh = std::move(merged.mesh);
    outcome.faceMap = std::move(faceMap);
    return outcome;
}

[[nodiscard]] FixAllStage mergeGroupStage(MergeGroupJob job) {
    std::size_t faces = 0;
    for (const auto& operand : job.operands) {
        faces += operand.size();
    }
    FixAllStage stage;
    stage.id = FixAllStageId::MergeShells;
    stage.label = "Merge shells";
    stage.activity = "merging " + counted(job.operands.size(), "shell", "shells") + " (Manifold)";
    stage.progressWeight = static_cast<double>(faces);
    stage.run = [job = std::move(job)](const Mesh& mesh, const FixAllStageContext& context) {
        return mergeGroup(mesh, context, job);
    };
    return stage;
}

} // namespace

std::vector<FixAllStage> fixAllMergeStages(const MergePlan& plan) {
    std::vector<FixAllStage> stages;
    if (!plan.complete || plan.needsWholeShells) {
        return stages;
    }
    const auto shell = [&](int s) -> std::vector<Eigen::Index> {
        return s >= 0 && static_cast<std::size_t>(s) < plan.shells.size() ? plan.shells[static_cast<std::size_t>(s)]
                                                                         : std::vector<Eigen::Index>{};
    };
    if (!plan.isolatedNestedSolids.empty()) {
        std::vector<NestedSolidJob> jobs;
        for (const MergePlanNestedSolid& nested : plan.isolatedNestedSolids) {
            jobs.push_back({shell(nested.shell), shell(nested.container)});
        }
        stages.push_back(deleteNestedSolidsStage(std::move(jobs)));
    }
    for (const MergePlanGroup& group : plan.groups) {
        MergeGroupJob job;
        for (const int s : group.shells) {
            job.operands.push_back(shell(s));
        }
        for (const int c : group.cavities) {
            job.cavities.push_back(shell(c));
            const int container = static_cast<std::size_t>(c) < plan.relations.shells.size()
                                      ? plan.relations.shells[static_cast<std::size_t>(c)].nestedIn
                                      : -1;
            const auto it = std::find(group.shells.begin(), group.shells.end(), container);
            job.cavityContainers.push_back(it == group.shells.end() ? -1 : static_cast<int>(it - group.shells.begin()));
        }
        stages.push_back(mergeGroupStage(std::move(job)));
    }
    return stages;
}

std::string mergeVolumeProblem(std::span<const double> solidVolumes, double unionVolume,
                               std::span<const double> cavityVolumes, double signedTotal) {
    double sum = 0.0;
    double largest = 0.0;
    for (const double v : solidVolumes) {
        sum += v;
        largest = std::max(largest, v);
    }
    double hollow = 0.0;
    for (const double v : cavityVolumes) {
        hollow += std::abs(v);
    }
    const double scale = std::max({std::abs(sum), largest, std::abs(unionVolume)});
    const double tolerance = settings::kFixAllMergeVolumeRelTolerance * scale;
    if (!std::isfinite(unionVolume) || unionVolume < largest - tolerance) {
        return "The union's volume (" + number(unionVolume) + ") is below the largest shell's (" + number(largest) +
               "): a shell may be inside out or misread, so nothing was merged.";
    }
    if (unionVolume > sum + tolerance) {
        return "The union's volume (" + number(unionVolume) + ") is above the sum of the shells' volumes (" + number(sum) +
               "): a shell may be inside out or misread, so nothing was merged.";
    }
    const double expected = unionVolume - hollow;
    if (!std::isfinite(signedTotal) || std::abs(signedTotal - expected) > tolerance) {
        return "The merged shells' signed volume (" + number(signedTotal) + ") is not the union's less its cavities (" +
               number(expected) + "), so nothing was merged.";
    }
    return {};
}

} // namespace ogun
