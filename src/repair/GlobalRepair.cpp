#include "repair/GlobalRepair.hpp"

#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "mesh/Units.hpp"
#include "repair/FixAllDetail.hpp"
#include "repair/MergePlan.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/SurfaceRebuild.hpp"

#include <algorithm>
#include <chrono>
#include <unordered_set>

namespace ogun {

namespace {

void report(std::atomic<float>* progress, float value) {
    if (progress != nullptr) {
        progress->store(value);
    }
}

/// Decimate / merge: edit the faces in scope inside the whole mesh, so the scope may be any faces.
RepairCandidate runInPlace(const Mesh& mesh, std::span<const std::uint8_t> faceInScope, RepairCandidate candidate,
                           std::stop_token stopToken, std::atomic<float>* progress) {
    const GlobalRepairParams& params = candidate.params;
    const auto start = std::chrono::steady_clock::now();
    Mesh result;
    if (candidate.method == GlobalRepairMethod::Decimate) {
        DecimateResult r = decimate(mesh, params.decimateRatio, faceInScope);
        candidate.statusMessage = r.statusMessage;
        result = std::move(r.mesh);
    } else if (candidate.method == GlobalRepairMethod::MergeVertices) {
        MergeByDistanceResult r =
            mergeByDistance(mesh, params.mergeDistance, faceInScope, params.mergeKeepSurfacesApart, {}, params.mergePreserveOpenEdges);
        candidate.statusMessage = r.statusMessage;
        result = std::move(r.mesh);
    } else if (candidate.method == GlobalRepairMethod::Weld) {
        MergeByDistanceResult r = weldOpenEdges(mesh, params.weldDistance, faceInScope);
        candidate.statusMessage = r.statusMessage;
        result = std::move(r.mesh);
    }
    const double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    if (result.empty()) {
        return candidate;
    }
    report(progress, 0.7f);
    const FaceDiff diff = faceDiff(mesh, result);
    candidate.changedFaces = diff.added;
    candidate.removedFaces = diff.removed;
    candidate.vertexDeviation = vertexDeviation(result, mesh);
    report(progress, 0.8f);
    candidate.metrics = computeRepairMetrics(mesh, result, params.checkParams, stopToken);
    candidate.metrics.elapsedMs = elapsedMs;
    candidate.mesh = std::move(result);
    candidate.cancelled = stopToken.stop_requested();
    candidate.success = !candidate.cancelled;
    report(progress, 1.0f);
    return candidate;
}

/// Faces whose flag is set (an empty mask: every face).
std::vector<Eigen::Index> flaggedFaces(const Mesh& mesh, std::span<const std::uint8_t> flags) {
    std::vector<Eigen::Index> faces;
    faces.reserve(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (flags.empty() || (static_cast<std::size_t>(f) < flags.size() && flags[static_cast<std::size_t>(f)] != 0)) {
            faces.push_back(f);
        }
    }
    return faces;
}

/// The preview data and metrics of a Fix All `result` computed from `input` (scope `inputScope`): what
/// changed against the input, the deviation, the geometry metrics of the input's scope before and the
/// carried scope after, the counts `before` (the input's) and the result's, and the shells in scope.
void fillFixAllPreview(RepairCandidate& candidate, const Mesh& input, const FixAllScope& inputScope,
                       const FixAllResult& result, const FixAllCounts& before, std::size_t shellsBefore,
                       std::stop_token stopToken) {
    const FaceDiff diff = faceDiff(input, result.mesh);
    candidate.changedFaces = diff.added;
    candidate.removedFaces = diff.removed;
    candidate.vertexDeviation = vertexDeviation(result.mesh, input);
    const Mesh scopedBefore = extractFaces(input, flaggedFaces(input, inputScope.faceInScope));
    const Mesh scopedAfter = extractFaces(result.mesh, flaggedFaces(result.mesh, result.scope.faceInScope));
    RepairMetrics& m = candidate.metrics;
    m = computeRepairGeometryMetrics(scopedBefore, scopedAfter, stopToken);
    const FixAllCounts& after = result.countsAfter;
    m.shellsBefore = shellsBefore;
    m.shellsAfter = result.report.shellsAfter;
    m.openEdgesBefore = before.openEdges;
    m.openEdgesAfter = after.openEdges;
    m.nonManifoldEdgesBefore = before.nonManifoldEdges;
    m.nonManifoldEdgesAfter = after.nonManifoldEdges;
    m.nonManifoldVerticesBefore = before.nonManifoldVertices;
    m.nonManifoldVerticesAfter = after.nonManifoldVertices;
    m.flippedFacesBefore = before.flippedFaces;
    m.flippedFacesAfter = after.flippedFaces;
    m.invertedShellsBefore = before.invertedShells;
    m.invertedShellsAfter = after.invertedShells;
    m.intersectingFacesBefore = before.intersectingFaces;
    m.intersectingFacesAfter = after.intersectingFaces;
    m.degenerateFacesBefore = before.degenerateFaces;
    m.degenerateFacesAfter = after.degenerateFaces;
    m.duplicateFacesBefore = before.duplicateFaces;
    m.duplicateFacesAfter = after.duplicateFaces;
    m.thinFacesBefore = before.thinFaces;
    m.thinFacesAfter = after.thinFaces;
}

/// Moves `result`'s mesh into the candidate and wraps the rest of the run (with each face's report shell).
std::shared_ptr<const FixAllCandidate> takeFixAllData(RepairCandidate& candidate, FixAllResult result, FixAllScopeKind kind,
                                                      bool onePass, std::optional<FixAllShellRebuild> rebuild) {
    auto data = std::make_shared<FixAllCandidate>();
    data->scopeKind = kind;
    data->onePassRequested = onePass;
    data->faceShell.assign(static_cast<std::size_t>(result.mesh.numFaces()), -1);
    for (std::size_t s = 0; s < result.report.shells.size(); ++s) {
        for (const Eigen::Index f : result.report.shells[s]) {
            data->faceShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }
    candidate.mesh = std::move(result.mesh);
    result.mesh = Mesh{};
    data->result = std::move(result);
    data->rebuild = std::move(rebuild);
    return data;
}

/// Fix All on the whole mesh with the scope: the run's result, the preview data and the metrics of the
/// faces in scope before and after.
RepairCandidate runFixAllCandidate(const Mesh& mesh, std::span<const std::uint8_t> faceInScope, RepairCandidate candidate,
                                   std::stop_token stopToken, std::atomic<float>* progress, const FixAllRunInput* input) {
    const FixAllScopeKind kind = faceInScope.empty() ? FixAllScopeKind::Whole
                                 : input != nullptr  ? input->scopeKind
                                                     : FixAllScopeKind::Shells;
    const FixAllScope scope = FixAllScope::fromFaceMask(mesh, kind, faceInScope);
    if (scope.faceCount() == 0) {
        candidate.statusMessage = "No faces in scope.";
        return candidate;
    }
    const FixAllParams params = fixAllRunParams(candidate.params);

    FixAllProgress local;
    FixAllProgress* shared = input != nullptr ? input->progress : nullptr;
    if (shared == nullptr && progress != nullptr) {
        local.observer = [progress](float value, const std::string&) { progress->store(value); };
    }
    FixAllResult result = runFixAll(mesh, scope, params, stopToken, shared != nullptr ? shared : &local,
                                    input != nullptr ? input->hooks : nullptr);
    if (result.cancelled || stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    if (!result.success) {
        candidate.statusMessage = result.error.empty() ? std::string("Fix All failed.") : result.error;
        return candidate;
    }

    fillFixAllPreview(candidate, mesh, scope, result, result.countsBefore, result.report.shellsBefore, stopToken);
    candidate.metrics.elapsedMs = result.elapsedMs;
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    candidate.statusMessage = result.report.passesLine + ". " + result.report.leftLine;
    candidate.fixAll = takeFixAllData(candidate, std::move(result), kind, params.passes <= 1, std::nullopt);
    candidate.success = true;
    report(progress, 1.0f);
    return candidate;
}

/// Self-union of each shell that crosses itself; the others are copied as they are.
Mesh unitePerShell(const Mesh& mesh, const CheckParams& params, std::string& status) {
    const std::vector<int> faceShell = shellIdsOfFaces(mesh);
    const CategoryResult check = checkSelfIntersections(mesh, params);
    std::unordered_set<int> crossing;
    for (const IntersectionIsland& island : clusterIntersectionIslands(mesh, check, faceShell)) {
        crossing.insert(island.shell);
    }
    std::vector<std::vector<Eigen::Index>> shellFaces;
    std::vector<Eigen::Index> unchanged;
    for (std::size_t f = 0; f < faceShell.size(); ++f) {
        const int s = faceShell[f];
        if (s >= 0 && crossing.count(s) != 0) {
            if (static_cast<std::size_t>(s) >= shellFaces.size()) {
                shellFaces.resize(static_cast<std::size_t>(s) + 1);
            }
            shellFaces[static_cast<std::size_t>(s)].push_back(static_cast<Eigen::Index>(f));
        } else {
            unchanged.push_back(static_cast<Eigen::Index>(f));
        }
    }
    Mesh out = extractFaces(mesh, unchanged);
    std::size_t united = 0;
    for (const std::vector<Eigen::Index>& faces : shellFaces) {
        if (faces.empty()) {
            continue;
        }
        const Mesh shell = extractFaces(mesh, faces);
        std::string shellStatus;
        Mesh result = selfUnion(shell, &shellStatus);
        if (result.empty()) {
            appendMesh(out, shell);
            status = shellStatus;
        } else {
            appendMesh(out, result);
            ++united;
        }
    }
    status = "United " + std::to_string(united) + " of " + std::to_string(crossing.size()) + " self-crossing shells." +
             (status.empty() ? "" : " " + status);
    return out;
}

} // namespace

FixAllParams fixAllRunParams(const GlobalRepairParams& params) {
    FixAllParams run = params.fixAll;
    run.checkParams = params.checkParams;
    run.holeFill = params.holeSettings;
    run.islandRepair = params.islandSettings;
    return run;
}

const char* globalRepairMethodLabel(GlobalRepairMethod method) noexcept {
    switch (method) {
    case GlobalRepairMethod::FixAll:
        return "Fix All";
    case GlobalRepairMethod::FillHoles:
        return "Fill each hole";
    case GlobalRepairMethod::PatchIslands:
        return "Patch each island";
    case GlobalRepairMethod::SelfUnion:
        return "Self-union";
    case GlobalRepairMethod::WindingRebuild:
        return "Winding-number rebuild";
    case GlobalRepairMethod::AlphaWrap:
        return "Alpha wrap";
    case GlobalRepairMethod::Decimate:
        return "Decimate";
    case GlobalRepairMethod::MergeVertices:
        return "Merge by distance";
    case GlobalRepairMethod::Weld:
        return "Weld";
    }
    return "?";
}

std::span<const GlobalRepairMethod> globalRepairMethodsFor(DefectCategory target) noexcept {
    if (target == DefectCategory::OpenEdges) {
        return kHoleRepairMethods;
    }
    return kGlobalRepairMethods;
}

double alphaWrapAlpha(const GlobalRepairParams& params, double diag) noexcept {
    return diag / params.alphaWrapDetail;
}

double alphaWrapOffset(const GlobalRepairParams& params, double alpha) noexcept {
    const double ratio = params.alphaWrapOffsetRatio > 0.0 ? params.alphaWrapOffsetRatio : settings::kDefaultAlphaWrapOffsetRatio;
    return alpha / ratio;
}

bool GlobalRepairParams::operator==(const GlobalRepairParams& other) const noexcept {
    return unionCrossingShells == other.unionCrossingShells && alphaWrapDetail == other.alphaWrapDetail &&
           alphaWrapOffsetRatio == other.alphaWrapOffsetRatio && decimateRatio == other.decimateRatio &&
           mergeDistance == other.mergeDistance && mergeKeepSurfacesApart == other.mergeKeepSurfacesApart &&
           mergePreserveOpenEdges == other.mergePreserveOpenEdges && weldDistance == other.weldDistance &&
           windingResolution == other.windingResolution &&
           simplifyRebuild == other.simplifyRebuild && islandSettings.dilationRings == other.islandSettings.dilationRings &&
           islandSettings.cgalMaxSteps == other.islandSettings.cgalMaxSteps &&
           holeMethod == other.holeMethod && holeSettings == other.holeSettings &&
           checkParams.intersectionTolerance == other.checkParams.intersectionTolerance &&
           fixAll.passes == other.fixAll.passes && fixAll.timeBudgetSeconds == other.fixAll.timeBudgetSeconds &&
           fixAll.weld == other.fixAll.weld && fixAll.weldTolerance.absFloor == other.fixAll.weldTolerance.absFloor &&
           fixAll.weldTolerance.relTol == other.fixAll.weldTolerance.relTol &&
           fixAll.removeTinyShells == other.fixAll.removeTinyShells &&
           fixAll.tinyShellFraction == other.fixAll.tinyShellFraction && fixAll.removeFins == other.fixAll.removeFins &&
           fixAll.closeCracks == other.fixAll.closeCracks && fixAll.healThinFaces == other.fixAll.healThinFaces &&
           fixAll.mergeShells == other.fixAll.mergeShells;
}

RepairCandidate runGlobalRepair(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress,
    const FixAllRunInput* fixAll) {
    RepairCandidate candidate;
    candidate.method = method;
    candidate.params = params;
    if (method == GlobalRepairMethod::FixAll) {
        return runFixAllCandidate(mesh, faceInScope, std::move(candidate), stopToken, progress, fixAll);
    }
    candidate.mesh = mesh;

    std::vector<Eigen::Index> inFaces;
    std::vector<Eigen::Index> outFaces;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        (faceInScope.empty() || faceInScope[static_cast<std::size_t>(f)] ? inFaces : outFaces).push_back(f);
    }
    const Mesh scoped = extractFaces(mesh, inFaces);
    if (scoped.empty()) {
        candidate.statusMessage = "No faces in scope.";
        return candidate;
    }
    report(progress, 0.05f);
    if (editsInPlace(method)) {
        return runInPlace(mesh, faceInScope, std::move(candidate), stopToken, progress);
    }

    const auto start = std::chrono::steady_clock::now();
    Mesh repaired;
    std::string status;
    const double diag = scoped.boundingBox().diagonal();
    switch (method) {
    case GlobalRepairMethod::FillHoles: {
        const std::vector<HoleLoop> holes = findHoleLoops(scoped, params.holeSettings.shape);
        if (holes.empty()) {
            status = "No holes to fill.";
            break;
        }
        HoleBatchResult batch = fillHoles(scoped, holes, params.holeMethod, params.holeSettings, stopToken);
        status = batch.statusMessage;
        if (batch.success) {
            repaired = std::move(batch.mesh);
        }
        break;
    }
    case GlobalRepairMethod::PatchIslands: {
        IslandBatchResult batch = healIntersectionIslands(scoped, params.checkParams, {}, {}, params.islandSettings, stopToken);
        status = batch.statusMessage;
        if (batch.success) {
            repaired = std::move(batch.mesh);
        }
        break;
    }
    case GlobalRepairMethod::SelfUnion:
        repaired = params.unionCrossingShells ? selfUnion(scoped, &status) : unitePerShell(scoped, params.checkParams, status);
        break;
    case GlobalRepairMethod::WindingRebuild:
        repaired = windingNumberRebuild(scoped, params.windingResolution, &status);
        break;
    case GlobalRepairMethod::AlphaWrap: {
        const double alpha = alphaWrapAlpha(params, diag);
        repaired = alphaWrap(scoped, alpha, alphaWrapOffset(params, alpha), &status, stopToken);
        break;
    }
    case GlobalRepairMethod::Decimate:
    case GlobalRepairMethod::MergeVertices:
    case GlobalRepairMethod::Weld:
    case GlobalRepairMethod::FixAll:
        break; // runInPlace / runFixAllCandidate
    }
    if (params.simplifyRebuild && !repaired.empty() &&
        (method == GlobalRepairMethod::WindingRebuild || method == GlobalRepairMethod::AlphaWrap)) {
        repaired = simplifyToFaceCount(repaired, static_cast<std::size_t>(scoped.numFaces()));
        status += " Simplified to " + std::to_string(repaired.numFaces()) + " faces.";
    }
    const double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    candidate.statusMessage = status;
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    if (repaired.empty()) {
        if (candidate.statusMessage.empty()) {
            candidate.statusMessage = std::string(globalRepairMethodLabel(method)) + " produced no mesh.";
        }
        return candidate;
    }
    report(progress, 0.7f);

    Mesh assembled = extractFaces(mesh, outFaces);
    const Eigen::Index offset = assembled.numFaces();
    appendMesh(assembled, repaired);
    const FaceDiff diff = faceDiff(scoped, repaired);
    candidate.changedFaces.reserve(diff.added.size());
    for (const Eigen::Index f : diff.added) {
        candidate.changedFaces.push_back(offset + f);
    }
    candidate.removedFaces.reserve(diff.removed.size());
    for (const Eigen::Index f : diff.removed) {
        candidate.removedFaces.push_back(inFaces[static_cast<std::size_t>(f)]);
    }
    candidate.vertexDeviation = Eigen::VectorXd::Zero(assembled.numVertices());
    candidate.vertexDeviation.tail(repaired.numVertices()) = vertexDeviation(repaired, scoped);
    report(progress, 0.8f);
    candidate.metrics = computeRepairMetrics(scoped, repaired, params.checkParams, stopToken);
    candidate.metrics.elapsedMs = elapsedMs;
    candidate.mesh = std::move(assembled);
    candidate.cancelled = stopToken.stop_requested();
    candidate.success = !candidate.cancelled;
    report(progress, 1.0f);
    return candidate;
}

// -----------------------------------------------------------------------------
// Per-shell rebuild of a Fix All result (Rebuild shell #k...)
// -----------------------------------------------------------------------------

namespace {

/// What the rebuild's merge plan said about the rebuilt shell(s).
struct RebuildPlanFacts {
    bool planned = false;   ///< Phase 2 planned (the merge is on and whole shells are in scope)
    bool inGroup = false;   ///< A merge group holds a rebuilt shell
    bool nested = false;    ///< A rebuilt shell is a nested solid touching nothing (the merge deletes it)
    std::string notMerged;  ///< Why the plan does not merge it (skipped, refused group, stopped group)
};

/// "wrapped" / "united" / "rebuilt".
const char* rebuildVerb(GlobalRepairMethod method) noexcept {
    switch (method) {
    case GlobalRepairMethod::AlphaWrap:
        return "wrapped";
    case GlobalRepairMethod::SelfUnion:
        return "united";
    default:
        return "rebuilt";
    }
}

/// Keeps only what concerns the rebuilt shells (the shells holding a face at row `firstRebuilt` or later)
/// in `plan`: their groups, their nested-solid deletion, and the lines naming them. Other shells were
/// planned by Fix All already and stay exactly as they are.
void keepRebuiltShellsOnly(MergePlan& plan, Eigen::Index firstRebuilt, RebuildPlanFacts& facts) {
    facts.planned = true;
    std::vector<std::uint8_t> rebuilt(plan.shells.size(), 0);
    for (std::size_t s = 0; s < plan.shells.size(); ++s) {
        const std::vector<Eigen::Index>& faces = plan.shells[s];
        rebuilt[s] = static_cast<std::uint8_t>(
            std::any_of(faces.begin(), faces.end(), [&](Eigen::Index f) { return f >= firstRebuilt; }));
    }
    const auto isRebuilt = [&](int s) { return s >= 0 && static_cast<std::size_t>(s) < rebuilt.size() && rebuilt[static_cast<std::size_t>(s)] != 0; };
    const auto namesRebuilt = [&](const std::vector<int>& shells) { return std::any_of(shells.begin(), shells.end(), isRebuilt); };

    std::erase_if(plan.groups, [&](const MergePlanGroup& group) { return !namesRebuilt(group.shells); });
    std::erase_if(plan.isolatedNestedSolids, [&](const MergePlanNestedSolid& nested) { return !isRebuilt(nested.shell); });
    std::erase_if(plan.lines, [&](const MergePlanLine& line) { return !namesRebuilt(line.shells); });
    facts.inGroup = !plan.groups.empty();
    facts.nested = !plan.isolatedNestedSolids.empty();
    for (const MergePlanSkipped& skipped : plan.skipped) {
        if (isRebuilt(skipped.shell) && facts.notMerged.empty()) {
            facts.notMerged = skipped.reason;
        }
    }
    for (const MergePlanRefusedGroup& group : plan.refusedGroups) {
        if (namesRebuilt(group.shells) && facts.notMerged.empty()) {
            facts.notMerged = group.reason;
        }
    }
    for (const std::vector<int>& group : plan.stoppedGroups) {
        if (namesRebuilt(group) && facts.notMerged.empty()) {
            facts.notMerged = "it is in a group that looks inside out (see Inverted shells)";
        }
    }
}

/// The shell of `result` (report numbering) holding face `face`, -1 when none.
int reportShellOf(const FixAllResult& result, Eigen::Index face) {
    for (std::size_t s = 0; s < result.report.shells.size(); ++s) {
        const std::vector<Eigen::Index>& faces = result.report.shells[s];
        if (std::find(faces.begin(), faces.end(), face) != faces.end()) {
            return static_cast<int>(s);
        }
    }
    return -1;
}

} // namespace

RepairCandidate computeShellRebuildCandidate(
    const Mesh& input,
    std::span<const std::uint8_t> faceInScope,
    const RepairCandidate& fixAll,
    int shell,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    RepairCandidate candidate;
    candidate.method = method;
    candidate.params = params;
    const std::shared_ptr<const FixAllCandidate> base = fixAll.fixAll;
    if (!fixAll.success || base == nullptr || base->rebuild) {
        candidate.statusMessage = "Preview Fix All first: a rebuild row builds on its result.";
        return candidate;
    }
    if (std::find(kShellRebuildMethods.begin(), kShellRebuildMethods.end(), method) == kShellRebuildMethods.end()) {
        candidate.statusMessage = std::string(globalRepairMethodLabel(method)) + " is not a rebuild.";
        return candidate;
    }
    const FixAllResult& run = base->result;
    const FixAllFinalReport& finalReport = run.report;
    const Mesh& mesh = fixAll.mesh;
    const std::string number = "#" + std::to_string(shell + 1);
    if (shell < 0 || static_cast<std::size_t>(shell) >= finalReport.shells.size() ||
        finalReport.shells[static_cast<std::size_t>(shell)].empty() || !run.scope.matches(mesh)) {
        candidate.statusMessage = "There is no shell " + number + " in Fix All's result.";
        return candidate;
    }
    const std::vector<Eigen::Index>& faces = finalReport.shells[static_cast<std::size_t>(shell)];
    if (!std::all_of(faces.begin(), faces.end(), [&](Eigen::Index f) { return run.scope.inScope(f); })) {
        candidate.statusMessage = number + " is not wholly in Fix All's scope: rebuilding it would change faces outside "
                                           "the selection. Select whole shells, or clear the selection.";
        return candidate;
    }
    if (std::find(finalReport.cavitiesKept.begin(), finalReport.cavitiesKept.end(), shell) != finalReport.cavitiesKept.end()) {
        candidate.statusMessage = number + " is a cavity, a hollow kept on purpose: rebuilding it would fill the hollow.";
        return candidate;
    }

    // The rebuild of that shell alone: every other shell (a cavity inside it too) is kept exactly
    const auto start = std::chrono::steady_clock::now();
    report(progress, 0.05f);
    const Mesh shellMesh = extractFaces(mesh, faces);
    std::string status;
    Mesh rebuilt;
    switch (method) {
    case GlobalRepairMethod::AlphaWrap: {
        const double alpha = alphaWrapAlpha(params, shellMesh.boundingBox().diagonal());
        rebuilt = alphaWrap(shellMesh, alpha, alphaWrapOffset(params, alpha), &status, stopToken);
        break;
    }
    case GlobalRepairMethod::SelfUnion:
        rebuilt = selfUnion(shellMesh, &status);
        break;
    default:
        rebuilt = windingNumberRebuild(shellMesh, params.windingResolution, &status);
        break;
    }
    if (params.simplifyRebuild && !rebuilt.empty() && method != GlobalRepairMethod::SelfUnion) {
        rebuilt = simplifyToFaceCount(rebuilt, static_cast<std::size_t>(shellMesh.numFaces()));
    }
    const double rebuildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    if (rebuilt.empty()) {
        candidate.statusMessage = std::string(globalRepairMethodLabel(method)) + " of " + number + " produced no mesh" +
                                  (status.empty() ? std::string(".") : ": " + status);
        return candidate;
    }
    report(progress, 0.4f);

    // The Fix All result with the shell replaced (its faces removed, the rebuild appended), scope carried
    std::vector<std::uint8_t> inShell(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const Eigen::Index f : faces) {
        inShell[static_cast<std::size_t>(f)] = 1;
    }
    std::vector<Eigen::Index> keep;
    keep.reserve(static_cast<std::size_t>(mesh.numFaces()) - faces.size());
    std::vector<Eigen::Index> faceMap(static_cast<std::size_t>(mesh.numFaces()), -1);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (inShell[static_cast<std::size_t>(f)] == 0) {
            faceMap[static_cast<std::size_t>(f)] = static_cast<Eigen::Index>(keep.size());
            keep.push_back(f);
        }
    }
    Mesh assembled = extractFaces(mesh, keep);
    const Eigen::Index firstRebuilt = assembled.numFaces();
    appendMesh(assembled, rebuilt);
    FixAllScope scope = run.scope;
    scope.carry(faceMap, assembled);
    const fixall_detail::EdgeDefectCounts edges = fixall_detail::countEdgeDefects(rebuilt);
    const bool closed = edges.open == 0 && edges.nonManifold == 0;

    // One pass of phases 2 and 3 only: the merge of the rebuilt shell's groups and the clean-up
    FixAllParams runParams = fixAllRunParams(params);
    runParams.passes = 1;
    RebuildPlanFacts facts;
    FixAllTestHooks hooks;
    hooks.adjustStages = [](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (phase == FixAllPhase::Repair) {
            stages.clear();
        }
    };
    hooks.adjustPlan = [&facts, firstRebuilt](int, MergePlan& plan) { keepRebuiltShellsOnly(plan, firstRebuilt, facts); };
    FixAllProgress passProgress;
    if (progress != nullptr) {
        passProgress.observer = [progress](float value, const std::string&) { progress->store(0.4f + 0.55f * value); };
    }
    FixAllResult after = runFixAll(assembled, scope, runParams, stopToken, &passProgress, &hooks);
    if (after.cancelled || stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    if (!after.success) {
        candidate.statusMessage = after.error.empty() ? std::string("The merge after the rebuild failed.") : after.error;
        return candidate;
    }

    // What became of the rebuilt shell
    bool merged = false;
    std::string refusal;
    for (const FixAllStepReport& step : after.steps) {
        if (step.stage != FixAllStageId::MergeShells && step.stage != FixAllStageId::DeleteNestedSolids) {
            continue;
        }
        if (step.accepted()) {
            merged = true;
        } else if (step.status == FixAllStepStatus::Refused && refusal.empty()) {
            refusal = step.reason;
        }
    }
    const std::string verb = rebuildVerb(method);
    std::string note;
    if (!closed) {
        note = "still open";
        if (edges.open > 0) {
            note += " (" + fixall_detail::counted(edges.open, "open edge", "open edges") + ")";
        } else {
            note += " (" + fixall_detail::counted(edges.nonManifold, "non-manifold edge", "non-manifold edges") + ")";
        }
        note += runParams.mergeShells && scope.kind != FixAllScopeKind::Faces ? ": not merged" : "";
    } else if (!runParams.mergeShells) {
        note = verb + "; the merge is off";
    } else if (!facts.planned) {
        note = verb + "; the merge needs whole shells";
    } else if (merged) {
        note = verb + ", then merged" + (facts.nested && !facts.inGroup ? " (it lay inside another shell and touched nothing: deleted)" : "");
    } else {
        // Shells the rebuilt shell still crosses, in the row's numbering (its mesh is `assembled`)
        const int rebuiltShell = reportShellOf(after, after.mesh.numFaces() - 1);
        std::vector<int> crosses;
        for (const FixAllCrossingLeft& pair : after.report.crossingsLeft) {
            if (pair.a == rebuiltShell || pair.b == rebuiltShell) {
                crosses.push_back(pair.a == rebuiltShell ? pair.b : pair.a);
            }
        }
        std::sort(crosses.begin(), crosses.end());
        const std::string crossing = crosses.empty() ? std::string() : "still crosses " + fixall_detail::shellList(crosses) + ": ";
        if (!refusal.empty()) {
            note = verb + "; " + crossing + "merge refused (" + refusal + ")";
        } else if (!facts.notMerged.empty()) {
            note = verb + "; " + crossing + "not merged: " + facts.notMerged;
        } else {
            note = verb + "; it overlaps no other shell, so there is nothing to merge";
        }
    }

    FixAllShellRebuild rebuild;
    rebuild.base = base;
    rebuild.shell = shell;
    rebuild.suggested = method == GlobalRepairMethod::AlphaWrap;
    rebuild.closed = closed;
    rebuild.merged = merged;
    rebuild.rebuiltFaces = static_cast<std::size_t>(rebuilt.numFaces());
    rebuild.label = "+ " + std::string(globalRepairMethodLabel(method)) + " " + number;
    rebuild.note = note;
    rebuild.rebuildMs = rebuildMs;

    const FixAllScopeKind kind = faceInScope.empty() ? FixAllScopeKind::Whole : base->scopeKind;
    const FixAllScope inputScope = FixAllScope::fromFaceMask(input, kind, faceInScope);
    fillFixAllPreview(candidate, input, inputScope, after, run.countsBefore, finalReport.shellsBefore, stopToken);
    candidate.metrics.elapsedMs = rebuildMs + after.elapsedMs;
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }
    candidate.statusMessage = rebuild.label.substr(2) + ": " + note + ". " + after.report.leftLine;
    candidate.fixAll = takeFixAllData(candidate, std::move(after), kind, base->onePassRequested, std::move(rebuild));
    candidate.success = true;
    report(progress, 1.0f);
    return candidate;
}

std::vector<RepairCandidate> computeShellRebuildCandidates(
    const Mesh& input,
    std::span<const std::uint8_t> faceInScope,
    const RepairCandidate& fixAll,
    int shell,
    const GlobalRepairParams& params,
    std::stop_token stopToken) {
    std::vector<RepairCandidate> rows;
    rows.reserve(kShellRebuildMethods.size());
    for (const GlobalRepairMethod method : kShellRebuildMethods) {
        if (stopToken.stop_requested()) {
            break;
        }
        rows.push_back(computeShellRebuildCandidate(input, faceInScope, fixAll, shell, method, params, stopToken));
    }
    return rows;
}

std::optional<std::vector<int>> cavitiesFilledByRebuild(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    double intersectionTolerance,
    std::stop_token stopToken) {
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    const std::vector<int> needed = shellsRuleONeeds(describeShells(mesh, shells));
    std::vector<int> cavities;
    if (needed.empty()) {
        return cavities; // nothing is wound inward
    }
    ShellRelationParams relationParams;
    relationParams.intersectionTolerance = intersectionTolerance;
    relationParams.involving = needed;
    const ShellRelations relations = computeShellRelations(mesh, shells, relationParams, stopToken);
    if (!relations.complete || stopToken.stop_requested()) {
        return std::nullopt;
    }
    const ShellRoles roles = classifyShellRoles(mesh, relations);
    for (std::size_t s = 0; s < roles.shells.size() && s < shells.size(); ++s) {
        if (roles.shells[s].role != ShellRole::Cavity) {
            continue;
        }
        const bool inScope = faceInScope.empty() || std::any_of(shells[s].begin(), shells[s].end(), [&](Eigen::Index f) {
                                 return static_cast<std::size_t>(f) < faceInScope.size() && faceInScope[static_cast<std::size_t>(f)] != 0;
                             });
        if (inScope) {
            cavities.push_back(static_cast<int>(s));
        }
    }
    return cavities;
}

} // namespace ogun
