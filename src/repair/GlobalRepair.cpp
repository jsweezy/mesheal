#include "repair/GlobalRepair.hpp"

#include "diagnostics/SelfIntersections.hpp"
#include "mesh/Units.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/SurfaceRebuild.hpp"

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
    } else {
        MergeByDistanceResult r = mergeByDistance(mesh, params.mergeDistance, faceInScope, params.mergeKeepSurfacesApart);
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
    FixAllParams params = candidate.params.fixAll;
    params.checkParams = candidate.params.checkParams;
    params.holeFill = candidate.params.holeSettings;
    params.islandRepair = candidate.params.islandSettings;

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

    const FaceDiff diff = faceDiff(mesh, result.mesh);
    candidate.changedFaces = diff.added;
    candidate.removedFaces = diff.removed;
    candidate.vertexDeviation = vertexDeviation(result.mesh, mesh);
    // Metrics of the faces in scope: the input's scope before, the carried scope after
    const Mesh scopedBefore = extractFaces(mesh, flaggedFaces(mesh, scope.faceInScope));
    const Mesh scopedAfter = extractFaces(result.mesh, flaggedFaces(result.mesh, result.scope.faceInScope));
    RepairMetrics& m = candidate.metrics;
    m = computeRepairGeometryMetrics(scopedBefore, scopedAfter, stopToken);
    const FixAllCounts& before = result.countsBefore;
    const FixAllCounts& after = result.countsAfter;
    m.shellsBefore = result.report.shellsBefore;
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
    m.elapsedMs = result.elapsedMs;
    if (stopToken.stop_requested()) {
        candidate.cancelled = true;
        candidate.statusMessage = "Cancelled.";
        return candidate;
    }

    auto data = std::make_shared<FixAllCandidate>();
    data->scopeKind = kind;
    data->onePassRequested = params.passes <= 1;
    data->faceShell.assign(static_cast<std::size_t>(result.mesh.numFaces()), -1);
    for (std::size_t s = 0; s < result.report.shells.size(); ++s) {
        for (const Eigen::Index f : result.report.shells[s]) {
            data->faceShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }
    candidate.statusMessage = result.report.passesLine + ". " + result.report.leftLine;
    candidate.mesh = std::move(result.mesh);
    result.mesh = Mesh{};
    data->result = std::move(result);
    candidate.fixAll = std::move(data);
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
        repaired = alphaWrap(scoped, alpha, alphaWrapOffset(params, alpha), &status);
        break;
    }
    case GlobalRepairMethod::Decimate:
    case GlobalRepairMethod::MergeVertices:
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

} // namespace ogun
