#include "repair/GlobalRepair.hpp"

#include "diagnostics/SelfIntersections.hpp"
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
    case GlobalRepairMethod::PatchIslands:
        return "Patch each island";
    case GlobalRepairMethod::SelfUnion:
        return "Self-union";
    case GlobalRepairMethod::WindingRebuild:
        return "Winding-number rebuild";
    case GlobalRepairMethod::AlphaWrap:
        return "Alpha wrap";
    }
    return "?";
}

bool GlobalRepairParams::operator==(const GlobalRepairParams& other) const noexcept {
    return unionCrossingShells == other.unionCrossingShells && alphaWrapDetail == other.alphaWrapDetail &&
           alphaWrapOffsetRatio == other.alphaWrapOffsetRatio && windingResolution == other.windingResolution &&
           simplifyRebuild == other.simplifyRebuild && islandSettings.dilationRings == other.islandSettings.dilationRings &&
           islandSettings.cgalMaxSteps == other.islandSettings.cgalMaxSteps &&
           checkParams.intersectionTolerance == other.checkParams.intersectionTolerance;
}

RepairCandidate runGlobalRepair(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    RepairCandidate candidate;
    candidate.method = method;
    candidate.params = params;
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

    const auto start = std::chrono::steady_clock::now();
    Mesh repaired;
    std::string status;
    const double diag = scoped.boundingBox().diagonal();
    switch (method) {
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
        const double alpha = diag / params.alphaWrapDetail;
        repaired = alphaWrap(scoped, alpha, alpha / params.alphaWrapOffsetRatio, &status);
        break;
    }
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
