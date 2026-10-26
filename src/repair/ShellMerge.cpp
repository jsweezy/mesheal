#include "repair/ShellMerge.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "repair/RepairMetrics.hpp"

#include <manifold/manifold.h>

#include <chrono>
#include <cstdint>
#include <utility>

namespace ogun {

namespace {

[[nodiscard]] const char* manifoldErrorText(manifold::Manifold::Error error) noexcept {
    switch (error) {
    case manifold::Manifold::Error::NoError:
        return "no error";
    case manifold::Manifold::Error::NonFiniteVertex:
        return "a vertex is not finite";
    case manifold::Manifold::Error::NotManifold:
        return "it is not closed and consistently oriented (open edges, flipped faces or fins)";
    default:
        return "Manifold could not read it";
    }
}

} // namespace

ShellMergeResult mergeShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shells) {
    const auto start = std::chrono::steady_clock::now();
    ShellMergeResult result;
    result.shellsIn = shells.size();
    if (shells.size() < 2) {
        result.message = "Select two or more shells to merge.";
        return result;
    }

    std::vector<std::uint8_t> merged(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::vector<manifold::Manifold> solids;
    solids.reserve(shells.size());
    Mesh inputs;
    inputs.fileUnit = mesh.fileUnit;
    for (std::size_t s = 0; s < shells.size(); ++s) {
        const Mesh shell = extractFaces(mesh, shells[s]);
        manifold::Manifold solid(shell.toMeshGL64());
        if (solid.Status() != manifold::Manifold::Error::NoError) {
            result.rejectedShell = static_cast<int>(s);
            result.message = std::string("Manifold cannot merge this shell: ") + manifoldErrorText(solid.Status()) + ".";
            return result;
        }
        result.volumeBefore += solid.Volume();
        result.areaBefore += solid.SurfaceArea();
        solids.push_back(std::move(solid));
        appendMesh(inputs, shell);
        for (const Eigen::Index f : shells[s]) {
            merged[static_cast<std::size_t>(f)] = 1;
        }
    }

    const manifold::Manifold united = manifold::Manifold::BatchBoolean(solids, manifold::OpType::Add);
    if (united.Status() != manifold::Manifold::Error::NoError) {
        result.message = std::string("The union failed: ") + manifoldErrorText(united.Status()) + ".";
        return result;
    }
    Mesh unionMesh = Mesh::fromMeshGL64(united.GetMeshGL64(), mesh.fileUnit);
    result.volumeAfter = united.Volume();
    result.areaAfter = united.SurfaceArea();
    result.shellsOut = checkShells(unionMesh).infoCount;
    if (inputs.numFaces() > 0) {
        const FaceDiff diff = faceDiff(inputs, unionMesh);
        result.unchangedFraction = 1.0 - static_cast<double>(diff.removed.size()) / static_cast<double>(inputs.numFaces());
    }

    std::vector<Eigen::Index> kept;
    kept.reserve(merged.size());
    for (std::size_t f = 0; f < merged.size(); ++f) {
        if (merged[f] == 0) {
            kept.push_back(static_cast<Eigen::Index>(f));
        }
    }
    result.mesh = extractFaces(mesh, kept);
    result.firstMergedFace = result.mesh.numFaces();
    appendMesh(result.mesh, unionMesh);
    result.success = true;
    result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return result;
}

} // namespace ogun
