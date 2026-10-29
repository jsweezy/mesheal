#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <stop_token>
#include <vector>

namespace ogun {

/// Before / after numbers shown next to every repair candidate, so the user can judge a coarse
/// global fix against a surgical one (VISION.md "options plus information").
struct RepairMetrics {
    Eigen::Index facesBefore = 0;
    Eigen::Index facesAfter = 0;
    std::size_t shellsBefore = 0;
    std::size_t shellsAfter = 0;
    double volumeBefore = 0.0;  ///< Sum of signed shell volumes (overlapping shells count twice)
    double volumeAfter = 0.0;
    double areaBefore = 0.0;
    double areaAfter = 0.0;
    double maxDeviation = 0.0;  ///< Farthest result point from the original surface
    double maxLost = 0.0;       ///< Farthest original point from the result (removed walls, filled pockets)
    std::size_t intersectingFacesBefore = 0;
    std::size_t intersectingFacesAfter = 0;
    std::size_t thinFacesBefore = 0;
    std::size_t thinFacesAfter = 0;
    std::size_t openEdgesBefore = 0;
    std::size_t openEdgesAfter = 0;
    std::size_t nonManifoldEdgesBefore = 0;
    std::size_t nonManifoldEdgesAfter = 0;
    std::size_t nonManifoldVerticesBefore = 0;
    std::size_t nonManifoldVerticesAfter = 0;
    std::size_t flippedFacesBefore = 0;      ///< Faces wound against their shell's majority
    std::size_t flippedFacesAfter = 0;
    std::size_t invertedShellsBefore = 0;    ///< Inside-out closed shells (rule O, as the Inverted shells row)
    std::size_t invertedShellsAfter = 0;
    std::size_t degenerateFacesBefore = 0;   ///< Repeated vertex or no reliable normal
    std::size_t degenerateFacesAfter = 0;
    std::size_t duplicateFacesBefore = 0;
    std::size_t duplicateFacesAfter = 0;
    double unchangedFraction = 0.0;  ///< Share of original faces that survive bit-identical
    double elapsedMs = 0.0;          ///< Run time of the repair itself (not of the metrics)

    [[nodiscard]] bool watertight() const noexcept {
        return openEdgesAfter == 0 && nonManifoldEdgesAfter == 0;
    }
    [[nodiscard]] double volumeChangePercent() const noexcept;
    [[nodiscard]] double areaChangePercent() const noexcept;
};

/// Faces of `before` that are missing from `after` and faces of `after` that are new. Faces are
/// matched by their three corner positions, so any re-indexing, reordering or vertex renumbering by
/// CGAL or libigl is ignored and only real geometric change is reported.
struct FaceDiff {
    std::vector<Eigen::Index> removed;  ///< Indices into `before`
    std::vector<Eigen::Index> added;    ///< Indices into `after`
};

[[nodiscard]] FaceDiff faceDiff(const Mesh& before, const Mesh& after);

/// Corner positions of face `f`, rotated so the smallest corner comes first (keeps the winding).
/// Equal keys mean the same triangle, whatever the vertex or face numbering.
using FaceGeometryKey = std::array<double, 9>;

struct FaceGeometryKeyHash {
    [[nodiscard]] std::size_t operator()(const FaceGeometryKey& key) const noexcept;
};

[[nodiscard]] FaceGeometryKey faceGeometryKey(const Mesh& mesh, Eigen::Index f);

/// Sum over faces of the signed tetrahedron volumes against the origin.
[[nodiscard]] double signedVolume(const Mesh& mesh) noexcept;

[[nodiscard]] double surfaceArea(const Mesh& mesh) noexcept;

/// Distance from each vertex of `mesh` to the surface of `reference` (for "Color by deviation").
[[nodiscard]] Eigen::VectorXd vertexDeviation(const Mesh& mesh, const Mesh& reference);

/// Fills every field except `elapsedMs`. `params` gives the intersection tolerance and thin-face
/// thresholds used for the before / after counts. Cancel through `stopToken` leaves the counts at 0.
[[nodiscard]] RepairMetrics computeRepairMetrics(
    const Mesh& before,
    const Mesh& after,
    const CheckParams& params,
    std::stop_token stopToken = {});

} // namespace ogun
