#pragma once

#include "mesh/Mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace ogun {

/// Whole-mesh (or selected-faces) clean-up operations: Mesh > Clean Up > Weld / Merge Vertices and
/// Mesh > Simplify > Decimate. `faceInScope` (one flag per face, empty = every face) limits the work
/// to a region; faces outside it keep their vertices and positions exactly.

struct MergeByDistanceResult {
    bool success = false;
    Mesh mesh;
    std::size_t verticesMerged = 0;   ///< Vertices snapped onto another one and removed
    std::size_t degenerateRemoved = 0; ///< Faces that collapsed (two corners merged)
    std::size_t duplicatesRemoved = 0; ///< Faces that became copies of another face
    double maxMove = 0.0;             ///< Largest distance a merged vertex moved
    std::string statusMessage;
};

/// Merges every vertex of the faces in scope that lies within `distance` of an earlier one (in index
/// order, so no vertex moves more than `distance`: chains are not followed). Faces that collapse are
/// removed, as are faces that become copies of another (both, when the copies face opposite ways).
[[nodiscard]] MergeByDistanceResult mergeByDistance(const Mesh& mesh, double distance,
                                                    std::span<const std::uint8_t> faceInScope = {});

struct DecimateResult {
    bool success = false;
    Mesh mesh;
    std::size_t facesBefore = 0; ///< Faces in scope before
    std::size_t facesAfter = 0;  ///< Faces that replace them
    std::string statusMessage;
};

/// Garland-Heckbert edge collapse of the faces in scope down to `ratio` of their count (0 < ratio <= 1).
/// Border edges (holes and the edge of the region) and every vertex shared with a face outside the
/// scope stay fixed, so the result joins the rest of the mesh exactly. Fails on a non-manifold region.
[[nodiscard]] DecimateResult decimate(const Mesh& mesh, double ratio, std::span<const std::uint8_t> faceInScope = {});

} // namespace ogun
