#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ogun {

/// Whole-mesh (or selected-faces) clean-up operations: Mesh > Modify > Merge by Distance,
/// Mesh > Modify > Weld, and Mesh > Simplify > Decimate. `faceInScope` (one flag per face,
/// empty = every face) limits the work to a region; faces outside it keep their vertices and positions exactly.

struct MergeByDistanceResult {
    bool success = false;
    Mesh mesh;
    std::size_t verticesMerged = 0;   ///< Vertices snapped onto another one and removed
    std::size_t degenerateRemoved = 0; ///< Faces that collapsed (two corners merged)
    std::size_t duplicatesRemoved = 0; ///< Faces that became copies of another face
    std::size_t pinnedPairsKept = 0;  ///< Matching pairs left apart because both vertices are pinned
    double maxMove = 0.0;             ///< Largest straight-line distance a merged vertex moved
    double maxTolerance = 0.0;        ///< Widest per-axis tolerance among the candidates (T_max); the distance itself for the straight-line form
    std::vector<Eigen::Index> faceMap;   ///< Input face -> output face, -1 = removed (kept faces keep their order)
    std::vector<Eigen::Index> vertexMap; ///< Input vertex -> output vertex it became (a merged one maps to its target), -1 = no face uses it
    std::string statusMessage;
};

/// Merges every vertex of the faces in scope that lies within `distance` of an earlier one (in index
/// order, so no vertex moves more than `distance`: chains are not followed). Faces that collapse are
/// removed, as are faces that become copies of another (both, when the copies face opposite ways).
/// With `keepSurfacesApart`, two vertices only merge when an edge joins them (a short edge collapses)
/// or both lie on open edges (a crack is welded): surfaces and shells that touch are not pinched.
/// With `preserveOpenEdges`, vertices across open edges / cracks do not merge (preserving boundary seams).
/// `pinned` (one flag per vertex, empty = none): a pinned vertex may absorb others but never moves or
/// disappears; pinned vertices take their turn first. Vertices with a non-finite coordinate are skipped.
/// The output keeps only the vertices faces use (see `vertexMap`).
[[nodiscard]] MergeByDistanceResult mergeByDistance(const Mesh& mesh, double distance,
                                                    std::span<const std::uint8_t> faceInScope = {},
                                                    bool keepSurfacesApart = false,
                                                    std::span<const std::uint8_t> pinned = {},
                                                    bool preserveOpenEdges = false);

/// Welds vertices on open boundary edges across cracks/gaps within `distance`. Interior vertices and
/// triangles are never altered or collapsed.
[[nodiscard]] MergeByDistanceResult weldOpenEdges(const Mesh& mesh, double distance,
                                                  std::span<const std::uint8_t> faceInScope = {},
                                                  std::span<const std::uint8_t> pinned = {});

/// The same merge with a per-axis, coordinate-relative tolerance (Fix All's weld): two vertices p and
/// q match when on every axis |p_i - q_i| <= max(tol.absFloor, tol.relTol x max(|p_i|, |q_i|))
/// (`tol.verticesMatchWithinTolerance`), so a merged vertex moves at most that on each axis (up to
/// sqrt(3) x it in a straight line). The search grid is T_max = max(absFloor, relTol x M) wide, M the
/// largest |coordinate| of a candidate, which keeps the 27-cell search exact. Unlike the straight-line
/// form, faces outside `faceInScope` stay bit-identical: every vertex they use is pinned (it may still
/// absorb an in-scope vertex, closing a crack along the scope's edge, even when no in-scope face uses
/// it; with surfaces kept apart, only its open-edge vertices), and such a face is never removed as a
/// copy (the in-scope copies go instead). Needs absFloor, relTol >= 0, not both 0.
[[nodiscard]] MergeByDistanceResult mergeByDistance(const Mesh& mesh, const ToleranceConfig& tol,
                                                    std::span<const std::uint8_t> faceInScope = {},
                                                    bool keepSurfacesApart = false,
                                                    std::span<const std::uint8_t> pinned = {},
                                                    bool preserveOpenEdges = false);

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
