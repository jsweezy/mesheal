#pragma once

#include "mesh/Mesh.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace mesheal {

/// Outcome of a vertex split. Coordinates never move: split vertices get copies at the same position.
struct NonManifoldSplitResult {
    Mesh mesh;
    std::size_t verticesSplit = 0; ///< Original vertices that got copies
    std::size_t verticesAdded = 0; ///< Copies appended to `mesh.V`
    std::size_t edgesSplit = 0;    ///< Touching edges separated (the whole seam of each requested edge)
    std::size_t edgesSkipped = 0;  ///< Requested edges that duplicating vertices cannot fix (fins, odd counts)
};

/// Gives every fan of triangles after the first at each listed vertex its own copy of the vertex.
[[nodiscard]] NonManifoldSplitResult splitNonManifoldVertices(const Mesh& mesh, std::span<const Eigen::Index> vertices);

/// Separates solids touching along the listed edges (sorted vertex pairs): the faces around each edge
/// are paired into closed wedges (`pairTouchingFaces`) and the vertices of the edge's whole seam (the
/// chain of touching edges it lies on) are split by fan, so no pinched vertex is left at a seam end.
/// Edges that are not Touching (`touchingEdges`) are left unchanged and counted in `edgesSkipped`.
[[nodiscard]] NonManifoldSplitResult splitNonManifoldEdges(const Mesh& mesh, std::span<const std::array<int, 2>> edges);

} // namespace mesheal
