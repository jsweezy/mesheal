#pragma once

#include <Eigen/Core>

#include <array>

namespace mesheal {

using Triangle3d = std::array<Eigen::Vector3d, 3>;

struct TriTriHit {
    bool hit = false;
    bool coplanar = false; ///< The triangles lie in one plane (within the tolerance)
    /// A non-coplanar hit where one triangle only rests on the other: it lies on one side of the
    /// other's plane (within the tolerance), so an edge or corner lies on the other face. Not a crossing.
    bool touching = false;
};

/// Möller's triangle-triangle test ("A Fast Triangle-Triangle Intersection Test", 1997).
///
/// Reports a hit only when the triangles overlap by more than `tol` (a length): crossing triangles
/// share a segment longer than `tol`, and coplanar triangles overlap in area. Touching at a point or
/// along a shared edge is not a hit. `sharedVertices` is how many vertex indices the two faces share:
/// with 2 (an edge) only a coplanar fold-over can be a hit. Degenerate triangles never hit.
[[nodiscard]] TriTriHit intersectTriangles(const Triangle3d& a, const Triangle3d& b, int sharedVertices, double tol);

/// Smallest distance between two triangles (0 when they touch or cross). Degenerate triangles are
/// measured by their corners and edges.
[[nodiscard]] double triangleDistance(const Triangle3d& a, const Triangle3d& b);

/// True when the triangles come within `tol` of each other (`triangleDistance(a, b) <= tol`, with a
/// cheaper plane test first).
[[nodiscard]] bool trianglesWithin(const Triangle3d& a, const Triangle3d& b, double tol);

} // namespace mesheal
