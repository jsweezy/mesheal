#pragma once

#include <Eigen/Core>

#include <array>

namespace ogun {

using Triangle3d = std::array<Eigen::Vector3d, 3>;

struct TriTriHit {
    bool hit = false;
    bool coplanar = false; ///< The triangles lie in one plane (within the tolerance)
};

/// Möller's triangle-triangle test ("A Fast Triangle-Triangle Intersection Test", 1997).
///
/// Reports a hit only when the triangles overlap by more than `tol` (a length): crossing triangles
/// share a segment longer than `tol`, and coplanar triangles overlap in area. Touching at a point or
/// along a shared edge is not a hit. `sharedVertices` is how many vertex indices the two faces share:
/// with 2 (an edge) only a coplanar fold-over can be a hit. Degenerate triangles never hit.
[[nodiscard]] TriTriHit intersectTriangles(const Triangle3d& a, const Triangle3d& b, int sharedVertices, double tol);

} // namespace ogun
