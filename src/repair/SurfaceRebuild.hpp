#pragma once

#include "mesh/Mesh.hpp"

#include <cstddef>
#include <string>

namespace ogun {

/// Whole-surface operations behind the global intersection repairs. Each takes a mesh (any number
/// of shells, crossings allowed) and returns a new mesh; `statusOut` explains failures.

/// Self-union that keeps every uncrossed original triangle: CGAL autorefinement splits the
/// triangles along all crossings (with snap rounding back to doubles), then the generalized winding
/// number of the input on both sides of each piece keeps only the pieces between inside
/// (w >= 0.5) and outside. Internal walls, flaps and pockets are dropped; crossing shells merge.
/// Returns an empty mesh on failure.
[[nodiscard]] Mesh selfUnion(const Mesh& mesh, std::string* statusOut = nullptr);

/// `CGAL::alpha_wrap_3` around the triangles: always watertight, 2-manifold and free of crossings,
/// but a brand-new surface. `alpha` sets the detail (smaller = tighter), `offset` the distance of
/// the wrap from the input; both in mesh units.
[[nodiscard]] Mesh alphaWrap(const Mesh& mesh, double alpha, double offset, std::string* statusOut = nullptr);

/// Marching cubes of the generalized winding number (w = 0.5) of the input on a regular grid with
/// `resolution` cells along the longest side. A new surface that unions overlapping parts and
/// closes small holes.
[[nodiscard]] Mesh windingNumberRebuild(const Mesh& mesh, int resolution, std::string* statusOut = nullptr);

/// Garland-Heckbert edge-collapse simplification down to at most `targetFaces` faces. Returns the
/// input unchanged when it is already small enough or cannot be converted.
[[nodiscard]] Mesh simplifyToFaceCount(const Mesh& mesh, std::size_t targetFaces);

} // namespace ogun
