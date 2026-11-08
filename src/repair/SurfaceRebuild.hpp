#pragma once

#include "mesh/Mesh.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <stop_token>
#include <string>

namespace mesheal {

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
/// the wrap from the input; both in mesh units. Cancellable: `stopToken` is polled before every step of
/// the wrap's flood fill (through CGAL's wrapping visitor); a stopped wrap returns an empty mesh with the
/// status "Cancelled.". `onStep` (tests) is called with the step number before each poll.
[[nodiscard]] Mesh alphaWrap(const Mesh& mesh, double alpha, double offset, std::string* statusOut = nullptr,
                             std::stop_token stopToken = {}, const std::function<void(std::size_t)>& onStep = {});

/// Marching cubes of the generalized winding number (w = 0.5) of the input on a regular grid with
/// `resolution` cells along the longest side. A new surface that unions overlapping parts and
/// closes small holes.
[[nodiscard]] Mesh windingNumberRebuild(const Mesh& mesh, int resolution, std::string* statusOut = nullptr);

/// Returns the clamped Marching Cubes grid dimensions `(nx, ny, nz)` used by `voxelHealAndRemesh`
/// for a mesh with `bbox` at `targetEdgeLength`.
[[nodiscard]] std::array<int, 3> estimateVoxelGridDimensions(const BoundingBox& bbox, double targetEdgeLength) noexcept;

/// Pipeline A — Voxel Heal & Remesh (Fast):
/// Evaluates the generalized winding number (`w = 0.5`) on an off-lattice voxel grid sized from
/// `targetEdgeLength`, extracts the Marching Cubes isosurface, applies `smoothingPasses` passes of
/// volume-preserving Taubin (`lambda = 0.50, mu = -0.53`) de-terracing, and runs CGAL isotropic
/// remeshing at `targetEdgeLength`.
[[nodiscard]] Mesh voxelHealAndRemesh(const Mesh& mesh, double targetEdgeLength, int smoothingPasses,
                                      std::string* statusOut = nullptr, std::stop_token stopToken = {});

/// Pipeline B — Precision Isosurface (FEA Grade):
/// Meshes the exact `w = 0.5` winding-number level set via `CGAL::make_mesh_3` (`surface_only()`,
/// `manifold()`) with Delaunay ball radius `targetEdgeLength / sqrt(3)` and chord distance
/// `targetEdgeLength * chordRatio`, followed by `smoothingPasses` passes of tangential relaxation
/// and valence-improving flips.
[[nodiscard]] Mesh precisionIsosurfaceRemesh(const Mesh& mesh, double targetEdgeLength, double chordRatio,
                                             int smoothingPasses, std::string* statusOut = nullptr,
                                             std::stop_token stopToken = {});

/// Garland-Heckbert edge-collapse simplification down to at most `targetFaces` faces. Returns the
/// input unchanged when it is already small enough or cannot be converted.
[[nodiscard]] Mesh simplifyToFaceCount(const Mesh& mesh, std::size_t targetFaces);

} // namespace mesheal
