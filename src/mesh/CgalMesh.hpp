#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace ogun {

using CgalKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint3 = CgalKernel::Point_3;
using CgalVector3 = CgalKernel::Vector_3;
using CgalSurfaceMesh = CGAL::Surface_mesh<CgalPoint3>;
using CgalVertexIndex = CgalSurfaceMesh::Vertex_index;
using CgalHalfedgeIndex = CgalSurfaceMesh::Halfedge_index;
using CgalEdgeIndex = CgalSurfaceMesh::Edge_index;
using CgalFaceIndex = CgalSurfaceMesh::Face_index;

/// Builds a halfedge mesh from `mesh`. Vertices and faces keep their indices when the
/// input is 2-manifold; otherwise falls back to polygon-soup orientation/repair.
/// Returns false if `mesh` is empty or the conversion fails.
bool toCgalSurfaceMesh(const Mesh& mesh, CgalSurfaceMesh& outSurfaceMesh);

/// Converts `surfaceMesh` into a contiguous `Mesh` (lazily deleted elements are skipped,
/// so `collect_garbage()` is not required). Each vertex is displaced along its vertex
/// normal by `normalOffset` (used to avoid Z-fighting on preview patches).
[[nodiscard]] Mesh fromCgalSurfaceMesh(
    const CgalSurfaceMesh& surfaceMesh,
    Unit unit = Unit::Millimeter,
    double normalOffset = 0.0);

/// Copies the `numRings`-ring face neighborhood around `centerFace` into `outPatch`.
/// Returns the center face's index inside `outPatch`, or `null_face()` on failure.
CgalFaceIndex extractLocalPatch(
    const CgalSurfaceMesh& baseMesh,
    CgalFaceIndex centerFace,
    int numRings,
    CgalSurfaceMesh& outPatch);

} // namespace ogun

