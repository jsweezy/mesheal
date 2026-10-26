#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ogun {

/// Built-in synthetic and defective test mesh generators for unit tests and the GUI.
struct SampleMeshes {
    static constexpr Eigen::Index kCleanCubeVertexCount = 8;
    static constexpr Eigen::Index kCleanCubeFaceCount = 12;

    /// Known defective face indices in `createSampleDefectiveMesh()`:
    static constexpr Eigen::Index kZeroAreaFaceIndex = 12;
    static constexpr Eigen::Index kMicroscopicFaceIndex = 13;
    static constexpr Eigen::Index kNeedleSliverFaceIndex = 14;
    static constexpr Eigen::Index kDefectiveSampleFaceCount = 15;

    /// Generates a clean watertight cube mesh [0, sideLength]^3 with 8 vertices and 12 triangles.
    [[nodiscard]] static Mesh createCubeMesh(
        double sideLength = 10.0,
        Unit unit = Unit::Millimeter);

    /// Generates the `createCubeMesh()` cube with its top corner (s, s, s) pulled down to (s/2, s/2, -s/2),
    /// so the faces around that corner pass through the bottom. Still one closed, manifold,
    /// consistently wound shell: only the self-intersection check reports it
    /// (`examples/self_intersecting_box.stl`).
    [[nodiscard]] static Mesh createSelfIntersectingBoxMesh(
        double sideLength = 10.0,
        Unit unit = Unit::Millimeter);

    /// Generates a closed cube [0, sideLength]^3 whose six sides are `cells` x `cells` grids of
    /// triangle pairs with shared vertices (outward winding).
    [[nodiscard]] static Mesh createGridCubeMesh(
        int cells = 8,
        double sideLength = 8.0,
        Unit unit = Unit::Millimeter);

    /// Generates the 8 x 8 grid cube (side 8) with two intersection islands in one shell:
    ///   - a local fold: top vertex (2, 2, 8) dragged in-plane to (3.5, 2.2, 8), folding its fan over its neighbours;
    ///   - a pass-through: bottom vertex (6, 6, 0) pushed up to (6, 6, 9), through the top wall.
    /// File > Load Sample > "Folded cube" (try the intersection repairs on it).
    [[nodiscard]] static Mesh createFoldedCubeMesh(Unit unit = Unit::Millimeter);

    /// Generates three welded 2 x 2 grid cubes (side 2) and a fin, for the non-manifold repairs:
    ///   - cube A [0, 2]^3 and cube B [2, 4] x [2, 4] x [0, 2] touch along x = y = 2 (two Touching edges);
    ///   - cube C [-2, 0]^3 touches A only at the origin (one non-manifold vertex);
    ///   - a fin triangle on A's top edge (0, 2, 2)-(1, 2, 2) (one Fin edge and two open edges).
    /// File > Load Sample > "Touching cubes".
    [[nodiscard]] static Mesh createTouchingCubesMesh(Unit unit = Unit::Millimeter);

    /// Generates an ASCII STL string representing the 12-triangle cube (36 raw soup vertices).
    [[nodiscard]] static std::string createCubeAsciiStl(double sideLength = 10.0);

    /// Generates a binary STL byte buffer representing the 12-triangle cube (36 raw soup vertices).
    [[nodiscard]] static std::vector<std::uint8_t> createCubeBinaryStl(double sideLength = 10.0);

    /// Generates a clean 12-triangle cube mesh injected with 3 known defective triangles:
    ///   - Face 12 (`kZeroAreaFaceIndex`): collinear zero-area triangle (A = 0, h_min = 0)
    ///   - Face 13 (`kMicroscopicFaceIndex`): microscopic-area triangle (A = 5e-9, h_min ~ 7.07e-5)
    ///   - Face 14 (`kNeedleSliverFaceIndex`): long needle/sliver triangle (h_min = 2e-4, AR = 50,000)
    [[nodiscard]] static Mesh createSampleDefectiveMesh(Unit unit = Unit::Millimeter);
};

/// Free-function convenience wrapper around `SampleMeshes::createSampleDefectiveMesh()`.
[[nodiscard]] inline Mesh createSampleDefectiveMesh(Unit unit = Unit::Millimeter) {
    return SampleMeshes::createSampleDefectiveMesh(unit);
}

} // namespace ogun
