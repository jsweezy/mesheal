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
