#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <filesystem>
#include <istream>
#include <string>

namespace ogun {

/// Welds duplicate vertices across triangles using local coordinate-relative tolerance
/// (`tolConfig`) while enforcing the Edge-Aware Guard: two corners belonging to the
/// same triangle are never merged into the same vertex index.
[[nodiscard]] Mesh weldVerticesLocal(
    const Eigen::MatrixXd& rawV,
    const Eigen::MatrixXi& rawF,
    Unit unit = Unit::Millimeter,
    const ToleranceConfig& tolConfig = ToleranceConfig{});

/// Convenience overload for an unindexed triangle soup `soupV` of size (3 * |F|) x 3,
/// where triangle `f` consists of rows (3*f, 3*f + 1, 3*f + 2).
[[nodiscard]] Mesh weldVerticesLocal(
    const Eigen::MatrixXd& soupV,
    Unit unit = Unit::Millimeter,
    const ToleranceConfig& tolConfig = ToleranceConfig{});

/// Reads an ASCII or binary STL stream via `igl::readSTL` and welds duplicate vertices
/// across triangles using `weldVerticesLocal`.
/// Throws `std::runtime_error` if the stream cannot be parsed as a valid STL.
[[nodiscard]] Mesh loadStl(
    std::istream& input,
    Unit unit = Unit::Millimeter,
    const ToleranceConfig& tolConfig = ToleranceConfig{});

/// Reads an ASCII or binary STL file from disk via `igl::readSTL` and welds duplicate
/// vertices across triangles using `weldVerticesLocal`.
/// Throws `std::runtime_error` if the file cannot be opened or parsed.
[[nodiscard]] Mesh loadStl(
    const std::filesystem::path& filepath,
    Unit unit = Unit::Millimeter,
    const ToleranceConfig& tolConfig = ToleranceConfig{});

/// Non-throwing helper that loads an STL file into `outMesh`.
/// Returns true on success, or false and populates `errorOut` (if non-null) on failure.
bool loadStl(
    const std::filesystem::path& filepath,
    Mesh& outMesh,
    Unit unit = Unit::Millimeter,
    const ToleranceConfig& tolConfig = ToleranceConfig{},
    std::string* errorOut = nullptr);

} // namespace ogun
