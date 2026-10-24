#pragma once

#include "mesh/Mesh.hpp"

#include <filesystem>
#include <ostream>
#include <string>
#include <string_view>

namespace ogun {

enum class StlFormat { Binary, Ascii };

/// Writes a binary STL (80-byte header, uint32 triangle count, 50-byte little-endian records).
/// Throws `std::runtime_error` if the mesh is empty or the stream fails.
void writeStlBinary(std::ostream& output, const Mesh& mesh);

/// Writes an ASCII STL with round-trip (max_digits10) double precision.
/// Throws `std::runtime_error` if the mesh is empty or the stream fails.
void writeStlAscii(std::ostream& output, const Mesh& mesh, std::string_view solidName = "ogun_mesh");

/// Non-throwing file export. Returns true on success, or false and populates `errorOut` on failure.
bool saveStl(
    const std::filesystem::path& filepath,
    const Mesh& mesh,
    StlFormat format = StlFormat::Binary,
    std::string* errorOut = nullptr);

} // namespace ogun

