#include "io/StlWriter.hpp"

#include <Eigen/Geometry>

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace ogun {

namespace {

void requireNonEmpty(const Mesh& mesh) {
    if (mesh.empty()) {
        throw std::runtime_error("Cannot write STL: mesh is empty");
    }
}

Eigen::Vector3d faceNormal(const Mesh& mesh, Eigen::Index f) {
    if (mesh.N.rows() == mesh.F.rows() && mesh.N.cols() == 3) {
        return mesh.N.row(f).transpose();
    }
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(f, 0)).transpose();
    const Eigen::Vector3d v1 = mesh.V.row(mesh.F(f, 1)).transpose();
    const Eigen::Vector3d v2 = mesh.V.row(mesh.F(f, 2)).transpose();
    Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
    const double len = n.norm();
    return len > 0.0 ? Eigen::Vector3d(n / len) : Eigen::Vector3d::Zero();
}

void putU32(std::array<char, 50>& rec, std::size_t offset, std::uint32_t value) {
    for (std::size_t i = 0; i < 4; ++i) {
        rec[offset + i] = static_cast<char>((value >> (8 * i)) & 0xFFu);
    }
}

void putFloat(std::array<char, 50>& rec, std::size_t offset, double value) {
    putU32(rec, offset, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
}

} // namespace

void writeStlBinary(std::ostream& output, const Mesh& mesh) {
    requireNonEmpty(mesh);

    std::array<char, 80> header{};
    constexpr std::string_view kTag = "Ogun binary STL";
    std::copy(kTag.begin(), kTag.end(), header.begin());
    output.write(header.data(), static_cast<std::streamsize>(header.size()));

    std::array<char, 50> rec{};
    putU32(rec, 0, 0);
    std::array<char, 4> count{};
    const auto numFaces = static_cast<std::uint32_t>(mesh.numFaces());
    for (std::size_t i = 0; i < 4; ++i) {
        count[i] = static_cast<char>((numFaces >> (8 * i)) & 0xFFu);
    }
    output.write(count.data(), 4);

    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        rec.fill(0);
        const Eigen::Vector3d n = faceNormal(mesh, f);
        for (int c = 0; c < 3; ++c) {
            putFloat(rec, static_cast<std::size_t>(c) * 4, n(c));
        }
        for (int k = 0; k < 3; ++k) {
            for (int c = 0; c < 3; ++c) {
                putFloat(rec, 12 + static_cast<std::size_t>(k) * 12 + static_cast<std::size_t>(c) * 4,
                         mesh.V(mesh.F(f, k), c));
            }
        }
        output.write(rec.data(), static_cast<std::streamsize>(rec.size()));
    }

    if (!output) {
        throw std::runtime_error("Failed while writing binary STL stream");
    }
}

void writeStlAscii(std::ostream& output, const Mesh& mesh, std::string_view solidName) {
    requireNonEmpty(mesh);

    const auto oldPrecision = output.precision(std::numeric_limits<double>::max_digits10);
    output << "solid " << solidName << '\n';
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d n = faceNormal(mesh, f);
        output << "  facet normal " << n.x() << ' ' << n.y() << ' ' << n.z() << '\n'
               << "    outer loop\n";
        for (int k = 0; k < 3; ++k) {
            output << "      vertex " << mesh.V(mesh.F(f, k), 0) << ' ' << mesh.V(mesh.F(f, k), 1) << ' '
                   << mesh.V(mesh.F(f, k), 2) << '\n';
        }
        output << "    endloop\n  endfacet\n";
    }
    output << "endsolid " << solidName << '\n';
    output.precision(oldPrecision);

    if (!output) {
        throw std::runtime_error("Failed while writing ASCII STL stream");
    }
}

bool saveStl(
    const std::filesystem::path& filepath,
    const Mesh& mesh,
    StlFormat format,
    std::string* errorOut) {
    try {
        requireNonEmpty(mesh);
        std::ofstream file(filepath, std::ios::binary | std::ios::trunc);
        if (!file) {
            throw std::runtime_error("Cannot open file for writing: " + filepath.string());
        }
        if (format == StlFormat::Binary) {
            writeStlBinary(file, mesh);
        } else {
            writeStlAscii(file, mesh);
        }
        file.close();
        if (!file) {
            throw std::runtime_error("Failed to finalize file: " + filepath.string());
        }
        return true;
    } catch (const std::exception& e) {
        if (errorOut != nullptr) {
            *errorOut = e.what();
        }
        return false;
    }
}

} // namespace ogun

