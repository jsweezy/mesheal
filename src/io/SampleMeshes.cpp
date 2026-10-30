#include "io/SampleMeshes.hpp"

#include "io/StlReader.hpp"
#include "mesh/ToleranceConfig.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <numbers>
#include <sstream>
#include <vector>

namespace ogun {

namespace {

Mesh meshFromFaces(const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 3>>& faces, Unit unit) {
    Mesh mesh;
    mesh.fileUnit = unit;
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Welds the corners of a triangle soup by exact position (the STL reader's weld for exact copies).
Mesh weldSoup(const std::vector<std::array<Eigen::Vector3d, 3>>& soup, Unit unit) {
    std::map<std::array<double, 3>, int> index;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::array<int, 3>> faces;
    faces.reserve(soup.size());
    for (const auto& tri : soup) {
        std::array<int, 3> face{};
        for (std::size_t c = 0; c < 3; ++c) {
            const Eigen::Vector3d& p = tri[c];
            const auto [it, inserted] = index.try_emplace({p.x(), p.y(), p.z()}, static_cast<int>(points.size()));
            if (inserted) {
                points.push_back(p);
            }
            face[c] = it->second;
        }
        faces.push_back(face);
    }
    return meshFromFaces(points, faces, unit);
}

/// Appends the faces of `mesh` whose centroid fails `drop`, moved by `offset`, to `soup`.
template <typename Drop>
void appendSoup(std::vector<std::array<Eigen::Vector3d, 3>>& soup, const Mesh& mesh, const Eigen::Vector3d& offset, Drop drop) {
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        std::array<Eigen::Vector3d, 3> tri;
        for (int c = 0; c < 3; ++c) {
            tri[static_cast<std::size_t>(c)] = mesh.V.row(mesh.F(f, c)).transpose();
        }
        if (!drop((tri[0] + tri[1] + tri[2]) / 3.0)) {
            for (Eigen::Vector3d& p : tri) {
                p += offset;
            }
            soup.push_back(tri);
        }
    }
}

void appendFloatLE(std::vector<std::uint8_t>& buf, float val) {
    std::uint8_t bytes[sizeof(float)];
    std::memcpy(bytes, &val, sizeof(float));
    buf.insert(buf.end(), bytes, bytes + sizeof(float));
}

} // namespace

Mesh SampleMeshes::createCubeMesh(double sideLength, Unit unit) {
    const double s = sideLength;

    Mesh mesh;
    mesh.fileUnit = unit;

    mesh.V.resize(kCleanCubeVertexCount, 3);
    mesh.V << 0.0, 0.0, 0.0,
              s,   0.0, 0.0,
              s,   s,   0.0,
              0.0, s,   0.0,
              0.0, 0.0, s,
              s,   0.0, s,
              s,   s,   s,
              0.0, s,   s;

    mesh.F.resize(kCleanCubeFaceCount, 3);
    mesh.F << 0, 2, 1, // Bottom (-Z)
              0, 3, 2,
              4, 5, 6, // Top (+Z)
              4, 6, 7,
              0, 1, 5, // Front (-Y)
              0, 5, 4,
              2, 3, 7, // Back (+Y)
              2, 7, 6,
              0, 4, 7, // Left (-X)
              0, 7, 3,
              1, 2, 6, // Right (+X)
              1, 6, 5;

    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createSelfIntersectingBoxMesh(double sideLength, Unit unit) {
    Mesh mesh = createCubeMesh(sideLength, unit);
    constexpr Eigen::Index kTopCorner = 6;
    mesh.V.row(kTopCorner) << 0.5 * sideLength, 0.5 * sideLength, -0.5 * sideLength;
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createGridCubeMesh(int cells, double sideLength, Unit unit) {
    const int n = std::max(cells, 1);
    std::map<std::array<int, 3>, int> index;
    std::vector<std::array<int, 3>> lattice;
    std::vector<std::array<int, 3>> faces;
    const auto vertex = [&](const std::array<int, 3>& p) {
        const auto [it, inserted] = index.try_emplace(p, static_cast<int>(lattice.size()));
        if (inserted) {
            lattice.push_back(p);
        }
        return it->second;
    };
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t u = (axis + 1) % 3;
        const std::size_t v = (axis + 2) % 3;
        for (const int side : {0, n}) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    std::array<std::array<int, 3>, 4> c{};
                    for (std::size_t k = 0; k < 4; ++k) {
                        c[k][axis] = side;
                        c[k][u] = i + (k == 1 || k == 2 ? 1 : 0);
                        c[k][v] = j + (k >= 2 ? 1 : 0);
                    }
                    // u x v points along +axis: keep this winding on the far side, flip it on the near side
                    std::array<int, 3> t0{vertex(c[0]), vertex(c[1]), vertex(c[2])};
                    std::array<int, 3> t1{vertex(c[0]), vertex(c[2]), vertex(c[3])};
                    if (side == 0) {
                        std::swap(t0[1], t0[2]);
                        std::swap(t1[1], t1[2]);
                    }
                    faces.push_back(t0);
                    faces.push_back(t1);
                }
            }
        }
    }

    Mesh mesh;
    mesh.fileUnit = unit;
    const double step = sideLength / n;
    mesh.V.resize(static_cast<Eigen::Index>(lattice.size()), 3);
    for (std::size_t i = 0; i < lattice.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) << lattice[i][0] * step, lattice[i][1] * step, lattice[i][2] * step;
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createFoldedCubeMesh(Unit unit) {
    Mesh mesh = createGridCubeMesh(8, 8.0, unit);
    const auto vertexAt = [&](const Eigen::RowVector3d& p) {
        Eigen::Index nearest = 0;
        (mesh.V.rowwise() - p).rowwise().squaredNorm().minCoeff(&nearest);
        return nearest;
    };
    mesh.V.row(vertexAt({2.0, 2.0, 8.0})) << 3.5, 2.2, 8.0;
    mesh.V.row(vertexAt({6.0, 6.0, 0.0})) << 6.0, 6.0, 9.0;
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createTouchingCubesMesh(Unit unit) {
    const Mesh cube = createGridCubeMesh(2, 2.0, unit);
    const std::array<Eigen::RowVector3d, 3> offsets = {
        Eigen::RowVector3d(0.0, 0.0, 0.0), Eigen::RowVector3d(2.0, 2.0, 0.0), Eigen::RowVector3d(-2.0, -2.0, -2.0)};

    // Weld the copies by exact position, as the STL reader would
    std::map<std::array<double, 3>, int> index;
    std::vector<Eigen::RowVector3d> positions;
    const auto vertex = [&](const Eigen::RowVector3d& p) {
        const auto [it, inserted] = index.try_emplace({p.x(), p.y(), p.z()}, static_cast<int>(positions.size()));
        if (inserted) {
            positions.push_back(p);
        }
        return it->second;
    };
    std::vector<std::array<int, 3>> faces;
    for (const Eigen::RowVector3d& offset : offsets) {
        for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
            faces.push_back({vertex(cube.V.row(cube.F(f, 0)) + offset), vertex(cube.V.row(cube.F(f, 1)) + offset),
                             vertex(cube.V.row(cube.F(f, 2)) + offset)});
        }
    }
    faces.push_back({vertex({1.0, 2.0, 2.0}), vertex({0.0, 2.0, 2.0}), vertex({0.5, 2.5, 3.0})});

    Mesh mesh;
    mesh.fileUnit = unit;
    mesh.V.resize(static_cast<Eigen::Index>(positions.size()), 3);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = positions[i];
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createSphereMesh(int rings, int segments, double radius, Unit unit) {
    rings = std::max(rings, 2);
    segments = std::max(segments, 3);
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(2 + (rings - 1) * segments));
    points.emplace_back(0.0, 0.0, radius);
    for (int i = 1; i < rings; ++i) {
        const double t = std::numbers::pi * i / rings;
        for (int j = 0; j < segments; ++j) {
            const double p = 2.0 * std::numbers::pi * j / segments;
            points.emplace_back(radius * std::sin(t) * std::cos(p), radius * std::sin(t) * std::sin(p), radius * std::cos(t));
        }
    }
    points.emplace_back(0.0, 0.0, -radius);
    const int south = static_cast<int>(points.size()) - 1;
    const auto ring = [&](int i, int j) { return 1 + (i - 1) * segments + (j % segments); };
    std::vector<std::array<int, 3>> faces;
    for (int j = 0; j < segments; ++j) {
        faces.push_back({0, ring(1, j), ring(1, j + 1)});
    }
    for (int i = 1; i + 1 < rings; ++i) {
        for (int j = 0; j < segments; ++j) {
            faces.push_back({ring(i, j), ring(i + 1, j), ring(i + 1, j + 1)});
            faces.push_back({ring(i, j), ring(i + 1, j + 1), ring(i, j + 1)});
        }
    }
    for (int j = 0; j < segments; ++j) {
        faces.push_back({south, ring(rings - 1, j + 1), ring(rings - 1, j)});
    }
    return meshFromFaces(points, faces, unit);
}

Mesh SampleMeshes::createCylinderMesh(int segments, int bands, double radius, double height, Unit unit) {
    segments = std::max(segments, 3);
    bands = std::max(bands, 1);
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i <= bands; ++i) {
        for (int j = 0; j < segments; ++j) {
            const double p = 2.0 * std::numbers::pi * j / segments;
            points.emplace_back(radius * std::cos(p), radius * std::sin(p), height * i / bands);
        }
    }
    const int bottom = static_cast<int>(points.size());
    points.emplace_back(0.0, 0.0, 0.0);
    const int top = static_cast<int>(points.size());
    points.emplace_back(0.0, 0.0, height);
    const auto at = [&](int i, int j) { return i * segments + (j % segments); };
    std::vector<std::array<int, 3>> faces;
    for (int i = 0; i < bands; ++i) {
        for (int j = 0; j < segments; ++j) {
            faces.push_back({at(i, j), at(i, j + 1), at(i + 1, j + 1)});
            faces.push_back({at(i, j), at(i + 1, j + 1), at(i + 1, j)});
        }
    }
    for (int j = 0; j < segments; ++j) {
        faces.push_back({bottom, at(0, j + 1), at(0, j)});
        faces.push_back({top, at(bands, j), at(bands, j + 1)});
    }
    return meshFromFaces(points, faces, unit);
}

Mesh SampleMeshes::createCrackedBoxMesh(Unit unit) {
    const Mesh cube = createGridCubeMesh(4, 4.0, unit);
    std::vector<std::array<Eigen::Vector3d, 3>> soup;
    appendSoup(soup, cube, Eigen::Vector3d::Zero(), [](const Eigen::Vector3d& c) { return c.z() > 3.99; });
    const auto add = [&](double ax, double ay, double bx, double by, double cx, double cy) {
        soup.push_back({Eigen::Vector3d(ax, ay, 4.0), Eigen::Vector3d(bx, by, 4.0), Eigen::Vector3d(cx, cy, 4.0)});
    };
    // Unit grids on [0, 2] and [3, 4]; the strip [2, 3] uses only y = 0, 2, 4 on its x = 2 side
    for (const double x : {0.0, 1.0, 3.0}) {
        for (int yi = 0; yi < 4; ++yi) {
            const auto y = static_cast<double>(yi);
            add(x, y, x + 1.0, y, x + 1.0, y + 1.0);
            add(x, y, x + 1.0, y + 1.0, x, y + 1.0);
        }
    }
    const std::array<double, 3> seam{0.0, 2.0, 4.0};
    std::size_t i = 0;
    int j = 0;
    while (i + 1 < seam.size() || j < 4) {
        if (j < 4 && (i + 1 >= seam.size() || j + 1 <= seam[i + 1])) {
            add(2.0, seam[i], 3.0, j, 3.0, j + 1);
            ++j;
        } else {
            add(2.0, seam[i], 3.0, j, 2.0, seam[i + 1]);
            ++i;
        }
    }
    return weldSoup(soup, unit);
}

Mesh SampleMeshes::createHolesSampleMesh(Unit unit) {
    std::vector<std::array<Eigen::Vector3d, 3>> soup;
    const auto keepAll = [](const Eigen::Vector3d&) { return false; };
    appendSoup(soup, createCrackedBoxMesh(unit), Eigen::Vector3d::Zero(), keepAll);
    appendSoup(soup, createGridCubeMesh(4, 4.0, unit), Eigen::Vector3d(6.0, 0.0, 0.0), [](const Eigen::Vector3d& c) {
        return c.z() > 3.99 && c.x() > 1.0 && c.x() < 3.0 && c.y() > 1.0 && c.y() < 3.0;
    });
    appendSoup(soup, createSphereMesh(12, 24, 2.0, unit), Eigen::Vector3d(14.0, 2.0, 2.0),
               [](const Eigen::Vector3d& c) { return c.z() > 1.4; });
    appendSoup(soup, createCylinderMesh(24, 4, 2.0, 4.0, unit), Eigen::Vector3d(20.0, 2.0, 0.0),
               [](const Eigen::Vector3d& c) { return c.z() > 3.99; });
    return weldSoup(soup, unit);
}

std::string SampleMeshes::createCubeAsciiStl(double sideLength) {
    const Mesh cube = createCubeMesh(sideLength, Unit::Millimeter);

    std::ostringstream oss;
    oss << std::setprecision(10);
    oss << "solid ogun_sample_cube\n";
    for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
        const Eigen::Vector3d n = cube.N.row(f).transpose();
        oss << "  facet normal " << n.x() << " " << n.y() << " " << n.z() << "\n";
        oss << "    outer loop\n";
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d v = cube.V.row(cube.F(f, c)).transpose();
            oss << "      vertex " << v.x() << " " << v.y() << " " << v.z() << "\n";
        }
        oss << "    endloop\n";
        oss << "  endfacet\n";
    }
    oss << "endsolid ogun_sample_cube\n";
    return oss.str();
}

std::vector<std::uint8_t> SampleMeshes::createCubeBinaryStl(double sideLength) {
    const Mesh cube = createCubeMesh(sideLength, Unit::Millimeter);
    const auto numTris = static_cast<std::uint32_t>(cube.numFaces());

    std::vector<std::uint8_t> buffer;
    buffer.reserve(80 + 4 + static_cast<std::size_t>(numTris) * 50);

    // 80-byte binary header (must not start with "solid")
    const char headerText[] = "Binary STL generated by Ogun SampleMeshes";
    buffer.resize(80, 0);
    std::memcpy(buffer.data(), headerText, sizeof(headerText) - 1);

    // 4-byte unsigned integer triangle count
    std::uint8_t countBytes[4];
    std::memcpy(countBytes, &numTris, 4);
    buffer.insert(buffer.end(), countBytes, countBytes + 4);

    // 50-byte triangle records: normal (3 floats), v0/v1/v2 (9 floats), attribute byte count (2 bytes)
    for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
        const Eigen::Vector3d n = cube.N.row(f).transpose();
        appendFloatLE(buffer, static_cast<float>(n.x()));
        appendFloatLE(buffer, static_cast<float>(n.y()));
        appendFloatLE(buffer, static_cast<float>(n.z()));

        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d v = cube.V.row(cube.F(f, c)).transpose();
            appendFloatLE(buffer, static_cast<float>(v.x()));
            appendFloatLE(buffer, static_cast<float>(v.y()));
            appendFloatLE(buffer, static_cast<float>(v.z()));
        }

        buffer.push_back(0);
        buffer.push_back(0);
    }

    return buffer;
}

Mesh SampleMeshes::createBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max, bool inward, Unit unit) {
    Mesh mesh = createCubeMesh(1.0, unit);
    const Eigen::RowVector3d size = (max - min).transpose();
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        mesh.V.row(v) = min.transpose() + mesh.V.row(v).cwiseProduct(size);
    }
    if (inward) {
        mesh.F.col(1).swap(mesh.F.col(2));
    }
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::combineMeshes(const std::vector<Mesh>& parts) {
    Mesh mesh;
    if (!parts.empty()) {
        mesh.fileUnit = parts.front().fileUnit;
    }
    for (const Mesh& part : parts) {
        appendMesh(mesh, part);
    }
    return mesh;
}

Mesh SampleMeshes::createCavityBoxMesh(bool insideOut, Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}, insideOut, unit),
                          createBox({1.0, 1.0, 1.0}, {3.0, 3.0, 3.0}, !insideOut, unit)});
}

Mesh SampleMeshes::createOverlapPlusCavityMesh(Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}, false, unit),
                          createBox({3.5, 0.5, 0.5}, {7.5, 4.5, 4.5}, false, unit),
                          createBox({1.0, 1.0, 1.0}, {2.0, 3.0, 3.0}, true, unit)});
}

Mesh SampleMeshes::createCavityReachedMesh(CavityReachedVariant variant, Unit unit) {
    std::vector<Mesh> parts{createBox({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}, false, unit),
                            createBox({3.5, 0.5, 0.5}, {7.5, 4.5, 4.5}, false, unit)};
    if (variant == CavityReachedVariant::InsideOverlap) {
        parts.push_back(createBox({3.6, 1.0, 1.0}, {3.9, 2.0, 2.0}, true, unit));
    } else {
        parts.push_back(createBox({3.0, 1.0, 1.0}, {3.8, 3.0, 3.0}, true, unit));
    }
    if (variant == CavityReachedVariant::WithSeparatePair) {
        parts.push_back(createBox({20.0, 0.0, 0.0}, {24.0, 4.0, 4.0}, false, unit));
        parts.push_back(createBox({23.5, 0.5, 0.5}, {27.5, 4.5, 4.5}, false, unit));
    }
    return combineMeshes(parts);
}

Mesh SampleMeshes::createNestedSolidAndCavityMesh(bool crossingVariant, Unit unit) {
    std::vector<Mesh> parts{createBox({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}, false, unit),
                            createBox({0.5, 0.5, 0.5}, {1.5, 1.5, 1.5}, true, unit),
                            createBox({2.5, 2.5, 2.5}, {3.5, 3.5, 3.5}, false, unit)};
    if (crossingVariant) {
        parts.push_back(createBox({3.2, 2.7, 2.8}, {3.8, 3.3, 3.4}, false, unit));
    }
    return combineMeshes(parts);
}

Mesh SampleMeshes::createIslandInCavityMesh(bool islandInsideOut, Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {6.0, 6.0, 6.0}, false, unit),
                          createBox({1.0, 1.0, 1.0}, {5.0, 5.0, 5.0}, true, unit),
                          createBox({2.0, 2.0, 2.0}, {4.0, 4.0, 4.0}, islandInsideOut, unit)});
}

Mesh SampleMeshes::createEdgeTouchingBoxesMesh(Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}, false, unit),
                          createBox({2.0, 2.0, 0.0}, {4.0, 4.0, 2.0}, false, unit)});
}

Mesh SampleMeshes::createFaceTouchingBoxesMesh(Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}, false, unit),
                          createBox({2.0, 0.5, 0.5}, {4.0, 1.5, 1.5}, false, unit)});
}

Mesh SampleMeshes::createTurkeyLikeMesh(Unit unit) {
    std::vector<Mesh> parts{createBox({0.0, 0.0, 0.0}, {10.0, 6.0, 6.0}, false, unit)};
    for (const double x : {1.1, 2.6, 4.1, 5.6, 7.1, 8.6}) {
        parts.push_back(createBox({x, 4.7, 1.7}, {x + 0.3, 9.3, 4.3}, false, unit));
    }
    parts.push_back(createBox({3.2, 1.2, 1.2}, {4.8, 2.8, 2.8}, false, unit));
    parts.push_back(createBox({14.0, 0.0, 0.0}, {16.0, 2.0, 2.0}, false, unit));
    return combineMeshes(parts);
}

namespace {

/// `mesh` without the faces whose centroid passes `drop` (vertices kept).
template <typename Drop>
Mesh withoutFacesWhere(const Mesh& mesh, Drop drop) {
    std::vector<Eigen::Index> kept;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c = (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        if (!drop(c)) {
            kept.push_back(f);
        }
    }
    Mesh out = mesh;
    out.F.resize(static_cast<Eigen::Index>(kept.size()), 3);
    for (std::size_t i = 0; i < kept.size(); ++i) {
        out.F.row(static_cast<Eigen::Index>(i)) = mesh.F.row(kept[i]);
    }
    out.recomputeNormals();
    return out;
}

} // namespace

Mesh SampleMeshes::createFillCrossesShellMesh(FillCrossesVariant variant, Unit unit) {
    const Mesh body = createBox({0.0, 0.0, 0.0}, {6.0, 6.0, 6.0}, false, unit);
    if (variant == FillCrossesVariant::OpenBoxThroughBody) {
        const Mesh open = withoutFacesWhere(createBox({4.0, 1.0, 1.0}, {9.0, 3.0, 3.0}, false, unit),
                                            [](const Eigen::Vector3d& c) { return c.x() > 9.0 - 1e-9; });
        return combineMeshes({body, open});
    }
    Mesh plate = createBox({1.0, 1.0, 2.9}, {5.0, 5.0, 3.1}, false, unit);
    if (variant == FillCrossesVariant::OpenPlate) {
        // One triangle of the plate's far side x = 1, well clear of the open box
        bool dropped = false;
        plate = withoutFacesWhere(plate, [&](const Eigen::Vector3d& c) {
            if (!dropped && c.x() < 1.0 + 1e-9) {
                dropped = true;
                return true;
            }
            return false;
        });
    }
    const Mesh open = withoutFacesWhere(createBox({2.0, 2.0, 2.0}, {4.0, 4.0, 4.0}, false, unit),
                                        [](const Eigen::Vector3d& c) { return c.x() > 4.0 - 1e-9; });
    return combineMeshes({body, plate, open});
}

Mesh SampleMeshes::createOpenShellAmongCubesMesh(Unit unit) {
    const Mesh open = withoutFacesWhere(createBox({-3.0, 1.0, 1.0}, {0.5, 3.0, 3.0}, false, unit),
                                        [](const Eigen::Vector3d& c) { return c.x() < -3.0 + 1e-9; });
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}, false, unit),
                          createBox({3.5, 0.5, 0.5}, {7.5, 4.5, 4.5}, false, unit), open});
}

Mesh SampleMeshes::createCavityHoleSliversIntersectionsMesh(Unit unit) {
    Mesh body = createGridCubeMesh(4, 8.0, unit);
    const auto findFace = [&](auto&& inCell) {
        for (Eigen::Index f = 0; f < body.numFaces(); ++f) {
            const Eigen::Vector3d c =
                (body.V.row(body.F(f, 0)) + body.V.row(body.F(f, 1)) + body.V.row(body.F(f, 2))).transpose() / 3.0;
            if (inCell(c)) {
                return f;
            }
        }
        return Eigen::Index{-1};
    };
    // Splits face f in three around a new point `p` inside it, keeping its winding
    const auto splitAt = [&](Eigen::Index f, int k, const Eigen::Vector3d& p) {
        const std::array<int, 3> t{body.F(f, k), body.F(f, (k + 1) % 3), body.F(f, (k + 2) % 3)};
        const auto pi = static_cast<int>(body.numVertices());
        body.V.conservativeResize(pi + 1, 3);
        body.V.row(pi) = p.transpose();
        const Eigen::Index nf = body.numFaces();
        body.F.conservativeResize(nf + 2, 3);
        body.F.row(f) << t[0], t[1], pi;
        body.F.row(nf) << t[1], t[2], pi;
        body.F.row(nf + 1) << t[2], t[0], pi;
    };
    const auto at = [&](Eigen::Index f, int c) -> Eigen::Vector3d { return body.V.row(body.F(f, c)).transpose(); };

    // Cap: a point 0.01 off the middle of the longest edge, toward the opposite corner
    const Eigen::Index capFace = findFace([](const Eigen::Vector3d& c) {
        return c.x() < 1.0e-9 && c.y() > 4.0 && c.y() < 6.0 && c.z() > 2.0 && c.z() < 4.0;
    });
    int longest = 0;
    for (int k = 1; k < 3; ++k) {
        if ((at(capFace, (k + 1) % 3) - at(capFace, k)).norm() > (at(capFace, (longest + 1) % 3) - at(capFace, longest)).norm()) {
            longest = k;
        }
    }
    const Eigen::Vector3d mid = 0.5 * (at(capFace, longest) + at(capFace, (longest + 1) % 3));
    splitAt(capFace, longest, mid + 0.01 * (at(capFace, (longest + 2) % 3) - mid).normalized());

    // Needles: a point 0.01 from a corner, toward the centroid
    const Eigen::Index needleFace = findFace([](const Eigen::Vector3d& c) {
        return c.x() < 1.0e-9 && c.y() > 2.0 && c.y() < 4.0 && c.z() > 4.0 && c.z() < 6.0;
    });
    const Eigen::Vector3d centroid = (at(needleFace, 0) + at(needleFace, 1) + at(needleFace, 2)) / 3.0;
    splitAt(needleFace, 0, at(needleFace, 0) + 0.01 * (centroid - at(needleFace, 0)).normalized());

    // Fold: drag a top vertex 1.5 cells across its neighbours
    for (Eigen::Index v = 0; v < body.numVertices(); ++v) {
        if ((body.V.row(v) - Eigen::RowVector3d(2.0, 2.0, 8.0)).norm() < 1.0e-9) {
            body.V.row(v) << 5.0, 2.4, 8.0;
        }
    }

    // Hole: drop the two triangles of one front cell
    std::vector<Eigen::Index> keep;
    keep.reserve(static_cast<std::size_t>(body.numFaces()));
    for (Eigen::Index f = 0; f < body.numFaces(); ++f) {
        const Eigen::Vector3d c = (at(f, 0) + at(f, 1) + at(f, 2)) / 3.0;
        if (!(c.y() < 1.0e-9 && c.x() > 4.0 && c.x() < 6.0 && c.z() > 2.0 && c.z() < 4.0)) {
            keep.push_back(f);
        }
    }
    Mesh holed = extractFaces(body, keep);
    holed.fileUnit = unit;
    return combineMeshes({holed,
                          createBox({1.5, 1.5, 1.5}, {3.5, 3.5, 3.5}, true, unit),
                          createBox({6.6, 5.4, 5.3}, {10.4, 6.6, 6.7}, false, unit)});
}

Mesh SampleMeshes::createDoubledFacesCubeMesh(Unit unit) {
    const Mesh cube = createCubeMesh(10.0, unit);
    Mesh mesh;
    mesh.fileUnit = unit;
    mesh.V.resize(kCleanCubeVertexCount + 1, 3);
    mesh.V.topRows(kCleanCubeVertexCount) = cube.V;
    mesh.V.row(kCleanCubeVertexCount) = Eigen::RowVector3d(5.0, 0.0, 0.0); // midpoint of edge 0-1

    mesh.F.resize(kRepeatedVertexFaceIndex + 1, 3);
    mesh.F.topRows(kCleanCubeFaceCount) = cube.F;
    mesh.F.row(0) = Eigen::RowVector3i(0, 2, 8); // bottom (0, 2, 1) split at vertex 8
    mesh.F.row(12) = Eigen::RowVector3i(8, 2, 1);
    mesh.F.row(kFlatCapFaceIndex) = Eigen::RowVector3i(0, 8, 1);
    mesh.F.row(kSameWindingCopyFaceIndex) = Eigen::RowVector3i(5, 6, 4);     // top (4, 5, 6)
    mesh.F.row(kOppositeWindingCopyFaceIndex) = Eigen::RowVector3i(2, 7, 3); // back (2, 3, 7)
    mesh.F.row(kRepeatedVertexFaceIndex) = Eigen::RowVector3i(6, 6, 7);
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createTetrahedron(const Eigen::Vector3d& corner, double edge, Unit unit) {
    const std::vector<Eigen::Vector3d> points{corner, corner + Eigen::Vector3d(edge, 0.0, 0.0),
                                              corner + Eigen::Vector3d(0.0, edge, 0.0), corner + Eigen::Vector3d(0.0, 0.0, edge)};
    return meshFromFaces(points, {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}, unit);
}

Mesh SampleMeshes::createTinyFloatingShellsMesh(Unit unit) {
    return combineMeshes({createBox({0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}, false, unit),
                          createBox({12.0, 0.0, 0.0}, {12.5, 0.5, 0.5}, false, unit),
                          createBox({5.0, 5.0, 5.0}, {5.01, 5.01, 5.01}, true, unit),
                          createTetrahedron({3.0, 6.0, 10.0}, 0.01, unit),
                          createTetrahedron({20.0, 20.0, 0.0}, 0.01, unit)});
}

Mesh SampleMeshes::createFinSeamBoxesMesh(Unit unit) {
    const auto on = [](double a, double b) { return std::abs(a - b) < 1.0e-12; };
    const Mesh left = createBox({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, false, unit);
    const Mesh right = createBox({1.0, 0.0, 0.0}, {2.0, 1.0, 1.0}, false, unit);
    std::vector<std::array<Eigen::Vector3d, 3>> soup;
    appendSoup(soup, left, Eigen::Vector3d::Zero(), [&](const Eigen::Vector3d& c) { return on(c.x(), 1.0); });
    appendSoup(soup, right, Eigen::Vector3d::Zero(),
               [&](const Eigen::Vector3d& c) { return on(c.x(), 1.0) || on(c.z(), 1.0); });
    appendSoup(soup, left, Eigen::Vector3d::Zero(), [&](const Eigen::Vector3d& c) { return !on(c.x(), 1.0); });
    return weldSoup(soup, unit);
}

Mesh SampleMeshes::createSeamBoxMesh(const Eigen::Vector3d& min, const Eigen::Vector3d& max, int axis, double offset, Unit unit) {
    const int a = std::clamp(axis, 0, 2);
    const double cut = 0.5 * (min[a] + max[a]);
    Eigen::Vector3d lowMax = max;
    lowMax[a] = cut;
    Eigen::Vector3d highMin = min;
    highMin[a] = cut + offset;
    const Mesh low = createBox(min, lowMax, false, unit);
    const Mesh high = createBox(highMin, max, false, unit);
    const double eps = 1.0e-6 * (max[a] - min[a]); // a centroid on the cut plane, up to rounding
    std::vector<std::array<Eigen::Vector3d, 3>> soup;
    appendSoup(soup, low, Eigen::Vector3d::Zero(), [&](const Eigen::Vector3d& c) { return std::abs(c[a] - cut) < eps; });
    appendSoup(soup, high, Eigen::Vector3d::Zero(),
               [&](const Eigen::Vector3d& c) { return std::abs(c[a] - highMin[a]) < eps; });
    return weldSoup(soup, unit);
}

Mesh SampleMeshes::createWeldSeamCubeMesh(double offset, const Eigen::Vector3d& translate, double scale, Unit unit) {
    Mesh mesh = createSeamBoxMesh({10.0, 10.0, 10.0}, {20.0, 20.0, 20.0}, 0, offset, unit);
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        mesh.V.row(v) = scale * mesh.V.row(v) + translate.transpose();
    }
    mesh.recomputeNormals();
    return mesh;
}

Mesh SampleMeshes::createCupWithPostMesh(CupPost post, Unit unit) {
    constexpr int kSegments = 32;
    constexpr double kOpeningRadius = 3.0;
    constexpr double kOpeningHeight = 8.0;
    constexpr double kOuterRadius = 5.0;
    constexpr double kHeight = 10.0;
    constexpr double kPostRadius = 1.0;
    const double postHeight = post == CupPost::BelowOpening ? 7.5 : post == CupPost::ThroughOpening ? 9.0 : 2.0;

    // Profile in (r, z) from the opening's rim to the post's top centre; the material lies to its left
    std::vector<std::array<double, 2>> profile;
    const auto segment = [&](double r0, double z0, double r1, double z1, int steps) {
        for (int i = profile.empty() ? 0 : 1; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            profile.push_back({r0 + t * (r1 - r0), z0 + t * (z1 - z0)});
        }
    };
    segment(kOpeningRadius, kOpeningHeight, kOuterRadius, kHeight, 4); // lip
    segment(kOuterRadius, kHeight, kOuterRadius, 0.0, 5);              // outer wall
    segment(kOuterRadius, 0.0, kPostRadius, 0.0, 4);                   // bottom
    segment(kPostRadius, 0.0, kPostRadius, postHeight, 5);             // post wall
    segment(kPostRadius, postHeight, 0.0, postHeight, 1);              // post top (the last point is its centre)

    std::vector<Eigen::Vector3d> points;
    const int rings = static_cast<int>(profile.size()) - 1; // the last profile point is one vertex
    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < kSegments; ++j) {
            const double a = 2.0 * std::numbers::pi * j / kSegments;
            points.emplace_back(profile[static_cast<std::size_t>(i)][0] * std::cos(a),
                                profile[static_cast<std::size_t>(i)][0] * std::sin(a), profile[static_cast<std::size_t>(i)][1]);
        }
    }
    const int apex = static_cast<int>(points.size());
    points.emplace_back(0.0, 0.0, postHeight);
    const auto at = [&](int i, int j) { return i * kSegments + (j % kSegments); };
    std::vector<std::array<int, 3>> faces;
    for (int i = 0; i + 1 < rings; ++i) {
        for (int j = 0; j < kSegments; ++j) {
            faces.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
            faces.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
        }
    }
    for (int j = 0; j < kSegments; ++j) {
        faces.push_back({at(rings - 1, j), apex, at(rings - 1, j + 1)});
    }
    Mesh cup = meshFromFaces(points, faces, unit);
    if (post != CupPost::RodThroughOpening) {
        return cup;
    }
    return combineMeshes({cup, createBox(Eigen::Vector3d(-0.5, -0.5, 4.0), Eigen::Vector3d(0.5, 0.5, 12.0), false, unit)});
}

Mesh SampleMeshes::createCrackedFlippedCubeSoup(Unit unit) {
    constexpr double kOffset = 1.0e-7;
    const Mesh grid = createGridCubeMesh(2, 1.0, unit);
    const auto on = [](double a, double b) { return a == b; };
    const auto uses = [&](Eigen::Index f, const Eigen::Vector3d& p) {
        for (int c = 0; c < 3; ++c) {
            if (grid.V.row(grid.F(f, c)).transpose() == p) {
                return true;
            }
        }
        return false;
    };
    const auto sideFaces = [&](int axis, double side) {
        std::vector<Eigen::Index> faces;
        for (Eigen::Index f = 0; f < grid.numFaces(); ++f) {
            if (on(grid.V(grid.F(f, 0), axis), side) && on(grid.V(grid.F(f, 1), axis), side) && on(grid.V(grid.F(f, 2), axis), side)) {
                faces.push_back(f);
            }
        }
        return faces;
    };
    const auto first = [&](int axis, double side, const auto& accept) {
        for (const Eigen::Index f : sideFaces(axis, side)) {
            if (accept(f)) {
                return f;
            }
        }
        return Eigen::Index{-1};
    };
    const Eigen::Vector3d top(1.0, 1.0, 1.0);
    const Eigen::Vector3d bottom(0.0, 0.0, 0.0);
    const Eigen::Index topCrack = first(2, 1.0, [&](Eigen::Index f) { return uses(f, top); });
    const Eigen::Index bottomCrack = first(2, 0.0, [&](Eigen::Index f) { return uses(f, bottom); });
    const auto centroid = [&](Eigen::Index f) {
        return Eigen::Vector3d((grid.V.row(grid.F(f, 0)) + grid.V.row(grid.F(f, 1)) + grid.V.row(grid.F(f, 2))).transpose() / 3.0);
    };
    // Missing: a triangle of the right side's lower front cell, away from both cracks
    const Eigen::Index missing = first(0, 1.0, [&](Eigen::Index f) {
        const Eigen::Vector3d c = centroid(f);
        return c.y() < 0.5 && c.z() < 0.5;
    });

    Eigen::MatrixXd soup(3 * (grid.numFaces() - 1), 3);
    Eigen::Index row = 0;
    for (Eigen::Index f = 0; f < grid.numFaces(); ++f) {
        if (f == missing) {
            continue;
        }
        const Eigen::Vector3d c = centroid(f);
        const bool backwards = c.x() == 0.0 && c.y() > 0.5 && c.z() > 0.5; // the left side's upper back cell
        for (int k = 0; k < 3; ++k) {
            Eigen::Vector3d p = grid.V.row(grid.F(f, backwards ? 2 - k : k)).transpose();
            if (f == topCrack && p == top) {
                p.y() -= kOffset;
            } else if (f == bottomCrack && p == bottom) {
                p.x() += kOffset;
            }
            soup.row(row++) = p.transpose();
        }
    }
    return separateTouchingContacts(weldVerticesLocal(soup, unit, ToleranceConfig{}));
}

Mesh SampleMeshes::createCollinearCapCubeMesh(Unit unit) {
    Mesh mesh = createGridCubeMesh(2, 2.0, unit);
    const auto find = [&](const Eigen::Vector3d& p) {
        for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
            if (mesh.V.row(v).transpose() == p) {
                return static_cast<int>(v);
            }
        }
        return -1;
    };
    const int a = find({1.0, 0.0, 2.0});
    const int b = find({1.0, 1.0, 2.0});
    // The face running a -> b is split at the edge's midpoint m; the cap (a, b, m) closes the T-junction
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (mesh.F(f, c) != a || mesh.F(f, (c + 1) % 3) != b) {
                continue;
            }
            const int opposite = mesh.F(f, (c + 2) % 3);
            const auto m = static_cast<int>(mesh.numVertices());
            mesh.V.conservativeResize(m + 1, 3);
            mesh.V.row(m) << 1.0, 0.5, 2.0;
            const Eigen::Index n = mesh.numFaces();
            mesh.F.conservativeResize(n + 2, 3);
            mesh.F.row(f) << a, m, opposite;
            mesh.F.row(n) << m, b, opposite;
            mesh.F.row(n + 1) << a, b, m;
            mesh.recomputeNormals();
            return mesh;
        }
    }
    return mesh;
}

Mesh SampleMeshes::createBoundaryCapPlateMesh(Unit unit) {
    const double delta = std::tan(0.05 * std::numbers::pi / 180.0); // apex angle 179.9 degrees over a 2-long edge
    const std::vector<Eigen::Vector3d> points{
        {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}, {0.0, 10.0, 0.0},  // 0-3 bottom corners
        {0.0, 0.0, 1.0}, {10.0, 0.0, 1.0}, {10.0, 10.0, 1.0}, {0.0, 10.0, 1.0},  // 4-7 top corners
        {4.0, 4.0, 1.0}, {6.0, 4.0, 1.0}, {6.0, 6.0, 1.0}, {4.0, 6.0, 1.0},      // 8-11 hole corners
        {5.0, 4.0 - delta, 1.0}};                                                // 12 cap apex
    const std::vector<std::array<int, 3>> faces{
        {0, 2, 1}, {0, 3, 2},                                  // bottom
        {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},            // front, right
        {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},            // back, left
        {4, 5, 12}, {4, 12, 8}, {5, 9, 12}, {8, 12, 9},        // top front strip, the cap last
        {5, 6, 10}, {5, 10, 9}, {6, 7, 11}, {6, 11, 10},       // top right and back strips
        {7, 4, 8}, {7, 8, 11}};                                // top left strip
    return meshFromFaces(points, faces, unit);
}

Mesh SampleMeshes::createSampleDefectiveMesh(Unit unit) {
    const Mesh cube = createCubeMesh(10.0, unit);

    Mesh mesh;
    mesh.fileUnit = unit;

    // 8 cube vertices + 7 vertices for the 3 injected defective faces = 15 vertices
    mesh.V.resize(15, 3);
    mesh.V.topRows(kCleanCubeVertexCount) = cube.V;

    // Vertex 8: midpoint along bottom front edge (0,0,0) -> (10,0,0) for collinear zero-area face
    mesh.V.row(8)  = Eigen::RowVector3d(5.0,    0.0,    0.0);

    // Vertices 9, 10, 11: microscopic right-isosceles triangle on top face (legs = 1e-4, A = 5e-9)
    mesh.V.row(9)  = Eigen::RowVector3d(2.0,    2.0,    10.0);
    mesh.V.row(10) = Eigen::RowVector3d(2.0001, 2.0,    10.0);
    mesh.V.row(11) = Eigen::RowVector3d(2.0,    2.0001, 10.0);

    // Vertices 12, 13, 14: long needle/sliver triangle on top face (base = 10.0, h_min = 2e-4, AR = 50,000)
    mesh.V.row(12) = Eigen::RowVector3d(0.0,    5.0,    10.0);
    mesh.V.row(13) = Eigen::RowVector3d(10.0,   5.0,    10.0);
    mesh.V.row(14) = Eigen::RowVector3d(5.0,    5.0002, 10.0);

    mesh.F.resize(kDefectiveSampleFaceCount, 3);
    mesh.F.topRows(kCleanCubeFaceCount) = cube.F;

    mesh.F.row(kZeroAreaFaceIndex)     = Eigen::RowVector3i(0,  8,  1);
    mesh.F.row(kMicroscopicFaceIndex)  = Eigen::RowVector3i(9,  10, 11);
    mesh.F.row(kNeedleSliverFaceIndex) = Eigen::RowVector3i(12, 13, 14);

    mesh.recomputeNormals();
    return mesh;
}

} // namespace ogun
