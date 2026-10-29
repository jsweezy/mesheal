#include "io/SampleMeshes.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <numbers>
#include <sstream>

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
