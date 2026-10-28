#include "io/StlReader.hpp"
#include "core/Constants.hpp"
#include "diagnostics/MeshChecks.hpp"

#include <igl/readSTL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace ogun {

namespace {

double computeProjectionKey(const Eigen::Vector3d& p) noexcept {
    return kAlphaX * p.x() + kAlphaY * p.y() + kAlphaZ * p.z();
}

// Computes an upper bound on |key(p) - key(q)| for any point q that matches p
// within `tolConfig` along all three coordinate axes, including an IEEE-754 ULP safety margin.
double computeMaxKeyDelta(
    const Eigen::Vector3d& p,
    const ToleranceConfig& tolConfig) noexcept {
    const double epsAbs = std::max(0.0, tolConfig.absFloor);
    const double rel = std::clamp(tolConfig.relTol, 0.0, 0.5);
    const double denom = 1.0 - rel;

    const double dx = std::max(epsAbs, rel * std::abs(p.x())) / denom;
    const double dy = std::max(epsAbs, rel * std::abs(p.y())) / denom;
    const double dz = std::max(epsAbs, rel * std::abs(p.z())) / denom;

    const double l1 = std::abs(p.x()) + std::abs(p.y()) + std::abs(p.z());
    const double ulpMargin = 32.0 * std::numeric_limits<double>::epsilon() * (l1 + 1.0);

    return kAlphaX * dx + kAlphaY * dy + kAlphaZ * dz + epsAbs + ulpMargin;
}

struct CornerEntry {
    double key = 0.0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Index face = 0;
    int corner = 0;
    int weldedId = -1;
};

} // namespace

Mesh weldVerticesLocal(
    const Eigen::MatrixXd& rawV,
    const Eigen::MatrixXi& rawF,
    Unit unit,
    const ToleranceConfig& tolConfig) {
    Mesh mesh;
    mesh.fileUnit = unit;

    if (rawV.rows() == 0 || rawF.rows() == 0) {
        return mesh;
    }

    if (rawV.cols() != 3) {
        throw std::invalid_argument("weldVerticesLocal: rawV must have 3 columns.");
    }
    if (rawF.cols() != 3) {
        throw std::invalid_argument("weldVerticesLocal: rawF must have 3 columns.");
    }

    const Eigen::Index numFaces = rawF.rows();
    const Eigen::Index numRawVerts = rawV.rows();
    const auto totalCorners = static_cast<std::size_t>(numFaces) * 3;

    // Identify corners belonging to intra-face sub-tolerance edges so we can seed their
    // distinct endpoints atomically in Pass 1 before welding neighboring faces in Pass 2.
    std::vector<std::array<bool, 3>> isSubTolCorner(static_cast<std::size_t>(numFaces),
                                                    {false, false, false});
    std::vector<CornerEntry> sortedCorners;
    sortedCorners.reserve(totalCorners);

    for (Eigen::Index f = 0; f < numFaces; ++f) {
        std::array<Eigen::Vector3d, 3> triPos;
        for (int c = 0; c < 3; ++c) {
            const int vIdx = rawF(f, c);
            if (vIdx < 0 || static_cast<Eigen::Index>(vIdx) >= numRawVerts) {
                throw std::out_of_range("weldVerticesLocal: face vertex index out of bounds.");
            }
            triPos[static_cast<std::size_t>(c)] = rawV.row(vIdx).transpose();
        }

        const bool m01 = tolConfig.verticesMatchWithinTolerance(triPos[0], triPos[1]);
        const bool m12 = tolConfig.verticesMatchWithinTolerance(triPos[1], triPos[2]);
        const bool m20 = tolConfig.verticesMatchWithinTolerance(triPos[2], triPos[0]);

        const auto fIdx = static_cast<std::size_t>(f);
        isSubTolCorner[fIdx][0] = m01 || m20;
        isSubTolCorner[fIdx][1] = m01 || m12;
        isSubTolCorner[fIdx][2] = m12 || m20;

        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d& p = triPos[static_cast<std::size_t>(c)];
            sortedCorners.push_back(CornerEntry{
                computeProjectionKey(p),
                p,
                f,
                c,
                -1
            });
        }
    }

    std::sort(sortedCorners.begin(), sortedCorners.end(),
              [](const CornerEntry& a, const CornerEntry& b) {
                  return std::tie(a.key, a.pos.x(), a.pos.y(), a.pos.z(), a.face, a.corner) <
                         std::tie(b.key, b.pos.x(), b.pos.y(), b.pos.z(), b.face, b.corner);
              });

    std::vector<std::array<std::size_t, 3>> faceCornerSortedIdx(static_cast<std::size_t>(numFaces));
    for (std::size_t idx = 0; idx < sortedCorners.size(); ++idx) {
        const auto fIdx = static_cast<std::size_t>(sortedCorners[idx].face);
        const auto cIdx = static_cast<std::size_t>(sortedCorners[idx].corner);
        faceCornerSortedIdx[fIdx][cIdx] = idx;
    }

    std::vector<std::array<int, 3>> faceVerts(static_cast<std::size_t>(numFaces), {-1, -1, -1});
    std::vector<Eigen::Vector3d> weldedPositions;
    weldedPositions.reserve(static_cast<std::size_t>(numFaces) + 4);

    auto assignCorner = [&](std::size_t idx) {
        CornerEntry& entry = sortedCorners[idx];
        if (entry.weldedId != -1) {
            return;
        }

        const auto fIdx = static_cast<std::size_t>(entry.face);
        const auto cIdx = static_cast<std::size_t>(entry.corner);
        const Eigen::Vector3d& p = entry.pos;
        const double key = entry.key;
        const double maxDeltaKey = computeMaxKeyDelta(p, tolConfig);

        int bestU = -1;
        double bestDistSq = std::numeric_limits<double>::infinity();

        auto considerCandidate = [&](std::size_t candidateIdx) {
            const int u = sortedCorners[candidateIdx].weldedId;
            if (u == -1) {
                return;
            }
            // Edge-Aware Guard: never merge two corners belonging to the same triangle.
            if (faceVerts[fIdx][0] == u || faceVerts[fIdx][1] == u || faceVerts[fIdx][2] == u) {
                return;
            }
            const Eigen::Vector3d& q = weldedPositions[static_cast<std::size_t>(u)];
            if (!tolConfig.verticesMatchWithinTolerance(p, q)) {
                return;
            }
            const double distSq = (p - q).squaredNorm();
            if (distSq < bestDistSq || (distSq == bestDistSq && u < bestU)) {
                bestDistSq = distSq;
                bestU = u;
            }
        };

        // Scan left in 1D key order within maxDeltaKey
        for (std::size_t j = idx; j > 0;) {
            --j;
            if (key - sortedCorners[j].key > maxDeltaKey) {
                break;
            }
            considerCandidate(j);
        }

        // Scan right in 1D key order within maxDeltaKey
        for (std::size_t j = idx + 1; j < sortedCorners.size(); ++j) {
            if (sortedCorners[j].key - key > maxDeltaKey) {
                break;
            }
            considerCandidate(j);
        }

        if (bestU != -1) {
            entry.weldedId = bestU;
            faceVerts[fIdx][cIdx] = bestU;
        } else {
            const int newU = static_cast<int>(weldedPositions.size());
            weldedPositions.push_back(p);
            entry.weldedId = newU;
            faceVerts[fIdx][cIdx] = newU;
        }
    };

    // Pass 1: Assign sub-tolerance intra-face corners face-by-face so both distinct endpoints
    // of any sub-tolerance edge exist before adjacent faces are welded.
    for (Eigen::Index f = 0; f < numFaces; ++f) {
        const auto fIdx = static_cast<std::size_t>(f);
        for (int c = 0; c < 3; ++c) {
            if (isSubTolCorner[fIdx][static_cast<std::size_t>(c)]) {
                assignCorner(faceCornerSortedIdx[fIdx][static_cast<std::size_t>(c)]);
            }
        }
    }

    // Pass 2: Assign all remaining corners in sorted key order.
    for (std::size_t idx = 0; idx < sortedCorners.size(); ++idx) {
        if (sortedCorners[idx].weldedId == -1) {
            assignCorner(idx);
        }
    }

    // Remap welded vertex IDs in deterministic order of first appearance in `rawF`.
    std::vector<int> oldToNew(weldedPositions.size(), -1);
    std::vector<Eigen::Vector3d> finalVertices;
    finalVertices.reserve(weldedPositions.size());

    mesh.F.resize(numFaces, 3);
    for (Eigen::Index f = 0; f < numFaces; ++f) {
        const auto fIdx = static_cast<std::size_t>(f);
        for (int c = 0; c < 3; ++c) {
            const int oldU = faceVerts[fIdx][static_cast<std::size_t>(c)];
            int newU = oldToNew[static_cast<std::size_t>(oldU)];
            if (newU == -1) {
                newU = static_cast<int>(finalVertices.size());
                oldToNew[static_cast<std::size_t>(oldU)] = newU;
                finalVertices.push_back(weldedPositions[static_cast<std::size_t>(oldU)]);
            }
            mesh.F(f, c) = newU;
        }
    }

    mesh.V.resize(static_cast<Eigen::Index>(finalVertices.size()), 3);
    for (std::size_t i = 0; i < finalVertices.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = finalVertices[i].transpose();
    }

    mesh.recomputeNormals();
    return mesh;
}

Mesh weldVerticesLocal(
    const Eigen::MatrixXd& soupV,
    Unit unit,
    const ToleranceConfig& tolConfig) {
    if (soupV.rows() == 0) {
        Mesh emptyMesh;
        emptyMesh.fileUnit = unit;
        return emptyMesh;
    }
    if (soupV.cols() != 3 || (soupV.rows() % 3) != 0) {
        throw std::invalid_argument(
            "weldVerticesLocal: soupV must have 3 columns and a row count divisible by 3.");
    }

    const Eigen::Index numFaces = soupV.rows() / 3;
    Eigen::MatrixXi rawF(numFaces, 3);
    for (Eigen::Index f = 0; f < numFaces; ++f) {
        const int base = static_cast<int>(3 * f);
        rawF(f, 0) = base;
        rawF(f, 1) = base + 1;
        rawF(f, 2) = base + 2;
    }

    return weldVerticesLocal(soupV, rawF, unit, tolConfig);
}

Mesh loadStl(
    std::istream& input,
    Unit unit,
    const ToleranceConfig& tolConfig) {
    if (!input.good()) {
        throw std::runtime_error("loadStl: input stream is not readable.");
    }

    Eigen::MatrixXd rawV;
    Eigen::MatrixXi rawF;
    Eigen::MatrixXd rawN;

    const bool ok = igl::readSTL(input, rawV, rawF, rawN);
    if (!ok || rawF.rows() == 0 || rawV.rows() == 0) {
        throw std::runtime_error("loadStl: failed to parse STL data from stream.");
    }

    return separateTouchingContacts(weldVerticesLocal(rawV, rawF, unit, tolConfig));
}

Mesh separateTouchingContacts(const Mesh& mesh) {
    // With every touching edge paired, each vertex is split into its fans
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = touchingEdges(mesh, topo);
    return splitVerticesByFan(mesh, cornerFans(mesh, topo, touching)).mesh;
}

Mesh loadStl(
    const std::filesystem::path& filepath,
    Unit unit,
    const ToleranceConfig& tolConfig) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("loadStl: failed to open STL file: " + filepath.string());
    }
    return loadStl(file, unit, tolConfig);
}

bool loadStl(
    const std::filesystem::path& filepath,
    Mesh& outMesh,
    Unit unit,
    const ToleranceConfig& tolConfig,
    std::string* errorOut) {
    try {
        outMesh = loadStl(filepath, unit, tolConfig);
        if (errorOut != nullptr) {
            errorOut->clear();
        }
        return true;
    } catch (const std::exception& ex) {
        if (errorOut != nullptr) {
            *errorOut = ex.what();
        }
        return false;
    }
}

} // namespace ogun
