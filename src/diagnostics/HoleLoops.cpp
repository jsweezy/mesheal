#include "diagnostics/HoleLoops.hpp"

#include "core/Constants.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace mesheal {

namespace {

std::uint32_t findRoot(std::vector<std::uint32_t>& parent, std::uint32_t x) noexcept {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

/// Orders the rim edges into walks that follow each edge with one leaving its end vertex.
void orderRim(HoleLoop& hole) {
    std::unordered_map<int, std::vector<std::size_t>> leaving;
    for (std::size_t i = 0; i < hole.edges.size(); ++i) {
        leaving[hole.edges[i][0]].push_back(i);
    }
    std::vector<std::uint8_t> used(hole.edges.size(), 0);
    std::vector<std::array<int, 2>> edges;
    std::vector<Eigen::Index> faces;
    edges.reserve(hole.edges.size());
    faces.reserve(hole.faces.size());
    const std::size_t n = hole.edges.size();
    for (std::size_t start = 0; start < n; ++start) {
        std::size_t i = start;
        while (i < n && used[i] == 0) {
            used[i] = 1;
            edges.push_back(hole.edges[i]);
            faces.push_back(hole.faces[i]);
            const auto it = leaving.find(hole.edges[i][1]);
            i = n;
            if (it != leaving.end()) {
                const auto next = std::find_if(it->second.begin(), it->second.end(), [&](std::size_t j) { return used[j] == 0; });
                i = next != it->second.end() ? *next : n;
            }
        }
    }
    hole.edges = std::move(edges);
    hole.faces = std::move(faces);
}

} // namespace

const char* holeShapeLabel(HoleShape shape) noexcept {
    switch (shape) {
    case HoleShape::Flat:
        return "Flat";
    case HoleShape::Curved:
        return "Curved";
    case HoleShape::Crack:
        return "Crack";
    case HoleShape::OpenChain:
        return "Open chain";
    }
    return "?";
}

void measureHoleLoop(const Mesh& mesh, HoleLoop& hole, const HoleShapeParams& params) {
    hole.vertices.clear();
    std::unordered_map<int, int> balance;
    for (const auto& [a, b] : hole.edges) {
        hole.vertices.push_back(a);
        hole.vertices.push_back(b);
        ++balance[a];
        --balance[b];
    }
    std::sort(hole.vertices.begin(), hole.vertices.end());
    hole.vertices.erase(std::unique(hole.vertices.begin(), hole.vertices.end()), hole.vertices.end());
    hole.closed = !hole.edges.empty() &&
                  std::all_of(balance.begin(), balance.end(), [](const auto& entry) { return entry.second == 0; });

    hole.centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo;
    for (const int v : hole.vertices) {
        const Eigen::Vector3d p = mesh.V.row(v).transpose();
        hole.centroid += p;
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    if (hole.vertices.empty()) {
        hole.shape = HoleShape::OpenChain;
        return;
    }
    hole.centroid /= static_cast<double>(hole.vertices.size());
    hole.size = (hi - lo).norm();

    hole.perimeter = 0.0;
    Eigen::Vector3d vectorArea = Eigen::Vector3d::Zero();
    for (const auto& [a, b] : hole.edges) {
        const Eigen::Vector3d pa = mesh.V.row(a).transpose() - hole.centroid;
        const Eigen::Vector3d pb = mesh.V.row(b).transpose() - hole.centroid;
        hole.perimeter += (pb - pa).norm();
        vectorArea += 0.5 * pa.cross(pb);
    }
    hole.meanEdgeLength = hole.perimeter / static_cast<double>(hole.edges.size());
    hole.area = vectorArea.norm();
    hole.width = hole.perimeter > 0.0 ? 2.0 * hole.area / hole.perimeter : 0.0;

    // Best-fit plane: the smallest principal axis of the rim vertices
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const int v : hole.vertices) {
        const Eigen::Vector3d d = mesh.V.row(v).transpose() - hole.centroid;
        covariance += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    hole.normal = solver.eigenvectors().col(0);
    // The rim runs against its faces, so it circles the hole counter-clockwise seen from outside
    if (hole.area > 0.0 && hole.normal.dot(vectorArea) < 0.0) {
        hole.normal = -hole.normal;
    }
    hole.planeDeviation = 0.0;
    for (const int v : hole.vertices) {
        hole.planeDeviation = std::max(hole.planeDeviation, std::abs((mesh.V.row(v).transpose() - hole.centroid).dot(hole.normal)));
    }

    const double coplanar = std::cos(params.flatNeighbourDeg * kDegToRad);
    const double perpendicular = std::sin(params.flatNeighbourDeg * kDegToRad);
    std::size_t counted = 0;
    std::size_t flat = 0;
    for (const Eigen::Index f : hole.faces) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0)).transpose();
        const Eigen::Vector3d n = (Eigen::Vector3d(mesh.V.row(mesh.F(f, 1)).transpose()) - a)
                                      .cross(Eigen::Vector3d(mesh.V.row(mesh.F(f, 2)).transpose()) - a);
        if (!(n.norm() > 0.0)) {
            continue;
        }
        const double c = std::abs(n.normalized().dot(hole.normal));
        ++counted;
        flat += c >= coplanar || c <= perpendicular ? 1 : 0;
    }
    hole.flatNeighbourShare = counted > 0 ? static_cast<double>(flat) / static_cast<double>(counted) : 0.0;

    if (!hole.closed) {
        hole.shape = HoleShape::OpenChain;
    } else if (hole.width < params.crackWidthRatio * hole.meanEdgeLength) {
        hole.shape = HoleShape::Crack;
    } else if (hole.planeDeviation <= params.flatTolerance * hole.size && hole.flatNeighbourShare >= params.flatNeighbourShare) {
        hole.shape = HoleShape::Flat;
    } else {
        hole.shape = HoleShape::Curved;
    }
}

std::vector<HoleLoop> findHoleLoops(const Mesh& mesh, const EdgeTopology& topo, const HoleShapeParams& params) {
    // Holes: connected components of the open-edge graph, numbered by their first open edge
    std::vector<std::uint32_t> parent(static_cast<std::size_t>(mesh.numVertices()));
    std::iota(parent.begin(), parent.end(), std::uint32_t{0});
    std::vector<std::size_t> openEdges;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) == 1) {
            openEdges.push_back(e);
            const std::uint32_t a = findRoot(parent, static_cast<std::uint32_t>(topo.edges[e][0]));
            const std::uint32_t b = findRoot(parent, static_cast<std::uint32_t>(topo.edges[e][1]));
            parent[std::max(a, b)] = std::min(a, b);
        }
    }
    std::unordered_map<std::uint32_t, std::size_t> holeOfRoot;
    std::vector<HoleLoop> holes;
    for (const std::size_t e : openEdges) {
        const std::uint32_t root = findRoot(parent, static_cast<std::uint32_t>(topo.edges[e][0]));
        const auto [it, inserted] = holeOfRoot.try_emplace(root, holes.size());
        if (inserted) {
            holes.emplace_back();
        }
        const std::uint32_t i = topo.faceOffsets[e];
        const auto [a, b] = topo.edges[e];
        // The face runs a -> b when forward, so the rim (the hole's side) runs b -> a
        HoleLoop& hole = holes[it->second];
        hole.edges.push_back(topo.incidentForward[i] != 0 ? std::array<int, 2>{b, a} : std::array<int, 2>{a, b});
        hole.faces.push_back(static_cast<Eigen::Index>(topo.incidentFaces[i]));
    }
    for (HoleLoop& hole : holes) {
        orderRim(hole);
        measureHoleLoop(mesh, hole, params);
    }
    return holes;
}

std::vector<HoleLoop> findHoleLoops(const Mesh& mesh, const HoleShapeParams& params) {
    return findHoleLoops(mesh, buildEdgeTopology(mesh), params);
}

int findHoleOfEdge(std::span<const HoleLoop> holes, int a, int b) noexcept {
    for (std::size_t h = 0; h < holes.size(); ++h) {
        for (const auto& edge : holes[h].edges) {
            if ((edge[0] == a && edge[1] == b) || (edge[0] == b && edge[1] == a)) {
                return static_cast<int>(h);
            }
        }
    }
    return -1;
}

} // namespace mesheal
