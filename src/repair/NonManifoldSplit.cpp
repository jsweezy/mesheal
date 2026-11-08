#include "repair/NonManifoldSplit.hpp"

#include "diagnostics/MeshChecks.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mesheal {

namespace {

NonManifoldSplitResult fromVertexSplit(VertexSplit split) {
    NonManifoldSplitResult result;
    result.mesh = std::move(split.mesh);
    result.verticesSplit = split.verticesSplit;
    result.verticesAdded = split.verticesAdded;
    return result;
}

} // namespace

NonManifoldSplitResult splitNonManifoldVertices(const Mesh& mesh, std::span<const Eigen::Index> vertices) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    std::vector<std::uint8_t> target(static_cast<std::size_t>(mesh.numVertices()), 0);
    for (const Eigen::Index v : vertices) {
        if (v >= 0 && v < mesh.numVertices()) {
            target[static_cast<std::size_t>(v)] = 1;
        }
    }
    return fromVertexSplit(splitVerticesByFan(mesh, cornerFans(mesh, topo), target));
}

NonManifoldSplitResult splitNonManifoldEdges(const Mesh& mesh, std::span<const std::array<int, 2>> edges) {
    // Every touching edge is paired; only the seams of the listed edges are split
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = touchingEdges(mesh, topo);
    std::vector<std::uint8_t> target(static_cast<std::size_t>(mesh.numVertices()), 0);
    std::vector<int> queue;
    std::size_t edgesSkipped = 0;
    for (const auto& edge : edges) {
        const auto it = std::lower_bound(topo.edges.begin(), topo.edges.end(), edge);
        if (it == topo.edges.end() || *it != edge) {
            continue;
        }
        const auto e = static_cast<std::size_t>(it - topo.edges.begin());
        if (topo.faceCount(e) < 3) {
            continue;
        }
        if (touching[e] == 0) {
            ++edgesSkipped;
            continue;
        }
        queue.push_back(edge[0]);
        queue.push_back(edge[1]);
    }

    // Split each listed edge's whole seam: splitting one end of a touching edge separates it, which
    // would leave its other end pinched unless that is split too
    std::unordered_map<int, std::vector<int>> seamNeighbors;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (touching[e] != 0) {
            seamNeighbors[topo.edges[e][0]].push_back(topo.edges[e][1]);
            seamNeighbors[topo.edges[e][1]].push_back(topo.edges[e][0]);
        }
    }
    while (!queue.empty()) {
        const int v = queue.back();
        queue.pop_back();
        if (target[static_cast<std::size_t>(v)] != 0) {
            continue;
        }
        target[static_cast<std::size_t>(v)] = 1;
        for (const int w : seamNeighbors[v]) {
            queue.push_back(w);
        }
    }
    std::size_t edgesSplit = 0;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (touching[e] != 0 && target[static_cast<std::size_t>(topo.edges[e][0])] != 0) {
            ++edgesSplit;
        }
    }

    NonManifoldSplitResult result = fromVertexSplit(splitVerticesByFan(mesh, cornerFans(mesh, topo, touching), target));
    result.edgesSplit = edgesSplit;
    result.edgesSkipped = edgesSkipped;
    return result;
}

} // namespace mesheal
