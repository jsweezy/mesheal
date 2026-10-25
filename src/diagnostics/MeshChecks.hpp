#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <array>
#include <cstdint>
#include <stop_token>
#include <vector>

namespace ogun {

/// Undirected edges of a face-vertex mesh with their incident faces. Faces with a repeated vertex
/// index are skipped (the Degenerate check reports them).
struct EdgeTopology {
    std::vector<std::array<int, 2>> edges;      ///< Sorted vertex pair of each edge
    std::vector<std::uint32_t> faceOffsets{0};  ///< CSR: edge e owns incidences [faceOffsets[e], faceOffsets[e + 1])
    std::vector<std::uint32_t> incidentFaces;   ///< Face of each incidence
    std::vector<std::uint8_t> incidentForward;  ///< 1 if that face traverses the edge edges[e][0] -> edges[e][1]

    [[nodiscard]] std::size_t numEdges() const noexcept {
        return edges.size();
    }

    [[nodiscard]] std::uint32_t faceCount(std::size_t edge) const noexcept {
        return faceOffsets[edge + 1] - faceOffsets[edge];
    }
};

/// True if face `f` references a vertex index twice.
[[nodiscard]] bool faceHasRepeatedVertex(const Mesh& mesh, Eigen::Index f) noexcept;

/// Builds the undirected edge table by sorting packed vertex pairs: O(F log F).
[[nodiscard]] EdgeTopology buildEdgeTopology(const Mesh& mesh);

/// Edges used by exactly one triangle. `infoCount` = number of boundary loops.
[[nodiscard]] CategoryResult checkOpenEdges(const Mesh& mesh, std::stop_token stopToken = {});

/// Edges shared by three or more triangles.
[[nodiscard]] CategoryResult checkNonManifoldEdges(const Mesh& mesh, std::stop_token stopToken = {});

/// Vertices whose incident triangles form more than one edge-connected fan (bow-ties).
[[nodiscard]] CategoryResult checkNonManifoldVertices(const Mesh& mesh, std::stop_token stopToken = {});

/// Triangles wound opposite to the majority of their edge-connected shell.
[[nodiscard]] CategoryResult checkFlippedNormals(const Mesh& mesh, std::stop_token stopToken = {});

/// Edge-connected pieces, smallest first. `infoCount` = number of shells.
[[nodiscard]] CategoryResult checkShells(const Mesh& mesh, std::stop_token stopToken = {});

/// Thin / sliver faces from `ThinFaceChecker`, sorted by ascending h_min, bucketed by shape.
[[nodiscard]] CategoryResult checkThinFaces(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken = {});

/// Faces with a repeated vertex index or area <= `degenerateAreaEpsilon * diag^2`.
[[nodiscard]] CategoryResult checkDegenerateFaces(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken = {});

/// Faces with the same three vertices as an earlier face (the later copies are reported).
[[nodiscard]] CategoryResult checkDuplicateFaces(const Mesh& mesh, std::stop_token stopToken = {});

} // namespace ogun
