#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
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

/// Pairs the faces around non-manifold edge `e` into closed wedges when the edge is where solids
/// touch: faces sorted radially around the edge, each face paired with the neighbour across the
/// solid it bounds (decided by winding), every pair one forward and one backward face. Returns the
/// pairs as face indices, or nothing for fins, odd counts, mixed winding, zero-area or coincident faces.
[[nodiscard]] std::optional<std::vector<std::array<std::uint32_t, 2>>> pairTouchingFaces(
    const Mesh& mesh,
    const EdgeTopology& topo,
    std::size_t e);

/// Fan id of every corner (`3 * face + c`; `kNoFan` for faces with a repeated vertex index). Corners
/// of a vertex share a fan when their faces are joined across edges through it. Non-manifold edges
/// join all their faces, except edges flagged in `pairEdges` (one flag per edge, or empty) that
/// `pairTouchingFaces` can pair: those join only the faces of each pair.
[[nodiscard]] std::vector<std::uint32_t> cornerFans(
    const Mesh& mesh,
    const EdgeTopology& topo,
    std::span<const std::uint8_t> pairEdges = {});

/// One flag per edge: 1 for non-manifold edges where solids touch, i.e. `pairTouchingFaces` pairs
/// their faces and, with every such edge paired, copying vertices by `cornerFans` gives each pair
/// its own edge.
[[nodiscard]] std::vector<std::uint8_t> touchingEdges(const Mesh& mesh, const EdgeTopology& topo);

/// Holes: one element per loop of edges used by exactly one triangle (`findHoleLoops` order), with
/// its rim faces and rim edges (`edgesOf`). `infoCount` = number of open edges.
[[nodiscard]] CategoryResult checkOpenEdges(const Mesh& mesh, std::stop_token stopToken = {});

/// Edges shared by three or more triangles. Sub-type 0 (Touching) for `touchingEdges`, which copying
/// vertices fixes; sub-type 1 (Fin / other) otherwise.
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

/// Faces with an invalid or repeated vertex index, or with no reliable normal (`hasReliableNormal`).
[[nodiscard]] CategoryResult checkDegenerateFaces(
    const Mesh& mesh,
    std::stop_token stopToken = {});

/// Faces with the same three vertices as an earlier face (the later copies are reported).
[[nodiscard]] CategoryResult checkDuplicateFaces(const Mesh& mesh, std::stop_token stopToken = {});

} // namespace ogun
