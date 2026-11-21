#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string_view>
#include <vector>

namespace mesheal {

/// What a pair of faces reported by the intersection check really is, judged in this order: neighbours
/// first (by shared vertex indices), then the kind of contact.
enum class CrossingKind : std::uint8_t {
    Folded,          ///< The faces share an edge and lie folded flat onto each other
    VertexNeighbour, ///< The faces share one vertex and cross by more than the tolerance
    Coplanar,        ///< The faces lie in one plane and share an area
    Touching,        ///< One face only rests on the other (no passage through)
    Crossing         ///< One face passes through the other
};

inline constexpr std::size_t kCrossingKindCount = 5;

[[nodiscard]] std::string_view crossingKindName(CrossingKind kind) noexcept;

/// One face pair the intersection check reports, classified, with the shells of both faces
/// (`shellsLargestFirst` indices).
struct CrossingPair {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    CrossingKind kind = CrossingKind::Crossing;
    int shellA = -1;
    int shellB = -1;

    [[nodiscard]] bool withinShell() const noexcept { return shellA == shellB; }
};

/// Shell index of every face (`shellsLargestFirst` order: 0 is the largest shell).
[[nodiscard]] std::vector<int> shellIndexOfFaces(const Mesh& mesh);

/// The face pairs behind `checkSelfIntersections` (same tolerance), classified. Sorted by (a, b).
/// std::nullopt when stopped.
[[nodiscard]] std::optional<std::vector<CrossingPair>> classifyCrossings(const Mesh& mesh, const CheckParams& params,
                                                                        std::stop_token stopToken = {});

/// The state of a mesh in numbers: every Mesh Health count, plus the crossing pairs by kind, within one
/// shell and between two. Used to trace what each repair step leaves behind, and as the input of
/// decisions that depend on what a mesh actually has.
struct MeshState {
    std::size_t faces = 0;
    std::size_t vertices = 0;
    std::size_t shells = 0;
    double volume = 0.0; ///< Signed (positive when the faces point outward)
    std::size_t openEdges = 0;
    std::size_t nonManifoldEdges = 0;
    std::size_t nonManifoldVertices = 0;
    std::size_t flippedFaces = 0;
    std::size_t invertedShells = 0;
    std::size_t degenerateFaces = 0;
    std::size_t duplicateFaces = 0;
    std::size_t thinFaces = 0;
    std::size_t crossingFaces = 0; ///< Faces in at least one reported pair (the Intersections count)
    std::array<std::size_t, kCrossingKindCount> pairsWithinShell{}; ///< By `CrossingKind`
    std::array<std::size_t, kCrossingKindCount> pairsBetweenShells{};
    bool complete = true; ///< False when stopped part way
};

/// Measures `mesh` (all of it). Runs every check once, so it costs about as much as a full Mesh Health.
[[nodiscard]] MeshState measureMeshState(const Mesh& mesh, const CheckParams& params, std::stop_token stopToken = {});

} // namespace mesheal
