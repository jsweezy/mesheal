#pragma once

#include "core/Settings.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <atomic>
#include <cstddef>
#include <set>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace ogun {

/// What `computeShellRelations` works out, and for which shells.
struct ShellRelationParams {
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Fraction of the bbox diagonal (`CheckParams`)
    bool contacts = true;      ///< Also look for edge and point contacts (pairs that touch without crossing)
    std::vector<int> involving; ///< Empty: every pair of shells. Otherwise only pairs with one of these shells
    unsigned maxThreads = settings::kIntersectionMaxThreads;
};

/// Facts about one shell.
struct ShellFacts {
    std::vector<Eigen::Index> faces;
    BoundingBox box;
    bool closed = false;        ///< Every edge is shared by exactly two of its faces (repeated-vertex faces ignored)
    double signedVolume = 0.0;  ///< Positive when its faces point outward; a cavity is negative
    int nestedIn = -1;          ///< Smallest shell it lies inside (-1: none, or nesting unsure)
    std::vector<int> containers; ///< Every shell it lies inside, smallest box first
    bool nestingUnsure = false; ///< The vertex and face-centroid probes disagreed: treated as not nested
};

/// How two shells meet, counted in face pairs. Only pairs that meet at all are listed.
struct ShellPairRelation {
    int a = -1; ///< Lower shell index
    int b = -1;
    std::size_t crossings = 0;        ///< Face pairs that pass through each other
    std::size_t sharedAreaPairs = 0;  ///< Coplanar face pairs that overlap in area (a shared face area)
    std::size_t contacts = 0;         ///< Face pairs that only touch: an edge or corner on a face, or within the tolerance
    std::size_t intersectingPairs = 0; ///< Face pairs `checkSelfIntersections` reports between them (crossings, shared areas and edges resting on faces)

    [[nodiscard]] bool crosses() const noexcept {
        return crossings > 0;
    }

    [[nodiscard]] bool sharesFaceArea() const noexcept {
        return sharedAreaPairs > 0;
    }

    /// They touch along an edge or at a point only: no crossing, no shared face area.
    [[nodiscard]] bool edgeOrPointContactOnly() const noexcept {
        return crossings == 0 && sharedAreaPairs == 0 && contacts > 0;
    }
};

/// Crossings, shared face areas, edge / point contacts and nesting between the shells of a mesh.
struct ShellRelations {
    std::vector<ShellFacts> shells;
    std::vector<ShellPairRelation> pairs; ///< Sorted by (a, b)
    double tolerance = 0.0;               ///< Absolute (file units)
    bool complete = true;                 ///< False when stopped: `pairs` and the nesting are empty

    /// The relation of shells a and b (either order), nullptr if they do not meet.
    [[nodiscard]] const ShellPairRelation* pair(int a, int b) const noexcept;

    /// Shell pairs (low, high) whose faces cross or share an area (the pairs ruled out of nesting).
    [[nodiscard]] std::set<std::pair<int, int>> overlappingPairs() const;
};

/// Edge-connected shells (`checkShells`) in the Shells panel's order: most faces first, then lowest
/// first face. Faces with a repeated vertex index are not in any shell.
[[nodiscard]] std::vector<std::vector<Eigen::Index>> shellsLargestFirst(const Mesh& mesh);

/// For every pair of the given shells: do they cross, share a face area, or touch only along an edge or
/// at a point; and which shell lies inside which. Pairs are tested with the intersection check's tree
/// and triangle test (tolerance `params.intersectionTolerance` x the mesh's bbox diagonal), between
/// different shells only, and only where the shells' boxes overlap. Nesting follows `findNestedShells`
/// (pairs that cross or share an area are never nested), probed from a vertex and from a face centroid;
/// a shell whose probes disagree is marked `nestingUnsure` and treated as not nested. With
/// `params.involving` set, crossings of other pairs are not known, so their nesting is not ruled out.
/// Polls `stopToken`; reports 0..1 through `progress`.
[[nodiscard]] ShellRelations computeShellRelations(
    const Mesh& mesh,
    std::span<const std::vector<Eigen::Index>> shellFaces,
    const ShellRelationParams& params = {},
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

/// `computeShellRelations` over `shellsLargestFirst(mesh)`, so shell k is the Shells panel's #(k + 1).
[[nodiscard]] ShellRelations computeShellRelations(
    const Mesh& mesh,
    const ShellRelationParams& params = {},
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace ogun
