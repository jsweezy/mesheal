#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <vector>

namespace mesheal {

/// What `orientShells` does.
struct OrientShellsOptions {
    bool flipFaces = true;        ///< (a) Flip faces wound against the majority of their shell (majority wins)
    bool reverseInsideOut = true; ///< (b) Rule O: reverse each closed shell it calls inside out
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Fraction of the bbox diagonal
    /// Pairs and nesting of the input's shells (`shellsLargestFirst(mesh)` order), holding every pair (or
    /// at least every pair rule O needs after the face pass). nullptr: computed here when rule O needs
    /// them. Flipping faces changes no shell and no pair, so only the winding facts are recomputed.
    const ShellRelations* relations = nullptr;
};

/// Result of `orientShells`. Shell indices refer to `shells` (`shellsLargestFirst` order, which is the
/// Shells panel's unless faces repeat a vertex).
struct OrientShellsResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap;             ///< Identity: orientation removes and adds no face
    std::vector<std::vector<Eigen::Index>> shells; ///< Faces of each shell
    std::size_t flippedFaces = 0;                  ///< Faces flipped to agree with their shell's majority
    std::vector<int> shellsWithFlippedFaces;
    std::vector<int> reversedShells;               ///< Inside-out shells reversed whole (rule O)
    std::vector<int> cavitiesKept;                 ///< Cavities: wound inward inside a solid, left as they are
    std::vector<std::vector<int>> stopped;         ///< Inward bodies holding shells, outer shell first: not flipped (stop and report)
    std::vector<int> unclassified;                 ///< Inward closed shells rule O cannot judge: not flipped
    std::vector<int> selfCrossing;                 ///< Would be inside out, but crosses itself: not flipped
    std::vector<int> nonOrientable;                ///< Faces cannot all agree (a Moebius-like twist): left as they are
    std::vector<int> outOfScope;                   ///< Inside out, but not wholly in scope: not flipped
    ShellRoles roles;     ///< Rule O's roles after the face pass (empty when no closed shell is wound inward)
    bool complete = true; ///< False when stopped: `mesh` is the input, nothing reported
};

/// Orients the faces of every shell without ever flipping a cavity.
/// (a) Over each patch of faces joined by two-face edges, the relative winding is propagated from its
///     first face; the minority faces are flipped (ties keep the first face's winding), as
///     `checkFlippedNormals` counts them. A patch where the propagation meets itself the other way round
///     (a Moebius strip) is left as it is and its shell reported.
/// (b) Rule O (`classifyShellRoles`) on the result: only `InsideOut` shells are reversed, and only those
///     wholly in scope and not crossing themselves (checked for those shells only). Cavities, nested
///     solids, islands, stopped bodies and unclassified shells are never flipped. Nesting and contacts
///     are read from the whole mesh. Relations are computed only when a closed shell is wound inward,
///     for the shells rule O needs (`shellsRuleONeeds`), as the Inverted shells check does.
/// Faces outside `faceInScope` (one flag per face, empty = every face) are never flipped. Never calls
/// CGAL's `orient` or `orient_to_bound_a_volume`. `V` is untouched.
[[nodiscard]] OrientShellsResult orientShells(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope = {},
    const OrientShellsOptions& options = {},
    std::stop_token stopToken = {});

struct FlipShellsResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap; ///< Identity
    std::size_t flippedFaces = 0;
};

/// Reverses the winding of every face listed (each face once, however often it is listed): the user's
/// *Flip* for shells Fix All will not guess about. The faces are the scope; nothing else changes.
[[nodiscard]] FlipShellsResult flipShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shellFaceSets);

} // namespace mesheal
