#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <vector>

namespace mesheal {

/// Combinatorial clean-ups that only remove faces: nothing moves and the vertex list `V` is kept as it
/// is (no vertex is renumbered; a vertex no face uses any more stays, unused). Each takes an optional
/// per-face scope mask (one flag per face, empty = every face): faces outside it are never removed.
/// `faceMap` maps each input face to its row in the result (-1 = removed); kept faces keep their order.

struct RepeatedVertexFacesResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap;
    std::size_t removed = 0; ///< Faces that repeat a vertex index or use an invalid one
};

/// Removes the faces in scope that use one vertex twice (collapsed faces) or an index outside `V`.
/// Edge counts do not change: the edge checks skip such faces already.
[[nodiscard]] RepeatedVertexFacesResult removeRepeatedVertexFaces(const Mesh& mesh,
                                                                  std::span<const std::uint8_t> faceInScope = {});

struct DuplicateFacesResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap;
    std::size_t removedSameWinding = 0;     ///< Extra copies of a face, all wound the same way
    std::size_t removedOppositeWinding = 0; ///< Faces removed from copies wound both ways
    std::size_t groups = 0;                 ///< Sets of two or more faces on the same three vertices
    std::size_t groupsSkipped = 0;          ///< Sets left whole because a face that must go is out of scope

    [[nodiscard]] std::size_t removed() const noexcept {
        return removedSameWinding + removedOppositeWinding;
    }
};

/// Resolves every set of faces on the same three vertices:
/// - all wound the same way: the first is kept, the copies are removed;
/// - wound both ways: the faces of the winding that agrees with the faces around it (across its three
///   edges, faces outside the set) are kept, one of them; when neither winding agrees more, the
///   majority winding keeps one face, and equal counts remove them all (libigl's
///   `resolve_duplicated_faces` parity rule; e.g. a wall between two solids that touch across a face).
/// Faces outside the scope are always kept, and the face kept is one of them when there is one. A set
/// with a face outside the scope of a winding that the rule removes (or when it keeps none) is left
/// whole and counted in `groupsSkipped`.
[[nodiscard]] DuplicateFacesResult removeDuplicateFaces(const Mesh& mesh,
                                                        std::span<const std::uint8_t> faceInScope = {});

struct FinRemovalResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap;
    std::vector<Eigen::Index> removedFaces; ///< Input faces removed, ascending
    std::size_t finFaces = 0;         ///< Removed faces of sheets with an open edge of their own (dangling fins)
    std::size_t wallFaces = 0;        ///< Removed faces of sheets bounded only by Fin / other edges (internal walls)
    std::size_t finEdgesBefore = 0;   ///< Fin / other edges in the input
    std::size_t finEdgesLeft = 0;     ///< Fin / other edges still there (no safe choice, or a sheet out of scope)
    std::size_t seamEdgesOpened = 0;  ///< Edges that had two or more faces and now have one (a seam a wall held shut)
    std::size_t sheetsOutOfScope = 0; ///< Sheets that would have gone but have a face outside the scope
    int rounds = 0;                   ///< Detect-and-remove rounds run
};

/// Removes fins and internal walls: the extra sheets at *Fin / other* non-manifold edges (edges with
/// three or more faces that `touchingEdges` cannot pair; touching contacts are never touched).
/// Faces are grouped into **sheets**: joined across two-face edges and across the pairs of touching
/// edges, never across a Fin / other edge, nor across a two-face edge whose two ends both lie on Fin /
/// other edges and whose faces bend by more than `kFinSeamCreaseAngleDeg` (the seam goes on there: a
/// wall can hold it shut). At each Fin / other edge the sheets kept are
/// - the one sheet that has exactly two faces there, run in opposite directions (it closes the edge
///   by itself: a dangling fin on a body), or
/// - when every sheet has one face there, the two of largest area (a wall between two halves);
/// every other sheet at the edge is a fin there, provided its area is at most `kFinMaxAreaShare` of
/// each kept sheet's. Anything else (two closing sheets, a sheet with three faces there, sheets of
/// similar size) is left as it is. A sheet is removed when it is a fin at some edge, kept at none, and
/// wholly in scope. Repeats up to `kFinRemovalMaxRounds` times. Only removes faces; `V` is kept whole.
[[nodiscard]] FinRemovalResult removeFinFaces(const Mesh& mesh, std::span<const std::uint8_t> faceInScope = {});

/// What `findTinyFloatingShells` looks at.
struct TinyShellSearch {
    double fraction = settings::kDefaultFixAllTinyShellFraction; ///< Of the whole model's bounding-box diagonal; 0 finds none
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Fraction of the bbox diagonal (`ShellRelations`)
    std::vector<int> scopeShells; ///< Shells that may qualify (indices into the shell list); empty = every shell
    /// Relations over `shellsLargestFirst(mesh)` covering every pair with a candidate (all pairs, or
    /// `involving` the candidates), or nullptr: computed here, only for candidates whose boxes meet another.
    const ShellRelations* relations = nullptr;
};

struct TinyFloatingShell {
    int shell = -1;                   ///< Index in the shell list (`shellsLargestFirst` order)
    std::vector<Eigen::Index> faces;
    double sizeFraction = 0.0;        ///< Its bounding-box diagonal / the whole model's
};

struct TinyFloatingShellsResult {
    std::vector<TinyFloatingShell> shells; ///< In shell order
    double modelDiagonal = 0.0;            ///< Bounding-box diagonal of every vertex a face uses
    bool complete = true;                  ///< False when stopped (then `shells` is empty)

    [[nodiscard]] double largestFraction() const noexcept;
};

/// Shells that are debris: bounding-box diagonal below `fraction` x the whole model's, and floating:
/// they cross, share an area with, touch, contain and lie inside no other shell. A box test first (a
/// shell whose box, grown by the tolerance, meets no other shell's box floats); `ShellRelations` only
/// for the candidates whose boxes meet another. A shell whose nesting is unsure does not float.
/// Cavities and islands lie inside a shell, so they never qualify. Face count is not used.
[[nodiscard]] TinyFloatingShellsResult findTinyFloatingShells(
    const Mesh& mesh,
    const TinyShellSearch& search = {},
    std::stop_token stopToken = {});

struct TinyShellRemovalResult {
    Mesh mesh;
    std::vector<Eigen::Index> faceMap;
    TinyFloatingShellsResult found; ///< The shells removed (input face numbers)
    std::size_t removedFaces = 0;
};

/// Removes the tiny floating shells (`findTinyFloatingShells`) that lie wholly in scope (a per-face
/// mask, empty = every face). Only removes faces; `V` is kept whole. Unchanged mesh when stopped.
[[nodiscard]] TinyShellRemovalResult removeTinyFloatingShells(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope = {},
    double fraction = settings::kDefaultFixAllTinyShellFraction,
    double intersectionTolerance = settings::kDefaultIntersectionTolerance,
    std::stop_token stopToken = {});

} // namespace mesheal
