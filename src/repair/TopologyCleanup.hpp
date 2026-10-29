#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ogun {

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

} // namespace ogun
