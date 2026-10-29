#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <atomic>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace ogun {

/// Faces that pass through other faces of the same mesh, found the way Blender's 3D-Print Toolbox
/// does: a self-overlap traversal of an AABB tree, then Möller's triangle test on each candidate pair.
///
/// One element per intersecting face, most crossings first; `relatedOf(i)` lists the faces it crosses.
/// Edge neighbours are reported only when folded flat onto each other, vertex neighbours only when
/// they cross by more than the tolerance. Runs on up to `settings::kIntersectionMaxThreads` threads.
[[nodiscard]] CategoryResult checkSelfIntersections(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr,
    unsigned maxThreads = settings::kIntersectionMaxThreads);

/// Which face pairs `findFacePairs` tests.
struct FacePairQuery {
    /// Group of each face (e.g. its shell). Empty: every pair is tested. Otherwise only pairs from two
    /// different groups are tested, and faces in a negative group are left out.
    std::span<const int> faceGroup;
    /// Per group: 1 = of interest. Empty: every group. Otherwise a pair needs a group of interest.
    std::span<const std::uint8_t> groupOfInterest;
    /// Also report pairs that do not intersect but come within the tolerance (`trianglesWithin`).
    bool contacts = false;
};

/// A face pair found by `findFacePairs` (a < b).
struct FacePairHit {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    bool hit = false;      ///< Reported by the intersection check (`intersectTriangles`)
    bool coplanar = false; ///< A hit in one plane: the faces share an area
    bool touching = false; ///< Not a crossing: a hit where one face only rests on the other, or a contact
};

/// The face pairs behind `checkSelfIntersections` (same tree, same test, same tolerance `tol`, a length),
/// restricted by `query`. Unsorted. std::nullopt when stopped.
[[nodiscard]] std::optional<std::vector<FacePairHit>> findFacePairs(
    const Mesh& mesh,
    double tol,
    const FacePairQuery& query,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr,
    unsigned maxThreads = settings::kIntersectionMaxThreads);

} // namespace ogun
