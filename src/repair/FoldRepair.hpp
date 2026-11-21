#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>

namespace mesheal {

/// What `unfoldFaces` did.
struct FoldRepairResult {
    std::size_t folds = 0;    ///< Folded pairs found among the target faces
    std::size_t unfolded = 0; ///< Folds removed
    int flips = 0;            ///< ... by flipping the shared edge
    int collapses = 0;        ///< ... by collapsing a face's tip onto the shared edge
    double maxMove = 0.0;     ///< Farthest a vertex moved
    bool tooMany = false;     ///< More folds than `settings::kMaxFoldRepairFolds`: none was tried
    bool cancelled = false;
};

/// Unfolds pairs of faces that share an edge and lie folded flat onto each other (the intersection check's
/// "folded" pairs), where both faces are targets (`target` empty: every face). No other repair removes
/// them: the faces are not thin, and the island patches cannot cut along a shared edge.
///
/// For each fold it tries flipping the shared edge (nothing moves) and collapsing the tip of either face
/// onto either end of the shared edge. A change qualifies when it leaves every edge manifold and
/// consistently wound and the collapsed vertex one fan, makes no face degenerate, moves no vertex farther
/// than `maxMove`, and leaves fewer crossing pairs among the faces it touches than there were. Of those it
/// keeps the one that leaves the fewest, then the smallest move. A face changed for one fold is not
/// changed again in the same call. A mesh with more than `settings::kMaxFoldRepairFolds` folds is left as it
/// is (`tooMany`). Changes `mesh` in place; unused vertices are removed.
[[nodiscard]] FoldRepairResult unfoldFaces(Mesh& mesh, const CheckParams& params, double maxMove,
                                           std::span<const std::uint8_t> target = {}, std::stop_token stopToken = {});

} // namespace mesheal
