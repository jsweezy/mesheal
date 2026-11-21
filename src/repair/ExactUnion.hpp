#pragma once

#include "mesh/Mesh.hpp"

#include <cstddef>
#include <stop_token>
#include <string>

namespace mesheal {

/// How `exactSelfUnion` ended.
enum class ExactUnionStatus {
    United,    ///< `mesh` is the union (empty when the input encloses no volume)
    NotClosed, ///< The input is not closed and consistently wound, so "inside" is not defined
    TooLarge,  ///< More faces than the limit: the exact cut cannot be stopped once started
    Failed,    ///< The exact cut threw (see `message`)
    Cancelled  ///< Stopped before the cut started
};

struct ExactUnionResult {
    ExactUnionStatus status = ExactUnionStatus::Failed;
    Mesh mesh;
    std::string message;
};

/// Exact self-union of a closed, consistently wound surface (libigl's `copyleft::cgal::mesh_boolean`):
/// every crossing is cut with exact arithmetic, the winding number is carried exactly from piece to
/// piece, and only the pieces between inside (winding >= 1) and outside are kept, rounded back to
/// doubles. Folds that pass through the surface are resolved; walls inside the solid are dropped.
/// Faces that only come within the intersection tolerance of each other (no exact crossing) are
/// left as they are. The cut has no stop point: `stopToken` is checked before it starts, and inputs
/// with more than `maxFaces` faces are refused instead (`TooLarge`).
[[nodiscard]] ExactUnionResult exactSelfUnion(const Mesh& mesh, std::size_t maxFaces, std::stop_token stopToken = {});

/// Deletes every shell of `mesh` wound inward (negative volume): the pockets a union of one folded shell
/// encloses, which only exist because the surface passed through itself. Returns how many were filled;
/// `volumeOut` (optional) gets the volume they enclosed (positive).
std::size_t fillEnclosedPockets(Mesh& mesh, double* volumeOut = nullptr);

} // namespace mesheal
