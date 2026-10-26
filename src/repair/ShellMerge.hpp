#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace ogun {

/// Outcome of `mergeShells`. On failure `mesh` is empty and `message` says why.
struct ShellMergeResult {
    bool success = false;
    std::string message;
    int rejectedShell = -1;         ///< Index into the input shells Manifold refused (-1: none)
    Mesh mesh;                      ///< Whole mesh: the other shells' faces, then the union's
    Eigen::Index firstMergedFace = 0; ///< Faces from here on came from the union
    std::size_t shellsIn = 0;
    std::size_t shellsOut = 0;      ///< Separate pieces of the union (shells that never touched stay apart)
    double volumeBefore = 0.0;      ///< Sum of the input shells' volumes (overlaps counted twice)
    double volumeAfter = 0.0;
    double areaBefore = 0.0;
    double areaAfter = 0.0;
    double unchangedFraction = 0.0; ///< Share of the input shells' faces the union kept bit-identical
    double elapsedMs = 0.0;
};

/// Boolean union of the given shells (faces of each) with Manifold's `BatchBoolean`. Every shell
/// must be closed and consistently oriented (Manifold's "even manifold"); the first one that is not
/// fails the merge. Faces of the other shells are kept as they are.
[[nodiscard]] ShellMergeResult mergeShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shells);

} // namespace ogun
