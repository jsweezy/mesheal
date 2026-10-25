#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <atomic>
#include <stop_token>

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

} // namespace ogun
