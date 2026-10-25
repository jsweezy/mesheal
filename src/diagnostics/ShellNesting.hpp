#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <set>
#include <span>
#include <utility>
#include <vector>

namespace ogun {

/// True if the ray `origin + t * dir` (t > 0) hits triangle (a, b, c) (Moller-Trumbore).
[[nodiscard]] bool rayHitsTriangle(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& dir,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c) noexcept;

/// For each shell, the shell it lies inside without crossing it (-1: none). A shell is inside k when
/// its box lies inside k's box, the pair (low, high) is not in `crossingPairs`, and one of its
/// vertices is inside k (odd ray-hit count). The smallest container wins, so a shell inside a cavity
/// reports the cavity.
[[nodiscard]] std::vector<int> findNestedShells(
    const Mesh& mesh,
    std::span<const std::vector<Eigen::Index>> shellFaces,
    const std::set<std::pair<int, int>>& crossingPairs);

} // namespace ogun
