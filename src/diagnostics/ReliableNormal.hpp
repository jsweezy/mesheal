#pragma once

#include "core/Constants.hpp"
#include "core/Settings.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace mesheal {

/// True when the triangle (v0, v1, v2) has a normal that is more than rounding noise: every
/// coordinate is finite, its smallest height h_min = 2A / l_max exceeds `kReliableNormalRatio`
/// x l_max (aspect below 1e10), and h_min exceeds `kNormalFloorCoordinateFactor` x the largest
/// absolute coordinate of its vertices (the spacing of doubles there). The one test that decides a
/// degenerate face for Mesh Check, the Thin faces Zero-area bucket and the sliver healer.
[[nodiscard]] inline bool hasReliableNormal(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2) noexcept {
    if (!v0.allFinite() || !v1.allFinite() || !v2.allFinite()) {
        return false;
    }
    const double longestSq = std::max({(v1 - v0).squaredNorm(), (v2 - v1).squaredNorm(), (v0 - v2).squaredNorm()});
    if (!(longestSq > 0.0) || !std::isfinite(longestSq)) {
        return false;
    }
    const double longest = std::sqrt(longestSq);
    const double twiceArea = (v1 - v0).cross(v2 - v0).norm();
    const double minHeight = twiceArea / longest;
    const double largestCoordinate = std::max({v0.cwiseAbs().maxCoeff(), v1.cwiseAbs().maxCoeff(), v2.cwiseAbs().maxCoeff()});
    return std::isfinite(minHeight) && minHeight > kReliableNormalRatio * longest &&
           minHeight > settings::kNormalFloorCoordinateFactor * largestCoordinate;
}

} // namespace mesheal
