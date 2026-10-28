#pragma once

#include "core/Settings.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace ogun {

/// Configuration for local coordinate-relative vertex welding with an atomic floor.
///
/// Two coordinates p_i and q_i along axis i in {x, y, z} are considered matching if:
///   |p_i - q_i| <= max(absFloor, relTol * max(|p_i|, |q_i|))
/// `absFloor` is in file coordinates (unitless: changing the file unit never changes it).
struct ToleranceConfig {
    double absFloor = settings::kDefaultWeldAbsFloor;
    double relTol = settings::kDefaultWeldRelTol;

    /// Computes the local coordinate-relative tolerance for a single coordinate pair (p, q).
    [[nodiscard]] double coordinateTolerance(double p, double q) const noexcept {
        const double epsAbs = absFloor;
        const double maxMag = std::max(std::abs(p), std::abs(q));
        return std::max(epsAbs, relTol * maxMag);
    }

    /// Computes the per-axis tolerance vector for two 3D points (p, q).
    [[nodiscard]] Eigen::Vector3d vertexWeldTolerance(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& q) const noexcept {
        return Eigen::Vector3d(
            coordinateTolerance(p.x(), q.x()),
            coordinateTolerance(p.y(), q.y()),
            coordinateTolerance(p.z(), q.z()));
    }

    /// Returns true if all three coordinates of `p` and `q` match within local tolerance.
    [[nodiscard]] bool verticesMatchWithinTolerance(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& q) const noexcept {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(p[axis] - q[axis]) > coordinateTolerance(p[axis], q[axis])) {
                return false;
            }
        }
        return true;
    }
};

} // namespace ogun
