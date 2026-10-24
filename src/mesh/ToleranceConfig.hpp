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
///   |p_i - q_i| <= max(eps_abs(unit), relTol * max(|p_i|, |q_i|))
/// where eps_abs(unit) converts `absFloorCm` (default 1e-9 cm, ~1 hydrogen atom) into `unit`.
struct ToleranceConfig {
    double absFloorCm = settings::kDefaultWeldAbsFloorCm;
    double relTol = settings::kDefaultWeldRelTol;

    /// Returns the absolute floor tolerance expressed in the given length unit.
    [[nodiscard]] constexpr double absFloorInUnit(Unit unit) const noexcept {
        return centimetersToUnit(absFloorCm, unit);
    }

    /// Computes the local coordinate-relative tolerance for a single coordinate pair (p, q).
    [[nodiscard]] double coordinateTolerance(double p, double q, Unit unit) const noexcept {
        const double epsAbs = absFloorInUnit(unit);
        const double maxMag = std::max(std::abs(p), std::abs(q));
        return std::max(epsAbs, relTol * maxMag);
    }

    /// Computes the per-axis tolerance vector for two 3D points (p, q).
    [[nodiscard]] Eigen::Vector3d vertexWeldTolerance(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& q,
        Unit unit) const noexcept {
        return Eigen::Vector3d(
            coordinateTolerance(p.x(), q.x(), unit),
            coordinateTolerance(p.y(), q.y(), unit),
            coordinateTolerance(p.z(), q.z(), unit));
    }

    /// Returns true if all three coordinates of `p` and `q` match within local tolerance.
    [[nodiscard]] bool verticesMatchWithinTolerance(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& q,
        Unit unit) const noexcept {
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(p[axis] - q[axis]) > coordinateTolerance(p[axis], q[axis], unit)) {
                return false;
            }
        }
        return true;
    }
};

} // namespace ogun
