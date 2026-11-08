#pragma once

#include <numbers>

namespace mesheal {

// Fundamental mathematical constants
inline constexpr double kPi = std::numbers::pi;
inline constexpr double kSqrt3 = std::numbers::sqrt3;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;

// Incommensurable positive projection weights for 1D spatial hashing & vertex sorting (StlReader).
// Computed via constexpr math to avoid floating-point decimal truncation:
//   kAlphaX = 1.0
//   kAlphaY = 1 / phi = (sqrt(5) - 1) / 2
//   kAlphaZ = sqrt(2) - 1
inline constexpr double kAlphaX = 1.0;
inline constexpr double kAlphaY = 1.0 / std::numbers::phi;
inline constexpr double kAlphaZ = std::numbers::sqrt2 - 1.0;

// Numerical tolerances
inline constexpr double kReliableNormalRatio = 1.0e-10;

// Uniform search grids (mergeByDistance): a cell is at least this share of the largest |coordinate|, so
// a cell index stays below 1e9 however large the coordinates, and is grown by this relative margin so
// rounding in x / cell never puts two points one cell width apart into cells two apart.
inline constexpr double kGridMinCellCoordinateShare = 1.0e-9;
inline constexpr double kGridCellRoundingMargin = 1.0e-6;

} // namespace mesheal
