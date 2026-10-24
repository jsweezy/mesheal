#pragma once

#include <numbers>

namespace ogun {

// Fundamental mathematical constants
inline constexpr double kPi = std::numbers::pi;
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

} // namespace ogun
