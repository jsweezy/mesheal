#pragma once

#include "core/Settings.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <vector>

namespace mesheal {

/// User-configurable thresholds for flagging thin triangles. The aspect ratio is the main,
/// scale-free measure; the height and area floors are optional (0 = off), in the active `mesh.fileUnit`.
struct ThinFaceThresholds {
    /// Advanced floor: smallest allowed height h_min = 2A / l_max (in `fileUnit`; 0 = off).
    double minAltitude = settings::kDefaultThinFaceMinAltitude;

    /// Advanced floor: smallest allowed area A (in `fileUnit^2`; 0 = off).
    double minArea = settings::kDefaultThinFaceMinArea;

    /// Faces stretched more than this aspect ratio l_max / h_min (dimensionless) are thin.
    double maxAspectRatio = settings::kDefaultThinFaceMaxAspectRatio;

    /// Converts length and area thresholds from `fromUnit` into `toUnit`.
    /// Aspect ratio is dimensionless and remains unchanged.
    [[nodiscard]] constexpr ThinFaceThresholds convertedToUnit(
        Unit fromUnit,
        Unit toUnit) const noexcept {
        const double s = unitConversionFactor(fromUnit, toUnit);
        return ThinFaceThresholds{
            minAltitude * s,
            minArea * (s * s),
            maxAspectRatio
        };
    }

    /// Returns default thresholds expressed in the specified unit (floors referenced to millimetres).
    [[nodiscard]] static constexpr ThinFaceThresholds defaultsForUnit(
        Unit unit = Unit::Millimeter) noexcept {
        return ThinFaceThresholds{}.convertedToUnit(Unit::Millimeter, unit);
    }
};

/// Geometric metrics and threshold-violation flags for a single triangle facet.
struct ThinFaceDefect {
    Eigen::Index faceIndex = 0;

    double area = 0.0;
    double minAltitude = 0.0;
    double shortestEdge = 0.0;
    double longestEdge = 0.0;
    double aspectRatio = std::numeric_limits<double>::infinity();
    double maxAngleDeg = 0.0; ///< Interior angle opposite the longest edge (180 for collinear faces).

    bool belowMinArea = false;
    bool belowMinAltitude = false;
    bool aboveMaxAspectRatio = false;
    bool hasDuplicateVertexIndices = false;
    bool reliableNormal = false; ///< `hasReliableNormal` of the three vertices (ReliableNormal.hpp)

    /// Degenerate: a repeated vertex index, or no reliable normal (the Degenerate faces row agrees).
    [[nodiscard]] bool isZeroArea() const noexcept {
        return hasDuplicateVertexIndices || !reliableNormal;
    }

    /// Thin: degenerate, stretched over the aspect limit, or below an advanced floor that is set.
    [[nodiscard]] bool isDefective() const noexcept {
        return isZeroArea() || belowMinArea || belowMinAltitude || aboveMaxAspectRatio;
    }
};

/// Geometric shape bucket of a thin face, used by the Tool panel's sub-type filter.
enum class ThinFaceSubtype {
    ZeroArea, ///< No reliable normal (ReliableNormal.hpp) or a repeated vertex index.
    Needle,   ///< One edge much shorter than the longest (l_min / l_max below the needle ratio).
    Cap,      ///< One interior angle close to 180 degrees.
    Small     ///< Flagged but neither: below a floor, or stretched just over the limit (aspect 100 to ~121).
};

inline constexpr int kThinFaceSubtypeCount = 4;

[[nodiscard]] const char* thinFaceSubtypeLabel(ThinFaceSubtype subtype) noexcept;

/// Buckets `defect` by shape: zero-area first, then needle (edge ratio), then cap (max angle), else small.
[[nodiscard]] ThinFaceSubtype classifyThinDefect(
    const ThinFaceDefect& defect,
    double needleEdgeRatio = settings::kDefaultNeedleEdgeRatio,
    double capAngleDeg = settings::kDefaultCapAngleDeg) noexcept;

/// Result of running `ThinFaceChecker::check()` across an entire `mesheal::Mesh`.
struct ThinFaceReport {
    Unit unit = Unit::Millimeter;
    ThinFaceThresholds thresholds{};
    Eigen::Index totalFacesChecked = 0;

    /// Detailed metrics for each offending face, ordered by ascending `faceIndex`.
    std::vector<ThinFaceDefect> defects;

    /// Offending 0-based face indices (mirrors `defects[i].faceIndex` for quick selection/highlighting).
    std::vector<Eigen::Index> faceIndices;

    [[nodiscard]] bool empty() const noexcept {
        return defects.empty();
    }

    [[nodiscard]] bool hasDefects() const noexcept {
        return !defects.empty();
    }

    [[nodiscard]] std::size_t numDefects() const noexcept {
        return defects.size();
    }

    /// Looks up the defect record for `faceIdx`, or returns `nullptr` if `faceIdx` is not defective.
    [[nodiscard]] const ThinFaceDefect* findDefectForFace(Eigen::Index faceIdx) const noexcept;
};

/// Diagnostic checker that detects degenerate faces and faces stretched over the aspect limit.
class ThinFaceChecker {
public:
    /// Computes geometric metrics and threshold flags for a single face `faceIndex` of `mesh`.
    [[nodiscard]] static ThinFaceDefect evaluateFace(
        const Mesh& mesh,
        Eigen::Index faceIndex,
        const ThinFaceThresholds& thresholds = {});

    /// Computes geometric metrics and threshold flags for the triangle (v0, v1, v2).
    [[nodiscard]] static ThinFaceDefect evaluateTriangle(
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1,
        const Eigen::Vector3d& v2,
        const ThinFaceThresholds& thresholds = {},
        bool hasDuplicateVertexIndices = false);

    /// Scans all faces of `mesh` and returns a `ThinFaceReport` containing all offending faces.
    [[nodiscard]] static ThinFaceReport check(
        const Mesh& mesh,
        const ThinFaceThresholds& thresholds = {});
};

} // namespace mesheal
