#include "diagnostics/ThinFaceChecker.hpp"
#include "core/Constants.hpp"
#include "diagnostics/ReliableNormal.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ogun {

const char* thinFaceSubtypeLabel(ThinFaceSubtype subtype) noexcept {
    switch (subtype) {
    case ThinFaceSubtype::ZeroArea:
        return "Zero-area";
    case ThinFaceSubtype::Needle:
        return "Needle";
    case ThinFaceSubtype::Cap:
        return "Cap";
    case ThinFaceSubtype::Small:
        return "Small";
    }
    return "Unknown";
}

ThinFaceSubtype classifyThinDefect(
    const ThinFaceDefect& defect,
    double needleEdgeRatio,
    double capAngleDeg) noexcept {
    if (defect.isZeroArea()) {
        return ThinFaceSubtype::ZeroArea;
    }
    if (defect.longestEdge > 0.0 && defect.shortestEdge / defect.longestEdge < needleEdgeRatio) {
        return ThinFaceSubtype::Needle;
    }
    if (defect.maxAngleDeg > capAngleDeg) {
        return ThinFaceSubtype::Cap;
    }
    return ThinFaceSubtype::Small;
}

const ThinFaceDefect* ThinFaceReport::findDefectForFace(Eigen::Index faceIdx) const noexcept {
    const auto it = std::lower_bound(
        defects.begin(),
        defects.end(),
        faceIdx,
        [](const ThinFaceDefect& d, Eigen::Index targetIdx) {
            return d.faceIndex < targetIdx;
        });
    if (it != defects.end() && it->faceIndex == faceIdx) {
        return &(*it);
    }
    return nullptr;
}

ThinFaceDefect ThinFaceChecker::evaluateFace(
    const Mesh& mesh,
    Eigen::Index faceIndex,
    const ThinFaceThresholds& thresholds) {
    ThinFaceDefect defect;
    defect.faceIndex = faceIndex;

    if (mesh.empty() || faceIndex < 0 || faceIndex >= mesh.numFaces() || mesh.F.cols() < 3) {
        defect.hasDuplicateVertexIndices = true;
        defect.belowMinArea = true;
        defect.belowMinAltitude = true;
        defect.aboveMaxAspectRatio = true;
        return defect;
    }

    const int i0 = mesh.F(faceIndex, 0);
    const int i1 = mesh.F(faceIndex, 1);
    const int i2 = mesh.F(faceIndex, 2);

    const Eigen::Index numVerts = mesh.numVertices();
    if (i0 < 0 || i0 >= numVerts ||
        i1 < 0 || i1 >= numVerts ||
        i2 < 0 || i2 >= numVerts) {
        defect.hasDuplicateVertexIndices = true;
        defect.belowMinArea = true;
        defect.belowMinAltitude = true;
        defect.aboveMaxAspectRatio = true;
        return defect;
    }

    const bool duplicate = (i0 == i1) || (i1 == i2) || (i2 == i0);
    ThinFaceDefect result = evaluateTriangle(
        mesh.V.row(i0).transpose(),
        mesh.V.row(i1).transpose(),
        mesh.V.row(i2).transpose(),
        thresholds,
        duplicate);
    result.faceIndex = faceIndex;
    return result;
}

ThinFaceDefect ThinFaceChecker::evaluateTriangle(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    const ThinFaceThresholds& thresholds,
    bool hasDuplicateVertexIndices) {
    ThinFaceDefect defect;
    defect.hasDuplicateVertexIndices = hasDuplicateVertexIndices;
    defect.reliableNormal = !hasDuplicateVertexIndices && hasReliableNormal(v0, v1, v2);

    const double e0 = (v1 - v0).norm();
    const double e1 = (v2 - v1).norm();
    const double e2 = (v0 - v2).norm();

    defect.shortestEdge = std::min({e0, e1, e2});
    defect.longestEdge = std::max({e0, e1, e2});

    // Law of cosines for the angle opposite the longest edge
    const double a = defect.shortestEdge;
    const double b = e0 + e1 + e2 - defect.shortestEdge - defect.longestEdge;
    if (a > 0.0 && b > 0.0) {
        const double c = defect.longestEdge;
        const double cosAngle = std::clamp((a * a + b * b - c * c) / (2.0 * a * b), -1.0, 1.0);
        defect.maxAngleDeg = std::acos(cosAngle) * kRadToDeg;
    }

    if (!defect.hasDuplicateVertexIndices && defect.longestEdge > 0.0) {
        defect.area = 0.5 * (v1 - v0).cross(v2 - v0).norm();
        defect.minAltitude = (2.0 * defect.area) / defect.longestEdge;
        if (defect.minAltitude > 0.0) {
            defect.aspectRatio = defect.longestEdge / defect.minAltitude;
        } else {
            defect.aspectRatio = std::numeric_limits<double>::infinity();
        }
    } else {
        defect.area = 0.0;
        defect.minAltitude = 0.0;
        defect.aspectRatio = std::numeric_limits<double>::infinity();
    }

    // The floors are off at 0 (nothing is below 0); a zero area or height is always flagged
    defect.belowMinArea =
        defect.hasDuplicateVertexIndices ||
        !(defect.area > 0.0) ||
        (defect.area < thresholds.minArea);

    defect.belowMinAltitude =
        defect.hasDuplicateVertexIndices ||
        !(defect.minAltitude > 0.0) ||
        (defect.minAltitude < thresholds.minAltitude);

    defect.aboveMaxAspectRatio =
        defect.hasDuplicateVertexIndices ||
        !std::isfinite(defect.aspectRatio) ||
        (thresholds.maxAspectRatio > 0.0 && defect.aspectRatio > thresholds.maxAspectRatio);

    return defect;
}

ThinFaceReport ThinFaceChecker::check(
    const Mesh& mesh,
    const ThinFaceThresholds& thresholds) {
    ThinFaceReport report;
    report.unit = mesh.fileUnit;
    report.thresholds = thresholds;

    if (mesh.empty()) {
        return report;
    }

    report.totalFacesChecked = mesh.numFaces();

    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const ThinFaceDefect defect = evaluateFace(mesh, f, thresholds);
        if (defect.isDefective()) {
            report.defects.push_back(defect);
            report.faceIndices.push_back(f);
        }
    }

    return report;
}

} // namespace ogun
