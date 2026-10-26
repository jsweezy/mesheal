#include "repair/RepairMetrics.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ThinFaceChecker.hpp"

#include <Eigen/Geometry>

#include <igl/point_mesh_squared_distance.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

namespace ogun {

namespace {

/// Vertices plus face centroids of `mesh`: a cheap surface sample for deviation.
Eigen::MatrixXd surfaceSamples(const Mesh& mesh) {
    Eigen::MatrixXd samples(mesh.numVertices() + mesh.numFaces(), 3);
    samples.topRows(mesh.numVertices()) = mesh.V;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        samples.row(mesh.numVertices() + f) =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))) / 3.0;
    }
    return samples;
}

double maxDistance(const Eigen::MatrixXd& points, const Mesh& reference) {
    if (points.rows() == 0) {
        return 0.0;
    }
    if (reference.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::VectorXd sqrD;
    Eigen::VectorXi closestFace;
    Eigen::MatrixXd closest;
    igl::point_mesh_squared_distance(points, reference.V, reference.F, sqrD, closestFace, closest);
    return std::sqrt(sqrD.maxCoeff());
}

} // namespace

std::size_t FaceGeometryKeyHash::operator()(const FaceGeometryKey& key) const noexcept {
    std::size_t h = 0;
    for (const double x : key) {
        h ^= std::hash<double>{}(x) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return h;
}

FaceGeometryKey faceGeometryKey(const Mesh& mesh, Eigen::Index f) {
    std::array<std::array<double, 3>, 3> corners{};
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
            corners[static_cast<std::size_t>(c)][static_cast<std::size_t>(k)] = mesh.V(mesh.F(f, c), k);
        }
    }
    const auto first = static_cast<std::size_t>(std::min_element(corners.begin(), corners.end()) - corners.begin());
    FaceGeometryKey key{};
    for (std::size_t c = 0; c < 3; ++c) {
        const auto& p = corners[(first + c) % 3];
        std::copy(p.begin(), p.end(), key.begin() + static_cast<std::ptrdiff_t>(3 * c));
    }
    return key;
}

double RepairMetrics::volumeChangePercent() const noexcept {
    return volumeBefore != 0.0 ? 100.0 * (volumeAfter - volumeBefore) / std::abs(volumeBefore) : 0.0;
}

double RepairMetrics::areaChangePercent() const noexcept {
    return areaBefore != 0.0 ? 100.0 * (areaAfter - areaBefore) / areaBefore : 0.0;
}

FaceDiff faceDiff(const Mesh& before, const Mesh& after) {
    std::unordered_map<FaceGeometryKey, int, FaceGeometryKeyHash> count;
    count.reserve(static_cast<std::size_t>(before.numFaces()));
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        ++count[faceGeometryKey(before, f)];
    }
    FaceDiff diff;
    for (Eigen::Index f = 0; f < after.numFaces(); ++f) {
        const auto it = count.find(faceGeometryKey(after, f));
        if (it != count.end() && it->second > 0) {
            --it->second;
        } else {
            diff.added.push_back(f);
        }
    }
    // Whatever is left unmatched in `before` was removed
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        const auto it = count.find(faceGeometryKey(before, f));
        if (it->second > 0) {
            --it->second;
            diff.removed.push_back(f);
        }
    }
    return diff;
}

double signedVolume(const Mesh& mesh) noexcept {
    double sum6 = 0.0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
        const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1));
        const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2));
        sum6 += a.cross(b).dot(c);
    }
    return sum6 / 6.0;
}

double surfaceArea(const Mesh& mesh) noexcept {
    double sum = 0.0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
        const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1));
        const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2));
        sum += 0.5 * (b - a).cross(c - a).norm();
    }
    return sum;
}

Eigen::VectorXd vertexDeviation(const Mesh& mesh, const Mesh& reference) {
    if (mesh.numVertices() == 0 || reference.empty()) {
        return Eigen::VectorXd::Zero(mesh.numVertices());
    }
    Eigen::VectorXd sqrD;
    Eigen::VectorXi closestFace;
    Eigen::MatrixXd closest;
    igl::point_mesh_squared_distance(mesh.V, reference.V, reference.F, sqrD, closestFace, closest);
    return sqrD.cwiseSqrt();
}

RepairMetrics computeRepairMetrics(
    const Mesh& before,
    const Mesh& after,
    const CheckParams& params,
    std::stop_token stopToken) {
    RepairMetrics m;
    m.facesBefore = before.numFaces();
    m.facesAfter = after.numFaces();
    m.volumeBefore = signedVolume(before);
    m.volumeAfter = signedVolume(after);
    m.areaBefore = surfaceArea(before);
    m.areaAfter = surfaceArea(after);
    if (before.numFaces() > 0) {
        const FaceDiff diff = faceDiff(before, after);
        m.unchangedFraction = 1.0 - static_cast<double>(diff.removed.size()) / static_cast<double>(before.numFaces());
    }
    if (stopToken.stop_requested()) {
        return m;
    }
    m.maxDeviation = maxDistance(surfaceSamples(after), before);
    m.maxLost = maxDistance(surfaceSamples(before), after);
    if (stopToken.stop_requested()) {
        return m;
    }
    m.shellsBefore = checkShells(before, stopToken).infoCount;
    m.shellsAfter = checkShells(after, stopToken).infoCount;
    m.openEdgesBefore = checkOpenEdges(before, stopToken).size();
    m.openEdgesAfter = checkOpenEdges(after, stopToken).size();
    m.nonManifoldEdgesAfter = checkNonManifoldEdges(after, stopToken).size();
    m.thinFacesBefore = ThinFaceChecker::check(before, params.thinThresholds).numDefects();
    m.thinFacesAfter = ThinFaceChecker::check(after, params.thinThresholds).numDefects();
    m.intersectingFacesBefore = checkSelfIntersections(before, params, stopToken).size();
    m.intersectingFacesAfter = checkSelfIntersections(after, params, stopToken).size();
    return m;
}

} // namespace ogun
