#include "mesh/MeshTransform.hpp"

#include "core/Constants.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <utility>

namespace ogun {

namespace {

/// sin / cos of an angle in degrees, exact for multiples of 90 so 90-degree turns keep 0 and 1 exact.
[[nodiscard]] std::pair<double, double> sinCosDeg(double degrees) noexcept {
    const double reduced = std::fmod(degrees, 360.0);
    const double quarter = reduced / 90.0;
    if (quarter == std::round(quarter)) {
        switch ((static_cast<int>(std::round(quarter)) % 4 + 4) % 4) {
        case 0:
            return {0.0, 1.0};
        case 1:
            return {1.0, 0.0};
        case 2:
            return {0.0, -1.0};
        default:
            return {-1.0, 0.0};
        }
    }
    const double radians = reduced * kDegToRad;
    return {std::sin(radians), std::cos(radians)};
}

} // namespace

bool MeshTransform::isIdentity() const noexcept {
    return translation.isZero(0.0) && std::fmod(rotationDeg.x(), 360.0) == 0.0 &&
           std::fmod(rotationDeg.y(), 360.0) == 0.0 && std::fmod(rotationDeg.z(), 360.0) == 0.0;
}

Eigen::Matrix3d rotationMatrixXYZ(const Eigen::Vector3d& rotationDeg) noexcept {
    const auto [sx, cx] = sinCosDeg(rotationDeg.x());
    const auto [sy, cy] = sinCosDeg(rotationDeg.y());
    const auto [sz, cz] = sinCosDeg(rotationDeg.z());
    Eigen::Matrix3d rx;
    rx << 1.0, 0.0, 0.0,
          0.0, cx, -sx,
          0.0, sx, cx;
    Eigen::Matrix3d ry;
    ry << cy, 0.0, sy,
          0.0, 1.0, 0.0,
          -sy, 0.0, cy;
    Eigen::Matrix3d rz;
    rz << cz, -sz, 0.0,
          sz, cz, 0.0,
          0.0, 0.0, 1.0;
    return rz * ry * rx;
}

Eigen::Vector3d transformPoint(const MeshTransform& transform, const Eigen::Vector3d& point) noexcept {
    return rotationMatrixXYZ(transform.rotationDeg) * (point + transform.translation);
}

void transformMesh(Mesh& mesh, const MeshTransform& transform) {
    const Eigen::Matrix3d r = rotationMatrixXYZ(transform.rotationDeg);
    // Rows are points: V' = (V + 1 t^T) R^T
    mesh.V.rowwise() += transform.translation.transpose();
    mesh.V = (mesh.V * r.transpose()).eval();
    mesh.recomputeNormals();
}

void transformMesh(Mesh& mesh, const Eigen::Vector3d& translation, const Eigen::Vector3d& rotationDeg) {
    transformMesh(mesh, MeshTransform{translation, rotationDeg});
}

std::optional<Eigen::Vector3d> facesCentroid(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
    Eigen::Vector3d plain = Eigen::Vector3d::Zero();
    double totalArea = 0.0;
    std::size_t count = 0;
    for (const Eigen::Index f : faces) {
        if (f < 0 || f >= mesh.numFaces()) {
            continue;
        }
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0)).transpose();
        const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1)).transpose();
        const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2)).transpose();
        const Eigen::Vector3d centroid = (a + b + c) / 3.0;
        const double area = 0.5 * (b - a).cross(c - a).norm();
        weighted += area * centroid;
        plain += centroid;
        totalArea += area;
        ++count;
    }
    if (count == 0) {
        return std::nullopt;
    }
    return totalArea > 0.0 ? weighted / totalArea : plain / static_cast<double>(count);
}

} // namespace ogun
