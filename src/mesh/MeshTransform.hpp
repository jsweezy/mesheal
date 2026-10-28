#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <optional>
#include <span>

namespace ogun {

/// Rigid transform of Mesh > Transform > Translate / Rotate: p' = R * (p + t), with
/// R = Rz * Ry * Rx (rotate about the world X axis first, then world Y, then world Z, all about the
/// world origin). The translation is in file units, the angles in degrees.
struct MeshTransform {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotationDeg = Eigen::Vector3d::Zero();

    /// No translation and every angle a multiple of 360 degrees.
    [[nodiscard]] bool isIdentity() const noexcept;

    [[nodiscard]] bool operator==(const MeshTransform& other) const noexcept {
        return translation == other.translation && rotationDeg == other.rotationDeg;
    }
};

/// R = Rz * Ry * Rx for angles in degrees. Multiples of 90 degrees give exact 0 / +-1 entries.
[[nodiscard]] Eigen::Matrix3d rotationMatrixXYZ(const Eigen::Vector3d& rotationDeg) noexcept;

/// p' = R * (p + t).
[[nodiscard]] Eigen::Vector3d transformPoint(const MeshTransform& transform, const Eigen::Vector3d& point) noexcept;

/// Moves every vertex by `transform` and recomputes the normals. Faces (and their winding) are unchanged.
void transformMesh(Mesh& mesh, const MeshTransform& transform);

/// Convenience overload: translation in file units, then rotation about world X, Y, Z (degrees).
void transformMesh(Mesh& mesh, const Eigen::Vector3d& translation, const Eigen::Vector3d& rotationDeg);

/// Area-weighted centroid of the listed triangles (the triangle centroid for one face). Falls back to
/// the plain average of the triangle centroids when their total area is 0. Out-of-range faces are
/// skipped; returns nullopt when none is left.
[[nodiscard]] std::optional<Eigen::Vector3d> facesCentroid(const Mesh& mesh, std::span<const Eigen::Index> faces);

} // namespace ogun
