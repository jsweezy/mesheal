#pragma once

#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>
#include <manifold/manifold.h>

#include <span>

namespace ogun {

/// Axis-aligned 3D bounding box in the mesh's active file units.
struct BoundingBox {
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();

    [[nodiscard]] Eigen::Vector3d extents() const noexcept {
        return max - min;
    }

    [[nodiscard]] Eigen::Vector3d center() const noexcept {
        return 0.5 * (min + max);
    }

    [[nodiscard]] double diagonal() const noexcept {
        return extents().norm();
    }
};

/// Indexed Face-Vertex triangle mesh representation.
/// - `V`: |V| x 3 vertex positions in `fileUnit`
/// - `F`: |F| x 3 0-based triangle vertex indices (CCW outward)
/// - `N`: |F| x 3 unit face normals
struct Mesh {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd N;
    Unit fileUnit = Unit::Millimeter;

    [[nodiscard]] Eigen::Index numVertices() const noexcept {
        return V.rows();
    }

    [[nodiscard]] Eigen::Index numFaces() const noexcept {
        return F.rows();
    }

    [[nodiscard]] bool empty() const noexcept {
        return V.rows() == 0 || F.rows() == 0;
    }

    /// Updates `fileUnit` without altering vertex coordinates `V`.
    void setFileUnit(Unit unit) noexcept {
        fileUnit = unit;
    }

    /// Computes the axis-aligned bounding box of `V` (returns zero box if `V` is empty).
    [[nodiscard]] BoundingBox boundingBox() const;

    /// Recomputes per-face unit normals `N` from `V` and `F` (zero vector for degenerate faces).
    void recomputeNormals();

    /// Converts this mesh (`V`, `F`) to a single-precision `manifold::MeshGL`.
    [[nodiscard]] manifold::MeshGL toMeshGL() const;

    /// Converts this mesh (`V`, `F`) to a double-precision `manifold::MeshGL64`.
    [[nodiscard]] manifold::MeshGL64 toMeshGL64() const;

    /// Constructs an `ogun::Mesh` from a single-precision `manifold::MeshGL`.
    [[nodiscard]] static Mesh fromMeshGL(
        const manifold::MeshGL& meshGl,
        Unit unit = Unit::Millimeter);

    /// Constructs an `ogun::Mesh` from a double-precision `manifold::MeshGL64`.
    [[nodiscard]] static Mesh fromMeshGL64(
        const manifold::MeshGL64& meshGl64,
        Unit unit = Unit::Millimeter);
};

/// Copies the listed faces (in order) into a new mesh and keeps only the vertices they use.
[[nodiscard]] Mesh extractFaces(const Mesh& mesh, std::span<const Eigen::Index> faces);

/// Free-function convenience wrapper around `Mesh::toMeshGL()`.
[[nodiscard]] manifold::MeshGL toMeshGL(const Mesh& mesh);

/// Free-function convenience wrapper around `Mesh::fromMeshGL()`.
[[nodiscard]] Mesh fromMeshGL(
    const manifold::MeshGL& meshGl,
    Unit unit = Unit::Millimeter);

} // namespace ogun
