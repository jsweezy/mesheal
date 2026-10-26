#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>
#include <igl/AABB.h>

#include <optional>
#include <vector>

namespace ogun {

/// The camera as the viewport sees it: enough to project points to screen coordinates (in the same
/// units as the mouse position, origin top-left) and to cast rays from the eye.
struct ScreenView {
    Eigen::Matrix4d viewProjection = Eigen::Matrix4d::Identity(); ///< World -> clip space
    Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d viewDirection = -Eigen::Vector3d::UnitZ();   ///< Unit; used for orthographic rays
    bool orthographic = false;
    double width = 0.0;
    double height = 0.0;

    /// Screen position of `p`, or nothing when it is behind the camera.
    [[nodiscard]] std::optional<Eigen::Vector2d> project(const Eigen::Vector3d& p) const;
};

/// Answers "is this face the first thing the eye sees through its centre?" with a ray cast against an
/// AABB tree of the mesh (Blender's selection without X-ray). The tree holds a copy of the mesh scaled
/// to a fixed size, because libigl's ray-triangle test misses tiny triangles (its epsilon is absolute).
class FaceVisibility {
public:
    void build(const Mesh& mesh);
    [[nodiscard]] bool built() const noexcept {
        return built_;
    }
    [[nodiscard]] bool visible(const Mesh& mesh, Eigen::Index face, const ScreenView& view) const;
    /// The first face under the screen point `screen`, if any.
    [[nodiscard]] std::optional<Eigen::Index> faceAt(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& screen) const;

private:
    [[nodiscard]] Eigen::Vector3d toWork(const Eigen::Vector3d& p) const {
        return scale_ * (p - origin_);
    }

    igl::AABB<Eigen::MatrixXd, 3> tree_;
    Eigen::MatrixXd V_; ///< The mesh vertices at the working size
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    double scale_ = 1.0;
    bool built_ = false;
};

/// Faces whose centre projects into the rectangle with corners `a` and `b` (any order). Without
/// `visibility` (select through) hidden faces count too.
[[nodiscard]] std::vector<Eigen::Index> facesInRectangle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                                         const Eigen::Vector2d& b, const FaceVisibility* visibility);

/// Faces whose centre projects within `radius` of `center`.
[[nodiscard]] std::vector<Eigen::Index> facesInCircle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& center,
                                                      double radius, const FaceVisibility* visibility);

} // namespace ogun
