#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>
#include <igl/AABB.h>

#include <cstdint>
#include <optional>
#include <span>
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
    /// Only the `shown` faces (empty = all) are in the tree: hidden shells neither block nor get hit.
    void build(const Mesh& mesh, std::span<const std::uint8_t> shown = {});
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
    /// The mesh face the ray (working space) hits first, and the ray parameter there.
    [[nodiscard]] std::optional<Eigen::Index> firstHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, double* t) const;

    igl::AABB<Eigen::MatrixXd, 3> tree_;
    Eigen::MatrixXd V_; ///< The mesh vertices at the working size
    Eigen::MatrixXi F_; ///< The shown faces
    std::vector<Eigen::Index> treeFace_; ///< Mesh face of each row of F_
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    double scale_ = 1.0;
    bool built_ = false;
};

/// The `shown` faces (empty = all) whose centre projects into the rectangle with corners `a` and `b`
/// (any order). Without `visibility` (select through) faces behind others count too.
[[nodiscard]] std::vector<Eigen::Index> facesInRectangle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                                         const Eigen::Vector2d& b, const FaceVisibility* visibility,
                                                         std::span<const std::uint8_t> shown = {});

/// The `shown` faces whose centre projects within `radius` of the segment `a`-`b` (a brush stroke;
/// a circle when a = b).
[[nodiscard]] std::vector<Eigen::Index> facesInStroke(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                                      const Eigen::Vector2d& b, double radius, const FaceVisibility* visibility,
                                                      std::span<const std::uint8_t> shown = {});

} // namespace ogun
