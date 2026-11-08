#pragma once

#include "mesh/Mesh.hpp"

#include <Eigen/Core>
#include <igl/AABB.h>

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace mesheal {

/// A slice plane cutting the view: points with `(p - origin) . normal < 0` are cut away and not drawn
/// (Polyscope's scene slice planes).
struct ClipPlane {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX(); ///< Unit, pointing to the kept side
};

/// The camera as the viewport sees it: enough to project points to screen coordinates (in the same
/// units as the mouse position, origin top-left) and to cast rays from the eye.
struct ScreenView {
    Eigen::Matrix4d viewProjection = Eigen::Matrix4d::Identity(); ///< World -> clip space
    Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d viewDirection = -Eigen::Vector3d::UnitZ();   ///< Unit; used for orthographic rays
    bool orthographic = false;
    double width = 0.0;
    double height = 0.0;
    /// The enabled slice planes: what they cut away is neither selected nor hides what is behind it.
    std::vector<ClipPlane> clipPlanes;

    /// Screen position of `p`, or nothing when it is behind the camera.
    [[nodiscard]] std::optional<Eigen::Vector2d> project(const Eigen::Vector3d& p) const;
    /// True when a slice plane cuts `p` away.
    [[nodiscard]] bool clipped(const Eigen::Vector3d& p) const noexcept {
        return clippedBy(clipPlanes, p);
    }
    [[nodiscard]] static bool clippedBy(std::span<const ClipPlane> planes, const Eigen::Vector3d& p) noexcept;
};

/// A face is cut away when its centre is, as the selection tools test faces by their centre: a face
/// the plane crosses counts as shown while its centre is on the kept side.
[[nodiscard]] bool faceClipped(const Mesh& mesh, Eigen::Index face, std::span<const ClipPlane> planes) noexcept;

/// Answers "is this face the first thing the eye sees through its centre?" with a ray cast against an
/// AABB tree of the mesh (Blender's selection without X-ray). The tree holds a copy of the mesh scaled
/// to a fixed size, because libigl's ray-triangle test misses tiny triangles (its epsilon is absolute).
/// The view's slice planes are applied per ray, so moving a plane needs no rebuild: geometry they cut
/// away is never hit and so never hides a face behind it.
class FaceVisibility {
public:
    /// Only the `shown` faces (empty = all) are in the tree: hidden shells neither block nor get hit.
    void build(const Mesh& mesh, std::span<const std::uint8_t> shown = {});
    [[nodiscard]] bool built() const noexcept {
        return built_;
    }
    /// The face is shown (not cut away by a slice plane) and the first thing the eye sees through it.
    [[nodiscard]] bool visible(const Mesh& mesh, Eigen::Index face, const ScreenView& view) const;
    /// The first face under the screen point `screen`, if any.
    [[nodiscard]] std::optional<Eigen::Index> faceAt(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& screen) const;

private:
    [[nodiscard]] Eigen::Vector3d toWork(const Eigen::Vector3d& p) const {
        return scale_ * (p - origin_);
    }
    [[nodiscard]] Eigen::Vector3d fromWork(const Eigen::Vector3d& p) const {
        return p / scale_ + origin_;
    }
    /// The mesh face the ray (working space) hits first where no slice plane cuts it away, and the ray
    /// parameter there.
    [[nodiscard]] std::optional<Eigen::Index> firstHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                                                       std::span<const ClipPlane> planes, double* t) const;

    igl::AABB<Eigen::MatrixXd, 3> tree_;
    Eigen::MatrixXd V_; ///< The mesh vertices at the working size
    Eigen::MatrixXi F_; ///< The shown faces
    std::vector<Eigen::Index> treeFace_; ///< Mesh face of each row of F_
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    double scale_ = 1.0;
    bool built_ = false;
};

/// `faces` without those whose centre is behind a triangle of `occluder` (e.g. an opaque repair-preview
/// patch, which is drawn but is not part of `mesh`). Faces in front of or level with it stay.
[[nodiscard]] std::vector<Eigen::Index> withoutFacesBehind(const Mesh& mesh, const ScreenView& view,
                                                           std::vector<Eigen::Index> faces, const Mesh& occluder);

/// The `shown` faces (empty = all) whose centre projects into the rectangle with corners `a` and `b`
/// (any order). Without `visibility` (select through) faces behind others count too. Faces the view's
/// slice planes cut away never count.
[[nodiscard]] std::vector<Eigen::Index> facesInRectangle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                                         const Eigen::Vector2d& b, const FaceVisibility* visibility,
                                                         std::span<const std::uint8_t> shown = {});

/// The `shown` faces whose centre projects within `radius` of the segment `a`-`b` (a brush stroke;
/// a circle when a = b).
[[nodiscard]] std::vector<Eigen::Index> facesInStroke(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                                      const Eigen::Vector2d& b, double radius, const FaceVisibility* visibility,
                                                      std::span<const std::uint8_t> shown = {});

} // namespace mesheal
