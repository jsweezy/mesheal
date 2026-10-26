#include "app/FaceSelection.hpp"

#include "core/Settings.hpp"

#include <Eigen/Geometry>
#include <igl/Hit.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace ogun {

namespace {

Eigen::Vector3d centroid(const Mesh& mesh, Eigen::Index f) {
    return (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
}

std::vector<Eigen::Index> facesWhere(const Mesh& mesh, const ScreenView& view, const FaceVisibility* visibility,
                                     const std::function<bool(const Eigen::Vector2d&)>& inside) {
    std::vector<Eigen::Index> faces;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const std::optional<Eigen::Vector2d> p = view.project(centroid(mesh, f));
        if (p && inside(*p) && (visibility == nullptr || visibility->visible(mesh, f, view))) {
            faces.push_back(f);
        }
    }
    return faces;
}

} // namespace

std::optional<Eigen::Vector2d> ScreenView::project(const Eigen::Vector3d& p) const {
    const Eigen::Vector4d clip = viewProjection * p.homogeneous();
    if (clip.w() <= 1e-12) {
        return std::nullopt;
    }
    const double x = clip.x() / clip.w();
    const double y = clip.y() / clip.w();
    return Eigen::Vector2d((x + 1.0) * 0.5 * width, (1.0 - y) * 0.5 * height);
}

void FaceVisibility::build(const Mesh& mesh) {
    tree_ = igl::AABB<Eigen::MatrixXd, 3>();
    built_ = !mesh.empty();
    if (built_) {
        const BoundingBox box = mesh.boundingBox();
        origin_ = box.min;
        scale_ = settings::kFaceVisibilityWorkingSize / std::max(box.diagonal(), 1e-300);
        V_ = (mesh.V.rowwise() - origin_.transpose()) * scale_;
        tree_.init(V_, mesh.F);
    }
}

bool FaceVisibility::visible(const Mesh& mesh, Eigen::Index face, const ScreenView& view) const {
    if (!built_) {
        return true;
    }
    const Eigen::Vector3d a = V_.row(mesh.F(face, 0)).transpose();
    const Eigen::Vector3d b = V_.row(mesh.F(face, 1)).transpose();
    const Eigen::Vector3d c = V_.row(mesh.F(face, 2)).transpose();
    const Eigen::Vector3d normal = (b - a).cross(c - a).normalized();
    const Eigen::Vector3d eye = toWork(view.cameraPosition);
    const double reach = 2.0 * settings::kFaceVisibilityWorkingSize + (a - eye).norm();

    // Rays from the eye (perspective) or from far in front (orthographic) to three points inside the
    // face, t = 1 there. A ray along a shared edge of the faces in front can slip between them, so the
    // face counts as seen when at least two of the three rays reach it.
    const std::array<Eigen::Vector3d, 3> targets = {(a + b + c) / 3.0, 0.6 * a + 0.2 * b + 0.2 * c, 0.2 * a + 0.6 * b + 0.2 * c};
    int seen = 0;
    for (const Eigen::Vector3d& target : targets) {
        const Eigen::Vector3d origin = view.orthographic ? Eigen::Vector3d(target - reach * view.viewDirection) : eye;
        const Eigen::Vector3d dir = target - origin;
        if (std::abs(normal.dot(dir.normalized())) < 1e-3) {
            return false; // edge-on: the ray runs in the face's plane
        }
        igl::Hit hit{};
        if (tree_.intersect_ray(V_, mesh.F, origin.transpose(), dir.transpose(), hit) &&
            (hit.id == static_cast<int>(face) || hit.t > 1.0 - 1e-6)) {
            ++seen;
        }
    }
    return seen >= 2;
}

std::optional<Eigen::Index> FaceVisibility::faceAt(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& screen) const {
    if (!built_ || view.width <= 0.0 || view.height <= 0.0) {
        return std::nullopt;
    }
    // The ray through the near plane and halfway in depth (the far plane may be at infinity), cast from
    // the eye or, orthographic, from well in front of the mesh: the near plane can cut into it
    const Eigen::Matrix4d inverse = view.viewProjection.inverse();
    const double x = 2.0 * screen.x() / view.width - 1.0;
    const double y = 1.0 - 2.0 * screen.y() / view.height;
    const auto unproject = [&](double z) {
        const Eigen::Vector4d p = inverse * Eigen::Vector4d(x, y, z, 1.0);
        return toWork(p.head<3>() / p.w());
    };
    const Eigen::Vector3d nearPoint = unproject(-1.0);
    const Eigen::Vector3d dir = (unproject(0.0) - nearPoint).normalized();
    const Eigen::Vector3d origin = view.orthographic
        ? Eigen::Vector3d(nearPoint - (nearPoint.norm() + 2.0 * settings::kFaceVisibilityWorkingSize) * dir)
        : toWork(view.cameraPosition);
    igl::Hit hit{};
    if (!tree_.intersect_ray(V_, mesh.F, origin.transpose(), dir.transpose(), hit)) {
        return std::nullopt;
    }
    return static_cast<Eigen::Index>(hit.id);
}

std::vector<Eigen::Index> facesInRectangle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                           const Eigen::Vector2d& b, const FaceVisibility* visibility) {
    const Eigen::Vector2d lo = a.cwiseMin(b);
    const Eigen::Vector2d hi = a.cwiseMax(b);
    return facesWhere(mesh, view, visibility, [&](const Eigen::Vector2d& p) {
        return p.x() >= lo.x() && p.x() <= hi.x() && p.y() >= lo.y() && p.y() <= hi.y();
    });
}

std::vector<Eigen::Index> facesInCircle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& center, double radius,
                                        const FaceVisibility* visibility) {
    const double r2 = radius * radius;
    return facesWhere(mesh, view, visibility, [&](const Eigen::Vector2d& p) { return (p - center).squaredNorm() <= r2; });
}

} // namespace ogun
