#include "app/FaceSelection.hpp"

#include "core/Settings.hpp"

#include <Eigen/Geometry>
#include <igl/Hit.h>
#include <igl/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

namespace ogun {

namespace {

Eigen::Vector3d centroid(const Mesh& mesh, Eigen::Index f) {
    return (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
}

std::vector<Eigen::Index> facesWhere(const Mesh& mesh, const ScreenView& view, const FaceVisibility* visibility,
                                     std::span<const std::uint8_t> shown,
                                     const std::function<bool(const Eigen::Vector2d&)>& inside) {
    // In parallel: the visibility rays are most of the cost on a big mesh
    std::vector<std::uint8_t> hit(static_cast<std::size_t>(mesh.numFaces()), 0);
    igl::parallel_for(
        mesh.numFaces(),
        [&](Eigen::Index f) {
            if (!shown.empty() && shown[static_cast<std::size_t>(f)] == 0) {
                return;
            }
            const Eigen::Vector3d c = centroid(mesh, f);
            if (view.clipped(c)) {
                return;
            }
            const std::optional<Eigen::Vector2d> p = view.project(c);
            hit[static_cast<std::size_t>(f)] = p && inside(*p) && (visibility == nullptr || visibility->visible(mesh, f, view)) ? 1 : 0;
        },
        1000);
    std::vector<Eigen::Index> faces;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (hit[static_cast<std::size_t>(f)] != 0) {
            faces.push_back(f);
        }
    }
    return faces;
}

} // namespace

bool ScreenView::clippedBy(std::span<const ClipPlane> planes, const Eigen::Vector3d& p) noexcept {
    return std::any_of(planes.begin(), planes.end(), [&](const ClipPlane& plane) { return (p - plane.origin).dot(plane.normal) < 0.0; });
}

bool faceClipped(const Mesh& mesh, Eigen::Index face, std::span<const ClipPlane> planes) noexcept {
    return !planes.empty() && ScreenView::clippedBy(planes, centroid(mesh, face));
}

std::optional<Eigen::Vector2d> ScreenView::project(const Eigen::Vector3d& p) const {
    const Eigen::Vector4d clip = viewProjection * p.homogeneous();
    if (clip.w() <= 1e-12) {
        return std::nullopt;
    }
    const double x = clip.x() / clip.w();
    const double y = clip.y() / clip.w();
    return Eigen::Vector2d((x + 1.0) * 0.5 * width, (1.0 - y) * 0.5 * height);
}

void FaceVisibility::build(const Mesh& mesh, std::span<const std::uint8_t> shown) {
    tree_ = igl::AABB<Eigen::MatrixXd, 3>();
    treeFace_.clear();
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (shown.empty() || shown[static_cast<std::size_t>(f)] != 0) {
            treeFace_.push_back(f);
        }
    }
    built_ = !mesh.empty();
    if (!built_) {
        return;
    }
    const BoundingBox box = mesh.boundingBox();
    origin_ = box.min;
    scale_ = settings::kFaceVisibilityWorkingSize / std::max(box.diagonal(), 1e-300);
    V_ = (mesh.V.rowwise() - origin_.transpose()) * scale_;
    F_.resize(static_cast<Eigen::Index>(treeFace_.size()), 3);
    for (Eigen::Index row = 0; row < F_.rows(); ++row) {
        F_.row(row) = mesh.F.row(treeFace_[static_cast<std::size_t>(row)]);
    }
    if (F_.rows() > 0) {
        tree_.init(V_, F_);
    }
}

std::optional<Eigen::Index> FaceVisibility::firstHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                                                     std::span<const ClipPlane> planes, double* t) const {
    if (F_.rows() == 0) {
        return std::nullopt;
    }
    // The planes keep a convex part of the ray, [tMin, tMax]: cast from tMin, so the first hit there
    // is the first shown one, and drop it past tMax. Scaling and moving to working space keeps t.
    double tMin = 0.0;
    double tMax = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d worldOrigin = fromWork(origin);
    const Eigen::Vector3d worldDir = dir / scale_;
    for (const ClipPlane& plane : planes) {
        const double side = (worldOrigin - plane.origin).dot(plane.normal); // >= 0 on the kept side
        const double rate = worldDir.dot(plane.normal);
        if (rate > 0.0) {
            tMin = std::max(tMin, -side / rate);
        } else if (rate < 0.0) {
            tMax = std::min(tMax, -side / rate);
        } else if (side < 0.0) {
            return std::nullopt; // parallel to the plane, on the cut side
        }
    }
    if (tMin > tMax) {
        return std::nullopt;
    }
    igl::Hit hit{};
    const Eigen::Vector3d start = origin + tMin * dir;
    if (!tree_.intersect_ray(V_, F_, start.transpose(), dir.transpose(), hit) || tMin + hit.t > tMax) {
        return std::nullopt;
    }
    if (t != nullptr) {
        *t = tMin + hit.t;
    }
    return treeFace_[static_cast<std::size_t>(hit.id)];
}

bool FaceVisibility::visible(const Mesh& mesh, Eigen::Index face, const ScreenView& view) const {
    if (faceClipped(mesh, face, view.clipPlanes)) {
        return false;
    }
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
    // face counts as seen when at least two of the three rays reach it. Points a slice plane cuts away
    // are not drawn and do not vote; the centre is kept (tested above), and with fewer than three
    // voters all of them must see the face.
    std::array<Eigen::Vector3d, 3> targets = {(a + b + c) / 3.0, 0.6 * a + 0.2 * b + 0.2 * c, 0.2 * a + 0.6 * b + 0.2 * c};
    std::size_t voters = 0;
    for (const Eigen::Vector3d& target : targets) {
        if (!view.clipped(fromWork(target))) {
            targets[voters++] = target;
        }
    }
    voters = std::max<std::size_t>(voters, 1); // the centre, should rounding put it on the plane
    const std::size_t needed = voters == 3 ? 2 : voters;
    std::size_t seen = 0;
    std::size_t missed = 0;
    for (std::size_t i = 0; i < voters; ++i) {
        if (seen >= needed || missed > voters - needed) {
            break; // the vote is decided
        }
        const Eigen::Vector3d& target = targets[i];
        const Eigen::Vector3d origin = view.orthographic ? Eigen::Vector3d(target - reach * view.viewDirection) : eye;
        const Eigen::Vector3d dir = target - origin;
        if (std::abs(normal.dot(dir.normalized())) < 1e-3) {
            return false; // edge-on: the ray runs in the face's plane
        }
        double t = 0.0;
        const std::optional<Eigen::Index> first = firstHit(origin, dir, view.clipPlanes, &t);
        if (first && (*first == face || t > 1.0 - 1e-6)) {
            ++seen;
        } else {
            ++missed;
        }
    }
    return seen >= needed;
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
    (void)mesh;
    return firstHit(origin, dir, view.clipPlanes, nullptr);
}

std::vector<Eigen::Index> withoutFacesBehind(const Mesh& mesh, const ScreenView& view, std::vector<Eigen::Index> faces,
                                             const Mesh& occluder) {
    if (occluder.empty() || faces.empty()) {
        return faces;
    }
    // Moller-Trumbore from the eye (or, orthographic, from in front along the view direction) to the face
    // centre: t = 1 at the centre, and a hit short of it by more than the tolerance hides the face
    const double reach = 4.0 * std::max(mesh.boundingBox().diagonal(), occluder.boundingBox().diagonal());
    const auto behind = [&](Eigen::Index f) {
        const Eigen::Vector3d target = centroid(mesh, f);
        const Eigen::Vector3d origin = view.orthographic ? Eigen::Vector3d(target - reach * view.viewDirection) : view.cameraPosition;
        const Eigen::Vector3d dir = target - origin;
        for (Eigen::Index o = 0; o < occluder.numFaces(); ++o) {
            const Eigen::Vector3d a = occluder.V.row(occluder.F(o, 0));
            const Eigen::Vector3d e1 = Eigen::Vector3d(occluder.V.row(occluder.F(o, 1))) - a;
            const Eigen::Vector3d e2 = Eigen::Vector3d(occluder.V.row(occluder.F(o, 2))) - a;
            const Eigen::Vector3d pv = dir.cross(e2);
            const double det = e1.dot(pv);
            if (std::abs(det) < 1e-18) {
                continue;
            }
            const Eigen::Vector3d tv = origin - a;
            const double u = tv.dot(pv) / det;
            const Eigen::Vector3d qv = tv.cross(e1);
            const double v = dir.dot(qv) / det;
            const double t = e2.dot(qv) / det;
            if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > 0.0 && t < 1.0 - 1e-3) {
                return true;
            }
        }
        return false;
    };
    std::erase_if(faces, behind);
    return faces;
}

std::vector<Eigen::Index> facesInRectangle(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a,
                                           const Eigen::Vector2d& b, const FaceVisibility* visibility,
                                           std::span<const std::uint8_t> shown) {
    const Eigen::Vector2d lo = a.cwiseMin(b);
    const Eigen::Vector2d hi = a.cwiseMax(b);
    return facesWhere(mesh, view, visibility, shown, [&](const Eigen::Vector2d& p) {
        return p.x() >= lo.x() && p.x() <= hi.x() && p.y() >= lo.y() && p.y() <= hi.y();
    });
}

std::vector<Eigen::Index> facesInStroke(const Mesh& mesh, const ScreenView& view, const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                                        double radius, const FaceVisibility* visibility, std::span<const std::uint8_t> shown) {
    const double r2 = radius * radius;
    const Eigen::Vector2d ab = b - a;
    const double len2 = ab.squaredNorm();
    return facesWhere(mesh, view, visibility, shown, [&](const Eigen::Vector2d& p) {
        const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return (p - (a + t * ab)).squaredNorm() <= r2;
    });
}

} // namespace ogun
