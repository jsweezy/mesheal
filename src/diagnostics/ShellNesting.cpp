#include "diagnostics/ShellNesting.hpp"
#include "core/Constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>

namespace ogun {

namespace {

[[nodiscard]] Eigen::Vector3d corner(const Mesh& mesh, Eigen::Index f, int c) {
    return mesh.V.row(mesh.F(f, c)).transpose();
}

[[nodiscard]] BoundingBox boxOf(const Mesh& mesh, const std::vector<Eigen::Index>& faces) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    BoundingBox box{Eigen::Vector3d::Constant(kInf), Eigen::Vector3d::Constant(-kInf)};
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = corner(mesh, f, c);
            box.min = box.min.cwiseMin(p);
            box.max = box.max.cwiseMax(p);
        }
    }
    return box;
}

[[nodiscard]] bool boxInside(const BoundingBox& inner, const BoundingBox& outer) {
    return (inner.min.array() > outer.min.array()).all() && (inner.max.array() < outer.max.array()).all();
}

[[nodiscard]] double boxVolume(const BoundingBox& box) {
    return box.extents().prod();
}

} // namespace

bool rayHitsTriangle(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& dir,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c) noexcept {
    const Eigen::Vector3d e1 = b - a;
    const Eigen::Vector3d e2 = c - a;
    const Eigen::Vector3d p = dir.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) <= 1.0e-14 * e1.norm() * e2.norm()) {
        return false;
    }
    const double inv = 1.0 / det;
    const Eigen::Vector3d s = origin - a;
    const double u = s.dot(p) * inv;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Eigen::Vector3d q = s.cross(e1);
    const double v = dir.dot(q) * inv;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    return e2.dot(q) * inv > 0.0;
}

std::vector<int> findNestedShells(
    const Mesh& mesh,
    std::span<const std::vector<Eigen::Index>> shellFaces,
    const std::set<std::pair<int, int>>& crossingPairs) {
    const std::size_t n = shellFaces.size();
    std::vector<int> nestedIn(n, -1);
    std::vector<BoundingBox> boxes;
    boxes.reserve(n);
    for (const auto& faces : shellFaces) {
        boxes.push_back(boxOf(mesh, faces));
    }
    std::vector<std::size_t> bySize(n);
    std::iota(bySize.begin(), bySize.end(), std::size_t{0});
    std::stable_sort(bySize.begin(), bySize.end(), [&](std::size_t a, std::size_t b) {
        return boxVolume(boxes[a]) < boxVolume(boxes[b]);
    });

    // The skewed direction avoids rays that run along edges or through vertices
    const Eigen::Vector3d dir = Eigen::Vector3d(kAlphaX, kAlphaY, kAlphaZ).normalized();
    for (std::size_t s = 0; s < n; ++s) {
        if (shellFaces[s].empty()) {
            continue;
        }
        const Eigen::Vector3d probe = corner(mesh, shellFaces[s].front(), 0);
        for (const std::size_t k : bySize) {
            const auto si = static_cast<int>(s);
            const auto ki = static_cast<int>(k);
            if (k == s || !boxInside(boxes[s], boxes[k]) ||
                crossingPairs.contains({std::min(si, ki), std::max(si, ki)})) {
                continue;
            }
            std::size_t hits = 0;
            for (const Eigen::Index f : shellFaces[k]) {
                hits += rayHitsTriangle(probe, dir, corner(mesh, f, 0), corner(mesh, f, 1), corner(mesh, f, 2)) ? 1 : 0;
            }
            if (hits % 2 == 1) {
                nestedIn[s] = ki;
                break;
            }
        }
    }
    return nestedIn;
}

} // namespace ogun
