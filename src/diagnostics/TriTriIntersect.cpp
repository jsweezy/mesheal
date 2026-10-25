#include "diagnostics/TriTriIntersect.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace ogun {

namespace {

struct Plane {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero(); ///< Unit length, so plane distances are lengths
    double offset = 0.0;
    bool valid = false;
};

[[nodiscard]] Plane planeOf(const Triangle3d& t) {
    Plane plane;
    const Eigen::Vector3d n = (t[1] - t[0]).cross(t[2] - t[0]);
    const double length = n.norm();
    if (!(length > 0.0) || !std::isfinite(length)) {
        return plane;
    }
    plane.normal = n / length;
    plane.offset = -plane.normal.dot(t[0]);
    plane.valid = true;
    return plane;
}

/// Signed distances of `t`'s corners to `plane`. Unlike Möller's EPSILON, single distances are not
/// snapped to 0: a vertex within `tol` of a nearly parallel plane can lie up to tol / sin(angle) from
/// the intersection line, so snapping it would invent long overlaps between smooth neighbours.
[[nodiscard]] std::array<double, 3> planeDistances(const Plane& plane, const Triangle3d& t) {
    return {plane.normal.dot(t[0]) + plane.offset,
            plane.normal.dot(t[1]) + plane.offset,
            plane.normal.dot(t[2]) + plane.offset};
}

[[nodiscard]] bool allOnOneSide(const std::array<double, 3>& d) noexcept {
    return (d[0] > 0.0 && d[1] > 0.0 && d[2] > 0.0) || (d[0] < 0.0 && d[1] < 0.0 && d[2] < 0.0);
}

/// Coplanar within the tolerance: every corner is closer than `tol` to the other plane.
[[nodiscard]] bool allOnPlane(const std::array<double, 3>& d, double tol) noexcept {
    return std::abs(d[0]) < tol && std::abs(d[1]) < tol && std::abs(d[2]) < tol;
}

struct Interval {
    double lo = 0.0;
    double hi = 0.0;
};

/// Eq. 4: the interval where the triangle meets the planes' intersection line. `lone` is the vertex
/// alone on its side of the other plane; `p` are the vertices projected onto the line (eq. 6).
[[nodiscard]] Interval lineInterval(const std::array<double, 3>& p, const std::array<double, 3>& d, std::size_t lone) {
    const std::size_t i1 = (lone + 1) % 3;
    const std::size_t i2 = (lone + 2) % 3;
    const double t0 = p[lone] + (p[i1] - p[lone]) * d[lone] / (d[lone] - d[i1]);
    const double t1 = p[lone] + (p[i2] - p[lone]) * d[lone] / (d[lone] - d[i2]);
    return {std::min(t0, t1), std::max(t0, t1)};
}

/// The vertex on its own side of the other plane (Möller's compute_intervals case analysis).
[[nodiscard]] std::size_t loneVertex(const std::array<double, 3>& d) noexcept {
    if (d[0] * d[1] > 0.0) {
        return 2;
    }
    if (d[0] * d[2] > 0.0) {
        return 1;
    }
    if (d[1] * d[2] > 0.0 || d[0] != 0.0) {
        return 0;
    }
    return d[1] != 0.0 ? 1 : 2;
}

/// Coplanar case: separating-axis test on the 6 in-plane edge normals. Interiors overlap (a hit) only
/// if every axis overlaps by more than `tol`, so neighbours touching at a vertex or edge never hit.
[[nodiscard]] bool coplanarOverlap(const Triangle3d& a, const Triangle3d& b, const Eigen::Vector3d& normal, double tol) {
    const auto separatedAlong = [&](const Eigen::Vector3d& edge) {
        const Eigen::Vector3d axis = normal.cross(edge);
        const double length = axis.norm();
        if (!(length > 0.0)) {
            return false;
        }
        double loA = axis.dot(a[0]);
        double hiA = loA;
        double loB = axis.dot(b[0]);
        double hiB = loB;
        for (std::size_t i = 1; i < 3; ++i) {
            const double pa = axis.dot(a[i]);
            const double pb = axis.dot(b[i]);
            loA = std::min(loA, pa);
            hiA = std::max(hiA, pa);
            loB = std::min(loB, pb);
            hiB = std::max(hiB, pb);
        }
        return (std::min(hiA, hiB) - std::max(loA, loB)) / length <= tol;
    };
    for (const Triangle3d* t : {&a, &b}) {
        for (std::size_t i = 0; i < 3; ++i) {
            if (separatedAlong((*t)[(i + 1) % 3] - (*t)[i])) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

TriTriHit intersectTriangles(const Triangle3d& a, const Triangle3d& b, int sharedVertices, double tol) {
    TriTriHit result;
    if (sharedVertices >= 3) {
        return result;
    }

    // Steps 1-2: reject if A lies strictly on one side of B's plane (coplanar if within tol of it)
    const Plane planeB = planeOf(b);
    const Plane planeA = planeOf(a);
    if (!planeA.valid || !planeB.valid) {
        return result;
    }
    // Coplanar is tested first so faces stacked within tol of each other are not rejected
    const std::array<double, 3> da = planeDistances(planeB, a);
    if (allOnPlane(da, tol)) {
        result.coplanar = true;
        result.hit = coplanarOverlap(a, b, planeB.normal, tol);
        return result;
    }
    if (allOnOneSide(da)) {
        return result;
    }
    // Edge neighbours that are not coplanar meet only along their shared edge
    if (sharedVertices == 2) {
        return result;
    }

    // Steps 3-4: reject if B lies strictly on one side of A's plane
    const std::array<double, 3> db = planeDistances(planeA, b);
    if (allOnPlane(db, tol)) {
        result.coplanar = true;
        result.hit = coplanarOverlap(a, b, planeA.normal, tol);
        return result;
    }
    if (allOnOneSide(db)) {
        return result;
    }

    // Step 5: project onto the coordinate axis most aligned with the intersection line (eq. 6)
    const Eigen::Vector3d dir = planeA.normal.cross(planeB.normal);
    Eigen::Index axis = 0;
    dir.cwiseAbs().maxCoeff(&axis);
    if (!(std::abs(dir[axis]) > 0.0)) {
        return result;
    }
    const std::array<double, 3> pa{a[0][axis], a[1][axis], a[2][axis]};
    const std::array<double, 3> pb{b[0][axis], b[1][axis], b[2][axis]};

    // Steps 6-7: overlap of the two intervals, converted from the axis to a true length on the line
    const Interval ia = lineInterval(pa, da, loneVertex(da));
    const Interval ib = lineInterval(pb, db, loneVertex(db));
    const double overlap = (std::min(ia.hi, ib.hi) - std::max(ia.lo, ib.lo)) * dir.norm() / std::abs(dir[axis]);
    result.hit = overlap > tol;
    return result;
}

} // namespace ogun
