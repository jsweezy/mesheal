#include "diagnostics/TriTriIntersect.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mesheal {

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

/// Strictly on both sides of the other plane, beyond the tolerance.
[[nodiscard]] bool straddles(const std::array<double, 3>& d, double tol) noexcept {
    const auto [lo, hi] = std::minmax({d[0], d[1], d[2]});
    return lo < -tol && hi > tol;
}

/// Closest point on triangle (a, b, c) to `p` (Ericson, Real-Time Collision Detection, 5.1.5).
[[nodiscard]] Eigen::Vector3d closestOnTriangle(const Eigen::Vector3d& p, const Triangle3d& t) {
    const Eigen::Vector3d& a = t[0];
    const Eigen::Vector3d& b = t[1];
    const Eigen::Vector3d& c = t[2];
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    const Eigen::Vector3d ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }
    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 && d1 - d3 > 0.0) {
        return a + ab * (d1 / (d1 - d3));
    }
    const Eigen::Vector3d cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 && d2 - d6 > 0.0) {
        return a + ac * (d2 / (d2 - d6));
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 && (d4 - d3) + (d5 - d6) > 0.0) {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const double sum = va + vb + vc;
    if (!(std::abs(sum) > 0.0)) {
        return a; // degenerate: the edge regions above already covered every closest point
    }
    return a + ab * (vb / sum) + ac * (vc / sum);
}

/// Squared distance between segments p1-q1 and p2-q2 (Ericson 5.1.9).
[[nodiscard]] double segmentDistanceSquared(
    const Eigen::Vector3d& p1, const Eigen::Vector3d& q1, const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
    const Eigen::Vector3d d1 = q1 - p1;
    const Eigen::Vector3d d2 = q2 - p2;
    const Eigen::Vector3d r = p1 - p2;
    const double a = d1.squaredNorm();
    const double e = d2.squaredNorm();
    const double f = d2.dot(r);
    double s = 0.0;
    double t = 0.0;
    if (a <= 0.0 && e <= 0.0) {
        return r.squaredNorm();
    }
    if (a <= 0.0) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= 0.0) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2);
            const double denom = a * e - b * b;
            s = denom > 0.0 ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return ((p1 + d1 * s) - (p2 + d2 * t)).squaredNorm();
}

/// True when segment p-q passes through the plane of `t` at a point inside `t` (a piercing edge).
[[nodiscard]] bool segmentPiercesTriangle(const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Triangle3d& t) {
    const Plane plane = planeOf(t);
    if (!plane.valid) {
        return false;
    }
    const double dp = plane.normal.dot(p) + plane.offset;
    const double dq = plane.normal.dot(q) + plane.offset;
    if ((dp > 0.0 && dq > 0.0) || (dp < 0.0 && dq < 0.0) || dp == dq) {
        return false; // one side, or lying in the plane (the corner and edge distances cover that)
    }
    const Eigen::Vector3d x = p + (q - p) * (dp / (dp - dq));
    const Eigen::Vector3d n = (t[1] - t[0]).cross(t[2] - t[0]);
    for (std::size_t i = 0; i < 3; ++i) {
        if ((t[(i + 1) % 3] - t[i]).cross(x - t[i]).dot(n) < 0.0) {
            return false;
        }
    }
    return true;
}

} // namespace

double triangleDistance(const Triangle3d& a, const Triangle3d& b) {
    for (std::size_t i = 0; i < 3; ++i) {
        if (segmentPiercesTriangle(a[i], a[(i + 1) % 3], b) || segmentPiercesTriangle(b[i], b[(i + 1) % 3], a)) {
            return 0.0;
        }
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 3; ++i) {
        best = std::min(best, (a[i] - closestOnTriangle(a[i], b)).squaredNorm());
        best = std::min(best, (b[i] - closestOnTriangle(b[i], a)).squaredNorm());
        for (std::size_t j = 0; j < 3; ++j) {
            best = std::min(best, segmentDistanceSquared(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3]));
        }
    }
    return std::sqrt(best);
}

bool trianglesWithin(const Triangle3d& a, const Triangle3d& b, double tol) {
    // Every corner of one triangle beyond `tol` on one side of the other's plane: too far apart
    for (const auto& [from, to] : {std::pair{&a, &b}, std::pair{&b, &a}}) {
        const Plane plane = planeOf(*to);
        if (plane.valid) {
            const std::array<double, 3> d = planeDistances(plane, *from);
            if ((d[0] > tol && d[1] > tol && d[2] > tol) || (d[0] < -tol && d[1] < -tol && d[2] < -tol)) {
                return false;
            }
        }
    }
    return triangleDistance(a, b) <= tol;
}

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
    result.touching = result.hit && (!straddles(da, tol) || !straddles(db, tol));
    return result;
}

} // namespace mesheal
