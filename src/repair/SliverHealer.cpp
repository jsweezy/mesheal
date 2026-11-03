#include "repair/SliverHealer.hpp"
#include "core/Constants.hpp"
#include "diagnostics/ReliableNormal.hpp"

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>
#include <optional>
#include <set>
#include <span>
#include <vector>

namespace ogun {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

using Triangle = std::array<Eigen::Vector3d, 3>;

Eigen::Vector3d toEigen(const CgalPoint3& p) {
    return {CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())};
}

CgalPoint3 toCgal(const Eigen::Vector3d& p) {
    return {p.x(), p.y(), p.z()};
}

Eigen::Vector3d pointOf(const CgalSurfaceMesh& mesh, CgalVertexIndex v) {
    return toEigen(mesh.point(v));
}

Eigen::Vector3d areaNormal(const Triangle& t) {
    return (t[1] - t[0]).cross(t[2] - t[0]);
}

bool hasReliableNormal(const Triangle& t) {
    return ogun::hasReliableNormal(t[0], t[1], t[2]);
}

bool isLiveFace(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    return f != CgalSurfaceMesh::null_face() && mesh.is_valid(f) && !mesh.is_removed(f);
}

using PinMap = CgalSurfaceMesh::Property_map<CgalVertexIndex, std::uint8_t>;
using VertexOriginMap = CgalSurfaceMesh::Property_map<CgalVertexIndex, std::int64_t>;
using FaceOriginMap = CgalSurfaceMesh::Property_map<CgalFaceIndex, std::int64_t>;
constexpr const char* kVertexOriginProperty = "v:ogun_origin";
constexpr const char* kFaceOriginProperty = "f:ogun_origin";

std::optional<PinMap> pinMap(const CgalSurfaceMesh& mesh) {
    return mesh.property_map<CgalVertexIndex, std::uint8_t>(SliverHealer::kPinnedVertexProperty);
}

bool isPinned(const std::optional<PinMap>& pins, CgalVertexIndex v) {
    return pins && get(*pins, v) != 0;
}

std::array<CgalHalfedgeIndex, 3> faceHalfedges(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    const CgalHalfedgeIndex h0 = mesh.halfedge(f);
    const CgalHalfedgeIndex h1 = mesh.next(h0);
    return {h0, h1, mesh.next(h1)};
}

Triangle facePoints(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    const auto hs = faceHalfedges(mesh, f);
    return {pointOf(mesh, mesh.target(hs[0])), pointOf(mesh, mesh.target(hs[1])), pointOf(mesh, mesh.target(hs[2]))};
}

double halfedgeLength(const CgalSurfaceMesh& mesh, CgalHalfedgeIndex h) {
    return (pointOf(mesh, mesh.target(h)) - pointOf(mesh, mesh.source(h))).norm();
}

// Index (0..2) of the longest or shortest halfedge of `f`.
std::size_t extremeHalfedge(const CgalSurfaceMesh& mesh, CgalFaceIndex f, bool longest) {
    const auto hs = faceHalfedges(mesh, f);
    std::size_t best = 0;
    double bestLen = halfedgeLength(mesh, hs[0]);
    for (std::size_t k = 1; k < 3; ++k) {
        const double len = halfedgeLength(mesh, hs[k]);
        if (longest ? len > bestLen : len < bestLen) {
            best = k;
            bestLen = len;
        }
    }
    return best;
}

CgalHalfedgeIndex longestHalfedge(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    return faceHalfedges(mesh, f)[extremeHalfedge(mesh, f, true)];
}

CgalHalfedgeIndex shortestHalfedge(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    return faceHalfedges(mesh, f)[extremeHalfedge(mesh, f, false)];
}

int countBorderEdges(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    int count = 0;
    for (const CgalHalfedgeIndex h : faceHalfedges(mesh, f)) {
        if (mesh.is_border(mesh.opposite(h))) {
            ++count;
        }
    }
    return count;
}

ThinFaceDefect evaluateCgalFace(const CgalSurfaceMesh& mesh, CgalFaceIndex f, const ThinFaceThresholds& thresholds) {
    const Triangle t = facePoints(mesh, f);
    return ThinFaceChecker::evaluateTriangle(t[0], t[1], t[2], thresholds);
}

std::vector<CgalFaceIndex> collectThinFaces(const CgalSurfaceMesh& mesh, const ThinFaceThresholds& thresholds) {
    std::vector<CgalFaceIndex> result;
    for (const CgalFaceIndex f : mesh.faces()) {
        if (evaluateCgalFace(mesh, f, thresholds).isDefective()) {
            result.push_back(f);
        }
    }
    return result;
}

std::size_t countThinFaces(const CgalSurfaceMesh& mesh, const ThinFaceThresholds& thresholds) {
    return collectThinFaces(mesh, thresholds).size();
}

// Checks whether flipping `h` (shared by face (s, t, a) and neighbor (t, s, o)) into diagonal a-o is safe.
bool canFlipEdge(
    const CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string* reason) {
    const auto fail = [reason](const char* msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    const CgalHalfedgeIndex oh = mesh.opposite(h);
    if (mesh.is_border(h) || mesh.is_border(oh)) {
        return fail("Longest edge lies on an open border; there is no neighbor face to flip with.");
    }
    const CgalVertexIndex s = mesh.source(h);
    const CgalVertexIndex t = mesh.target(h);
    const CgalVertexIndex a = mesh.target(mesh.next(h));
    const CgalVertexIndex o = mesh.target(mesh.next(oh));
    if (a == o || mesh.halfedge(a, o) != CgalSurfaceMesh::null_halfedge()) {
        return fail("Flipped diagonal already exists; flipping would create a duplicate edge.");
    }
    if (const std::optional<PinMap> pins = pinMap(mesh); isPinned(pins, s) && isPinned(pins, t)) {
        return fail("Both ends of the longest edge are pinned (the edge of the scope); it stays as it is.");
    }

    const Eigen::Vector3d ps = pointOf(mesh, s);
    const Eigen::Vector3d pt = pointOf(mesh, t);
    const Eigen::Vector3d pa = pointOf(mesh, a);
    const Eigen::Vector3d po = pointOf(mesh, o);

    const Triangle opp{pt, ps, po};
    const Eigen::Vector3d nOpp = areaNormal(opp);
    if (!hasReliableNormal(opp)) {
        return fail("Neighbor face across the longest edge is itself degenerate.");
    }
    const Eigen::Vector3d nOppHat = nOpp.normalized();

    const Triangle f0{ps, pt, pa};
    const Eigen::Vector3d n0 = areaNormal(f0);
    if (hasReliableNormal(f0) && n0.normalized().dot(nOppHat) < settings.coplanarDot) {
        // When f0 is an ultra-thin cap or sliver, its normal tilts wildly due to micron-scale altitude on curved surfaces.
        // Check whether neighbor faces extending across the side arms are coplanar with nOpp.
        // If the side-arm neighbors are coplanar with nOpp, the local region is smooth and not a true feature crease.
        const CgalHalfedgeIndex hArm1 = mesh.next(h);
        const CgalHalfedgeIndex hArm2 = mesh.prev(h);
        const CgalFaceIndex fArm1 = mesh.is_border(mesh.opposite(hArm1)) ? CgalSurfaceMesh::null_face() : mesh.face(mesh.opposite(hArm1));
        const CgalFaceIndex fArm2 = mesh.is_border(mesh.opposite(hArm2)) ? CgalSurfaceMesh::null_face() : mesh.face(mesh.opposite(hArm2));

        bool sideArmsCoplanar = false;
        int checkedArms = 0;
        int coplanarArms = 0;

        const double creaseDot = std::cos(settings.creaseAngleDeg * kDegToRad);
        for (const CgalFaceIndex fArm : {fArm1, fArm2}) {
            if (fArm != CgalSurfaceMesh::null_face()) {
                const Triangle tArm = facePoints(mesh, fArm);
                if (hasReliableNormal(tArm)) {
                    ++checkedArms;
                    const Eigen::Vector3d nArm = areaNormal(tArm).normalized();
                    if (nArm.dot(nOppHat) >= creaseDot) {
                        ++coplanarArms;
                    }
                }
            }
        }

        if (checkedArms > 0 && coplanarArms == checkedArms) {
            sideArmsCoplanar = true;
        }

        if (!sideArmsCoplanar) {
            return fail("Neighbor face lies across a sharp feature crease; flipping would distort the surface.");
        }
    }

    // Post-flip triangles must be non-degenerate and keep the neighbor's orientation (convex, planar quad).
    const Triangle t1{ps, po, pa};
    const Triangle t2{po, pt, pa};
    const Eigen::Vector3d n1 = areaNormal(t1);
    const Eigen::Vector3d n2 = areaNormal(t2);
    if (!hasReliableNormal(t1) || !hasReliableNormal(t2)) {
        return fail("Flip would create a zero-area triangle.");
    }
    if (n1.normalized().dot(nOppHat) < settings.coplanarDot || n2.normalized().dot(nOppHat) < settings.coplanarDot) {
        return fail("Cap and neighbor form a non-convex or non-planar quad; flipping would fold the surface.");
    }
    if (ThinFaceChecker::evaluateTriangle(t1[0], t1[1], t1[2], thresholds).isDefective() ||
        ThinFaceChecker::evaluateTriangle(t2[0], t2[1], t2[2], thresholds).isDefective()) {
        return fail("Flipping would still leave a thin / sliver triangle.");
    }
    return true;
}

bool canSplitNeighborEdge(
    const CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const SliverHealerSettings& settings,
    std::string* reason) {
    const auto fail = [reason](const char* msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    const CgalHalfedgeIndex oh = mesh.opposite(h);
    if (mesh.is_border(h) || mesh.is_border(oh)) {
        return fail("Longest edge lies on an open border; there is no neighbor face to split.");
    }
    const CgalVertexIndex s = mesh.source(h);
    const CgalVertexIndex t = mesh.target(h);
    const CgalVertexIndex a = mesh.target(mesh.next(h));
    const CgalVertexIndex o = mesh.target(mesh.next(oh));
    if (a == o || mesh.halfedge(a, o) != CgalSurfaceMesh::null_halfedge()) {
        return fail("Split diagonal already exists; splitting would create a duplicate edge.");
    }
    if (const std::optional<PinMap> pins = pinMap(mesh); isPinned(pins, s) && isPinned(pins, t)) {
        return fail("Both ends of the longest edge are pinned (the edge of the scope); it stays as it is.");
    }

    const Eigen::Vector3d ps = pointOf(mesh, s);
    const Eigen::Vector3d pt = pointOf(mesh, t);
    const Eigen::Vector3d pa = pointOf(mesh, a);
    const Eigen::Vector3d po = pointOf(mesh, o);

    const Triangle opp{pt, ps, po};
    if (!hasReliableNormal(opp)) {
        return fail("Neighbor face across the longest edge is degenerate.");
    }
    const Eigen::Vector3d nOppHat = areaNormal(opp).normalized();

    const Triangle t1{ps, po, pa};
    const Triangle t2{po, pt, pa};
    const Eigen::Vector3d n1 = areaNormal(t1);
    const Eigen::Vector3d n2 = areaNormal(t2);
    if (!hasReliableNormal(t1) || !hasReliableNormal(t2)) {
        return fail("Split would create a zero-area triangle.");
    }

    if (n1.normalized().dot(nOppHat) < settings.minNormalDot || n2.normalized().dot(nOppHat) < settings.minNormalDot) {
        return fail("Split would fold or invert the neighbor face.");
    }
    return true;
}

bool canDissolveCollinearVertex(
    const CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const SliverHealerSettings& settings,
    std::string* reason) {
    const auto fail = [reason](const char* msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    const CgalHalfedgeIndex hLong = longestHalfedge(mesh, f);
    const CgalVertexIndex apex = mesh.target(mesh.next(hLong));

    if (mesh.is_border(apex)) {
        return fail("Collinear vertex lies on an open border; cannot dissolve.");
    }
    if (const std::optional<PinMap> pins = pinMap(mesh); isPinned(pins, apex)) {
        return fail("Collinear vertex is pinned (on the edge of the scope); it stays as it is.");
    }

    int count = 0;
    Eigen::Vector3d refNormal = Eigen::Vector3d::Zero();
    for (const CgalFaceIndex adj : CGAL::faces_around_target(mesh.halfedge(apex), mesh)) {
        if (adj == CgalSurfaceMesh::null_face()) {
            return fail("Collinear vertex is on an open boundary.");
        }
        const Triangle tri = facePoints(mesh, adj);
        if (!hasReliableNormal(tri)) {
            continue;
        }
        const Eigen::Vector3d n = areaNormal(tri).normalized();
        if (count == 0) {
            refNormal = n;
        } else if (n.dot(refNormal) < settings.coplanarDot) {
            return fail("Incident faces around the vertex are not coplanar (vertex lies on a feature crease or curve).");
        }
        ++count;
    }
    if (count < 3) {
        return fail("Collinear vertex has fewer than 3 incident faces.");
    }
    return true;
}

// Smallest n_old . n_new over faces surviving the collapse of edge(h) to `newPos`
// (-infinity if a non-degenerate face would become degenerate).
double worstNormalDot(const CgalSurfaceMesh& mesh, CgalHalfedgeIndex h, const Eigen::Vector3d& newPos) {
    const CgalVertexIndex v0 = mesh.source(h);
    const CgalVertexIndex v1 = mesh.target(h);
    const CgalFaceIndex fA = mesh.face(h);
    const CgalFaceIndex fB = mesh.face(mesh.opposite(h));

    double worst = 1.0;
    for (const CgalVertexIndex v : std::array<CgalVertexIndex, 2>{v0, v1}) {
        for (const CgalFaceIndex f : CGAL::faces_around_target(mesh.halfedge(v), mesh)) {
            if (f == CgalSurfaceMesh::null_face() || f == fA || f == fB) {
                continue;
            }
            const auto hs = faceHalfedges(mesh, f);
            Triangle before;
            Triangle after;
            for (std::size_t k = 0; k < 3; ++k) {
                const CgalVertexIndex vk = mesh.target(hs[k]);
                before[k] = pointOf(mesh, vk);
                after[k] = (vk == v0 || vk == v1) ? newPos : before[k];
            }
            const Eigen::Vector3d nOld = areaNormal(before);
            if (!hasReliableNormal(before)) {
                continue; // Already degenerate: orientation cannot be judged.
            }
            const Eigen::Vector3d nNew = areaNormal(after);
            if (!hasReliableNormal(after)) {
                return -std::numeric_limits<double>::infinity();
            }
            worst = std::min(worst, nOld.normalized().dot(nNew.normalized()));
        }
    }
    return worst;
}

// Checks if edge(h) can be safely collapsed to one of candidates[0..numCandidates-1].
bool canCollapseEdgeSafely(
    const CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const std::array<Eigen::Vector3d, 3>& candidates,
    std::size_t numCandidates,
    const SliverHealerSettings& settings,
    std::size_t* chosenCandidate = nullptr,
    std::string* reason = nullptr) {
    const auto fail = [reason](const char* msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    const CgalEdgeIndex e = mesh.edge(h);
    if (!CGAL::Euler::does_satisfy_link_condition(e, mesh)) {
        return fail("Collapse violates the topological link condition (the surface would become non-manifold).");
    }

    const std::optional<PinMap> pins = pinMap(mesh);
    const bool pinned0 = isPinned(pins, mesh.source(h));
    const bool pinned1 = isPinned(pins, mesh.target(h));
    if (pinned0 && pinned1) {
        return fail("Both ends of the edge are pinned (the edge of the scope); it stays as it is.");
    }
    const CgalVertexIndex pinnedEnd = pinned0 ? mesh.source(h) : mesh.target(h);
    const Eigen::Vector3d pinnedPoint = pointOf(mesh, pinnedEnd);

    double bestDot = -std::numeric_limits<double>::infinity();
    std::size_t best = numCandidates;
    for (std::size_t i = 0; i < numCandidates; ++i) {
        if ((pinned0 || pinned1) && candidates[i] != pinnedPoint) {
            continue;
        }
        const double d = worstNormalDot(mesh, h, candidates[i]);
        if (best == numCandidates || d > bestDot + 1.0e-12) {
            bestDot = d;
            best = i;
        }
    }
    if (best == numCandidates) {
        return fail("The collapse would move a pinned vertex (the edge of the scope).");
    }
    if (!(bestDot > settings.minNormalDot)) {
        return fail("Collapse would invert or degenerate neighboring faces.");
    }
    if (chosenCandidate) {
        *chosenCandidate = best;
    }
    return true;
}

// Collapses edge(h) to the candidate position that best preserves the surrounding normals.
bool collapseEdgeSafely(
    CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const std::array<Eigen::Vector3d, 3>& candidates,
    std::size_t numCandidates,
    const SliverHealerSettings& settings,
    std::string& status) {
    std::size_t best = 0;
    std::string err;
    if (!canCollapseEdgeSafely(mesh, h, candidates, numCandidates, settings, &best, &err)) {
        status = err;
        return false;
    }

    const CgalEdgeIndex e = mesh.edge(h);
    const std::optional<PinMap> pins = pinMap(mesh);
    const bool pinned0 = isPinned(pins, mesh.source(h));
    const bool pinned1 = isPinned(pins, mesh.target(h));
    const CgalVertexIndex pinnedEnd = pinned0 ? mesh.source(h) : mesh.target(h);

    const std::optional<VertexOriginMap> origin = mesh.property_map<CgalVertexIndex, std::int64_t>(kVertexOriginProperty);
    const std::int64_t pinnedOrigin = (pinned0 || pinned1) && origin ? get(*origin, pinnedEnd) : -1;
    const CgalVertexIndex kept = CGAL::Euler::collapse_edge(e, mesh);
    mesh.point(kept) = toCgal(candidates[best]);
    if (pinned0 || pinned1) {
        put(*pins, kept, std::uint8_t{1});
        if (origin) {
            put(*origin, kept, pinnedOrigin);
        }
    }
    return true;
}

bool canSnapCollinear(
    const CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const SliverHealerSettings& settings,
    std::string* reason = nullptr) {
    const CgalHalfedgeIndex hLong = longestHalfedge(mesh, f);
    const CgalVertexIndex apex = mesh.target(mesh.next(hLong));
    std::array<CgalHalfedgeIndex, 2> arms{mesh.next(hLong), mesh.prev(hLong)};
    std::array<CgalVertexIndex, 2> corners{mesh.target(hLong), mesh.source(hLong)};
    if (halfedgeLength(mesh, arms[1]) < halfedgeLength(mesh, arms[0])) {
        std::swap(arms[0], arms[1]);
        std::swap(corners[0], corners[1]);
    }

    std::string lastErr;
    for (std::size_t i = 0; i < 2; ++i) {
        if (mesh.is_border(apex) && !mesh.is_border(corners[i])) {
            lastErr = "Snapping would pull an open border vertex into the interior.";
            continue;
        }
        const std::array<Eigen::Vector3d, 3> candidates{pointOf(mesh, corners[i]), {}, {}};
        std::string err;
        if (canCollapseEdgeSafely(mesh, arms[i], candidates, 1, settings, nullptr, &err)) {
            return true;
        }
        lastErr = std::move(err);
    }
    if (reason) {
        *reason = lastErr.empty() ? "Cap vertex cannot be snapped onto either corner." : lastErr;
    }
    return false;
}

double vertexLocalScale(const CgalSurfaceMesh& mesh, CgalVertexIndex v) {
    double maxLen = 0.0;
    for (const CgalHalfedgeIndex h : CGAL::halfedges_around_target(mesh.halfedge(v), mesh)) {
        maxLen = std::max(maxLen, (pointOf(mesh, mesh.source(h)) - pointOf(mesh, v)).norm());
    }
    return maxLen;
}

struct ClusterCollapseEval {
    double worstNormalDot = 1.0;
    int thinBefore = 0;
    int thinAfter = 0;
    bool valid = false;
};

ClusterCollapseEval evaluateClusterCollapse(
    const CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const Eigen::Vector3d& newPos,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    const CgalVertexIndex v0 = mesh.source(h);
    const CgalVertexIndex v1 = mesh.target(h);
    const CgalFaceIndex fA = mesh.face(h);
    const CgalFaceIndex fB = mesh.face(mesh.opposite(h));

    struct NeighborFace {
        Triangle before;
        Triangle after;
        double areaBefore = 0.0;
    };
    std::vector<NeighborFace> faces;
    std::set<CgalFaceIndex> seenFaces;
    double maxArea = 0.0;

    for (const CgalVertexIndex v : std::array<CgalVertexIndex, 2>{v0, v1}) {
        for (const CgalFaceIndex f : CGAL::faces_around_target(mesh.halfedge(v), mesh)) {
            if (f == CgalSurfaceMesh::null_face() || f == fA || f == fB) {
                continue;
            }
            if (!seenFaces.insert(f).second) {
                continue;
            }
            const auto hs = faceHalfedges(mesh, f);
            NeighborFace nf;
            for (std::size_t k = 0; k < 3; ++k) {
                const CgalVertexIndex vk = mesh.target(hs[k]);
                nf.before[k] = pointOf(mesh, vk);
                nf.after[k] = (vk == v0 || vk == v1) ? newPos : nf.before[k];
            }
            const Eigen::Vector3d nOld = areaNormal(nf.before);
            nf.areaBefore = 0.5 * nOld.norm();
            maxArea = std::max(maxArea, nf.areaBefore);
            faces.push_back(nf);
        }
    }

    ClusterCollapseEval eval;
    eval.valid = true;
    const double significantAreaThreshold = settings.tinyFaceAreaFraction * maxArea;

    for (const NeighborFace& nf : faces) {
        if (ThinFaceChecker::evaluateTriangle(nf.before[0], nf.before[1], nf.before[2], thresholds).isDefective()) {
            ++eval.thinBefore;
        }

        const Eigen::Vector3d nNew = areaNormal(nf.after);
        if (!hasReliableNormal(nf.after)) {
            ++eval.thinAfter;
            if (nf.areaBefore >= significantAreaThreshold) {
                eval.worstNormalDot = -std::numeric_limits<double>::infinity();
                eval.valid = false;
            }
            continue;
        }

        if (ThinFaceChecker::evaluateTriangle(nf.after[0], nf.after[1], nf.after[2], thresholds).isDefective()) {
            ++eval.thinAfter;
        }

        if (nf.areaBefore >= significantAreaThreshold && hasReliableNormal(nf.before)) {
            const double dot = areaNormal(nf.before).normalized().dot(nNew.normalized());
            eval.worstNormalDot = std::min(eval.worstNormalDot, dot);
        }
    }

    if (eval.thinAfter > eval.thinBefore || eval.worstNormalDot < settings.clusterMinNormalDot) {
        eval.valid = false;
    }
    return eval;
}

bool collapseMicroCluster(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string& status) {
    if (!isLiveFace(mesh, f)) {
        return false;
    }
    const CgalHalfedgeIndex hShort = shortestHalfedge(mesh, f);
    const CgalVertexIndex s0 = mesh.source(hShort);
    const CgalVertexIndex t0 = mesh.target(hShort);

    std::set<CgalVertexIndex> cluster{s0, t0};
    std::vector<CgalVertexIndex> queue{s0, t0};

    for (std::size_t i = 0; i < queue.size() && static_cast<int>(cluster.size()) < settings.maxMicroClusterSize; ++i) {
        const CgalVertexIndex v = queue[i];
        const double scaleV = vertexLocalScale(mesh, v);
        for (const CgalHalfedgeIndex h : CGAL::halfedges_around_target(mesh.halfedge(v), mesh)) {
            const CgalVertexIndex w = mesh.source(h);
            if (cluster.count(w)) {
                continue;
            }
            const double scaleW = vertexLocalScale(mesh, w);
            const double len = (pointOf(mesh, w) - pointOf(mesh, v)).norm();
            if (len < settings.needleEdgeRatio * std::max(scaleV, scaleW)) {
                cluster.insert(w);
                queue.push_back(w);
                if (static_cast<int>(cluster.size()) >= settings.maxMicroClusterSize) {
                    break;
                }
            }
        }
    }

    // Gather unique internal edges within the cluster
    std::vector<std::pair<double, CgalHalfedgeIndex>> clusterEdges;
    for (const CgalVertexIndex v : cluster) {
        for (const CgalHalfedgeIndex h : CGAL::halfedges_around_target(mesh.halfedge(v), mesh)) {
            const CgalVertexIndex w = mesh.source(h);
            if (cluster.count(w) && v < w) {
                clusterEdges.push_back({(pointOf(mesh, w) - pointOf(mesh, v)).norm(), h});
            }
        }
    }
    std::sort(clusterEdges.begin(), clusterEdges.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    const std::optional<PinMap> pins = pinMap(mesh);
    const std::optional<VertexOriginMap> origin = mesh.property_map<CgalVertexIndex, std::int64_t>(kVertexOriginProperty);

    for (const auto& [len, h] : clusterEdges) {
        const CgalEdgeIndex e = mesh.edge(h);
        if (mesh.is_border(h) || mesh.is_border(mesh.opposite(h))) {
            continue;
        }
        if (!CGAL::Euler::does_satisfy_link_condition(e, mesh)) {
            continue;
        }

        const CgalVertexIndex vSrc = mesh.source(h);
        const CgalVertexIndex vTgt = mesh.target(h);
        const bool pinned0 = isPinned(pins, vSrc);
        const bool pinned1 = isPinned(pins, vTgt);
        if (pinned0 && pinned1) {
            continue;
        }

        const Eigen::Vector3d pSrc = pointOf(mesh, vSrc);
        const Eigen::Vector3d pTgt = pointOf(mesh, vTgt);

        std::array<Eigen::Vector3d, 3> candidates;
        std::size_t numCandidates = 0;
        if (pinned0) {
            candidates[0] = pSrc;
            numCandidates = 1;
        } else if (pinned1) {
            candidates[0] = pTgt;
            numCandidates = 1;
        } else {
            candidates = {0.5 * (pSrc + pTgt), pTgt, pSrc};
            numCandidates = 3;
        }

        int bestIndex = -1;
        double bestDot = -std::numeric_limits<double>::infinity();

        for (std::size_t k = 0; k < numCandidates; ++k) {
            const ClusterCollapseEval eval = evaluateClusterCollapse(mesh, h, candidates[k], thresholds, settings);
            if (!eval.valid) {
                continue;
            }
            if (eval.worstNormalDot > bestDot) {
                bestDot = eval.worstNormalDot;
                bestIndex = static_cast<int>(k);
            }
        }

        if (bestIndex < 0) {
            continue;
        }

        const CgalVertexIndex pinnedEnd = pinned0 ? vSrc : vTgt;
        const std::int64_t pinnedOrigin = (pinned0 || pinned1) && origin ? get(*origin, pinnedEnd) : -1;
        const CgalVertexIndex kept = CGAL::Euler::collapse_edge(e, mesh);
        mesh.point(kept) = toCgal(candidates[static_cast<std::size_t>(bestIndex)]);
        if (pinned0 || pinned1) {
            put(*pins, kept, std::uint8_t{1});
            if (origin) {
                put(*origin, kept, pinnedOrigin);
            }
        }
        status = "Collapsed micro-edge in needle cluster.";
        return true;
    }

    return false;
}

bool splitNeighborEdge(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string& status) {
    (void)thresholds;
    const CgalHalfedgeIndex hLong = longestHalfedge(mesh, f);
    std::string err;
    if (canSplitNeighborEdge(mesh, hLong, settings, &err)) {
        CGAL::Euler::flip_edge(hLong, mesh);
        status = "Split the neighbor face across the longest edge.";
        return true;
    }
    status = err.empty() ? "Cannot split neighbor face." : err;
    return false;
}

bool dissolveCollinearVertex(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string& status) {
    (void)thresholds;
    std::string err;
    if (!canDissolveCollinearVertex(mesh, f, settings, &err)) {
        status = err.empty() ? "Cannot dissolve collinear vertex." : err;
        return false;
    }

    const CgalHalfedgeIndex hLong = longestHalfedge(mesh, f);
    const CgalVertexIndex apex = mesh.target(mesh.next(hLong));
    const CgalHalfedgeIndex hIn = mesh.halfedge(apex);

    const CgalHalfedgeIndex hRet = CGAL::Euler::remove_center_vertex(hIn, mesh);
    const CgalFaceIndex mergedFace = mesh.face(hRet);
    if (mergedFace == CgalSurfaceMesh::null_face()) {
        status = "Failed to merge incident faces around collinear vertex.";
        return false;
    }

    if (!PMP::triangulate_face(mergedFace, mesh)) {
        status = "Failed to retriangulate coplanar patch after dissolving vertex.";
        return false;
    }

    status = "Dissolved the collinear vertex and retriangulated the coplanar patch.";
    return true;
}

bool flipLongestEdge(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string& status) {
    const auto hs = faceHalfedges(mesh, f);
    std::array<CgalHalfedgeIndex, 3> sortedHs = hs;
    std::sort(sortedHs.begin(), sortedHs.end(), [&](CgalHalfedgeIndex a, CgalHalfedgeIndex b) {
        return halfedgeLength(mesh, a) > halfedgeLength(mesh, b);
    });

    for (std::size_t i = 0; i < sortedHs.size(); ++i) {
        std::string err;
        if (canFlipEdge(mesh, sortedHs[i], thresholds, settings, &err)) {
            CGAL::Euler::flip_edge(sortedHs[i], mesh);
            status = i == 0 ? "Flipped the longest edge into the neighbor's opposite diagonal."
                            : "Flipped edge into the neighbor's opposite diagonal.";
            return true;
        }
        if (i == 0) {
            status = err;
        }
    }
    return false;
}

bool collapseShortestEdge(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex f,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    std::string& status) {
    const CgalHalfedgeIndex h = shortestHalfedge(mesh, f);
    const CgalVertexIndex v0 = mesh.source(h);
    const CgalVertexIndex v1 = mesh.target(h);
    const Eigen::Vector3d p0 = pointOf(mesh, v0);
    const Eigen::Vector3d p1 = pointOf(mesh, v1);
    const bool border0 = mesh.is_border(v0);
    const bool border1 = mesh.is_border(v1);

    // Keep open boundaries in place: a border endpoint never moves toward the interior.
    std::array<Eigen::Vector3d, 3> candidates{0.5 * (p0 + p1), p1, p0};
    std::size_t count = 3;
    if (border0 != border1) {
        candidates[0] = border0 ? p0 : p1;
        count = 1;
    }
    if (collapseEdgeSafely(mesh, h, candidates, count, settings, status)) {
        status = "Collapsed the shortest edge.";
        return true;
    }
    if (collapseMicroCluster(mesh, f, thresholds, settings, status)) {
        return true;
    }
    return false;
}

bool snapCollinearCollapse(CgalSurfaceMesh& mesh, CgalFaceIndex f, const SliverHealerSettings& settings, std::string& status) {
    // Face (s, t, apex): the side-arms are t->apex (corner t) and apex->s (corner s).
    const CgalHalfedgeIndex hLong = longestHalfedge(mesh, f);
    const CgalVertexIndex apex = mesh.target(mesh.next(hLong));
    std::array<CgalHalfedgeIndex, 2> arms{mesh.next(hLong), mesh.prev(hLong)};
    std::array<CgalVertexIndex, 2> corners{mesh.target(hLong), mesh.source(hLong)};
    if (halfedgeLength(mesh, arms[1]) < halfedgeLength(mesh, arms[0])) {
        std::swap(arms[0], arms[1]);
        std::swap(corners[0], corners[1]);
    }

    status = "Cap vertex cannot be snapped onto either corner.";
    for (std::size_t i = 0; i < 2; ++i) {
        if (mesh.is_border(apex) && !mesh.is_border(corners[i])) {
            status = "Snapping would pull an open border vertex into the interior.";
            continue;
        }
        const std::array<Eigen::Vector3d, 3> candidates{pointOf(mesh, corners[i]), {}, {}};
        if (collapseEdgeSafely(mesh, arms[i], candidates, 1, settings, status)) {
            status = "Snapped the cap vertex onto its corner vertex.";
            return true;
        }
    }
    return false;
}

bool removeDegenerateFace(CgalSurfaceMesh& mesh, CgalFaceIndex f, std::string& status) {
    if (countBorderEdges(mesh, f) < 2) {
        status = "Removing this face would open a hole in the surface (it has fewer than two border edges).";
        return false;
    }
    // A pinned vertex used by this face only would go with it
    if (const std::optional<PinMap> pins = pinMap(mesh)) {
        for (const CgalHalfedgeIndex h : faceHalfedges(mesh, f)) {
            const CgalVertexIndex v = mesh.target(h);
            if (isPinned(pins, v)) {
                std::size_t faces = 0;
                for (const CgalFaceIndex g : CGAL::faces_around_target(mesh.halfedge(v), mesh)) {
                    faces += g != CgalSurfaceMesh::null_face() ? 1 : 0;
                }
                if (faces <= 1) {
                    status = "Removing this face would remove a pinned vertex (the edge of the scope).";
                    return false;
                }
            }
        }
    }
    CGAL::Euler::remove_face(mesh.halfedge(f), mesh);
    status = "Removed the dangling degenerate face.";
    return true;
}

void countAction(RepairResults& results, HealAction action) {
    switch (action) {
    case HealAction::SplitNeighborEdge:
    case HealAction::FlipLongestEdge:
        ++results.edgeFlips;
        break;
    case HealAction::CollapseShortestEdge:
    case HealAction::SnapCollinearCollapse:
        ++results.edgeCollapses;
        break;
    case HealAction::DissolveCollinearVertex:
    case HealAction::RemoveDegenerateFace:
        ++results.facesRemoved;
        break;
    case HealAction::None:
        break;
    }
}

// Recommended action first, followed by every other repair as a fallback.
std::array<HealAction, 6> fallbackOrder(HealAction recommended) {
    std::array<HealAction, 6> order;
    switch (recommended) {
    case HealAction::FlipLongestEdge:
        order = {HealAction::FlipLongestEdge, HealAction::DissolveCollinearVertex, HealAction::SnapCollinearCollapse,
                 HealAction::SplitNeighborEdge, HealAction::CollapseShortestEdge, HealAction::RemoveDegenerateFace};
        break;
    case HealAction::DissolveCollinearVertex:
        order = {HealAction::DissolveCollinearVertex, HealAction::FlipLongestEdge, HealAction::SnapCollinearCollapse,
                 HealAction::SplitNeighborEdge, HealAction::CollapseShortestEdge, HealAction::RemoveDegenerateFace};
        break;
    case HealAction::SnapCollinearCollapse:
        order = {HealAction::SnapCollinearCollapse, HealAction::CollapseShortestEdge, HealAction::SplitNeighborEdge,
                 HealAction::DissolveCollinearVertex, HealAction::FlipLongestEdge, HealAction::RemoveDegenerateFace};
        break;
    case HealAction::SplitNeighborEdge:
        order = {HealAction::SplitNeighborEdge, HealAction::DissolveCollinearVertex, HealAction::SnapCollinearCollapse,
                 HealAction::FlipLongestEdge, HealAction::CollapseShortestEdge, HealAction::RemoveDegenerateFace};
        break;
    case HealAction::CollapseShortestEdge:
        order = {HealAction::CollapseShortestEdge, HealAction::SnapCollinearCollapse, HealAction::SplitNeighborEdge,
                 HealAction::DissolveCollinearVertex, HealAction::FlipLongestEdge, HealAction::RemoveDegenerateFace};
        break;
    case HealAction::RemoveDegenerateFace:
    case HealAction::None:
        order = {HealAction::RemoveDegenerateFace, HealAction::CollapseShortestEdge, HealAction::SnapCollinearCollapse,
                 HealAction::SplitNeighborEdge, HealAction::DissolveCollinearVertex, HealAction::FlipLongestEdge};
        break;
    }
    return order;
}

double boundingDiagonal(const CgalSurfaceMesh& mesh) {
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo;
    for (const CgalVertexIndex v : mesh.vertices()) {
        const Eigen::Vector3d p = pointOf(mesh, v);
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    return mesh.number_of_vertices() > 0 ? (hi - lo).norm() : 0.0;
}

bool toFaceIndex(const CgalSurfaceMesh& mesh, Eigen::Index faceIndex, CgalFaceIndex& out) {
    if (faceIndex < 0 || static_cast<std::size_t>(faceIndex) >= mesh.number_of_faces()) {
        return false;
    }
    out = CgalFaceIndex(static_cast<CgalSurfaceMesh::size_type>(faceIndex));
    return isLiveFace(mesh, out);
}

std::vector<Eigen::Index> identityMap(Eigen::Index n) {
    std::vector<Eigen::Index> map(static_cast<std::size_t>(n));
    std::iota(map.begin(), map.end(), Eigen::Index{0});
    return map;
}

/// Builds the halfedge mesh of a batch repair with origin and pin properties. False (with `status`)
/// when it cannot be built, or when pins are given but the input had to be read as a polygon soup.
bool buildTracked(const Mesh& mesh, std::span<const std::uint8_t> pinned, CgalSurfaceMesh& surface, bool& tracked, std::string& status) {
    if (!pinned.empty() && pinned.size() != static_cast<std::size_t>(mesh.numVertices())) {
        status = "The pinned-vertex mask does not match the mesh.";
        return false;
    }
    bool soup = false;
    if (!toCgalSurfaceMesh(mesh, surface, &soup)) {
        status = "Mesh could not be converted to a halfedge mesh.";
        return false;
    }
    const bool anyPinned = std::any_of(pinned.begin(), pinned.end(), [](std::uint8_t p) { return p != 0; });
    if (soup && anyPinned) {
        status = "The mesh could not be read without renumbering its vertices, so pinned vertices cannot be kept: "
                 "fix its non-manifold edges and flipped faces first.";
        return false;
    }
    tracked = !soup;
    surface.set_recycle_garbage(false); // new elements get new indices, so their origin stays -1
    if (tracked) {
        VertexOriginMap vertexOrigin = surface.add_property_map<CgalVertexIndex, std::int64_t>(kVertexOriginProperty, -1).first;
        FaceOriginMap faceOrigin = surface.add_property_map<CgalFaceIndex, std::int64_t>(kFaceOriginProperty, -1).first;
        for (const CgalVertexIndex v : surface.vertices()) {
            put(vertexOrigin, v, static_cast<std::int64_t>(v.idx()));
        }
        for (const CgalFaceIndex f : surface.faces()) {
            put(faceOrigin, f, static_cast<std::int64_t>(f.idx()));
        }
    }
    if (anyPinned) {
        PinMap pins = surface.add_property_map<CgalVertexIndex, std::uint8_t>(SliverHealer::kPinnedVertexProperty, 0).first;
        for (const CgalVertexIndex v : surface.vertices()) {
            put(pins, v, pinned[v.idx()]);
        }
    }
    return true;
}

/// The result mesh with the kept faces in input order, then new faces, plus the face and vertex maps.
void finishTracked(const CgalSurfaceMesh& surface, const Mesh& input, bool tracked, RepairResults& results) {
    results.mesh = fromCgalSurfaceMesh(surface, input.fileUnit);
    if (!tracked) {
        results.faceMap.clear();
        results.vertexMap.clear();
        return;
    }
    const FaceOriginMap faceOrigin = *surface.property_map<CgalFaceIndex, std::int64_t>(kFaceOriginProperty);
    const VertexOriginMap vertexOrigin = *surface.property_map<CgalVertexIndex, std::int64_t>(kVertexOriginProperty);

    // `fromCgalSurfaceMesh` writes live faces in index order and used vertices in index order
    std::vector<std::int64_t> origin;
    origin.reserve(surface.number_of_faces());
    std::vector<std::uint8_t> used(surface.num_vertices() + surface.number_of_removed_vertices(), 0);
    for (const CgalFaceIndex f : surface.faces()) {
        origin.push_back(get(faceOrigin, f));
        for (const CgalHalfedgeIndex h : faceHalfedges(surface, f)) {
            used[surface.target(h).idx()] = 1;
        }
    }
    std::vector<std::size_t> order(origin.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const auto key = [&](std::size_t k) { return origin[k] < 0 ? std::numeric_limits<std::int64_t>::max() : origin[k]; };
        return key(a) < key(b);
    });
    Eigen::MatrixXi faces(results.mesh.F.rows(), 3);
    results.faceMap.assign(static_cast<std::size_t>(input.numFaces()), -1);
    for (std::size_t k = 0; k < order.size(); ++k) {
        faces.row(static_cast<Eigen::Index>(k)) = results.mesh.F.row(static_cast<Eigen::Index>(order[k]));
        const std::int64_t o = origin[order[k]];
        if (o >= 0 && o < input.numFaces()) {
            results.faceMap[static_cast<std::size_t>(o)] = static_cast<Eigen::Index>(k);
        }
    }
    results.mesh.F = std::move(faces);
    results.mesh.recomputeNormals();

    results.vertexMap.assign(static_cast<std::size_t>(input.numVertices()), -1);
    Eigen::Index next = 0;
    for (const CgalVertexIndex v : surface.vertices()) {
        if (used[v.idx()] == 0) {
            continue;
        }
        const std::int64_t o = get(vertexOrigin, v);
        if (o >= 0 && o < input.numVertices()) {
            results.vertexMap[static_cast<std::size_t>(o)] = next;
        }
        ++next;
    }
}

/// The input back, unchanged.
void leaveUnchanged(const Mesh& mesh, RepairResults& results) {
    results.mesh = mesh;
    results.faceMap = identityMap(mesh.numFaces());
    results.vertexMap = identityMap(mesh.numVertices());
    results.remainingThinFaces = results.initialThinFaces;
    results.edgeFlips = 0;
    results.edgeCollapses = 0;
    results.facesRemoved = 0;
    results.success = false;
}

void cancel(const Mesh& mesh, RepairResults& results) {
    leaveUnchanged(mesh, results);
    results.cancelled = true;
    results.statusMessage = "Cancelled.";
}

} // namespace

const char* sliverTypeLabel(SliverType type) noexcept {
    switch (type) {
    case SliverType::Healthy:
        return "HEALTHY";
    case SliverType::Needle:
        return "NEEDLE";
    case SliverType::Cap:
        return "CAP";
    case SliverType::Microscopic:
        return "MICROSCOPIC";
    case SliverType::DanglingFlap:
        return "DANGLING FLAP";
    }
    return "UNKNOWN";
}

const char* healActionLabel(HealAction action) noexcept {
    switch (action) {
    case HealAction::None:
        return "None";
    case HealAction::SplitNeighborEdge:
        return "Split Neighbor Face";
    case HealAction::DissolveCollinearVertex:
        return "Dissolve Collinear Vertex";
    case HealAction::FlipLongestEdge:
        return "Flip Longest Edge";
    case HealAction::CollapseShortestEdge:
        return "Collapse Shortest Edge";
    case HealAction::SnapCollinearCollapse:
        return "Snap Collinear Vertex";
    case HealAction::RemoveDegenerateFace:
        return "Remove Degenerate Flap";
    }
    return "Unknown";
}

SliverAnalysis SliverHealer::analyzeFace(
    const CgalSurfaceMesh& mesh,
    CgalFaceIndex face,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    SliverAnalysis analysis;
    if (!isLiveFace(mesh, face)) {
        return analysis;
    }
    analysis.faceIndex = static_cast<Eigen::Index>(face.idx());

    const auto hs = faceHalfedges(mesh, face);
    std::array<double, 3> len{};
    for (std::size_t k = 0; k < 3; ++k) {
        len[k] = halfedgeLength(mesh, hs[k]);
    }
    const std::size_t iLong = extremeHalfedge(mesh, face, true);
    analysis.minEdgeLength = *std::min_element(len.begin(), len.end());
    analysis.maxEdgeLength = len[iLong];
    analysis.edgeRatio = analysis.maxEdgeLength > 0.0 ? analysis.minEdgeLength / analysis.maxEdgeLength : 0.0;

    // Law of cosines for the angle opposite the longest edge c: cos = (a^2 + b^2 - c^2) / 2ab.
    const double a = len[(iLong + 1) % 3];
    const double b = len[(iLong + 2) % 3];
    const double c = len[iLong];
    if (a * b > 0.0) {
        const double cosAngle = std::clamp((a * a + b * b - c * c) / (2.0 * a * b), -1.0, 1.0);
        analysis.maxAngleDeg = std::acos(cosAngle) * kRadToDeg;
    } else {
        analysis.maxAngleDeg = 180.0;
    }

    const Triangle tri = facePoints(mesh, face);
    const ThinFaceDefect defect = ThinFaceChecker::evaluateTriangle(tri[0], tri[1], tri[2], thresholds);
    analysis.area = defect.area;
    analysis.violatesThresholds = defect.isDefective();

    analysis.numBorderEdges = countBorderEdges(mesh, face);
    analysis.longestEdgeIsBorder = mesh.is_border(mesh.opposite(hs[iLong]));
    analysis.longestEdgeFlippable = canFlipEdge(mesh, hs[iLong], thresholds, settings, nullptr);
    if (!analysis.longestEdgeFlippable) {
        for (const CgalHalfedgeIndex h : hs) {
            if (h != hs[iLong] && canFlipEdge(mesh, h, thresholds, settings, nullptr)) {
                analysis.longestEdgeFlippable = true;
                break;
            }
        }
    }

    if (analysis.edgeRatio < settings.needleEdgeRatio) {
        analysis.type = SliverType::Needle;
    } else if (analysis.maxAngleDeg > settings.capAngleDeg) {
        analysis.type = SliverType::Cap;
    } else if (analysis.violatesThresholds) {
        analysis.type = SliverType::Microscopic;
    }

    if (analysis.type != SliverType::Healthy) {
        const bool zeroArea = !hasReliableNormal(tri);
        if (analysis.numBorderEdges == 3 || (analysis.numBorderEdges >= 2 && zeroArea)) {
            analysis.type = SliverType::DanglingFlap;
        }
    }

    switch (analysis.type) {
    case SliverType::Healthy:
        analysis.recommendedAction = HealAction::None;
        break;
    case SliverType::Needle:
    case SliverType::Microscopic:
        analysis.recommendedAction = HealAction::CollapseShortestEdge;
        break;
    case SliverType::Cap: {
        if (analysis.longestEdgeFlippable) {
            analysis.recommendedAction = HealAction::FlipLongestEdge;
        } else if (canDissolveCollinearVertex(mesh, face, settings, nullptr)) {
            analysis.recommendedAction = HealAction::DissolveCollinearVertex;
        } else if (canSnapCollinear(mesh, face, settings, nullptr)) {
            analysis.recommendedAction = HealAction::SnapCollinearCollapse;
        } else if (canSplitNeighborEdge(mesh, hs[iLong], settings, nullptr)) {
            analysis.recommendedAction = HealAction::SplitNeighborEdge;
        } else {
            analysis.recommendedAction = HealAction::SnapCollinearCollapse;
        }
        break;
    }
    case SliverType::DanglingFlap:
        analysis.recommendedAction = HealAction::RemoveDegenerateFace;
        break;
    }
    return analysis;
}

SliverAnalysis SliverHealer::analyzeFace(
    const Mesh& mesh,
    Eigen::Index faceIndex,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    CgalSurfaceMesh surface;
    CgalFaceIndex face;
    if (!toCgalSurfaceMesh(mesh, surface) || !toFaceIndex(surface, faceIndex, face)) {
        return {};
    }
    return analyzeFace(surface, face, thresholds, settings);
}

bool SliverHealer::applyHealAction(
    CgalSurfaceMesh& mesh,
    CgalFaceIndex face,
    HealAction action,
    const SliverHealerSettings& settings,
    std::string* statusOut,
    const ThinFaceThresholds& thresholds) {
    std::string status;
    bool ok = false;
    if (!isLiveFace(mesh, face)) {
        status = "Face is not part of the mesh.";
    } else {
        switch (action) {
        case HealAction::None:
            status = "No repair action selected.";
            break;
        case HealAction::SplitNeighborEdge:
            ok = splitNeighborEdge(mesh, face, thresholds, settings, status);
            break;
        case HealAction::DissolveCollinearVertex:
            ok = dissolveCollinearVertex(mesh, face, thresholds, settings, status);
            break;
        case HealAction::FlipLongestEdge:
            ok = flipLongestEdge(mesh, face, thresholds, settings, status);
            break;
        case HealAction::CollapseShortestEdge:
            ok = collapseShortestEdge(mesh, face, thresholds, settings, status);
            break;
        case HealAction::SnapCollinearCollapse:
            ok = snapCollinearCollapse(mesh, face, settings, status);
            break;
        case HealAction::RemoveDegenerateFace:
            ok = removeDegenerateFace(mesh, face, status);
            break;
        }
    }
    if (statusOut) {
        *statusOut = status;
    }
    return ok;
}

SingleFaceRepairPreview SliverHealer::previewSingleFaceRepair(
    const CgalSurfaceMesh& mesh,
    CgalFaceIndex face,
    std::optional<HealAction> action,
    Unit unit,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    SingleFaceRepairPreview preview;
    if (!isLiveFace(mesh, face)) {
        preview.statusMessage = "Face is not part of the mesh.";
        return preview;
    }
    preview.analysis = analyzeFace(mesh, face, thresholds, settings);
    preview.appliedAction = action.value_or(preview.analysis.recommendedAction);
    if (preview.appliedAction == HealAction::None) {
        preview.statusMessage = "Face is healthy; no repair needed.";
        return preview;
    }

    CgalSurfaceMesh patch;
    std::vector<CgalFaceIndex> sourceFaces;
    const CgalFaceIndex center = extractLocalPatch(mesh, face, settings.previewRings, patch, &sourceFaces);
    if (center == CgalSurfaceMesh::null_face()) {
        preview.statusMessage = "Could not extract the local neighborhood patch around this face.";
        return preview;
    }
    if (!applyHealAction(patch, center, preview.appliedAction, settings, &preview.statusMessage, thresholds)) {
        return preview;
    }
    const double offset = settings.previewOffsetFactor * boundingDiagonal(patch);
    preview.previewPatchMesh = fromCgalSurfaceMesh(patch, unit, offset);
    preview.valid = !preview.previewPatchMesh.empty() || preview.appliedAction == HealAction::RemoveDegenerateFace;
    if (preview.valid) {
        preview.sourceFaceIndices.reserve(sourceFaces.size());
        for (const CgalFaceIndex f : sourceFaces) {
            preview.sourceFaceIndices.push_back(static_cast<Eigen::Index>(f.idx()));
        }
    }
    return preview;
}

SingleFaceRepairPreview SliverHealer::previewSingleFaceRepair(
    const Mesh& mesh,
    Eigen::Index faceIndex,
    std::optional<HealAction> action,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    CgalSurfaceMesh surface;
    CgalFaceIndex face;
    if (!toCgalSurfaceMesh(mesh, surface) || !toFaceIndex(surface, faceIndex, face)) {
        SingleFaceRepairPreview preview;
        preview.statusMessage = "Face index is out of range for this mesh.";
        return preview;
    }
    return previewSingleFaceRepair(surface, face, action, mesh.fileUnit, thresholds, settings);
}

RepairResults SliverHealer::healSingleFace(
    const Mesh& mesh,
    Eigen::Index faceIndex,
    HealAction action,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings) {
    RepairResults results;
    results.mesh = mesh;
    results.initialThinFaces = ThinFaceChecker::check(mesh, thresholds).numDefects();
    results.remainingThinFaces = results.initialThinFaces;

    CgalSurfaceMesh surface;
    if (!toCgalSurfaceMesh(mesh, surface)) {
        results.statusMessage = "Mesh could not be converted to a halfedge mesh.";
        return results;
    }
    CgalFaceIndex face;
    if (!toFaceIndex(surface, faceIndex, face)) {
        results.statusMessage = "Face index is out of range for this mesh.";
        return results;
    }
    if (!applyHealAction(surface, face, action, settings, &results.statusMessage, thresholds)) {
        return results;
    }
    countAction(results, action);

    surface.collect_garbage();
    results.mesh = fromCgalSurfaceMesh(surface, mesh.fileUnit);
    results.remainingThinFaces = ThinFaceChecker::check(results.mesh, thresholds).numDefects();
    results.success = true;
    return results;
}

RepairResults SliverHealer::healFaces(
    const Mesh& mesh,
    const std::vector<Eigen::Index>& faceIndices,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    const SliverHealOptions& options) {
    RepairResults results;
    results.initialThinFaces = ThinFaceChecker::check(mesh, thresholds).numDefects();
    leaveUnchanged(mesh, results);
    if (options.stopToken.stop_requested()) {
        cancel(mesh, results);
        return results;
    }

    CgalSurfaceMesh surface;
    bool tracked = false;
    if (!buildTracked(mesh, options.pinnedVertices, surface, tracked, results.statusMessage)) {
        return results;
    }

    // Lazy deletion keeps the handles of faces not yet visited valid while earlier repairs run
    std::vector<CgalFaceIndex> faces;
    faces.reserve(faceIndices.size());
    for (const Eigen::Index faceIndex : faceIndices) {
        CgalFaceIndex face;
        if (toFaceIndex(surface, faceIndex, face)) {
            faces.push_back(face);
        }
    }

    const std::size_t initialCount = faces.size();
    std::size_t healed = 0;
    for (int iter = 0; iter < settings.maxBatchIterations; ++iter) {
        bool changed = false;
        for (const CgalFaceIndex f : faces) {
            if (options.stopToken.stop_requested()) {
                cancel(mesh, results);
                return results;
            }
            if (!isLiveFace(surface, f)) {
                continue;
            }
            const SliverAnalysis a = analyzeFace(surface, f, thresholds, settings);
            if (a.recommendedAction == HealAction::None) {
                continue;
            }
            for (const HealAction action : fallbackOrder(a.recommendedAction)) {
                if (applyHealAction(surface, f, action, settings, nullptr, thresholds)) {
                    countAction(results, action);
                    ++healed;
                    changed = true;
                    break;
                }
            }
        }
        if (!changed) {
            break;
        }
    }

    if (healed == 0) {
        leaveUnchanged(mesh, results);
        results.statusMessage = "None of the " + std::to_string(initialCount) + " selected faces could be repaired safely.";
        return results;
    }
    surface.collect_garbage();
    finishTracked(surface, mesh, tracked, results);
    results.remainingThinFaces = ThinFaceChecker::check(results.mesh, thresholds).numDefects();
    results.success = true;
    results.statusMessage = "Repaired " + std::to_string(healed) + " of " + std::to_string(initialCount) +
                            " selected faces (" + std::to_string(results.remainingThinFaces) + " thin faces remaining).";
    return results;
}

RepairResults SliverHealer::healAllThinFaces(
    const Mesh& mesh,
    const ThinFaceThresholds& thresholds,
    const SliverHealerSettings& settings,
    const SliverHealOptions& options) {
    RepairResults results;
    results.initialThinFaces = ThinFaceChecker::check(mesh, thresholds).numDefects();
    leaveUnchanged(mesh, results);
    const std::stop_token& stop = options.stopToken;
    if (stop.stop_requested()) {
        cancel(mesh, results);
        return results;
    }

    CgalSurfaceMesh surface;
    bool tracked = false;
    if (!buildTracked(mesh, options.pinnedVertices, surface, tracked, results.statusMessage)) {
        return results;
    }

    for (int iter = 0; iter < settings.maxBatchIterations; ++iter) {
        bool changed = false;

        // Pass 1: flip coplanar caps and remove dangling flaps.
        for (const CgalFaceIndex f : collectThinFaces(surface, thresholds)) {
            if (stop.stop_requested()) {
                cancel(mesh, results);
                return results;
            }
            if (!isLiveFace(surface, f)) {
                continue;
            }
            const SliverAnalysis a = analyzeFace(surface, f, thresholds, settings);
            const bool pass1 = a.recommendedAction == HealAction::FlipLongestEdge ||
                               a.recommendedAction == HealAction::SplitNeighborEdge ||
                               a.recommendedAction == HealAction::DissolveCollinearVertex ||
                               a.recommendedAction == HealAction::RemoveDegenerateFace;
            if (pass1 && applyHealAction(surface, f, a.recommendedAction, settings, nullptr, thresholds)) {
                countAction(results, a.recommendedAction);
                changed = true;
            }
        }

        // Pass 2: collapse needles, creased caps and microscopic faces (with fallbacks).
        for (const CgalFaceIndex f : collectThinFaces(surface, thresholds)) {
            if (stop.stop_requested()) {
                cancel(mesh, results);
                return results;
            }
            if (!isLiveFace(surface, f)) {
                continue;
            }
            const SliverAnalysis a = analyzeFace(surface, f, thresholds, settings);
            if (a.recommendedAction == HealAction::None) {
                continue;
            }
            for (const HealAction action : fallbackOrder(a.recommendedAction)) {
                if (applyHealAction(surface, f, action, settings, nullptr, thresholds)) {
                    countAction(results, action);
                    changed = true;
                    break;
                }
            }
        }

        results.iterations = iter + 1;
        if (!changed || countThinFaces(surface, thresholds) == 0) {
            break;
        }
    }
    if (stop.stop_requested()) {
        cancel(mesh, results);
        return results;
    }

    // Pass 3: CGAL cleanup of whatever the local operators could not heal, protecting sharp creases
    // (and pinned vertices, and edges between two of them).
    surface.collect_garbage();
    const std::vector<CgalFaceIndex> remaining = collectThinFaces(surface, thresholds);
    if (!remaining.empty()) {
        const std::optional<PinMap> pins = pinMap(surface);
        std::optional<CgalSurfaceMesh> beforeCleanup;
        if (pins) {
            beforeCleanup = surface;
        }
        auto creaseMap = surface.add_property_map<CgalEdgeIndex, bool>("e:ogun_crease", false).first;
        PMP::detect_sharp_edges(surface, settings.creaseAngleDeg, creaseMap);
        bool cleanupFailed = false;
        try {
            if (pins) {
                for (const CgalEdgeIndex e : surface.edges()) {
                    const CgalHalfedgeIndex h = surface.halfedge(e);
                    if (isPinned(pins, surface.source(h)) && isPinned(pins, surface.target(h))) {
                        put(creaseMap, e, true);
                    }
                }
                PMP::remove_almost_degenerate_faces(
                    remaining,
                    surface,
                    CGAL::parameters::cap_threshold(std::cos(settings.capAngleDeg * kDegToRad))
                        .needle_threshold(1.0 / settings.needleEdgeRatio)
                        .edge_is_constrained_map(creaseMap)
                        .vertex_is_constrained_map(*pins));
            } else {
                PMP::remove_almost_degenerate_faces(
                    remaining,
                    surface,
                    CGAL::parameters::cap_threshold(std::cos(settings.capAngleDeg * kDegToRad))
                        .needle_threshold(1.0 / settings.needleEdgeRatio)
                        .edge_is_constrained_map(creaseMap));
            }
        } catch (const std::exception&) {
            cleanupFailed = true; // leave the mesh as healed by passes 1-2
        }
        surface.remove_property_map(creaseMap);

        // CGAL keeps a constrained vertex but may not promise its position: undo the pass if one moved
        if (pins && !cleanupFailed) {
            const VertexOriginMap origin = *surface.property_map<CgalVertexIndex, std::int64_t>(kVertexOriginProperty);
            const PinMap after = *pinMap(surface);
            std::size_t pinnedLeft = 0;
            bool moved = false;
            for (const CgalVertexIndex v : surface.vertices()) {
                if (get(after, v) == 0) {
                    continue;
                }
                ++pinnedLeft;
                const std::int64_t o = get(origin, v);
                moved = moved || o < 0 || pointOf(surface, v) != Eigen::Vector3d(mesh.V.row(static_cast<Eigen::Index>(o)).transpose());
            }
            const auto pinnedBefore = static_cast<std::size_t>(
                std::count_if(options.pinnedVertices.begin(), options.pinnedVertices.end(), [](std::uint8_t p) { return p != 0; }));
            if (moved || pinnedLeft < pinnedBefore) {
                surface = std::move(*beforeCleanup);
            }
        }
        surface.collect_garbage();
        results.usedPmpCleanup = true;
    }

    finishTracked(surface, mesh, tracked, results);
    results.remainingThinFaces = ThinFaceChecker::check(results.mesh, thresholds).numDefects();
    results.success = results.remainingThinFaces < results.initialThinFaces || results.initialThinFaces == 0;
    results.statusMessage = "Healed " + std::to_string(results.initialThinFaces - std::min(results.initialThinFaces, results.remainingThinFaces)) +
                            " of " + std::to_string(results.initialThinFaces) + " thin faces (" +
                            std::to_string(results.edgeFlips) + " flips, " +
                            std::to_string(results.edgeCollapses) + " collapses, " +
                            std::to_string(results.facesRemoved) + " removals).";
    return results;
}

} // namespace ogun
