#include "repair/SliverHealer.hpp"

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <vector>

namespace ogun {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A triangle normal is only trusted when |2A| exceeds this fraction of its squared longest edge.
constexpr double kReliableNormalRatio = 1.0e-10;

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

bool hasReliableNormal(const Triangle& t, const Eigen::Vector3d& n) {
    const double maxEdgeSq = std::max({(t[1] - t[0]).squaredNorm(),
                                       (t[2] - t[1]).squaredNorm(),
                                       (t[0] - t[2]).squaredNorm()});
    return maxEdgeSq > 0.0 && n.norm() > kReliableNormalRatio * maxEdgeSq;
}

bool isLiveFace(const CgalSurfaceMesh& mesh, CgalFaceIndex f) {
    return f != CgalSurfaceMesh::null_face() && mesh.is_valid(f) && !mesh.is_removed(f);
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

    const Eigen::Vector3d ps = pointOf(mesh, s);
    const Eigen::Vector3d pt = pointOf(mesh, t);
    const Eigen::Vector3d pa = pointOf(mesh, a);
    const Eigen::Vector3d po = pointOf(mesh, o);

    const Triangle opp{pt, ps, po};
    const Eigen::Vector3d nOpp = areaNormal(opp);
    if (!hasReliableNormal(opp, nOpp)) {
        return fail("Neighbor face across the longest edge is itself degenerate.");
    }
    const Eigen::Vector3d nOppHat = nOpp.normalized();

    const Triangle f0{ps, pt, pa};
    const Eigen::Vector3d n0 = areaNormal(f0);
    if (hasReliableNormal(f0, n0) && n0.normalized().dot(nOppHat) < settings.coplanarDot) {
        return fail("Neighbor face lies across a sharp feature crease; flipping would distort the surface.");
    }

    // Post-flip triangles must be non-degenerate and keep the neighbor's orientation (convex, planar quad).
    const Triangle t1{ps, po, pa};
    const Triangle t2{po, pt, pa};
    const Eigen::Vector3d n1 = areaNormal(t1);
    const Eigen::Vector3d n2 = areaNormal(t2);
    if (!hasReliableNormal(t1, n1) || !hasReliableNormal(t2, n2)) {
        return fail("Flip would create a zero-area triangle.");
    }
    if (n1.normalized().dot(nOppHat) < settings.coplanarDot || n2.normalized().dot(nOppHat) < settings.coplanarDot) {
        return fail("Cap and neighbor form a non-convex or non-planar quad; flipping would fold the surface.");
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
            if (!hasReliableNormal(before, nOld)) {
                continue; // Already degenerate: orientation cannot be judged.
            }
            const Eigen::Vector3d nNew = areaNormal(after);
            if (!hasReliableNormal(after, nNew)) {
                return -std::numeric_limits<double>::infinity();
            }
            worst = std::min(worst, nOld.normalized().dot(nNew.normalized()));
        }
    }
    return worst;
}

// Collapses edge(h) to the candidate position that best preserves the surrounding normals.
bool collapseEdgeSafely(
    CgalSurfaceMesh& mesh,
    CgalHalfedgeIndex h,
    const std::array<Eigen::Vector3d, 3>& candidates,
    std::size_t numCandidates,
    const SliverHealerSettings& settings,
    std::string& status) {
    const CgalEdgeIndex e = mesh.edge(h);
    if (!CGAL::Euler::does_satisfy_link_condition(e, mesh)) {
        status = "Collapse violates the topological link condition (the surface would become non-manifold).";
        return false;
    }

    double bestDot = -std::numeric_limits<double>::infinity();
    std::size_t best = 0;
    for (std::size_t i = 0; i < numCandidates; ++i) {
        const double d = worstNormalDot(mesh, h, candidates[i]);
        if (d > bestDot + 1.0e-12) {
            bestDot = d;
            best = i;
        }
    }
    if (!(bestDot > settings.minNormalDot)) {
        status = "Collapse would invert or degenerate neighboring faces.";
        return false;
    }

    const CgalVertexIndex kept = CGAL::Euler::collapse_edge(e, mesh);
    mesh.point(kept) = toCgal(candidates[best]);
    return true;
}

bool flipLongestEdge(CgalSurfaceMesh& mesh, CgalFaceIndex f, const SliverHealerSettings& settings, std::string& status) {
    const CgalHalfedgeIndex h = longestHalfedge(mesh, f);
    if (!canFlipEdge(mesh, h, settings, &status)) {
        return false;
    }
    CGAL::Euler::flip_edge(h, mesh);
    status = "Flipped the longest edge into the neighbor's opposite diagonal.";
    return true;
}

bool collapseShortestEdge(CgalSurfaceMesh& mesh, CgalFaceIndex f, const SliverHealerSettings& settings, std::string& status) {
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
    if (!collapseEdgeSafely(mesh, h, candidates, count, settings, status)) {
        return false;
    }
    status = "Collapsed the shortest edge.";
    return true;
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
    CGAL::Euler::remove_face(mesh.halfedge(f), mesh);
    status = "Removed the dangling degenerate face.";
    return true;
}

void countAction(RepairResults& results, HealAction action) {
    switch (action) {
    case HealAction::FlipLongestEdge:
        ++results.edgeFlips;
        break;
    case HealAction::CollapseShortestEdge:
    case HealAction::SnapCollinearCollapse:
        ++results.edgeCollapses;
        break;
    case HealAction::RemoveDegenerateFace:
        ++results.facesRemoved;
        break;
    case HealAction::None:
        break;
    }
}

// Recommended action first, followed by every other repair as a fallback.
std::array<HealAction, 4> fallbackOrder(HealAction recommended) {
    constexpr std::array<HealAction, 4> kAll{
        HealAction::CollapseShortestEdge,
        HealAction::SnapCollinearCollapse,
        HealAction::FlipLongestEdge,
        HealAction::RemoveDegenerateFace};
    std::array<HealAction, 4> order{recommended, HealAction::None, HealAction::None, HealAction::None};
    std::size_t n = 1;
    for (const HealAction a : kAll) {
        if (a != recommended && n < order.size()) {
            order[n++] = a;
        }
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
        analysis.maxAngleDeg = std::acos(cosAngle) * 180.0 / kPi;
    } else {
        analysis.maxAngleDeg = 180.0;
    }

    const Triangle tri = facePoints(mesh, face);
    const ThinFaceDefect defect = ThinFaceChecker::evaluateTriangle(tri[0], tri[1], tri[2], thresholds);
    analysis.area = defect.area;
    analysis.violatesThresholds = defect.isDefective();

    analysis.numBorderEdges = countBorderEdges(mesh, face);
    analysis.longestEdgeIsBorder = mesh.is_border(mesh.opposite(hs[iLong]));
    analysis.longestEdgeFlippable = canFlipEdge(mesh, hs[iLong], settings, nullptr);

    if (analysis.edgeRatio < settings.needleEdgeRatio) {
        analysis.type = SliverType::Needle;
    } else if (analysis.maxAngleDeg > settings.capAngleDeg) {
        analysis.type = SliverType::Cap;
    } else if (analysis.violatesThresholds) {
        analysis.type = SliverType::Microscopic;
    }

    if (analysis.type != SliverType::Healthy) {
        const bool zeroArea = !hasReliableNormal(tri, areaNormal(tri));
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
    case SliverType::Cap:
        analysis.recommendedAction = analysis.longestEdgeFlippable ? HealAction::FlipLongestEdge
                                                                   : HealAction::SnapCollinearCollapse;
        break;
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
    std::string* statusOut) {
    std::string status;
    bool ok = false;
    if (!isLiveFace(mesh, face)) {
        status = "Face is not part of the mesh.";
    } else {
        switch (action) {
        case HealAction::None:
            status = "No repair action selected.";
            break;
        case HealAction::FlipLongestEdge:
            ok = flipLongestEdge(mesh, face, settings, status);
            break;
        case HealAction::CollapseShortestEdge:
            ok = collapseShortestEdge(mesh, face, settings, status);
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
    const CgalFaceIndex center = extractLocalPatch(mesh, face, settings.previewRings, patch);
    if (center == CgalSurfaceMesh::null_face()) {
        preview.statusMessage = "Could not extract the local neighborhood patch around this face.";
        return preview;
    }
    if (!applyHealAction(patch, center, preview.appliedAction, settings, &preview.statusMessage)) {
        return preview;
    }
    const double offset = settings.previewOffsetFactor * boundingDiagonal(patch);
    preview.previewPatchMesh = fromCgalSurfaceMesh(patch, unit, offset);
    preview.valid = !preview.previewPatchMesh.empty() || preview.appliedAction == HealAction::RemoveDegenerateFace;
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
    if (!applyHealAction(surface, face, action, settings, &results.statusMessage)) {
        return results;
    }
    countAction(results, action);

    surface.collect_garbage();
    results.mesh = fromCgalSurfaceMesh(surface, mesh.fileUnit);
    results.remainingThinFaces = ThinFaceChecker::check(results.mesh, thresholds).numDefects();
    results.success = true;
    return results;
}

RepairResults SliverHealer::healAllThinFaces(
    const Mesh& mesh,
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

    for (int iter = 0; iter < settings.maxBatchIterations; ++iter) {
        bool changed = false;

        // Pass 1: flip coplanar caps and remove dangling flaps.
        for (const CgalFaceIndex f : collectThinFaces(surface, thresholds)) {
            if (!isLiveFace(surface, f)) {
                continue;
            }
            const SliverAnalysis a = analyzeFace(surface, f, thresholds, settings);
            const bool pass1 = a.recommendedAction == HealAction::FlipLongestEdge ||
                               a.recommendedAction == HealAction::RemoveDegenerateFace;
            if (pass1 && applyHealAction(surface, f, a.recommendedAction, settings)) {
                countAction(results, a.recommendedAction);
                changed = true;
            }
        }

        // Pass 2: collapse needles, creased caps and microscopic faces (with fallbacks).
        for (const CgalFaceIndex f : collectThinFaces(surface, thresholds)) {
            if (!isLiveFace(surface, f)) {
                continue;
            }
            const SliverAnalysis a = analyzeFace(surface, f, thresholds, settings);
            if (a.recommendedAction == HealAction::None) {
                continue;
            }
            for (const HealAction action : fallbackOrder(a.recommendedAction)) {
                if (applyHealAction(surface, f, action, settings)) {
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

    // Pass 3: CGAL cleanup of whatever the local operators could not heal, protecting sharp creases.
    surface.collect_garbage();
    const std::vector<CgalFaceIndex> remaining = collectThinFaces(surface, thresholds);
    if (!remaining.empty()) {
        auto creaseMap = surface.add_property_map<CgalEdgeIndex, bool>("e:ogun_crease", false).first;
        PMP::detect_sharp_edges(surface, settings.creaseAngleDeg, creaseMap);
        try {
            PMP::remove_almost_degenerate_faces(
                remaining,
                surface,
                CGAL::parameters::cap_threshold(std::cos(settings.capAngleDeg * kPi / 180.0))
                    .needle_threshold(1.0 / settings.needleEdgeRatio)
                    .edge_is_constrained_map(creaseMap));
        } catch (const std::exception&) {
            // Leave the mesh as healed by passes 1-2.
        }
        surface.remove_property_map(creaseMap);
        surface.collect_garbage();
        results.usedPmpCleanup = true;
    }

    results.mesh = fromCgalSurfaceMesh(surface, mesh.fileUnit);
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
