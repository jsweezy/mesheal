#include "repair/SliverHealer.hpp"

#include "core/Constants.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "repair/FixAll.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/TopologyCleanup.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/CgalMesh.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>

#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <stop_token>
#include <vector>

namespace {

// Planar cap (s, t, a) with neighbor (t, s, o) across its longest edge s-t and two faces above the side-arms.
// Vertices: s = 0, t = 1, a = 2, o = 3, u = 4. Face 0 is the cap.
ogun::Mesh makeCapMesh(double apexY, const Eigen::RowVector3d& opposite = {0.0, -1.0, 0.0}) {
    ogun::Mesh mesh;
    mesh.V.resize(5, 3);
    mesh.V << -1.0, 0.0, 0.0,
               1.0, 0.0, 0.0,
               0.0, apexY, 0.0,
               0.0, 0.0, 0.0,
               0.0, 1.0, 0.0;
    mesh.V.row(3) = opposite;
    mesh.F.resize(4, 3);
    mesh.F << 0, 1, 2,
              1, 0, 3,
              0, 2, 4,
              2, 1, 4;
    mesh.recomputeNormals();
    return mesh;
}

double capApexForAngle(double maxAngleDeg) {
    return std::tan((180.0 - maxAngleDeg) * 0.5 * ogun::kDegToRad);
}

// Planar disk with needle (p, q, r) (face 0) whose 1e-4 short edge p-q is shared with needle (q, p, w).
ogun::Mesh makeNeedleMesh() {
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << -1.0,   0.0, 0.0,  // L
               0.0,   0.0, 0.0,  // p
               1.0e-4, 0.0, 0.0, // q
               1.0,   0.0, 0.0,  // R
               5.0e-5, 1.0, 0.0, // r
               5.0e-5, -1.0, 0.0; // w
    mesh.F.resize(6, 3);
    mesh.F << 1, 2, 4,
              2, 1, 5,
              0, 1, 4,
              2, 3, 4,
              1, 0, 5,
              3, 2, 5;
    mesh.recomputeNormals();
    return mesh;
}

ogun::Mesh makeTetrahedron() {
    ogun::Mesh mesh;
    mesh.V.resize(4, 3);
    mesh.V << 0.0, 0.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
    mesh.F.resize(4, 3);
    mesh.F << 0, 2, 1,
              0, 1, 3,
              1, 2, 3,
              0, 3, 2;
    mesh.recomputeNormals();
    return mesh;
}

// Closed cube [0, side]^3 with every face subdivided into an n x n grid of quads (2 triangles each).
ogun::Mesh makeSubdividedCube(int n, double side) {
    std::map<std::array<int, 3>, int> ids;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::array<int, 3>> tris;
    const auto vid = [&](const std::array<int, 3>& c) {
        const auto [it, inserted] = ids.emplace(c, static_cast<int>(points.size()));
        if (inserted) {
            points.emplace_back(side * c[0] / n, side * c[1] / n, side * c[2] / n);
        }
        return it->second;
    };
    for (int axis = 0; axis < 3; ++axis) {
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;
        for (const int layer : {0, n}) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    std::array<std::array<int, 3>, 4> c{};
                    for (auto& corner : c) {
                        corner[static_cast<std::size_t>(axis)] = layer;
                    }
                    const std::array<std::array<int, 2>, 4> uv{{{i, j}, {i + 1, j}, {i + 1, j + 1}, {i, j + 1}}};
                    for (std::size_t k = 0; k < 4; ++k) {
                        c[k][static_cast<std::size_t>(u)] = uv[k][0];
                        c[k][static_cast<std::size_t>(v)] = uv[k][1];
                    }
                    const int c00 = vid(c[0]);
                    const int c10 = vid(c[1]);
                    const int c11 = vid(c[2]);
                    const int c01 = vid(c[3]);
                    if (layer == n) {
                        tris.push_back({c00, c10, c11});
                        tris.push_back({c00, c11, c01});
                    } else {
                        tris.push_back({c00, c11, c10});
                        tris.push_back({c00, c01, c11});
                    }
                }
            }
        }
    }
    ogun::Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(tris.size()), 3);
    for (std::size_t f = 0; f < tris.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << tris[f][0], tris[f][1], tris[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

Eigen::Index nearestFace(const ogun::Mesh& mesh, const Eigen::Vector3d& target) {
    Eigen::Index best = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d centroid =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        const double d = (centroid - target).norm();
        if (d < bestDist) {
            bestDist = d;
            best = f;
        }
    }
    return best;
}

Eigen::Vector3d vertexOf(const ogun::Mesh& mesh, Eigen::Index f, Eigen::Index corner) {
    return mesh.V.row(mesh.F(f, corner)).transpose();
}

// Splits face f = (a, b, c) at `pos` into (a, b, m), (b, c, m), (c, a, m); returns m's index.
int splitFace(ogun::Mesh& mesh, Eigen::Index f, const Eigen::Vector3d& pos) {
    const int a = mesh.F(f, 0);
    const int b = mesh.F(f, 1);
    const int c = mesh.F(f, 2);
    const int m = static_cast<int>(mesh.numVertices());
    mesh.V.conservativeResize(m + 1, 3);
    mesh.V.row(m) = pos.transpose();
    const Eigen::Index nf = mesh.numFaces();
    mesh.F.conservativeResize(nf + 2, 3);
    mesh.F.row(f) << a, b, m;
    mesh.F.row(nf) << b, c, m;
    mesh.F.row(nf + 1) << c, a, m;
    mesh.recomputeNormals();
    return m;
}

bool isClosedManifold(const ogun::Mesh& mesh) {
    ogun::CgalSurfaceMesh surface;
    return ogun::toCgalSurfaceMesh(mesh, surface) && CGAL::is_closed(surface) && surface.is_valid(false);
}

bool allNormalsAlign(const ogun::Mesh& mesh, const Eigen::Vector3d& n, double minDot) {
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (mesh.N.row(f).dot(n.transpose()) < minDot) {
            return false;
        }
    }
    return true;
}


// ---- CAD-style test geometry (built in memory) ----------------------------------------------------------------

ogun::Mesh meshFromPoints(const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 3>>& tris) {
    ogun::Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(tris.size()), 3);
    for (std::size_t i = 0; i < tris.size(); ++i) {
        mesh.F.row(static_cast<Eigen::Index>(i)) << tris[i][0], tris[i][1], tris[i][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

// Open patch of a CAD part, modelled on a real tessellated corner: the front wall (plane y = 0) ends in a nearly
// straight top edge L - A - R (z 5.75 / 5.7 / 5.7) whose vertex A is a true corner: the ceiling (z = 5.7) and
// the step wall (x = 12) meet the wall there. A cap sliver (L, A, R) sits on that edge next to a second sliver
// (L, R, B) whose vertex B is a fold, so no flip, dissolve or split can heal the cap. The ceiling is fanned from
// A into long needles. The old healer snapped A onto R and tilted the step wall.
// Face 2 is the cap.
constexpr Eigen::Index kCornerCapFace = 2;

ogun::Mesh makeCornerPatch(int needleSegments = 180) {
    std::vector<Eigen::Vector3d> P;
    const auto add = [&](double x, double y, double z) {
        P.emplace_back(x, y, z);
        return static_cast<int>(P.size()) - 1;
    };
    const int BL = add(-30, 0, 0), B12 = add(12, 0, 0), A = add(12, 0, 5.7), L = add(-30, 0, 5.75);
    const int R = add(30, 0, 5.7), B = add(-6.4532, 0, 5.75), TL = add(-30, 0, 17);
    const int TM = add(-6.4532, 0, 17), TR = add(30, 5.27, 17);
    const int C = add(12, 36, 5.7), D = add(12, 36, 0), E = add(30, 36, 5.7);
    std::vector<std::array<int, 3>> T;
    T.push_back({BL, B12, L});
    T.push_back({B12, A, L});
    T.push_back({L, A, R}); // the cap (face 2)
    T.push_back({L, R, B});
    T.push_back({L, B, TL});
    T.push_back({B, TM, TL});
    T.push_back({B, TR, TM});
    T.push_back({B, R, TR});
    T.push_back({B12, D, C});
    T.push_back({B12, C, A});
    int prev = R;
    for (int i = 1; i <= needleSegments; ++i) {
        const int q = (i == needleSegments) ? E : add(30, 36.0 * i / needleSegments, 5.7);
        T.push_back({A, q, prev});
        prev = q;
    }
    T.push_back({A, C, E});
    return meshFromPoints(P, T);
}

// Flat strip [0, 40] x [0, 1] (or two halves folded 90 degrees along the rung at x = 20) triangulated badly: fans
// from two corners over the far edges. Every triangle is a long needle that Delaunay flips turn into compact ones.
ogun::Mesh makeFannedStrip(bool fold) {
    constexpr int kCells = 20;
    std::vector<Eigen::Vector3d> P;
    std::array<int, kCells + 1> b{};
    std::array<int, kCells + 1> t{};
    for (int i = 0; i <= kCells; ++i) {
        const double x = 2.0 * i;
        for (int row = 0; row < 2; ++row) {
            P.push_back(fold && x > 20.0 ? Eigen::Vector3d(20.0, row, x - 20.0) : Eigen::Vector3d(x, row, 0.0));
            (row == 0 ? b : t)[static_cast<std::size_t>(i)] = static_cast<int>(P.size()) - 1;
        }
    }
    const auto B = [&](int i) { return b[static_cast<std::size_t>(i)]; };
    const auto Tp = [&](int i) { return t[static_cast<std::size_t>(i)]; };
    std::vector<std::array<int, 3>> T;
    const int mid = fold ? kCells / 2 : kCells;
    for (int i = 0; i < mid; ++i) {
        T.push_back({B(0), Tp(i + 1), Tp(i)});
        T.push_back({Tp(mid), B(i), B(i + 1)});
    }
    if (fold) {
        for (int i = mid; i < kCells; ++i) {
            T.push_back({B(mid), Tp(i + 1), Tp(i)});
            T.push_back({Tp(kCells), B(i), B(i + 1)});
        }
    }
    return meshFromPoints(P, T);
}

// Flat 40 x 40 plate (z = 0) with a 96-sided hole of radius 3 whose wall goes 3 down. The plate is fanned from its
// four corners into needles whose short edge is a hole-rim segment, like a CAD hole.
ogun::Mesh makeHolePlate() {
    constexpr int kSides = 96;
    constexpr double kRadius = 3.0;
    std::vector<Eigen::Vector3d> P;
    const auto add = [&](double x, double y, double z) {
        P.emplace_back(x, y, z);
        return static_cast<int>(P.size()) - 1;
    };
    const std::array<int, 4> corner{add(40, 40, 0), add(0, 40, 0), add(0, 0, 0), add(40, 0, 0)};
    std::array<int, kSides> top{};
    std::array<int, kSides> bottom{};
    for (int k = 0; k < kSides; ++k) {
        const double a = 2.0 * ogun::kPi * k / kSides;
        top[static_cast<std::size_t>(k)] = add(20 + kRadius * std::cos(a), 20 + kRadius * std::sin(a), 0.0);
    }
    for (int k = 0; k < kSides; ++k) {
        const double a = 2.0 * ogun::kPi * k / kSides;
        bottom[static_cast<std::size_t>(k)] = add(20 + kRadius * std::cos(a), 20 + kRadius * std::sin(a), -3.0);
    }
    const auto Top = [&](int k) { return top[static_cast<std::size_t>(k % kSides)]; };
    const auto Bot = [&](int k) { return bottom[static_cast<std::size_t>(k % kSides)]; };
    std::vector<std::array<int, 3>> T;
    for (int q = 0; q < 4; ++q) {
        const int c = corner[static_cast<std::size_t>(q)];
        for (int k = 24 * q; k < 24 * q + 24; ++k) {
            T.push_back({c, Top(k + 1), Top(k)});
        }
        T.push_back({c, corner[static_cast<std::size_t>((q + 1) % 4)], Top(24 * (q + 1))});
    }
    for (int k = 0; k < kSides; ++k) {
        T.push_back({Top(k), Top(k + 1), Bot(k)});
        T.push_back({Bot(k), Top(k + 1), Bot(k + 1)});
    }
    return meshFromPoints(P, T);
}

bool hasVertexAt(const ogun::Mesh& mesh, const Eigen::Vector3d& p) {
    for (Eigen::Index i = 0; i < mesh.numVertices(); ++i) {
        if ((mesh.V.row(i).transpose() - p).norm() == 0.0) {
            return true;
        }
    }
    return false;
}

bool hasEdgeBetween(const ogun::Mesh& mesh, const Eigen::Vector3d& p, const Eigen::Vector3d& q) {
    const auto at = [&](Eigen::Index f, int k) { return Eigen::Vector3d(mesh.V.row(mesh.F(f, k)).transpose()); };
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        for (int k = 0; k < 3; ++k) {
            if ((at(f, k) == p && at(f, (k + 1) % 3) == q) || (at(f, k) == q && at(f, (k + 1) % 3) == p)) {
                return true;
            }
        }
    }
    return false;
}

bool sameMesh(const ogun::Mesh& a, const ogun::Mesh& b) {
    return a.V.rows() == b.V.rows() && a.F.rows() == b.F.rows() && a.V == b.V && a.F == b.F;
}

std::size_t countIntersectingFaces(const ogun::Mesh& mesh) {
    return ogun::checkSelfIntersections(mesh, ogun::CheckParams{}).size();
}

} // namespace

TEST_CASE("analyzeFace classifies Caps vs. Needles", "[repair][classify]") {
    SECTION("179.5 deg cap with equal side-arms is a Cap that recommends FlipLongestEdge") {
        const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(makeCapMesh(capApexForAngle(179.5)), 0);
        CHECK(a.faceIndex == 0);
        CHECK(a.type == ogun::SliverType::Cap);
        CHECK(a.recommendedAction == ogun::HealAction::FlipLongestEdge);
        CHECK(a.maxAngleDeg == Catch::Approx(179.5).margin(1e-6));
        CHECK(a.longestEdgeFlippable);
        CHECK_FALSE(a.longestEdgeIsBorder);
    }

    SECTION("triangle with l_min / l_max = 1e-4 is a Needle that recommends CollapseShortestEdge") {
        const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(makeNeedleMesh(), 0);
        CHECK(a.type == ogun::SliverType::Needle);
        CHECK(a.recommendedAction == ogun::HealAction::CollapseShortestEdge);
        CHECK(a.edgeRatio == Catch::Approx(1e-4).margin(1e-7));
        CHECK(a.violatesThresholds);
    }

    SECTION("well-shaped faces are Healthy") {
        const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(ogun::SampleMeshes::createCubeMesh(), 0);
        CHECK(a.type == ogun::SliverType::Healthy);
        CHECK(a.recommendedAction == ogun::HealAction::None);
        CHECK(a.maxAngleDeg == Catch::Approx(90.0));
    }

    SECTION("isolated microscopic island is a DanglingFlap that recommends RemoveDegenerateFace") {
        // Well shaped (aspect 2): only an advanced height floor flags it
        const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
        CHECK(ogun::SliverHealer::analyzeFace(sample, ogun::SampleMeshes::kMicroscopicFaceIndex).type ==
              ogun::SliverType::Healthy);
        const ogun::ThinFaceThresholds heightFloor{1.0e-4, 0.0, ogun::settings::kDefaultThinFaceMaxAspectRatio};
        const ogun::SliverAnalysis a =
            ogun::SliverHealer::analyzeFace(sample, ogun::SampleMeshes::kMicroscopicFaceIndex, heightFloor);
        CHECK(a.numBorderEdges == 3);
        CHECK(a.type == ogun::SliverType::DanglingFlap);
        CHECK(a.recommendedAction == ogun::HealAction::RemoveDegenerateFace);
    }
}

TEST_CASE("FlipLongestEdge heals coplanar caps", "[repair][flip]") {
    const double apexY = GENERATE(capApexForAngle(179.5), 0.0);
    CAPTURE(apexY);

    const ogun::Mesh mesh = makeCapMesh(apexY);
    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(mesh, surface));
    REQUIRE(ogun::SliverHealer::analyzeFace(surface, ogun::CgalFaceIndex(0)).type == ogun::SliverType::Cap);

    std::string status;
    REQUIRE(ogun::SliverHealer::applyHealAction(
        surface, ogun::CgalFaceIndex(0), ogun::HealAction::FlipLongestEdge, {}, &status));
    CHECK_FALSE(status.empty());

    const ogun::Mesh healed = ogun::fromCgalSurfaceMesh(surface);
    CHECK(healed.numVertices() == mesh.numVertices());
    CHECK(healed.numFaces() == mesh.numFaces());
    CHECK(allNormalsAlign(healed, Eigen::Vector3d::UnitZ(), 0.999));
    CHECK(ogun::ThinFaceChecker::check(healed).empty());
}

TEST_CASE("FlipLongestEdge rejects creases and non-convex quads", "[repair][flip]") {
    const double apexY = capApexForAngle(179.5);

    SECTION("neighbor across a 90 deg crease") {
        ogun::CgalSurfaceMesh surface;
        REQUIRE(ogun::toCgalSurfaceMesh(makeCapMesh(apexY, {0.0, 0.0, -1.0}), surface));
        const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(surface, ogun::CgalFaceIndex(0));
        CHECK(a.type == ogun::SliverType::Cap);
        CHECK_FALSE(a.longestEdgeFlippable);
        CHECK(a.recommendedAction == ogun::HealAction::DissolveCollinearVertex);

        std::string status;
        CHECK_FALSE(ogun::SliverHealer::applyHealAction(
            surface, ogun::CgalFaceIndex(0), ogun::HealAction::FlipLongestEdge, {}, &status));
        CHECK(status.find("crease") != std::string::npos);
        CHECK(surface.number_of_faces() == 4);
    }

    SECTION("neighbor forms a non-convex quad") {
        ogun::CgalSurfaceMesh surface;
        REQUIRE(ogun::toCgalSurfaceMesh(makeCapMesh(apexY, {-2.0, -0.001, 0.0}), surface));
        std::string status;
        CHECK_FALSE(ogun::SliverHealer::applyHealAction(
            surface, ogun::CgalFaceIndex(0), ogun::HealAction::FlipLongestEdge, {}, &status));
        CHECK(status.find("non-convex") != std::string::npos);
    }

    SECTION("neighbor flip would still leave a thin / sliver face") {
        ogun::CgalSurfaceMesh surface;
        // With opposite very close to the edge (y = -1e-4), post-flip triangles are slivers (aspect > 100)
        REQUIRE(ogun::toCgalSurfaceMesh(makeCapMesh(apexY, {0.0, -1.0e-4, 0.0}), surface));
        const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(surface, ogun::CgalFaceIndex(0));
        CHECK(a.type == ogun::SliverType::Cap);
        CHECK_FALSE(a.longestEdgeFlippable);
        CHECK(a.recommendedAction == ogun::HealAction::DissolveCollinearVertex);

        std::string status;
        CHECK_FALSE(ogun::SliverHealer::applyHealAction(
            surface, ogun::CgalFaceIndex(0), ogun::HealAction::FlipLongestEdge, {}, &status));
        CHECK(status.find("still leave a thin") != std::string::npos);
    }
}

TEST_CASE("CollapseShortestEdge dissolves needles and respects the link condition", "[repair][collapse]") {
    SECTION("needle collapse removes one vertex and two faces without inverting neighbors") {
        const ogun::Mesh mesh = makeNeedleMesh();
        const ogun::RepairResults r =
            ogun::SliverHealer::healSingleFace(mesh, 0, ogun::HealAction::CollapseShortestEdge);
        REQUIRE(r.success);
        CHECK(r.edgeCollapses == 1);
        CHECK(r.mesh.numVertices() == mesh.numVertices() - 1);
        CHECK(r.mesh.numFaces() == mesh.numFaces() - 2);
        CHECK(allNormalsAlign(r.mesh, Eigen::Vector3d::UnitZ(), 0.999));
        CHECK(r.initialThinFaces == 2);
        CHECK(r.remainingThinFaces == 0);
    }

    SECTION("collapse on a minimal tetrahedron violates the link condition") {
        const ogun::Mesh tet = makeTetrahedron();
        ogun::CgalSurfaceMesh surface;
        REQUIRE(ogun::toCgalSurfaceMesh(tet, surface));
        std::string status;
        CHECK_FALSE(ogun::SliverHealer::applyHealAction(
            surface, ogun::CgalFaceIndex(0), ogun::HealAction::CollapseShortestEdge, {}, &status));
        CHECK(status.find("link condition") != std::string::npos);
        CHECK(surface.number_of_vertices() == 4);
        CHECK(surface.number_of_faces() == 4);

        const ogun::RepairResults r =
            ogun::SliverHealer::healSingleFace(tet, 0, ogun::HealAction::CollapseShortestEdge);
        CHECK_FALSE(r.success);
        CHECK(r.mesh.V == tet.V);
        CHECK(r.mesh.F == tet.F);
    }
}

TEST_CASE("SnapCollinearCollapse snaps a creased cap onto its corner vertex", "[repair][snap]") {
    // Cap across a 90 deg crease: flipping is rejected, so the apex is collapsed onto a fixed corner.
    const ogun::Mesh mesh = makeCapMesh(capApexForAngle(179.5), {0.0, 0.0, -1.0});
    const ogun::RepairResults r =
        ogun::SliverHealer::healSingleFace(mesh, 0, ogun::HealAction::SnapCollinearCollapse);
    REQUIRE(r.success);
    CHECK(r.mesh.numVertices() == mesh.numVertices() - 1);
    CHECK(r.mesh.numFaces() == mesh.numFaces() - 2);
    // Corner vertices s and t are kept exactly in place.
    const auto hasVertex = [&](const Eigen::RowVector3d& p) {
        for (Eigen::Index i = 0; i < r.mesh.numVertices(); ++i) {
            if (r.mesh.V.row(i) == p) {
                return true;
            }
        }
        return false;
    };
    CHECK(hasVertex(mesh.V.row(0)));
    CHECK(hasVertex(mesh.V.row(1)));
}

TEST_CASE("DissolveCollinearVertex dissolves apex and retriangulates coplanar patch", "[repair][dissolve]") {
    const double apexY = capApexForAngle(179.5);
    // Neighbor face across longest edge is at a 90 deg crease (z = -1), so FlipLongestEdge is rejected.
    // Incident faces around the apex are all coplanar in the XY plane -> DissolveCollinearVertex recommended.
    const ogun::Mesh mesh = makeCapMesh(apexY, {0.0, 0.0, -1.0});
    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(mesh, surface));
    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(surface, ogun::CgalFaceIndex(0));
    CHECK(a.type == ogun::SliverType::Cap);
    CHECK_FALSE(a.longestEdgeFlippable);
    CHECK(a.recommendedAction == ogun::HealAction::DissolveCollinearVertex);

    const ogun::RepairResults r =
        ogun::SliverHealer::healSingleFace(mesh, 0, ogun::HealAction::DissolveCollinearVertex);
    REQUIRE(r.success);
    CHECK(r.facesRemoved >= 1);
    CHECK(r.mesh.numVertices() == mesh.numVertices() - 1);
    CHECK(ogun::ThinFaceChecker::check(r.mesh).empty());
}

TEST_CASE("SplitNeighborEdge splits neighbor face without moving vertex on a crease", "[repair][split]") {
    const double apexY = capApexForAngle(179.5);
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << -1.0, 0.0, 1.0,     // 0: s
               1.0, 0.0, 1.0,     // 1: t
               0.0, apexY, 1.0,   // 2: apex
               0.0, -1.0, 0.0,    // 3: o (creased neighbor face across s-t)
              -1.0, apexY, 0.0,   // 4: wall bottom left
               1.0, apexY, 0.0;   // 5: wall bottom right
    mesh.F.resize(5, 3);
    mesh.F << 0, 1, 2,  // cap
              1, 0, 3,  // neighbor across s-t
              0, 2, 4,  // wall
              2, 1, 5,  // wall
              2, 5, 4;  // wall
    mesh.recomputeNormals();

    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(mesh, surface));
    auto pins = surface.add_property_map<ogun::CgalVertexIndex, std::uint8_t>(
        ogun::SliverHealer::kPinnedVertexProperty, 0).first;
    pins[ogun::CgalVertexIndex(2)] = 1; // Pin apex (must stay fixed in place)

    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(surface, ogun::CgalFaceIndex(0));
    CHECK(a.type == ogun::SliverType::Cap);
    CHECK_FALSE(a.longestEdgeFlippable);
    // Apex is pinned and on a crease: cannot dissolve or collapse, so SplitNeighborEdge is recommended.
    CHECK(a.recommendedAction == ogun::HealAction::SplitNeighborEdge);

    std::string status;
    CHECK(ogun::SliverHealer::applyHealAction(
        surface, ogun::CgalFaceIndex(0), ogun::HealAction::SplitNeighborEdge, {}, &status));
    CHECK(status.find("Split the neighbor face") != std::string::npos);

    // Verify all original vertex coordinates are exactly preserved
    for (Eigen::Index i = 0; i < mesh.numVertices(); ++i) {
        bool found = false;
        for (const auto v : surface.vertices()) {
            const auto p = surface.point(v);
            if ((Eigen::Vector3d(p.x(), p.y(), p.z()) - mesh.V.row(i).transpose()).norm() < 1e-12) {
                found = true;
                break;
            }
        }
        CHECK(found);
    }
}

TEST_CASE("RemoveDegenerateFace removes only dangling faces", "[repair][remove]") {
    const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
    const ogun::RepairResults removed = ogun::SliverHealer::healSingleFace(
        sample, ogun::SampleMeshes::kNeedleSliverFaceIndex, ogun::HealAction::RemoveDegenerateFace);
    REQUIRE(removed.success);
    CHECK(removed.mesh.numFaces() == sample.numFaces() - 1);
    CHECK(removed.remainingThinFaces == removed.initialThinFaces - 1);

    const ogun::RepairResults rejected =
        ogun::SliverHealer::healSingleFace(makeNeedleMesh(), 0, ogun::HealAction::RemoveDegenerateFace);
    CHECK_FALSE(rejected.success);
    CHECK(rejected.statusMessage.find("hole") != std::string::npos);
}

TEST_CASE("previewSingleFaceRepair heals a 2-ring patch without touching the base mesh", "[repair][preview]") {
    const ogun::Mesh mesh = makeNeedleMesh();
    const ogun::Mesh before = mesh;

    const ogun::SingleFaceRepairPreview preview = ogun::SliverHealer::previewSingleFaceRepair(mesh, 0);
    REQUIRE(preview.valid);
    CHECK(preview.analysis.type == ogun::SliverType::Needle);
    CHECK(preview.appliedAction == ogun::HealAction::CollapseShortestEdge);
    CHECK_FALSE(preview.statusMessage.empty());

    CHECK(mesh.V == before.V);
    CHECK(mesh.F == before.F);

    // The whole disk is within 2 rings, so the healed patch has the needle pair removed.
    const ogun::Mesh& patch = preview.previewPatchMesh;
    REQUIRE_FALSE(patch.empty());
    CHECK(patch.numFaces() == mesh.numFaces() - 2);
    CHECK(patch.numVertices() == mesh.numVertices() - 1);

    // Every base face lies within 2 rings, so the patch replaces all of them.
    std::vector<Eigen::Index> replaced = preview.sourceFaceIndices;
    std::sort(replaced.begin(), replaced.end());
    CHECK(replaced == std::vector<Eigen::Index>{0, 1, 2, 3, 4, 5});

    // Planar patch is lifted along +Z by previewOffsetFactor * bounding diagonal.
    const double expectedOffset = ogun::SliverHealerSettings{}.previewOffsetFactor * std::sqrt(8.0);
    for (Eigen::Index i = 0; i < patch.numVertices(); ++i) {
        CHECK(patch.V(i, 2) == Catch::Approx(expectedOffset).epsilon(1e-6));
    }

    SECTION("an unsafe strategy yields an invalid preview with a reason") {
        const ogun::SingleFaceRepairPreview bad = ogun::SliverHealer::previewSingleFaceRepair(
            mesh, 0, ogun::HealAction::RemoveDegenerateFace);
        CHECK_FALSE(bad.valid);
        CHECK(bad.appliedAction == ogun::HealAction::RemoveDegenerateFace);
        CHECK_FALSE(bad.statusMessage.empty());
        CHECK(bad.previewPatchMesh.empty());
        CHECK(bad.sourceFaceIndices.empty());
    }

    SECTION("a healthy face needs no preview") {
        const ogun::SingleFaceRepairPreview healthy =
            ogun::SliverHealer::previewSingleFaceRepair(ogun::SampleMeshes::createCubeMesh(), 0);
        CHECK_FALSE(healthy.valid);
        CHECK(healthy.appliedAction == ogun::HealAction::None);
    }
}

TEST_CASE("healAllThinFaces heals every thin face", "[repair][batch]") {
    SECTION("sample defective mesh") {
        const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
        // Zero-area and needle; the well-shaped microscopic face is not thin at 100 : 1
        REQUIRE(ogun::ThinFaceChecker::check(sample).numDefects() == 2);

        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(sample);
        CHECK(r.success);
        CHECK(r.initialThinFaces == 2);
        CHECK(r.remainingThinFaces == 0);
        CHECK(ogun::ThinFaceChecker::check(r.mesh).empty());
        CHECK(r.mesh.fileUnit == sample.fileUnit);
    }

    SECTION("subdivided manifold cube with injected Cap, Needle and Microscopic slivers") {
        // The 1e-5-scale triangle is Microscopic only under an advanced height floor
        const ogun::ThinFaceThresholds heightFloor{1.0e-4, 0.0, ogun::settings::kDefaultThinFaceMaxAspectRatio};
        ogun::Mesh cube = makeSubdividedCube(4, 10.0);
        REQUIRE(isClosedManifold(cube));
        REQUIRE(ogun::ThinFaceChecker::check(cube, heightFloor).empty());
        const ogun::BoundingBox box = cube.boundingBox();

        // Cap: vertex 1e-5 off the midpoint of an interior grid edge on the top face.
        {
            const Eigen::Index f = nearestFace(cube, {4.17, 3.33, 10.0});
            const Eigen::Vector3d a = vertexOf(cube, f, 0);
            const Eigen::Vector3d b = vertexOf(cube, f, 1);
            const Eigen::Vector3d c = vertexOf(cube, f, 2);
            const Eigen::Vector3d mid = 0.5 * (a + b);
            splitFace(cube, f, mid + 1e-5 * (c - mid).normalized());
        }
        // Needle: vertex 1e-5 from an interior grid vertex on the +X face.
        {
            const Eigen::Index f = nearestFace(cube, {10.0, 4.17, 3.33});
            const Eigen::Vector3d c = vertexOf(cube, f, 2);
            const Eigen::Vector3d centroid = (vertexOf(cube, f, 0) + vertexOf(cube, f, 1) + c) / 3.0;
            splitFace(cube, f, c + 1e-5 * (centroid - c).normalized());
        }
        // Microscopic: a second split inside the resulting needle forms a 1e-5-scale triangle on the +Y face.
        {
            const Eigen::Index f = nearestFace(cube, {3.33, 10.0, 4.17});
            const Eigen::Vector3d b = vertexOf(cube, f, 1);
            const Eigen::Vector3d c = vertexOf(cube, f, 2);
            const Eigen::Vector3d centroid = (vertexOf(cube, f, 0) + b + c) / 3.0;
            const Eigen::Vector3d m1 = c + 1e-5 * (centroid - c).normalized();
            splitFace(cube, f, m1);
            const Eigen::Index needle = cube.numFaces() - 2; // (b, c, m1)
            splitFace(cube, needle, 0.5 * (c + m1) + 0.5e-5 * (b - c).normalized());
        }
        REQUIRE(isClosedManifold(cube));

        const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(cube, heightFloor);
        REQUIRE(before.numDefects() >= 3);
        bool sawCap = false;
        bool sawNeedle = false;
        bool sawMicroscopic = false;
        for (const Eigen::Index f : before.faceIndices) {
            const ogun::SliverType type = ogun::SliverHealer::analyzeFace(cube, f, heightFloor).type;
            sawCap = sawCap || type == ogun::SliverType::Cap;
            sawNeedle = sawNeedle || type == ogun::SliverType::Needle;
            sawMicroscopic = sawMicroscopic || type == ogun::SliverType::Microscopic;
        }
        CHECK(sawCap);
        CHECK(sawNeedle);
        CHECK(sawMicroscopic);

        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(cube, heightFloor);
        CHECK(r.success);
        CHECK(r.remainingThinFaces == 0);
        CHECK(ogun::ThinFaceChecker::check(r.mesh, heightFloor).empty());
        CHECK(r.edgeFlips >= 1);
        CHECK(r.edgeCollapses >= 2);
        CHECK(isClosedManifold(r.mesh));

        const ogun::BoundingBox after = r.mesh.boundingBox();
        CHECK((after.min - box.min).norm() < 1e-9);
        CHECK((after.max - box.max).norm() < 1e-9);
    }
}

TEST_CASE("healFaces repairs only the listed faces with their recommended actions", "[repair][selected]") {
    ogun::Mesh cube = makeSubdividedCube(4, 10.0);
    {
        const Eigen::Index f = nearestFace(cube, {4.17, 3.33, 10.0});
        const Eigen::Vector3d a = vertexOf(cube, f, 0);
        const Eigen::Vector3d b = vertexOf(cube, f, 1);
        const Eigen::Vector3d c = vertexOf(cube, f, 2);
        const Eigen::Vector3d mid = 0.5 * (a + b);
        splitFace(cube, f, mid + 1e-5 * (c - mid).normalized());
    }
    {
        const Eigen::Index f = nearestFace(cube, {10.0, 4.17, 3.33});
        const Eigen::Vector3d c = vertexOf(cube, f, 2);
        const Eigen::Vector3d centroid = (vertexOf(cube, f, 0) + vertexOf(cube, f, 1) + c) / 3.0;
        splitFace(cube, f, c + 1e-5 * (centroid - c).normalized());
    }
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(cube);
    REQUIRE(before.numDefects() >= 2);

    const ogun::RepairResults one = ogun::SliverHealer::healFaces(cube, {before.faceIndices.front()});
    CHECK(one.success);
    CHECK(one.remainingThinFaces < before.numDefects());
    CHECK(one.remainingThinFaces > 0);

    const ogun::RepairResults all = ogun::SliverHealer::healFaces(cube, before.faceIndices);
    CHECK(all.success);
    CHECK(all.remainingThinFaces < one.remainingThinFaces);

    const ogun::RepairResults none = ogun::SliverHealer::healFaces(cube, {});
    CHECK_FALSE(none.success);
    CHECK(none.mesh.numFaces() == cube.numFaces());
}

TEST_CASE("The aspect-110 Small face is collapsed along its shortest edge", "[repair][thin-faces]") {
    // Planar patch around the Small face (p, q, r): aspect 110, short edge p-r 5.08 %, largest angle 169.2 deg
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << 0.0,   0.0,          0.0,  // p
              1.0,   0.0,          0.0,  // q
              0.05,  1.0 / 110.0,  0.0,  // r
              0.5,  -1.0,          0.0,  // below p-q
              0.5,   1.0,          0.0,  // above q-r
             -0.5,   1.0,          0.0;  // above r-p
    mesh.F.resize(5, 3);
    mesh.F << 0, 1, 2,
              1, 0, 3,
              2, 1, 4,
              0, 2, 5,
              2, 4, 5;
    mesh.recomputeNormals();

    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(mesh);
    REQUIRE(before.faceIndices == std::vector<Eigen::Index>{0});
    REQUIRE(ogun::classifyThinDefect(before.defects[0]) == ogun::ThinFaceSubtype::Small);

    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(mesh, 0);
    CHECK(a.type == ogun::SliverType::Microscopic);
    CHECK(a.recommendedAction == ogun::HealAction::CollapseShortestEdge);

    // The Delaunay flip pass would heal this planar patch without a collapse (see the flip-first test below);
    // switch it off to keep exercising the collapse of the shortest edge.
    ogun::SliverHealerSettings noFlips;
    noFlips.delaunayFlipFirst = false;
    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(mesh, {}, noFlips);
    REQUIRE(r.success);
    CHECK(r.initialThinFaces == 1);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.edgeCollapses == 1);
    // p and r merged: the Small face and its neighbour across p-r are gone
    CHECK(r.mesh.numFaces() == mesh.numFaces() - 2);
    CHECK(r.mesh.numVertices() == mesh.numVertices() - 1);
    CHECK(allNormalsAlign(r.mesh, Eigen::Vector3d::UnitZ(), 0.99));
}

namespace {

/// A closed subdivided cube with one needle: a vertex 1e-5 from grid vertex `c` on the +X face.
struct NeedleCube {
    ogun::Mesh mesh;
    int c = -1; ///< The grid vertex
    int m = -1; ///< The vertex next to it
};

NeedleCube makeNeedleCube() {
    NeedleCube out;
    out.mesh = makeSubdividedCube(4, 10.0);
    const Eigen::Index f = nearestFace(out.mesh, {10.0, 4.17, 3.33});
    out.c = out.mesh.F(f, 2);
    const Eigen::Vector3d c = vertexOf(out.mesh, f, 2);
    const Eigen::Vector3d centroid = (vertexOf(out.mesh, f, 0) + vertexOf(out.mesh, f, 1) + c) / 3.0;
    out.m = splitFace(out.mesh, f, c + 1e-5 * (centroid - c).normalized());
    return out;
}

} // namespace

TEST_CASE("The batch sliver heals stop when asked and return the input", "[repair][batch][cancel]") {
    const NeedleCube cube = makeNeedleCube();
    std::stop_source stop;
    stop.request_stop();
    ogun::SliverHealOptions options;
    options.stopToken = stop.get_token();

    const ogun::RepairResults all = ogun::SliverHealer::healAllThinFaces(cube.mesh, {}, {}, options);
    CHECK_FALSE(all.success);
    CHECK(all.cancelled);
    CHECK(all.statusMessage == "Cancelled.");
    CHECK(all.mesh.V == cube.mesh.V);
    CHECK(all.mesh.F == cube.mesh.F);
    CHECK(all.faceMap.size() == static_cast<std::size_t>(cube.mesh.numFaces()));
    CHECK(all.faceMap.back() == cube.mesh.numFaces() - 1);

    const ogun::RepairResults some = ogun::SliverHealer::healFaces(cube.mesh, ogun::ThinFaceChecker::check(cube.mesh).faceIndices, {}, {}, options);
    CHECK(some.cancelled);
    CHECK(some.mesh.F == cube.mesh.F);

    // Not stopped: healed as before
    const ogun::RepairResults healed = ogun::SliverHealer::healAllThinFaces(cube.mesh);
    CHECK_FALSE(healed.cancelled);
    CHECK(healed.remainingThinFaces == 0);
}

TEST_CASE("Pinned vertices never move in the sliver healer", "[repair][batch][pinned]") {
    const NeedleCube cube = makeNeedleCube();
    const ogun::Mesh& mesh = cube.mesh;
    REQUIRE(ogun::ThinFaceChecker::check(mesh).numDefects() >= 2);
    std::vector<std::uint8_t> pinned(static_cast<std::size_t>(mesh.numVertices()), 0);
    ogun::SliverHealOptions options;

    const auto requirePinsKept = [&](const ogun::RepairResults& r) {
        REQUIRE(r.vertexMap.size() == static_cast<std::size_t>(mesh.numVertices()));
        for (std::size_t v = 0; v < pinned.size(); ++v) {
            if (pinned[v] != 0) {
                REQUIRE(r.vertexMap[v] >= 0);
                REQUIRE(r.mesh.V.row(r.vertexMap[v]) == mesh.V.row(static_cast<Eigen::Index>(v)));
            }
        }
    };

    SECTION("The grid vertex pinned: the needle collapses onto it") {
        // Without pins the collapse goes to the midpoint (the grid vertex moves by 5e-6)
        const ogun::RepairResults free = ogun::SliverHealer::healAllThinFaces(mesh);
        REQUIRE(free.remainingThinFaces == 0);
        const Eigen::Index cAfter = free.vertexMap[static_cast<std::size_t>(cube.c)];
        const Eigen::Index merged = cAfter >= 0 ? cAfter : free.vertexMap[static_cast<std::size_t>(cube.m)];
        REQUIRE(merged >= 0);
        REQUIRE(free.mesh.V.row(merged) != mesh.V.row(cube.c));

        pinned[static_cast<std::size_t>(cube.c)] = 1;
        options.pinnedVertices = pinned;
        for (const ogun::RepairResults& r :
             {ogun::SliverHealer::healAllThinFaces(mesh, {}, {}, options),
              ogun::SliverHealer::healFaces(mesh, ogun::ThinFaceChecker::check(mesh).faceIndices, {}, {}, options)}) {
            REQUIRE(r.success);
            REQUIRE(r.edgeCollapses >= 1);
            REQUIRE(r.remainingThinFaces == 0);
            REQUIRE(r.vertexMap[static_cast<std::size_t>(cube.m)] == -1); // merged into the pinned vertex
            requirePinsKept(r);
            REQUIRE(isClosedManifold(r.mesh));
        }
    }

    SECTION("Both ends of the short edge pinned: nothing moves") {
        pinned[static_cast<std::size_t>(cube.c)] = 1;
        pinned[static_cast<std::size_t>(cube.m)] = 1;
        options.pinnedVertices = pinned;
        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(mesh, {}, {}, options);
        requirePinsKept(r);
    }

    SECTION("Every vertex pinned: the surface is unchanged") {
        std::fill(pinned.begin(), pinned.end(), std::uint8_t{1});
        options.pinnedVertices = pinned;
        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(mesh, {}, {}, options);
        REQUIRE(r.edgeCollapses == 0);
        REQUIRE(r.edgeFlips == 0);
        requirePinsKept(r);
        REQUIRE(r.mesh.numFaces() == mesh.numFaces());
    }

    SECTION("A pin mask of the wrong size is refused") {
        const std::vector<std::uint8_t> wrong(3, 1);
        options.pinnedVertices = wrong;
        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(mesh, {}, {}, options);
        REQUIRE_FALSE(r.success);
        REQUIRE(r.mesh.F == mesh.F);
    }
}

TEST_CASE("The batch sliver heals return face and vertex maps in input order", "[repair][batch][maps]") {
    const NeedleCube cube = makeNeedleCube();
    // Isolate the collapse: the Delaunay flip pass may re-corner faces next to the needle (they keep their map entry).
    ogun::SliverHealerSettings noFlips;
    noFlips.delaunayFlipFirst = false;
    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(cube.mesh, {}, noFlips);
    REQUIRE(r.success);
    REQUIRE(r.faceMap.size() == static_cast<std::size_t>(cube.mesh.numFaces()));
    Eigen::Index next = 0;
    for (const Eigen::Index g : r.faceMap) {
        if (g >= 0) {
            REQUIRE(g == next++); // kept faces stay in input order
        }
    }
    REQUIRE(next == r.mesh.numFaces()); // the collapse added no face
    REQUIRE(next == cube.mesh.numFaces() - 2);
    // A face whose corners did not move maps to the same corners
    for (Eigen::Index f = 0; f < cube.mesh.numFaces(); ++f) {
        const Eigen::Index g = r.faceMap[static_cast<std::size_t>(f)];
        bool untouched = g >= 0;
        for (int k = 0; k < 3 && untouched; ++k) {
            untouched = cube.mesh.F(f, k) != cube.c && cube.mesh.F(f, k) != cube.m;
        }
        if (untouched) {
            for (int k = 0; k < 3; ++k) {
                REQUIRE(r.mesh.F(g, k) == r.vertexMap[static_cast<std::size_t>(cube.mesh.F(f, k))]);
            }
        }
    }
}

TEST_CASE("SliverHealer heals micro-cluster needles on Turkey_left_with_needles", "[repair][needle][cluster]") {
    const std::filesystem::path needleStl =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "build" / "examples" / "Turkey_left_with_needles.stl";
    if (!std::filesystem::exists(needleStl)) {
        return;
    }
    const ogun::Mesh mesh = ogun::loadStl(needleStl);
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(mesh, {});
    REQUIRE(before.numDefects() == 8);

    const ogun::RepairResults rAll = ogun::SliverHealer::healAllThinFaces(mesh);
    REQUIRE(rAll.success);
    CHECK(rAll.remainingThinFaces == 0);
    CHECK(rAll.edgeCollapses > 0);

    const ogun::RepairResults rScoped = ogun::SliverHealer::healFaces(mesh, before.faceIndices);
    REQUIRE(rScoped.success);
    CHECK(rScoped.remainingThinFaces <= 1);
}

TEST_CASE("SliverHealer heals curved-surface cap on turkey_single_cap", "[repair][cap][flip]") {
    const std::filesystem::path capStl =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "build" / "examples" / "turkey_single_cap.stl";
    if (!std::filesystem::exists(capStl)) {
        return;
    }
    const ogun::Mesh mesh = ogun::loadStl(capStl);
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(mesh, {});
    REQUIRE(before.numDefects() == 1);

    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(mesh, before.faceIndices[0]);
    CHECK(a.type == ogun::SliverType::Cap);
    CHECK(a.longestEdgeFlippable);
    CHECK(a.recommendedAction == ogun::HealAction::FlipLongestEdge);

    const ogun::RepairResults rAll = ogun::SliverHealer::healAllThinFaces(mesh);
    REQUIRE(rAll.success);
    CHECK(rAll.remainingThinFaces == 0);
    CHECK(rAll.edgeFlips == 1);

    const ogun::RepairResults rScoped = ogun::SliverHealer::healFaces(mesh, before.faceIndices);
    REQUIRE(rScoped.success);
    CHECK(rScoped.remainingThinFaces == 0);
    CHECK(rScoped.edgeFlips == 1);
}

TEST_CASE("SliverHealer heals all thin faces on cuboctahedron_faces.stl via link-condition fallback",
          "[repair][sliver][cuboctahedron]") {
    const std::filesystem::path stlPath =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "cuboctahedron_faces.stl";
    if (!std::filesystem::exists(stlPath)) {
        return;
    }
    const ogun::Mesh mesh = ogun::loadStl(stlPath);
    REQUIRE(mesh.numFaces() == 112);

    // Initial mesh has 33 thin/sliver faces
    const ogun::ThinFaceReport thinReport = ogun::ThinFaceChecker::check(mesh, {});
    REQUIRE(thinReport.numDefects() == 33);

    // Single-face repair preview and execution on Face 33 (which violates CGAL's Euler link condition)
    const ogun::SingleFaceRepairPreview preview = ogun::SliverHealer::previewSingleFaceRepair(mesh, 33);
    REQUIRE(preview.valid);
    CHECK(preview.appliedAction == ogun::HealAction::CollapseShortestEdge);

    const ogun::RepairResults singleRes =
        ogun::SliverHealer::healSingleFace(mesh, 33, ogun::HealAction::CollapseShortestEdge);
    REQUIRE(singleRes.success);
    CHECK(singleRes.remainingThinFaces < thinReport.numDefects());

    // SliverHealer heals all 33 thin faces (CGAL passes heal 31, raw collapse fallback heals the remaining 2)
    const ogun::RepairResults results = ogun::SliverHealer::healAllThinFaces(mesh);
    REQUIRE(results.success);
    CHECK(results.remainingThinFaces == 0);
    CHECK(results.edgeCollapses == 25);

    const ogun::ThinFaceReport afterReport = ogun::ThinFaceChecker::check(results.mesh, {});
    REQUIRE(afterReport.numDefects() == 0);
}

TEST_CASE("The flip pass heals a fan of needles on a flat strip without moving a vertex", "[repair][batch][flip-first]") {
    const bool fold = GENERATE(false, true);
    CAPTURE(fold);
    const ogun::Mesh strip = makeFannedStrip(fold);
    const std::size_t thinBefore = ogun::ThinFaceChecker::check(strip).numDefects();
    REQUIRE(thinBefore >= 10);

    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(strip);
    REQUIRE(r.success);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.edgeFlips > 0);
    CHECK(r.edgeCollapses == 0);
    CHECK(r.facesRemoved == 0);
    // Same vertices, same positions, same face count: only the diagonals changed
    CHECK(r.mesh.V == strip.V);
    CHECK(r.mesh.numFaces() == strip.numFaces());
    CHECK(ogun::surfaceArea(r.mesh) == Catch::Approx(ogun::surfaceArea(strip)).epsilon(1e-12));
    CHECK(ogun::signedVolume(r.mesh) == Catch::Approx(ogun::signedVolume(strip)).margin(1e-9));
    CHECK(ogun::computeRepairGeometryMetrics(strip, r.mesh).maxDeviation < 1e-9);

    if (fold) {
        // The crease between the two halves (the rung at x = 20) is never flipped
        CHECK(hasEdgeBetween(r.mesh, {20.0, 0.0, 0.0}, {20.0, 1.0, 0.0}));
    }
}

TEST_CASE("The scoped heal flips the listed thin faces without moving a vertex", "[repair][selected][flip-first]") {
    const ogun::Mesh strip = makeFannedStrip(false);
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(strip);
    REQUIRE(before.numDefects() >= 10);

    const ogun::RepairResults r = ogun::SliverHealer::healFaces(strip, before.faceIndices);
    REQUIRE(r.success);
    CHECK(r.edgeFlips > 0);
    CHECK(r.edgeCollapses == 0);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.mesh.V == strip.V);
    CHECK(r.mesh.numFaces() == strip.numFaces());
    CHECK(ogun::surfaceArea(r.mesh) == Catch::Approx(ogun::surfaceArea(strip)).epsilon(1e-12));
}

// A flat quad cut into a cap (apex 0.002 off the long edge, aspect 500) and an ordinary triangle below it.
// Flipping the shared edge gives two well-shaped triangles; the ordinary one is not thin, so it is never listed.
ogun::Mesh makeCapOnQuad() {
    const std::vector<Eigen::Vector3d> P = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.002, 0.0}, {0.5, -0.5, 0.0}};
    return meshFromPoints(P, {{0, 1, 2}, {1, 0, 3}});
}

TEST_CASE("The scoped heal flips a thin face with a neighbour that is not thin", "[repair][selected][flip-first]") {
    // Fix All heals only the listed thin faces. The flip partner of a lone cap is a normal triangle, so a flip
    // limited to listed faces could never run (the last cap of the Turkey Fix All test was left that way).
    const ogun::Mesh quad = makeCapOnQuad();
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(quad);
    REQUIRE(before.faceIndices == std::vector<Eigen::Index>{0});

    const ogun::RepairResults r = ogun::SliverHealer::healFaces(quad, {0});
    REQUIRE(r.success);
    CHECK(r.edgeFlips == 1);
    CHECK(r.edgeCollapses == 0);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.mesh.V == quad.V); // no vertex moved
    CHECK(r.mesh.numFaces() == 2);
    CHECK(ogun::surfaceArea(r.mesh) == Catch::Approx(ogun::surfaceArea(quad)).epsilon(1e-12));
}

TEST_CASE("A needle whose long edge flips into two healthy triangles is flipped, not collapsed", "[repair][needle][flip]") {
    // Needle 0-1-2 (short edge 1-2, aspect over 100) and a neighbour across its long edge 0-2. Collapsing the
    // short edge moves a vertex and can leave another needle; the flip moves nothing.
    const std::vector<Eigen::Vector3d> P = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.3, 0.08, 0.0}, {6.0, 3.0, 0.0}};
    const ogun::Mesh mesh = meshFromPoints(P, {{0, 1, 2}, {0, 2, 3}});
    REQUIRE(ogun::ThinFaceChecker::check(mesh).faceIndices == std::vector<Eigen::Index>{0});

    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(mesh, 0);
    REQUIRE(a.type == ogun::SliverType::Needle);
    CHECK(a.recommendedAction == ogun::HealAction::FlipLongestEdge);

    ogun::SliverHealerSettings noFlipPass; // the action itself, not the Delaunay pass in front of it
    noFlipPass.delaunayFlipFirst = false;
    const ogun::RepairResults r = ogun::SliverHealer::healFaces(mesh, {0}, {}, noFlipPass);
    REQUIRE(r.success);
    CHECK(r.edgeFlips == 1);
    CHECK(r.edgeCollapses == 0);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.mesh.V == mesh.V);
}

TEST_CASE("The scoped heal also heals the thin faces its own flips and repairs leave", "[repair][selected][flip-first]") {
    // One listed needle in a fan of needles: each flip or collapse can pass the needle on to a neighbour that
    // was not listed. Those join the list, so the call does not stop with thin faces next to where it worked.
    const ogun::Mesh strip = makeFannedStrip(false);
    const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(strip);
    REQUIRE(before.numDefects() >= 10);

    const ogun::RepairResults r = ogun::SliverHealer::healFaces(strip, {before.faceIndices.front()});
    REQUIRE(r.success);
    CHECK(r.remainingThinFaces == 0);
}

TEST_CASE("The scoped heal never flips an edge on the rim of a pinned scope", "[repair][selected][flip-first][pinned]") {
    // Both ends of the shared edge pinned (Fix All pins the vertices a scope shares with the rest of the mesh)
    const ogun::Mesh quad = makeCapOnQuad();
    const std::vector<std::uint8_t> pinned = {1, 1, 0, 0};
    ogun::SliverHealOptions options;
    options.pinnedVertices = pinned;
    const ogun::RepairResults r = ogun::SliverHealer::healFaces(quad, {0}, {}, {}, options);
    CHECK(r.edgeFlips == 0);
    for (const int v : {0, 1}) { // still there, at the same place (a collapse may renumber the free vertices)
        CHECK(hasVertexAt(r.mesh, quad.V.row(v).transpose()));
    }
}

TEST_CASE("The flip pass ends on a fan of nearly cocircular points", "[repair][batch][flip-first]") {
    // 400 points on a circle, fanned from one of them: every edge is a near tie for the Delaunay test, the case
    // where rounding could flip two edges back and forth. A deterministic 1e-9 wobble breaks the exact ties.
    constexpr int kPoints = 400;
    std::vector<Eigen::Vector3d> P;
    for (int i = 0; i < kPoints; ++i) {
        const double a = 2.0 * ogun::kPi * i / kPoints;
        const double r = 10.0 * (1.0 + 1.0e-9 * std::sin(7.0 * i));
        P.emplace_back(r * std::cos(a), r * std::sin(a), 0.0);
    }
    std::vector<std::array<int, 3>> T;
    for (int i = 1; i + 1 < kPoints; ++i) {
        T.push_back({0, i, i + 1});
    }
    const ogun::Mesh fan = meshFromPoints(P, T);
    REQUIRE(ogun::ThinFaceChecker::check(fan).numDefects() > 100);

    ogun::SliverHealerSettings flipsOnly;
    flipsOnly.maxBatchIterations = 1;
    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(fan, {}, flipsOnly);
    // It returns: the flips end even though nearly every edge is a tie (collapses may follow; not checked here)
    CHECK(r.edgeFlips > 0);
}

TEST_CASE("Snap is refused at a corner vertex where walls meet", "[repair][snap][corner]") {
    const ogun::Mesh patch = makeCornerPatch(30);
    const Eigen::Vector3d corner(12.0, 0.0, 5.7);
    REQUIRE(hasVertexAt(patch, corner));

    const ogun::SliverAnalysis a = ogun::SliverHealer::analyzeFace(patch, kCornerCapFace);
    REQUIRE(a.type == ogun::SliverType::Cap);
    CHECK_FALSE(a.longestEdgeFlippable);

    // Even with the old, permissive normal limit the corner must not slide onto R: the step wall would tilt
    for (const double minNormalDot : {ogun::settings::kDefaultMinNormalDot, 0.1}) {
        CAPTURE(minNormalDot);
        ogun::SliverHealerSettings settings;
        settings.minNormalDot = minNormalDot;
        ogun::CgalSurfaceMesh surface;
        REQUIRE(ogun::toCgalSurfaceMesh(patch, surface));
        std::string status;
        CHECK_FALSE(ogun::SliverHealer::applyHealAction(
            surface, ogun::CgalFaceIndex(static_cast<std::uint32_t>(kCornerCapFace)), ogun::HealAction::SnapCollinearCollapse, settings, &status));
        CHECK_FALSE(status.empty());
        const ogun::RepairResults single =
            ogun::SliverHealer::healSingleFace(patch, kCornerCapFace, ogun::HealAction::SnapCollinearCollapse, {}, settings);
        CHECK_FALSE(single.success);
        CHECK(sameMesh(single.mesh, patch));
    }
}

TEST_CASE("healAllThinFaces leaves a CAD corner and its step wall in place", "[repair][batch][corner]") {
    // Measured on the previous healer (snap onto R, minNormalDot 0.1): signed volume -290 -> -1971, area +74 %,
    // surface moved by 11.8 mm. Now the flip pass and the guarded snap keep the part.
    const ogun::Mesh patch = makeCornerPatch();
    const std::size_t thinBefore = ogun::ThinFaceChecker::check(patch).numDefects();
    REQUIRE(thinBefore >= 100);
    REQUIRE(countIntersectingFaces(patch) == 0);

    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(patch);
    REQUIRE(r.success);
    CHECK(r.remainingThinFaces == 0);
    CHECK(r.edgeFlips > 0);

    const ogun::RepairMetrics m = ogun::computeRepairGeometryMetrics(patch, r.mesh);
    CHECK(m.maxDeviation < 0.01);
    CHECK(std::abs(m.volumeAfter - m.volumeBefore) < 0.01 * std::abs(m.volumeBefore));
    CHECK(std::abs(m.areaAfter - m.areaBefore) < 0.01 * m.areaBefore);
    CHECK(countIntersectingFaces(r.mesh) == 0);
    // The corner and the step wall's base survive exactly
    CHECK(hasVertexAt(r.mesh, {12.0, 0.0, 5.7}));
    CHECK(hasVertexAt(r.mesh, {12.0, 36.0, 0.0}));
    CHECK(hasVertexAt(r.mesh, {30.0, 0.0, 5.7}));
}

TEST_CASE("The safety net rejects a heal that moves the surface too far", "[repair][batch][safety]") {
    const ogun::Mesh plate = makeHolePlate();
    REQUIRE(ogun::ThinFaceChecker::check(plate).numDefects() >= 50);

    SECTION("with the default limit the rim collapses stay inside it") {
        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(plate);
        REQUIRE(r.success);
        CHECK(r.remainingThinFaces == 0);
        CHECK(ogun::computeRepairGeometryMetrics(plate, r.mesh).maxDeviation <= 0.01 * plate.boundingBox().diagonal());
    }

    SECTION("a tight deviation limit returns the input unchanged and says why") {
        ogun::SliverHealerSettings settings;
        settings.maxDeviationFraction = 1.0e-4;
        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(plate, {}, settings);
        CHECK_FALSE(r.success);
        CHECK(sameMesh(r.mesh, plate));
        CHECK(r.edgeCollapses == 0);
        CHECK(r.remainingThinFaces == r.initialThinFaces);
        CHECK(r.statusMessage.find("not applied") != std::string::npos);
        CHECK(r.statusMessage.find("move the surface by up to") != std::string::npos);
        CHECK(r.statusMessage.find("left unchanged") != std::string::npos);
        CHECK(r.statusMessage.find("mm") != std::string::npos);

        // Switched off, the same heal goes through (and is what the net caught)
        settings.enableSafetyNet = false;
        const ogun::RepairResults unchecked = ogun::SliverHealer::healAllThinFaces(plate, {}, settings);
        CHECK(unchecked.success);
        CHECK_FALSE(sameMesh(unchecked.mesh, plate));
    }

    SECTION("the scoped heal is guarded too") {
        ogun::SliverHealerSettings settings;
        settings.maxDeviationFraction = 1.0e-4;
        const ogun::ThinFaceReport thin = ogun::ThinFaceChecker::check(plate);
        const ogun::RepairResults r = ogun::SliverHealer::healFaces(plate, thin.faceIndices, {}, settings);
        CHECK_FALSE(r.success);
        CHECK(sameMesh(r.mesh, plate));
        CHECK(r.statusMessage.find("not applied") != std::string::npos);
    }
}

TEST_CASE("The safety net rejects a volume change on a closed mesh only", "[repair][batch][safety]") {
    // A sphere with a needle next to a vertex: collapsing it moves the curved surface a hair, so the volume changes
    ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(8, 12, 5.0);
    const Eigen::Index f = nearestFace(sphere, {2.0, 2.0, 4.0});
    const Eigen::Vector3d c = vertexOf(sphere, f, 2);
    const Eigen::Vector3d centroid = (vertexOf(sphere, f, 0) + vertexOf(sphere, f, 1) + c) / 3.0;
    splitFace(sphere, f, c + 1e-5 * (centroid - c).normalized());
    REQUIRE(isClosedManifold(sphere));
    REQUIRE(ogun::ThinFaceChecker::check(sphere).numDefects() >= 1);

    ogun::SliverHealerSettings strict;
    strict.maxVolumeChange = 1.0e-14;
    strict.maxDeviationFraction = 1.0; // only the volume can trip the net
    const ogun::RepairResults rejected = ogun::SliverHealer::healAllThinFaces(sphere, {}, strict);
    CHECK_FALSE(rejected.success);
    CHECK(sameMesh(rejected.mesh, sphere));
    CHECK(rejected.statusMessage.find("change the volume by") != std::string::npos);
    CHECK(rejected.statusMessage.find("left unchanged") != std::string::npos);

    // The default limit (0.5 %) accepts this harmless heal
    const ogun::RepairResults accepted = ogun::SliverHealer::healAllThinFaces(sphere);
    CHECK(accepted.success);
    CHECK(accepted.remainingThinFaces == 0);

    // An open mesh has no meaningful volume: the same tiny volume limit does not reject it
    ogun::Mesh open = makeCornerPatch(30);
    ogun::SliverHealerSettings volumeOnly;
    volumeOnly.maxVolumeChange = 1.0e-14;
    const ogun::RepairResults openResult = ogun::SliverHealer::healAllThinFaces(open, {}, volumeOnly);
    CHECK(openResult.success);
}
