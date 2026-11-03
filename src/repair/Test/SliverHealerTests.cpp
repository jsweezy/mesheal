#include "repair/SliverHealer.hpp"

#include "core/Constants.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
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

    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(mesh);
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
    const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(cube.mesh);
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
