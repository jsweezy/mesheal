#include "repair/SliverHealer.hpp"

#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "mesh/CgalMesh.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>

#include <CGAL/boost/graph/helpers.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

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
    return std::tan((180.0 - maxAngleDeg) * 0.5 * kPi / 180.0);
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
        const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
        const ogun::SliverAnalysis a =
            ogun::SliverHealer::analyzeFace(sample, ogun::SampleMeshes::kMicroscopicFaceIndex);
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
        CHECK(a.recommendedAction == ogun::HealAction::SnapCollinearCollapse);

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

TEST_CASE("RemoveDegenerateFace removes only dangling faces", "[repair][remove]") {
    const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
    const ogun::RepairResults removed = ogun::SliverHealer::healSingleFace(
        sample, ogun::SampleMeshes::kMicroscopicFaceIndex, ogun::HealAction::RemoveDegenerateFace);
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
        REQUIRE(ogun::ThinFaceChecker::check(sample).numDefects() == 3);

        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(sample);
        CHECK(r.success);
        CHECK(r.initialThinFaces == 3);
        CHECK(r.remainingThinFaces == 0);
        CHECK(ogun::ThinFaceChecker::check(r.mesh).empty());
        CHECK(r.mesh.fileUnit == sample.fileUnit);
    }

    SECTION("subdivided manifold cube with injected Cap, Needle and Microscopic slivers") {
        ogun::Mesh cube = makeSubdividedCube(4, 10.0);
        REQUIRE(isClosedManifold(cube));
        REQUIRE(ogun::ThinFaceChecker::check(cube).empty());
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

        const ogun::ThinFaceReport before = ogun::ThinFaceChecker::check(cube);
        REQUIRE(before.numDefects() >= 3);
        bool sawCap = false;
        bool sawNeedle = false;
        bool sawMicroscopic = false;
        for (const Eigen::Index f : before.faceIndices) {
            const ogun::SliverType type = ogun::SliverHealer::analyzeFace(cube, f).type;
            sawCap = sawCap || type == ogun::SliverType::Cap;
            sawNeedle = sawNeedle || type == ogun::SliverType::Needle;
            sawMicroscopic = sawMicroscopic || type == ogun::SliverType::Microscopic;
        }
        CHECK(sawCap);
        CHECK(sawNeedle);
        CHECK(sawMicroscopic);

        const ogun::RepairResults r = ogun::SliverHealer::healAllThinFaces(cube);
        CHECK(r.success);
        CHECK(r.remainingThinFaces == 0);
        CHECK(ogun::ThinFaceChecker::check(r.mesh).empty());
        CHECK(r.edgeFlips >= 1);
        CHECK(r.edgeCollapses >= 2);
        CHECK(isClosedManifold(r.mesh));

        const ogun::BoundingBox after = r.mesh.boundingBox();
        CHECK((after.min - box.min).norm() < 1e-9);
        CHECK((after.max - box.max).norm() < 1e-9);
    }
}
