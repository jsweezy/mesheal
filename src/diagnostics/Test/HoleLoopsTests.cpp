#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <functional>
#include <vector>

namespace {

/// The faces of `mesh` whose centroid fails `drop`.
ogun::Mesh removeFaces(const ogun::Mesh& mesh, const std::function<bool(const Eigen::Vector3d&)>& drop) {
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        if (!drop(c)) {
            keep.push_back(f);
        }
    }
    return ogun::extractFaces(mesh, keep);
}

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

} // namespace

TEST_CASE("A square hole in a flat face is one closed Flat loop", "[diagnostics][holes]") {
    const ogun::Mesh holed = removeFaces(ogun::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(holed);
    REQUIRE(holes.size() == 1);
    const ogun::HoleLoop& hole = holes.front();
    REQUIRE(hole.closed);
    REQUIRE(hole.edgeCount() == 8);
    REQUIRE(hole.vertices.size() == 8);
    REQUIRE(hole.perimeter == Catch::Approx(8.0));
    REQUIRE(hole.area == Catch::Approx(4.0));
    REQUIRE(hole.width == Catch::Approx(1.0));
    REQUIRE(hole.planeDeviation == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(hole.normal.z() == Catch::Approx(1.0)); // outward: the side the fill must face
    REQUIRE(hole.flatNeighbourShare == Catch::Approx(1.0));
    REQUIRE(hole.shape == ogun::HoleShape::Flat);
    // The rim is ordered: each edge starts where the previous one ends
    for (std::size_t i = 0; i < hole.edges.size(); ++i) {
        REQUIRE(hole.edges[i][1] == hole.edges[(i + 1) % hole.edges.size()][0]);
    }
    REQUIRE(ogun::findHoleOfEdge(holes, hole.edges[3][1], hole.edges[3][0]) == 0);
    REQUIRE(ogun::findHoleOfEdge(holes, 0, 1) == -1);
}

TEST_CASE("Hole shapes: cylinder cap, sphere cap, crack and fin", "[diagnostics][holes]") {
    // End cap: the walls meet it at a right angle
    const ogun::Mesh cylinder = removeFaces(ogun::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0),
                                            [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
    std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(cylinder);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().shape == ogun::HoleShape::Flat);
    REQUIRE(holes.front().normal.z() == Catch::Approx(1.0));

    // Sphere cap: a planar rim, but the surface around it is tilted
    const ogun::Mesh sphere = removeFaces(ogun::SampleMeshes::createSphereMesh(24, 32, 5.0),
                                          [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
    holes = ogun::findHoleLoops(sphere);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().planeDeviation < 1e-12);
    REQUIRE(holes.front().flatNeighbourShare == Catch::Approx(0.0));
    REQUIRE(holes.front().shape == ogun::HoleShape::Curved);

    // T-junction crack: zero width
    const ogun::Mesh cracked = ogun::separateTouchingContacts(ogun::SampleMeshes::createCrackedBoxMesh());
    holes = ogun::findHoleLoops(cracked);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().edgeCount() == 6);
    REQUIRE(holes.front().closed);
    REQUIRE(holes.front().width == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(holes.front().shape == ogun::HoleShape::Crack);

    // A fin's two open edges end at a non-manifold edge: not a loop
    const ogun::Mesh touching = ogun::separateTouchingContacts(ogun::SampleMeshes::createTouchingCubesMesh());
    holes = ogun::findHoleLoops(touching);
    REQUIRE(holes.size() == 1);
    REQUIRE_FALSE(holes.front().closed);
    REQUIRE(holes.front().shape == ogun::HoleShape::OpenChain);
}

TEST_CASE("Open edges carry their hole's shape as the sub-type", "[diagnostics][holes]") {
    const ogun::Mesh sample = ogun::separateTouchingContacts(ogun::SampleMeshes::createHolesSampleMesh());
    const ogun::CategoryResult open = ogun::checkOpenEdges(sample);
    REQUIRE(open.infoCount == 4);
    const std::array<std::size_t, 4> counts = open.subtypeCounts();
    REQUIRE(counts[static_cast<std::size_t>(ogun::HoleShape::Crack)] == 6);
    REQUIRE(counts[static_cast<std::size_t>(ogun::HoleShape::Flat)] == 8 + 24);
    REQUIRE(counts[static_cast<std::size_t>(ogun::HoleShape::Curved)] == 24);
    REQUIRE(counts[static_cast<std::size_t>(ogun::HoleShape::OpenChain)] == 0);
    REQUIRE(ogun::checkNonManifoldEdges(sample).empty());
    REQUIRE(ogun::checkShells(sample).infoCount == 4);
}

TEST_CASE("The bunny's five holes are curved loops", "[diagnostics][holes]") {
    const ogun::Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(bunny);
    REQUIRE(holes.size() == 5);
    std::size_t edges = 0;
    for (const ogun::HoleLoop& hole : holes) {
        REQUIRE(hole.closed);
        REQUIRE(hole.shape == ogun::HoleShape::Curved);
        edges += hole.edgeCount();
    }
    REQUIRE(edges == 39 + 22 + 42 + 80 + 40);
    REQUIRE(ogun::checkOpenEdges(bunny).size() == edges);
}
