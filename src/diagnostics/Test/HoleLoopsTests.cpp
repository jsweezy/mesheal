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
mesheal::Mesh removeFaces(const mesheal::Mesh& mesh, const std::function<bool(const Eigen::Vector3d&)>& drop) {
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        if (!drop(c)) {
            keep.push_back(f);
        }
    }
    return mesheal::extractFaces(mesh, keep);
}

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

} // namespace

TEST_CASE("A square hole in a flat face is one closed Flat loop", "[diagnostics][holes]") {
    const mesheal::Mesh holed = removeFaces(mesheal::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(holed);
    REQUIRE(holes.size() == 1);
    const mesheal::HoleLoop& hole = holes.front();
    REQUIRE(hole.closed);
    REQUIRE(hole.edgeCount() == 8);
    REQUIRE(hole.vertices.size() == 8);
    REQUIRE(hole.perimeter == Catch::Approx(8.0));
    REQUIRE(hole.area == Catch::Approx(4.0));
    REQUIRE(hole.width == Catch::Approx(1.0));
    REQUIRE(hole.planeDeviation == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(hole.normal.z() == Catch::Approx(1.0)); // outward: the side the fill must face
    REQUIRE(hole.flatNeighbourShare == Catch::Approx(1.0));
    REQUIRE(hole.shape == mesheal::HoleShape::Flat);
    // The rim is ordered: each edge starts where the previous one ends
    for (std::size_t i = 0; i < hole.edges.size(); ++i) {
        REQUIRE(hole.edges[i][1] == hole.edges[(i + 1) % hole.edges.size()][0]);
    }
    REQUIRE(mesheal::findHoleOfEdge(holes, hole.edges[3][1], hole.edges[3][0]) == 0);
    REQUIRE(mesheal::findHoleOfEdge(holes, 0, 1) == -1);
}

TEST_CASE("Hole shapes: cylinder cap, sphere cap, crack and fin", "[diagnostics][holes]") {
    // End cap: the walls meet it at a right angle
    const mesheal::Mesh cylinder = removeFaces(mesheal::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0),
                                            [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
    std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(cylinder);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().shape == mesheal::HoleShape::Flat);
    REQUIRE(holes.front().normal.z() == Catch::Approx(1.0));

    // Sphere cap: a planar rim, but the surface around it is tilted
    const mesheal::Mesh sphere = removeFaces(mesheal::SampleMeshes::createSphereMesh(24, 32, 5.0),
                                          [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
    holes = mesheal::findHoleLoops(sphere);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().planeDeviation < 1e-12);
    REQUIRE(holes.front().flatNeighbourShare == Catch::Approx(0.0));
    REQUIRE(holes.front().shape == mesheal::HoleShape::Curved);

    // T-junction crack: zero width
    const mesheal::Mesh cracked = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createCrackedBoxMesh());
    holes = mesheal::findHoleLoops(cracked);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().edgeCount() == 6);
    REQUIRE(holes.front().closed);
    REQUIRE(holes.front().width == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(holes.front().shape == mesheal::HoleShape::Crack);

    // A fin's two open edges end at a non-manifold edge: not a loop
    const mesheal::Mesh touching = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createTouchingCubesMesh());
    holes = mesheal::findHoleLoops(touching);
    REQUIRE(holes.size() == 1);
    REQUIRE_FALSE(holes.front().closed);
    REQUIRE(holes.front().shape == mesheal::HoleShape::OpenChain);
}

TEST_CASE("Each hole is one Open edges row with its shape as the sub-type", "[diagnostics][holes]") {
    const mesheal::Mesh sample = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createHolesSampleMesh());
    const mesheal::CategoryResult open = mesheal::checkOpenEdges(sample);
    REQUIRE(open.size() == 4);
    REQUIRE(open.infoCount == 6 + 8 + 24 + 24);
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(sample);
    for (std::size_t h = 0; h < open.size(); ++h) {
        REQUIRE(open.elements[h].index == static_cast<Eigen::Index>(h)); // row h is hole h
        REQUIRE(open.edgesOf(h).size() == holes[h].edgeCount());
        REQUIRE(mesheal::findHoleOfEdge(holes, open.elements[h].edge[0], open.elements[h].edge[1]) == static_cast<int>(h));
    }
    const std::array<std::size_t, 4> counts = open.subtypeCounts();
    REQUIRE(counts[static_cast<std::size_t>(mesheal::HoleShape::Crack)] == 1);
    REQUIRE(counts[static_cast<std::size_t>(mesheal::HoleShape::Flat)] == 2);
    REQUIRE(counts[static_cast<std::size_t>(mesheal::HoleShape::Curved)] == 1);
    REQUIRE(counts[static_cast<std::size_t>(mesheal::HoleShape::OpenChain)] == 0);
    REQUIRE(mesheal::checkNonManifoldEdges(sample).empty());
    REQUIRE(mesheal::checkShells(sample).infoCount == 4);
}

TEST_CASE("The bunny's five holes are curved loops", "[diagnostics][holes]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(bunny);
    REQUIRE(holes.size() == 5);
    std::size_t edges = 0;
    for (const mesheal::HoleLoop& hole : holes) {
        REQUIRE(hole.closed);
        REQUIRE(hole.shape == mesheal::HoleShape::Curved);
        edges += hole.edgeCount();
    }
    REQUIRE(edges == 39 + 22 + 42 + 80 + 40);
    const mesheal::CategoryResult open = mesheal::checkOpenEdges(bunny);
    REQUIRE(open.size() == 5);
    REQUIRE(open.infoCount == edges);
}
