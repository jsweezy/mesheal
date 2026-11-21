// The exact self-union (libigl's copyleft CGAL booleans) on closed shells that cross themselves.

#include "repair/ExactUnion.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <stop_token>

namespace {

using mesheal::ExactUnionStatus;
using mesheal::Mesh;
using mesheal::SampleMeshes;

constexpr std::size_t kNoLimit = 1000000;

bool closedAndManifold(const Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).elements.empty() && mesheal::checkNonManifoldEdges(mesh).elements.empty();
}

} // namespace

TEST_CASE("Exact self-union cuts a closed shell that passes through itself", "[repair][exact-union]") {
    const Mesh box = SampleMeshes::createSelfIntersectingBoxMesh();
    REQUIRE_FALSE(mesheal::checkSelfIntersections(box, mesheal::CheckParams{}).elements.empty());

    const mesheal::ExactUnionResult result = mesheal::exactSelfUnion(box, kNoLimit);
    REQUIRE(result.status == ExactUnionStatus::United);
    REQUIRE(result.mesh.numFaces() > 0);
    CHECK(closedAndManifold(result.mesh));
    CHECK(mesheal::checkSelfIntersections(result.mesh, mesheal::CheckParams{}).elements.empty());
    // The fold through the bottom only adds material outside the box, never removes it
    CHECK(mesheal::signedVolume(result.mesh) > 0.0);
    CHECK(result.mesh.fileUnit == box.fileUnit);
}

TEST_CASE("Exact self-union leaves a clean shell's surface where it was", "[repair][exact-union]") {
    const Mesh cube = SampleMeshes::createGridCubeMesh(4, 4.0);
    const mesheal::ExactUnionResult result = mesheal::exactSelfUnion(cube, kNoLimit);
    REQUIRE(result.status == ExactUnionStatus::United);
    CHECK(closedAndManifold(result.mesh));
    CHECK(mesheal::signedVolume(result.mesh) == Catch::Approx(mesheal::signedVolume(cube)));
}

TEST_CASE("Exact self-union refuses what it cannot or should not cut", "[repair][exact-union]") {
    SECTION("an open shell: inside is not defined") {
        Mesh open = SampleMeshes::createSelfIntersectingBoxMesh();
        open.F.conservativeResize(open.F.rows() - 1, 3);
        open.recomputeNormals();
        const mesheal::ExactUnionResult result = mesheal::exactSelfUnion(open, kNoLimit);
        CHECK(result.status == ExactUnionStatus::NotClosed);
        CHECK_FALSE(result.message.empty());
    }
    SECTION("more faces than the limit: the cut could not be stopped") {
        const Mesh box = SampleMeshes::createSelfIntersectingBoxMesh();
        const mesheal::ExactUnionResult result = mesheal::exactSelfUnion(box, 11);
        CHECK(result.status == ExactUnionStatus::TooLarge);
    }
    SECTION("a stop already requested") {
        std::stop_source stop;
        stop.request_stop();
        const mesheal::ExactUnionResult result =
            mesheal::exactSelfUnion(SampleMeshes::createSelfIntersectingBoxMesh(), kNoLimit, stop.get_token());
        CHECK(result.status == ExactUnionStatus::Cancelled);
    }
}

TEST_CASE("The pocket a union encloses is filled, never kept as a cavity", "[repair][exact-union]") {
    // Six slabs around an empty 2 x 2 x 2 core: their union is a box [0, 4]^3 with the core as an inward pocket
    const Mesh slabs = SampleMeshes::combineMeshes({
        SampleMeshes::createBox({0, 0, 0}, {1, 4, 4}), SampleMeshes::createBox({3, 0, 0}, {4, 4, 4}),
        SampleMeshes::createBox({0, 0, 0}, {4, 1, 4}), SampleMeshes::createBox({0, 3, 0}, {4, 4, 4}),
        SampleMeshes::createBox({0, 0, 0}, {4, 4, 1}), SampleMeshes::createBox({0, 0, 3}, {4, 4, 4})});
    mesheal::ExactUnionResult result = mesheal::exactSelfUnion(slabs, kNoLimit);
    REQUIRE(result.status == ExactUnionStatus::United);
    REQUIRE(mesheal::signedVolume(result.mesh) == Catch::Approx(64.0 - 8.0));

    double filled = 0.0;
    CHECK(mesheal::fillEnclosedPockets(result.mesh, &filled) == 1);
    CHECK(filled == Catch::Approx(8.0));
    CHECK(mesheal::signedVolume(result.mesh) == Catch::Approx(64.0));
    CHECK(closedAndManifold(result.mesh));
    CHECK(mesheal::fillEnclosedPockets(result.mesh) == 0); // nothing left to fill
}
