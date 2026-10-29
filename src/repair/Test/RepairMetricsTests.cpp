#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/Orientation.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SliverHealer.hpp"
#include "repair/TopologyCleanup.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Repair metrics count degenerate, duplicate, flipped and non-manifold elements", "[repair][metrics]") {
    SECTION("The sample defective mesh before and after the sliver heal") {
        const ogun::Mesh sample = ogun::createSampleDefectiveMesh();
        const ogun::RepairResults healed = ogun::SliverHealer::healAllThinFaces(sample);
        REQUIRE(healed.success);
        const ogun::RepairMetrics m = ogun::computeRepairMetrics(sample, healed.mesh, ogun::CheckParams{});
        CHECK(m.degenerateFacesBefore == 1); // the zero-area flap on the bottom front edge
        CHECK(m.degenerateFacesAfter == 0);
        CHECK(m.thinFacesBefore == 2);
        CHECK(m.thinFacesAfter == 0);
        CHECK(m.duplicateFacesBefore == 0);
        CHECK(m.duplicateFacesAfter == 0);
        CHECK(m.flippedFacesBefore == ogun::checkFlippedNormals(sample).size());
        CHECK(m.flippedFacesAfter == ogun::checkFlippedNormals(healed.mesh).size());
        CHECK(m.nonManifoldEdgesBefore == ogun::checkNonManifoldEdges(sample).size());
        CHECK(m.nonManifoldVerticesBefore == ogun::checkNonManifoldVertices(sample).size());
        CHECK(m.nonManifoldVerticesAfter == ogun::checkNonManifoldVertices(healed.mesh).size());
        CHECK(m.invertedShellsBefore == 0);
        CHECK(m.shellsBefore == ogun::checkShells(sample).infoCount);
        CHECK(m.shellsAfter == ogun::checkShells(healed.mesh).infoCount);
    }

    SECTION("Doubled and collapsed faces, then removed") {
        const ogun::Mesh doubled = ogun::SampleMeshes::createDoubledFacesCubeMesh();
        const ogun::Mesh cleaned = ogun::removeDuplicateFaces(ogun::removeRepeatedVertexFaces(doubled).mesh).mesh;
        const ogun::RepairMetrics m = ogun::computeRepairMetrics(doubled, cleaned, ogun::CheckParams{});
        CHECK(m.degenerateFacesBefore == 2);
        CHECK(m.duplicateFacesBefore == 2);
        CHECK(m.degenerateFacesAfter == 1); // the flat cap stays until the healer flips it
        CHECK(m.duplicateFacesAfter == 0);
    }

    SECTION("An inside-out cube and a flipped face, before and after orientation") {
        ogun::Mesh mesh = ogun::SampleMeshes::combineMeshes(
            {ogun::SampleMeshes::createBox({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}),
             ogun::SampleMeshes::createBox({5.0, 0.0, 0.0}, {6.0, 1.0, 1.0}, true)});
        std::swap(mesh.F(0, 1), mesh.F(0, 2)); // one face of the outward box wound the other way
        mesh.recomputeNormals();
        const ogun::OrientShellsResult oriented = ogun::orientShells(mesh);
        const ogun::RepairMetrics m = ogun::computeRepairMetrics(mesh, oriented.mesh, ogun::CheckParams{});
        CHECK(m.flippedFacesBefore == 1);
        CHECK(m.flippedFacesAfter == 0);
        CHECK(m.invertedShellsBefore == 1);
        CHECK(m.invertedShellsAfter == 0);
        CHECK(m.shellsBefore == 2);
        CHECK(m.shellsAfter == 2);
    }
}
