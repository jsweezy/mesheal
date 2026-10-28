#include "diagnostics/MeshChecks.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

namespace {

bool closedManifold(const ogun::Mesh& mesh) {
    return ogun::checkOpenEdges(mesh).empty() && ogun::checkNonManifoldEdges(mesh).empty();
}

/// The grid cube with one vertex of face 0 pulled to within `gap` of another corner of that face.
ogun::Mesh cubeWithShortEdge(double gap) {
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(4, 8.0);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    mesh.V.row(b) = mesh.V.row(a) + gap * (mesh.V.row(b) - mesh.V.row(a)).normalized();
    mesh.recomputeNormals();
    return mesh;
}

std::vector<std::uint8_t> facesAbove(const ogun::Mesh& mesh, double z) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const double cz = (mesh.V(mesh.F(f, 0), 2) + mesh.V(mesh.F(f, 1), 2) + mesh.V(mesh.F(f, 2), 2)) / 3.0;
        mask[static_cast<std::size_t>(f)] = cz > z ? 1 : 0;
    }
    return mask;
}

} // namespace

TEST_CASE("Merge by distance welds a split corner and closes the mesh", "[repair][cleanup][merge]") {
    // Face 0 uses a copy of its first vertex, 0.01 away: two open edges
    ogun::Mesh mesh = ogun::SampleMeshes::createCubeMesh(10.0);
    const int corner = mesh.F(0, 0);
    mesh.V.conservativeResize(mesh.numVertices() + 1, 3);
    mesh.V.row(mesh.numVertices() - 1) = mesh.V.row(corner) + Eigen::RowVector3d(0.01, 0.0, 0.0);
    mesh.F(0, 0) = static_cast<int>(mesh.numVertices() - 1);
    REQUIRE_FALSE(closedManifold(mesh));

    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 1);
    REQUIRE(r.degenerateRemoved == 0);
    REQUIRE(r.maxMove == Catch::Approx(0.01));
    REQUIRE(r.mesh.numVertices() == 8);
    REQUIRE(r.mesh.numFaces() == 12);
    REQUIRE(closedManifold(r.mesh));

    // Too small a distance changes nothing
    const ogun::MergeByDistanceResult none = ogun::mergeByDistance(mesh, 0.005);
    REQUIRE(none.verticesMerged == 0);
    REQUIRE(none.mesh.numFaces() == mesh.numFaces());
}

TEST_CASE("Merge by distance collapses a short edge and removes its two faces", "[repair][cleanup][merge]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    REQUIRE(closedManifold(mesh));
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 1);
    REQUIRE(r.degenerateRemoved == 2);
    REQUIRE(r.duplicatesRemoved == 0);
    REQUIRE(r.maxMove <= 0.05);
    REQUIRE(r.mesh.numFaces() == mesh.numFaces() - 2);
    REQUIRE(closedManifold(r.mesh));
}

TEST_CASE("Merge by distance only touches vertices of the faces in scope", "[repair][cleanup][merge]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 1);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (mesh.F(f, c) == a || mesh.F(f, c) == b) {
                mask[static_cast<std::size_t>(f)] = 0;
            }
        }
    }
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05, mask);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 0);
    REQUIRE(r.mesh.V == mesh.V);
    REQUIRE(r.mesh.F == mesh.F);
}

TEST_CASE("Merge by distance never moves a vertex more than the distance", "[repair][cleanup][merge]") {
    // A fine sphere with a large distance: the clusters are centred on their first vertex, not chained
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(32, 48, 5.0);
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(sphere, 0.6);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged > 0);
    REQUIRE(r.maxMove <= 0.6);
    REQUIRE(r.mesh.V.rowwise().norm().maxCoeff() == Catch::Approx(5.0)); // kept vertices never move
}

TEST_CASE("Decimate halves a sphere and keeps it closed", "[repair][cleanup][decimate]") {
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(32, 48, 5.0);
    const ogun::DecimateResult r = ogun::decimate(sphere, 0.5);
    REQUIRE(r.success);
    REQUIRE(r.facesBefore == static_cast<std::size_t>(sphere.numFaces()));
    REQUIRE(static_cast<double>(r.facesAfter) == Catch::Approx(0.5 * static_cast<double>(sphere.numFaces())).epsilon(0.02));
    REQUIRE(r.mesh.numFaces() == static_cast<Eigen::Index>(r.facesAfter));
    REQUIRE(closedManifold(r.mesh));
    REQUIRE(ogun::vertexDeviation(r.mesh, sphere).maxCoeff() < 0.1);

    const ogun::DecimateResult same = ogun::decimate(sphere, 1.0);
    REQUIRE(same.mesh.numFaces() == sphere.numFaces());
}

TEST_CASE("Decimating selected faces keeps every other face and joins them exactly", "[repair][cleanup][decimate]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(8, 8.0);
    const std::vector<std::uint8_t> top = facesAbove(mesh, 7.99); // the top side only
    const auto topCount = static_cast<std::size_t>(std::count(top.begin(), top.end(), 1));
    REQUIRE(topCount == 128);
    const ogun::DecimateResult r = ogun::decimate(mesh, 0.25, top);
    REQUIRE(r.success);
    REQUIRE(r.facesBefore == topCount);
    REQUIRE(r.facesAfter < topCount);
    REQUIRE(r.mesh.numFaces() == mesh.numFaces() - static_cast<Eigen::Index>(topCount - r.facesAfter));
    REQUIRE(closedManifold(r.mesh));
    // Only top faces changed; the flat top stays flat
    const ogun::FaceDiff diff = ogun::faceDiff(mesh, r.mesh);
    for (const Eigen::Index f : diff.removed) {
        REQUIRE(top[static_cast<std::size_t>(f)] == 1);
    }
    REQUIRE(ogun::vertexDeviation(r.mesh, mesh).maxCoeff() < 1e-9);
}

TEST_CASE("Decimate and merge run as one-method operators with the merge distance in file units", "[repair][cleanup][global]") {
    // A centimetre mesh: the distance is 0.05 in file coordinates, so a 0.04 edge collapses and a 0.06 one does not
    ogun::Mesh mesh = cubeWithShortEdge(0.04);
    mesh.fileUnit = ogun::Unit::Centimeter;
    ogun::GlobalRepairParams params;
    REQUIRE(params.mergeDistance == 0.05);
    REQUIRE(params.decimateRatio == 0.5);
    const ogun::RepairCandidate merged = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(merged.success);
    REQUIRE(merged.mesh.numFaces() == mesh.numFaces() - 2);
    REQUIRE(merged.removedFaces.size() >= 2);
    REQUIRE(merged.metrics.watertight());

    ogun::Mesh wider = cubeWithShortEdge(0.06);
    wider.fileUnit = ogun::Unit::Centimeter;
    const ogun::RepairCandidate kept = ogun::runGlobalRepair(wider, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(kept.success);
    REQUIRE(kept.mesh.numFaces() == wider.numFaces());
    REQUIRE(kept.changedFaces.empty());

    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const ogun::RepairCandidate decimated = ogun::runGlobalRepair(sphere, {}, ogun::GlobalRepairMethod::Decimate, params);
    REQUIRE(decimated.success);
    REQUIRE(decimated.metrics.facesAfter < decimated.metrics.facesBefore);
    REQUIRE(decimated.metrics.watertight());
    REQUIRE(decimated.vertexDeviation.size() == decimated.mesh.numVertices());
}

TEST_CASE("Merge keeps touching surfaces apart unless asked", "[repair][merge]") {
    // Cube B's corner (0, 0, 0) sits 0.02 mm from cube A's corner (8, 8, 8): no edge joins them
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(2, 8.0);
    ogun::Mesh other = mesh;
    other.V.rowwise() += Eigen::RowVector3d(8.02, 8.0, 8.0);
    ogun::appendMesh(mesh, other);
    mesh.recomputeNormals();

    ogun::GlobalRepairParams params;
    REQUIRE(params.mergeKeepSurfacesApart);
    const ogun::RepairCandidate apart = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(apart.success);
    REQUIRE(apart.changedFaces.empty());
    REQUIRE(apart.mesh.numVertices() == mesh.numVertices());

    params.mergeKeepSurfacesApart = false;
    const ogun::RepairCandidate glued = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(glued.success);
    REQUIRE(glued.mesh.numVertices() == mesh.numVertices() - 1); // the two shells now share a vertex
}

TEST_CASE("Keeping surfaces apart still welds cracks and collapses short edges", "[repair][merge]") {
    // Two open unit squares with a 0.02 crack between them: the crack's four vertices are on open edges
    ogun::Mesh sheets;
    sheets.V.resize(8, 3);
    sheets.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1.02, 0, 0, 2, 0, 0, 2, 1, 0, 1.02, 1, 0;
    sheets.F.resize(4, 3);
    sheets.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    sheets.recomputeNormals();
    const ogun::MergeByDistanceResult crack = ogun::mergeByDistance(sheets, 0.05, {}, true);
    REQUIRE(crack.success);
    REQUIRE(crack.verticesMerged == 2);
    REQUIRE(crack.mesh.numVertices() == 6);

    const ogun::Mesh shortEdge = cubeWithShortEdge(0.02);
    const ogun::MergeByDistanceResult collapsed = ogun::mergeByDistance(shortEdge, 0.05, {}, true);
    REQUIRE(collapsed.verticesMerged == 1);
    REQUIRE(collapsed.degenerateRemoved == 2);
}

TEST_CASE("Decimate stops at the target face count, not one collapse below it", "[repair][decimate]") {
    const ogun::Mesh box = ogun::SampleMeshes::createGridCubeMesh(1, 10.0); // 12 faces
    REQUIRE(box.numFaces() == 12);
    const ogun::DecimateResult r = ogun::decimate(box, 0.5);
    REQUIRE(r.success);
    REQUIRE(r.facesAfter >= 6);
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const ogun::DecimateResult s = ogun::decimate(sphere, 0.3);
    REQUIRE(s.facesAfter >= static_cast<std::size_t>(std::lround(0.3 * static_cast<double>(sphere.numFaces()))));
}

TEST_CASE("Merge distance is in file coordinates whatever the file unit", "[repair][merge][units]") {
    for (const ogun::Unit unit : {ogun::Unit::Millimeter, ogun::Unit::Centimeter, ogun::Unit::Meter}) {
        ogun::GlobalRepairParams params;
        params.mergeDistance = 0.05;

        ogun::Mesh closer = cubeWithShortEdge(0.04);
        closer.fileUnit = unit;
        const ogun::RepairCandidate merged = ogun::runGlobalRepair(closer, {}, ogun::GlobalRepairMethod::MergeVertices, params);
        REQUIRE(merged.success);
        REQUIRE(merged.mesh.numFaces() == closer.numFaces() - 2);

        ogun::Mesh farther = cubeWithShortEdge(0.06);
        farther.fileUnit = unit;
        const ogun::RepairCandidate kept = ogun::runGlobalRepair(farther, {}, ogun::GlobalRepairMethod::MergeVertices, params);
        REQUIRE(kept.success);
        REQUIRE(kept.mesh.numFaces() == farther.numFaces());
    }
}
