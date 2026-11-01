#include "repair/GlobalRepair.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SurfaceRebuild.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cstdio>
#include <numeric>
#include <vector>

namespace {

Eigen::Index vertexAt(const ogun::Mesh& mesh, const Eigen::RowVector3d& p) {
    Eigen::Index best = 0;
    (mesh.V.rowwise() - p).rowwise().squaredNorm().minCoeff(&best);
    return best;
}

// 6 x 6 grid cube (side 6) with the top-centre vertex dragged 1.5 cells sideways in the top plane.
ogun::Mesh makeFoldedCube() {
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(6, 6.0);
    mesh.V.row(vertexAt(mesh, {3.0, 3.0, 6.0})) << 4.5, 3.2, 6.0;
    mesh.recomputeNormals();
    return mesh;
}

// Same cube with the top-centre vertex pushed down through the bottom wall.
ogun::Mesh makePokedCube() {
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(6, 6.0);
    mesh.V.row(vertexAt(mesh, {3.0, 3.0, 6.0})) << 3.0, 3.0, -1.0;
    mesh.recomputeNormals();
    return mesh;
}

ogun::Mesh twoOverlappingCubes() {
    const ogun::Mesh a = ogun::SampleMeshes::createCubeMesh(10.0);
    ogun::Mesh mesh = a;
    ogun::Mesh b = a;
    b.V = a.V.rowwise() + Eigen::RowVector3d(5.0, 5.0, 5.0);
    ogun::appendMesh(mesh, b);
    return mesh;
}

std::size_t crossings(const ogun::Mesh& mesh) {
    return ogun::checkSelfIntersections(mesh, ogun::CheckParams{}).size();
}

bool closedManifold(const ogun::Mesh& mesh) {
    return ogun::checkOpenEdges(mesh).empty() && ogun::checkNonManifoldEdges(mesh).empty();
}

std::vector<ogun::IntersectionIsland> islandsOf(const ogun::Mesh& mesh) {
    const ogun::CategoryResult check = ogun::checkSelfIntersections(mesh, ogun::CheckParams{});
    return ogun::clusterIntersectionIslands(mesh, check, ogun::shellIdsOfFaces(mesh));
}

} // namespace

TEST_CASE("faceDiff ignores renumbering and reports real changes", "[repair][metrics]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(2.0);
    ogun::Mesh reordered = cube;
    reordered.F = cube.F.colwise().reverse();                  // faces in reverse order
    reordered.F.col(0).swap(reordered.F.col(1));               // ... and rotated corners
    reordered.F.col(1).swap(reordered.F.col(2));
    REQUIRE(ogun::faceDiff(cube, reordered).removed.empty());
    REQUIRE(ogun::faceDiff(cube, reordered).added.empty());

    ogun::Mesh moved = cube;
    moved.V.row(0) += Eigen::RowVector3d(0.1, 0.0, 0.0);
    const ogun::FaceDiff diff = ogun::faceDiff(cube, moved);
    REQUIRE_FALSE(diff.removed.empty());
    REQUIRE(diff.removed.size() == diff.added.size());
}

TEST_CASE("Repair metrics measure volume, area and deviation", "[repair][metrics]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(2.0);
    const ogun::RepairMetrics same = ogun::computeRepairMetrics(cube, cube, ogun::CheckParams{});
    REQUIRE(same.maxDeviation == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(same.unchangedFraction == Catch::Approx(1.0));
    REQUIRE(same.watertight());

    ogun::Mesh bigger = cube;
    bigger.V *= 1.5;
    const ogun::RepairMetrics m = ogun::computeRepairMetrics(cube, bigger, ogun::CheckParams{});
    REQUIRE(m.volumeBefore == Catch::Approx(8.0));
    REQUIRE(m.volumeAfter == Catch::Approx(27.0));
    REQUIRE(m.areaChangePercent() == Catch::Approx(125.0));
    REQUIRE(m.maxDeviation > 0.0);
    REQUIRE(m.unchangedFraction == Catch::Approx(0.0));
}

TEST_CASE("Islands: a fold is local, a poke-through is not, crossings between shells are skipped",
          "[repair][intersections][islands]") {
    const ogun::Mesh folded = makeFoldedCube();
    REQUIRE(crossings(folded) > 0);
    const auto foldIslands = islandsOf(folded);
    REQUIRE(foldIslands.size() == 1);
    REQUIRE(foldIslands[0].local);
    const ogun::IslandAnalysis fold = ogun::analyzeIsland(folded, foldIslands[0]);
    REQUIRE(fold.recommended == ogun::IntersectionRepairMethod::FillFlat); // flat collar

    const ogun::Mesh poked = makePokedCube();
    const auto pokeIslands = islandsOf(poked);
    REQUIRE(pokeIslands.size() == 1);
    REQUIRE_FALSE(pokeIslands[0].local);
    REQUIRE(ogun::analyzeIsland(poked, pokeIslands[0]).recommended == ogun::IntersectionRepairMethod::SelfUnion);

    const ogun::Mesh cubes = twoOverlappingCubes();
    REQUIRE(crossings(cubes) > 0);
    REQUIRE(islandsOf(cubes).empty());
}

TEST_CASE("Every island method repairs a fold without new crossings", "[repair][intersections][islands]") {
    const auto method = GENERATE(ogun::IntersectionRepairMethod::FillFlat, ogun::IntersectionRepairMethod::FillSmooth,
                                 ogun::IntersectionRepairMethod::CgalLocalRepair, ogun::IntersectionRepairMethod::SelfUnion);
    CAPTURE(ogun::intersectionRepairMethodLabel(method));
    const ogun::Mesh folded = makeFoldedCube();
    const auto islands = islandsOf(folded);
    REQUIRE(islands.size() == 1);

    const ogun::IslandRepairResult r = ogun::repairIsland(folded, islands[0], method, ogun::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(r.crossingsBefore > 0);
    REQUIRE(r.crossingsAfter == 0);
    REQUIRE(r.newCrossings == 0);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(closedManifold(r.mesh));
    REQUIRE(std::abs(r.volumeChange) < 1.0); // the cube holds 216

    const ogun::IslandRepairPreview preview = ogun::previewIslandRepair(folded, islands[0], method, ogun::CheckParams{});
    REQUIRE(preview.valid);
    REQUIRE(preview.facesRemoved == r.removedFaces.size());
    REQUIRE(preview.sourceFaceIndices == r.removedFaces);
    REQUIRE(preview.previewPatchMesh.numFaces() == static_cast<Eigen::Index>(r.addedFaces.size()));
}

TEST_CASE("Cut & Merge Walls repairs a poke-through into a watertight shell", "[repair][intersections][islands]") {
    const ogun::Mesh poked = makePokedCube();
    const auto islands = islandsOf(poked);
    REQUIRE(islands.size() == 1);
    const ogun::IslandRepairResult r =
        ogun::repairIsland(poked, islands[0], ogun::IntersectionRepairMethod::SelfUnion, ogun::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(closedManifold(r.mesh));
}

TEST_CASE("Filling an island on an open sheet keeps the sheet's own border open", "[repair][intersections][islands]") {
    // Top side of the folded cube only: an open 6 x 6 sheet with the fold in the middle
    const ogun::Mesh folded = makeFoldedCube();
    std::vector<Eigen::Index> top;
    for (Eigen::Index f = 0; f < folded.numFaces(); ++f) {
        if ((folded.V(folded.F(f, 0), 2) + folded.V(folded.F(f, 1), 2) + folded.V(folded.F(f, 2), 2)) > 17.9) {
            top.push_back(f);
        }
    }
    const ogun::Mesh sheet = ogun::extractFaces(folded, top);
    const std::size_t borderBefore = ogun::checkOpenEdges(sheet).size();
    const auto islands = islandsOf(sheet);
    REQUIRE(islands.size() == 1);

    const ogun::IslandRepairResult r =
        ogun::repairIsland(sheet, islands[0], ogun::IntersectionRepairMethod::FillFlat, ogun::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(ogun::checkOpenEdges(r.mesh).size() == borderBefore);
}

TEST_CASE("Batch island repair: all islands, or only the seeded ones", "[repair][intersections][batch]") {
    ogun::Mesh both = makeFoldedCube();
    ogun::Mesh poked = makePokedCube();
    poked.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    ogun::appendMesh(both, poked);
    REQUIRE(islandsOf(both).size() == 2);

    const ogun::IslandBatchResult all = ogun::healIntersectionIslands(both, ogun::CheckParams{});
    REQUIRE(all.success);
    REQUIRE(all.islandsBefore == 2);
    REQUIRE(all.islandsAfter == 0);
    REQUIRE(crossings(all.mesh) == 0);
    REQUIRE(closedManifold(all.mesh));

    // Seed only the fold (the first cube's faces come first)
    const auto islands = islandsOf(both);
    const auto& foldIsland = islands[0].local ? islands[0] : islands[1];
    const std::vector<Eigen::Index> seed{foldIsland.faces.front()};
    const ogun::IslandBatchResult some = ogun::healIntersectionIslands(both, ogun::CheckParams{}, {}, seed);
    REQUIRE(some.success);
    REQUIRE(some.islandsBefore == 1);
    REQUIRE(islandsOf(some.mesh).size() == 1); // the poke-through is untouched
}

TEST_CASE("Global repairs merge overlapping cubes into one clean shell", "[repair][intersections][global]") {
    const auto method = GENERATE(ogun::GlobalRepairMethod::SelfUnion, ogun::GlobalRepairMethod::WindingRebuild,
                                 ogun::GlobalRepairMethod::AlphaWrap);
    CAPTURE(ogun::globalRepairMethodLabel(method));
    const ogun::Mesh cubes = twoOverlappingCubes();
    ogun::GlobalRepairParams params;
    params.alphaWrapDetail = 20.0; // coarse: keeps the test fast
    params.windingResolution = 48;
    const ogun::RepairCandidate c = ogun::runGlobalRepair(cubes, {}, method, params);
    REQUIRE(c.success);
    REQUIRE(c.metrics.shellsBefore == 2);
    REQUIRE(c.metrics.shellsAfter == 1);
    REQUIRE(c.metrics.intersectingFacesBefore > 0);
    REQUIRE(c.metrics.intersectingFacesAfter == 0);
    REQUIRE(c.metrics.watertight());
    REQUIRE_FALSE(c.changedFaces.empty());
    if (method == ogun::GlobalRepairMethod::SelfUnion) {
        // Union of two 10-cubes overlapping in a 5-cube: exact, and the uncrossed faces survive
        REQUIRE(c.metrics.volumeAfter == Catch::Approx(1875.0));
        REQUIRE(c.metrics.maxDeviation < 1e-9);
    }
}

TEST_CASE("Alpha wrap offset is alpha / x and follows alpha", "[repair][global][wrap]") {
    ogun::GlobalRepairParams params;
    // The default x reproduces the old automatic offset, alpha / 30
    REQUIRE(params.alphaWrapOffsetRatio == ogun::settings::kDefaultAlphaWrapOffsetRatio);
    REQUIRE(params.alphaWrapOffsetRatio == 30.0);
    const double diag = 100.0;
    const double alpha = ogun::alphaWrapAlpha(params, diag);
    REQUIRE(alpha == Catch::Approx(diag / ogun::settings::kDefaultAlphaWrapDetail));
    REQUIRE(ogun::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 30.0));

    params.alphaWrapOffsetRatio = 75.0;
    REQUIRE(ogun::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 75.0));

    // For a fixed x, a larger alpha (lower detail) gives a proportionally larger offset
    params.alphaWrapDetail = 25.0;
    const double coarseAlpha = ogun::alphaWrapAlpha(params, diag);
    REQUIRE(coarseAlpha == Catch::Approx(4.0 * alpha));
    REQUIRE(ogun::alphaWrapOffset(params, coarseAlpha) == Catch::Approx(4.0 * ogun::alphaWrapOffset(params, alpha)));
    REQUIRE(ogun::alphaWrapOffset(params, coarseAlpha) == Catch::Approx(coarseAlpha / 75.0));

    // A non-positive x (never set by the app) falls back to the default
    params.alphaWrapOffsetRatio = 0.0;
    REQUIRE(ogun::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 30.0));
}

TEST_CASE("The alpha wrap lies alpha / x outside the input", "[repair][global][wrap]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(10.0);
    ogun::GlobalRepairParams params;
    params.alphaWrapDetail = 20.0; // coarse: keeps the test fast
    const double alpha = ogun::alphaWrapAlpha(params, cube.boundingBox().diagonal());

    const ogun::RepairCandidate standard = ogun::runGlobalRepair(cube, {}, ogun::GlobalRepairMethod::AlphaWrap, params);
    params.alphaWrapOffsetRatio = 2.0;
    const ogun::RepairCandidate wide = ogun::runGlobalRepair(cube, {}, ogun::GlobalRepairMethod::AlphaWrap, params);
    REQUIRE(standard.success);
    REQUIRE(wide.success);
    REQUIRE(wide.metrics.watertight());
    REQUIRE(standard.params.alphaWrapOffsetRatio == 30.0);
    REQUIRE(wide.params.alphaWrapOffsetRatio == 2.0);

    // The wrap stays about `offset` outside the cube: never closer, and the volume grows with it
    for (const auto* c : {&standard, &wide}) {
        const double offset = alpha / c->params.alphaWrapOffsetRatio;
        CAPTURE(offset);
        REQUIRE(ogun::alphaWrapOffset(c->params, alpha) == Catch::Approx(offset));
        REQUIRE(c->metrics.maxDeviation >= 0.9 * offset);
        REQUIRE(c->metrics.maxDeviation <= 2.0 * offset);
        REQUIRE(c->metrics.volumeAfter > 1000.0);
        // The effective offset alpha / x is what reaches CGAL
        std::array<char, 96> text{};
        std::snprintf(text.data(), text.size(), "Wrapped with alpha %.3g and offset %.3g.", alpha, offset);
        CAPTURE(c->statusMessage);
        REQUIRE(c->statusMessage.find(text.data()) != std::string::npos);
    }
    REQUIRE(wide.metrics.volumeAfter > standard.metrics.volumeAfter + 100.0); // offset 0.43: 10.87^3 - 10^3 is 283 before rounding
}

TEST_CASE("Winding-number rebuild of an axis-aligned cube has no degenerate faces", "[repair][intersections][global]") {
    // The cube's sides lie on grid planes unless the samples are shifted: w = 0.5 exactly there
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(10.0);
    const ogun::Mesh rebuilt = ogun::windingNumberRebuild(cube, 16);
    REQUIRE_FALSE(rebuilt.empty());
    for (Eigen::Index f = 0; f < rebuilt.numFaces(); ++f) {
        const Eigen::RowVector3d a = rebuilt.V.row(rebuilt.F(f, 0));
        const Eigen::RowVector3d b = rebuilt.V.row(rebuilt.F(f, 1));
        const Eigen::RowVector3d c = rebuilt.V.row(rebuilt.F(f, 2));
        REQUIRE((b - a).cross(c - a).norm() > 1e-9);
    }
    REQUIRE(crossings(rebuilt) == 0);
    REQUIRE(closedManifold(rebuilt));
}

TEST_CASE("Global repairs leave shells out of scope untouched", "[repair][intersections][global]") {
    ogun::Mesh mesh = makeFoldedCube();
    const Eigen::Index foldFaces = mesh.numFaces();
    ogun::Mesh other = makePokedCube();
    other.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    ogun::appendMesh(mesh, other);

    std::vector<std::uint8_t> inScope(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::fill(inScope.begin(), inScope.begin() + foldFaces, std::uint8_t{1});

    const ogun::RepairCandidate c =
        ogun::runGlobalRepair(mesh, inScope, ogun::GlobalRepairMethod::PatchIslands, ogun::GlobalRepairParams{});
    REQUIRE(c.success);
    REQUIRE(c.metrics.intersectingFacesAfter == 0);
    // The poked cube is copied as is and still crosses itself
    std::vector<Eigen::Index> otherFaces(static_cast<std::size_t>(other.numFaces()));
    std::iota(otherFaces.begin(), otherFaces.end(), Eigen::Index{0});
    const ogun::Mesh kept = ogun::extractFaces(c.mesh, otherFaces);
    REQUIRE(ogun::faceDiff(other, kept).removed.empty());
    REQUIRE(crossings(c.mesh) > 0);
}

TEST_CASE("Self-union per shell keeps crossing shells apart", "[repair][intersections][global]") {
    ogun::Mesh mesh = twoOverlappingCubes();
    ogun::appendMesh(mesh, [] {
        ogun::Mesh poked = makePokedCube();
        poked.V.rowwise() += Eigen::RowVector3d(30.0, 0.0, 0.0);
        return poked;
    }());
    ogun::GlobalRepairParams params;
    params.unionCrossingShells = false;
    const ogun::RepairCandidate c = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::SelfUnion, params);
    REQUIRE(c.success);
    REQUIRE(c.metrics.shellsAfter == 3);            // the two cubes stay separate
    REQUIRE(c.metrics.intersectingFacesAfter > 0);  // ... so they still cross each other
    REQUIRE(islandsOf(c.mesh).empty());             // but nothing crosses within a shell
}

TEST_CASE("Voxel Heal & Remesh and Precision Isosurface heal overlapping cubes into a closed manifold",
          "[repair][intersections][global][remesh]") {
    const ogun::Mesh cubes = twoOverlappingCubes();
    std::string voxelStatus;
    const ogun::Mesh voxelOut = ogun::voxelHealAndRemesh(cubes, 0.75, 3, &voxelStatus);
    REQUIRE_FALSE(voxelOut.empty());
    REQUIRE(voxelStatus.find("Grid ") != std::string::npos);
    REQUIRE(crossings(voxelOut) == 0);
    REQUIRE(closedManifold(voxelOut));

    std::string isoStatus;
    const ogun::Mesh isoOut = ogun::precisionIsosurfaceRemesh(cubes, 1.0, 0.10, 1, &isoStatus);
    REQUIRE_FALSE(isoOut.empty());
    REQUIRE(isoStatus.find("Isosurface extracted") != std::string::npos);
    REQUIRE(crossings(isoOut) == 0);
    REQUIRE(closedManifold(isoOut));

    // Cancellation via stop_token returns an empty mesh with status "Cancelled."
    std::stop_source stopSource;
    stopSource.request_stop();
    std::string cancelStatus;
    REQUIRE(ogun::voxelHealAndRemesh(cubes, 0.75, 3, &cancelStatus, stopSource.get_token()).empty());
    REQUIRE(cancelStatus == "Cancelled.");
    REQUIRE(ogun::precisionIsosurfaceRemesh(cubes, 1.0, 0.10, 1, &cancelStatus, stopSource.get_token()).empty());
    REQUIRE(cancelStatus == "Cancelled.");
}

TEST_CASE("Precision Isosurface rejects an open flat sheet without hanging", "[repair][intersections][global][remesh]") {
    ogun::Mesh sheet;
    sheet.V.resize(4, 3);
    sheet.V << 0.0, 0.0, 0.0,
               10.0, 0.0, 0.0,
               10.0, 10.0, 0.0,
               0.0, 10.0, 0.01;
    sheet.F.resize(2, 3);
    sheet.F << 0, 1, 2,
               0, 2, 3;
    sheet.recomputeNormals();

    std::string status;
    const ogun::Mesh out = ogun::precisionIsosurfaceRemesh(sheet, 1.0, 0.10, 1, &status);
    REQUIRE(out.empty());
    REQUIRE(status.find("no enclosed volume") != std::string::npos);
}
