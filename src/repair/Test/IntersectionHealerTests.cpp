#include "repair/GlobalRepair.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SurfaceRebuild.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <filesystem>
#include <cstdio>
#include <numeric>
#include <vector>

namespace {

Eigen::Index vertexAt(const mesheal::Mesh& mesh, const Eigen::RowVector3d& p) {
    Eigen::Index best = 0;
    (mesh.V.rowwise() - p).rowwise().squaredNorm().minCoeff(&best);
    return best;
}

// 6 x 6 grid cube (side 6) with the top-centre vertex dragged 1.5 cells sideways in the top plane.
mesheal::Mesh makeFoldedCube() {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createGridCubeMesh(6, 6.0);
    mesh.V.row(vertexAt(mesh, {3.0, 3.0, 6.0})) << 4.5, 3.2, 6.0;
    mesh.recomputeNormals();
    return mesh;
}

// Same cube with the top-centre vertex pushed down through the bottom wall.
mesheal::Mesh makePokedCube() {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createGridCubeMesh(6, 6.0);
    mesh.V.row(vertexAt(mesh, {3.0, 3.0, 6.0})) << 3.0, 3.0, -1.0;
    mesh.recomputeNormals();
    return mesh;
}

mesheal::Mesh twoOverlappingCubes() {
    const mesheal::Mesh a = mesheal::SampleMeshes::createCubeMesh(10.0);
    mesheal::Mesh mesh = a;
    mesheal::Mesh b = a;
    b.V = a.V.rowwise() + Eigen::RowVector3d(5.0, 5.0, 5.0);
    mesheal::appendMesh(mesh, b);
    return mesh;
}

std::size_t crossings(const mesheal::Mesh& mesh) {
    return mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{}).size();
}

bool closedManifold(const mesheal::Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).empty() && mesheal::checkNonManifoldEdges(mesh).empty();
}

std::vector<mesheal::IntersectionIsland> islandsOf(const mesheal::Mesh& mesh) {
    const mesheal::CategoryResult check = mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{});
    return mesheal::clusterIntersectionIslands(mesh, check, mesheal::shellIdsOfFaces(mesh));
}

} // namespace

TEST_CASE("faceDiff ignores renumbering and reports real changes", "[repair][metrics]") {
    const mesheal::Mesh cube = mesheal::SampleMeshes::createCubeMesh(2.0);
    mesheal::Mesh reordered = cube;
    reordered.F = cube.F.colwise().reverse();                  // faces in reverse order
    reordered.F.col(0).swap(reordered.F.col(1));               // ... and rotated corners
    reordered.F.col(1).swap(reordered.F.col(2));
    REQUIRE(mesheal::faceDiff(cube, reordered).removed.empty());
    REQUIRE(mesheal::faceDiff(cube, reordered).added.empty());

    mesheal::Mesh moved = cube;
    moved.V.row(0) += Eigen::RowVector3d(0.1, 0.0, 0.0);
    const mesheal::FaceDiff diff = mesheal::faceDiff(cube, moved);
    REQUIRE_FALSE(diff.removed.empty());
    REQUIRE(diff.removed.size() == diff.added.size());
}

TEST_CASE("Repair metrics measure volume, area and deviation", "[repair][metrics]") {
    const mesheal::Mesh cube = mesheal::SampleMeshes::createCubeMesh(2.0);
    const mesheal::RepairMetrics same = mesheal::computeRepairMetrics(cube, cube, mesheal::CheckParams{});
    REQUIRE(same.maxDeviation == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(same.unchangedFraction == Catch::Approx(1.0));
    REQUIRE(same.watertight());

    mesheal::Mesh bigger = cube;
    bigger.V *= 1.5;
    const mesheal::RepairMetrics m = mesheal::computeRepairMetrics(cube, bigger, mesheal::CheckParams{});
    REQUIRE(m.volumeBefore == Catch::Approx(8.0));
    REQUIRE(m.volumeAfter == Catch::Approx(27.0));
    REQUIRE(m.areaChangePercent() == Catch::Approx(125.0));
    REQUIRE(m.maxDeviation > 0.0);
    REQUIRE(m.unchangedFraction == Catch::Approx(0.0));
}

TEST_CASE("Islands: a fold is local, a poke-through is not, crossings between shells are skipped",
          "[repair][intersections][islands]") {
    const mesheal::Mesh folded = makeFoldedCube();
    REQUIRE(crossings(folded) > 0);
    const auto foldIslands = islandsOf(folded);
    REQUIRE(foldIslands.size() == 1);
    REQUIRE(foldIslands[0].local);
    const mesheal::IslandAnalysis fold = mesheal::analyzeIsland(folded, foldIslands[0]);
    REQUIRE(fold.recommended == mesheal::IntersectionRepairMethod::FillFlat); // flat collar

    const mesheal::Mesh poked = makePokedCube();
    const auto pokeIslands = islandsOf(poked);
    REQUIRE(pokeIslands.size() == 1);
    REQUIRE_FALSE(pokeIslands[0].local);
    REQUIRE(mesheal::analyzeIsland(poked, pokeIslands[0]).recommended == mesheal::IntersectionRepairMethod::SelfUnion);

    const mesheal::Mesh cubes = twoOverlappingCubes();
    REQUIRE(crossings(cubes) > 0);
    REQUIRE(islandsOf(cubes).empty());
}

TEST_CASE("Every island method repairs a fold without new crossings", "[repair][intersections][islands]") {
    const auto method = GENERATE(mesheal::IntersectionRepairMethod::FillFlat, mesheal::IntersectionRepairMethod::FillSmooth,
                                 mesheal::IntersectionRepairMethod::CgalLocalRepair, mesheal::IntersectionRepairMethod::SelfUnion);
    CAPTURE(mesheal::intersectionRepairMethodLabel(method));
    const mesheal::Mesh folded = makeFoldedCube();
    const auto islands = islandsOf(folded);
    REQUIRE(islands.size() == 1);

    const mesheal::IslandRepairResult r = mesheal::repairIsland(folded, islands[0], method, mesheal::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(r.crossingsBefore > 0);
    REQUIRE(r.crossingsAfter == 0);
    REQUIRE(r.newCrossings == 0);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(closedManifold(r.mesh));
    REQUIRE(std::abs(r.volumeChange) < 1.0); // the cube holds 216

    const mesheal::IslandRepairPreview preview = mesheal::previewIslandRepair(folded, islands[0], method, mesheal::CheckParams{});
    REQUIRE(preview.valid);
    REQUIRE(preview.facesRemoved == r.removedFaces.size());
    REQUIRE(preview.sourceFaceIndices == r.removedFaces);
    REQUIRE(preview.previewPatchMesh.numFaces() == static_cast<Eigen::Index>(r.addedFaces.size()));
}

TEST_CASE("Cut & Merge Walls repairs a poke-through into a watertight shell", "[repair][intersections][islands]") {
    const mesheal::Mesh poked = makePokedCube();
    const auto islands = islandsOf(poked);
    REQUIRE(islands.size() == 1);
    const mesheal::IslandRepairResult r =
        mesheal::repairIsland(poked, islands[0], mesheal::IntersectionRepairMethod::SelfUnion, mesheal::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(closedManifold(r.mesh));
}

TEST_CASE("Filling an island on an open sheet keeps the sheet's own border open", "[repair][intersections][islands]") {
    // Top side of the folded cube only: an open 6 x 6 sheet with the fold in the middle
    const mesheal::Mesh folded = makeFoldedCube();
    std::vector<Eigen::Index> top;
    for (Eigen::Index f = 0; f < folded.numFaces(); ++f) {
        if ((folded.V(folded.F(f, 0), 2) + folded.V(folded.F(f, 1), 2) + folded.V(folded.F(f, 2), 2)) > 17.9) {
            top.push_back(f);
        }
    }
    const mesheal::Mesh sheet = mesheal::extractFaces(folded, top);
    const std::size_t borderBefore = mesheal::checkOpenEdges(sheet).size();
    const auto islands = islandsOf(sheet);
    REQUIRE(islands.size() == 1);

    const mesheal::IslandRepairResult r =
        mesheal::repairIsland(sheet, islands[0], mesheal::IntersectionRepairMethod::FillFlat, mesheal::CheckParams{});
    REQUIRE(r.success);
    REQUIRE(crossings(r.mesh) == 0);
    REQUIRE(mesheal::checkOpenEdges(r.mesh).size() == borderBefore);
}

TEST_CASE("Batch island repair: all islands, or only the seeded ones", "[repair][intersections][batch]") {
    mesheal::Mesh both = makeFoldedCube();
    mesheal::Mesh poked = makePokedCube();
    poked.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    mesheal::appendMesh(both, poked);
    REQUIRE(islandsOf(both).size() == 2);

    const mesheal::IslandBatchResult all = mesheal::healIntersectionIslands(both, mesheal::CheckParams{});
    REQUIRE(all.success);
    REQUIRE(all.islandsBefore == 2);
    REQUIRE(all.islandsAfter == 0);
    REQUIRE(crossings(all.mesh) == 0);
    REQUIRE(closedManifold(all.mesh));

    // Seed only the fold (the first cube's faces come first)
    const auto islands = islandsOf(both);
    const auto& foldIsland = islands[0].local ? islands[0] : islands[1];
    const std::vector<Eigen::Index> seed{foldIsland.faces.front()};
    const mesheal::IslandBatchResult some = mesheal::healIntersectionIslands(both, mesheal::CheckParams{}, {}, seed);
    REQUIRE(some.success);
    REQUIRE(some.islandsBefore == 1);
    REQUIRE(islandsOf(some.mesh).size() == 1); // the poke-through is untouched
}

TEST_CASE("Batch island repair keeps a CAD part's walls when a fill would bulge over them",
          "[repair][intersections][batch][tom-shuttle]") {
    // CAD part by teil3 (CC BY-SA, examples/tom_x_axis_shuttle_license.md): 6 small crossings next to large walls
    const std::filesystem::path path =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "tom_x_axis_shuttle.stl";
    REQUIRE(std::filesystem::exists(path));
    const mesheal::Mesh shuttle = mesheal::loadStl(path);
    REQUIRE(islandsOf(shuttle).size() == 6);

    // Without the guard the first round's fills cut whole walls out and bulge over them (the old behaviour)
    mesheal::IntersectionRepairSettings unguarded;
    unguarded.maxDeviationFraction = 0.0;
    const mesheal::IslandBatchResult old = mesheal::healIntersectionIslands(shuttle, mesheal::CheckParams{}, {}, {}, unguarded);
    REQUIRE(old.success);
    CHECK(mesheal::maxSurfaceDistance(shuttle, old.mesh) > 5.0); // mm of wall lost

    // With it that round is undone and the next method cuts along the crossings: nothing moves
    const mesheal::IslandBatchResult r = mesheal::healIntersectionIslands(shuttle, mesheal::CheckParams{});
    REQUIRE(r.success);
    CHECK(r.islandsAfter == 0);
    CHECK(crossings(r.mesh) == 0);
    CHECK(r.roundsRefusedForShape >= 1);
    CHECK(r.statusMessage.find("would have changed the shape") != std::string::npos);
    CHECK(mesheal::maxSurfaceDistance(shuttle, r.mesh) < 0.01);
    CHECK(mesheal::maxSurfaceDistance(r.mesh, shuttle) < 0.01);
    CHECK(mesheal::signedVolume(r.mesh) == Catch::Approx(mesheal::signedVolume(shuttle)).epsilon(1e-4));
}

TEST_CASE("Global repairs merge overlapping cubes into one clean shell", "[repair][intersections][global]") {
    const auto method = GENERATE(mesheal::GlobalRepairMethod::SelfUnion, mesheal::GlobalRepairMethod::WindingRebuild,
                                 mesheal::GlobalRepairMethod::AlphaWrap);
    CAPTURE(mesheal::globalRepairMethodLabel(method));
    const mesheal::Mesh cubes = twoOverlappingCubes();
    mesheal::GlobalRepairParams params;
    params.alphaWrapDetail = 20.0; // coarse: keeps the test fast
    params.windingResolution = 48;
    const mesheal::RepairCandidate c = mesheal::runGlobalRepair(cubes, {}, method, params);
    REQUIRE(c.success);
    REQUIRE(c.metrics.shellsBefore == 2);
    REQUIRE(c.metrics.shellsAfter == 1);
    REQUIRE(c.metrics.intersectingFacesBefore > 0);
    REQUIRE(c.metrics.intersectingFacesAfter == 0);
    REQUIRE(c.metrics.watertight());
    REQUIRE_FALSE(c.changedFaces.empty());
    if (method == mesheal::GlobalRepairMethod::SelfUnion) {
        // Union of two 10-cubes overlapping in a 5-cube: exact, and the uncrossed faces survive
        REQUIRE(c.metrics.volumeAfter == Catch::Approx(1875.0));
        REQUIRE(c.metrics.maxDeviation < 1e-9);
    }
}

TEST_CASE("Alpha wrap offset is alpha / x and follows alpha", "[repair][global][wrap]") {
    mesheal::GlobalRepairParams params;
    // The default x reproduces the old automatic offset, alpha / 30
    REQUIRE(params.alphaWrapOffsetRatio == mesheal::settings::kDefaultAlphaWrapOffsetRatio);
    REQUIRE(params.alphaWrapOffsetRatio == 30.0);
    const double diag = 100.0;
    const double alpha = mesheal::alphaWrapAlpha(params, diag);
    REQUIRE(alpha == Catch::Approx(diag / mesheal::settings::kDefaultAlphaWrapDetail));
    REQUIRE(mesheal::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 30.0));

    params.alphaWrapOffsetRatio = 75.0;
    REQUIRE(mesheal::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 75.0));

    // For a fixed x, a larger alpha (lower detail) gives a proportionally larger offset
    params.alphaWrapDetail = 25.0;
    const double coarseAlpha = mesheal::alphaWrapAlpha(params, diag);
    REQUIRE(coarseAlpha == Catch::Approx(4.0 * alpha));
    REQUIRE(mesheal::alphaWrapOffset(params, coarseAlpha) == Catch::Approx(4.0 * mesheal::alphaWrapOffset(params, alpha)));
    REQUIRE(mesheal::alphaWrapOffset(params, coarseAlpha) == Catch::Approx(coarseAlpha / 75.0));

    // A non-positive x (never set by the app) falls back to the default
    params.alphaWrapOffsetRatio = 0.0;
    REQUIRE(mesheal::alphaWrapOffset(params, alpha) == Catch::Approx(alpha / 30.0));
}

TEST_CASE("The alpha wrap lies alpha / x outside the input", "[repair][global][wrap]") {
    const mesheal::Mesh cube = mesheal::SampleMeshes::createCubeMesh(10.0);
    mesheal::GlobalRepairParams params;
    params.alphaWrapDetail = 20.0; // coarse: keeps the test fast
    const double alpha = mesheal::alphaWrapAlpha(params, cube.boundingBox().diagonal());

    const mesheal::RepairCandidate standard = mesheal::runGlobalRepair(cube, {}, mesheal::GlobalRepairMethod::AlphaWrap, params);
    params.alphaWrapOffsetRatio = 2.0;
    const mesheal::RepairCandidate wide = mesheal::runGlobalRepair(cube, {}, mesheal::GlobalRepairMethod::AlphaWrap, params);
    REQUIRE(standard.success);
    REQUIRE(wide.success);
    REQUIRE(wide.metrics.watertight());
    REQUIRE(standard.params.alphaWrapOffsetRatio == 30.0);
    REQUIRE(wide.params.alphaWrapOffsetRatio == 2.0);

    // The wrap stays about `offset` outside the cube: never closer, and the volume grows with it
    for (const auto* c : {&standard, &wide}) {
        const double offset = alpha / c->params.alphaWrapOffsetRatio;
        CAPTURE(offset);
        REQUIRE(mesheal::alphaWrapOffset(c->params, alpha) == Catch::Approx(offset));
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
    const mesheal::Mesh cube = mesheal::SampleMeshes::createCubeMesh(10.0);
    const mesheal::Mesh rebuilt = mesheal::windingNumberRebuild(cube, 16);
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
    mesheal::Mesh mesh = makeFoldedCube();
    const Eigen::Index foldFaces = mesh.numFaces();
    mesheal::Mesh other = makePokedCube();
    other.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    mesheal::appendMesh(mesh, other);

    std::vector<std::uint8_t> inScope(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::fill(inScope.begin(), inScope.begin() + foldFaces, std::uint8_t{1});

    const mesheal::RepairCandidate c =
        mesheal::runGlobalRepair(mesh, inScope, mesheal::GlobalRepairMethod::PatchIslands, mesheal::GlobalRepairParams{});
    REQUIRE(c.success);
    REQUIRE(c.metrics.intersectingFacesAfter == 0);
    // The poked cube is copied as is and still crosses itself
    std::vector<Eigen::Index> otherFaces(static_cast<std::size_t>(other.numFaces()));
    std::iota(otherFaces.begin(), otherFaces.end(), Eigen::Index{0});
    const mesheal::Mesh kept = mesheal::extractFaces(c.mesh, otherFaces);
    REQUIRE(mesheal::faceDiff(other, kept).removed.empty());
    REQUIRE(crossings(c.mesh) > 0);
}

TEST_CASE("Self-union per shell keeps crossing shells apart", "[repair][intersections][global]") {
    mesheal::Mesh mesh = twoOverlappingCubes();
    mesheal::appendMesh(mesh, [] {
        mesheal::Mesh poked = makePokedCube();
        poked.V.rowwise() += Eigen::RowVector3d(30.0, 0.0, 0.0);
        return poked;
    }());
    mesheal::GlobalRepairParams params;
    params.unionCrossingShells = false;
    const mesheal::RepairCandidate c = mesheal::runGlobalRepair(mesh, {}, mesheal::GlobalRepairMethod::SelfUnion, params);
    REQUIRE(c.success);
    REQUIRE(c.metrics.shellsAfter == 3);            // the two cubes stay separate
    REQUIRE(c.metrics.intersectingFacesAfter > 0);  // ... so they still cross each other
    REQUIRE(islandsOf(c.mesh).empty());             // but nothing crosses within a shell
}

TEST_CASE("Voxel Heal & Remesh and Precision Isosurface heal overlapping cubes into a closed manifold",
          "[repair][intersections][global][remesh]") {
    const mesheal::Mesh cubes = twoOverlappingCubes();
    std::string voxelStatus;
    const mesheal::Mesh voxelOut = mesheal::voxelHealAndRemesh(cubes, 0.75, 3, &voxelStatus);
    REQUIRE_FALSE(voxelOut.empty());
    REQUIRE(voxelStatus.find("Grid ") != std::string::npos);
    REQUIRE(crossings(voxelOut) == 0);
    REQUIRE(closedManifold(voxelOut));

    std::string isoStatus;
    const mesheal::Mesh isoOut = mesheal::precisionIsosurfaceRemesh(cubes, 1.0, 0.10, 1, &isoStatus);
    REQUIRE_FALSE(isoOut.empty());
    REQUIRE(isoStatus.find("Isosurface extracted") != std::string::npos);
    REQUIRE(crossings(isoOut) == 0);
    REQUIRE(mesheal::ThinFaceChecker::check(isoOut).defects.empty());
    REQUIRE(closedManifold(isoOut));

    const mesheal::Mesh simplified = mesheal::simplifyToFaceCount(isoOut, 64);
    REQUIRE(crossings(simplified) == 0);
    REQUIRE(mesheal::ThinFaceChecker::check(simplified).defects.empty());
    REQUIRE(closedManifold(simplified));

    // Cancellation via stop_token returns an empty mesh with status "Cancelled."
    std::stop_source stopSource;
    stopSource.request_stop();
    std::string cancelStatus;
    REQUIRE(mesheal::voxelHealAndRemesh(cubes, 0.75, 3, &cancelStatus, stopSource.get_token()).empty());
    REQUIRE(cancelStatus == "Cancelled.");
    REQUIRE(mesheal::precisionIsosurfaceRemesh(cubes, 1.0, 0.10, 1, &cancelStatus, stopSource.get_token()).empty());
    REQUIRE(cancelStatus == "Cancelled.");
}

TEST_CASE("Precision Isosurface rejects an open flat sheet without hanging", "[repair][intersections][global][remesh]") {
    mesheal::Mesh sheet;
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
    const mesheal::Mesh out = mesheal::precisionIsosurfaceRemesh(sheet, 1.0, 0.10, 1, &status);
    REQUIRE(out.empty());
    REQUIRE(status.find("no enclosed volume") != std::string::npos);
}

TEST_CASE("Batch island repair handles multiple islands on one shell cleanly", "[repair][intersections][batch]") {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createGridCubeMesh(12, 12.0);
    // Two separate local folds on opposite faces of the same shell
    mesh.V.row(vertexAt(mesh, {3.0, 3.0, 12.0})) << 4.5, 3.2, 12.0;
    mesh.V.row(vertexAt(mesh, {9.0, 9.0, 0.0})) << 10.5, 9.2, 0.0;
    mesh.recomputeNormals();
    REQUIRE(islandsOf(mesh).size() == 2);

    const mesheal::IslandBatchResult r = mesheal::healIntersectionIslands(mesh, mesheal::CheckParams{});
    REQUIRE(r.success);
    CHECK(r.islandsBefore == 2);
    CHECK(r.islandsAfter == 0);
    CHECK(crossings(r.mesh) == 0);
    CHECK(closedManifold(r.mesh));
}

TEST_CASE("CgalLocalRepair skips crossing patches whose boundary exceeds limit", "[repair][intersections][islands]") {
    // Closed triangulated bar subdivided along X so a single fold has > kMaxCgalLocalRepairBoundaryEdges rim edges
    constexpr int nx = 110;
    mesheal::Mesh bar;
    bar.V.resize(4 * (nx + 1), 3);
    const auto vIdx = [&](int i, int corner) { return 4 * i + corner; };
    for (int i = 0; i <= nx; ++i) {
        const double x = static_cast<double>(i);
        bar.V.row(vIdx(i, 0)) << x, 0.0, 0.0;
        bar.V.row(vIdx(i, 1)) << x, 1.0, 0.0;
        bar.V.row(vIdx(i, 2)) << x, 1.0, 1.0;
        bar.V.row(vIdx(i, 3)) << x, 0.0, 1.0;
    }
    std::vector<Eigen::Vector3i> tris;
    tris.reserve(8 * nx + 4);
    const auto addQuad = [&](int a, int b, int c, int d) {
        tris.emplace_back(a, b, c);
        tris.emplace_back(a, c, d);
    };
    for (int i = 0; i < nx; ++i) {
        for (int s = 0; s < 4; ++s) {
            const int sNext = (s + 1) % 4;
            addQuad(vIdx(i, s), vIdx(i + 1, s), vIdx(i + 1, sNext), vIdx(i, sNext));
        }
    }
    addQuad(vIdx(0, 0), vIdx(0, 3), vIdx(0, 2), vIdx(0, 1));
    addQuad(vIdx(nx, 0), vIdx(nx, 1), vIdx(nx, 2), vIdx(nx, 3));
    bar.F.resize(static_cast<Eigen::Index>(tris.size()), 3);
    for (std::size_t k = 0; k < tris.size(); ++k) {
        bar.F.row(static_cast<Eigen::Index>(k)) = tris[k];
    }
    // Fold corner 2 along x = 1..nx-1 inward across corner 0..3 so all crossing faces form one large island
    for (int i = 1; i < nx; ++i) {
        bar.V.row(vIdx(i, 2)) << static_cast<double>(i), -0.5, 0.25;
    }
    bar.recomputeNormals();

    const auto islands = islandsOf(bar);
    REQUIRE(islands.size() == 1);
    REQUIRE(islands[0].faces.size() > mesheal::settings::kMaxCgalLocalRepairBoundaryEdges);

    const mesheal::IslandRepairResult r = mesheal::repairIsland(
        bar, islands[0], mesheal::IntersectionRepairMethod::CgalLocalRepair, mesheal::CheckParams{});
    CHECK_FALSE(r.success);
    CHECK(r.statusMessage.find("skipped CGAL local repair") != std::string::npos);
}

TEST_CASE("Batch CgalLocalRepair skips closely spaced islands whose dilated boundary exceeds limit",
          "[repair][intersections][islands][batch]") {
    // Open 220 x 18 sheet with 28 small folds spaced 7 quads apart along the y = 0 border:
    // FillFlat/FillSmooth refuse border-touching cuts, leaving Round 1 (CgalLocalRepair) with 28 tiny
    // components (each <= 10 boundary edges) whose 7-ring neighborhoods merge past kMaxCubicHoleFillEdges.
    constexpr int nx = 220;
    constexpr int ny = 18;
    mesheal::Mesh sheet;
    sheet.V.resize((nx + 1) * (ny + 1), 3);
    const auto vIdx = [&](int i, int j) { return j * (nx + 1) + i; };
    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i <= nx; ++i) {
            sheet.V.row(vIdx(i, j)) << static_cast<double>(i), static_cast<double>(j), 0.0;
        }
    }
    // Fold y = 0 border vertices at spaced x positions across the y = 1 row to create 28 separate crossings
    for (int i = 10; i <= nx - 10; i += 7) {
        sheet.V.row(vIdx(i, 0)) << static_cast<double>(i), 1.6, 0.0;
    }
    sheet.F.resize(2 * nx * ny, 3);
    Eigen::Index f = 0;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int v00 = vIdx(i, j);
            const int v10 = vIdx(i + 1, j);
            const int v11 = vIdx(i + 1, j + 1);
            const int v01 = vIdx(i, j + 1);
            sheet.F.row(f++) << v00, v10, v11;
            sheet.F.row(f++) << v00, v11, v01;
        }
    }
    sheet.recomputeNormals();

    const auto islands = islandsOf(sheet);
    REQUIRE(islands.size() >= 20);
    for (const auto& island : islands) {
        REQUIRE(island.faces.size() <= 16);
    }

    const mesheal::IslandBatchResult batch = mesheal::healIntersectionIslands(sheet, mesheal::CheckParams{});
    CHECK_FALSE(batch.success);
    CHECK(batch.islandsAfter == batch.islandsBefore);
}
