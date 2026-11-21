#include "repair/FoldRepair.hpp"
#include "repair/IntersectionHealer.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshState.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace {

constexpr double kAnyMove = std::numeric_limits<double>::infinity();

/// 6 x 6 grid cube (side 6). One top face around the top-centre vertex has that vertex moved across the
/// face's far edge into the middle of the neighbour, so the face lies folded flat onto the neighbour.
mesheal::Mesh foldedTopCube() {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createGridCubeMesh(6, 6.0);
    Eigen::Index x = 0;
    (mesh.V.rowwise() - Eigen::RowVector3d(3.0, 3.0, 6.0)).rowwise().squaredNorm().minCoeff(&x);
    const auto onTop = [&](Eigen::Index f) {
        return (mesh.V(mesh.F(f, 0), 2) > 5.99) && (mesh.V(mesh.F(f, 1), 2) > 5.99) && (mesh.V(mesh.F(f, 2), 2) > 5.99);
    };
    for (Eigen::Index a = 0; a < mesh.numFaces(); ++a) {
        int k = 0;
        while (k < 3 && mesh.F(a, k) != x) {
            ++k;
        }
        if (k == 3 || !onTop(a)) {
            continue;
        }
        const int p = mesh.F(a, (k + 1) % 3);
        const int q = mesh.F(a, (k + 2) % 3);
        for (Eigen::Index b = 0; b < mesh.numFaces(); ++b) {
            const auto has = [&](int v) { return mesh.F(b, 0) == v || mesh.F(b, 1) == v || mesh.F(b, 2) == v; };
            if (b != a && has(p) && has(q) && onTop(b)) {
                mesh.V.row(x) = (mesh.V.row(mesh.F(b, 0)) + mesh.V.row(mesh.F(b, 1)) + mesh.V.row(mesh.F(b, 2))) / 3.0;
                mesh.recomputeNormals();
                return mesh;
            }
        }
    }
    FAIL("no top face to fold");
    return mesh;
}

std::size_t folds(const mesheal::Mesh& mesh) {
    const auto pairs = mesheal::classifyCrossings(mesh, mesheal::CheckParams{});
    REQUIRE(pairs.has_value());
    return static_cast<std::size_t>(std::count_if(pairs->begin(), pairs->end(), [](const mesheal::CrossingPair& p) {
        return p.kind == mesheal::CrossingKind::Folded;
    }));
}

bool closedManifold(const mesheal::Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).elements.empty() && mesheal::checkNonManifoldEdges(mesh).elements.empty() &&
           mesheal::checkNonManifoldVertices(mesh).elements.empty();
}

} // namespace

TEST_CASE("Fold repair unfolds a face folded flat onto its neighbour", "[repair][folds]") {
    mesheal::Mesh mesh = foldedTopCube();
    REQUIRE(folds(mesh) >= 1);
    REQUIRE(closedManifold(mesh));

    const mesheal::FoldRepairResult r = mesheal::unfoldFaces(mesh, mesheal::CheckParams{}, kAnyMove);
    REQUIRE_FALSE(r.cancelled);
    CHECK(r.folds >= 1);
    CHECK(r.unfolded >= 1);
    CHECK(folds(mesh) == 0);
    CHECK(mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{}).elements.empty());
    CHECK(closedManifold(mesh));
    CHECK(mesheal::signedVolume(mesh) == Catch::Approx(216.0)); // the top stays flat
}

TEST_CASE("Fold repair leaves a clean mesh alone", "[repair][folds]") {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createGridCubeMesh(4, 2.0);
    const Eigen::MatrixXi faces = mesh.F;
    const mesheal::FoldRepairResult r = mesheal::unfoldFaces(mesh, mesheal::CheckParams{}, kAnyMove);
    CHECK(r.folds == 0);
    CHECK(r.unfolded == 0);
    CHECK(mesh.F == faces);
}

TEST_CASE("Fold repair only touches its target faces", "[repair][folds]") {
    mesheal::Mesh mesh = foldedTopCube();
    const Eigen::MatrixXd vertices = mesh.V;
    const Eigen::MatrixXi faces = mesh.F;
    const std::vector<std::uint8_t> none(static_cast<std::size_t>(mesh.numFaces()), 0);
    const mesheal::FoldRepairResult r = mesheal::unfoldFaces(mesh, mesheal::CheckParams{}, kAnyMove, none);
    CHECK(r.folds == 0);
    CHECK(r.unfolded == 0);
    CHECK(mesh.F == faces);
    CHECK(mesh.V == vertices);
}

TEST_CASE("Fold repair moves no vertex farther than allowed", "[repair][folds]") {
    mesheal::Mesh mesh = foldedTopCube();
    const mesheal::FoldRepairResult r = mesheal::unfoldFaces(mesh, mesheal::CheckParams{}, 0.0);
    CHECK(r.collapses == 0); // only a flip moves nothing
    CHECK(r.maxMove == 0.0);
    CHECK(closedManifold(mesh));
}

TEST_CASE("Batch island repair unfolds folded faces", "[repair][folds][intersections][batch]") {
    const mesheal::Mesh mesh = foldedTopCube();
    const mesheal::IslandBatchResult r = mesheal::healIntersectionIslands(mesh, mesheal::CheckParams{});
    REQUIRE(r.success);
    CHECK(r.foldsUnfolded >= 1);
    CHECK(r.islandsAfter == 0);
    CHECK(mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{}).elements.empty());
    CHECK(closedManifold(r.mesh));
    CHECK(r.statusMessage.find("folded face pair") != std::string::npos);
}
