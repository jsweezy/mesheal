// The mesh state a repair trace records: every check's count, and the crossing pairs by kind and shell.

#include "diagnostics/MeshState.hpp"

#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <numeric>

namespace {

using mesheal::CrossingKind;
using mesheal::Mesh;
using mesheal::SampleMeshes;

std::size_t total(const std::array<std::size_t, mesheal::kCrossingKindCount>& counts) {
    return std::accumulate(counts.begin(), counts.end(), std::size_t{0});
}

std::size_t of(const std::array<std::size_t, mesheal::kCrossingKindCount>& counts, CrossingKind kind) {
    return counts[static_cast<std::size_t>(kind)];
}

} // namespace

TEST_CASE("A clean closed box measures clean", "[diagnostics][mesh-state]") {
    const Mesh box = SampleMeshes::createBox({0, 0, 0}, {2, 3, 4});
    const mesheal::MeshState state = mesheal::measureMeshState(box, mesheal::CheckParams{});
    CHECK(state.complete);
    CHECK(state.faces == 12);
    CHECK(state.shells == 1);
    CHECK(state.volume == Catch::Approx(24.0));
    CHECK(state.openEdges + state.nonManifoldEdges + state.nonManifoldVertices + state.flippedFaces == 0);
    CHECK(state.crossingFaces == 0);
    CHECK(total(state.pairsWithinShell) + total(state.pairsBetweenShells) == 0);
}

TEST_CASE("Crossing pairs are told apart by kind and by shell", "[diagnostics][mesh-state]") {
    SECTION("two boxes passing through each other: crossings between shells") {
        const Mesh mesh = SampleMeshes::combineMeshes(
            {SampleMeshes::createBox({0, 0, 0}, {2, 2, 2}), SampleMeshes::createBox({1, 1.1, 1.2}, {3, 3.1, 3.2})});
        const auto pairs = mesheal::classifyCrossings(mesh, mesheal::CheckParams{});
        REQUIRE(pairs);
        REQUIRE_FALSE(pairs->empty());
        for (const mesheal::CrossingPair& pair : *pairs) {
            CHECK(pair.a < pair.b);
            CHECK_FALSE(pair.withinShell());
            CHECK(pair.kind == CrossingKind::Crossing);
        }
        const mesheal::MeshState state = mesheal::measureMeshState(mesh, mesheal::CheckParams{});
        CHECK(state.shells == 2);
        CHECK(total(state.pairsWithinShell) == 0);
        CHECK(of(state.pairsBetweenShells, CrossingKind::Crossing) == pairs->size());
        // The same faces the intersection check counts
        CHECK(state.crossingFaces == mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{}).size());
    }
    SECTION("a face folded flat onto its edge neighbour") {
        Mesh mesh;
        mesh.V.resize(4, 3);
        mesh.V << 0, 0, 0, 2, 0, 0, 0, 2, 0, 1, 1, 0;
        mesh.F.resize(2, 3);
        mesh.F << 0, 1, 2, 1, 0, 3;
        mesh.recomputeNormals();
        const auto pairs = mesheal::classifyCrossings(mesh, mesheal::CheckParams{});
        REQUIRE(pairs);
        REQUIRE(pairs->size() == 1);
        CHECK(pairs->front().kind == CrossingKind::Folded);
        CHECK(pairs->front().withinShell());
    }
    SECTION("a shell that passes through itself: crossings within the shell") {
        const Mesh box = SampleMeshes::createSelfIntersectingBoxMesh();
        const mesheal::MeshState state = mesheal::measureMeshState(box, mesheal::CheckParams{});
        CHECK(state.shells == 1);
        CHECK(total(state.pairsBetweenShells) == 0);
        CHECK(total(state.pairsWithinShell) > 0);
        CHECK(state.crossingFaces == mesheal::checkSelfIntersections(box, mesheal::CheckParams{}).size());
    }
}

TEST_CASE("Shell indices follow the Shells panel order", "[diagnostics][mesh-state]") {
    const Mesh mesh = SampleMeshes::combineMeshes(
        {SampleMeshes::createBox({5, 5, 5}, {6, 6, 6}), SampleMeshes::createGridCubeMesh(3, 2.0)});
    const std::vector<int> shellOf = mesheal::shellIndexOfFaces(mesh);
    REQUIRE(shellOf.size() == static_cast<std::size_t>(mesh.numFaces()));
    CHECK(shellOf.front() == 1); // the small box comes first in the file but is the second-largest shell
    CHECK(shellOf.back() == 0);
}
