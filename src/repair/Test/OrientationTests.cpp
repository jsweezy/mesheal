#include "repair/Orientation.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/SliverHealer.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <optional>
#include <vector>

namespace {

using ogun::SampleMeshes;

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

ogun::Mesh box(Eigen::Vector3d min, Eigen::Vector3d max, bool inward = false) {
    return SampleMeshes::createBox(min, max, inward);
}

ogun::Mesh meshOf(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& faces) {
    ogun::Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(vertices.size()), 3);
    for (std::size_t v = 0; v < vertices.size(); ++v) {
        mesh.V.row(static_cast<Eigen::Index>(v)) = vertices[v].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

void flip(ogun::Mesh& mesh, Eigen::Index f) {
    std::swap(mesh.F(f, 1), mesh.F(f, 2));
    mesh.recomputeNormals();
}

/// Closed octahedron around `c` with half-sizes `r`, wound inward.
ogun::Mesh inwardOctahedron(const Eigen::Vector3d& c, const Eigen::Vector3d& r) {
    const std::vector<Eigen::Vector3d> v = {c + Eigen::Vector3d(r.x(), 0, 0), c - Eigen::Vector3d(r.x(), 0, 0),
                                            c + Eigen::Vector3d(0, r.y(), 0), c - Eigen::Vector3d(0, r.y(), 0),
                                            c + Eigen::Vector3d(0, 0, r.z()), c - Eigen::Vector3d(0, 0, r.z())};
    return meshOf(v, {{0, 4, 2}, {2, 4, 1}, {1, 4, 3}, {3, 4, 0}, {0, 2, 5}, {2, 1, 5}, {1, 3, 5}, {3, 0, 5}});
}

/// Open Mobius strip around the z axis (radius 3, half-width 1), `segments` quads.
ogun::Mesh mobiusStrip(int segments) {
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i < segments; ++i) {
        const double t = 2.0 * std::numbers::pi * i / segments;
        const Eigen::Vector3d radial(std::cos(t), std::sin(t), 0.0);
        const Eigen::Vector3d across = std::cos(t / 2.0) * radial + std::sin(t / 2.0) * Eigen::Vector3d::UnitZ();
        v.push_back(3.0 * radial - across);
        v.push_back(3.0 * radial + across);
    }
    std::vector<std::array<int, 3>> f;
    for (int i = 0; i < segments; ++i) {
        const int a = 2 * i;
        const int b = 2 * i + 1;
        const bool last = i + 1 == segments;
        const int c = last ? 1 : 2 * (i + 1);
        const int d = last ? 0 : 2 * (i + 1) + 1;
        f.push_back({a, c, b});
        f.push_back({b, c, d});
    }
    return meshOf(v, f);
}

/// Flat open strip of `quads` unit squares along x (2 * quads faces, all facing +z).
ogun::Mesh strip(int quads) {
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i <= quads; ++i) {
        v.emplace_back(i, 0, 0);
        v.emplace_back(i, 1, 0);
    }
    std::vector<std::array<int, 3>> f;
    for (int i = 0; i < quads; ++i) {
        f.push_back({2 * i, 2 * i + 2, 2 * i + 3});
        f.push_back({2 * i, 2 * i + 3, 2 * i + 1});
    }
    return meshOf(v, f);
}

ogun::Mesh loaded(const ogun::Mesh& mesh) {
    return ogun::separateTouchingContacts(mesh);
}

/// Signed volume of the shell whose box starts at `min` (nullopt if there is none).
std::optional<double> volumeOfShellAt(const ogun::Mesh& mesh, const Eigen::Vector3d& min) {
    const auto shells = ogun::shellsLargestFirst(mesh);
    for (const ogun::ShellFacts& facts : ogun::describeShells(mesh, shells)) {
        if ((facts.box.min - min).norm() < 1e-9) {
            return facts.signedVolume;
        }
    }
    return std::nullopt;
}

} // namespace

TEST_CASE("orientShells flips the minority faces of a closed cube", "[repair][orientation]") {
    ogun::Mesh cube = box({0, 0, 0}, {2, 2, 2});
    const ogun::Mesh good = cube;
    for (const Eigen::Index f : {0, 5, 9}) {
        flip(cube, f);
    }
    REQUIRE(ogun::checkFlippedNormals(cube).size() == 3);

    const ogun::OrientShellsResult result = ogun::orientShells(cube);
    REQUIRE(result.complete);
    REQUIRE(result.flippedFaces == 3);
    REQUIRE(result.shellsWithFlippedFaces == std::vector<int>{0});
    REQUIRE(result.mesh.F == good.F);
    REQUIRE(result.reversedShells.empty());
    REQUIRE(result.faceMap == std::vector<Eigen::Index>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    REQUIRE(ogun::checkFlippedNormals(result.mesh).empty());
    REQUIRE(ogun::signedVolume(result.mesh) == Catch::Approx(8.0));
    REQUIRE(result.mesh.N.row(0).isApprox(good.N.row(0)));

    // Out of scope faces stay as they are
    std::vector<std::uint8_t> scope(12, 1);
    scope[9] = 0;
    const ogun::OrientShellsResult scoped = ogun::orientShells(cube, scope);
    REQUIRE(scoped.flippedFaces == 2);
    REQUIRE(scoped.mesh.F.row(9) == cube.F.row(9));
}

TEST_CASE("orientShells: on an open strip the majority wins", "[repair][orientation]") {
    ogun::Mesh mesh = strip(5); // 10 faces
    const ogun::Mesh good = mesh;
    for (const Eigen::Index f : {0, 3, 4, 8}) {
        flip(mesh, f);
    }
    const ogun::OrientShellsResult result = ogun::orientShells(mesh);
    REQUIRE(result.flippedFaces == 4);
    REQUIRE(result.mesh.F == good.F);

    // Six of ten flipped: the flipped ones are now the majority, so the other four turn instead
    ogun::Mesh most = good;
    for (const Eigen::Index f : {0, 1, 2, 3, 4, 5}) {
        flip(most, f);
    }
    const ogun::OrientShellsResult majority = ogun::orientShells(most);
    REQUIRE(majority.flippedFaces == 4);
    REQUIRE(majority.mesh.F.topRows(6) == most.F.topRows(6));
    for (Eigen::Index f = 6; f < 10; ++f) {
        REQUIRE(majority.mesh.F.row(f) != good.F.row(f));
    }
    REQUIRE(ogun::checkFlippedNormals(majority.mesh).empty());
}

TEST_CASE("orientShells reverses an inside-out cube and keeps a cavity", "[repair][orientation]") {
    const ogun::Mesh insideOut = box({0, 0, 0}, {2, 2, 2}, true);
    const ogun::OrientShellsResult reversed = ogun::orientShells(insideOut);
    REQUIRE(reversed.reversedShells == std::vector<int>{0});
    REQUIRE(reversed.flippedFaces == 0);
    REQUIRE(ogun::signedVolume(reversed.mesh) == Catch::Approx(8.0));
    REQUIRE(reversed.mesh.F == box({0, 0, 0}, {2, 2, 2}).F);

    const ogun::Mesh cavityBox = loaded(SampleMeshes::createCavityBoxMesh());
    const ogun::OrientShellsResult cavity = ogun::orientShells(cavityBox);
    REQUIRE(cavity.mesh.F == cavityBox.F);
    REQUIRE(cavity.reversedShells.empty());
    REQUIRE(cavity.cavitiesKept == std::vector<int>{1});
    REQUIRE(volumeOfShellAt(cavity.mesh, {1, 1, 1}) == Catch::Approx(-8.0));

    // An outward box inside a box is a nested solid: never turned into a cavity (CGAL parity would)
    const ogun::Mesh nested = loaded(SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({1, 1, 1}, {3, 3, 3})}));
    const ogun::OrientShellsResult nestedResult = ogun::orientShells(nested);
    REQUIRE(nestedResult.mesh.F == nested.F);
    REQUIRE(nestedResult.roles.shells.empty()); // nothing is wound inward: rule O was not needed
}

TEST_CASE("orientShells stops at a body written inside out with a shell inside it", "[repair][orientation]") {
    const ogun::Mesh mesh = loaded(SampleMeshes::createCavityBoxMesh(true));
    const ogun::OrientShellsResult result = ogun::orientShells(mesh);
    REQUIRE(result.mesh.F == mesh.F);
    REQUIRE(result.reversedShells.empty());
    REQUIRE(result.stopped == std::vector<std::vector<int>>{{0, 1}});

    // Flip outer only: A outward, C outward inside it = a nested solid
    const auto shells = ogun::shellsLargestFirst(mesh);
    const ogun::FlipShellsResult outer = ogun::flipShells(mesh, std::span(shells.data(), 1));
    REQUIRE(outer.flippedFaces == 12);
    const ogun::ShellRoles outerRoles = ogun::classifyShellRoles(outer.mesh, ogun::computeShellRelations(outer.mesh));
    REQUIRE(outerRoles.shells[0].role == ogun::ShellRole::Solid);
    REQUIRE(outerRoles.shells[1].role == ogun::ShellRole::NestedSolid);

    // Flip all: the correct cavity box
    const ogun::FlipShellsResult all = ogun::flipShells(mesh, shells);
    REQUIRE(all.flippedFaces == 24);
    REQUIRE(all.mesh.F == loaded(SampleMeshes::createCavityBoxMesh()).F);
    REQUIRE(ogun::orientShells(all.mesh).cavitiesKept == std::vector<int>{1});

    // A face listed twice is flipped once
    const std::vector<std::vector<Eigen::Index>> twice = {{0, 1}, {1}};
    REQUIRE(ogun::flipShells(mesh, twice).flippedFaces == 2);
}

TEST_CASE("orientShells leaves inward shells it cannot judge, and reverses inside-out islands only",
          "[repair][orientation]") {
    // An inward octahedron resting on the box's wall: cavity or inside out cannot be told
    const ogun::Mesh touching =
        loaded(SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), inwardOctahedron({1.5, 1.0, 2.5}, {1.5, 0.8, 0.8})}));
    const ogun::OrientShellsResult unclassified = ogun::orientShells(touching);
    REQUIRE(unclassified.mesh.F == touching.F);
    REQUIRE(unclassified.unclassified == std::vector<int>{1});

    const ogun::Mesh island = loaded(SampleMeshes::createIslandInCavityMesh());
    const ogun::OrientShellsResult islandResult = ogun::orientShells(island);
    REQUIRE(islandResult.mesh.F == island.F);
    REQUIRE(islandResult.cavitiesKept == std::vector<int>{1});

    const ogun::Mesh flippedIsland = loaded(SampleMeshes::createIslandInCavityMesh(true));
    const ogun::OrientShellsResult flippedResult = ogun::orientShells(flippedIsland);
    REQUIRE(flippedResult.reversedShells == std::vector<int>{2});
    REQUIRE(flippedResult.mesh.F == island.F);
    REQUIRE(volumeOfShellAt(flippedResult.mesh, {1, 1, 1}) == Catch::Approx(-64.0));

    // An inside-out box crossing a solid, mostly outside it, is reversed; one mostly inside is not
    const ogun::Mesh crossing = loaded(SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({3, 3, 3}, {7, 7, 7}, true)}));
    REQUIRE(ogun::orientShells(crossing).reversedShells == std::vector<int>{1});
    const ogun::Mesh reached = loaded(SampleMeshes::createCavityReachedMesh());
    const ogun::OrientShellsResult reachedResult = ogun::orientShells(reached);
    REQUIRE(reachedResult.mesh.F == reached.F);
    REQUIRE(reachedResult.unclassified == std::vector<int>{2});
}

TEST_CASE("orientShells reports a Mobius strip and leaves it as it is", "[repair][orientation]") {
    const ogun::Mesh mobius = mobiusStrip(24);
    const ogun::OrientShellsResult result = ogun::orientShells(mobius);
    REQUIRE(result.nonOrientable == std::vector<int>{0});
    REQUIRE(result.flippedFaces == 0);
    REQUIRE(result.mesh.F == mobius.F);

    // A non-manifold edge splits the propagation: each sheet is oriented on its own
    ogun::Mesh fin = SampleMeshes::createTouchingCubesMesh();
    const ogun::OrientShellsResult finResult = ogun::orientShells(fin);
    REQUIRE(finResult.complete);
    REQUIRE(ogun::checkFlippedNormals(finResult.mesh).empty());
}

TEST_CASE("orientShells reverses only inside-out shells wholly in scope, and none that cross themselves",
          "[repair][orientation]") {
    const ogun::Mesh two = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}, true), box({5, 0, 0}, {6, 1, 1}, true)});
    std::vector<std::uint8_t> scope(24, 0);
    std::fill(scope.begin(), scope.begin() + 12, 1);
    const ogun::OrientShellsResult scoped = ogun::orientShells(two, scope);
    REQUIRE(scoped.reversedShells == std::vector<int>{0});
    REQUIRE(scoped.outOfScope == std::vector<int>{1});
    REQUIRE(scoped.mesh.F.bottomRows(12) == two.F.bottomRows(12));

    // A closed box with a corner pushed through its bottom: wound so its volume is negative
    ogun::Mesh dented = box({0, 0, 0}, {2, 2, 2});
    for (Eigen::Index v = 0; v < dented.numVertices(); ++v) {
        if (dented.V.row(v).isApprox(Eigen::RowVector3d(2, 2, 2))) {
            dented.V.row(v) << 1.2, 1.4, -1.0;
        }
    }
    if (ogun::signedVolume(dented) > 0.0) {
        for (Eigen::Index f = 0; f < dented.numFaces(); ++f) {
            std::swap(dented.F(f, 1), dented.F(f, 2));
        }
    }
    dented.recomputeNormals();
    REQUIRE(ogun::signedVolume(dented) < 0.0);
    const auto shells = ogun::shellsLargestFirst(dented);
    const std::vector<int> which{0};
    const auto crossing = ogun::findSelfCrossingShells(dented, shells, which, 1e-9);
    REQUIRE(crossing.has_value());
    REQUIRE(*crossing == std::vector<std::uint8_t>{1});
    const ogun::OrientShellsResult self = ogun::orientShells(dented);
    REQUIRE(self.reversedShells.empty());
    REQUIRE(self.selfCrossing == std::vector<int>{0});
    REQUIRE(self.mesh.F == dented.F);

    // Stopped: the input comes back, flagged incomplete
    std::stop_source source;
    source.request_stop();
    const ogun::OrientShellsResult stopped = ogun::orientShells(box({0, 0, 0}, {2, 2, 2}, true), {}, {}, source.get_token());
    REQUIRE_FALSE(stopped.complete);
    REQUIRE(stopped.reversedShells.empty());
}

TEST_CASE("A cavity survives the CGAL-based repairs with its sign unchanged", "[repair][orientation][cavity]") {
    // The Fix All sample: a body with a hole, slivers and a fold, around a cavity [1.5, 3.5]^3, and a peg
    const ogun::Mesh sample = loaded(SampleMeshes::createCavityHoleSliversIntersectionsMesh());
    const Eigen::Vector3d cavityMin(1.5, 1.5, 1.5);
    REQUIRE(volumeOfShellAt(sample, cavityMin) == Catch::Approx(-8.0));

    const ogun::RepairResults slivers = ogun::SliverHealer::healAllThinFaces(sample);
    REQUIRE(slivers.success);
    REQUIRE(volumeOfShellAt(slivers.mesh, cavityMin) == Catch::Approx(-8.0));

    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(slivers.mesh);
    REQUIRE_FALSE(holes.empty());
    const ogun::HoleBatchResult filled = ogun::fillHoles(slivers.mesh, holes);
    REQUIRE(filled.success);
    REQUIRE(volumeOfShellAt(filled.mesh, cavityMin) == Catch::Approx(-8.0));

    const ogun::IslandBatchResult islands = ogun::healIntersectionIslands(filled.mesh, ogun::CheckParams{});
    const ogun::Mesh& repaired = islands.success ? islands.mesh : filled.mesh;
    REQUIRE(volumeOfShellAt(repaired, cavityMin) == Catch::Approx(-8.0));

    // Orientation then keeps it as a cavity of the closed body
    const ogun::OrientShellsResult oriented = ogun::orientShells(repaired);
    REQUIRE(oriented.reversedShells.empty());
    REQUIRE(oriented.cavitiesKept.size() == 1);
    REQUIRE(volumeOfShellAt(oriented.mesh, cavityMin) == Catch::Approx(-8.0));

    // The cavity box with a hole cut in A: the fill closes A and leaves C as it was
    ogun::Mesh holed = loaded(SampleMeshes::createCavityBoxMesh());
    const auto shells = ogun::shellsLargestFirst(holed);
    const Eigen::Index cut = shells[0].front();
    holed.F.row(cut) = holed.F.row(holed.numFaces() - 1);
    holed.F.conservativeResize(holed.numFaces() - 1, 3);
    holed.recomputeNormals();
    const ogun::HoleBatchResult closed = ogun::fillHoles(holed, ogun::findHoleLoops(holed));
    REQUIRE(closed.success);
    REQUIRE(volumeOfShellAt(closed.mesh, {1, 1, 1}) == Catch::Approx(-8.0));
    REQUIRE(volumeOfShellAt(closed.mesh, {0, 0, 0}) == Catch::Approx(64.0));
    REQUIRE(ogun::orientShells(closed.mesh).cavitiesKept.size() == 1);
}

TEST_CASE("Turkey orientation changes nothing and keeps its cavity", "[repair][orientation][.slow]") {
    const ogun::Mesh turkey = ogun::loadStl(examplePath("Turkey.stl"));
    const auto start = std::chrono::steady_clock::now();
    const ogun::OrientShellsResult result = ogun::orientShells(turkey);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    WARN("Turkey orientShells: " << ms << " ms");
    REQUIRE(result.complete);
    REQUIRE(result.flippedFaces == 0);
    REQUIRE(result.reversedShells.empty());
    REQUIRE(result.cavitiesKept == std::vector<int>{27}); // #28, the cavity of #12
    REQUIRE(result.mesh.F == turkey.F);
}
