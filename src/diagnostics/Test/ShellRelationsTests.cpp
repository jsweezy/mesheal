#include "diagnostics/ShellRelations.hpp"

#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

namespace {

using mesheal::SampleMeshes;

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

mesheal::Mesh box(Eigen::Vector3d min, Eigen::Vector3d max, bool inward = false) {
    return SampleMeshes::createBox(min, max, inward);
}

/// Loaded the way the app loads a sample.
mesheal::ShellRelations relationsOf(const mesheal::Mesh& mesh) {
    return mesheal::computeShellRelations(mesheal::separateTouchingContacts(mesh));
}

/// Shell pairs (low, high) that `checkSelfIntersections` reports between different shells.
std::set<std::pair<int, int>> checkedBetweenPairs(const mesheal::Mesh& mesh, const std::vector<std::vector<Eigen::Index>>& shells) {
    std::vector<int> faceShell(static_cast<std::size_t>(mesh.numFaces()), -1);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells[s]) {
            faceShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }
    const mesheal::CategoryResult result = mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{});
    std::set<std::pair<int, int>> pairs;
    for (std::size_t i = 0; i < result.size(); ++i) {
        const int own = faceShell[static_cast<std::size_t>(result.elements[i].index)];
        for (const Eigen::Index other : result.relatedOf(i)) {
            const int theirs = faceShell[static_cast<std::size_t>(other)];
            if (own != theirs) {
                pairs.insert({std::min(own, theirs), std::max(own, theirs)});
            }
        }
    }
    return pairs;
}

std::set<std::pair<int, int>> pairsWhere(const mesheal::ShellRelations& relations, bool (*keep)(const mesheal::ShellPairRelation&)) {
    std::set<std::pair<int, int>> pairs;
    for (const mesheal::ShellPairRelation& r : relations.pairs) {
        if (keep(r)) {
            pairs.insert({r.a, r.b});
        }
    }
    return pairs;
}

double totalVolume(const mesheal::ShellRelations& relations) {
    double total = 0.0;
    for (const mesheal::ShellFacts& shell : relations.shells) {
        total += shell.signedVolume;
    }
    return total;
}

} // namespace

TEST_CASE("Overlapping boxes cross; boxes apart have no relation", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations overlapping =
        relationsOf(SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1, 1, 1}, {3, 3, 3})}));
    REQUIRE(overlapping.complete);
    REQUIRE(overlapping.shells.size() == 2);
    REQUIRE(overlapping.pairs.size() == 1);
    const mesheal::ShellPairRelation* r = overlapping.pair(1, 0);
    REQUIRE(r != nullptr);
    REQUIRE(r->crosses());
    REQUIRE(r->crossings > 0);
    REQUIRE_FALSE(r->sharesFaceArea());
    REQUIRE_FALSE(r->edgeOrPointContactOnly());
    REQUIRE(overlapping.tolerance == Catch::Approx(1.0e-6 * std::sqrt(27.0)));

    const mesheal::ShellRelations apart =
        relationsOf(SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({5, 0, 0}, {7, 2, 2})}));
    REQUIRE(apart.pairs.empty());
    REQUIRE(apart.pair(0, 1) == nullptr);
    REQUIRE(apart.shells[0].nestedIn == -1);
    REQUIRE(apart.shells[1].nestedIn == -1);
}

TEST_CASE("Boxes touching along an edge or at a corner are a contact, not a crossing", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations edge = relationsOf(SampleMeshes::createEdgeTouchingBoxesMesh());
    REQUIRE(edge.shells.size() == 2);
    REQUIRE(edge.pairs.size() == 1);
    REQUIRE(edge.pairs[0].edgeOrPointContactOnly());
    REQUIRE_FALSE(edge.pairs[0].crosses());
    REQUIRE(edge.pairs[0].contacts > 0);
    REQUIRE(totalVolume(edge) == Catch::Approx(16.0));

    const mesheal::ShellRelations corner =
        relationsOf(SampleMeshes::combineMeshes({box({0, 0, 0}, {1, 1, 1}), box({1, 1, 1}, {2, 2, 2})}));
    REQUIRE(corner.pairs.size() == 1);
    REQUIRE(corner.pairs[0].edgeOrPointContactOnly());
    REQUIRE(corner.pairs[0].intersectingPairs == 0); // the intersection check ignores a point contact

    // The touching-cubes sample: two cubes along an edge, one at a corner (the fin is part of a cube)
    const mesheal::ShellRelations touching = relationsOf(SampleMeshes::createTouchingCubesMesh());
    REQUIRE(touching.shells.size() == 3);
    REQUIRE(touching.pairs.size() == 2);
    for (const mesheal::ShellPairRelation& r : touching.pairs) {
        REQUIRE(r.edgeOrPointContactOnly());
    }

    // Without the contact test only what the intersection check reports is left
    mesheal::ShellRelationParams noContacts;
    noContacts.contacts = false;
    const mesheal::ShellRelations cornerOnly = mesheal::computeShellRelations(
        SampleMeshes::combineMeshes({box({0, 0, 0}, {1, 1, 1}), box({1, 1, 1}, {2, 2, 2})}), noContacts);
    REQUIRE(cornerOnly.pairs.empty());
}

TEST_CASE("Boxes touching across a face share a face area", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations relations = relationsOf(SampleMeshes::createFaceTouchingBoxesMesh());
    REQUIRE(relations.pairs.size() == 1);
    const mesheal::ShellPairRelation& r = relations.pairs[0];
    REQUIRE(r.sharesFaceArea());
    REQUIRE_FALSE(r.crosses());
    REQUIRE_FALSE(r.edgeOrPointContactOnly());
    REQUIRE(totalVolume(relations) == Catch::Approx(10.0));
}

TEST_CASE("A cavity is nested in its box and meets nothing", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations relations = relationsOf(SampleMeshes::createCavityBoxMesh());
    REQUIRE(relations.shells.size() == 2);
    REQUIRE(relations.pairs.empty());
    REQUIRE(relations.shells[0].nestedIn == -1);
    REQUIRE(relations.shells[1].nestedIn == 0);
    REQUIRE(relations.shells[1].containers == std::vector<int>{0});
    REQUIRE_FALSE(relations.shells[1].nestingUnsure);
    REQUIRE(relations.shells[0].closed);
    REQUIRE(relations.shells[1].closed);
    REQUIRE(relations.shells[0].signedVolume == Catch::Approx(64.0));
    REQUIRE(relations.shells[1].signedVolume == Catch::Approx(-8.0));
    REQUIRE(totalVolume(relations) == Catch::Approx(56.0));

    // Inside out: nesting does not depend on winding, only the signs change
    const mesheal::ShellRelations insideOut = relationsOf(SampleMeshes::createCavityBoxMesh(true));
    REQUIRE(insideOut.shells[1].nestedIn == 0);
    REQUIRE(insideOut.shells[0].signedVolume == Catch::Approx(-64.0));
    REQUIRE(insideOut.shells[1].signedVolume == Catch::Approx(8.0));
}

TEST_CASE("An island in a cavity reports the cavity as its container", "[diagnostics][shell-relations]") {
    for (const bool insideOut : {false, true}) {
        const mesheal::ShellRelations relations = relationsOf(SampleMeshes::createIslandInCavityMesh(insideOut));
        REQUIRE(relations.shells.size() == 3);
        REQUIRE(relations.pairs.empty());
        REQUIRE(relations.shells[1].nestedIn == 0);
        REQUIRE(relations.shells[2].nestedIn == 1);
        REQUIRE(relations.shells[2].containers == std::vector<int>{1, 0});
        REQUIRE(relations.shells[1].signedVolume == Catch::Approx(-64.0));
        REQUIRE(relations.shells[2].signedVolume == Catch::Approx(insideOut ? -8.0 : 8.0));
    }
}

TEST_CASE("A nested box and a cavity both lie in the outer box", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations relations = relationsOf(SampleMeshes::createNestedSolidAndCavityMesh());
    REQUIRE(relations.shells.size() == 3);
    REQUIRE(relations.pairs.empty());
    REQUIRE(relations.shells[1].nestedIn == 0);
    REQUIRE(relations.shells[2].nestedIn == 0);
    REQUIRE(relations.shells[1].signedVolume == Catch::Approx(-1.0));
    REQUIRE(relations.shells[2].signedVolume == Catch::Approx(1.0));

    // A third inner box crossing the nested one: both still lie in the outer box, not in each other
    const mesheal::ShellRelations crossing = relationsOf(SampleMeshes::createNestedSolidAndCavityMesh(true));
    REQUIRE(crossing.shells.size() == 4);
    REQUIRE(crossing.pairs.size() == 1);
    REQUIRE(crossing.pair(2, 3) != nullptr);
    REQUIRE(crossing.pair(2, 3)->crosses());
    REQUIRE(crossing.shells[2].containers == std::vector<int>{0});
    REQUIRE(crossing.shells[3].containers == std::vector<int>{0});
}

TEST_CASE("Overlapping boxes with a cavity: the boxes cross, the cavity lies in one", "[diagnostics][shell-relations]") {
    const mesheal::ShellRelations relations = relationsOf(SampleMeshes::createOverlapPlusCavityMesh());
    REQUIRE(relations.shells.size() == 3);
    REQUIRE(relations.pairs.size() == 1);
    REQUIRE(relations.pair(0, 1)->crosses());
    REQUIRE_FALSE(relations.pair(0, 1)->sharesFaceArea());
    REQUIRE(relations.shells[2].nestedIn == 0);
    REQUIRE(relations.shells[2].containers == std::vector<int>{0});
    REQUIRE(relations.shells[0].signedVolume == Catch::Approx(64.0));
    REQUIRE(relations.shells[1].signedVolume == Catch::Approx(64.0));
    REQUIRE(relations.shells[2].signedVolume == Catch::Approx(-4.0));
}

TEST_CASE("A cavity reached by another shell is crossed by it or lies inside it", "[diagnostics][shell-relations]") {
    using Variant = SampleMeshes::CavityReachedVariant;
    const mesheal::ShellRelations crossed = relationsOf(SampleMeshes::createCavityReachedMesh(Variant::CrossesWall));
    REQUIRE(crossed.shells.size() == 3);
    REQUIRE(crossed.pair(0, 1)->crosses());
    REQUIRE(crossed.pair(1, 2) != nullptr);
    REQUIRE(crossed.pair(1, 2)->crosses()); // B's wall x = 3.5 passes through the cavity
    REQUIRE(crossed.pair(0, 2) == nullptr);
    REQUIRE(crossed.shells[2].nestedIn == 0);
    REQUIRE(crossed.shells[2].signedVolume == Catch::Approx(-3.2));
    REQUIRE(totalVolume(crossed) == Catch::Approx(124.8));

    const mesheal::ShellRelations inside = relationsOf(SampleMeshes::createCavityReachedMesh(Variant::InsideOverlap));
    REQUIRE(inside.pairs.size() == 1); // only A and B cross
    REQUIRE(inside.shells[2].containers == std::vector<int>{0, 1});
    REQUIRE(inside.shells[2].signedVolume == Catch::Approx(-0.3));

    const mesheal::ShellRelations separate = relationsOf(SampleMeshes::createCavityReachedMesh(Variant::WithSeparatePair));
    REQUIRE(separate.shells.size() == 5);
    REQUIRE(pairsWhere(separate, [](const mesheal::ShellPairRelation& r) { return r.crosses(); }) ==
            std::set<std::pair<int, int>>{{0, 1}, {1, 2}, {3, 4}});
}

TEST_CASE("Shell relations agree with the intersection check between shells", "[diagnostics][shell-relations]") {
    const auto agree = [](const mesheal::Mesh& mesh) {
        const std::vector<std::vector<Eigen::Index>> shells = mesheal::shellsLargestFirst(mesh);
        const mesheal::ShellRelations relations = mesheal::computeShellRelations(mesh, shells);
        const auto reported = pairsWhere(relations, [](const mesheal::ShellPairRelation& r) { return r.intersectingPairs > 0; });
        REQUIRE(reported == checkedBetweenPairs(mesh, shells));
        return relations;
    };

    const mesheal::Mesh selfIntersecting = mesheal::loadStl(examplePath("self_intersecting_box.stl"));
    REQUIRE_FALSE(mesheal::checkSelfIntersections(selfIntersecting, mesheal::CheckParams{}).empty());
    const mesheal::ShellRelations single = agree(selfIntersecting);
    REQUIRE(single.shells.size() == 1);
    REQUIRE(single.pairs.empty()); // its crossings are within its one shell

    const mesheal::ShellRelations turkeyLike = agree(mesheal::separateTouchingContacts(SampleMeshes::createTurkeyLikeMesh()));
    REQUIRE(turkeyLike.shells.size() == 9);
    REQUIRE(pairsWhere(turkeyLike, [](const mesheal::ShellPairRelation& r) { return r.crosses(); }) ==
            std::set<std::pair<int, int>>{{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});
    REQUIRE(turkeyLike.shells[7].nestedIn == 0); // the small box inside the body
    REQUIRE(turkeyLike.shells[8].nestedIn == -1); // the separate box
    for (int s = 1; s <= 6; ++s) {
        REQUIRE(turkeyLike.shells[static_cast<std::size_t>(s)].nestedIn == -1); // feathers cross, so are not nested
    }
}

TEST_CASE("Shell relations can be limited to pairs involving some shells", "[diagnostics][shell-relations]") {
    const mesheal::Mesh mesh = SampleMeshes::createTurkeyLikeMesh();
    mesheal::ShellRelationParams params;
    params.involving = {2};
    const mesheal::ShellRelations relations = mesheal::computeShellRelations(mesh, params);
    REQUIRE(relations.pairs.size() == 1);
    REQUIRE(relations.pair(0, 2) != nullptr);
    REQUIRE(relations.pair(0, 2)->crosses());

    params.involving = {8}; // the separate box meets nothing
    REQUIRE(mesheal::computeShellRelations(mesh, params).pairs.empty());
}

TEST_CASE("Disagreeing nesting probes leave a shell not nested", "[diagnostics][shell-relations]") {
    // An octahedron |x| + |y| + |z| <= 3 and a small box whose min corner (its first face's first corner,
    // the vertex probe) sits `offset` outside the face x + y + z = -3, while the rest of the box is inside
    const auto build = [](double offset) {
        mesheal::Mesh octahedron;
        octahedron.V.resize(6, 3);
        octahedron.V << 3, 0, 0, -3, 0, 0, 0, 3, 0, 0, -3, 0, 0, 0, 3, 0, 0, -3;
        octahedron.F.resize(8, 3);
        Eigen::Index f = 0;
        for (const int x : {0, 1}) {
            for (const int y : {2, 3}) {
                for (const int z : {4, 5}) {
                    const bool flip = ((x == 1) != (y == 3)) != (z == 5);
                    octahedron.F.row(f++) << x, flip ? z : y, flip ? y : z;
                }
            }
        }
        octahedron.recomputeNormals();
        const Eigen::Vector3d min = Eigen::Vector3d::Constant(-1.0 - offset);
        return SampleMeshes::combineMeshes({octahedron, box(min, min + Eigen::Vector3d::Constant(0.5))});
    };

    const mesheal::ShellRelations inside = mesheal::computeShellRelations(build(-1.0e-3));
    REQUIRE(inside.shells.size() == 2);
    REQUIRE(inside.shells[1].signedVolume == Catch::Approx(36.0)); // the octahedron, listed second (fewer faces)
    REQUIRE(inside.shells[0].nestedIn == 1);
    REQUIRE_FALSE(inside.shells[0].nestingUnsure);
    REQUIRE(inside.pairs.empty());

    // 1e-8 outside: well within the tolerance, so a contact, and only the vertex probe sees "outside"
    const mesheal::ShellRelations unsure = mesheal::computeShellRelations(build(1.0e-8));
    REQUIRE(unsure.shells[0].nestingUnsure);
    REQUIRE(unsure.shells[0].nestedIn == -1);
    REQUIRE(unsure.shells[0].containers.empty());
    REQUIRE(unsure.pairs.size() == 1);
    REQUIRE(unsure.pairs[0].edgeOrPointContactOnly());
}

TEST_CASE("Box with cavity, hole, slivers, and intersections has every defect", "[diagnostics][shell-relations]") {
    const mesheal::Mesh mesh = mesheal::separateTouchingContacts(SampleMeshes::createCavityHoleSliversIntersectionsMesh());
    const mesheal::ShellRelations relations = mesheal::computeShellRelations(mesh);
    REQUIRE(relations.shells.size() == 3);
    REQUIRE_FALSE(relations.shells[0].closed); // the hole
    REQUIRE(relations.shells[1].closed);
    REQUIRE(relations.shells[2].closed);
    REQUIRE(relations.shells[1].signedVolume == Catch::Approx(-8.0)); // the cavity
    REQUIRE(relations.shells[1].nestedIn == 0);
    REQUIRE(relations.pairs.size() == 1);
    REQUIRE(relations.pair(0, 2)->crosses()); // the peg

    // Within the body: the fold crosses its neighbours
    const std::vector<std::vector<Eigen::Index>> shells = mesheal::shellsLargestFirst(mesh);
    const mesheal::CategoryResult crossings = mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{});
    std::set<Eigen::Index> body(shells[0].begin(), shells[0].end());
    bool withinBody = false;
    for (std::size_t i = 0; i < crossings.size(); ++i) {
        for (const Eigen::Index other : crossings.relatedOf(i)) {
            withinBody = withinBody || (body.contains(crossings.elements[i].index) && body.contains(other));
        }
    }
    REQUIRE(withinBody);

    // Slivers at the default 100 : 1: one Cap and two Needles
    const mesheal::ThinFaceReport thin = mesheal::ThinFaceChecker::check(mesh);
    std::size_t needles = 0;
    std::size_t caps = 0;
    for (const mesheal::ThinFaceDefect& defect : thin.defects) {
        needles += mesheal::classifyThinDefect(defect) == mesheal::ThinFaceSubtype::Needle ? 1 : 0;
        caps += mesheal::classifyThinDefect(defect) == mesheal::ThinFaceSubtype::Cap ? 1 : 0;
    }
    REQUIRE(needles >= 2);
    REQUIRE(caps >= 1);
}

TEST_CASE("Shell relations stop when asked", "[diagnostics][shell-relations]") {
    std::stop_source stop;
    stop.request_stop();
    const mesheal::ShellRelations relations =
        mesheal::computeShellRelations(SampleMeshes::createTurkeyLikeMesh(), {}, stop.get_token());
    REQUIRE_FALSE(relations.complete);
    REQUIRE(relations.pairs.empty());
    REQUIRE(relations.shells.size() == 9); // per-shell facts are still filled in
}

TEST_CASE("Turkey shell relations", "[diagnostics][shell-relations][.slow]") {
    const mesheal::Mesh turkey = mesheal::loadStl(examplePath("Turkey.stl"));
    const std::vector<std::vector<Eigen::Index>> shells = mesheal::shellsLargestFirst(turkey);
    REQUIRE(shells.size() == 28);
    const auto start = std::chrono::steady_clock::now();
    const mesheal::ShellRelations relations = mesheal::computeShellRelations(turkey, shells);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(relations.complete);

    // Shells joined by crossings (union-find), the way a merge would group them
    std::vector<std::size_t> root(shells.size());
    std::iota(root.begin(), root.end(), std::size_t{0});
    const auto find = [&](std::size_t x) {
        while (root[x] != x) {
            x = root[x] = root[root[x]];
        }
        return x;
    };
    std::size_t crossBody = 0;
    std::size_t touchOnly = 0;
    std::size_t shared = 0;
    for (const mesheal::ShellPairRelation& r : relations.pairs) {
        crossBody += (r.a == 0 && r.crosses()) ? 1 : 0;
        touchOnly += r.edgeOrPointContactOnly() ? 1 : 0;
        shared += r.sharesFaceArea() ? 1 : 0;
        if (r.crosses()) {
            root[find(static_cast<std::size_t>(r.a))] = find(static_cast<std::size_t>(r.b));
        }
    }
    std::size_t largestGroup = 0;
    for (std::size_t s = 0; s < shells.size(); ++s) {
        std::size_t members = 0;
        for (std::size_t t = 0; t < shells.size(); ++t) {
            members += find(t) == find(s) ? 1 : 0;
        }
        largestGroup = std::max(largestGroup, members);
    }
    std::vector<int> nested;
    for (std::size_t s = 0; s < relations.shells.size(); ++s) {
        REQUIRE_FALSE(relations.shells[s].nestingUnsure);
        REQUIRE(relations.shells[s].closed);
        if (relations.shells[s].nestedIn >= 0) {
            nested.push_back(static_cast<int>(s));
        }
    }
    WARN("Turkey relations: " << relations.pairs.size() << " shell pairs meet, " << crossBody << " shells cross #1, "
                              << largestGroup << " shells joined by crossings, " << shared << " pairs share a face area, "
                              << touchOnly << " pairs touch only; #28 inside #" << relations.shells[27].nestedIn + 1
                              << " (signed volume " << relations.shells[27].signedVolume << "); " << ms << " ms");
    REQUIRE(touchOnly == 0);
    REQUIRE(largestGroup == 27); // every shell but the inner one
    REQUIRE(nested == std::vector<int>{27});
    REQUIRE(relations.shells[27].nestedIn == 11); // #28 inside #12, as the Shells panel shows
    REQUIRE(relations.shells[27].signedVolume < 0.0);
    REQUIRE(pairsWhere(relations, [](const mesheal::ShellPairRelation& r) { return r.intersectingPairs > 0; }) ==
            checkedBetweenPairs(turkey, shells));
}
