// Fix All phase 2: the merge plan (groups, kept cavities, refused groups, nested solids to delete,
// skipped shells, contacts, links outside the scope) and the default of the Merge shells box.

#include "repair/MergePlan.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/FixAll.hpp"
#include "repair/ShellMerge.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <optional>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

namespace {

using ogun::FixAllScope;
using ogun::FixAllScopeKind;
using ogun::MergePlan;
using ogun::MergePlanLineKind;
using ogun::MergeSkipReason;
using ogun::Mesh;
using ogun::SampleMeshes;
using Eigen::Vector3d;

Mesh box(const Vector3d& min, const Vector3d& max, bool inward = false) {
    return SampleMeshes::createBox(min, max, inward);
}

MergePlan plan(const Mesh& mesh) {
    return ogun::planMerge(mesh, FixAllScope::whole(mesh));
}

/// A scope of whole shells (indices of `shellsLargestFirst`).
FixAllScope shellScope(const Mesh& mesh, const std::vector<int>& shells) {
    const auto all = ogun::shellsLargestFirst(mesh);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const int s : shells) {
        for (const Eigen::Index f : all[static_cast<std::size_t>(s)]) {
            mask[static_cast<std::size_t>(f)] = 1;
        }
    }
    return FixAllScope::fromFaceMask(mesh, FixAllScopeKind::Shells, mask);
}

bool hasLine(const MergePlan& p, const std::string& text, MergePlanLineKind kind) {
    return std::any_of(p.lines.begin(), p.lines.end(),
                       [&](const ogun::MergePlanLine& line) { return line.text == text && line.kind == kind; });
}

bool lineContains(const MergePlan& p, const std::string& part) {
    return std::any_of(p.lines.begin(), p.lines.end(),
                       [&](const ogun::MergePlanLine& line) { return line.text.find(part) != std::string::npos; });
}

/// The mesh without the faces of the listed shells (`shellsLargestFirst` indices).
Mesh withoutShells(const Mesh& mesh, const std::vector<int>& drop) {
    const auto shells = ogun::shellsLargestFirst(mesh);
    std::vector<std::uint8_t> gone(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const int s : drop) {
        for (const Eigen::Index f : shells[static_cast<std::size_t>(s)]) {
            gone[static_cast<std::size_t>(f)] = 1;
        }
    }
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (gone[static_cast<std::size_t>(f)] == 0) {
            keep.push_back(f);
        }
    }
    return ogun::extractFaces(mesh, keep);
}

/// Faces of `mesh` except the last `n` rows.
Mesh dropLastFaces(const Mesh& mesh, Eigen::Index n) {
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f + n < mesh.numFaces(); ++f) {
        keep.push_back(f);
    }
    return ogun::extractFaces(mesh, keep);
}

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

} // namespace

TEST_CASE("Merge plan: overlapping boxes form one group and tick the box", "[repair][fixall][merge-plan]") {
    const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
    const MergePlan p = plan(mesh);
    REQUIRE(p.complete);
    CHECK(p.tickedByDefault);
    REQUIRE(p.groups.size() == 1);
    CHECK(p.groups[0].shells == std::vector<int>{0, 1});
    CHECK(p.groups[0].cavities.empty());
    CHECK(p.refusedGroups.empty());
    CHECK(p.skipped.empty());
    CHECK(p.isolatedNestedSolids.empty());
    CHECK(p.operandCount() == 2);
    CHECK(p.hasWork());
    CHECK(hasLine(p, "Merge 2 shells that overlap into 1 solid.", MergePlanLineKind::Info));
    CHECK(ogun::mergeTickedByDefault(mesh, FixAllScope::whole(mesh)) == std::optional<bool>(true));

    SECTION("Two separate assemblies: two groups") {
        const Mesh two = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3}),
                                                      box({10, 0, 0}, {12, 2, 2}), box({11.8, 0.2, 0.3}, {13.8, 2.2, 2.3})});
        const MergePlan q = plan(two);
        CHECK(q.tickedByDefault);
        REQUIRE(q.groups.size() == 2);
        CHECK(q.groups[0].shells == std::vector<int>{0, 1});
        CHECK(q.groups[1].shells == std::vector<int>{2, 3});
        CHECK(hasLine(q, "Merge 4 shells that overlap into 2 solids.", MergePlanLineKind::Info));
    }
}

TEST_CASE("Merge plan: shells that cannot be operands are skipped and named", "[repair][fixall][merge-plan]") {
    const Mesh a = box({0, 0, 0}, {2, 2, 2});
    const Mesh b = box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3});
    const Mesh third = box({-1, 0.4, 0.5}, {0.5, 1.4, 1.5});

    SECTION("An open shell among cubes") {
        const Mesh open = dropLastFaces(third, 1);
        REQUIRE_FALSE(ogun::manifoldAccepts(open, std::vector<Eigen::Index>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}).accepted);
        const Mesh mesh = SampleMeshes::combineMeshes({a, b, open});
        const MergePlan p = plan(mesh);
        CHECK(p.tickedByDefault);
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1});
        REQUIRE(p.skipped.size() == 1);
        CHECK(p.skipped[0].shell == 2);
        CHECK(p.skipped[0].why == MergeSkipReason::Open);
        CHECK(p.skipped[0].overlaps == std::vector<int>{0});
        CHECK(hasLine(p, "#3 cannot be merged yet: it is still open (3 open edges); it overlaps #1.", MergePlanLineKind::Warning));
    }

    SECTION("A shell with a flipped face") {
        Mesh flipped = third;
        std::swap(flipped.F(0, 1), flipped.F(0, 2));
        std::vector<Eigen::Index> all(12);
        std::iota(all.begin(), all.end(), 0);
        REQUIRE_FALSE(ogun::manifoldAccepts(flipped, all).accepted);
        const MergePlan p = plan(SampleMeshes::combineMeshes({a, b, flipped}));
        REQUIRE(p.skipped.size() == 1);
        CHECK(p.skipped[0].why == MergeSkipReason::Flipped);
        CHECK(lineContains(p, "#3 cannot be merged yet: its faces are not wound consistently"));
        CHECK(p.groups.size() == 1);
    }

    SECTION("A shell with a fin") {
        Mesh finned = third;
        // A fin triangle on the edge of faces 0 and 1 (three faces at that edge)
        const int v0 = finned.F(0, 0);
        const int v1 = finned.F(0, 1);
        finned.V.conservativeResize(finned.V.rows() + 1, 3);
        finned.V.row(finned.V.rows() - 1) = (finned.V.row(v0) + finned.V.row(v1)) / 2.0 + Eigen::RowVector3d(-0.3, 0.0, 0.0);
        finned.F.conservativeResize(finned.F.rows() + 1, 3);
        finned.F.row(finned.F.rows() - 1) << v0, v1, static_cast<int>(finned.V.rows() - 1);
        std::vector<Eigen::Index> all(13);
        std::iota(all.begin(), all.end(), 0);
        REQUIRE_FALSE(ogun::manifoldAccepts(finned, all).accepted);
        const MergePlan p = plan(SampleMeshes::combineMeshes({a, b, finned}));
        REQUIRE(p.skipped.size() == 1);
        CHECK(p.skipped[0].shell == 0); // 13 faces: largest first
        CHECK(p.skipped[0].why == MergeSkipReason::FinEdges);
        CHECK(lineContains(p, "#1 cannot be merged yet: it has 1 Fin / other edge"));
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{1, 2});
    }

    SECTION("An inside-out box crossing two good ones") {
        // Six of its eight corners lie outside every other closed shell: rule O calls it inside out
        const MergePlan p = plan(SampleMeshes::combineMeshes({a, b, box({-1, -1, 0.5}, {0.5, 0.5, 1.5}, true)}));
        REQUIRE(p.roles.shells[2].role == ogun::ShellRole::InsideOut);
        REQUIRE(p.skipped.size() == 1);
        CHECK(p.skipped[0].why == MergeSkipReason::InsideOut);
        CHECK(hasLine(p, "#3 cannot be merged yet: it is inside out (see Inverted shells); it overlaps #1.",
                      MergePlanLineKind::Warning));
        CHECK(p.groups.size() == 1);
        CHECK(p.refusedGroups.empty());
    }
}

TEST_CASE("Merge plan: a cavity is kept and never ticks the box", "[repair][fixall][merge-plan][cavity]") {
    SECTION("Fixture 1: cavity box") {
        const Mesh mesh = SampleMeshes::createCavityBoxMesh();
        const MergePlan p = plan(mesh);
        CHECK_FALSE(p.tickedByDefault);
        CHECK(p.groups.empty());
        CHECK_FALSE(p.hasWork());
        REQUIRE(p.cavitiesKept.size() == 1);
        CHECK(p.cavitiesKept[0].shell == 1);
        CHECK(p.cavitiesKept[0].container == 0);
        CHECK_FALSE(p.cavitiesKept[0].passThrough);
        CHECK(hasLine(p, "No shells overlap: nothing to merge.", MergePlanLineKind::Info));
        CHECK(hasLine(p, "1 cavity kept: #2.", MergePlanLineKind::Info));
    }

    SECTION("Fixture 2: overlap plus cavity") {
        const MergePlan p = plan(SampleMeshes::createOverlapPlusCavityMesh());
        CHECK(p.tickedByDefault);
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1});
        CHECK(p.groups[0].cavities == std::vector<int>{2});
        REQUIRE(p.cavitiesKept.size() == 1);
        CHECK(p.cavitiesKept[0].passThrough);
        CHECK(p.refusedGroups.empty());
        CHECK(hasLine(p, "Merge 2 shells that overlap into 1 solid (1 cavity kept).", MergePlanLineKind::Info));
    }

    SECTION("An island floating in a cavity is kept, not deleted") {
        const MergePlan p = plan(SampleMeshes::createIslandInCavityMesh());
        REQUIRE(p.roles.shells[2].role == ogun::ShellRole::Island);
        CHECK_FALSE(p.tickedByDefault);
        CHECK(p.isolatedNestedSolids.empty());
        CHECK(p.groups.empty());
        REQUIRE(p.cavitiesKept.size() == 1);
        CHECK(p.cavitiesKept[0].shell == 1);
    }

    SECTION("Inside out with a cavity: stopped and reported") {
        const MergePlan p = plan(SampleMeshes::createCavityBoxMesh(true));
        CHECK_FALSE(p.tickedByDefault);
        REQUIRE(p.stoppedGroups.size() == 1);
        CHECK(p.stoppedGroups[0] == std::vector<int>{0, 1});
        CHECK(p.groups.empty());
        CHECK(p.cavitiesKept.empty());
        CHECK(hasLine(p, "#1 and its inner shell #2 look inside out together: not merged. Flip them by hand (see Inverted shells).",
                      MergePlanLineKind::Warning));
    }
}

TEST_CASE("Merge plan: a group that reaches into a cavity is refused, other groups proceed",
          "[repair][fixall][merge-plan][cavity]") {
    using Variant = SampleMeshes::CavityReachedVariant;
    const std::string tail = ", so their group of 2 shells is not merged: Fix All cannot tell whether the hollow should "
                             "cut them or they should fill it. Merge them by hand.";

    SECTION("Fixture 3: B's wall crosses the cavity") {
        const MergePlan p = plan(SampleMeshes::createCavityReachedMesh(Variant::CrossesWall));
        CHECK(p.tickedByDefault);
        CHECK(p.groups.empty());
        CHECK_FALSE(p.hasWork());
        REQUIRE(p.refusedGroups.size() == 1);
        CHECK(p.refusedGroups[0].shells == std::vector<int>{0, 1});
        CHECK(p.refusedGroups[0].hollows == std::vector<int>{2});
        REQUIRE(p.refusedGroups[0].reaches.size() == 1);
        CHECK(p.refusedGroups[0].reaches[0].shell == 1);
        CHECK(p.refusedGroups[0].reason == "#2 reaches into cavity #3 of #1" + tail);
        CHECK(hasLine(p, "Nothing can be merged: every overlap is refused or skipped (below).", MergePlanLineKind::Info));
        CHECK(hasLine(p, "#2 reaches into cavity #3 of #1" + tail, MergePlanLineKind::Warning));
        CHECK(p.cavitiesKept.empty());
        CHECK(p.skipped.empty());
    }

    SECTION("Fixture 3b: the cavity lies wholly inside both boxes") {
        const MergePlan p = plan(SampleMeshes::createCavityReachedMesh(Variant::InsideOverlap));
        REQUIRE(p.roles.shells[2].role == ogun::ShellRole::Cavity);
        CHECK(p.groups.empty());
        REQUIRE(p.refusedGroups.size() == 1);
        CHECK(p.refusedGroups[0].shells == std::vector<int>{0, 1});
        CHECK(p.refusedGroups[0].hollows == std::vector<int>{2});
        REQUIRE(p.refusedGroups[0].reaches.size() == 1);
        const int container = p.relations.shells[2].nestedIn;
        CHECK(p.refusedGroups[0].reaches[0].shell == 1 - container);
        CHECK(lineContains(p, "reaches into cavity #3 of #" + std::to_string(container + 1)));
    }

    SECTION("Fixture 3c: the separate pair still merges") {
        const MergePlan p = plan(SampleMeshes::createCavityReachedMesh(Variant::WithSeparatePair));
        REQUIRE(p.refusedGroups.size() == 1);
        CHECK(p.refusedGroups[0].shells == std::vector<int>{0, 1});
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{3, 4});
        CHECK(hasLine(p, "Merge 2 shells that overlap into 1 solid.", MergePlanLineKind::Info));
        CHECK(hasLine(p, "#2 reaches into cavity #3 of #1" + tail, MergePlanLineKind::Warning));
    }
}

TEST_CASE("Merge plan: nested solids are deleted when they touch nothing, merged otherwise",
          "[repair][fixall][merge-plan]") {
    SECTION("Nested solid vs cavity") {
        const MergePlan p = plan(SampleMeshes::createNestedSolidAndCavityMesh());
        REQUIRE(p.roles.shells[2].role == ogun::ShellRole::NestedSolid);
        CHECK(p.tickedByDefault); // a nested solid is an overlap
        CHECK(p.groups.empty());
        REQUIRE(p.isolatedNestedSolids.size() == 1);
        CHECK(p.isolatedNestedSolids[0].shell == 2);
        CHECK(p.isolatedNestedSolids[0].container == 0);
        CHECK(p.hasWork());
        REQUIRE(p.cavitiesKept.size() == 1);
        CHECK(p.cavitiesKept[0].shell == 1);
        CHECK_FALSE(p.cavitiesKept[0].passThrough);
        CHECK(hasLine(p, "Delete nested solid #3 (wholly inside #1, touching nothing).", MergePlanLineKind::Info));
        CHECK(hasLine(p, "1 cavity kept: #2.", MergePlanLineKind::Info));
    }

    SECTION("Crossing variant: the nested solids become operands") {
        const MergePlan p = plan(SampleMeshes::createNestedSolidAndCavityMesh(true));
        CHECK(p.tickedByDefault);
        CHECK(p.isolatedNestedSolids.empty());
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 2, 3});
        CHECK(p.groups[0].cavities == std::vector<int>{1});
        CHECK(p.refusedGroups.empty());
    }

    SECTION("A nested solid that only touches another along an edge is merged, not deleted") {
        const Mesh mesh = SampleMeshes::combineMeshes(
            {box({0, 0, 0}, {10, 10, 10}), box({2, 2, 2}, {4, 4, 4}), box({4, 4, 2}, {6, 6, 4})});
        const MergePlan p = plan(mesh);
        REQUIRE(p.roles.shells[1].role == ogun::ShellRole::NestedSolid);
        REQUIRE(p.roles.shells[2].role == ogun::ShellRole::NestedSolid);
        REQUIRE(p.relations.pair(1, 2) != nullptr);
        REQUIRE(p.relations.pair(1, 2)->edgeOrPointContactOnly());
        CHECK(p.isolatedNestedSolids.empty());
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1, 2});
        CHECK(p.edgeOrPointContacts.empty()); // same group: nothing to report
    }

    SECTION("Turkey-like: body and feathers in one group, inner box deleted, separate box untouched") {
        const MergePlan p = plan(SampleMeshes::createTurkeyLikeMesh());
        CHECK(p.tickedByDefault);
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1, 2, 3, 4, 5, 6});
        REQUIRE(p.isolatedNestedSolids.size() == 1);
        CHECK(p.isolatedNestedSolids[0].shell == 7);
        CHECK(p.isolatedNestedSolids[0].container == 0);
        CHECK(p.skipped.empty());
        CHECK(p.refusedGroups.empty());
        CHECK(p.edgeOrPointContacts.empty());
        CHECK(hasLine(p, "Merge 7 shells that overlap into 1 solid.", MergePlanLineKind::Info));
        CHECK(hasLine(p, "Delete nested solid #8 (wholly inside #1, touching nothing).", MergePlanLineKind::Info));
    }

    SECTION("A nested solid in a refused group is not deleted") {
        // Fixture 3b plus a small box inside A only, touching nothing
        const Mesh mesh = SampleMeshes::combineMeshes(
            {SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::InsideOverlap),
             box({0.5, 0.5, 0.5}, {1, 1, 1})});
        const MergePlan p = plan(mesh);
        REQUIRE(p.roles.shells[3].role == ogun::ShellRole::NestedSolid);
        CHECK(p.isolatedNestedSolids.empty());
        REQUIRE(p.refusedGroups.size() == 1);
        CHECK(p.refusedGroups[0].shells == std::vector<int>{0, 1, 3});
        CHECK(lineContains(p, "so their group of 3 shells is not merged"));
    }
}

TEST_CASE("Merge plan: contacts along an edge are reported, a shared face area is merged",
          "[repair][fixall][merge-plan]") {
    SECTION("4b: touching along an edge") {
        const Mesh mesh = SampleMeshes::createEdgeTouchingBoxesMesh();
        const MergePlan p = plan(mesh);
        CHECK_FALSE(p.tickedByDefault);
        CHECK(p.groups.empty());
        REQUIRE(p.edgeOrPointContacts.size() == 1);
        CHECK(p.edgeOrPointContacts[0].a == 0);
        CHECK(p.edgeOrPointContacts[0].b == 1);
        CHECK(hasLine(p, "No shells overlap: nothing to merge.", MergePlanLineKind::Info));
        CHECK(hasLine(p, "#1 and #2 touch along an edge or at a point only and cannot be one solid: left as they are.",
                      MergePlanLineKind::Info));
        CHECK(ogun::mergeTickedByDefault(mesh, FixAllScope::whole(mesh)) == std::optional<bool>(false));
    }

    SECTION("4c: touching across a face area") {
        const MergePlan p = plan(SampleMeshes::createFaceTouchingBoxesMesh());
        CHECK(p.tickedByDefault);
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1});
        CHECK(p.edgeOrPointContacts.empty());
    }
}

TEST_CASE("Merge plan follows the selection", "[repair][fixall][merge-plan][scope]") {
    SECTION("S1: links only between selected shells, the outside link reported") {
        const Mesh mesh = SampleMeshes::combineMeshes(
            {box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3}), box({3.6, 0.4, 0.5}, {5.6, 2.4, 2.5})});
        const MergePlan p = ogun::planMerge(mesh, shellScope(mesh, {0, 1}));
        CHECK(p.shellInScope == std::vector<std::uint8_t>{1, 1, 0});
        CHECK(p.tickedByDefault);
        REQUIRE(p.groups.size() == 1);
        CHECK(p.groups[0].shells == std::vector<int>{0, 1});
        REQUIRE(p.outsideScopeLinks.size() == 1);
        CHECK(p.outsideScopeLinks[0].shell == 1);
        CHECK(p.outsideScopeLinks[0].outside == 2);
        CHECK(p.outsideScopeLinks[0].kind == ogun::MergeOutsideLinkKind::Crosses);
        CHECK(hasLine(p, "#2 still crosses #3, which is not selected: select both to merge them.", MergePlanLineKind::Warning));

        // Only the unlinked pair selected: nothing overlaps in scope
        const MergePlan q = ogun::planMerge(mesh, shellScope(mesh, {0, 2}));
        CHECK_FALSE(q.tickedByDefault);
        CHECK(q.groups.empty());
        CHECK(q.outsideScopeLinks.size() == 2);
    }

    SECTION("A selected nested solid inside an unselected container is not deleted") {
        const Mesh mesh = SampleMeshes::createTurkeyLikeMesh();
        const MergePlan p = ogun::planMerge(mesh, shellScope(mesh, {7, 8}));
        CHECK(p.isolatedNestedSolids.empty());
        CHECK_FALSE(p.tickedByDefault);
        CHECK(hasLine(p, "#8 lies inside #1, which is not selected: not deleted (select #1 too).", MergePlanLineKind::Warning));

        const MergePlan q = ogun::planMerge(mesh, shellScope(mesh, {0, 1}));
        REQUIRE(q.groups.size() == 1);
        CHECK(q.groups[0].shells == std::vector<int>{0, 1});
        CHECK(q.isolatedNestedSolids.empty());
        CHECK(hasLine(q, "#1 holds #8, which is not selected: left as it is.", MergePlanLineKind::Warning));
        CHECK(lineContains(q, "#1 still crosses #3, which is not selected"));
    }

    SECTION("An unselected cavity reached by selected shells still refuses their group") {
        const Mesh mesh = SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::CrossesWall);
        const MergePlan p = ogun::planMerge(mesh, shellScope(mesh, {0, 1}));
        CHECK(p.groups.empty());
        REQUIRE(p.refusedGroups.size() == 1);
        CHECK(p.refusedGroups[0].hollows == std::vector<int>{2});
    }

    SECTION("A face scope: no merge, the box is not ticked") {
        const Mesh mesh = SampleMeshes::createOverlapPlusCavityMesh();
        std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 0);
        mask[0] = 1;
        const FixAllScope faces = FixAllScope::fromFaceMask(mesh, FixAllScopeKind::Faces, mask);
        const MergePlan p = ogun::planMerge(mesh, faces);
        CHECK(p.needsWholeShells);
        CHECK_FALSE(p.tickedByDefault);
        CHECK_FALSE(p.hasWork());
        CHECK(p.shells.empty());
        CHECK(hasLine(p, "Merge needs whole shells: select shells, or clear the selection.", MergePlanLineKind::Info));
        CHECK(ogun::mergeTickedByDefault(mesh, faces) == std::optional<bool>(false));
    }

    SECTION("Stopped") {
        const Mesh mesh = SampleMeshes::createTurkeyLikeMesh();
        std::stop_source source;
        source.request_stop();
        const MergePlan p = ogun::planMerge(mesh, FixAllScope::whole(mesh), {}, source.get_token());
        CHECK_FALSE(p.complete);
        CHECK(p.groups.empty());
        CHECK_FALSE(ogun::mergeTickedByDefault(mesh, FixAllScope::whole(mesh), {}, source.get_token()).has_value());
    }
}

TEST_CASE("Turkey merge plan: the group reaching cavity #28 is refused; without #28 it merges",
          "[repair][fixall][merge-plan][.slow]") {
    const Mesh turkey = ogun::loadStl(examplePath("Turkey.stl"));
    const auto start = std::chrono::steady_clock::now();
    const MergePlan p = plan(turkey);
    const double planMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(p.complete);
    REQUIRE(p.shells.size() == 28);
    REQUIRE(p.roles.shells[27].role == ogun::ShellRole::Cavity);
    CHECK(p.relations.shells[27].nestedIn == 11);
    CHECK(p.tickedByDefault);
    CHECK(p.groups.empty());
    CHECK(p.isolatedNestedSolids.empty());
    CHECK(p.skipped.empty());
    CHECK(p.cavitiesKept.empty());
    REQUIRE(p.refusedGroups.size() == 1);
    CHECK(p.refusedGroups[0].shells.size() == 27);
    CHECK(p.refusedGroups[0].hollows == std::vector<int>{27});
    CHECK(p.refusedGroups[0].reason.starts_with("#6 and #9 reach into cavity #28 of #12, so their group of 27 shells is not merged"));
    std::cout << "Turkey merge plan: " << planMs << " ms\n";

    // The user deletes #28 by hand: the 27 shells form one group and merge into 1
    const Mesh without = withoutShells(turkey, {27});
    const MergePlan q = plan(without);
    REQUIRE(q.complete);
    REQUIRE(q.shells.size() == 27);
    CHECK(q.tickedByDefault);
    CHECK(q.refusedGroups.empty());
    CHECK(q.skipped.empty());
    REQUIRE(q.groups.size() == 1);
    CHECK(q.groups[0].shells.size() == 27);
    CHECK(hasLine(q, "Merge 27 shells that overlap into 1 solid.", MergePlanLineKind::Info));

    ogun::ShellMergeInput input;
    for (const int s : q.groups[0].shells) {
        input.operands.push_back(q.shells[static_cast<std::size_t>(s)]);
    }
    const ogun::ShellMergeResult merged = ogun::mergeShells(without, input);
    REQUIRE(merged.success);
    CHECK(merged.operandsMerged == 27);
    CHECK(merged.refusedGroups.empty());
    CHECK(merged.shellsOut == 1);
    CHECK(ogun::checkShells(merged.mesh).infoCount == 1);
    std::cout << "Turkey without #28: plan " << q.elapsedMs << " ms, merge " << merged.elapsedMs << " ms, faces "
              << without.numFaces() << " -> " << merged.mesh.numFaces() << "\n";
}
