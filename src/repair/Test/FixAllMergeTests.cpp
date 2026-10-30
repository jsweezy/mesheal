// Fix All phase 2: the merge plan (groups, kept cavities, refused groups, nested solids to delete,
// skipped shells, contacts, links outside the scope), the default of the Merge shells box, and the merge
// run inside each pass with its guards, followed by the phase-3 clean-up.

#include "repair/MergePlan.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/FixAll.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/ShellMerge.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
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

// ---------------------------------------------------------------------------------------------
// Phase 2 in the pass: the merge runs, guarded, and phase 3 cleans up after it
// ---------------------------------------------------------------------------------------------

namespace {

using ogun::FixAllGuard;
using ogun::FixAllPhase;
using ogun::FixAllResult;
using ogun::FixAllStage;
using ogun::FixAllStageId;
using ogun::FixAllStepReport;
using ogun::FixAllStepStatus;
using ogun::FixAllStopReason;

ogun::FixAllParams mergeOn(int passes = ogun::settings::kDefaultFixAllPasses) {
    ogun::FixAllParams params;
    params.mergeShells = true;
    params.passes = passes;
    return params;
}

FixAllResult runAll(const Mesh& mesh, const ogun::FixAllParams& params, const ogun::FixAllTestHooks* hooks = nullptr) {
    return ogun::runFixAll(mesh, FixAllScope::whole(mesh), params, {}, nullptr, hooks);
}

const FixAllStepReport* stepIn(const FixAllResult& result, FixAllStageId id, int pass = 1) {
    const auto it = std::find_if(result.steps.begin(), result.steps.end(),
                                 [&](const FixAllStepReport& step) { return step.stage == id && step.pass == pass; });
    return it == result.steps.end() ? nullptr : &*it;
}

std::size_t shellCount(const Mesh& mesh) {
    return ogun::shellsLargestFirst(mesh).size();
}

bool same(const Mesh& a, const Mesh& b) {
    return a.V.rows() == b.V.rows() && a.F.rows() == b.F.rows() && a.V == b.V && a.F == b.F;
}

/// Every face of `part` is in `mesh` with the same corners (the part was kept bit-identical).
bool keeps(const Mesh& mesh, const Mesh& part) {
    std::vector<ogun::FaceGeometryKey> keys;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        keys.push_back(ogun::faceGeometryKey(mesh, f));
    }
    for (Eigen::Index f = 0; f < part.numFaces(); ++f) {
        if (std::find(keys.begin(), keys.end(), ogun::faceGeometryKey(part, f)) == keys.end()) {
            return false;
        }
    }
    return true;
}

bool notesContain(const FixAllStepReport* step, const std::string& text) {
    return step != nullptr && std::any_of(step->notes.begin(), step->notes.end(),
                                          [&](const ogun::FixAllShellNote& note) { return note.text == text; });
}

std::size_t phaseSteps(const FixAllResult& result, FixAllPhase phase, int pass) {
    return static_cast<std::size_t>(std::count_if(result.steps.begin(), result.steps.end(), [&](const FixAllStepReport& step) {
        return step.phase == phase && step.pass == pass;
    }));
}

std::size_t mergeStepsKept(const FixAllResult& result, int pass) {
    return static_cast<std::size_t>(std::count_if(result.steps.begin(), result.steps.end(), [&](const FixAllStepReport& step) {
        return step.stage == FixAllStageId::MergeShells && step.pass == pass && step.accepted();
    }));
}

/// The same stage list with every stage `id` removed in every phase.
ogun::FixAllTestHooks without(std::vector<FixAllStageId> ids) {
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [ids = std::move(ids)](int, FixAllPhase, std::vector<FixAllStage>& stages) {
        std::erase_if(stages, [&](const FixAllStage& s) { return std::find(ids.begin(), ids.end(), s.id) != ids.end(); });
    };
    return hooks;
}

} // namespace

TEST_CASE("Fix All merges two overlapping closed shells and cleans up after the union", "[repair][fixall][merge]") {
    const Mesh a = box({0, 0, 0}, {2, 2, 2});
    const Mesh b = box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3});
    const Mesh mesh = SampleMeshes::combineMeshes({a, b});
    const double overlap = 0.2 * 1.8 * 1.7;
    REQUIRE(ogun::mergeTickedByDefault(mesh, FixAllScope::whole(mesh)) == std::optional<bool>(true));

    SECTION("Merge on: 2 shells -> 1, no crossings, closed, the overlap counted once") {
        ogun::FixAllProgress progress;
        std::vector<std::string> texts;
        progress.observer = [&](float, const std::string& text) { texts.push_back(text); };
        const FixAllResult result = ogun::runFixAll(mesh, FixAllScope::whole(mesh), mergeOn(), {}, &progress);
        REQUIRE(result.success);
        CHECK(shellCount(result.mesh) == 1);
        CHECK(result.countsBefore.intersectingFaces > 0);
        CHECK(result.countsAfter.intersectingFaces == 0);
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(result.countsAfter.nonManifoldEdges == 0);
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(16.0 - overlap));
        CHECK(result.merges == 1);
        REQUIRE_FALSE(result.passes.empty());
        CHECK(result.passes[0].groupsMerged == 1);
        CHECK(result.passes[0].merged());
        CHECK(result.stopReason == FixAllStopReason::Clean);

        const FixAllStepReport* planned = stepIn(result, FixAllStageId::PlanMerge);
        REQUIRE(planned != nullptr);
        CHECK(planned->phase == FixAllPhase::Merge);
        CHECK(planned->detail == "Merge 2 shells that overlap into 1 solid.");
        REQUIRE(planned->notes.size() == 1);
        CHECK(planned->notes[0].shells.size() == 2);
        const FixAllStepReport* merge = stepIn(result, FixAllStageId::MergeShells);
        REQUIRE(merge != nullptr);
        REQUIRE(merge->accepted());
        CHECK(merge->detail.starts_with("Merged 2 shells into 1 ("));
        CHECK(merge->detail.find("crossing faces ") != std::string::npos);
        CHECK(merge->countBefore == 2);
        CHECK(merge->countAfter == 1);
        CHECK(notesContain(merge, "Merged 2 shells into #1"));
        // Phase 3 ran after the merge
        CHECK(phaseSteps(result, FixAllPhase::Cleanup, 1) == ogun::fixAllCleanupStages().size());
        // Faces the union removed, in the input's numbering
        CHECK_FALSE(result.mergeRemovedFaces.empty());
        CHECK(std::is_sorted(result.mergeRemovedFaces.begin(), result.mergeRemovedFaces.end()));
        CHECK(result.mergeRemovedFaces.back() < mesh.numFaces());
        CHECK(result.scope.matches(result.mesh));
        CHECK(std::find(texts.begin(), texts.end(), "pass 1 of 3, phase 2, planning the merge...") != texts.end());
        CHECK(std::find(texts.begin(), texts.end(), "pass 1 of 3, phase 2, merging 2 shells (Manifold)...") != texts.end());

        // Thin faces are no worse than right after the union
        const ogun::ShellMergeResult direct = ogun::mergeShells(mesh, ogun::shellsLargestFirst(mesh));
        REQUIRE(direct.success);
        const ogun::FixAllCounts union0 =
            ogun::countFixAllDefects(direct.mesh, FixAllScope::whole(direct.mesh), ogun::FixAllParams{}.checkParams);
        CHECK(result.countsAfter.thinFaces <= union0.thinFaces);
    }
    SECTION("Merge off: two shells, the crossings stay and are counted") {
        const FixAllResult result = runAll(mesh, {});
        CHECK(shellCount(result.mesh) == 2);
        CHECK(result.countsAfter.intersectingFaces > 0);
        CHECK(same(result.mesh, mesh));
        CHECK(result.merges == 0);
        CHECK(phaseSteps(result, FixAllPhase::Merge, 1) == 0);
        CHECK(result.mergeRemovedFaces.empty());
    }
}

TEST_CASE("Fix All merges the closed shells and leaves an open one it cannot close", "[repair][fixall][merge]") {
    const Mesh mesh = SampleMeshes::createOpenShellAmongCubesMesh();
    REQUIRE(shellCount(mesh) == 3);
    const Mesh open = withoutShells(mesh, {0, 1});
    ogun::FixAllParams params = mergeOn();
    params.holeFill.maxHoleEdges = 3; // its 4-edge hole cannot be filled
    const FixAllResult result = runAll(mesh, params);
    REQUIRE(result.success);
    CHECK(shellCount(result.mesh) == 2);
    CHECK(keeps(result.mesh, open)); // bit-identical
    CHECK(result.countsAfter.openEdges == 4);
    CHECK(result.merges == 1);
    CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge, 1),
                       "#3 cannot be merged yet: it is still open (4 open edges); it overlaps #1."));
    for (int pass = 2; pass <= result.passesRun; ++pass) { // the union is #1 now, the open box #2
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge, pass),
                           "#2 cannot be merged yet: it is still open (4 open edges); it overlaps #1."));
    }
    CHECK(result.passesRun == 2);
    CHECK(result.stopReason == FixAllStopReason::RepeatedState);
    CHECK(result.countsAfter.intersectingFaces > 0); // the open box still crosses the union
    // The union of the two closed boxes: 64 + 64 - 6.125
    const Mesh merged = withoutShells(result.mesh, {1});
    CHECK(ogun::signedVolume(merged) == Catch::Approx(121.875));
}

TEST_CASE("A merge the plan or the merge guards refuse changes nothing", "[repair][fixall][merge]") {
    // An inside-out box crossing a good one, with orientation hooked off: rule O calls it inside out
    const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({-1, -1, 0.5}, {0.5, 0.5, 1.5}, true)});
    const ogun::FixAllTestHooks noOrient = without({FixAllStageId::Orient, FixAllStageId::OrientAgain});

    SECTION("The plan skips the inside-out box: no group, nothing merged") {
        const FixAllResult result = runAll(mesh, mergeOn(1), &noOrient);
        REQUIRE(result.success);
        CHECK(same(result.mesh, mesh));
        CHECK(stepIn(result, FixAllStageId::MergeShells) == nullptr);
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge),
                           "#2 cannot be merged yet: it is inside out (see Inverted shells); it overlaps #1."));
    }
    SECTION("Forced into one group anyway, the merge guard refuses it") {
        MergePlan forced = plan(mesh);
        REQUIRE(forced.groups.empty());
        forced.groups.push_back({{0, 1}, {}});
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [&](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
            if (phase == FixAllPhase::Repair) {
                stages.clear();
            } else if (phase == FixAllPhase::Merge) {
                stages = ogun::fixAllMergeStages(forced);
            }
        };
        const FixAllResult result = runAll(mesh, mergeOn(1), &hooks);
        const FixAllStepReport* merge = stepIn(result, FixAllStageId::MergeShells);
        REQUIRE(merge != nullptr);
        CHECK(merge->status == FixAllStepStatus::Refused);
        CHECK(merge->guard == FixAllGuard::Merge);
        CHECK(merge->reason == "#1 reaches into #2, which is wound inward: fix its orientation, or merge them by hand.");
        CHECK(same(result.mesh, mesh));
        CHECK(stepIn(result, FixAllStageId::Slivers) == nullptr); // no phase 3
    }
    SECTION("Volume sanity") {
        const std::vector<double> none;
        CHECK(ogun::mergeVolumeProblem(std::vector<double>{8.0, 8.0}, 15.388, none, 15.388).empty());
        CHECK(ogun::mergeVolumeProblem(std::vector<double>{8.0, 8.0}, 17.0, none, 17.0).starts_with("The union's volume (17) is above"));
        CHECK(ogun::mergeVolumeProblem(std::vector<double>{8.0, 8.0}, 7.0, none, 7.0).starts_with("The union's volume (7) is below"));
        // An inside-out shell counted as a solid: the union exceeds the sum
        CHECK_FALSE(ogun::mergeVolumeProblem(std::vector<double>{8.0, -1.0}, 8.0, none, 8.0).empty());
        // Overlap plus cavity: 121.875 - 4 = 117.875
        CHECK(ogun::mergeVolumeProblem(std::vector<double>{64.0, 64.0}, 121.875, std::vector<double>{-4.0}, 117.875).empty());
        CHECK(ogun::mergeVolumeProblem(std::vector<double>{64.0, 64.0}, 121.875, std::vector<double>{-4.0}, 121.875)
                  .starts_with("The merged shells' signed volume (121.875) is not the union's less its cavities (117.875)"));
    }
}

TEST_CASE("Fix All on the Turkey-like assembly: feathers merged, inner box deleted, separate box untouched",
          "[repair][fixall][merge]") {
    const Mesh mesh = SampleMeshes::createTurkeyLikeMesh();
    REQUIRE(shellCount(mesh) == 9);
    const Mesh separate = withoutShells(mesh, {0, 1, 2, 3, 4, 5, 6, 7});
    const FixAllResult result = runAll(mesh, mergeOn());
    REQUIRE(result.success);
    CHECK(shellCount(result.mesh) == 2);
    CHECK(keeps(result.mesh, separate));
    CHECK(result.countsAfter.openEdges == 0);
    CHECK(result.countsAfter.intersectingFaces == 0);
    CHECK(result.stopReason == FixAllStopReason::Clean);
    const FixAllStepReport* deleted = stepIn(result, FixAllStageId::DeleteNestedSolids);
    REQUIRE(deleted != nullptr);
    REQUIRE(deleted->accepted());
    CHECK(deleted->detail == "Deleted nested solid #8 (wholly inside #1, touching nothing)");
    REQUIRE(deleted->notes.size() == 1);
    CHECK(deleted->notes[0].shells.size() == 2);
    CHECK(deleted->notes[0].shells[0].empty()); // deleted
    // The container's faces, carried to the result: the union kept only some of the body's 12 faces as they were
    CHECK(deleted->notes[0].shells[1].size() <= 12);
    CHECK_FALSE(deleted->notes[0].shells[1].empty());
    CHECK(result.passes[0].nestedSolidsDeleted == 1);
    CHECK(result.passes[0].groupsMerged == 1);
    const FixAllStepReport* merge = stepIn(result, FixAllStageId::MergeShells);
    REQUIRE(merge != nullptr);
    CHECK(merge->detail.starts_with("Merged 7 shells into 1 ("));
    // The inner box's 12 faces are among the faces phase 2 removed
    CHECK(result.mergeRemovedFaces.size() >= 12);

    SECTION("S3: nothing selected gives the same result as every shell selected") {
        std::vector<std::uint8_t> all(static_cast<std::size_t>(mesh.numFaces()), 1);
        const FixAllResult selected =
            ogun::runFixAll(mesh, FixAllScope::fromFaceMask(mesh, FixAllScopeKind::Shells, all), mergeOn());
        CHECK(ogun::meshGeometryHash(selected.mesh) == ogun::meshGeometryHash(result.mesh));
        CHECK(selected.passesRun == result.passesRun);
    }
}

TEST_CASE("Fix All keeps every cavity through the merge", "[repair][fixall][merge][cavity]") {
    SECTION("Nested solid vs cavity: the nested solid is deleted, 3 -> 2, volume 63") {
        const Mesh mesh = SampleMeshes::createNestedSolidAndCavityMesh();
        const Mesh body = withoutShells(mesh, {1, 2});
        const Mesh cavity = withoutShells(mesh, {0, 2});
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 2);
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(63.0));
        CHECK(keeps(result.mesh, body));
        CHECK(keeps(result.mesh, cavity));
        CHECK(result.countsAfter.invertedShells == 0);
        CHECK(stepIn(result, FixAllStageId::DeleteNestedSolids)->accepted());
        CHECK(stepIn(result, FixAllStageId::MergeShells) == nullptr);
    }
    SECTION("Fixture 1: the box is ticked by hand, nothing to merge") {
        const Mesh mesh = SampleMeshes::createCavityBoxMesh();
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(same(result.mesh, mesh));
        CHECK(result.passesRun == 1);
        CHECK(result.stopReason == FixAllStopReason::Clean);
        CHECK(stepIn(result, FixAllStageId::PlanMerge)->detail == "No shells overlap: nothing to merge. 1 cavity kept: #2.");
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(56.0));
    }
    SECTION("Fixture 2: A and B united around the cavity, 3 -> 2, signed volume 117.875") {
        const Mesh mesh = SampleMeshes::createOverlapPlusCavityMesh();
        const Mesh cavity = withoutShells(mesh, {0, 1});
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 2);
        CHECK(keeps(result.mesh, cavity));
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(117.875));
        CHECK(ogun::signedVolume(withoutShells(result.mesh, {0})) == Catch::Approx(-4.0)); // the cavity is #2 now
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(result.countsAfter.intersectingFaces == 0);
        CHECK(result.countsAfter.invertedShells == 0);
        const FixAllStepReport* merge = stepIn(result, FixAllStageId::MergeShells);
        REQUIRE(merge != nullptr);
        CHECK(merge->detail.ends_with("; 1 cavity kept as it was: #3"));
        CHECK(notesContain(merge, "Kept cavity #2 as it was"));
    }
    SECTION("Fixtures 3 and 3b: the group reaching into the cavity is refused, nothing changes") {
        using Variant = SampleMeshes::CavityReachedVariant;
        for (const Variant variant : {Variant::CrossesWall, Variant::InsideOverlap}) {
            const Mesh mesh = SampleMeshes::createCavityReachedMesh(variant);
            const FixAllResult result = runAll(mesh, mergeOn());
            CHECK(same(result.mesh, mesh));
            CHECK(shellCount(result.mesh) == 3);
            CHECK(result.countsAfter.intersectingFaces > 0);
            CHECK(stepIn(result, FixAllStageId::MergeShells) == nullptr);
            const FixAllStepReport* planned = stepIn(result, FixAllStageId::PlanMerge);
            REQUIRE(planned != nullptr);
            CHECK(std::any_of(planned->notes.begin(), planned->notes.end(), [](const ogun::FixAllShellNote& note) {
                return note.warning && note.text.find("reaches into cavity #3 of #") != std::string::npos;
            }));
        }
        CHECK(ogun::signedVolume(SampleMeshes::createCavityReachedMesh()) == Catch::Approx(124.8));
    }
    SECTION("Fixture 3c: the separate pair is still merged, 5 -> 4") {
        const Mesh mesh = SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::WithSeparatePair);
        const Mesh refused = withoutShells(mesh, {3, 4});
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 4);
        CHECK(keeps(result.mesh, refused));
        CHECK(mergeStepsKept(result, 1) == 1);
    }
}

TEST_CASE("Fix All always fills a hole whose patch crosses another shell; the merge unites them",
          "[repair][fixall][merge]") {
    using Variant = SampleMeshes::FillCrossesVariant;
    const Mesh mesh = SampleMeshes::createFillCrossesShellMesh(Variant::PlateThroughOpenBox);
    REQUIRE(shellCount(mesh) == 3);
    REQUIRE(ogun::mergeTickedByDefault(mesh, FixAllScope::whole(mesh)) == std::optional<bool>(true));

    SECTION("Fixture 4, merge on: 3 -> 1, the body's volume, one merge, one pass") {
        const FixAllResult result = runAll(mesh, mergeOn());
        const FixAllStepReport* fill = stepIn(result, FixAllStageId::FillHoles);
        REQUIRE(fill != nullptr);
        CHECK(fill->accepted());
        CHECK(std::any_of(fill->notes.begin(), fill->notes.end(), [](const ogun::FixAllShellNote& note) {
            return note.text.ends_with("left for the merge");
        }));
        CHECK(shellCount(result.mesh) == 1);
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(216.0));
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(result.countsAfter.intersectingFaces == 0);
        CHECK(result.merges == 1);
        CHECK(result.passesRun == 1);
        CHECK(result.stopReason == FixAllStopReason::Clean);
    }
    SECTION("Fixture 4, merge off: the fill is kept, D is closed, the crossing is reported") {
        const FixAllResult result = runAll(mesh, {});
        const FixAllStepReport* fill = stepIn(result, FixAllStageId::FillHoles);
        REQUIRE(fill != nullptr);
        CHECK(fill->accepted());
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(shellCount(result.mesh) == 3);
        CHECK(result.countsAfter.intersectingFaces > 0);
        REQUIRE_FALSE(fill->deferred.empty());
        CHECK(fill->deferred.back().kind == ogun::FixAllDeferredKind::CrossingsWithShell);
        CHECK(fill->deferred.back().text.ends_with("with other shells: reported (the merge is off)"));
    }
    SECTION("4d: the plate not selected; the open box is filled and, a nested solid touching something, merged into the body") {
        const FixAllResult result = ogun::runFixAll(mesh, shellScope(mesh, {0, 2}), mergeOn());
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(keeps(result.mesh, withoutShells(mesh, {0, 2}))); // the plate is bit-identical
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge),
                           "#3 still crosses #2, which is not selected: select both to merge them."));
        CHECK(shellCount(result.mesh) == 2);
        CHECK(result.merges == 1);
    }
    SECTION("4e: the plate is open and its fill hooked off: the box is filled, the plate skipped and named") {
        const Mesh openPlate = SampleMeshes::createFillCrossesShellMesh(Variant::OpenPlate);
        const Mesh plate = withoutShells(openPlate, {0, 2}); // 11 faces: #2 by size until the open box is filled
        REQUIRE(plate.numFaces() == 11);
        // The fill runs on everything but the plate
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [](int, FixAllPhase, std::vector<FixAllStage>& stages) {
            for (FixAllStage& stage : stages) {
                if (stage.id != FixAllStageId::FillHoles) {
                    continue;
                }
                stage.run = [fill = stage.run](const Mesh& m, const ogun::FixAllStageContext& context) {
                    FixAllScope scope = context.scope;
                    const auto shells = ogun::shellsLargestFirst(m);
                    for (const auto& shell : shells) {
                        if (shell.size() == 11) {
                            for (const Eigen::Index f : shell) {
                                scope.faceInScope[static_cast<std::size_t>(f)] = 0;
                            }
                        }
                    }
                    scope.updatePins(m);
                    return fill(m, ogun::FixAllStageContext{context.params, scope, context.pass, context.phase,
                                                            context.stopToken, context.preMerge, context.planFaceMap});
                };
            }
        };
        const FixAllResult result = runAll(openPlate, mergeOn(), &hooks);
        CHECK(keeps(result.mesh, plate));
        CHECK(result.countsAfter.openEdges == 3); // the plate's hole
        const FixAllStepReport* planned = stepIn(result, FixAllStageId::PlanMerge);
        REQUIRE(planned != nullptr);
        CHECK(std::any_of(planned->notes.begin(), planned->notes.end(), [](const ogun::FixAllShellNote& note) {
            return note.warning && note.text.starts_with("#3 cannot be merged yet: it is still open (3 open edges)"); // the filled box has 12 faces now
        }));
        CHECK(shellCount(result.mesh) == 2);
    }
    SECTION("4a: a box closed only in pass 2 is merged in pass 2") {
        // A duplicate face gives pass 1 something to repair, so it is progress; the fill is off in pass 1
        Mesh crossing = SampleMeshes::createFillCrossesShellMesh(Variant::OpenBoxThroughBody);
        crossing.F.conservativeResize(crossing.F.rows() + 1, 3);
        crossing.F.row(crossing.F.rows() - 1) = crossing.F.row(0);
        crossing.recomputeNormals();
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
            if (pass == 1 && phase == FixAllPhase::Repair) {
                std::erase_if(stages, [](const FixAllStage& s) { return s.id == FixAllStageId::FillHoles; });
            }
        };
        const FixAllResult result = runAll(crossing, mergeOn(), &hooks);
        CHECK(result.passesRun == 2);
        CHECK(mergeStepsKept(result, 1) == 0);
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge, 1),
                           "#2 cannot be merged yet: it is still open (4 open edges); it overlaps #1."));
        CHECK(mergeStepsKept(result, 2) == 1);
        CHECK(result.merges == 1);
        CHECK(shellCount(result.mesh) == 1);
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(result.stopReason == FixAllStopReason::Clean);
    }
}

TEST_CASE("Fix All merges across a shared face area, never along an edge", "[repair][fixall][merge]") {
    SECTION("4b: touching along an edge: no merge in any pass") {
        const Mesh mesh = SampleMeshes::createEdgeTouchingBoxesMesh();
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 2);
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(16.0));
        CHECK(result.merges == 0);
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge),
                           "#1 and #2 touch along an edge or at a point only and cannot be one solid: left as they are."));
    }
    SECTION("4c: touching across a face area: 2 -> 1, volume 10") {
        const Mesh mesh = SampleMeshes::createFaceTouchingBoxesMesh();
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 1);
        CHECK(ogun::signedVolume(result.mesh) == Catch::Approx(10.0));
        CHECK(result.countsAfter.openEdges == 0);
        CHECK(result.countsAfter.nonManifoldEdges == 0);
        CHECK(result.merges == 1);
    }
}

TEST_CASE("Fix All merges each group of an assembly, and only the selected shells", "[repair][fixall][merge][scope]") {
    SECTION("Two separate assemblies: 4 -> 2 in one merge") {
        const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3}),
                                                       box({10, 0, 0}, {12, 2, 2}), box({11.8, 0.2, 0.3}, {13.8, 2.2, 2.3})});
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(shellCount(result.mesh) == 2);
        CHECK(result.passes[0].groupsMerged == 2);
        CHECK(result.merges == 1);
        CHECK(stepIn(result, FixAllStageId::PlanMerge)->detail == "Merge 4 shells that overlap into 2 solids.");
        const FixAllResult off = runAll(mesh, {});
        CHECK(shellCount(off.mesh) == 4);
        CHECK(off.countsAfter.intersectingFaces > 0);
    }
    SECTION("S1: #1 and #2 selected are merged; #3 is left bit-identical and named") {
        const Mesh mesh = SampleMeshes::combineMeshes(
            {box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3}), box({3.6, 0.4, 0.5}, {5.6, 2.4, 2.5})});
        const Mesh third = withoutShells(mesh, {0, 1});
        const FixAllResult result = ogun::runFixAll(mesh, shellScope(mesh, {0, 1}), mergeOn());
        CHECK(shellCount(result.mesh) == 2);
        CHECK(keeps(result.mesh, third));
        CHECK(notesContain(stepIn(result, FixAllStageId::PlanMerge),
                           "#2 still crosses #3, which is not selected: select both to merge them."));
        CHECK(result.merges == 1);
    }
    SECTION("A face scope: the merge needs whole shells") {
        const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
        std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 0);
        mask[0] = 1;
        const FixAllResult result = ogun::runFixAll(mesh, FixAllScope::fromFaceMask(mesh, FixAllScopeKind::Faces, mask), mergeOn());
        const FixAllStepReport* planned = stepIn(result, FixAllStageId::PlanMerge);
        REQUIRE(planned != nullptr);
        CHECK(planned->status == FixAllStepStatus::Skipped);
        CHECK(planned->reason == "Merge needs whole shells: select shells, or clear the selection.");
        CHECK(shellCount(result.mesh) == 2);
    }
}

TEST_CASE("Fix All merges at most once per pass and still stops", "[repair][fixall][merge]") {
    // A mocked phase 3 that always adds a new box crossing the body: each pass merges it, then adds another
    const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({3.5, 0.5, 0.5}, {7.5, 4.5, 4.5})});
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (phase != FixAllPhase::Cleanup) {
            return;
        }
        FixAllStage splitter;
        splitter.label = "Split off a crossing shell";
        splitter.run = [pass](const Mesh& m, const ogun::FixAllStageContext&) {
            ogun::FixAllStageOutcome outcome;
            const double y = 0.3 * pass;
            outcome.mesh = SampleMeshes::combineMeshes({m, box({1.0, 1.0 + y, -1.0}, {2.0, 2.0 + y, 1.0})});
            outcome.changed = true;
            outcome.targetBefore = 1;
            return outcome;
        };
        stages.push_back(std::move(splitter));
    };
    const FixAllResult result = runAll(mesh, mergeOn(), &hooks);
    REQUIRE(result.success);
    for (int pass = 1; pass <= result.passesRun; ++pass) {
        CHECK(mergeStepsKept(result, pass) <= 1);
        CHECK(std::count_if(result.steps.begin(), result.steps.end(), [&](const FixAllStepReport& step) {
                  return step.stage == FixAllStageId::PlanMerge && step.pass == pass;
              }) == 1);
    }
    CHECK(result.passesRun <= 3);
    CHECK((result.stopReason == FixAllStopReason::NoProgress || result.stopReason == FixAllStopReason::PassLimit ||
           result.stopReason == FixAllStopReason::RepeatedState));
}

TEST_CASE("Cancel during a merge drops the run once the union returns", "[repair][fixall][merge]") {
    const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3}),
                                                   box({10, 0, 0}, {12, 2, 2}), box({11.8, 0.2, 0.3}, {13.8, 2.2, 2.3})});
    std::stop_source source;
    ogun::FixAllProgress progress;
    int merging = 0;
    progress.observer = [&](float, const std::string& text) {
        if (text.find("merging") != std::string::npos && ++merging == 2) {
            source.request_stop(); // between the groups: the second one is not kept
        }
    };
    const FixAllResult result = ogun::runFixAll(mesh, FixAllScope::whole(mesh), mergeOn(), source.get_token(), &progress);
    CHECK(merging == 2);
    CHECK(result.cancelled);
    CHECK_FALSE(result.success);
    CHECK(same(result.mesh, mesh));
}

TEST_CASE("The dead-end guard asks the merge plan whether the merge can take a crossing", "[repair][fixall][merge]") {
    // Pull the first box's corner (2, 2, 2) out through the second box, which is open
    const Mesh openBox = dropLastFaces(box({3, 0.5, 0.5}, {4, 1.5, 1.5}), 1);
    const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), openBox});
    FixAllStage pull;
    pull.label = "Pull";
    pull.movesVertices = true;
    pull.run = [](const Mesh& m, const ogun::FixAllStageContext&) {
        ogun::FixAllStageOutcome outcome;
        outcome.mesh = m;
        for (Eigen::Index v = 0; v < m.numVertices(); ++v) {
            if (m.V.row(v) == Eigen::RowVector3d(2.0, 2.0, 2.0)) {
                outcome.mesh.V.row(v) << 5.0, 1.0, 1.0;
            }
        }
        outcome.mesh.recomputeNormals();
        outcome.changed = true;
        outcome.targetBefore = 1;
        return outcome;
    };
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [&](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (pass == 1 && phase == FixAllPhase::Repair) {
            stages.assign(1, pull);
        }
    };
    const FixAllResult result = runAll(mesh, mergeOn(1), &hooks);
    REQUIRE_FALSE(result.steps.empty());
    CHECK(result.steps.front().guard == FixAllGuard::DeadEnd);
    CHECK(result.steps.front().reason ==
          "It would move faces of #1 through #2, and the merge cannot take them (#2 is not closed), so nothing was changed.");
}

namespace {

using ogun::FixAllSuggestion;

const ogun::FixAllShellResidue* residueOf(const FixAllResult& result, int shell) {
    for (const ogun::FixAllShellResidue& r : result.report.residue) {
        if (r.shell == shell) {
            return &r;
        }
    }
    return nullptr;
}

/// The shell of `mesh` (`shellsLargestFirst`) holding a face with the corners of `part`'s face 0.
int shellHolding(const Mesh& mesh, const Mesh& part) {
    const auto shells = ogun::shellsLargestFirst(mesh);
    const ogun::FaceGeometryKey key = ogun::faceGeometryKey(part, 0);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells[s]) {
            if (ogun::faceGeometryKey(mesh, f) == key) {
                return static_cast<int>(s);
            }
        }
    }
    return -1;
}

/// A unit cube whose first face is split at a point 0.001 off the middle of its first edge: one needle.
Mesh cubeWithNeedle() {
    Mesh mesh = box({0, 0, 0}, {1, 1, 1});
    const Eigen::RowVector3d a = mesh.V.row(mesh.F(0, 0));
    const Eigen::RowVector3d b = mesh.V.row(mesh.F(0, 1));
    const Eigen::RowVector3d c = mesh.V.row(mesh.F(0, 2));
    const Eigen::RowVector3d mid = 0.5 * (a + b);
    const Eigen::RowVector3d p = mid + 0.001 * (c - mid);
    const int ia = mesh.F(0, 0);
    const int ib = mesh.F(0, 1);
    const int ic = mesh.F(0, 2);
    const auto ip = static_cast<int>(mesh.V.rows());
    mesh.V.conservativeResize(mesh.V.rows() + 1, 3);
    mesh.V.row(ip) = p;
    mesh.F.row(0) << ia, ib, ip;
    mesh.F.conservativeResize(mesh.F.rows() + 2, 3);
    mesh.F.row(mesh.F.rows() - 2) << ib, ic, ip;
    mesh.F.row(mesh.F.rows() - 1) << ic, ia, ip;
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("Fix All's final report: what is left in each shell and the suggestion", "[repair][fixall][report]") {
    SECTION("An open shell the merge skipped: alpha wrap it; the union names why it was not merged") {
        const Mesh mesh = SampleMeshes::createOpenShellAmongCubesMesh();
        ogun::FixAllParams params = mergeOn();
        params.holeFill.maxHoleEdges = 3;
        const FixAllResult result = runAll(mesh, params);
        const ogun::FixAllFinalReport& report = result.report;
        REQUIRE(report.complete);
        const ogun::FixAllShellResidue* open = residueOf(result, 1);
        REQUIRE(open != nullptr);
        CHECK(open->openEdges == 4);
        CHECK(open->suggestion == FixAllSuggestion::AlphaWrap);
        CHECK(open->reason == "#2 still has 4 open edges. It still crosses #1. Suggested: alpha wrap it.");
        const ogun::FixAllShellResidue* body = residueOf(result, 0);
        REQUIRE(body != nullptr);
        CHECK(body->suggestion == FixAllSuggestion::ReportOnly);
        CHECK(body->reason == "#1 still crosses #2: #2 cannot be merged: it is still open (4 open edges).");
        REQUIRE(report.crossingsLeft.size() == 1);
        CHECK(report.crossingsLeft[0].fills.empty());
        CHECK(report.crossingsLeft[0].whyNotMerged == "#2 cannot be merged: it is still open (4 open edges)");
        CHECK(report.shellsLine == "Shells: 3 -> 2 (merged 2 into 1; skipped #2)");
        CHECK(report.passesLine == "Passes: 2 of 3 (stopped: repeated state)");
        CHECK(report.leftLine.starts_with("Left after 2 passes: 4 open edges and "));
        CHECK(report.leftLine.ends_with("; in #1 and #2."));
        CHECK(report.closedLine == "Watertight: 1 of 2 shells");
        CHECK(report.shellsBefore == 3);
        CHECK(report.shellsAfter == 2);
        CHECK(report.crossingFacesWithin == 0);
        CHECK(report.crossingFacesBetween == result.countsAfter.intersectingFaces);
        CHECK(report.health.size() == ogun::kDefectCategoryCount);
    }
    SECTION("Merge unticked with crossings: tick Merge shells") {
        const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
        const FixAllResult result = runAll(mesh, {});
        REQUIRE(result.report.residue.size() == 2);
        for (const ogun::FixAllShellResidue& r : result.report.residue) {
            CHECK(r.suggestion == FixAllSuggestion::TickMerge);
        }
        CHECK(residueOf(result, 0)->reason == "#1 still crosses #2: the merge is off. Tick Merge shells and preview again.");
        REQUIRE(result.report.crossingsLeft.size() == 1);
        CHECK(result.report.crossingsLeft[0].text.starts_with("#1 and #2 still cross ("));
        CHECK(result.report.crossingsLeft[0].text.ends_with(" crossing face pairs): the merge is off."));
        CHECK(result.report.shellsLine == "Shells: 2 -> 2");
        CHECK(result.report.closedLine == "Watertight: 2 of 2 shells");
    }
    SECTION("Q20: a group reaching into a cavity is reported, never merged") {
        const Mesh mesh = SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::InsideOverlap);
        const FixAllResult result = runAll(mesh, mergeOn());
        const ogun::FixAllFinalReport& report = result.report;
        REQUIRE(report.groupsNotMerged.size() == 1);
        CHECK(report.groupsNotMerged[0].text.find(" into cavity #3 of #") != std::string::npos);
        CHECK(report.groupsNotMerged[0].warning);
        for (const ogun::FixAllShellResidue& r : report.residue) {
            CHECK(r.suggestion == FixAllSuggestion::ReportOnly);
            CHECK(r.reason.find("their group reaches into cavity #3 of #") != std::string::npos);
        }
        CHECK(report.cavitiesKept == std::vector<int>{2});
        CHECK(report.cavitiesLine.starts_with("Cavities kept: 1 (#3 inside #"));
        CHECK(report.cavitiesLine.ends_with(", unchanged)"));
        CHECK(report.shellsLine == "Shells: 3 -> 3 (1 cavity kept; 1 group not merged)");
        CHECK(report.closedLine == "Watertight: 3 of 3 shells (2 solids + 1 cavity)");
    }
    SECTION("Q23: an inside-out body with a shell inside is reported with both shells") {
        const Mesh mesh = SampleMeshes::createCavityBoxMesh(true);
        const FixAllResult result = runAll(mesh, {});
        REQUIRE(result.report.groupsNotMerged.size() == 1);
        CHECK(result.report.groupsNotMerged[0].text.starts_with("#1 and its inner shell #2 look inside out together"));
        CHECK(result.report.groupsNotMerged[0].shells == std::vector<int>{0, 1});
        REQUIRE(result.report.residue.size() == 2);
        CHECK(residueOf(result, 0)->suggestion == FixAllSuggestion::ReportOnly);
        CHECK(residueOf(result, 0)->reason == "#1 looks inside out together with #2: left as it is. Flip them by hand (see Inverted shells).");
    }
    SECTION("Rule O unsure (an inward shell crossing another): reported") {
        const Mesh mesh = SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::CrossesWall);
        const FixAllResult result = runAll(mesh, {});
        const ogun::FixAllShellResidue* inward = residueOf(result, 2);
        REQUIRE(inward != nullptr);
        CHECK(inward->suggestion == FixAllSuggestion::ReportOnly);
        CHECK(inward->reason.starts_with("#3 is wound inward and crosses #"));
    }
    SECTION("An edge contact only: reported") {
        const Mesh mesh = SampleMeshes::createEdgeTouchingBoxesMesh();
        const FixAllResult result = runAll(mesh, mergeOn());
        REQUIRE(result.report.residue.size() == 2);
        CHECK(residueOf(result, 0)->suggestion == FixAllSuggestion::ReportOnly);
        CHECK(residueOf(result, 0)->reason ==
              "#1 touches #2 along an edge or at a point only and cannot be one solid with it: left as it is.");
        CHECK(result.report.crossingsLeft.empty());
    }
    SECTION("Thin faces only: no suggestion, a line saying they are left") {
        const Mesh mesh = cubeWithNeedle();
        ogun::FixAllParams params;
        params.healThinFaces = false; // stands in for faces the healer cannot heal (e.g. on a sharp crease)
        const FixAllResult result = runAll(mesh, params);
        REQUIRE(result.countsAfter.thinFaces >= 1);
        CHECK(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.report.residue.size() == 1);
        CHECK(result.report.residue[0].suggestion == FixAllSuggestion::None);
        CHECK(result.report.residue[0].reason.starts_with("#1: 1 thin face left."));
        CHECK(result.report.leftLine == "Clean after 1 pass. 1 thin face left.");
        CHECK(result.report.crossingsLeft.empty());
    }
    SECTION("A clean result has no residue") {
        const Mesh mesh = box({0, 0, 0}, {1, 1, 1});
        const FixAllResult result = runAll(mesh, mergeOn());
        CHECK(result.report.residue.empty());
        CHECK(result.report.passesLine == "Passes: 1 of 3 (stopped: clean)");
        CHECK(result.report.leftLine == "Clean after 1 pass.");
        CHECK(result.report.lines.front().text == "Passes: 1 of 3 (stopped: clean)");
    }
}

TEST_CASE("Fix All's final report lists every crossing left by shell pair and the fill that made it",
          "[repair][fixall][report]") {
    using Variant = SampleMeshes::FillCrossesVariant;
    const Mesh mesh = SampleMeshes::createFillCrossesShellMesh(Variant::PlateThroughOpenBox);
    const Mesh plate = withoutShells(mesh, {0, 2});

    SECTION("Fixture 4, merge off: the fill of the open box crosses the plate; the merge is off") {
        const FixAllResult result = runAll(mesh, {});
        const int plateShell = shellHolding(result.mesh, plate);
        REQUIRE(plateShell >= 0);
        const auto it = std::find_if(result.report.crossingsLeft.begin(), result.report.crossingsLeft.end(),
                                     [](const ogun::FixAllCrossingLeft& left) { return !left.fills.empty(); });
        REQUIRE(it != result.report.crossingsLeft.end());
        CHECK((it->a == plateShell || it->b == plateShell));
        CHECK(it->fills[0].hole == 1);
        CHECK(it->fills[0].holeShell != plateShell);
        CHECK(it->fills[0].facePairs > 0);
        CHECK(it->whyNotMerged == "the merge is off");
        CHECK(it->text.find("'s fill of hole 1 ") != std::string::npos);
        CHECK(it->text.find(" crosses #" + std::to_string(plateShell + 1) + " in ") != std::string::npos);
        CHECK(it->text.ends_with(": the merge is off. The hole stays filled."));
    }
    SECTION("4d, merge off: the fill crosses a shell that is not selected") {
        const FixAllResult result = ogun::runFixAll(mesh, shellScope(mesh, {0, 2}), {});
        const int plateShell = shellHolding(result.mesh, plate);
        const auto it = std::find_if(result.report.crossingsLeft.begin(), result.report.crossingsLeft.end(),
                                     [](const ogun::FixAllCrossingLeft& left) { return !left.fills.empty(); });
        REQUIRE(it != result.report.crossingsLeft.end());
        CHECK(it->whyNotMerged == "#" + std::to_string(plateShell + 1) + " is not selected");
        CHECK(it->text.ends_with("is not selected. The hole stays filled."));
        CHECK(result.report.shellInScope[static_cast<std::size_t>(plateShell)] == 0);
        CHECK(residueOf(result, plateShell) == nullptr); // not in scope: no residue row
    }
    SECTION("Fixture 4 and 4d, merge on: nothing crosses any more") {
        CHECK(runAll(mesh, mergeOn()).report.crossingsLeft.empty());
        CHECK(ogun::runFixAll(mesh, shellScope(mesh, {0, 2}), mergeOn()).report.crossingsLeft.empty());
    }
    SECTION("4e: the plate stays open: alpha wrap it") {
        const Mesh openPlate = SampleMeshes::createFillCrossesShellMesh(Variant::OpenPlate);
        const Mesh platePart = withoutShells(openPlate, {0, 2});
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [](int, FixAllPhase, std::vector<FixAllStage>& stages) {
            std::erase_if(stages, [](const FixAllStage& s) { return s.id == FixAllStageId::FillHoles; });
        };
        const FixAllResult result = runAll(openPlate, mergeOn(), &hooks);
        const int plateShell = shellHolding(result.mesh, platePart);
        const ogun::FixAllShellResidue* r = residueOf(result, plateShell);
        REQUIRE(r != nullptr);
        CHECK(r->suggestion == FixAllSuggestion::AlphaWrap);
        CHECK(r->openEdges == 3);
    }
}

TEST_CASE("Fix All's report notes name faces of the returned mesh", "[repair][fixall][report]") {
    SECTION("A full run: the cavity a note names is the cavity of the result") {
        const Mesh mesh = SampleMeshes::createCavityHoleSliversIntersectionsMesh();
        const FixAllResult result = runAll(mesh, {});
        REQUIRE(result.success);
        const auto shells = ogun::shellsLargestFirst(result.mesh);
        bool found = false;
        for (const FixAllStepReport& step : result.steps) {
            if (!step.notesInResult) {
                continue;
            }
            for (const ogun::FixAllShellNote& note : step.notes) {
                for (const auto& faces : note.shells) {
                    for (const Eigen::Index f : faces) {
                        REQUIRE(f < result.mesh.numFaces());
                    }
                }
                if (note.text.ends_with("1 cavity inward: #2") && step.pass == result.bestPass) {
                    REQUIRE(note.shells.size() == 1);
                    std::vector<Eigen::Index> faces = note.shells[0];
                    std::sort(faces.begin(), faces.end());
                    std::vector<Eigen::Index> cavity = shells[1];
                    std::sort(cavity.begin(), cavity.end());
                    CHECK(faces == cavity);
                    found = true;
                }
            }
        }
        CHECK(found);
    }
    SECTION("Merge on: the merge note names the union in the result") {
        const Mesh mesh = SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
        const FixAllResult result = runAll(mesh, mergeOn());
        const FixAllStepReport* merge = stepIn(result, FixAllStageId::MergeShells);
        REQUIRE(merge != nullptr);
        REQUIRE(merge->notesInResult);
        REQUIRE(merge->notes.size() == 1);
        REQUIRE(merge->notes[0].shells.size() == 1);
        // Phase 3 healed the cut slivers after the union: the note follows to the final faces
        CHECK(merge->notes[0].shells[0].size() <= static_cast<std::size_t>(result.mesh.numFaces()));
        CHECK_FALSE(merge->notes[0].shells[0].empty());
        for (const Eigen::Index f : merge->notes[0].shells[0]) {
            CHECK(f < result.mesh.numFaces());
        }
        // The Turkey-like assembly: the deleted nested solid is listed with the input faces it had
        const Mesh turkeyLike = SampleMeshes::createTurkeyLikeMesh();
        const FixAllResult deleted = runAll(turkeyLike, mergeOn());
        REQUIRE(deleted.report.deletions.size() == 1);
        CHECK(deleted.report.deletions[0].stage == FixAllStageId::DeleteNestedSolids);
        CHECK(deleted.report.deletions[0].inputFaces.size() == 12);
        CHECK(deleted.report.shellsLine == "Shells: 9 -> 2 (merged 7 into 1; deleted 1 nested solid)");
    }
    SECTION("A full run whose best is the input: no note is in the result") {
        const Mesh mesh = box({0, 0, 0}, {1, 1, 1});
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
            if (phase != FixAllPhase::Repair) {
                return;
            }
            FixAllStage worse;
            worse.label = "Add a crossing box";
            worse.run = [](const Mesh& m, const ogun::FixAllStageContext&) {
                ogun::FixAllStageOutcome outcome;
                outcome.mesh = SampleMeshes::combineMeshes({m, box({0.5, 0.5, 0.5}, {1.5, 1.5, 1.5})});
                outcome.changed = true;
                outcome.targetBefore = 1;
                outcome.notes.push_back({"Added a box", {{0}}});
                return outcome;
            };
            stages.assign(1, worse);
        };
        const FixAllResult result = runAll(mesh, {}, &hooks);
        REQUIRE(result.bestPass == 0);
        CHECK(same(result.mesh, mesh));
        for (const FixAllStepReport& step : result.steps) {
            CHECK_FALSE(step.notesInResult);
        }
        CHECK(result.report.passesLine.ends_with("; no pass was better than the input, so it is returned unchanged"));
    }
}

TEST_CASE("Fix All on the Turkey with the merge on: its group is refused; without #28 it merges", "[repair][fixall][merge][.slow]") {
    const Mesh turkey = ogun::loadStl(examplePath("Turkey.stl"));
    const auto timed = [](const Mesh& mesh) {
        const auto start = std::chrono::steady_clock::now();
        FixAllResult result = runAll(mesh, mergeOn());
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return std::make_pair(std::move(result), ms);
    };
    const auto print = [](const char* name, const FixAllResult& result, double ms) {
        double guards = 0.0;
        for (const FixAllStepReport& step : result.steps) {
            guards += step.guardMs;
            if (step.phase != FixAllPhase::Repair || step.status != FixAllStepStatus::Unchanged) {
                WARN(name << " pass " << step.pass << " phase " << static_cast<int>(step.phase) << ": " << step.label << " ["
                          << static_cast<int>(step.status) << "] " << step.detail << step.reason << " | " << step.elapsedMs
                          << " ms, guards " << step.guardMs << " ms");
            }
        }
        WARN(name << ": " << ms << " ms, guards " << guards << " ms, passes " << result.passesRun << ", stop "
                  << ogun::fixAllStopReasonLabel(result.stopReason) << ", shells " << shellCount(result.mesh) << ", crossings "
                  << result.countsBefore.intersectingFaces << " -> " << result.countsAfter.intersectingFaces << ", thin "
                  << result.countsBefore.thinFaces << " -> " << result.countsAfter.thinFaces << ", faces "
                  << result.mesh.numFaces() << ", report " << result.report.elapsedMs << " ms");
        for (const ogun::FixAllReportLine& line : result.report.lines) {
            if (line.text.find("still cross (") == std::string::npos) { // one line per crossing pair: counted below
                WARN(name << " report: " << line.text.substr(0, 300));
            }
        }
        WARN(name << " report: " << result.report.crossingsLeft.size() << " crossing pairs left, "
                  << result.report.residue.size() << " shells with something left");
    };

    // #6 and #9 reach into cavity #28 of #12: the group of 27 is refused, nothing merged
    const auto [refused, refusedMs] = timed(turkey);
    print("Turkey merge on", refused, refusedMs);
    REQUIRE(refused.success);
    CHECK(refused.merges == 0);
    CHECK(shellCount(refused.mesh) == 28);
    CHECK(refused.countsAfter.openEdges == 0);
    const FixAllStepReport* planned = stepIn(refused, FixAllStageId::PlanMerge);
    REQUIRE(planned != nullptr);
    CHECK(std::any_of(planned->notes.begin(), planned->notes.end(), [](const ogun::FixAllShellNote& note) {
        return note.warning && note.text.starts_with("#6 and #9 reach into cavity #28 of #12");
    }));
    // The report: the refused group by shell number, the cavity kept, every crossing pair with the refusal
    const ogun::FixAllFinalReport& report = refused.report;
    REQUIRE(report.complete);
    REQUIRE(report.groupsNotMerged.size() == 1);
    CHECK(report.groupsNotMerged[0].text.starts_with("#6 and #9 reach into cavity #28 of #12, so their group of 27 shells is not merged"));
    CHECK(report.groupsNotMerged[0].shells.size() == 28);
    CHECK(report.cavitiesKept == std::vector<int>{27});
    CHECK(report.cavitiesLine.starts_with("Cavities kept: 1 (#28 inside #12, "));
    CHECK(report.cavitiesLine.ends_with(", unchanged)"));
    CHECK(report.shellsLine == "Shells: 28 -> 28 (1 cavity kept; 1 group not merged)");
    CHECK(report.closedLine == "Watertight: 28 of 28 shells (27 solids + 1 cavity)");
    CHECK(report.crossingFacesWithin == 0);
    CHECK(report.crossingFacesBetween == refused.countsAfter.intersectingFaces);
    CHECK(report.crossingsLeft.size() == 100);
    for (const ogun::FixAllCrossingLeft& left : report.crossingsLeft) {
        CHECK(left.whyNotMerged == "their group reaches into cavity #28 of #12, so it is not merged (merge them by hand)");
        CHECK(left.fills.empty()); // the Turkey has no hole
    }
    CHECK(report.residue.size() == 27);
    for (const ogun::FixAllShellResidue& r : report.residue) {
        CHECK(r.suggestion == ogun::FixAllSuggestion::ReportOnly);
        CHECK_FALSE(r.unresolved());
    }
    CHECK(report.leftLine.starts_with("Left after 2 passes: "));

    // The user deletes #28 by hand: 27 shells -> 1
    const Mesh without = withoutShells(turkey, {27});
    const auto [merged, mergedMs] = timed(without);
    print("Turkey without #28, merge on", merged, mergedMs);
    REQUIRE(merged.success);
    CHECK(merged.merges >= 1);
    CHECK(shellCount(merged.mesh) == 1);
    CHECK(merged.countsAfter.openEdges == 0);
    CHECK(merged.report.shellsLine == "Shells: 27 -> 1 (merged 27 into 1)");
    CHECK(merged.report.closedLine == "Watertight: 1 of 1 shell");
    CHECK(merged.report.crossingsLeft.empty());
    CHECK(merged.report.groupsNotMerged.empty());
    CHECK(merged.report.cavitiesKept.empty());
    for (const ogun::FixAllShellResidue& r : merged.report.residue) {
        CHECK(r.suggestion == ogun::FixAllSuggestion::None); // thin faces only
    }
}

// ---------------------------------------------------------------------------------------------
// Rebuild shell #k...: per-shell rebuild rows built on a Fix All result (never run unless asked)
// ---------------------------------------------------------------------------------------------

namespace {

using ogun::GlobalRepairMethod;
using ogun::RepairCandidate;

/// Fix All through the Repair All entry point (the candidate a rebuild row builds on).
ogun::GlobalRepairParams rebuildParams(bool merge) {
    ogun::GlobalRepairParams params;
    params.fixAll.mergeShells = merge;
    params.holeSettings.maxHoleEdges = 3; // the open box's 4-edge hole stays open
    params.alphaWrapDetail = ogun::settings::kMinAlphaWrapDetail; // coarse rebuilds keep the tests fast
    params.windingResolution = ogun::settings::kMinWindingGridResolution;
    return params;
}

const RepairCandidate& row(const std::vector<RepairCandidate>& rows, GlobalRepairMethod method) {
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const RepairCandidate& c) { return c.method == method; });
    REQUIRE(it != rows.end());
    return *it;
}

} // namespace

TEST_CASE("Rebuild shell: a shell Fix All left open is wrapped and merged; a rebuild that stays open is not",
          "[repair][fixall][rebuild]") {
    const Mesh mesh = SampleMeshes::createOpenShellAmongCubesMesh();
    const ogun::GlobalRepairParams params = rebuildParams(true);
    const RepairCandidate fixAll = ogun::runGlobalRepair(mesh, {}, GlobalRepairMethod::FixAll, params);
    REQUIRE(fixAll.success);
    REQUIRE(fixAll.fixAll != nullptr);
    // No rebuild runs unless asked: Fix All leaves the open box as it was and only suggests a wrap
    const ogun::FixAllFinalReport& report = fixAll.fixAll->result.report;
    REQUIRE(report.shells.size() == 2);
    CHECK_FALSE(fixAll.fixAll->rebuild.has_value());
    CHECK(fixAll.metrics.openEdgesAfter == 4);
    const auto open = std::find_if(report.residue.begin(), report.residue.end(),
                                   [](const ogun::FixAllShellResidue& r) { return r.shell == 1; });
    REQUIRE(open != report.residue.end());
    CHECK(open->suggestion == FixAllSuggestion::AlphaWrap);
    const Mesh body = withoutShells(fixAll.mesh, {1});

    const std::vector<RepairCandidate> rows = ogun::computeShellRebuildCandidates(mesh, {}, fixAll, 1, params);
    REQUIRE(rows.size() == ogun::kShellRebuildMethods.size());
    CHECK(rows[0].method == GlobalRepairMethod::AlphaWrap);

    {   // Alpha wrap (suggested): closes the box, merges it into the body and cleans up
        const RepairCandidate& wrap = row(rows, GlobalRepairMethod::AlphaWrap);
        REQUIRE(wrap.success);
        REQUIRE(wrap.fixAll != nullptr);
        REQUIRE(wrap.fixAll->rebuild.has_value());
        const ogun::FixAllShellRebuild& rebuild = *wrap.fixAll->rebuild;
        CHECK(rebuild.base == fixAll.fixAll);
        CHECK(rebuild.shell == 1);
        CHECK(rebuild.suggested);
        CHECK(rebuild.closed);
        CHECK(rebuild.merged);
        CHECK(rebuild.label == "+ Alpha wrap #2");
        CHECK(rebuild.note == "wrapped, then merged");
        CHECK(shellCount(wrap.mesh) == 1);
        CHECK(wrap.metrics.shellsBefore == 3);
        CHECK(wrap.metrics.shellsAfter == 1);
        CHECK(wrap.metrics.openEdgesBefore == fixAll.metrics.openEdgesBefore);
        CHECK(wrap.metrics.openEdgesAfter == 0);
        CHECK(wrap.metrics.nonManifoldEdgesAfter == 0);
        CHECK(wrap.metrics.intersectingFacesAfter == 0);
        CHECK(wrap.statusMessage.starts_with("Alpha wrap #2: wrapped, then merged. "));
        CHECK(wrap.fixAll->faceShell.size() == static_cast<std::size_t>(wrap.mesh.numFaces()));
        // Its pass merged only the wrapped shell's group, and phase 1 did not run
        const ogun::FixAllResult& pass = wrap.fixAll->result;
        CHECK(pass.passesRun == 1);
        CHECK(std::none_of(pass.steps.begin(), pass.steps.end(),
                           [](const FixAllStepReport& step) { return step.phase == FixAllPhase::Repair; }));
        CHECK(mergeStepsKept(pass, 1) == 1);
        CHECK(phaseSteps(pass, FixAllPhase::Cleanup, 1) > 0);
    }
    {   // Self-union of an open box stays open: not merged, the body kept exactly
        const RepairCandidate& unite = row(rows, GlobalRepairMethod::SelfUnion);
        if (unite.success) {
            const ogun::FixAllShellRebuild& rebuild = *unite.fixAll->rebuild;
            CHECK_FALSE(rebuild.suggested);
            CHECK_FALSE(rebuild.closed);
            CHECK_FALSE(rebuild.merged);
            CHECK(rebuild.note.starts_with("still open ("));
            CHECK(rebuild.note.ends_with("): not merged"));
            CHECK(keeps(unite.mesh, body));
            CHECK(unite.metrics.shellsAfter == 2);
            CHECK(unite.metrics.openEdgesAfter > 0);
        } else {
            CHECK(unite.statusMessage.starts_with("Self-union of #2 produced no mesh"));
        }
    }
    {   // Winding rebuild: closed, so merged too
        const RepairCandidate& winding = row(rows, GlobalRepairMethod::WindingRebuild);
        REQUIRE(winding.success);
        const ogun::FixAllShellRebuild& rebuild = *winding.fixAll->rebuild;
        CHECK(rebuild.closed);
        CHECK(rebuild.merged);
        CHECK(rebuild.note == "rebuilt, then merged");
        CHECK(rebuild.label == "+ Winding-number rebuild #2");
        CHECK(winding.metrics.shellsAfter == 1);
    }
    {   // A stopped rebuild returns nothing
        std::stop_source source;
        source.request_stop();
        const RepairCandidate stopped =
            ogun::computeShellRebuildCandidate(mesh, {}, fixAll, 1, GlobalRepairMethod::AlphaWrap, params, source.get_token());
        CHECK(stopped.cancelled);
        CHECK_FALSE(stopped.success);
        CHECK(ogun::computeShellRebuildCandidates(mesh, {}, fixAll, 1, params, source.get_token()).empty());
    }
    {   // A shell that is not there, or a Fix All candidate that is not one, is refused
        const RepairCandidate none = ogun::computeShellRebuildCandidate(mesh, {}, fixAll, 7, GlobalRepairMethod::AlphaWrap, params);
        CHECK_FALSE(none.success);
        CHECK(none.statusMessage == "There is no shell #8 in Fix All's result.");
        const RepairCandidate notFixAll = ogun::runGlobalRepair(mesh, {}, GlobalRepairMethod::WindingRebuild, params);
        CHECK(ogun::computeShellRebuildCandidate(mesh, {}, notFixAll, 0, GlobalRepairMethod::AlphaWrap, params).statusMessage ==
              "Preview Fix All first: a rebuild row builds on its result.");
    }
}

TEST_CASE("Rebuild shell: with the merge off, the wrapped shell is closed but not merged", "[repair][fixall][rebuild]") {
    const Mesh mesh = SampleMeshes::createOpenShellAmongCubesMesh();
    const ogun::GlobalRepairParams off = rebuildParams(false);
    const RepairCandidate fixAllOff = ogun::runGlobalRepair(mesh, {}, GlobalRepairMethod::FixAll, off);
    REQUIRE(fixAllOff.success);
    const auto& shells = fixAllOff.fixAll->result.report.shells;
    REQUIRE(shells.size() == 3);
    // The open box is the smallest shell, #3
    const RepairCandidate wrap =
        ogun::computeShellRebuildCandidate(mesh, {}, fixAllOff, 2, GlobalRepairMethod::AlphaWrap, off);
    REQUIRE(wrap.success);
    CHECK(wrap.fixAll->rebuild->closed);
    CHECK_FALSE(wrap.fixAll->rebuild->merged);
    CHECK(wrap.fixAll->rebuild->note == "wrapped; the merge is off");
    CHECK(wrap.metrics.shellsAfter == 3);
    CHECK(wrap.metrics.openEdgesAfter == 0);
    CHECK(keeps(wrap.mesh, withoutShells(fixAllOff.mesh, {2})));
}

TEST_CASE("Rebuild shell: a cavity is never in a rebuild's input and stays in every row", "[repair][fixall][rebuild][cavity]") {
    // Fixture 2: A and B merged around the cavity of A
    const Mesh mesh = SampleMeshes::createOverlapPlusCavityMesh();
    const ogun::GlobalRepairParams params = rebuildParams(true);
    const RepairCandidate fixAll = ogun::runGlobalRepair(mesh, {}, GlobalRepairMethod::FixAll, params);
    REQUIRE(fixAll.success);
    const ogun::FixAllFinalReport& report = fixAll.fixAll->result.report;
    REQUIRE(report.shells.size() == 2);
    REQUIRE(report.cavitiesKept == std::vector<int>{1});
    const Mesh cavity = withoutShells(fixAll.mesh, {0});
    REQUIRE(ogun::signedVolume(cavity) == Catch::Approx(-4.0));

    // Rebuilding the merged body: the cavity, its own shell, is kept bit-identical in every row
    for (const RepairCandidate& rebuilt : ogun::computeShellRebuildCandidates(mesh, {}, fixAll, 0, params)) {
        INFO(ogun::globalRepairMethodLabel(rebuilt.method));
        REQUIRE(rebuilt.success);
        CHECK(keeps(rebuilt.mesh, cavity));
        CHECK(shellCount(rebuilt.mesh) == 2);
        CHECK(rebuilt.fixAll->result.report.cavitiesKept.size() == 1);
        CHECK(rebuilt.fixAll->rebuild->note.ends_with("; it overlaps no other shell, so there is nothing to merge"));
    }
    // Rebuilding the cavity itself is refused: it would fill the hollow
    const RepairCandidate refused = ogun::computeShellRebuildCandidate(mesh, {}, fixAll, 1, GlobalRepairMethod::AlphaWrap, params);
    CHECK_FALSE(refused.success);
    CHECK(refused.statusMessage == "#2 is a cavity, a hollow kept on purpose: rebuilding it would fill the hollow.");
}

TEST_CASE("Rebuild side: an alpha wrap of the scope would fill the cavities in it", "[repair][fixall][rebuild][cavity]") {
    const Mesh cavityBox = SampleMeshes::createCavityBoxMesh();
    CHECK(ogun::cavitiesFilledByRebuild(cavityBox, {}) == std::vector<int>{1});
    // Only the outer box selected: the cavity is not in the scope
    CHECK(ogun::cavitiesFilledByRebuild(cavityBox, shellScope(cavityBox, {0}).faceInScope)->empty());
    CHECK(ogun::cavitiesFilledByRebuild(cavityBox, shellScope(cavityBox, {0, 1}).faceInScope) == std::vector<int>{1});
    CHECK(ogun::cavitiesFilledByRebuild(SampleMeshes::createOverlapPlusCavityMesh(), {}) == std::vector<int>{2});
    // Inside out, the inner box is no cavity; a plain box has none
    CHECK(ogun::cavitiesFilledByRebuild(SampleMeshes::createCavityBoxMesh(true), {})->empty());
    CHECK(ogun::cavitiesFilledByRebuild(box({0, 0, 0}, {1, 1, 1}), {})->empty());
    std::stop_source source;
    source.request_stop();
    CHECK_FALSE(ogun::cavitiesFilledByRebuild(cavityBox, {}, ogun::settings::kDefaultIntersectionTolerance, source.get_token()));
}
