// Fix All: face and shell scopes carried through every step, the clean-up after a merge (phase 3), and
// the first real files.

#include "repair/FixAll.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/Orientation.hpp"
#include "repair/RepairMetrics.hpp"
#include "repair/ShellMerge.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using ogun::FixAllPhase;
using ogun::FixAllScope;
using ogun::FixAllScopeKind;
using ogun::FixAllStage;
using ogun::FixAllStageId;
using ogun::FixAllStageOutcome;
using ogun::FixAllStepStatus;
using ogun::FixAllStopReason;
using ogun::Mesh;
using ogun::SampleMeshes;

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

Eigen::Vector3d centroid(const Mesh& mesh, Eigen::Index f) {
    return (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
}

/// A scope of the faces whose centroid passes `pick`.
FixAllScope scopeWhere(const Mesh& mesh, FixAllScopeKind kind, const std::function<bool(const Eigen::Vector3d&)>& pick) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        mask[static_cast<std::size_t>(f)] = pick(centroid(mesh, f)) ? 1 : 0;
    }
    return FixAllScope::fromFaceMask(mesh, kind, mask);
}

FixAllScope scopeOfFaces(const Mesh& mesh, FixAllScopeKind kind, const std::vector<Eigen::Index>& faces) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const Eigen::Index f : faces) {
        mask[static_cast<std::size_t>(f)] = 1;
    }
    return FixAllScope::fromFaceMask(mesh, kind, mask);
}

/// Every face of `before` outside `scope` is still in `after` with the same corners.
bool outsideKept(const Mesh& before, const FixAllScope& scope, const Mesh& after) {
    std::unordered_map<ogun::FaceGeometryKey, int, ogun::FaceGeometryKeyHash> count;
    for (Eigen::Index f = 0; f < after.numFaces(); ++f) {
        ++count[ogun::faceGeometryKey(after, f)];
    }
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        if (scope.inScope(f)) {
            continue;
        }
        const auto it = count.find(ogun::faceGeometryKey(before, f));
        if (it == count.end() || it->second == 0) {
            return false;
        }
        --it->second;
    }
    return true;
}

/// `mesh` without the listed faces.
Mesh withoutFaces(const Mesh& mesh, std::vector<Eigen::Index> faces) {
    std::sort(faces.begin(), faces.end());
    Mesh out = mesh;
    out.F.resize(mesh.numFaces() - static_cast<Eigen::Index>(faces.size()), 3);
    Eigen::Index row = 0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (!std::binary_search(faces.begin(), faces.end(), f)) {
            out.F.row(row++) = mesh.F.row(f);
        }
    }
    out.recomputeNormals();
    return out;
}

/// The face whose centroid is nearest `p`.
Eigen::Index faceNear(const Mesh& mesh, const Eigen::Vector3d& p) {
    Eigen::Index best = 0;
    for (Eigen::Index f = 1; f < mesh.numFaces(); ++f) {
        if ((centroid(mesh, f) - p).norm() < (centroid(mesh, best) - p).norm()) {
            best = f;
        }
    }
    return best;
}

/// Face `f` = (a, b, c) split in three around a new vertex `t` of the way from a to the centroid:
/// rows f = (a, b, p), then appended (b, c, p) and (c, a, p). Small `t` makes two needles at a.
Mesh splitNearCorner(const Mesh& mesh, Eigen::Index f, double t) {
    Mesh out = mesh;
    const int a = mesh.F(f, 0);
    const int b = mesh.F(f, 1);
    const int c = mesh.F(f, 2);
    const Eigen::RowVector3d p = mesh.V.row(a) + t * (centroid(mesh, f).transpose() - mesh.V.row(a));
    const auto pv = static_cast<int>(mesh.numVertices());
    out.V.conservativeResize(pv + 1, 3);
    out.V.row(pv) = p;
    out.F.row(f) << a, b, pv;
    out.F.conservativeResize(mesh.numFaces() + 2, 3);
    out.F.row(mesh.numFaces()) << b, c, pv;
    out.F.row(mesh.numFaces() + 1) << c, a, pv;
    out.recomputeNormals();
    return out;
}

const ogun::FixAllStepReport* stepOf(const ogun::FixAllResult& result, FixAllStageId id, int pass = 1,
                                     FixAllPhase phase = FixAllPhase::Repair) {
    const auto it = std::find_if(result.steps.begin(), result.steps.end(), [&](const ogun::FixAllStepReport& step) {
        return step.stage == id && step.pass == pass && step.phase == phase;
    });
    return it == result.steps.end() ? nullptr : &*it;
}

std::size_t openEdges(const Mesh& mesh) {
    return ogun::runCategoryCheck(ogun::DefectCategory::OpenEdges, mesh, {}).infoCount;
}

/// Share of the faces of `before` that are still in `after` with the same corners.
double unchangedFraction(const Mesh& before, const Mesh& after) {
    return 1.0 - static_cast<double>(ogun::faceDiff(before, after).removed.size()) / static_cast<double>(before.numFaces());
}

/// The fold island of the folded cube (its faces on the top near (2.75, 2.1, 8)).
std::vector<Eigen::Index> foldIsland(const Mesh& folded) {
    const ogun::CategoryResult check = ogun::runCategoryCheck(ogun::DefectCategory::Intersections, folded, {});
    const auto islands = ogun::clusterIntersectionIslands(folded, check, ogun::shellIdsOfFaces(folded));
    for (const ogun::IntersectionIsland& island : islands) {
        const Eigen::Vector3d c = centroid(folded, island.faces.front());
        if (c.z() > 7.5 && c.x() < 4.5 && c.y() < 4.0) {
            return island.faces;
        }
    }
    return {};
}

/// Faces plus `rings` rings around them (sharing a vertex).
std::vector<Eigen::Index> grown(const Mesh& mesh, std::vector<Eigen::Index> faces, int rings) {
    for (int r = 0; r < rings; ++r) {
        std::vector<std::uint8_t> vertex(static_cast<std::size_t>(mesh.numVertices()), 0);
        for (const Eigen::Index f : faces) {
            for (int c = 0; c < 3; ++c) {
                vertex[static_cast<std::size_t>(mesh.F(f, c))] = 1;
            }
        }
        faces.clear();
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            if (vertex[static_cast<std::size_t>(mesh.F(f, 0))] != 0 || vertex[static_cast<std::size_t>(mesh.F(f, 1))] != 0 ||
                vertex[static_cast<std::size_t>(mesh.F(f, 2))] != 0) {
                faces.push_back(f);
            }
        }
    }
    return faces;
}

/// A phase-2 stage for tests: unites every shell with `mergeShells` (the merge itself is not built yet).
FixAllStage mergeEveryShellStage() {
    FixAllStage stage;
    stage.label = "Merge every shell";
    stage.activity = "merging";
    stage.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        const auto shells = ogun::shellsLargestFirst(mesh);
        outcome.targetBefore = shells.size();
        if (shells.size() < 2) {
            return outcome;
        }
        ogun::ShellMergeResult merged = ogun::mergeShells(mesh, shells);
        if (!merged.success || merged.operandsMerged == 0) {
            outcome.failure = merged.message;
            return outcome;
        }
        // Old face -> the face of the union with the same corners (the union re-orders faces)
        std::unordered_map<ogun::FaceGeometryKey, std::vector<Eigen::Index>, ogun::FaceGeometryKeyHash> rows;
        for (Eigen::Index f = merged.mesh.numFaces() - 1; f >= 0; --f) {
            rows[ogun::faceGeometryKey(merged.mesh, f)].push_back(f);
        }
        outcome.faceMap.assign(static_cast<std::size_t>(mesh.numFaces()), -1);
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            auto it = rows.find(ogun::faceGeometryKey(mesh, f));
            if (it != rows.end() && !it->second.empty()) {
                outcome.faceMap[static_cast<std::size_t>(f)] = it->second.back();
                it->second.pop_back();
            }
        }
        outcome.changed = true;
        outcome.targetAfter = ogun::shellsLargestFirst(merged.mesh).size();
        outcome.mesh = std::move(merged.mesh);
        outcome.detail = "Merged " + std::to_string(shells.size()) + " shells";
        return outcome;
    };
    return stage;
}

ogun::FixAllTestHooks mergeInPassOne() {
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (pass == 1 && phase == FixAllPhase::Merge) {
            stages.push_back(mergeEveryShellStage());
        }
    };
    return hooks;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Selection fixtures
// ---------------------------------------------------------------------------------------------

TEST_CASE("S2: a face selection around one of two holes fills only that hole", "[repair][fixall][scope]") {
    const Mesh grid = SampleMeshes::createGridCubeMesh(4, 4.0);
    const Eigen::Index top = faceNear(grid, {0.4, 0.4, 4.0});
    const Eigen::Index bottom = faceNear(grid, {3.6, 3.6, 0.0});
    const Eigen::Vector3d hole1 = centroid(grid, top);
    const Mesh holed = withoutFaces(grid, {top, bottom});
    REQUIRE(ogun::findHoleLoops(holed).size() == 2);
    const FixAllScope scope = scopeWhere(holed, FixAllScopeKind::Faces, [&](const Eigen::Vector3d& c) { return (c - hole1).norm() < 1.5; });
    REQUIRE(scope.faceCount() > 3);
    REQUIRE(std::count(scope.pinned.begin(), scope.pinned.end(), std::uint8_t{1}) > 0);

    ogun::FixAllParams params;
    params.mergeShells = true; // a face scope never merges
    const ogun::FixAllResult result = ogun::runFixAll(holed, scope, params);
    REQUIRE(result.success);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(result.passesRun == 1);
    REQUIRE(result.countsBefore.openEdges == 3); // only hole 1 touches the selection
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(openEdges(result.mesh) == 3);        // hole 2 is left
    REQUIRE(result.mesh.numFaces() == holed.numFaces() + 1);
    REQUIRE(outsideKept(holed, scope, result.mesh));
    REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 1 of 1 hole");
    const ogun::FixAllStepReport* tiny = stepOf(result, FixAllStageId::TinyShells);
    REQUIRE(tiny->status == FixAllStepStatus::Skipped);
    REQUIRE(tiny->reason == "Needs whole shells: select shells, or clear the selection.");
    // The new face is in the carried scope; the faces outside stay outside
    REQUIRE(result.scope.kind == FixAllScopeKind::Faces);
    REQUIRE(result.scope.faceCount() == scope.faceCount() + 1);
    REQUIRE(result.scope.matches(result.mesh));

    SECTION("A phase-2 stage is skipped for a face scope") {
        const ogun::FixAllTestHooks hooks = mergeInPassOne();
        const ogun::FixAllResult merged = ogun::runFixAll(holed, scope, params, {}, nullptr, &hooks);
        const ogun::FixAllStepReport* merge = stepOf(merged, FixAllStageId::Custom, 1, FixAllPhase::Merge);
        REQUIRE(merge != nullptr);
        REQUIRE(merge->status == FixAllStepStatus::Skipped);
        REQUIRE(merge->reason == "Merge needs whole shells: select shells, or clear the selection.");
        REQUIRE(stepOf(merged, FixAllStageId::Slivers, 1, FixAllPhase::Cleanup) == nullptr);
    }
}

TEST_CASE("S4: a selected container's hole is filled, its unselected cavity stays as it is", "[repair][fixall][scope]") {
    const Mesh box = SampleMeshes::createCavityBoxMesh();
    const auto shells = ogun::shellsLargestFirst(box);
    REQUIRE(shells.size() == 2);
    const Mesh holed = withoutFaces(box, {shells[0].front()}); // a 3-edge hole in A
    const FixAllScope scope = scopeWhere(holed, FixAllScopeKind::Shells, [](const Eigen::Vector3d& c) {
        return c.minCoeff() < 1.0 - 1e-9 || c.maxCoeff() > 3.0 + 1e-9; // A's faces, not C's ([1, 3]^3)
    });
    REQUIRE(scope.faceCount() == 11);

    const ogun::FixAllResult result = ogun::runFixAll(holed, scope, {});
    REQUIRE(result.success);
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(openEdges(result.mesh) == 0);
    REQUIRE(outsideKept(holed, scope, result.mesh));
    REQUIRE(stepOf(result, FixAllStageId::FillHoles)->accepted());
    // C is still a cavity by rule O, read from the whole mesh
    const ogun::OrientShellsResult roles = ogun::orientShells(result.mesh);
    REQUIRE(roles.cavitiesKept.size() == 1);
    REQUIRE(roles.reversedShells.empty());
    REQUIRE(ogun::signedVolume(result.mesh) == Catch::Approx(56.0));
    REQUIRE(result.countsAfter.invertedShells == 0);
}

TEST_CASE("S5: a needle on the selection's edge collapses onto its pinned vertex", "[repair][fixall][scope]") {
    const Mesh grid = SampleMeshes::createGridCubeMesh(4, 4.0);
    const Eigen::Index f = faceNear(grid, {2.4, 2.4, 4.0});
    const Mesh needles = splitNearCorner(grid, f, 0.01);
    const std::vector<Eigen::Index> split = {f, needles.numFaces() - 2, needles.numFaces() - 1};
    const auto newVertex = static_cast<std::size_t>(needles.numVertices() - 1);

    SECTION("The three split faces selected: the free vertex moves onto the pinned corner") {
        const FixAllScope scope = scopeOfFaces(needles, FixAllScopeKind::Faces, split);
        REQUIRE(scope.pinned[newVertex] == 0);
        REQUIRE(scope.pinned[static_cast<std::size_t>(needles.F(f, 0))] == 1);
        const ogun::FixAllResult result = ogun::runFixAll(needles, scope, {});
        REQUIRE(result.success);
        REQUIRE(result.countsBefore.thinFaces == 2);
        REQUIRE(result.countsAfter.thinFaces == 0);
        REQUIRE(stepOf(result, FixAllStageId::Slivers)->accepted());
        REQUIRE(outsideKept(needles, scope, result.mesh));
        REQUIRE(result.mesh.numFaces() == grid.numFaces());
        REQUIRE(result.priorityAfter.clean());
    }
    SECTION("One needle selected: every corner is pinned, so it is not collapsed") {
        const FixAllScope scope = scopeOfFaces(needles, FixAllScopeKind::Faces, {f});
        REQUIRE(scope.pinned[newVertex] == 1);
        const ogun::FixAllResult result = ogun::runFixAll(needles, scope, {});
        REQUIRE(result.success);
        REQUIRE(outsideKept(needles, scope, result.mesh));
        REQUIRE(result.mesh.V == needles.V);
        REQUIRE(result.countsAfter.thinFaces == 1);
        REQUIRE_FALSE(stepOf(result, FixAllStageId::Slivers)->accepted());
    }
}

TEST_CASE("S6: an island whose cut leaves the selection is skipped and named", "[repair][fixall][scope]") {
    const Mesh folded = SampleMeshes::createFoldedCubeMesh();
    const std::vector<Eigen::Index> island = foldIsland(folded);
    REQUIRE_FALSE(island.empty());

    SECTION("Only the island's faces selected: its cut plus one ring is outside") {
        const FixAllScope scope = scopeOfFaces(folded, FixAllScopeKind::Faces, island);
        const ogun::FixAllResult result = ogun::runFixAll(folded, scope, {});
        REQUIRE(result.success);
        REQUIRE(outsideKept(folded, scope, result.mesh));
        const ogun::FixAllStepReport* islands = stepOf(result, FixAllStageId::Islands);
        REQUIRE(islands->status == FixAllStepStatus::Unchanged);
        const std::string name = "Island at face #" + std::to_string(*std::min_element(island.begin(), island.end()));
        REQUIRE(islands->detail == name + " extends beyond the selection");
        REQUIRE(result.countsAfter.intersectingFaces > 0);
    }
    SECTION("The island and two rings selected: it is repaired, nothing outside changes") {
        const FixAllScope scope = scopeOfFaces(folded, FixAllScopeKind::Faces, grown(folded, island, 2));
        const ogun::FixAllResult result = ogun::runFixAll(folded, scope, {});
        REQUIRE(result.success);
        REQUIRE(outsideKept(folded, scope, result.mesh));
        const ogun::FixAllStepReport* islands = stepOf(result, FixAllStageId::Islands);
        REQUIRE(islands->accepted());
        REQUIRE(islands->detail.rfind("Repaired 1 of 1 island", 0) == 0);
        REQUIRE(result.countsAfter.intersectingFaces == 0);
    }
}

TEST_CASE("Weld (h): a face selection whose edge runs along the seam keeps its pinned side still", "[repair][fixall][scope]") {
    const Mesh seam = SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    for (const bool lowerHalf : {true, false}) {
        CAPTURE(lowerHalf);
        const FixAllScope scope = scopeWhere(seam, FixAllScopeKind::Faces, [&](const Eigen::Vector3d& c) {
            return lowerHalf ? c.x() < 15.0 : c.x() > 15.0;
        });
        REQUIRE(scope.faceCount() == 10);
        const ogun::FixAllResult result = ogun::runFixAll(seam, scope, {});
        REQUIRE(result.success);
        const ogun::FixAllStepReport* weld = stepOf(result, FixAllStageId::Weld);
        REQUIRE(weld->accepted());
        REQUIRE(weld->detail.rfind("Welded 4 vertices", 0) == 0);
        REQUIRE(weld->maxMove <= 1.5e-5 * std::sqrt(3.0));
        REQUIRE(openEdges(result.mesh) == 0);
        // The unselected half is bit-identical: the selected half moved onto it
        REQUIRE(outsideKept(seam, scope, result.mesh));
        REQUIRE(result.mesh.numFaces() == 20);
        REQUIRE(std::count(result.scope.pinned.begin(), result.scope.pinned.end(), std::uint8_t{1}) == 4);
    }
}

TEST_CASE("A shell split off under a shell scope stays in scope", "[repair][fixall][scope]") {
    const Mesh touching = SampleMeshes::createTouchingCubesMesh();
    const Mesh far = SampleMeshes::createBox(Eigen::Vector3d::Constant(20.0), Eigen::Vector3d::Constant(21.0));
    const Mesh both = SampleMeshes::combineMeshes({touching, far});
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(both.numFaces()), 1);
    std::fill(mask.end() - 12, mask.end(), std::uint8_t{0});
    const FixAllScope scope = FixAllScope::fromFaceMask(both, FixAllScopeKind::Shells, mask);
    const std::size_t shellsBefore = ogun::shellsLargestFirst(both).size();

    const ogun::FixAllResult result = ogun::runFixAll(both, scope, {});
    REQUIRE(result.success);
    REQUIRE(stepOf(result, FixAllStageId::SplitTouching)->accepted());
    const auto shells = ogun::shellsLargestFirst(result.mesh);
    REQUIRE(shells.size() > shellsBefore);
    // Every shell but the far box is wholly in scope; the far box is wholly outside and unchanged
    for (const auto& shell : shells) {
        const bool farBox = centroid(result.mesh, shell.front()).minCoeff() > 19.0;
        for (const Eigen::Index f : shell) {
            REQUIRE(result.scope.inScope(f) == !farBox);
        }
    }
    REQUIRE(result.scope.faceCount() == static_cast<std::size_t>(result.mesh.numFaces()) - 12);
    REQUIRE(outsideKept(both, scope, result.mesh));
    REQUIRE(result.scope.kind == FixAllScopeKind::Shells);
}

TEST_CASE("A crack whose closing would move a pinned vertex is left for the hole fill", "[repair][fixall][scope]") {
    // The cracked box: a T-junction crack along x = 2 on its top. Select the top's faces on one side of the
    // crack only, so the other side's rim is outside the selection
    const Mesh cracked = ogun::separateTouchingContacts(SampleMeshes::createCrackedBoxMesh());
    const auto cracks = ogun::findHoleLoops(cracked);
    REQUIRE(cracks.size() == 1);
    REQUIRE(cracks[0].shape == ogun::HoleShape::Crack);
    for (const bool left : {true, false}) {
        CAPTURE(left);
        const FixAllScope scope = scopeWhere(cracked, FixAllScopeKind::Faces, [&](const Eigen::Vector3d& c) {
            return c.z() > 3.9 && (c.x() < 2.0) == left;
        });
        ogun::FixAllParams params;
        params.weld = false;
        const ogun::FixAllResult result = ogun::runFixAll(cracked, scope, params);
        REQUIRE(result.success);
        REQUIRE(outsideKept(cracked, scope, result.mesh));
        const ogun::FixAllStepReport* crack = stepOf(result, FixAllStageId::CloseCracks);
        if (left) {
            // Closing it would move the unselected side's rim: skipped and named; a zero-width crack
            // has no fill either, so it stays open
            REQUIRE(crack->status == FixAllStepStatus::Unchanged);
            REQUIRE(crack->detail == "1 crack would move a vertex on the selection's edge: left for the hole fill");
            REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Hole 1 could not be filled");
            REQUIRE(result.countsAfter.openEdges == 6);
            REQUIRE(result.mesh.V == cracked.V);
        } else {
            // The other side is selected: Close Crack closes it without moving a vertex outside
            REQUIRE(crack->accepted());
            REQUIRE(crack->detail == "Closed 1 crack (max vertex move 0 mm)");
            REQUIRE(result.countsAfter.openEdges == 0);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Phase 3: the clean-up after a merge
// ---------------------------------------------------------------------------------------------

TEST_CASE("Phase 3 cleans up a mesh merged by mergeShells", "[repair][fixall][cleanup]") {
    // Two cubes whose union leaves 0.02-wide steps: long thin faces along the cut
    const Mesh cubes = SampleMeshes::combineMeshes(
        {SampleMeshes::createBox(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(10.0)),
         SampleMeshes::createBox(Eigen::Vector3d(5.0, 0.02, 0.02), Eigen::Vector3d(15.0, 10.02, 10.02))});
    const ogun::ShellMergeResult direct = ogun::mergeShells(cubes, ogun::shellsLargestFirst(cubes));
    REQUIRE(direct.success);
    const ogun::FixAllParams defaults;
    const ogun::FixAllPriority mergedP =
        ogun::countFixAllDefects(direct.mesh, FixAllScope::whole(direct.mesh), defaults.checkParams).priority();
    REQUIRE(mergedP.thin > 0);

    const ogun::FixAllTestHooks hooks = mergeInPassOne();
    ogun::FixAllParams params;
    params.passes = 1;
    params.mergeShells = true;
    ogun::FixAllProgress progress;
    std::vector<std::string> texts;
    progress.observer = [&](float, const std::string& text) { texts.push_back(text); };
    const ogun::FixAllResult result = ogun::runFixAll(cubes, FixAllScope::whole(cubes), params, {}, &progress, &hooks);
    REQUIRE(result.success);
    REQUIRE(stepOf(result, FixAllStageId::Custom, 1, FixAllPhase::Merge)->accepted());
    const ogun::FixAllStepReport* slivers = stepOf(result, FixAllStageId::Slivers, 1, FixAllPhase::Cleanup);
    REQUIRE(slivers != nullptr);
    REQUIRE(slivers->accepted());
    REQUIRE(slivers->detail == "Healed 16 of 16 cut slivers (0 flips, 8 collapses)");
    REQUIRE(result.priorityAfter <= mergedP);
    REQUIRE(result.countsAfter.thinFaces < mergedP.thin);
    REQUIRE(ogun::shellsLargestFirst(result.mesh).size() == 1);
    REQUIRE(result.scope.matches(result.mesh));
    REQUIRE(std::find(texts.begin(), texts.end(), "pass 1 of 1, phase 3, healing cut slivers...") != texts.end());
    // Phase 3 runs the clean-up list in order after the merge
    std::vector<FixAllStageId> cleanup;
    for (const ogun::FixAllStepReport& step : result.steps) {
        if (step.phase == FixAllPhase::Cleanup) {
            cleanup.push_back(step.stage);
        }
    }
    std::vector<FixAllStageId> expected;
    for (const FixAllStage& stage : ogun::fixAllCleanupStages()) {
        expected.push_back(stage.id);
    }
    REQUIRE(cleanup == expected);

    SECTION("No phase 3 when nothing was merged") {
        const ogun::FixAllResult plain = ogun::runFixAll(cubes, FixAllScope::whole(cubes), params);
        REQUIRE(std::none_of(plain.steps.begin(), plain.steps.end(),
                             [](const ogun::FixAllStepReport& step) { return step.phase != FixAllPhase::Repair; }));
    }
}

// ---------------------------------------------------------------------------------------------
// Real files and samples
// ---------------------------------------------------------------------------------------------

/// Prints each step of `result` with its time and the guards' share.
void printSteps(const char* name, const ogun::FixAllResult& result) {
    for (const ogun::FixAllStepReport& step : result.steps) {
        WARN(name << " pass " << step.pass << " phase " << static_cast<int>(step.phase) << ": " << step.label << " ["
                  << static_cast<int>(step.status) << "] " << step.detail << step.reason << " | " << step.elapsedMs
                  << " ms, guards " << step.guardMs << " ms");
    }
}

TEST_CASE("Fix All on small real files and samples", "[repair][fixall][files]") {
    SECTION("self_intersecting_box.stl: the corner pulled through the bottom is cut out and filled") {
        const Mesh box = ogun::loadStl(examplePath("self_intersecting_box.stl"));
        const ogun::FixAllResult result = ogun::runFixAll(box, FixAllScope::whole(box), {});
        REQUIRE(result.success);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.priorityBefore.intersecting == 6);
        REQUIRE(result.priorityAfter.clean());
        REQUIRE(stepOf(result, FixAllStageId::Islands)->accepted());
        REQUIRE(box.numFaces() == 12);
        REQUIRE(result.mesh.numFaces() == 16);
        REQUIRE(unchangedFraction(box, result.mesh) == Catch::Approx(5.0 / 12.0));
    }
    SECTION("Folded cube: both islands repaired in one pass") {
        const Mesh folded = SampleMeshes::createFoldedCubeMesh();
        const ogun::FixAllResult result = ogun::runFixAll(folded, FixAllScope::whole(folded), {});
        REQUIRE(result.success);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.priorityBefore.intersecting == 22);
        REQUIRE(result.priorityAfter.clean());
        REQUIRE(stepOf(result, FixAllStageId::Islands)->detail == "Repaired 2 of 2 islands");
        REQUIRE(unchangedFraction(folded, result.mesh) == Catch::Approx(0.9427).margin(1e-4));
    }
    SECTION("Holes sample: the crack closed, three holes filled") {
        const Mesh holes = ogun::separateTouchingContacts(SampleMeshes::createHolesSampleMesh());
        const ogun::FixAllResult result = ogun::runFixAll(holes, FixAllScope::whole(holes), {});
        REQUIRE(result.success);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.countsBefore.openEdges == 62);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(stepOf(result, FixAllStageId::CloseCracks)->detail == "Closed 1 crack (max vertex move 0 mm)");
        REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 3 of 3 holes");
        REQUIRE(ogun::shellsLargestFirst(result.mesh).size() == 4);
        REQUIRE(unchangedFraction(holes, result.mesh) == Catch::Approx(0.998).margin(1e-3));
    }
}

TEST_CASE("Fix All on the bunny: one pass, five holes filled, one cap healed", "[repair][fixall][files][.slow]") {
    const Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    const auto start = std::chrono::steady_clock::now();
    const ogun::FixAllResult result = ogun::runFixAll(bunny, FixAllScope::whole(bunny), {});
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(result.success);
    WARN("bunny Fix All: " << ms << " ms");
    printSteps("bunny", result);
    REQUIRE(result.passesRun == 1);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(stepOf(result, FixAllStageId::Weld)->status == FixAllStepStatus::Unchanged);
    REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 5 of 5 holes");
    REQUIRE(stepOf(result, FixAllStageId::Slivers)->detail == "Healed 1 of 1 thin face (0 flips, 1 collapse)");
    REQUIRE(result.countsBefore.openEdges == 223);
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(result.countsBefore.thinFaces == 1);
    REQUIRE(result.countsAfter.thinFaces == 0);
    REQUIRE(ogun::faceDiff(bunny, result.mesh).removed.size() == 4); // the collapse; every other face is kept
}

TEST_CASE("Fix All on the Turkey with the merge off", "[repair][fixall][files][.slow]") {
    const Mesh turkey = ogun::loadStl(examplePath("Turkey.stl"));
    const auto start = std::chrono::steady_clock::now();
    const ogun::FixAllResult result = ogun::runFixAll(turkey, FixAllScope::whole(turkey), {});
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(result.success);
    double guards = 0.0;
    for (const ogun::FixAllStepReport& step : result.steps) {
        guards += step.guardMs;
    }
    WARN("Turkey Fix All (merge off): " << ms << " ms, guards " << guards << " ms, passes " << result.passesRun
                                        << ", stop " << ogun::fixAllStopReasonLabel(result.stopReason) << ", crossings "
                                        << result.countsBefore.intersectingFaces << " -> " << result.countsAfter.intersectingFaces
                                        << ", thin " << result.countsBefore.thinFaces << " -> " << result.countsAfter.thinFaces);
    printSteps("Turkey", result);
    REQUIRE(result.priorityAfter <= result.priorityBefore);
    REQUIRE(result.countsAfter.intersectingFaces > 0); // crossings between shells: the merge is off
    REQUIRE(result.countsBefore.intersectingFaces == 35628);
    REQUIRE(result.countsAfter.intersectingFaces == 35591);
    REQUIRE(result.countsBefore.thinFaces == 78);
    REQUIRE(result.countsAfter.thinFaces == 18);
    REQUIRE(stepOf(result, FixAllStageId::Islands)->detail == "Repaired 3 of 3 islands");
    REQUIRE(stepOf(result, FixAllStageId::Orient)->detail == "Kept 1 cavity inward: #28");
    REQUIRE(result.passesRun == 2);
    REQUIRE(result.stopReason == FixAllStopReason::RepeatedState);
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(ogun::shellsLargestFirst(result.mesh).size() == 28);
}
