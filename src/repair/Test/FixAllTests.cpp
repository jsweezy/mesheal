#include "repair/FixAll.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ReliableNormal.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "repair/SliverHealer.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/ToleranceConfig.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <functional>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

namespace {

using ogun::FixAllPhase;
using ogun::FixAllStage;
using ogun::FixAllStageOutcome;
using ogun::FixAllStepStatus;
using ogun::FixAllStopReason;
using ogun::Mesh;
using ogun::SampleMeshes;

ogun::FixAllResult run(const Mesh& mesh, const ogun::FixAllParams& params = {}, const ogun::FixAllTestHooks* hooks = nullptr,
                       std::stop_token stop = {}, ogun::FixAllProgress* progress = nullptr) {
    return ogun::runFixAll(mesh, ogun::FixAllScope::whole(mesh), params, std::move(stop), progress, hooks);
}

bool same(const Mesh& a, const Mesh& b) {
    return a.V.rows() == b.V.rows() && a.F.rows() == b.F.rows() && a.V == b.V && a.F == b.F;
}

/// The 12-triangle cube without its last face: one 3-edge hole that no P10 stage can close.
Mesh openCube() {
    Mesh cube = SampleMeshes::createCubeMesh();
    cube.F.conservativeResize(cube.F.rows() - 1, 3);
    cube.recomputeNormals();
    return cube;
}

/// `mesh` with a same-winding copy of each listed face appended.
Mesh withCopies(Mesh mesh, const std::vector<Eigen::Index>& faces) {
    const Eigen::Index first = mesh.F.rows();
    mesh.F.conservativeResize(first + static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t i = 0; i < faces.size(); ++i) {
        mesh.F.row(first + static_cast<Eigen::Index>(i)) = mesh.F.row(faces[i]);
    }
    mesh.recomputeNormals();
    return mesh;
}

Eigen::Index vertexAt(const Mesh& mesh, const Eigen::Vector3d& p) {
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        if ((mesh.V.row(v).transpose() - p).norm() == 0.0) {
            return v;
        }
    }
    return -1;
}

/// A stage that changes the mesh with `change` (which returns false for "nothing to do").
FixAllStage mockStage(std::function<bool(Mesh&, int pass)> change) {
    FixAllStage stage;
    stage.label = "Mock";
    stage.activity = "mocking";
    stage.run = [change = std::move(change)](const Mesh& mesh, const ogun::FixAllStageContext& context) {
        FixAllStageOutcome outcome;
        outcome.mesh = mesh;
        outcome.targetBefore = 1;
        if (!change(outcome.mesh, context.pass)) {
            return outcome;
        }
        outcome.changed = true;
        outcome.targetAfter = 0;
        outcome.mesh.recomputeNormals();
        return outcome;
    };
    return stage;
}

/// Hooks that run only `stage` in every pass's phase 1.
ogun::FixAllTestHooks onlyStage(FixAllStage stage) {
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [stage = std::move(stage)](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (phase == FixAllPhase::Repair) {
            stages.assign(1, stage);
        }
    };
    return hooks;
}

/// Moves the top corner (10, 10, 10) through the bottom (crossings) on odd passes, back on even ones.
bool oscillate(Mesh& mesh, int pass) {
    const Eigen::Index up = vertexAt(mesh, Eigen::Vector3d(10.0, 10.0, 10.0));
    const Eigen::Index down = vertexAt(mesh, Eigen::Vector3d(5.0, 5.0, -5.0));
    if (pass % 2 == 1 && up >= 0) {
        mesh.V.row(up) << 5.0, 5.0, -5.0;
        return true;
    }
    if (pass % 2 == 0 && down >= 0) {
        mesh.V.row(down) << 10.0, 10.0, 10.0;
        return true;
    }
    return false;
}

bool translate(Mesh& mesh, int) {
    mesh.V.col(0).array() += 1.0;
    return true;
}

/// Removes the last face while there are more than 12 (one duplicate per pass).
FixAllStage dropLastFaceStage() {
    FixAllStage stage;
    stage.label = "Drop one copy";
    stage.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        outcome.targetBefore = static_cast<std::size_t>(mesh.numFaces()) - 12;
        if (mesh.numFaces() <= 12) {
            return outcome;
        }
        outcome.changed = true;
        outcome.targetAfter = outcome.targetBefore - 1;
        outcome.mesh = mesh;
        outcome.mesh.F.conservativeResize(mesh.numFaces() - 1, 3);
        outcome.mesh.recomputeNormals();
        outcome.faceMap.resize(static_cast<std::size_t>(mesh.numFaces()));
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            outcome.faceMap[static_cast<std::size_t>(f)] = f + 1 < mesh.numFaces() ? f : -1;
        }
        return outcome;
    };
    return stage;
}

const ogun::FixAllStepReport* stepOf(const ogun::FixAllResult& result, ogun::FixAllStageId id, int pass = 1);
std::size_t shellCount(const Mesh& mesh);

} // namespace

TEST_CASE("Fix All leaves a clean cube alone in one pass", "[repair][fixall]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    const ogun::FixAllResult result = run(cube);
    REQUIRE(result.success);
    REQUIRE(result.passesRun == 1);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(std::string(ogun::fixAllStopReasonLabel(result.stopReason)) == "clean");
    REQUIRE_FALSE(result.changed);
    REQUIRE(same(result.mesh, cube));
    REQUIRE(result.bestPass == 0);
    REQUIRE(result.priorityBefore.clean());
    REQUIRE(result.priorityAfter == result.priorityBefore);
    REQUIRE(result.steps.size() == ogun::fixAllRepairStages().size());
    REQUIRE(result.steps.size() == 12);
    for (const ogun::FixAllStepReport& step : result.steps) {
        REQUIRE(step.status == FixAllStepStatus::Unchanged);
        REQUIRE(step.pass == 1);
        REQUIRE(step.phase == FixAllPhase::Repair);
    }
    REQUIRE(result.scope.matches(result.mesh));
}

TEST_CASE("Fix All's weld uses its own tolerance, never the load weld's", "[repair][fixall]") {
    const ogun::FixAllParams defaults;
    REQUIRE(defaults.weldTolerance.absFloor == 1.0e-6);
    REQUIRE(defaults.weldTolerance.relTol == 1.0e-6);
    REQUIRE(ogun::ToleranceConfig{}.absFloor == 1.0e-9);
    REQUIRE(ogun::ToleranceConfig{}.relTol == 1.0e-9);
    REQUIRE(defaults.passes == 3);
    REQUIRE(defaults.weld);

    const Mesh seam = SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    SECTION("The default weld closes the hairline seam in one pass") {
        const ogun::FixAllResult result = run(seam);
        REQUIRE(result.countsBefore.openEdges == 8);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.priorityAfter.clean());
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.bestPass == 1);
        REQUIRE(result.changed);
        REQUIRE(result.mesh.numFaces() == 20);
        const ogun::FixAllStepReport& weld = result.steps.front();
        REQUIRE(weld.stage == ogun::FixAllStageId::Weld);
        REQUIRE(weld.accepted());
        REQUIRE(weld.countBefore == 4);
        REQUIRE(weld.countAfter == 0);
        REQUIRE(weld.maxMove > 0.0);
        REQUIRE(weld.maxMove <= std::sqrt(3.0) * 2.0e-5);
        REQUIRE(weld.detail.rfind("Welded 4 vertices (max move ", 0) == 0);
    }
    SECTION("The load weld's tolerance would not close it") {
        ogun::FixAllParams params;
        params.weldTolerance = ogun::ToleranceConfig{};
        params.closeCracks = false;
        const ogun::FixAllResult result = run(seam, params);
        REQUIRE(result.steps.front().stage == ogun::FixAllStageId::Weld);
        REQUIRE(result.steps.front().status == FixAllStepStatus::Unchanged);
    }
    SECTION("Unticking the weld skips the step; the fill caps each half instead") {
        // The two open halves have one flat 4-edge hole each, not a crack between them
        ogun::FixAllParams params;
        params.weld = false;
        const ogun::FixAllResult result = run(seam, params);
        REQUIRE(result.steps.front().status == FixAllStepStatus::Skipped);
        REQUIRE_FALSE(result.steps.front().reason.empty());
        REQUIRE(stepOf(result, ogun::FixAllStageId::CloseCracks)->status == FixAllStepStatus::Unchanged);
        REQUIRE(stepOf(result, ogun::FixAllStageId::FillHoles)->detail.rfind("Filled 2 of 2 holes", 0) == 0);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(shellCount(result.mesh) == 2);
    }
}

TEST_CASE("The weld's largest tolerance and its note come from the scope before the run", "[repair][fixall]") {
    SECTION("Near the origin: no note") {
        const Mesh seam = SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
        const ogun::FixAllResult result = run(seam);
        REQUIRE(result.weldPlan.maxTolerance == Catch::Approx(2.0e-5));
        REQUIRE(result.weldPlan.scopeDiagonal == Catch::Approx(10.0 * std::sqrt(3.0)).epsilon(1e-6));
        REQUIRE_FALSE(result.weldPlan.noteNeeded);
        REQUIRE(result.weldPlan.collapsingEdges == 0);
    }
    SECTION("Far from the origin: the tolerance grows, the seam welds, the note shows") {
        const Mesh far = SampleMeshes::createWeldSeamCubeMesh(5.0e-3, Eigen::Vector3d::Constant(1.0e4 - 10.0));
        const ogun::FixAllResult result = run(far);
        REQUIRE(result.weldPlan.maxTolerance == Catch::Approx(1.0e-6 * (1.0e4 + 10.0)));
        REQUIRE(result.weldPlan.noteNeeded);
        REQUIRE(result.weldPlan.diagonalShare() > 1.0e-4);
        REQUIRE(result.countsAfter.openEdges == 0);
    }
    SECTION("A tiny model: the floor is a large share, the note shows") {
        const Mesh tiny = SampleMeshes::createSeamBoxMesh(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0e-3), 0, 5.0e-7);
        const ogun::FixAllResult result = run(tiny);
        REQUIRE(result.weldPlan.maxTolerance == 1.0e-6);
        REQUIRE(result.weldPlan.noteNeeded);
        REQUIRE(result.countsAfter.openEdges == 0);
    }
    SECTION("Edges within the tolerance are counted; a weld that is off shows no note") {
        Mesh needle;
        needle.V.resize(4, 3);
        needle.V << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 5.0e-7, 0.0;
        needle.F.resize(2, 3);
        needle.F << 0, 1, 2, 1, 3, 2;
        needle.recomputeNormals();
        const ogun::FixAllWeldPlan plan =
            ogun::planFixAllWeld(needle, ogun::FixAllScope::whole(needle), ogun::FixAllParams{}.weldTolerance);
        REQUIRE(plan.collapsingEdges == 1);

        ogun::FixAllParams params;
        params.weld = false;
        const Mesh far = SampleMeshes::createWeldSeamCubeMesh(5.0e-3, Eigen::Vector3d::Constant(1.0e4 - 10.0));
        REQUIRE_FALSE(run(far, params).weldPlan.noteNeeded);
    }
}

TEST_CASE("Fix All stops when clean, on a repeated state, without progress, at the pass limit or the time budget",
          "[repair][fixall]") {
    SECTION("Clean: dedupe removes a copy and the run stops after pass 1") {
        const Mesh doubled = withCopies(SampleMeshes::createCubeMesh(), {0});
        const ogun::FixAllResult result = run(doubled);
        REQUIRE(result.countsBefore.duplicateFaces == 1);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.changed);
        REQUIRE(result.mesh.numFaces() == 12);
        REQUIRE(result.scope.matches(result.mesh));
    }
    SECTION("Repeated state: a stage that adds and removes a crossing in turn (oscillation)") {
        const Mesh open = openCube();
        const ogun::FixAllTestHooks hooks = onlyStage(mockStage(oscillate));
        ogun::FixAllParams params;
        params.passes = 5;
        const ogun::FixAllResult result = run(open, params, &hooks);
        REQUIRE(result.passes.at(0).after.intersecting > 0);
        REQUIRE(result.stopReason == FixAllStopReason::RepeatedState);
        REQUIRE(result.passesRun == 2);
        REQUIRE(result.passes.at(1).repeated);
        REQUIRE(result.bestPass == 0);
        REQUIRE(same(result.mesh, open));
        REQUIRE_FALSE(result.changed);
    }
    SECTION("No progress: two passes in a row that do not beat the best") {
        const Mesh open = openCube();
        const ogun::FixAllTestHooks hooks = onlyStage(mockStage(translate));
        ogun::FixAllParams params;
        params.passes = 5;
        const ogun::FixAllResult result = run(open, params, &hooks);
        REQUIRE(result.stopReason == FixAllStopReason::NoProgress);
        REQUIRE(result.passesRun == 2);
        REQUIRE(same(result.mesh, open));
    }
    SECTION("Pass limit: one duplicate removed per pass") {
        const Mesh doubled = withCopies(SampleMeshes::createCubeMesh(), {0, 1, 2});
        const ogun::FixAllTestHooks hooks = onlyStage(dropLastFaceStage());
        ogun::FixAllParams params;
        params.passes = 2;
        const ogun::FixAllResult result = run(doubled, params, &hooks);
        REQUIRE(result.stopReason == FixAllStopReason::PassLimit);
        REQUIRE(result.passesRun == 2);
        REQUIRE(result.bestPass == 2);
        REQUIRE(result.mesh.numFaces() == 13);
        REQUIRE(result.countsAfter.duplicateFaces == 1);
        REQUIRE(result.passes.at(0).newBest);
        REQUIRE(result.passes.at(1).newBest);
    }
    SECTION("Time budget: stops after the pass that ends past it") {
        const Mesh doubled = withCopies(SampleMeshes::createCubeMesh(), {0, 1, 2});
        const ogun::FixAllTestHooks hooks = onlyStage(dropLastFaceStage());
        ogun::FixAllParams params;
        params.passes = 3;
        params.timeBudgetSeconds = 1.0e-9;
        const ogun::FixAllResult result = run(doubled, params, &hooks);
        REQUIRE(result.stopReason == FixAllStopReason::TimeBudget);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.mesh.numFaces() == 14);
    }
    SECTION("Passes are clamped to 1..10") {
        const Mesh doubled = withCopies(SampleMeshes::createCubeMesh(), {0, 1, 2});
        const ogun::FixAllTestHooks hooks = onlyStage(dropLastFaceStage());
        ogun::FixAllParams params;
        params.passes = 0;
        const ogun::FixAllResult result = run(doubled, params, &hooks);
        REQUIRE(result.onePass);
        REQUIRE(result.passesRun == 1);
    }
}

TEST_CASE("One pass returns a worse pass; a full run returns the input when later passes do not recover",
          "[repair][fixall]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    // Pass 1 pulls a corner through the bottom (crossings); later passes only move the mesh
    const auto worsen = [](Mesh& mesh, int pass) { return pass == 1 ? oscillate(mesh, 1) : translate(mesh, pass); };
    const ogun::FixAllTestHooks hooks = onlyStage(mockStage(worsen));

    ogun::FixAllParams onePass;
    onePass.passes = 1;
    const ogun::FixAllResult stepped = run(cube, onePass, &hooks);
    REQUIRE(stepped.success);
    REQUIRE(stepped.onePass);
    REQUIRE(stepped.temporarilyWorse);
    REQUIRE(stepped.changed);
    REQUIRE(stepped.priorityAfter.intersecting > 0);
    REQUIRE(stepped.priorityBefore < stepped.priorityAfter);
    REQUIRE(stepped.stopReason == FixAllStopReason::PassLimit);
    REQUIRE(stepped.bestPass == 1);
    REQUIRE(stepped.report.passesLine == "Passes: 1 (One pass)");
    REQUIRE(stepped.report.temporarilyWorseText ==
            "This pass left more crossing faces (0 -> " + std::to_string(stepped.countsAfter.intersectingFaces) +
                "). The next pass should repair them: run one more pass.");
    REQUIRE(stepped.report.lines.at(2).warning);
    REQUIRE(stepped.report.lines.at(2).text == stepped.report.temporarilyWorseText);

    const ogun::FixAllResult full = run(cube, ogun::FixAllParams{}, &hooks);
    REQUIRE_FALSE(full.onePass);
    REQUIRE_FALSE(full.temporarilyWorse);
    REQUIRE(full.bestPass == 0);
    REQUIRE(full.stopReason == FixAllStopReason::NoProgress);
    REQUIRE(full.passesRun == 2);
    REQUIRE_FALSE(full.changed);
    REQUIRE(same(full.mesh, cube));
    REQUIRE(full.priorityAfter == full.priorityBefore);
}

TEST_CASE("A full Fix All run is never worse than its input", "[repair][fixall]") {
    const std::vector<Mesh> fixtures = {
        SampleMeshes::createCubeMesh(),
        SampleMeshes::createWeldSeamCubeMesh(5.0e-6),
        SampleMeshes::createTouchingCubesMesh(),
        SampleMeshes::createDoubledFacesCubeMesh(),
        SampleMeshes::createSampleDefectiveMesh(),
        SampleMeshes::createCavityBoxMesh(),
        SampleMeshes::createCavityBoxMesh(true),
        SampleMeshes::createFoldedCubeMesh(),
        SampleMeshes::createCavityHoleSliversIntersectionsMesh(),
        SampleMeshes::createFinSeamBoxesMesh(),
        SampleMeshes::createCrackedFlippedCubeSoup(),
        SampleMeshes::createCollinearCapCubeMesh(),
        SampleMeshes::createBoundaryCapPlateMesh(),
        SampleMeshes::createTinyFloatingShellsMesh(),
        SampleMeshes::createNestedSolidAndCavityMesh(),
    };
    for (std::size_t i = 0; i < fixtures.size(); ++i) {
        CAPTURE(i);
        const ogun::FixAllResult result = run(fixtures[i]);
        REQUIRE(result.success);
        REQUIRE(result.priorityAfter <= result.priorityBefore);
        REQUIRE(result.scope.matches(result.mesh));
        const ogun::FixAllCounts recount = ogun::countFixAllDefects(
            result.mesh, result.scope, ogun::FixAllParams{}.checkParams, ogun::FixAllParams{}.tinyShellFraction);
        REQUIRE(recount.priority() == result.priorityAfter);
        REQUIRE(result.passesRun >= 1);
        REQUIRE(result.passesRun <= 3);
    }
}

TEST_CASE("Fix All splits touching contacts and removes collapsed and duplicate faces", "[repair][fixall]") {
    SECTION("Touching cubes: touching edges and the pinched vertex are split") {
        const Mesh touching = SampleMeshes::createTouchingCubesMesh();
        const ogun::FixAllResult result = run(touching);
        REQUIRE(result.countsBefore.nonManifoldVertices > 0);
        REQUIRE(result.countsAfter.nonManifoldVertices == 0);
        REQUIRE(result.countsAfter.nonManifoldEdges == 0); // the fin went in the fin step
        const ogun::FixAllStepReport& split = result.steps.at(3);
        REQUIRE(split.stage == ogun::FixAllStageId::SplitTouching);
        REQUIRE(split.accepted());
        REQUIRE(split.detail.rfind("Split 2 touching edges and ", 0) == 0);
        REQUIRE(result.mesh.numFaces() == touching.numFaces() - 1); // the fin
    }
    SECTION("Doubled faces cube: the collapsed face and both copies go") {
        const Mesh doubled = SampleMeshes::createDoubledFacesCubeMesh();
        const ogun::FixAllResult result = run(doubled);
        REQUIRE(result.countsAfter.duplicateFaces == 0);
        REQUIRE(result.countsAfter.degenerateFaces == 0); // the flat cap: healed by the sliver stage, not deleted
        std::size_t removed = 0;
        for (const ogun::FixAllStepReport& step : result.steps) {
            if (step.stage == ogun::FixAllStageId::Weld || step.stage == ogun::FixAllStageId::DropRepeated ||
                step.stage == ogun::FixAllStageId::Dedupe) {
                removed += step.facesRemoved;
            }
        }
        REQUIRE(removed == 3);
        REQUIRE(result.priorityAfter.clean());
        REQUIRE(result.countsAfter.openEdges == 0);
    }
}

TEST_CASE("Fix All follows a shell scope and carries it through each step", "[repair][fixall]") {
    // Two cubes apart, each with a copy of one face; only the first cube is in scope
    const Mesh a = withCopies(SampleMeshes::createBox(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)), {0});
    const Mesh b = withCopies(SampleMeshes::createBox(Eigen::Vector3d::Constant(5.0), Eigen::Vector3d::Constant(6.0)), {0});
    const Mesh both = SampleMeshes::combineMeshes({a, b});
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(both.numFaces()), 0);
    std::fill(mask.begin(), mask.begin() + a.numFaces(), std::uint8_t{1});
    const ogun::FixAllScope scope = ogun::FixAllScope::fromFaceMask(both, ogun::FixAllScopeKind::Shells, mask);
    REQUIRE(scope.faceCount() == 13);
    REQUIRE(std::count(scope.pinned.begin(), scope.pinned.end(), std::uint8_t{1}) == 0);

    const ogun::FixAllResult result = ogun::runFixAll(both, scope, ogun::FixAllParams{});
    REQUIRE(result.success);
    REQUIRE(result.countsBefore.duplicateFaces == 1);
    REQUIRE(result.countsAfter.duplicateFaces == 0);
    REQUIRE(result.mesh.numFaces() == both.numFaces() - 1);
    REQUIRE(result.scope.faceCount() == 12);
    REQUIRE(result.scope.kind == ogun::FixAllScopeKind::Shells);
    // b's faces (all out of scope) are the last 13 rows, unchanged
    for (Eigen::Index f = 0; f < b.numFaces(); ++f) {
        const Eigen::Index out = result.mesh.numFaces() - b.numFaces() + f;
        REQUIRE_FALSE(result.scope.inScope(out));
        for (int c = 0; c < 3; ++c) {
            REQUIRE(result.mesh.V.row(result.mesh.F(out, c)) == both.V.row(both.F(a.numFaces() + f, c)));
        }
    }
}

TEST_CASE("The Fix All scope maps faces through a step and pins its edge", "[repair][fixall]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    std::vector<std::uint8_t> mask(12, 0);
    mask[0] = 1;
    mask[1] = 1;
    ogun::FixAllScope scope = ogun::FixAllScope::fromFaceMask(cube, ogun::FixAllScopeKind::Faces, mask);
    REQUIRE(scope.matches(cube));
    REQUIRE(scope.faceCount() == 2);
    // Every vertex of the two selected faces is also used by unselected faces
    std::vector<std::uint8_t> used(8, 0);
    for (Eigen::Index f = 0; f < 2; ++f) {
        for (int c = 0; c < 3; ++c) {
            used[static_cast<std::size_t>(cube.F(f, c))] = 1;
        }
    }
    REQUIRE(scope.pinned == used);

    // Face 0 removed, the others shifted down by one, one new face appended
    Mesh next = cube;
    next.F.conservativeResize(12, 3);
    next.F.topRows(11) = cube.F.bottomRows(11);
    next.F.row(11) << 0, 1, 2;
    std::vector<Eigen::Index> faceMap(12);
    faceMap[0] = -1;
    for (Eigen::Index f = 1; f < 12; ++f) {
        faceMap[static_cast<std::size_t>(f)] = f - 1;
    }
    scope.carry(faceMap, next);
    REQUIRE(scope.matches(next));
    REQUIRE(scope.inScope(0));     // old face 1
    REQUIRE_FALSE(scope.inScope(1));
    REQUIRE(scope.inScope(11));    // new
    REQUIRE(scope.faceCount() == 2);

    REQUIRE_FALSE(ogun::FixAllScope::fromFaceMask(cube, ogun::FixAllScopeKind::Faces, std::vector<std::uint8_t>(3, 1)).matches(cube));
}

TEST_CASE("Fix All refuses a step that fails, returns a bad result or does not reduce its target", "[repair][fixall]") {
    const Mesh doubled = withCopies(SampleMeshes::createCubeMesh(), {0});

    FixAllStage throws;
    throws.label = "Throws";
    throws.run = [](const Mesh&, const ogun::FixAllStageContext&) -> FixAllStageOutcome {
        throw std::runtime_error("boom");
    };
    FixAllStage badIndices;
    badIndices.label = "Bad indices";
    badIndices.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        outcome.mesh = mesh;
        outcome.mesh.F(0, 0) = 99;
        outcome.changed = true;
        outcome.targetBefore = 1;
        return outcome;
    };
    FixAllStage churn;
    churn.label = "Churn";
    churn.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        outcome.mesh = mesh;
        outcome.mesh.V.col(0).array() += 1.0;
        outcome.changed = true;
        outcome.targetBefore = 2;
        outcome.targetAfter = 2;
        return outcome;
    };
    FixAllStage shortMap;
    shortMap.label = "Short map";
    shortMap.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        outcome.mesh = mesh;
        outcome.mesh.F.conservativeResize(mesh.numFaces() - 1, 3);
        outcome.changed = true;
        outcome.targetBefore = 1;
        return outcome;
    };

    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [&](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (phase == FixAllPhase::Repair) {
            stages.insert(stages.begin(), {throws, badIndices, churn, shortMap});
        }
    };
    ogun::FixAllParams params;
    params.passes = 1;
    const ogun::FixAllResult result = run(doubled, params, &hooks);
    REQUIRE(result.success);
    for (std::size_t i = 0; i < 4; ++i) {
        CAPTURE(i);
        REQUIRE(result.steps.at(i).status == FixAllStepStatus::Refused);
        REQUIRE_FALSE(result.steps.at(i).reason.empty());
    }
    REQUIRE(result.steps.at(0).reason == "It failed: boom");
    // The real stages still ran on the unchanged mesh: dedupe removed the copy
    REQUIRE(result.mesh.numFaces() == 12);
    REQUIRE(result.mesh.V == doubled.V);
    REQUIRE(result.priorityAfter.clean());
}

TEST_CASE("A cancelled Fix All returns the input unchanged", "[repair][fixall]") {
    const Mesh seam = SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    SECTION("Cancelled before it starts") {
        std::stop_source source;
        source.request_stop();
        const ogun::FixAllResult result = run(seam, {}, nullptr, source.get_token());
        REQUIRE(result.cancelled);
        REQUIRE_FALSE(result.success);
        REQUIRE(result.stopReason == FixAllStopReason::Cancelled);
        REQUIRE(same(result.mesh, seam));
    }
    SECTION("Cancelled after a step changed the mesh") {
        std::stop_source source;
        ogun::FixAllTestHooks hooks;
        hooks.adjustStages = [&](int, FixAllPhase phase, std::vector<FixAllStage>& stages) {
            if (phase != FixAllPhase::Repair) {
                return;
            }
            stages.insert(stages.begin() + 1, mockStage([&](Mesh&, int) {
                              source.request_stop();
                              return false;
                          }));
        };
        const ogun::FixAllResult result = run(seam, {}, &hooks, source.get_token());
        REQUIRE(result.cancelled);
        REQUIRE_FALSE(result.changed);
        REQUIRE(same(result.mesh, seam));
        REQUIRE(result.scope.matches(seam));
    }
}

TEST_CASE("Fix All progress never runs backwards and reaches 1", "[repair][fixall]") {
    const Mesh open = openCube();
    const ogun::FixAllTestHooks hooks = onlyStage(mockStage(translate));
    ogun::FixAllProgress progress;
    std::vector<float> values;
    std::vector<std::string> texts;
    progress.observer = [&](float value, const std::string& text) {
        values.push_back(value);
        texts.push_back(text);
    };
    ogun::FixAllParams params;
    params.passes = 3;
    const ogun::FixAllResult result = run(open, params, &hooks, {}, &progress);
    REQUIRE(result.stopReason == FixAllStopReason::NoProgress);
    REQUIRE(values.size() > 4);
    for (std::size_t i = 1; i < values.size(); ++i) {
        REQUIRE(values[i] >= values[i - 1]);
    }
    REQUIRE(values.back() == 1.0f);
    REQUIRE(progress.fraction.load() == 1.0f);
    REQUIRE(std::find(texts.begin(), texts.end(), "pass 2 of 3, phase 1, mocking...") != texts.end());
    REQUIRE(std::find(texts.begin(), texts.end(), "pass 1 of 3, checking the result...") != texts.end());

    // The default stages name themselves
    ogun::FixAllProgress plain;
    std::vector<std::string> plainTexts;
    plain.observer = [&](float, const std::string& text) { plainTexts.push_back(text); };
    (void)run(SampleMeshes::createCubeMesh(), {}, nullptr, {}, &plain);
    REQUIRE(std::find(plainTexts.begin(), plainTexts.end(), "pass 1 of 3, phase 1, welding...") != plainTexts.end());
    REQUIRE(plain.text() == "done");
}

TEST_CASE("Fix All refuses invalid input", "[repair][fixall]") {
    Mesh bad = SampleMeshes::createCubeMesh();
    bad.F(0, 0) = 50;
    const ogun::FixAllResult invalid = ogun::runFixAll(bad, ogun::FixAllScope::whole(bad), {});
    REQUIRE_FALSE(invalid.success);
    REQUIRE(invalid.stopReason == FixAllStopReason::InvalidInput);

    const Mesh cube = SampleMeshes::createCubeMesh();
    const Mesh other = openCube();
    const ogun::FixAllResult mismatch = ogun::runFixAll(cube, ogun::FixAllScope::whole(other), {});
    REQUIRE_FALSE(mismatch.success);
    REQUIRE(mismatch.stopReason == FixAllStopReason::InvalidInput);
    REQUIRE(same(mismatch.mesh, cube));
}

TEST_CASE("The mesh geometry hash ignores numbering", "[repair][fixall]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    Mesh reordered = cube;
    reordered.F.row(0) = cube.F.row(11);
    reordered.F.row(11) = cube.F.row(0);
    reordered.F.row(3) << cube.F(3, 1), cube.F(3, 2), cube.F(3, 0); // same face, rotated
    REQUIRE(ogun::meshGeometryHash(reordered) == ogun::meshGeometryHash(cube));
    REQUIRE(ogun::meshGeometryHash(openCube()) != ogun::meshGeometryHash(cube));
}


// ---------------------------------------------------------------------------------------------
// Phase 1 stages, hard guards, deferral and interaction fixtures
// ---------------------------------------------------------------------------------------------

namespace {

using ogun::FixAllGuard;
using ogun::FixAllStageId;

/// The report of `id` in `pass` (the first one), or nullptr.
const ogun::FixAllStepReport* stepOf(const ogun::FixAllResult& result, FixAllStageId id, int pass) {
    const auto it = std::find_if(result.steps.begin(), result.steps.end(), [&](const ogun::FixAllStepReport& step) {
        return step.stage == id && step.pass == pass;
    });
    return it == result.steps.end() ? nullptr : &*it;
}

std::size_t shellCount(const Mesh& mesh) {
    return ogun::shellsLargestFirst(mesh).size();
}

/// Hooks that move the stage `id` to just before `before` in pass `pass`.
ogun::FixAllTestHooks moveStage(FixAllStageId id, FixAllStageId before, int pass) {
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [=](int p, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (p != pass || phase != FixAllPhase::Repair) {
            return;
        }
        const auto from = std::find_if(stages.begin(), stages.end(), [&](const FixAllStage& s) { return s.id == id; });
        const FixAllStage moved = *from;
        stages.erase(from);
        const auto to = std::find_if(stages.begin(), stages.end(), [&](const FixAllStage& s) { return s.id == before; });
        stages.insert(to, moved);
    };
    return hooks;
}

/// Hooks that insert `stage` at the front of pass 1 only.
ogun::FixAllTestHooks firstInPassOne(FixAllStage stage) {
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [stage = std::move(stage)](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (pass == 1 && phase == FixAllPhase::Repair) {
            stages.insert(stages.begin(), stage);
        }
    };
    return hooks;
}

/// A stage that applies `change` (with an identity face map) and claims to have lowered its target.
FixAllStage changingStage(std::function<void(Mesh&)> change, bool movesVertices) {
    FixAllStage stage;
    stage.label = "Hooked";
    stage.movesVertices = movesVertices;
    stage.run = [change = std::move(change)](const Mesh& mesh, const ogun::FixAllStageContext&) {
        FixAllStageOutcome outcome;
        outcome.mesh = mesh;
        change(outcome.mesh);
        outcome.mesh.recomputeNormals();
        outcome.changed = true;
        outcome.targetBefore = 1;
        return outcome;
    };
    return stage;
}

} // namespace

TEST_CASE("Fix All fills a cup's opening with a fill that crosses nothing, else repairs the crossing in the same pass",
          "[repair][fixall]") {
    SECTION("Post below the opening: a fill that crosses nothing is kept") {
        const ogun::FixAllResult result = run(SampleMeshes::createCupWithPostMesh(SampleMeshes::CupPost::BelowOpening));
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        const ogun::FixAllStepReport* fill = stepOf(result, FixAllStageId::FillHoles);
        REQUIRE(fill->accepted());
        REQUIRE(fill->detail == "Filled 1 of 1 hole");
        REQUIRE(fill->deferred.empty());
        REQUIRE(stepOf(result, FixAllStageId::Islands)->status == FixAllStepStatus::Unchanged);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.countsAfter.intersectingFaces == 0);
    }
    SECTION("Post through the opening: the fewest-crossing fill is kept and the island step repairs it") {
        const ogun::FixAllResult result = run(SampleMeshes::createCupWithPostMesh(SampleMeshes::CupPost::ThroughOpening));
        const ogun::FixAllStepReport* fill = stepOf(result, FixAllStageId::FillHoles);
        REQUIRE(fill->accepted());
        REQUIRE(fill->detail == "Filled 1 of 1 hole (hole 1: every fill crosses its shell, Fill Flat used, left for the island step)");
        REQUIRE(fill->deferred.size() == 1);
        REQUIRE(fill->deferred[0].kind == ogun::FixAllDeferredKind::CrossingsWithinShell);
        REQUIRE(fill->deferred[0].count == 68);
        const ogun::FixAllStepReport* islands = stepOf(result, FixAllStageId::Islands);
        REQUIRE(islands->accepted());
        REQUIRE(islands->detail == "Repaired 1 of 1 island");
        REQUIRE(result.passesRun == 2);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.countsAfter.intersectingFaces == 0);
        REQUIRE(result.countsAfter.thinFaces == 0);
    }
}

TEST_CASE("Fix All always fills a hole, even when every fill crosses another shell", "[repair][fixall]") {
    // The rod through the cup's opening is a second shell: every fill crosses it
    const Mesh cup = SampleMeshes::createCupWithPostMesh(SampleMeshes::CupPost::RodThroughOpening);
    SECTION("Merge off: the fill is kept and the crossing reported with both shells") {
        const ogun::FixAllResult result = run(cup);
        const ogun::FixAllStepReport* fill = stepOf(result, FixAllStageId::FillHoles);
        REQUIRE(fill->accepted());
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.countsAfter.intersectingFaces > 0);
        REQUIRE(fill->deferred.size() == 1);
        REQUIRE(fill->deferred[0].kind == ogun::FixAllDeferredKind::CrossingsWithShell);
        REQUIRE(fill->deferred[0].text == "10 crossings with other shells: reported (the merge is off)");
        REQUIRE(fill->notes.size() == 1);
        REQUIRE(fill->notes[0].text == "#1's hole 1: its patch crosses #2, reported (the merge is off)");
        REQUIRE(fill->notes[0].shells.size() == 2);
        REQUIRE(fill->notes[0].shells[1].size() == 12); // the rod
        REQUIRE(result.bestPass == 1);
    }
    SECTION("Merge on: the crossing is left for the merge") {
        ogun::FixAllParams params;
        params.mergeShells = true;
        const ogun::FixAllResult result = run(cup, params);
        const ogun::FixAllStepReport* fill = stepOf(result, FixAllStageId::FillHoles);
        REQUIRE(fill->accepted());
        REQUIRE(fill->notes.at(0).text == "#1's hole 1: its patch crosses #2, left for the merge");
    }
}

TEST_CASE("Duplicate vertices with flipped faces and a hole are welded then oriented then filled in one pass", "[repair][fixall]") {
    const Mesh soup = SampleMeshes::createCrackedFlippedCubeSoup();
    REQUIRE(soup.numFaces() == 47);
    SECTION("The weld closes both cracks") {
        const ogun::FixAllResult result = run(soup);
        REQUIRE(result.countsBefore.flippedFaces == 2);
        REQUIRE(result.countsBefore.openEdges == 11);
        REQUIRE(stepOf(result, FixAllStageId::Weld)->detail.rfind("Welded 2 vertices", 0) == 0);
        REQUIRE(stepOf(result, FixAllStageId::Orient)->detail == "Flipped 2 faces in 1 shell");
        REQUIRE(stepOf(result, FixAllStageId::CloseCracks)->status == FixAllStepStatus::Unchanged);
        REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 1 of 1 hole");
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.countsAfter.flippedFaces == 0);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.mesh.numFaces() == 48);
    }
    SECTION("Weld unticked: Close Crack closes a crack instead, with the same result") {
        ogun::FixAllParams params;
        params.weld = false;
        const ogun::FixAllResult result = run(soup, params);
        REQUIRE(stepOf(result, FixAllStageId::CloseCracks)->accepted());
        REQUIRE(stepOf(result, FixAllStageId::CloseCracks)->maxMove < 1.0e-6);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        REQUIRE(result.countsAfter.flippedFaces == 0);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.mesh.numFaces() == 48);
    }
    SECTION("Filling before orienting gives the same result: the fill reads only a ring around the hole") {
        const ogun::FixAllTestHooks hooks = moveStage(FixAllStageId::FillHoles, FixAllStageId::Orient, 1);
        ogun::FixAllParams params;
        params.passes = 1;
        const ogun::FixAllResult result = run(soup, params, &hooks);
        REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 1 of 1 hole");
        REQUIRE(stepOf(result, FixAllStageId::Orient)->detail == "Flipped 2 faces in 1 shell");
        REQUIRE(result.countsAfter.flippedFaces == 0);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.mesh.numFaces() == 48);
    }
}

TEST_CASE("Fix All heals a zero-area cap by flipping it, where deleting it would open a crack", "[repair][fixall]") {
    const Mesh capped = SampleMeshes::createCollinearCapCubeMesh();
    const Eigen::Index cap = capped.numFaces() - 1;
    REQUIRE_FALSE(ogun::hasReliableNormal(capped.V.row(capped.F(cap, 0)).transpose(), capped.V.row(capped.F(cap, 1)).transpose(),
                                          capped.V.row(capped.F(cap, 2)).transpose()));
    // Deleting the cap opens a 3-edge crack
    Mesh deleted = capped;
    deleted.F.conservativeResize(cap, 3);
    REQUIRE(ogun::runCategoryCheck(ogun::DefectCategory::OpenEdges, deleted, {}).infoCount == 3);

    SECTION("The sliver step flips it") {
        const ogun::FixAllResult result = run(capped);
        REQUIRE(result.countsBefore.degenerateFaces == 1);
        const ogun::FixAllStepReport* slivers = stepOf(result, FixAllStageId::Slivers);
        REQUIRE(slivers->accepted());
        REQUIRE(slivers->detail == "Healed 1 of 1 thin face (1 flip, 0 collapses)");
        REQUIRE(slivers->deferred.empty());
        REQUIRE(result.countsAfter.degenerateFaces == 0);
        REQUIRE(result.countsAfter.openEdges == 0);
        REQUIRE(result.mesh.numFaces() == capped.numFaces());
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
    }
    SECTION("Healing before orienting: a flipped face makes the healer read a polygon soup, so the step is refused") {
        Mesh flipped = capped;
        flipped.F.row(0) << capped.F(0, 0), capped.F(0, 2), capped.F(0, 1);
        flipped.recomputeNormals();
        const ogun::FixAllTestHooks hooks = moveStage(FixAllStageId::Slivers, FixAllStageId::Orient, 1);
        const ogun::FixAllResult result = run(flipped, {}, &hooks);
        const ogun::FixAllStepReport* first = stepOf(result, FixAllStageId::Slivers, 1);
        REQUIRE(first->status == FixAllStepStatus::Refused);
        REQUIRE(first->guard == FixAllGuard::Failed);
        REQUIRE(first->reason == "The healer had to read the mesh as a polygon soup, which loses the face numbers.");
        // Oriented at the end of pass 1, the cap is healed in pass 2
        REQUIRE(stepOf(result, FixAllStageId::Slivers, 2)->accepted());
        REQUIRE(result.passesRun == 2);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.countsAfter.degenerateFaces == 0);
    }
}

TEST_CASE("Fix All heals a cap on a hole's rim and fills the hole in the same pass", "[repair][fixall]") {
    const Mesh plate = SampleMeshes::createBoundaryCapPlateMesh();
    const std::vector<ogun::HoleLoop> before = ogun::findHoleLoops(plate);
    REQUIRE(before.size() == 1);
    REQUIRE(before[0].edgeCount() == 4);
    // Ogun's healer dissolves the cap's apex before CGAL's clean-up pass could remove the cap: the hole
    // keeps its 4 edges (CGAL's `remove_almost_degenerate_faces` would enlarge it)
    const ogun::RepairResults all = ogun::SliverHealer::healAllThinFaces(plate);
    REQUIRE(all.success);
    REQUIRE(all.facesRemoved == 1);
    REQUIRE_FALSE(all.usedPmpCleanup);
    const std::vector<ogun::HoleLoop> afterAll = ogun::findHoleLoops(all.mesh);
    REQUIRE(afterAll.size() == 1);
    REQUIRE(afterAll[0].edgeCount() == 4);

    const ogun::FixAllResult result = run(plate);
    REQUIRE(result.countsBefore.thinFaces == 1);
    REQUIRE(stepOf(result, FixAllStageId::Slivers)->detail == "Healed 1 of 1 thin face (0 flips, 0 collapses)");
    REQUIRE(stepOf(result, FixAllStageId::FillHoles)->detail == "Filled 1 of 1 hole");
    REQUIRE(result.passesRun == 1);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(result.countsAfter.thinFaces == 0);
}

TEST_CASE("Fix All removes a wall that holds a seam and fills the seam it opens", "[repair][fixall]") {
    const ogun::FixAllResult result = run(SampleMeshes::createFinSeamBoxesMesh());
    const ogun::FixAllStepReport* fins = stepOf(result, FixAllStageId::Fins);
    REQUIRE(fins->accepted());
    REQUIRE(fins->detail == "Removed 2 internal-wall faces (opened 1 seam edge, filled below)");
    REQUIRE(fins->deferred.size() == 1);
    REQUIRE(fins->deferred[0].kind == ogun::FixAllDeferredKind::OpenEdges);
    REQUIRE(fins->deferred[0].text == "1 more open edge: left for the hole fill");
    REQUIRE(stepOf(result, FixAllStageId::FillHoles)->accepted());
    REQUIRE(result.passesRun <= 2);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(result.countsAfter.nonManifoldEdges == 0);
}

TEST_CASE("Fix All deletes only the tiny shell that floats", "[repair][fixall]") {
    const Mesh debris = SampleMeshes::createTinyFloatingShellsMesh();
    SECTION("At 0.1 %: the far tetrahedron") {
        const ogun::FixAllResult result = run(debris);
        const ogun::FixAllStepReport* tiny = stepOf(result, FixAllStageId::TinyShells);
        REQUIRE(tiny->accepted());
        REQUIRE(tiny->detail == "Deleted 1 tiny floating shell (0.058 % of the model's size)");
        REQUIRE(tiny->facesRemoved == 4);
        REQUIRE(result.countsBefore.tinyShells == 1);
        REQUIRE(result.countsAfter.tinyShells == 0);
        REQUIRE(result.bestPass == 1);
        REQUIRE(result.mesh.numFaces() == debris.numFaces() - 4);
        REQUIRE(shellCount(result.mesh) == 4); // the small cube, the cavity and the touching tetrahedron stay
        // The cavity is kept inward
        REQUIRE(stepOf(result, FixAllStageId::Orient)->notes.at(0).text == "kept 1 cavity inward: #3");
    }
    SECTION("At 0: none") {
        ogun::FixAllParams params;
        params.tinyShellFraction = 0.0;
        const ogun::FixAllResult result = run(debris, params);
        REQUIRE(stepOf(result, FixAllStageId::TinyShells)->status == FixAllStepStatus::Unchanged);
        REQUIRE(same(result.mesh, debris));
    }
    SECTION("Off for a face scope") {
        const ogun::FixAllScope scope = ogun::FixAllScope::fromFaceMask(debris, ogun::FixAllScopeKind::Faces, {});
        const ogun::FixAllResult result = ogun::runFixAll(debris, scope, {});
        const ogun::FixAllStepReport* tiny = stepOf(result, FixAllStageId::TinyShells);
        REQUIRE(tiny->status == FixAllStepStatus::Skipped);
        REQUIRE(tiny->reason == "Needs whole shells: select shells, or clear the selection.");
        REQUIRE(result.mesh.numFaces() == debris.numFaces());
    }
}

TEST_CASE("Fix All splits touching cubes and removes their fin", "[repair][fixall]") {
    const ogun::FixAllResult result = run(SampleMeshes::createTouchingCubesMesh());
    REQUIRE(stepOf(result, FixAllStageId::SplitTouching)->accepted());
    REQUIRE(stepOf(result, FixAllStageId::Fins)->detail == "Removed 1 fin face");
    REQUIRE(result.countsAfter.openEdges == 0);
    REQUIRE(result.countsAfter.nonManifoldEdges == 0);
    REQUIRE(result.countsAfter.nonManifoldVertices == 0);
    REQUIRE(shellCount(result.mesh) == 3);
}

TEST_CASE("Fix All keeps cavities and leaves inside-out bodies with inner shells to the user", "[repair][fixall]") {
    SECTION("Box with cavity, merge off: nothing changes, the cavity is named") {
        const Mesh box = SampleMeshes::createCavityBoxMesh();
        const ogun::FixAllResult result = run(box);
        REQUIRE(result.stopReason == FixAllStopReason::Clean);
        REQUIRE(result.passesRun == 1);
        REQUIRE_FALSE(result.changed);
        REQUIRE(same(result.mesh, box));
        REQUIRE(shellCount(result.mesh) == 2);
        REQUIRE(result.countsAfter.invertedShells == 0);
        const ogun::FixAllStepReport* orient = stepOf(result, FixAllStageId::Orient);
        REQUIRE(orient->status == FixAllStepStatus::Unchanged);
        REQUIRE(orient->detail == "Kept 1 cavity inward: #2");
        REQUIRE(orient->notes.size() == 1);
        REQUIRE(orient->notes[0].shells.at(0).size() == 12);
        REQUIRE(ogun::signedVolume(result.mesh) == Catch::Approx(56.0));
    }
    SECTION("Box with cavity written inside out: stopped, nothing flipped, both named") {
        const Mesh inverted = SampleMeshes::createCavityBoxMesh(true);
        const ogun::FixAllResult result = run(inverted);
        REQUIRE(same(result.mesh, inverted));
        REQUIRE_FALSE(result.changed);
        const ogun::FixAllStepReport* orient = stepOf(result, FixAllStageId::Orient);
        REQUIRE(orient->notes.size() == 1);
        REQUIRE(orient->notes[0].text ==
                "#1 and its inner shell #2 look inside out together: left for you (Flip all or Flip outer only, under Inverted shells)");
        REQUIRE(orient->notes[0].shells.size() == 2);
    }
    SECTION("Nested solid and cavity, merge off: three shells, nothing flipped") {
        const Mesh nested = SampleMeshes::createNestedSolidAndCavityMesh();
        const ogun::FixAllResult result = run(nested);
        REQUIRE(same(result.mesh, nested));
        REQUIRE(shellCount(result.mesh) == 3);
        REQUIRE(result.countsAfter.invertedShells == 0);
        REQUIRE(stepOf(result, FixAllStageId::Orient)->detail == "Kept 1 cavity inward: #2");
    }
}

TEST_CASE("Fix All judges a pass at its end: closed but crossing is kept, the next pass repairs the crossing",
          "[repair][fixall]") {
    // The island step is off in pass 1 only
    ogun::FixAllTestHooks hooks;
    hooks.adjustStages = [](int pass, FixAllPhase phase, std::vector<FixAllStage>& stages) {
        if (pass == 1 && phase == FixAllPhase::Repair) {
            std::erase_if(stages, [](const FixAllStage& stage) { return stage.id == FixAllStageId::Islands; });
        }
    };
    const ogun::FixAllResult result = run(SampleMeshes::createCupWithPostMesh(SampleMeshes::CupPost::ThroughOpening), {}, &hooks);
    REQUIRE(result.passes.size() == 3);
    REQUIRE(result.passes[0].after.closedManifold == 0);
    REQUIRE(result.passes[0].after.intersecting > 0);
    REQUIRE(result.passes[0].newBest); // closed comes first in the score
    REQUIRE(stepOf(result, FixAllStageId::Islands, 2)->accepted());
    REQUIRE(result.passes[1].newBest);
    REQUIRE(result.bestPass == 3);
    REQUIRE(result.stopReason == FixAllStopReason::Clean);
    REQUIRE(result.countsAfter.intersectingFaces == 0);
    REQUIRE(result.countsAfter.thinFaces == 0);
}

TEST_CASE("Fix All's hard guards refuse a step at once", "[repair][fixall]") {
    ogun::FixAllParams onePass;
    onePass.passes = 1;
    SECTION("Losing a cavity") {
        const Mesh box = SampleMeshes::createCavityBoxMesh();
        FixAllStage remove;
        remove.label = "Remove the cavity";
        remove.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
            FixAllStageOutcome outcome;
            outcome.mesh = mesh;
            outcome.mesh.F.conservativeResize(12, 3);
            outcome.mesh.recomputeNormals();
            outcome.faceMap.resize(24, -1);
            for (Eigen::Index f = 0; f < 12; ++f) {
                outcome.faceMap[static_cast<std::size_t>(f)] = f;
            }
            outcome.changed = true;
            outcome.targetBefore = 1;
            return outcome;
        };
        const ogun::FixAllTestHooks hooks = firstInPassOne(remove);
        const ogun::FixAllResult result = run(box, onePass, &hooks);
        REQUIRE(result.steps.front().status == FixAllStepStatus::Refused);
        REQUIRE(result.steps.front().guard == FixAllGuard::Cavity);
        REQUIRE(result.steps.front().reason ==
                "It would remove #2, which is wound inward (a cavity, or a shell rule O cannot judge), so nothing was changed.");
        REQUIRE(same(result.mesh, box));
    }
    SECTION("Turning a cavity outward or shrinking it") {
        const Mesh box = SampleMeshes::createCavityBoxMesh();
        const auto flip = [](Mesh& mesh) {
            for (Eigen::Index f = 12; f < 24; ++f) {
                std::swap(mesh.F(f, 1), mesh.F(f, 2));
            }
        };
        const ogun::FixAllTestHooks flipHooks = firstInPassOne(changingStage(flip, false));
        const ogun::FixAllResult flipped = run(box, onePass, &flipHooks);
        REQUIRE(flipped.steps.front().guard == FixAllGuard::Cavity);
        REQUIRE(flipped.steps.front().reason.rfind("It would turn outward #2", 0) == 0);

        const auto shrink = [](Mesh& mesh) {
            for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
                if (mesh.V(v, 0) == 3.0) {
                    mesh.V(v, 0) = 2.5; // the cavity [1, 3]^3 loses a quarter
                }
            }
        };
        const ogun::FixAllTestHooks shrinkHooks = firstInPassOne(changingStage(shrink, false));
        const ogun::FixAllResult shrunk = run(box, onePass, &shrinkHooks);
        REQUIRE(shrunk.steps.front().guard == FixAllGuard::Cavity);
        REQUIRE(shrunk.steps.front().reason.rfind("It would shrink #2", 0) == 0);
        REQUIRE(same(shrunk.mesh, box));
    }
    SECTION("Changing a face outside the scope") {
        const Mesh two = SampleMeshes::combineMeshes({SampleMeshes::createBox(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)),
                                                      SampleMeshes::createBox(Eigen::Vector3d::Constant(3.0), Eigen::Vector3d::Constant(4.0))});
        std::vector<std::uint8_t> mask(24, 0);
        std::fill(mask.begin(), mask.begin() + 12, std::uint8_t{1});
        const ogun::FixAllScope scope = ogun::FixAllScope::fromFaceMask(two, ogun::FixAllScopeKind::Shells, mask);
        const ogun::FixAllTestHooks hooks = firstInPassOne(changingStage([](Mesh& mesh) { mesh.V(15, 0) += 0.25; }, false));
        const ogun::FixAllResult result = ogun::runFixAll(two, scope, onePass, {}, nullptr, &hooks);
        REQUIRE(result.steps.front().guard == FixAllGuard::Scope);
        REQUIRE(result.steps.front().reason == "It would change faces outside the scope, so nothing was changed.");
        REQUIRE(same(result.mesh, two));
    }
    SECTION("Moving vertices through a shell the merge cannot take; a fill-like step is exempt") {
        const Mesh two = SampleMeshes::combineMeshes({SampleMeshes::createBox(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(2.0)),
                                                      SampleMeshes::createBox(Eigen::Vector3d(3.0, 0.5, 0.5), Eigen::Vector3d(4.0, 1.5, 1.5))});
        // Pull the first box's corner (2, 2, 2) out through the second box
        const auto pull = [](Mesh& mesh) {
            for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
                if (mesh.V.row(v) == Eigen::RowVector3d(2.0, 2.0, 2.0)) {
                    mesh.V.row(v) << 5.0, 1.0, 1.0;
                }
            }
        };
        const ogun::FixAllTestHooks moving = firstInPassOne(changingStage(pull, true));
        const ogun::FixAllResult refused = run(two, onePass, &moving);
        REQUIRE(refused.steps.front().guard == FixAllGuard::DeadEnd);
        REQUIRE(refused.steps.front().reason ==
                "It would move faces of #1 through #2, and the merge cannot take them (the merge is off), so nothing was changed.");
        REQUIRE(same(refused.mesh, two));

        ogun::FixAllParams merge = onePass;
        merge.mergeShells = true;
        const ogun::FixAllResult mergeable = run(two, merge, &moving);
        REQUIRE(mergeable.steps.front().accepted()); // both closed and outward: the merge can unite them

        const ogun::FixAllTestHooks adding = firstInPassOne(changingStage(pull, false));
        const ogun::FixAllResult exempt = run(two, onePass, &adding);
        REQUIRE(exempt.steps.front().accepted());
    }
}

TEST_CASE("Fix All's weld follows scale and placement per axis and keeps surfaces apart", "[repair][fixall]") {
    const auto weldStep = [](const ogun::FixAllResult& result) { return stepOf(result, FixAllStageId::Weld); };
    SECTION("(a) Scaled by 1e4 about the origin: the same faces") {
        const ogun::FixAllResult plain = run(SampleMeshes::createWeldSeamCubeMesh(5.0e-6));
        const ogun::FixAllResult scaled = run(SampleMeshes::createWeldSeamCubeMesh(5.0e-6, Eigen::Vector3d::Zero(), 1.0e4));
        REQUIRE(weldStep(scaled)->accepted());
        REQUIRE(scaled.mesh.F == plain.mesh.F);
        REQUIRE(scaled.countsAfter.openEdges == 0);
    }
    SECTION("(c) A tiny cube: a 5e-6 seam is over the floor and is not welded") {
        const Mesh tiny = SampleMeshes::createSeamBoxMesh(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0e-3), 0, 5.0e-6);
        REQUIRE(weldStep(run(tiny))->status == FixAllStepStatus::Unchanged);
    }
    SECTION("(d) Per axis: an x seam at x = 1000 welds, a y seam near y = 0 does not") {
        const Eigen::Vector3d lo(1000.0, 0.0, 0.0);
        const Eigen::Vector3d hi(1001.0, 0.5, 0.5);
        REQUIRE(weldStep(run(SampleMeshes::createSeamBoxMesh(lo, hi, 0, 5.0e-4)))->accepted());
        REQUIRE(weldStep(run(SampleMeshes::createSeamBoxMesh(lo, hi, 1, 5.0e-4)))->status == FixAllStepStatus::Unchanged);
    }
    SECTION("(e) 0.9 of the tolerance on every axis welds, moving sqrt(3) times as far") {
        Mesh seam = SampleMeshes::createSeamBoxMesh(Eigen::Vector3d::Constant(10.0), Eigen::Vector3d::Constant(20.0), 0, 9.0e-6);
        for (Eigen::Index v = 0; v < seam.numVertices(); ++v) {
            if (seam.V(v, 0) > 15.0) {
                seam.V(v, 1) += 9.0e-6;
                seam.V(v, 2) += 9.0e-6;
            }
        }
        seam.recomputeNormals();
        const ogun::FixAllResult result = run(seam);
        REQUIRE(weldStep(result)->accepted());
        REQUIRE(weldStep(result)->maxMove == Catch::Approx(std::sqrt(3.0) * 9.0e-6).epsilon(1.0e-6));
    }
    SECTION("(f) Two closed cubes within the tolerance are not welded together") {
        const Mesh near = SampleMeshes::combineMeshes(
            {SampleMeshes::createBox(Eigen::Vector3d::Constant(10.0), Eigen::Vector3d::Constant(20.0)),
             SampleMeshes::createBox(Eigen::Vector3d(20.0 + 5.0e-6, 10.0, 10.0), Eigen::Vector3d(30.0, 20.0, 20.0))});
        const ogun::FixAllResult result = run(near);
        REQUIRE(weldStep(result)->status == FixAllStepStatus::Unchanged);
        REQUIRE(shellCount(result.mesh) == 2);
    }
    SECTION("(g) Touching cubes are not pinched") {
        const ogun::FixAllResult result = run(SampleMeshes::createTouchingCubesMesh());
        REQUIRE(weldStep(result)->status == FixAllStepStatus::Unchanged);
    }
}

TEST_CASE("Fix All timings per step, phase and pass", "[repair][fixall][.benchmark]") {
    const std::filesystem::path examples =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples";
    const auto bench = [](const char* name, const Mesh& mesh, bool merge) {
        ogun::FixAllParams params;
        params.mergeShells = merge;
        const auto start = std::chrono::steady_clock::now();
        const ogun::FixAllResult result = run(mesh, params);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        REQUIRE(result.success);
        std::map<std::pair<int, int>, double> perPhase;
        for (const ogun::FixAllStepReport& step : result.steps) {
            perPhase[{step.pass, static_cast<int>(step.phase)}] += step.elapsedMs;
            if (step.elapsedMs >= 1.0) {
                WARN(name << " pass " << step.pass << " phase " << static_cast<int>(step.phase) << " " << step.label << ": "
                          << step.elapsedMs << " ms (guards " << step.guardMs << " ms)");
            }
        }
        for (const auto& [key, phaseMs] : perPhase) {
            WARN(name << " pass " << key.first << " phase " << key.second << ": " << phaseMs << " ms");
        }
        for (const ogun::FixAllPassReport& pass : result.passes) {
            WARN(name << " pass " << pass.pass << ": " << pass.elapsedMs << " ms (incl. the pass-end check)");
        }
        WARN(name << ": " << ms << " ms in all, final report " << result.report.elapsedMs << " ms, passes "
                  << result.passesRun << " (stopped: " << ogun::fixAllStopReasonLabel(result.stopReason) << "), "
                  << result.report.shellsLine << ", " << result.report.leftLine);
    };
    const Mesh bunny = ogun::loadStl(examples / "stanford_bunny.stl");
    const Mesh turkey = ogun::loadStl(examples / "Turkey.stl");
    const Mesh box = ogun::loadStl(examples / "self_intersecting_box.stl");
    bench("bunny", bunny, false);
    bench("Turkey merge off", turkey, false);
    bench("Turkey merge on", turkey, true);
    bench("self_intersecting_box", box, false);
}
