#include "repair/FixAll.hpp"

#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"
#include "mesh/ToleranceConfig.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <stop_token>
#include <string>
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
    hooks.adjustStages = [stage = std::move(stage)](int, FixAllPhase, std::vector<FixAllStage>& stages) {
        stages.assign(1, stage);
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
    REQUIRE(result.steps.size() == 4);
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
        const ogun::FixAllResult result = run(seam, params);
        REQUIRE(result.countsAfter.openEdges == 8);
        REQUIRE_FALSE(result.changed);
        REQUIRE(result.stopReason == FixAllStopReason::RepeatedState);
    }
    SECTION("Unticking the weld skips the step") {
        ogun::FixAllParams params;
        params.weld = false;
        const ogun::FixAllResult result = run(seam, params);
        REQUIRE(result.steps.front().status == FixAllStepStatus::Skipped);
        REQUIRE_FALSE(result.steps.front().reason.empty());
        REQUIRE(result.countsAfter.openEdges == 8);
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
    };
    for (std::size_t i = 0; i < fixtures.size(); ++i) {
        CAPTURE(i);
        const ogun::FixAllResult result = run(fixtures[i]);
        REQUIRE(result.success);
        REQUIRE(result.priorityAfter <= result.priorityBefore);
        REQUIRE(result.scope.matches(result.mesh));
        const ogun::FixAllCounts recount =
            ogun::countFixAllDefects(result.mesh, result.scope, ogun::FixAllParams{}.checkParams);
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
        REQUIRE(result.countsAfter.nonManifoldEdges == 1); // the fin, for a later stage
        const ogun::FixAllStepReport& split = result.steps.at(3);
        REQUIRE(split.stage == ogun::FixAllStageId::SplitTouching);
        REQUIRE(split.accepted());
        REQUIRE(split.detail.rfind("Split 2 touching edges and ", 0) == 0);
        REQUIRE(result.mesh.numFaces() == touching.numFaces());
    }
    SECTION("Doubled faces cube: the collapsed face and both copies go") {
        const Mesh doubled = SampleMeshes::createDoubledFacesCubeMesh();
        const ogun::FixAllResult result = run(doubled);
        REQUIRE(result.countsAfter.duplicateFaces == 0);
        REQUIRE(result.countsAfter.degenerateFaces == 1); // the flat cap: healed by the sliver stage, not deleted
        std::size_t removed = 0;
        for (const ogun::FixAllStepReport& step : result.steps) {
            removed += step.facesRemoved;
        }
        REQUIRE(removed == 3);
        REQUIRE(result.mesh.numFaces() == doubled.numFaces() - 3);
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
    hooks.adjustStages = [&](int, FixAllPhase, std::vector<FixAllStage>& stages) {
        stages.insert(stages.begin(), {throws, badIndices, churn, shortMap});
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
        hooks.adjustStages = [&](int, FixAllPhase, std::vector<FixAllStage>& stages) {
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
