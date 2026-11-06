#include "app/AppController.hpp"
#include "app/FixAllPanelModel.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using ogun::AppController;
using ogun::DefectCategory;
using ogun::FixAllPanelState;
using ogun::FixAllScopeKind;
using ogun::FixAllStageId;
using ogun::HealthStatus;
using ogun::Mesh;
using ogun::SampleMeshes;
using Eigen::Vector3d;

namespace {

Mesh box(const Vector3d& min, const Vector3d& max) {
    return SampleMeshes::createBox(min, max);
}

/// Two overlapping closed cubes (Fix All merges them into one with the merge on).
Mesh overlappingCubes() {
    return SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
}

/// A box with its last face removed: one shell, one 3-edge hole.
Mesh openBox(const Vector3d& min, const Vector3d& max) {
    const Mesh closed = box(min, max);
    std::vector<Eigen::Index> keep(static_cast<std::size_t>(closed.numFaces() - 1));
    for (std::size_t f = 0; f < keep.size(); ++f) {
        keep[f] = static_cast<Eigen::Index>(f);
    }
    return ogun::extractFaces(closed, keep);
}

/// Two open boxes far apart (the second one is the smaller one, #2).
Mesh twoOpenBoxes() {
    return SampleMeshes::combineMeshes({openBox({0, 0, 0}, {4, 4, 4}), openBox({10, 0, 0}, {12, 2, 2})});
}

/// Every face of `before` that is in `faces` is still in `after`, corners and all.
bool facesKept(const Mesh& before, const std::vector<Eigen::Index>& faces, const Mesh& after) {
    const ogun::FaceDiff diff = ogun::faceDiff(before, after);
    return std::none_of(faces.begin(), faces.end(), [&](Eigen::Index f) {
        return std::find(diff.removed.begin(), diff.removed.end(), f) != diff.removed.end();
    });
}

std::size_t openEdges(const Mesh& mesh) {
    return ogun::checkOpenEdges(mesh).infoCount;
}

/// The shell (Shells-panel index) whose box lies at x >= `x`.
std::size_t shellRightOf(const AppController& controller, double x) {
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        if (controller.shells().shell(s).bbox.min.x() >= x) {
            return s;
        }
    }
    FAIL("no shell right of x");
    return 0;
}

bool waitUntil(const std::atomic<bool>& flag) {
    for (int i = 0; i < 20000 && !flag.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return flag.load();
}

/// Test hooks that hold the run inside a stage of `id` until `release` is set (`entered` tells the test).
struct HoldInStage {
    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
};

std::shared_ptr<const ogun::FixAllTestHooks> holdHooks(const std::shared_ptr<HoldInStage>& hold, FixAllStageId id) {
    auto hooks = std::make_shared<ogun::FixAllTestHooks>();
    hooks->adjustStages = [hold, id](int, ogun::FixAllPhase, std::vector<ogun::FixAllStage>& stages) {
        for (ogun::FixAllStage& stage : stages) {
            if (stage.id != id) {
                continue;
            }
            auto inner = stage.run;
            stage.run = [hold, inner](const Mesh& mesh, const ogun::FixAllStageContext& context) {
                hold->entered = true;
                while (!hold->release.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                return inner(mesh, context);
            };
        }
    };
    return hooks;
}

/// Test hooks whose phase 1 is a single step that removes one duplicate face per run.
std::shared_ptr<const ogun::FixAllTestHooks> oneDuplicatePerPass() {
    auto hooks = std::make_shared<ogun::FixAllTestHooks>();
    hooks->adjustStages = [](int, ogun::FixAllPhase phase, std::vector<ogun::FixAllStage>& stages) {
        if (phase != ogun::FixAllPhase::Repair) {
            return;
        }
        ogun::FixAllStage stage;
        stage.label = "Remove one duplicate";
        stage.activity = "removing a duplicate";
        stage.run = [](const Mesh& mesh, const ogun::FixAllStageContext&) {
            ogun::FixAllStageOutcome out;
            const auto duplicates = ogun::checkDuplicateFaces(mesh).size();
            out.targetBefore = duplicates;
            out.targetAfter = duplicates;
            for (Eigen::Index f = mesh.numFaces() - 1; f >= 0 && !out.changed; --f) {
                for (Eigen::Index g = 0; g < f; ++g) {
                    if (ogun::faceGeometryKey(mesh, f) == ogun::faceGeometryKey(mesh, g)) {
                        std::vector<Eigen::Index> keep;
                        out.faceMap.assign(static_cast<std::size_t>(mesh.numFaces()), -1);
                        for (Eigen::Index k = 0; k < mesh.numFaces(); ++k) {
                            if (k != f) {
                                out.faceMap[static_cast<std::size_t>(k)] = static_cast<Eigen::Index>(keep.size());
                                keep.push_back(k);
                            }
                        }
                        out.mesh = mesh;
                        out.mesh.F.resize(static_cast<Eigen::Index>(keep.size()), 3);
                        for (std::size_t r = 0; r < keep.size(); ++r) {
                            out.mesh.F.row(static_cast<Eigen::Index>(r)) = mesh.F.row(keep[r]);
                        }
                        out.changed = true;
                        out.targetAfter = ogun::checkDuplicateFaces(out.mesh).size();
                        out.detail = "Removed 1 duplicate face";
                        break;
                    }
                }
            }
            return out;
        };
        stages = {stage};
    };
    return hooks;
}

} // namespace

TEST_CASE("renumberShellText rewrites shell numbers in order", "[app][fixall]") {
    const std::vector<int> numbers = {3, 1};
    CHECK(ogun::renumberShellText("#1 and its inner shell #2 look inside out together", numbers) ==
          "#3 and its inner shell #1 look inside out together");
    const std::vector<int> keepFirst = {0, 7};
    CHECK(ogun::renumberShellText("Deleted nested solid #8 (wholly inside #1, touching nothing)", keepFirst) ==
          "Deleted nested solid #8 (wholly inside #7, touching nothing)");
    const std::vector<int> one = {5};
    CHECK(ogun::renumberShellText("#6 and #9 reach into cavity #28 of #12", one) == "#6 and #9 reach into cavity #28 of #12");
    CHECK(ogun::renumberShellText("No shells overlap: nothing to merge.", {}) == "No shells overlap: nothing to merge.");
    CHECK(ogun::renumberShellText("Kept cavity #12", one) == "Kept cavity #5");
}

TEST_CASE("Fix All: plan, preview without a change, Apply as one undo step, undo restores everything",
          "[app][fixall][undo][merge]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createTurkeyLikeMesh(), "Turkey-like");
    controller.checkMesh();
    controller.setAutoCheck(DefectCategory::Intersections, false);
    REQUIRE(controller.shells().size() == 9);
    const Mesh before = controller.mesh();
    const ogun::MeshHealthReport healthBefore = controller.healthReport();
    const std::size_t undoBefore = controller.canUndo() ? 1 : 0;
    REQUIRE(undoBefore == 0);

    controller.openFixAll(3);
    REQUIRE(controller.fixAllOpen());
    REQUIRE(controller.activeTool() == ogun::ToolSlot::FixAll);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairMethods().size() == ogun::kFixAllMethods.size());
    CHECK(controller.fixAllState() == FixAllPanelState::Plan);
    CHECK(controller.fixAllParams().passes == 3);
    CHECK(controller.fixAllScope().kind == FixAllScopeKind::Whole);
    CHECK(controller.fixAllScope().text == "whole mesh, 9 shells (nothing selected)");
    // The plan-time check ran (inline without background checks) and ticked the merge box
    CHECK_FALSE(controller.fixAllPlanChecking());
    REQUIRE(controller.fixAllMergePlan() != nullptr);
    CHECK(controller.fixAllMergePlan()->tickedByDefault);
    CHECK(controller.fixAllParams().mergeShells);
    REQUIRE(controller.fixAllWeldPlan() != nullptr);
    CHECK(controller.fixAllWeldPlan()->maxTolerance > 0.0);

    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == FixAllPanelState::Result);
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);

    CHECK(controller.candidateIsCurrent(*candidate));
    CHECK(controller.shownGlobalCandidate() == candidate);
    CHECK(controller.globalPreviewShown());
    CHECK(candidate->metrics.shellsBefore == 9);
    CHECK(candidate->metrics.shellsAfter == 2);
    CHECK(candidate->metrics.intersectingFacesAfter == 0);
    CHECK(candidate->fixAll->result.mesh.empty()); // moved into the candidate
    CHECK(candidate->fixAll->faceShell.size() == static_cast<std::size_t>(candidate->mesh.numFaces()));
    CHECK(candidate->statusMessage.starts_with("Passes: 1 of 3 (stopped: clean)."));
    // Preview never touches the mesh
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
    CHECK_FALSE(controller.canUndo());

    // Shell numbers before Apply name the result's shells; Show maps them to the current mesh by geometry
    const ogun::FixAllFinalReport& report = candidate->fixAll->result.report;
    REQUIRE(report.shells.size() == 2);
    const std::vector<int> separate = {1}; // the separate box, unchanged
    const std::vector<int> mapped = controller.fixAllShellsInCurrentMesh(separate);
    REQUIRE(mapped.size() == 1);
    CHECK(controller.shells().shell(static_cast<std::size_t>(mapped[0])).bbox.min.x() >= 14.0);
    const std::vector<int> body = {0};
    const std::vector<int> parts = controller.fixAllShellsInCurrentMesh(body); // faces the union kept as they were
    CHECK_FALSE(parts.empty());
    CHECK(std::find(parts.begin(), parts.end(), mapped[0]) == parts.end());

    REQUIRE(controller.applyFixAll());
    CHECK(controller.lastUndoActionLabel() == "Fix All");
    CHECK(controller.fixAllState() == FixAllPanelState::Report);
    CHECK(controller.fixAllUndoAvailable());
    REQUIRE(controller.shells().size() == 2);
    CHECK(controller.rowCount(DefectCategory::Intersections) == 0);
    // The run's final check is installed: no row is stale, running or unchecked (Intersections included)
    for (const DefectCategory category : ogun::kAllDefectCategories) {
        INFO(ogun::categoryInfo(category).name);
        const HealthStatus status = controller.healthReport()[category].status;
        CHECK((status == HealthStatus::Ok || status == HealthStatus::Defects));
    }
    // The merged shell is selected, and the report's shell numbers are the Shells panel's
    CHECK(controller.shells().selectedCount() == 1);
    const ogun::FixAllCandidate* shown = controller.fixAllShown();
    REQUIRE(shown != nullptr);
    for (std::size_t k = 0; k < shown->result.report.shells.size(); ++k) {
        CHECK(controller.shells().shellOfFace(shown->result.report.shells[k].front()) == static_cast<int>(k));
    }
    for (const ogun::FixAllReportLine& line : shown->result.report.lines) {
        if (line.text.find('#') == std::string::npos) {
            continue;
        }
        for (const int s : line.shells) {
            CHECK(line.text.find("#" + std::to_string(AppController::fixAllShellNumber(s))) != std::string::npos);
        }
    }
    // Step notes are renumbered to the result: the merge note names the union, now #1
    for (const ogun::FixAllStepReport& step : shown->result.steps) {
        if (step.stage == FixAllStageId::MergeShells && step.accepted()) {
            REQUIRE_FALSE(step.notes.empty());
            CHECK(controller.fixAllNoteShellNumbers(step, step.notes.front()) == std::vector<int>{1});
            CHECK(controller.fixAllNoteText(step, step.notes.front()) == "Merged 7 shells into #1");
        }
    }

    // One undo step restores the mesh, the health report and the shell list exactly
    REQUIRE(controller.undoFixAll());
    CHECK_FALSE(controller.canUndo());
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
    CHECK(controller.shells().size() == 9);
    CHECK(controller.shells().selectedCount() == 0);
    for (const DefectCategory category : ogun::kAllDefectCategories) {
        INFO(ogun::categoryInfo(category).name);
        CHECK(controller.healthReport()[category].status == healthBefore[category].status);
        CHECK(controller.healthReport()[category].displayCount(category) == healthBefore[category].displayCount(category));
    }
    CHECK_FALSE(controller.fixAllApplied().has_value());
    CHECK(controller.fixAllState() == FixAllPanelState::Plan);
    CHECK(controller.fixAllOpen());
}

TEST_CASE("Fix All: a one-pass run is one undo step, three of them three", "[app][fixall][undo]") {
    AppController controller;
    // A cube with three extra copies of faces; the hooked pass removes one copy per run
    Mesh mesh = box({0, 0, 0}, {2, 2, 2});
    const Eigen::MatrixXi cube = mesh.F;
    mesh.F.resize(cube.rows() + 3, 3);
    mesh.F << cube, cube.topRows(3);
    controller.setMesh(mesh, "cube with copies");
    const Mesh before = controller.mesh();
    controller.setFixAllTestHooks(oneDuplicatePerPass());

    controller.openFixAll(1);
    CHECK(controller.fixAllParams().passes == 1);
    for (int i = 0; i < 3; ++i) {
        INFO("pass " << i + 1);
        controller.runFixAllOneMorePass();
        const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
        REQUIRE(candidate != nullptr);
        REQUIRE(candidate->success);
        CHECK(candidate->fixAll->onePassRequested);
        CHECK(candidate->fixAll->result.onePass);
        REQUIRE(controller.applyFixAll());
        CHECK(controller.lastUndoActionLabel() == "Fix All (one pass)");
        CHECK(controller.mesh().numFaces() == before.numFaces() - 1 - i);
    }
    CHECK(ogun::checkDuplicateFaces(controller.mesh()).empty());
    for (int i = 0; i < 3; ++i) {
        CHECK(controller.lastUndoActionLabel() == "Fix All (one pass)");
        REQUIRE(controller.undoLastModification());
    }
    CHECK_FALSE(controller.canUndo());
    CHECK(controller.mesh().F == before.F);

    SECTION("A one-pass preview stays current whatever the Passes field says") {
        controller.setFixAllPasses(3);
        controller.previewFixAll(true);
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(controller.candidateIsCurrent(*controller.fixAllCandidate()));
        controller.setFixAllPasses(5);
        CHECK(controller.candidateIsCurrent(*controller.fixAllCandidate()));
        ogun::FixAllParams params = controller.fixAllParams();
        params.removeFins = false;
        controller.setFixAllParams(params);
        CHECK_FALSE(controller.candidateIsCurrent(*controller.fixAllCandidate()));
    }
    SECTION("A run that changes nothing adds no undo step") {
        controller.setFixAllTestHooks(nullptr);
        controller.setMesh(box({0, 0, 0}, {1, 1, 1}), "cube");
        controller.openFixAll(3);
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        REQUIRE_FALSE(controller.fixAllCandidate()->fixAll->result.changed);
        REQUIRE(controller.applyFixAll());
        CHECK_FALSE(controller.canUndo());
        CHECK(controller.fixAllState() == FixAllPanelState::Report);
        CHECK_FALSE(controller.fixAllUndoAvailable());
        CHECK(controller.lastRepairStatusMessage().starts_with("Fix All changed nothing"));
    }
}

TEST_CASE("Fix All: an edit discards the candidate; undo Fix All only while it is on top", "[app][fixall][undo]") {
    AppController controller;
    controller.setMesh(twoOpenBoxes(), "two open boxes");
    controller.openFixAll();
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);

    const std::vector<int> second = {1};
    REQUIRE(controller.flipShells(second)); // any edit
    CHECK(controller.fixAllCandidate() == nullptr);
    CHECK(controller.fixAllState() == FixAllPanelState::Plan);
    REQUIRE(controller.undoLastModification());

    controller.previewFixAll();
    REQUIRE(controller.applyFixAll());
    CHECK(openEdges(controller.mesh()) == 0);
    CHECK(controller.fixAllUndoAvailable());
    REQUIRE(controller.flipShells(second));
    CHECK_FALSE(controller.fixAllUndoAvailable());
    CHECK_FALSE(controller.undoFixAll());
    REQUIRE(controller.undoLastModification()); // the flip
    CHECK(controller.fixAllUndoAvailable());
    REQUIRE(controller.undoFixAll());
    CHECK(openEdges(controller.mesh()) == 6);

    SECTION("A selection change drops the candidate too") {
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        controller.clickShell(0, false, false, {});
        CHECK(controller.fixAllCandidate() == nullptr);
    }
    SECTION("Closing the panel forgets the plan and the report") {
        controller.closeTool();
        CHECK_FALSE(controller.fixAllOpen());
        CHECK(controller.fixAllMergePlan() == nullptr);
        CHECK_FALSE(controller.fixAllApplied().has_value());
    }
}

TEST_CASE("Fix All follows the selection: faces, shells, nothing; hidden shells never change", "[app][fixall][scope]") {
    AppController controller;
    controller.setMesh(twoOpenBoxes(), "two open boxes");
    REQUIRE(controller.shells().size() == 2);
    const std::size_t left = 0;
    const std::size_t right = shellRightOf(controller, 10.0);
    REQUIRE(right == 1);
    const Mesh before = controller.mesh();
    const std::vector<Eigen::Index> leftFaces = controller.shells().shell(left).faces;
    const std::vector<Eigen::Index> rightFaces = controller.shells().shell(right).faces;

    SECTION("Nothing selected: the whole mesh") {
        controller.openFixAll();
        CHECK(controller.fixAllScope().kind == FixAllScopeKind::Whole);
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(openEdges(controller.fixAllCandidate()->mesh) == 0);
    }
    SECTION("Selected faces: only they change, and Apply selects the scope faces again") {
        controller.selectFaces(leftFaces, true);
        controller.openFixAll();
        const ogun::FixAllScopeInfo scope = controller.fixAllScope();
        CHECK(scope.kind == FixAllScopeKind::Faces);
        CHECK(scope.faces == leftFaces.size());
        CHECK(scope.text == "11 selected faces (in 1 shell)");
        REQUIRE(controller.fixAllMergePlan() != nullptr);
        CHECK(controller.fixAllMergePlan()->needsWholeShells);
        CHECK_FALSE(controller.fixAllParams().mergeShells);
        controller.previewFixAll();
        const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
        REQUIRE(candidate != nullptr);
        REQUIRE(candidate->success);
        CHECK(candidate->fixAll->scopeKind == FixAllScopeKind::Faces);
        CHECK(openEdges(candidate->mesh) == 3); // the right box's hole is left
        CHECK(facesKept(before, rightFaces, candidate->mesh));
        const std::size_t carried = candidate->fixAll->result.scope.faceCount();
        REQUIRE(carried > leftFaces.size());

        REQUIRE(controller.applyFixAll());
        CHECK(controller.selectedFaceCount() == carried);
        CHECK(controller.fixAllScope().kind == FixAllScopeKind::Faces);
        CHECK(controller.fixAllScope().faces == carried);
        // Run one more pass works on the same faces
        controller.runFixAllOneMorePass();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(controller.fixAllCandidate()->fixAll->scopeKind == FixAllScopeKind::Faces);
    }
    SECTION("Selected shells: the others are left; Apply selects the shells in scope") {
        controller.clickShell(right, false, false, {});
        controller.openFixAll();
        CHECK(controller.fixAllScope().kind == FixAllScopeKind::Shells);
        CHECK(controller.fixAllScope().text == "1 of 2 shells (selected)");
        controller.previewFixAll();
        const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
        REQUIRE(candidate != nullptr);
        CHECK(openEdges(candidate->mesh) == 3);
        CHECK(facesKept(before, leftFaces, candidate->mesh));
        REQUIRE(controller.applyFixAll());
        REQUIRE(controller.shells().selectedCount() == 1);
        CHECK(controller.shells().shell(controller.shells().selectedShells().front()).bbox.min.x() >= 10.0);
    }
    SECTION("A hidden shell is never changed, even when selected") {
        controller.setShellVisible(right, false);
        controller.openFixAll();
        ogun::FixAllScopeInfo scope = controller.fixAllScope();
        CHECK(scope.kind == FixAllScopeKind::Shells);
        CHECK(scope.hiddenShells == 1);
        CHECK(scope.text == "1 of 2 shells (the visible ones); 1 hidden shell is left as it is");
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(facesKept(before, rightFaces, controller.fixAllCandidate()->mesh));
        CHECK(openEdges(controller.fixAllCandidate()->mesh) == 3);

        // Selected, then hidden: still left out
        controller.closeTool();
        controller.showAllShells();
        controller.clickShell(left, false, false, {});
        controller.clickShell(right, true, false, {});
        REQUIRE(controller.shells().selectedCount() == 2);
        controller.setShellVisible(right, false);
        controller.openFixAll();
        scope = controller.fixAllScope();
        CHECK(scope.faces == leftFaces.size());
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(facesKept(before, rightFaces, controller.fixAllCandidate()->mesh));
    }
}

TEST_CASE("Fix All: the plan-time check sets the merge box and the user's click wins", "[app][fixall][merge]") {
    AppController controller;
    SECTION("Overlapping shells tick it; a click unticks it for good") {
        controller.setMesh(overlappingCubes(), "overlapping cubes");
        controller.openFixAll();
        CHECK(controller.fixAllParams().mergeShells);
        CHECK_FALSE(controller.fixAllMergeUserSet());
        controller.setFixAllMergeShells(false);
        CHECK(controller.fixAllMergeUserSet());
        // A new scope re-checks the plan but keeps the click
        controller.clickShell(0, false, false, {});
        controller.clickShell(1, true, false, {});
        controller.waitForFixAllPlan();
        REQUIRE(controller.fixAllMergePlan() != nullptr);
        CHECK(controller.fixAllMergePlan()->tickedByDefault);
        CHECK_FALSE(controller.fixAllParams().mergeShells);
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(controller.fixAllCandidate()->metrics.shellsAfter == 2); // not merged
        // Opening the panel again forgets the click
        controller.openFixAll();
        CHECK(controller.fixAllParams().mergeShells);
        CHECK_FALSE(controller.fixAllMergeUserSet());
    }
    SECTION("Shells that do not overlap leave it unticked") {
        controller.setMesh(twoOpenBoxes(), "two open boxes");
        controller.openFixAll();
        REQUIRE(controller.fixAllMergePlan() != nullptr);
        CHECK_FALSE(controller.fixAllParams().mergeShells);
    }
    SECTION("In the background: checking..., then ticked; a preview waits for it") {
        controller.setMesh(overlappingCubes(), "overlapping cubes");
        controller.setBackgroundChecks(true);
        controller.openFixAll();
        CHECK(controller.fixAllPlanChecking());
        CHECK(controller.fixAllMergePlan() == nullptr);
        controller.previewFixAll();
        CHECK(controller.fixAllRunning());
        CHECK(controller.fixAllStatusText() == "Fix All: checking which shells overlap...");
        controller.waitForFixAll();
        controller.setBackgroundChecks(false);
        CHECK_FALSE(controller.fixAllPlanChecking());
        CHECK(controller.fixAllParams().mergeShells);
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(controller.fixAllCandidate()->metrics.shellsAfter == 1);
    }
}

TEST_CASE("Fix All in the background: progress, cancel, cancel during a merge, stale results dropped",
          "[app][fixall][background]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "overlapping cubes");
    const Mesh before = controller.mesh();
    controller.setBackgroundChecks(true);

    SECTION("Cancel during phase 1: the run stops, nothing changes") {
        auto hold = std::make_shared<HoldInStage>();
        controller.setFixAllTestHooks(holdHooks(hold, FixAllStageId::Dedupe));
        controller.openFixAll();
        controller.setFixAllMergeShells(true);
        controller.previewFixAll();
        REQUIRE(waitUntil(hold->entered));
        CHECK(controller.fixAllRunning());
        CHECK(controller.fixAllState() == FixAllPanelState::Running);
        CHECK(controller.globalRepairRunning());
        CHECK(controller.fixAllStatusText().starts_with("Fix All: pass 1 of 3, phase 1, removing duplicate faces..."));
        CHECK(controller.fixAllStatusText().ends_with(" %"));
        CHECK(controller.fixAllProgress() > 0.0f);

        controller.cancelFixAll();
        CHECK(controller.fixAllCancelling());
        CHECK(controller.statusMessage() == "Cancelling Fix All...");
        CHECK(controller.fixAllStatusText() == "Cancelling Fix All...");
        CHECK(controller.fixAllRunning());
        hold->release = true;
        controller.waitForFixAll();
        CHECK_FALSE(controller.fixAllRunning());
        CHECK(controller.fixAllCandidate() == nullptr);
        CHECK(controller.fixAllState() == FixAllPanelState::Plan);
        CHECK(controller.statusMessage() == "Fix All cancelled: nothing was changed.");
        CHECK(controller.mesh().F == before.F);
        CHECK_FALSE(controller.canUndo());
    }
    SECTION("Cancel during a merge takes effect when the union returns; its result is dropped") {
        auto hold = std::make_shared<HoldInStage>();
        controller.setFixAllTestHooks(holdHooks(hold, FixAllStageId::MergeShells));
        controller.openFixAll();
        controller.setFixAllMergeShells(true);
        controller.previewFixAll();
        REQUIRE(waitUntil(hold->entered));
        CHECK(controller.fixAllStatusText().starts_with("Fix All: pass 1 of 3, phase 2, merging 2 shells (Manifold)..."));
        controller.cancelFixAll();
        CHECK(controller.statusMessage() == "Cancelling after the current merge...");
        CHECK(controller.fixAllStatusText() == "Cancelling after the current merge...");
        hold->release = true; // the union runs to its end, then the run stops
        controller.waitForFixAll();
        CHECK(controller.fixAllCandidate() == nullptr);
        CHECK(controller.mesh().F == before.F);
        CHECK(controller.shells().size() == 2);
    }
    SECTION("An edit while the run computes drops its result") {
        auto hold = std::make_shared<HoldInStage>();
        controller.setFixAllTestHooks(holdHooks(hold, FixAllStageId::Dedupe));
        controller.openFixAll();
        controller.setFixAllMergeShells(true);
        controller.previewFixAll();
        REQUIRE(waitUntil(hold->entered));
        const std::vector<int> first = {0};
        REQUIRE(controller.flipShells(first)); // does not wait for the run
        CHECK_FALSE(controller.fixAllRunning());
        hold->release = true;
        controller.waitForFixAll();
        CHECK(controller.fixAllCandidate() == nullptr);
    }
    SECTION("A run that finished for an older mesh is dropped") {
        controller.setFixAllTestHooks(nullptr);
        controller.openFixAll();
        controller.setFixAllMergeShells(true);
        controller.previewFixAll();
        while (controller.repairWorkerBusy()) { // finished, not yet collected
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const std::vector<int> first = {0};
        REQUIRE(controller.flipShells(first));
        controller.waitForFixAll();
        CHECK(controller.fixAllCandidate() == nullptr);
        CHECK(controller.shells().size() == 2);
    }
    SECTION("A finished run is collected by the frame's poll") {
        controller.setFixAllTestHooks(nullptr);
        controller.openFixAll();
        controller.setFixAllMergeShells(true);
        controller.previewFixAll();
        controller.waitForFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        CHECK(controller.fixAllState() == FixAllPanelState::Result);
        CHECK(controller.fixAllStatusText().empty());
        CHECK(controller.statusMessage() == "Fix All preview ready.");
        REQUIRE(controller.applyFixAll());
        CHECK(controller.shells().size() == 1);
        controller.waitForChecks();
    }
    controller.setBackgroundChecks(false);
}

TEST_CASE("Fix All on a whole mesh selects the merged shells; one more pass keeps the whole mesh", "[app][fixall][merge]") {
    AppController controller;
    Mesh mesh = SampleMeshes::combineMeshes({overlappingCubes(), box({10, 0, 0}, {11, 1, 1})});
    controller.setMesh(mesh, "two overlapping cubes and a box");
    REQUIRE(controller.shells().size() == 3);
    controller.openFixAll();
    REQUIRE(controller.fixAllParams().mergeShells);
    controller.previewFixAll();
    REQUIRE(controller.applyFixAll());
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(controller.shells().selectedCount() == 1);
    CHECK(controller.shells().shell(controller.shells().selectedShells().front()).bbox.max.x() < 5.0);
    CHECK(controller.fixAllScope().kind == FixAllScopeKind::Whole);
    CHECK(controller.fixAllScope().text == "whole mesh, 2 shells (as in the last run)");

    controller.runFixAllOneMorePass();
    REQUIRE(controller.fixAllCandidate() != nullptr);
    CHECK(controller.fixAllCandidate()->fixAll->scopeKind == FixAllScopeKind::Whole);
    // Once the user changes the selection, the selection is the scope again
    controller.clickShell(1, false, false, {});
    CHECK(controller.fixAllScope().kind == FixAllScopeKind::Shells);
}

TEST_CASE("Fix All on the cavity box inside out: Flip all from the report is one undo step", "[app][fixall][orient]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createCavityBoxMesh(true), "inside-out cavity box");
    controller.openFixAll();
    controller.previewFixAll();
    const ogun::FixAllCandidate* shown = controller.fixAllShown();
    REQUIRE(shown != nullptr);
    REQUIRE(shown->result.report.groupsNotMerged.size() == 1);
    const std::vector<int>& named = shown->result.report.groupsNotMerged.front().shells;
    REQUIRE(named.size() == 2);
    CHECK(controller.fixAllShellsInCurrentMesh(named).size() == 2);
    REQUIRE(controller.fixAllFlipShells(named));
    CHECK(controller.lastUndoActionLabel() == "Flip Shells #1 and #2");
    CHECK(controller.fixAllCandidate() == nullptr); // the edit dropped the preview
    CHECK(ogun::signedVolume(controller.mesh()) == Catch::Approx(64.0 - 8.0));
}

TEST_CASE("Fix All on the Turkey through the app: plan, preview, Apply and undo (timings)", "[app][fixall][.slow]") {
    using Clock = std::chrono::steady_clock;
    const auto seconds = [](Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); };
    AppController controller;
    controller.setMesh(ogun::loadStl(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
                                     "examples" / "Turkey.stl"),
                       "Turkey.stl");
    REQUIRE(controller.shells().size() == 28);
    const Mesh before = controller.mesh();

    auto start = Clock::now();
    controller.openFixAll();
    const double planSeconds = seconds(start);
    REQUIRE(controller.fixAllMergePlan() != nullptr);
    CHECK(controller.fixAllParams().mergeShells); // the feathers cross the body
    CHECK(controller.fixAllMergePlan()->groups.empty());
    REQUIRE(controller.fixAllMergePlan()->refusedGroups.size() == 1);

    start = Clock::now();
    controller.previewFixAll();
    const double previewSeconds = seconds(start);
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    CHECK(candidate->metrics.shellsAfter == 28);
    const double runSeconds = candidate->fixAll->result.elapsedMs / 1000.0;

    start = Clock::now();
    REQUIRE(controller.applyFixAll());
    const double applySeconds = seconds(start);
    CHECK(controller.lastUndoActionLabel() == "Fix All");
    for (const DefectCategory category : ogun::kAllDefectCategories) {
        const HealthStatus status = controller.healthReport()[category].status;
        CHECK((status == HealthStatus::Ok || status == HealthStatus::Defects));
    }
    CHECK(controller.rowCount(DefectCategory::OpenEdges) == 0);

    start = Clock::now();
    REQUIRE(controller.undoFixAll());
    const double undoSeconds = seconds(start);
    CHECK(controller.mesh().F == before.F);
    WARN("Turkey Fix All in the app: plan " << planSeconds << " s, preview " << previewSeconds << " s (run " << runSeconds
                                            << " s), apply " << applySeconds << " s, undo " << undoSeconds << " s");
}

namespace {

/// Fix All on three overlapping boxes, one of them open, whose hole Fix All may not fill (its fill steps
/// are hooked off): the open box is left for *Rebuild shell #2...*. Coarse rebuild settings.
void openFixAllOnOpenBox(AppController& controller) {
    controller.setMesh(SampleMeshes::createOpenShellAmongCubesMesh(), "Overlapping boxes, one open");
    auto hooks = std::make_shared<ogun::FixAllTestHooks>();
    hooks->adjustStages = [](int, ogun::FixAllPhase, std::vector<ogun::FixAllStage>& stages) {
        std::erase_if(stages, [](const ogun::FixAllStage& s) {
            return s.id == FixAllStageId::FillHoles || s.id == FixAllStageId::CloseCracks;
        });
    };
    controller.setFixAllTestHooks(hooks);
    controller.openFixAll(3);
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = ogun::settings::kMinAlphaWrapDetail;
    params.windingResolution = ogun::settings::kMinWindingGridResolution;
    controller.setGlobalRepairParams(params);
}

} // namespace

TEST_CASE("Fix All + Rebuild: rows only when asked, Apply is one undo step, undo restores everything",
          "[app][fixall][rebuild][undo]") {
    AppController controller;
    openFixAllOnOpenBox(controller);
    const Mesh before = controller.mesh();
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.fixAllParams().mergeShells); // the boxes overlap: ticked by the plan

    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == FixAllPanelState::Result);
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    const ogun::FixAllFinalReport& report = candidate->fixAll->result.report;
    REQUIRE(report.shells.size() == 2);
    const auto open = std::find_if(report.residue.begin(), report.residue.end(),
                                   [](const ogun::FixAllShellResidue& r) { return r.suggestion == ogun::FixAllSuggestion::AlphaWrap; });
    REQUIRE(open != report.residue.end());
    CHECK(open->shell == 1);
    // Nothing is rebuilt unless asked
    CHECK(controller.fixAllRebuilds().empty());
    CHECK_FALSE(controller.fixAllRebuildShell().has_value());

    controller.rebuildFixAllShell(1);
    CHECK(controller.fixAllRebuildShell() == 1);
    const std::vector<ogun::RepairCandidate>& rows = controller.fixAllRebuilds();
    REQUIRE(rows.size() == ogun::kShellRebuildMethods.size());
    const ogun::RepairCandidate& wrap = rows.front();
    REQUIRE(wrap.method == ogun::GlobalRepairMethod::AlphaWrap);
    REQUIRE(wrap.success);
    CHECK(wrap.fixAll->rebuild->suggested);
    CHECK(wrap.fixAll->rebuild->note == "wrapped, then merged");
    CHECK(AppController::fixAllRebuildUndoLabel(wrap) == "Fix All + Rebuild #2");
    CHECK(controller.fixAllCandidate() == candidate); // the Fix All row stays
    // Showing a rebuild row draws it; the mesh is untouched until Apply
    controller.showFixAllRebuild(ogun::GlobalRepairMethod::AlphaWrap);
    CHECK(controller.shownGlobalCandidate() == &wrap);
    controller.showGlobalCandidate(ogun::GlobalRepairMethod::FixAll);
    CHECK(controller.shownGlobalCandidate() == candidate);
    controller.showFixAllRebuild(ogun::GlobalRepairMethod::AlphaWrap);
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
    CHECK_FALSE(controller.canUndo());

    REQUIRE(controller.applyGlobalRepair()); // the shown row: the rebuild
    CHECK(controller.lastUndoActionLabel() == "Fix All + Rebuild #2");
    CHECK(controller.fixAllState() == FixAllPanelState::Report);
    CHECK(controller.fixAllUndoAvailable());
    CHECK(controller.shells().size() == 1);
    CHECK(openEdges(controller.mesh()) == 0);
    CHECK(controller.fixAllRebuilds().empty());
    REQUIRE(controller.fixAllShown() != nullptr);
    CHECK(controller.fixAllShown()->rebuild.has_value());
    CHECK(controller.statusMessage().starts_with("Applied Fix All + Rebuild #2: Passes: "));
    for (const DefectCategory category : ogun::kAllDefectCategories) {
        INFO(ogun::categoryInfo(category).name);
        const HealthStatus status = controller.healthReport()[category].status;
        CHECK((status == HealthStatus::Ok || status == HealthStatus::Defects));
    }

    // One undo step restores the mesh exactly
    REQUIRE(controller.undoFixAll());
    CHECK_FALSE(controller.canUndo());
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
    CHECK(controller.shells().size() == 3);
}

TEST_CASE("Fix All + Rebuild: going back to the plan or a new run drops the rebuild rows", "[app][fixall][rebuild]") {
    AppController controller;
    openFixAllOnOpenBox(controller);
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);
    // A cavity or a shell that is not there gives a row that says why
    controller.rebuildFixAllShell(5);
    REQUIRE(controller.fixAllRebuilds().size() == ogun::kShellRebuildMethods.size());
    CHECK_FALSE(controller.fixAllRebuilds().front().success);
    CHECK(controller.fixAllRebuilds().front().statusMessage == "There is no shell #6 in Fix All's result.");
    CHECK_FALSE(controller.applyFixAllRebuild(ogun::GlobalRepairMethod::AlphaWrap));
    CHECK_FALSE(controller.canUndo());

    controller.rebuildFixAllShell(1);
    REQUIRE(controller.fixAllRebuilds().front().success);
    controller.previewFixAll(); // a new run: its rows are gone
    CHECK(controller.fixAllRebuilds().empty());
    controller.rebuildFixAllShell(1);
    REQUIRE_FALSE(controller.fixAllRebuilds().empty());
    controller.backToFixAllPlan();
    CHECK(controller.fixAllRebuilds().empty());
    CHECK_FALSE(controller.fixAllRebuildShell().has_value());
    // No Fix All candidate: nothing to rebuild
    controller.rebuildFixAllShell(1);
    CHECK(controller.fixAllRebuilds().empty());
}

TEST_CASE("Fix All + Rebuild in the background: Cancel stops the alpha wrap mid-run", "[app][fixall][rebuild][background]") {
    AppController controller;
    openFixAllOnOpenBox(controller);
    controller.previewFixAll(); // inline
    REQUIRE(controller.fixAllCandidate() != nullptr);
    const Mesh before = controller.mesh();
    controller.setBackgroundChecks(true);
    // A wrap fine enough to take minutes in Debug; the stop is seen at the wrap's next step once CGAL's
    // set-up (splitting the input to about 3 alpha, about 2 s here in Debug) is done
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = 300.0;
    controller.setGlobalRepairParams(params);

    controller.rebuildFixAllShell(1);
    CHECK(controller.fixAllRebuildRunning());
    CHECK(controller.globalRepairRunning());
    for (int i = 0; i < 20000 && controller.globalRepairProgress() <= 0.0f; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // inside the wrap
    CHECK(controller.globalRepairCurrentMethod() == ogun::GlobalRepairMethod::AlphaWrap);
    CHECK(controller.fixAllRebuildStatusText().starts_with("Rebuilding #2: shrink-Wrap (Outer Envelope) (1 of 3)... "));

    const auto start = std::chrono::steady_clock::now();
    controller.cancelGlobalRepair(); // the status bar's Cancel
    CHECK(controller.statusMessage() == "Rebuild cancelled: nothing was changed.");
    controller.waitForFixAll();
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(10));
    CHECK_FALSE(controller.fixAllRebuildRunning());
    CHECK(controller.fixAllRebuilds().empty());
    CHECK(controller.fixAllState() == FixAllPanelState::Result); // the Fix All preview stays
    CHECK(controller.mesh().F == before.F);
    CHECK_FALSE(controller.canUndo());
}

TEST_CASE("Fix All: undo Fix All survives a full undo history", "[app][fixall][undo]") {
    AppController controller;
    controller.setMesh(twoOpenBoxes(), "two open boxes");
    controller.openFixAll();
    // Fill the undo history to its cap (an even number of flips leaves the mesh as it was)
    const std::vector<int> second = {1};
    for (std::size_t i = 0; i < ogun::settings::kMaxUndoHistoryDepth; ++i) {
        REQUIRE(controller.flipShells(second));
    }
    controller.previewFixAll();
    REQUIRE(controller.applyFixAll());
    CHECK(controller.fixAllUndoAvailable());
    REQUIRE(controller.flipShells(second)); // a later edit: the oldest entry is dropped again
    CHECK_FALSE(controller.fixAllUndoAvailable());
    REQUIRE(controller.undoLastModification()); // the flip: Fix All is on top again
    CHECK(controller.fixAllApplied().has_value());
    CHECK(controller.fixAllState() == FixAllPanelState::Report);
    CHECK(controller.fixAllUndoAvailable());
    REQUIRE(controller.undoFixAll());
    CHECK(openEdges(controller.mesh()) == 6);
    CHECK_FALSE(controller.fixAllApplied().has_value());
}

TEST_CASE("Fix All + Rebuild: the report's step notes keep the numbering of the run they describe",
          "[app][fixall][rebuild]") {
    AppController controller;
    openFixAllOnOpenBox(controller);
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);
    controller.rebuildFixAllShell(1);
    REQUIRE_FALSE(controller.fixAllRebuilds().empty());
    REQUIRE(controller.fixAllRebuilds().front().success);
    controller.showFixAllRebuild(ogun::GlobalRepairMethod::AlphaWrap);
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.fixAllState() == FixAllPanelState::Report);

    // The Report draws the steps of the Fix All run the rebuild builds on: their notes name shells of
    // that run's result, so they are numbered with it, not with the rebuilt mesh
    const ogun::FixAllCandidate* run = controller.fixAllShown();
    REQUIRE(run != nullptr);
    REQUIRE(run->rebuild.has_value());
    const ogun::FixAllCandidate& base = *run->rebuild->base;
    std::size_t checked = 0;
    for (const ogun::FixAllStepReport& step : base.result.steps) {
        if (!step.notesInResult) {
            continue;
        }
        for (const ogun::FixAllShellNote& note : step.notes) {
            if (note.faceSet) {
                continue;
            }
            std::vector<int> expected;
            for (const std::vector<Eigen::Index>& faces : note.shells) {
                int number = 0;
                for (const Eigen::Index f : faces) {
                    if (f >= 0 && static_cast<std::size_t>(f) < base.faceShell.size() && base.faceShell[static_cast<std::size_t>(f)] >= 0) {
                        number = base.faceShell[static_cast<std::size_t>(f)] + 1;
                        break;
                    }
                }
                expected.push_back(number);
            }
            INFO(note.text);
            CHECK(controller.fixAllNoteShellNumbers(step, note) == expected);
            ++checked;
        }
    }
    CHECK(checked > 0);
}

TEST_CASE("Fix All: the weld row follows the weld tolerance set in the panel", "[app][fixall]") {
    Mesh needle;
    needle.V.resize(4, 3);
    needle.V << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 5.0e-5, 0.0;
    needle.F.resize(2, 3);
    needle.F << 0, 1, 2, 1, 3, 2;
    needle.recomputeNormals();
    AppController controller;
    controller.setMesh(needle, "needle");
    controller.openFixAll();
    REQUIRE(controller.fixAllWeldPlan() != nullptr);
    CHECK(controller.fixAllWeldPlan()->collapsingEdges == 0);

    ogun::FixAllParams params = controller.fixAllParams();
    params.weldTolerance.absFloor = 1.0e-4; // now the 5e-5 edge is within the weld's reach
    controller.setFixAllParams(params);
    controller.waitForFixAllPlan();
    REQUIRE(controller.fixAllWeldPlan() != nullptr);
    CHECK(controller.fixAllWeldPlan()->collapsingEdges == 1);
    CHECK(controller.fixAllWeldPlan()->maxTolerance == Catch::Approx(1.0e-4));
}

TEST_CASE("Fix All on the Turkey as the GUI drives it: background plan check, preview, per-frame polling (timings)",
          "[app][fixall][.benchmark]") {
    using Clock = std::chrono::steady_clock;
    const auto seconds = [](Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); };
    AppController controller;
    controller.setMesh(ogun::loadStl(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
                                     "examples" / "Turkey.stl"),
                       "Turkey.stl");
    controller.setBackgroundChecks(true); // as PolyscopeGui::run does

    // One GUI frame: poll the jobs, then what the panel reads every frame (its plan rows, scope and status)
    double pollSeconds = 0.0;
    double maxFrameSeconds = 0.0;
    int frames = 0;
    const auto frame = [&] {
        const auto start = Clock::now();
        static_cast<void>(controller.pollBackgroundChecks());
        if (controller.fixAllState() == FixAllPanelState::Plan) {
            static_cast<void>(ogun::fixAllPlanRows(controller));
        }
        static_cast<void>(controller.fixAllScope());
        static_cast<void>(controller.fixAllStatusText());
        const double s = seconds(start);
        pollSeconds += s;
        maxFrameSeconds = std::max(maxFrameSeconds, s);
        ++frames;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    };

    while (controller.backgroundWorkRunning()) {
        frame(); // the load checks, if any
    }

    auto start = Clock::now();
    controller.openFixAll();
    while (controller.fixAllPlanChecking()) {
        frame();
    }
    const double planSeconds = seconds(start);
    REQUIRE(controller.fixAllMergePlan() != nullptr);
    CHECK(controller.fixAllParams().mergeShells);

    pollSeconds = 0.0;
    maxFrameSeconds = 0.0;
    frames = 0;
    start = Clock::now();
    controller.previewFixAll();
    while (controller.fixAllRunning()) {
        frame();
    }
    frame(); // collects the finished candidate
    const double previewSeconds = seconds(start);
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    CHECK(candidate->metrics.shellsAfter == 28);
    const double runSeconds = candidate->fixAll->result.elapsedMs / 1000.0;

    // The Result panel's text, built every frame
    start = Clock::now();
    static_cast<void>(ogun::fixAllPassLines(controller, candidate->fixAll->result));
    static_cast<void>(ogun::fixAllResultLines(candidate->fixAll->result, true));
    const double resultTextSeconds = seconds(start);
    controller.setBackgroundChecks(false);
    WARN("Turkey Fix All as in the GUI: plan check " << planSeconds << " s, preview " << previewSeconds << " s (run " << runSeconds
                                                     << " s; " << frames << " frames polled, " << pollSeconds
                                                     << " s in all, slowest " << maxFrameSeconds << " s), result text "
                                                     << resultTextSeconds << " s");
}

namespace {

/// Waits until the worker has ended without collecting its results (no poll): the frame in which a job
/// finishes just after the user clicked Cancel.
bool waitForWorkerEnd(const AppController& controller) {
    for (int i = 0; i < 60000 && controller.repairWorkerBusy(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !controller.repairWorkerBusy();
}

} // namespace

TEST_CASE("Cancel drops a result that arrives after the click: rebuild preview, Fix All, rebuild rows, Compare",
          "[app][fixall][background]") {
    AppController controller;
    SECTION("Rebuild side: alpha wrap finished but not collected when Cancel is clicked") {
        controller.setMesh(overlappingCubes(), "cubes");
        const Mesh before = controller.mesh();
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        ogun::GlobalRepairParams params = controller.globalRepairParams();
        params.alphaWrapDetail = ogun::settings::kMinAlphaWrapDetail;
        controller.setGlobalRepairParams(params);
        controller.setBackgroundChecks(true);
        controller.setGlobalRepairMethod(ogun::GlobalRepairMethod::AlphaWrap);
        controller.previewGlobalRepair();
        REQUIRE(waitForWorkerEnd(controller));
        CHECK(controller.globalRepairRunning()); // still drawn as running: the Cancel button is there
        controller.cancelGlobalRepair();
        CHECK(controller.globalRepairCancelling());
        CHECK(controller.statusMessage() == "Preview cancelled: nothing was changed.");
        controller.waitForGlobalRepair();
        CHECK_FALSE(controller.globalRepairRunning());
        CHECK(controller.globalCandidates().empty());
        CHECK(controller.shownGlobalCandidate() == nullptr); // no Apply
        CHECK_FALSE(controller.globalPreviewShown());        // the 3D view shows the mesh
        CHECK(controller.fixAllState() == FixAllPanelState::Plan);
        CHECK(controller.mesh().F == before.F);
        CHECK(controller.mesh().V == before.V);
        CHECK_FALSE(controller.canUndo());

        // A preview cancelled while it computes: nothing either; the next one is kept
        controller.previewGlobalRepair();
        controller.cancelGlobalRepair();
        controller.waitForGlobalRepair();
        CHECK(controller.globalCandidates().empty());
        controller.previewGlobalRepair();
        controller.waitForGlobalRepair();
        REQUIRE(controller.shownGlobalCandidate() != nullptr);
        CHECK(controller.shownGlobalCandidate()->success);
    }
    SECTION("Fix All: the run ended but is not collected when Cancel is clicked") {
        controller.setMesh(overlappingCubes(), "cubes");
        const Mesh before = controller.mesh();
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        controller.setBackgroundChecks(true);
        controller.previewFixAll();
        REQUIRE(waitForWorkerEnd(controller));
        CHECK(controller.fixAllRunning());
        CHECK(controller.fixAllState() == FixAllPanelState::Running);
        controller.cancelFixAll();
        CHECK(controller.fixAllCancelling());
        controller.waitForFixAll();
        CHECK_FALSE(controller.fixAllRunning());
        CHECK(controller.fixAllCandidate() == nullptr);
        CHECK(controller.shownGlobalCandidate() == nullptr);
        CHECK(controller.fixAllState() == FixAllPanelState::Plan);
        CHECK(controller.statusMessage() == "Fix All cancelled: nothing was changed.");
        CHECK(controller.mesh().F == before.F);
    }
    SECTION("Rebuild shell #k: the rows ended but are not collected when Cancel is clicked") {
        openFixAllOnOpenBox(controller);
        controller.previewFixAll(); // inline
        REQUIRE(controller.fixAllCandidate() != nullptr);
        const Mesh before = controller.mesh();
        controller.setBackgroundChecks(true);
        controller.rebuildFixAllShell(1);
        REQUIRE(waitForWorkerEnd(controller));
        CHECK(controller.fixAllRebuildRunning());
        controller.cancelGlobalRepair();
        controller.waitForFixAll();
        CHECK_FALSE(controller.fixAllRebuildRunning());
        CHECK(controller.fixAllRebuilds().empty());
        CHECK(controller.fixAllState() == FixAllPanelState::Result); // the Fix All preview stays
        REQUIRE(controller.shownGlobalCandidate() != nullptr);
        CHECK(controller.shownGlobalCandidate()->method == ogun::GlobalRepairMethod::FixAll);
        CHECK(controller.mesh().F == before.F);
    }
    SECTION("Compare: a row collected by the poll of the frame in which Cancel is clicked is dropped too") {
        controller.setMesh(overlappingCubes(), "cubes");
        const Mesh before = controller.mesh();
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        ogun::GlobalRepairParams params = controller.globalRepairParams();
        params.alphaWrapDetail = 300.0; // the last row, slow: still computing when Fix All's row arrives
        params.windingResolution = ogun::settings::kMinWindingGridResolution;
        controller.setGlobalRepairParams(params);
        controller.setBackgroundChecks(true);
        controller.compareAllGlobalRepairs();
        // Frame by frame until the poll brings Fix All's row; Cancel is handled in that same frame
        for (int i = 0; i < 60000 && controller.fixAllCandidate() == nullptr; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            static_cast<void>(controller.pollBackgroundChecks());
        }
        REQUIRE(controller.fixAllCandidate() != nullptr);
        REQUIRE(controller.globalRepairRunning());
        controller.cancelGlobalRepair();
        CHECK(controller.fixAllCandidate() == nullptr);
        controller.waitForFixAll();
        CHECK_FALSE(controller.globalRepairRunning());
        CHECK(controller.fixAllCandidate() == nullptr);
        CHECK(std::none_of(controller.globalCandidates().begin(), controller.globalCandidates().end(),
                           [](const ogun::RepairCandidate& c) { return c.method == ogun::GlobalRepairMethod::AlphaWrap; }));
        CHECK(controller.fixAllState() == FixAllPanelState::Plan);
        CHECK(controller.mesh().F == before.F);
    }
    controller.setBackgroundChecks(false);
}

TEST_CASE("Fix All on Turkey without shell 28 runs multiple passes to heal all thin faces", "[app][controller][fixall][turkey]") {
    const std::filesystem::path turkeyStl =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl";
    if (!std::filesystem::exists(turkeyStl)) {
        return;
    }
    ogun::AppController controller;
    controller.setMesh(ogun::loadStl(turkeyStl), "Turkey.stl");
    REQUIRE(controller.shells().size() == 28);

    // 1. Remove shell #28 (index 27 in 0-based indexing)
    controller.clickShell(27, false, false, {});
    REQUIRE(controller.shells().selectedCount() == 1);
    REQUIRE(controller.deleteSelectedShells());
    REQUIRE(controller.shells().size() == 27);

    // 2. Open Fix All with 3 passes and run it
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll(false);
    controller.waitForFixAll();

    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);

    // Verify the run healed all thin faces and stopped clean. It once needed 3 passes; flips with the thin faces'
    // neighbours and healing the faces a repair leaves thin within the same step now finish it in one.
    CHECK(candidate->fixAll->result.passesRun >= 1);
    CHECK(candidate->fixAll->result.stopReason == ogun::FixAllStopReason::Clean);
    CHECK(candidate->fixAll->result.countsAfter.thinFaces == 0);

    // Apply Fix All and verify no defect remains on the mesh
    REQUIRE(controller.applyFixAll());
    REQUIRE(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    REQUIRE(controller.rowCount(ogun::DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.rowCount(ogun::DefectCategory::NonManifoldEdges) == 0);
    REQUIRE(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
    REQUIRE(controller.rowCount(ogun::DefectCategory::Degenerate) == 0);
}

TEST_CASE("Fix All on cupcakemakerplunger heals all defects in multiple passes", "[app][repair][cupcake]") {
    const std::filesystem::path cupcakePath =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "cupcakemakerplunger.stl";
    REQUIRE(std::filesystem::exists(cupcakePath));

    ogun::AppController controller;
    REQUIRE(controller.loadStlFile(cupcakePath));
    const std::size_t initialThin = controller.rowCount(ogun::DefectCategory::ThinFaces);
    WARN("Initial thin faces on cupcake: " << initialThin);

    controller.openFixAll(4);
    controller.waitForFixAllPlan();
    controller.previewFixAll(false);
    controller.waitForFixAll();

    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);

    CHECK(candidate->fixAll->result.passesRun <= 4);
    CHECK(candidate->fixAll->result.stopReason == ogun::FixAllStopReason::Clean);
    CHECK(candidate->fixAll->result.countsAfter.thinFaces == 0);
    CHECK(candidate->fixAll->result.countsAfter.openEdges == 0);

    REQUIRE(controller.applyFixAll());
    const auto hits = ogun::findFacePairs(controller.mesh(), 1e-6, {});
    if (hits) {
        WARN("Total intersection hits after Fix All: " << hits->size());
        for (const auto& hit : *hits) {
            WARN("Hit: (" << hit.a << ", " << hit.b << ") hit=" << hit.hit << " coplanar=" << hit.coplanar << " touching=" << hit.touching);
        }
    }
    CHECK(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::OpenEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::NonManifoldEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Degenerate) == 0);
}

namespace {

std::filesystem::path tomCornerPath() {
    // CAD part by teil3 (CC BY-SA, examples/tom_x_axis_corner_license.md): flat faces fanned into needles
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" /
           "tom_x_axis_corner.stl";
}

std::filesystem::path tomShuttlePath() {
    // Same CAD project (examples/tom_x_axis_shuttle_license.md): small crossings next to large walls
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" /
           "tom_x_axis_shuttle.stl";
}

} // namespace

TEST_CASE("Fix All on a CAD shuttle keeps its walls", "[app][repair][fixall][tom-shuttle]") {
    // The island step once cut out whole walls (faces 1802, 1805, 2253 among them) to undo 6 small crossings:
    // +2.3 % volume, surface lost by 6 mm
    REQUIRE(std::filesystem::exists(tomShuttlePath()));
    ogun::AppController controller;
    REQUIRE(controller.loadStlFile(tomShuttlePath()));
    const ogun::Mesh before = controller.mesh();

    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll(false);
    controller.waitForFixAll();
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);
    CHECK(candidate->fixAll->result.stopReason == ogun::FixAllStopReason::Clean);

    REQUIRE(controller.applyFixAll());
    CHECK(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    const ogun::RepairMetrics shape = ogun::computeRepairGeometryMetrics(before, controller.mesh());
    CHECK(std::abs(shape.volumeChangePercent()) < 0.5);
    CHECK(shape.maxDeviation < 1.0);
    CHECK(shape.maxLost < 1.0); // mm; the part is 68 x 84 x 35 mm
}

TEST_CASE("Repair All Thin / Sliver Faces on a CAD corner keeps its shape", "[app][repair][thin][tom-corner]") {
    // The healer once cut this part's volume by 9.4 % (a snap dragged a corner 18 mm and turned a wall into a
    // ramp), moved the surface by 5.7 mm and made 44 faces cross
    REQUIRE(std::filesystem::exists(tomCornerPath()));
    ogun::AppController controller;
    REQUIRE(controller.loadStlFile(tomCornerPath()));
    REQUIRE(controller.mesh().numFaces() == 6272);
    REQUIRE(controller.rowCount(ogun::DefectCategory::ThinFaces) == 927);

    controller.openRepairAllThinFaces();
    const ogun::RepairCandidate* shown = controller.shownGlobalCandidate();
    REQUIRE(shown != nullptr);
    REQUIRE(shown->success);
    const ogun::RepairMetrics& m = shown->metrics;
    CHECK(m.thinFacesBefore == 927);
    CHECK(m.thinFacesAfter == 0);
    CHECK(std::abs(m.volumeChangePercent()) < 0.5);
    CHECK(m.maxDeviation < 1.0); // mm; the part is 60 x 78 x 17 mm
    CHECK(m.intersectingFacesBefore == 0);
    CHECK(m.intersectingFacesAfter == 0);

    REQUIRE(controller.applyGlobalRepair());
    CHECK(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
}

TEST_CASE("Fix All on a CAD corner heals every thin face", "[app][repair][fixall][tom-corner]") {
    // Fix All once stopped on a repeated state with 2 needles: a flip passed one on to an unlisted neighbour,
    // and the other was collapsed into a new needle 10 times instead of flipped
    REQUIRE(std::filesystem::exists(tomCornerPath()));
    ogun::AppController controller;
    REQUIRE(controller.loadStlFile(tomCornerPath()));
    const ogun::Mesh before = controller.mesh();

    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll(false);
    controller.waitForFixAll();
    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);
    CHECK(candidate->fixAll->result.stopReason == ogun::FixAllStopReason::Clean);
    CHECK(candidate->fixAll->result.countsAfter.thinFaces == 0);

    REQUIRE(controller.applyFixAll());
    CHECK(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::OpenEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::NonManifoldEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
    // It also fills the holes, so the volume moves a little (+0.16 %, 0.30 mm when this test was written)
    const ogun::RepairMetrics shape = ogun::computeRepairGeometryMetrics(before, controller.mesh());
    CHECK(std::abs(shape.volumeChangePercent()) < 0.5);
    CHECK(shape.maxDeviation < 1.0);
    CHECK(shape.maxLost < 1.0);
}

TEST_CASE("Fix All on cuboctahedron_faces heals all defects", "[app][repair][cuboctahedron]") {
    const std::filesystem::path cubPath =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "cuboctahedron_faces.stl";
    REQUIRE(std::filesystem::exists(cubPath));

    ogun::AppController controller;
    REQUIRE(controller.loadStlFile(cubPath));

    controller.openFixAll(4);
    controller.waitForFixAllPlan();
    controller.previewFixAll(false);
    controller.waitForFixAll();

    const ogun::RepairCandidate* candidate = controller.fixAllCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(candidate->fixAll != nullptr);

    CHECK(candidate->fixAll->result.countsAfter.thinFaces == 0);
    CHECK(candidate->fixAll->result.countsAfter.nonManifoldEdges == 0);
    CHECK(candidate->fixAll->result.countsAfter.openEdges == 0);
    CHECK(candidate->fixAll->result.countsAfter.duplicateFaces == 0);

    REQUIRE(controller.applyFixAll());
    CHECK(controller.rowCount(ogun::DefectCategory::ThinFaces) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::OpenEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::NonManifoldEdges) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Degenerate) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Duplicates) == 0);
    CHECK(controller.rowCount(ogun::DefectCategory::Intersections) == 0);
}
