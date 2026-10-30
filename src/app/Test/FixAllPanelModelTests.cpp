#include "app/AppController.hpp"
#include "app/FixAllPanelModel.hpp"
#include "core/Settings.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/MergePlan.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using ogun::AppController;
using ogun::FixAllPlanRow;
using ogun::FixAllRowSwitch;
using ogun::Mesh;
using ogun::SampleMeshes;
using Eigen::Vector3d;

namespace {

Mesh box(const Vector3d& min, const Vector3d& max) {
    return SampleMeshes::createBox(min, max);
}

Mesh overlappingCubes() {
    return SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({1.8, 0.2, 0.3}, {3.8, 2.2, 2.3})});
}

Mesh separateCubes() {
    return SampleMeshes::combineMeshes({box({0, 0, 0}, {2, 2, 2}), box({10, 0, 0}, {12, 2, 2})});
}

const FixAllPlanRow& rowOf(const std::vector<FixAllPlanRow>& rows, int number) {
    return rows.at(static_cast<std::size_t>(number));
}

} // namespace

TEST_CASE("Fix All panel rows: the twelve rows, their phases and switches", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();

    const std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
    REQUIRE(rows.size() == 12);
    for (int i = 0; i < 12; ++i) {
        CHECK(rowOf(rows, i).number == i);
        CHECK_FALSE(rowOf(rows, i).label.empty());
        CHECK_FALSE(rowOf(rows, i).tooltip.empty());
    }
    CHECK(rowOf(rows, 0).phase == 1);
    CHECK(rowOf(rows, 9).phase == 1);
    CHECK(rowOf(rows, 10).phase == 2);
    CHECK(rowOf(rows, 11).phase == 3);
    for (const int n : {0, 3, 4, 6, 7, 10}) {
        CHECK(rowOf(rows, n).sw == FixAllRowSwitch::Option);
    }
    for (const int n : {1, 2, 5, 8, 9}) {
        CHECK(rowOf(rows, n).sw == FixAllRowSwitch::Always);
    }
    CHECK(rowOf(rows, 11).sw == FixAllRowSwitch::Required);
    CHECK(rowOf(rows, 0).label == "Weld vertices within 1e-6 x coordinate");
    CHECK(rowOf(rows, 3).label == "Delete tiny floating shells (< 0.1 %)");
    CHECK(rowOf(rows, 7).label == "Heal thin faces (stretched over 100 : 1)");
}

TEST_CASE("Fix All panel rows: found-now counts are ? until Check Mesh, then the Mesh Health counts", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
    controller.checkMesh();
    const bool anyUnknown = rowOf(rows, 1).found == "?" || rowOf(rows, 7).found == "?";
    rows = ogun::fixAllPlanRows(controller);
    CHECK(rowOf(rows, 1).found == "0 + 0");
    CHECK(rowOf(rows, 2).found == "0");
    CHECK(rowOf(rows, 7).found == "0");
    CHECK(rowOf(rows, 8).found == "0 holes");
    CHECK(rowOf(rows, 5).found == "0 faces");
    CHECK((anyUnknown || controller.categoryHealth(ogun::DefectCategory::Degenerate).status != ogun::HealthStatus::NotRun));
}

TEST_CASE("Fix All panel rows: merge estimate and the default tick", "[app][fixall][gui]") {
    {
        AppController controller;
        controller.setMesh(overlappingCubes(), "cubes");
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        const std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
        CHECK(rowOf(rows, 10).on);
        CHECK(rowOf(rows, 10).found == "2 -> 1 shell");
        CHECK(rowOf(rows, 11).on);
        CHECK(rowOf(rows, 11).found == "(required after 10)");
        REQUIRE(controller.fixAllMergePlan() != nullptr);
        CHECK(ogun::shellsInPlanScope(*controller.fixAllMergePlan()) == 2);
        CHECK(ogun::shellsAfterPlannedMerge(*controller.fixAllMergePlan()) == 1);
    }
    {
        AppController controller;
        controller.setMesh(separateCubes(), "cubes");
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        const std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
        CHECK_FALSE(rowOf(rows, 10).on);
        CHECK(rowOf(rows, 10).found == "nothing to merge");
        CHECK(rowOf(rows, 11).found == "(skipped: runs only after a merge)");
        CHECK(ogun::shellsAfterPlannedMerge(*controller.fixAllMergePlan()) == 2);
    }
}

TEST_CASE("Fix All panel rows: a face selection turns off the whole-shell rows", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    const std::vector<Eigen::Index> faces = {0, 1, 2, 3};
    controller.selectFaces(faces, true);
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    const std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
    CHECK_FALSE(rowOf(rows, 3).available);
    CHECK(rowOf(rows, 3).found == "needs whole shells");
    CHECK_FALSE(rowOf(rows, 10).available);
    CHECK(rowOf(rows, 10).found == "needs whole shells");
    CHECK(rowOf(rows, 11).found == "(skipped: runs only after a merge)");
    CHECK(rowOf(rows, 4).available);
}

TEST_CASE("Fix All panel rows: the row 10 tick follows the user's click and params", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.setFixAllMergeShells(false);
    ogun::FixAllParams params = controller.fixAllParams();
    params.weld = false;
    controller.setFixAllParams(params);
    const std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
    CHECK_FALSE(rowOf(rows, 10).on);
    CHECK_FALSE(rowOf(rows, 11).on);
    CHECK_FALSE(rowOf(rows, 0).on);
    CHECK(rowOf(rows, 10).found == "2 -> 1 shell");
}

TEST_CASE("Fix All panel: the weld note appears only for a large tolerance", "[app][fixall][gui]") {
    ogun::FixAllWeldPlan weld;
    weld.maxTolerance = 1.0e-6;
    weld.scopeDiagonal = 100.0;
    CHECK(ogun::weldNoteText(weld, ogun::Unit::Millimeter).empty());
    weld.noteNeeded = true;
    weld.maxTolerance = 0.5;
    const std::string note = ogun::weldNoteText(weld, ogun::Unit::Millimeter);
    CHECK_FALSE(note.empty());
    CHECK(note.find("mm") != std::string::npos);
}

TEST_CASE("Fix All panel: merge estimate text", "[app][fixall][gui]") {
    ogun::MergePlan plan;
    plan.needsWholeShells = true;
    CHECK(ogun::mergeEstimateText(plan) == "needs whole shells");
    plan.needsWholeShells = false;
    plan.shellInScope = {1, 1, 1};
    CHECK(ogun::mergeEstimateText(plan) == "nothing to merge");
    plan.groups.push_back({{0, 1, 2}, {}});
    CHECK(ogun::mergeEstimateText(plan) == "3 -> 1 shell");
    plan.groups.clear();
    plan.groups.push_back({{0, 1}, {}});
    CHECK(ogun::mergeEstimateText(plan) == "3 -> 2 shells");
}

TEST_CASE("Fix All panel rows: rows with no box carry a plain note, never a greyed box", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    std::vector<FixAllPlanRow> rows = ogun::fixAllPlanRows(controller);
    for (const int n : {0, 3, 4, 6, 7, 10}) {
        CHECK(rowOf(rows, n).hasBox());
        CHECK(rowOf(rows, n).note.empty());
    }
    for (const int n : {1, 2, 5, 8, 9}) {
        CHECK_FALSE(rowOf(rows, n).hasBox());
        CHECK(rowOf(rows, n).note == "always runs");
    }
    CHECK_FALSE(rowOf(rows, 11).hasBox());
    CHECK(rowOf(rows, 11).note == "runs after a merge");

    // A face selection removes the box of the whole-shell rows too
    controller.selectFaces(std::vector<Eigen::Index>{0, 1, 2}, true);
    controller.waitForFixAllPlan();
    rows = ogun::fixAllPlanRows(controller);
    CHECK_FALSE(rowOf(rows, 3).hasBox());
    CHECK(rowOf(rows, 3).note == "needs whole shells");
    CHECK_FALSE(rowOf(rows, 10).hasBox());
}

TEST_CASE("Fix All panel: step numbers, times and the running text", "[app][fixall][gui]") {
    using ogun::FixAllPhase;
    using ogun::FixAllStageId;
    CHECK(ogun::fixAllStepNumber(FixAllStageId::Weld, FixAllPhase::Repair) == 0);
    CHECK(ogun::fixAllStepNumber(FixAllStageId::Dedupe, FixAllPhase::Repair) == 1);
    CHECK(ogun::fixAllStepNumber(FixAllStageId::OrientAgain, FixAllPhase::Repair) == 5);
    CHECK(ogun::fixAllStepNumber(FixAllStageId::FillHoles, FixAllPhase::Repair) == 8);
    CHECK(ogun::fixAllStepNumber(FixAllStageId::MergeShells, FixAllPhase::Merge) == 10);
    CHECK(ogun::fixAllStepNumber(FixAllStageId::FillHoles, FixAllPhase::Cleanup) == 11);

    CHECK(ogun::fixAllTimeText(6.2) == "6 ms");
    CHECK(ogun::fixAllTimeText(1234.0) == "1.2 s");
    CHECK(ogun::fixAllTimeText(65000.0) == "1 min 5 s");

    const ogun::FixAllRunningText full = ogun::fixAllRunningText("pass 2 of 3, phase 2, merging 28 shells...");
    CHECK(full.pass == "Pass 2 of 3");
    CHECK(full.phase == "Phase 2: merge shells");
    CHECK(full.detail == "merging 28 shells...");
    const ogun::FixAllRunningText noPhase = ogun::fixAllRunningText("pass 1 of 1, checking the result...");
    CHECK(noPhase.pass == "Pass 1 of 1");
    CHECK(noPhase.phase.empty());
    CHECK(noPhase.detail == "checking the result...");
    const ogun::FixAllRunningText other = ogun::fixAllRunningText("checking the mesh...");
    CHECK(other.pass.empty());
    CHECK(other.detail == "checking the mesh...");
}

TEST_CASE("Fix All panel: step lists and report lines of a real run", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == ogun::FixAllPanelState::Result);
    const ogun::FixAllCandidate* run = controller.fixAllShown();
    REQUIRE(run != nullptr);
    const ogun::FixAllResult& result = run->result;

    const std::vector<ogun::FixAllPassLines> passes = ogun::fixAllPassLines(controller, result);
    REQUIRE(passes.size() == result.passes.size());
    REQUIRE_FALSE(passes.empty());
    std::size_t steps = 0;
    for (const ogun::FixAllPassLines& pass : passes) {
        CHECK(pass.header.starts_with("Pass " + std::to_string(pass.pass) + ": "));
        steps += pass.steps.size();
        for (const ogun::FixAllStepLine& line : pass.steps) {
            CHECK_FALSE(line.text.empty());
            CHECK(line.number >= 0);
            CHECK(line.number <= 11);
        }
    }
    CHECK(steps > 0);
    CHECK(ogun::fixAllStepsSummary(result).find("pass") != std::string::npos);
    bool sawMerge = false;
    for (const ogun::FixAllStepLine& line : passes.front().steps) {
        sawMerge = sawMerge || (line.number == 10 && line.dot == ogun::FixAllDot::Green);
    }
    CHECK(sawMerge);

    const std::vector<ogun::FixAllResultLine> before = ogun::fixAllResultLines(result, true);
    REQUIRE(before.size() >= 4);
    CHECK(before.front().text == result.report.passesLine);
    const auto has = [](const std::vector<ogun::FixAllResultLine>& lines, const std::string& text) {
        return std::any_of(lines.begin(), lines.end(), [&](const ogun::FixAllResultLine& l) { return l.text == text; });
    };
    CHECK(has(before, result.report.shellsLine));
    CHECK(has(before, result.report.closedLine));
    CHECK(result.report.shellsLine.starts_with("Shells: 2 -> 1"));
    const bool removedLine = std::any_of(before.begin(), before.end(), [](const ogun::FixAllResultLine& l) {
        return l.action == ogun::FixAllLineAction::ShowRemoved;
    });
    CHECK(removedLine == !result.mergeRemovedFaces.empty());
    const std::vector<ogun::FixAllResultLine> after = ogun::fixAllResultLines(result, false);
    CHECK(std::none_of(after.begin(), after.end(), [](const ogun::FixAllResultLine& l) {
        return l.action == ogun::FixAllLineAction::ShowRemoved;
    }));
}

TEST_CASE("Fix All: Back to plan drops the preview and changes nothing", "[app][fixall][gui][undo]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    const Eigen::Index faces = controller.mesh().numFaces();
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == ogun::FixAllPanelState::Result);
    controller.backToFixAllPlan();
    CHECK(controller.fixAllState() == ogun::FixAllPanelState::Plan);
    CHECK(controller.fixAllCandidate() == nullptr);
    CHECK(controller.mesh().numFaces() == faces);
    CHECK_FALSE(controller.canUndo());

    // After an Apply the Report comes back, and Undo Fix All is still the top entry
    controller.previewFixAll();
    REQUIRE(controller.applyFixAll());
    REQUIRE(controller.fixAllState() == ogun::FixAllPanelState::Report);
    controller.runFixAllOneMorePass();
    REQUIRE(controller.fixAllState() == ogun::FixAllPanelState::Result);
    controller.backToFixAllPlan();
    CHECK(controller.fixAllState() == ogun::FixAllPanelState::Report);
    CHECK(controller.fixAllUndoAvailable());
    CHECK(controller.undoFixAll());
    CHECK(controller.mesh().numFaces() == faces);
}

TEST_CASE("Fix All Rebuild side: the copy carries the live numbers and the trade-offs", "[app][fixall][gui][rebuild]") {
    const ogun::FixAllRebuildCopy copy = ogun::fixAllRebuildCopy("0.006 mm", "0.17 mm");
    REQUIRE(copy.costs.size() == 4);
    CHECK(copy.costs[0].find("0.006 mm") != std::string::npos);
    CHECK(copy.costs[1].find("0.17 mm") != std::string::npos);
    CHECK(copy.costs[2].find("cavities") != std::string::npos);
    CHECK(copy.guarantee.find("no original face") != std::string::npos);
    CHECK(copy.advice.find("Repair") != std::string::npos);
    const std::string tradeOff = ogun::fixAllRebuildTradeOffText();
    for (const char* word : {"offset", "narrower than alpha", "cavities", "original faces", "Apply"}) {
        CHECK(tradeOff.find(word) != std::string::npos);
    }
    CHECK(ogun::fixAllRebuildButtonLabel(11) == "Rebuild shell #12...");
    CHECK(ogun::fixAllRebuildCavityText({}).empty());
    CHECK(ogun::fixAllRebuildCavityText({1}) == "Closed cavity #2 will be filled in.");
    CHECK(ogun::fixAllRebuildCavityText({1, 4}) == "Closed cavities #2 and #5 will be filled in.");
    CHECK(ogun::fixAllRebuildCavityText({1, 4, 6}) == "Closed cavities #2, #5 and #7 will be filled in.");
}

TEST_CASE("Fix All Rebuild side: a cavity box names the cavity an alpha wrap would fill", "[app][fixall][gui][rebuild]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createCavityBoxMesh(false), "cavity box");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    const std::vector<int> cavities = controller.fixAllRebuildFilledCavities();
    REQUIRE(cavities.size() == 1);
    CHECK(ogun::fixAllRebuildCavityText(cavities) == "Closed cavity #" + std::to_string(cavities.front() + 1) + " will be filled in.");
    // A face selection has no whole shells: the GUI hides the switch (the scope kind says so)
    controller.selectFaces(std::vector<Eigen::Index>{0, 1, 2}, true);
    CHECK(controller.fixAllScope().kind == ogun::FixAllScopeKind::Faces);
}

TEST_CASE("Fix All Rebuild: the residue line offers the button, the rows are tagged, the Report says what was rebuilt",
          "[app][fixall][gui][rebuild]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createOpenShellAmongCubesMesh(), "Overlapping boxes, one open");
    auto hooks = std::make_shared<ogun::FixAllTestHooks>();
    hooks->adjustStages = [](int, ogun::FixAllPhase, std::vector<ogun::FixAllStage>& stages) {
        std::erase_if(stages, [](const ogun::FixAllStage& s) {
            return s.id == ogun::FixAllStageId::FillHoles || s.id == ogun::FixAllStageId::CloseCracks;
        });
    };
    controller.setFixAllTestHooks(hooks);
    controller.openFixAll(3);
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = ogun::settings::kMinAlphaWrapDetail;
    params.windingResolution = ogun::settings::kMinWindingGridResolution;
    controller.setGlobalRepairParams(params);
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);

    // Before Apply the open shell's line carries the shell to rebuild; the Report's lines are the same
    const std::vector<ogun::FixAllResultLine> lines = ogun::fixAllResultLines(controller.fixAllCandidate()->fixAll->result, true);
    const auto it = std::find_if(lines.begin(), lines.end(), [](const ogun::FixAllResultLine& l) { return l.rebuildShell >= 0; });
    REQUIRE(it != lines.end());
    CHECK(it->warning);
    CHECK(ogun::fixAllRebuildButtonLabel(it->rebuildShell) == "Rebuild shell #2...");

    controller.rebuildFixAllShell(it->rebuildShell);
    const std::vector<ogun::RepairCandidate>& rows = controller.fixAllRebuilds();
    REQUIRE(rows.size() == ogun::kShellRebuildMethods.size());
    CHECK(ogun::fixAllRebuildRowLabel(rows[0]) == "+ Alpha wrap #2  (suggested)");
    CHECK(ogun::fixAllRebuildRowLabel(rows[1]) == "+ Self-union #2");
    CHECK(ogun::fixAllRebuildRowLabel(*controller.fixAllCandidate()) == "Fix All");
    REQUIRE(rows[0].success);
    CHECK(ogun::fixAllRebuiltLine(*rows[0].fixAll->rebuild) == "Shell #2 rebuilt by alpha wrap and merged.");

    // Show the row, Apply the shown row: one undo step "Fix All + Rebuild #2", the Report knows the rebuild
    controller.showFixAllRebuild(ogun::GlobalRepairMethod::AlphaWrap);
    REQUIRE(controller.applyGlobalRepair());
    CHECK(controller.lastUndoActionLabel() == "Fix All + Rebuild #2");
    REQUIRE(controller.fixAllShown() != nullptr);
    REQUIRE(controller.fixAllShown()->rebuild.has_value());
    CHECK(ogun::fixAllRebuiltLine(*controller.fixAllShown()->rebuild) == "Shell #2 rebuilt by alpha wrap and merged.");
}

TEST_CASE("Fix All Rebuild side: a whole-scope rebuild previews and applies as one undo step", "[app][fixall][gui][rebuild][undo]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = ogun::settings::kMinAlphaWrapDetail;
    controller.setGlobalRepairParams(params);
    const Mesh before = controller.mesh();
    controller.setGlobalRepairMethod(ogun::GlobalRepairMethod::AlphaWrap);
    controller.previewGlobalRepair();
    REQUIRE(controller.shownGlobalCandidate() != nullptr);
    REQUIRE(controller.shownGlobalCandidate()->success);
    CHECK(controller.fixAllState() == ogun::FixAllPanelState::Plan); // no Fix All row: the Rebuild side stays in the plan
    CHECK(controller.mesh().F == before.F);
    REQUIRE(controller.applyGlobalRepair());
    CHECK(controller.lastUndoActionLabel() == "Rebuild (Alpha wrap)");
    REQUIRE(controller.undoLastModification());
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
}
