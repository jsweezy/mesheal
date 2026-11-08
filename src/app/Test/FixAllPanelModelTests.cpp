#include "app/AppController.hpp"
#include "app/FixAllPanelModel.hpp"
#include "core/Settings.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/MergePlan.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using mesheal::AppController;
using mesheal::FixAllPlanRow;
using mesheal::FixAllRowSwitch;
using mesheal::Mesh;
using mesheal::SampleMeshes;
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

    const std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
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
    std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
    controller.checkMesh();
    const bool anyUnknown = rowOf(rows, 1).found == "?" || rowOf(rows, 7).found == "?";
    rows = mesheal::fixAllPlanRows(controller);
    CHECK(rowOf(rows, 1).found == "0 + 0");
    CHECK(rowOf(rows, 2).found == "0");
    CHECK(rowOf(rows, 7).found == "0");
    CHECK(rowOf(rows, 8).found == "0 holes");
    CHECK(rowOf(rows, 5).found == "0 faces");
    CHECK((anyUnknown || controller.categoryHealth(mesheal::DefectCategory::Degenerate).status != mesheal::HealthStatus::NotRun));
}

TEST_CASE("Fix All panel rows: merge estimate and the default tick", "[app][fixall][gui]") {
    {
        AppController controller;
        controller.setMesh(overlappingCubes(), "cubes");
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        const std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
        CHECK(rowOf(rows, 10).on);
        CHECK(rowOf(rows, 10).found == "2 -> 1 shell");
        CHECK(rowOf(rows, 11).on);
        CHECK(rowOf(rows, 11).found == "(required after 10)");
        REQUIRE(controller.fixAllMergePlan() != nullptr);
        CHECK(mesheal::shellsInPlanScope(*controller.fixAllMergePlan()) == 2);
        CHECK(mesheal::shellsAfterPlannedMerge(*controller.fixAllMergePlan()) == 1);
    }
    {
        AppController controller;
        controller.setMesh(separateCubes(), "cubes");
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        const std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
        CHECK_FALSE(rowOf(rows, 10).on);
        CHECK(rowOf(rows, 10).found == "nothing to merge");
        CHECK(rowOf(rows, 11).found == "(skipped: runs only after a merge)");
        CHECK(mesheal::shellsAfterPlannedMerge(*controller.fixAllMergePlan()) == 2);
    }
}

TEST_CASE("Fix All panel rows: a face selection turns off the whole-shell rows", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    const std::vector<Eigen::Index> faces = {0, 1, 2, 3};
    controller.selectFaces(faces, true);
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    const std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
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
    mesheal::FixAllParams params = controller.fixAllParams();
    params.weld = false;
    controller.setFixAllParams(params);
    const std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
    CHECK_FALSE(rowOf(rows, 10).on);
    CHECK_FALSE(rowOf(rows, 11).on);
    CHECK_FALSE(rowOf(rows, 0).on);
    CHECK(rowOf(rows, 10).found == "2 -> 1 shell");
}

TEST_CASE("Fix All panel: the weld note appears only for a large tolerance", "[app][fixall][gui]") {
    mesheal::FixAllWeldPlan weld;
    weld.maxTolerance = 1.0e-6;
    weld.scopeDiagonal = 100.0;
    CHECK(mesheal::weldNoteText(weld, mesheal::Unit::Millimeter).empty());
    weld.noteNeeded = true;
    weld.maxTolerance = 0.5;
    const std::string note = mesheal::weldNoteText(weld, mesheal::Unit::Millimeter);
    CHECK_FALSE(note.empty());
    CHECK(note.find("mm") != std::string::npos);
}

TEST_CASE("Fix All panel: merge estimate text", "[app][fixall][gui]") {
    mesheal::MergePlan plan;
    plan.needsWholeShells = true;
    CHECK(mesheal::mergeEstimateText(plan) == "needs whole shells");
    plan.needsWholeShells = false;
    plan.shellInScope = {1, 1, 1};
    CHECK(mesheal::mergeEstimateText(plan) == "nothing to merge");
    plan.groups.push_back({{0, 1, 2}, {}});
    CHECK(mesheal::mergeEstimateText(plan) == "3 -> 1 shell");
    plan.groups.clear();
    plan.groups.push_back({{0, 1}, {}});
    CHECK(mesheal::mergeEstimateText(plan) == "3 -> 2 shells");
}

TEST_CASE("Fix All panel rows: rows with no box carry a plain note, never a greyed box", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    std::vector<FixAllPlanRow> rows = mesheal::fixAllPlanRows(controller);
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
    rows = mesheal::fixAllPlanRows(controller);
    CHECK_FALSE(rowOf(rows, 3).hasBox());
    CHECK(rowOf(rows, 3).note == "needs whole shells");
    CHECK_FALSE(rowOf(rows, 10).hasBox());
}

TEST_CASE("Fix All panel: step numbers, times and the running text", "[app][fixall][gui]") {
    using mesheal::FixAllPhase;
    using mesheal::FixAllStageId;
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::Weld, FixAllPhase::Repair) == 0);
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::Dedupe, FixAllPhase::Repair) == 1);
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::OrientAgain, FixAllPhase::Repair) == 5);
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::FillHoles, FixAllPhase::Repair) == 8);
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::MergeShells, FixAllPhase::Merge) == 10);
    CHECK(mesheal::fixAllStepNumber(FixAllStageId::FillHoles, FixAllPhase::Cleanup) == 11);

    CHECK(mesheal::fixAllTimeText(6.2) == "6 ms");
    CHECK(mesheal::fixAllTimeText(1234.0) == "1.2 s");
    CHECK(mesheal::fixAllTimeText(65000.0) == "1 min 5 s");

    const mesheal::FixAllRunningText full = mesheal::fixAllRunningText("pass 2 of 3, phase 2, merging 28 shells...");
    CHECK(full.pass == "Pass 2 of 3");
    CHECK(full.phase == "Phase 2: merge shells");
    CHECK(full.detail == "merging 28 shells...");
    const mesheal::FixAllRunningText noPhase = mesheal::fixAllRunningText("pass 1 of 1, checking the result...");
    CHECK(noPhase.pass == "Pass 1 of 1");
    CHECK(noPhase.phase.empty());
    CHECK(noPhase.detail == "checking the result...");
    const mesheal::FixAllRunningText other = mesheal::fixAllRunningText("checking the mesh...");
    CHECK(other.pass.empty());
    CHECK(other.detail == "checking the mesh...");
}

TEST_CASE("Fix All panel: step lists and report lines of a real run", "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == mesheal::FixAllPanelState::Result);
    const mesheal::FixAllCandidate* run = controller.fixAllShown();
    REQUIRE(run != nullptr);
    const mesheal::FixAllResult& result = run->result;

    const std::vector<mesheal::FixAllPassLines> passes = mesheal::fixAllPassLines(controller, result);
    REQUIRE(passes.size() == result.passes.size());
    REQUIRE_FALSE(passes.empty());
    std::size_t steps = 0;
    for (const mesheal::FixAllPassLines& pass : passes) {
        CHECK(pass.header.starts_with("Pass " + std::to_string(pass.pass) + ": "));
        steps += pass.steps.size();
        for (const mesheal::FixAllStepLine& line : pass.steps) {
            CHECK_FALSE(line.text.empty());
            CHECK(line.number >= 0);
            CHECK(line.number <= 11);
        }
    }
    CHECK(steps > 0);
    CHECK(mesheal::fixAllStepsSummary(result).find("pass") != std::string::npos);
    bool sawMerge = false;
    for (const mesheal::FixAllStepLine& line : passes.front().steps) {
        sawMerge = sawMerge || (line.number == 10 && line.dot == mesheal::FixAllDot::Green);
    }
    CHECK(sawMerge);

    const std::vector<mesheal::FixAllResultLine> before = mesheal::fixAllResultLines(result, true);
    REQUIRE(before.size() >= 4);
    CHECK(before.front().text == result.report.passesLine);
    const auto has = [](const std::vector<mesheal::FixAllResultLine>& lines, const std::string& text) {
        return std::any_of(lines.begin(), lines.end(), [&](const mesheal::FixAllResultLine& l) { return l.text == text; });
    };
    CHECK(has(before, result.report.shellsLine));
    CHECK(has(before, result.report.closedLine));
    CHECK(result.report.shellsLine.starts_with("Shells: 2 -> 1"));
    const bool removedLine = std::any_of(before.begin(), before.end(), [](const mesheal::FixAllResultLine& l) {
        return l.action == mesheal::FixAllLineAction::ShowRemoved;
    });
    CHECK(removedLine == !result.mergeRemovedFaces.empty());
    const std::vector<mesheal::FixAllResultLine> after = mesheal::fixAllResultLines(result, false);
    CHECK(std::none_of(after.begin(), after.end(), [](const mesheal::FixAllResultLine& l) {
        return l.action == mesheal::FixAllLineAction::ShowRemoved;
    }));
}

TEST_CASE("Fix All: Back to plan drops the preview and changes nothing", "[app][fixall][gui][undo]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    const Eigen::Index faces = controller.mesh().numFaces();
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll();
    REQUIRE(controller.fixAllState() == mesheal::FixAllPanelState::Result);
    controller.backToFixAllPlan();
    CHECK(controller.fixAllState() == mesheal::FixAllPanelState::Plan);
    CHECK(controller.fixAllCandidate() == nullptr);
    CHECK(controller.mesh().numFaces() == faces);
    CHECK_FALSE(controller.canUndo());

    // After an Apply the Report comes back, and Undo Fix All is still the top entry
    controller.previewFixAll();
    REQUIRE(controller.applyFixAll());
    REQUIRE(controller.fixAllState() == mesheal::FixAllPanelState::Report);
    controller.runFixAllOneMorePass();
    REQUIRE(controller.fixAllState() == mesheal::FixAllPanelState::Result);
    controller.backToFixAllPlan();
    CHECK(controller.fixAllState() == mesheal::FixAllPanelState::Report);
    CHECK(controller.fixAllUndoAvailable());
    CHECK(controller.undoFixAll());
    CHECK(controller.mesh().numFaces() == faces);
}

TEST_CASE("Fix All Rebuild side: the copy carries the live numbers and the trade-offs", "[app][fixall][gui][rebuild]") {
    const mesheal::FixAllRebuildCopy copy = mesheal::fixAllRebuildCopy("0.006 mm", "0.17 mm");
    REQUIRE(copy.costs.size() == 4);
    CHECK(copy.costs[0].find("0.006 mm") != std::string::npos);
    CHECK(copy.costs[1].find("0.17 mm") != std::string::npos);
    CHECK(copy.costs[2].find("cavities") != std::string::npos);
    CHECK(copy.guarantee.find("no original face") != std::string::npos);
    CHECK(copy.advice.find("Repair") != std::string::npos);
    const std::string tradeOff = mesheal::fixAllRebuildTradeOffText();
    for (const char* word : {"offset", "narrower than alpha", "cavities", "original faces", "Apply"}) {
        CHECK(tradeOff.find(word) != std::string::npos);
    }
    CHECK(mesheal::fixAllRebuildButtonLabel(11) == "Rebuild shell #12...");
    CHECK(mesheal::fixAllRebuildCavityText({}).empty());
    CHECK(mesheal::fixAllRebuildCavityText({1}) == "Closed cavity #2 will be filled in.");
    CHECK(mesheal::fixAllRebuildCavityText({1, 4}) == "Closed cavities #2 and #5 will be filled in.");
    CHECK(mesheal::fixAllRebuildCavityText({1, 4, 6}) == "Closed cavities #2, #5 and #7 will be filled in.");
}

TEST_CASE("Fix All Rebuild side: a cavity box names the cavity an alpha wrap would fill", "[app][fixall][gui][rebuild]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createCavityBoxMesh(false), "cavity box");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    const std::vector<int> cavities = controller.fixAllRebuildFilledCavities();
    REQUIRE(cavities.size() == 1);
    CHECK(mesheal::fixAllRebuildCavityText(cavities) == "Closed cavity #" + std::to_string(cavities.front() + 1) + " will be filled in.");
    // A face selection has no whole shells: the GUI hides the switch (the scope kind says so)
    controller.selectFaces(std::vector<Eigen::Index>{0, 1, 2}, true);
    CHECK(controller.fixAllScope().kind == mesheal::FixAllScopeKind::Faces);
}

TEST_CASE("Fix All Rebuild: the residue line offers the button, the rows are tagged, the Report says what was rebuilt",
          "[app][fixall][gui][rebuild]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createOpenShellAmongCubesMesh(), "Overlapping boxes, one open");
    auto hooks = std::make_shared<mesheal::FixAllTestHooks>();
    hooks->adjustStages = [](int, mesheal::FixAllPhase, std::vector<mesheal::FixAllStage>& stages) {
        std::erase_if(stages, [](const mesheal::FixAllStage& s) {
            return s.id == mesheal::FixAllStageId::FillHoles || s.id == mesheal::FixAllStageId::CloseCracks;
        });
    };
    controller.setFixAllTestHooks(hooks);
    controller.openFixAll(3);
    mesheal::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = mesheal::settings::kMinAlphaWrapDetail;
    params.windingResolution = mesheal::settings::kMinWindingGridResolution;
    controller.setGlobalRepairParams(params);
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);

    // Before Apply the open shell's line carries the shell to rebuild; the Report's lines are the same
    const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(controller.fixAllCandidate()->fixAll->result, true);
    const auto it = std::find_if(lines.begin(), lines.end(), [](const mesheal::FixAllResultLine& l) { return l.rebuildShell >= 0; });
    REQUIRE(it != lines.end());
    CHECK(it->warning);
    CHECK(mesheal::fixAllRebuildButtonLabel(it->rebuildShell) == "Rebuild shell #2...");

    controller.rebuildFixAllShell(it->rebuildShell);
    const std::vector<mesheal::RepairCandidate>& rows = controller.fixAllRebuilds();
    REQUIRE(rows.size() == mesheal::kShellRebuildMethods.size());
    CHECK(mesheal::fixAllRebuildRowLabel(rows[0]) == "+ Shrink-Wrap (Outer Envelope) #2  (suggested)");
    CHECK(mesheal::fixAllRebuildRowLabel(rows[1]) == "+ Self-union #2");
    CHECK(mesheal::fixAllRebuildRowLabel(*controller.fixAllCandidate()) == "Fix All");
    REQUIRE(rows[0].success);
    CHECK(mesheal::fixAllRebuiltLine(*rows[0].fixAll->rebuild) == "Shell #2 rebuilt by shrink-Wrap (Outer Envelope) and merged.");

    // Show the row, Apply the shown row: one undo step "Fix All + Rebuild #2", the Report knows the rebuild
    controller.showFixAllRebuild(mesheal::GlobalRepairMethod::AlphaWrap);
    REQUIRE(controller.applyGlobalRepair());
    CHECK(controller.lastUndoActionLabel() == "Fix All + Rebuild #2");
    REQUIRE(controller.fixAllShown() != nullptr);
    REQUIRE(controller.fixAllShown()->rebuild.has_value());
    CHECK(mesheal::fixAllRebuiltLine(*controller.fixAllShown()->rebuild) == "Shell #2 rebuilt by shrink-Wrap (Outer Envelope) and merged.");
}

TEST_CASE("Fix All Rebuild side: a whole-scope rebuild previews and applies as one undo step", "[app][fixall][gui][rebuild][undo]") {
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    mesheal::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = mesheal::settings::kMinAlphaWrapDetail;
    controller.setGlobalRepairParams(params);
    const Mesh before = controller.mesh();
    controller.setGlobalRepairMethod(mesheal::GlobalRepairMethod::AlphaWrap);
    controller.previewGlobalRepair();
    REQUIRE(controller.shownGlobalCandidate() != nullptr);
    REQUIRE(controller.shownGlobalCandidate()->success);
    CHECK(controller.fixAllState() == mesheal::FixAllPanelState::Plan); // no Fix All row: the Rebuild side stays in the plan
    CHECK(controller.mesh().F == before.F);
    REQUIRE(controller.applyGlobalRepair());
    CHECK(controller.lastUndoActionLabel() == "Rebuild (Shrink-Wrap (Outer Envelope))");
    REQUIRE(controller.undoLastModification());
    CHECK(controller.mesh().F == before.F);
    CHECK(controller.mesh().V == before.V);
}

TEST_CASE("Fix All lines: a cavity another shell reaches into is not also reported as 'cavity or inside out?'",
          "[app][fixall][gui]") {
    AppController controller;
    controller.setMesh(SampleMeshes::createCavityReachedMesh(SampleMeshes::CavityReachedVariant::CrossesWall), "cavity reached");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    controller.previewFixAll();
    REQUIRE(controller.fixAllCandidate() != nullptr);
    REQUIRE(controller.fixAllCandidate()->fixAll != nullptr);
    const mesheal::FixAllResult& result = controller.fixAllCandidate()->fixAll->result;
    const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(result, true);
    const auto count = [&](const std::string& part) {
        return std::count_if(lines.begin(), lines.end(), [&](const mesheal::FixAllResultLine& l) { return l.text.find(part) != std::string::npos; });
    };
    CHECK(count("reaches into cavity") >= 1);
    CHECK(count("cavity or inside out?") == 0);
}

TEST_CASE("Fix All lines: repeated crossing lines fold into one group", "[app][fixall][gui]") {
    std::vector<mesheal::FixAllResultLine> lines(1);
    lines[0].text = "Shells: 9 -> 9";
    for (int i = 0; i < 5; ++i) {
        mesheal::FixAllResultLine l;
        l.text = "#" + std::to_string(i + 1) + " and #9 still cross";
        l.shells = {i, 8};
        l.groupKey = "their group reaches into cavity #28";
        lines.push_back(l);
    }
    for (int i = 0; i < 2; ++i) { // too few to group
        mesheal::FixAllResultLine l;
        l.shells = {i, 7};
        l.groupKey = "the merge is off";
        lines.push_back(l);
    }
    const std::vector<mesheal::FixAllLineGroup> groups = mesheal::fixAllLineGroups(lines);
    REQUIRE(groups.size() == 1);
    CHECK(groups[0].summary == "5 shell pairs among 6 shells still cross: their group reaches into cavity #28.");
    CHECK(groups[0].members == std::vector<std::size_t>{1, 2, 3, 4, 5});
    CHECK(groups[0].shells == std::vector<int>{0, 8, 1, 2, 3, 4});
}

namespace {

/// One character per unit of width, so the test can count.
float charWidth(std::string_view text) {
    return static_cast<float>(text.size());
}

bool hasLetterOrDigit(const std::string& word) {
    return std::any_of(word.begin(), word.end(), [](char c) { return std::isalnum(static_cast<unsigned char>(c)) != 0; });
}

/// A residue line "#k still crosses <others>: <why>." as the engine words it (crossing only).
mesheal::FixAllShellResidue crossingResidue(int shell, std::vector<int> others, const std::string& why) {
    mesheal::FixAllShellResidue r;
    r.shell = shell;
    r.crosses = others;
    r.suggestion = mesheal::FixAllSuggestion::ReportOnly;
    std::string list;
    for (std::size_t i = 0; i < others.size(); ++i) {
        if (i > 0) {
            list += i + 1 == others.size() ? " and " : ", ";
        }
        list += "#" + std::to_string(others[i] + 1);
    }
    r.reason = "#" + std::to_string(shell + 1) + " still crosses " + list + ": " + why + ".";
    r.named = {shell};
    r.named.insert(r.named.end(), others.begin(), others.end());
    return r;
}

mesheal::FixAllCrossingLeft crossingPair(int a, int b, const std::string& why) {
    mesheal::FixAllCrossingLeft left;
    left.a = a;
    left.b = b;
    left.facePairs = 9;
    left.whyNotMerged = why;
    left.text = "#" + std::to_string(a + 1) + " and #" + std::to_string(b + 1) + " still cross (9 crossing face pairs): " + why + ".";
    return left;
}

std::size_t countContaining(const std::vector<mesheal::FixAllResultLine>& lines, const std::string& part) {
    return static_cast<std::size_t>(std::count_if(lines.begin(), lines.end(), [&](const mesheal::FixAllResultLine& l) {
        return l.text.find(part) != std::string::npos;
    }));
}

} // namespace

TEST_CASE("Fix All panel: plan labels wrap only at spaces, never inside a number or next to a sign", "[app][fixall][gui]") {
    // The Turkey's row 3 wrapped as "Delete tiny floating shells (< 0." / "1 %)"
    const std::string row3 = "3  Delete tiny floating shells (< 0.1 %)";
    for (float width = 8.0f; width <= 45.0f; width += 1.0f) {
        const std::vector<std::string> lines = mesheal::fixAllWrapLines(row3, width, charWidth);
        const bool together = std::any_of(lines.begin(), lines.end(), [](const std::string& l) {
            return l.find("(< 0.1 %)") != std::string::npos;
        });
        CHECK(together);
    }
    CHECK(mesheal::fixAllWrapLines(row3, 100.0f, charWidth) == std::vector<std::string>{row3});
    CHECK(mesheal::fixAllWrapLines("Heal thin faces (stretched over 100 : 1)", 30.0f, charWidth) ==
          std::vector<std::string>{"Heal thin faces (stretched", "over 100 : 1)"});
    CHECK(mesheal::fixAllWrapLines("", 10.0f, charWidth) == std::vector<std::string>{""});

    // Every row of the plan at every width: lines are the label's words in order, split only at spaces, and
    // no line starts or ends with a word that has no letter or digit
    AppController controller;
    controller.setMesh(overlappingCubes(), "cubes");
    controller.openFixAll(3);
    controller.waitForFixAllPlan();
    for (const FixAllPlanRow& row : mesheal::fixAllPlanRows(controller)) {
        const std::string text = std::to_string(row.number) + "  " + row.label;
        for (float width = 6.0f; width <= 60.0f; width += 1.0f) {
            const std::vector<std::string> lines = mesheal::fixAllWrapLines(text, width, charWidth);
            std::string joined;
            for (const std::string& line : lines) {
                joined += (joined.empty() ? "" : " ") + line;
                const std::string first = line.substr(0, line.find(' '));
                const std::string last = line.substr(line.rfind(' ') + 1);
                // At every break, both neighbouring words have a letter or digit
                CHECK((hasLetterOrDigit(first) || &line == &lines.front()));
                CHECK((hasLetterOrDigit(last) || &line == &lines.back()));
            }
            const auto squash = [](std::string t) { // a break swallows a run of spaces ("11  Split")
                for (std::size_t at = t.find("  "); at != std::string::npos; at = t.find("  ")) {
                    t.erase(at, 1);
                }
                return t;
            };
            CHECK(squash(joined) == squash(text));
        }
    }
}

TEST_CASE("Fix All lines: each crossing pair is said once", "[app][fixall][gui]") {
    SECTION("Merge off: the counted pair line, with the advice to tick the merge") {
        AppController controller;
        controller.setMesh(overlappingCubes(), "cubes");
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        controller.setFixAllMergeShells(false);
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        const mesheal::FixAllResult& result = controller.fixAllCandidate()->fixAll->result;
        REQUIRE(result.report.crossingsLeft.size() == 1);
        REQUIRE(result.report.residue.size() == 2); // the engine still has a line per shell
        const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(result, true);
        CHECK(countContaining(lines, "still cross") == 1);
        CHECK(countContaining(lines, "#1 and #2 still cross (") == 1);
        CHECK(countContaining(lines, "the merge is off. Tick Merge shells and preview again.") == 1);
    }
    SECTION("One of two crossing shells selected") {
        AppController controller;
        controller.setMesh(overlappingCubes(), "cubes");
        controller.selectShellsWhere([](std::size_t s) { return s == 0; });
        controller.openFixAll(3);
        controller.waitForFixAllPlan();
        controller.previewFixAll();
        REQUIRE(controller.fixAllCandidate() != nullptr);
        const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(controller.fixAllCandidate()->fixAll->result, true);
        CHECK(countContaining(lines, "is not selected") == 1);
        CHECK(countContaining(lines, "still crosses") == 0);
        CHECK(countContaining(lines, "still cross (") == 1);
    }
    SECTION("A shell line naming a pair without its own line is kept") {
        mesheal::FixAllResult result;
        result.report.complete = true;
        result.report.residue.push_back(crossingResidue(0, {1, 2}, "the merge is off"));
        result.report.crossingsLeft.push_back(crossingPair(0, 1, "the merge is off"));
        const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(result, true);
        CHECK(countContaining(lines, "#1 still crosses #2 and #3") == 1);
        CHECK(countContaining(lines, "#1 and #2 still cross (") == 1);
    }
}

TEST_CASE("Fix All lines: shells that cross for the same reason fold into one group, whatever shells they name",
          "[app][fixall][gui]") {
    const std::string why = "their group reaches into cavity #7 of #5, so it is not merged (merge them by hand)";
    mesheal::FixAllResult result;
    result.report.complete = true;
    // As on the Turkey: each shell crosses different shells, so the texts differ but the reason is one
    result.report.residue.push_back(crossingResidue(0, {1, 2, 3}, why));
    result.report.residue.push_back(crossingResidue(1, {0, 3}, why));
    result.report.residue.push_back(crossingResidue(2, {0}, why));
    result.report.residue.push_back(crossingResidue(3, {0, 1}, why));
    result.report.residue.push_back(crossingResidue(5, {4}, "#5 is not selected"));

    SECTION("Shell lines without pair lines: one summary of the shells") {
        const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(result, false);
        const std::vector<mesheal::FixAllLineGroup> groups = mesheal::fixAllLineGroups(lines);
        REQUIRE(groups.size() == 1);
        CHECK(groups[0].kind == mesheal::FixAllLineGroupKind::Shells);
        CHECK(groups[0].summary == "4 shells still cross other shells: " + why + ".");
        REQUIRE(groups[0].members.size() == 4);
        for (const std::size_t m : groups[0].members) {
            CHECK(lines[m].text.find(why) != std::string::npos);
            CHECK(lines[m].action == mesheal::FixAllLineAction::ShowThem);
        }
        CHECK(groups[0].shells == std::vector<int>{0, 1, 2, 3});
        CHECK(countContaining(lines, "#6 still crosses #5: #5 is not selected.") == 1); // its own reason: alone
    }
    SECTION("With the counted pair lines: the shell lines are left out, the pairs fold into one summary") {
        for (const auto& [a, b] : std::vector<std::pair<int, int>>{{0, 1}, {0, 2}, {0, 3}, {1, 3}}) {
            result.report.crossingsLeft.push_back(crossingPair(a, b, why));
        }
        result.report.crossingsLeft.push_back(crossingPair(4, 5, "#5 is not selected"));
        const std::vector<mesheal::FixAllResultLine> lines = mesheal::fixAllResultLines(result, false);
        CHECK(countContaining(lines, "still crosses") == 0);
        const std::vector<mesheal::FixAllLineGroup> groups = mesheal::fixAllLineGroups(lines);
        REQUIRE(groups.size() == 1);
        CHECK(groups[0].kind == mesheal::FixAllLineGroupKind::ShellPairs);
        CHECK(groups[0].summary == "4 shell pairs among 4 shells still cross: " + why + ".");
        CHECK(groups[0].members.size() == 4);
    }
}
