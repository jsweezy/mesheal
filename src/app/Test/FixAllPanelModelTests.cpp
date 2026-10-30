#include "app/AppController.hpp"
#include "app/FixAllPanelModel.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/MergePlan.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
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
