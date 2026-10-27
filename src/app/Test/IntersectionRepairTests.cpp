#include "app/AppController.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

using ogun::DefectCategory;
using ogun::GlobalRepairMethod;
using ogun::IntersectionRepairMethod;

namespace {

std::size_t crossings(const ogun::Mesh& mesh) {
    return ogun::checkSelfIntersections(mesh, ogun::CheckParams{}).size();
}

/// Loads the folded cube and opens the Intersecting faces row.
void loadFoldedCube(ogun::AppController& controller) {
    controller.setMesh(ogun::SampleMeshes::createFoldedCubeMesh(), "folded cube");
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);
    controller.selectCategory(DefectCategory::Intersections);
}

/// Focuses the first intersecting face whose island is (or is not) local. Returns false if none.
bool focusIsland(ogun::AppController& controller, bool local) {
    const ogun::CategoryResult& result = *controller.browser().result();
    for (std::size_t e = 0; e < result.size(); ++e) {
        controller.focusDefect(e);
        if (controller.islandInspectorActive() && controller.focusedIsland()->local == local) {
            return true;
        }
    }
    return false;
}

} // namespace

TEST_CASE("Island Inspector: recommended method, live preview, commit and undo",
          "[app][intersections][repair][undo]") {
    ogun::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(controller.intersectionIslandCount() == 2);
    const ogun::Mesh before = controller.mesh();

    REQUIRE(focusIsland(controller, true));
    REQUIRE(controller.focusedIslandAnalysis().recommended == IntersectionRepairMethod::FillFlat);
    REQUIRE(controller.selectedIslandMethod() == IntersectionRepairMethod::FillFlat);
    REQUIRE(controller.islandPreview().valid);
    REQUIRE(controller.islandPreview().newCrossings == 0);
    REQUIRE(controller.repairPreviewShown());
    REQUIRE_FALSE(controller.previewPatchMesh().empty());
    REQUIRE(controller.previewSourceFaces() == controller.islandPreview().sourceFaceIndices);
    REQUIRE(controller.mesh().F == before.F); // previews never touch the mesh

    for (const IntersectionRepairMethod method : ogun::kIntersectionRepairMethods) {
        controller.setSelectedIslandMethod(method);
        INFO(ogun::intersectionRepairMethodLabel(method));
        REQUIRE(controller.islandPreview().method == method);
        REQUIRE(controller.islandPreview().valid);
    }

    controller.setSelectedIslandMethod(IntersectionRepairMethod::FillSmooth);
    REQUIRE(controller.commitIslandRepair());
    REQUIRE(controller.lastUndoActionLabel().rfind("Island Repair (Remove & Fill Smooth", 0) == 0);
    REQUIRE(controller.isModified());
    REQUIRE(crossings(controller.mesh()) > 0); // the pass-through is still there

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.islandInspectorActive());
    REQUIRE(controller.selectedIslandMethod() == IntersectionRepairMethod::FillSmooth);
}

TEST_CASE("R commits the focused island's repair", "[app][intersections][repair]") {
    ogun::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(focusIsland(controller, true));
    REQUIRE(controller.commitFocusedRepair());
    REQUIRE(controller.lastUndoActionLabel().rfind("Island Repair (", 0) == 0);
}

TEST_CASE("Island Inspector recommends Cut & Merge Walls for a pass-through", "[app][intersections][repair]") {
    ogun::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(focusIsland(controller, false));
    REQUIRE(controller.selectedIslandMethod() == IntersectionRepairMethod::SelfUnion);
    REQUIRE(controller.commitIslandRepair());
    REQUIRE(controller.lastUndoActionLabel().rfind("Island Repair (Cut & Merge Walls", 0) == 0);
}

TEST_CASE("Fix Selected repairs the islands of the selected faces in one undo step",
          "[app][intersections][repair][undo]") {
    ogun::AppController controller;
    loadFoldedCube(controller);
    const ogun::Mesh before = controller.mesh();
    controller.selectAllVisibleDefects();
    REQUIRE(controller.fixSelectedIntersections());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Intersections (2 islands)");
    REQUIRE(controller.categoryHealth(DefectCategory::Intersections).status == ogun::HealthStatus::Ok);
    REQUIRE(crossings(controller.mesh()) == 0);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);
}

TEST_CASE("Repair All: preview, compare, show modes, apply and undo", "[app][intersections][repair][undo][global]") {
    ogun::AppController controller;
    controller.setMesh(ogun::SampleMeshes::createFoldedCubeMesh(), "folded cube");
    const ogun::Mesh before = controller.mesh();

    controller.openRepairAllIntersections();
    REQUIRE(controller.activeTool() == ogun::ToolSlot::RepairAllIntersections);
    REQUIRE_FALSE(controller.globalPreviewShown());

    controller.setGlobalRepairMethod(GlobalRepairMethod::SelfUnion);
    controller.previewGlobalRepair();
    REQUIRE(controller.globalCandidates().size() == 1);
    const ogun::RepairCandidate* shown = controller.shownGlobalCandidate();
    REQUIRE(shown != nullptr);
    REQUIRE(shown->method == GlobalRepairMethod::SelfUnion);
    REQUIRE(shown->success);
    REQUIRE(shown->metrics.intersectingFacesAfter == 0);
    REQUIRE(shown->metrics.watertight());
    REQUIRE(shown->vertexDeviation.size() == shown->mesh.numVertices());
    REQUIRE(controller.candidateIsCurrent(*shown));

    // Result mode replaces the whole mesh in 3D; Original hides the candidate
    REQUIRE(controller.globalPreviewShown());
    REQUIRE(controller.buildBaseMeshDisplay().isCut);
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == 0);
    controller.setGlobalPreviewShow(ogun::GlobalPreviewShow::Original);
    REQUIRE_FALSE(controller.globalPreviewShown());
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
    controller.setGlobalPreviewShow(ogun::GlobalPreviewShow::Both);
    REQUIRE(controller.globalPreviewShown());
    // Both ghosts only the faces the repair removed: the kept ones are drawn by the result
    REQUIRE_FALSE(shown->removedFaces.empty());
    REQUIRE(controller.buildBaseMeshDisplay().faceIndices == shown->removedFaces);
    REQUIRE(static_cast<std::size_t>(controller.buildBaseMeshDisplay().F.rows()) == shown->removedFaces.size());

    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = 20.0;
    params.windingResolution = 48;
    controller.setGlobalRepairParams(params);
    REQUIRE_FALSE(controller.candidateIsCurrent(*controller.shownGlobalCandidate()));
    controller.compareAllGlobalRepairs();
    REQUIRE(controller.globalCandidates().size() == ogun::kGlobalRepairMethods.size());
    for (const ogun::RepairCandidate& c : controller.globalCandidates()) {
        INFO(ogun::globalRepairMethodLabel(c.method));
        REQUIRE(c.success);
        REQUIRE(controller.candidateIsCurrent(c));
        REQUIRE(c.metrics.intersectingFacesAfter == 0);
    }
    REQUIRE(controller.shownGlobalCandidate()->method == GlobalRepairMethod::SelfUnion);

    controller.showGlobalCandidate(GlobalRepairMethod::PatchIslands);
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Repair All Intersections (Patch each island)");
    REQUIRE(controller.globalCandidates().empty()); // the mesh changed
    REQUIRE(controller.activeTool() == ogun::ToolSlot::RepairAllIntersections);
    REQUIRE(controller.categoryHealth(DefectCategory::Intersections).status == ogun::HealthStatus::Ok);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);

    // Esc closes the operator and drops its candidates
    controller.previewGlobalRepair();
    REQUIRE_FALSE(controller.globalCandidates().empty());
    REQUIRE(controller.escape());
    REQUIRE(controller.activeTool() == ogun::ToolSlot::None);
    REQUIRE(controller.globalCandidates().empty());
}

TEST_CASE("Repair All runs in the background and respects the shell scope", "[app][intersections][repair][global][shells]") {
    ogun::AppController controller;
    ogun::Mesh mesh = ogun::SampleMeshes::createFoldedCubeMesh();
    ogun::Mesh second = ogun::SampleMeshes::createFoldedCubeMesh();
    second.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    ogun::appendMesh(mesh, second);
    controller.setMesh(std::move(mesh), "two folded cubes");
    REQUIRE(controller.shells().size() == 2);

    std::size_t first = 0;
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        if (controller.shells().shell(s).bbox.max.x() < 10.0) {
            first = s;
        }
    }
    controller.clickShell(first, false, false, {});
    controller.setBackgroundChecks(true);
    controller.openRepairAllIntersections();
    controller.setGlobalRepairMethod(GlobalRepairMethod::PatchIslands);
    controller.previewGlobalRepair();
    controller.waitForGlobalRepair();
    controller.setBackgroundChecks(false);

    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(static_cast<std::size_t>(c->metrics.facesBefore) == controller.shells().shell(first).faces.size());
    REQUIRE(c->metrics.intersectingFacesAfter == 0);
    REQUIRE(crossings(c->mesh) > 0); // the second cube was left alone
}

TEST_CASE("Mesh > Clean Up > Alpha Wrap: one method, offset, apply and undo", "[app][global][wrap][undo][shells]") {
    ogun::AppController controller;
    ogun::Mesh mesh = ogun::SampleMeshes::createFoldedCubeMesh();
    ogun::Mesh second = ogun::SampleMeshes::createFoldedCubeMesh();
    second.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    ogun::appendMesh(mesh, second);
    controller.setMesh(std::move(mesh), "two folded cubes");
    const ogun::Mesh before = controller.mesh();

    controller.openAlphaWrap();
    REQUIRE(controller.activeTool() == ogun::ToolSlot::AlphaWrap);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairMethod() == GlobalRepairMethod::AlphaWrap);
    REQUIRE(controller.globalRepairMethods().size() == 1);

    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = 20.0;
    params.alphaWrapOffset = -1.0; // clamped to 0: automatic
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().alphaWrapOffset == 0.0);
    params.alphaWrapOffset = 0.4;
    controller.setGlobalRepairParams(params);

    // Only the shell in scope is wrapped
    std::size_t first = 0;
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        if (controller.shells().shell(s).bbox.max.x() < 10.0) {
            first = s;
        }
    }
    controller.clickShell(first, false, false, {});
    controller.compareAllGlobalRepairs(); // the wrap only
    REQUIRE(controller.globalCandidates().size() == 1);
    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->params.alphaWrapOffset == 0.4);
    REQUIRE(c->metrics.watertight());
    REQUIRE(c->metrics.intersectingFacesAfter == 0);
    REQUIRE(static_cast<std::size_t>(c->metrics.facesBefore) == controller.shells().shell(first).faces.size());
    REQUIRE(controller.globalPreviewShown());

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Alpha Wrap");
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(crossings(controller.mesh()) > 0); // the second cube was left alone
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
}
