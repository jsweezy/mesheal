#include "app/AppController.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

using mesheal::DefectCategory;
using mesheal::GlobalRepairMethod;
using mesheal::IntersectionRepairMethod;

namespace {

std::size_t crossings(const mesheal::Mesh& mesh) {
    return mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{}).size();
}

/// Loads the folded cube and opens the Intersecting faces row.
void loadFoldedCube(mesheal::AppController& controller) {
    controller.setMesh(mesheal::SampleMeshes::createFoldedCubeMesh(), "folded cube");
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);
    controller.selectCategory(DefectCategory::Intersections);
}

/// Focuses the first intersecting face whose island is (or is not) local. Returns false if none.
bool focusIsland(mesheal::AppController& controller, bool local) {
    const mesheal::CategoryResult& result = *controller.browser().result();
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
    mesheal::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(controller.intersectionIslandCount() == 2);
    const mesheal::Mesh before = controller.mesh();

    REQUIRE(focusIsland(controller, true));
    REQUIRE(controller.focusedIslandAnalysis().recommended == IntersectionRepairMethod::FillFlat);
    REQUIRE(controller.selectedIslandMethod() == IntersectionRepairMethod::FillFlat);
    REQUIRE(controller.islandPreview().valid);
    REQUIRE(controller.islandPreview().newCrossings == 0);
    REQUIRE(controller.repairPreviewShown());
    REQUIRE_FALSE(controller.previewPatchMesh().empty());
    REQUIRE(controller.previewSourceFaces() == controller.islandPreview().sourceFaceIndices);
    REQUIRE(controller.mesh().F == before.F); // previews never touch the mesh

    for (const IntersectionRepairMethod method : mesheal::kIntersectionRepairMethods) {
        controller.setSelectedIslandMethod(method);
        INFO(mesheal::intersectionRepairMethodLabel(method));
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
    mesheal::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(focusIsland(controller, true));
    REQUIRE(controller.commitFocusedRepair());
    REQUIRE(controller.lastUndoActionLabel().rfind("Island Repair (", 0) == 0);
}

TEST_CASE("Island Inspector recommends Cut & Merge Walls for a pass-through", "[app][intersections][repair]") {
    mesheal::AppController controller;
    loadFoldedCube(controller);
    REQUIRE(focusIsland(controller, false));
    REQUIRE(controller.selectedIslandMethod() == IntersectionRepairMethod::SelfUnion);
    REQUIRE(controller.commitIslandRepair());
    REQUIRE(controller.lastUndoActionLabel().rfind("Island Repair (Cut & Merge Walls", 0) == 0);
}

TEST_CASE("Fix Selected repairs the islands of the selected faces in one undo step",
          "[app][intersections][repair][undo]") {
    mesheal::AppController controller;
    loadFoldedCube(controller);
    const mesheal::Mesh before = controller.mesh();
    controller.selectAllVisibleDefects();
    REQUIRE(controller.fixSelectedIntersections());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Intersections (2 islands)");
    REQUIRE(controller.categoryHealth(DefectCategory::Intersections).status == mesheal::HealthStatus::Ok);
    REQUIRE(crossings(controller.mesh()) == 0);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);
}

TEST_CASE("Repair All: preview, compare, show modes, apply and undo", "[app][intersections][repair][undo][global]") {
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createFoldedCubeMesh(), "folded cube");
    const mesheal::Mesh before = controller.mesh();

    controller.openRepairAllIntersections();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::RepairAllIntersections);
    REQUIRE_FALSE(controller.globalPreviewShown());

    controller.setGlobalRepairMethod(GlobalRepairMethod::SelfUnion);
    controller.previewGlobalRepair();
    REQUIRE(controller.globalCandidates().size() == 1);
    const mesheal::RepairCandidate* shown = controller.shownGlobalCandidate();
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
    controller.setGlobalPreviewShow(mesheal::GlobalPreviewShow::Original);
    REQUIRE_FALSE(controller.globalPreviewShown());
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
    controller.setGlobalPreviewShow(mesheal::GlobalPreviewShow::Both);
    REQUIRE(controller.globalPreviewShown());
    // Both ghosts only the faces the repair removed: the kept ones are drawn by the result
    REQUIRE_FALSE(shown->removedFaces.empty());
    REQUIRE(controller.buildBaseMeshDisplay().faceIndices == shown->removedFaces);
    REQUIRE(static_cast<std::size_t>(controller.buildBaseMeshDisplay().F.rows()) == shown->removedFaces.size());

    mesheal::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = 20.0;
    params.windingResolution = 48;
    controller.setGlobalRepairParams(params);
    REQUIRE_FALSE(controller.candidateIsCurrent(*controller.shownGlobalCandidate()));
    controller.compareAllGlobalRepairs();
    REQUIRE(controller.globalCandidates().size() == mesheal::kGlobalRepairMethods.size());
    for (const mesheal::RepairCandidate& c : controller.globalCandidates()) {
        INFO(mesheal::globalRepairMethodLabel(c.method));
        REQUIRE(c.success);
        REQUIRE(controller.candidateIsCurrent(c));
        REQUIRE(c.metrics.intersectingFacesAfter == 0);
    }
    REQUIRE(controller.shownGlobalCandidate()->method == GlobalRepairMethod::SelfUnion);

    controller.showGlobalCandidate(GlobalRepairMethod::PatchIslands);
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Repair All Intersections (Patch each island)");
    REQUIRE(controller.globalCandidates().empty()); // the mesh changed
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::RepairAllIntersections);
    REQUIRE(controller.categoryHealth(DefectCategory::Intersections).status == mesheal::HealthStatus::Ok);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);

    // Esc closes the operator and drops its candidates
    controller.previewGlobalRepair();
    REQUIRE_FALSE(controller.globalCandidates().empty());
    REQUIRE(controller.escape());
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::None);
    REQUIRE(controller.globalCandidates().empty());
}

TEST_CASE("Repair All runs in the background and respects the shell scope", "[app][intersections][repair][global][shells]") {
    mesheal::AppController controller;
    mesheal::Mesh mesh = mesheal::SampleMeshes::createFoldedCubeMesh();
    mesheal::Mesh second = mesheal::SampleMeshes::createFoldedCubeMesh();
    second.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    mesheal::appendMesh(mesh, second);
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

    const mesheal::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(static_cast<std::size_t>(c->metrics.facesBefore) == controller.shells().shell(first).faces.size());
    REQUIRE(c->metrics.intersectingFacesAfter == 0);
    REQUIRE(crossings(c->mesh) > 0); // the second cube was left alone
}

TEST_CASE("Mesh > Clean Up > Alpha Wrap: one method, offset, apply and undo", "[app][global][wrap][undo][shells]") {
    mesheal::AppController controller;
    mesheal::Mesh mesh = mesheal::SampleMeshes::createFoldedCubeMesh();
    mesheal::Mesh second = mesheal::SampleMeshes::createFoldedCubeMesh();
    second.V.rowwise() += Eigen::RowVector3d(20.0, 0.0, 0.0);
    mesheal::appendMesh(mesh, second);
    controller.setMesh(std::move(mesh), "two folded cubes");
    const mesheal::Mesh before = controller.mesh();

    controller.openAlphaWrap();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::AlphaWrap);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairMethod() == GlobalRepairMethod::AlphaWrap);
    REQUIRE(controller.globalRepairMethods().size() == 1);

    mesheal::GlobalRepairParams params = controller.globalRepairParams();
    params.alphaWrapDetail = 20.0;
    REQUIRE(params.alphaWrapOffsetRatio == mesheal::settings::kDefaultAlphaWrapOffsetRatio);
    params.alphaWrapOffsetRatio = -1.0; // clamped to the slider range
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().alphaWrapOffsetRatio == mesheal::settings::kMinAlphaWrapOffsetRatio);
    params.alphaWrapOffsetRatio = 1.0e6;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().alphaWrapOffsetRatio == mesheal::settings::kMaxAlphaWrapOffsetRatio);
    params.alphaWrapOffsetRatio = 10.0;
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
    const mesheal::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->params.alphaWrapOffsetRatio == 10.0);
    {
        // The wrap used offset = alpha / x for the scope's diagonal
        const double alpha = mesheal::alphaWrapAlpha(c->params, controller.shells().shell(first).bbox.diagonal());
        REQUIRE(mesheal::alphaWrapOffset(c->params, alpha) == Catch::Approx(alpha / 10.0));
        REQUIRE(c->metrics.maxDeviation >= 0.9 * alpha / 10.0);
    }

    // A new x (or a new alpha, which moves the offset with it) makes the candidate stale
    params.alphaWrapOffsetRatio = 20.0;
    controller.setGlobalRepairParams(params);
    REQUIRE_FALSE(c->params == controller.globalRepairParams());
    params.alphaWrapOffsetRatio = 10.0;
    params.alphaWrapDetail = 30.0;
    REQUIRE_FALSE(c->params == params);
    params.alphaWrapDetail = 20.0;
    controller.setGlobalRepairParams(params);
    REQUIRE(c->params == controller.globalRepairParams());
    REQUIRE(c->metrics.watertight());
    REQUIRE(c->metrics.intersectingFacesAfter == 0);
    REQUIRE(static_cast<std::size_t>(c->metrics.facesBefore) == controller.shells().shell(first).faces.size());
    REQUIRE(controller.globalPreviewShown());

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Shrink-Wrap (Outer Envelope)");
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(crossings(controller.mesh()) > 0); // the second cube was left alone
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
}

TEST_CASE("Voxel Heal & Remesh and Precision Isosurface operators preview, apply and undo", "[app][intersections]") {
    mesheal::AppController controller;
    controller.setBackgroundChecks(false);
    controller.setMesh(mesheal::SampleMeshes::createFoldedCubeMesh(), "folded");
    const mesheal::Mesh before = controller.mesh();

    // Voxel Heal & Remesh (Fast)
    controller.openVoxelHealRemesh();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::VoxelHealRemesh);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairMethods().size() == 1);
    REQUIRE(controller.globalRepairMethod() == mesheal::GlobalRepairMethod::VoxelHealRemesh);

    mesheal::GlobalRepairParams params = controller.globalRepairParams();
    REQUIRE(params.remeshTargetSize > 0.0);
    params.voxelSmoothingPasses = 99;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().voxelSmoothingPasses == mesheal::settings::kMaxRemeshSmoothingPasses);
    params.remeshTargetSize = 0.75;
    params.voxelSmoothingPasses = 2;
    controller.setGlobalRepairParams(params);

    controller.previewGlobalRepair();
    const mesheal::RepairCandidate* voxelCand = controller.shownGlobalCandidate();
    REQUIRE(voxelCand != nullptr);
    REQUIRE(voxelCand->success);
    REQUIRE(voxelCand->metrics.watertight());
    REQUIRE(voxelCand->metrics.intersectingFacesAfter == 0);
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Voxel Heal & Remesh (Fast)");
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);

    // Precision Isosurface (FEA Grade)
    controller.openPrecisionIsosurface();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::PrecisionIsosurface);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairMethods().size() == 1);
    REQUIRE(controller.globalRepairMethod() == mesheal::GlobalRepairMethod::PrecisionIsosurface);

    params = controller.globalRepairParams();
    params.isosurfaceChordRatio = 0.0001;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().isosurfaceChordRatio == mesheal::settings::kMinIsosurfaceChordRatio);
    params.remeshTargetSize = 1.0;
    params.isosurfaceChordRatio = 0.15;
    params.isosurfaceSmoothingPasses = 1;
    controller.setGlobalRepairParams(params);

    controller.recheckCategory(mesheal::DefectCategory::ThinFaces);
    controller.recheckCategory(mesheal::DefectCategory::InvertedShells);
    controller.previewGlobalRepair();
    const mesheal::RepairCandidate* isoCand = controller.shownGlobalCandidate();
    REQUIRE(isoCand != nullptr);
    REQUIRE(isoCand->success);
    REQUIRE(isoCand->metrics.watertight());
    REQUIRE(isoCand->metrics.intersectingFacesAfter == 0);
    REQUIRE(isoCand->metrics.thinFacesAfter == 0);
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Precision Isosurface (FEA Grade)");
    REQUIRE(controller.categoryHealth(mesheal::DefectCategory::ThinFaces).status != mesheal::HealthStatus::Stale);
    REQUIRE(controller.categoryHealth(mesheal::DefectCategory::InvertedShells).status != mesheal::HealthStatus::Stale);
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 0);
    REQUIRE(controller.rowCount(mesheal::DefectCategory::InvertedShells) == 0);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
}
