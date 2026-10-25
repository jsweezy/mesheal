#include "app/AppController.hpp"
#include "io/SampleMeshes.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>

namespace {

std::size_t thinCount(const ogun::AppController& controller) {
    return controller.categoryHealth(ogun::DefectCategory::ThinFaces).displayCount(ogun::DefectCategory::ThinFaces);
}

ogun::HealthStatus thinStatus(const ogun::AppController& controller) {
    return controller.categoryHealth(ogun::DefectCategory::ThinFaces).status;
}

} // namespace

TEST_CASE("AppController loads sample mesh, changes File Units without scaling V, and scales V explicitly",
          "[app][controller][units]") {
    ogun::AppController controller;
    REQUIRE_FALSE(controller.hasMesh());
    REQUIRE(controller.fileUnit() == ogun::Unit::Millimeter);

    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.hasMesh());
    REQUIRE(controller.mesh().numFaces() == ogun::SampleMeshes::kDefectiveSampleFaceCount);
    REQUIRE(controller.loadedSourceLabel() == "Sample Defective Mesh");

    const Eigen::MatrixXd originalV = controller.mesh().V;
    REQUIRE(controller.mesh().boundingBox().extents().isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-12));

    // Changing File Units dropdown from mm to cm leaves V untouched and converts physical thresholds
    controller.setFileUnit(ogun::Unit::Centimeter);
    REQUIRE(controller.fileUnit() == ogun::Unit::Centimeter);
    REQUIRE(controller.mesh().V.isApprox(originalV, 0.0));
    REQUIRE(controller.thresholds().minAltitude == Catch::Approx(1.0e-5).epsilon(1e-12));
    REQUIRE(controller.thresholds().minArea == Catch::Approx(1.0e-10).epsilon(1e-12));
    REQUIRE(controller.thresholds().maxAspectRatio == Catch::Approx(1000.0).epsilon(1e-12));
    REQUIRE(controller.toleranceConfig().absFloorInUnit(controller.fileUnit()) ==
            Catch::Approx(1.0e-9).epsilon(1e-15));

    // Switch back to mm, then explicitly rescale coordinates from mm to cm
    controller.setFileUnit(ogun::Unit::Millimeter);
    REQUIRE(controller.thresholds().minAltitude == Catch::Approx(1.0e-4).epsilon(1e-12));
    REQUIRE(controller.thresholds().minArea == Catch::Approx(1.0e-8).epsilon(1e-12));

    controller.scaleMeshCoordinatesToUnit(ogun::Unit::Centimeter);
    REQUIRE(controller.fileUnit() == ogun::Unit::Centimeter);
    REQUIRE(controller.mesh().V.isApprox(originalV * 0.1, 1e-14));
    REQUIRE(controller.mesh().boundingBox().extents().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0), 1e-12));
}

TEST_CASE("AppController checks the mesh on load, toggles row vs single-face selection, and builds 3D highlight overlays",
          "[app][controller][diagnostics][selection]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Check Mesh runs after loading: every row is filled in, nothing is selected yet
    REQUIRE(thinStatus(controller) == ogun::HealthStatus::Defects);
    REQUIRE(thinCount(controller) == 3);
    REQUIRE_FALSE(controller.selectedCategory());
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay());

    const std::vector<Eigen::Index> allExpectedDefects = {
        ogun::SampleMeshes::kZeroAreaFaceIndex,
        ogun::SampleMeshes::kMicroscopicFaceIndex,
        ogun::SampleMeshes::kNeedleSliverFaceIndex
    };

    // Selecting the row tints all 3 faces and draws tubes / spheres for them (under the overlay cap)
    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    REQUIRE(controller.activeTool() == ogun::ToolSlot::DefectCategory);
    REQUIRE(controller.browsableResult());
    REQUIRE(controller.browser().visibleRows().size() == 3);
    const ogun::HighlightOverlayGeometry categoryOverlay = controller.buildHighlightOverlay();
    REQUIRE(categoryOverlay.faceColors.size() == 15);
    REQUIRE(categoryOverlay.hasSelectionOverlay());
    REQUIRE(categoryOverlay.overlayPoints.size() == 9);
    REQUIRE(categoryOverlay.overlayEdges.size() == 9);
    REQUIRE(categoryOverlay.overlayFaces.empty());
    for (const Eigen::Index f : allExpectedDefects) {
        REQUIRE(categoryOverlay.faceColors[static_cast<std::size_t>(f)].isApprox(
            controller.overlayAppearance().categoryTintColor));
    }

    // Picking a defective face focuses it: it is drawn by the selected-face overlay
    REQUIRE(controller.pickFace(ogun::SampleMeshes::kNeedleSliverFaceIndex) == ogun::PickOutcome::FocusedDefect);
    REQUIRE(controller.focusedFaceIndex() == ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.highlightedFaceIndices() ==
            std::vector<Eigen::Index>{ogun::SampleMeshes::kNeedleSliverFaceIndex});

    const ogun::HighlightOverlayGeometry singleOverlay = controller.buildHighlightOverlay();
    REQUIRE(singleOverlay.overlayFaces.size() == 1);
    REQUIRE(singleOverlay.overlayPoints.size() == 9);
    REQUIRE(singleOverlay.overlayEdges.size() == 9);
    REQUIRE_FALSE(singleOverlay.faceColors[0].isApprox(
        singleOverlay.faceColors[static_cast<std::size_t>(ogun::SampleMeshes::kNeedleSliverFaceIndex)], 1e-6));

    // A clean face is not in the selected row: only a status message
    REQUIRE(controller.pickFace(0) == ogun::PickOutcome::NotInSelectedRow);
    REQUIRE(controller.focusedFaceIndex() == ogun::SampleMeshes::kNeedleSliverFaceIndex);

    // Esc clears the defect selection first, then closes the Tool
    REQUIRE(controller.escape());
    REQUIRE(controller.selectedCategory() == ogun::DefectCategory::ThinFaces);
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE(controller.escape());
    REQUIRE_FALSE(controller.selectedCategory());
    REQUIRE(controller.activeTool() == ogun::ToolSlot::None);
    REQUIRE_FALSE(controller.escape());

    // With no row selected, clicking a defect opens its row; clicking a clean face just picks it
    REQUIRE(controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex) == ogun::PickOutcome::SelectedRow);
    REQUIRE(controller.selectedCategory() == ogun::DefectCategory::ThinFaces);
    controller.setMesh(ogun::SampleMeshes::createCubeMesh(), "Clean Cube");
    REQUIRE(controller.pickFace(0) == ogun::PickOutcome::PickedFace);
    REQUIRE(controller.highlightedFaceIndices() == std::vector<Eigen::Index>{0});
    REQUIRE_FALSE(controller.inspectorActive());
    controller.clearSelection();
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay());
}

TEST_CASE("AppController computes 3D camera focus bounding box and look-at target when '.' is triggered",
          "[app][controller][camera]") {
    ogun::AppController controller;

    // Without a loaded mesh, camera focus is invalid
    REQUIRE_FALSE(controller.computeCameraFocusTarget().valid);

    controller.loadSampleDefectiveMesh();
    controller.selectCategory(ogun::DefectCategory::ThinFaces);

    SECTION("Focusing on a single microscopic defective face centers camera on that facet's centroid") {
        controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex);

        const Eigen::Vector3d viewDir(0.0, 0.0, -1.0); // Looking straight down -Z
        const ogun::CameraFocusTarget focus = controller.computeCameraFocusTarget(viewDir);

        const Eigen::Vector3d expectedCentroid(2.0 + 0.0001 / 3.0, 2.0 + 0.0001 / 3.0, 10.0);
        REQUIRE(focus.valid);
        REQUIRE(focus.selectionBounds.min.isApprox(Eigen::Vector3d(2.0, 2.0, 10.0), 1e-12));
        REQUIRE(focus.selectionBounds.max.isApprox(Eigen::Vector3d(2.0001, 2.0001, 10.0), 1e-12));
        REQUIRE(focus.targetCenter.isApprox(expectedCentroid, 1e-12));
        REQUIRE(controller.orbitPivot().isApprox(expectedCentroid, 1e-12));
        REQUIRE(focus.focusRadius > 0.0);
        // Camera is positioned at targetCenter - viewDir * focusRadius (i.e. above +Z looking down at z=10)
        REQUIRE(focus.cameraPosition.isApprox(
            focus.targetCenter - viewDir.normalized() * focus.focusRadius, 1e-12));
    }

    SECTION("Focusing on the entire Thin Faces category bounds all offending faces") {
        const ogun::CameraFocusTarget focus = controller.computeCameraFocusTarget();

        REQUIRE(focus.valid);
        // All 3 faces span X in [0, 10], Y in [0, 5.0002], Z in [0, 10]
        REQUIRE(focus.selectionBounds.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
        REQUIRE(focus.selectionBounds.max.isApprox(Eigen::Vector3d(10.0, 5.0002, 10.0), 1e-12));
        REQUIRE(focus.targetCenter.isApprox(Eigen::Vector3d(5.0, 2.5001, 5.0), 1e-12));
    }

    SECTION("Focusing with no active selection frames the entire mesh bounding box") {
        controller.clearSelection();
        const ogun::CameraFocusTarget focus = controller.computeCameraFocusTarget();

        REQUIRE(focus.valid);
        REQUIRE(focus.selectionBounds.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
        REQUIRE(focus.selectionBounds.max.isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-12));
        REQUIRE(focus.targetCenter.isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));
    }
}

TEST_CASE("AppController loads STL file from disk and manages ghost mesh settings",
          "[app][controller][io]") {
    ogun::AppController controller;

    // Write a temporary cube STL to disk and load via AppController::loadStlFile
    const std::string asciiStl = ogun::SampleMeshes::createCubeAsciiStl(20.0);
    const auto tempDir = std::filesystem::temp_directory_path();
    const auto uniqueSuffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto tempPath = tempDir / ("ogun_app_controller_test_" + uniqueSuffix + ".stl");
    {
        std::ofstream out(tempPath, std::ios::binary);
        out << asciiStl;
    }

    const bool loaded = controller.loadStlFile(tempPath);
    std::filesystem::remove(tempPath);

    REQUIRE(loaded);
    REQUIRE(controller.hasMesh());
    REQUIRE(controller.mesh().numVertices() == 8);
    REQUIRE(controller.mesh().numFaces() == 12);
    REQUIRE(controller.lastErrorMessage().empty());

    // A clean cube has no thin faces and is watertight
    REQUIRE(thinStatus(controller) == ogun::HealthStatus::Ok);
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.watertightState() == ogun::WatertightState::Watertight);
    REQUIRE(controller.totalProblemCount() == 0);
    REQUIRE_FALSE(controller.isModified());
    REQUIRE(controller.currentFilePath().filename() == tempPath.filename());

    // Ghost mesh toggle & opacity clamping
    REQUIRE_FALSE(controller.ghostMeshEnabled());
    controller.setGhostMeshEnabled(true);
    REQUIRE(controller.ghostMeshEnabled());
    controller.setGhostMeshOpacity(0.4);
    REQUIRE(controller.ghostMeshOpacity() == Catch::Approx(0.4));
}

TEST_CASE("AppController persists overlay styles across defect selections and computes the near-clip zoom scale",
          "[app][controller][highlight][zoom]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(ogun::DefectCategory::ThinFaces);

    // Screen-space defaults: 3 px vertex spheres and 1 px edges on both overlay surfaces
    REQUIRE(controller.overlayAppearance().pointRadiusPx == Catch::Approx(3.0));
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(controller.overlayAppearance().previewPatch.edgeWidthPx == Catch::Approx(1.0));

    // User customizes point radius and defect edge width
    ogun::OverlayAppearance appearance = controller.overlayAppearance();
    appearance.pointRadiusPx = 6.0;
    appearance.selectedFaces.edgeWidthPx = 2.0;
    controller.setOverlayAppearance(appearance);

    // Switching between defective faces, category selection, and re-running diagnostics must NOT reset styles
    const auto requireCustomStyles = [&controller]() {
        REQUIRE(controller.overlayAppearance().pointRadiusPx == Catch::Approx(6.0));
        REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(2.0));
    };
    controller.pickFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
    requireCustomStyles();
    controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    requireCustomStyles();
    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    requireCustomStyles();
    controller.checkMesh();
    requireCustomStyles();

    // Zoom scale (drives the near clip plane) relative to the full-mesh reference distance
    controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    const ogun::CameraFocusTarget focus = controller.computeCameraFocusTarget(Eigen::Vector3d(0.0, 0.0, -1.0));
    REQUIRE(focus.valid);

    const double refDist = controller.referenceFullMeshViewDistance();
    REQUIRE(refDist > 0.0);

    const Eigen::Vector3d camFar = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 1.2);
    REQUIRE(controller.computeHighlightZoomScale(camFar) == Catch::Approx(1.0));

    const Eigen::Vector3d camQuarter = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 0.25);
    REQUIRE(controller.computeHighlightZoomScale(camQuarter) == Catch::Approx(0.25).epsilon(1e-6));

    const Eigen::Vector3d camClose = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 0.01);
    REQUIRE(controller.computeHighlightZoomScale(camClose) == Catch::Approx(0.01).epsilon(1e-6));
}

TEST_CASE("AppController updates orbit pivot to face centroid and aligns camera to +Normal / -Normal with '.' toggle",
          "[app][controller][camera][pivot][normal]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Initially with no selection, orbit pivot is at the full mesh center (5, 5, 5)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));

    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    // Selecting the thin-face row sets orbit pivot to the defects' bounds center (5.0, 2.5001, 5.0)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 2.5001, 5.0), 1e-12));

    // Selecting Face #14 (kNeedleSliverFaceIndex: vertices (0,5,10), (10,5,10), (5,5.0002,10))
    // immediately sets orbit pivot to its true centroid (5.0, 5.0 + 0.0002/3.0, 10.0)
    controller.pickFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    const Eigen::Vector3d expectedNeedleCentroid(5.0, 5.0 + 0.0002 / 3.0, 10.0);
    REQUIRE(controller.computeFaceCentroid(ogun::SampleMeshes::kNeedleSliverFaceIndex)
                .isApprox(expectedNeedleCentroid, 1e-12));
    REQUIRE(controller.orbitPivot().isApprox(expectedNeedleCentroid, 1e-12));

    // Face #14 lies on z=10 with CCW vertices in XY, so its outward normal is +Z (0, 0, 1)
    const Eigen::Vector3d needleNormal =
        controller.computeFaceInspectionNormal(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(needleNormal.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-12));

    // 1st '.' press (AutoToggleNormal): frames from +Normal side (camera above +Z looking along -Z)
    const ogun::CameraFocusTarget focusPos = controller.focusOnSelection(
        ogun::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusPos.valid);
    REQUIRE(focusPos.normalSideSign == 1);
    REQUIRE(controller.lastFocusedNormalSign() == 1);
    REQUIRE(focusPos.targetCenter.isApprox(expectedNeedleCentroid, 1e-12));
    REQUIRE(focusPos.cameraPosition.isApprox(
        expectedNeedleCentroid + Eigen::Vector3d(0.0, 0.0, focusPos.focusRadius), 1e-12));
    // Up vector must be unit-length and orthogonal to the look direction (0, 0, -1)
    REQUIRE(focusPos.cameraUpDirection.norm() == Catch::Approx(1.0).epsilon(1e-12));
    REQUIRE(std::abs(focusPos.cameraUpDirection.dot(Eigen::Vector3d(0.0, 0.0, -1.0))) < 1e-12);

    // 2nd '.' press (or rapid '..'): toggles to -Normal side (camera below -Z looking along +Z)
    const ogun::CameraFocusTarget focusNeg = controller.focusOnSelection(
        ogun::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusNeg.valid);
    REQUIRE(focusNeg.normalSideSign == -1);
    REQUIRE(controller.lastFocusedNormalSign() == -1);
    REQUIRE(focusNeg.cameraPosition.isApprox(
        expectedNeedleCentroid - Eigen::Vector3d(0.0, 0.0, focusNeg.focusRadius), 1e-12));

    // 3rd '.' press: toggles back to +Normal side
    const ogun::CameraFocusTarget focusPosAgain = controller.focusOnSelection(
        ogun::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusPosAgain.normalSideSign == 1);

    // If the user orbits the camera manually, notifyCameraRotated() resets the toggle so next '.' is +Normal
    controller.notifyCameraRotated();
    REQUIRE(controller.lastFocusedNormalSign() == 0);
    const ogun::CameraFocusTarget focusAfterOrbit = controller.focusOnSelection(
        ogun::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusAfterOrbit.normalSideSign == 1);

    // 'Shift + .' (FocusNormalSide::NegativeNormal) always frames -Normal directly
    const ogun::CameraFocusTarget directNeg = controller.focusOnSelection(
        ogun::FocusNormalSide::NegativeNormal);
    REQUIRE(directNeg.normalSideSign == -1);
    REQUIRE(controller.lastFocusedNormalSign() == -1);

    // Panning translates the orbit pivot in world space, and pressing '.' snaps it back to the face centroid
    controller.translateOrbitPivot(Eigen::Vector3d(1.5, -0.5, 0.25));
    REQUIRE(controller.orbitPivot().isApprox(
        expectedNeedleCentroid + Eigen::Vector3d(1.5, -0.5, 0.25), 1e-12));
    controller.focusOnSelection(ogun::FocusNormalSide::PositiveNormal);
    REQUIRE(controller.orbitPivot().isApprox(expectedNeedleCentroid, 1e-12));

    // Collinear zero-area face (Face #12 along X-axis (0,0,0)->(5,0,0)->(10,0,0)) gets a valid perpendicular normal
    controller.pickFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
    const Eigen::Vector3d zeroAreaNormal =
        controller.computeFaceInspectionNormal(ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(zeroAreaNormal.norm() == Catch::Approx(1.0).epsilon(1e-12));
    // Must be perpendicular to the collinear edge direction (1, 0, 0)
    REQUIRE(std::abs(zeroAreaNormal.dot(Eigen::Vector3d(1.0, 0.0, 0.0))) < 1e-12);

    // Home reset returns orbit pivot to full mesh center
    controller.resetOrbitPivotToMeshCenter();
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));
    REQUIRE(controller.lastFocusedNormalSign() == 0);
}

TEST_CASE("AppController resolves the Ogun application toolbar icon asset",
          "[app][controller][icon]") {
    const std::filesystem::path iconPath = ogun::AppController::resolveApplicationIconPath();
    REQUIRE_FALSE(iconPath.empty());
    REQUIRE(std::filesystem::exists(iconPath));
    REQUIRE((iconPath.filename() == "ogun.png" || iconPath.filename() == "ogun.jpeg"));
}

namespace {

std::filesystem::path makeTempStlPath(const std::string& tag) {
    const auto uniqueSuffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return std::filesystem::temp_directory_path() / ("ogun_app_" + tag + "_" + uniqueSuffix + ".stl");
}

// Planar disk with needle (p, q, r) (face 0) whose 1e-4 short edge p-q is shared with needle (q, p, w).
ogun::Mesh makeNeedleDiskMesh() {
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << -1.0,   0.0, 0.0,
               0.0,   0.0, 0.0,
               1.0e-4, 0.0, 0.0,
               1.0,   0.0, 0.0,
               5.0e-5, 1.0, 0.0,
               5.0e-5, -1.0, 0.0;
    mesh.F.resize(6, 3);
    mesh.F << 1, 2, 4,
              2, 1, 5,
              0, 1, 4,
              2, 3, 4,
              1, 0, 5,
              3, 2, 5;
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("AppController selecting a single face analyzes it and builds a live 2-ring repair preview",
          "[app][controller][repair][preview]") {
    ogun::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    const ogun::Mesh before = controller.mesh();

    REQUIRE(controller.pickFace(0) == ogun::PickOutcome::SelectedRow);
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.selectedFaceAnalysis().faceIndex == 0);
    REQUIRE(controller.selectedFaceAnalysis().type == ogun::SliverType::Needle);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::CollapseShortestEdge);

    const ogun::SingleFaceRepairPreview& preview = controller.repairPreview();
    REQUIRE(preview.valid);
    REQUIRE(preview.appliedAction == ogun::HealAction::CollapseShortestEdge);
    REQUIRE(preview.previewPatchMesh.numFaces() == before.numFaces() - 2);
    REQUIRE(controller.repairPreviewVisible());

    // Previewing never mutates the base mesh
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE_FALSE(controller.canUndo());

    controller.setRepairPreviewVisible(false);
    REQUIRE_FALSE(controller.repairPreviewVisible());

    // Clearing the selection drops the preview
    controller.clearSelection();
    REQUIRE_FALSE(controller.repairPreview().valid);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::None);
}

TEST_CASE("AppController switching HealAction re-evaluates the preview and flags unsafe strategies",
          "[app][controller][repair][preview]") {
    ogun::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    controller.pickFace(0);
    REQUIRE(controller.repairPreview().valid);

    // Removing an interior needle would open a hole
    controller.setSelectedHealAction(ogun::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().appliedAction == ogun::HealAction::RemoveDegenerateFace);
    REQUIRE_FALSE(controller.repairPreview().valid);
    REQUIRE_FALSE(controller.repairPreview().statusMessage.empty());
    REQUIRE(controller.repairPreview().previewPatchMesh.empty());

    controller.setSelectedHealAction(ogun::HealAction::CollapseShortestEdge);
    REQUIRE(controller.repairPreview().valid);

    // Re-selecting the same face keeps the user's chosen strategy
    controller.setSelectedHealAction(ogun::HealAction::RemoveDegenerateFace);
    controller.pickFace(0);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::RemoveDegenerateFace);

    // Without a single-face selection, choosing a strategy is ignored
    controller.clearSelection();
    controller.setSelectedHealAction(ogun::HealAction::FlipLongestEdge);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::None);
}

TEST_CASE("AppController commits a single-face repair, refreshes diagnostics, and undoes it",
          "[app][controller][repair][undo]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    const ogun::Mesh before = controller.mesh();
    REQUIRE(thinCount(controller) == 3);
    REQUIRE_FALSE(controller.isModified());

    controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().valid);

    REQUIRE(controller.commitSelectedFaceRepair());
    REQUIRE(controller.mesh().numFaces() == before.numFaces() - 1);
    REQUIRE(controller.mesh().F.maxCoeff() < controller.mesh().numVertices());
    REQUIRE(thinCount(controller) == 2);
    REQUIRE(controller.isModified());
    REQUIRE(controller.selectedCategory() == ogun::DefectCategory::ThinFaces);
    REQUIRE_FALSE(controller.inspectorActive());
    REQUIRE(controller.loadedSourceLabel() == "Sample Defective Mesh");
    REQUIRE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel() == "Single-Face Repair (Face #13)");

    REQUIRE(controller.undoLastModification());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel().empty());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(thinCount(controller) == 3);
    REQUIRE_FALSE(controller.isModified());
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.focusedFaceIndex() == ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(controller.selectedHealAction() == ogun::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().valid);

    // Nothing left to undo
    REQUIRE_FALSE(controller.undoLastModification());
}

TEST_CASE("AppController rejects an unsafe single-face commit without recording undo",
          "[app][controller][repair][undo]") {
    ogun::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    const ogun::Mesh before = controller.mesh();

    // No single-face selection -> nothing to commit
    REQUIRE_FALSE(controller.commitSelectedFaceRepair());

    controller.pickFace(0);
    controller.setSelectedHealAction(ogun::HealAction::RemoveDegenerateFace);
    REQUIRE_FALSE(controller.commitSelectedFaceRepair());
    REQUIRE_FALSE(controller.lastRepairStatusMessage().empty());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("AppController fixAllThinFaces heals the sample mesh and can be undone",
          "[app][controller][repair][batch][undo]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    const ogun::Mesh before = controller.mesh();
    REQUIRE(thinCount(controller) == 3);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(thinStatus(controller) == ogun::HealthStatus::Ok);
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Thin / Sliver Faces");
    REQUIRE_FALSE(controller.lastRepairStatusMessage().empty());

    // A clean mesh has nothing to fix and records no further undo entry
    REQUIRE_FALSE(controller.fixAllThinFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Thin / Sliver Faces");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(thinCount(controller) == 3);
    REQUIRE(controller.selectedCategory() == ogun::DefectCategory::ThinFaces);
    REQUIRE(controller.browser().visibleRows().size() == 3);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController records coordinate scaling on the undo stack",
          "[app][controller][units][undo]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    const Eigen::MatrixXd originalV = controller.mesh().V;
    const ogun::ThinFaceThresholds originalThresholds = controller.thresholds();

    controller.scaleMeshCoordinatesToUnit(ogun::Unit::Centimeter);
    REQUIRE(controller.mesh().V.isApprox(originalV * 0.1, 1e-14));
    REQUIRE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel() == "Scale Mesh Coordinates (mm -> cm)");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.fileUnit() == ogun::Unit::Millimeter);
    REQUIRE(controller.mesh().V == originalV);
    REQUIRE(controller.thresholds().minAltitude == originalThresholds.minAltitude);
    REQUIRE(controller.thresholds().minArea == originalThresholds.minArea);
    REQUIRE(thinCount(controller) == 3);
}

TEST_CASE("AppController undoes a sequence of modifications step by step and clears history on load",
          "[app][controller][undo]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    const ogun::Mesh original = controller.mesh();

    controller.pickFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.commitSelectedFaceRepair());
    const ogun::Mesh afterSingle = controller.mesh();
    REQUIRE(thinCount(controller) == 2);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Thin / Sliver Faces");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == afterSingle.F);
    REQUIRE(thinCount(controller) == 2);
    REQUIRE(controller.lastUndoActionLabel() == "Single-Face Repair (Face #12)");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == original.F);
    REQUIRE(thinCount(controller) == 3);
    REQUIRE(controller.focusedFaceIndex() == ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE_FALSE(controller.canUndo());

    // Loading a new mesh discards the undo history
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.canUndo());
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController saves the repaired mesh as Binary and ASCII STL that reload clean",
          "[app][controller][io][save]") {
    ogun::AppController controller;

    // Saving without a mesh fails with an error message
    REQUIRE_FALSE(controller.saveStlFile(makeTempStlPath("empty")));
    REQUIRE_FALSE(controller.lastErrorMessage().empty());

    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.isModified());
    const ogun::Mesh repaired = controller.mesh();

    for (const ogun::StlFormat format : {ogun::StlFormat::Binary, ogun::StlFormat::Ascii}) {
        const auto path = makeTempStlPath(format == ogun::StlFormat::Binary ? "binary" : "ascii");
        REQUIRE(controller.saveStlFile(path, format));
        REQUIRE(controller.lastErrorMessage().empty());
        REQUIRE_FALSE(controller.isModified());
        REQUIRE(controller.currentFilePath() == path);

        ogun::AppController reloaded;
        const bool loaded = reloaded.loadStlFile(path);
        std::filesystem::remove(path);
        REQUIRE(loaded);
        REQUIRE(reloaded.mesh().numFaces() == repaired.numFaces());
        REQUIRE(reloaded.mesh().numVertices() == repaired.numVertices());
        REQUIRE(thinCount(reloaded) == 0);
    }

    REQUIRE_FALSE(controller.saveStlFile(
        std::filesystem::path("/nonexistent_ogun_dir_xyz") / "out.stl"));
    REQUIRE_FALSE(controller.lastErrorMessage().empty());
}

TEST_CASE("AppController overlay styles default to 1 px edges and drive the selected face color",
          "[app][controller][highlight][appearance]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();

    const ogun::OverlayAppearance& defaults = controller.overlayAppearance();
    REQUIRE(defaults.selectedFaces.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(defaults.previewPatch.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(defaults.previewPatch.color.isApprox(Eigen::Vector3d(0.20, 0.85, 0.30)));
    // Pre-repair edges are thinner than the patch edges and semi-transparent so surviving edges show through
    REQUIRE(defaults.originalEdgeWidthPx == Catch::Approx(0.5));
    REQUIRE(defaults.originalEdgeWidthPx < defaults.previewPatch.edgeWidthPx);
    REQUIRE(defaults.originalEdgeOpacity == Catch::Approx(0.5));

    ogun::OverlayAppearance custom = defaults;
    custom.selectedFaces.color = Eigen::Vector3d(0.1, 0.2, 1.5); // out-of-range channel is clamped
    custom.selectedFaces.edgeWidthPx = 100.0;                   // clamped to the px maximum
    custom.pointRadiusPx = 0.0;                                 // clamped to the px minimum
    custom.originalEdgeWidthPx = 0.0;                           // clamped to the px minimum
    custom.originalEdgeOpacity = 3.0;                           // clamped to fully opaque
    controller.setOverlayAppearance(custom);
    REQUIRE(controller.overlayAppearance().selectedFaces.color.isApprox(Eigen::Vector3d(0.1, 0.2, 1.0)));
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx ==
            Catch::Approx(ogun::settings::kMaxOverlayEdgeWidthPx));
    REQUIRE(controller.overlayAppearance().pointRadiusPx ==
            Catch::Approx(ogun::settings::kMinHighlightPointRadiusPx));
    REQUIRE(controller.overlayAppearance().originalEdgeWidthPx ==
            Catch::Approx(ogun::settings::kMinOverlayEdgeWidthPx));
    REQUIRE(controller.overlayAppearance().originalEdgeOpacity == Catch::Approx(1.0));

    controller.pickFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    const ogun::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    REQUIRE(overlay.faceColors[static_cast<std::size_t>(ogun::SampleMeshes::kNeedleSliverFaceIndex)]
                .isApprox(Eigen::Vector3d(0.1, 0.2, 1.0)));
    REQUIRE(overlay.overlayFaces.size() == 1);
    REQUIRE(overlay.overlayFaceIndices == std::vector<Eigen::Index>{ogun::SampleMeshes::kNeedleSliverFaceIndex});

    controller.resetOverlayAppearanceToDefaults();
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(1.0));
}

TEST_CASE("SurfaceStyle remembers its edge width across Polyscope's Edges checkbox toggle",
          "[app][controller][appearance]") {
    ogun::SurfaceStyle style;
    style.edgeWidthPx = 1.6;

    // Unchanged width: nothing to do
    REQUIRE_FALSE(style.adoptPolyscopeEdgeWidth(1.6));

    // "Edges" unchecked -> Polyscope writes 0; the width is remembered
    REQUIRE(style.adoptPolyscopeEdgeWidth(0.0));
    REQUIRE_FALSE(style.showEdges);
    REQUIRE(style.effectiveEdgeWidth() == 0.0);
    REQUIRE(style.edgeWidthPx == Catch::Approx(1.6));

    // "Edges" re-checked -> Polyscope writes exactly 1.0; restore 1.6 instead
    REQUIRE(style.adoptPolyscopeEdgeWidth(1.0));
    REQUIRE(style.showEdges);
    REQUIRE(style.effectiveEdgeWidth() == Catch::Approx(1.6));

    // Dragging Polyscope's Width slider is adopted
    REQUIRE(style.adoptPolyscopeEdgeWidth(0.4));
    REQUIRE(style.edgeWidthPx == Catch::Approx(0.4));
}

TEST_CASE("AppController converts screen pixels to world lengths at the selection",
          "[app][controller][highlight][appearance]") {
    // 90 deg FOV at distance 10 spans a 20-unit view height; over 1000 px that is 0.02 units per pixel
    REQUIRE(ogun::AppController::worldLengthPerPixel(10.0, 90.0, 1000.0) == Catch::Approx(0.02));
    REQUIRE(ogun::AppController::worldLengthPerPixel(0.0, 90.0, 1000.0) == 0.0);
    REQUIRE(ogun::AppController::worldLengthPerPixel(10.0, 90.0, 0.0) == 0.0);

    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.pickFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    const Eigen::Vector3d camPos =
        controller.computeFaceCentroid(ogun::SampleMeshes::kNeedleSliverFaceIndex) + Eigen::Vector3d(0.0, 0.0, 10.0);
    REQUIRE(controller.worldLengthAtSelection(1.0, camPos, 90.0, 1000.0) == Catch::Approx(0.02));
    REQUIRE(controller.worldLengthAtSelection(2.5, camPos, 90.0, 1000.0) == Catch::Approx(0.05));
}

TEST_CASE("AppController cuts highlighted faces out of the base mesh display",
          "[app][controller][highlight]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);

    // Selecting the row only tints; selecting its defects cuts them out (the overlay draws them)
    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
    controller.selectAllVisibleDefects();
    const ogun::BaseMeshDisplay all = controller.buildBaseMeshDisplay();
    REQUIRE(all.isCut);
    REQUIRE(all.F.rows() == controller.mesh().numFaces() - 3);
    for (std::size_t row = 0; row < all.faceIndices.size(); ++row) {
        REQUIRE(all.F.row(static_cast<Eigen::Index>(row)) == controller.mesh().F.row(all.faceIndices[row]));
    }

    controller.clearSelection();
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
}

TEST_CASE("AppController preview display modes control the base-mesh cutout and original-edge overlay",
          "[app][controller][repair][preview]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.pickFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(controller.repairPreviewShown());
    REQUIRE(controller.repairPreview().sourceFaceIndices ==
            std::vector<Eigen::Index>{ogun::SampleMeshes::kMicroscopicFaceIndex});

    // Cutout + Original Edges is the default
    REQUIRE(controller.previewDisplayMode() == ogun::PreviewDisplayMode::CutoutOriginalEdges);
    const ogun::HighlightOverlayGeometry originalEdges = controller.buildRepairPreviewOriginalEdges();
    REQUIRE(originalEdges.overlayPoints.size() == 3);
    REQUIRE(originalEdges.overlayEdges.size() == 3);

    // Translucent modes draw no original edges; only the highlighted face is cut from the base
    controller.setPreviewDisplayMode(ogun::PreviewDisplayMode::TranslucentBase);
    REQUIRE(controller.previewBaseOpacity() == Catch::Approx(0.40));
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == controller.mesh().numFaces() - 1);
    controller.setPreviewDisplayMode(ogun::PreviewDisplayMode::TranslucentPatch);
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());

    // Hiding the preview drops the original-edge overlay
    controller.setPreviewDisplayMode(ogun::PreviewDisplayMode::CutoutOriginalEdges);
    controller.setRepairPreviewVisible(false);
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());

    // Opacity sliders are clamped
    controller.setPreviewBaseOpacity(0.0);
    controller.setPreviewPatchOpacity(2.0);
    REQUIRE(controller.previewBaseOpacity() == Catch::Approx(ogun::settings::kMinPreviewOpacity));
    REQUIRE(controller.previewPatchOpacity() == Catch::Approx(1.0));
}

TEST_CASE("AppController cutout preview hides every replaced face of a 2-ring patch",
          "[app][controller][repair][preview]") {
    ogun::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    controller.pickFace(0);

    // A needle in a planar disk: the whole 6-face disk is replaced, leaving no base faces;
    // a disk has V + F - 1 = 11 edges
    const ogun::BaseMeshDisplay display = controller.buildBaseMeshDisplay();
    REQUIRE(display.isCut);
    REQUIRE(display.F.rows() == 0);

    controller.setPreviewDisplayMode(ogun::PreviewDisplayMode::TranslucentBase);
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == controller.mesh().numFaces() - 1);

    controller.setPreviewDisplayMode(ogun::PreviewDisplayMode::CutoutOriginalEdges);
    const ogun::HighlightOverlayGeometry originalEdges = controller.buildRepairPreviewOriginalEdges();
    REQUIRE(originalEdges.overlayPoints.size() == 6);
    REQUIRE(originalEdges.overlayEdges.size() == 11);
}
