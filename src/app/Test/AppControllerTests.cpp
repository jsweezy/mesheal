#include "app/AppController.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "core/Settings.hpp"
#include "mesh/Units.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace {

std::size_t thinCount(const mesheal::AppController& controller) {
    return controller.categoryHealth(mesheal::DefectCategory::ThinFaces).displayCount(mesheal::DefectCategory::ThinFaces);
}

mesheal::HealthStatus thinStatus(const mesheal::AppController& controller) {
    return controller.categoryHealth(mesheal::DefectCategory::ThinFaces).status;
}

} // namespace

TEST_CASE("AppController loads sample mesh, changes File Units without scaling V, and scales V explicitly",
          "[app][controller][units]") {
    mesheal::AppController controller;
    REQUIRE_FALSE(controller.hasMesh());
    REQUIRE(controller.fileUnit() == mesheal::Unit::Millimeter);

    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.hasMesh());
    REQUIRE(controller.mesh().numFaces() == mesheal::SampleMeshes::kDefectiveSampleFaceCount);
    REQUIRE(controller.loadedSourceLabel() == "Sample Defective Mesh");

    const Eigen::MatrixXd originalV = controller.mesh().V;
    REQUIRE(controller.mesh().boundingBox().extents().isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-12));

    // Changing File Units from mm to cm only relabels: V and every setting keep their numbers
    const mesheal::ThinFaceThresholds thresholdsBefore = controller.thresholds();
    const mesheal::ToleranceConfig toleranceBefore = controller.toleranceConfig();
    const double mergeBefore = controller.globalRepairParams().mergeDistance;
    for (const mesheal::Unit unit : mesheal::kAllUnits) {
        controller.setFileUnit(unit);
        REQUIRE(controller.fileUnit() == unit);
        REQUIRE(controller.mesh().V.isApprox(originalV, 0.0));
        REQUIRE(controller.thresholds().minAltitude == thresholdsBefore.minAltitude);
        REQUIRE(controller.thresholds().minArea == thresholdsBefore.minArea);
        REQUIRE(controller.thresholds().maxAspectRatio == thresholdsBefore.maxAspectRatio);
        REQUIRE(controller.toleranceConfig().absFloor == toleranceBefore.absFloor);
        REQUIRE(controller.toleranceConfig().relTol == toleranceBefore.relTol);
        REQUIRE(controller.globalRepairParams().mergeDistance == mergeBefore);
    }
    controller.setFileUnit(mesheal::Unit::Millimeter);

    // Only the explicit scale converts numbers

    controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
    REQUIRE(controller.fileUnit() == mesheal::Unit::Centimeter);
    REQUIRE(controller.mesh().V.isApprox(originalV * 0.1, 1e-14));
    REQUIRE(controller.mesh().boundingBox().extents().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0), 1e-12));
}

TEST_CASE("AppController checks the mesh on load, toggles row vs single-face selection, and builds 3D highlight overlays",
          "[app][controller][diagnostics][selection]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Check Mesh runs after loading: every row is filled in, nothing is selected yet. At the default
    // 100 : 1 aspect limit the well-shaped microscopic face is not thin (the floors are off)
    REQUIRE(thinStatus(controller) == mesheal::HealthStatus::Defects);
    REQUIRE(thinCount(controller) == 2);
    REQUIRE_FALSE(controller.selectedCategory());
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay());

    const std::vector<Eigen::Index> allExpectedDefects = {
        mesheal::SampleMeshes::kZeroAreaFaceIndex,
        mesheal::SampleMeshes::kNeedleSliverFaceIndex
    };

    // Selecting the row tints both faces and draws tubes / spheres for them (under the overlay cap)
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::DefectCategory);
    REQUIRE(controller.browsableResult());
    REQUIRE(controller.browser().visibleRows().size() == 2);
    const mesheal::HighlightOverlayGeometry categoryOverlay = controller.buildHighlightOverlay();
    REQUIRE(categoryOverlay.faceColors.size() == 15);
    REQUIRE(categoryOverlay.hasSelectionOverlay());
    REQUIRE(categoryOverlay.overlayPoints.size() == 6);
    REQUIRE(categoryOverlay.overlayEdges.size() == 6);
    REQUIRE(categoryOverlay.overlayFaces.empty());
    for (const Eigen::Index f : allExpectedDefects) {
        REQUIRE(categoryOverlay.faceColors[static_cast<std::size_t>(f)].isApprox(
            controller.overlayAppearance().categoryTintColor));
    }

    // Picking a defective face focuses it: it is drawn by the selected-face overlay
    REQUIRE(controller.pickFace(mesheal::SampleMeshes::kNeedleSliverFaceIndex) == mesheal::PickOutcome::FocusedDefect);
    REQUIRE(controller.focusedFaceIndex() == mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.highlightedFaceIndices() ==
            std::vector<Eigen::Index>{mesheal::SampleMeshes::kNeedleSliverFaceIndex});

    const mesheal::HighlightOverlayGeometry singleOverlay = controller.buildHighlightOverlay();
    REQUIRE(singleOverlay.overlayFaces.size() == 1);
    REQUIRE(singleOverlay.overlayPoints.size() == 6); // the row's tubes stay (under the overlay cap)
    REQUIRE(singleOverlay.overlayEdges.size() == 6);
    REQUIRE_FALSE(singleOverlay.faceColors[0].isApprox(
        singleOverlay.faceColors[static_cast<std::size_t>(mesheal::SampleMeshes::kNeedleSliverFaceIndex)], 1e-6));

    // A clean face is not in the selected row: only a status message
    REQUIRE(controller.pickFace(0) == mesheal::PickOutcome::NotInSelectedRow);
    REQUIRE(controller.focusedFaceIndex() == mesheal::SampleMeshes::kNeedleSliverFaceIndex);

    // Esc clears the defect selection first, then closes the Tool
    REQUIRE(controller.escape());
    REQUIRE(controller.selectedCategory() == mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE(controller.escape());
    REQUIRE_FALSE(controller.selectedCategory());
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::None);
    REQUIRE_FALSE(controller.escape());

    // With no row selected, clicking a defect opens its row; clicking a clean face just picks it
    REQUIRE(controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex) == mesheal::PickOutcome::SelectedRow);
    REQUIRE(controller.selectedCategory() == mesheal::DefectCategory::ThinFaces);
    controller.setMesh(mesheal::SampleMeshes::createCubeMesh(), "Clean Cube");
    REQUIRE(controller.pickFace(0) == mesheal::PickOutcome::PickedFace);
    REQUIRE(controller.highlightedFaceIndices() == std::vector<Eigen::Index>{0});
    REQUIRE_FALSE(controller.inspectorActive());
    controller.clearSelection();
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay());
}

TEST_CASE("AppController computes 3D camera focus bounding box and look-at target when '.' is triggered",
          "[app][controller][camera]") {
    mesheal::AppController controller;

    // Without a loaded mesh, camera focus is invalid
    REQUIRE_FALSE(controller.computeCameraFocusTarget().valid);

    controller.loadSampleDefectiveMesh();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);

    SECTION("Focusing on a single microscopic defective face centers camera on that facet's centroid") {
        // Only an advanced height floor flags the well-shaped microscopic face (h_min ~7.07e-5)
        controller.setThresholds(mesheal::ThinFaceThresholds{1.0e-4, 0.0, mesheal::settings::kDefaultThinFaceMaxAspectRatio});
        controller.pickFace(mesheal::SampleMeshes::kMicroscopicFaceIndex);

        const Eigen::Vector3d viewDir(0.0, 0.0, -1.0); // Looking straight down -Z
        const mesheal::CameraFocusTarget focus = controller.computeCameraFocusTarget(viewDir);

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
        const mesheal::CameraFocusTarget focus = controller.computeCameraFocusTarget();

        REQUIRE(focus.valid);
        // All 3 faces span X in [0, 10], Y in [0, 5.0002], Z in [0, 10]
        REQUIRE(focus.selectionBounds.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
        REQUIRE(focus.selectionBounds.max.isApprox(Eigen::Vector3d(10.0, 5.0002, 10.0), 1e-12));
        REQUIRE(focus.targetCenter.isApprox(Eigen::Vector3d(5.0, 2.5001, 5.0), 1e-12));
    }

    SECTION("Focusing with no active selection frames the entire mesh bounding box") {
        controller.clearSelection();
        const mesheal::CameraFocusTarget focus = controller.computeCameraFocusTarget();

        REQUIRE(focus.valid);
        REQUIRE(focus.selectionBounds.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
        REQUIRE(focus.selectionBounds.max.isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-12));
        REQUIRE(focus.targetCenter.isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));
    }
}

TEST_CASE("AppController loads STL file from disk and manages ghost mesh settings",
          "[app][controller][io]") {
    mesheal::AppController controller;

    // Write a temporary cube STL to disk and load via AppController::loadStlFile
    const std::string asciiStl = mesheal::SampleMeshes::createCubeAsciiStl(20.0);
    const auto tempDir = std::filesystem::temp_directory_path();
    const auto uniqueSuffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto tempPath = tempDir / ("mesheal_app_controller_test_" + uniqueSuffix + ".stl");
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
    REQUIRE(thinStatus(controller) == mesheal::HealthStatus::Ok);
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.watertightState() == mesheal::WatertightState::Watertight);
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
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);

    // Screen-space defaults: 3 px vertex spheres and 1 px edges on both overlay surfaces
    REQUIRE(controller.overlayAppearance().pointRadiusPx == Catch::Approx(3.0));
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(controller.overlayAppearance().previewPatch.edgeWidthPx == Catch::Approx(1.0));

    // User customizes point radius and defect edge width
    mesheal::OverlayAppearance appearance = controller.overlayAppearance();
    appearance.pointRadiusPx = 6.0;
    appearance.selectedFaces.edgeWidthPx = 2.0;
    controller.setOverlayAppearance(appearance);

    // Switching between defective faces, category selection, and re-running diagnostics must NOT reset styles
    const auto requireCustomStyles = [&controller]() {
        REQUIRE(controller.overlayAppearance().pointRadiusPx == Catch::Approx(6.0));
        REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(2.0));
    };
    controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    requireCustomStyles();
    controller.pickFace(mesheal::SampleMeshes::kMicroscopicFaceIndex);
    requireCustomStyles();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    requireCustomStyles();
    controller.checkMesh();
    requireCustomStyles();

    // Zoom scale (drives the near clip plane) relative to the full-mesh reference distance
    controller.pickFace(mesheal::SampleMeshes::kMicroscopicFaceIndex);
    const mesheal::CameraFocusTarget focus = controller.computeCameraFocusTarget(Eigen::Vector3d(0.0, 0.0, -1.0));
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
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Initially with no selection, orbit pivot is at the full mesh center (5, 5, 5)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));

    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    // Selecting the thin-face row sets orbit pivot to the defects' bounds center (5.0, 2.5001, 5.0)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 2.5001, 5.0), 1e-12));

    // Selecting Face #14 (kNeedleSliverFaceIndex: vertices (0,5,10), (10,5,10), (5,5.0002,10))
    // immediately sets orbit pivot to its true centroid (5.0, 5.0 + 0.0002/3.0, 10.0)
    controller.pickFace(mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    const Eigen::Vector3d expectedNeedleCentroid(5.0, 5.0 + 0.0002 / 3.0, 10.0);
    REQUIRE(controller.computeFaceCentroid(mesheal::SampleMeshes::kNeedleSliverFaceIndex)
                .isApprox(expectedNeedleCentroid, 1e-12));
    REQUIRE(controller.orbitPivot().isApprox(expectedNeedleCentroid, 1e-12));

    // Face #14 lies on z=10 with CCW vertices in XY, so its outward normal is +Z (0, 0, 1)
    const Eigen::Vector3d needleNormal =
        controller.computeFaceInspectionNormal(mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(needleNormal.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-12));

    // 1st '.' press (AutoToggleNormal): frames from +Normal side (camera above +Z looking along -Z)
    const mesheal::CameraFocusTarget focusPos = controller.focusOnSelection(
        mesheal::FocusNormalSide::AutoToggleNormal);
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
    const mesheal::CameraFocusTarget focusNeg = controller.focusOnSelection(
        mesheal::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusNeg.valid);
    REQUIRE(focusNeg.normalSideSign == -1);
    REQUIRE(controller.lastFocusedNormalSign() == -1);
    REQUIRE(focusNeg.cameraPosition.isApprox(
        expectedNeedleCentroid - Eigen::Vector3d(0.0, 0.0, focusNeg.focusRadius), 1e-12));

    // 3rd '.' press: toggles back to +Normal side
    const mesheal::CameraFocusTarget focusPosAgain = controller.focusOnSelection(
        mesheal::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusPosAgain.normalSideSign == 1);

    // If the user orbits the camera manually, notifyCameraRotated() resets the toggle so next '.' is +Normal
    controller.notifyCameraRotated();
    REQUIRE(controller.lastFocusedNormalSign() == 0);
    const mesheal::CameraFocusTarget focusAfterOrbit = controller.focusOnSelection(
        mesheal::FocusNormalSide::AutoToggleNormal);
    REQUIRE(focusAfterOrbit.normalSideSign == 1);

    // 'Shift + .' (FocusNormalSide::NegativeNormal) always frames -Normal directly
    const mesheal::CameraFocusTarget directNeg = controller.focusOnSelection(
        mesheal::FocusNormalSide::NegativeNormal);
    REQUIRE(directNeg.normalSideSign == -1);
    REQUIRE(controller.lastFocusedNormalSign() == -1);

    // Panning translates the orbit pivot in world space, and pressing '.' snaps it back to the face centroid
    controller.translateOrbitPivot(Eigen::Vector3d(1.5, -0.5, 0.25));
    REQUIRE(controller.orbitPivot().isApprox(
        expectedNeedleCentroid + Eigen::Vector3d(1.5, -0.5, 0.25), 1e-12));
    controller.focusOnSelection(mesheal::FocusNormalSide::PositiveNormal);
    REQUIRE(controller.orbitPivot().isApprox(expectedNeedleCentroid, 1e-12));

    // Collinear zero-area face (Face #12 along X-axis (0,0,0)->(5,0,0)->(10,0,0)) gets a valid perpendicular normal
    controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    const Eigen::Vector3d zeroAreaNormal =
        controller.computeFaceInspectionNormal(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(zeroAreaNormal.norm() == Catch::Approx(1.0).epsilon(1e-12));
    // Must be perpendicular to the collinear edge direction (1, 0, 0)
    REQUIRE(std::abs(zeroAreaNormal.dot(Eigen::Vector3d(1.0, 0.0, 0.0))) < 1e-12);

    // Home reset returns orbit pivot to full mesh center
    controller.resetOrbitPivotToMeshCenter();
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));
    REQUIRE(controller.lastFocusedNormalSign() == 0);
}

TEST_CASE("AppController resolves the Mesheal application toolbar icon asset",
          "[app][controller][icon]") {
    const std::filesystem::path iconPath = mesheal::AppController::resolveApplicationIconPath();
    REQUIRE_FALSE(iconPath.empty());
    REQUIRE(std::filesystem::exists(iconPath));
    REQUIRE((iconPath.filename() == "mesheal.png" || iconPath.filename() == "mesheal.jpeg"));
}

namespace {

std::filesystem::path makeTempStlPath(const std::string& tag) {
    const auto uniqueSuffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return std::filesystem::temp_directory_path() / ("mesheal_app_" + tag + "_" + uniqueSuffix + ".stl");
}

// Planar disk with needle (p, q, r) (face 0) whose 1e-4 short edge p-q is shared with needle (q, p, w).
mesheal::Mesh makeNeedleDiskMesh() {
    mesheal::Mesh mesh;
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
    mesheal::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    const mesheal::Mesh before = controller.mesh();

    REQUIRE(controller.pickFace(0) == mesheal::PickOutcome::SelectedRow);
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.selectedFaceAnalysis().faceIndex == 0);
    REQUIRE(controller.selectedFaceAnalysis().type == mesheal::SliverType::Needle);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::CollapseShortestEdge);

    const mesheal::SingleFaceRepairPreview& preview = controller.repairPreview();
    REQUIRE(preview.valid);
    REQUIRE(preview.appliedAction == mesheal::HealAction::CollapseShortestEdge);
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
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::None);
}

TEST_CASE("AppController switching HealAction re-evaluates the preview and flags unsafe strategies",
          "[app][controller][repair][preview]") {
    mesheal::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    controller.pickFace(0);
    REQUIRE(controller.repairPreview().valid);

    // Removing an interior needle would open a hole
    controller.setSelectedHealAction(mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().appliedAction == mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE_FALSE(controller.repairPreview().valid);
    REQUIRE_FALSE(controller.repairPreview().statusMessage.empty());
    REQUIRE(controller.repairPreview().previewPatchMesh.empty());

    controller.setSelectedHealAction(mesheal::HealAction::CollapseShortestEdge);
    REQUIRE(controller.repairPreview().valid);

    // Re-selecting the same face keeps the user's chosen strategy
    controller.setSelectedHealAction(mesheal::HealAction::RemoveDegenerateFace);
    controller.pickFace(0);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::RemoveDegenerateFace);

    // Without a single-face selection, choosing a strategy is ignored
    controller.clearSelection();
    controller.setSelectedHealAction(mesheal::HealAction::FlipLongestEdge);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::None);
}

TEST_CASE("R commits the focused thin face's repair", "[app][controller][repair]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.commitFocusedRepair()); // no defect focused
    REQUIRE(controller.mesh().numFaces() == 15);
    controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.commitFocusedRepair());
    REQUIRE(controller.mesh().numFaces() == 14);
    REQUIRE(controller.lastUndoActionLabel().rfind("Single-Face Repair", 0) == 0);
}

TEST_CASE("AppController commits a single-face repair, refreshes diagnostics, and undoes it",
          "[app][controller][repair][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(thinCount(controller) == 2);
    REQUIRE_FALSE(controller.isModified());

    controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().valid);

    REQUIRE(controller.commitSelectedFaceRepair());
    REQUIRE(controller.mesh().numFaces() == before.numFaces() - 1);
    REQUIRE(controller.mesh().F.maxCoeff() < controller.mesh().numVertices());
    REQUIRE(thinCount(controller) == 1);
    REQUIRE(controller.isModified());
    REQUIRE(controller.selectedCategory() == mesheal::DefectCategory::ThinFaces);
    REQUIRE_FALSE(controller.inspectorActive());
    REQUIRE(controller.loadedSourceLabel() == "Sample Defective Mesh");
    REQUIRE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel() == "Single-Face Repair (Face #12)");

    REQUIRE(controller.undoLastModification());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel().empty());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(thinCount(controller) == 2);
    REQUIRE_FALSE(controller.isModified());
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.focusedFaceIndex() == mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE(controller.repairPreview().valid);

    // Nothing left to undo
    REQUIRE_FALSE(controller.undoLastModification());
}

TEST_CASE("AppController rejects an unsafe single-face commit without recording undo",
          "[app][controller][repair][undo]") {
    mesheal::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    const mesheal::Mesh before = controller.mesh();

    // No single-face selection -> nothing to commit
    REQUIRE_FALSE(controller.commitSelectedFaceRepair());

    controller.pickFace(0);
    controller.setSelectedHealAction(mesheal::HealAction::RemoveDegenerateFace);
    REQUIRE_FALSE(controller.commitSelectedFaceRepair());
    REQUIRE(controller.lastRepairStatusMessage().find("Repair blocked:") != std::string::npos);
    REQUIRE(controller.lastRepairStatusMessage().find("No changes were made.") != std::string::npos);
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("AppController fixAllThinFaces heals the sample mesh and can be undone",
          "[app][controller][repair][batch][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(thinCount(controller) == 2);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(thinStatus(controller) == mesheal::HealthStatus::Ok);
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
    REQUIRE(thinCount(controller) == 2);
    REQUIRE(controller.selectedCategory() == mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.browser().visibleRows().size() == 2);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("Repair All Thin / Sliver Faces previews the heal with its numbers before applying",
          "[app][controller][repair][global][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(thinCount(controller) == 2);

    // Opening the operator computes the preview, but leaves the mesh alone
    controller.openRepairAllThinFaces();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::RepairAllThinFaces);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairTarget() == mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.globalRepairMethods().size() == 1);
    const mesheal::RepairCandidate* shown = controller.shownGlobalCandidate();
    REQUIRE(shown != nullptr);
    REQUIRE(shown->success);
    REQUIRE(shown->method == mesheal::GlobalRepairMethod::HealThinFaces);
    REQUIRE(shown->metrics.thinFacesBefore == 2);
    REQUIRE(shown->metrics.thinFacesAfter == 0);
    REQUIRE(shown->metrics.facesAfter < shown->metrics.facesBefore);
    REQUIRE(shown->metrics.volumeBefore != 0.0);
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.mesh().F == before.F);

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Repair All Thin / Sliver Faces");
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("Whole-mesh preview draws the original ghosted or as a wireframe in Both",
          "[app][controller][repair][global][preview]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.openRepairAllThinFaces();
    const mesheal::RepairCandidate* shown = controller.shownGlobalCandidate();
    REQUIRE(shown != nullptr);
    REQUIRE_FALSE(shown->removedFaces.empty());
    REQUIRE(controller.wholeMeshPreviewShown());

    // Result: no original drawn at all, in either style
    controller.setGlobalPreviewShow(mesheal::GlobalPreviewShow::Result);
    mesheal::WholeMeshPreviewStyle style = controller.wholeMeshPreviewStyle();
    style.original = mesheal::WholeMeshOriginalStyle::Wireframe;
    controller.setWholeMeshPreviewStyle(style);
    REQUIRE_FALSE(controller.wholeMeshOriginalWireframeShown());
    REQUIRE_FALSE(controller.buildWholeMeshOriginalWireframe().hasSelectionOverlay());
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == 0);

    // Both + Wireframe: the removed faces become edges and leave the base surface
    controller.setGlobalPreviewShow(mesheal::GlobalPreviewShow::Both);
    REQUIRE(controller.wholeMeshOriginalWireframeShown());
    REQUIRE(controller.buildWholeMeshOriginalWireframe().hasSelectionOverlay());
    REQUIRE_FALSE(controller.buildWholeMeshOriginalWireframe().overlayEdges.empty());
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == 0);

    // Both + Ghosted: the removed faces are drawn as a surface, no wireframe
    style.original = mesheal::WholeMeshOriginalStyle::Ghosted;
    controller.setWholeMeshPreviewStyle(style);
    REQUIRE_FALSE(controller.buildWholeMeshOriginalWireframe().hasSelectionOverlay());
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == static_cast<Eigen::Index>(shown->removedFaces.size()));

    // Opacities are clamped
    style.resultOpacity = 2.0;
    style.originalOpacity = 0.0;
    controller.setWholeMeshPreviewStyle(style);
    REQUIRE(controller.wholeMeshPreviewStyle().resultOpacity == Catch::Approx(1.0));
    REQUIRE(controller.wholeMeshPreviewStyle().originalOpacity == Catch::Approx(mesheal::settings::kMinPreviewOpacity));
}

TEST_CASE("Whole-mesh preview focus frames the clicked triangle without leaving the preview",
          "[app][controller][repair][global][preview][focus]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.wholeMeshPreviewOpen());
    controller.openRepairAllThinFaces();
    REQUIRE(controller.wholeMeshPreviewOpen());
    const mesheal::ToolSlot tool = controller.activeTool();
    const auto category = controller.selectedCategory();
    const std::size_t selectedBefore = controller.selectedFaceCount();

    // A triangle in the z = 1 plane, normal +Z
    const Eigen::Vector3d a(0.0, 0.0, 1.0);
    const Eigen::Vector3d b(1.0, 0.0, 1.0);
    const Eigen::Vector3d c(0.0, 1.0, 1.0);
    controller.focusWholeMeshPreviewTriangle(a, b, c);
    const Eigen::Vector3d centroid = (a + b + c) / 3.0;
    REQUIRE(controller.wholeMeshPreviewFocus().has_value());
    REQUIRE(controller.orbitPivot().isApprox(centroid));

    // '.' looks at it from +Z first, then from -Z
    const Eigen::Vector3d up(0.0, 1.0, 0.0);
    const mesheal::CameraFocusTarget front =
        controller.focusOnSelection(mesheal::FocusNormalSide::AutoToggleNormal, Eigen::Vector3d(1.0, 0.0, 0.0), up);
    REQUIRE(front.valid);
    REQUIRE(front.targetCenter.isApprox(centroid));
    REQUIRE(front.cameraPosition.z() > centroid.z());
    const mesheal::CameraFocusTarget back =
        controller.focusOnSelection(mesheal::FocusNormalSide::AutoToggleNormal, Eigen::Vector3d(1.0, 0.0, 0.0), up);
    REQUIRE(back.cameraPosition.z() < centroid.z());
    const mesheal::CameraFocusTarget minus =
        controller.focusOnSelection(mesheal::FocusNormalSide::NegativeNormal, Eigen::Vector3d(1.0, 0.0, 0.0), up);
    REQUIRE(minus.cameraPosition.z() < centroid.z());

    // The preview, its tool and the selection are untouched
    REQUIRE(controller.activeTool() == tool);
    REQUIRE(controller.selectedCategory() == category);
    REQUIRE(controller.selectedFaceCount() == selectedBefore);
    REQUIRE(controller.shownGlobalCandidate() != nullptr);
}

TEST_CASE("Mesh stats keep the loaded mesh to show the change of every edit", "[app][controller][undo]") {
    mesheal::AppController controller;
    REQUIRE_FALSE(controller.originalMeshStats().has_value());
    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.originalMeshStats().has_value());
    const mesheal::MeshStats loaded = *controller.originalMeshStats();
    REQUIRE(loaded.faces == controller.mesh().numFaces());
    REQUIRE(loaded.vertices == controller.mesh().numVertices());
    REQUIRE(loaded.volume == controller.meshStats().volume);
    REQUIRE(loaded.area > 0.0);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.meshStats().faces < loaded.faces);
    REQUIRE(controller.originalMeshStats()->faces == loaded.faces); // the baseline survives edits

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.meshStats().faces == loaded.faces);
    REQUIRE(controller.meshStats().volume == loaded.volume);

    // Loading again sets a new baseline
    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.originalMeshStats()->faces == controller.meshStats().faces);
}

TEST_CASE("AppController fixAllThinFaces on isolated Turkey shell 27 heals all slivers in one pass",
          "[app][controller][repair][batch][turkey]") {
    mesheal::AppController controller;
    const std::filesystem::path turkeyPath =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl";
    controller.setMesh(mesheal::loadStl(turkeyPath), "Turkey.stl");
    REQUIRE(controller.shells().size() == 28);

    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    REQUIRE(thinCount(controller) == 78);

    std::size_t shellWith50 = 28;
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        controller.selectShellsWhere([s](std::size_t x) { return x == s; });
        if (controller.rowCount(mesheal::DefectCategory::ThinFaces) == 50) {
            shellWith50 = s;
            break;
        }
    }
    REQUIRE(shellWith50 < 28);

    controller.selectShellsWhere([shellWith50](std::size_t s) { return s == shellWith50; });
    REQUIRE(controller.toggleIsolateShells());
    REQUIRE(controller.shells().scoped());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 50);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 0);
    REQUIRE(thinCount(controller) == 28);
    REQUIRE(controller.rowStatus(mesheal::DefectCategory::ThinFaces) == mesheal::HealthStatus::Ok);
    REQUIRE(controller.canUndo());

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 50);
    REQUIRE(thinCount(controller) == 78);
}

TEST_CASE("AppController fixAllThinFaces heals 8 needles on Turkey_left_with_needles",
          "[app][controller][repair][batch][needles]") {
    const std::filesystem::path needleStl =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "build" / "examples" / "Turkey_left_with_needles.stl";
    if (!std::filesystem::exists(needleStl)) {
        return;
    }
    mesheal::AppController controller;
    controller.setMesh(mesheal::loadStl(needleStl), "Turkey_left_with_needles.stl");
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 8);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 0);
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.canUndo());

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 8);
    REQUIRE(thinCount(controller) == 8);
}

TEST_CASE("AppController fixAllThinFaces heals single cap on turkey_single_cap",
          "[app][controller][repair][batch][cap]") {
    const std::filesystem::path capStl =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "build" / "examples" / "turkey_single_cap.stl";
    if (!std::filesystem::exists(capStl)) {
        return;
    }
    mesheal::AppController controller;
    controller.setMesh(mesheal::loadStl(capStl), "turkey_single_cap.stl");
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 1);

    controller.focusDefect(0);
    REQUIRE(controller.inspectorActive());
    REQUIRE(controller.selectedHealAction() == mesheal::HealAction::FlipLongestEdge);
    REQUIRE(controller.repairPreview().valid);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 0);
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.canUndo());

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 1);
    REQUIRE(thinCount(controller) == 1);
}

TEST_CASE("AppController records coordinate scaling on the undo stack",
          "[app][controller][units][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const Eigen::MatrixXd originalV = controller.mesh().V;
    const mesheal::ThinFaceThresholds originalThresholds = controller.thresholds();

    controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
    REQUIRE(controller.mesh().V.isApprox(originalV * 0.1, 1e-14));
    REQUIRE(controller.canUndo());
    REQUIRE(controller.lastUndoActionLabel() == "Scale Mesh Coordinates (mm -> cm)");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.fileUnit() == mesheal::Unit::Millimeter);
    REQUIRE(controller.mesh().V == originalV);
    REQUIRE(controller.thresholds().minAltitude == originalThresholds.minAltitude);
    REQUIRE(controller.thresholds().minArea == originalThresholds.minArea);
    REQUIRE(thinCount(controller) == 2);
}

TEST_CASE("Scale Mesh Coordinates rescales the thin-face floors when they are set", "[app][controller][units]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Off by default: the floors stay 0 and the aspect limit never changes
    controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
    CHECK(controller.thresholds().minAltitude == 0.0);
    CHECK(controller.thresholds().minArea == 0.0);
    CHECK(controller.thresholds().maxAspectRatio == mesheal::settings::kDefaultThinFaceMaxAspectRatio);
    CHECK(thinCount(controller) == 2);

    // Set floors keep their physical size, so the same faces stay flagged
    mesheal::AppController withFloors;
    withFloors.loadSampleDefectiveMesh();
    withFloors.setThresholds(mesheal::ThinFaceThresholds{1.0e-4, 1.0e-8, mesheal::settings::kDefaultThinFaceMaxAspectRatio});
    REQUIRE(thinCount(withFloors) == 3); // the microscopic face is below the height floor
    withFloors.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
    CHECK(withFloors.thresholds().minAltitude == Catch::Approx(1.0e-5));
    CHECK(withFloors.thresholds().minArea == Catch::Approx(1.0e-10));
    CHECK(withFloors.thresholds().maxAspectRatio == mesheal::settings::kDefaultThinFaceMaxAspectRatio);
    CHECK(thinCount(withFloors) == 3);
}

TEST_CASE("AppController undoes a sequence of modifications step by step and clears history on load",
          "[app][controller][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const mesheal::Mesh original = controller.mesh();

    controller.pickFace(mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.commitSelectedFaceRepair());
    const mesheal::Mesh afterSingle = controller.mesh();
    REQUIRE(thinCount(controller) == 1);

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Thin / Sliver Faces");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == afterSingle.F);
    REQUIRE(thinCount(controller) == 1);
    REQUIRE(controller.lastUndoActionLabel() == "Single-Face Repair (Face #12)");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == original.F);
    REQUIRE(thinCount(controller) == 2);
    REQUIRE(controller.focusedFaceIndex() == mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE_FALSE(controller.canUndo());

    // Loading a new mesh discards the undo history
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.canUndo());
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController saves the repaired mesh as Binary and ASCII STL that reload clean",
          "[app][controller][io][save]") {
    mesheal::AppController controller;

    // Saving without a mesh fails with an error message
    REQUIRE_FALSE(controller.saveStlFile(makeTempStlPath("empty")));
    REQUIRE_FALSE(controller.lastErrorMessage().empty());

    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.isModified());
    const mesheal::Mesh repaired = controller.mesh();

    for (const mesheal::StlFormat format : {mesheal::StlFormat::Binary, mesheal::StlFormat::Ascii}) {
        const auto path = makeTempStlPath(format == mesheal::StlFormat::Binary ? "binary" : "ascii");
        REQUIRE(controller.saveStlFile(path, format));
        REQUIRE(controller.lastErrorMessage().empty());
        REQUIRE_FALSE(controller.isModified());
        REQUIRE(controller.currentFilePath() == path);

        mesheal::AppController reloaded;
        const bool loaded = reloaded.loadStlFile(path);
        std::filesystem::remove(path);
        REQUIRE(loaded);
        REQUIRE(reloaded.mesh().numFaces() == repaired.numFaces());
        REQUIRE(reloaded.mesh().numVertices() == repaired.numVertices());
        REQUIRE(thinCount(reloaded) == 0);
    }

    REQUIRE_FALSE(controller.saveStlFile(
        std::filesystem::path("/nonexistent_mesheal_dir_xyz") / "out.stl"));
    REQUIRE_FALSE(controller.lastErrorMessage().empty());
}

TEST_CASE("AppController overlay styles default to 1 px edges and drive the selected face color",
          "[app][controller][highlight][appearance]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();

    const mesheal::OverlayAppearance& defaults = controller.overlayAppearance();
    REQUIRE(defaults.selectedFaces.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(defaults.previewPatch.edgeWidthPx == Catch::Approx(1.0));
    REQUIRE(defaults.previewPatch.color.isApprox(Eigen::Vector3d(0.20, 0.85, 0.30)));
    // Pre-repair edges are thinner than the patch edges and semi-transparent so surviving edges show through
    REQUIRE(defaults.originalEdgeWidthPx == Catch::Approx(0.5));
    REQUIRE(defaults.originalEdgeWidthPx < defaults.previewPatch.edgeWidthPx);
    REQUIRE(defaults.originalEdgeOpacity == Catch::Approx(0.5));

    mesheal::OverlayAppearance custom = defaults;
    custom.selectedFaces.color = Eigen::Vector3d(0.1, 0.2, 1.5); // out-of-range channel is clamped
    custom.selectedFaces.edgeWidthPx = 100.0;                   // clamped to the px maximum
    custom.pointRadiusPx = 0.0;                                 // clamped to the px minimum
    custom.originalEdgeWidthPx = 0.0;                           // clamped to the px minimum
    custom.originalEdgeOpacity = 3.0;                           // clamped to fully opaque
    controller.setOverlayAppearance(custom);
    REQUIRE(controller.overlayAppearance().selectedFaces.color.isApprox(Eigen::Vector3d(0.1, 0.2, 1.0)));
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx ==
            Catch::Approx(mesheal::settings::kMaxOverlayEdgeWidthPx));
    REQUIRE(controller.overlayAppearance().pointRadiusPx ==
            Catch::Approx(mesheal::settings::kMinHighlightPointRadiusPx));
    REQUIRE(controller.overlayAppearance().originalEdgeWidthPx ==
            Catch::Approx(mesheal::settings::kMinOverlayEdgeWidthPx));
    REQUIRE(controller.overlayAppearance().originalEdgeOpacity == Catch::Approx(1.0));

    controller.pickFace(mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    const mesheal::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    REQUIRE(overlay.faceColors[static_cast<std::size_t>(mesheal::SampleMeshes::kNeedleSliverFaceIndex)]
                .isApprox(Eigen::Vector3d(0.1, 0.2, 1.0)));
    REQUIRE(overlay.overlayFaces.size() == 1);
    REQUIRE(overlay.overlayFaceIndices == std::vector<Eigen::Index>{mesheal::SampleMeshes::kNeedleSliverFaceIndex});

    controller.resetOverlayAppearanceToDefaults();
    REQUIRE(controller.overlayAppearance().selectedFaces.edgeWidthPx == Catch::Approx(1.0));
}

TEST_CASE("Display panel base-mesh styles are kept: back faces, color, edges, shading, tint", "[app][overlay]") {
    mesheal::AppController controller;
    mesheal::OverlayAppearance custom = controller.overlayAppearance();
    REQUIRE(custom.backFaceMode == mesheal::BackFaceMode::Darker);
    for (const mesheal::BackFaceMode mode : {mesheal::BackFaceMode::Identical, mesheal::BackFaceMode::Custom, mesheal::BackFaceMode::Cull,
                                          mesheal::BackFaceMode::Darker}) {
        custom.backFaceMode = mode;
        controller.setOverlayAppearance(custom);
        REQUIRE(controller.overlayAppearance().backFaceMode == mode);
    }
    custom.backFaceMode = mesheal::BackFaceMode::Custom;
    custom.backFaceColor = Eigen::Vector3d(1.0, 0.0, 2.0); // clamped
    custom.baseMesh.color = Eigen::Vector3d(0.2, 0.3, 0.4);
    custom.baseMesh.showEdges = false;
    custom.smoothShade = true;
    custom.categoryTintColor = Eigen::Vector3d(0.9, 0.1, 0.1);
    controller.setOverlayAppearance(custom);
    const mesheal::OverlayAppearance& kept = controller.overlayAppearance();
    REQUIRE(kept.backFaceMode == mesheal::BackFaceMode::Custom);
    REQUIRE(kept.backFaceColor.isApprox(Eigen::Vector3d(1.0, 0.0, 1.0)));
    REQUIRE(kept.baseMesh.color.isApprox(custom.baseMesh.color));
    REQUIRE_FALSE(kept.baseMesh.showEdges);
    REQUIRE(kept.smoothShade);
    REQUIRE(kept.categoryTintColor.isApprox(custom.categoryTintColor));

    controller.resetOverlayAppearanceToDefaults();
    REQUIRE(controller.overlayAppearance().backFaceMode == mesheal::BackFaceMode::Darker);
}

TEST_CASE("SurfaceStyle remembers its edge width across Polyscope's Edges checkbox toggle",
          "[app][controller][appearance]") {
    mesheal::SurfaceStyle style;
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
    REQUIRE(mesheal::AppController::worldLengthPerPixel(10.0, 90.0, 1000.0) == Catch::Approx(0.02));
    REQUIRE(mesheal::AppController::worldLengthPerPixel(0.0, 90.0, 1000.0) == 0.0);
    REQUIRE(mesheal::AppController::worldLengthPerPixel(10.0, 90.0, 0.0) == 0.0);

    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.pickFace(mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    const Eigen::Vector3d camPos =
        controller.computeFaceCentroid(mesheal::SampleMeshes::kNeedleSliverFaceIndex) + Eigen::Vector3d(0.0, 0.0, 10.0);
    REQUIRE(controller.worldLengthAtSelection(1.0, camPos, 90.0, 1000.0) == Catch::Approx(0.02));
    REQUIRE(controller.worldLengthAtSelection(2.5, camPos, 90.0, 1000.0) == Catch::Approx(0.05));
}

TEST_CASE("AppController cuts highlighted faces out of the base mesh display",
          "[app][controller][highlight]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);

    // Selecting the row only tints; selecting its defects cuts them out (the overlay draws them)
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
    controller.selectAllVisibleDefects();
    const mesheal::BaseMeshDisplay all = controller.buildBaseMeshDisplay();
    REQUIRE(all.isCut);
    REQUIRE(all.F.rows() == controller.mesh().numFaces() - 2);
    for (std::size_t row = 0; row < all.faceIndices.size(); ++row) {
        REQUIRE(all.F.row(static_cast<Eigen::Index>(row)) == controller.mesh().F.row(all.faceIndices[row]));
    }

    // Face-selected defect faces go back into the base mesh, which shows them in the selection color
    controller.selectAllFaces();
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == controller.mesh().numFaces());

    controller.clearSelection();
    controller.clearFaceSelection();
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
}

TEST_CASE("AppController preview display modes control the base-mesh cutout and original-edge overlay",
          "[app][controller][repair][preview]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.pickFace(mesheal::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.repairPreviewShown());
    REQUIRE(controller.repairPreview().sourceFaceIndices ==
            std::vector<Eigen::Index>{mesheal::SampleMeshes::kNeedleSliverFaceIndex});

    // Cutout + Original Edges is the default
    REQUIRE(controller.previewDisplayMode() == mesheal::PreviewDisplayMode::CutoutOriginalEdges);
    const mesheal::HighlightOverlayGeometry originalEdges = controller.buildRepairPreviewOriginalEdges();
    REQUIRE(originalEdges.overlayPoints.size() == 3);
    REQUIRE(originalEdges.overlayEdges.size() == 3);

    // Translucent modes draw no original edges; only the highlighted face is cut from the base
    controller.setPreviewDisplayMode(mesheal::PreviewDisplayMode::TranslucentBase);
    REQUIRE(controller.previewBaseOpacity() == Catch::Approx(0.40));
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == controller.mesh().numFaces() - 1);
    controller.setPreviewDisplayMode(mesheal::PreviewDisplayMode::TranslucentPatch);
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());

    // Hiding the preview drops the original-edge overlay
    controller.setPreviewDisplayMode(mesheal::PreviewDisplayMode::CutoutOriginalEdges);
    controller.setRepairPreviewVisible(false);
    REQUIRE_FALSE(controller.buildRepairPreviewOriginalEdges().hasSelectionOverlay());

    // Opacity sliders are clamped
    controller.setPreviewBaseOpacity(0.0);
    controller.setPreviewPatchOpacity(2.0);
    REQUIRE(controller.previewBaseOpacity() == Catch::Approx(mesheal::settings::kMinPreviewOpacity));
    REQUIRE(controller.previewPatchOpacity() == Catch::Approx(1.0));
}

TEST_CASE("AppController cutout preview hides every replaced face of a 2-ring patch",
          "[app][controller][repair][preview]") {
    mesheal::AppController controller;
    controller.setMesh(makeNeedleDiskMesh(), "Needle Disk");
    controller.pickFace(0);

    // A needle in a planar disk: the whole 6-face disk is replaced, leaving no base faces;
    // a disk has V + F - 1 = 11 edges
    const mesheal::BaseMeshDisplay display = controller.buildBaseMeshDisplay();
    REQUIRE(display.isCut);
    REQUIRE(display.F.rows() == 0);

    controller.setPreviewDisplayMode(mesheal::PreviewDisplayMode::TranslucentBase);
    REQUIRE(controller.buildBaseMeshDisplay().F.rows() == controller.mesh().numFaces() - 1);

    controller.setPreviewDisplayMode(mesheal::PreviewDisplayMode::CutoutOriginalEdges);
    const mesheal::HighlightOverlayGeometry originalEdges = controller.buildRepairPreviewOriginalEdges();
    REQUIRE(originalEdges.overlayPoints.size() == 6);
    REQUIRE(originalEdges.overlayEdges.size() == 11);
}

namespace {

/// Appends `part` shifted by `offset` to `mesh` (a new shell).
void appendShell(mesheal::Mesh& mesh, mesheal::Mesh part, const Eigen::Vector3d& offset) {
    part.V.rowwise() += offset.transpose();
    const Eigen::Index nv = mesh.V.rows();
    const Eigen::Index nf = mesh.F.rows();
    mesh.V.conservativeResize(nv + part.V.rows(), 3);
    mesh.F.conservativeResize(nf + part.F.rows(), 3);
    mesh.V.bottomRows(part.V.rows()) = part.V;
    mesh.F.bottomRows(part.F.rows()) = part.F.array() + static_cast<int>(nv);
    mesh.recomputeNormals();
}

/// Shells: the defective sample (a cube and two loose triangles, 3 thin faces, inside 0..10), a clean
/// cube crossing it (5..15) and a clean cube far away. Loads them into `controller` and returns the
/// sample cube's, the crossing cube's and the far cube's shell numbers.
void loadThreeShells(mesheal::AppController& controller, std::size_t& sample, std::size_t& crossing, std::size_t& farShell) {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createSampleDefectiveMesh();
    appendShell(mesh, mesheal::SampleMeshes::createCubeMesh(10.0, mesheal::Unit::Millimeter), {5.0, 5.0, 5.0});
    appendShell(mesh, mesheal::SampleMeshes::createCubeMesh(2.0, mesheal::Unit::Millimeter), {40.0, 0.0, 0.0});
    controller.setMesh(std::move(mesh), "three_shells.stl");
    const mesheal::ShellBrowser& shells = controller.shells();
    for (std::size_t s = 0; s < shells.size(); ++s) {
        const mesheal::ShellInfo& shell = shells.shell(s);
        if (shell.bbox.min.x() > 30.0) {
            farShell = s;
        } else if (shell.bbox.min.x() == 5.0) {
            crossing = s;
        } else if (shell.faces.size() == 13) {
            sample = s;
        }
    }
}

/// The three shells of the defective sample.
bool isSample(const mesheal::AppController& controller, std::size_t shell) {
    return controller.shells().shell(shell).bbox.max.x() <= 10.0;
}

std::size_t count(const mesheal::AppController& controller, mesheal::DefectCategory category) {
    return controller.rowCount(category);
}

} // namespace

TEST_CASE("AppController scopes Mesh Health to the selected shells, else the visible ones",
          "[app][controller][shells]") {
    std::size_t sample = 0;
    std::size_t crossing = 0;
    std::size_t farShell = 0;
    mesheal::AppController controller;
    loadThreeShells(controller, sample, crossing, farShell);
    REQUIRE(controller.shells().size() == 5);
    using mesheal::DefectCategory;
    const std::size_t allCrossings = count(controller, DefectCategory::Intersections);
    REQUIRE(allCrossings > 0);
    REQUIRE(count(controller, DefectCategory::ThinFaces) == 2);
    REQUIRE(controller.shellHealthKnown());
    std::size_t sampleThin = 0;
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        sampleThin += controller.shellHealth()[s].counts[mesheal::categoryIndex(DefectCategory::ThinFaces)];
    }
    REQUIRE(sampleThin == 2);
    REQUIRE(controller.shellHealth()[sample].counts[mesheal::categoryIndex(DefectCategory::ThinFaces)] == 1);
    REQUIRE(controller.shellHealth()[crossing].between > 0);

    // One clean shell: nothing in scope, the rows read OK
    controller.clickShell(farShell, false, false, {});
    REQUIRE(count(controller, DefectCategory::ThinFaces) == 0);
    REQUIRE(count(controller, DefectCategory::Intersections) == 0);
    REQUIRE(controller.rowStatus(DefectCategory::ThinFaces) == mesheal::HealthStatus::Ok);
    REQUIRE(controller.totalProblemCount() == 0);
    controller.selectCategory(DefectCategory::ThinFaces);
    REQUIRE(controller.browser().visibleRows().empty());

    // The sample alone: its thin faces, but not its crossings with the unselected cube
    controller.selectShellsWhere([&](std::size_t s) { return isSample(controller, s); });
    REQUIRE(count(controller, DefectCategory::ThinFaces) == 2);
    REQUIRE(controller.browser().visibleRows().size() == 2);
    const std::size_t sampleOnly = count(controller, DefectCategory::Intersections);
    REQUIRE(sampleOnly < allCrossings);

    // Both crossing shells selected: every crossing is back
    controller.clickShell(crossing, true, false, {});
    REQUIRE(count(controller, DefectCategory::Intersections) == allCrossings);

    // No selection: the visible shells are the scope
    controller.selectShellsWhere([](std::size_t) { return false; });
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        controller.setShellVisible(s, !isSample(controller, s));
    }
    REQUIRE(count(controller, DefectCategory::ThinFaces) == 0);
    REQUIRE(count(controller, DefectCategory::Intersections) == 0);
    controller.showAllShells();
    REQUIRE(count(controller, DefectCategory::Intersections) == allCrossings);
}

TEST_CASE("AppController hides shells from the base mesh display and Esc clears the shell selection",
          "[app][controller][shells]") {
    std::size_t sample = 0;
    std::size_t crossing = 0;
    std::size_t farShell = 0;
    mesheal::AppController controller;
    loadThreeShells(controller, sample, crossing, farShell);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);

    controller.setShellVisible(farShell, false);
    const mesheal::BaseMeshDisplay display = controller.buildBaseMeshDisplay();
    REQUIRE(display.isCut);
    REQUIRE(display.faceIndices.size() ==
            static_cast<std::size_t>(controller.mesh().numFaces()) - controller.shells().shell(farShell).faces.size());
    for (const Eigen::Index f : display.faceIndices) {
        REQUIRE(controller.shells().shellOfFace(f) != static_cast<int>(farShell));
    }

    // Color by shell (on by default) and the selected-shell tint
    REQUIRE(controller.colorByShell());
    controller.showAllShells();
    REQUIRE(controller.pickShell(controller.shells().shell(farShell).faces.front(), false));
    REQUIRE(controller.shells().shell(farShell).selected);
    const mesheal::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    const Eigen::Index farFace = controller.shells().shell(farShell).faces.front();
    const Eigen::Index otherFace = controller.shells().shell(crossing).faces.front();
    REQUIRE(overlay.faceColors[static_cast<std::size_t>(otherFace)].isApprox(mesheal::AppController::shellColor(crossing)));
    REQUIRE_FALSE(overlay.faceColors[static_cast<std::size_t>(farFace)].isApprox(mesheal::AppController::shellColor(farShell)));

    // Selected shells are framed when no defect is highlighted
    const mesheal::CameraFocusTarget focus = controller.computeCameraFocusTarget();
    REQUIRE(focus.selectionBounds.min.isApprox(controller.shells().shell(farShell).bbox.min));

    REQUIRE(controller.escape());
    REQUIRE(controller.shells().selectedCount() == 0);
    REQUIRE_FALSE(controller.escape());
}

TEST_CASE("AppController deletes the selected shells as one undoable step",
          "[app][controller][shells][undo]") {
    std::size_t sample = 0;
    std::size_t crossing = 0;
    std::size_t farShell = 0;
    mesheal::AppController controller;
    loadThreeShells(controller, sample, crossing, farShell);
    const mesheal::Mesh before = controller.mesh();
    const std::size_t farFaces = controller.shells().shell(farShell).faces.size();

    REQUIRE_FALSE(controller.deleteSelectedShells()); // nothing selected
    controller.selectShellsWhere([](std::size_t) { return true; });
    REQUIRE_FALSE(controller.deleteSelectedShells()); // would delete everything
    REQUIRE_FALSE(controller.canUndo());

    controller.setShellVisible(sample, false);
    controller.clickShell(farShell, false, false, {});
    REQUIRE(controller.deleteSelectedShells());
    REQUIRE(controller.shells().size() == 4);
    REQUIRE(controller.mesh().numFaces() == before.numFaces() - static_cast<Eigen::Index>(farFaces));
    REQUIRE(controller.mesh().numVertices() == before.numVertices() - 8);
    REQUIRE(controller.isModified());
    REQUIRE(controller.lastUndoActionLabel() == "Delete 1 Shell(s)");
    // The hidden shell stays hidden
    std::size_t hidden = 0;
    for (const mesheal::ShellInfo& shell : controller.shells().shells()) {
        hidden += shell.visible ? 0 : 1;
    }
    REQUIRE(hidden == 1);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.shells().size() == 5);
    REQUIRE(controller.shells().shell(farShell).selected);
    REQUIRE_FALSE(controller.shells().shell(sample).visible);
    REQUIRE_FALSE(controller.isModified());

    // Hiding all shells and deleting all but one unhides the remaining shell; undo restores the hidden state
    controller.hideAllShells();
    REQUIRE(controller.shells().visibleCount() == 0);
    REQUIRE(controller.buildBaseMeshDisplay().isCut);
    controller.selectShellsWhere([&](std::size_t s) { return s != farShell; });
    REQUIRE(controller.deleteSelectedShells());
    REQUIRE(controller.shells().size() == 1);
    REQUIRE_FALSE(controller.shells().multiShell());
    REQUIRE(controller.shells().visibleCount() == 1);
    REQUIRE(controller.shells().shell(0).visible);
    REQUIRE(controller.shells().selectedCount() == 0);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.shells().size() == 5);
    REQUIRE(controller.shells().visibleCount() == 0);
}

TEST_CASE("AppController Fix All heals only the shells in scope and undo restores them",
          "[app][controller][shells][repair][undo]") {
    std::size_t sample = 0;
    std::size_t crossing = 0;
    std::size_t farShell = 0;
    mesheal::AppController controller;
    loadThreeShells(controller, sample, crossing, farShell);
    const mesheal::Mesh before = controller.mesh();

    controller.clickShell(farShell, false, false, {});
    REQUIRE_FALSE(controller.fixAllThinFaces()); // the farShell cube is clean
    REQUIRE_FALSE(controller.canUndo());

    controller.selectShellsWhere([&](std::size_t s) { return isSample(controller, s); });
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Thin / Sliver Faces (3 shells)");
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 0);
    // The other shells are untouched and stay unselected
    std::size_t untouched = 0;
    for (const mesheal::ShellInfo& shell : controller.shells().shells()) {
        if (shell.bbox.min.x() >= 5.0) {
            REQUIRE(shell.faces.size() == 12);
            REQUIRE_FALSE(shell.selected);
            ++untouched;
        }
    }
    REQUIRE(untouched == 2);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(mesheal::DefectCategory::ThinFaces) == 2);
    REQUIRE(controller.shells().shell(sample).selected);
}

TEST_CASE("AppController saves selected shells to one STL or one file each",
          "[app][controller][shells][io]") {
    std::size_t sample = 0;
    std::size_t crossing = 0;
    std::size_t farShell = 0;
    mesheal::AppController controller;
    loadThreeShells(controller, sample, crossing, farShell);
    REQUIRE(controller.shellFileStem() == "three_shells");
    const auto path = makeTempStlPath("shells");
    REQUIRE_FALSE(controller.saveSelectedShells(path)); // nothing selected

    controller.clickShell(crossing, false, false, {});
    controller.clickShell(farShell, true, false, {});
    REQUIRE(controller.saveSelectedShells(path));
    REQUIRE_FALSE(controller.isModified());
    {
        mesheal::AppController reloaded;
        REQUIRE(reloaded.loadStlFile(path));
        REQUIRE(reloaded.mesh().numFaces() == 24);
        REQUIRE(reloaded.shells().size() == 2);
    }
    std::filesystem::remove(path);

    const std::filesystem::path dir = path.parent_path() / (path.stem().string() + "_each");
    std::filesystem::create_directories(dir);
    REQUIRE(controller.saveEachSelectedShell(dir) == 2);
    for (const std::size_t s : {crossing, farShell}) {
        const auto file = dir / ("three_shells_shell_0" + std::to_string(s + 1) + ".stl");
        REQUIRE(std::filesystem::exists(file));
        REQUIRE(std::filesystem::file_size(file) == 84 + 50 * 12);
    }
    std::filesystem::remove_all(dir);
}

TEST_CASE("AppController tags a shell lying inside another without crossing it as nested",
          "[app][controller][shells]") {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createCubeMesh(10.0, mesheal::Unit::Millimeter);
    appendShell(mesh, mesheal::SampleMeshes::createCubeMesh(2.0, mesheal::Unit::Millimeter), {4.0, 4.0, 4.0});
    appendShell(mesh, mesheal::SampleMeshes::createCubeMesh(2.0, mesheal::Unit::Millimeter), {40.0, 0.0, 0.0});
    mesheal::AppController controller;
    controller.setMesh(std::move(mesh), "nested.stl");
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.shells().shell(0).nestedIn == -1);
    REQUIRE(controller.shells().shell(1).nestedIn == 0);
    REQUIRE(controller.shells().shell(2).nestedIn == -1);

    // The tag survives deleting another shell, before the intersection check runs again
    controller.setAutoCheck(mesheal::DefectCategory::Intersections, false);
    controller.clickShell(2, false, false, {});
    REQUIRE(controller.deleteSelectedShells());
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(controller.categoryHealth(mesheal::DefectCategory::Intersections).status == mesheal::HealthStatus::Stale);
    REQUIRE(controller.shells().shell(1).nestedIn == 0);
}

TEST_CASE("AppController Fix All splits touching edges and non-manifold vertices, each undoable",
          "[app][controller][repair][non-manifold][undo]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createTouchingCubesMesh(), "Touching cubes");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 3);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 1);
    REQUIRE(controller.shells().size() == 2);

    REQUIRE(controller.fixAllNonManifoldEdges());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Non-manifold Edges");
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 1); // the fin
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 1);
    REQUIRE(controller.mesh().numVertices() == before.numVertices() + 3);
    REQUIRE(controller.mesh().V.topRows(before.numVertices()) == before.V);
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.isModified());
    REQUIRE(controller.lastRepairStatusMessage().find("1 Fin / other edge") != std::string::npos);

    // Only the fin is left, which splitting cannot fix: no change and no undo entry
    REQUIRE_FALSE(controller.fixAllNonManifoldEdges());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Non-manifold Edges");

    REQUIRE(controller.fixAllNonManifoldVertices());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Non-manifold Vertices");
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 0);
    REQUIRE(controller.mesh().numVertices() == before.numVertices() + 4);
    REQUIRE_FALSE(controller.fixAllNonManifoldVertices());

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 1);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 3);
    REQUIRE(controller.shells().size() == 2);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController Fix Selected splits only the selected non-manifold elements and undoes",
          "[app][controller][repair][non-manifold][undo]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createTouchingCubesMesh(), "Touching cubes");
    const mesheal::Mesh before = controller.mesh();

    controller.selectCategory(DefectCategory::NonManifoldEdges);
    REQUIRE_FALSE(controller.fixSelectedNonManifoldEdges()); // nothing selected
    controller.setSubtypeFilter(1);                           // Fin / other
    controller.selectAllVisibleDefects();
    REQUIRE(controller.browser().selectedCount() == 1);
    REQUIRE_FALSE(controller.fixSelectedNonManifoldEdges());
    REQUIRE_FALSE(controller.canUndo());

    controller.setSubtypeFilter(0); // Touching
    controller.clickDefectRow(0, false, false);
    REQUIRE(controller.browser().selectedCount() == 1);
    REQUIRE(controller.fixSelectedNonManifoldEdges());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Non-manifold Edges (1)");
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) < 3);
    REQUIRE(controller.mesh().V.topRows(before.numVertices()) == before.V);

    controller.selectCategory(DefectCategory::NonManifoldVertices);
    controller.selectAllVisibleDefects();
    REQUIRE(controller.fixSelectedNonManifoldVertices());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Non-manifold Vertices (1)");
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 0);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
}

TEST_CASE("AppController non-manifold Fix All respects the shell scope", "[app][controller][shells][non-manifold]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createTouchingCubesMesh(), "Touching cubes");
    // Shell of the corner-touching cube C (bounding box in the negative octant) holds no non-manifold edge
    std::size_t cubeC = controller.shells().size();
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        if (controller.shells().shell(s).bbox.max.x() <= 0.0) {
            cubeC = s;
        }
    }
    REQUIRE(cubeC < controller.shells().size());
    controller.clickShell(cubeC, false, false, {});
    REQUIRE_FALSE(controller.fixAllNonManifoldEdges());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.fixAllNonManifoldVertices()); // the bow-tie touches cube C
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Non-manifold Vertices (1 shells)");
}

TEST_CASE("AppController opening an STL never shows touching contacts as defects",
          "[app][controller][non-manifold][io]") {
    using mesheal::DefectCategory;
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "mesheal_touching_cubes_test.stl";
    {
        std::ofstream file(path, std::ios::binary);
        mesheal::writeStlBinary(file, mesheal::SampleMeshes::createTouchingCubesMesh());
    }
    mesheal::AppController controller;
    REQUIRE(controller.loadStlFile(path));
    std::filesystem::remove(path);

    REQUIRE(controller.rowStatus(DefectCategory::NonManifoldVertices) == mesheal::HealthStatus::Ok);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldVertices) == 0);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 1); // only the fin
    REQUIRE(controller.shells().size() == 3);
    REQUIRE_FALSE(controller.fixAllNonManifoldVertices());
    REQUIRE_FALSE(controller.fixAllNonManifoldEdges());
    REQUIRE_FALSE(controller.canUndo());
}

namespace {

/// Cube A [0, 2]^3, cube B overlapping it ([1, 3]^3) and cube C far away; `openB` removes one face of B.
mesheal::Mesh mergeFixture(bool openB = false) {
    mesheal::Mesh mesh = mesheal::SampleMeshes::createCubeMesh(2.0);
    mesheal::Mesh b = mesheal::SampleMeshes::createCubeMesh(2.0);
    if (openB) {
        b.F.conservativeResize(11, 3);
    }
    appendShell(mesh, b, {1.0, 1.0, 1.0});
    appendShell(mesh, mesheal::SampleMeshes::createCubeMesh(2.0), {10.0, 0.0, 0.0});
    return mesh;
}

/// Selects the two overlapping cubes (everything left of x = 5).
void selectOverlappingCubes(mesheal::AppController& controller) {
    controller.selectShellsWhere([&](std::size_t s) { return controller.shells().shell(s).bbox.max.x() < 5.0; });
}

} // namespace

TEST_CASE("AppController merges the selected shells with Manifold as one undo step",
          "[app][controller][shells][merge][undo]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.setMesh(mergeFixture(), "three_cubes.stl");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) > 0);

    controller.openMergeShells();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::MergeShells);
    REQUIRE_FALSE(controller.mergeChecklist().canApply()); // nothing selected
    REQUIRE_FALSE(controller.mergeSelectedShells());

    selectOverlappingCubes(controller);
    const mesheal::MergeChecklist list = controller.mergeChecklist();
    REQUIRE(list.shells.size() == 2);
    REQUIRE(list.faces == 24);
    for (const mesheal::MergeCheck& check : list.checks) {
        REQUIRE(check.known);
        REQUIRE(check.failing.empty());
    }
    REQUIRE(list.crossingsBetween > 0);
    REQUIRE(list.canApply());

    REQUIRE(controller.mergeSelectedShells());
    REQUIRE(controller.lastUndoActionLabel() == "Merge 2 Shells");
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(controller.rowStatus(DefectCategory::Intersections) == mesheal::HealthStatus::Stale); // Auto off
    controller.recheckCategory(DefectCategory::Intersections);
    REQUIRE(controller.rowCount(DefectCategory::Intersections) == 0);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.watertightState() == mesheal::WatertightState::Watertight);
    REQUIRE(controller.shells().selectedCount() == 1); // the merged shell
    const std::vector<std::size_t> merged = controller.shells().selectedShells();
    REQUIRE(controller.shells().shell(merged.front()).bbox.max.isApprox(Eigen::Vector3d(3.0, 3.0, 3.0)));
    REQUIRE(controller.lastRepairStatusMessage().find("Merged 2 shells into 1") != std::string::npos);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.shells().selectedCount() == 2);

    // Merging down to 1 shell unhides everything and clears shell selection
    controller.selectShellsWhere([](std::size_t s) { return s == 2; });
    REQUIRE(controller.deleteSelectedShells());
    REQUIRE(controller.shells().size() == 2);
    controller.hideAllShells();
    REQUIRE(controller.shells().visibleCount() == 0);
    controller.selectShellsWhere([](std::size_t) { return true; });
    REQUIRE(controller.mergeSelectedShells());
    REQUIRE(controller.shells().size() == 1);
    REQUIRE_FALSE(controller.shells().multiShell());
    REQUIRE(controller.shells().visibleCount() == 1);
    REQUIRE(controller.shells().shell(0).visible);
    REQUIRE(controller.shells().selectedCount() == 0);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(controller.shells().visibleCount() == 0);
}

TEST_CASE("AppController Merge Shells is blocked by a shell Manifold rejects", "[app][controller][shells][merge]") {
    mesheal::AppController controller;
    controller.setMesh(mergeFixture(true), "open_cube.stl");
    selectOverlappingCubes(controller);

    const mesheal::MergeChecklist list = controller.mergeChecklist();
    REQUIRE(list.shells.size() == 2);
    REQUIRE(list.checks[0].failing.size() == 1); // not closed
    REQUIRE(list.checks[0].blocking);
    REQUIRE_FALSE(list.canApply());
    REQUIRE_FALSE(controller.mergeSelectedShells());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.shells().size() == 3);
}

namespace {

bool hasProblem(const mesheal::MergeChecklist& list, const std::string& text) {
    return std::any_of(list.problems.begin(), list.problems.end(),
                       [&](const std::string& p) { return p.find(text) != std::string::npos; });
}

void selectAllShells(mesheal::AppController& controller) {
    controller.selectShellsWhere([](std::size_t) { return true; });
}

} // namespace

TEST_CASE("AppController Merge Shells keeps a cavity, and undo restores it exactly",
          "[app][controller][shells][merge][cavity][undo]") {
    // A = [0, 4]^3 and B overlap; C = [1, 2] x [1, 3]^2 is a cavity of A that B does not reach
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createOverlapPlusCavityMesh(), "overlap_plus_cavity.stl");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.shells().size() == 3);
    selectAllShells(controller);

    const mesheal::MergeChecklist list = controller.mergeChecklist();
    REQUIRE(list.checks[mesheal::MergeChecklist::kOutwardOrCavity].known);
    REQUIRE(list.checks[mesheal::MergeChecklist::kOutwardOrCavity].failing.empty());
    REQUIRE(list.problems.empty());
    REQUIRE(list.operands == std::vector<std::size_t>{0, 1});
    REQUIRE(list.cavities == std::vector<std::size_t>{2});
    REQUIRE(list.canApply());

    REQUIRE(controller.mergeSelectedShells());
    REQUIRE(controller.lastUndoActionLabel() == "Merge 2 Shells");
    REQUIRE(controller.shells().size() == 2); // the union and the cavity
    REQUIRE(mesheal::signedVolume(controller.mesh()) == Catch::Approx(117.875));
    const std::string& message = controller.lastRepairStatusMessage();
    REQUIRE(message.find("Merged 2 shells into 1") != std::string::npos);
    REQUIRE(message.find("volume 128 -> 121.875") != std::string::npos);
    REQUIRE(message.find("1 cavity kept as it was: #2 (-4 mm^3).") != std::string::npos);
    REQUIRE(controller.shells().selectedCount() == 1);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.shells().size() == 3);
    REQUIRE(controller.mergeChecklist().cavities == std::vector<std::size_t>{2});
}

TEST_CASE("AppController Merge Shells is blocked when a shell reaches into a cavity", "[app][controller][shells][merge][cavity]") {
    using Variant = mesheal::SampleMeshes::CavityReachedVariant;
    mesheal::AppController controller;
    controller.setMesh(mesheal::SampleMeshes::createCavityReachedMesh(Variant::WithSeparatePair), "cavity_reached.stl");
    REQUIRE(controller.shells().size() == 5);

    SECTION("the whole group selected") {
        controller.selectShellsWhere([](std::size_t s) { return s < 3; });
        const mesheal::MergeChecklist list = controller.mergeChecklist();
        REQUIRE(hasProblem(list, "#2 reaches into cavity #3 of #1: merge them by hand."));
        REQUIRE(list.checks[mesheal::MergeChecklist::kOutwardOrCavity].failing == std::vector<std::size_t>{0, 1, 2});
        REQUIRE_FALSE(list.canApply());
    }
    SECTION("the cavity left out of the selection") {
        controller.selectShellsWhere([](std::size_t s) { return s < 2; });
        const mesheal::MergeChecklist list = controller.mergeChecklist();
        REQUIRE(hasProblem(list, "#2 reaches into cavity #3 of #1: merge them by hand."));
        REQUIRE_FALSE(list.canApply());
    }
    REQUIRE_FALSE(controller.mergeSelectedShells());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.shells().size() == 5);

    SECTION("the separate pair still merges") {
        controller.selectShellsWhere([](std::size_t s) { return s >= 3; });
        REQUIRE(controller.mergeChecklist().canApply());
        REQUIRE(controller.mergeSelectedShells());
        REQUIRE(controller.shells().size() == 4);
    }
}

TEST_CASE("AppController Merge Shells blocks inside-out shells and needs two outward ones", "[app][controller][shells][merge][cavity]") {
    mesheal::AppController controller;
    SECTION("a cavity box written inside out") {
        controller.setMesh(mesheal::SampleMeshes::createCavityBoxMesh(true), "cavity_box_inside_out.stl");
        selectAllShells(controller);
        const mesheal::MergeChecklist list = controller.mergeChecklist();
        REQUIRE(hasProblem(list, "#1 and its inner shell #2 look inside out together: fix orientation first."));
        REQUIRE(list.checks[mesheal::MergeChecklist::kOutwardOrCavity].failing == std::vector<std::size_t>{0, 1});
        REQUIRE_FALSE(list.canApply());
    }
    SECTION("a free-standing inside-out box") {
        mesheal::Mesh mesh = mesheal::SampleMeshes::createOverlapPlusCavityMesh();
        mesheal::appendMesh(mesh, mesheal::SampleMeshes::createBox({20.0, 0.0, 0.0}, {22.0, 2.0, 2.0}, true));
        controller.setMesh(mesh, "inside_out_box.stl");
        selectAllShells(controller);
        const mesheal::MergeChecklist list = controller.mergeChecklist();
        REQUIRE(hasProblem(list, "#4 is inside out: fix orientation first"));
        REQUIRE_FALSE(list.canApply());
    }
    SECTION("a cavity box: one outward shell, nothing to unite") {
        controller.setMesh(mesheal::SampleMeshes::createCavityBoxMesh(), "cavity_box.stl");
        selectAllShells(controller);
        const mesheal::MergeChecklist list = controller.mergeChecklist();
        REQUIRE(list.problems.empty());
        REQUIRE(list.cavities == std::vector<std::size_t>{1});
        REQUIRE(list.operands.size() == 1);
        REQUIRE_FALSE(list.canApply());
    }
    REQUIRE_FALSE(controller.mergeSelectedShells());
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController Merge Shells on the Turkey is blocked by its cavity", "[app][controller][shells][merge][cavity][.slow]") {
    mesheal::AppController controller;
    controller.setMesh(mesheal::loadStl(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
                                     "examples" / "Turkey.stl"),
                       "Turkey.stl");
    REQUIRE(controller.shells().size() == 28);
    selectAllShells(controller);
    const mesheal::MergeChecklist list = controller.mergeChecklist();
    REQUIRE(list.problems == std::vector<std::string>{"#6 and #9 reach into cavity #28 of #12: merge them by hand."});
    REQUIRE(list.checks[mesheal::MergeChecklist::kOutwardOrCavity].failing == std::vector<std::size_t>{5, 8, 11, 27});
    REQUIRE_FALSE(list.canApply());
}

TEST_CASE("AppController merges in the background, can cancel, and drops a stale merge",
          "[app][controller][shells][merge]") {
    mesheal::AppController controller;
    controller.setBackgroundChecks(true);
    controller.setMesh(mergeFixture(), "three_cubes.stl");
    controller.waitForChecks();
    selectOverlappingCubes(controller);

    SECTION("finishes and applies") {
        REQUIRE(controller.mergeSelectedShells());
        controller.waitForMerge();
        REQUIRE_FALSE(controller.mergeRunning());
        REQUIRE(controller.lastUndoActionLabel() == "Merge 2 Shells");
        REQUIRE(controller.shells().size() == 2);
    }
    SECTION("cancel drops the result") {
        REQUIRE(controller.mergeSelectedShells());
        controller.cancelMerge();
        REQUIRE_FALSE(controller.mergeRunning());
        controller.waitForMerge();
        REQUIRE_FALSE(controller.canUndo());
        REQUIRE(controller.shells().size() == 3);
    }
    SECTION("a mesh edit while merging drops the result") {
        REQUIRE(controller.mergeSelectedShells());
        controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
        controller.waitForMerge();
        REQUIRE(controller.lastUndoActionLabel() != "Merge 2 Shells");
        REQUIRE(controller.shells().size() == 3);
    }
}

TEST_CASE("Thin face defaults start at the built-in values and are independent of the live thresholds",
          "[app][controller][preferences]") {
    mesheal::AppController controller;
    const auto& def = controller.preferenceDefaults().thinFace;
    CHECK(def.minAltitude == mesheal::settings::kDefaultThinFaceMinAltitude);
    CHECK(def.minArea == mesheal::settings::kDefaultThinFaceMinArea);
    CHECK(def.maxAspectRatio == mesheal::settings::kDefaultThinFaceMaxAspectRatio);
    CHECK(controller.thresholds().minAltitude == def.minAltitude);

    // Editing a default updates the live value
    mesheal::ThinFaceThresholds edited = def;
    edited.minAltitude = 0.25;
    edited.minArea = 0.5;
    edited.maxAspectRatio = 20.0;
    controller.setThinFaceDefaults(edited);
    CHECK(controller.preferenceDefaults().thinFace.minAltitude == 0.25);
    CHECK(controller.thresholds().minAltitude == 0.25);
    CHECK(controller.thresholds().minArea == 0.5);
    CHECK(controller.thresholds().maxAspectRatio == 20.0);

    // Editing the live value leaves the default alone
    mesheal::ThinFaceThresholds live = controller.thresholds();
    live.minAltitude = 0.75;
    live.maxAspectRatio = 5.0;
    controller.setThresholds(live);
    CHECK(controller.thresholds().minAltitude == 0.75);
    CHECK(controller.preferenceDefaults().thinFace.minAltitude == 0.25);
    CHECK(controller.preferenceDefaults().thinFace.maxAspectRatio == 20.0);

    // Loading a mesh keeps the live thresholds (it does not reset them)
    controller.loadSampleDefectiveMesh();
    CHECK(controller.thresholds().minAltitude == 0.75);
    CHECK(controller.preferenceDefaults().thinFace.minAltitude == 0.25);
}

TEST_CASE("Changing a thin face default marks an Auto-off Thin row Stale like the tool controls",
          "[app][controller][preferences]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.setAutoCheck(mesheal::DefectCategory::ThinFaces, false);
    controller.checkMesh();
    REQUIRE(thinStatus(controller) != mesheal::HealthStatus::Stale);

    mesheal::ThinFaceThresholds strict = controller.preferenceDefaults().thinFace;
    strict.minArea = 1.0e9;
    controller.setThinFaceDefaults(strict);
    CHECK(controller.thresholds().minArea == 1.0e9);
    CHECK(thinStatus(controller) == mesheal::HealthStatus::Stale);
}

TEST_CASE("Set Origin to Selection moves the area-weighted selection centroid to the origin and undoes exactly",
          "[app][controller][transform]") {
    mesheal::AppController controller;
    // Two triangles of areas 4.5 and 18, apart from each other
    mesheal::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << 0.0, 0.0, 5.0,
              3.0, 0.0, 5.0,
              0.0, 3.0, 5.0,
              10.0, 0.0, 5.0,
              16.0, 0.0, 5.0,
              10.0, 6.0, 5.0;
    mesh.F.resize(2, 3);
    mesh.F << 0, 1, 2,
              3, 4, 5;
    mesh.recomputeNormals();
    controller.setMesh(mesh);
    const Eigen::MatrixXd originalV = controller.mesh().V;

    REQUIRE_FALSE(controller.selectionCentroid());
    REQUIRE_FALSE(controller.setOriginToSelection()); // nothing selected
    REQUIRE_FALSE(controller.canUndo());

    // One facet: its triangle centroid
    const Eigen::Index first[] = {0};
    controller.selectFaces(first, true);
    REQUIRE(controller.selectionCentroid()->isApprox(Eigen::Vector3d(1.0, 1.0, 5.0), 1e-12));

    // Two faces: weighted 1 : 4 by area
    const Eigen::Index second[] = {1};
    controller.selectFaces(second, true);
    const Eigen::Vector3d centroid = *controller.selectionCentroid();
    REQUIRE(centroid.isApprox(Eigen::Vector3d(49.0 / 5.0, 9.0 / 5.0, 5.0), 1e-12));

    REQUIRE(controller.setOriginToSelection());
    REQUIRE(controller.isModified());
    REQUIRE(controller.lastUndoActionLabel() == "Set Origin to Selection");
    REQUIRE(controller.mesh().numFaces() == 2);
    REQUIRE(controller.mesh().F == mesh.F);
    REQUIRE(controller.mesh().V.isApprox(originalV.rowwise() - centroid.transpose(), 1e-12));
    // Rigid: the faces keep their numbers, so the selection stays and its centroid is now the origin
    REQUIRE(controller.selectedFaceCount() == 2);
    REQUIRE(controller.selectionCentroid()->norm() < 1e-12);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == originalV);
    REQUIRE_FALSE(controller.isModified());
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("Translate / Rotate previews without changing the mesh, applies as one undo step and re-checks",
          "[app][controller][transform]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE(thinCount(controller) == 2);
    const mesheal::Mesh original = controller.mesh();

    controller.openTransform();
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::Transform);
    REQUIRE(controller.transformParams().isIdentity());
    REQUIRE_FALSE(controller.transformPreviewShown());
    REQUIRE_FALSE(controller.applyTransform()); // identity: nothing to apply
    REQUIRE_FALSE(controller.canUndo());

    const mesheal::MeshTransform transform{Eigen::Vector3d(10.0, -2.0, 0.5), Eigen::Vector3d(90.0, 0.0, 30.0)};
    controller.previewTransform(transform);
    REQUIRE(controller.transformParams() == transform);
    REQUIRE(controller.transformPreviewShown());
    REQUIRE(controller.mesh().V == original.V); // the preview is a copy
    const Eigen::MatrixXd& preview = controller.transformPreviewVertices();
    REQUIRE(preview.rows() == original.numVertices());
    for (Eigen::Index v = 0; v < preview.rows(); ++v) {
        const Eigen::Vector3d expected = mesheal::transformPoint(transform, original.V.row(v).transpose());
        REQUIRE((preview.row(v).transpose() - expected).norm() < 1e-12);
    }

    // Result hides the base mesh; Both draws it ghosted behind the preview; Original shows no preview
    controller.setTransformPreviewShow(mesheal::GlobalPreviewShow::Result);
    const mesheal::BaseMeshDisplay hidden = controller.buildBaseMeshDisplay();
    REQUIRE(hidden.isCut);
    REQUIRE(hidden.F.rows() == 0);
    controller.setTransformPreviewShow(mesheal::GlobalPreviewShow::Both);
    REQUIRE_FALSE(controller.buildBaseMeshDisplay().isCut);
    controller.setTransformPreviewShow(mesheal::GlobalPreviewShow::Original);
    REQUIRE_FALSE(controller.transformPreviewShown());
    controller.setTransformPreviewShow(mesheal::GlobalPreviewShow::Both);

    // Non-finite input is ignored
    mesheal::MeshTransform bad = transform;
    bad.translation.x() = std::numeric_limits<double>::quiet_NaN();
    controller.previewTransform(bad);
    REQUIRE(controller.transformParams().translation.x() == 0.0);
    controller.previewTransform(transform);

    const Eigen::MatrixXd expectedV = preview;
    REQUIRE(controller.applyTransform());
    REQUIRE(controller.lastUndoActionLabel() == "Translate / Rotate");
    REQUIRE(controller.mesh().V.isApprox(expectedV, 1e-14));
    REQUIRE(controller.mesh().F == original.F);
    REQUIRE(controller.activeTool() == mesheal::ToolSlot::Transform); // stays open, fields back to 0
    REQUIRE(controller.transformParams().isIdentity());
    REQUIRE_FALSE(controller.transformPreviewShown());
    // A rigid transform keeps the thin faces; the Auto rows ran again on the moved mesh
    REQUIRE(thinStatus(controller) == mesheal::HealthStatus::Defects);
    REQUIRE(thinCount(controller) == 2);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == original.V);
    REQUIRE_FALSE(controller.canUndo());

    // Closing the tool drops the preview
    controller.previewTransform(transform);
    REQUIRE(controller.transformPreviewShown());
    controller.closeTool();
    REQUIRE_FALSE(controller.transformPreviewShown());
    REQUIRE(controller.transformParams().isIdentity());
}

TEST_CASE("Apply Scale rescales the merge distance with the thresholds and undo restores it", "[app][controller][units][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const double distance = controller.globalRepairParams().mergeDistance;
    const double altitude = controller.thresholds().minAltitude;

    controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Meter);
    REQUIRE(controller.globalRepairParams().mergeDistance == Catch::Approx(distance * 0.001).epsilon(1e-12));
    REQUIRE(controller.thresholds().minAltitude == Catch::Approx(altitude * 0.001).epsilon(1e-12));

    // Changing the label alone never touches it
    controller.setFileUnit(mesheal::Unit::Centimeter);
    REQUIRE(controller.globalRepairParams().mergeDistance == Catch::Approx(distance * 0.001).epsilon(1e-12));

    controller.setFileUnit(mesheal::Unit::Meter);
    controller.undoLastModification();
    REQUIRE(controller.globalRepairParams().mergeDistance == distance);
    REQUIRE(controller.thresholds().minAltitude == altitude);
}

TEST_CASE("Undoing a scale or transform reports that it moves the whole mesh (the view re-frames)",
          "[app][controller][transform][undo]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.undoMovesWholeMesh()); // nothing to undo

    // A local edit does not move the whole mesh
    const Eigen::Index face[] = {0};
    controller.selectFaces(face, true);
    REQUIRE(controller.deleteSelectedFaces());
    REQUIRE_FALSE(controller.undoMovesWholeMesh());

    controller.scaleMeshCoordinatesToUnit(mesheal::Unit::Centimeter);
    REQUIRE(controller.undoMovesWholeMesh());

    controller.selectFaces(face, true);
    REQUIRE(controller.setOriginToSelection());
    REQUIRE(controller.undoMovesWholeMesh());

    controller.openTransform();
    controller.previewTransform(mesheal::MeshTransform{Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero()});
    REQUIRE(controller.applyTransform());
    REQUIRE(controller.undoMovesWholeMesh());

    // Each undo pops one entry: Translate / Rotate, Set Origin, Scale, then the face delete
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.undoMovesWholeMesh());
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.undoMovesWholeMesh());
    REQUIRE(controller.undoLastModification());
    REQUIRE_FALSE(controller.undoMovesWholeMesh());
    REQUIRE(controller.undoLastModification());
    REQUIRE_FALSE(controller.canUndo());
}

namespace {

/// The cached frame box (and mesh box) must equal a fresh computation. Returns the box so callers can
/// check that the steps really changed it.
mesheal::BoundingBox requireFrameCacheFresh(const mesheal::AppController& controller) {
    const mesheal::BoundingBox cached = controller.activeSelectionOrMeshBounds();
    const mesheal::BoundingBox fresh = controller.computeActiveSelectionOrMeshBounds();
    REQUIRE(cached.min == fresh.min);
    REQUIRE(cached.max == fresh.max);
    const mesheal::BoundingBox mesh = controller.mesh().boundingBox();
    REQUIRE(controller.meshBounds().min == mesh.min);
    REQUIRE(controller.meshBounds().max == mesh.max);
    if (controller.focusedFaceIndex() < 0) {
        REQUIRE(controller.computeSelectionCentroid() == fresh.center());
    }
    const std::vector<std::uint8_t> scope = controller.operatorScopeMask();
    REQUIRE(controller.operatorScopeFaceCount() ==
            (scope.empty() ? static_cast<std::size_t>(controller.mesh().numFaces())
                           : static_cast<std::size_t>(std::count(scope.begin(), scope.end(), std::uint8_t{1}))));
    return cached;
}

/// Counts the distinct boxes seen, so a test can tell that its steps changed the framed box.
struct BoxLog {
    std::vector<mesheal::BoundingBox> seen;
    void add(const mesheal::BoundingBox& box) {
        const bool known = std::any_of(seen.begin(), seen.end(), [&](const mesheal::BoundingBox& b) { return b.min == box.min && b.max == box.max; });
        if (!known) {
            seen.push_back(box);
        }
    }
};

} // namespace

TEST_CASE("The per-frame selection box cache follows defect selection, filters, thresholds, edits and undo",
          "[app][controller][camera][cache]") {
    // The sample's three thin faces plus a second cube far away (two shells)
    mesheal::Mesh mesh = mesheal::SampleMeshes::createSampleDefectiveMesh();
    mesheal::Mesh farShell = mesheal::SampleMeshes::createCubeMesh();
    farShell.V.col(0).array() += 100.0;
    mesheal::appendMesh(mesh, farShell);
    mesh.recomputeNormals();
    mesheal::AppController controller;
    controller.setMesh(mesh, "defective + cube");
    REQUIRE(controller.shells().multiShell());
    BoxLog log;
    const auto step = [&] { log.add(requireFrameCacheFresh(controller)); };
    step();

    controller.selectCategory(mesheal::DefectCategory::ThinFaces); // the visible defects
    step();
    controller.selectAllVisibleDefects();
    step();
    controller.selectNoDefects();
    step();
    REQUIRE(controller.focusNextDefect(1));
    step();
    REQUIRE(controller.focusNextDefect(1));
    step();
    controller.clickDefectRow(2, true, false); // Ctrl+click adds a row
    step();
    controller.setDefectSort(0, true);
    step();
    controller.selectHistogramBin(0, false);
    step();
    controller.clearHistogramBinFilter();
    step();
    controller.selectCategory(std::nullopt);
    step();
    REQUIRE(controller.pickFace(mesh.numFaces() - 1) == mesheal::PickOutcome::PickedFace); // on the farShell cube
    step();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    step();

    // Looser thresholds re-run the check: a new result (fewer rows) at the same address
    mesheal::ThinFaceThresholds loose = controller.thresholds();
    loose.minAltitude = 1e-9;
    loose.minArea = 1e-12;
    loose.maxAspectRatio = 1e9;
    controller.setThresholds(loose);
    step();
    controller.setThresholds(mesheal::ThinFaceThresholds::defaultsForUnit(mesheal::Unit::Millimeter));
    step();

    // Edits and undo
    controller.focusDefect(1);
    step();
    REQUIRE(controller.fixSelectedThinFaces());
    step();
    controller.selectCategory(mesheal::DefectCategory::ThinFaces);
    step();
    REQUIRE(controller.undoLastModification());
    step();
    controller.selectAllFaces();
    REQUIRE(controller.deleteSelectedFaces() == false); // every face: refused
    controller.clearFaceSelection();
    controller.toggleFaceSelected(0);
    REQUIRE(controller.deleteSelectedFaces());
    step();
    REQUIRE(controller.undoLastModification());
    step();
    REQUIRE(log.seen.size() >= 4);
}

TEST_CASE("The per-frame selection box cache follows shell visibility, shell selection and hole focus",
          "[app][controller][camera][cache][shells]") {
    mesheal::AppController controller;
    controller.setMesh(mesheal::separateTouchingContacts(mesheal::SampleMeshes::createHolesSampleMesh()), "holes");
    REQUIRE(controller.shells().size() == 4);
    BoxLog log;
    const auto step = [&] { log.add(requireFrameCacheFresh(controller)); };
    step();

    // No row: the visible / selected shells
    controller.setShellVisible(0, false);
    step();
    controller.setShellVisible(1, false);
    step();
    controller.showAllShells();
    step();
    const std::size_t order[] = {0, 1, 2, 3};
    controller.clickShell(2, false, false, order);
    step();
    controller.clickShell(3, true, false, order);
    step();
    REQUIRE(controller.toggleIsolateShells());
    step();
    REQUIRE(controller.toggleIsolateShells());
    step();
    controller.selectShellsWhere([](std::size_t) { return false; });
    step();
    controller.openDecimate(); // its panel shows the scope's face count every frame
    step();
    controller.toggleFaceSelected(0);
    step();
    controller.setShellVisible(static_cast<std::size_t>(controller.shells().shellOfFace(0)), false);
    step();
    controller.showAllShells();
    controller.clearFaceSelection();
    step();
    controller.closeTool();
    step();
    controller.hideAllShells();
    step();
    controller.showAllShells();
    step();

    // Open edges: the focused hole frames its rim
    controller.selectCategory(mesheal::DefectCategory::OpenEdges);
    step();
    for (int i = 0; i < 4; ++i) {
        REQUIRE(controller.focusNextDefect(1));
        step();
    }
    controller.setShellVisible(static_cast<std::size_t>(controller.shells().size() - 1), false); // scope changes
    step();
    controller.showAllShells();
    step();
    REQUIRE(controller.focusNextDefect(1));
    REQUIRE(controller.commitFocusedRepair()); // fills the hole (an edit)
    step();
    REQUIRE(controller.undoLastModification());
    step();
    REQUIRE(log.seen.size() >= 8);
}

namespace {

/// The doubled faces cube, loaded the way File > Load Sample loads it.
mesheal::Mesh doubledFacesCube() {
    return mesheal::separateTouchingContacts(mesheal::SampleMeshes::createDoubledFacesCubeMesh());
}

} // namespace

TEST_CASE("AppController Fix All Degenerate Faces heals the zero-area face of the sample and undoes",
          "[app][controller][repair][degenerate][undo]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
    REQUIRE(mesheal::categoryInfo(DefectCategory::Degenerate).hasFixAll);

    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Degenerate Faces");
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 0);
    REQUIRE(controller.lastRepairStatusMessage().find("Healed 1 of 1 zero-area face") != std::string::npos);
    REQUIRE(controller.isModified());

    // Nothing left: no change and no undo entry
    REQUIRE_FALSE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.lastRepairStatusMessage() == "No degenerate faces to fix.");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController heals a flat cap in a closed cube instead of deleting it",
          "[app][controller][repair][degenerate][undo]") {
    using mesheal::DefectCategory;
    // The closed part of the doubled faces cube: a T-junction on the bottom edge closed by a flat cap
    std::vector<Eigen::Index> closed(static_cast<std::size_t>(mesheal::SampleMeshes::kFlatCapFaceIndex + 1));
    std::iota(closed.begin(), closed.end(), Eigen::Index{0});
    mesheal::AppController controller;
    controller.setMesh(mesheal::extractFaces(mesheal::SampleMeshes::createDoubledFacesCubeMesh(), closed), "cap.stl");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    const double volumeBefore = mesheal::signedVolume(before);

    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 0);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 0);
    // Not deleted: flipped into the front face (or collapsed with a bottom half), the cube stays closed
    REQUIRE(controller.mesh().numFaces() >= 12);
    REQUIRE(mesheal::signedVolume(controller.mesh()) == Catch::Approx(volumeBefore));
    REQUIRE(controller.lastRepairStatusMessage().find("Healed 1 of 1 zero-area face") != std::string::npos);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
}

TEST_CASE("AppController Fix All Duplicate and Degenerate Faces clean the doubled faces cube, each undoable",
          "[app][controller][repair][duplicates][degenerate][undo]") {
    using mesheal::DefectCategory;
    mesheal::AppController controller;
    controller.setMesh(doubledFacesCube(), "Cube with doubled and collapsed faces");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 2);
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 2);
    REQUIRE(mesheal::categoryInfo(DefectCategory::Duplicates).hasFixAll);

    // The collapsed face goes; the flat cap waits for the duplicates, which the healer cannot read
    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
    REQUIRE(controller.lastRepairStatusMessage().find("Removed 1 collapsed face") != std::string::npos);
    REQUIRE(controller.lastRepairStatusMessage().find("Fix the Duplicate faces row first") != std::string::npos);
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 2);
    REQUIRE_FALSE(controller.fixAllDegenerateFaces()); // still blocked: no change, no undo entry
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Degenerate Faces");

    REQUIRE(controller.fixAllDuplicateFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Duplicate Faces");
    REQUIRE(controller.lastRepairStatusMessage() == "Removed 2 duplicate faces (1 same, 1 opposite winding).");
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 0);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 0);
    REQUIRE(controller.rowCount(DefectCategory::FlippedNormals) == 0);
    REQUIRE_FALSE(controller.fixAllDuplicateFaces());

    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 0);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.rowCount(DefectCategory::NonManifoldEdges) == 0);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 2);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 2);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController Fix All Duplicate and Degenerate Faces respect the shell scope",
          "[app][controller][repair][duplicates][degenerate][shells]") {
    using mesheal::DefectCategory;
    mesheal::Mesh mesh = doubledFacesCube();
    appendShell(mesh, doubledFacesCube(), {20.0, 0.0, 0.0});
    mesheal::AppController controller;
    controller.setMesh(std::move(mesh), "two_cubes.stl");
    REQUIRE(controller.shells().size() == 2);
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 4);

    const auto faceRowsOfShell = [&](std::size_t s) {
        std::vector<std::array<double, 9>> rows;
        for (const Eigen::Index f : controller.shells().shell(s).faces) {
            std::array<double, 9> row{};
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    row[static_cast<std::size_t>(3 * c + k)] = controller.mesh().V(controller.mesh().F(f, c), k);
                }
            }
            rows.push_back(row);
        }
        std::sort(rows.begin(), rows.end());
        return rows;
    };
    const auto shellAt = [&](double x) {
        for (std::size_t s = 0; s < controller.shells().size(); ++s) {
            if (controller.shells().shell(s).bbox.min.x() == x) {
                return s;
            }
        }
        return controller.shells().size();
    };

    const std::vector<std::array<double, 9>> farBefore = faceRowsOfShell(shellAt(20.0));
    controller.clickShell(shellAt(0.0), false, false, {});
    REQUIRE(controller.fixAllDuplicateFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Duplicate Faces (1 shells)");
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 0); // in scope
    REQUIRE(faceRowsOfShell(shellAt(20.0)) == farBefore);
    controller.selectShellsWhere([](std::size_t) { return false; });
    REQUIRE(controller.rowCount(DefectCategory::Duplicates) == 2);

    // Dedupe the far cube too. A face that repeats a vertex belongs to the shell of its vertices
    REQUIRE(controller.fixAllDuplicateFaces());
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 4);
    const std::vector<std::array<double, 9>> farDeduped = faceRowsOfShell(shellAt(20.0));
    controller.clickShell(shellAt(0.0), false, false, {});
    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Degenerate Faces (1 shells)");
    // Only the near collapsed face goes; the far one would make the healer tear the mesh, so the cap waits
    REQUIRE(controller.lastRepairStatusMessage().find("Removed 1 collapsed face") != std::string::npos);
    REQUIRE(controller.lastRepairStatusMessage().find("shells outside the scope have faces with a repeated vertex") !=
            std::string::npos);
    REQUIRE(faceRowsOfShell(shellAt(20.0)) == farDeduped);

    // The far cube: its collapsed face goes and, with none left anywhere, its cap is healed
    const std::vector<std::array<double, 9>> nearBefore = faceRowsOfShell(shellAt(0.0));
    controller.selectShellsWhere([](std::size_t) { return false; });
    controller.clickShell(shellAt(20.0), false, false, {});
    REQUIRE(controller.fixAllDegenerateFaces());
    REQUIRE(controller.lastRepairStatusMessage().find("Healed 1 of 1 zero-area face") != std::string::npos);
    REQUIRE(faceRowsOfShell(shellAt(0.0)) == nearBefore);
    controller.selectShellsWhere([](std::size_t) { return false; });
    REQUIRE(controller.rowCount(DefectCategory::Degenerate) == 1); // the near cube's cap
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
}

TEST_CASE("AppController Fix All Flipped Normals makes each shell agree with its majority, and undoes",
          "[app][controller][repair][orientation][undo]") {
    using mesheal::DefectCategory;
    const auto flipFaces = [](mesheal::Mesh mesh, std::initializer_list<Eigen::Index> faces) {
        for (const Eigen::Index f : faces) {
            std::swap(mesh.F(f, 1), mesh.F(f, 2));
        }
        mesh.recomputeNormals();
        return mesh;
    };
    const mesheal::Mesh good = mesheal::SampleMeshes::createBox({0, 0, 0}, {2, 2, 2});
    mesheal::AppController controller;
    controller.setMesh(flipFaces(good, {0, 5, 9}), "cube with three flipped faces");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(mesheal::categoryInfo(DefectCategory::FlippedNormals).hasFixAll);
    REQUIRE(controller.rowCount(DefectCategory::FlippedNormals) == 3);

    REQUIRE(controller.fixAllFlippedNormals());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Flipped Normals");
    REQUIRE(controller.lastRepairStatusMessage() == "Flipped 3 faces in 1 shell to agree with their neighbours.");
    REQUIRE(controller.rowCount(DefectCategory::FlippedNormals) == 0);
    REQUIRE(controller.mesh().F == good.F);
    REQUIRE_FALSE(controller.fixAllFlippedNormals());
    REQUIRE(controller.lastRepairStatusMessage() == "No flipped faces to fix.");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::FlippedNormals) == 3);
    REQUIRE_FALSE(controller.canUndo());

    // Seven of twelve flipped: the majority wins, so the cube ends up inside out as a whole
    controller.setMesh(flipFaces(good, {0, 1, 2, 3, 4, 5, 6}), "cube mostly flipped");
    REQUIRE(controller.fixAllFlippedNormals());
    REQUIRE(controller.lastRepairStatusMessage().find("#1 now faces inward as a whole") != std::string::npos);
    REQUIRE(controller.rowCount(DefectCategory::FlippedNormals) == 0);
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 1);
    REQUIRE(controller.fixAllInvertedShells());
    REQUIRE(controller.mesh().F == good.F);
}

TEST_CASE("AppController Fix All Inverted Shells reverses inside-out shells only, never a cavity, and undoes",
          "[app][controller][repair][orientation][cavity][undo]") {
    using mesheal::DefectCategory;
    using mesheal::SampleMeshes;
    mesheal::AppController controller;
    controller.setMesh(SampleMeshes::combineMeshes({SampleMeshes::createCavityBoxMesh(),
                                                     SampleMeshes::createBox({10, 0, 0}, {12, 2, 2}, true)}),
                       "cavity box and an inside-out cube");
    const mesheal::Mesh before = controller.mesh();
    REQUIRE(mesheal::categoryInfo(DefectCategory::InvertedShells).hasFixAll);
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 1);

    REQUIRE(controller.fixAllInvertedShells());
    REQUIRE(controller.lastUndoActionLabel() == "Fix All Inverted Shells");
    REQUIRE(controller.lastRepairStatusMessage() == "Reversed 1 inside-out shell: #3. Kept 1 cavity inward: #2.");
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 0);
    REQUIRE(controller.mesh().F.topRows(24) == before.F.topRows(24)); // the cavity box is untouched
    REQUIRE(mesheal::signedVolume(controller.mesh()) == Catch::Approx(64.0 - 8.0 + 8.0));
    REQUIRE_FALSE(controller.fixAllInvertedShells());
    REQUIRE(controller.lastRepairStatusMessage() == "No inside-out shells to reverse. Kept 1 cavity inward: #2.");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 1);

    // Scope: only the selected shell can be reversed
    controller.clickShell(0, false, false, {});
    REQUIRE_FALSE(controller.fixAllInvertedShells());
    REQUIRE(controller.lastRepairStatusMessage().find("1 inside-out shell outside the scope was left") != std::string::npos);
    controller.selectShellsWhere([](std::size_t) { return false; });

    // A body written inside out with a shell inside: stop and report, nothing flipped
    controller.setMesh(SampleMeshes::createCavityBoxMesh(true), "Box with cavity, inside out");
    const mesheal::Mesh stopped = controller.mesh();
    REQUIRE_FALSE(controller.fixAllInvertedShells());
    REQUIRE(controller.mesh().F == stopped.F);
    REQUIRE(controller.lastRepairStatusMessage().find("#1 and its inner shell #2 look inside out together") !=
            std::string::npos);
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController Flip shells: Flip all and Flip outer only on a box written inside out, each undoable",
          "[app][controller][repair][orientation][cavity][undo]") {
    using mesheal::DefectCategory;
    using mesheal::SampleMeshes;
    mesheal::AppController controller;
    controller.setMesh(SampleMeshes::createCavityBoxMesh(true), "Box with cavity, inside out");
    const mesheal::Mesh before = controller.mesh();
    const auto warnings = [&] { return controller.categoryHealth(DefectCategory::InvertedShells).result->warnings; };
    REQUIRE(warnings().size() == 1);
    REQUIRE(warnings()[0].kind == mesheal::WarningKind::StoppedGroup);
    const std::vector<int> group = warnings()[0].shells;
    REQUIRE(group == std::vector<int>{0, 1});

    // Flip all: A outward, C inward, the correct cavity box
    REQUIRE(controller.flipShells(group));
    REQUIRE(controller.lastUndoActionLabel() == "Flip Shells #1 and #2");
    REQUIRE(controller.lastRepairStatusMessage() == "Flipped #1 and #2 (24 faces): they now face the other way.");
    REQUIRE(controller.mesh().F == SampleMeshes::createCavityBoxMesh().F);
    REQUIRE(warnings().empty());
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 0);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(warnings().size() == 1);

    // Flip outer only: A outward with C outward inside it, a nested solid
    const std::vector<int> outer{group.front()};
    REQUIRE(controller.flipShells(outer));
    REQUIRE(controller.lastUndoActionLabel() == "Flip Shell #1");
    REQUIRE(warnings().empty());
    REQUIRE(controller.rowCount(DefectCategory::InvertedShells) == 0);
    REQUIRE(mesheal::signedVolume(controller.mesh()) == Catch::Approx(64.0 + 8.0));
    REQUIRE(controller.shells().shell(1).nestedIn == 0);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE_FALSE(controller.canUndo());

    // Nothing to flip without shells
    const std::vector<int> none;
    REQUIRE_FALSE(controller.flipShells(none));
    REQUIRE_FALSE(controller.canUndo());
}

TEST_CASE("AppController on cuboctahedron_faces.stl sliver repair", "[app][sliver][cuboctahedron]") {
    const std::filesystem::path stlPath =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "cuboctahedron_faces.stl";
    if (!std::filesystem::exists(stlPath)) {
        return;
    }

    mesheal::AppController controller;
    controller.setMesh(mesheal::loadStl(stlPath), "cuboctahedron_faces.stl");

    REQUIRE(thinCount(controller) == 33);

    // Batch fix heals all 33 thin faces down to 0
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);

    // Verify undo restores the original mesh
    REQUIRE(controller.canUndo());
    REQUIRE(controller.undoLastModification());
    REQUIRE(thinCount(controller) == 33);

    // Re-heal
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(thinCount(controller) == 0);
}




TEST_CASE("Custom back faces draw the face selection as its own surface; other modes do not",
          "[app][controller][selection][backface]") {
    mesheal::AppController controller;
    controller.loadSampleDefectiveMesh();
    const Eigen::Index faces[] = {0, 1};
    controller.selectFaces(faces, true);

    mesheal::OverlayAppearance appearance = controller.overlayAppearance();
    for (const auto mode : {mesheal::BackFaceMode::Identical, mesheal::BackFaceMode::Darker, mesheal::BackFaceMode::Cull}) {
        appearance.backFaceMode = mode;
        controller.setOverlayAppearance(appearance);
        REQUIRE(controller.buildHighlightOverlay().selectedFaceTriangles.empty());
    }

    appearance.backFaceMode = mesheal::BackFaceMode::Custom;
    controller.setOverlayAppearance(appearance);
    const mesheal::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    REQUIRE(overlay.selectedFaceIndices == std::vector<Eigen::Index>{0, 1});
    REQUIRE(overlay.selectedFaceTriangles.size() == 2);
    REQUIRE(overlay.selectedFacePoints.size() <= 6);

    controller.clearFaceSelection();
    REQUIRE(controller.buildHighlightOverlay().selectedFaceTriangles.empty());
}
