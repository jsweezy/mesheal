#include "app/AppController.hpp"
#include "io/SampleMeshes.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>

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

TEST_CASE("AppController runs Check Manifold, toggles category vs single-face selection, and builds 3D highlight overlays",
          "[app][controller][diagnostics][selection]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.hasDiagnosticsReport());
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::None);

    const ogun::ThinFaceReport& report = controller.runCheckManifold();
    REQUIRE(controller.hasDiagnosticsReport());
    REQUIRE(report.numDefects() == 3);

    const std::vector<Eigen::Index> allExpectedDefects = {
        ogun::SampleMeshes::kZeroAreaFaceIndex,
        ogun::SampleMeshes::kMicroscopicFaceIndex,
        ogun::SampleMeshes::kNeedleSliverFaceIndex
    };

    // Running Check Manifold when defects exist selects the "Thin Faces" category by default
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::AllThinFaces);
    REQUIRE(controller.highlightedFaceIndices() == allExpectedDefects);

    const ogun::HighlightOverlayGeometry categoryOverlay = controller.buildHighlightOverlay();
    REQUIRE(categoryOverlay.faceColors.size() == 15);
    REQUIRE(categoryOverlay.hasSelectionOverlay());
    // 3 defective triangles with distinct vertices -> 9 overlay vertices (since Face 12, 13, 14 are disjoint) and 9 edges
    REQUIRE(categoryOverlay.overlayPoints.size() == 9);
    REQUIRE(categoryOverlay.overlayEdges.size() == 9);

    // Selecting a single defective face index highlights only that facet
    controller.selectSingleFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::SingleFace);
    REQUIRE(controller.selectedFaceIndex() == ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.highlightedFaceIndices() ==
            std::vector<Eigen::Index>{ogun::SampleMeshes::kNeedleSliverFaceIndex});

    const ogun::HighlightOverlayGeometry singleOverlay = controller.buildHighlightOverlay();
    REQUIRE(singleOverlay.faceColors.size() == 15);
    REQUIRE(singleOverlay.hasSelectionOverlay());
    REQUIRE(singleOverlay.overlayPoints.size() == 3);
    REQUIRE(singleOverlay.overlayEdges.size() == 3);
    // Clean face color differs from highlighted face color
    REQUIRE_FALSE(singleOverlay.faceColors[0].isApprox(
        singleOverlay.faceColors[static_cast<std::size_t>(ogun::SampleMeshes::kNeedleSliverFaceIndex)], 1e-6));

    // Re-selecting the category header restores all 3 faces
    controller.selectAllThinFaces();
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::AllThinFaces);
    REQUIRE(controller.highlightedFaceIndices() == allExpectedDefects);

    // Clearing selection removes overlay geometry
    controller.clearSelection();
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::None);
    REQUIRE(controller.highlightedFaceIndices().empty());
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay());
}

TEST_CASE("AppController computes 3D camera focus bounding box and look-at target when '.' is triggered",
          "[app][controller][camera]") {
    ogun::AppController controller;

    // Without a loaded mesh, camera focus is invalid
    REQUIRE_FALSE(controller.computeCameraFocusTarget().valid);

    controller.loadSampleDefectiveMesh();
    controller.runCheckManifold();

    SECTION("Focusing on a single microscopic defective face centers camera on that facet's centroid") {
        controller.selectSingleFace(ogun::SampleMeshes::kMicroscopicFaceIndex);

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
        controller.selectAllThinFaces();
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

    // Clean cube should produce 0 defects on Check Manifold and SelectionMode::None
    const ogun::ThinFaceReport& report = controller.runCheckManifold();
    REQUIRE(report.empty());
    REQUIRE(controller.selectionMode() == ogun::SelectionMode::None);

    // Ghost mesh toggle & opacity clamping
    REQUIRE_FALSE(controller.ghostMeshEnabled());
    controller.setGhostMeshEnabled(true);
    REQUIRE(controller.ghostMeshEnabled());
    controller.setGhostMeshOpacity(0.4);
    REQUIRE(controller.ghostMeshOpacity() == Catch::Approx(0.4));
}

TEST_CASE("AppController persists highlight point radius and edge width across defect selections and supports dynamic zoom scaling",
          "[app][controller][highlight][zoom]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.runCheckManifold();

    // Verify refined, non-intrusive default relative radii
    REQUIRE(controller.highlightPointRadius() ==
            Catch::Approx(ogun::AppController::kDefaultHighlightPointRadius));
    REQUIRE(controller.highlightEdgeRadius() ==
            Catch::Approx(ogun::AppController::kDefaultHighlightEdgeRadius));
    REQUIRE(controller.highlightPointRadius() < 0.003);
    REQUIRE(controller.highlightEdgeRadius() < 0.002);
    REQUIRE(controller.dynamicHighlightSizingEnabled());

    // User customizes point radius and edge width
    controller.setHighlightPointRadius(0.00045);
    controller.setHighlightEdgeRadius(0.00022);
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    // Switching between defective faces, category selection, and re-running diagnostics must NOT reset sizes
    controller.selectSingleFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    controller.selectSingleFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    controller.selectSingleFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    controller.selectAllThinFaces();
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    controller.runCheckManifold();
    REQUIRE(controller.highlightPointRadius() == Catch::Approx(0.00045));
    REQUIRE(controller.highlightEdgeRadius() == Catch::Approx(0.00022));

    // Dynamic zoom scaling: select a single face and test camera distances relative to full-mesh reference distance
    controller.selectSingleFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    const ogun::CameraFocusTarget focus = controller.computeCameraFocusTarget(Eigen::Vector3d(0.0, 0.0, -1.0));
    REQUIRE(focus.valid);

    const double refDist = controller.referenceFullMeshViewDistance();
    REQUIRE(refDist > 0.0);

    // At or beyond full-mesh viewing distance, zoom scale is 1.0
    const Eigen::Vector3d camFar = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 1.2);
    REQUIRE(controller.computeHighlightZoomScale(camFar) == Catch::Approx(1.0));
    REQUIRE(controller.effectiveHighlightPointRadius(camFar) == Catch::Approx(0.00045));
    REQUIRE(controller.effectiveHighlightEdgeRadius(camFar) == Catch::Approx(0.00022));

    // At 25% of full-mesh viewing distance, radii shrink proportionally to 25%
    const Eigen::Vector3d camQuarter = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 0.25);
    REQUIRE(controller.computeHighlightZoomScale(camQuarter) == Catch::Approx(0.25).epsilon(1e-6));
    REQUIRE(controller.effectiveHighlightPointRadius(camQuarter) == Catch::Approx(0.00045 * 0.25).epsilon(1e-6));
    REQUIRE(controller.effectiveHighlightEdgeRadius(camQuarter) == Catch::Approx(0.00022 * 0.25).epsilon(1e-6));

    // At 1% of full-mesh viewing distance (zoomed in close), radii shrink to 1%
    const Eigen::Vector3d camClose = focus.targetCenter + Eigen::Vector3d(0.0, 0.0, refDist * 0.01);
    REQUIRE(controller.computeHighlightZoomScale(camClose) == Catch::Approx(0.01).epsilon(1e-6));
    REQUIRE(controller.effectiveHighlightPointRadius(camClose) == Catch::Approx(0.00045 * 0.01).epsilon(1e-6));
    REQUIRE(controller.effectiveHighlightEdgeRadius(camClose) == Catch::Approx(0.00022 * 0.01).epsilon(1e-6));

    // Disabling dynamic sizing keeps effective radii at 100% of base value regardless of zoom
    controller.setDynamicHighlightSizingEnabled(false);
    REQUIRE_FALSE(controller.dynamicHighlightSizingEnabled());
    REQUIRE(controller.computeHighlightZoomScale(camClose) == Catch::Approx(1.0));
    REQUIRE(controller.effectiveHighlightPointRadius(camClose) == Catch::Approx(0.00045));
    REQUIRE(controller.effectiveHighlightEdgeRadius(camClose) == Catch::Approx(0.00022));
}

TEST_CASE("AppController updates orbit pivot to face centroid and aligns camera to +Normal / -Normal with '.' toggle",
          "[app][controller][camera][pivot][normal]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();

    // Initially with no selection, orbit pivot is at the full mesh center (5, 5, 5)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 5.0, 5.0), 1e-12));

    controller.runCheckManifold();
    // Selecting all thin faces sets orbit pivot to the selection bounds center (5.0, 2.5001, 5.0)
    REQUIRE(controller.orbitPivot().isApprox(Eigen::Vector3d(5.0, 2.5001, 5.0), 1e-12));

    // Selecting Face #14 (kNeedleSliverFaceIndex: vertices (0,5,10), (10,5,10), (5,5.0002,10))
    // immediately sets orbit pivot to its true centroid (5.0, 5.0 + 0.0002/3.0, 10.0)
    controller.selectSingleFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
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
    controller.selectSingleFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
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
