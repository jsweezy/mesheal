#include "app/AppController.hpp"
#include "app/HealthCheckRunner.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <memory>

namespace {

using ogun::DefectCategory;
using ogun::HealthStatus;

HealthStatus status(const ogun::AppController& c, DefectCategory category) {
    return c.categoryHealth(category).status;
}

/// Subdivided-grid "terrain" mesh with many triangles so a background job takes measurable time.
ogun::Mesh makeGrid(int n) {
    ogun::Mesh mesh;
    mesh.V.resize((n + 1) * (n + 1), 3);
    for (int y = 0; y <= n; ++y) {
        for (int x = 0; x <= n; ++x) {
            mesh.V.row(y * (n + 1) + x) << x, y, 0.0;
        }
    }
    mesh.F.resize(2 * n * n, 3);
    int f = 0;
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const int a = y * (n + 1) + x;
            mesh.F.row(f++) << a, a + 1, a + n + 2;
            mesh.F.row(f++) << a, a + n + 2, a + n + 1;
        }
    }
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("Check Mesh fills every row and the watertight badge", "[app][health]") {
    ogun::AppController controller;
    controller.setCheckMeshAfterLoad(false);
    controller.loadSampleDefectiveMesh();
    for (const DefectCategory category : ogun::kAllDefectCategories) {
        REQUIRE(status(controller, category) == HealthStatus::NotRun);
    }
    REQUIRE(controller.watertightState() == ogun::WatertightState::Unknown);

    controller.checkMesh();
    REQUIRE(status(controller, DefectCategory::ThinFaces) == HealthStatus::Defects);
    REQUIRE(status(controller, DefectCategory::Shells) == HealthStatus::Ok); // information only
    REQUIRE(status(controller, DefectCategory::Degenerate) == HealthStatus::Defects);
    REQUIRE(controller.totalProblemCount() > 0);
    // The sample's extra triangles are loose, so the mesh is not watertight
    REQUIRE(controller.watertightState() == ogun::WatertightState::NotWatertight);
    REQUIRE_FALSE(controller.checksRunning());
}

TEST_CASE("Edits re-run Auto rows and mark Auto-off rows Stale; Re-check clears Stale", "[app][health]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.setAutoCheck(DefectCategory::Degenerate, false);
    REQUIRE(status(controller, DefectCategory::Degenerate) == HealthStatus::Defects);

    controller.selectCategory(DefectCategory::Degenerate);
    REQUIRE(controller.browsableResult());

    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(status(controller, DefectCategory::ThinFaces) == HealthStatus::Ok);
    REQUIRE(status(controller, DefectCategory::Degenerate) == HealthStatus::Stale);
    REQUIRE(controller.categoryHealth(DefectCategory::Degenerate).result.has_value()); // old count kept
    REQUIRE_FALSE(controller.browsableResult()); // stale indices are never browsed

    controller.recheckCategory(DefectCategory::Degenerate);
    REQUIRE(status(controller, DefectCategory::Degenerate) != HealthStatus::Stale);
    REQUIRE(controller.browsableResult());

    // Turning Auto back on re-runs a stale row
    controller.undoLastModification();
    controller.setAutoCheck(DefectCategory::Degenerate, false);
    REQUIRE(controller.fixAllThinFaces());
    REQUIRE(status(controller, DefectCategory::Degenerate) == HealthStatus::Stale);
    controller.setAutoCheck(DefectCategory::Degenerate, true);
    REQUIRE(status(controller, DefectCategory::Degenerate) != HealthStatus::Stale);
}

TEST_CASE("Threshold changes re-run the thin row; File Units re-interpret thresholds", "[app][health]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.categoryHealth(DefectCategory::ThinFaces).result->size() == 3);

    ogun::ThinFaceThresholds loose = controller.thresholds();
    loose.minAltitude = 0.0;
    loose.minArea = 0.0;
    loose.maxAspectRatio = 1.0e9;
    controller.setThresholds(loose);
    REQUIRE(controller.categoryHealth(DefectCategory::ThinFaces).result->size() == 1); // only zero-area

    controller.setAutoCheck(DefectCategory::ThinFaces, false);
    controller.setThresholds(ogun::ThinFaceThresholds::defaultsForUnit(ogun::Unit::Millimeter));
    REQUIRE(status(controller, DefectCategory::ThinFaces) == HealthStatus::Stale);
}

TEST_CASE("Fix Selected repairs only the selected thin faces and is undoable", "[app][health][repair]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE_FALSE(controller.fixSelectedThinFaces()); // nothing selected

    controller.selectCategory(DefectCategory::ThinFaces);
    controller.focusDefect(1); // the microscopic face (second worst)
    REQUIRE(controller.browser().selectedCount() == 1);
    const ogun::Mesh before = controller.mesh();
    REQUIRE(controller.fixSelectedThinFaces());
    REQUIRE(controller.categoryHealth(DefectCategory::ThinFaces).result->size() == 2);
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Thin Faces (1)");

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.categoryHealth(DefectCategory::ThinFaces).result->size() == 3);
}

TEST_CASE("Histogram bins select their defects and Next/Prev focus within them", "[app][health][selection]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(DefectCategory::ThinFaces);
    REQUIRE_FALSE(controller.browser().histogram().empty());

    controller.stepHistogramBin(+1);
    REQUIRE(controller.browser().binFilter().has_value());
    REQUIRE(controller.browser().selectedCount() == controller.browser().visibleRows().size());
    controller.clearHistogramBinFilter();

    REQUIRE(controller.focusNextDefect(+1));
    const Eigen::Index first = controller.focusedFaceIndex();
    REQUIRE(controller.focusNextDefect(+1));
    REQUIRE(controller.focusedFaceIndex() != first);
    REQUIRE(controller.focusNextDefect(-1));
    REQUIRE(controller.focusedFaceIndex() == first);

    // Changing the sub-type filter drops a focus that is no longer visible (FB-13)
    controller.setSubtypeFilter(static_cast<int>(ogun::ThinFaceSubtype::Needle));
    REQUIRE_FALSE(controller.inspectorActive());
}

TEST_CASE("The overlay cap limits tubes to the selected defects", "[app][health][highlight]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(DefectCategory::ThinFaces);
    controller.setOverlayCap(2);
    REQUIRE_FALSE(controller.buildHighlightOverlay().hasSelectionOverlay()); // 3 visible > cap: tint only

    controller.focusDefect(0);
    const ogun::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    REQUIRE(overlay.overlayFaces.size() == 1);
    REQUIRE(overlay.overlayEdges.size() == 3);
}

TEST_CASE("Highlighting 20k faces builds its overlay quickly", "[app][health][perf]") {
    ogun::AppController controller;
    controller.setCheckMeshAfterLoad(false);
    controller.setMesh(makeGrid(100), "Grid"); // 20,000 faces

    // Every face becomes a thin defect, then all are selected (the old O(E^2) path)
    ogun::ThinFaceThresholds everything = controller.thresholds();
    everything.minArea = 10.0;
    controller.setThresholds(everything);
    controller.checkMesh();
    controller.selectCategory(DefectCategory::ThinFaces);
    REQUIRE(controller.browser().visibleRows().size() == 20000);
    controller.selectAllVisibleDefects();

    const auto start = std::chrono::steady_clock::now();
    const ogun::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(overlay.overlayFaces.size() == 20000);
    REQUIRE(overlay.overlayEdges.size() == 3 * 100 * 100 + 2 * 100);
    REQUIRE(seconds < 1.0);
}

TEST_CASE("Background checks arrive through polling, can be cancelled, and drop stale generations",
          "[app][health][background]") {
    ogun::AppController controller;
    controller.setCheckMeshAfterLoad(false);
    controller.setBackgroundChecks(true);
    controller.setMesh(makeGrid(150), "Grid");

    controller.checkMesh();
    REQUIRE(controller.checksRunning());
    controller.waitForChecks();
    REQUIRE_FALSE(controller.checksRunning());
    REQUIRE(status(controller, DefectCategory::OpenEdges) == HealthStatus::Defects); // a grid has a rim
    REQUIRE(controller.watertightState() == ogun::WatertightState::NotWatertight);

    controller.checkMesh();
    controller.cancelChecks();
    REQUIRE_FALSE(controller.checksRunning());
    REQUIRE(status(controller, DefectCategory::OpenEdges) == HealthStatus::Stale);
    controller.pollBackgroundChecks();
    REQUIRE(status(controller, DefectCategory::OpenEdges) == HealthStatus::Stale);

    // Results computed for an older mesh generation are ignored
    ogun::HealthCheckRunner runner;
    runner.start(ogun::CheckRequest{
        controller.meshGeneration() - 1,
        std::make_shared<const ogun::Mesh>(controller.mesh()),
        {DefectCategory::Shells},
        ogun::CheckParams{}});
    runner.waitIdle();
    const std::vector<ogun::CategoryResult> finished = runner.takeFinished();
    REQUIRE(finished.size() == 1);
    REQUIRE(finished.front().meshGeneration == controller.meshGeneration() - 1);
    REQUIRE_FALSE(runner.busy());
    REQUIRE(runner.progress() == 1.0f);
}

TEST_CASE("Undo never leaves the Tool slot open without a selected row", "[app][health][undo]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    REQUIRE(controller.fixAllThinFaces()); // snapshot taken with no row selected
    controller.selectCategory(DefectCategory::Degenerate);
    REQUIRE(controller.undoLastModification());
    REQUIRE_FALSE(controller.selectedCategory());
    REQUIRE(controller.activeTool() == ogun::ToolSlot::None);
}

TEST_CASE("Intersecting faces: Auto off, Check Mesh runs it, edits and tolerance mark it Stale",
          "[app][health][intersections]") {
    ogun::AppController controller;
    REQUIRE_FALSE(controller.categoryHealth(DefectCategory::Intersections).autoCheck);
    controller.setMesh(ogun::SampleMeshes::createSelfIntersectingBoxMesh(), "box");
    REQUIRE(status(controller, DefectCategory::Intersections) == HealthStatus::Defects);

    controller.scaleMeshCoordinatesToUnit(ogun::Unit::Centimeter);
    REQUIRE(status(controller, DefectCategory::Intersections) == HealthStatus::Stale);
    controller.checkMesh();
    REQUIRE(status(controller, DefectCategory::Intersections) == HealthStatus::Defects);

    controller.setIntersectionTolerance(1.0e-3);
    REQUIRE(status(controller, DefectCategory::Intersections) == HealthStatus::Stale);
    controller.recheckCategory(DefectCategory::Intersections);
    REQUIRE(status(controller, DefectCategory::Intersections) == HealthStatus::Defects);
}

TEST_CASE("Intersection partners are highlighted and can be focused", "[app][health][intersections]") {
    ogun::AppController controller;
    controller.setMesh(ogun::SampleMeshes::createSelfIntersectingBoxMesh(), "box");
    controller.selectCategory(DefectCategory::Intersections);
    controller.focusDefect(0);
    const Eigen::Index focused = controller.focusedFaceIndex();
    const std::vector<Eigen::Index> partners(controller.focusedRelatedFaces().begin(),
                                             controller.focusedRelatedFaces().end());
    REQUIRE(focused >= 0);
    REQUIRE_FALSE(partners.empty());

    const std::vector<Eigen::Index> highlighted = controller.highlightedFaceIndices();
    REQUIRE(std::find(highlighted.begin(), highlighted.end(), focused) != highlighted.end());
    for (const Eigen::Index partner : partners) {
        REQUIRE(std::find(highlighted.begin(), highlighted.end(), partner) != highlighted.end());
    }

    REQUIRE(controller.focusRelatedFace(partners.front()));
    REQUIRE(controller.focusedFaceIndex() == partners.front());
    const auto back = controller.focusedRelatedFaces();
    REQUIRE(std::find(back.begin(), back.end(), focused) != back.end());
}

TEST_CASE("The self-intersecting box example is watertight and only intersects itself",
          "[app][health][intersections]") {
    ogun::AppController controller;
    REQUIRE_FALSE(ogun::AppController::resolveExamplePath("self_intersecting_box.stl").empty());
    REQUIRE(controller.loadSampleFile("self_intersecting_box.stl"));
    REQUIRE(controller.currentFilePath().empty()); // samples are never overwritten by Save
    REQUIRE(controller.watertightState() == ogun::WatertightState::Watertight);
    for (const DefectCategory category : {DefectCategory::OpenEdges, DefectCategory::NonManifoldEdges,
                                          DefectCategory::NonManifoldVertices, DefectCategory::FlippedNormals,
                                          DefectCategory::ThinFaces, DefectCategory::Degenerate,
                                          DefectCategory::Duplicates}) {
        INFO(ogun::categoryInfo(category).name);
        REQUIRE(status(controller, category) == HealthStatus::Ok);
    }
    const auto& intersections = controller.categoryHealth(DefectCategory::Intersections).result;
    REQUIRE(intersections.has_value());
    const ogun::CategoryResult expected = ogun::runCategoryCheck(
        DefectCategory::Intersections, ogun::SampleMeshes::createSelfIntersectingBoxMesh(), ogun::CheckParams{});
    REQUIRE(intersections->size() == expected.size());
    REQUIRE_FALSE(controller.loadSampleFile("no_such_example.stl"));
}
