#include "app/AppController.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <vector>

using ogun::DefectCategory;
using ogun::GlobalRepairMethod;
using ogun::HoleFillMethod;
using ogun::HoleShape;

namespace {

/// Loads File > Load Sample > "Holes" (four shells, one hole each) and opens the Open edges row.
void loadHolesSample(ogun::AppController& controller) {
    controller.setMesh(ogun::separateTouchingContacts(ogun::SampleMeshes::createHolesSampleMesh()), "holes");
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 4); // one row per hole
    controller.selectCategory(DefectCategory::OpenEdges);
}

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

/// Focuses the first hole with `shape`. Returns false if none.
bool focusHole(ogun::AppController& controller, HoleShape shape) {
    const ogun::CategoryResult& result = *controller.browser().result();
    for (std::size_t e = 0; e < result.size(); ++e) {
        if (result.elements[e].subtype == static_cast<int>(shape)) {
            controller.focusDefect(e);
            return controller.holeInspectorActive();
        }
    }
    return false;
}

} // namespace

TEST_CASE("Hole Inspector: recommended fill per shape, live preview, commit and undo", "[app][holes][repair][undo]") {
    ogun::AppController controller;
    loadHolesSample(controller);
    const ogun::Mesh before = controller.mesh();
    REQUIRE(controller.holeCountInScope() == 4);

    REQUIRE(focusHole(controller, HoleShape::Crack));
    REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::CloseCrack);
    REQUIRE(controller.holePreview().valid);
    REQUIRE(controller.holePreview().openEdgesLeft == 0);
    controller.setSelectedHoleMethod(HoleFillMethod::Flat);
    REQUIRE_FALSE(controller.holePreview().valid); // no triangle fits a zero-width gap
    REQUIRE(controller.holePreview().statusMessage.find("Close Crack") != std::string::npos);
    REQUIRE_FALSE(controller.repairPreviewShown());

    REQUIRE(focusHole(controller, HoleShape::Flat));
    REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::Flat);

    REQUIRE(focusHole(controller, HoleShape::Curved));
    const ogun::HoleLoop hole = *controller.focusedHole();
    REQUIRE(hole.edgeCount() == 24);
    REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::Smooth);
    REQUIRE(controller.holePreview().valid);
    REQUIRE(controller.holePreview().verticesAdded > 0);
    REQUIRE(controller.repairPreviewShown());
    REQUIRE(controller.previewPatchMesh().numFaces() == static_cast<Eigen::Index>(controller.holePreview().facesAdded));
    REQUIRE(controller.previewSourceFaces().empty()); // a fill removes nothing
    REQUIRE(controller.mesh().F == before.F);          // previews never touch the mesh
    // The whole rim is highlighted (and framed), not just the focused edge's face
    const std::vector<Eigen::Index> highlighted = controller.highlightedFaceIndices();
    for (const Eigen::Index f : hole.faces) {
        REQUIRE(std::binary_search(highlighted.begin(), highlighted.end(), f));
    }
    for (const HoleFillMethod method : {HoleFillMethod::Flat, HoleFillMethod::Refine, HoleFillMethod::Smooth}) {
        controller.setSelectedHoleMethod(method);
        INFO(ogun::holeFillMethodLabel(method));
        REQUIRE(controller.holePreview().method == method);
        REQUIRE(controller.holePreview().valid);
        REQUIRE(controller.holePreview().newCrossings == 0);
    }

    controller.setSelectedHoleMethod(HoleFillMethod::Smooth);
    const std::size_t number = controller.focusedHoleNumber();
    REQUIRE(controller.commitHoleFill());
    REQUIRE(controller.lastUndoActionLabel() == "Hole Fill (Fill Smooth, hole " + std::to_string(number) + ")");
    REQUIRE(controller.isModified());
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 3);
    REQUIRE(controller.mesh().F.topRows(before.numFaces()) == before.F); // the fill is appended

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.holeInspectorActive());
    REQUIRE(controller.focusedHole()->edgeCount() == 24);
    REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::Smooth);
}

TEST_CASE("Hole Inspector frames a hole looking into it along its outward normal", "[app][holes][camera]") {
    ogun::AppController controller;
    loadHolesSample(controller);
    REQUIRE(focusHole(controller, HoleShape::Flat));
    const ogun::HoleLoop& hole = *controller.focusedHole();
    const ogun::CameraFocusTarget plus = controller.computeCameraFocusTarget(Eigen::Vector3d(0.0, 0.0, -1.0),
                                                                             ogun::FocusNormalSide::PositiveNormal);
    REQUIRE(plus.valid);
    REQUIRE(plus.normalSideSign == 1);
    REQUIRE((plus.cameraPosition - plus.targetCenter).normalized().dot(hole.normal) > 0.999);
    const ogun::CameraFocusTarget minus = controller.computeCameraFocusTarget(Eigen::Vector3d(0.0, 0.0, -1.0),
                                                                              ogun::FocusNormalSide::NegativeNormal);
    REQUIRE((minus.cameraPosition - minus.targetCenter).normalized().dot(hole.normal) < -0.999);
}

TEST_CASE("Committing a fill that crosses other faces is refused", "[app][holes][repair]") {
    // Open cylinder with a wide plate just above it: the smooth dome would pass through the plate
    ogun::Mesh mesh = ogun::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0);
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (mesh.N(f, 2) < 0.99 || mesh.V(mesh.F(f, 0), 2) < 9.99) {
            keep.push_back(f);
        }
    }
    mesh = ogun::extractFaces(mesh, keep);
    ogun::Mesh plate = ogun::SampleMeshes::createCubeMesh(14.0);
    plate.V.col(0).array() -= 7.0;
    plate.V.col(1).array() -= 7.0;
    plate.V.col(2) = plate.V.col(2) * (0.2 / 14.0);
    plate.V.col(2).array() += 10.5;
    ogun::appendMesh(mesh, plate);

    ogun::AppController controller;
    controller.setMesh(std::move(mesh), "cylinder and plate");
    controller.selectCategory(DefectCategory::OpenEdges);
    controller.focusDefect(0);
    REQUIRE(controller.holeInspectorActive());
    REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::Flat);
    REQUIRE(controller.holePreview().newCrossings == 0);
    controller.setSelectedHoleMethod(HoleFillMethod::Smooth);
    REQUIRE(controller.holePreview().valid);
    REQUIRE(controller.holePreview().newCrossings > 0);
    REQUIRE_FALSE(controller.commitHoleFill());
    REQUIRE_FALSE(controller.canUndo());
    REQUIRE(controller.lastRepairStatusMessage().find("Try another method") != std::string::npos);
}

TEST_CASE("Fix Selected fills the selected holes in one undo step", "[app][holes][repair][undo]") {
    ogun::AppController controller;
    loadHolesSample(controller);
    const ogun::Mesh before = controller.mesh();

    // One selected row fills its hole, and only that hole
    controller.setSubtypeFilter(static_cast<int>(HoleShape::Curved));
    controller.clickDefectRow(0, false, false);
    REQUIRE(controller.browser().selectedCount() == 1);
    REQUIRE(controller.fixSelectedHoles());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Holes (1 hole)");
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 3);
    REQUIRE(controller.undoLastModification());

    controller.setSubtypeFilter(-1);
    controller.selectAllVisibleDefects();
    REQUIRE(controller.fixSelectedHoles());
    REQUIRE(controller.lastUndoActionLabel() == "Fix Selected Holes (4 holes)");
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.watertightState() == ogun::WatertightState::Watertight);
    REQUIRE(controller.shells().size() == 4);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 4);
}

TEST_CASE("Each of the bunny's holes is a row: step through them or click a rim face", "[app][holes]") {
    ogun::AppController controller;
    controller.setMesh(ogun::loadStl(examplePath("stanford_bunny.stl")), "bunny");
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 5);
    controller.selectCategory(DefectCategory::OpenEdges);
    const ogun::CategoryResult& result = *controller.browser().result();
    REQUIRE(result.infoCount == 39 + 22 + 42 + 80 + 40);

    // Next / previous visit every hole once, each opening the Inspector on that hole
    controller.focusDefect(0);
    std::vector<std::size_t> visited;
    for (int step = 0; step < 5; ++step) {
        REQUIRE(controller.holeInspectorActive());
        const std::size_t element = controller.browser().focusedElement();
        REQUIRE(controller.focusedHoleNumber() == static_cast<std::size_t>(result.elements[element].index) + 1);
        REQUIRE(controller.focusedHole()->edgeCount() == result.edgesOf(element).size());
        REQUIRE(controller.selectedHoleMethod() == HoleFillMethod::Smooth);
        REQUIRE(controller.holePreview().valid);
        visited.push_back(element);
        REQUIRE(controller.focusNextDefect(1));
    }
    std::sort(visited.begin(), visited.end());
    REQUIRE(std::adjacent_find(visited.begin(), visited.end()) == visited.end());

    // Clicking any rim face of a hole focuses that hole's row
    const std::size_t last = result.size() - 1;
    REQUIRE(controller.pickFace(result.facesOf(last).back()) == ogun::PickOutcome::FocusedDefect);
    REQUIRE(controller.browser().focusedElement() == last);
    REQUIRE(controller.focusedHoleNumber() == 5);
}

TEST_CASE("Repair All Holes: compare fills and rebuilds, apply and undo", "[app][holes][repair][undo][global]") {
    ogun::AppController controller;
    loadHolesSample(controller);
    const ogun::Mesh before = controller.mesh();

    controller.openRepairAllHoles();
    REQUIRE(controller.activeTool() == ogun::ToolSlot::RepairAllHoles);
    REQUIRE(controller.repairAllOpen());
    REQUIRE(controller.globalRepairTarget() == DefectCategory::OpenEdges);
    REQUIRE(controller.globalRepairMethod() == GlobalRepairMethod::FillHoles);

    controller.previewGlobalRepair();
    const ogun::RepairCandidate* shown = controller.shownGlobalCandidate();
    REQUIRE(shown != nullptr);
    REQUIRE(shown->success);
    REQUIRE(shown->metrics.watertight());
    REQUIRE(shown->metrics.openEdgesBefore == 62);
    REQUIRE(controller.globalPreviewShown());

    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.windingResolution = 32;
    params.alphaWrapDetail = 20.0;
    controller.setGlobalRepairParams(params);
    controller.compareAllGlobalRepairs();
    REQUIRE(controller.globalCandidates().size() == ogun::kHoleRepairMethods.size());
    for (const ogun::RepairCandidate& c : controller.globalCandidates()) {
        INFO(ogun::globalRepairMethodLabel(c.method));
        REQUIRE(c.success);
        REQUIRE(c.metrics.watertight());
    }

    // A forced method for every hole: Fill Flat cannot close the crack
    params.holeMethod = HoleFillMethod::Flat;
    controller.setGlobalRepairParams(params);
    controller.setGlobalRepairMethod(GlobalRepairMethod::FillHoles);
    controller.previewGlobalRepair();
    REQUIRE(controller.shownGlobalCandidate()->metrics.openEdgesAfter == 6);

    params.holeMethod = HoleFillMethod::None;
    controller.setGlobalRepairParams(params);
    controller.previewGlobalRepair();
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Repair All Holes (Fill each hole)");
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 0);
    REQUIRE(controller.activeTool() == ogun::ToolSlot::RepairAllHoles);

    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);

    // The intersection operator still opens with its own methods
    controller.openRepairAllIntersections();
    REQUIRE(controller.globalRepairTarget() == DefectCategory::Intersections);
    REQUIRE(controller.globalRepairMethod() == GlobalRepairMethod::PatchIslands);
    REQUIRE(controller.globalCandidates().empty());
}

TEST_CASE("Hole counts and Fill each hole follow the shell scope", "[app][holes][shells]") {
    ogun::AppController controller;
    loadHolesSample(controller);
    REQUIRE(controller.shells().size() == 4);
    // Select the sphere (its box spans x in [12, 16])
    std::size_t sphere = 0;
    for (std::size_t s = 0; s < controller.shells().size(); ++s) {
        if (controller.shells().shell(s).bbox.min.x() > 11.0 && controller.shells().shell(s).bbox.max.x() < 17.0) {
            sphere = s;
        }
    }
    controller.clickShell(sphere, false, false, {});
    REQUIRE(controller.holeCountInScope() == 1);
    REQUIRE(controller.rowCount(DefectCategory::OpenEdges) == 1);

    controller.openRepairAllHoles();
    controller.previewGlobalRepair();
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(ogun::checkOpenEdges(controller.mesh()).size() == 3); // the other holes stay open
}
