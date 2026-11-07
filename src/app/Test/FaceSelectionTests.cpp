#include "app/AppController.hpp"
#include "app/FaceSelection.hpp"
#include "io/SampleMeshes.hpp"

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

using ogun::GlobalRepairMethod;
using ogun::ToolSlot;

namespace {

/// An OpenGL-style camera at `eye` looking at `target` (y up), 800 x 600 points.
ogun::ScreenView makeView(const Eigen::Vector3d& eye, const Eigen::Vector3d& target, bool orthographic = false) {
    const Eigen::Vector3d f = (target - eye).normalized();
    const Eigen::Vector3d s = f.cross(Eigen::Vector3d::UnitY()).normalized();
    const Eigen::Vector3d u = s.cross(f);
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view.block<1, 3>(0, 0) = s.transpose();
    view.block<1, 3>(1, 0) = u.transpose();
    view.block<1, 3>(2, 0) = -f.transpose();
    view(0, 3) = -s.dot(eye);
    view(1, 3) = -u.dot(eye);
    view(2, 3) = f.dot(eye);

    const double width = 800.0;
    const double height = 600.0;
    const double aspect = width / height;
    const double zn = 0.1;
    const double zf = 100.0;
    Eigen::Matrix4d proj = Eigen::Matrix4d::Zero();
    if (orthographic) {
        const double top = 10.0;
        proj(0, 0) = 1.0 / (top * aspect);
        proj(1, 1) = 1.0 / top;
        proj(2, 2) = -2.0 / (zf - zn);
        proj(2, 3) = -(zf + zn) / (zf - zn);
        proj(3, 3) = 1.0;
    } else {
        const double fy = 1.0 / std::tan(0.5 * 60.0 * std::numbers::pi / 180.0);
        proj(0, 0) = fy / aspect;
        proj(1, 1) = fy;
        proj(2, 2) = (zf + zn) / (zn - zf);
        proj(2, 3) = 2.0 * zf * zn / (zn - zf);
        proj(3, 2) = -1.0;
    }
    ogun::ScreenView v;
    v.viewProjection = proj * view;
    v.cameraPosition = eye;
    v.viewDirection = f;
    v.orthographic = orthographic;
    v.width = width;
    v.height = height;
    return v;
}

/// The 4 x 4 grid cube [0, 8]^3 (32 faces per side) seen from straight above its top.
ogun::Mesh gridCube() {
    return ogun::SampleMeshes::createGridCubeMesh(4, 8.0);
}

ogun::ScreenView topView(bool orthographic = false) {
    return makeView(Eigen::Vector3d(4.0, 4.0, 30.0), Eigen::Vector3d(4.0, 4.0, 4.0), orthographic);
}

bool allOnTop(const ogun::Mesh& mesh, const std::vector<Eigen::Index>& faces) {
    return std::all_of(faces.begin(), faces.end(), [&](Eigen::Index f) { return mesh.N(f, 2) > 0.99; });
}

const Eigen::Vector2d kScreenMin(0.0, 0.0);
const Eigen::Vector2d kScreenMax(800.0, 600.0);

} // namespace

TEST_CASE("Projection puts the target at the screen centre and rejects points behind the eye", "[app][selection]") {
    const ogun::ScreenView view = topView();
    const auto centre = view.project(Eigen::Vector3d(4.0, 4.0, 4.0));
    REQUIRE(centre);
    REQUIRE(std::abs(centre->x() - 400.0) < 1e-9);
    REQUIRE(std::abs(centre->y() - 300.0) < 1e-9);
    // +y in the world is up on screen (smaller y)
    REQUIRE(view.project(Eigen::Vector3d(4.0, 6.0, 4.0))->y() < 300.0);
    REQUIRE_FALSE(view.project(Eigen::Vector3d(4.0, 4.0, 40.0)));
}

TEST_CASE("Box select picks only the faces you can see unless selecting through", "[app][selection]") {
    const ogun::Mesh mesh = gridCube();
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    for (const bool ortho : {false, true}) {
        CAPTURE(ortho);
        const ogun::ScreenView view = topView(ortho);
        const std::vector<Eigen::Index> seen = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, &visibility);
        REQUIRE(seen.size() == 32);
        REQUIRE(allOnTop(mesh, seen));
        const std::vector<Eigen::Index> through = ogun::facesInRectangle(mesh, view, kScreenMax, kScreenMin, nullptr);
        REQUIRE(through.size() == static_cast<std::size_t>(mesh.numFaces()));
    }
    // A box around the left half of the top
    const ogun::ScreenView view = topView();
    const Eigen::Vector2d left = *view.project(Eigen::Vector3d(0.0, 8.0, 8.0));
    const Eigen::Vector2d middle = *view.project(Eigen::Vector3d(4.0, 0.0, 8.0));
    const std::vector<Eigen::Index> half = ogun::facesInRectangle(mesh, view, left, middle, &visibility);
    REQUIRE(half.size() == 16);
    REQUIRE(allOnTop(mesh, half));
}

TEST_CASE("Box select sees the faces of a tiny mesh (the 0.15 mm bunny)", "[app][selection]") {
    // libigl's ray-triangle epsilon is absolute: at this size every ray used to miss
    const double s = 1e-4;
    ogun::Mesh mesh = gridCube();
    mesh.V *= s;
    mesh.recomputeNormals();
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    const ogun::ScreenView view = makeView(Eigen::Vector3d(4.0, 4.0, 30.0) * s, Eigen::Vector3d(4.0, 4.0, 4.0) * s);
    const std::vector<Eigen::Index> seen = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, &visibility);
    REQUIRE(seen.size() == 32);
    REQUIRE(allOnTop(mesh, seen));
}

TEST_CASE("Shift+click finds the face under the mouse with a ray, at any scale", "[app][selection]") {
    for (const double s : {1.0, 1e-4}) {
        CAPTURE(s);
        ogun::Mesh mesh = gridCube();
        mesh.V *= s;
        mesh.recomputeNormals();
        ogun::FaceVisibility visibility;
        visibility.build(mesh);
        const ogun::ScreenView view = makeView(Eigen::Vector3d(4.0, 4.0, 30.0) * s, Eigen::Vector3d(4.0, 4.0, 4.0) * s);
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            if (mesh.N(f, 2) < 0.99) {
                continue;
            }
            const Eigen::Vector3d c = (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
            const std::optional<Eigen::Index> hit = visibility.faceAt(mesh, view, *view.project(c));
            REQUIRE(hit);
            REQUIRE(*hit == f);
        }
        REQUIRE_FALSE(visibility.faceAt(mesh, view, Eigen::Vector2d(1.0, 1.0))); // the background
    }

    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    for (const bool ortho : {false, true}) {
        CAPTURE(ortho);
        const auto face = controller.faceAtScreenPoint(topView(ortho), Eigen::Vector2d(400.0, 290.0));
        REQUIRE(face);
        REQUIRE(controller.mesh().N(*face, 2) > 0.99);
    }
}

TEST_CASE("Faces of hidden shells are neither selected, nor block, nor edited", "[app][selection]") {
    // Cube A [0, 8]^3 and cube B above it (nearer the top view's eye)
    ogun::Mesh mesh = gridCube();
    ogun::Mesh above = gridCube();
    above.V.col(2).array() += 10.0;
    ogun::appendMesh(mesh, above);
    mesh.recomputeNormals();
    ogun::AppController controller;
    controller.setMesh(mesh, "two cubes");
    const auto isA = [](Eigen::Index f) { return f < 192; };
    const std::size_t shellB = static_cast<std::size_t>(controller.shells().shellOfFace(192));

    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32); // B's top hides A
    controller.selectAllFaces();
    REQUIRE(controller.selectedFaceCount() == 384);

    controller.setShellVisible(shellB, false);
    controller.openDecimate();
    REQUIRE(controller.operatorScopeFaceCount() == 192); // B's selected faces are out of the scope
    REQUIRE(controller.operatorScopeText() == "the 192 selected faces");
    controller.clearFaceSelection();
    controller.selectAllFaces();
    REQUIRE(controller.selectedFaceCount() == 192);
    controller.clearFaceSelection();

    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32); // now A's top
    for (Eigen::Index f = 0; f < 384; ++f) {
        if (controller.faceSelected(f)) {
            REQUIRE(isA(f));
            REQUIRE(controller.mesh().N(f, 2) > 0.99);
        }
    }
    const auto face = controller.faceAtScreenPoint(topView(), Eigen::Vector2d(400.0, 290.0));
    REQUIRE(face);
    REQUIRE(isA(*face));
    controller.setSelectThrough(true);
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 160); // the rest of A only
}

TEST_CASE("Circle select takes the visible faces under the brush", "[app][selection]") {
    const ogun::Mesh mesh = gridCube();
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    const ogun::ScreenView view = topView();
    const Eigen::Vector2d centre = *view.project(Eigen::Vector3d(4.0, 4.0, 8.0));
    // One cell is 2 units: a brush the size of one cell catches the faces around the centre vertex only
    const double cell = (*view.project(Eigen::Vector3d(6.0, 4.0, 8.0)) - centre).norm();
    const std::vector<Eigen::Index> small = ogun::facesInStroke(mesh, view, centre, centre, 0.8 * cell, &visibility);
    REQUIRE(small.size() >= 4);
    REQUIRE(small.size() <= 8);
    REQUIRE(allOnTop(mesh, small));
    REQUIRE(ogun::facesInStroke(mesh, view, centre, centre, 0.8 * cell, nullptr).size() > small.size()); // the bottom too

    // A fast stroke covers the whole path between two brush positions, not just its ends
    const Eigen::Vector2d left = *view.project(Eigen::Vector3d(1.0, 4.0, 8.0));
    const Eigen::Vector2d right = *view.project(Eigen::Vector3d(7.0, 4.0, 8.0));
    const std::vector<Eigen::Index> stroke = ogun::facesInStroke(mesh, view, left, right, 0.4 * cell, &visibility);
    const std::size_t ends = ogun::facesInStroke(mesh, view, left, left, 0.4 * cell, &visibility).size() +
                             ogun::facesInStroke(mesh, view, right, right, 0.4 * cell, &visibility).size();
    REQUIRE(stroke.size() > ends);
    REQUIRE(allOnTop(mesh, stroke));
}

TEST_CASE("Face selection: toggle, select all, Esc and mesh edits clear it", "[app][selection]") {
    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    REQUIRE(controller.selectedFaceCount() == 0);
    controller.toggleFaceSelected(3);
    REQUIRE(controller.faceSelected(3));
    REQUIRE(controller.selectedFaceCount() == 1);
    controller.toggleFaceSelected(3);
    REQUIRE(controller.selectedFaceCount() == 0);

    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32);
    REQUIRE(controller.selectedFaceCount() == 32);
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 0); // already selected
    controller.setGhostMeshEnabled(true); // Ghost selects through, as Blender's X-ray
    REQUIRE(controller.selectsThrough());
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 160);
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, false) == 192);
    controller.setGhostMeshEnabled(false);
    REQUIRE_FALSE(controller.selectsThrough());
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32);
    controller.setSelectThrough(true);
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 160);
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, false) == 192); // deselect
    REQUIRE(controller.selectedFaceCount() == 0);

    // The selection is painted over the base color
    controller.selectAllFaces();
    REQUIRE(controller.selectedFaceCount() == static_cast<std::size_t>(controller.mesh().numFaces()));
    const ogun::HighlightOverlayGeometry overlay = controller.buildHighlightOverlay();
    REQUIRE(overlay.faceColors[0].isApprox(ogun::rgbFromArray(ogun::settings::kDefaultFaceSelectionColor)));
    REQUIRE(controller.escape()); // the face selection is one of Esc's stages
    REQUIRE(controller.selectedFaceCount() == 0);

    controller.selectAllFaces();
    controller.setMesh(gridCube(), "again");
    REQUIRE(controller.selectedFaceCount() == 0);
}

TEST_CASE("Decimate works on the selected faces only, applies and undoes", "[app][selection][decimate][undo]") {
    ogun::AppController controller;
    controller.setMesh(ogun::SampleMeshes::createGridCubeMesh(8, 8.0), "grid cube");
    const ogun::Mesh before = controller.mesh();

    controller.openDecimate();
    REQUIRE(controller.activeTool() == ToolSlot::Decimate);
    REQUIRE(controller.globalRepairMethod() == GlobalRepairMethod::Decimate);
    REQUIRE(controller.globalRepairMethods().size() == 1);
    REQUIRE(controller.operatorScopeText() == "the whole mesh");
    REQUIRE(controller.operatorScopeFaceCount() == static_cast<std::size_t>(before.numFaces()));

    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 128);
    REQUIRE(controller.operatorScopeText() == "the 128 selected faces");
    REQUIRE(controller.operatorScopeFaceCount() == 128);
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.decimateRatio = 0.25;
    controller.setGlobalRepairParams(params);
    controller.previewGlobalRepair();
    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->metrics.watertight());
    REQUIRE(c->mesh.numFaces() < before.numFaces());
    for (const Eigen::Index f : c->removedFaces) {
        REQUIRE(before.N(f, 2) > 0.99); // only top faces were replaced
    }

    // A preview made on another selection is dropped
    controller.toggleFaceSelected(0);
    REQUIRE(controller.shownGlobalCandidate() == nullptr);
    REQUIRE(controller.globalCandidates().empty());
    controller.toggleFaceSelected(0);
    controller.previewGlobalRepair();
    c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Decimate");
    REQUIRE(controller.selectedFaceCount() == 0); // face numbers changed
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("Merge Vertices uses 0.05 by default, applies and undoes", "[app][selection][merge][undo]") {
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(4, 8.0);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    mesh.V.row(b) = mesh.V.row(a) + 0.02 * (mesh.V.row(b) - mesh.V.row(a)).normalized();
    mesh.recomputeNormals();
    ogun::AppController controller;
    controller.setMesh(mesh, "short edge");
    const ogun::Mesh before = controller.mesh();

    controller.openMergeVertices();
    REQUIRE(controller.activeTool() == ToolSlot::MergeVertices);
    REQUIRE(controller.globalRepairParams().mergeDistance == 0.05);
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.mergeDistance = 100.0; // the slider's range is 0.0001 to 10
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().mergeDistance == ogun::settings::kMaxMergeDistance);
    params.mergeDistance = 0.0;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().mergeDistance == ogun::settings::kMinMergeDistance);
    params.mergeDistance = 0.05;
    controller.setGlobalRepairParams(params);
    controller.previewGlobalRepair();
    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->mesh.numFaces() == before.numFaces() - 2);
    REQUIRE(c->metrics.watertight());

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Merge by Distance");
    REQUIRE(controller.mesh().numFaces() == before.numFaces() - 2);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("Mesh > Weld welds open edges across cracks and is undoable", "[app][weld]") {
    ogun::Mesh sheets;
    sheets.V.resize(8, 3);
    sheets.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1.02, 0, 0, 2, 0, 0, 2, 1, 0, 1.02, 1, 0;
    sheets.F.resize(4, 3);
    sheets.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    sheets.recomputeNormals();

    ogun::AppController controller;
    controller.setMesh(sheets, "open sheets");
    const ogun::Mesh before = controller.mesh();

    controller.openWeld();
    REQUIRE(controller.activeTool() == ToolSlot::Weld);
    REQUIRE(controller.globalRepairMethod() == ogun::GlobalRepairMethod::Weld);
    REQUIRE(controller.globalRepairParams().weldDistance == 0.05);

    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.weldDistance = 100.0;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().weldDistance == ogun::settings::kMaxWeldDistance);
    params.weldDistance = 0.05;
    controller.setGlobalRepairParams(params);

    controller.previewGlobalRepair();
    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->mesh.numVertices() == 6);

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Weld");
    REQUIRE(controller.mesh().numVertices() == 6);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
}

TEST_CASE("Loading 'Cube with a visible crack' sample in AppController and healing with Weld", "[app][weld][crack]") {
    ogun::AppController controller;
    const ogun::Mesh mesh = ogun::SampleMeshes::createVisibleCrackCubeMesh(0.04);
    controller.setMesh(mesh, "Cube with a visible crack");
    const ogun::Mesh before = controller.mesh();

    // Verify initial health: 1 shell, 6 open edges
    REQUIRE(controller.shells().size() == 1);
    const ogun::CategoryResult openEdgesBefore = ogun::checkOpenEdges(controller.mesh());
    REQUIRE(openEdgesBefore.infoCount == 6);

    controller.openWeld();
    REQUIRE(controller.activeTool() == ToolSlot::Weld);
    REQUIRE(controller.globalRepairMethod() == ogun::GlobalRepairMethod::Weld);

    // Preview weld with default distance (0.05 mm)
    controller.previewGlobalRepair();
    const ogun::RepairCandidate* candidate = controller.shownGlobalCandidate();
    REQUIRE(candidate != nullptr);
    REQUIRE(candidate->success);
    REQUIRE(ogun::checkOpenEdges(candidate->mesh).empty());

    // Apply and verify
    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Weld");
    REQUIRE(ogun::checkOpenEdges(controller.mesh()).empty());
    REQUIRE(controller.shells().size() == 1);

    // Undo restores the crack
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().V == before.V);
    REQUIRE(controller.mesh().F == before.F);
    REQUIRE(ogun::checkOpenEdges(controller.mesh()).infoCount == 6);
}

TEST_CASE("The face selection color can be changed and reset", "[app][selection]") {
    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    controller.toggleFaceSelected(5);
    ogun::OverlayAppearance appearance = controller.overlayAppearance();
    REQUIRE(appearance.faceSelectionColor.isApprox(ogun::rgbFromArray(ogun::settings::kDefaultFaceSelectionColor)));
    appearance.faceSelectionColor = Eigen::Vector3d(0.1, 0.9, 0.2);
    controller.setOverlayAppearance(appearance);
    REQUIRE(controller.overlayAppearance().faceSelectionColor.isApprox(Eigen::Vector3d(0.1, 0.9, 0.2)));
    REQUIRE(controller.buildHighlightOverlay().faceColors[5].isApprox(Eigen::Vector3d(0.1, 0.9, 0.2)));
    controller.resetOverlayAppearanceToDefaults();
    REQUIRE(controller.buildHighlightOverlay().faceColors[5].isApprox(
        ogun::rgbFromArray(ogun::settings::kDefaultFaceSelectionColor)));
}

TEST_CASE("Selected faces show the selection color over defect tints", "[app][selection]") {
    ogun::AppController controller;
    controller.loadSampleDefectiveMesh();
    controller.selectCategory(ogun::DefectCategory::ThinFaces);
    const Eigen::Index tinted = ogun::SampleMeshes::kZeroAreaFaceIndex;
    const Eigen::Index focused = ogun::SampleMeshes::kNeedleSliverFaceIndex;
    const ogun::OverlayAppearance& appearance = controller.overlayAppearance();
    const auto colorOf = [&](Eigen::Index f) { return controller.buildHighlightOverlay().faceColors[static_cast<std::size_t>(f)]; };

    // A tinted defect face takes the selection color, and the tint again once deselected
    REQUIRE(colorOf(tinted).isApprox(appearance.categoryTintColor));
    controller.toggleFaceSelected(tinted);
    REQUIRE(colorOf(tinted).isApprox(appearance.faceSelectionColor));
    controller.toggleFaceSelected(tinted);
    REQUIRE(colorOf(tinted).isApprox(appearance.categoryTintColor));

    // The focused defect face too: its overlay triangle steps aside, the outline stays
    REQUIRE(controller.pickFace(focused) == ogun::PickOutcome::FocusedDefect);
    REQUIRE(controller.buildHighlightOverlay().overlayFaces.size() == 1);
    controller.toggleFaceSelected(focused);
    const ogun::HighlightOverlayGeometry selected = controller.buildHighlightOverlay();
    REQUIRE(selected.faceColors[static_cast<std::size_t>(focused)].isApprox(appearance.faceSelectionColor));
    REQUIRE(selected.overlayFaces.empty());
    REQUIRE(selected.overlayEdges.size() == 6); // tubes of both thin faces of the row
    controller.toggleFaceSelected(focused);
    REQUIRE(colorOf(focused).isApprox(appearance.selectedFaces.color));
    REQUIRE(controller.buildHighlightOverlay().overlayFaces.size() == 1);
}

TEST_CASE("Delete Selected Faces removes them as one undo step", "[app][selection][undo]") {
    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    const ogun::Mesh before = controller.mesh();
    REQUIRE_FALSE(controller.deleteSelectedFaces()); // nothing selected

    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32);
    REQUIRE(controller.deleteSelectedFaces());
    REQUIRE(controller.mesh().numFaces() == 160);
    REQUIRE(controller.lastUndoActionLabel() == "Delete 32 Faces");
    REQUIRE(controller.selectedFaceCount() == 0);
    for (Eigen::Index f = 0; f < controller.mesh().numFaces(); ++f) {
        REQUIRE(controller.mesh().N(f, 2) < 0.99); // the top is gone
    }
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);

    controller.selectAllFaces();
    REQUIRE_FALSE(controller.deleteSelectedFaces()); // would leave no mesh
    REQUIRE(controller.mesh().numFaces() == 192);
}

TEST_CASE("Delete Selected Faces leaves hidden shells alone", "[app][selection]") {
    ogun::Mesh mesh = gridCube();
    ogun::Mesh above = gridCube();
    above.V.col(2).array() += 10.0;
    ogun::appendMesh(mesh, above);
    mesh.recomputeNormals();
    ogun::AppController controller;
    controller.setMesh(mesh, "two cubes");
    controller.selectAllFaces();
    controller.setShellVisible(static_cast<std::size_t>(controller.shells().shellOfFace(192)), false);
    REQUIRE(controller.deleteSelectedFaces());
    REQUIRE(controller.mesh().numFaces() == 192); // only the shown cube went
    REQUIRE(controller.mesh().V.col(2).minCoeff() >= 10.0 - 1e-9);
}

TEST_CASE("Focus (.) frames the selected faces, square on, and flips on a second press", "[app][selection][camera]") {
    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    REQUIRE_FALSE(controller.faceSelectionFocused());
    REQUIRE(controller.selectFacesInRectangle(topView(), kScreenMin, kScreenMax, true) == 32); // the top, z = 8
    REQUIRE(controller.faceSelectionFocused());

    const Eigen::Vector3d sideways(1.0, 0.0, 0.0);
    const ogun::CameraFocusTarget first = controller.focusOnSelection(ogun::FocusNormalSide::AutoToggleNormal, sideways);
    REQUIRE(first.valid);
    REQUIRE(first.targetCenter.isApprox(Eigen::Vector3d(4.0, 4.0, 8.0)));
    REQUIRE(first.normalSideSign == +1);
    REQUIRE(first.cameraPosition.z() > 8.0); // above the top, looking down at it
    REQUIRE(std::abs(first.cameraPosition.x() - 4.0) < 1e-9);
    REQUIRE(controller.orbitPivot().isApprox(first.targetCenter));
    const ogun::CameraFocusTarget second = controller.focusOnSelection(ogun::FocusNormalSide::AutoToggleNormal, sideways);
    REQUIRE(second.normalSideSign == -1);
    REQUIRE(second.cameraPosition.z() < 8.0); // from inside, under the top

    // A selection facing every way (the whole cube) keeps the view direction
    controller.selectAllFaces();
    const ogun::CameraFocusTarget all = controller.focusOnSelection(ogun::FocusNormalSide::AutoToggleNormal, sideways);
    REQUIRE(all.normalSideSign == 0);
    REQUIRE((all.targetCenter - all.cameraPosition).normalized().isApprox(sideways));

    // Clicking a face focuses it instead; selecting again gives the focus back to the selection
    controller.pickFace(0);
    REQUIRE_FALSE(controller.faceSelectionFocused());
    controller.toggleFaceSelected(1);
    REQUIRE(controller.faceSelectionFocused());
    controller.clearFaceSelection();
    REQUIRE_FALSE(controller.faceSelectionFocused());
}

namespace {

Eigen::Vector3d faceCentre(const ogun::Mesh& mesh, Eigen::Index f) {
    return (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
}

/// Keeps z <= 6: cuts the top (z = 8) off the grid cube.
const ogun::ClipPlane kCutTop{Eigen::Vector3d(4.0, 4.0, 6.0), -Eigen::Vector3d::UnitZ()};

} // namespace

TEST_CASE("Slice planes: cut-away faces are not selected, by box, brush, click or select all", "[app][selection][slice]") {
    const ogun::Mesh mesh = gridCube();
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    const auto keptCount = [&](std::span<const ogun::ClipPlane> planes) {
        std::size_t n = 0;
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            n += ogun::faceClipped(mesh, f, planes) ? 0 : 1;
        }
        return n;
    };

    for (const bool ortho : {false, true}) {
        CAPTURE(ortho);
        ogun::ScreenView view = topView(ortho);
        // No planes: the top only, as before
        REQUIRE(ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, &visibility).size() == 32);

        view.clipPlanes = {kCutTop};
        const std::vector<Eigen::Index> seen = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, &visibility);
        // The cut-away top neither counts nor hides the bottom (seen from inside) behind it
        std::size_t bottom = 0;
        for (const Eigen::Index f : seen) {
            REQUIRE(faceCentre(mesh, f).z() <= 6.0);
            bottom += mesh.N(f, 2) < -0.99 ? 1 : 0;
        }
        REQUIRE(bottom == 32);
        // Select through: every face the plane keeps
        const std::vector<Eigen::Index> through = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, nullptr);
        REQUIRE(through.size() == keptCount(view.clipPlanes));

        // The brush and a click reach the bottom through the cut-away top
        const Eigen::Vector2d centre(400.0, 290.0);
        const std::vector<Eigen::Index> brushed = ogun::facesInStroke(mesh, view, centre, centre, 60.0, &visibility);
        REQUIRE_FALSE(brushed.empty());
        for (const Eigen::Index f : brushed) {
            REQUIRE(mesh.N(f, 2) < -0.99);
        }
        const std::optional<Eigen::Index> clicked = visibility.faceAt(mesh, view, centre);
        REQUIRE(clicked);
        REQUIRE(mesh.N(*clicked, 2) < -0.99);
    }

    // The controller passes the view's planes on; Select All takes them too
    ogun::AppController controller;
    controller.setMesh(gridCube(), "grid cube");
    ogun::ScreenView view = topView();
    view.clipPlanes = {kCutTop};
    REQUIRE(controller.selectFacesInRectangle(view, kScreenMin, kScreenMax, true) > 32);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        REQUIRE((controller.faceSelected(f) ? faceCentre(mesh, f).z() <= 6.0 : true));
    }
    const auto face = controller.faceAtScreenPoint(view, Eigen::Vector2d(400.0, 290.0));
    REQUIRE(face);
    REQUIRE(mesh.N(*face, 2) < -0.99);
    controller.clearFaceSelection();
    controller.selectAllFaces(view.clipPlanes);
    REQUIRE(controller.selectedFaceCount() == keptCount(view.clipPlanes));
    controller.selectAllFaces();
    REQUIRE(controller.selectedFaceCount() == 192);
}

TEST_CASE("Slice planes: a face seen through a hole is not brushed when the plane cuts it away", "[app][selection][slice]") {
    // The grid cube with a hole in the bottom around (4, 4, 0), seen from below: through the hole the
    // brush reaches the inside of the top
    const ogun::Mesh cube = gridCube();
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
        const Eigen::Vector3d c = faceCentre(cube, f);
        if (!(cube.N(f, 2) < -0.99 && (c.head<2>() - Eigen::Vector2d(4.0, 4.0)).norm() < 2.0)) {
            keep.push_back(f);
        }
    }
    ogun::Mesh mesh = ogun::extractFaces(cube, keep);
    mesh.recomputeNormals();
    REQUIRE(mesh.numFaces() < cube.numFaces());
    ogun::FaceVisibility visibility;
    visibility.build(mesh);

    ogun::ScreenView view = makeView(Eigen::Vector3d(4.0, 4.0, -22.0), Eigen::Vector3d(4.0, 4.0, 4.0));
    const Eigen::Vector2d hole = *view.project(Eigen::Vector3d(4.0, 4.0, 0.0));
    const Eigen::Vector2d rim = *view.project(Eigen::Vector3d(7.0, 4.0, 0.0));
    const double radius = (rim - hole).norm();
    const auto topFaces = [&](const std::vector<Eigen::Index>& faces) {
        return std::count_if(faces.begin(), faces.end(), [&](Eigen::Index f) { return mesh.N(f, 2) > 0.99; });
    };

    const std::vector<Eigen::Index> plain = ogun::facesInStroke(mesh, view, hole, hole, radius, &visibility);
    REQUIRE(topFaces(plain) > 0); // without a plane the top shows through the hole

    view.clipPlanes = {kCutTop};
    for (const ogun::FaceVisibility* v : {static_cast<const ogun::FaceVisibility*>(&visibility), static_cast<const ogun::FaceVisibility*>(nullptr)}) {
        const std::vector<Eigen::Index> cut = ogun::facesInStroke(mesh, view, hole, hole, radius, v);
        REQUIRE_FALSE(cut.empty());
        REQUIRE(topFaces(cut) == 0);
        for (const Eigen::Index f : cut) {
            REQUIRE(faceCentre(mesh, f).z() <= 6.0);
        }
    }
    // The faces around the hole are still brushed
    const std::vector<Eigen::Index> cut = ogun::facesInStroke(mesh, view, hole, hole, radius, &visibility);
    REQUIRE(std::count_if(cut.begin(), cut.end(), [&](Eigen::Index f) { return mesh.N(f, 2) < -0.99; }) > 0);
    // A click in the hole hits nothing now (the top is cut away, the inside of the cube is open)
    REQUIRE(visibility.faceAt(mesh, view, hole).has_value() == false);
}

TEST_CASE("Slice planes: several planes, and faces the plane crosses go by their centre", "[app][selection][slice]") {
    const ogun::Mesh mesh = gridCube();
    // Two planes: keep z <= 6 and x <= 6
    const std::vector<ogun::ClipPlane> planes = {kCutTop, ogun::ClipPlane{Eigen::Vector3d(6.0, 0.0, 0.0), -Eigen::Vector3d::UnitX()}};
    ogun::ScreenView view = topView();
    view.clipPlanes = planes;
    const std::vector<Eigen::Index> through = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, nullptr);
    std::size_t expected = 0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c = faceCentre(mesh, f);
        expected += c.z() <= 6.0 && c.x() <= 6.0 ? 1 : 0;
    }
    REQUIRE(through.size() == expected);
    for (const Eigen::Index f : through) {
        REQUIRE(faceCentre(mesh, f).x() <= 6.0);
        REQUIRE(faceCentre(mesh, f).z() <= 6.0);
    }
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    const std::vector<Eigen::Index> seen = ogun::facesInRectangle(mesh, view, kScreenMin, kScreenMax, &visibility);
    REQUIRE_FALSE(seen.empty());
    for (const Eigen::Index f : seen) {
        REQUIRE(faceCentre(mesh, f).x() <= 6.0);
        REQUIRE(faceCentre(mesh, f).z() <= 6.0);
    }

    // z <= 5 crosses the side cells spanning z 4..6: the triangle whose centre is below 5 is kept
    const std::vector<ogun::ClipPlane> cut5 = {ogun::ClipPlane{Eigen::Vector3d(0.0, 0.0, 5.0), -Eigen::Vector3d::UnitZ()}};
    std::size_t crossedKept = 0;
    std::size_t crossedCut = 0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        double lo = 1e9;
        double hi = -1e9;
        for (int c = 0; c < 3; ++c) {
            lo = std::min(lo, mesh.V(mesh.F(f, c), 2));
            hi = std::max(hi, mesh.V(mesh.F(f, c), 2));
        }
        if (lo < 5.0 && hi > 5.0) {
            const bool cut = ogun::faceClipped(mesh, f, cut5);
            REQUIRE(cut == (faceCentre(mesh, f).z() > 5.0));
            (cut ? crossedCut : crossedKept) += 1;
        }
    }
    REQUIRE(crossedKept > 0);
    REQUIRE(crossedCut > 0);
    REQUIRE_FALSE(ogun::faceClipped(mesh, 0, {})); // no planes: nothing is cut
}

TEST_CASE("Slice planes: only the shown part of a face crossed by a plane hides what is behind it", "[app][selection][slice]") {
    // Two parallel squares facing the eye, front (z = 2) and back (z = 0). The plane keeps x + z <= 6:
    // the front is cut beyond x = 4, the back only beyond x = 6
    ogun::Mesh mesh;
    mesh.V.resize(8, 3);
    mesh.V << 0, 0, 2, 8, 0, 2, 8, 8, 2, 0, 8, 2, //
        0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0;
    mesh.F.resize(4, 3);
    mesh.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    mesh.recomputeNormals();
    ogun::FaceVisibility visibility;
    visibility.build(mesh);
    for (const bool ortho : {false, true}) {
        CAPTURE(ortho);
        ogun::ScreenView view = makeView(Eigen::Vector3d(4.0, 4.0, 30.0), Eigen::Vector3d(4.0, 4.0, 0.0), ortho);
        view.clipPlanes = {ogun::ClipPlane{Eigen::Vector3d(4.0, 0.0, 2.0), Eigen::Vector3d(-1.0, 0.0, -1.0).normalized()}};
        const auto at = [&](double x, double y) { return visibility.faceAt(mesh, view, *view.project(Eigen::Vector3d(x, y, 2.0))); };
        // Front face 1 (y > x) is crossed by the plane, its centre (2.7, 5.3) kept; face 0's centre is cut
        REQUIRE_FALSE(ogun::faceClipped(mesh, 1, view.clipPlanes));
        REQUIRE(ogun::faceClipped(mesh, 0, view.clipPlanes));
        REQUIRE(at(2.0, 6.0) == std::optional<Eigen::Index>(1)); // the shown part of face 1
        REQUIRE(at(4.5, 6.0) == std::optional<Eigen::Index>(3)); // through its cut-away part: the back
        REQUIRE(at(5.0, 2.0) == std::optional<Eigen::Index>(2)); // through the cut-away face 0
        REQUIRE_FALSE(visibility.visible(mesh, 0, view));        // cut away
        REQUIRE(visibility.visible(mesh, 1, view));
        REQUIRE_FALSE(visibility.visible(mesh, 3, view)); // behind the shown part of face 1
    }
}

TEST_CASE("Shortest edge skips zero-length edges and is at most the mean edge", "[app][selection][merge]") {
    ogun::AppController controller;
    REQUIRE(controller.shortestEdgeLength() == 0.0);
    ogun::Mesh mesh;
    mesh.V.resize(5, 3);
    mesh.V << 0, 0, 0, 2, 0, 0, 0, 1, 0, 2, 1, 0, 2, 1, 0; // vertex 4 duplicates vertex 3
    mesh.F.resize(3, 3);
    mesh.F << 0, 1, 2, 1, 3, 2, 1, 3, 4; // the last face is degenerate (zero-length edge 3-4)
    controller.setMesh(mesh, "strip");
    REQUIRE(std::abs(controller.shortestEdgeLength() - 1.0) < 1e-12);
    REQUIRE(controller.shortestEdgeLength() <= controller.meanEdgeLength());
}

TEST_CASE("Shortest edge follows the Merge scope: selected faces, else the whole mesh", "[app][selection][merge]") {
    ogun::AppController controller;
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 5, 0, 0, 9, 0, 0, 5, 4, 0;
    mesh.F.resize(2, 3);
    mesh.F << 0, 1, 2, 3, 4, 5; // a small triangle (shortest edge 1) and a large one (shortest edge 4)
    controller.setMesh(mesh, "two triangles");
    controller.openMergeVertices();
    REQUIRE(std::abs(controller.shortestEdgeLength() - 1.0) < 1e-12);
    controller.toggleFaceSelected(1);
    REQUIRE(std::abs(controller.shortestEdgeLength() - 4.0) < 1e-12);
    controller.toggleFaceSelected(1);
    REQUIRE(std::abs(controller.shortestEdgeLength() - 1.0) < 1e-12);

    // Small triangle edges 1, 1, 1.41; large one 4, 4, 5.66
    REQUIRE(controller.edgesShorterThanCount(0.5) == 0);
    REQUIRE(controller.edgesShorterThanCount(1.2) == 2);
    REQUIRE(controller.edgesShorterThanCount(4.5) == 5);
    controller.toggleFaceSelected(1);
    REQUIRE(controller.edgesShorterThanCount(4.5) == 2);
}

TEST_CASE("Faces behind an opaque preview patch are dropped from a selection, faces in front stay", "[app][selection]") {
    const ogun::Mesh mesh = gridCube();
    std::vector<Eigen::Index> all(static_cast<std::size_t>(mesh.numFaces()));
    for (std::size_t f = 0; f < all.size(); ++f) {
        all[f] = static_cast<Eigen::Index>(f);
    }
    const auto plane = [](double z) {
        ogun::Mesh patch;
        patch.V.resize(4, 3);
        patch.V << -4, -4, z, 12, -4, z, 12, 12, z, -4, 12, z;
        patch.F.resize(2, 3);
        patch.F << 0, 1, 2, 0, 2, 3;
        return patch;
    };
    for (const bool ortho : {false, true}) {
        CAPTURE(ortho);
        const ogun::ScreenView view = topView(ortho);
        // A patch between the eye and the cube hides every face
        REQUIRE(ogun::withoutFacesBehind(mesh, view, all, plane(20.0)).empty());
        // A patch below the cube is behind every face
        REQUIRE(ogun::withoutFacesBehind(mesh, view, all, plane(-5.0)).size() == all.size());
        // A patch through the middle hides only what lies below it
        const std::vector<Eigen::Index> kept = ogun::withoutFacesBehind(mesh, view, all, plane(4.0));
        REQUIRE_FALSE(kept.empty());
        REQUIRE(kept.size() < all.size());
    }
}

#include "io/StlReader.hpp"
#include <filesystem>
TEST_CASE("Circle select on the bunny never picks faces behind its hole-fill patch, even partly", "[app][selection]") {
    ogun::AppController controller;
    controller.setMesh(ogun::loadStl(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "stanford_bunny.stl"), "bunny");
    controller.selectCategory(ogun::DefectCategory::OpenEdges);
    controller.focusDefect(0);
    const ogun::Mesh& patch = controller.previewPatchMesh();
    const ogun::Mesh& mesh = controller.mesh();
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (Eigen::Index i = 0; i < patch.numFaces(); ++i) {
        c += (patch.V.row(patch.F(i,0)) + patch.V.row(patch.F(i,1)) + patch.V.row(patch.F(i,2))).transpose() / 3.0;
        n += Eigen::Vector3d(patch.V.row(patch.F(i,1)) - patch.V.row(patch.F(i,0))).cross(Eigen::Vector3d(patch.V.row(patch.F(i,2)) - patch.V.row(patch.F(i,0))));
    }
    c /= static_cast<double>(patch.numFaces()); n.normalize();
    const auto hitsBefore = [&](const Eigen::Vector3d& eye, const Eigen::Vector3d& target) {
        const Eigen::Vector3d dir = target - eye;
        for (Eigen::Index o = 0; o < patch.numFaces(); ++o) {
            const Eigen::Vector3d a = patch.V.row(patch.F(o, 0));
            const Eigen::Vector3d e1 = Eigen::Vector3d(patch.V.row(patch.F(o, 1))) - a;
            const Eigen::Vector3d e2 = Eigen::Vector3d(patch.V.row(patch.F(o, 2))) - a;
            const Eigen::Vector3d pv = dir.cross(e2); const double det = e1.dot(pv);
            if (std::abs(det) < 1e-30) continue;
            const Eigen::Vector3d tv = eye - a; const double u = tv.dot(pv) / det;
            const Eigen::Vector3d qv = tv.cross(e1); const double v = dir.dot(qv) / det; const double t = e2.dot(qv) / det;
            if (u >= 0 && v >= 0 && u + v <= 1 && t > 0 && t < 1 - 1e-3) return true;
        }
        return false;
    };
    int totalBad = 0, totalSel = 0;
    for (int k = 0; k < 8; ++k) {
        const double ang = 0.2 + 0.25 * k;
        Eigen::Vector3d tang = n.cross(Eigen::Vector3d::UnitY()).normalized();
        Eigen::Vector3d dirv = (std::cos(ang * 0.5) * n + std::sin(ang * 0.5) * (std::cos(ang) * tang + std::sin(ang) * n.cross(tang))).normalized();
        for (double sgn : {1.0, -1.0}) {
            const Eigen::Vector3d eye = c + sgn * dirv * 0.03;
            ogun::ScreenView view = makeView(eye, c, false);
            const Eigen::Vector2d ctr(400, 300);
            controller.selectFacesInStroke(view, ctr, ctr, 150.0, true);
            for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
                if (!controller.faceSelected(f)) continue;
                ++totalSel;
                const Eigen::Vector3d a = mesh.V.row(mesh.F(f,0)), b = mesh.V.row(mesh.F(f,1)), d = mesh.V.row(mesh.F(f,2));
                int hid = 0;
                for (const Eigen::Vector3d& t : std::array<Eigen::Vector3d, 3>{Eigen::Vector3d((a+b+d)/3.0), Eigen::Vector3d(0.6*a+0.2*b+0.2*d), Eigen::Vector3d(0.2*a+0.6*b+0.2*d)}) hid += hitsBefore(eye, t) ? 1 : 0;
                if (hid >= 1) {
                    ++totalBad;
                }
            }
            controller.clearFaceSelection();
        }
    }
    REQUIRE(totalSel > 0);
    REQUIRE(totalBad == 0);
}
