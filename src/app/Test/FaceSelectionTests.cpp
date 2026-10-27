#include "app/AppController.hpp"
#include "app/FaceSelection.hpp"
#include "io/SampleMeshes.hpp"

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
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

TEST_CASE("Merge Vertices uses 0.05 mm by default, applies and undoes", "[app][selection][merge][undo]") {
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
    REQUIRE(controller.globalRepairParams().mergeDistanceMm == 0.05);
    ogun::GlobalRepairParams params = controller.globalRepairParams();
    params.mergeDistanceMm = 100.0; // the slider's range is 0.0001 to 10 mm
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().mergeDistanceMm == ogun::settings::kMaxMergeDistanceMm);
    params.mergeDistanceMm = 0.0;
    controller.setGlobalRepairParams(params);
    REQUIRE(controller.globalRepairParams().mergeDistanceMm == ogun::settings::kMinMergeDistanceMm);
    params.mergeDistanceMm = 0.05;
    controller.setGlobalRepairParams(params);
    controller.previewGlobalRepair();
    const ogun::RepairCandidate* c = controller.shownGlobalCandidate();
    REQUIRE(c != nullptr);
    REQUIRE(c->success);
    REQUIRE(c->mesh.numFaces() == before.numFaces() - 2);
    REQUIRE(c->metrics.watertight());

    REQUIRE(controller.applyGlobalRepair());
    REQUIRE(controller.lastUndoActionLabel() == "Merge Vertices");
    REQUIRE(controller.mesh().numFaces() == before.numFaces() - 2);
    REQUIRE(controller.undoLastModification());
    REQUIRE(controller.mesh().F == before.F);
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
    REQUIRE(selected.overlayEdges.size() == 9);
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
