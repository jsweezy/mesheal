// Face selection tools in the viewport, with Blender's keys: B box select, C circle select (brush),
// Shift+click toggles a face, A selects all, Alt+A deselects all. Decimate and Merge Vertices then
// work on the selected faces.

#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>
#include <polyscope/polyscope.h>
#include <polyscope/slice_plane.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

ImU32 selectionColor(const Eigen::Vector3d& c, float alpha) {
    return ImGui::GetColorU32(ImVec4(static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()), alpha));
}

ImGuiMouseButton imguiButton(MouseInput input) noexcept {
    switch (input) {
    case MouseInput::Right: return ImGuiMouseButton_Right;
    case MouseInput::Middle: return ImGuiMouseButton_Middle;
    case MouseInput::Left:
    case MouseInput::Wheel:
    case MouseInput::None:
        break;
    }
    return ImGuiMouseButton_Left;
}

} // namespace

ScreenView PolyscopeGui::currentScreenView() const {
    ScreenView view;
    const glm::mat4 vp = polyscope::view::getCameraPerspectiveMatrix() * polyscope::view::getCameraViewMatrix();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            view.viewProjection(r, c) = static_cast<double>(vp[c][r]); // glm is column-major
        }
    }
    view.cameraPosition = currentCameraWorldPosition();
    glm::vec3 look(0.0f, 0.0f, -1.0f);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 right(1.0f, 0.0f, 0.0f);
    polyscope::view::getCameraFrame(look, up, right);
    view.viewDirection = Eigen::Vector3d(look.x, look.y, look.z).normalized();
    view.orthographic = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
    view.width = static_cast<double>(polyscope::view::windowWidth);
    view.height = static_cast<double>(polyscope::view::windowHeight);
    view.clipPlanes = currentClipPlanes();
    return view;
}

std::vector<ClipPlane> PolyscopeGui::currentClipPlanes() {
    std::vector<ClipPlane> planes;
    polyscope::SurfaceMesh* base = polyscope::hasSurfaceMesh(kSurfaceMeshName) ? polyscope::getSurfaceMesh(kSurfaceMeshName) : nullptr;
    for (const std::unique_ptr<polyscope::SlicePlane>& plane : polyscope::state::slicePlanes) {
        if (!plane->getActive() || (base != nullptr && base->getIgnoreSlicePlane(plane->name))) {
            continue;
        }
        // As SlicePlane::getCenter / getNormal: the plane's x axis is its normal, kept side positive
        const glm::mat4 t = plane->getTransform();
        const Eigen::Vector3d normal(t[0][0], t[0][1], t[0][2]);
        if (normal.norm() > 0.0) {
            planes.push_back(ClipPlane{Eigen::Vector3d(t[3][0], t[3][1], t[3][2]), normal.normalized()});
        }
    }
    return planes;
}

void PolyscopeGui::startFaceSelectMode(FaceSelectMode mode) {
    if (!controller_.hasMesh()) {
        return;
    }
    faceSelectMode_ = mode;
    boxDragging_ = false;
    lastPaint_ = Eigen::Vector2d::Constant(-1.0e9);
    controller_.postStatusMessage(mode == FaceSelectMode::Box
        ? "Box select: drag to select faces, Shift+drag to deselect. Esc or right-click cancels."
        : "Circle select: drag to paint faces, Shift+drag (or middle drag) to deselect, wheel resizes. "
          "Esc, Enter or right-click finishes.");
}

void PolyscopeGui::exitFaceSelectMode() {
    if (faceSelectMode_ == FaceSelectMode::Circle) {
        controller_.postStatusMessage("Circle select done: " + withThousands(controller_.selectedFaceCount()) + " faces selected.");
    } else if (faceSelectMode_ == FaceSelectMode::Box) {
        controller_.postStatusMessage("Box select cancelled.");
    }
    faceSelectMode_ = FaceSelectMode::None;
    boxDragging_ = false;
}

void PolyscopeGui::handleFaceSelectTool() {
    const ImGuiIO& io = ImGui::GetIO();
    const Eigen::Vector2d mouse(io.MousePos.x, io.MousePos.y);
    // The Select button of Preferences > Mouse draws (Left by default); right-click cancels unless it selects
    const MouseInput selectInput = selectToolButton(mouseMapping_);
    const ImGuiMouseButton selectButton = imguiButton(selectInput);
    if (selectInput != MouseInput::Right && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        exitFaceSelectMode();
        return;
    }
    const bool removeModifier = io.KeyShift || io.KeyCtrl || io.KeySuper;

    if (faceSelectMode_ == FaceSelectMode::Box) {
        if (!boxDragging_ && ImGui::IsMouseClicked(selectButton)) {
            boxDragging_ = true;
            boxStart_ = mouse;
            boxRemoves_ = removeModifier;
        } else if (boxDragging_ && ImGui::IsMouseReleased(selectButton)) {
            const std::size_t changed = controller_.selectFacesInRectangle(currentScreenView(), boxStart_, mouse, !boxRemoves_);
            controller_.postStatusMessage(withThousands(changed) + (boxRemoves_ ? " faces deselected; " : " faces selected; ") +
                                          withThousands(controller_.selectedFaceCount()) + " selected.");
            faceSelectMode_ = FaceSelectMode::None; // one box per B, as in Blender
            boxDragging_ = false;
            requestSync();
        }
        return;
    }

    // Circle: the wheel sizes the brush; dragging paints (or erases)
    if (io.MouseWheel != 0.0f) {
        circleRadiusPx_ = std::clamp(circleRadiusPx_ * std::pow(1.15f, io.MouseWheel), settings::kMinCircleSelectRadiusPx,
                                     settings::kMaxCircleSelectRadiusPx);
    }
    const bool paint = ImGui::IsMouseDown(selectButton);
    const bool erase = selectInput != MouseInput::Middle && ImGui::IsMouseDown(ImGuiMouseButton_Middle);
    if (!paint && !erase) {
        lastPaint_ = Eigen::Vector2d::Constant(-1.0e9);
        return;
    }
    // Only where the brush moved on (a quarter radius), so a held click does not repeat the work; the
    // whole stroke since the last paint, so a fast drag leaves no gaps
    const bool strokeStarted = lastPaint_.x() > -1.0e8;
    if (strokeStarted && (mouse - lastPaint_).norm() < 0.25 * circleRadiusPx_) {
        return;
    }
    const Eigen::Vector2d from = strokeStarted ? lastPaint_ : mouse;
    lastPaint_ = mouse;
    if (controller_.selectFacesInStroke(currentScreenView(), from, mouse, circleRadiusPx_, paint && !removeModifier) > 0) {
        requestSync();
    }
}

void PolyscopeGui::drawFaceSelectOverlay() {
    if (faceSelectMode_ == FaceSelectMode::None) {
        return;
    }
    ImDrawList* draw = ImGui::GetForegroundDrawList();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const Eigen::Vector3d& color = controller_.overlayAppearance().faceSelectionColor;
    if (faceSelectMode_ == FaceSelectMode::Circle) {
        draw->AddCircle(mouse, circleRadiusPx_, selectionColor(color, 0.9f), 48, 1.5f);
        return;
    }
    if (boxDragging_) {
        const ImVec2 start(static_cast<float>(boxStart_.x()), static_cast<float>(boxStart_.y()));
        draw->AddRectFilled(start, mouse, selectionColor(color, 0.12f));
        draw->AddRect(start, mouse, selectionColor(color, 0.9f), 0.0f, 0, 1.5f);
    } else {
        // Crosshair until the drag starts
        draw->AddLine(ImVec2(mouse.x - 12.0f, mouse.y), ImVec2(mouse.x + 12.0f, mouse.y), selectionColor(color, 0.9f));
        draw->AddLine(ImVec2(mouse.x, mouse.y - 12.0f), ImVec2(mouse.x, mouse.y + 12.0f), selectionColor(color, 0.9f));
    }
}

void PolyscopeGui::drawSelectFacesMenu() {
    if (!ImGui::BeginMenu("Select Faces", controller_.hasMesh())) {
        return;
    }
    if (ImGui::MenuItem("Box Select", "B")) {
        startFaceSelectMode(FaceSelectMode::Box);
    }
    tooltip("Drag a rectangle over the faces to select (Shift+drag deselects).");
    if (ImGui::MenuItem("Circle Select", "C")) {
        startFaceSelectMode(FaceSelectMode::Circle);
    }
    tooltip("Paint faces with a circle brush (Shift or middle drag deselects, wheel resizes, Esc finishes).");
    ImGui::TextDisabled("Shift+click a face to toggle it");
    ImGui::Separator();
    if (ImGui::MenuItem("Select All", "A")) {
        controller_.selectAllFaces(currentClipPlanes());
        requestSync();
    }
    if (ImGui::MenuItem("Deselect All", "Alt+A", false, controller_.selectedFaceCount() > 0)) {
        controller_.clearFaceSelection();
        requestSync();
    }
    bool through = controller_.selectsThrough();
    if (ImGui::MenuItem("Select Through (X-ray)", nullptr, &through, !controller_.ghostMeshEnabled())) {
        controller_.setSelectThrough(through);
    }
    tooltip(controller_.ghostMeshEnabled()
        ? "On while Ghost (G) shows the hidden faces: box and circle select them too."
        : "Off: box and circle select only the faces you can see. On: also the hidden faces behind them. "
          "Faces cut away by a slice plane are never selected.");
    ImGui::Separator();
    ImGui::TextDisabled("%s faces selected", withThousands(controller_.selectedFaceCount()).c_str());
    ImGui::TextDisabled("Use them: Mesh > Decimate, Weld / Merge Vertices, Delete Selected Faces (Del)");
    ImGui::TextDisabled("Color: Display > Base mesh > Selected faces");
    ImGui::EndMenu();
}

} // namespace ogun
