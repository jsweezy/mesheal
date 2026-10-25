#include "app/PolyscopeGui.hpp"
#include "core/Constants.hpp"
#include "core/Settings.hpp"

#ifdef __APPLE__
#include "app/MacAppIcon.hpp"
#endif

#include <imgui.h>
#include <polyscope/curve_network.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/render/engine.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <queue>
#include <string>
#include <vector>

extern "C" {
unsigned char* stbi_load(const char* filename, int* x, int* y, int* channels_in_file, int desired_channels);
void stbi_image_free(void* retval_from_stbi_load);
}

namespace ogun {

namespace {

/// When loading a 3-channel JPEG icon that has a white/shadow background outside the rounded
/// metallic frame, flood-fills the outer margin to transparent alpha with 2px edge feathering.
void makeOuterBackgroundTransparent(std::vector<unsigned char>& rgba, int w, int h) {
    if (w <= 0 || h <= 0 || rgba.size() < static_cast<std::size_t>(w * h * 4)) {
        return;
    }

    auto isBgCandidate = [&](int idx) noexcept {
        const int r = rgba[static_cast<std::size_t>(idx) * 4 + 0];
        const int g = rgba[static_cast<std::size_t>(idx) * 4 + 1];
        const int b = rgba[static_cast<std::size_t>(idx) * 4 + 2];
        const int maxCh = std::max({r, g, b});
        const int minCh = std::min({r, g, b});
        const int diff = maxCh - minCh;
        return (minCh >= 230 && diff <= 12) || (minCh >= 175 && diff <= 6);
    };

    std::vector<std::uint8_t> visited(static_cast<std::size_t>(w * h), 0);
    std::queue<int> q;

    for (int x = 0; x < w; ++x) {
        const int topIdx = x;
        const int botIdx = (h - 1) * w + x;
        if (isBgCandidate(topIdx)) {
            visited[static_cast<std::size_t>(topIdx)] = 1;
            q.push(topIdx);
        }
        if (isBgCandidate(botIdx)) {
            visited[static_cast<std::size_t>(botIdx)] = 1;
            q.push(botIdx);
        }
    }
    for (int y = 0; y < h; ++y) {
        const int leftIdx = y * w;
        const int rightIdx = y * w + (w - 1);
        if (!visited[static_cast<std::size_t>(leftIdx)] && isBgCandidate(leftIdx)) {
            visited[static_cast<std::size_t>(leftIdx)] = 1;
            q.push(leftIdx);
        }
        if (!visited[static_cast<std::size_t>(rightIdx)] && isBgCandidate(rightIdx)) {
            visited[static_cast<std::size_t>(rightIdx)] = 1;
            q.push(rightIdx);
        }
    }

    constexpr int kDx[4] = {1, -1, 0, 0};
    constexpr int kDy[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        const int cur = q.front();
        q.pop();
        const int cx = cur % w;
        const int cy = cur / w;
        for (int d = 0; d < 4; ++d) {
            const int nx = cx + kDx[d];
            const int ny = cy + kDy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
                continue;
            }
            const int nidx = ny * w + nx;
            if (!visited[static_cast<std::size_t>(nidx)] && isBgCandidate(nidx)) {
                visited[static_cast<std::size_t>(nidx)] = 1;
                q.push(nidx);
            }
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int idx = y * w + x;
            if (visited[static_cast<std::size_t>(idx)]) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 0;
                continue;
            }
            bool nearBg1 = false;
            bool nearBg2 = false;
            for (int oy = -2; oy <= 2; ++oy) {
                for (int ox = -2; ox <= 2; ++ox) {
                    const int nx = std::clamp(x + ox, 0, w - 1);
                    const int ny = std::clamp(y + oy, 0, h - 1);
                    if (visited[static_cast<std::size_t>(ny * w + nx)]) {
                        if (std::abs(ox) <= 1 && std::abs(oy) <= 1) {
                            nearBg1 = true;
                        } else {
                            nearBg2 = true;
                        }
                    }
                }
            }
            if (nearBg1) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 128;
            } else if (nearBg2) {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 220;
            } else {
                rgba[static_cast<std::size_t>(idx) * 4 + 3] = 255;
            }
        }
    }
}

constexpr double kColorReadbackTolerance = 1.0e-4;

glm::vec3 toGlm(const Eigen::Vector3d& v) {
    return glm::vec3(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

Eigen::Vector3d fromGlm(const glm::vec3& v) {
    return Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
}

/// Adopts `polyscopeValue` into `target` when the user changed it in Polyscope's UI.
bool adoptColor(const glm::vec3& polyscopeValue, Eigen::Vector3d& target) {
    const Eigen::Vector3d value = fromGlm(polyscopeValue);
    if ((value - target).cwiseAbs().maxCoeff() <= kColorReadbackTolerance) {
        return false;
    }
    target = value;
    return true;
}

/// Re-sizing is skipped unless the target radius drifted noticeably from the last applied one.
bool radiusIsStale(double radius, double lastApplied, bool forceUpdate) {
    return radius > 0.0 &&
           (forceUpdate || lastApplied <= 0.0 || std::abs(radius - lastApplied) > 2.0e-3 * lastApplied);
}

void removeSurfaceMeshIfPresent(const char* name) {
    if (polyscope::hasSurfaceMesh(name)) {
        polyscope::removeSurfaceMesh(name, false);
    }
}

void removeCurveNetworkIfPresent(const char* name) {
    if (polyscope::hasCurveNetwork(name)) {
        polyscope::removeCurveNetwork(name, false);
    }
}

void removePointCloudIfPresent(const char* name) {
    if (polyscope::hasPointCloud(name)) {
        polyscope::removePointCloud(name, false);
    }
}

} // namespace

PolyscopeGui::PolyscopeGui(AppController& controller)
    : controller_(controller) {}

void PolyscopeGui::run() {
    polyscope::options::programName = "Ogun - STL Mesh Inspection & Repair Tool";
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::options::openImGuiWindowForUserCallback = false;
    polyscope::options::buildDefaultGuiPanels = false;

    polyscope::init();
    polyscope::state::doDefaultMouseInteraction = false;
    applyTransparencyModeForPreview();
    loadAndApplyApplicationIcon();

    controller_.setBackgroundChecks(true);
    if (controller_.hasMesh()) {
        syncMeshStructureToPolyscope(true);
    }

    polyscope::state::userCallback = [this]() {
        drawUiFrame();
    };

    polyscope::show();
    appIconTexture_.reset();
}

void PolyscopeGui::loadAndApplyApplicationIcon() {
    const std::filesystem::path iconPath = AppController::resolveApplicationIconPath();
    if (iconPath.empty()) {
        return;
    }

    int width = 0;
    int height = 0;
    int channelsInFile = 0;
    unsigned char* rawPixels = stbi_load(iconPath.string().c_str(), &width, &height, &channelsInFile, 4);
    if (rawPixels == nullptr || width <= 0 || height <= 0) {
        if (rawPixels != nullptr) {
            stbi_image_free(rawPixels);
        }
        return;
    }

    const std::size_t byteCount =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u;
    std::vector<unsigned char> rgba(rawPixels, rawPixels + byteCount);
    stbi_image_free(rawPixels);

    if (channelsInFile == 3) {
        makeOuterBackgroundTransparent(rgba, width, height);
    }

#ifdef __APPLE__
    setMacApplicationIconFromRgba(rgba.data(), width, height);
#endif

    if (polyscope::render::engine != nullptr) {
        appIconTexture_ = polyscope::render::engine->generateTextureBuffer(
            polyscope::TextureFormat::RGBA8,
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            rgba.data());
        if (appIconTexture_ != nullptr) {
            appIconTexture_->setFilterMode(polyscope::FilterMode::Linear);
        }
    }
}

void PolyscopeGui::drawUiFrame() {
    if (controller_.pollBackgroundChecks()) {
        requestSync();
    }
    syncStylesFromPolyscopeUi();
    handleViewportMouseNavigation();
    handleKeyboardShortcuts();

    ImGui::GetIO().FontGlobalScale = uiScale_;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float menuHeight = drawMenuBar();
    const float statusHeight = ImGui::GetFrameHeight() + 2.0f * ImGui::GetStyle().WindowPadding.y;
    const float top = menuHeight + settings::kPanelMarginPx;
    const float bottom = display.y - statusHeight - settings::kPanelMarginPx;

    drawRightColumn(top, bottom);
    const float rightLeft = display.x - rightColumnWidth_ - settings::kPanelMarginPx;
    drawLeftColumn(top, bottom, rightLeft);
    drawStatusBar(display.y - statusHeight, statusHeight);

    if (showPreferences_) {
        drawPreferencesWindow();
    }
    if (showNavigationHelpWindow_) {
        drawNavigationHelpWindow();
    }
    drawModalPopups();
    layoutResetPending_ = false;

    // Deferred: deleting rebuilds the shell list the Shells table may still be iterating
    if (deleteShellsRequested_) {
        deleteShellsRequested_ = false;
        if (controller_.deleteSelectedShells()) {
            requestSync(true);
        }
    }
    processPendingSync();
    applyEffectiveHighlightSizingToPolyscope(false);
}

void PolyscopeGui::requestSync(bool meshChanged) noexcept {
    syncPending_ = true;
    syncMeshChanged_ = syncMeshChanged_ || meshChanged;
}

void PolyscopeGui::processPendingSync() {
    if (!syncPending_) {
        return;
    }
    if (syncMeshChanged_) {
        polyscope::resetSelection();
    }
    syncPending_ = false;
    syncMeshChanged_ = false;
    syncHighlightOverlaysToPolyscope();
}

void PolyscopeGui::undoLastModification() {
    if (controller_.undoLastModification()) {
        requestSync(true);
    }
}

void PolyscopeGui::handleViewportMouseNavigation() {
    ImGuiIO& io = ImGui::GetIO();
    if (polyscope::view::windowWidth <= 0 || polyscope::view::windowHeight <= 0) {
        return;
    }

    bool widgetCapturedMouse = false;
    for (polyscope::WeakHandle<polyscope::Widget>& wHandle : polyscope::state::widgets) {
        if (wHandle.isValid()) {
            polyscope::Widget& w = wHandle.get();
            widgetCapturedMouse = w.interact();
            if (widgetCapturedMouse) {
                break;
            }
        }
    }

    if (io.WantCaptureMouse || widgetCapturedMouse) {
        return;
    }

    const Eigen::Vector3d camPos = currentCameraWorldPosition();
    const Eigen::Vector3d pivot = controller_.orbitPivot();
    const double rawDist = (camPos - pivot).norm();
    const double meshDiag = controller_.hasMesh()
        ? std::max(controller_.mesh().boundingBox().diagonal(), 1.0e-6)
        : 1.0;
    const double minNavDist = 1.0e-6 * meshDiag;
    const double effectiveDist = (std::isfinite(rawDist) && rawDist > minNavDist)
        ? rawDist
        : std::max(minNavDist, 1.0e-3 * meshDiag);

    // 1. Scroll wheel zoom (or Shift+Scroll clip plane shift)
    const double xoffset = io.MouseWheelH;
    const double yoffset = io.MouseWheel;
    if (xoffset != 0.0 || yoffset != 0.0) {
        const double maxScroll = (std::abs(yoffset) > std::abs(xoffset)) ? yoffset : xoffset;
        if (maxScroll != 0.0) {
            if (io.KeyShift) {
                polyscope::view::processClipPlaneShift(maxScroll);
            } else {
                processPivotZoom(maxScroll, effectiveDist);
            }
        }
    }

    // 2. Mouse drags (Orbit, Pan, or Drag-Zoom)
    const bool dragLeft = ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    const bool dragRight = !dragLeft && ImGui::IsMouseDragging(ImGuiMouseButton_Right);
    const bool dragMiddle = !dragLeft && !dragRight && ImGui::IsMouseDragging(ImGuiMouseButton_Middle);
    if (dragLeft || dragRight || dragMiddle) {
        const glm::vec2 dragDelta{
            io.MouseDelta.x / static_cast<float>(polyscope::view::windowWidth),
            -io.MouseDelta.y / static_cast<float>(polyscope::view::windowHeight)};
        dragDistSinceLastRelease_ += std::abs(dragDelta.x) + std::abs(dragDelta.y);

        const bool isDragZoom = dragLeft && io.KeyShift && io.KeyCtrl;
        const bool isTranslate = (dragLeft && io.KeyShift && !io.KeyCtrl) || dragRight || dragMiddle;
        const bool isRotate = dragLeft && !io.KeyShift && !io.KeyCtrl;

        if (isDragZoom) {
            processPivotZoom(static_cast<double>(dragDelta.y) * 5.0, effectiveDist);
        } else if (isRotate) {
            processPivotRotate(dragDelta.x, dragDelta.y);
        } else if (isTranslate) {
            processPivotTranslate(dragDelta.x, dragDelta.y, effectiveDist);
        }
    }

    // 3. Click picks on release when not dragging
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (dragDistSinceLastRelease_ < settings::kDragIgnoreThreshold) {
            const ImVec2 p = ImGui::GetMousePos();
            const polyscope::PickResult pickResult = polyscope::pickAtScreenCoords(glm::vec2{p.x, p.y});
            polyscope::setSelection(pickResult);
            handleViewportFacePick(pickResult);
        }
        dragDistSinceLastRelease_ = 0.0f;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        if (dragDistSinceLastRelease_ < settings::kDragIgnoreThreshold) {
            polyscope::resetSelection();
        }
        dragDistSinceLastRelease_ = 0.0f;
    }
}

void PolyscopeGui::handleViewportFacePick(const polyscope::PickResult& pickResult) {
    if (!pickResult.isHit || !controller_.hasMesh()) {
        return;
    }

    // Both pickable surfaces may be drawn with a subset of faces; map the local face back to the mesh
    const std::vector<Eigen::Index>* faceMap = nullptr;
    std::uint64_t nVerts = 0;
    std::uint64_t nFaces = 0;
    if (pickResult.structureName == kSurfaceMeshName) {
        nVerts = static_cast<std::uint64_t>(controller_.mesh().numVertices());
        nFaces = baseIsCut_
            ? static_cast<std::uint64_t>(baseFaceIndices_.size())
            : static_cast<std::uint64_t>(controller_.mesh().numFaces());
        faceMap = baseIsCut_ ? &baseFaceIndices_ : nullptr;
    } else if (pickResult.structureName == kDefectFacesName && polyscope::hasSurfaceMesh(kDefectFacesName)) {
        nVerts = static_cast<std::uint64_t>(polyscope::getSurfaceMesh(kDefectFacesName)->nVertices());
        nFaces = static_cast<std::uint64_t>(defectFaceIndices_.size());
        faceMap = &defectFaceIndices_;
    } else {
        return;
    }

    // Polyscope's combined pick index lists vertices [0, nVerts) before faces [nVerts, nVerts + nFaces)
    const std::uint64_t elementId = pickResult.localIndex;
    if (elementId < nVerts || elementId >= nVerts + nFaces) {
        return;
    }

    const auto localFace = static_cast<std::size_t>(elementId - nVerts);
    const Eigen::Index face = faceMap ? (*faceMap)[localFace] : static_cast<Eigen::Index>(localFace);
    // Alt+click selects the shell (Cmd/Ctrl too: toggles it); a plain click never changes the shell selection
    const ImGuiIO& io = ImGui::GetIO();
    if (io.KeyAlt && controller_.shells().multiShell()) {
        if (controller_.pickShell(face, io.KeyCtrl || io.KeySuper)) {
            showShells_ = true;
            requestSync();
        }
        return;
    }
    const PickOutcome outcome = controller_.pickFace(face);
    if (outcome == PickOutcome::FocusedDefect || outcome == PickOutcome::SelectedRow) {
        scrollListToFocus_ = true;
    }
    requestSync();
}

void PolyscopeGui::processPivotRotate(float dragDeltaX, float dragDeltaY) {
    if (dragDeltaX == 0.0f && dragDeltaY == 0.0f) {
        return;
    }
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None ||
        polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::Planar) {
        return;
    }

    glm::vec3 frameLookDir(0.0f, 0.0f, -1.0f);
    glm::vec3 frameUpDir(0.0f, 1.0f, 0.0f);
    glm::vec3 frameRightDir(1.0f, 0.0f, 0.0f);
    polyscope::view::getCameraFrame(frameLookDir, frameUpDir, frameRightDir);

    const Eigen::Vector3d& pivotEig = controller_.orbitPivot();
    const glm::vec3 pivot(
        static_cast<float>(pivotEig.x()),
        static_cast<float>(pivotEig.y()),
        static_cast<float>(pivotEig.z()));

    const float moveScale = static_cast<float>(polyscope::view::moveScale);
    const float delTheta = 2.0f * dragDeltaX * moveScale;
    const float delPhi = 2.0f * dragDeltaY * moveScale;

    switch (polyscope::view::getNavigateStyle()) {
    case polyscope::NavigateStyle::Turntable: {
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, pivot);
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, frameRightDir);
        polyscope::view::viewMat = polyscope::view::viewMat * phiCamR;
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, polyscope::view::getUpVec());
        polyscope::view::viewMat = polyscope::view::viewMat * thetaCamR;
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, -pivot);
        break;
    }
    case polyscope::NavigateStyle::Free:
    case polyscope::NavigateStyle::Arcball: {
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, pivot);
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, frameUpDir);
        polyscope::view::viewMat = polyscope::view::viewMat * thetaCamR;
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, frameRightDir);
        polyscope::view::viewMat = polyscope::view::viewMat * phiCamR;
        polyscope::view::viewMat = glm::translate(polyscope::view::viewMat, -pivot);
        break;
    }
    case polyscope::NavigateStyle::FirstPerson: {
        const glm::vec3 rotAx = glm::mat3(polyscope::view::viewMat) * polyscope::view::getUpVec();
        const glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0f), delTheta, rotAx);
        polyscope::view::viewMat = thetaCamR * polyscope::view::viewMat;
        const glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0f), -delPhi, glm::vec3(1.0f, 0.0f, 0.0f));
        polyscope::view::viewMat = phiCamR * polyscope::view::viewMat;
        break;
    }
    case polyscope::NavigateStyle::Planar:
    case polyscope::NavigateStyle::None:
        break;
    }

    controller_.notifyCameraRotated();
    polyscope::view::immediatelyEndFlight();
    polyscope::requestRedraw();
}

void PolyscopeGui::processPivotTranslate(float dragDeltaX, float dragDeltaY, double effectiveDist) {
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None) {
        return;
    }
    if (dragDeltaX == 0.0f && dragDeltaY == 0.0f) {
        return;
    }

    const double clampedFovDeg = std::clamp(
        static_cast<double>(polyscope::view::fov),
        settings::kMinCameraFovDegrees,
        settings::kMaxCameraFovDegrees);
    const double fovRadHalf = (clampedFovDeg * 0.5) * kDegToRad;
    const double viewHeightAtPivot = 2.0 * effectiveDist * std::tan(fovRadHalf);
    const auto movementScale = static_cast<float>(viewHeightAtPivot * polyscope::view::moveScale);

    const Eigen::Vector3d oldCamPos = currentCameraWorldPosition();
    const glm::mat4x4 camSpaceT = glm::translate(
        glm::mat4x4(1.0f),
        movementScale * glm::vec3(dragDeltaX, dragDeltaY, 0.0f));
    polyscope::view::viewMat = camSpaceT * polyscope::view::viewMat;
    const Eigen::Vector3d newCamPos = currentCameraWorldPosition();

    // Shift the orbit pivot in world space along with the camera so subsequent rotations stay centered
    controller_.translateOrbitPivot(newCamPos - oldCamPos);

    polyscope::view::immediatelyEndFlight();
    polyscope::requestRedraw();
}

void PolyscopeGui::processPivotZoom(double amount, double effectiveDist) {
    if (amount == 0.0) {
        return;
    }
    if (polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::None ||
        polyscope::view::getNavigateStyle() == polyscope::NavigateStyle::FirstPerson) {
        return;
    }

    if (polyscope::view::projectionMode == polyscope::ProjectionMode::Perspective) {
        const auto movementScale = static_cast<float>(effectiveDist * 0.12 * polyscope::view::moveScale);
        const glm::mat4x4 camSpaceT = glm::translate(
            glm::mat4x4(1.0f),
            glm::vec3(0.0f, 0.0f, movementScale * static_cast<float>(amount)));
        polyscope::view::viewMat = camSpaceT * polyscope::view::viewMat;
        polyscope::view::immediatelyEndFlight();
        polyscope::requestRedraw();
    } else {
        polyscope::view::processZoom(amount);
    }
}

void PolyscopeGui::handleKeyboardShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) {
        return;
    }
    const bool mod = io.KeyCtrl || io.KeySuper;

    if ((ImGui::IsKeyPressed(ImGuiKey_Slash, false) && io.KeyShift) ||
        ImGui::IsKeyPressed(ImGuiKey_F1, false)) {
        showNavigationHelpWindow_ = !showNavigationHelpWindow_;
    }
    if (mod && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        undoLastModification();
    }
    if (mod && ImGui::IsKeyPressed(ImGuiKey_O, false)) {
        openFileDialog();
    }
    if (mod && ImGui::IsKeyPressed(ImGuiKey_S, false)) {
        if (io.KeyShift) {
            saveAsDialog(saveFormat_);
        } else {
            saveCurrentFile();
        }
    }
    if (mod && ImGui::IsKeyPressed(ImGuiKey_Comma, false)) {
        showPreferences_ = true;
    }

    // Plain keys below must not fire while an ImGui widget has keyboard focus (e.g. a slider)
    if (io.WantCaptureKeyboard && ImGui::IsAnyItemActive()) {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (controller_.escape()) {
            requestSync();
        }
    }
    if (!controller_.hasMesh() || mod) {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) {
        checkMesh();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_N, false)) {
        stepDefectFocus(io.KeyShift ? -1 : +1);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_G, false)) {
        toggleGhost();
    }
    if (controller_.shells().multiShell()) {
        if (ImGui::IsKeyPressed(ImGuiKey_I, false)) {
            toggleIsolateShells();
        }
        if ((ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) &&
            controller_.shells().selectedCount() > 0) {
            deleteSelectedShells();
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Period, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false)) {
        focusCameraOnSelection(io.KeyShift ? FocusNormalSide::NegativeNormal : FocusNormalSide::AutoToggleNormal);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false) ||
        ImGui::IsKeyPressed(ImGuiKey_H, false)) {
        resetCameraAndPivotToHome();
    }
}

void PolyscopeGui::stepDefectFocus(int direction) {
    if (controller_.focusNextDefect(direction)) {
        scrollListToFocus_ = true;
        requestSync();
        focusCameraOnSelection(FocusNormalSide::PositiveNormal);
    }
}

void PolyscopeGui::syncMeshStructureToPolyscope(bool resetCameraToHome) {
    if (!controller_.hasMesh()) {
        removeSurfaceMeshIfPresent(kSurfaceMeshName);
        removeSurfaceMeshIfPresent(kDefectFacesName);
        removeSurfaceMeshIfPresent(kRepairPreviewName);
        removeCurveNetworkIfPresent(kOverlayCurveName);
        removeCurveNetworkIfPresent(kOriginalEdgesName);
        removePointCloudIfPresent(kOverlayPointName);
        baseIsCut_ = false;
        baseFaceIndices_.clear();
        defectFaceIndices_.clear();
        lastAppliedDefectEdgeRadius_ = -1.0;
        lastAppliedOriginalEdgeRadius_ = -1.0;
        lastAppliedPointRadius_ = -1.0;
        return;
    }

    syncHighlightOverlaysToPolyscope();

    if (resetCameraToHome) {
        controller_.resetOrbitPivotToSelection();
        polyscope::view::resetCameraToHomeView();
    }
}

void PolyscopeGui::syncHighlightOverlaysToPolyscope() {
    if (!controller_.hasMesh()) {
        return;
    }

    const HighlightOverlayGeometry overlay = controller_.buildHighlightOverlay();
    registerBaseMeshToPolyscope(overlay);
    syncDefectOverlaysToPolyscope(overlay);
    syncRepairPreviewToPolyscope();
    applyOverlayAppearanceToPolyscope();
    syncMeshTransparencyToPolyscope();
}

void PolyscopeGui::registerBaseMeshToPolyscope(const HighlightOverlayGeometry& overlay) {
    // Re-registering replaces the structure, so carry over the user's Structures-panel settings
    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        auto* previous = polyscope::getSurfaceMesh(kSurfaceMeshName);
        baseMaterial_ = previous->getMaterial();
        baseEnabled_ = previous->isEnabled();
        baseStyleCaptured_ = true;
    }

    const Mesh& mesh = controller_.mesh();
    BaseMeshDisplay display = controller_.buildBaseMeshDisplay();
    baseIsCut_ = display.isCut;
    baseFaceIndices_ = std::move(display.faceIndices);
    if (baseIsCut_ && display.F.rows() == 0) {
        removeSurfaceMeshIfPresent(kSurfaceMeshName);
        return;
    }

    auto* psMesh = polyscope::registerSurfaceMesh(kSurfaceMeshName, mesh.V, baseIsCut_ ? display.F : mesh.F);
    if (baseStyleCaptured_) {
        psMesh->setMaterial(baseMaterial_);
        psMesh->setEnabled(baseEnabled_);
    }
    applyBaseMeshStyleToPolyscope();

    std::vector<Eigen::Vector3d> faceColors;
    if (baseIsCut_) {
        faceColors.reserve(baseFaceIndices_.size());
        for (const Eigen::Index f : baseFaceIndices_) {
            faceColors.push_back(overlay.faceColors[static_cast<std::size_t>(f)]);
        }
    } else {
        faceColors = overlay.faceColors;
    }
    psMesh->addFaceColorQuantity(kFaceColorQuantityName, faceColors)->setEnabled(true);
}

void PolyscopeGui::applyBaseMeshStyleToPolyscope() {
    if (!polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        return;
    }
    const OverlayAppearance& appearance = controller_.overlayAppearance();
    auto* psMesh = polyscope::getSurfaceMesh(kSurfaceMeshName);
    psMesh->setSurfaceColor(toGlm(appearance.baseMesh.color));
    psMesh->setEdgeColor(toGlm(appearance.baseMesh.edgeColor));
    psMesh->setEdgeWidth(appearance.baseMesh.effectiveEdgeWidth());
    psMesh->setSmoothShade(appearance.smoothShade);
    switch (appearance.backFaceMode) {
    case BackFaceMode::Identical:
        psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Identical);
        break;
    case BackFaceMode::Darker:
        psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Different);
        break;
    case BackFaceMode::Custom:
        psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Custom);
        psMesh->setBackFaceColor(toGlm(appearance.backFaceColor));
        break;
    case BackFaceMode::Cull:
        psMesh->setBackFacePolicy(polyscope::BackFacePolicy::Cull);
        break;
    }
}

void PolyscopeGui::syncDefectOverlaysToPolyscope(const HighlightOverlayGeometry& overlay) {
    defectFaceIndices_ = overlay.overlayFaceIndices;
    lastAppliedDefectEdgeRadius_ = -1.0;
    lastAppliedPointRadius_ = -1.0;
    // Polyscope aborts when drawing a structure without elements, so each one is registered only when
    // it has content (a selected row with no selected defects has tubes and points but no faces)
    if (overlay.overlayFaces.empty()) {
        removeSurfaceMeshIfPresent(kDefectFacesName);
    } else {
        auto* psFaces = polyscope::registerSurfaceMesh(kDefectFacesName, overlay.overlayPoints, overlay.overlayFaces);
        psFaces->setSmoothShade(false);
        psFaces->setBackFacePolicy(polyscope::BackFacePolicy::Identical);
    }

    // Edge tubes and vertex spheres keep zero-area and sub-pixel slivers visible at any zoom
    if (overlay.overlayEdges.empty()) {
        removeCurveNetworkIfPresent(kOverlayCurveName);
    } else {
        polyscope::registerCurveNetwork(kOverlayCurveName, overlay.overlayPoints, overlay.overlayEdges);
    }
    if (overlay.overlayPoints.empty()) {
        removePointCloudIfPresent(kOverlayPointName);
    } else {
        polyscope::registerPointCloud(kOverlayPointName, overlay.overlayPoints);
    }
}

void PolyscopeGui::syncMeshTransparencyToPolyscope() {
    auto alpha = static_cast<float>(controller_.effectiveBaseOpacity());
    const bool previewShown = controller_.repairPreviewShown();
    if (previewShown && controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentBase) {
        alpha = std::min(alpha, static_cast<float>(controller_.previewBaseOpacity()));
    }
    const bool translucentPatch = previewShown &&
                                  controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentPatch;

    // Polyscope switches to Pretty on its own when it first sees transparency; pick the mode's preference first
    if ((alpha < 1.0f || translucentPatch) &&
        polyscope::options::transparencyMode == polyscope::TransparencyMode::None) {
        applyTransparencyModeForPreview();
    }

    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        polyscope::getSurfaceMesh(kSurfaceMeshName)->setTransparency(alpha);
    }
    if (polyscope::hasSurfaceMesh(kRepairPreviewName)) {
        polyscope::getSurfaceMesh(kRepairPreviewName)->setTransparency(
            translucentPatch ? static_cast<float>(controller_.previewPatchOpacity()) : 1.0f);
    }
}

void PolyscopeGui::applyTransparencyModeForPreview() {
    // Translucent Base looks best with Simple blending; the other modes with Pretty depth peeling
    polyscope::options::transparencyMode =
        controller_.previewDisplayMode() == PreviewDisplayMode::TranslucentBase
            ? polyscope::TransparencyMode::Simple
            : polyscope::TransparencyMode::Pretty;
}

void PolyscopeGui::syncRepairPreviewToPolyscope() {
    const Mesh& patch = controller_.repairPreview().previewPatchMesh;
    if (controller_.repairPreviewShown() && !patch.empty()) {
        auto* psPatch = polyscope::registerSurfaceMesh(kRepairPreviewName, patch.V, patch.F);
        psPatch->setSmoothShade(false);
        psPatch->setBackFacePolicy(polyscope::BackFacePolicy::Identical);
    } else {
        removeSurfaceMeshIfPresent(kRepairPreviewName);
    }

    lastAppliedOriginalEdgeRadius_ = -1.0;
    const HighlightOverlayGeometry originalEdges = controller_.buildRepairPreviewOriginalEdges();
    if (originalEdges.hasSelectionOverlay()) {
        polyscope::registerCurveNetwork(kOriginalEdgesName, originalEdges.overlayPoints, originalEdges.overlayEdges);
    } else {
        removeCurveNetworkIfPresent(kOriginalEdgesName);
    }
}

void PolyscopeGui::applyOverlayAppearanceToPolyscope() {
    const OverlayAppearance& appearance = controller_.overlayAppearance();
    const auto applySurfaceStyle = [](const char* name, const SurfaceStyle& style) {
        if (polyscope::hasSurfaceMesh(name)) {
            auto* psMesh = polyscope::getSurfaceMesh(name);
            psMesh->setSurfaceColor(toGlm(style.color));
            psMesh->setEdgeColor(toGlm(style.edgeColor));
            psMesh->setEdgeWidth(style.effectiveEdgeWidth());
        }
    };
    applySurfaceStyle(kDefectFacesName, appearance.selectedFaces);
    applySurfaceStyle(kRepairPreviewName, appearance.previewPatch);

    if (polyscope::hasCurveNetwork(kOverlayCurveName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOverlayCurveName);
        curveNet->setColor(toGlm(appearance.selectedFaces.edgeColor));
        curveNet->setEnabled(appearance.selectedFaces.showEdges);
    }
    if (polyscope::hasPointCloud(kOverlayPointName)) {
        polyscope::getPointCloud(kOverlayPointName)->setPointColor(toGlm(appearance.pointColor));
    }
    if (polyscope::hasCurveNetwork(kOriginalEdgesName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOriginalEdgesName);
        curveNet->setColor(toGlm(appearance.originalEdgeColor));
        curveNet->setTransparency(static_cast<float>(appearance.originalEdgeOpacity));
    }
    applyEffectiveHighlightSizingToPolyscope(true);
}

void PolyscopeGui::syncStylesFromPolyscopeUi() {
    if (!controller_.hasMesh()) {
        return;
    }

    OverlayAppearance appearance = controller_.overlayAppearance();
    bool changed = false;
    if (polyscope::hasSurfaceMesh(kSurfaceMeshName)) {
        auto* psMesh = polyscope::getSurfaceMesh(kSurfaceMeshName);
        changed |= adoptColor(psMesh->getEdgeColor(), appearance.baseMesh.edgeColor);
        changed |= appearance.baseMesh.adoptPolyscopeEdgeWidth(psMesh->getEdgeWidth());
    }
    const auto adoptSurfaceStyle = [&changed](const char* name, SurfaceStyle& style) {
        if (polyscope::hasSurfaceMesh(name)) {
            auto* psMesh = polyscope::getSurfaceMesh(name);
            changed |= adoptColor(psMesh->getSurfaceColor(), style.color);
            changed |= adoptColor(psMesh->getEdgeColor(), style.edgeColor);
            changed |= style.adoptPolyscopeEdgeWidth(psMesh->getEdgeWidth());
        }
    };
    adoptSurfaceStyle(kDefectFacesName, appearance.selectedFaces);
    adoptSurfaceStyle(kRepairPreviewName, appearance.previewPatch);

    if (polyscope::hasCurveNetwork(kOverlayCurveName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOverlayCurveName);
        changed |= adoptColor(curveNet->getColor(), appearance.selectedFaces.edgeColor);
        changed |= adoptOverlayRadius(
            static_cast<double>(curveNet->getRadius()), lastAppliedDefectEdgeRadius_, appearance.selectedFaces.edgeWidthPx);
    }
    if (polyscope::hasCurveNetwork(kOriginalEdgesName)) {
        auto* curveNet = polyscope::getCurveNetwork(kOriginalEdgesName);
        changed |= adoptColor(curveNet->getColor(), appearance.originalEdgeColor);
        changed |= adoptOverlayRadius(
            static_cast<double>(curveNet->getRadius()), lastAppliedOriginalEdgeRadius_, appearance.originalEdgeWidthPx);
        const auto alpha = static_cast<double>(curveNet->getTransparency());
        if (std::abs(alpha - appearance.originalEdgeOpacity) > 1.0e-4) {
            appearance.originalEdgeOpacity = alpha;
            changed = true;
        }
    }
    if (polyscope::hasPointCloud(kOverlayPointName)) {
        auto* ptCloud = polyscope::getPointCloud(kOverlayPointName);
        changed |= adoptColor(ptCloud->getPointColor(), appearance.pointColor);
        changed |= adoptOverlayRadius(ptCloud->getPointRadius(), lastAppliedPointRadius_, appearance.pointRadiusPx);
    }

    if (changed) {
        controller_.setOverlayAppearance(appearance);
        applyBaseMeshStyleToPolyscope();
        applyOverlayAppearanceToPolyscope();
    }
}

bool PolyscopeGui::adoptOverlayRadius(double polyscopeRadius, double& lastApplied, double& valuePx) {
    if (lastApplied <= 0.0 || std::abs(polyscopeRadius - lastApplied) <= 1.0e-4 * lastApplied) {
        return false;
    }
    const double worldPerPx = controller_.worldLengthAtSelection(
        1.0,
        currentCameraWorldPosition(),
        polyscope::view::fov,
        static_cast<double>(polyscope::view::windowHeight));
    if (worldPerPx <= 0.0) {
        return false;
    }
    valuePx = polyscopeRadius / worldPerPx;
    lastApplied = polyscopeRadius;
    return true;
}

Eigen::Vector3d PolyscopeGui::currentCameraWorldPosition() const noexcept {
    const glm::vec3 camPos = polyscope::view::getCameraWorldPosition();
    return Eigen::Vector3d(
        static_cast<double>(camPos.x),
        static_cast<double>(camPos.y),
        static_cast<double>(camPos.z));
}

void PolyscopeGui::applyEffectiveHighlightSizingToPolyscope(bool forceUpdate) {
    if (!controller_.hasMesh()) {
        return;
    }

    const Eigen::Vector3d camPos = currentCameraWorldPosition();
    const double fovDeg = polyscope::view::fov;
    const double zScale = controller_.computeHighlightZoomScale(camPos, fovDeg);

    // Keep the near clip plane proportional when zoomed in close so microscopic defects are never clipped
    const double targetNearClip = std::clamp(
        polyscope::view::defaultNearClipRatio * zScale,
        1.0e-6,
        polyscope::view::defaultNearClipRatio);
    if (std::abs(polyscope::view::nearClipRatio - targetNearClip) > 1.0e-7) {
        polyscope::view::nearClipRatio = targetNearClip;
    }

    // Tubes and spheres are re-sized as the camera moves so they keep a constant on-screen size
    const OverlayAppearance& appearance = controller_.overlayAppearance();
    const auto toWorld = [&](double pixels) {
        return controller_.worldLengthAtSelection(
            pixels, camPos, fovDeg, static_cast<double>(polyscope::view::windowHeight));
    };

    const double defectEdgeRadius = toWorld(appearance.selectedFaces.edgeWidthPx);
    if (polyscope::hasCurveNetwork(kOverlayCurveName) &&
        radiusIsStale(defectEdgeRadius, lastAppliedDefectEdgeRadius_, forceUpdate)) {
        polyscope::getCurveNetwork(kOverlayCurveName)->setRadius(static_cast<float>(defectEdgeRadius), false);
        lastAppliedDefectEdgeRadius_ = defectEdgeRadius;
    }

    const double originalEdgeRadius = toWorld(appearance.originalEdgeWidthPx);
    if (polyscope::hasCurveNetwork(kOriginalEdgesName) &&
        radiusIsStale(originalEdgeRadius, lastAppliedOriginalEdgeRadius_, forceUpdate)) {
        polyscope::getCurveNetwork(kOriginalEdgesName)->setRadius(static_cast<float>(originalEdgeRadius), false);
        lastAppliedOriginalEdgeRadius_ = originalEdgeRadius;
    }

    const double pointRadius = toWorld(appearance.pointRadiusPx);
    if (polyscope::hasPointCloud(kOverlayPointName) &&
        radiusIsStale(pointRadius, lastAppliedPointRadius_, forceUpdate)) {
        polyscope::getPointCloud(kOverlayPointName)->setPointRadius(pointRadius, false);
        lastAppliedPointRadius_ = pointRadius;
    }
}

void PolyscopeGui::focusCameraOnSelection(FocusNormalSide normalSide) {
    if (!controller_.hasMesh()) {
        return;
    }

    glm::vec3 lookDir(0.0f, 0.0f, -1.0f);
    glm::vec3 upDir(0.0f, 1.0f, 0.0f);
    glm::vec3 rightDir(1.0f, 0.0f, 0.0f);
    polyscope::view::getCameraFrame(lookDir, upDir, rightDir);

    const Eigen::Vector3d currentViewDir(
        static_cast<double>(lookDir.x),
        static_cast<double>(lookDir.y),
        static_cast<double>(lookDir.z));

    const glm::vec3 sceneUp = polyscope::view::getUpVec();
    const Eigen::Vector3d preferredUpDir(
        static_cast<double>(sceneUp.x),
        static_cast<double>(sceneUp.y),
        static_cast<double>(sceneUp.z));

    const CameraFocusTarget focus = controller_.focusOnSelection(
        normalSide,
        currentViewDir,
        preferredUpDir);
    if (!focus.valid) {
        return;
    }

    const glm::vec3 camPos(
        static_cast<float>(focus.cameraPosition.x()),
        static_cast<float>(focus.cameraPosition.y()),
        static_cast<float>(focus.cameraPosition.z()));
    const glm::vec3 targetPos(
        static_cast<float>(focus.targetCenter.x()),
        static_cast<float>(focus.targetCenter.y()),
        static_cast<float>(focus.targetCenter.z()));
    const glm::vec3 camUp(
        static_cast<float>(focus.cameraUpDirection.x()),
        static_cast<float>(focus.cameraUpDirection.y()),
        static_cast<float>(focus.cameraUpDirection.z()));

    polyscope::view::lookAt(camPos, targetPos, camUp, true);
}

void PolyscopeGui::resetCameraAndPivotToHome() {
    if (!controller_.hasMesh()) {
        return;
    }
    controller_.resetOrbitPivotToMeshCenter();
    polyscope::view::flyToHomeView();
}

} // namespace ogun
