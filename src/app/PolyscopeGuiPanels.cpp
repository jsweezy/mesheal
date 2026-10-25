#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>
#include <polyscope/polyscope.h>
#include <polyscope/render/engine.h>
#include <polyscope/structure.h>
#include <polyscope/view.h>

#include <array>
#include <string>

namespace ogun {

using gui::tooltip;

namespace {

constexpr std::array<PreviewDisplayMode, 3> kPreviewDisplayModes = {
    PreviewDisplayMode::CutoutOriginalEdges,
    PreviewDisplayMode::TranslucentBase,
    PreviewDisplayMode::TranslucentPatch,
};

constexpr std::array<BackFaceMode, 4> kBackFaceModes = {
    BackFaceMode::Identical,
    BackFaceMode::Darker,
    BackFaceMode::Custom,
    BackFaceMode::Cull,
};

/// Small color swatch editing an Eigen RGB color in place.
bool colorSwatch(const char* label, Eigen::Vector3d& color) {
    std::array<float, 3> rgb = {static_cast<float>(color.x()), static_cast<float>(color.y()), static_cast<float>(color.z())};
    if (!ImGui::ColorEdit3(label, rgb.data(), ImGuiColorEditFlags_NoInputs)) {
        return false;
    }
    color = Eigen::Vector3d(rgb[0], rgb[1], rgb[2]);
    return true;
}

bool widthSlider(const char* label, double& valuePx) {
    auto value = static_cast<float>(valuePx);
    if (!ImGui::SliderFloat(label, &value,
                            static_cast<float>(settings::kMinOverlayEdgeWidthPx),
                            static_cast<float>(settings::kMaxOverlayEdgeWidthPx), "%.2f px")) {
        return false;
    }
    valuePx = static_cast<double>(value);
    return true;
}

bool opacitySlider(const char* label, double& value) {
    auto v = static_cast<float>(value);
    if (!ImGui::SliderFloat(label, &v, static_cast<float>(settings::kMinPreviewOpacity), 1.0f, "%.2f")) {
        return false;
    }
    value = static_cast<double>(v);
    return true;
}

glm::vec3 toGlmVec(const Eigen::Vector3d& v) {
    return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

} // namespace

// -----------------------------------------------------------------------------
// Display panel (UI_DESIGN.md §9.1)
// -----------------------------------------------------------------------------

void PolyscopeGui::toggleGhost() {
    controller_.setGhostMeshEnabled(!controller_.ghostMeshEnabled());
    syncMeshTransparencyToPolyscope();
}

void PolyscopeGui::drawDisplayPanel() {
    OverlayAppearance appearance = controller_.overlayAppearance();
    bool baseChanged = false;
    bool overlayChanged = false;
    bool tintChanged = false;
    ImGui::PushItemWidth(130.0f);

    if (ImGui::CollapsingHeader("Base mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        tintChanged |= colorSwatch("Colour##base", appearance.baseMesh.color);
        ImGui::SameLine();
        baseChanged |= ImGui::Checkbox("Edges##base", &appearance.baseMesh.showEdges);
        ImGui::SameLine();
        baseChanged |= colorSwatch("##baseEdgeColor", appearance.baseMesh.edgeColor);
        tooltip("Edge colour");
        if (appearance.baseMesh.showEdges) {
            baseChanged |= widthSlider("Edge width##base", appearance.baseMesh.edgeWidthPx);
        }

        auto opacity = static_cast<float>(controller_.effectiveBaseOpacity());
        if (ImGui::SliderFloat("Transparency", &opacity, static_cast<float>(settings::kMinGhostOpacity), 1.0f, "%.2f")) {
            controller_.setBaseMeshTransparency(static_cast<double>(opacity));
            syncMeshTransparencyToPolyscope();
        }
        tooltip("Base-mesh opacity (1 = opaque). G toggles between opaque and the last ghost opacity.");

        if (ImGui::BeginCombo("Back faces", backFaceModeLabel(appearance.backFaceMode))) {
            for (const BackFaceMode mode : kBackFaceModes) {
                if (ImGui::Selectable(backFaceModeLabel(mode), mode == appearance.backFaceMode)) {
                    appearance.backFaceMode = mode;
                    baseChanged = true;
                }
            }
            ImGui::EndCombo();
        }
        tooltip("How the inside of triangles is drawn. 'Custom' with a bright colour makes flipped normals stand out.");
        if (appearance.backFaceMode == BackFaceMode::Custom) {
            ImGui::SameLine();
            baseChanged |= colorSwatch("##backFaceColor", appearance.backFaceColor);
        }
        baseChanged |= ImGui::Checkbox("Smooth shading", &appearance.smoothShade);
    }

    if (ImGui::CollapsingHeader("Defect highlight")) {
        auto pointRadiusPx = static_cast<float>(appearance.pointRadiusPx);
        if (ImGui::SliderFloat("Point radius", &pointRadiusPx,
                               static_cast<float>(settings::kMinHighlightPointRadiusPx),
                               static_cast<float>(settings::kMaxHighlightPointRadiusPx), "%.1f px")) {
            appearance.pointRadiusPx = static_cast<double>(pointRadiusPx);
            overlayChanged = true;
        }
        if (widthSlider("Edge width##defect", appearance.selectedFaces.edgeWidthPx)) {
            appearance.selectedFaces.showEdges = true;
            overlayChanged = true;
        }
        tintChanged |= colorSwatch("Tint", appearance.categoryTintColor);
        tooltip("Colour of every defect of the selected Mesh Health row.");
        ImGui::SameLine();
        tintChanged |= colorSwatch("Selected", appearance.selectedFaces.color);
        ImGui::SameLine();
        overlayChanged |= colorSwatch("Edges##defect", appearance.selectedFaces.edgeColor);
        ImGui::SameLine();
        overlayChanged |= colorSwatch("Points", appearance.pointColor);
    }

    if (ImGui::CollapsingHeader("Repair preview")) {
        bool showPreview = controller_.repairPreviewVisible();
        if (ImGui::Checkbox("Show preview patch", &showPreview)) {
            controller_.setRepairPreviewVisible(showPreview);
            requestSync();
        }
        if (ImGui::BeginCombo("Mode", previewDisplayModeLabel(controller_.previewDisplayMode()))) {
            for (const PreviewDisplayMode mode : kPreviewDisplayModes) {
                if (ImGui::Selectable(previewDisplayModeLabel(mode), mode == controller_.previewDisplayMode())) {
                    controller_.setPreviewDisplayMode(mode);
                    applyTransparencyModeForPreview();
                    requestSync();
                }
            }
            ImGui::EndCombo();
        }
        switch (controller_.previewDisplayMode()) {
        case PreviewDisplayMode::TranslucentBase: {
            double opacity = controller_.previewBaseOpacity();
            if (opacitySlider("Base opacity", opacity)) {
                controller_.setPreviewBaseOpacity(opacity);
                syncMeshTransparencyToPolyscope();
            }
            break;
        }
        case PreviewDisplayMode::TranslucentPatch: {
            double opacity = controller_.previewPatchOpacity();
            if (opacitySlider("Patch opacity", opacity)) {
                controller_.setPreviewPatchOpacity(opacity);
                syncMeshTransparencyToPolyscope();
            }
            break;
        }
        case PreviewDisplayMode::CutoutOriginalEdges:
            overlayChanged |= widthSlider("Original edges", appearance.originalEdgeWidthPx);
            overlayChanged |= opacitySlider("Original opacity", appearance.originalEdgeOpacity);
            break;
        }
        overlayChanged |= widthSlider("Patch edge width", appearance.previewPatch.edgeWidthPx);
        overlayChanged |= colorSwatch("Patch", appearance.previewPatch.color);
        ImGui::SameLine();
        overlayChanged |= colorSwatch("Patch edges", appearance.previewPatch.edgeColor);
        if (controller_.previewDisplayMode() == PreviewDisplayMode::CutoutOriginalEdges) {
            ImGui::SameLine();
            overlayChanged |= colorSwatch("Original", appearance.originalEdgeColor);
        }
    }

    // Polyscope's own "Appearance" section: background, transparency mode None / Simple / Pretty, ground plane
    if (polyscope::render::engine != nullptr) {
        polyscope::render::engine->buildEngineGui();
    }
    ImGui::PopItemWidth();

    if (ImGui::Button("Reset styles")) {
        controller_.resetOverlayAppearanceToDefaults();
        requestSync();
        return;
    }
    tooltip("Restore the default base, highlight and preview styles.");

    if (baseChanged || overlayChanged || tintChanged) {
        controller_.setOverlayAppearance(appearance);
    }
    if (baseChanged) {
        applyBaseMeshStyleToPolyscope();
    }
    if (overlayChanged) {
        applyOverlayAppearanceToPolyscope();
    }
    if (tintChanged) {
        requestSync();
    }
}

// -----------------------------------------------------------------------------
// Navigate panel (UI_DESIGN.md §9.2)
// -----------------------------------------------------------------------------

void PolyscopeGui::flyToStandardView(int view) {
    if (!controller_.hasMesh()) {
        return;
    }
    const glm::vec3 up = polyscope::view::getUpVec();
    glm::vec3 front = polyscope::view::getFrontVec();
    if (std::abs(glm::dot(up, front)) > 0.01f) {
        front = glm::vec3(front.z, front.x, front.y);
    }
    const glm::vec3 right = glm::cross(up, front);

    // Camera sits on the given side of the mesh and looks at its center (Polyscope's home view is "Front")
    glm::vec3 side = front;
    glm::vec3 screenUp = up;
    switch (view) {
    case 0: side = front; break;
    case 1: side = -front; break;
    case 2: side = right; break;
    case 3: side = -right; break;
    case 4: side = up; screenUp = -front; break;
    case 5: side = -up; screenUp = front; break;
    default: break;
    }
    const BoundingBox bbox = controller_.mesh().boundingBox();
    const glm::vec3 center = toGlmVec(bbox.center());
    const auto distance = static_cast<float>(1.6 * std::max(bbox.diagonal(), 1.0e-9));
    controller_.resetOrbitPivotToMeshCenter();
    polyscope::view::lookAt(center + distance * side, center, screenUp, true);
}

void PolyscopeGui::toggleOrthographic() {
    const bool ortho = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
    polyscope::view::projectionMode = ortho ? polyscope::ProjectionMode::Perspective
                                            : polyscope::ProjectionMode::Orthographic;
}

void PolyscopeGui::drawNavigatePanel() {
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float w = (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
    const ImVec2 size(w, 0.0f);
    const bool hasMesh = controller_.hasMesh();

    ImGui::BeginDisabled(!hasMesh);
    static constexpr std::array<const char*, 6> kViews = {"Front", "Back", "Right", "Left", "Top", "Bottom"};
    static constexpr std::array<int, 6> kGridOrder = {0, 4, 2, 1, 5, 3}; // Front Top Right / Back Bottom Left
    for (std::size_t i = 0; i < kGridOrder.size(); ++i) {
        if (i % 3 != 0) {
            ImGui::SameLine();
        }
        const auto view = static_cast<std::size_t>(kGridOrder[i]);
        if (ImGui::Button(kViews[view], size)) {
            flyToStandardView(kGridOrder[i]);
        }
        tooltip("Standard view around the whole mesh (View > Camera).");
    }

    ImGui::Separator();
    if (ImGui::Button("Home", size)) {
        resetCameraAndPivotToHome();
    }
    if (ImGui::BeginItemTooltip()) {
        ImGui::TextUnformatted("Frame the whole mesh.  Key: Home or H");
        const Eigen::Vector3d& pivot = controller_.orbitPivot();
        ImGui::TextDisabled("Orbit pivot: (%.4g, %.4g, %.4g) %s", pivot.x(), pivot.y(), pivot.z(),
                            std::string(unitSymbol(controller_.fileUnit())).c_str());
        const int sign = controller_.lastFocusedNormalSign();
        if (sign != 0) {
            ImGui::TextDisabled("Last framed from the %s side", sign > 0 ? "+Normal" : "-Normal");
        }
        ImGui::EndTooltip();
    }
    ImGui::SameLine();
    if (ImGui::Button("+Normal", size)) {
        focusCameraOnSelection(FocusNormalSide::PositiveNormal);
    }
    tooltip("Frame the focused defect (or the selection) from its front.  Key: .  (press again for the back)");
    ImGui::SameLine();
    if (ImGui::Button("-Normal", size)) {
        focusCameraOnSelection(FocusNormalSide::NegativeNormal);
    }
    tooltip("Frame the focused defect from behind.  Key: Shift+.");

    ImGui::BeginDisabled(!controller_.browsableResult() || controller_.browser().visibleRows().empty());
    if (ImGui::Button("< Prev", size)) {
        stepDefectFocus(-1);
    }
    tooltip("Fly to the previous defect of the selected row.  Key: Shift+N");
    ImGui::SameLine();
    if (ImGui::Button("Next >", size)) {
        stepDefectFocus(+1);
    }
    tooltip("Fly to the next defect of the selected row (worst first).  Key: N");
    ImGui::EndDisabled();
    ImGui::SameLine();
    const bool ortho = polyscope::view::projectionMode == polyscope::ProjectionMode::Orthographic;
    if (ImGui::Button(ortho ? "Ortho" : "Persp", size)) {
        toggleOrthographic();
    }
    tooltip("Toggle perspective / orthographic projection.");
    ImGui::EndDisabled();
    ImGui::TextDisabled("Hover a button to learn its key.");

    // Polyscope's own "View" section: navigation style, up / front direction, FOV, clip planes
    polyscope::view::buildViewGui();
}

// -----------------------------------------------------------------------------
// Structures (advanced) panel (UI_DESIGN.md §9.3)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawStructuresPanel() {
    ImGui::TextDisabled("Polyscope's own per-structure controls.");
    for (auto& [typeName, structureMap] : polyscope::state::structures) {
        if (structureMap.empty()) {
            continue;
        }
        ImGui::PushID(typeName.c_str());
        const std::string header = typeName + " (" + std::to_string(structureMap.size()) + ")";
        if (ImGui::CollapsingHeader(header.c_str())) {
            for (auto& entry : structureMap) {
                entry.second->buildUI();
            }
        }
        ImGui::PopID();
    }
}

} // namespace ogun
