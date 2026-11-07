#include "app/GuiLayout.hpp"
#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include <polyscope/options.h>
#include <polyscope/polyscope.h>
#include <polyscope/render/engine.h>
#include <polyscope/slice_plane.h>
#include <polyscope/structure.h>
#include <polyscope/view.h>

#include <algorithm>
#include <array>
#include <initializer_list>
#include <string>
#include <vector>

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
        tintChanged |= colorSwatch("Color##base", appearance.baseMesh.color);
        ImGui::SameLine();
        baseChanged |= ImGui::Checkbox("Edges##base", &appearance.baseMesh.showEdges);
        ImGui::SameLine();
        baseChanged |= colorSwatch("##baseEdgeColor", appearance.baseMesh.edgeColor);
        tooltip("Edge color");
        if (appearance.baseMesh.showEdges) {
            baseChanged |= widthSlider("Edge width##base", appearance.baseMesh.edgeWidthPx);
        }

        auto opacity = static_cast<float>(controller_.effectiveBaseOpacity());
        if (ImGui::SliderFloat("Opacity", &opacity, static_cast<float>(settings::kMinGhostOpacity), 1.0f, "%.2f")) {
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
        tooltip("How the inside of triangles is drawn. 'Custom' with a bright color makes flipped normals stand out.");
        if (appearance.backFaceMode == BackFaceMode::Custom) {
            ImGui::SameLine();
            baseChanged |= colorSwatch("##backFaceColor", appearance.backFaceColor);
        }
        baseChanged |= ImGui::Checkbox("Smooth shading", &appearance.smoothShade);
        ImGui::SameLine();
        tintChanged |= colorSwatch("Selected faces", appearance.faceSelectionColor);
        tooltip("Color of the faces selected with Shift+click, B (box) or C (circle).");
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
        tooltip("Color of every defect of the selected Mesh Health row.");
        ImGui::SameLine();
        tintChanged |= colorSwatch("Selected", appearance.selectedFaces.color);
        tooltip("Fill of the selected defect.");
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
    const BoundingBox bbox = controller_.meshBounds();
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
    tooltip("Frame the focused defect (or the selected shells) from its front.  Key: .  (press again for the back)");
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

// -----------------------------------------------------------------------------
// View > Polyscope windows, Compass and the empty-state hint
// -----------------------------------------------------------------------------

namespace {

/// Floating window for one of Polyscope's own sections, opened in its own tile of the free 3D view and
/// kept inside it; its tree nodes start open (`openNodes`, outermost first).
template <typename Body>
void polyscopeWindow(const char* title, bool* open, const LayoutRect& tile, std::initializer_list<const char*> openNodes,
                     Body&& body) {
    if (!*open) {
        return;
    }
    ImGui::SetNextWindowPos(ImVec2(tile.x, tile.y), ImGuiCond_Appearing);
    ImGui::SetNextWindowSizeConstraints(ImVec2(0.0f, 0.0f), ImVec2(tile.width, tile.height));
    if (ImGui::Begin(title, open, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::IsWindowAppearing()) {
            // A tree node's open state is stored under its ID, nested nodes under their parent's ID
            ImGuiID parent = ImGui::GetCurrentWindow()->IDStack.back();
            for (const char* node : openNodes) {
                const ImGuiID id = ImHashStr(node, 0, parent);
                ImGui::GetStateStorage()->SetInt(id, 1);
                parent = id;
            }
        }
        ImGui::PushItemWidth(130.0f);
        body();
        ImGui::PopItemWidth();
    }
    ImGui::End();
}

} // namespace

void PolyscopeGui::drawPolyscopeWindows(float top, float bottom, float rightColumnLeft) {
    // One tile each, side by side from the left of the free 3D view (more rows when it is narrow), so
    // any set of them opens without overlapping; each row is tall enough for its windows' content. Each opens in its tile (also after Reset Layout).
    const float margin = settings::kPanelMarginPx;
    const float freeLeft = settings::kLeftColumnWidthPx + 2.0f * margin;
    const LayoutRect free{freeLeft, top, std::max(rightColumnLeft - margin - freeLeft, 0.0f), std::max(bottom - top, 0.0f)};
    std::vector<float> heights(settings::kPolyscopeWindowHeightsPx.begin(), settings::kPolyscopeWindowHeightsPx.end());
    for (float& h : heights) {
        h *= uiScale_;
    }
    const std::vector<LayoutRect> tiles =
        tileWindows(free, 4, settings::kPolyscopeWindowWidthPx, settings::kPolyscopeWindowMinWidthPx, margin, heights);
    polyscopeWindow("Polyscope: Slice Planes", &showPolySlicePlanes_, tiles[0], {"Slice Planes"},
                    [] { polyscope::buildSlicePlaneGUI(); });
    polyscopeWindow("Polyscope: Appearance", &showPolyAppearance_, tiles[1], {"Appearance", "Transparency"}, [] {
        if (polyscope::render::engine != nullptr) {
            polyscope::render::engine->buildEngineGui();
        }
    });
    polyscopeWindow("Polyscope: Camera & Clipping", &showPolyView_, tiles[2], {"View"},
                    [] { polyscope::view::buildViewGui(); });
    polyscopeWindow("Polyscope: Render & Debug", &showPolyRender_, tiles[3], {}, [] {
        ImGui::Text("Rolling: %.1f ms/frame (%.1f fps)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        if (ImGui::InputInt("max fps", &polyscope::options::maxFPS, 0) &&
            polyscope::options::maxFPS < 1 && polyscope::options::maxFPS != -1) {
            polyscope::options::maxFPS = -1;
        }
        ImGui::Checkbox("vsync", &polyscope::options::enableVSync);
        ImGui::Checkbox("Always redraw", &polyscope::options::alwaysRedraw);
        ImGui::Checkbox("Show pick buffer", &polyscope::options::debugDrawPickBuffer);
        if (ImGui::Button("Force refresh")) {
            polyscope::refresh();
        }
    });
}

void PolyscopeGui::drawCompass(float left, float bottom) {
    if (!showCompass_ || polyscope::view::windowWidth <= 0) {
        return;
    }
    const float scale = uiScale_;
    const float length = 46.0f * scale;
    const ImVec2 origin(left + length + 18.0f * scale, bottom - length - 14.0f * scale);

    struct Axis {
        const char* label;
        glm::vec3 direction;
        ImU32 color;
    };
    const glm::mat3 rotation(polyscope::view::viewMat);
    std::array<Axis, 3> axes = {{{"X", rotation * glm::vec3(1, 0, 0), IM_COL32(230, 70, 70, 255)},
                                 {"Y", rotation * glm::vec3(0, 1, 0), IM_COL32(80, 200, 80, 255)},
                                 {"Z", rotation * glm::vec3(0, 0, 1), IM_COL32(80, 130, 240, 255)}}};
    // Axes pointing away from the viewer first (view space looks down -z), so near ones draw on top
    std::sort(axes.begin(), axes.end(), [](const Axis& a, const Axis& b) { return a.direction.z < b.direction.z; });

    ImDrawList* draw = ImGui::GetBackgroundDrawList();
    draw->AddCircleFilled(origin, length + 12.0f * scale, IM_COL32(0, 0, 0, 40), 32);
    for (const Axis& axis : axes) {
        const ImVec2 tip(origin.x + axis.direction.x * length, origin.y - axis.direction.y * length);
        const bool near = axis.direction.z >= 0.0f;
        const ImU32 color = near ? axis.color : (axis.color & 0x00FFFFFFu) | (140u << 24);
        draw->AddLine(origin, tip, color, 2.5f * scale);
        draw->AddCircleFilled(tip, 8.0f * scale, color, 16);
        const ImVec2 textSize = ImGui::CalcTextSize(axis.label);
        draw->AddText(ImVec2(tip.x - textSize.x * 0.5f, tip.y - textSize.y * 0.5f), IM_COL32(255, 255, 255, 255),
                      axis.label);
    }
}

void PolyscopeGui::drawNoMeshHint(float left, float top, float bottom) {
    if (controller_.hasMesh()) {
        return;
    }
    // Centred in the free view between the columns and wrapped to its width
    const float margin = settings::kPanelMarginPx;
    const float rightLeft = ImGui::GetIO().DisplaySize.x - rightColumnWidth_ - margin;
    const LayoutRect free = freeViewArea(left - 2.0f * margin, rightLeft, top, bottom, margin);
    const std::vector<std::string> lines =
        wrapText("No mesh loaded. Open an STL (File > Open) or load a sample (File > Load Sample).",
                 std::max(free.width - 2.0f * margin, 1.0f),
                 [](const std::string& s) { return ImGui::CalcTextSize(s.c_str()).x; });
    const float lineHeight = ImGui::GetTextLineHeightWithSpacing();
    const float blockHeight = lineHeight * static_cast<float>(lines.size());
    float y = free.y + (free.height - blockHeight) * 0.5f;
    float widest = 0.0f;
    for (const std::string& line : lines) {
        widest = std::max(widest, ImGui::CalcTextSize(line.c_str()).x);
    }
    const ImRect textRect(ImVec2(free.x + (free.width - widest) * 0.5f, y),
                          ImVec2(free.x + (free.width + widest) * 0.5f, y + blockHeight));
    // Background list: every window covers it. Preferences and the Polyscope windows are translucent,
    // so skip the hint while one of them is open over the text.
    const auto covers = [&textRect](const ImVec2& pos, const ImVec2& size) {
        return textRect.Overlaps(ImRect(pos, ImVec2(pos.x + size.x, pos.y + size.y)));
    };
    for (const char* name : {"Preferences", "About Ogun", "Polyscope: Slice Planes", "Polyscope: Appearance",
                             "Polyscope: Camera & Clipping", "Polyscope: Render & Debug"}) {
        const ImGuiWindow* window = ImGui::FindWindowByName(name);
        if (window != nullptr && window->Active && !window->Hidden && covers(window->Pos, window->Size)) {
            return;
        }
    }
    // Dark grey on a light background, light grey on a dark one (the UI text colour is white)
    const auto& bg = polyscope::view::bgColor;
    const float luminance = 0.299f * bg[0] + 0.587f * bg[1] + 0.114f * bg[2];
    const ImU32 hintColor = luminance > 0.5f ? IM_COL32(70, 70, 70, 255) : IM_COL32(190, 190, 190, 255);
    ImDrawList* draw = ImGui::GetBackgroundDrawList();
    for (const std::string& line : lines) {
        const float width = ImGui::CalcTextSize(line.c_str()).x;
        draw->AddText(ImVec2(free.x + (free.width - width) * 0.5f, y), hintColor, line.c_str());
        y += lineHeight;
    }
}

} // namespace ogun
