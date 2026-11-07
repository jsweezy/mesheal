#include "app/GuiTheme.hpp"

#include <imgui.h>
#include <polyscope/options.h>

#include <functional>

namespace ogun::gui {

namespace {

UiTheme gTheme = UiTheme::CustomLight;
std::function<void()> gPolyscopeStyle;

// Light style from Pacome Danhiez (user itamago):
//   https://github.com/ocornut/imgui/pull/511#issuecomment-175719267
// as collected in Doug Binks' SetupImGuiStyle:
//   https://gist.github.com/dougbinks/8089b4bbaccaaf6fa204236978d165a9
// Ported to current ImGui names; colours the original predates keep the StyleColorsLight values.
void customLight() {
    ImGui::StyleColorsLight();
    ImGuiStyle& style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.FrameRounding = 3.0f;
    ImVec4* c = style.Colors;
    c[ImGuiCol_Text] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    c[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 0.94f);
    c[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    c[ImGuiCol_PopupBg] = ImVec4(1.00f, 1.00f, 1.00f, 0.94f);
    c[ImGuiCol_Border] = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
    c[ImGuiCol_BorderShadow] = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
    c[ImGuiCol_FrameBg] = ImVec4(1.00f, 1.00f, 1.00f, 0.94f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    c[ImGuiCol_TitleBg] = ImVec4(0.96f, 0.96f, 0.96f, 1.00f);
    c[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 1.00f, 1.00f, 0.51f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.82f, 0.82f, 0.82f, 1.00f);
    c[ImGuiCol_MenuBarBg] = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    c[ImGuiCol_ScrollbarBg] = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
    c[ImGuiCol_ScrollbarGrab] = ImVec4(0.69f, 0.69f, 0.69f, 1.00f);
    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.59f, 0.59f, 0.59f, 1.00f);
    c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
    c[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_SliderGrab] = ImVec4(0.24f, 0.52f, 0.88f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_Button] = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
    c[ImGuiCol_Header] = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_Separator] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
    c[ImGuiCol_SeparatorHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
    c[ImGuiCol_SeparatorActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    c[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
    c[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    c[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    c[ImGuiCol_PlotLines] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
    c[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    c[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    c[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    c[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);
}

} // namespace

void applyUiTheme(UiTheme theme) {
    switch (theme) {
    case UiTheme::PolyscopeDefault:
        if (gPolyscopeStyle) {
            gPolyscopeStyle();
        }
        break;
    case UiTheme::Dark: ImGui::StyleColorsDark(); break;
    case UiTheme::Light: ImGui::StyleColorsLight(); break;
    case UiTheme::Classic: ImGui::StyleColorsClassic(); break;
    case UiTheme::CustomLight: customLight(); break;
    }
}

void installUiTheme(UiTheme theme) {
    gTheme = theme;
    if (!gPolyscopeStyle) {
        gPolyscopeStyle = polyscope::options::configureImGuiStyleCallback;
    }
    polyscope::options::configureImGuiStyleCallback = []() {
        // Polyscope's style also sets sizes; keep those for every theme, then colour on top
        if (gPolyscopeStyle) {
            gPolyscopeStyle();
        }
        if (gTheme != UiTheme::PolyscopeDefault) {
            applyUiTheme(gTheme);
        }
    };
}

void setUiTheme(UiTheme theme) {
    gTheme = theme;
    if (ImGui::GetCurrentContext() != nullptr) {
        // Re-run Polyscope's style first so the sizes are Polyscope's, whatever theme was set before
        if (gPolyscopeStyle) {
            gPolyscopeStyle();
        }
        if (theme != UiTheme::PolyscopeDefault) {
            applyUiTheme(theme);
        }
    }
}

} // namespace ogun::gui
