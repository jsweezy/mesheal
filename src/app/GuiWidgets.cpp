#include "app/GuiWidgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mesheal::gui {

namespace {

/// The colour as is on a dark window, darkened on a light one (judged by the window background).
ImVec4 forWindow(const ImVec4& color) {
    const ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
    const float luminance = 0.299f * bg.x + 0.587f * bg.y + 0.114f * bg.z;
    if (luminance < 0.5f) {
        return color;
    }
    return ImVec4(color.x * 0.55f, color.y * 0.55f, color.z * 0.55f, color.w);
}

} // namespace

ImVec4 okColor() { return forWindow(ImVec4(0.35f, 0.80f, 0.40f, 1.0f)); }
ImVec4 defectColor() { return forWindow(ImVec4(1.00f, 0.62f, 0.20f, 1.0f)); }
ImVec4 runningColor() { return forWindow(ImVec4(0.35f, 0.65f, 1.00f, 1.0f)); }
ImVec4 mutedColor() { return forWindow(ImVec4(0.55f, 0.55f, 0.55f, 1.0f)); }
ImVec4 infoColor() { return forWindow(ImVec4(0.55f, 0.70f, 0.85f, 1.0f)); }
ImVec4 errorColor() { return forWindow(ImVec4(1.0f, 0.35f, 0.35f, 1.0f)); }

std::string withThousands(std::size_t value) {
    std::string digits = std::to_string(value);
    for (auto pos = static_cast<std::ptrdiff_t>(digits.size()) - 3; pos > 0; pos -= 3) {
        digits.insert(static_cast<std::size_t>(pos), ",");
    }
    return digits;
}

std::string signedThousands(long long delta) {
    const std::string digits = withThousands(static_cast<std::size_t>(delta < 0 ? -delta : delta));
    return delta > 0 ? "+" + digits : delta < 0 ? "-" + digits : digits;
}

std::string signedPercent(double percent) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%+.3g%%", percent);
    return buffer;
}

std::string significant(double value, int digits) {
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.*g", digits, value);
    return buffer;
}

std::string modShortcut(const char* key) {
    return std::string(kModKey) + "+" + key;
}

void tooltip(const char* text) {
    if (!text || text[0] == '\0') {
        return;
    }
    // Suppress tooltips while dragging or interacting with widgets so values underneath stay visible
    if (ImGui::IsItemActive() || ImGui::IsAnyItemActive()) {
        return;
    }

    const ImGuiID id = ImGui::GetItemID();
    static ImGuiID s_suppressedId = 0;
    if (id != 0 && s_suppressedId == id) {
        if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            s_suppressedId = 0;
        } else {
            return;
        }
    }

    // Right-clicking the item dismisses its tooltip until the mouse leaves the item
    if (id != 0 && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        s_suppressedId = id;
        return;
    }

    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

MouseModifiers heldMouseModifiers() {
    const ImGuiIO& io = ImGui::GetIO();
    return static_cast<MouseModifiers>((io.KeyShift ? kMouseModShift : 0) | (io.KeyCtrl ? kMouseModCtrl : 0) |
                                       (io.KeyAlt ? kMouseModAlt : 0));
}

void helpMarker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    tooltip(text);
}

void statusDot(HealthStatus status, bool infoOnly) {
    const float lineHeight = ImGui::GetTextLineHeight();
    const float radius = lineHeight * 0.3f;
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 center(cursor.x + radius + 2.0f, cursor.y + lineHeight * 0.5f);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    switch (status) {
    case HealthStatus::NotRun:
        draw->AddCircle(center, radius, ImGui::GetColorU32(mutedColor()), 12, 1.5f);
        break;
    case HealthStatus::Running: {
        const auto t = static_cast<float>(ImGui::GetTime());
        const float start = std::fmod(t * 6.0f, 6.2832f);
        draw->PathArcTo(center, radius, start, start + 4.2f, 12);
        draw->PathStroke(ImGui::GetColorU32(runningColor()), 0, 2.0f);
        break;
    }
    case HealthStatus::Ok:
        draw->AddCircleFilled(center, radius, ImGui::GetColorU32(infoOnly ? infoColor() : okColor()), 12);
        break;
    case HealthStatus::Defects:
        draw->AddCircleFilled(center, radius, ImGui::GetColorU32(defectColor()), 12);
        break;
    case HealthStatus::Stale:
        draw->AddCircleFilled(center, radius * 0.7f, ImGui::GetColorU32(mutedColor()), 12);
        draw->AddCircle(center, radius, ImGui::GetColorU32(defectColor()), 12, 1.5f);
        break;
    }
    ImGui::Dummy(ImVec2(2.0f * radius + 4.0f, lineHeight));
}

std::string dimensionSuffix(ValueDimension dimension, Unit unit) {
    switch (dimension) {
    case ValueDimension::Length:
        return std::string(unitSymbol(unit));
    case ValueDimension::Area:
        return std::string(unitSymbol(unit)) + "^2";
    case ValueDimension::Volume:
        return std::string(unitSymbol(unit)) + "^3";
    case ValueDimension::None:
        break;
    }
    return {};
}

std::string formatValue(double value, ValueDimension dimension) {
    char buffer[32];
    if (!std::isfinite(value)) {
        return "inf";
    }
    if (dimension == ValueDimension::None && value == std::floor(value) && std::abs(value) < 1.0e9) {
        std::snprintf(buffer, sizeof(buffer), "%.0f", value);
    } else if (dimension == ValueDimension::None) {
        std::snprintf(buffer, sizeof(buffer), "%.4g", value);
    } else {
        std::snprintf(buffer, sizeof(buffer), "%.3e", value);
    }
    return buffer;
}

bool unitCombo(const char* id, Unit& unit, float width) {
    bool changed = false;
    ImGui::SetNextItemWidth(width);
    if (ImGui::BeginCombo(id, std::string(unitDisplayName(unit)).c_str())) {
        for (const Unit candidate : kAllUnits) {
            const bool selected = candidate == unit;
            if (ImGui::Selectable(std::string(unitDisplayName(candidate)).c_str(), selected)) {
                unit = candidate;
                changed = true;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

bool thinFaceThresholdFields(ThinFaceThresholds& thresholds, Unit unit, float width) {
    const std::string sym(unitSymbol(unit));
    bool changed = false;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Flag faces stretched more than");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(width);
    if (ImGui::InputDouble("##maxAspectRatio", &thresholds.maxAspectRatio, 0.0, 0.0, "%.6g",
                           ImGuiInputTextFlags_EnterReturnsTrue)) {
        thresholds.maxAspectRatio = std::max(1.0, thresholds.maxAspectRatio);
        changed = true;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(": 1");
    helpMarker("Aspect ratio = longest edge / smallest height. It does not depend on the model's size or unit, "
               "so a small part and a large one are judged alike. Faces with no reliable normal (flat, or "
               "smaller than rounding at their distance from the origin) are always flagged as Zero-area.");
    if (ImGui::TreeNode("Advanced: absolute floors (file units), 0 = off")) {
        ImGui::SetNextItemWidth(width);
        if (ImGui::InputDouble(("Min height h_min (" + sym + ")").c_str(), &thresholds.minAltitude, 0.0, 0.0, "%.6g",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            thresholds.minAltitude = std::max(0.0, thresholds.minAltitude);
            changed = true;
        }
        ImGui::SetNextItemWidth(width);
        if (ImGui::InputDouble(("Min area (" + sym + "^2)").c_str(), &thresholds.minArea, 0.0, 0.0, "%.6g",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            thresholds.minArea = std::max(0.0, thresholds.minArea);
            changed = true;
        }
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Also flag faces lower or smaller than these, e.g. for a printer's resolution. "
                            "Scale Mesh Coordinates rescales them.");
        ImGui::PopTextWrapPos();
        ImGui::TreePop();
    }
    return changed;
}

} // namespace mesheal::gui
