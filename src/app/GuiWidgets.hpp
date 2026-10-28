#pragma once

#include "app/AppController.hpp"
#include "app/MouseMapping.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Units.hpp"

#include <imgui.h>

#include <cstddef>
#include <string>

namespace ogun::gui {

#ifdef __APPLE__
inline constexpr const char* kModKey = "Cmd";
#else
inline constexpr const char* kModKey = "Ctrl";
#endif

inline const ImVec4 kOkColor(0.35f, 0.80f, 0.40f, 1.0f);
inline const ImVec4 kDefectColor(1.00f, 0.62f, 0.20f, 1.0f);
inline const ImVec4 kRunningColor(0.35f, 0.65f, 1.00f, 1.0f);
inline const ImVec4 kMutedColor(0.55f, 0.55f, 0.55f, 1.0f);
inline const ImVec4 kInfoColor(0.55f, 0.70f, 0.85f, 1.0f);
inline const ImVec4 kErrorColor(1.0f, 0.35f, 0.35f, 1.0f);
inline const ImVec4 kFixAllButtonColor(0.20f, 0.45f, 0.25f, 1.0f);

/// "20000" -> "20,000".
[[nodiscard]] std::string withThousands(std::size_t value);

/// Shortcut label with the platform modifier, e.g. "Cmd+S".
[[nodiscard]] std::string modShortcut(const char* key);

/// Tooltip on the last item (short delay).
void tooltip(const char* text);

/// Shift / Ctrl / Alt held now, as Preferences > Mouse modifier bits.
[[nodiscard]] MouseModifiers heldMouseModifiers();

/// "(?)" marker with a wrapped tooltip.
void helpMarker(const char* text);

/// Drawn status dot (the fonts have no check / warning glyphs). Advances the cursor.
void statusDot(HealthStatus status, bool infoOnly);

/// Unit suffix for a value of `dimension` ("mm", "mm^2" or "").
[[nodiscard]] std::string dimensionSuffix(ValueDimension dimension, Unit unit);

/// Compact number for lists: integers plainly, others in %.3g / %.2e.
[[nodiscard]] std::string formatValue(double value, ValueDimension dimension);

/// Combo over all units; returns true and updates `unit` when the user picks one.
bool unitCombo(const char* id, Unit& unit, float width);

} // namespace ogun::gui
