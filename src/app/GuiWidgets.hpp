#pragma once

#include "app/AppController.hpp"
#include "app/MouseMapping.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Units.hpp"

#include <imgui.h>

#include <cstddef>
#include <string>

namespace mesheal::gui {

#ifdef __APPLE__
inline constexpr const char* kModKey = "Cmd";
#else
inline constexpr const char* kModKey = "Ctrl";
#endif

/// Status text colours. Tuned for a dark window; on a light theme they are darkened to stay readable.
[[nodiscard]] ImVec4 okColor();
[[nodiscard]] ImVec4 defectColor();
[[nodiscard]] ImVec4 runningColor();
[[nodiscard]] ImVec4 mutedColor();
[[nodiscard]] ImVec4 infoColor();
[[nodiscard]] ImVec4 errorColor();
inline const ImVec4 kFixAllButtonColor(0.20f, 0.45f, 0.25f, 1.0f);

/// "20000" -> "20,000".
[[nodiscard]] std::string withThousands(std::size_t value);

/// Signed count with thousands separators: "+12", "-1,204", "0".
[[nodiscard]] std::string signedThousands(long long delta);

/// Signed percentage: "+1.2%", "-35%".
[[nodiscard]] std::string signedPercent(double percent);

/// Plain number with `digits` significant digits ("41234.6", "0.00123").
[[nodiscard]] std::string significant(double value, int digits = 6);

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

/// Thin-face threshold fields (Thin faces tool and Preferences): "Flag faces stretched more than [N] : 1"
/// first, the absolute floors under a collapsed "Advanced" node. A value applies on Enter; returns true then.
bool thinFaceThresholdFields(ThinFaceThresholds& thresholds, Unit unit, float width);

} // namespace mesheal::gui
