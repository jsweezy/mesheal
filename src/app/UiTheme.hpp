#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mesheal {

/// Colour theme of the ImGui panels (Preferences > Interface). Independent of the 3D viewport background.
enum class UiTheme : std::uint8_t {
    PolyscopeDefault, ///< Polyscope's own dark green style
    Dark,             ///< ImGui's StyleColorsDark
    Light,            ///< ImGui's StyleColorsLight
    Classic,          ///< ImGui's StyleColorsClassic
    CustomLight,      ///< Light style by Pacome Danhiez, via a gist of Doug Binks
};
inline constexpr std::size_t kUiThemeCount = 5;
inline constexpr std::array<UiTheme, kUiThemeCount> kAllUiThemes = {
    UiTheme::PolyscopeDefault, UiTheme::Dark, UiTheme::Light, UiTheme::Classic, UiTheme::CustomLight,
};

/// Name shown in the Theme combo.
[[nodiscard]] const char* uiThemeName(UiTheme theme) noexcept;

/// Stable word saved in preferences.conf (never the display name).
[[nodiscard]] const char* uiThemeKey(UiTheme theme) noexcept;

} // namespace mesheal
