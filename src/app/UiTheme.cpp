#include "app/UiTheme.hpp"

namespace mesheal {

const char* uiThemeName(UiTheme theme) noexcept {
    switch (theme) {
    case UiTheme::PolyscopeDefault: return "Polyscope Default";
    case UiTheme::Dark: return "Dark";
    case UiTheme::Light: return "Light";
    case UiTheme::Classic: return "Classic";
    case UiTheme::CustomLight: return "Custom Light";
    }
    return "Polyscope Default";
}

const char* uiThemeKey(UiTheme theme) noexcept {
    switch (theme) {
    case UiTheme::PolyscopeDefault: return "polyscope_default";
    case UiTheme::Dark: return "dark";
    case UiTheme::Light: return "light";
    case UiTheme::Classic: return "classic";
    case UiTheme::CustomLight: return "custom_light";
    }
    return "polyscope_default";
}

} // namespace mesheal
