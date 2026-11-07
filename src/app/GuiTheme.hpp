#pragma once

#include "app/UiTheme.hpp"

namespace ogun::gui {

/// Sets the ImGui colours of `theme` (sizes and rounding stay as Polyscope set them). Needs an ImGui
/// context. `PolyscopeDefault` re-runs the style callback Polyscope had installed before Ogun's.
void applyUiTheme(UiTheme theme);

/// Remembers the theme and replaces `polyscope::options::configureImGuiStyleCallback` so that Polyscope's
/// own style set-up (inside `polyscope::init()`) applies it. Call before `polyscope::init()`.
void installUiTheme(UiTheme theme);

/// Changes the theme now and for later Polyscope style set-ups.
void setUiTheme(UiTheme theme);

} // namespace ogun::gui
