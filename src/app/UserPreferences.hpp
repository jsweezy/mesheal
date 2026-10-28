#pragma once

#include "app/AppController.hpp"
#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/ToleranceConfig.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace ogun {

/// File in `configDirectory()` holding the saved preferences.
inline constexpr const char* kPreferencesFileName = "preferences.conf";

/// Default Auto state of every Mesh Health row (`CategoryInfo::autoDefault`).
[[nodiscard]] std::array<bool, kDefectCategoryCount> builtInAutoCheckDefaults() noexcept;

/// Display and tolerance preferences saved across sessions (UI_DESIGN.md §9.5): everything in the
/// Preferences window plus the Display panel styles. Default-constructed = built-in defaults.
struct UserPreferences {
    // ---- AppController state (`AppController::userPreferences` / `applyUserPreferences`) ----
    OverlayAppearance appearance{};
    double ghostOpacity = settings::kDefaultGhostOpacity;
    bool repairPreviewVisible = true;
    PreviewDisplayMode previewDisplayMode = PreviewDisplayMode::CutoutOriginalEdges;
    double previewBaseOpacity = settings::kDefaultPreviewBaseOpacity;
    double previewPatchOpacity = settings::kDefaultPreviewPatchOpacity;
    ToleranceConfig tolerance{};
    PreferenceDefaults defaults{};
    bool checkMeshAfterLoad = settings::kDefaultCheckMeshAfterLoad;
    std::array<bool, kDefectCategoryCount> autoCheckDefaults = builtInAutoCheckDefaults();
    std::size_t listCap = settings::kDefaultListCap;
    std::size_t overlayCap = settings::kDefaultOverlayCap;

    // ---- GUI-only state (PolyscopeGui) ----
    float uiScale = 1.0f;
    std::array<float, 4> backgroundColor = settings::kDefaultBackgroundColor;
};

/// `<configDir>/preferences.conf` (empty when `configDir` is empty).
[[nodiscard]] std::filesystem::path preferencesFilePath(const std::filesystem::path& configDir);

/// Human-readable `key = value` text, one setting per line (colors as "r g b" in [0, 1]).
/// Formatting is exact: parsing the text gives back the same values.
[[nodiscard]] std::string formatUserPreferences(const UserPreferences& prefs);

/// Parses `formatUserPreferences` text. Starts from the built-in defaults; unknown keys, comments and
/// malformed values are skipped (that key keeps its default); numbers are clamped to their valid range.
[[nodiscard]] UserPreferences parseUserPreferences(std::string_view text);

/// True when every saved value of `a` equals that of `b` exactly.
[[nodiscard]] bool sameUserPreferences(const UserPreferences& a, const UserPreferences& b);

/// Every key written by `formatUserPreferences`, in file order.
[[nodiscard]] std::vector<std::string> userPreferenceKeys();

/// Reads and parses `file`; a missing or unreadable file gives the built-in defaults.
[[nodiscard]] UserPreferences loadUserPreferences(const std::filesystem::path& file);

/// Writes `prefs` to `file` atomically (creating its directory). Returns false and sets `error` on failure.
bool saveUserPreferences(const UserPreferences& prefs, const std::filesystem::path& file, std::string* error = nullptr);

/// Startup: loads `file` and applies it to `controller` (before any mesh is loaded, so the weld floor and the
/// thin-face thresholds apply to it). Returns the loaded preferences for the GUI-only fields.
UserPreferences loadAndApplyUserPreferences(AppController& controller, const std::filesystem::path& file);

} // namespace ogun
