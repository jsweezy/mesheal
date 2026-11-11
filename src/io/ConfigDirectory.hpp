#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace mesheal {

/// Mesheal's per-user config directory (preferences, recent files): `$XDG_CONFIG_HOME/mesheal` when
/// XDG_CONFIG_HOME is set and non-empty, else `~/.config/mesheal` on macOS and Linux, and
/// `%APPDATA%\mesheal` on Windows. Empty when no home directory is known. Does not create the directory.
[[nodiscard]] std::filesystem::path configDirectory();

/// `configDirectory()` for the given environment values (null or empty means unset).
[[nodiscard]] std::filesystem::path configDirectoryFor(const char* xdgConfigHome, const char* home);

/// One-time move of the pre-rename config directory (`ogun` beside `configDir`) to `configDir`, so
/// saved preferences and recent files survive the Ogun -> Mesheal rename. Does nothing unless the legacy
/// directory exists and `configDir` does not. Returns true when it moved.
bool migrateLegacyConfigDirectory(const std::filesystem::path& configDir);

/// Writes `contents` to a temporary file next to `file`, then renames it over `file`, so readers never
/// see a half-written file. Creates missing parent directories. On failure returns false and sets `error`.
bool writeFileAtomically(const std::filesystem::path& file, std::string_view contents, std::string* error = nullptr);

} // namespace mesheal
