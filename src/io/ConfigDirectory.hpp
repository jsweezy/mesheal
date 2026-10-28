#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace ogun {

/// Ogun's per-user config directory (preferences, recent files): `$XDG_CONFIG_HOME/ogun` when
/// XDG_CONFIG_HOME is set and non-empty, else `~/.config/ogun`, on macOS and Linux alike.
/// Empty when no home directory is known. Does not create the directory.
[[nodiscard]] std::filesystem::path configDirectory();

/// `configDirectory()` for the given environment values (null or empty means unset).
[[nodiscard]] std::filesystem::path configDirectoryFor(const char* xdgConfigHome, const char* home);

/// Writes `contents` to a temporary file next to `file`, then renames it over `file`, so readers never
/// see a half-written file. Creates missing parent directories. On failure returns false and sets `error`.
bool writeFileAtomically(const std::filesystem::path& file, std::string_view contents, std::string* error = nullptr);

} // namespace ogun
