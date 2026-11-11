#pragma once

#include <string>
#include <string_view>

namespace mesheal {

/// Public documentation site (built from docs/ with MkDocs and published on GitHub Pages).
inline constexpr std::string_view kDocumentationUrl = "https://jsweezy.github.io/mesheal/";
/// Source repository and issue tracker.
inline constexpr std::string_view kRepositoryUrl = "https://github.com/jsweezy/mesheal";
inline constexpr std::string_view kIssuesUrl = "https://github.com/jsweezy/mesheal/issues";

/// Opens an http(s) URL in the default browser (`open` on macOS, `xdg-open` on Linux) without a
/// shell. Returns false and fills `error` when the URL is not http(s) or the launcher cannot start.
bool openUrlInBrowser(std::string_view url, std::string* error = nullptr);

/// The GPLv3 copyright notice shown in Help > About Mesheal > Copyright.
std::string_view copyrightNotice();

/// Plain-text build summary (version, build date, compiler, flags, libraries) for bug reports.
std::string buildInfoText();

/// Comma-separated "Name version" list of the libraries Mesheal is built with.
std::string librariesText();

} // namespace mesheal
