#pragma once

#include <string>
#include <string_view>

namespace mesheal {

/// The GPLv3 copyright notice shown in Help > About Mesheal > Copyright.
std::string_view copyrightNotice();

/// Plain-text build summary (version, build date, compiler, flags, libraries) for bug reports.
std::string buildInfoText();

/// Comma-separated "Name version" list of the libraries Mesheal is built with.
std::string librariesText();

} // namespace mesheal
