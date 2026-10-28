#pragma once

#include <string>
#include <string_view>

namespace ogun {

/// The GPLv3 copyright notice shown in Help > About Ogun > Copyright.
std::string_view copyrightNotice();

/// Plain-text build summary (version, build date, compiler, flags, libraries) for bug reports.
std::string buildInfoText();

/// Comma-separated "Name version" list of the libraries Ogun is built with.
std::string librariesText();

} // namespace ogun
