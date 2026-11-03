#pragma once

#include <filesystem>
#include <functional>

namespace ogun {

/// Installs the macOS AppleEvent listener for Finder file opening (kAEOpenDocuments).
/// If `isBusyPredicate` returns true (e.g. a model is already loaded in this window),
/// double-clicking another STL in Finder will spawn a new independent Ogun instance.
/// If `isBusyPredicate` returns false (e.g. at startup or empty window), the file is
/// enqueued into `FileOpenQueue` to be loaded directly in this window.
void installMacFileOpenHandler(std::function<bool()> isBusyPredicate);

/// Launches a new instance of the Ogun application loaded with the given STL file.
bool launchNewMacAppInstance(const std::filesystem::path& stlPath);

} // namespace ogun
