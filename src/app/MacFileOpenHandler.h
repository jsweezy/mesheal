#pragma once

#include <filesystem>
#include <functional>

namespace mesheal {

/// Installs the macOS AppleEvent listener and Cocoa delegate hooks for file opening.
void installMacFileOpenHandler(std::function<bool()> isBusyPredicate);

/// Disables spawning new instances from this process (e.g. for child instances).
void disableMacRelaunch();

/// Notifies the handler that the initial launch sequence has completed and the UI is active.
void markMacInitialLaunchComplete();

/// Updates the currently loaded model path so duplicate open events for the same file are ignored.
void setMacCurrentLoadedPath(const std::filesystem::path& path);

/// Launches a new instance of the Mesheal application loaded with the given STL file.
bool launchNewMacAppInstance(const std::filesystem::path& stlPath);

} // namespace mesheal
