#include "app/AppController.hpp"
#include "app/FileOpenQueue.hpp"
#include "app/PolyscopeGui.hpp"
#include "app/UserPreferences.hpp"
#include "app/RecentFiles.hpp"
#include "io/ConfigDirectory.hpp"

#ifdef __APPLE__
#include "app/MacFileOpenHandler.h"
#endif

#include <filesystem>
#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
    ogun::AppController controller;
#ifdef __APPLE__
    ogun::installMacFileOpenHandler([&controller]() {
        return controller.hasMesh();
    });
#endif
    // Check Mesh after load runs on a worker, so the window opens (with Cancel) while it runs
    controller.setBackgroundChecks(true);
    // Saved preferences first: the weld floor and thin-face thresholds apply to the file loaded below
    const std::filesystem::path prefsFile = ogun::preferencesFilePath(ogun::configDirectory());
    const ogun::UserPreferences prefs = ogun::loadAndApplyUserPreferences(controller, prefsFile);

    const std::filesystem::path recentFile = ogun::recentFilesPath(ogun::configDirectory());

    std::filesystem::path initialPath;
    if (argc > 1) {
        const std::string_view arg1(argv[1]);
        if (arg1 == "-h" || arg1 == "--help") {
            std::cout << "Usage: " << argv[0] << " [path/to/model.stl]\n"
                      << "  Launches Ogun STL inspection and repair viewer.\n"
                      << "  If no STL file path is provided, nothing is loaded: use File > Open or File > Load Sample.\n";
            return 0;
        }

        // Ignore macOS LaunchServices internal process serial number arguments (e.g. -psn_0_123456)
        if (!arg1.starts_with("-psn_") && !arg1.starts_with("-")) {
            initialPath = std::filesystem::path(arg1);
        }
    }

    if (initialPath.empty()) {
        if (auto pending = ogun::FileOpenQueue::instance().popFile()) {
            initialPath = *pending;
        }
    }

    if (!initialPath.empty()) {
        if (!controller.loadStlFile(initialPath)) {
            std::cerr << "Warning: " << controller.lastErrorMessage() << "\n";
        } else {
            ogun::RecentFiles recent(recentFile);
            recent.load();
            recent.add(initialPath);
            if (std::string error; !recent.save(&error)) {
                std::cerr << "Warning: could not save the recent files list: " << error << "\n";
            }
        }
    }

    ogun::PolyscopeGui gui(controller, prefsFile, prefs, recentFile);
    gui.run();
    return 0;
}
