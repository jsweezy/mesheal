#include "app/AppController.hpp"
#include "app/PolyscopeGui.hpp"
#include "app/UserPreferences.hpp"
#include "app/RecentFiles.hpp"
#include "io/ConfigDirectory.hpp"

#include <filesystem>
#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
    ogun::AppController controller;
    // Check Mesh after load runs on a worker, so the window opens (with Cancel) while it runs
    controller.setBackgroundChecks(true);
    // Saved preferences first: the weld floor and thin-face thresholds apply to the file loaded below
    const std::filesystem::path prefsFile = ogun::preferencesFilePath(ogun::configDirectory());
    const ogun::UserPreferences prefs = ogun::loadAndApplyUserPreferences(controller, prefsFile);

    const std::filesystem::path recentFile = ogun::recentFilesPath(ogun::configDirectory());

    if (argc > 1) {
        const std::string_view arg1(argv[1]);
        if (arg1 == "-h" || arg1 == "--help") {
            std::cout << "Usage: " << argv[0] << " [path/to/model.stl]\n"
                      << "  Launches Ogun STL inspection and repair viewer.\n"
                      << "  If no STL file path is provided, loads the built-in sample defective mesh.\n";
            return 0;
        }

        const std::filesystem::path stlPath(arg1);
        if (!controller.loadStlFile(stlPath)) {
            std::cerr << "Warning: " << controller.lastErrorMessage() << "\n";
        } else {
            ogun::RecentFiles recent(recentFile);
            recent.load();
            recent.add(stlPath);
            if (std::string error; !recent.save(&error)) {
                std::cerr << "Warning: could not save the recent files list: " << error << "\n";
            }
        }
    } else {
        controller.loadSampleDefectiveMesh();
    }

    ogun::PolyscopeGui gui(controller, prefsFile, prefs, recentFile);
    gui.run();
    return 0;
}
