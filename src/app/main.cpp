#include "app/AppController.hpp"
#include "app/CliRunner.hpp"
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
    const mesheal::CliOptions cli = mesheal::parseCliArguments(argc, argv);
    if (mesheal::isHeadlessCommand(cli)) {
        return mesheal::executeCliCommand(cli);
    }

    if (cli.noRelaunch) {
#ifdef __APPLE__
        mesheal::disableMacRelaunch();
#endif
    }

    mesheal::AppController controller;
#ifdef __APPLE__
    mesheal::installMacFileOpenHandler([&controller]() {
        return controller.hasMesh();
    });
#endif
    // Check Mesh after load runs on a worker, so the window opens (with Cancel) while it runs
    controller.setBackgroundChecks(true);
    // Saved preferences first: the weld floor and thin-face thresholds apply to the file loaded below
    mesheal::migrateLegacyConfigDirectory(mesheal::configDirectory());
    const std::filesystem::path prefsFile = mesheal::preferencesFilePath(mesheal::configDirectory());
    const mesheal::UserPreferences prefs = mesheal::loadAndApplyUserPreferences(controller, prefsFile);

    const std::filesystem::path recentFile = mesheal::recentFilesPath(mesheal::configDirectory());

    std::filesystem::path initialPath = cli.inputPath;
    if (initialPath.empty()) {
        if (auto pending = mesheal::FileOpenQueue::instance().popFile()) {
            initialPath = *pending;
        }
    }

    if (!initialPath.empty()) {
        if (!controller.loadStlFile(initialPath)) {
            std::cerr << "Warning: " << controller.lastErrorMessage() << "\n";
        } else {
#ifdef __APPLE__
            mesheal::setMacCurrentLoadedPath(initialPath);
#endif
            mesheal::RecentFiles recent(recentFile);
            recent.load();
            recent.add(initialPath);
            if (std::string error; !recent.save(&error)) {
                std::cerr << "Warning: could not save the recent files list: " << error << "\n";
            }
        }
    }

    mesheal::PolyscopeGui gui(controller, prefsFile, prefs, recentFile);
    gui.run();
    return 0;
}
