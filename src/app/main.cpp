#include "app/AppController.hpp"
#include "app/PolyscopeGui.hpp"

#include <filesystem>
#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
    ogun::AppController controller;

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
        }
    } else {
        controller.loadSampleDefectiveMesh();
    }

    ogun::PolyscopeGui gui(controller);
    gui.run();
    return 0;
}
