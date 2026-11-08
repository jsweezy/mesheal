#include "io/StlReader.hpp"
#include "mockup/UiMockup.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

int main(int argc, char** argv) {
    // --fake: random defects instead of the real checks (the original P0 mockup data)
    bool fakeDefects = false;
    std::filesystem::path path(MESHEAL_MOCKUP_DEFAULT_STL);
    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--fake") {
            fakeDefects = true;
        } else {
            path = argv[i];
        }
    }

    mesheal::Mesh mesh;
    std::string error;
    if (!mesheal::loadStl(path, mesh, mesheal::Unit::Millimeter, mesheal::ToleranceConfig{}, &error)) {
        std::cerr << "Error: " << error << "\n"
                  << "Usage: " << argv[0] << " [--fake] [path/to/model.stl]\n";
        return 1;
    }

    mesheal::mockup::UiMockup mockup(std::move(mesh), path.filename().string(), fakeDefects);
    mockup.run();
    return 0;
}
