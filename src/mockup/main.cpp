#include "io/StlReader.hpp"
#include "mockup/UiMockup.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::filesystem::path path = argc > 1
        ? std::filesystem::path(argv[1])
        : std::filesystem::path(OGUN_MOCKUP_DEFAULT_STL);

    ogun::Mesh mesh;
    std::string error;
    if (!ogun::loadStl(path, mesh, ogun::Unit::Millimeter, ogun::ToleranceConfig{}, &error)) {
        std::cerr << "Error: " << error << "\n"
                  << "Usage: " << argv[0] << " [path/to/model.stl]\n";
        return 1;
    }

    ogun::mockup::UiMockup mockup(std::move(mesh), path.filename().string());
    mockup.run();
    return 0;
}
