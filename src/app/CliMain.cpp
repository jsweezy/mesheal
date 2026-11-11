// Entry point of the command-line-only Mesheal executable (no Polyscope, OpenGL or windowing).
// Built as `mesheal` when MESHEAL_BUILD_GUI=OFF, and as `mesheal-cli` next to the desktop app otherwise.
#include "app/CliRunner.hpp"

#include <iostream>

int main(int argc, char** argv) {
    const mesheal::CliOptions cli = mesheal::parseCliArguments(argc, argv);
    if (mesheal::isHeadlessCommand(cli)) {
        return mesheal::executeCliCommand(cli);
    }

    std::cerr << "This build of Mesheal is command-line only and has no desktop app.\n"
                 "Give an action such as --diagnostics or --fix-all; run with --help for the full list.\n";
    return 2;
}
