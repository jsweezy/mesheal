#include "app/AboutInfo.hpp"

#include "ogun/Version.hpp"

namespace ogun {

std::string_view copyrightNotice() {
    return "Ogun -- A STL/Triangle mesh repair utility for engineering applications.\n"
           "Copyright (C) 2026 by Jeremy Sweezy\n"
           "\n"
           "This program is free software: you can redistribute it and/or modify\n"
           "it under the terms of the GNU General Public License as published by\n"
           "the Free Software Foundation, either version 3 of the License, or\n"
           "(at your option) any later version.\n"
           "\n"
           "This program is distributed in the hope that it will be useful,\n"
           "but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
           "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
           "GNU General Public License for more details.\n"
           "\n"
           "You should have received a copy of the GNU General Public License\n"
           "along with this program.  If not, see <https://www.gnu.org/licenses/>.\n";
}

std::string librariesText() {
    std::string out;
    for (const DependencyInfo& dep : kDependencies) {
        if (!out.empty()) {
            out += ", ";
        }
        out += dep.name;
        out += ' ';
        out += dep.version;
    }
    return out;
}

std::string buildInfoText() {
    std::string out;
    const auto line = [&out](std::string_view label, std::string_view value) {
        out += label;
        out += ": ";
        out += value;
        out += '\n';
    };
    line("Ogun version", std::string(kVersionString) + " (" + std::string(kVersionDate) + ")");
    line("Build date", kBuildTimestamp);
    line("Build type", kBuildType);
    line("Compiler", std::string(kCompilerId) + " " + std::string(kCompilerVersion));
    line("C++ standard", kCxxStandard);
    line("Platform", kPlatform);
    line("Compiler flags", kCxxFlags);
    line("Linker flags", kLinkerFlags);
    line("Libraries", librariesText());
    return out;
}

} // namespace ogun
