#include "io/ConfigDirectory.hpp"

#include <cstdlib>
#include <fstream>
#include <system_error>

#ifndef _WIN32
#include <pwd.h>
#include <unistd.h>
#endif

namespace ogun {

std::filesystem::path configDirectoryFor(const char* xdgConfigHome, const char* home) {
    if (xdgConfigHome != nullptr && *xdgConfigHome != '\0') {
        return std::filesystem::path(xdgConfigHome) / "ogun";
    }
    if (home != nullptr && *home != '\0') {
        return std::filesystem::path(home) / ".config" / "ogun";
    }
    return {};
}

std::filesystem::path configDirectory() {
    const char* home = std::getenv("HOME");
#ifndef _WIN32
    if (home == nullptr || *home == '\0') {
        if (const passwd* pw = getpwuid(getuid()); pw != nullptr) {
            home = pw->pw_dir;
        }
    }
#endif
    return configDirectoryFor(std::getenv("XDG_CONFIG_HOME"), home);
}

bool writeFileAtomically(const std::filesystem::path& file, std::string_view contents, std::string* error) {
    const auto fail = [&](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };
    if (file.empty()) {
        return fail("No file path");
    }
    std::error_code ec;
    if (file.has_parent_path()) {
        std::filesystem::create_directories(file.parent_path(), ec);
        if (ec) {
            return fail("Cannot create " + file.parent_path().string() + ": " + ec.message());
        }
    }
    std::filesystem::path temp = file;
#ifndef _WIN32
    temp += "." + std::to_string(getpid()) + ".tmp"; // unique per process
#else
    temp += ".tmp";
#endif
    {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        if (!out) {
            return fail("Cannot write " + temp.string());
        }
        out.write(contents.data(), static_cast<std::streamsize>(contents.size()));
        out.flush();
        if (!out) {
            out.close();
            std::filesystem::remove(temp, ec);
            return fail("Cannot write " + temp.string());
        }
    }
    std::filesystem::rename(temp, file, ec);
    if (ec) {
        std::filesystem::remove(temp, ec);
        return fail("Cannot replace " + file.string());
    }
    return true;
}

} // namespace ogun
