#include "io/ConfigDirectory.hpp"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace {

/// Sets (or unsets, with nullptr) an environment variable for the scope, then restores it.
class ScopedEnv {
public:
    ScopedEnv(const char* name, const char* value)
        : name_(name) {
        if (const char* old = std::getenv(name); old != nullptr) {
            old_ = old;
        }
        set(value);
    }
    ~ScopedEnv() {
        set(old_ ? old_->c_str() : nullptr);
    }
    ScopedEnv(const ScopedEnv&) = delete;
    ScopedEnv& operator=(const ScopedEnv&) = delete;

private:
    void set(const char* value) {
#ifdef _WIN32
        // An empty value removes the variable on Windows
        ::_putenv_s(name_, value != nullptr ? value : "");
#else
        if (value != nullptr) {
            ::setenv(name_, value, 1);
        } else {
            ::unsetenv(name_);
        }
#endif
    }
    const char* name_;
    std::optional<std::string> old_;
};

std::filesystem::path uniqueTempDir(const char* tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / (std::string("mesheal_") + tag + "_" + std::to_string(stamp));
}

std::string readAll(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

} // namespace

TEST_CASE("configDirectoryFor prefers XDG_CONFIG_HOME, then ~/.config", "[io][config]") {
    CHECK(mesheal::configDirectoryFor("/xdg", "/home/me") == std::filesystem::path("/xdg/mesheal"));
    CHECK(mesheal::configDirectoryFor("", "/home/me") == std::filesystem::path("/home/me/.config/mesheal"));
    CHECK(mesheal::configDirectoryFor(nullptr, "/home/me") == std::filesystem::path("/home/me/.config/mesheal"));
    CHECK(mesheal::configDirectoryFor(nullptr, nullptr).empty());
}

TEST_CASE("configDirectory honours the XDG_CONFIG_HOME environment variable", "[io][config]") {
    const std::filesystem::path dir = uniqueTempDir("xdg");
    {
        const ScopedEnv xdg("XDG_CONFIG_HOME", dir.string().c_str());
        CHECK(mesheal::configDirectory() == dir / "mesheal");
    }
    {
        const ScopedEnv xdg("XDG_CONFIG_HOME", "");
#ifdef _WIN32
        const ScopedEnv appData("APPDATA", "C:\\Users\\someone\\AppData\\Roaming");
        CHECK(mesheal::configDirectory() == std::filesystem::path("C:\\Users\\someone\\AppData\\Roaming\\mesheal"));
#else
        const ScopedEnv home("HOME", "/home/someone");
        CHECK(mesheal::configDirectory() == std::filesystem::path("/home/someone/.config/mesheal"));
#endif
    }
    CHECK_FALSE(std::filesystem::exists(dir)); // resolving never creates the directory
}

TEST_CASE("migrateLegacyConfigDirectory moves the old ogun directory once", "[io][config]") {
    const std::filesystem::path root = uniqueTempDir("migrate");
    const std::filesystem::path legacy = root / "ogun";
    const std::filesystem::path current = root / "mesheal";

    CHECK_FALSE(mesheal::migrateLegacyConfigDirectory(current)); // nothing to migrate
    REQUIRE(mesheal::writeFileAtomically(legacy / "preferences.conf", "old\n"));

    CHECK(mesheal::migrateLegacyConfigDirectory(current));
    CHECK(readAll(current / "preferences.conf") == "old\n");
    CHECK_FALSE(std::filesystem::exists(legacy));

    // An existing new directory is never overwritten
    REQUIRE(mesheal::writeFileAtomically(legacy / "preferences.conf", "again\n"));
    CHECK_FALSE(mesheal::migrateLegacyConfigDirectory(current));
    CHECK(readAll(current / "preferences.conf") == "old\n");

    std::error_code ec;
    std::filesystem::remove_all(root, ec);
}

TEST_CASE("writeFileAtomically creates parent directories and replaces the file", "[io][config]") {
    const std::filesystem::path dir = uniqueTempDir("atomic");
    const std::filesystem::path file = dir / "nested" / "prefs.conf";

    std::string error;
    REQUIRE(mesheal::writeFileAtomically(file, "first\n", &error));
    CHECK(readAll(file) == "first\n");
    REQUIRE(mesheal::writeFileAtomically(file, "second\n", &error));
    CHECK(readAll(file) == "second\n");

    // Only the file itself is left behind (no temporary files)
    std::size_t entries = 0;
    for ([[maybe_unused]] const auto& entry : std::filesystem::directory_iterator(file.parent_path())) {
        ++entries;
    }
    CHECK(entries == 1);

    CHECK_FALSE(mesheal::writeFileAtomically({}, "x", &error));
    CHECK_FALSE(error.empty());
    std::filesystem::remove_all(dir);
}
