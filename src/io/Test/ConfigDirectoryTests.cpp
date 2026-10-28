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
        if (value != nullptr) {
            ::setenv(name_, value, 1);
        } else {
            ::unsetenv(name_);
        }
    }
    const char* name_;
    std::optional<std::string> old_;
};

std::filesystem::path uniqueTempDir(const char* tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / (std::string("ogun_") + tag + "_" + std::to_string(stamp));
}

std::string readAll(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

} // namespace

TEST_CASE("configDirectoryFor prefers XDG_CONFIG_HOME, then ~/.config", "[io][config]") {
    CHECK(ogun::configDirectoryFor("/xdg", "/home/me") == std::filesystem::path("/xdg/ogun"));
    CHECK(ogun::configDirectoryFor("", "/home/me") == std::filesystem::path("/home/me/.config/ogun"));
    CHECK(ogun::configDirectoryFor(nullptr, "/home/me") == std::filesystem::path("/home/me/.config/ogun"));
    CHECK(ogun::configDirectoryFor(nullptr, nullptr).empty());
}

TEST_CASE("configDirectory honours the XDG_CONFIG_HOME environment variable", "[io][config]") {
    const std::filesystem::path dir = uniqueTempDir("xdg");
    {
        const ScopedEnv xdg("XDG_CONFIG_HOME", dir.string().c_str());
        CHECK(ogun::configDirectory() == dir / "ogun");
    }
    {
        const ScopedEnv xdg("XDG_CONFIG_HOME", "");
        const ScopedEnv home("HOME", "/home/someone");
        CHECK(ogun::configDirectory() == std::filesystem::path("/home/someone/.config/ogun"));
    }
    CHECK_FALSE(std::filesystem::exists(dir)); // resolving never creates the directory
}

TEST_CASE("writeFileAtomically creates parent directories and replaces the file", "[io][config]") {
    const std::filesystem::path dir = uniqueTempDir("atomic");
    const std::filesystem::path file = dir / "nested" / "prefs.conf";

    std::string error;
    REQUIRE(ogun::writeFileAtomically(file, "first\n", &error));
    CHECK(readAll(file) == "first\n");
    REQUIRE(ogun::writeFileAtomically(file, "second\n", &error));
    CHECK(readAll(file) == "second\n");

    // Only the file itself is left behind (no temporary files)
    std::size_t entries = 0;
    for ([[maybe_unused]] const auto& entry : std::filesystem::directory_iterator(file.parent_path())) {
        ++entries;
    }
    CHECK(entries == 1);

    CHECK_FALSE(ogun::writeFileAtomically({}, "x", &error));
    CHECK_FALSE(error.empty());
    std::filesystem::remove_all(dir);
}
