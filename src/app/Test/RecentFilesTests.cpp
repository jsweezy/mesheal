#include "app/RecentFiles.hpp"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

namespace fs = std::filesystem;

/// Fresh directory under the system temp dir (never the real ~/.config/mesheal).
fs::path uniqueTempDir(const char* tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() / (std::string("mesheal_recent_") + tag + "_" + std::to_string(stamp));
}

fs::path abs(const char* name) {
    return fs::absolute(fs::path("/mesheal_recent_test") / name).lexically_normal();
}

} // namespace

TEST_CASE("RecentFiles add moves to front and collapses duplicates", "[recent]") {
    mesheal::RecentFiles recent;
    REQUIRE(recent.empty());
    recent.add(abs("a.stl"));
    recent.add(abs("b.stl"));
    recent.add(abs("c.stl"));
    recent.add(abs("a.stl"));
    REQUIRE(recent.entries().size() == 3);
    CHECK(recent.entries()[0] == abs("a.stl"));
    CHECK(recent.entries()[1] == abs("c.stl"));
    CHECK(recent.entries()[2] == abs("b.stl"));

    // A differently spelled path to the same file is the same entry
    recent.add("/mesheal_recent_test/x/../b.stl");
    REQUIRE(recent.entries().size() == 3);
    CHECK(recent.entries()[0] == abs("b.stl"));
}

TEST_CASE("RecentFiles keeps at most ten entries", "[recent]") {
    mesheal::RecentFiles recent;
    for (int i = 0; i < 15; ++i) {
        recent.add(abs((std::to_string(i) + ".stl").c_str()));
    }
    REQUIRE(recent.entries().size() == mesheal::kMaxRecentFiles);
    CHECK(recent.entries().front() == abs("14.stl"));
    CHECK(recent.entries().back() == abs("5.stl"));
}

TEST_CASE("RecentFiles stores relative paths as absolute", "[recent]") {
    mesheal::RecentFiles recent;
    recent.add("models/./part.stl");
    REQUIRE(recent.entries().size() == 1);
    CHECK(recent.entries()[0].is_absolute());
    CHECK(recent.entries()[0] == (fs::current_path() / "models" / "part.stl").lexically_normal());
}

TEST_CASE("RecentFiles save and load round trip", "[recent]") {
    const fs::path dir = uniqueTempDir("roundtrip");
    const fs::path file = mesheal::recentFilesPath(dir);
    {
        mesheal::RecentFiles recent(file);
        recent.add(abs("a.stl"));
        recent.add(abs("b c.stl"));
        recent.add(abs("d.stl"));
        std::string error;
        REQUIRE(recent.save(&error));
    }
    mesheal::RecentFiles loaded(file);
    loaded.load();
    REQUIRE(loaded.entries().size() == 3);
    CHECK(loaded.entries()[0] == abs("d.stl"));
    CHECK(loaded.entries()[1] == abs("b c.stl"));
    CHECK(loaded.entries()[2] == abs("a.stl"));
    fs::remove_all(dir);
}

TEST_CASE("RecentFiles load tolerates a missing or malformed file", "[recent]") {
    const fs::path dir = uniqueTempDir("malformed");
    mesheal::RecentFiles missing(mesheal::recentFilesPath(dir));
    missing.load();
    CHECK(missing.empty());

    fs::create_directories(dir);
    const fs::path file = mesheal::recentFilesPath(dir);
    {
        std::ofstream out(file);
        out << "\n  \nrelative/path.stl\n" << abs("a.stl").string() << "\r\n" << abs("a.stl").string() << "\n";
        for (int i = 0; i < 20; ++i) {
            out << abs((std::to_string(i) + ".stl").c_str()).string() << "\n";
        }
    }
    mesheal::RecentFiles loaded(file);
    loaded.load();
    REQUIRE(loaded.entries().size() == mesheal::kMaxRecentFiles);
    CHECK(loaded.entries()[0] == abs("a.stl"));
    CHECK(loaded.entries()[1] == abs("0.stl"));
    fs::remove_all(dir);
}

TEST_CASE("RecentFiles clear empties the list", "[recent]") {
    mesheal::RecentFiles recent;
    recent.add(abs("a.stl"));
    recent.clear();
    CHECK(recent.empty());
}
