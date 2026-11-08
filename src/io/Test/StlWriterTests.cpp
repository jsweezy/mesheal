#include "io/StlWriter.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

namespace mesheal {

struct TempTestDir {
    std::filesystem::path path;
    explicit TempTestDir(std::string_view name) : path(std::filesystem::current_path() / name) {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
        std::filesystem::create_directories(path, ec);
    }
    ~TempTestDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

TEST_CASE("resolveAvailableOutputPath auto-increments collision-free filenames", "[io][stl_writer]") {
    TempTestDir sandbox("test_scratch_writer");
    const auto input = sandbox.path / "desk.stl";

    SECTION("Defaults to _fixed_1 when no collision exists") {
        const auto out = resolveAvailableOutputPath(input);
        CHECK(out == sandbox.path / "desk_fixed_1.stl");
    }

    SECTION("Increments to _fixed_2 and _fixed_3 when prior numbers exist") {
        const auto f1 = sandbox.path / "desk_fixed_1.stl";
        std::ofstream(f1) << "dummy";
        REQUIRE(std::filesystem::exists(f1));

        const auto out2 = resolveAvailableOutputPath(input);
        CHECK(out2 == sandbox.path / "desk_fixed_2.stl");

        const auto f2 = sandbox.path / "desk_fixed_2.stl";
        std::ofstream(f2) << "dummy";
        REQUIRE(std::filesystem::exists(f2));

        const auto out3 = resolveAvailableOutputPath(input);
        CHECK(out3 == sandbox.path / "desk_fixed_3.stl");
    }

    SECTION("Accepts valid custom output path distinct from input") {
        const auto custom = sandbox.path / "custom_out.stl";
        const auto out = resolveAvailableOutputPath(input, custom);
        CHECK(out == custom);
    }

    SECTION("Rejects output path that overwrites input file") {
        std::ofstream(input) << "dummy";
        CHECK_THROWS_AS(resolveAvailableOutputPath(input, input), std::invalid_argument);
    }
}

TEST_CASE("saveStl writes readable STL file", "[io][stl_writer]") {
    TempTestDir sandbox("test_scratch_writer_save");
    const auto out = sandbox.path / "cube.stl";
    const Mesh cube = SampleMeshes::createCubeMesh();

    std::string err;
    REQUIRE(saveStl(out, cube, StlFormat::Binary, &err));
    CHECK(std::filesystem::exists(out));
    CHECK(std::filesystem::file_size(out) > 0);
}

} // namespace mesheal
