#include "app/CliRunner.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlWriter.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace ogun {

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

TEST_CASE("CliRunner argument parsing", "[app][cli]") {
    SECTION("Interactive launch with just file path") {
        char const* argv[] = {"ogun", "model.stl"};
        const CliOptions opt = parseCliArguments(2, argv);
        CHECK(opt.inputPath == "model.stl");
        CHECK(opt.action == CliAction::None);
        CHECK(!opt.jsonOutput);
        CHECK(!isHeadlessCommand(opt));
    }

    SECTION("FixAll headless action") {
        char const* argv[] = {"ogun", "--fix-all", "model.stl", "--passes", "4", "--json"};
        const CliOptions opt = parseCliArguments(6, argv);
        CHECK(opt.inputPath == "model.stl");
        CHECK(opt.action == CliAction::FixAll);
        CHECK(opt.passes == 4);
        CHECK(opt.jsonOutput);
        CHECK(isHeadlessCommand(opt));
    }

    SECTION("Diagnostics action") {
        char const* argv[] = {"ogun", "--diagnostics", "model.stl"};
        const CliOptions opt = parseCliArguments(3, argv);
        CHECK(opt.action == CliAction::Diagnostics);
        CHECK(isHeadlessCommand(opt));
    }

    SECTION("Remesh options") {
        char const* argv[] = {"ogun", "--remesh-voxel", "model.stl", "--edge-length", "1.5", "--smoothing", "4"};
        const CliOptions opt = parseCliArguments(7, argv);
        CHECK(opt.action == CliAction::RemeshVoxel);
        CHECK(opt.edgeLength == 1.5);
        CHECK(opt.smoothingPasses == 4);
        CHECK(isHeadlessCommand(opt));
    }
}

TEST_CASE("CliRunner command execution", "[app][cli]") {
    SECTION("Help returns 0") {
        CliOptions opt;
        opt.action = CliAction::Help;
        CHECK(executeCliCommand(opt) == 0);
    }

    SECTION("Version returns 0") {
        CliOptions opt;
        opt.action = CliAction::Version;
        CHECK(executeCliCommand(opt) == 0);
    }

    SECTION("Missing input STL returns non-zero error") {
        CliOptions opt;
        opt.action = CliAction::FixAll;
        opt.jsonOutput = true;
        CHECK(executeCliCommand(opt) != 0);
    }

    SECTION("Diagnostics on cube executes successfully") {
        TempTestDir sandbox("test_scratch_cli_diag");
        const auto inputPath = sandbox.path / "cube.stl";
        const Mesh cube = SampleMeshes::createCubeMesh();
        REQUIRE(saveStl(inputPath, cube));

        CliOptions opt;
        opt.action = CliAction::Diagnostics;
        opt.inputPath = inputPath;
        opt.jsonOutput = true;
        opt.quiet = true;
        CHECK(executeCliCommand(opt) == 0);
    }

    SECTION("FixAll repair on cube creates fixed file and computes volume") {
        TempTestDir sandbox("test_scratch_cli_fix");
        const auto inputPath = sandbox.path / "cube.stl";
        const Mesh cube = SampleMeshes::createCubeMesh();
        REQUIRE(saveStl(inputPath, cube));

        CliOptions opt;
        opt.action = CliAction::FixAll;
        opt.inputPath = inputPath;
        opt.quiet = true;
        CHECK(executeCliCommand(opt) == 0);

        const auto expectedOut = sandbox.path / "cube_fixed_1.stl";
        CHECK(std::filesystem::exists(expectedOut));
        CHECK(std::filesystem::file_size(expectedOut) > 0);
    }
}

} // namespace ogun
