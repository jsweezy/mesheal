// The whole-surface rebuilds: alpha wrap stops mid-run through CGAL's wrapping visitor.

#include "repair/SurfaceRebuild.hpp"

#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/GlobalRepair.hpp"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <stop_token>
#include <string>

namespace {

using mesheal::Mesh;
using mesheal::SampleMeshes;

/// A box wrapped finely enough to take thousands of flood-fill steps, coarsely enough to be quick in Debug.
constexpr double kDetail = 15.0;

} // namespace

TEST_CASE("Alpha wrap stops mid-run when its stop token is set", "[repair][rebuild][alpha-wrap][cancel]") {
    const Mesh cube = SampleMeshes::createBox({0, 0, 0}, {2, 2, 2});
    const double alpha = cube.boundingBox().diagonal() / kDetail;
    const double offset = alpha / 30.0;

    // The whole wrap, counting its steps
    std::size_t fullSteps = 0;
    std::string status;
    const Mesh full = mesheal::alphaWrap(cube, alpha, offset, &status, {}, [&](std::size_t step) { fullSteps = step + 1; });
    REQUIRE_FALSE(full.empty());
    REQUIRE(fullSteps > 100);
    CHECK(status.starts_with("Wrapped with alpha"));

    // Stopped half way: the visitor is asked once more, says stop, and the partial wrap is dropped
    std::stop_source source;
    const std::size_t stopAt = fullSteps / 2;
    std::size_t lastStep = 0;
    const Mesh stopped = mesheal::alphaWrap(cube, alpha, offset, &status, source.get_token(), [&](std::size_t step) {
        lastStep = step;
        if (step == stopAt) {
            source.request_stop();
        }
    });
    CHECK(stopped.empty());
    CHECK(status == "Cancelled.");
    CHECK(lastStep == stopAt); // no step ran after the stop

    // Stopped before it starts: the flood fill takes no step
    std::stop_source before;
    before.request_stop();
    std::size_t steps = 0;
    CHECK(mesheal::alphaWrap(cube, alpha, offset, &status, before.get_token(), [&](std::size_t) { ++steps; }).empty());
    CHECK(steps == 1);
}

TEST_CASE("Repair All alpha wrap passes its stop token to the wrap", "[repair][rebuild][alpha-wrap][cancel]") {
    const Mesh cube = SampleMeshes::createBox({0, 0, 0}, {2, 2, 2});
    mesheal::GlobalRepairParams params;
    // A fine wrap: about 1.2 s in Release (15 s or more in Debug), its set-up before the first stop check
    // about 4 % of that (see the timings benchmark), so it only returns this soon if the wrap stopped
    params.alphaWrapDetail = 200.0;
    std::stop_source source;
    source.request_stop();
    const auto start = std::chrono::steady_clock::now();
    const mesheal::RepairCandidate candidate =
        mesheal::runGlobalRepair(cube, {}, mesheal::GlobalRepairMethod::AlphaWrap, params, source.get_token());
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(6));
    CHECK(candidate.cancelled);
    CHECK_FALSE(candidate.success);
    CHECK(candidate.statusMessage == "Cancelled.");
}

TEST_CASE("Alpha wrap timings: set-up before the first stop check vs the whole wrap", "[repair][.benchmark]") {
    // Before its first step CGAL splits the input triangles to about 3 alpha and builds its tree; no stop
    // check runs there. Large flat triangles at a fine detail make that part long.
    const auto examples = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples";
    const Mesh bunny = mesheal::loadStl(examples / "stanford_bunny.stl");
    const Mesh cube = SampleMeshes::createBox({0, 0, 0}, {2, 2, 2});
    struct Case {
        const char* name;
        const Mesh* mesh;
        double detail;
    };
    for (const Case& c : {Case{"cube", &cube, 100.0}, Case{"cube", &cube, 300.0}, Case{"bunny", &bunny, 100.0},
                          Case{"bunny", &bunny, 300.0}}) {
        const double alpha = c.mesh->boundingBox().diagonal() / c.detail;
        const auto start = std::chrono::steady_clock::now();
        double firstStepMs = -1.0;
        std::size_t steps = 0;
        const Mesh wrap = mesheal::alphaWrap(*c.mesh, alpha, alpha / 30.0, nullptr, {}, [&](std::size_t step) {
            if (step == 0) {
                firstStepMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            }
            steps = step + 1;
        });
        const double totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        std::cout << "Alpha wrap " << c.name << " detail " << c.detail << ": first stop check after " << firstStepMs
                  << " ms, " << steps << " steps, " << wrap.numFaces() << " faces, total " << totalMs << " ms\n";
        CHECK_FALSE(wrap.empty());
    }
}
