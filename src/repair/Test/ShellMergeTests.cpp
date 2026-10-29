#include "repair/ShellMerge.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <numeric>
#include <vector>

namespace {

ogun::Mesh cubeAt(const Eigen::RowVector3d& offset, double side = 2.0) {
    ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(side);
    cube.V.rowwise() += offset;
    cube.recomputeNormals();
    return cube;
}

/// Unwelded shells side by side; each gets 12 consecutive faces.
ogun::Mesh shells(const std::vector<ogun::Mesh>& parts) {
    ogun::Mesh mesh;
    for (const ogun::Mesh& part : parts) {
        appendMesh(mesh, part);
    }
    return mesh;
}

std::vector<Eigen::Index> faceRange(Eigen::Index first, Eigen::Index count) {
    std::vector<Eigen::Index> faces(static_cast<std::size_t>(count));
    std::iota(faces.begin(), faces.end(), first);
    return faces;
}

std::size_t count(ogun::DefectCategory category, const ogun::Mesh& mesh) {
    return ogun::runCategoryCheck(category, mesh, ogun::CheckParams{}).size();
}

bool closedAndClean(const ogun::Mesh& mesh) {
    return count(ogun::DefectCategory::OpenEdges, mesh) == 0 && count(ogun::DefectCategory::NonManifoldEdges, mesh) == 0 &&
           count(ogun::DefectCategory::FlippedNormals, mesh) == 0 && count(ogun::DefectCategory::Intersections, mesh) == 0;
}

} // namespace

TEST_CASE("Merging two overlapping cubes gives one closed shell without crossings", "[repair][merge]") {
    const ogun::Mesh mesh = shells({cubeAt({0.0, 0.0, 0.0}), cubeAt({1.0, 1.0, 1.0})});
    REQUIRE(count(ogun::DefectCategory::Intersections, mesh) > 0);

    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, 12), faceRange(12, 12)};
    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE(r.success);
    REQUIRE(r.shellsIn == 2);
    REQUIRE(r.shellsOut == 1);
    REQUIRE(r.firstMergedFace == 0);
    REQUIRE(r.volumeBefore == Catch::Approx(16.0));
    REQUIRE(r.volumeAfter == Catch::Approx(15.0)); // the 1 x 1 x 1 overlap counts once
    REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(15.0));
    REQUIRE(r.unchangedFraction < 1.0);
    REQUIRE(closedAndClean(r.mesh));
    REQUIRE(ogun::checkShells(r.mesh).infoCount == 1);
}

TEST_CASE("Shells left out of the merge are not touched", "[repair][merge]") {
    const ogun::Mesh bystander = cubeAt({10.0, 0.0, 0.0});
    const ogun::Mesh mesh = shells({cubeAt({0.0, 0.0, 0.0}), bystander, cubeAt({1.0, 1.0, 1.0})});
    std::vector<Eigen::Index> second = faceRange(24, 12);
    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, 12), second};

    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE(r.success);
    REQUIRE(r.firstMergedFace == 12);
    REQUIRE(ogun::faceDiff(bystander, ogun::extractFaces(r.mesh, faceRange(0, 12))).removed.empty());
    REQUIRE(ogun::checkShells(r.mesh).infoCount == 2);
    REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(15.0 + 8.0));
}

TEST_CASE("Merging shells that do not touch keeps them apart and unchanged", "[repair][merge]") {
    const ogun::Mesh mesh = shells({cubeAt({0.0, 0.0, 0.0}), cubeAt({5.0, 0.0, 0.0})});
    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, 12), faceRange(12, 12)};
    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE(r.success);
    REQUIRE(r.shellsOut == 2);
    REQUIRE(r.volumeAfter == Catch::Approx(16.0));
    REQUIRE(r.unchangedFraction == 1.0);
    REQUIRE(closedAndClean(r.mesh));
}

TEST_CASE("Shells touching along an edge merge into valid topology", "[repair][merge]") {
    // As loaded from STL: the touching edge is already split, so there are two shells
    const ogun::Mesh cube = ogun::SampleMeshes::createGridCubeMesh(2, 2.0);
    ogun::Mesh shifted = cube;
    shifted.V.rowwise() += Eigen::RowVector3d(2.0, 2.0, 0.0);
    const ogun::Mesh mesh = shells({cube, shifted});
    const auto n = cube.numFaces();
    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, n), faceRange(n, n)};

    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE(r.success);
    REQUIRE(r.volumeAfter == Catch::Approx(16.0));
    REQUIRE(count(ogun::DefectCategory::OpenEdges, r.mesh) == 0);
    REQUIRE(count(ogun::DefectCategory::FlippedNormals, r.mesh) == 0);
    // Loading the result the way an STL is loaded leaves no non-manifold edges or vertices
    const ogun::Mesh separated = ogun::separateTouchingContacts(r.mesh);
    REQUIRE(count(ogun::DefectCategory::NonManifoldEdges, separated) == 0);
    REQUIRE(count(ogun::DefectCategory::NonManifoldVertices, separated) == 0);
}

TEST_CASE("Manifold rejects an open shell and the merge reports which one", "[repair][merge]") {
    ogun::Mesh open = cubeAt({1.0, 1.0, 1.0});
    open.F.conservativeResize(11, 3);
    const ogun::Mesh mesh = shells({cubeAt({0.0, 0.0, 0.0}), open});
    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, 12), faceRange(12, 11)};

    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE_FALSE(r.success);
    REQUIRE(r.rejectedShell == 1);
    REQUIRE_FALSE(r.message.empty());
    REQUIRE(r.mesh.empty());

    const std::vector<std::vector<Eigen::Index>> one = {faceRange(0, 12)};
    REQUIRE_FALSE(ogun::mergeShells(mesh, one).success);
}

TEST_CASE("Turkey thin faces after merging all shells at 100 to 1", "[repair][merge][thin-faces][.slow]") {
    const ogun::Mesh turkey = ogun::loadStl(
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl");
    const ogun::CategoryResult shellRows = ogun::checkShells(turkey);
    REQUIRE(shellRows.infoCount == 28);
    std::vector<std::vector<Eigen::Index>> parts;
    for (std::size_t s = 0; s < shellRows.size(); ++s) {
        const auto faces = shellRows.facesOf(s);
        parts.emplace_back(faces.begin(), faces.end());
    }

    const ogun::ThinFaceThresholds legacy{ogun::settings::kLegacyThinFaceMinAltitude, ogun::settings::kLegacyThinFaceMinArea,
                                          ogun::settings::kLegacyThinFaceMaxAspectRatio};
    const std::size_t before = ogun::ThinFaceChecker::check(turkey).numDefects();
    const ogun::ShellMergeResult r = ogun::mergeShells(turkey, parts);
    REQUIRE(r.success);
    const ogun::ThinFaceReport afterReport = ogun::ThinFaceChecker::check(r.mesh);
    const std::size_t after = afterReport.numDefects();
    const std::size_t afterLegacy = ogun::ThinFaceChecker::check(r.mesh, legacy).numDefects();
    std::array<std::size_t, ogun::kThinFaceSubtypeCount> bySubtype{};
    for (const ogun::ThinFaceDefect& defect : afterReport.defects) {
        ++bySubtype[static_cast<std::size_t>(ogun::classifyThinDefect(defect))];
    }
    WARN("Turkey thin faces: " << before << " before the merge, " << after << " after it at 100 : 1 (zero-area "
                               << bySubtype[0] << ", needle " << bySubtype[1] << ", cap " << bySubtype[2] << ", small "
                               << bySubtype[3] << "; " << afterLegacy << " at the old thresholds); faces "
                               << turkey.numFaces() << " -> " << r.mesh.numFaces() << ", shells " << r.shellsIn
                               << " -> " << r.shellsOut);
    CHECK(after > before);
}
