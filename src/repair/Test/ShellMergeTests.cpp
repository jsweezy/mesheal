#include "repair/ShellMerge.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <numeric>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

/// The 2 x 2 x 2 box with its min corner at `min` (12 faces).
ogun::Mesh box2(const Eigen::Vector3d& min) {
    return ogun::SampleMeshes::createBox(min, min + Eigen::Vector3d::Constant(2.0));
}

/// Unwelded shells side by side; each box gets 12 consecutive faces.
ogun::Mesh shells(const std::vector<ogun::Mesh>& parts) {
    return ogun::SampleMeshes::combineMeshes(parts);
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
    const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}), box2({1.0, 1.0, 1.0})});
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
    const ogun::Mesh bystander = box2({10.0, 0.0, 0.0});
    const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}), bystander, box2({1.0, 1.0, 1.0})});
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
    const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}), box2({5.0, 0.0, 0.0})});
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

TEST_CASE("An open operand among two good ones is left out and the others merge", "[repair][merge]") {
    ogun::Mesh open = box2({10.0, 0.0, 0.0});
    open.F.conservativeResize(11, 3);
    const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}), box2({1.0, 1.0, 1.0}), open});
    const std::vector<std::vector<Eigen::Index>> parts = {faceRange(0, 12), faceRange(12, 12), faceRange(24, 11)};

    REQUIRE_FALSE(ogun::manifoldAccepts(mesh, parts[2]).accepted);
    REQUIRE_FALSE(ogun::manifoldAccepts(mesh, parts[2]).reason.empty());
    REQUIRE(ogun::manifoldAccepts(mesh, parts[0]).accepted);
    REQUIRE(ogun::manifoldAccepts(mesh, parts[0]).volume == Catch::Approx(8.0));

    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
    REQUIRE(r.success);
    REQUIRE(r.operandsMerged == 2);
    REQUIRE(r.unionShells == 1);
    REQUIRE(r.shellsOut == 2);
    REQUIRE(r.refusedShells.size() == 1);
    REQUIRE(r.refusedShells[0].operand == 2);
    REQUIRE(r.refusedShells[0].why == ogun::MergeRefusal::NotManifold);
    REQUIRE_FALSE(r.refusedShells[0].reason.empty());
    REQUIRE(r.firstMergedFace == 11); // the open shell's faces first, as they were
    REQUIRE(ogun::faceDiff(open, ogun::extractFaces(r.mesh, faceRange(0, 11))).removed.empty());
    REQUIRE(r.volumeBefore == Catch::Approx(16.0));
    REQUIRE(r.volumeAfter == Catch::Approx(15.0));

    const std::vector<std::vector<Eigen::Index>> one = {faceRange(0, 12)};
    REQUIRE_FALSE(ogun::mergeShells(mesh, one).success);
}

TEST_CASE("Turkey thin faces after merging all shells at 100 to 1", "[repair][merge][thin-faces][.slow]") {
    const ogun::Mesh turkey = ogun::loadStl(
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl");
    const ogun::CategoryResult shellRows = ogun::checkShells(turkey);
    REQUIRE(shellRows.infoCount == 28);
    // Every outward shell; the inward one (#28, a cavity of #12) would make the merge refuse the group
    std::vector<std::vector<Eigen::Index>> parts;
    for (std::size_t s = 0; s < shellRows.size(); ++s) {
        const auto faces = shellRows.facesOf(s);
        std::vector<Eigen::Index> shell(faces.begin(), faces.end());
        if (ogun::signedVolume(ogun::extractFaces(turkey, shell)) > 0.0) {
            parts.push_back(std::move(shell));
        }
    }
    REQUIRE(parts.size() == 27);

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

namespace {

/// Faces of shell k of a fixture made of 12-face boxes.
std::vector<Eigen::Index> boxFaces(Eigen::Index k) {
    return faceRange(12 * k, 12);
}

std::string label(int k) {
    return "#" + std::to_string(k + 1);
}

/// Faces [first, first + count) of `after` are those of `before`, bit for bit.
bool sameFaces(const ogun::Mesh& before, std::span<const Eigen::Index> faces, const ogun::Mesh& after, Eigen::Index first) {
    return ogun::faceDiff(ogun::extractFaces(before, faces),
                          ogun::extractFaces(after, faceRange(first, static_cast<Eigen::Index>(faces.size()))))
        .removed.empty();
}

} // namespace

TEST_CASE("A cavity is never filled by a merge", "[repair][merge][cavity]") {
    // Cavity box: A = [0, 4]^3 outward, C = [1, 3]^3 inward inside it
    const ogun::Mesh mesh = ogun::SampleMeshes::createCavityBoxMesh();
    REQUIRE(ogun::signedVolume(mesh) == Catch::Approx(56.0));

    SECTION("passed as an operand (the old call): refused as wound inward, nothing merged") {
        // Before cavities were handled, this call united A and C into a solid A: volume 64, the hollow filled
        const std::vector<std::vector<Eigen::Index>> parts = {boxFaces(0), boxFaces(1)};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
        REQUIRE(r.success);
        REQUIRE(r.operandsMerged == 0);
        REQUIRE(r.refusedShells.size() == 1);
        REQUIRE(r.refusedShells[0].operand == 1);
        REQUIRE(r.refusedShells[0].why == ogun::MergeRefusal::InwardVolume);
        REQUIRE(r.refusedShells[0].reason.find("fix orientation first") != std::string::npos);
        REQUIRE(r.mesh.F == mesh.F);
        REQUIRE(r.mesh.V == mesh.V);
        REQUIRE(r.shellsOut == 2);
    }
    SECTION("passed as a cavity: kept bit-identical") {
        ogun::ShellMergeInput input;
        input.operands = {boxFaces(0)};
        input.cavities = {{boxFaces(1), 0}};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, input);
        REQUIRE(r.success);
        REQUIRE(r.shellsOut == 2);
        REQUIRE(r.cavitiesKept == 1);
        REQUIRE(r.cavityVolume == Catch::Approx(-8.0));
        REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(56.0));
        REQUIRE(sameFaces(mesh, boxFaces(1), r.mesh, 12));
    }
}

TEST_CASE("Overlapping boxes merge around a cavity they do not reach", "[repair][merge][cavity]") {
    // A = [0, 4]^3 and B overlap; C = [1, 2] x [1, 3]^2 inward inside A only
    const ogun::Mesh mesh = ogun::SampleMeshes::createOverlapPlusCavityMesh();
    ogun::ShellMergeInput input;
    input.operands = {boxFaces(0), boxFaces(1)};
    input.cavities = {{boxFaces(2), 0}};

    const ogun::ShellMergeResult r = ogun::mergeShells(mesh, input);
    REQUIRE(r.success);
    REQUIRE(r.refusedShells.empty());
    REQUIRE(r.refusedGroups.empty());
    REQUIRE(r.operandsMerged == 2);
    REQUIRE(r.unionShells == 1);
    REQUIRE(r.shellsIn == 3);
    REQUIRE(r.shellsOut == 2); // one solid and the cavity
    REQUIRE(r.volumeBefore == Catch::Approx(128.0)); // solids only
    REQUIRE(r.unionVolume == Catch::Approx(121.875));
    REQUIRE(r.cavitiesKept == 1);
    REQUIRE(r.cavityVolume == Catch::Approx(-4.0));
    REQUIRE(r.volumeAfter == Catch::Approx(117.875));
    REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(117.875));
    REQUIRE(r.firstMergedFace == 12);
    REQUIRE(sameFaces(mesh, boxFaces(2), r.mesh, 0)); // C bit-identical, still inward
    REQUIRE(ogun::signedVolume(ogun::extractFaces(r.mesh, faceRange(0, 12))) == Catch::Approx(-4.0));
    REQUIRE_FALSE(r.removedInputFaces.empty());
    REQUIRE(std::all_of(r.removedInputFaces.begin(), r.removedInputFaces.end(), [](Eigen::Index f) { return f < 24; }));
    REQUIRE(closedAndClean(r.mesh));

    SECTION("every shell as an operand: the inward one is refused and the group with it") {
        const std::vector<std::vector<Eigen::Index>> parts = {boxFaces(0), boxFaces(1), boxFaces(2)};
        const ogun::ShellMergeResult old = ogun::mergeShells(mesh, parts);
        REQUIRE(old.success);
        REQUIRE(old.operandsMerged == 0);
        REQUIRE(old.refusedGroups.size() == 1);
        REQUIRE(old.refusedGroups[0].inwardOperand == 2);
        REQUIRE(old.refusedGroups[0].reaching == std::vector<int>{0}); // A holds it and was not named its container
        REQUIRE(old.mesh.F == mesh.F);
        ogun::ShellMergeInput asOperands;
        asOperands.operands = parts;
        REQUIRE(ogun::describeRefusedGroup(asOperands, old.refusedGroups[0], label, label) ==
                "#1 reaches into #3, which is wound inward: fix its orientation, or merge them by hand.");
    }
}

TEST_CASE("A group that reaches into a cavity is refused and other groups still merge", "[repair][merge][cavity]") {
    using Variant = ogun::SampleMeshes::CavityReachedVariant;
    const auto run = [](const ogun::Mesh& mesh, std::size_t operandCount) {
        ogun::ShellMergeInput input;
        for (std::size_t k = 0; k < operandCount + 1; ++k) {
            if (k != 2) {
                input.operands.push_back(boxFaces(static_cast<Eigen::Index>(k)));
            }
        }
        input.cavities = {{boxFaces(2), 0}};
        return std::pair{input, ogun::mergeShells(mesh, input)};
    };
    const auto cavityLabel = [](int) { return std::string("#3"); };

    SECTION("B's wall crosses the cavity") {
        const ogun::Mesh mesh = ogun::SampleMeshes::createCavityReachedMesh(Variant::CrossesWall);
        const auto [input, r] = run(mesh, 2);
        REQUIRE(r.success);
        REQUIRE(r.operandsMerged == 0);
        REQUIRE(r.refusedGroups.size() == 1);
        REQUIRE(r.refusedGroups[0].operands == std::vector<int>{0, 1});
        REQUIRE(r.refusedGroups[0].cavities == std::vector<int>{0});
        REQUIRE(ogun::describeRefusedGroup(input, r.refusedGroups[0], label, cavityLabel) ==
                "#2 reaches into cavity #3 of #1: merge them by hand.");
        REQUIRE(r.mesh.F == mesh.F);
        REQUIRE(r.mesh.V == mesh.V);
        REQUIRE(r.shellsOut == 3);
        REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(124.8));
    }
    SECTION("the cavity lies wholly inside both boxes") {
        const ogun::Mesh mesh = ogun::SampleMeshes::createCavityReachedMesh(Variant::InsideOverlap);
        const auto [input, r] = run(mesh, 2);
        REQUIRE(r.operandsMerged == 0);
        REQUIRE(r.refusedGroups.size() == 1);
        REQUIRE(ogun::describeRefusedGroup(input, r.refusedGroups[0], label, cavityLabel) ==
                "#2 reaches into cavity #3 of #1: merge them by hand.");
        REQUIRE(r.mesh.F == mesh.F);
    }
    SECTION("a separate pair still merges") {
        const ogun::Mesh mesh = ogun::SampleMeshes::createCavityReachedMesh(Variant::WithSeparatePair);
        const auto [input, r] = run(mesh, 4);
        REQUIRE(input.operands.size() == 4);
        REQUIRE(r.success);
        REQUIRE(r.refusedGroups.size() == 1);
        REQUIRE(r.refusedGroups[0].operands == std::vector<int>{0, 1});
        REQUIRE(r.operandsMerged == 2);
        REQUIRE(r.unionShells == 1);
        REQUIRE(r.shellsIn == 5);
        REQUIRE(r.shellsOut == 4);
        REQUIRE(r.cavitiesKept == 0); // in the refused group
        REQUIRE(r.firstMergedFace == 36);
        REQUIRE(sameFaces(mesh, faceRange(0, 36), r.mesh, 0)); // A, B and the cavity as they were
    }
}

TEST_CASE("A nested solid is still absorbed and an island in a cavity is left as it is", "[repair][merge][cavity]") {
    SECTION("nested solid") {
        // A = [0, 4]^3, cavity [0.5, 1.5]^3, nested solid [2.5, 3.5]^3 touching nothing
        const ogun::Mesh mesh = ogun::SampleMeshes::createNestedSolidAndCavityMesh();
        ogun::ShellMergeInput input;
        input.operands = {boxFaces(0), boxFaces(2)};
        input.cavities = {{boxFaces(1), 0}};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, input);
        REQUIRE(r.success);
        REQUIRE(r.operandsMerged == 2);
        REQUIRE(r.unionShells == 1);
        REQUIRE(r.shellsOut == 2);
        REQUIRE(r.volumeBefore == Catch::Approx(65.0));
        REQUIRE(r.volumeAfter == Catch::Approx(63.0));
        REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(63.0));
        REQUIRE(sameFaces(mesh, boxFaces(1), r.mesh, 0));
    }
    SECTION("island") {
        // A = [0, 6]^3, cavity [1, 5]^3, island [2, 4]^3; B crosses A's wall x = 6, clear of the cavity
        const ogun::Mesh mesh = shells({ogun::SampleMeshes::createIslandInCavityMesh(),
                                        ogun::SampleMeshes::createBox({5.5, 1.0, 1.0}, {8.0, 2.0, 2.0})});
        ogun::ShellMergeInput input;
        input.operands = {boxFaces(0), boxFaces(2), boxFaces(3)};
        input.cavities = {{boxFaces(1), 0}};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, input);
        REQUIRE(r.success);
        REQUIRE(r.refusedGroups.empty());
        REQUIRE(r.refusedShells.size() == 1);
        REQUIRE(r.refusedShells[0].operand == 1);
        REQUIRE(r.refusedShells[0].why == ogun::MergeRefusal::InsideCavity);
        REQUIRE(r.refusedShells[0].cavity == 0);
        REQUIRE(r.operandsMerged == 2);
        REQUIRE(r.shellsOut == 3);
        REQUIRE(ogun::signedVolume(r.mesh) == Catch::Approx(216.0 + 2.0 - 64.0 + 8.0));
        REQUIRE(sameFaces(mesh, faceRange(12, 24), r.mesh, 0)); // cavity and island as they were
    }
}

TEST_CASE("An inside-out operand is refused with a reason", "[repair][merge][cavity]") {
    SECTION("apart from the others: they still merge") {
        const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}), box2({1.0, 1.0, 1.0}),
                                        ogun::SampleMeshes::createBox({10.0, 0.0, 0.0}, {12.0, 2.0, 2.0}, true)});
        const std::vector<std::vector<Eigen::Index>> parts = {boxFaces(0), boxFaces(1), boxFaces(2)};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
        REQUIRE(r.success);
        REQUIRE(r.operandsMerged == 2);
        REQUIRE(r.refusedGroups.empty());
        REQUIRE(r.refusedShells.size() == 1);
        REQUIRE(r.refusedShells[0].operand == 2);
        REQUIRE(r.refusedShells[0].why == ogun::MergeRefusal::InwardVolume);
        REQUIRE(r.shellsOut == 2);
        REQUIRE(sameFaces(mesh, boxFaces(2), r.mesh, 0));
    }
    SECTION("crossing another operand: that group is refused") {
        const ogun::Mesh mesh = shells({box2({0.0, 0.0, 0.0}),
                                        ogun::SampleMeshes::createBox({1.0, 1.0, 1.0}, {3.0, 3.0, 3.0}, true)});
        const std::vector<std::vector<Eigen::Index>> parts = {boxFaces(0), boxFaces(1)};
        const ogun::ShellMergeResult r = ogun::mergeShells(mesh, parts);
        REQUIRE(r.success);
        REQUIRE(r.operandsMerged == 0);
        REQUIRE(r.refusedShells.size() == 1);
        REQUIRE(r.refusedGroups.size() == 1);
        REQUIRE(r.mesh.F == mesh.F);
    }
}

TEST_CASE("Turkey merge with its cavity", "[repair][merge][cavity][.slow]") {
    const ogun::Mesh turkey = ogun::loadStl(
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl");
    const ogun::ShellRelations relations = ogun::computeShellRelations(turkey);
    const ogun::ShellRoles roles = ogun::classifyShellRoles(turkey, relations);
    ogun::ShellMergeInput input;
    std::vector<int> operandIds;
    std::vector<int> cavityIds;
    for (std::size_t s = 0; s < roles.shells.size(); ++s) {
        if (roles.shells[s].role != ogun::ShellRole::Cavity) {
            operandIds.push_back(static_cast<int>(s));
            input.operands.push_back(relations.shells[s].faces);
        }
    }
    for (std::size_t s = 0; s < roles.shells.size(); ++s) {
        if (roles.shells[s].role == ogun::ShellRole::Cavity) {
            cavityIds.push_back(static_cast<int>(s));
            const int container = relations.shells[s].nestedIn;
            const auto it = std::find(operandIds.begin(), operandIds.end(), container);
            input.cavities.push_back({relations.shells[s].faces,
                                      it == operandIds.end() ? -1 : static_cast<int>(it - operandIds.begin())});
        }
    }
    REQUIRE(cavityIds == std::vector<int>{27});
    const ogun::ShellMergeResult r = ogun::mergeShells(turkey, input);
    REQUIRE(r.success);
    const auto operandLabel = [&](int o) { return label(operandIds[static_cast<std::size_t>(o)]); };
    const auto cavityLabel = [&](int c) { return label(cavityIds[static_cast<std::size_t>(c)]); };
    for (const ogun::RefusedMergeGroup& group : r.refusedGroups) {
        WARN("Turkey: " << ogun::describeRefusedGroup(input, group, operandLabel, cavityLabel) << " (" << group.operands.size()
                        << " shells in the group)");
    }
    WARN("Turkey merge: shells " << r.shellsIn << " -> " << r.shellsOut << ", united " << r.operandsMerged << ", "
                                 << r.elapsedMs << " ms");
    // #28 is a cavity of #12, and lies wholly inside #6 and #9 too (where they overlap #12): they reach into it
    REQUIRE(relations.shells[27].containers == std::vector<int>{11, 5, 8});
    REQUIRE(r.refusedGroups.size() == 1);
    REQUIRE(r.refusedGroups[0].operands.size() == 27);
    REQUIRE(ogun::describeRefusedGroup(input, r.refusedGroups[0], operandLabel, cavityLabel) ==
            "#6 and #9 reach into cavity #28 of #12: merge them by hand.");
    REQUIRE(r.operandsMerged == 0);
    REQUIRE(r.mesh.F == turkey.F);
}
