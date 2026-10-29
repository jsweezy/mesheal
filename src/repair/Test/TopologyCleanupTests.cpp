#include "repair/TopologyCleanup.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace {

using ogun::DefectCategory;
using ogun::Mesh;
using ogun::SampleMeshes;

std::size_t count(DefectCategory category, const Mesh& mesh) {
    return ogun::runCategoryCheck(category, mesh, ogun::CheckParams{}).size();
}

/// `mesh` with `faces` appended (normals left as they are, so faces with a bad index are allowed).
Mesh withFaces(Mesh mesh, const std::vector<std::array<int, 3>>& faces) {
    const Eigen::Index first = mesh.F.rows();
    mesh.F.conservativeResize(first + static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t i = 0; i < faces.size(); ++i) {
        mesh.F.row(first + static_cast<Eigen::Index>(i)) << faces[i][0], faces[i][1], faces[i][2];
    }
    return mesh;
}

/// Every kept face of `before` sits at its mapped row of `after` unchanged.
void requireFaceMap(const Mesh& before, const Mesh& after, const std::vector<Eigen::Index>& faceMap) {
    REQUIRE(faceMap.size() == static_cast<std::size_t>(before.numFaces()));
    Eigen::Index kept = 0;
    Eigen::Index last = -1;
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        const Eigen::Index row = faceMap[static_cast<std::size_t>(f)];
        if (row < 0) {
            continue;
        }
        REQUIRE(row > last); // order kept
        last = row;
        ++kept;
        REQUIRE(after.F.row(row) == before.F.row(f));
    }
    REQUIRE(kept == after.numFaces());
    REQUIRE(after.V == before.V);
    REQUIRE(after.N.rows() == after.F.rows());
}

} // namespace

TEST_CASE("Faces that repeat a vertex or use a bad index are removed and no edge count changes",
          "[repair][topology][degenerate]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    const Mesh mesh = withFaces(cube, {{6, 6, 7}, {0, 1, 99}, {3, 3, 3}});
    const ogun::EdgeTopology topoBefore = ogun::buildEdgeTopology(mesh);
    REQUIRE(count(DefectCategory::Degenerate, mesh) == 3);

    const ogun::RepeatedVertexFacesResult result = ogun::removeRepeatedVertexFaces(mesh);
    REQUIRE(result.removed == 3);
    REQUIRE(result.mesh.F == cube.F);
    requireFaceMap(mesh, result.mesh, result.faceMap);
    REQUIRE(result.faceMap[12] == -1);
    REQUIRE(result.faceMap[13] == -1);
    REQUIRE(result.faceMap[14] == -1);
    const ogun::EdgeTopology topoAfter = ogun::buildEdgeTopology(result.mesh);
    REQUIRE(topoAfter.edges == topoBefore.edges);
    REQUIRE(topoAfter.incidentFaces == topoBefore.incidentFaces);
    REQUIRE(count(DefectCategory::Degenerate, result.mesh) == 0);
    REQUIRE(count(DefectCategory::OpenEdges, result.mesh) == 0);

    SECTION("faces out of scope are kept") {
        std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 1);
        scope[13] = 0;
        const ogun::RepeatedVertexFacesResult scoped = ogun::removeRepeatedVertexFaces(mesh, scope);
        REQUIRE(scoped.removed == 2);
        requireFaceMap(mesh, scoped.mesh, scoped.faceMap);
        REQUIRE(scoped.faceMap[13] == 12);
        REQUIRE(scoped.mesh.F.row(12) == mesh.F.row(13));
    }

    SECTION("a clean mesh is unchanged") {
        const ogun::RepeatedVertexFacesResult clean = ogun::removeRepeatedVertexFaces(cube);
        REQUIRE(clean.removed == 0);
        REQUIRE(clean.mesh.F == cube.F);
        REQUIRE(clean.mesh.N == cube.N);
    }
}

TEST_CASE("Copies wound the same way keep the first face", "[repair][topology][duplicates]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    // Two rotations of top face 2 (4, 5, 6) and one of right face 10 (1, 2, 6)
    const Mesh mesh = withFaces(cube, {{5, 6, 4}, {6, 4, 5}, {2, 6, 1}});
    REQUIRE(count(DefectCategory::Duplicates, mesh) == 3);

    const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(mesh);
    REQUIRE(result.groups == 2);
    REQUIRE(result.removedSameWinding == 3);
    REQUIRE(result.removedOppositeWinding == 0);
    REQUIRE(result.groupsSkipped == 0);
    REQUIRE(result.mesh.F == cube.F);
    requireFaceMap(mesh, result.mesh, result.faceMap);
    REQUIRE(count(DefectCategory::Duplicates, result.mesh) == 0);
    REQUIRE(count(DefectCategory::NonManifoldEdges, result.mesh) == 0);
    REQUIRE(count(DefectCategory::OpenEdges, result.mesh) == 0);
}

TEST_CASE("A copy wound the other way inside a closed cube: the face its neighbours agree with stays",
          "[repair][topology][duplicates]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    for (const bool copyFirst : {false, true}) {
        // The reversed back face (2, 7, 3), placed after or before the cube's own faces
        Mesh mesh;
        if (copyFirst) {
            mesh = withFaces(Mesh{cube.V, Eigen::MatrixXi(0, 3), Eigen::MatrixXd(0, 3), cube.fileUnit}, {{2, 7, 3}});
            mesh.F.conservativeResize(13, 3);
            mesh.F.bottomRows(12) = cube.F;
        } else {
            mesh = withFaces(cube, {{2, 7, 3}});
        }
        mesh.recomputeNormals();
        REQUIRE(count(DefectCategory::Duplicates, mesh) == 1);

        const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(mesh);
        REQUIRE(result.removedOppositeWinding == 1);
        REQUIRE(result.removedSameWinding == 0);
        REQUIRE(result.mesh.F == cube.F);
        requireFaceMap(mesh, result.mesh, result.faceMap);
        REQUIRE(count(DefectCategory::OpenEdges, result.mesh) == 0);
        REQUIRE(count(DefectCategory::NonManifoldEdges, result.mesh) == 0);
        REQUIRE(count(DefectCategory::FlippedNormals, result.mesh) == 0);
    }
}

TEST_CASE("Copies wound both ways with no neighbour to agree with follow the parity rule",
          "[repair][topology][duplicates]") {
    const Mesh cube = SampleMeshes::createCubeMesh();
    Mesh base = cube;
    base.V.conservativeResize(11, 3);
    base.V.row(8) << 20.0, 0.0, 0.0;
    base.V.row(9) << 21.0, 0.0, 0.0;
    base.V.row(10) << 20.0, 1.0, 0.0;

    SECTION("an opposite pair: both go, the cube is untouched") {
        const Mesh mesh = withFaces(base, {{8, 9, 10}, {8, 10, 9}});
        const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(mesh);
        REQUIRE(result.removedOppositeWinding == 2);
        REQUIRE(result.mesh.F == cube.F);
        REQUIRE(result.faceMap[12] == -1);
        REQUIRE(result.faceMap[13] == -1);
        requireFaceMap(mesh, result.mesh, result.faceMap);
    }
    SECTION("two one way and one the other: one face of the majority stays") {
        const Mesh mesh = withFaces(base, {{8, 10, 9}, {8, 9, 10}, {9, 10, 8}});
        const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(mesh);
        REQUIRE(result.removedOppositeWinding == 2);
        REQUIRE(result.mesh.numFaces() == 13);
        REQUIRE(result.faceMap[13] == 12); // the first of the majority (8, 9, 10)
        requireFaceMap(mesh, result.mesh, result.faceMap);
    }
    SECTION("a wall between two boxes that touch across a face is removed") {
        // Two unit boxes sharing the face x = 1; the shared square is stored once per box, wound both ways
        Mesh boxes = SampleMeshes::combineMeshes({SampleMeshes::createBox({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}),
                                                  SampleMeshes::createBox({1.0, 0.0, 0.0}, {2.0, 1.0, 1.0})});
        // Weld the second box's x = 1 corners onto the first box's
        for (Eigen::Index v = 8; v < 16; ++v) {
            for (Eigen::Index w = 0; w < 8; ++w) {
                if (boxes.V.row(v) == boxes.V.row(w)) {
                    for (Eigen::Index f = 0; f < boxes.numFaces(); ++f) {
                        for (int c = 0; c < 3; ++c) {
                            if (boxes.F(f, c) == v) {
                                boxes.F(f, c) = static_cast<int>(w);
                            }
                        }
                    }
                }
            }
        }
        REQUIRE(count(DefectCategory::Duplicates, boxes) == 2);
        const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(boxes);
        REQUIRE(result.groups == 2);
        REQUIRE(result.removedOppositeWinding == 4);
        REQUIRE(result.mesh.numFaces() == 20);
        REQUIRE(count(DefectCategory::OpenEdges, result.mesh) == 0);
        REQUIRE(count(DefectCategory::NonManifoldEdges, result.mesh) == 0);
        REQUIRE(count(DefectCategory::FlippedNormals, result.mesh) == 0);
    }
}

TEST_CASE("Duplicate removal leaves faces out of scope untouched", "[repair][topology][duplicates][scope]") {
    // Two cubes apart, each with a same-winding copy of its top face; only the first cube is in scope
    const Mesh a = withFaces(SampleMeshes::createBox({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}), {{5, 6, 4}});
    const Mesh b = withFaces(SampleMeshes::createBox({5.0, 0.0, 0.0}, {6.0, 1.0, 1.0}), {{6, 4, 5}});
    Mesh mesh = SampleMeshes::combineMeshes({a, b});
    mesh.recomputeNormals();
    REQUIRE(count(DefectCategory::Duplicates, mesh) == 2);
    std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::fill(scope.begin(), scope.begin() + 13, std::uint8_t{1});

    const ogun::DuplicateFacesResult result = ogun::removeDuplicateFaces(mesh, scope);
    REQUIRE(result.removedSameWinding == 1);
    requireFaceMap(mesh, result.mesh, result.faceMap);
    for (std::size_t f = 13; f < 26; ++f) {
        REQUIRE(result.faceMap[f] == static_cast<Eigen::Index>(f) - 1);
    }
    REQUIRE(count(DefectCategory::Duplicates, result.mesh) == 1);

    SECTION("face scope: a copy in scope goes when its twin outside stays") {
        // Only the copy (face 12) of the first cube's top face is in scope
        std::vector<std::uint8_t> copyOnly(static_cast<std::size_t>(mesh.numFaces()), 0);
        copyOnly[12] = 1;
        const ogun::DuplicateFacesResult copy = ogun::removeDuplicateFaces(mesh, copyOnly);
        REQUIRE(copy.removedSameWinding == 1);
        REQUIRE(copy.faceMap[12] == -1);

        // The original (top face 2) in scope, the copy out: the original goes, the copy stays
        std::vector<std::uint8_t> originalOnly(static_cast<std::size_t>(mesh.numFaces()), 0);
        constexpr Eigen::Index top = 2;
        originalOnly[top] = 1;
        const ogun::DuplicateFacesResult original = ogun::removeDuplicateFaces(mesh, originalOnly);
        REQUIRE(original.removedSameWinding == 1);
        REQUIRE(original.faceMap[static_cast<std::size_t>(top)] == -1);
        REQUIRE(original.faceMap[12] >= 0);
    }
    SECTION("a copy wound the wrong way that is out of scope leaves the set whole") {
        // Reversed back face of a closed cube: the reversed copy must go, but it is out of scope
        Mesh cube = withFaces(SampleMeshes::createCubeMesh(), {{2, 7, 3}});
        cube.recomputeNormals();
        std::vector<std::uint8_t> keepCopy(13, 1);
        keepCopy[12] = 0;
        const ogun::DuplicateFacesResult skipped = ogun::removeDuplicateFaces(cube, keepCopy);
        REQUIRE(skipped.removed() == 0);
        REQUIRE(skipped.groupsSkipped == 1);
        REQUIRE(skipped.mesh.F == cube.F);
    }
}

TEST_CASE("The doubled faces cube sample has 2 degenerate and 2 duplicate faces and cleans to a closed cube",
          "[repair][topology][samples]") {
    const Mesh mesh = SampleMeshes::createDoubledFacesCubeMesh();
    REQUIRE(count(DefectCategory::Degenerate, mesh) == 2);
    REQUIRE(count(DefectCategory::Duplicates, mesh) == 2);

    const ogun::RepeatedVertexFacesResult dropped = ogun::removeRepeatedVertexFaces(mesh);
    REQUIRE(dropped.removed == 1);
    REQUIRE(dropped.faceMap[SampleMeshes::kRepeatedVertexFaceIndex] == -1);
    const ogun::DuplicateFacesResult deduped = ogun::removeDuplicateFaces(dropped.mesh);
    REQUIRE(deduped.removedSameWinding == 1);
    REQUIRE(deduped.removedOppositeWinding == 1);
    REQUIRE(deduped.mesh.numFaces() == SampleMeshes::kFlatCapFaceIndex + 1);
    REQUIRE(count(DefectCategory::OpenEdges, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::NonManifoldEdges, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::FlippedNormals, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::Degenerate, deduped.mesh) == 1); // the flat cap, left for the healer
}
