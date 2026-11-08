#include "repair/TopologyCleanup.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <vector>

namespace {

using mesheal::DefectCategory;
using mesheal::Mesh;
using mesheal::SampleMeshes;

std::size_t count(DefectCategory category, const Mesh& mesh) {
    return mesheal::runCategoryCheck(category, mesh, mesheal::CheckParams{}).size();
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
    const mesheal::EdgeTopology topoBefore = mesheal::buildEdgeTopology(mesh);
    REQUIRE(count(DefectCategory::Degenerate, mesh) == 3);

    const mesheal::RepeatedVertexFacesResult result = mesheal::removeRepeatedVertexFaces(mesh);
    REQUIRE(result.removed == 3);
    REQUIRE(result.mesh.F == cube.F);
    requireFaceMap(mesh, result.mesh, result.faceMap);
    REQUIRE(result.faceMap[12] == -1);
    REQUIRE(result.faceMap[13] == -1);
    REQUIRE(result.faceMap[14] == -1);
    const mesheal::EdgeTopology topoAfter = mesheal::buildEdgeTopology(result.mesh);
    REQUIRE(topoAfter.edges == topoBefore.edges);
    REQUIRE(topoAfter.incidentFaces == topoBefore.incidentFaces);
    REQUIRE(count(DefectCategory::Degenerate, result.mesh) == 0);
    REQUIRE(count(DefectCategory::OpenEdges, result.mesh) == 0);

    SECTION("faces out of scope are kept") {
        std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 1);
        scope[13] = 0;
        const mesheal::RepeatedVertexFacesResult scoped = mesheal::removeRepeatedVertexFaces(mesh, scope);
        REQUIRE(scoped.removed == 2);
        requireFaceMap(mesh, scoped.mesh, scoped.faceMap);
        REQUIRE(scoped.faceMap[13] == 12);
        REQUIRE(scoped.mesh.F.row(12) == mesh.F.row(13));
    }

    SECTION("a clean mesh is unchanged") {
        const mesheal::RepeatedVertexFacesResult clean = mesheal::removeRepeatedVertexFaces(cube);
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

    const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(mesh);
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

        const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(mesh);
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
        const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(mesh);
        REQUIRE(result.removedOppositeWinding == 2);
        REQUIRE(result.mesh.F == cube.F);
        REQUIRE(result.faceMap[12] == -1);
        REQUIRE(result.faceMap[13] == -1);
        requireFaceMap(mesh, result.mesh, result.faceMap);
    }
    SECTION("two one way and one the other: one face of the majority stays") {
        const Mesh mesh = withFaces(base, {{8, 10, 9}, {8, 9, 10}, {9, 10, 8}});
        const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(mesh);
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
        const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(boxes);
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

    const mesheal::DuplicateFacesResult result = mesheal::removeDuplicateFaces(mesh, scope);
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
        const mesheal::DuplicateFacesResult copy = mesheal::removeDuplicateFaces(mesh, copyOnly);
        REQUIRE(copy.removedSameWinding == 1);
        REQUIRE(copy.faceMap[12] == -1);

        // The original (top face 2) in scope, the copy out: the original goes, the copy stays
        std::vector<std::uint8_t> originalOnly(static_cast<std::size_t>(mesh.numFaces()), 0);
        constexpr Eigen::Index top = 2;
        originalOnly[top] = 1;
        const mesheal::DuplicateFacesResult original = mesheal::removeDuplicateFaces(mesh, originalOnly);
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
        const mesheal::DuplicateFacesResult skipped = mesheal::removeDuplicateFaces(cube, keepCopy);
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

    const mesheal::RepeatedVertexFacesResult dropped = mesheal::removeRepeatedVertexFaces(mesh);
    REQUIRE(dropped.removed == 1);
    REQUIRE(dropped.faceMap[SampleMeshes::kRepeatedVertexFaceIndex] == -1);
    const mesheal::DuplicateFacesResult deduped = mesheal::removeDuplicateFaces(dropped.mesh);
    REQUIRE(deduped.removedSameWinding == 1);
    REQUIRE(deduped.removedOppositeWinding == 1);
    REQUIRE(deduped.mesh.numFaces() == SampleMeshes::kFlatCapFaceIndex + 1);
    REQUIRE(count(DefectCategory::OpenEdges, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::NonManifoldEdges, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::FlippedNormals, deduped.mesh) == 0);
    REQUIRE(count(DefectCategory::Degenerate, deduped.mesh) == 1); // the flat cap, left for the healer
}

namespace {

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

std::size_t openEdges(const Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).infoCount;
}

/// Non-manifold edges of one sub-type: 0 Touching, 1 Fin / other.
std::size_t nonManifoldEdges(const Mesh& mesh, int subtype) {
    const mesheal::CategoryResult result = mesheal::checkNonManifoldEdges(mesh);
    return static_cast<std::size_t>(std::count_if(result.elements.begin(), result.elements.end(),
                                                  [subtype](const mesheal::DefectElement& e) { return e.subtype == subtype; }));
}

/// Welds the vertices of `mesh` that sit at exactly the same position (the STL reader's exact weld).
Mesh weldExact(const Mesh& mesh) {
    std::map<std::array<double, 3>, int> index;
    std::vector<int> remap(static_cast<std::size_t>(mesh.numVertices()));
    std::vector<Eigen::RowVector3d> points;
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        const auto [it, inserted] =
            index.try_emplace({mesh.V(v, 0), mesh.V(v, 1), mesh.V(v, 2)}, static_cast<int>(points.size()));
        if (inserted) {
            points.push_back(mesh.V.row(v));
        }
        remap[static_cast<std::size_t>(v)] = it->second;
    }
    Mesh out;
    out.fileUnit = mesh.fileUnit;
    out.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        out.V.row(static_cast<Eigen::Index>(i)) = points[i];
    }
    out.F = mesh.F;
    for (Eigen::Index f = 0; f < out.F.rows(); ++f) {
        for (int c = 0; c < 3; ++c) {
            out.F(f, c) = remap[static_cast<std::size_t>(mesh.F(f, c))];
        }
    }
    out.recomputeNormals();
    return out;
}

/// Faces of `mesh` whose centroid passes `keep`, optionally reversed.
template <typename Keep>
std::vector<std::array<Eigen::Vector3d, 3>> facesWhere(const Mesh& mesh, Keep keep, bool reversed = false) {
    std::vector<std::array<Eigen::Vector3d, 3>> faces;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        std::array<Eigen::Vector3d, 3> t;
        for (int c = 0; c < 3; ++c) {
            t[static_cast<std::size_t>(c)] = mesh.V.row(mesh.F(f, c)).transpose();
        }
        if (keep((t[0] + t[1] + t[2]) / 3.0)) {
            if (reversed) {
                std::swap(t[1], t[2]);
            }
            faces.push_back(t);
        }
    }
    return faces;
}

/// A welded mesh from triangles given by their corner positions.
Mesh fromTriangles(const std::vector<std::array<Eigen::Vector3d, 3>>& triangles) {
    Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(3 * triangles.size()), 3);
    mesh.F.resize(static_cast<Eigen::Index>(triangles.size()), 3);
    for (std::size_t t = 0; t < triangles.size(); ++t) {
        for (int c = 0; c < 3; ++c) {
            const auto row = static_cast<Eigen::Index>(3 * t) + c;
            mesh.V.row(row) = triangles[t][static_cast<std::size_t>(c)].transpose();
            mesh.F(static_cast<Eigen::Index>(t), c) = static_cast<int>(row);
        }
    }
    return weldExact(mesh);
}

bool onPlane(double a, double b) {
    return std::abs(a - b) < 1.0e-12;
}

} // namespace

TEST_CASE("Fin removal takes the fin off the touching cubes and keeps the touching contact", "[repair][topology][fins]") {
    for (const bool loaded : {false, true}) {
        const Mesh raw = SampleMeshes::createTouchingCubesMesh();
        const Mesh mesh = loaded ? mesheal::separateTouchingContacts(raw) : raw;
        const std::size_t touchingBefore = nonManifoldEdges(mesh, 0);
        REQUIRE(touchingBefore == (loaded ? 0u : 2u));
        REQUIRE(nonManifoldEdges(mesh, 1) == 1);
        REQUIRE(openEdges(mesh) == 2);

        const mesheal::FinRemovalResult result = mesheal::removeFinFaces(mesh);
        REQUIRE(result.removedFaces == std::vector<Eigen::Index>{mesh.numFaces() - 1}); // the fin triangle
        REQUIRE(result.finFaces == 1);
        REQUIRE(result.wallFaces == 0);
        REQUIRE(result.finEdgesBefore == 1);
        REQUIRE(result.finEdgesLeft == 0);
        REQUIRE(result.seamEdgesOpened == 0);
        REQUIRE(result.rounds == 1);
        requireFaceMap(mesh, result.mesh, result.faceMap);
        REQUIRE(nonManifoldEdges(result.mesh, 1) == 0);
        REQUIRE(nonManifoldEdges(result.mesh, 0) == touchingBefore);
        REQUIRE(openEdges(result.mesh) == 0);

        // Nothing left to do
        const mesheal::FinRemovalResult again = mesheal::removeFinFaces(result.mesh);
        REQUIRE(again.removedFaces.empty());
        REQUIRE(again.finEdgesBefore == 0);
        REQUIRE(again.mesh.F == result.mesh.F);
    }
}

TEST_CASE("A wall that holds a seam shut is removed, the seam opens and the fill closes it", "[repair][topology][fins]") {
    const Mesh mesh = SampleMeshes::createFinSeamBoxesMesh();
    REQUIRE(mesh.numFaces() == 20);
    REQUIRE(nonManifoldEdges(mesh, 1) == 3);
    REQUIRE(openEdges(mesh) == 3);

    const mesheal::FinRemovalResult result = mesheal::removeFinFaces(mesh);
    REQUIRE(result.removedFaces == std::vector<Eigen::Index>{18, 19}); // the wall
    REQUIRE(result.wallFaces == 2);
    REQUIRE(result.finFaces == 0);
    REQUIRE(result.seamEdgesOpened == 1);
    REQUIRE(result.finEdgesLeft == 0);
    requireFaceMap(mesh, result.mesh, result.faceMap);
    REQUIRE(nonManifoldEdges(result.mesh, 1) == 0);
    REQUIRE(openEdges(result.mesh) == 4);
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(result.mesh);
    REQUIRE(holes.size() == 1);

    const mesheal::HoleBatchResult filled = mesheal::fillHoles(result.mesh, holes);
    REQUIRE(filled.success);
    REQUIRE(openEdges(filled.mesh) == 0);
    REQUIRE(count(DefectCategory::NonManifoldEdges, filled.mesh) == 0);
    REQUIRE(mesheal::signedVolume(filled.mesh) == Catch::Approx(2.0));
}

TEST_CASE("Internal walls inside a closed box are removed and the box stays closed", "[repair][topology][fins]") {
    const Mesh left = SampleMeshes::createBox({0, 0, 0}, {1, 1, 1});
    const Mesh right = SampleMeshes::createBox({1, 0, 0}, {2, 1, 1});
    const auto notWall = [](const Eigen::Vector3d& c) { return !onPlane(c.x(), 1.0); };
    const auto wall = [](const Eigen::Vector3d& c) { return onPlane(c.x(), 1.0); };
    auto halves = facesWhere(left, notWall);
    const auto rightHalf = facesWhere(right, notWall);
    halves.insert(halves.end(), rightHalf.begin(), rightHalf.end());

    SECTION("one wall between two open halves") {
        auto triangles = halves;
        const auto w = facesWhere(left, wall);
        triangles.insert(triangles.end(), w.begin(), w.end());
        const Mesh mesh = fromTriangles(triangles);
        REQUIRE(nonManifoldEdges(mesh, 1) == 4);
        const mesheal::FinRemovalResult result = mesheal::removeFinFaces(mesh);
        REQUIRE(result.wallFaces == 2);
        REQUIRE(result.seamEdgesOpened == 0);
        REQUIRE(openEdges(result.mesh) == 0);
        REQUIRE(count(DefectCategory::NonManifoldEdges, result.mesh) == 0);
        REQUIRE(mesheal::signedVolume(result.mesh) == Catch::Approx(2.0));
    }

    SECTION("two shells welded face to face: the wall is a pair of opposite copies") {
        auto triangles = halves;
        const auto w = facesWhere(left, wall);
        const auto back = facesWhere(left, wall, true);
        triangles.insert(triangles.end(), w.begin(), w.end());
        triangles.insert(triangles.end(), back.begin(), back.end());
        const Mesh mesh = fromTriangles(triangles);
        REQUIRE(count(DefectCategory::Duplicates, mesh) == 2);

        // Duplicate removal first (Fix All's order) takes the wall away; fin removal then finds nothing
        const mesheal::DuplicateFacesResult deduped = mesheal::removeDuplicateFaces(mesh);
        REQUIRE(deduped.removedOppositeWinding == 4);
        const mesheal::FinRemovalResult afterDedupe = mesheal::removeFinFaces(deduped.mesh);
        REQUIRE(afterDedupe.finEdgesBefore == 0);
        REQUIRE(afterDedupe.removedFaces.empty());
        REQUIRE(openEdges(afterDedupe.mesh) == 0);

        // Fin removal alone removes it too
        const mesheal::FinRemovalResult alone = mesheal::removeFinFaces(mesh);
        REQUIRE(alone.wallFaces == 4);
        REQUIRE(alone.removedFaces == std::vector<Eigen::Index>{20, 21, 22, 23});
        REQUIRE(openEdges(alone.mesh) == 0);
        REQUIRE(count(DefectCategory::NonManifoldEdges, alone.mesh) == 0);
        REQUIRE(mesheal::signedVolume(alone.mesh) == Catch::Approx(2.0));
    }

    SECTION("two shells welded face to face, each with its own wall faces") {
        auto triangles = halves;
        const auto w = facesWhere(left, wall);
        const auto w2 = facesWhere(right, wall);
        triangles.insert(triangles.end(), w.begin(), w.end());
        triangles.insert(triangles.end(), w2.begin(), w2.end());
        const Mesh mesh = fromTriangles(triangles);
        const mesheal::DuplicateFacesResult deduped = mesheal::removeDuplicateFaces(mesh);
        const mesheal::FinRemovalResult result = mesheal::removeFinFaces(deduped.mesh);
        REQUIRE(result.mesh.numFaces() == 20);
        REQUIRE(openEdges(result.mesh) == 0);
        REQUIRE(count(DefectCategory::NonManifoldEdges, result.mesh) == 0);
        REQUIRE(mesheal::signedVolume(result.mesh) == Catch::Approx(2.0));
    }
}

TEST_CASE("Fin removal leaves a sheet as big as the body and sheets out of scope", "[repair][topology][fins][scope]") {
    const Mesh cube = SampleMeshes::createCubeMesh(1.0);
    const auto withFlap = [&](const Eigen::Vector3d& apex) {
        auto triangles = facesWhere(cube, [](const Eigen::Vector3d&) { return true; });
        triangles.push_back({Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0, 1, 1), apex});
        return fromTriangles(triangles);
    };

    // A small flap on the top back edge goes; a flap larger than half the cube's area stays
    const mesheal::FinRemovalResult small = mesheal::removeFinFaces(withFlap({0.5, 1.5, 1.5}));
    REQUIRE(small.finFaces == 1);
    REQUIRE(small.finEdgesLeft == 0);
    const Mesh big = withFlap({0.5, 10.0, 10.0});
    const mesheal::FinRemovalResult kept = mesheal::removeFinFaces(big);
    REQUIRE(kept.removedFaces.empty());
    REQUIRE(kept.finEdgesLeft == 1);
    REQUIRE(kept.mesh.F == big.F);

    // The touching cubes' fin out of scope stays, faces bit-identical
    const Mesh touching = SampleMeshes::createTouchingCubesMesh();
    std::vector<std::uint8_t> scope(static_cast<std::size_t>(touching.numFaces()), 1);
    scope.back() = 0;
    const mesheal::FinRemovalResult outOfScope = mesheal::removeFinFaces(touching, scope);
    REQUIRE(outOfScope.removedFaces.empty());
    REQUIRE(outOfScope.sheetsOutOfScope == 1);
    REQUIRE(outOfScope.finEdgesLeft == 1);
    REQUIRE(outOfScope.mesh.F == touching.F);
    REQUIRE(outOfScope.mesh.V == touching.V);
}

TEST_CASE("Only the far tiny tetrahedron is a tiny floating shell", "[repair][topology][tiny]") {
    const Mesh mesh = SampleMeshes::createTinyFloatingShellsMesh();
    const auto shells = mesheal::shellsLargestFirst(mesh);
    REQUIRE(shells.size() == 5);

    const mesheal::TinyFloatingShellsResult found = mesheal::findTinyFloatingShells(mesh);
    REQUIRE(found.complete);
    REQUIRE(found.modelDiagonal == Catch::Approx(std::sqrt(2.0 * 20.01 * 20.01 + 10.01 * 10.01)));
    REQUIRE(found.shells.size() == 1);
    REQUIRE(found.shells[0].shell == 4);
    REQUIRE(found.shells[0].faces == shells[4]);
    REQUIRE(found.shells[0].sizeFraction == Catch::Approx(0.01 * std::sqrt(3.0) / found.modelDiagonal));
    REQUIRE(found.largestFraction() < 0.001);

    // Fraction 0: none; 5 %: the 0.5 cube too (2.9 %), never the cavity or the touching tetrahedron
    mesheal::TinyShellSearch none;
    none.fraction = 0.0;
    REQUIRE(mesheal::findTinyFloatingShells(mesh, none).shells.empty());
    mesheal::TinyShellSearch wide;
    wide.fraction = 0.05;
    const mesheal::TinyFloatingShellsResult two = mesheal::findTinyFloatingShells(mesh, wide);
    REQUIRE(two.shells.size() == 2);
    REQUIRE(two.shells[0].shell == 1);
    REQUIRE(two.shells[1].shell == 4);

    // Scope shells, and relations handed in, give the same answers
    mesheal::TinyShellSearch scoped;
    scoped.scopeShells = {0, 1, 2, 3};
    REQUIRE(mesheal::findTinyFloatingShells(mesh, scoped).shells.empty());
    const mesheal::ShellRelations relations = mesheal::computeShellRelations(mesh);
    mesheal::TinyShellSearch withRelations;
    withRelations.relations = &relations;
    const mesheal::TinyFloatingShellsResult same = mesheal::findTinyFloatingShells(mesh, withRelations);
    REQUIRE(same.shells.size() == 1);
    REQUIRE(same.shells[0].shell == 4);

    // Removal: 4 faces go, the map marks them, the rest keep their rows
    const mesheal::TinyShellRemovalResult removed = mesheal::removeTinyFloatingShells(mesh);
    REQUIRE(removed.removedFaces == 4);
    REQUIRE(removed.found.shells.size() == 1);
    requireFaceMap(mesh, removed.mesh, removed.faceMap);
    for (const Eigen::Index f : shells[4]) {
        REQUIRE(removed.faceMap[static_cast<std::size_t>(f)] == -1);
    }
    REQUIRE(mesheal::shellsLargestFirst(removed.mesh).size() == 4);

    // A face scope without the tetrahedron: nothing goes
    std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 1);
    scope[static_cast<std::size_t>(shells[4].front())] = 0;
    const mesheal::TinyShellRemovalResult outOfScope = mesheal::removeTinyFloatingShells(mesh, scope);
    REQUIRE(outOfScope.removedFaces == 0);
    REQUIRE(outOfScope.mesh.F == mesh.F);
}

TEST_CASE("The Turkey has no tiny floating shell and no fins", "[repair][topology][tiny][.slow]") {
    const Mesh turkey = mesheal::loadStl(examplePath("Turkey.stl"));
    const auto start = std::chrono::steady_clock::now();
    const mesheal::TinyFloatingShellsResult found = mesheal::findTinyFloatingShells(turkey);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(found.complete);
    REQUIRE(found.shells.empty());

    // The smallest shell (the 70-face cavity #28) is about 1.23 % of the model's size
    const std::vector<mesheal::ShellFacts> shells = mesheal::describeShells(turkey, mesheal::shellsLargestFirst(turkey));
    double smallest = 1.0;
    for (const mesheal::ShellFacts& shell : shells) {
        smallest = std::min(smallest, shell.box.diagonal() / found.modelDiagonal);
    }
    REQUIRE(smallest > 0.01);
    WARN("Turkey findTinyFloatingShells: " << ms << " ms; smallest shell " << 100.0 * smallest << " % of the model");

    const auto finStart = std::chrono::steady_clock::now();
    const mesheal::FinRemovalResult fins = mesheal::removeFinFaces(turkey);
    const double finMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - finStart).count();
    WARN("Turkey removeFinFaces: " << finMs << " ms");
    REQUIRE(fins.finEdgesBefore == 0);
    REQUIRE(fins.removedFaces.empty());
}
