#include "repair/NonManifoldSplit.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <manifold/manifold.h>

#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <vector>

namespace {

std::size_t count(mesheal::DefectCategory category, const mesheal::Mesh& mesh) {
    return mesheal::runCategoryCheck(category, mesh, mesheal::CheckParams{}).size();
}

std::size_t shellCount(const mesheal::Mesh& mesh) {
    return mesheal::checkShells(mesh).infoCount;
}

/// Edges of the Non-manifold edges result with the given sub-type (-1: all).
std::vector<std::array<int, 2>> nonManifoldEdges(const mesheal::Mesh& mesh, int subtype = -1) {
    std::vector<std::array<int, 2>> edges;
    for (const mesheal::DefectElement& element : mesheal::checkNonManifoldEdges(mesh).elements) {
        if (subtype < 0 || element.subtype == subtype) {
            edges.push_back(element.edge);
        }
    }
    return edges;
}

std::vector<Eigen::Index> nonManifoldVertices(const mesheal::Mesh& mesh) {
    std::vector<Eigen::Index> vertices;
    for (const mesheal::DefectElement& element : mesheal::checkNonManifoldVertices(mesh).elements) {
        vertices.push_back(element.index);
    }
    return vertices;
}

/// Every vertex keeps its position, and every added vertex copies an existing position.
bool geometryUnchanged(const mesheal::Mesh& before, const mesheal::Mesh& after) {
    if (after.F.rows() != before.F.rows() || after.V.rows() < before.V.rows() ||
        after.V.topRows(before.V.rows()) != before.V) {
        return false;
    }
    for (Eigen::Index f = 0; f < before.F.rows(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (after.V.row(after.F(f, c)) != before.V.row(before.F(f, c))) {
                return false;
            }
        }
    }
    return true;
}

/// Welds `meshes` by exact position (as the STL reader would) into one mesh.
mesheal::Mesh weld(const std::vector<mesheal::Mesh>& meshes) {
    std::map<std::array<double, 3>, int> index;
    std::vector<Eigen::RowVector3d> positions;
    std::vector<std::array<int, 3>> faces;
    for (const mesheal::Mesh& m : meshes) {
        for (Eigen::Index f = 0; f < m.F.rows(); ++f) {
            std::array<int, 3> face{};
            for (int c = 0; c < 3; ++c) {
                const Eigen::RowVector3d p = m.V.row(m.F(f, c));
                const auto [it, inserted] = index.try_emplace({p.x(), p.y(), p.z()}, static_cast<int>(positions.size()));
                if (inserted) {
                    positions.push_back(p);
                }
                face[static_cast<std::size_t>(c)] = it->second;
            }
            faces.push_back(face);
        }
    }
    mesheal::Mesh out;
    out.V.resize(static_cast<Eigen::Index>(positions.size()), 3);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        out.V.row(static_cast<Eigen::Index>(i)) = positions[i];
    }
    out.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        out.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    out.recomputeNormals();
    return out;
}

/// Closed prism over the wedge between `fromDeg` and `toDeg` around the z axis, z in [0, 1], wound outward.
mesheal::Mesh wedgePrism(double fromDeg, double toDeg) {
    const double a = fromDeg * std::numbers::pi / 180.0;
    const double b = toDeg * std::numbers::pi / 180.0;
    mesheal::Mesh m;
    m.V.resize(6, 3);
    m.V << 0.0, 0.0, 0.0, std::cos(a), std::sin(a), 0.0, std::cos(b), std::sin(b), 0.0,
           0.0, 0.0, 1.0, std::cos(a), std::sin(a), 1.0, std::cos(b), std::sin(b), 1.0;
    m.F.resize(8, 3);
    m.F << 0, 2, 1, 3, 4, 5,
           0, 1, 4, 0, 4, 3,
           1, 2, 5, 1, 5, 4,
           2, 0, 3, 2, 3, 5;
    return m;
}

/// Cube A and cube B of the touching-cubes sample only: closed, touching along one edge chain.
mesheal::Mesh twoCubesTouchingAlongAnEdge() {
    const mesheal::Mesh cube = mesheal::SampleMeshes::createGridCubeMesh(2, 2.0);
    mesheal::Mesh shifted = cube;
    shifted.V.rowwise() += Eigen::RowVector3d(2.0, 2.0, 0.0);
    return weld({cube, shifted});
}

manifold::Manifold::Error manifoldStatus(const mesheal::Mesh& mesh) {
    return manifold::Manifold(mesh.toMeshGL64()).Status();
}

} // namespace

TEST_CASE("The touching-cubes sample has Touching and Fin edges and one bow-tie", "[repair][non-manifold]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
    REQUIRE(nonManifoldEdges(mesh, 0).size() == 2);
    REQUIRE(nonManifoldEdges(mesh, 1).size() == 1);
    REQUIRE(count(mesheal::DefectCategory::NonManifoldVertices, mesh) == 1);
    REQUIRE(mesheal::checkOpenEdges(mesh).infoCount == 2); // the fin's two open edges
    REQUIRE(shellCount(mesh) == 2);
}

TEST_CASE("Splitting Touching edges separates the solids without moving any vertex", "[repair][non-manifold]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
    const std::vector<std::array<int, 2>> all = nonManifoldEdges(mesh);

    const mesheal::NonManifoldSplitResult r = mesheal::splitNonManifoldEdges(mesh, all);
    REQUIRE(r.edgesSplit == 2);
    REQUIRE(r.edgesSkipped == 1);
    REQUIRE(r.verticesSplit == 3); // the seam's two edges have three vertices
    REQUIRE(r.verticesAdded == 3);
    REQUIRE(geometryUnchanged(mesh, r.mesh));

    // Only the fin is left; the bow-tie belongs to the vertex row
    const std::vector<std::array<int, 2>> left = nonManifoldEdges(r.mesh);
    REQUIRE(left.size() == 1);
    REQUIRE(nonManifoldEdges(r.mesh, 1).size() == 1);
    REQUIRE(count(mesheal::DefectCategory::NonManifoldVertices, r.mesh) == 1);
    REQUIRE(mesheal::checkOpenEdges(r.mesh).infoCount == 2);
    REQUIRE(count(mesheal::DefectCategory::FlippedNormals, r.mesh) == 0);
    REQUIRE(shellCount(r.mesh) == 3); // A (with the fin), B and C

    // Asking again for the fin alone changes nothing
    const mesheal::NonManifoldSplitResult again = mesheal::splitNonManifoldEdges(r.mesh, left);
    REQUIRE(again.edgesSplit == 0);
    REQUIRE(again.edgesSkipped == 1);
    REQUIRE(again.verticesAdded == 0);
    REQUIRE(again.mesh.F == r.mesh.F);
}

TEST_CASE("Selecting one edge of a seam splits the whole seam", "[repair][non-manifold]") {
    const mesheal::Mesh mesh = twoCubesTouchingAlongAnEdge();
    const std::vector<std::array<int, 2>> seam = nonManifoldEdges(mesh, 0);
    REQUIRE(seam.size() == 2);

    // Splitting only the selected edge's ends would leave the far end of the seam pinched (a new bow-tie)
    for (const auto& edge : seam) {
        const std::array<std::array<int, 2>, 1> one{edge};
        const mesheal::NonManifoldSplitResult r = mesheal::splitNonManifoldEdges(mesh, one);
        REQUIRE(r.edgesSplit == 2);
        REQUIRE(r.verticesSplit == 3);
        REQUIRE(geometryUnchanged(mesh, r.mesh));
        REQUIRE(count(mesheal::DefectCategory::NonManifoldEdges, r.mesh) == 0);
        REQUIRE(count(mesheal::DefectCategory::NonManifoldVertices, r.mesh) == 0);
        REQUIRE(shellCount(r.mesh) == 2);
    }
}

TEST_CASE("Splitting bow-tie vertices gives each fan its own copy", "[repair][non-manifold]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
    const std::vector<Eigen::Index> vertices = nonManifoldVertices(mesh);
    REQUIRE(vertices.size() == 1);
    REQUIRE(mesh.V.row(vertices[0]).isZero());

    const mesheal::NonManifoldSplitResult r = mesheal::splitNonManifoldVertices(mesh, vertices);
    REQUIRE(r.verticesSplit == 1);
    REQUIRE(r.verticesAdded == 1);
    REQUIRE(geometryUnchanged(mesh, r.mesh));
    REQUIRE(count(mesheal::DefectCategory::NonManifoldVertices, r.mesh) == 0);
    REQUIRE(count(mesheal::DefectCategory::NonManifoldEdges, r.mesh) == 3); // the edge row is untouched
    REQUIRE(shellCount(r.mesh) == 2);                                     // shells are edge-connected already
}

TEST_CASE("Faces around an edge are paired radially by winding", "[repair][non-manifold]") {
    // Three 60 degree prisms sharing the z axis: six faces on one edge, three closed wedges
    const mesheal::Mesh mesh = weld({wedgePrism(0.0, 60.0), wedgePrism(120.0, 180.0), wedgePrism(240.0, 300.0)});
    const mesheal::EdgeTopology topo = mesheal::buildEdgeTopology(mesh);
    std::size_t axis = topo.numEdges();
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) == 6) {
            axis = e;
        }
    }
    REQUIRE(axis < topo.numEdges());
    const auto pairs = mesheal::pairTouchingFaces(mesh, topo, axis);
    REQUIRE(pairs.has_value());
    REQUIRE(pairs->size() == 3);
    for (const auto& [f, g] : *pairs) {
        REQUIRE(f / 8 == g / 8); // both faces from the same prism
    }

    const mesheal::NonManifoldSplitResult r = mesheal::splitNonManifoldEdges(mesh, nonManifoldEdges(mesh));
    REQUIRE(r.edgesSplit == 1);
    REQUIRE(r.verticesAdded == 4); // two more copies of each axis end
    REQUIRE(count(mesheal::DefectCategory::NonManifoldEdges, r.mesh) == 0);
    REQUIRE(count(mesheal::DefectCategory::NonManifoldVertices, r.mesh) == 0);
    REQUIRE(count(mesheal::DefectCategory::OpenEdges, r.mesh) == 0);
    REQUIRE(shellCount(r.mesh) == 3);

    SECTION("inside-out solids pair across the gaps, which copying vertices cannot separate") {
        mesheal::Mesh inverted = mesh;
        inverted.F.col(1).swap(inverted.F.col(2));
        const mesheal::EdgeTopology invertedTopo = mesheal::buildEdgeTopology(inverted);
        const auto bridged = mesheal::pairTouchingFaces(inverted, invertedTopo, axis);
        REQUIRE(bridged.has_value());
        for (const auto& [f, g] : *bridged) {
            REQUIRE(f / 8 != g / 8);
        }
        REQUIRE(nonManifoldEdges(inverted, 0).empty());
        const mesheal::NonManifoldSplitResult ri = mesheal::splitNonManifoldEdges(inverted, nonManifoldEdges(inverted));
        REQUIRE(ri.edgesSkipped == 1);
        REQUIRE(ri.verticesAdded == 0);
    }
}

TEST_CASE("Fins, odd counts and mixed winding are not pairable", "[repair][non-manifold]") {
    SECTION("a fin on a cube edge has three faces") {
        const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
        REQUIRE(nonManifoldEdges(mesh, 1).size() == 1);
    }
    SECTION("one prism wound inside out next to a correct one") {
        mesheal::Mesh flipped = wedgePrism(120.0, 180.0);
        flipped.F.col(1).swap(flipped.F.col(2));
        const mesheal::Mesh mesh = weld({wedgePrism(0.0, 60.0), flipped});
        REQUIRE(nonManifoldEdges(mesh, 0).empty());
        REQUIRE(nonManifoldEdges(mesh, 1).size() == 1);
        const mesheal::NonManifoldSplitResult r = mesheal::splitNonManifoldEdges(mesh, nonManifoldEdges(mesh));
        REQUIRE(r.edgesSkipped == 1);
        REQUIRE(r.verticesAdded == 0);
    }
}

TEST_CASE("Manifold v3 accepts welded bow-ties and touching edges but rejects fins", "[repair][non-manifold][manifold]") {
    // Documents the library's behaviour: the constructor needs every edge to have as many forward as
    // backward half-edges ("even manifold"), then splits pinched vertices and doubled edges itself.
    const mesheal::Mesh touching = twoCubesTouchingAlongAnEdge();
    REQUIRE(manifoldStatus(touching) == manifold::Manifold::Error::NoError);
    REQUIRE(manifold::Manifold(touching.toMeshGL64()).Volume() == Catch::Approx(16.0));

    const mesheal::Mesh sample = mesheal::SampleMeshes::createTouchingCubesMesh();
    REQUIRE(manifoldStatus(sample) == manifold::Manifold::Error::NotManifold); // the fin

    const mesheal::NonManifoldSplitResult split = mesheal::splitNonManifoldEdges(touching, nonManifoldEdges(touching));
    REQUIRE(manifoldStatus(split.mesh) == manifold::Manifold::Error::NoError);
    REQUIRE(manifold::Manifold(split.mesh.toMeshGL64()).Volume() == Catch::Approx(16.0));
}
