#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stop_token>
#include <utility>
#include <vector>

namespace {

mesheal::Mesh cube() {
    return mesheal::SampleMeshes::createCubeMesh(10.0, mesheal::Unit::Millimeter);
}

/// Appends `extraV` vertices and `extraF` faces (indices may reference the new vertices).
mesheal::Mesh withExtra(mesheal::Mesh mesh, const Eigen::MatrixXd& extraV, const Eigen::MatrixXi& extraF) {
    const Eigen::Index nv = mesh.V.rows();
    const Eigen::Index nf = mesh.F.rows();
    mesh.V.conservativeResize(nv + extraV.rows(), 3);
    mesh.F.conservativeResize(nf + extraF.rows(), 3);
    if (extraV.rows() > 0) {
        mesh.V.bottomRows(extraV.rows()) = extraV;
    }
    mesh.F.bottomRows(extraF.rows()) = extraF;
    mesh.recomputeNormals();
    return mesh;
}

std::size_t count(mesheal::DefectCategory category, const mesheal::Mesh& mesh) {
    return mesheal::runCategoryCheck(category, mesh, mesheal::CheckParams{}).size();
}

} // namespace

TEST_CASE("Mesh checks report a closed cube as clean with one shell", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh mesh = cube();
    for (const mesheal::DefectCategory category : mesheal::kAllDefectCategories) {
        const mesheal::CategoryResult result = mesheal::runCategoryCheck(category, mesh, mesheal::CheckParams{});
        REQUIRE_FALSE(result.cancelled);
        REQUIRE(result.category == category);
        if (category == mesheal::DefectCategory::Shells) {
            REQUIRE(result.infoCount == 1);
            REQUIRE(result.size() == 1);
            REQUIRE(result.facesOf(0).size() == 12);
        } else {
            INFO(mesheal::categoryInfo(category).name);
            REQUIRE(result.empty());
        }
    }
    const mesheal::EdgeTopology topo = mesheal::buildEdgeTopology(mesh);
    REQUIRE(topo.numEdges() == 18);
}

TEST_CASE("Open edges are listed as one row per hole with its rim", "[diagnostics][mesh-checks]") {
    mesheal::Mesh mesh = cube();
    mesh.F.conservativeResize(11, 3);
    const mesheal::CategoryResult open = mesheal::checkOpenEdges(mesh);
    REQUIRE(open.size() == 1);
    REQUIRE(open.infoCount == 3); // open edges
    REQUIRE(open.elements[0].index == 0);
    REQUIRE(open.elements[0].metric == 3.0);
    REQUIRE(open.elements[0].edge[0] < open.elements[0].edge[1]);
    REQUIRE(open.facesOf(0).size() == 3);
    REQUIRE(open.edgesOf(0).size() == 3);
    REQUIRE(mesheal::checkNonManifoldEdges(mesh).edgesOf(0).empty());
    REQUIRE(count(mesheal::DefectCategory::NonManifoldEdges, mesh) == 0);
}

TEST_CASE("A fin on a cube edge is one non-manifold edge", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh base = cube();
    Eigen::MatrixXd v(1, 3);
    v << 5.0, 5.0, 25.0;
    Eigen::MatrixXi f(1, 3);
    f << base.F(0, 0), base.F(0, 1), 8;
    const mesheal::Mesh mesh = withExtra(base, v, f);

    const mesheal::CategoryResult nm = mesheal::checkNonManifoldEdges(mesh);
    REQUIRE(nm.size() == 1);
    REQUIRE(nm.elements[0].metric == 3.0);
    REQUIRE(nm.facesOf(0).size() == 3);
    REQUIRE(nm.elements[0].subtype == 1); // Fin / other: copying vertices cannot fix it
    REQUIRE(mesheal::checkOpenEdges(mesh).infoCount == 2);
}

TEST_CASE("Solids touching along an edge are Touching edges, not non-manifold vertices", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
    const mesheal::CategoryResult nm = mesheal::checkNonManifoldEdges(mesh);
    REQUIRE(nm.size() == 3);
    REQUIRE(nm.subtypeCounts()[0] == 2);
    REQUIRE(nm.subtypeCounts()[1] == 1);
    for (std::size_t i = 0; i < nm.size(); ++i) {
        REQUIRE(nm.facesOf(i).size() == (nm.elements[i].subtype == 0 ? 4u : 3u));
    }

    // The seam's vertices have one fan (joined through the seam); only the corner-touching cube's vertex is reported
    const mesheal::CategoryResult nmv = mesheal::checkNonManifoldVertices(mesh);
    REQUIRE(nmv.size() == 1);
    REQUIRE(mesh.V.row(nmv.elements[0].index).isZero());
    REQUIRE(nmv.elements[0].metric == 2.0);
}

TEST_CASE("Corner fans split at paired edges only", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createTouchingCubesMesh();
    const mesheal::EdgeTopology topo = mesheal::buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = mesheal::touchingEdges(mesh, topo);
    REQUIRE(std::count(touching.begin(), touching.end(), std::uint8_t{1}) == 2);

    const auto fansAt = [&](const std::vector<std::uint32_t>& fans, int vertex) {
        std::vector<std::uint32_t> ids;
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                if (mesh.F(f, c) == vertex) {
                    ids.push_back(fans[static_cast<std::size_t>(f) * 3 + static_cast<std::size_t>(c)]);
                }
            }
        }
        std::sort(ids.begin(), ids.end());
        return static_cast<std::size_t>(std::unique(ids.begin(), ids.end()) - ids.begin());
    };
    const auto& seamEdge = topo.edges[static_cast<std::size_t>(std::find(touching.begin(), touching.end(), 1) - touching.begin())];
    REQUIRE(fansAt(mesheal::cornerFans(mesh, topo), seamEdge[0]) == 1);
    REQUIRE(fansAt(mesheal::cornerFans(mesh, topo, touching), seamEdge[0]) == 2);
}

TEST_CASE("Two triangles sharing only a vertex form a non-manifold vertex", "[diagnostics][mesh-checks]") {
    mesheal::Mesh mesh;
    mesh.V.resize(5, 3);
    mesh.V << 0, 0, 0,
              1, 0, 0,
              0, 1, 0,
              -1, 0, 0,
              0, -1, 0;
    mesh.F.resize(2, 3);
    mesh.F << 0, 1, 2,
              0, 3, 4;
    mesh.recomputeNormals();

    const mesheal::CategoryResult nmv = mesheal::checkNonManifoldVertices(mesh);
    REQUIRE(nmv.size() == 1);
    REQUIRE(nmv.elements[0].index == 0);
    REQUIRE(nmv.elements[0].metric == 2.0);
    REQUIRE(nmv.facesOf(0).size() == 2);
    REQUIRE(mesheal::checkShells(mesh).infoCount == 2);
    REQUIRE(mesheal::checkOpenEdges(mesh).infoCount == 6);
}

TEST_CASE("A reversed cube face is reported as the one flipped normal", "[diagnostics][mesh-checks]") {
    mesheal::Mesh mesh = cube();
    std::swap(mesh.F(4, 1), mesh.F(4, 2));
    const mesheal::CategoryResult flipped = mesheal::checkFlippedNormals(mesh);
    REQUIRE(flipped.size() == 1);
    REQUIRE(flipped.elements[0].index == 4);
}

TEST_CASE("Two separate cubes are two shells, smallest first", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh a = cube();
    mesheal::Mesh b = cube();
    b.V.array() += 50.0;
    Eigen::MatrixXi bf = b.F.array() + static_cast<int>(a.V.rows());
    mesheal::Mesh both = withExtra(a, b.V, bf);
    both.F.conservativeResize(both.F.rows() - 1, 3); // second cube loses a face: 11 faces

    const mesheal::CategoryResult shells = mesheal::checkShells(both);
    REQUIRE(shells.infoCount == 2);
    REQUIRE(shells.facesOf(0).size() == 11);
    REQUIRE(shells.facesOf(1).size() == 12);
    REQUIRE(shells.elements[0].metric < shells.elements[1].metric);
}

TEST_CASE("Degenerate faces: repeated vertex index and zero area", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh base = cube();
    const int a = base.F(0, 0);
    const int b = base.F(0, 1);
    Eigen::MatrixXd v(1, 3);
    v = 0.5 * (base.V.row(a) + base.V.row(b));
    Eigen::MatrixXi f(2, 3);
    f << a, a, b,
         a, 8, b;
    const mesheal::Mesh mesh = withExtra(base, v, f);

    const mesheal::CategoryResult degenerate = mesheal::checkDegenerateFaces(mesh);
    REQUIRE(degenerate.size() == 2);
    REQUIRE(degenerate.elements[0].index == 12);
    REQUIRE(degenerate.elements[0].subtype == 0);
    REQUIRE(degenerate.elements[1].index == 13);
    REQUIRE(degenerate.elements[1].subtype == 1);
    REQUIRE(degenerate.subtypeCounts()[0] == 1);
}

TEST_CASE("Duplicate faces report the later copies with their winding", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh base = cube();
    Eigen::MatrixXi f(2, 3);
    f << base.F(0, 1), base.F(0, 2), base.F(0, 0),
         base.F(1, 0), base.F(1, 2), base.F(1, 1);
    const mesheal::Mesh mesh = withExtra(base, Eigen::MatrixXd(0, 3), f);

    const mesheal::CategoryResult dupes = mesheal::checkDuplicateFaces(mesh);
    REQUIRE(dupes.size() == 2);
    REQUIRE(dupes.elements[0].index == 12);
    REQUIRE(dupes.elements[0].subtype == 0);
    REQUIRE(dupes.elements[0].columns[0] == 0.0);
    REQUIRE(dupes.elements[1].index == 13);
    REQUIRE(dupes.elements[1].subtype == 1);
}

TEST_CASE("Thin faces are sorted worst first and bucketed by shape", "[diagnostics][mesh-checks]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createSampleDefectiveMesh(mesheal::Unit::Millimeter);
    const mesheal::CategoryResult thin = mesheal::runCategoryCheck(mesheal::DefectCategory::ThinFaces, mesh, mesheal::CheckParams{});
    REQUIRE(thin.size() == 2); // zero-area and needle: the microscopic face is well shaped
    REQUIRE(thin.elements[0].index == mesheal::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(thin.elements[0].subtype == static_cast<int>(mesheal::ThinFaceSubtype::ZeroArea));
    for (std::size_t i = 1; i < thin.size(); ++i) {
        REQUIRE(thin.elements[i - 1].metric <= thin.elements[i].metric);
    }
    REQUIRE(thin.elapsedMs >= 0.0);
}

TEST_CASE("A stopped check returns a cancelled result", "[diagnostics][mesh-checks]") {
    std::stop_source source;
    source.request_stop();
    const mesheal::Mesh mesh = cube();
    for (const mesheal::DefectCategory category : mesheal::kAllDefectCategories) {
        INFO(mesheal::categoryInfo(category).name);
        REQUIRE(mesheal::runCategoryCheck(category, mesh, mesheal::CheckParams{}, source.get_token()).cancelled);
    }
}

TEST_CASE("Every category has a descriptor with a name and columns", "[diagnostics][mesh-checks]") {
    for (const mesheal::DefectCategory category : mesheal::kAllDefectCategories) {
        const mesheal::CategoryInfo& info = mesheal::categoryInfo(category);
        REQUIRE(info.name != nullptr);
        REQUIRE(info.columns[0].name != nullptr);
    }
    REQUIRE(mesheal::categorySubtypeCount(mesheal::DefectCategory::ThinFaces) == 4);
    REQUIRE(mesheal::categorySubtypeCount(mesheal::DefectCategory::OpenEdges) == mesheal::kHoleShapeCount);
    REQUIRE(mesheal::categoryInfo(mesheal::DefectCategory::Shells).infoOnly);
    REQUIRE(mesheal::categoryInfo(mesheal::DefectCategory::ThinFaces).hasFixAll);
}

TEST_CASE("The Degenerate row and the Zero-area bucket agree", "[diagnostics][mesh-checks][thin-faces]") {
    using mesheal::SampleMeshes;
    std::vector<std::pair<const char*, mesheal::Mesh>> meshes = {
        {"cube", SampleMeshes::createCubeMesh()},
        {"self-intersecting box", SampleMeshes::createSelfIntersectingBoxMesh()},
        {"grid cube", SampleMeshes::createGridCubeMesh()},
        {"folded cube", SampleMeshes::createFoldedCubeMesh()},
        {"touching cubes", SampleMeshes::createTouchingCubesMesh()},
        {"sphere", SampleMeshes::createSphereMesh()},
        {"cylinder", SampleMeshes::createCylinderMesh()},
        {"cracked box", SampleMeshes::createCrackedBoxMesh()},
        {"holes", SampleMeshes::createHolesSampleMesh()},
        {"defective sample", SampleMeshes::createSampleDefectiveMesh()},
    };
    // The defective sample far from the origin, where its hairline faces are only rounding noise, plus
    // repeated indices and a NaN vertex
    mesheal::Mesh shifted = SampleMeshes::createSampleDefectiveMesh();
    shifted.V.rowwise() += Eigen::RowVector3d(1.0e9, 0.0, 0.0);
    meshes.emplace_back("defective sample at 1e9", shifted);
    Eigen::MatrixXd nanVertex(1, 3);
    nanVertex << std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0;
    Eigen::MatrixXi bad(3, 3);
    bad << 0, 0, 1,
           5, 6, 5,
           0, 1, 8;
    meshes.emplace_back("bad faces", withExtra(cube(), nanVertex, bad));

    for (const auto& [name, mesh] : meshes) {
        INFO(name);
        const mesheal::CategoryResult degenerate = mesheal::checkDegenerateFaces(mesh);
        const mesheal::CategoryResult thin = mesheal::runCategoryCheck(mesheal::DefectCategory::ThinFaces, mesh, mesheal::CheckParams{});
        std::vector<Eigen::Index> zeroArea;
        for (const mesheal::DefectElement& element : thin.elements) {
            if (element.subtype == static_cast<int>(mesheal::ThinFaceSubtype::ZeroArea)) {
                zeroArea.push_back(element.index);
            }
        }
        std::vector<Eigen::Index> degenerateFaces;
        for (const mesheal::DefectElement& element : degenerate.elements) {
            degenerateFaces.push_back(element.index);
        }
        std::sort(zeroArea.begin(), zeroArea.end());
        std::sort(degenerateFaces.begin(), degenerateFaces.end());
        CHECK(zeroArea == degenerateFaces);
    }
    // Near the origin only the collinear face is degenerate; at 1e9 the two hairline faces are too
    CHECK(mesheal::checkDegenerateFaces(SampleMeshes::createSampleDefectiveMesh()).size() == 1);
    CHECK(mesheal::checkDegenerateFaces(shifted).size() == 3);
}
