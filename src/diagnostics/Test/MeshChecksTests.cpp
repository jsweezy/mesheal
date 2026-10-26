#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <stop_token>
#include <vector>

namespace {

ogun::Mesh cube() {
    return ogun::SampleMeshes::createCubeMesh(10.0, ogun::Unit::Millimeter);
}

/// Appends `extraV` vertices and `extraF` faces (indices may reference the new vertices).
ogun::Mesh withExtra(ogun::Mesh mesh, const Eigen::MatrixXd& extraV, const Eigen::MatrixXi& extraF) {
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

std::size_t count(ogun::DefectCategory category, const ogun::Mesh& mesh) {
    return ogun::runCategoryCheck(category, mesh, ogun::CheckParams{}).size();
}

} // namespace

TEST_CASE("Mesh checks report a closed cube as clean with one shell", "[diagnostics][mesh-checks]") {
    const ogun::Mesh mesh = cube();
    for (const ogun::DefectCategory category : ogun::kAllDefectCategories) {
        const ogun::CategoryResult result = ogun::runCategoryCheck(category, mesh, ogun::CheckParams{});
        REQUIRE_FALSE(result.cancelled);
        REQUIRE(result.category == category);
        if (category == ogun::DefectCategory::Shells) {
            REQUIRE(result.infoCount == 1);
            REQUIRE(result.size() == 1);
            REQUIRE(result.facesOf(0).size() == 12);
        } else {
            INFO(ogun::categoryInfo(category).name);
            REQUIRE(result.empty());
        }
    }
    const ogun::EdgeTopology topo = ogun::buildEdgeTopology(mesh);
    REQUIRE(topo.numEdges() == 18);
}

TEST_CASE("Open edges are counted per edge and grouped into boundary loops", "[diagnostics][mesh-checks]") {
    ogun::Mesh mesh = cube();
    mesh.F.conservativeResize(11, 3);
    const ogun::CategoryResult open = ogun::checkOpenEdges(mesh);
    REQUIRE(open.size() == 3);
    REQUIRE(open.infoCount == 1);
    for (std::size_t i = 0; i < open.size(); ++i) {
        REQUIRE(open.facesOf(i).size() == 1);
        REQUIRE(open.elements[i].edge[0] < open.elements[i].edge[1]);
        REQUIRE(open.elements[i].metric == 3.0);
    }
    REQUIRE(count(ogun::DefectCategory::NonManifoldEdges, mesh) == 0);
}

TEST_CASE("A fin on a cube edge is one non-manifold edge", "[diagnostics][mesh-checks]") {
    const ogun::Mesh base = cube();
    Eigen::MatrixXd v(1, 3);
    v << 5.0, 5.0, 25.0;
    Eigen::MatrixXi f(1, 3);
    f << base.F(0, 0), base.F(0, 1), 8;
    const ogun::Mesh mesh = withExtra(base, v, f);

    const ogun::CategoryResult nm = ogun::checkNonManifoldEdges(mesh);
    REQUIRE(nm.size() == 1);
    REQUIRE(nm.elements[0].metric == 3.0);
    REQUIRE(nm.facesOf(0).size() == 3);
    REQUIRE(nm.elements[0].subtype == 1); // Fin / other: copying vertices cannot fix it
    REQUIRE(count(ogun::DefectCategory::OpenEdges, mesh) == 2);
}

TEST_CASE("Solids touching along an edge are Touching edges, not non-manifold vertices", "[diagnostics][mesh-checks]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createTouchingCubesMesh();
    const ogun::CategoryResult nm = ogun::checkNonManifoldEdges(mesh);
    REQUIRE(nm.size() == 3);
    REQUIRE(nm.subtypeCounts()[0] == 2);
    REQUIRE(nm.subtypeCounts()[1] == 1);
    for (std::size_t i = 0; i < nm.size(); ++i) {
        REQUIRE(nm.facesOf(i).size() == (nm.elements[i].subtype == 0 ? 4u : 3u));
    }

    // The seam's vertices have one fan (joined through the seam); only the corner-touching cube's vertex is reported
    const ogun::CategoryResult nmv = ogun::checkNonManifoldVertices(mesh);
    REQUIRE(nmv.size() == 1);
    REQUIRE(mesh.V.row(nmv.elements[0].index).isZero());
    REQUIRE(nmv.elements[0].metric == 2.0);
}

TEST_CASE("Corner fans split at paired edges only", "[diagnostics][mesh-checks]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createTouchingCubesMesh();
    const ogun::EdgeTopology topo = ogun::buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = ogun::touchingEdges(mesh, topo);
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
    REQUIRE(fansAt(ogun::cornerFans(mesh, topo), seamEdge[0]) == 1);
    REQUIRE(fansAt(ogun::cornerFans(mesh, topo, touching), seamEdge[0]) == 2);
}

TEST_CASE("Two triangles sharing only a vertex form a non-manifold vertex", "[diagnostics][mesh-checks]") {
    ogun::Mesh mesh;
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

    const ogun::CategoryResult nmv = ogun::checkNonManifoldVertices(mesh);
    REQUIRE(nmv.size() == 1);
    REQUIRE(nmv.elements[0].index == 0);
    REQUIRE(nmv.elements[0].metric == 2.0);
    REQUIRE(nmv.facesOf(0).size() == 2);
    REQUIRE(ogun::checkShells(mesh).infoCount == 2);
    REQUIRE(count(ogun::DefectCategory::OpenEdges, mesh) == 6);
}

TEST_CASE("A reversed cube face is reported as the one flipped normal", "[diagnostics][mesh-checks]") {
    ogun::Mesh mesh = cube();
    std::swap(mesh.F(4, 1), mesh.F(4, 2));
    const ogun::CategoryResult flipped = ogun::checkFlippedNormals(mesh);
    REQUIRE(flipped.size() == 1);
    REQUIRE(flipped.elements[0].index == 4);
}

TEST_CASE("Two separate cubes are two shells, smallest first", "[diagnostics][mesh-checks]") {
    const ogun::Mesh a = cube();
    ogun::Mesh b = cube();
    b.V.array() += 50.0;
    Eigen::MatrixXi bf = b.F.array() + static_cast<int>(a.V.rows());
    ogun::Mesh both = withExtra(a, b.V, bf);
    both.F.conservativeResize(both.F.rows() - 1, 3); // second cube loses a face: 11 faces

    const ogun::CategoryResult shells = ogun::checkShells(both);
    REQUIRE(shells.infoCount == 2);
    REQUIRE(shells.facesOf(0).size() == 11);
    REQUIRE(shells.facesOf(1).size() == 12);
    REQUIRE(shells.elements[0].metric < shells.elements[1].metric);
}

TEST_CASE("Degenerate faces: repeated vertex index and zero area", "[diagnostics][mesh-checks]") {
    const ogun::Mesh base = cube();
    const int a = base.F(0, 0);
    const int b = base.F(0, 1);
    Eigen::MatrixXd v(1, 3);
    v = 0.5 * (base.V.row(a) + base.V.row(b));
    Eigen::MatrixXi f(2, 3);
    f << a, a, b,
         a, 8, b;
    const ogun::Mesh mesh = withExtra(base, v, f);

    const ogun::CategoryResult degenerate = ogun::checkDegenerateFaces(mesh, ogun::CheckParams{});
    REQUIRE(degenerate.size() == 2);
    REQUIRE(degenerate.elements[0].index == 12);
    REQUIRE(degenerate.elements[0].subtype == 0);
    REQUIRE(degenerate.elements[1].index == 13);
    REQUIRE(degenerate.elements[1].subtype == 1);
    REQUIRE(degenerate.subtypeCounts()[0] == 1);
}

TEST_CASE("Duplicate faces report the later copies with their winding", "[diagnostics][mesh-checks]") {
    const ogun::Mesh base = cube();
    Eigen::MatrixXi f(2, 3);
    f << base.F(0, 1), base.F(0, 2), base.F(0, 0),
         base.F(1, 0), base.F(1, 2), base.F(1, 1);
    const ogun::Mesh mesh = withExtra(base, Eigen::MatrixXd(0, 3), f);

    const ogun::CategoryResult dupes = ogun::checkDuplicateFaces(mesh);
    REQUIRE(dupes.size() == 2);
    REQUIRE(dupes.elements[0].index == 12);
    REQUIRE(dupes.elements[0].subtype == 0);
    REQUIRE(dupes.elements[0].columns[0] == 0.0);
    REQUIRE(dupes.elements[1].index == 13);
    REQUIRE(dupes.elements[1].subtype == 1);
}

TEST_CASE("Thin faces are sorted worst first and bucketed by shape", "[diagnostics][mesh-checks]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh(ogun::Unit::Millimeter);
    const ogun::CategoryResult thin = ogun::runCategoryCheck(ogun::DefectCategory::ThinFaces, mesh, ogun::CheckParams{});
    REQUIRE(thin.size() == 3);
    REQUIRE(thin.elements[0].index == ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(thin.elements[0].subtype == static_cast<int>(ogun::ThinFaceSubtype::ZeroArea));
    for (std::size_t i = 1; i < thin.size(); ++i) {
        REQUIRE(thin.elements[i - 1].metric <= thin.elements[i].metric);
    }
    REQUIRE(thin.elapsedMs >= 0.0);
}

TEST_CASE("A stopped check returns a cancelled result", "[diagnostics][mesh-checks]") {
    std::stop_source source;
    source.request_stop();
    const ogun::Mesh mesh = cube();
    for (const ogun::DefectCategory category : ogun::kAllDefectCategories) {
        INFO(ogun::categoryInfo(category).name);
        REQUIRE(ogun::runCategoryCheck(category, mesh, ogun::CheckParams{}, source.get_token()).cancelled);
    }
}

TEST_CASE("Every category has a descriptor with a name and columns", "[diagnostics][mesh-checks]") {
    for (const ogun::DefectCategory category : ogun::kAllDefectCategories) {
        const ogun::CategoryInfo& info = ogun::categoryInfo(category);
        REQUIRE(info.name != nullptr);
        REQUIRE(info.columns[0].name != nullptr);
    }
    REQUIRE(ogun::categorySubtypeCount(ogun::DefectCategory::ThinFaces) == 4);
    REQUIRE(ogun::categorySubtypeCount(ogun::DefectCategory::OpenEdges) == 0);
    REQUIRE(ogun::categoryInfo(ogun::DefectCategory::Shells).infoOnly);
    REQUIRE(ogun::categoryInfo(ogun::DefectCategory::ThinFaces).hasFixAll);
}
