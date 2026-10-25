#include "diagnostics/ShellNesting.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <vector>

namespace {

/// Cubes of the given side and min corner, one shell each; `shells` gets their face lists.
ogun::Mesh cubes(const std::vector<std::pair<double, Eigen::Vector3d>>& specs,
                 std::vector<std::vector<Eigen::Index>>& shells) {
    ogun::Mesh mesh;
    for (const auto& [side, offset] : specs) {
        ogun::Mesh c = ogun::SampleMeshes::createCubeMesh(side, ogun::Unit::Millimeter);
        c.V.rowwise() += offset.transpose();
        const Eigen::Index nv = mesh.V.rows();
        const Eigen::Index nf = mesh.F.rows();
        mesh.V.conservativeResize(nv + c.V.rows(), 3);
        mesh.F.conservativeResize(nf + c.F.rows(), 3);
        mesh.V.bottomRows(c.V.rows()) = c.V;
        mesh.F.bottomRows(c.F.rows()) = c.F.array() + static_cast<int>(nv);
        std::vector<Eigen::Index> faces(static_cast<std::size_t>(c.F.rows()));
        std::iota(faces.begin(), faces.end(), nf);
        shells.push_back(std::move(faces));
    }
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("rayHitsTriangle counts hits in front of the origin only", "[diagnostics][shells]") {
    const Eigen::Vector3d a(0.0, 0.0, 0.0);
    const Eigen::Vector3d b(1.0, 0.0, 0.0);
    const Eigen::Vector3d c(0.0, 1.0, 0.0);
    REQUIRE(ogun::rayHitsTriangle({0.2, 0.2, -1.0}, {0.0, 0.0, 1.0}, a, b, c));
    REQUIRE_FALSE(ogun::rayHitsTriangle({0.2, 0.2, 1.0}, {0.0, 0.0, 1.0}, a, b, c));
    REQUIRE_FALSE(ogun::rayHitsTriangle({0.8, 0.8, -1.0}, {0.0, 0.0, 1.0}, a, b, c));
    REQUIRE_FALSE(ogun::rayHitsTriangle({0.2, 0.2, -1.0}, {1.0, 0.0, 0.0}, a, b, c));
}

TEST_CASE("findNestedShells reports the smallest shell that contains another", "[diagnostics][shells]") {
    std::vector<std::vector<Eigen::Index>> shells;
    const ogun::Mesh mesh = cubes({{10.0, {0.0, 0.0, 0.0}},
                                   {4.0, {3.0, 3.0, 3.0}},
                                   {1.0, {4.5, 4.5, 4.5}},
                                   {2.0, {20.0, 0.0, 0.0}}},
                                  shells);

    const std::vector<int> nested = ogun::findNestedShells(mesh, shells, {});
    REQUIRE(nested == std::vector<int>{-1, 0, 1, -1});

    // A crossing with the container rules it out; the next container is used instead
    const std::vector<int> crossing = ogun::findNestedShells(mesh, shells, {{1, 2}});
    REQUIRE(crossing == std::vector<int>{-1, 0, 0, -1});
}
