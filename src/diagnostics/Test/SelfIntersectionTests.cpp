#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/TriTriIntersect.hpp"
#include "io/SampleMeshes.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <iterator>
#include <map>
#include <random>
#include <set>

namespace {

using FacePartners = std::map<Eigen::Index, std::set<Eigen::Index>>;

FacePartners partnersOf(const mesheal::CategoryResult& result) {
    FacePartners partners;
    for (std::size_t i = 0; i < result.size(); ++i) {
        REQUIRE(result.facesOf(i).size() == 1);
        const auto related = result.relatedOf(i);
        partners[result.facesOf(i).front()] = {related.begin(), related.end()};
    }
    return partners;
}

/// O(n^2) reference: every pair through the same triangle test.
FacePartners bruteForce(const mesheal::Mesh& mesh, double relativeTol) {
    const double tol = relativeTol * mesh.boundingBox().diagonal();
    FacePartners partners;
    for (Eigen::Index i = 0; i < mesh.numFaces(); ++i) {
        for (Eigen::Index j = i + 1; j < mesh.numFaces(); ++j) {
            int shared = 0;
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) {
                    shared += mesh.F(i, a) == mesh.F(j, b) ? 1 : 0;
                }
            }
            const mesheal::Triangle3d ti{mesh.V.row(mesh.F(i, 0)).transpose(), mesh.V.row(mesh.F(i, 1)).transpose(),
                                      mesh.V.row(mesh.F(i, 2)).transpose()};
            const mesheal::Triangle3d tj{mesh.V.row(mesh.F(j, 0)).transpose(), mesh.V.row(mesh.F(j, 1)).transpose(),
                                      mesh.V.row(mesh.F(j, 2)).transpose()};
            if (mesheal::intersectTriangles(ti, tj, shared, tol).hit) {
                partners[i].insert(j);
                partners[j].insert(i);
            }
        }
    }
    return partners;
}

/// Random small triangles in the unit cube; every few triangles reuse a vertex of the previous one.
mesheal::Mesh randomSoup(int count, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> position(0.0, 1.0);
    std::uniform_real_distribution<double> offset(-0.06, 0.06);
    mesheal::Mesh mesh;
    mesh.V.resize(3 * count, 3);
    mesh.F.resize(count, 3);
    for (int t = 0; t < count; ++t) {
        const Eigen::RowVector3d center(position(rng), position(rng), position(rng));
        for (int c = 0; c < 3; ++c) {
            mesh.V.row(3 * t + c) = center + Eigen::RowVector3d(offset(rng), offset(rng), offset(rng));
            mesh.F(t, c) = 3 * t + c;
        }
        if (t > 0 && t % 5 == 0) {
            mesh.F(t, 0) = mesh.F(t - 1, 2); // shared vertex with the previous triangle
        }
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Faces CGAL's exact-predicate soup test reports (it also counts point contacts and degenerate faces).
std::set<Eigen::Index> cgalIntersectingFaces(const mesheal::Mesh& mesh) {
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    std::vector<Kernel::Point_3> points;
    std::vector<std::array<std::size_t, 3>> triangles;
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        points.emplace_back(mesh.V(v, 0), mesh.V(v, 1), mesh.V(v, 2));
    }
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        triangles.push_back({static_cast<std::size_t>(mesh.F(f, 0)), static_cast<std::size_t>(mesh.F(f, 1)),
                             static_cast<std::size_t>(mesh.F(f, 2))});
    }
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    CGAL::Polygon_mesh_processing::triangle_soup_self_intersections(points, triangles, std::back_inserter(pairs));
    std::set<Eigen::Index> faces;
    for (const auto& [a, b] : pairs) {
        faces.insert(static_cast<Eigen::Index>(a));
        faces.insert(static_cast<Eigen::Index>(b));
    }
    return faces;
}

std::set<Eigen::Index> faceSet(const mesheal::CategoryResult& result) {
    std::set<Eigen::Index> faces;
    for (std::size_t i = 0; i < result.size(); ++i) {
        faces.insert(result.facesOf(i).front());
    }
    return faces;
}

mesheal::Mesh twoOverlappingCubes() {
    const mesheal::Mesh a = mesheal::SampleMeshes::createCubeMesh(10.0);
    mesheal::Mesh mesh = a;
    mesh.V.conservativeResize(16, 3);
    mesh.F.conservativeResize(24, 3);
    mesh.V.bottomRows(8) = a.V.rowwise() + Eigen::RowVector3d(5.0, 5.0, 5.0);
    mesh.F.bottomRows(12) = a.F.array() + 8;
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("A closed cube has no intersecting faces", "[diagnostics][intersections]") {
    const mesheal::CategoryResult result =
        mesheal::checkSelfIntersections(mesheal::SampleMeshes::createCubeMesh(10.0), mesheal::CheckParams{});
    REQUIRE_FALSE(result.cancelled);
    REQUIRE(result.empty());
}

TEST_CASE("The corner-pushed box intersects itself with symmetric partners", "[diagnostics][intersections]") {
    const mesheal::Mesh mesh = mesheal::SampleMeshes::createSelfIntersectingBoxMesh();
    const mesheal::CategoryResult result = mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{});
    REQUIRE_FALSE(result.empty());
    const FacePartners partners = partnersOf(result);
    for (const auto& [face, others] : partners) {
        for (const Eigen::Index other : others) {
            REQUIRE(partners.at(other).contains(face));
        }
    }
    REQUIRE(partners.contains(0)); // the bottom face is pierced
    REQUIRE(partners == bruteForce(mesh, mesheal::CheckParams{}.intersectionTolerance));
    for (std::size_t i = 1; i < result.size(); ++i) {
        REQUIRE(result.elements[i - 1].metric >= result.elements[i].metric); // most crossings first
    }
}

TEST_CASE("Two overlapping cubes intersect on the faces inside each other", "[diagnostics][intersections]") {
    const mesheal::CategoryResult result = mesheal::checkSelfIntersections(twoOverlappingCubes(), mesheal::CheckParams{});
    std::set<Eigen::Index> faces;
    for (std::size_t i = 0; i < result.size(); ++i) {
        faces.insert(result.facesOf(i).front());
        REQUIRE(result.elements[i].subtype == 0); // crossing, not coplanar
    }
    // Cube A's +Z, +Y, +X faces and cube B's -Z, -Y, -X faces. A's back triangle 6 (x + z <= 10)
    // touches B only at the point (5, 10, 5), so it is not reported.
    const std::set<Eigen::Index> expected{2, 3, 7, 10, 11, 12, 13, 16, 17, 20, 21};
    REQUIRE(faces == expected);
}

TEST_CASE("A face folded flat onto its neighbour is a coplanar overlap", "[diagnostics][intersections]") {
    mesheal::Mesh mesh;
    mesh.V.resize(4, 3);
    mesh.V << 0, 0, 0, 2, 0, 0, 0, 2, 0, 1, 1, 0;
    mesh.F.resize(2, 3);
    mesh.F << 0, 1, 2, 1, 0, 3;
    mesh.recomputeNormals();
    const mesheal::CategoryResult result = mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{});
    REQUIRE(result.size() == 2);
    REQUIRE(result.elements[0].subtype == 1);
    REQUIRE(result.subtypeCounts()[1] == 2);
}

TEST_CASE("The BVH finds the same pairs as brute force, threaded or not", "[diagnostics][intersections]") {
    const mesheal::Mesh mesh = randomSoup(1100, 7);
    const mesheal::CheckParams params;
    std::atomic<float> progress{0.0f};
    const mesheal::CategoryResult serial = mesheal::checkSelfIntersections(mesh, params, {}, &progress, 1);
    REQUIRE(progress.load() == 1.0f);
    const mesheal::CategoryResult threaded = mesheal::checkSelfIntersections(mesh, params, {}, nullptr, 4);

    const FacePartners expected = bruteForce(mesh, params.intersectionTolerance);
    REQUIRE_FALSE(expected.empty());
    REQUIRE(partnersOf(serial) == expected);
    REQUIRE(partnersOf(threaded) == expected);
    REQUIRE(serial.faces == threaded.faces);
}

TEST_CASE("The intersection row is a geometry row with Auto off", "[diagnostics][intersections]") {
    const mesheal::CategoryInfo& info = mesheal::categoryInfo(mesheal::DefectCategory::Intersections);
    REQUIRE_FALSE(info.autoDefault);
    REQUIRE(info.group == mesheal::CategoryGroup::Geometry);
    REQUIRE(mesheal::categorySubtypeCount(mesheal::DefectCategory::Intersections) == 2);

    const mesheal::CategoryResult viaRow = mesheal::runCategoryCheck(
        mesheal::DefectCategory::Intersections, mesheal::SampleMeshes::createSelfIntersectingBoxMesh(), mesheal::CheckParams{});
    REQUIRE(viaRow.category == mesheal::DefectCategory::Intersections);
    REQUIRE_FALSE(viaRow.empty());
}

TEST_CASE("Intersecting faces match CGAL's exact predicates on generic meshes", "[diagnostics][intersections]") {
    // No point contacts or degenerate faces here, so both definitions agree (the two overlapping cubes
    // touch at one point, which CGAL counts)
    for (const mesheal::Mesh& mesh : {randomSoup(1100, 11), mesheal::SampleMeshes::createSelfIntersectingBoxMesh()}) {
        REQUIRE(faceSet(mesheal::checkSelfIntersections(mesh, mesheal::CheckParams{})) == cgalIntersectingFaces(mesh));
    }
}
