#include "diagnostics/TriTriIntersect.hpp"

#include <catch2/catch_test_macros.hpp>

namespace {

constexpr double kTol = 1.0e-9;

ogun::Triangle3d tri(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
    return {a, b, c};
}

/// Right triangle in the z = 0 plane with legs of length 2 along +x and +y.
const ogun::Triangle3d kBase = tri({0, 0, 0}, {2, 0, 0}, {0, 2, 0});

ogun::TriTriHit test(const ogun::Triangle3d& b, int shared = 0) {
    return ogun::intersectTriangles(kBase, b, shared, kTol);
}

} // namespace

TEST_CASE("Separated and plane-rejected triangles do not intersect", "[diagnostics][tritri]") {
    REQUIRE_FALSE(test(tri({0, 0, 1}, {2, 0, 1}, {0, 2, 1})).hit);
    // Crosses the plane z = 0, but outside the base triangle
    REQUIRE_FALSE(test(tri({3, 3, -1}, {4, 3, -1}, {3.5, 3, 1})).hit);
}

TEST_CASE("Crossing triangles intersect, in either order", "[diagnostics][tritri]") {
    const ogun::Triangle3d small = tri({0.25, 0.5, -1}, {0.75, 0.5, -1}, {0.5, 0.5, 1});
    const ogun::TriTriHit hit = test(small);
    REQUIRE(hit.hit);
    REQUIRE_FALSE(hit.coplanar);
    REQUIRE(ogun::intersectTriangles(small, kBase, 0, kTol).hit);

    // A big triangle whose edge pierces the base and continues past it
    REQUIRE(test(tri({-5, 0.5, -1}, {5, 0.5, -1}, {0, 0.5, 1})).hit);
}

TEST_CASE("Touching at a point is not an intersection; an edge lying on a face is", "[diagnostics][tritri]") {
    // Vertex touching the base's interior
    REQUIRE_FALSE(test(tri({0.5, 0.5, 0}, {0, 0.5, 1}, {1, 0.5, 1})).hit);
    // Edge resting on the base (surface contact over a segment, as in Blender)
    REQUIRE(test(tri({0.2, 0.2, 0}, {0.6, 0.2, 0}, {0.4, 0.2, 1})).hit);
}

TEST_CASE("Faces sharing one vertex intersect only when they cross", "[diagnostics][tritri]") {
    // Passes through the base along (0,0,0)-(1,1,0)
    REQUIRE(test(tri({0, 0, 0}, {1, 1, -1}, {1, 1, 1}), 1).hit);
    // A fan neighbour that meets the base only at the shared vertex
    REQUIRE_FALSE(test(tri({0, 0, 0}, {-1, 0, 1}, {0, -1, 1}), 1).hit);
}

TEST_CASE("Edge neighbours intersect only when folded flat onto each other", "[diagnostics][tritri]") {
    REQUIRE_FALSE(test(tri({0, 0, 0}, {2, 0, 0}, {1, 0, 1}), 2).hit);
    REQUIRE_FALSE(test(tri({0, 0, 0}, {2, 0, 0}, {1, -1, 0}), 2).hit); // flat, opposite sides
    const ogun::TriTriHit folded = test(tri({0, 0, 0}, {2, 0, 0}, {1, 1, 0}), 2);
    REQUIRE(folded.hit);
    REQUIRE(folded.coplanar);
    REQUIRE_FALSE(test(kBase, 3).hit); // duplicates are the Duplicate faces row's job
}

TEST_CASE("Coplanar triangles intersect only when their areas overlap", "[diagnostics][tritri]") {
    const ogun::TriTriHit overlap = test(tri({0.5, 0.5, 0}, {2.5, 0.5, 0}, {0.5, 2.5, 0}));
    REQUIRE(overlap.hit);
    REQUIRE(overlap.coplanar);
    REQUIRE(test(kBase).hit); // identical copy with its own vertices

    const ogun::TriTriHit disjoint = test(tri({3, 3, 0}, {4, 3, 0}, {3, 4, 0}));
    REQUIRE_FALSE(disjoint.hit);
    REQUIRE(disjoint.coplanar);
    // Touching along the hypotenuse, and a T-junction on the bottom edge
    REQUIRE_FALSE(test(tri({2, 0, 0}, {2, 2, 0}, {0, 2, 0})).hit);
    REQUIRE_FALSE(test(tri({1, 0, 0}, {2, -1, 0}, {0, -1, 0})).hit);
}

TEST_CASE("Nearly coplanar triangles within the tolerance use the coplanar test", "[diagnostics][tritri]") {
    const ogun::TriTriHit hit = test(tri({0.5, 0.5, 1e-12}, {2.5, 0.5, -1e-12}, {0.5, 2.5, 0}));
    REQUIRE(hit.hit);
    REQUIRE(hit.coplanar);
    // Stacked on top of the base, closer than the tolerance
    const ogun::TriTriHit stacked = test(tri({0.5, 0.5, 1e-12}, {2.5, 0.5, 1e-12}, {0.5, 2.5, 1e-12}));
    REQUIRE(stacked.hit);
    REQUIRE(stacked.coplanar);
    REQUIRE_FALSE(test(tri({0.5, 0.5, 1e-3}, {2.5, 0.5, 1e-3}, {0.5, 2.5, 1e-3})).hit);
}

TEST_CASE("Nearly parallel vertex neighbours that only touch are not reported", "[diagnostics][tritri]") {
    // Fan neighbour bent 1e-4 rad out of the base plane: its far vertices sit within a
    // tolerance-sized distance of the plane but far from the intersection line
    const double lift = 1.0e-4;
    const ogun::Triangle3d bent = tri({0, 0, 0}, {-2, 0, 2 * lift}, {0, -2, 2 * lift});
    REQUIRE_FALSE(ogun::intersectTriangles(kBase, bent, 1, 1.0e-3).hit);
    REQUIRE_FALSE(ogun::intersectTriangles(kBase, bent, 1, kTol).hit);
}

TEST_CASE("Degenerate triangles never intersect", "[diagnostics][tritri]") {
    REQUIRE_FALSE(test(tri({0.5, 0.5, -1}, {0.5, 0.5, 1}, {0.5, 0.5, 0})).hit);
    REQUIRE_FALSE(ogun::intersectTriangles(tri({0, 0, 0}, {1, 0, 0}, {2, 0, 0}), kBase, 0, kTol).hit);
}
