#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

namespace {

ogun::Mesh makeTestTetrahedron(ogun::Unit unit = ogun::Unit::Millimeter) {
    ogun::Mesh mesh;
    mesh.fileUnit = unit;
    mesh.V.resize(4, 3);
    mesh.V << 0.0, 0.0, 0.0,
              10.0, 0.0, 0.0,
              0.0, 20.0, 0.0,
              0.0, 0.0, 30.0;

    mesh.F.resize(4, 3);
    mesh.F << 0, 2, 1,
              0, 1, 3,
              0, 3, 2,
              1, 2, 3;

    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("Changing File Units leaves vertex coordinates V and the weld floor untouched",
          "[mesh][units][tolerance]") {
    ogun::Mesh mesh = makeTestTetrahedron(ogun::Unit::Millimeter);
    const Eigen::MatrixXd originalV = mesh.V;
    const ogun::ToleranceConfig tolConfig; // Default: absFloor = 1e-9, relTol = 1e-9

    ogun::setFileUnit(mesh, ogun::Unit::Centimeter);

    REQUIRE(mesh.fileUnit == ogun::Unit::Centimeter);
    REQUIRE(mesh.V.isApprox(originalV, 0.0));
    REQUIRE(tolConfig.absFloor == ogun::settings::kDefaultWeldAbsFloor);
    REQUIRE(tolConfig.coordinateTolerance(0.0, 0.0) == ogun::settings::kDefaultWeldAbsFloor);
}

TEST_CASE("extractFaces keeps only the listed faces and the vertices they use", "[mesh]") {
    const ogun::Mesh mesh = makeTestTetrahedron(ogun::Unit::Inch);
    const std::vector<Eigen::Index> faces{3, 1};

    const ogun::Mesh part = ogun::extractFaces(mesh, faces);
    REQUIRE(part.fileUnit == ogun::Unit::Inch);
    REQUIRE(part.numFaces() == 2);
    REQUIRE(part.numVertices() == 4);
    REQUIRE(part.N.rows() == 2);
    for (Eigen::Index row = 0; row < part.numFaces(); ++row) {
        for (int corner = 0; corner < 3; ++corner) {
            REQUIRE(part.V.row(part.F(row, corner)) == mesh.V.row(mesh.F(faces[static_cast<std::size_t>(row)], corner)));
        }
    }

    const std::vector<Eigen::Index> one{2};
    const ogun::Mesh single = ogun::extractFaces(mesh, one);
    REQUIRE(single.numVertices() == 3);
    REQUIRE(single.F.maxCoeff() == 2);

    REQUIRE(ogun::extractFaces(mesh, {}).empty());
}

TEST_CASE("Explicit scaleMeshToUnit multiplies V by exact scale factor and updates bounding box",
          "[mesh][units]") {
    ogun::Mesh mesh = makeTestTetrahedron(ogun::Unit::Millimeter);
    const Eigen::MatrixXd originalV = mesh.V;

    const ogun::BoundingBox bboxBefore = mesh.boundingBox();
    REQUIRE(bboxBefore.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
    REQUIRE(bboxBefore.max.isApprox(Eigen::Vector3d(10.0, 20.0, 30.0)));
    REQUIRE(bboxBefore.extents().isApprox(Eigen::Vector3d(10.0, 20.0, 30.0)));

    // Explicitly scale coordinates from mm to cm (factor = 0.1)
    ogun::scaleMeshToUnit(mesh, ogun::Unit::Centimeter);

    REQUIRE(mesh.fileUnit == ogun::Unit::Centimeter);
    REQUIRE(mesh.V.isApprox(originalV * 0.1, 1e-15));

    const ogun::BoundingBox bboxAfter = mesh.boundingBox();
    REQUIRE(bboxAfter.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
    REQUIRE(bboxAfter.max.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    REQUIRE(bboxAfter.extents().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

    // Scaling from cm to inch (1 cm = 1 / 2.54 in) and back to mm (1 in = 25.4 mm)
    ogun::scaleMeshToUnit(mesh, ogun::Unit::Inch);
    REQUIRE(mesh.fileUnit == ogun::Unit::Inch);
    REQUIRE(mesh.boundingBox().extents().isApprox(Eigen::Vector3d(1.0 / 2.54, 2.0 / 2.54, 3.0 / 2.54), 1e-12));

    ogun::scaleMeshToUnit(mesh, ogun::Unit::Millimeter);
    REQUIRE(mesh.fileUnit == ogun::Unit::Millimeter);
    REQUIRE(mesh.V.isApprox(originalV, 1e-12));
}

TEST_CASE("Round-trip conversion between ogun::Mesh and manifold::MeshGL",
          "[mesh][manifold]") {
    const ogun::Mesh original = makeTestTetrahedron(ogun::Unit::Centimeter);

    const manifold::MeshGL meshGl = ogun::toMeshGL(original);
    REQUIRE(meshGl.numProp == 3);
    REQUIRE(meshGl.NumVert() == 4);
    REQUIRE(meshGl.NumTri() == 4);

    const ogun::Mesh roundTrip = ogun::fromMeshGL(meshGl, original.fileUnit);
    REQUIRE(roundTrip.fileUnit == original.fileUnit);
    REQUIRE(roundTrip.numVertices() == original.numVertices());
    REQUIRE(roundTrip.numFaces() == original.numFaces());
    REQUIRE(roundTrip.V.isApprox(original.V, 1e-6));
    REQUIRE(roundTrip.F == original.F);
    REQUIRE(roundTrip.N.rows() == 4);
    REQUIRE(roundTrip.N.cols() == 3);

    // Also verify double-precision MeshGL64 round-trip preserves exact double coordinates
    const manifold::MeshGL64 meshGl64 = original.toMeshGL64();
    const ogun::Mesh roundTrip64 = ogun::Mesh::fromMeshGL64(meshGl64, original.fileUnit);
    REQUIRE(roundTrip64.V.isApprox(original.V, 0.0));
    REQUIRE(roundTrip64.F == original.F);
}

TEST_CASE("Local coordinate-relative tolerance uses the 1e-9 floor near origin and scales at large coordinates",
          "[mesh][tolerance]") {
    const ogun::ToleranceConfig config{1e-9, 1e-9};

    SECTION("Near the origin the 1e-9 atomic floor dominates") {
        const double epsOrigin = config.coordinateTolerance(0.0, 1e-12);
        REQUIRE(epsOrigin == Catch::Approx(1e-9).epsilon(1e-15));

        const Eigen::Vector3d p(0.0, 0.0, 0.0);
        const Eigen::Vector3d qClose(5e-10, -5e-10, 9e-10);
        const Eigen::Vector3d qFar(2e-9, 0.0, 0.0);
        REQUIRE(config.verticesMatchWithinTolerance(p, qClose));
        REQUIRE_FALSE(config.verticesMatchWithinTolerance(p, qFar));
    }

    SECTION("At large coordinates (e.g., 1e9 cm), relative tolerance scales proportionally") {
        const double largeCoord = 1e9; // 10,000 km in cm
        const double epsLarge = config.coordinateTolerance(largeCoord, largeCoord + 0.5);
        // relTol * max(|p|, |q|) ~= 1e-9 * 1e9 = 1.0 cm >> 1e-9 cm floor
        REQUIRE(epsLarge == Catch::Approx(1e-9 * (largeCoord + 0.5)).epsilon(1e-12));

        const Eigen::Vector3d p(largeCoord, -largeCoord, 0.0);
        const Eigen::Vector3d qWithinRel(largeCoord + 0.5, -largeCoord - 0.5, 5e-10);
        const Eigen::Vector3d qOutsideRel(largeCoord + 2.0, -largeCoord, 0.0);
        REQUIRE(config.verticesMatchWithinTolerance(p, qWithinRel));
        REQUIRE_FALSE(config.verticesMatchWithinTolerance(p, qOutsideRel));
    }
}

// ---------------------------------------------------------------------------
// CGAL interop
// ---------------------------------------------------------------------------

#include "mesh/CgalMesh.hpp"

#include <CGAL/boost/graph/Euler_operations.h>

namespace {

ogun::Mesh makeTestOctahedron() {
    ogun::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << 1.0, 0.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, -1.0, 0.0,
              0.0, 0.0, 1.0,
              0.0, 0.0, -1.0;
    mesh.F.resize(8, 3);
    mesh.F << 0, 2, 4,
              2, 1, 4,
              1, 3, 4,
              3, 0, 4,
              2, 0, 5,
              1, 2, 5,
              3, 1, 5,
              0, 3, 5;
    mesh.recomputeNormals();
    return mesh;
}

} // namespace

TEST_CASE("CGAL round trip preserves coordinates, faces and normals", "[mesh][cgal]") {
    const ogun::Mesh original = makeTestTetrahedron(ogun::Unit::Centimeter);

    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(original, surface));
    REQUIRE(surface.number_of_vertices() == 4);
    REQUIRE(surface.number_of_faces() == 4);

    const ogun::Mesh back = ogun::fromCgalSurfaceMesh(surface, original.fileUnit);
    REQUIRE(back.fileUnit == ogun::Unit::Centimeter);
    REQUIRE(back.V == original.V);
    REQUIRE(back.F.rows() == original.F.rows());
    for (Eigen::Index f = 0; f < original.F.rows(); ++f) {
        REQUIRE(back.F(f, 0) == original.F(f, 0));
        REQUIRE(back.F(f, 1) == original.F(f, 1));
        REQUIRE(back.F(f, 2) == original.F(f, 2));
    }
    REQUIRE(back.N.isApprox(original.N, 1e-12));
}

TEST_CASE("fromCgalSurfaceMesh compacts lazily deleted elements", "[mesh][cgal]") {
    const ogun::Mesh original = makeTestOctahedron();
    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(original, surface));

    const ogun::CgalEdgeIndex edge = *surface.edges().begin();
    REQUIRE(CGAL::Euler::does_satisfy_link_condition(edge, surface));
    CGAL::Euler::collapse_edge(edge, surface);
    REQUIRE(surface.has_garbage());

    const ogun::Mesh healed = ogun::fromCgalSurfaceMesh(surface);
    REQUIRE(healed.numVertices() == 5);
    REQUIRE(healed.numFaces() == 6);
    REQUIRE(healed.N.rows() == 6);
    REQUIRE(healed.F.minCoeff() >= 0);
    REQUIRE(healed.F.maxCoeff() < healed.numVertices());
}

TEST_CASE("fromCgalSurfaceMesh offsets vertices along their normals", "[mesh][cgal]") {
    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(makeTestOctahedron(), surface));

    const ogun::Mesh offset = ogun::fromCgalSurfaceMesh(surface, ogun::Unit::Millimeter, 0.1);
    REQUIRE(offset.V.row(0).norm() == Catch::Approx(1.1).epsilon(1e-9));
}

TEST_CASE("extractLocalPatch returns the k-ring neighborhood", "[mesh][cgal]") {
    ogun::CgalSurfaceMesh surface;
    REQUIRE(ogun::toCgalSurfaceMesh(makeTestOctahedron(), surface));

    ogun::CgalSurfaceMesh patch;
    const ogun::CgalFaceIndex center = ogun::extractLocalPatch(surface, ogun::CgalFaceIndex(0), 1, patch);
    REQUIRE(center != ogun::CgalSurfaceMesh::null_face());
    REQUIRE(patch.is_valid(center));
    // Face 0 plus every face touching one of its three vertices (all but face 6).
    REQUIRE(patch.number_of_faces() == 7);

    const ogun::CgalFaceIndex zero = ogun::extractLocalPatch(surface, ogun::CgalFaceIndex(0), 0, patch);
    REQUIRE(zero != ogun::CgalSurfaceMesh::null_face());
    REQUIRE(patch.number_of_faces() == 1);

    const ogun::CgalFaceIndex two = ogun::extractLocalPatch(surface, ogun::CgalFaceIndex(0), 2, patch);
    REQUIRE(two != ogun::CgalSurfaceMesh::null_face());
    REQUIRE(patch.number_of_faces() == 8);
}
