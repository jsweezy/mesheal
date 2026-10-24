#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("Changing File Units leaves vertex coordinates V untouched and converts atomic floor tolerance",
          "[mesh][units][tolerance]") {
    ogun::Mesh mesh = makeTestTetrahedron(ogun::Unit::Millimeter);
    const Eigen::MatrixXd originalV = mesh.V;
    const ogun::ToleranceConfig tolConfig; // Default: absFloorCm = 1e-9, relTol = 1e-9

    REQUIRE(mesh.fileUnit == ogun::Unit::Millimeter);
    REQUIRE(tolConfig.absFloorInUnit(mesh.fileUnit) == Catch::Approx(1e-8).epsilon(1e-12));

    // Re-interpret file units from mm to cm without scaling geometry
    ogun::setFileUnit(mesh, ogun::Unit::Centimeter);

    REQUIRE(mesh.fileUnit == ogun::Unit::Centimeter);
    REQUIRE(mesh.V.isApprox(originalV, 0.0));
    REQUIRE(tolConfig.absFloorInUnit(mesh.fileUnit) == Catch::Approx(1e-9).epsilon(1e-12));

    // Check additional supported units
    REQUIRE(tolConfig.absFloorInUnit(ogun::Unit::Meter) == Catch::Approx(1e-11).epsilon(1e-12));
    REQUIRE(tolConfig.absFloorInUnit(ogun::Unit::Inch) == Catch::Approx(1e-9 / 2.54).epsilon(1e-12));
    REQUIRE(tolConfig.absFloorInUnit(ogun::Unit::Foot) == Catch::Approx(1e-9 / 30.48).epsilon(1e-12));
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

TEST_CASE("Local coordinate-relative tolerance uses 1e-9 cm floor near origin and scales at large coordinates",
          "[mesh][tolerance]") {
    const ogun::ToleranceConfig config{1e-9, 1e-9};

    SECTION("Near the origin in cm, the 1e-9 cm atomic floor dominates") {
        const double epsOrigin = config.coordinateTolerance(0.0, 1e-12, ogun::Unit::Centimeter);
        REQUIRE(epsOrigin == Catch::Approx(1e-9).epsilon(1e-15));

        const Eigen::Vector3d p(0.0, 0.0, 0.0);
        const Eigen::Vector3d qClose(5e-10, -5e-10, 9e-10);
        const Eigen::Vector3d qFar(2e-9, 0.0, 0.0);
        REQUIRE(config.verticesMatchWithinTolerance(p, qClose, ogun::Unit::Centimeter));
        REQUIRE_FALSE(config.verticesMatchWithinTolerance(p, qFar, ogun::Unit::Centimeter));
    }

    SECTION("Near the origin in mm, the converted 1e-8 mm floor dominates") {
        const double epsOriginMm = config.coordinateTolerance(0.0, 1e-12, ogun::Unit::Millimeter);
        REQUIRE(epsOriginMm == Catch::Approx(1e-8).epsilon(1e-15));
    }

    SECTION("At large coordinates (e.g., 1e9 cm), relative tolerance scales proportionally") {
        const double largeCoord = 1e9; // 10,000 km in cm
        const double epsLarge = config.coordinateTolerance(largeCoord, largeCoord + 0.5, ogun::Unit::Centimeter);
        // relTol * max(|p|, |q|) ~= 1e-9 * 1e9 = 1.0 cm >> 1e-9 cm floor
        REQUIRE(epsLarge == Catch::Approx(1e-9 * (largeCoord + 0.5)).epsilon(1e-12));

        const Eigen::Vector3d p(largeCoord, -largeCoord, 0.0);
        const Eigen::Vector3d qWithinRel(largeCoord + 0.5, -largeCoord - 0.5, 5e-10);
        const Eigen::Vector3d qOutsideRel(largeCoord + 2.0, -largeCoord, 0.0);
        REQUIRE(config.verticesMatchWithinTolerance(p, qWithinRel, ogun::Unit::Centimeter));
        REQUIRE_FALSE(config.verticesMatchWithinTolerance(p, qOutsideRel, ogun::Unit::Centimeter));
    }
}
