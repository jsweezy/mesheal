#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

TEST_CASE("ThinFaceChecker reports zero defects on a clean cube mesh and computes exact metrics for regular faces",
          "[diagnostics][thin-faces]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(10.0, ogun::Unit::Millimeter);
    const ogun::ThinFaceThresholds thresholds; // minAltitude = 1e-3, minArea = 1e-6, maxAspectRatio = 1000

    const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(cube, thresholds);

    REQUIRE(report.unit == ogun::Unit::Millimeter);
    REQUIRE(report.totalFacesChecked == 12);
    REQUIRE(report.empty());
    REQUIRE_FALSE(report.hasDefects());
    REQUIRE(report.numDefects() == 0);
    REQUIRE(report.defects.empty());
    REQUIRE(report.faceIndices.empty());

    // Verify per-face metrics on a regular right-isosceles cube triangle (legs = 10, hyp = 10*sqrt(2))
    const ogun::ThinFaceDefect face0 = ogun::ThinFaceChecker::evaluateFace(cube, 0, thresholds);
    const double expectedHypotenuse = 10.0 * std::sqrt(2.0);
    const double expectedMinAltitude = 100.0 / expectedHypotenuse; // 5 * sqrt(2) ~= 7.0710678

    REQUIRE(face0.faceIndex == 0);
    REQUIRE_FALSE(face0.isDefective());
    REQUIRE_FALSE(face0.isZeroArea());
    REQUIRE(face0.area == Catch::Approx(50.0).epsilon(1e-12));
    REQUIRE(face0.shortestEdge == Catch::Approx(10.0).epsilon(1e-12));
    REQUIRE(face0.longestEdge == Catch::Approx(expectedHypotenuse).epsilon(1e-12));
    REQUIRE(face0.minAltitude == Catch::Approx(expectedMinAltitude).epsilon(1e-12));
    REQUIRE(face0.aspectRatio == Catch::Approx(2.0).epsilon(1e-12));
}

TEST_CASE("ThinFaceChecker detects collinear zero-area, microscopic-area, and high-aspect-ratio needle/sliver faces",
          "[diagnostics][thin-faces]") {
    // Sample defective mesh contains 12 regular 10x10x10 mm cube triangles (faces 0..11) plus:
    //   Face 12: collinear zero-area triangle (A = 0, h_min = 0, edges = 5, 5, 10)
    //   Face 13: microscopic right-isosceles triangle (legs = 1e-4, A = 5e-9, h_min = 1e-4 / sqrt(2), AR = 2.0)
    //   Face 14: needle/sliver triangle (base = 10.0, altitude = 2e-4, A = 1e-3, AR = 50,000)
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh(ogun::Unit::Millimeter);
    const ogun::ThinFaceThresholds thresholds{
        1.0e-3, // minAltitude (mm)
        1.0e-6, // minArea (mm^2)
        1000.0  // maxAspectRatio
    };

    const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(mesh, thresholds);

    REQUIRE(report.totalFacesChecked == 15);
    REQUIRE(report.hasDefects());
    REQUIRE(report.numDefects() == 3);

    const std::vector<Eigen::Index> expectedIndices = {
        ogun::SampleMeshes::kZeroAreaFaceIndex,
        ogun::SampleMeshes::kMicroscopicFaceIndex,
        ogun::SampleMeshes::kNeedleSliverFaceIndex
    };
    REQUIRE(report.faceIndices == expectedIndices);

    // Regular cube faces (0..11) must not appear in the defect report
    for (Eigen::Index f = 0; f < ogun::SampleMeshes::kCleanCubeFaceCount; ++f) {
        REQUIRE(report.findDefectForFace(f) == nullptr);
    }

    // 1. Collinear zero-area triangle (Face 12)
    const ogun::ThinFaceDefect* zeroDefect = report.findDefectForFace(ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(zeroDefect != nullptr);
    REQUIRE(zeroDefect->faceIndex == 12);
    REQUIRE(zeroDefect->isDefective());
    REQUIRE(zeroDefect->isZeroArea());
    REQUIRE(zeroDefect->area == Catch::Approx(0.0).margin(1e-15));
    REQUIRE(zeroDefect->minAltitude == Catch::Approx(0.0).margin(1e-15));
    REQUIRE(zeroDefect->shortestEdge == Catch::Approx(5.0).epsilon(1e-12));
    REQUIRE(zeroDefect->longestEdge == Catch::Approx(10.0).epsilon(1e-12));
    REQUIRE( std::isinf(zeroDefect->aspectRatio));
    REQUIRE(zeroDefect->belowMinArea);
    REQUIRE(zeroDefect->belowMinAltitude);
    REQUIRE(zeroDefect->aboveMaxAspectRatio);

    // 2. Microscopic triangle (Face 13: legs = 1e-4, hyp = sqrt(2)*1e-4, A = 5e-9, AR = 2.0)
    const ogun::ThinFaceDefect* microDefect = report.findDefectForFace(ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(microDefect != nullptr);
    REQUIRE(microDefect->faceIndex == 13);
    REQUIRE(microDefect->isDefective());
    REQUIRE_FALSE(microDefect->isZeroArea());
    REQUIRE(microDefect->area == Catch::Approx(5.0e-9).epsilon(1e-9));
    REQUIRE(microDefect->shortestEdge == Catch::Approx(1.0e-4).epsilon(1e-9));
    REQUIRE(microDefect->longestEdge == Catch::Approx(std::sqrt(2.0) * 1.0e-4).epsilon(1e-9));
    REQUIRE(microDefect->minAltitude == Catch::Approx(1.0e-4 / std::sqrt(2.0)).epsilon(1e-9));
    REQUIRE(microDefect->aspectRatio == Catch::Approx(2.0).epsilon(1e-9));
    REQUIRE(microDefect->belowMinArea);
    REQUIRE(microDefect->belowMinAltitude);
    REQUIRE_FALSE(microDefect->aboveMaxAspectRatio); // Right-isosceles shape has AR = 2.0 <= 1000

    // 3. Long needle/sliver triangle (Face 14: base = 10.0, h_min = 2e-4, A = 1e-3, AR = 50,000)
    const ogun::ThinFaceDefect* sliverDefect = report.findDefectForFace(ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(sliverDefect != nullptr);
    REQUIRE(sliverDefect->faceIndex == 14);
    REQUIRE(sliverDefect->isDefective());
    REQUIRE_FALSE(sliverDefect->isZeroArea());
    REQUIRE(sliverDefect->area == Catch::Approx(1.0e-3).epsilon(1e-9));
    REQUIRE(sliverDefect->minAltitude == Catch::Approx(2.0e-4).epsilon(1e-9));
    REQUIRE(sliverDefect->shortestEdge == Catch::Approx(std::hypot(5.0, 2.0e-4)).epsilon(1e-12));
    REQUIRE(sliverDefect->longestEdge == Catch::Approx(10.0).epsilon(1e-12));
    REQUIRE(sliverDefect->aspectRatio == Catch::Approx(50000.0).epsilon(1e-9));
    REQUIRE_FALSE(sliverDefect->belowMinArea); // Moderate area 1e-3 > 1e-6
    REQUIRE(sliverDefect->belowMinAltitude);   // Tiny altitude 2e-4 < 1e-3
    REQUIRE(sliverDefect->aboveMaxAspectRatio); // Huge aspect ratio 50,000 > 1000
}

TEST_CASE("ThinFaceChecker threshold criteria can isolate individual defect categories",
          "[diagnostics][thin-faces][thresholds]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh(ogun::Unit::Millimeter);

    SECTION("Isolating minArea catches zero-area and microscopic faces but not moderate-area needle sliver") {
        const ogun::ThinFaceThresholds areaOnlyThresholds{
            0.0, // disable positive minAltitude threshold
            1.0e-6,
            std::numeric_limits<double>::infinity() // disable finite maxAspectRatio threshold
        };

        const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(mesh, areaOnlyThresholds);
        const std::vector<Eigen::Index> expected = {
            ogun::SampleMeshes::kZeroAreaFaceIndex,
            ogun::SampleMeshes::kMicroscopicFaceIndex
        };
        REQUIRE(report.faceIndices == expected);
    }

    SECTION("Isolating maxAspectRatio catches zero-area and needle sliver but not well-shaped microscopic face") {
        const ogun::ThinFaceThresholds aspectOnlyThresholds{
            0.0, // disable positive minAltitude threshold
            0.0, // disable positive minArea threshold
            1000.0
        };

        const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(mesh, aspectOnlyThresholds);
        const std::vector<Eigen::Index> expected = {
            ogun::SampleMeshes::kZeroAreaFaceIndex,
            ogun::SampleMeshes::kNeedleSliverFaceIndex
        };
        REQUIRE(report.faceIndices == expected);
    }

    SECTION("Converting thresholds across physical units preserves exact diagnostic behavior") {
        ogun::Mesh meshCm = mesh;
        ogun::scaleMeshToUnit(meshCm, ogun::Unit::Centimeter);

        const ogun::ThinFaceThresholds thresholdsMm{1.0e-3, 1.0e-6, 1000.0};
        const ogun::ThinFaceThresholds thresholdsCm =
            thresholdsMm.convertedToUnit(ogun::Unit::Millimeter, ogun::Unit::Centimeter);

        REQUIRE(thresholdsCm.minAltitude == Catch::Approx(1.0e-4).epsilon(1e-12));
        REQUIRE(thresholdsCm.minArea == Catch::Approx(1.0e-8).epsilon(1e-12));
        REQUIRE(thresholdsCm.maxAspectRatio == Catch::Approx(1000.0).epsilon(1e-12));

        const ogun::ThinFaceReport reportCm = ogun::ThinFaceChecker::check(meshCm, thresholdsCm);
        REQUIRE(reportCm.unit == ogun::Unit::Centimeter);
        REQUIRE(reportCm.numDefects() == 3);
        REQUIRE(reportCm.faceIndices == std::vector<Eigen::Index>{12, 13, 14});
    }
}

TEST_CASE("ThinFaceChecker handles empty meshes and topologically degenerate faces with duplicate vertex indices",
          "[diagnostics][thin-faces][edge-cases]") {
    SECTION("Empty mesh produces empty report") {
        const ogun::Mesh emptyMesh;
        const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(emptyMesh);
        REQUIRE(report.empty());
        REQUIRE(report.totalFacesChecked == 0);
    }

    SECTION("Face with repeated vertex indices or all-coincident vertices is flagged as zero-area degenerate") {
        ogun::Mesh mesh;
        mesh.V.resize(3, 3);
        mesh.V << 0.0, 0.0, 0.0,
                  1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0;

        mesh.F.resize(2, 3);
        mesh.F << 0, 1, 2, // Face 0: valid right triangle
                  1, 1, 2; // Face 1: duplicate vertex index 1

        const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(mesh);
        REQUIRE(report.totalFacesChecked == 2);
        REQUIRE(report.faceIndices == std::vector<Eigen::Index>{1});
        REQUIRE(report.defects[0].hasDuplicateVertexIndices);
        REQUIRE(report.defects[0].isZeroArea());
        REQUIRE(report.defects[0].area == 0.0);
        REQUIRE(report.defects[0].minAltitude == 0.0);
        REQUIRE(std::isinf(report.defects[0].aspectRatio));
    }
}
