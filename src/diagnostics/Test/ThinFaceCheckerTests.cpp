#include "diagnostics/ThinFaceChecker.hpp"
#include "core/Constants.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ReliableNormal.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

TEST_CASE("ThinFaceChecker reports zero defects on a clean cube mesh and computes exact metrics for regular faces",
          "[diagnostics][thin-faces]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createCubeMesh(10.0, ogun::Unit::Millimeter);
    const ogun::ThinFaceThresholds thresholds; // defaults: aspect limit 100, floors off

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

TEST_CASE("ThinFaceChecker computes the max angle and buckets thin faces by shape",
          "[diagnostics][thin-faces]") {
    const ogun::ThinFaceThresholds thresholds{1.0e-3, 1.0e-6, 1000.0};
    const Eigen::Vector3d o(0.0, 0.0, 0.0);

    const ogun::ThinFaceDefect right = ogun::ThinFaceChecker::evaluateTriangle(o, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0), thresholds);
    REQUIRE(right.maxAngleDeg == Catch::Approx(90.0));

    const ogun::ThinFaceDefect cap = ogun::ThinFaceChecker::evaluateTriangle(o, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(5, 1.0e-4, 0), thresholds);
    REQUIRE(cap.maxAngleDeg > 179.0);
    REQUIRE(ogun::classifyThinDefect(cap) == ogun::ThinFaceSubtype::Cap);

    const ogun::ThinFaceDefect needle = ogun::ThinFaceChecker::evaluateTriangle(o, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(10, 1.0e-4, 0), thresholds);
    REQUIRE(ogun::classifyThinDefect(needle) == ogun::ThinFaceSubtype::Needle);

    const ogun::ThinFaceDefect flat = ogun::ThinFaceChecker::evaluateTriangle(o, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(5, 0, 0), thresholds);
    REQUIRE(ogun::classifyThinDefect(flat) == ogun::ThinFaceSubtype::ZeroArea);

    const ogun::ThinFaceDefect tiny = ogun::ThinFaceChecker::evaluateTriangle(o, Eigen::Vector3d(1.0e-4, 0, 0), Eigen::Vector3d(0, 1.0e-4, 0), thresholds);
    REQUIRE(ogun::classifyThinDefect(tiny) == ogun::ThinFaceSubtype::Small);
    REQUIRE(std::string(ogun::thinFaceSubtypeLabel(ogun::ThinFaceSubtype::Cap)) == "Cap");
}

namespace {

ogun::ThinFaceDefect evaluate(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    return ogun::ThinFaceChecker::evaluateTriangle(v0, v1, v2); // default thresholds: 100 : 1, floors off
}

ogun::ThinFaceSubtype subtypeOf(const ogun::ThinFaceDefect& defect) {
    return ogun::classifyThinDefect(defect);
}

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

} // namespace

TEST_CASE("Thin faces are judged by aspect ratio", "[diagnostics][thin-faces]") {
    const Eigen::Vector3d o(0.0, 0.0, 0.0);
    const Eigen::Vector3d x(1.0, 0.0, 0.0);

    // Equilateral: aspect 2 / sqrt(3)
    const ogun::ThinFaceDefect equilateral = evaluate(o, x, Eigen::Vector3d(0.5, std::sqrt(3.0) / 2.0, 0.0));
    CHECK_FALSE(equilateral.isDefective());

    // A needle just under the limit is an ordinary face
    const ogun::ThinFaceDefect needle99 = evaluate(o, x, Eigen::Vector3d(1.0, 1.0 / 99.0, 0.0));
    CHECK(needle99.aspectRatio == Catch::Approx(99.0).margin(0.05));
    CHECK_FALSE(needle99.isDefective());

    // Aspect 150 with a short edge under 1 %: a Needle
    const ogun::ThinFaceDefect needle150 = evaluate(o, x, Eigen::Vector3d(1.0, 1.0 / 150.0, 0.0));
    CHECK(needle150.aspectRatio == Catch::Approx(150.0).margin(0.05));
    CHECK(needle150.shortestEdge / needle150.longestEdge < 0.01);
    CHECK(needle150.isDefective());
    CHECK_FALSE(needle150.isZeroArea());
    CHECK(subtypeOf(needle150) == ogun::ThinFaceSubtype::Needle);

    // A 179 degree cap
    const double capHeight = 0.5 * std::tan(0.5 * ogun::kDegToRad);
    const ogun::ThinFaceDefect cap = evaluate(o, x, Eigen::Vector3d(0.5, capHeight, 0.0));
    CHECK(cap.maxAngleDeg == Catch::Approx(179.0).margin(1e-9));
    CHECK(cap.isDefective());
    CHECK(subtypeOf(cap) == ogun::ThinFaceSubtype::Cap);

    // Aspect 110, short edge 5.08 %, largest angle 169.2 degrees: flagged, neither Needle nor Cap
    const ogun::ThinFaceDefect small = evaluate(o, x, Eigen::Vector3d(0.05, 1.0 / 110.0, 0.0));
    CHECK(small.aspectRatio == Catch::Approx(110.0).margin(1e-9));
    CHECK(small.shortestEdge / small.longestEdge == Catch::Approx(0.0508).margin(1e-4));
    CHECK(small.maxAngleDeg == Catch::Approx(169.15).margin(0.01)); // under the 170 degree Cap limit
    CHECK(small.isDefective());
    CHECK_FALSE(small.belowMinAltitude);
    CHECK_FALSE(small.belowMinArea);
    CHECK(subtypeOf(small) == ogun::ThinFaceSubtype::Small);
}

TEST_CASE("Thin-face flags are scale invariant", "[diagnostics][thin-faces]") {
    const ogun::Mesh sample = ogun::SampleMeshes::createSampleDefectiveMesh();
    const ogun::ThinFaceReport reference = ogun::ThinFaceChecker::check(sample);
    // The well-shaped microscopic face is not stretched, so only the zero-area and needle faces are flagged
    REQUIRE(reference.faceIndices ==
            std::vector<Eigen::Index>{ogun::SampleMeshes::kZeroAreaFaceIndex, ogun::SampleMeshes::kNeedleSliverFaceIndex});

    for (const double scale : {1.0e-6, 1.0e6}) {
        CAPTURE(scale);
        ogun::Mesh scaled = sample;
        scaled.V *= scale;
        const ogun::ThinFaceReport report = ogun::ThinFaceChecker::check(scaled);
        REQUIRE(report.faceIndices == reference.faceIndices);
        for (std::size_t i = 0; i < report.defects.size(); ++i) {
            CHECK(subtypeOf(report.defects[i]) == subtypeOf(reference.defects[i]));
        }
    }
}

TEST_CASE("The reliable-normal floor", "[diagnostics][thin-faces]") {
    // Apex 1 ulp off its base line at coordinate 1e6: h_min passes 1e-10 x l_max but not 2^-40 x 1e6
    const double big = 1.0e6;
    const double ulp = std::nextafter(big, 2.0 * big) - big;
    const Eigen::Vector3d a(big, big, 0.0);
    const Eigen::Vector3d b(big + 1.0, big, 0.0);
    const Eigen::Vector3d c(big + 1.0, big + ulp, 0.0);
    CHECK_FALSE(ogun::hasReliableNormal(a, b, c));
    const ogun::ThinFaceDefect far = evaluate(a, b, c);
    CHECK(far.minAltitude > ogun::kReliableNormalRatio * far.longestEdge);
    CHECK(far.isDefective());
    CHECK(far.isZeroArea());
    CHECK(subtypeOf(far) == ogun::ThinFaceSubtype::ZeroArea);

    // The same shape 1e-6 across near the origin is well represented: a Needle
    const double size = 1.0e-6;
    const Eigen::Vector3d p(0.0, 0.0, 0.0);
    const Eigen::Vector3d q(size, 0.0, 0.0);
    const Eigen::Vector3d r(size, size * ulp, 0.0);
    CHECK(ogun::hasReliableNormal(p, q, r));
    const ogun::ThinFaceDefect near = evaluate(p, q, r);
    CHECK(near.isDefective());
    CHECK_FALSE(near.isZeroArea());
    CHECK(subtypeOf(near) == ogun::ThinFaceSubtype::Needle);

    // A NaN vertex has no normal
    const Eigen::Vector3d nan(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    CHECK_FALSE(ogun::hasReliableNormal(p, q, nan));
    const ogun::ThinFaceDefect notFinite = evaluate(p, q, nan);
    CHECK(notFinite.isDefective());
    CHECK(subtypeOf(notFinite) == ogun::ThinFaceSubtype::ZeroArea);

    // A repeated index is degenerate whatever the coordinates
    ogun::Mesh mesh;
    mesh.V.resize(3, 3);
    mesh.V << 0.0, 0.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0;
    mesh.F.resize(1, 3);
    mesh.F << 0, 1, 1;
    const ogun::ThinFaceDefect repeated = ogun::ThinFaceChecker::evaluateFace(mesh, 0);
    CHECK(repeated.isDefective());
    CHECK(subtypeOf(repeated) == ogun::ThinFaceSubtype::ZeroArea);
}

TEST_CASE("Advanced absolute floors still apply when set", "[diagnostics][thin-faces][thresholds]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh();
    const Eigen::Index micro = ogun::SampleMeshes::kMicroscopicFaceIndex;
    REQUIRE(ogun::ThinFaceChecker::check(mesh).findDefectForFace(micro) == nullptr);

    ogun::ThinFaceThresholds heightFloor;
    heightFloor.minAltitude = 1.0e-4; // the microscopic face's h_min is ~7.07e-5
    const ogun::ThinFaceReport byHeight = ogun::ThinFaceChecker::check(mesh, heightFloor);
    REQUIRE(byHeight.numDefects() == 3);
    const ogun::ThinFaceDefect* low = byHeight.findDefectForFace(micro);
    REQUIRE(low != nullptr);
    CHECK(low->belowMinAltitude);
    CHECK_FALSE(low->aboveMaxAspectRatio);
    CHECK(subtypeOf(*low) == ogun::ThinFaceSubtype::Small);

    ogun::ThinFaceThresholds areaFloor;
    areaFloor.minArea = 1.0e-8; // its area is 5e-9
    const ogun::ThinFaceReport byArea = ogun::ThinFaceChecker::check(mesh, areaFloor);
    REQUIRE(byArea.findDefectForFace(micro) != nullptr);
    CHECK(byArea.findDefectForFace(micro)->belowMinArea);
}

TEST_CASE("Bunny and Turkey thin faces at the new defaults", "[diagnostics][thin-faces][.slow]") {
    const ogun::Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    REQUIRE(bunny.numFaces() == 69451);
    const ogun::ThinFaceReport bunnyThin = ogun::ThinFaceChecker::check(bunny);
    CHECK(bunnyThin.numDefects() == 1);
    for (const ogun::ThinFaceDefect& defect : bunnyThin.defects) {
        CHECK(subtypeOf(defect) == ogun::ThinFaceSubtype::Cap);
    }
    CHECK(ogun::runCategoryCheck(ogun::DefectCategory::Degenerate, bunny, ogun::CheckParams{}).empty());

    const ogun::Mesh turkey = ogun::loadStl(examplePath("Turkey.stl"));
    REQUIRE(turkey.numFaces() == 258832);
    const ogun::ThinFaceReport turkeyThin = ogun::ThinFaceChecker::check(turkey);
    CHECK(turkeyThin.numDefects() == 78);
    for (const ogun::ThinFaceDefect& defect : turkeyThin.defects) {
        CHECK(subtypeOf(defect) == ogun::ThinFaceSubtype::Cap);
    }
    CHECK(ogun::runCategoryCheck(ogun::DefectCategory::Degenerate, turkey, ogun::CheckParams{}).empty());
}
