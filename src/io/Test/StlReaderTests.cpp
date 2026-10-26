#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "io/StlWriter.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

double computeTriangleArea(const ogun::Mesh& mesh, Eigen::Index faceIdx) {
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(faceIdx, 0)).transpose();
    const Eigen::Vector3d v1 = mesh.V.row(mesh.F(faceIdx, 1)).transpose();
    const Eigen::Vector3d v2 = mesh.V.row(mesh.F(faceIdx, 2)).transpose();
    return 0.5 * (v1 - v0).cross(v2 - v0).norm();
}

double computeTriangleMinAltitude(const ogun::Mesh& mesh, Eigen::Index faceIdx) {
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(faceIdx, 0)).transpose();
    const Eigen::Vector3d v1 = mesh.V.row(mesh.F(faceIdx, 1)).transpose();
    const Eigen::Vector3d v2 = mesh.V.row(mesh.F(faceIdx, 2)).transpose();
    const double e0 = (v1 - v0).norm();
    const double e1 = (v2 - v1).norm();
    const double e2 = (v0 - v2).norm();
    const double longestEdge = std::max({e0, e1, e2});
    if (longestEdge <= 0.0) {
        return 0.0;
    }
    const double area = 0.5 * (v1 - v0).cross(v2 - v0).norm();
    return (2.0 * area) / longestEdge;
}

} // namespace

TEST_CASE("Loading synthetic cube STL (ASCII and binary) welds 36 soup vertices into 8 vertices and 12 triangles",
          "[io][stl]") {
    SECTION("ASCII STL stream and file loading") {
        const std::string asciiStl = ogun::SampleMeshes::createCubeAsciiStl(10.0);
        REQUIRE_FALSE(asciiStl.empty());

        std::istringstream stream(asciiStl);
        const ogun::Mesh mesh = ogun::loadStl(stream, ogun::Unit::Millimeter);

        REQUIRE(mesh.fileUnit == ogun::Unit::Millimeter);
        REQUIRE(mesh.numVertices() == 8);
        REQUIRE(mesh.numFaces() == 12);
        REQUIRE(mesh.N.rows() == 12);
        REQUIRE(mesh.N.cols() == 3);

        const ogun::BoundingBox bbox = mesh.boundingBox();
        REQUIRE(bbox.min.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
        REQUIRE(bbox.max.isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-12));

        // Verify every triangle has 3 distinct valid vertex indices and unit normal
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            const int i0 = mesh.F(f, 0);
            const int i1 = mesh.F(f, 1);
            const int i2 = mesh.F(f, 2);
            REQUIRE(i0 >= 0);
            REQUIRE(i0 < 8);
            REQUIRE(i1 >= 0);
            REQUIRE(i1 < 8);
            REQUIRE(i2 >= 0);
            REQUIRE(i2 < 8);
            REQUIRE(i0 != i1);
            REQUIRE(i1 != i2);
            REQUIRE(i2 != i0);
            REQUIRE(mesh.N.row(f).norm() == Catch::Approx(1.0).epsilon(1e-12));
        }

        // Also test loading from a file on disk
        const auto tempDir = std::filesystem::temp_directory_path();
        const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        const auto tempAsciiPath = tempDir / ("ogun_test_cube_ascii_" + uniqueSuffix + ".stl");
        {
            std::ofstream out(tempAsciiPath, std::ios::binary);
            out << asciiStl;
        }
        const ogun::Mesh diskMesh = ogun::loadStl(tempAsciiPath, ogun::Unit::Centimeter);
        std::filesystem::remove(tempAsciiPath);

        REQUIRE(diskMesh.fileUnit == ogun::Unit::Centimeter);
        REQUIRE(diskMesh.numVertices() == 8);
        REQUIRE(diskMesh.numFaces() == 12);
    }

    SECTION("Binary STL file loading") {
        const std::vector<std::uint8_t> binaryStl = ogun::SampleMeshes::createCubeBinaryStl(10.0);
        // 80-byte header + 4-byte count + 12 * 50 bytes = 684 bytes
        REQUIRE(binaryStl.size() == 684);

        const auto tempDir = std::filesystem::temp_directory_path();
        const auto uniqueSuffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        const auto tempBinPath = tempDir / ("ogun_test_cube_bin_" + uniqueSuffix + ".stl");
        {
            std::ofstream out(tempBinPath, std::ios::binary);
            out.write(reinterpret_cast<const char*>(binaryStl.data()),
                      static_cast<std::streamsize>(binaryStl.size()));
        }

        ogun::Mesh binMesh;
        std::string err;
        const bool ok = ogun::loadStl(tempBinPath, binMesh, ogun::Unit::Millimeter, ogun::ToleranceConfig{}, &err);
        std::filesystem::remove(tempBinPath);

        REQUIRE(ok);
        REQUIRE(err.empty());
        REQUIRE(binMesh.numVertices() == 8);
        REQUIRE(binMesh.numFaces() == 12);
        REQUIRE(binMesh.boundingBox().extents().isApprox(Eigen::Vector3d(10.0, 10.0, 10.0), 1e-6));
    }
}

TEST_CASE("Multi-scale local coordinate-relative vertex welding",
          "[io][welding][tolerance]") {
    const ogun::ToleranceConfig tolConfig{1e-9, 1e-9};

    SECTION("Near the origin in cm: 5e-10 cm separation merges, while 1e-6 cm separation stays distinct") {
        // Two triangles near the origin sharing a corner at (0, 0, 0) vs (5e-10, 0, 0),
        // and another corner separated by 1e-6 cm: (1, 0, 0) vs (1 + 1e-6, 0, 0).
        Eigen::MatrixXd soupV(6, 3);
        soupV << 0.0,        0.0, 0.0,
                 1.0,        0.0, 0.0,
                 0.0,        1.0, 0.0,
                 5.0e-10,    0.0, 0.0,   // Within 1e-9 cm floor of (0,0,0) -> MUST merge
                 1.0 + 1e-6, 0.0, 0.0,   // Outside 1e-9 cm floor & rel_tol of (1,0,0) -> MUST NOT merge
                 0.0,       -1.0, 0.0;

        const ogun::Mesh welded = ogun::weldVerticesLocal(soupV, ogun::Unit::Centimeter, tolConfig);

        REQUIRE(welded.numFaces() == 2);
        // 6 raw vertices -> 1 pair merged -> 5 unique vertices
        REQUIRE(welded.numVertices() == 5);
        // Corner 0 of face 0 and corner 0 of face 1 merged
        REQUIRE(welded.F(0, 0) == welded.F(1, 0));
        // Corner 1 of face 0 and corner 1 of face 1 remained distinct
        REQUIRE(welded.F(0, 1) != welded.F(1, 1));
    }

    SECTION("Far from the origin (x = 1e9 cm): vertices within relative tolerance merge properly") {
        // At x = 1e9 cm, relative tolerance is 1e-9 * 1e9 = 1.0 cm.
        const double farX = 1.0e9;
        Eigen::MatrixXd soupV(6, 3);
        soupV << farX,         0.0, 0.0,
                 farX + 100.0, 0.0, 0.0,
                 farX,       100.0, 0.0,
                 farX + 0.5,   0.0, 0.0,   // Separated by 0.5 cm <= 1.0 cm rel tol -> MUST merge
                 farX + 102.0, 0.0, 0.0,   // Separated by 2.0 cm > ~1.0 cm rel tol -> MUST NOT merge
                 farX,      -100.0, 0.0;

        const ogun::Mesh welded = ogun::weldVerticesLocal(soupV, ogun::Unit::Centimeter, tolConfig);

        REQUIRE(welded.numFaces() == 2);
        REQUIRE(welded.numVertices() == 5);
        REQUIRE(welded.F(0, 0) == welded.F(1, 0));
        REQUIRE(welded.F(0, 1) != welded.F(1, 1));
    }

    SECTION("Simultaneous multi-scale mesh: micro-features at origin coexist with macro-features at 1e9 cm") {
        const double farX = 1.0e9;
        Eigen::MatrixXd soupV(12, 3);
        // Pair of micro-triangles near origin with 1e-6 cm feature size and 5e-10 cm weld gap
        soupV.row(0)  = Eigen::Vector3d(0.0,     0.0,  0.0);
        soupV.row(1)  = Eigen::Vector3d(1.0e-6,  0.0,  0.0);
        soupV.row(2)  = Eigen::Vector3d(0.0,     1e-6, 0.0);
        soupV.row(3)  = Eigen::Vector3d(5.0e-10, 0.0,  0.0);
        soupV.row(4)  = Eigen::Vector3d(1.0e-6,  0.0,  0.0);
        soupV.row(5)  = Eigen::Vector3d(0.0,    -1e-6, 0.0);

        // Pair of macro-triangles at x = 1e9 cm with 0.4 cm weld gap
        soupV.row(6)  = Eigen::Vector3d(farX,         0.0, 0.0);
        soupV.row(7)  = Eigen::Vector3d(farX + 100.0, 0.0, 0.0);
        soupV.row(8)  = Eigen::Vector3d(farX,       100.0, 0.0);
        soupV.row(9)  = Eigen::Vector3d(farX + 0.4,   0.0, 0.0);
        soupV.row(10) = Eigen::Vector3d(farX + 100.0, 0.0, 0.0);
        soupV.row(11) = Eigen::Vector3d(farX,      -100.0, 0.0);

        const ogun::Mesh welded = ogun::weldVerticesLocal(soupV, ogun::Unit::Centimeter, tolConfig);

        // Micro pair welds into 4 vertices (without collapsing its 1e-6 cm edge!)
        // Macro pair welds into 4 vertices (merging the 0.4 cm gap at 1e9 cm!)
        REQUIRE(welded.numFaces() == 4);
        REQUIRE(welded.numVertices() == 8);
        REQUIRE(welded.F(0, 0) == welded.F(1, 0));
        REQUIRE(welded.F(0, 1) == welded.F(1, 1));
        REQUIRE(welded.F(0, 0) != welded.F(0, 1));
        REQUIRE(welded.F(2, 0) == welded.F(3, 0));
        REQUIRE(welded.F(2, 1) == welded.F(3, 1));
    }
}

TEST_CASE("Edge-Aware Guard preserves sliver triangles whose own vertices are closer than weld tolerance",
          "[io][welding][edge-aware]") {
    const ogun::ToleranceConfig tolConfig{1e-9, 1e-9};

    SECTION("Sliver triangle with sub-tolerance edge (5e-10 cm < 1e-9 cm) keeps 3 distinct vertex indices") {
        // Face 0: A sliver triangle whose first two vertices are separated by only 5e-10 cm (< 1e-9 cm floor).
        // Face 1: A normal neighboring triangle sharing vertex (0,0,0) and (0,1,0).
        // Face 2: A normal neighboring triangle sharing vertex (5e-10,0,0) and (0,1,0).
        Eigen::MatrixXd soupV(9, 3);
        // Face 0 (sub-tolerance sliver)
        soupV.row(0) = Eigen::Vector3d(0.0,     0.0, 0.0);
        soupV.row(1) = Eigen::Vector3d(5.0e-10, 0.0, 0.0);
        soupV.row(2) = Eigen::Vector3d(0.0,     1.0, 0.0);
        // Face 1 (shares (0,0,0) and (0,1,0))
        soupV.row(3) = Eigen::Vector3d(0.0,     0.0, 0.0);
        soupV.row(4) = Eigen::Vector3d(0.0,     1.0, 0.0);
        soupV.row(5) = Eigen::Vector3d(-1.0,    0.0, 0.0);
        // Face 2 (shares (5e-10,0,0) and (0,1,0))
        soupV.row(6) = Eigen::Vector3d(5.0e-10, 0.0, 0.0);
        soupV.row(7) = Eigen::Vector3d(1.0,     0.0, 0.0);
        soupV.row(8) = Eigen::Vector3d(0.0,     1.0, 0.0);

        const ogun::Mesh welded = ogun::weldVerticesLocal(soupV, ogun::Unit::Centimeter, tolConfig);

        REQUIRE(welded.numFaces() == 3);
        // All 3 vertex indices of the sliver triangle (Face 0) MUST remain distinct!
        REQUIRE(welded.F(0, 0) != welded.F(0, 1));
        REQUIRE(welded.F(0, 1) != welded.F(0, 2));
        REQUIRE(welded.F(0, 2) != welded.F(0, 0));

        // Neighboring triangles still weld properly to their respective endpoints
        REQUIRE(welded.F(1, 0) == welded.F(0, 0));
        REQUIRE(welded.F(1, 1) == welded.F(0, 2));
        REQUIRE(welded.F(2, 0) == welded.F(0, 1));
        REQUIRE(welded.F(2, 2) == welded.F(0, 2));
        REQUIRE(welded.numVertices() == 5);
    }

    SECTION("Degenerate triangle with identical corner coordinates on load still keeps 3 distinct vertex indices") {
        Eigen::MatrixXd soupV(3, 3);
        soupV.row(0) = Eigen::Vector3d(1.0, 2.0, 3.0);
        soupV.row(1) = Eigen::Vector3d(1.0, 2.0, 3.0); // Exact duplicate within same triangle
        soupV.row(2) = Eigen::Vector3d(4.0, 5.0, 6.0);

        const ogun::Mesh welded = ogun::weldVerticesLocal(soupV, ogun::Unit::Millimeter, tolConfig);

        REQUIRE(welded.numFaces() == 1);
        REQUIRE(welded.numVertices() == 3);
        REQUIRE(welded.F(0, 0) != welded.F(0, 1));
        REQUIRE(welded.F(0, 1) != welded.F(0, 2));
        REQUIRE(welded.F(0, 2) != welded.F(0, 0));
    }
}

TEST_CASE("SampleMeshes::createSampleDefectiveMesh generates a clean cube with known zero-area, microscopic, and sliver triangles",
          "[io][sample]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh(ogun::Unit::Millimeter);

    REQUIRE(mesh.fileUnit == ogun::Unit::Millimeter);
    REQUIRE(mesh.numFaces() == ogun::SampleMeshes::kDefectiveSampleFaceCount);
    REQUIRE(mesh.numVertices() > ogun::SampleMeshes::kCleanCubeVertexCount);
    REQUIRE(mesh.N.rows() == mesh.numFaces());

    // Every triangle in the sample mesh must have 3 distinct valid vertex indices
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const int i0 = mesh.F(f, 0);
        const int i1 = mesh.F(f, 1);
        const int i2 = mesh.F(f, 2);
        REQUIRE(i0 >= 0);
        REQUIRE(i0 < mesh.numVertices());
        REQUIRE(i1 >= 0);
        REQUIRE(i1 < mesh.numVertices());
        REQUIRE(i2 >= 0);
        REQUIRE(i2 < mesh.numVertices());
        REQUIRE(i0 != i1);
        REQUIRE(i1 != i2);
        REQUIRE(i2 != i0);
    }

    // First 12 faces are the regular 10x10x10 mm cube faces (each area = 50 mm^2, h_min = 5*sqrt(2) ~= 7.071 mm)
    for (Eigen::Index f = 0; f < ogun::SampleMeshes::kCleanCubeFaceCount; ++f) {
        REQUIRE(computeTriangleArea(mesh, f) == Catch::Approx(50.0).epsilon(1e-9));
        REQUIRE(computeTriangleMinAltitude(mesh, f) == Catch::Approx(50.0 / std::sqrt(200.0) * 2.0).epsilon(1e-9));
    }

    // 1. Collinear zero-area triangle (Face 12)
    const double zeroArea = computeTriangleArea(mesh, ogun::SampleMeshes::kZeroAreaFaceIndex);
    const double zeroAlt = computeTriangleMinAltitude(mesh, ogun::SampleMeshes::kZeroAreaFaceIndex);
    REQUIRE(zeroArea == Catch::Approx(0.0).margin(1e-15));
    REQUIRE(zeroAlt == Catch::Approx(0.0).margin(1e-15));

    // 2. Microscopic-area triangle (Face 13)
    const double microArea = computeTriangleArea(mesh, ogun::SampleMeshes::kMicroscopicFaceIndex);
    const double microAlt = computeTriangleMinAltitude(mesh, ogun::SampleMeshes::kMicroscopicFaceIndex);
    REQUIRE(microArea > 0.0);
    REQUIRE(microArea < 1e-6);
    REQUIRE(microAlt > 0.0);
    REQUIRE(microAlt < 1e-3);

    // 3. Needle / sliver triangle (Face 14)
    const double sliverArea = computeTriangleArea(mesh, ogun::SampleMeshes::kNeedleSliverFaceIndex);
    const double sliverAlt = computeTriangleMinAltitude(mesh, ogun::SampleMeshes::kNeedleSliverFaceIndex);
    REQUIRE(sliverArea == Catch::Approx(1e-3).epsilon(1e-9));
    REQUIRE(sliverAlt == Catch::Approx(2e-4).epsilon(1e-9));
}

TEST_CASE("Binary STL export round-trips through loadStl within float32 precision", "[io][stlwriter]") {
    const ogun::Mesh original = ogun::SampleMeshes::createSampleDefectiveMesh();
    const auto path = std::filesystem::temp_directory_path() / "ogun_writer_binary.stl";

    std::string error;
    REQUIRE(ogun::saveStl(path, original, ogun::StlFormat::Binary, &error));
    REQUIRE(error.empty());
    CHECK(std::filesystem::file_size(path) == 84 + 50 * static_cast<std::uintmax_t>(original.numFaces()));

    const ogun::Mesh loaded = ogun::loadStl(path);
    std::filesystem::remove(path);

    CHECK(loaded.numFaces() == original.numFaces());
    CHECK(loaded.numVertices() == original.numVertices());
    const auto a = original.boundingBox();
    const auto b = loaded.boundingBox();
    CHECK((a.min - b.min).norm() < 1e-4);
    CHECK((a.max - b.max).norm() < 1e-4);
}

TEST_CASE("ASCII STL export round-trips with double precision", "[io][stlwriter]") {
    ogun::Mesh original = ogun::SampleMeshes::createSampleDefectiveMesh();
    original.V(0, 0) += 0.123456789012345;
    original.recomputeNormals();
    const auto path = std::filesystem::temp_directory_path() / "ogun_writer_ascii.stl";

    REQUIRE(ogun::saveStl(path, original, ogun::StlFormat::Ascii));
    const ogun::Mesh loaded = ogun::loadStl(path);
    std::filesystem::remove(path);

    REQUIRE(loaded.numFaces() == original.numFaces());
    REQUIRE(loaded.numVertices() == original.numVertices());
    const auto a = original.boundingBox();
    const auto b = loaded.boundingBox();
    CHECK((a.min - b.min).norm() < 1e-12);
    CHECK((a.max - b.max).norm() < 1e-12);
}

TEST_CASE("saveStl reports errors for empty meshes and invalid paths", "[io][stlwriter]") {
    std::string error;
    const ogun::Mesh empty;
    CHECK_FALSE(ogun::saveStl(std::filesystem::temp_directory_path() / "ogun_empty.stl", empty,
                              ogun::StlFormat::Binary, &error));
    CHECK_FALSE(error.empty());

    error.clear();
    const ogun::Mesh mesh = ogun::SampleMeshes::createSampleDefectiveMesh();
    CHECK_FALSE(ogun::saveStl("/nonexistent_ogun_dir/x/out.stl", mesh, ogun::StlFormat::Ascii, &error));
    CHECK_FALSE(error.empty());
}

namespace {

std::size_t defectCount(ogun::DefectCategory category, const ogun::Mesh& mesh) {
    return ogun::runCategoryCheck(category, mesh, ogun::CheckParams{}).size();
}

/// Each face's corners sit at the same positions in both meshes (same face order).
bool sameCornerPositions(const ogun::Mesh& a, const ogun::Mesh& b) {
    if (a.numFaces() != b.numFaces()) {
        return false;
    }
    for (Eigen::Index f = 0; f < a.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (!a.V.row(a.F(f, c)).isApprox(b.V.row(b.F(f, c)), 1e-12) &&
                !(a.V.row(a.F(f, c)).isZero() && b.V.row(b.F(f, c)).isZero())) {
                return false;
            }
        }
    }
    return true;
}

ogun::Mesh loadFromAsciiStl(const ogun::Mesh& mesh) {
    std::stringstream stl;
    ogun::writeStlAscii(stl, mesh);
    return ogun::loadStl(stl);
}

} // namespace

TEST_CASE("Loading an STL keeps solids that only touch apart", "[io][stl][non-manifold]") {
    // Two cubes touching along an edge, one touching at a corner, and a fin
    const ogun::Mesh welded = ogun::SampleMeshes::createTouchingCubesMesh();
    REQUIRE(defectCount(ogun::DefectCategory::NonManifoldEdges, welded) == 3);
    REQUIRE(defectCount(ogun::DefectCategory::NonManifoldVertices, welded) == 1);

    const ogun::Mesh loaded = loadFromAsciiStl(welded);
    REQUIRE(sameCornerPositions(welded, loaded));
    REQUIRE(loaded.numVertices() == welded.numVertices() + 4); // three seam vertices and the corner
    REQUIRE(defectCount(ogun::DefectCategory::NonManifoldVertices, loaded) == 0);
    const ogun::CategoryResult edges = ogun::checkNonManifoldEdges(loaded);
    REQUIRE(edges.size() == 1); // the fin is a real defect and stays
    REQUIRE(edges.elements[0].subtype == 1);
    REQUIRE(defectCount(ogun::DefectCategory::OpenEdges, loaded) == 2);
    REQUIRE(ogun::checkShells(loaded).infoCount == 3);

    SECTION("a saved and reopened STL comes back the same") {
        std::stringstream binary;
        ogun::writeStlBinary(binary, loaded);
        const ogun::Mesh reloaded = ogun::loadStl(binary);
        REQUIRE(reloaded.F == loaded.F);
        REQUIRE(reloaded.numVertices() == loaded.numVertices());
        REQUIRE(defectCount(ogun::DefectCategory::NonManifoldVertices, reloaded) == 0);
        REQUIRE(defectCount(ogun::DefectCategory::NonManifoldEdges, reloaded) == 1);
    }
}

TEST_CASE("Separating touching contacts leaves a clean mesh unchanged", "[io][stl][non-manifold]") {
    const ogun::Mesh cube = ogun::SampleMeshes::createGridCubeMesh(4, 4.0);
    const ogun::Mesh separated = ogun::separateTouchingContacts(cube);
    REQUIRE(separated.V == cube.V);
    REQUIRE(separated.F == cube.F);

    const ogun::Mesh sample = ogun::SampleMeshes::createSampleDefectiveMesh();
    REQUIRE(ogun::separateTouchingContacts(sample).F == sample.F);
}

TEST_CASE("The Turkey example loads with no non-manifold edges or vertices", "[io][stl][non-manifold][turkey]") {
    const std::filesystem::path turkey =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / "Turkey.stl";
    REQUIRE(std::filesystem::exists(turkey));
    // Welded by position alone it has a 4-edge seam where the surface touches itself and 13 bow-ties
    const ogun::Mesh mesh = ogun::loadStl(turkey);
    REQUIRE(defectCount(ogun::DefectCategory::NonManifoldEdges, mesh) == 0);
    REQUIRE(defectCount(ogun::DefectCategory::NonManifoldVertices, mesh) == 0);
    REQUIRE(defectCount(ogun::DefectCategory::OpenEdges, mesh) == 0);
    REQUIRE(ogun::checkShells(mesh).infoCount == 28);
}
