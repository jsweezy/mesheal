#include "mesh/Mesh.hpp"
#include "mesh/MeshTransform.hpp"

#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>

namespace {

constexpr double kTol = 1.0e-12;

mesheal::Mesh makeTetrahedron() {
    mesheal::Mesh mesh;
    mesh.V.resize(4, 3);
    mesh.V << 1.0, 2.0, 3.0,
              11.0, 2.0, 3.0,
              1.0, 22.0, 3.0,
              1.0, 2.0, 33.0;
    mesh.F.resize(4, 3);
    mesh.F << 0, 2, 1,
              0, 1, 3,
              0, 3, 2,
              1, 2, 3;
    mesh.recomputeNormals();
    return mesh;
}

double signedVolume(const mesheal::Mesh& mesh) {
    double volume = 0.0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0)).transpose();
        const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1)).transpose();
        const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2)).transpose();
        volume += a.dot(b.cross(c)) / 6.0;
    }
    return volume;
}

mesheal::MeshTransform rotation(double x, double y, double z) {
    return mesheal::MeshTransform{Eigen::Vector3d::Zero(), Eigen::Vector3d(x, y, z)};
}

} // namespace

TEST_CASE("MeshTransform translation moves every vertex and keeps the faces", "[mesh][transform]") {
    mesheal::Mesh mesh = makeTetrahedron();
    const mesheal::Mesh before = mesh;
    mesheal::transformMesh(mesh, Eigen::Vector3d(-1.0, 0.5, 10.0), Eigen::Vector3d::Zero());
    REQUIRE(mesh.F == before.F);
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        REQUIRE((mesh.V.row(v) - before.V.row(v) - Eigen::RowVector3d(-1.0, 0.5, 10.0)).norm() < kTol);
    }
    REQUIRE(mesh.N.isApprox(before.N, kTol));
}

TEST_CASE("MeshTransform rotates 90 degrees about each world axis exactly", "[mesh][transform]") {
    const Eigen::Vector3d p(1.0, 2.0, 3.0);
    // Right-handed: +90 about X takes Y to Z, about Y takes Z to X, about Z takes X to Y
    REQUIRE(mesheal::transformPoint(rotation(90.0, 0.0, 0.0), p) == Eigen::Vector3d(1.0, -3.0, 2.0));
    REQUIRE(mesheal::transformPoint(rotation(0.0, 90.0, 0.0), p) == Eigen::Vector3d(3.0, 2.0, -1.0));
    REQUIRE(mesheal::transformPoint(rotation(0.0, 0.0, 90.0), p) == Eigen::Vector3d(-2.0, 1.0, 3.0));
    REQUIRE(mesheal::transformPoint(rotation(-90.0, 0.0, 0.0), p) == Eigen::Vector3d(1.0, 3.0, -2.0));
    REQUIRE(mesheal::transformPoint(rotation(180.0, 0.0, 0.0), p) == Eigen::Vector3d(1.0, -2.0, -3.0));
    REQUIRE(mesheal::transformPoint(rotation(0.0, 0.0, 450.0), p) == Eigen::Vector3d(-2.0, 1.0, 3.0));

    // A general angle matches Eigen's axis-angle rotation
    const Eigen::Vector3d q = mesheal::transformPoint(rotation(0.0, 0.0, 30.0), p);
    const Eigen::Vector3d expected = Eigen::AngleAxisd(30.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitZ()) * p;
    REQUIRE((q - expected).norm() < kTol);
}

TEST_CASE("MeshTransform composes rotations as Rz * Ry * Rx", "[mesh][transform]") {
    const Eigen::Vector3d p(1.0, 2.0, 3.0);
    const Eigen::Vector3d xThenYThenZ = mesheal::transformPoint(
        rotation(0.0, 0.0, 90.0), mesheal::transformPoint(rotation(0.0, 90.0, 0.0), mesheal::transformPoint(rotation(90.0, 0.0, 0.0), p)));
    REQUIRE(mesheal::transformPoint(rotation(90.0, 90.0, 90.0), p) == xThenYThenZ);
    // Another order gives another point, so the order matters
    const Eigen::Vector3d zThenYThenX = mesheal::transformPoint(
        rotation(90.0, 0.0, 0.0), mesheal::transformPoint(rotation(0.0, 90.0, 0.0), mesheal::transformPoint(rotation(0.0, 0.0, 90.0), p)));
    REQUIRE((xThenYThenZ - zThenYThenX).norm() > 1.0);

    const Eigen::Vector3d angles(20.0, -35.0, 70.0);
    const Eigen::Matrix3d expected = (Eigen::AngleAxisd(angles.z() * EIGEN_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(angles.y() * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(angles.x() * EIGEN_PI / 180.0, Eigen::Vector3d::UnitX()))
                                         .toRotationMatrix();
    REQUIRE(mesheal::rotationMatrixXYZ(angles).isApprox(expected, kTol));
}

TEST_CASE("MeshTransform translates before it rotates", "[mesh][transform]") {
    const mesheal::MeshTransform transform{Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 90.0)};
    const Eigen::Vector3d p(1.0, 2.0, 3.0);
    // R (p + t): (11, 2, 3) turned about Z
    REQUIRE(mesheal::transformPoint(transform, p) == Eigen::Vector3d(-2.0, 11.0, 3.0));
    // Rotate-then-translate would give R p + t = (-2, 1, 3) + (10, 0, 0)
    const Eigen::Vector3d rotateThenTranslate = mesheal::rotationMatrixXYZ(transform.rotationDeg) * p + transform.translation;
    REQUIRE(rotateThenTranslate == Eigen::Vector3d(8.0, 1.0, 3.0));
    REQUIRE(mesheal::transformPoint(transform, p) != rotateThenTranslate);

    mesheal::Mesh mesh = makeTetrahedron();
    const mesheal::Mesh before = mesh;
    mesheal::transformMesh(mesh, transform);
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        const Eigen::Vector3d expected = mesheal::transformPoint(transform, before.V.row(v).transpose());
        REQUIRE((mesh.V.row(v).transpose() - expected).norm() < kTol);
    }
}

TEST_CASE("MeshTransform identity is a no-op", "[mesh][transform]") {
    REQUIRE(mesheal::MeshTransform{}.isIdentity());
    REQUIRE(rotation(360.0, -720.0, 0.0).isIdentity());
    REQUIRE_FALSE(rotation(0.0, 0.0, 1.0).isIdentity());
    REQUIRE_FALSE((mesheal::MeshTransform{Eigen::Vector3d(0.0, 1.0e-9, 0.0), Eigen::Vector3d::Zero()}).isIdentity());

    mesheal::Mesh mesh = makeTetrahedron();
    const mesheal::Mesh before = mesh;
    mesheal::transformMesh(mesh, mesheal::MeshTransform{});
    REQUIRE(mesh.V == before.V);
    REQUIRE(mesh.F == before.F);
}

TEST_CASE("MeshTransform rotation keeps the winding and the signed volume", "[mesh][transform]") {
    mesheal::Mesh mesh = makeTetrahedron();
    const double volume = signedVolume(mesh);
    REQUIRE(volume != 0.0);
    const mesheal::Mesh before = mesh;
    mesheal::transformMesh(mesh, mesheal::MeshTransform{Eigen::Vector3d(-4.0, 7.0, 1.5), Eigen::Vector3d(33.0, -71.0, 128.0)});
    REQUIRE(mesh.F == before.F);
    REQUIRE(signedVolume(mesh) == Catch::Approx(volume).epsilon(1.0e-12));
    // Normals turn with the mesh
    const Eigen::Matrix3d r = mesheal::rotationMatrixXYZ(Eigen::Vector3d(33.0, -71.0, 128.0));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        REQUIRE((mesh.N.row(f).transpose() - r * before.N.row(f).transpose()).norm() < 1.0e-12);
    }
}

TEST_CASE("facesCentroid is area weighted", "[mesh][transform]") {
    mesheal::Mesh mesh;
    mesh.V.resize(6, 3);
    mesh.V << 0.0, 0.0, 0.0,
              3.0, 0.0, 0.0,
              0.0, 3.0, 0.0,
              10.0, 0.0, 0.0,
              16.0, 0.0, 0.0,
              10.0, 6.0, 0.0;
    mesh.F.resize(2, 3);
    mesh.F << 0, 1, 2,
              3, 4, 5;

    // One facet: its triangle centroid
    const std::array<Eigen::Index, 1> first = {0};
    REQUIRE(mesheal::facesCentroid(mesh, first)->isApprox(Eigen::Vector3d(1.0, 1.0, 0.0), kTol));

    // Areas 4.5 and 18: the centroids (1, 1) and (12, 2) weighted 1 : 4
    const std::array<Eigen::Index, 2> both = {0, 1};
    REQUIRE(mesheal::facesCentroid(mesh, both)->isApprox(Eigen::Vector3d((1.0 + 4.0 * 12.0) / 5.0, (1.0 + 4.0 * 2.0) / 5.0, 0.0), kTol));

    REQUIRE_FALSE(mesheal::facesCentroid(mesh, std::span<const Eigen::Index>{}).has_value());
    const std::array<Eigen::Index, 1> outOfRange = {7};
    REQUIRE_FALSE(mesheal::facesCentroid(mesh, outOfRange).has_value());

    // Zero total area: the plain average of the triangle centroids
    mesheal::Mesh flat = mesh;
    flat.V.row(2) << 1.5, 0.0, 0.0;
    flat.V.row(5) << 13.0, 0.0, 0.0;
    REQUIRE(mesheal::facesCentroid(flat, both)->isApprox(Eigen::Vector3d((1.5 + 13.0) / 2.0, 0.0, 0.0), kTol));
}
