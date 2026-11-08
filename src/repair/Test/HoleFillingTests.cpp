#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/CgalMesh.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace {

namespace PMP = CGAL::Polygon_mesh_processing;

/// Faces of `mesh` whose centroid fails `drop`, compacted.
mesheal::Mesh removeFaces(const mesheal::Mesh& mesh, const std::function<bool(const Eigen::Vector3d&)>& drop) {
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        if (!drop(c)) {
            keep.push_back(f);
        }
    }
    return mesheal::extractFaces(mesh, keep);
}



std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

mesheal::CgalSurfaceMesh toCgal(const mesheal::Mesh& mesh) {
    mesheal::CgalSurfaceMesh sm;
    REQUIRE(mesheal::toCgalSurfaceMesh(mesh, sm));
    return sm;
}

std::vector<mesheal::CgalHalfedgeIndex> borderCycles(const mesheal::CgalSurfaceMesh& sm) {
    std::vector<mesheal::CgalHalfedgeIndex> cycles;
    std::vector<bool> seen(sm.number_of_halfedges() + sm.number_of_removed_halfedges(), false);
    for (const mesheal::CgalHalfedgeIndex h : sm.halfedges()) {
        if (!sm.is_border(h) || seen[h.idx()]) {
            continue;
        }
        cycles.push_back(h);
        for (const mesheal::CgalHalfedgeIndex b : CGAL::halfedges_around_face(h, sm)) {
            seen[b.idx()] = true;
        }
    }
    return cycles;
}

struct Phases {
    int planar = 0;
    bool planarOk = false;
    int quadratic = 0;
    bool quadraticOk = false;
    int cubic = 0;
    long cubicSteps = 0;
    bool throwInCubic = false;
};

/// Records which phases of CGAL's hole filling ran. CGAL copies the visitor on some paths, so the
/// counts live behind a pointer.
struct PhaseVisitor : PMP::Hole_filling::Default_visitor {
    Phases* phases;
    explicit PhaseVisitor(Phases& p) : phases(&p) {}

    void start_planar_phase() const { ++phases->planar; }
    void end_planar_phase(bool ok) const { phases->planarOk = ok; }
    void start_quadratic_phase(std::size_t) const { ++phases->quadratic; }
    void end_quadratic_phase(bool ok) const { phases->quadraticOk = ok; }
    void start_cubic_phase(int) const { ++phases->cubic; }
    void cubic_step() const {
        ++phases->cubicSteps;
        if (phases->throwInCubic) {
            throw std::runtime_error("cancelled");
        }
    }
};

double maxPlaneDistance(const mesheal::Mesh& mesh, const std::vector<Eigen::Index>& faces, double z) {
    double worst = 0.0;
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            worst = std::max(worst, std::abs(mesh.V(mesh.F(f, c), 2) - z));
        }
    }
    return worst;
}

std::vector<Eigen::Index> newFaces(const mesheal::Mesh& before, const mesheal::Mesh& after) {
    return mesheal::faceDiff(before, after).added;
}


mesheal::Mesh flatHoleBox() {
    return removeFaces(mesheal::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });
}

mesheal::Mesh sphereCap() {
    return removeFaces(mesheal::SampleMeshes::createSphereMesh(24, 32, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
}

mesheal::Mesh openCylinder() {
    return removeFaces(mesheal::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0), [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
}

bool watertight(const mesheal::Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).empty() && mesheal::checkNonManifoldEdges(mesh).empty();
}

/// Largest distance between the two surfaces (both directions).
double surfaceDistance(const mesheal::Mesh& a, const mesheal::Mesh& b) {
    return std::max(mesheal::vertexDeviation(a, b).maxCoeff(), mesheal::vertexDeviation(b, a).maxCoeff());
}

} // namespace

// -----------------------------------------------------------------------------
// What CGAL's hole filling actually does (pins the assumptions the hole repair is built on)
// -----------------------------------------------------------------------------

TEST_CASE("CGAL triangulate_hole uses the 2D CDT only when asked", "[repair][holes][cgal]") {
    // A flat 2 x 2-cell square hole in the top of a grid cube
    const mesheal::Mesh holed = removeFaces(mesheal::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });

    // The CGAL 6.x docs say use_2d_constrained_delaunay_triangulation defaults to true, the code to false
    mesheal::CgalSurfaceMesh sm = toCgal(holed);
    Phases byDefault;
    PMP::triangulate_hole(sm, borderCycles(sm).front(), CGAL::parameters::visitor(PhaseVisitor(byDefault)));
    REQUIRE(byDefault.planar == 0);
    REQUIRE(byDefault.quadraticOk); // 3D Delaunay search space (2D for a flat hole: no start call)
    REQUIRE(byDefault.cubic == 0);

    mesheal::CgalSurfaceMesh sm2 = toCgal(holed);
    Phases withCdt;
    PMP::triangulate_hole(sm2, borderCycles(sm2).front(),
                          CGAL::parameters::visitor(PhaseVisitor(withCdt)).use_2d_constrained_delaunay_triangulation(true));
    REQUIRE(withCdt.planar == 1);
    REQUIRE(withCdt.planarOk);
    REQUIRE(withCdt.quadratic == 0);
}

TEST_CASE("CGAL hole filling only adds faces and vertices", "[repair][holes][cgal]") {
    const mesheal::Mesh sphere = mesheal::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const mesheal::Mesh holed = removeFaces(sphere, [](const Eigen::Vector3d& c) { return c.z() > 3.5; });
    const std::size_t holedFaces = static_cast<std::size_t>(holed.numFaces());

    for (int method = 0; method < 3; ++method) {
        mesheal::CgalSurfaceMesh sm = toCgal(holed);
        std::vector<mesheal::CgalFaceIndex> patch;
        std::vector<mesheal::CgalVertexIndex> added;
        const auto h = borderCycles(sm).front();
        if (method == 0) {
            PMP::triangulate_hole(sm, h, CGAL::parameters::face_output_iterator(std::back_inserter(patch)));
        } else if (method == 1) {
            PMP::triangulate_and_refine_hole(sm, h, CGAL::parameters::face_output_iterator(std::back_inserter(patch))
                                                        .vertex_output_iterator(std::back_inserter(added)));
        } else {
            const auto r = PMP::triangulate_refine_and_fair_hole(
                sm, h, CGAL::parameters::face_output_iterator(std::back_inserter(patch)).vertex_output_iterator(std::back_inserter(added)));
            REQUIRE(std::get<0>(r));
        }
        CAPTURE(method);
        REQUIRE(sm.is_valid());
        REQUIRE(borderCycles(sm).empty());
        REQUIRE(sm.number_of_faces() == holedFaces + patch.size());
        // Original vertices keep their index and position; new ones come after
        for (Eigen::Index v = 0; v < holed.numVertices(); ++v) {
            const auto p = sm.point(mesheal::CgalVertexIndex(static_cast<std::uint32_t>(v)));
            REQUIRE(Eigen::Vector3d(p.x(), p.y(), p.z()) == Eigen::Vector3d(holed.V.row(v).transpose()));
        }
        for (const mesheal::CgalVertexIndex v : added) {
            REQUIRE(static_cast<Eigen::Index>(v.idx()) >= holed.numVertices());
        }
        REQUIRE((method == 0) == added.empty());
        // Original faces keep their index and corners
        for (Eigen::Index f = 0; f < holed.numFaces(); ++f) {
            const mesheal::CgalFaceIndex cf(static_cast<std::uint32_t>(f));
            std::vector<int> corners;
            for (const auto v : CGAL::vertices_around_face(sm.halfedge(cf), sm)) {
                corners.push_back(static_cast<int>(v.idx()));
            }
            const std::vector<int> expected{holed.F(f, 0), holed.F(f, 1), holed.F(f, 2)};
            REQUIRE(std::is_permutation(corners.begin(), corners.end(), expected.begin()));
        }
    }
}

TEST_CASE("CGAL fill modes on a cylinder end cap and a sphere cap", "[repair][holes][cgal]") {
    // Missing planar end cap: the walls meet the cap at 90 degrees
    const mesheal::Mesh cylinder = mesheal::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0);
    const mesheal::Mesh open = removeFaces(cylinder, [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
    const auto fill = [](const mesheal::Mesh& mesh, int method) {
        mesheal::CgalSurfaceMesh sm = toCgal(mesh);
        const auto h = borderCycles(sm).front();
        const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true);
        if (method == 0) {
            PMP::triangulate_hole(sm, h, np);
        } else if (method == 1) {
            PMP::triangulate_and_refine_hole(sm, h, np);
        } else {
            PMP::triangulate_refine_and_fair_hole(sm, h, np);
        }
        return mesheal::fromCgalSurfaceMesh(sm, mesh.fileUnit);
    };
    const mesheal::Mesh flat = fill(open, 0);
    const mesheal::Mesh refined = fill(open, 1);
    const mesheal::Mesh faired = fill(open, 2);
    REQUIRE(maxPlaneDistance(flat, newFaces(open, flat), 10.0) < 1e-12);
    REQUIRE(maxPlaneDistance(refined, newFaces(open, refined), 10.0) < 1e-9);
    // Fairing keeps tangent continuity with the walls, so the cap bulges into a dome
    REQUIRE(maxPlaneDistance(faired, newFaces(open, faired), 10.0) > 1.0);
    REQUIRE(newFaces(open, refined).size() > newFaces(open, flat).size());

    // Missing sphere cap (its rim is a planar circle too): only fairing restores the curvature
    const mesheal::Mesh sphere = mesheal::SampleMeshes::createSphereMesh(24, 32, 5.0);
    const mesheal::Mesh holed = removeFaces(sphere, [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
    const auto radiusError = [](const mesheal::Mesh& mesh, const std::vector<Eigen::Index>& faces) {
        double worst = 0.0;
        for (const Eigen::Index f : faces) {
            const Eigen::RowVector3d c = (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))) / 3.0;
            worst = std::max(worst, std::abs(c.norm() - 5.0));
        }
        return worst;
    };
    const mesheal::Mesh sphereFlat = fill(holed, 0);
    const mesheal::Mesh sphereFaired = fill(holed, 2);
    const double flatError = radiusError(sphereFlat, newFaces(holed, sphereFlat));
    const double fairError = radiusError(sphereFaired, newFaces(holed, sphereFaired));
    WARN("sphere cap radius error: flat " << flatError << ", faired " << fairError);
    REQUIRE(fairError < 0.25 * flatError);
}

TEST_CASE("A visitor exception aborts the cubic search and leaves the mesh untouched", "[repair][holes][cgal]") {
    const mesheal::Mesh holed = removeFaces(mesheal::SampleMeshes::createSphereMesh(16, 48, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 2.0; });
    mesheal::CgalSurfaceMesh sm = toCgal(holed);
    const std::size_t faces = sm.number_of_faces();
    Phases phases;
    phases.throwInCubic = true;
    REQUIRE_THROWS(PMP::triangulate_hole(sm, borderCycles(sm).front(),
                                         CGAL::parameters::visitor(PhaseVisitor(phases)).use_delaunay_triangulation(false)));
    REQUIRE(phases.cubic == 1);
    REQUIRE(phases.cubicSteps == 1);
    REQUIRE(sm.number_of_faces() == faces);
    REQUIRE(sm.is_valid());
    REQUIRE(borderCycles(sm).size() == 1);
}

TEST_CASE("stitch_borders only joins coincident points, which the load weld already merged", "[repair][holes][cgal]") {
    const mesheal::Mesh cube = mesheal::SampleMeshes::createGridCubeMesh(4, 4.0);
    mesheal::CgalSurfaceMesh welded = toCgal(cube);
    REQUIRE(PMP::stitch_borders(welded) == 0);

    // A crack 1e-6 wide (the right half of the top moved): no tolerance, nothing stitched
    mesheal::Mesh cracked = cube;
    std::vector<Eigen::Index> top;
    std::vector<Eigen::Index> rest;
    for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
        (cube.N(f, 2) > 0.5 ? top : rest).push_back(f);
    }
    mesheal::Mesh split = mesheal::extractFaces(cube, rest);
    mesheal::Mesh lid = mesheal::extractFaces(cube, top);
    mesheal::appendMesh(split, lid);  // same positions, separate vertices: an unwelded seam
    mesheal::CgalSurfaceMesh seam = toCgal(split);
    REQUIRE(borderCycles(seam).size() == 2);
    REQUIRE(PMP::stitch_borders(seam) == 16);
    REQUIRE(borderCycles(seam).empty());

    lid.V.col(2).array() += 1e-6;
    mesheal::Mesh gap = mesheal::extractFaces(cube, rest);
    mesheal::appendMesh(gap, lid);
    mesheal::CgalSurfaceMesh gapped = toCgal(gap);
    REQUIRE(PMP::stitch_borders(gapped) == 0);
}

// Hidden: timings that back the size limits (run in build_release with "[.holes-timing]")
TEST_CASE("Hole filling timings", "[.holes-timing]") {
    for (const int n : {100, 300, 600, 1200, 2400}) {
        const mesheal::Mesh holed = removeFaces(mesheal::SampleMeshes::createSphereMesh(n / 4, n, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 3.0; });
        for (int method = 0; method < 4; ++method) {
            if (method == 3 && n > 600) {
                continue; // cubic only
            }
            mesheal::CgalSurfaceMesh sm = toCgal(holed);
            Phases phases;
            const auto t0 = std::chrono::steady_clock::now();
            std::vector<mesheal::CgalFaceIndex> patch;
            const auto np = CGAL::parameters::visitor(PhaseVisitor(phases)).face_output_iterator(std::back_inserter(patch));
            const auto h = borderCycles(sm).front();
            if (method == 0) {
                PMP::triangulate_hole(sm, h, np);
            } else if (method == 1) {
                PMP::triangulate_and_refine_hole(sm, h, np);
            } else if (method == 2) {
                PMP::triangulate_refine_and_fair_hole(sm, h, np);
            } else {
                PMP::triangulate_hole(sm, h, np.use_delaunay_triangulation(false));
            }
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            WARN("n=" << n << " method=" << method << " " << ms << " ms, faces " << patch.size() << " planar " << phases.planar
                      << " quad " << phases.quadratic << "/" << phases.quadraticOk << " cubic " << phases.cubic);
        }
    }
}

TEST_CASE("Bunny holes", "[.holes-timing]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    mesheal::CgalSurfaceMesh base = toCgal(bunny);
    const auto cycles = borderCycles(base);
    WARN("bunny faces " << bunny.numFaces() << " holes " << cycles.size());
    for (std::size_t i = 0; i < cycles.size(); ++i) {
        std::vector<Eigen::Vector3d> pts;
        for (const auto h : CGAL::halfedges_around_face(cycles[i], base)) {
            const auto p = base.point(base.target(h));
            pts.emplace_back(p.x(), p.y(), p.z());
        }
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& p : pts) c += p;
        c /= static_cast<double>(pts.size());
        Eigen::MatrixXd A(pts.size(), 3);
        for (std::size_t k = 0; k < pts.size(); ++k) A.row(static_cast<Eigen::Index>(k)) = (pts[k] - c).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
        const Eigen::Vector3d n = svd.matrixV().col(2);
        double planeDev = 0.0, perim = 0.0, diam = 0.0;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            planeDev = std::max(planeDev, std::abs((pts[k] - c).dot(n)));
            perim += (pts[(k + 1) % pts.size()] - pts[k]).norm();
            diam = std::max(diam, (pts[k] - c).norm() * 2);
        }
        std::string line;
        for (int method = 0; method < 3; ++method) {
            mesheal::CgalSurfaceMesh sm = base;
            std::vector<mesheal::CgalFaceIndex> patch;
            const auto t0 = std::chrono::steady_clock::now();
            const auto np = CGAL::parameters::face_output_iterator(std::back_inserter(patch)).use_2d_constrained_delaunay_triangulation(true);
            if (method == 0) PMP::triangulate_hole(sm, cycles[i], np);
            else if (method == 1) PMP::triangulate_and_refine_hole(sm, cycles[i], np);
            else PMP::triangulate_refine_and_fair_hole(sm, cycles[i], np);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            line += " m" + std::to_string(method) + ":" + std::to_string(patch.size()) + "f/" + std::to_string(ms) + "ms";
        }
        WARN("hole " << i << " n=" << pts.size() << " perim " << perim << " diam " << diam << " planeDev/diam " << planeDev / diam << line);
    }
}

TEST_CASE("A T-junction crack loads as a zero-width hole that only snapping closes", "[repair][holes][cgal]") {
    const mesheal::Mesh cracked = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createCrackedBoxMesh());
    REQUIRE(mesheal::checkNonManifoldVertices(cracked).empty()); // the pinch at (2, 2, 4) was split on "load"
    const mesheal::CategoryResult open = mesheal::checkOpenEdges(cracked);
    REQUIRE(open.size() == 1);     // one slit loop
    REQUIRE(open.infoCount == 6);  // 4 edges on the left, 2 on the right

    // Every triangle on the slit's collinear rim is degenerate: CGAL finds no fill and changes nothing
    for (const bool cdt : {false, true}) {
        mesheal::CgalSurfaceMesh sm = toCgal(cracked);
        std::vector<mesheal::CgalFaceIndex> patch;
        PMP::triangulate_hole(sm, borderCycles(sm).front(),
                              CGAL::parameters::face_output_iterator(std::back_inserter(patch)).use_2d_constrained_delaunay_triangulation(cdt));
        REQUIRE(patch.empty());
        REQUIRE(borderCycles(sm).size() == 1);
    }

    // Snapping the rim vertices onto the opposite edges closes it without new faces in the gap
    mesheal::CgalSurfaceMesh sm = toCgal(cracked);
    const std::size_t faces = sm.number_of_faces();
    const std::size_t snapped = PMP::experimental::snap_borders(sm);
    const std::size_t stitched = PMP::stitch_borders(sm);
    WARN("snap_borders snapped " << snapped << ", stitched " << stitched << ", faces " << faces << " -> " << sm.number_of_faces());
    sm.collect_garbage();
    REQUIRE(borderCycles(sm).empty());
    const mesheal::Mesh closed = mesheal::fromCgalSurfaceMesh(sm);
    REQUIRE(mesheal::checkOpenEdges(closed).empty());
    REQUIRE(mesheal::checkNonManifoldEdges(closed).empty());
    REQUIRE(mesheal::signedVolume(closed) == Catch::Approx(64.0));
}

TEST_CASE("Soup cleanup after the Mesheal load", "[.holes-timing]") {
    for (const char* name : {"stanford_bunny.stl", "Turkey.stl", "self_intersecting_box.stl"}) {
        const mesheal::Mesh mesh = mesheal::loadStl(examplePath(name));
        std::vector<mesheal::CgalPoint3> points;
        for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
            points.emplace_back(mesh.V(v, 0), mesh.V(v, 1), mesh.V(v, 2));
        }
        std::vector<std::vector<std::size_t>> polygons;
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            polygons.push_back({static_cast<std::size_t>(mesh.F(f, 0)), static_cast<std::size_t>(mesh.F(f, 1)),
                                static_cast<std::size_t>(mesh.F(f, 2))});
        }
        const std::size_t p0 = points.size();
        const std::size_t f0 = polygons.size();
        PMP::repair_polygon_soup(points, polygons);
        mesheal::CgalSurfaceMesh sm;
        const bool direct = mesheal::toCgalSurfaceMesh(mesh, sm);
        const std::size_t borders = borderCycles(sm).size();
        const std::size_t stitched = PMP::stitch_borders(sm);
        WARN(name << ": points " << p0 << " -> " << points.size() << ", faces " << f0 << " -> " << polygons.size()
                  << "; halfedge mesh " << direct << ", holes " << borders << ", stitched " << stitched);
    }
}

// -----------------------------------------------------------------------------
// HoleFiller
// -----------------------------------------------------------------------------

TEST_CASE("Each fill closes a hole and keeps every input face and vertex", "[repair][holes]") {
    const mesheal::Mesh holed = sphereCap();
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(holed).front();
    for (const mesheal::HoleFillMethod method : {mesheal::HoleFillMethod::Flat, mesheal::HoleFillMethod::Refine, mesheal::HoleFillMethod::Smooth}) {
        CAPTURE(mesheal::holeFillMethodLabel(method));
        const mesheal::HoleFillResult r = mesheal::fillHole(holed, hole, method);
        REQUIRE(r.success);
        REQUIRE(r.closed());
        REQUIRE(watertight(r.mesh));
        REQUIRE(r.removedFaces.empty());
        REQUIRE(r.maxVertexMove == 0.0);
        REQUIRE(r.newCrossings == 0);
        // Input faces and vertices first, unchanged; the patch is appended
        REQUIRE(r.mesh.V.topRows(holed.numVertices()) == holed.V);
        REQUIRE(r.mesh.F.topRows(holed.numFaces()) == holed.F);
        REQUIRE(r.mesh.numVertices() == holed.numVertices() + static_cast<Eigen::Index>(r.verticesAdded));
        REQUIRE(r.addedFaces.size() == static_cast<std::size_t>(r.mesh.numFaces() - holed.numFaces()));
        REQUIRE(r.addedFaces.front() == holed.numFaces());
        REQUIRE((method == mesheal::HoleFillMethod::Flat) == (r.verticesAdded == 0));
        REQUIRE(r.patchArea > 0.0);
        REQUIRE(r.minNewAngleDeg > 0.0);
    }
}

TEST_CASE("The local patch fill matches CGAL on the whole mesh", "[repair][holes]") {
    const mesheal::Mesh holed = sphereCap();
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(holed).front();
    for (const mesheal::HoleFillMethod method : {mesheal::HoleFillMethod::Flat, mesheal::HoleFillMethod::Refine, mesheal::HoleFillMethod::Smooth}) {
        CAPTURE(mesheal::holeFillMethodLabel(method));
        mesheal::CgalSurfaceMesh sm = toCgal(holed);
        const auto h = borderCycles(sm).front();
        const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true);
        if (method == mesheal::HoleFillMethod::Flat) {
            PMP::triangulate_hole(sm, h, np);
        } else if (method == mesheal::HoleFillMethod::Refine) {
            PMP::triangulate_and_refine_hole(sm, h, np);
        } else {
            PMP::triangulate_refine_and_fair_hole(sm, h, np);
        }
        const mesheal::Mesh whole = mesheal::fromCgalSurfaceMesh(sm);
        const mesheal::HoleFillResult local = mesheal::fillHole(holed, hole, method);
        REQUIRE(local.mesh.numFaces() == whole.numFaces());
        REQUIRE(surfaceDistance(local.mesh, whole) < 1e-9);
    }
}

TEST_CASE("Fill Flat keeps a missing end cap planar; Fill Smooth domes it", "[repair][holes]") {
    const mesheal::Mesh open = openCylinder();
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(open).front();
    REQUIRE(mesheal::recommendedHoleFill(hole.shape) == mesheal::HoleFillMethod::Flat);
    const mesheal::HoleFillResult flat = mesheal::fillHole(open, hole, mesheal::HoleFillMethod::Flat);
    const mesheal::HoleFillResult smooth = mesheal::fillHole(open, hole, mesheal::HoleFillMethod::Smooth);
    REQUIRE(maxPlaneDistance(flat.mesh, flat.addedFaces, 10.0) < 1e-12);
    REQUIRE(maxPlaneDistance(smooth.mesh, smooth.addedFaces, 10.0) > 1.0);
    const double cylinderVolume = std::numbers::pi * 25.0 * 10.0;
    REQUIRE(mesheal::signedVolume(flat.mesh) == Catch::Approx(cylinderVolume).epsilon(0.01)); // 32-gon: -0.6%
    REQUIRE(mesheal::signedVolume(smooth.mesh) > mesheal::signedVolume(flat.mesh));
}

TEST_CASE("Close Crack seals a T-junction crack that every fill refuses", "[repair][holes]") {
    const mesheal::Mesh cracked = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createCrackedBoxMesh());
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(cracked).front();
    REQUIRE(mesheal::recommendedHoleFill(hole.shape) == mesheal::HoleFillMethod::CloseCrack);

    const mesheal::HoleFillResult flat = mesheal::fillHole(cracked, hole, mesheal::HoleFillMethod::Flat);
    REQUIRE_FALSE(flat.success);
    REQUIRE(flat.statusMessage.find("Close Crack") != std::string::npos);
    REQUIRE(flat.mesh.F == cracked.F);

    const mesheal::HoleFillResult crack = mesheal::fillHole(cracked, hole, mesheal::HoleFillMethod::CloseCrack);
    REQUIRE(crack.success);
    REQUIRE(crack.closed());
    REQUIRE(watertight(crack.mesh));
    REQUIRE(mesheal::checkNonManifoldVertices(crack.mesh).empty());
    REQUIRE(crack.maxVertexMove < 1e-12);         // the T-vertices already lie on the opposite edges
    REQUIRE(crack.removedFaces.size() == 2);      // the two strip faces split at (2, 1) and (2, 3) ...
    REQUIRE(crack.addedFaces.size() == 4);        // ... into four (renumbered faces are not changes)
    REQUIRE(crack.patchArea == Catch::Approx(2.0));
    const mesheal::FaceDiff diff = mesheal::faceDiff(cracked, crack.mesh);
    REQUIRE(diff.removed == crack.removedFaces);
    REQUIRE(diff.added.size() == 4);
    REQUIRE(mesheal::signedVolume(crack.mesh) == Catch::Approx(64.0));
    REQUIRE(crack.mesh.numVertices() == cracked.numVertices() - 1); // the pinch copies merged again
}

TEST_CASE("Close Crack narrows a gap by at most its width and refuses a real hole", "[repair][holes]") {
    // The strip's copy of (2, 2, 4) pulled 0.01 into the strip: a thin wedge-shaped crack
    mesheal::Mesh cracked = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createCrackedBoxMesh());
    std::vector<Eigen::Index> copies;
    for (Eigen::Index v = 0; v < cracked.numVertices(); ++v) {
        if ((cracked.V.row(v) - Eigen::RowVector3d(2.0, 2.0, 4.0)).norm() == 0.0) {
            copies.push_back(v);
        }
    }
    REQUIRE(copies.size() == 2);
    const mesheal::HoleLoop before = mesheal::findHoleLoops(cracked).front();
    // Move the copy whose faces lie in x > 2 (the strip)
    for (const Eigen::Index v : copies) {
        bool strip = false;
        for (Eigen::Index f = 0; f < cracked.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                strip = strip || (cracked.F(f, c) == v && cracked.V.row(cracked.F(f, (c + 1) % 3)).x() > 2.5);
            }
        }
        if (strip) {
            cracked.V(v, 0) += 0.01;
        }
    }
    cracked.recomputeNormals();
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(cracked).front();
    REQUIRE(hole.shape == mesheal::HoleShape::Crack);
    REQUIRE(hole.width > 0.0);
    const mesheal::HoleFillResult crack = mesheal::fillHole(cracked, hole, mesheal::HoleFillMethod::CloseCrack);
    REQUIRE(crack.closed());
    REQUIRE(watertight(crack.mesh));
    REQUIRE(crack.maxVertexMove <= 0.01 + 1e-12);
    REQUIRE(crack.maxVertexMove > 0.0);

    const mesheal::Mesh square = flatHoleBox();
    const mesheal::HoleFillResult refused = mesheal::fillHole(square, mesheal::findHoleLoops(square).front(), mesheal::HoleFillMethod::CloseCrack);
    REQUIRE_FALSE(refused.success);
    REQUIRE(refused.mesh.V == square.V);
}

TEST_CASE("Hole fills refuse open chains and oversized refined patches", "[repair][holes]") {
    const mesheal::Mesh touching = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createTouchingCubesMesh());
    const mesheal::HoleLoop chain = mesheal::findHoleLoops(touching).front();
    REQUIRE(chain.shape == mesheal::HoleShape::OpenChain);
    REQUIRE(mesheal::recommendedHoleFill(chain.shape) == mesheal::HoleFillMethod::None);
    const mesheal::HoleFillResult r = mesheal::fillHole(touching, chain, mesheal::HoleFillMethod::Flat);
    REQUIRE_FALSE(r.success);
    REQUIRE(r.statusMessage.find("closed loop") != std::string::npos);

    const mesheal::Mesh holed = sphereCap();
    mesheal::HoleFillSettings settings;
    settings.maxPatchFaces = 10;
    const mesheal::HoleFillResult big = mesheal::fillHole(holed, mesheal::findHoleLoops(holed).front(), mesheal::HoleFillMethod::Smooth, {}, settings);
    REQUIRE_FALSE(big.success);
    REQUIRE(big.statusMessage.find("limit 10") != std::string::npos);
    REQUIRE(big.mesh.F == holed.F);
}

TEST_CASE("A hole preview leaves the mesh alone and offsets the patch", "[repair][holes]") {
    const mesheal::Mesh holed = flatHoleBox();
    const mesheal::HoleLoop hole = mesheal::findHoleLoops(holed).front();
    const mesheal::HoleFillPreview preview = mesheal::previewHoleFill(holed, hole, mesheal::HoleFillMethod::Flat);
    REQUIRE(preview.valid);
    REQUIRE(preview.facesAdded == 6); // an 8-vertex rim
    REQUIRE(preview.verticesAdded == 0);
    REQUIRE(preview.openEdgesLeft == 0);
    REQUIRE(preview.sourceFaceIndices.empty());
    REQUIRE(preview.patchArea == Catch::Approx(4.0));
    REQUIRE(preview.previewPatchMesh.numFaces() == 6);
    REQUIRE(preview.previewPatchMesh.V.col(2).minCoeff() > 6.0); // lifted off the top along +z

    const mesheal::HoleFillPreview none = mesheal::previewHoleFill(holed, hole, mesheal::HoleFillMethod::None);
    REQUIRE_FALSE(none.valid);
}

TEST_CASE("A fill that pokes through another shell reports new crossings", "[repair][holes]") {
    // A plate (wider than the cylinder) just above its open top: the smooth dome rises through it
    mesheal::Mesh mesh = openCylinder();
    mesheal::Mesh plate = mesheal::SampleMeshes::createCubeMesh(14.0);
    plate.V.col(0).array() -= 7.0;
    plate.V.col(1).array() -= 7.0;
    plate.V.col(2) = plate.V.col(2) * (0.2 / 14.0);
    plate.V.col(2).array() += 10.5;
    plate.recomputeNormals();
    mesheal::appendMesh(mesh, plate);
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(mesh);
    REQUIRE(holes.size() == 1);
    REQUIRE(mesheal::fillHole(mesh, holes.front(), mesheal::HoleFillMethod::Flat).newCrossings == 0);
    const mesheal::HoleFillResult dome = mesheal::fillHole(mesh, holes.front(), mesheal::HoleFillMethod::Smooth);
    REQUIRE(dome.newCrossings > 0);
}

TEST_CASE("Filling every hole of the Holes sample makes four closed shells", "[repair][holes]") {
    const mesheal::Mesh sample = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createHolesSampleMesh());
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(sample);
    REQUIRE(holes.size() == 4);
    const mesheal::HoleBatchResult r = mesheal::fillHoles(sample, holes);
    REQUIRE(r.success);
    REQUIRE(r.holesClosed == 4);
    REQUIRE(r.holesFailed == 0);
    REQUIRE(r.holesSkipped == 0);
    REQUIRE(watertight(r.mesh));
    REQUIRE(mesheal::checkShells(r.mesh).infoCount == 4);
    REQUIRE(mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{}).empty());
    REQUIRE(mesheal::faceDiff(sample, r.mesh).added.size() == r.facesAdded); // incl. the 4 halves of the 2 split strip faces
    REQUIRE(mesheal::faceDiff(sample, r.mesh).removed.size() == 2);

    // A size limit skips the big holes; a forced method applies to every hole
    mesheal::HoleFillSettings settings;
    settings.maxHoleEdges = 10;
    const mesheal::HoleBatchResult small = mesheal::fillHoles(sample, holes, mesheal::HoleFillMethod::None, settings);
    REQUIRE(small.holesClosed == 2);
    REQUIRE(small.holesSkipped == 2);
    const mesheal::HoleBatchResult flat = mesheal::fillHoles(sample, holes, mesheal::HoleFillMethod::Flat);
    REQUIRE(flat.holesClosed == 3);
    REQUIRE(flat.holesFailed == 1); // the crack
    REQUIRE(flat.statusMessage.find("Close Crack") != std::string::npos);
}

TEST_CASE("Filling the bunny's holes closes it without new crossings", "[repair][holes]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(bunny);
    const mesheal::HoleBatchResult r = mesheal::fillHoles(bunny, holes);
    REQUIRE(r.holesClosed == 5);
    REQUIRE(watertight(r.mesh));
    REQUIRE(mesheal::checkShells(r.mesh).infoCount == 1);
    REQUIRE(r.mesh.F.topRows(bunny.numFaces()) == bunny.F);
    const mesheal::CategoryResult crossings = mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{});
    for (std::size_t e = 0; e < crossings.size(); ++e) {
        REQUIRE(crossings.facesOf(e)[0] < bunny.numFaces()); // only crossings the bunny already had
    }
    REQUIRE(mesheal::signedVolume(r.mesh) > 0.0);
}

TEST_CASE("Repair All Holes: Fill each hole in the shells in scope", "[repair][holes]") {
    const mesheal::Mesh sample = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createHolesSampleMesh());
    REQUIRE(mesheal::globalRepairMethodsFor(mesheal::DefectCategory::OpenEdges).front() == mesheal::GlobalRepairMethod::FillHoles);
    REQUIRE(mesheal::globalRepairMethodsFor(mesheal::DefectCategory::Intersections).size() == mesheal::kGlobalRepairMethods.size());

    const mesheal::RepairCandidate all = mesheal::runGlobalRepair(sample, {}, mesheal::GlobalRepairMethod::FillHoles, mesheal::GlobalRepairParams{});
    REQUIRE(all.success);
    REQUIRE(all.metrics.watertight());
    REQUIRE(all.metrics.openEdgesBefore == 6 + 8 + 24 + 24);
    REQUIRE(all.metrics.shellsAfter == 4);
    REQUIRE(all.metrics.intersectingFacesAfter == 0);
    REQUIRE(all.removedFaces.size() == 2);   // the crack's two split strip faces
    REQUIRE(all.metrics.unchangedFraction > 0.99);

    // Only the sphere in scope: the other shells and their holes stay as they are
    const mesheal::Mesh sphereOnly = mesheal::extractFaces(sample, [&] {
        std::vector<Eigen::Index> faces;
        for (Eigen::Index f = 0; f < sample.numFaces(); ++f) {
            if (sample.V(sample.F(f, 0), 0) > 11.0 && sample.V(sample.F(f, 0), 0) < 17.0) {
                faces.push_back(f);
            }
        }
        return faces;
    }());
    std::vector<std::uint8_t> inScope(static_cast<std::size_t>(sample.numFaces()), 0);
    for (Eigen::Index f = 0; f < sample.numFaces(); ++f) {
        inScope[static_cast<std::size_t>(f)] = sample.V(sample.F(f, 0), 0) > 11.0 && sample.V(sample.F(f, 0), 0) < 17.0 ? 1 : 0;
    }
    mesheal::GlobalRepairParams flat;
    flat.holeMethod = mesheal::HoleFillMethod::Flat;
    const mesheal::RepairCandidate scoped = mesheal::runGlobalRepair(sample, inScope, mesheal::GlobalRepairMethod::FillHoles, flat);
    REQUIRE(scoped.success);
    REQUIRE(scoped.metrics.watertight());                              // measured on the scope only
    REQUIRE(mesheal::checkOpenEdges(scoped.mesh).infoCount == 6 + 8 + 24); // the other three holes are untouched
    REQUIRE(scoped.metrics.facesAfter == sphereOnly.numFaces() + 22); // a flat 24-gon
    REQUIRE_FALSE(flat == mesheal::GlobalRepairParams{});
}

TEST_CASE("Repair All Holes on the bunny", "[.holes-timing]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    for (const mesheal::GlobalRepairMethod method : mesheal::kHoleRepairMethods) {
        const mesheal::RepairCandidate c = mesheal::runGlobalRepair(bunny, {}, method, mesheal::GlobalRepairParams{});
        WARN(mesheal::globalRepairMethodLabel(method) << ": " << c.metrics.elapsedMs << " ms, watertight " << c.metrics.watertight()
                                                   << ", open " << c.metrics.openEdgesAfter << ", dV " << c.metrics.volumeChangePercent()
                                                   << "%, max dev " << c.metrics.maxDeviation << ", lost " << c.metrics.maxLost
                                                   << ", unchanged " << c.metrics.unchangedFraction << ", faces " << c.metrics.facesAfter);
    }
}

namespace {

mesheal::HoleBatchResult fillWithPolicy(const mesheal::Mesh& mesh, mesheal::HoleCrossingPolicy policy, std::span<const std::uint8_t> scope = {}) {
    mesheal::HoleBatchOptions options;
    options.crossingPolicy = policy;
    options.faceInScope = scope;
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(mesh);
    return mesheal::fillHoles(mesh, holes, mesheal::HoleFillMethod::None, mesheal::HoleFillSettings{}, options);
}

/// Every input face the map keeps has the same corners in the result, and the new faces come last.
void requireFaceMapKeepsGeometry(const mesheal::Mesh& before, const mesheal::HoleBatchResult& r) {
    REQUIRE(r.faceMap.size() == static_cast<std::size_t>(before.numFaces()));
    Eigen::Index kept = 0;
    for (Eigen::Index f = 0; f < before.numFaces(); ++f) {
        const Eigen::Index g = r.faceMap[static_cast<std::size_t>(f)];
        if (g < 0) {
            continue;
        }
        REQUIRE(g == kept++);
        REQUIRE(mesheal::faceGeometryKey(before, f) == mesheal::faceGeometryKey(r.mesh, g));
    }
    for (const Eigen::Index f : r.addedFaces) {
        REQUIRE(f >= kept);
    }
}

} // namespace

TEST_CASE("The cup with a post: the batch keeps the first fill that crosses nothing", "[repair][holes][crossings]") {
    const mesheal::Mesh cup = mesheal::SampleMeshes::createCupWithPostMesh();
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(cup);
    REQUIRE(holes.size() == 1);
    REQUIRE(holes.front().shape == mesheal::HoleShape::Curved); // the rim faces slope at 45 degrees
    REQUIRE(mesheal::checkShells(cup).infoCount == 1);
    REQUIRE(mesheal::checkSelfIntersections(cup, mesheal::CheckParams{}).empty());
    REQUIRE(mesheal::fillHole(cup, holes.front(), mesheal::HoleFillMethod::Smooth).newCrossings > 0); // it sags onto the post
    REQUIRE(mesheal::fillHole(cup, holes.front(), mesheal::HoleFillMethod::Flat).newCrossings == 0);

    // Today's batch (no crossing test) keeps the recommended Smooth fill and its crossings
    const mesheal::HoleBatchResult plain = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::IgnoreCrossings);
    REQUIRE_FALSE(plain.crossingsTested);
    REQUIRE(plain.outcomes.size() == 1);
    REQUIRE(plain.outcomes.front().method == mesheal::HoleFillMethod::Smooth);
    REQUIRE(plain.outcomes.front().attempts.size() == 1);
    REQUIRE_FALSE(mesheal::checkSelfIntersections(plain.mesh, mesheal::CheckParams{}).empty());
    REQUIRE(plain.mesh.F == mesheal::fillHoles(cup, holes).mesh.F); // the old overload is the same call

    // Both crossing policies try Smooth (crosses the post), then Refine: flat like Fill Flat, so it crosses nothing
    for (const mesheal::HoleCrossingPolicy policy :
         {mesheal::HoleCrossingPolicy::NeverCommitCrossing, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings}) {
        const mesheal::HoleBatchResult r = fillWithPolicy(cup, policy);
        REQUIRE(r.crossingsTested);
        REQUIRE(r.holesClosed == 1);
        const mesheal::HoleFillOutcome& o = r.outcomes.front();
        REQUIRE(o.shell == 0);
        REQUIRE(o.attempts.size() == 2);
        REQUIRE(o.attempts[0].method == mesheal::HoleFillMethod::Smooth);
        REQUIRE(o.attempts[0].crossingsOwnShell > 0);
        REQUIRE(o.attempts[0].crossingsOtherShells == 0);
        REQUIRE(o.attempts[1].method == mesheal::HoleFillMethod::Refine);
        REQUIRE(o.method == mesheal::HoleFillMethod::Refine);
        REQUIRE(o.closed);
        REQUIRE(o.crossingsOwnShell + o.crossingsOtherShells == 0);
        REQUIRE(r.holesWithCrossings == 0);
        REQUIRE(watertight(r.mesh));
        REQUIRE(mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{}).empty());
        REQUIRE(mesheal::signedVolume(r.mesh) > 0.0);
        REQUIRE(r.mesh.V.col(2).maxCoeff() == 10.0); // nothing above the cup's top
        requireFaceMapKeepsGeometry(cup, r);
    }
}

TEST_CASE("A post through the opening: every fill crosses the hole's own shell", "[repair][holes][crossings]") {
    const mesheal::Mesh cup = mesheal::SampleMeshes::createCupWithPostMesh(mesheal::SampleMeshes::CupPost::ThroughOpening);

    // Always fill: the fill with the fewest crossings is kept, with them listed as its own shell's
    const mesheal::HoleBatchResult r = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings);
    REQUIRE(r.success);
    REQUIRE(r.holesClosed == 1);
    const mesheal::HoleFillOutcome& o = r.outcomes.front();
    REQUIRE(o.attempts.size() == 3); // Smooth, Refine, Flat: all cross
    std::size_t fewest = std::numeric_limits<std::size_t>::max();
    for (const mesheal::HoleFillAttempt& a : o.attempts) {
        REQUIRE(a.closed);
        REQUIRE(a.crossingsOwnShell > 0);
        REQUIRE(a.crossingsOtherShells == 0);
        fewest = std::min(fewest, a.crossingsOwnShell);
    }
    REQUIRE(o.method == mesheal::HoleFillMethod::Flat);
    REQUIRE(o.crossingsOwnShell == fewest);
    REQUIRE(o.crossingsOtherShells == 0);
    REQUIRE(o.otherShells.empty());
    REQUIRE(r.holesWithCrossings == 1);
    REQUIRE(r.crossingsOwnShell == fewest);
    REQUIRE(r.crossedShells.empty());
    REQUIRE(watertight(r.mesh));
    REQUIRE_FALSE(mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{}).empty()); // left for the island step
    REQUIRE(r.statusMessage.find("1 fill crosses other faces") != std::string::npos);
    requireFaceMapKeepsGeometry(cup, r);

    // Never commit a crossing: the hole stays open, the mesh is the input
    const mesheal::HoleBatchResult never = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::NeverCommitCrossing);
    REQUIRE_FALSE(never.success);
    REQUIRE(never.holesFailed == 1);
    REQUIRE(never.outcomes.front().method == mesheal::HoleFillMethod::None);
    REQUIRE(never.outcomes.front().attempts.size() == 3);
    REQUIRE(never.outcomes.front().status.find("crosses") != std::string::npos);
    REQUIRE(never.mesh.F == cup.F);
    REQUIRE(never.mesh.V == cup.V);
}

TEST_CASE("A hole whose every fill crosses another shell is filled, and the shell is named", "[repair][holes][crossings]") {
    const mesheal::Mesh cup = mesheal::SampleMeshes::createCupWithPostMesh(mesheal::SampleMeshes::CupPost::RodThroughOpening);
    REQUIRE(mesheal::checkShells(cup).infoCount == 2);
    REQUIRE(mesheal::checkSelfIntersections(cup, mesheal::CheckParams{}).empty()); // the rod touches nothing

    const mesheal::HoleBatchResult r = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings);
    REQUIRE(r.holesClosed == 1);
    const mesheal::HoleFillOutcome& o = r.outcomes.front();
    REQUIRE(o.shell == 0);                                   // the cup: #1
    REQUIRE(o.attempts.size() == 3);
    REQUIRE(o.crossingsOwnShell == 0);
    REQUIRE(o.crossingsOtherShells > 0);
    REQUIRE(o.otherShells == std::vector<int>{1});           // the rod: #2
    REQUIRE(o.otherShellCrossings == std::vector<std::size_t>{o.crossingsOtherShells});
    REQUIRE(r.crossedShells == std::vector<int>{1});
    REQUIRE(r.crossingsOtherShells == o.crossingsOtherShells);
    REQUIRE(watertight(r.mesh));

    // Never commit: open as before; no crossing test: today's Smooth fill
    const mesheal::HoleBatchResult never = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::NeverCommitCrossing);
    REQUIRE(never.holesFailed == 1);
    REQUIRE(never.mesh.F == cup.F);
    const mesheal::HoleBatchResult plain = fillWithPolicy(cup, mesheal::HoleCrossingPolicy::IgnoreCrossings);
    REQUIRE(plain.outcomes.front().method == mesheal::HoleFillMethod::Smooth);
    REQUIRE(plain.outcomes.front().otherShells.empty());
}

TEST_CASE("The batch fill leaves faces outside its scope bit-identical", "[repair][holes][crossings]") {
    const mesheal::Mesh sample = mesheal::separateTouchingContacts(mesheal::SampleMeshes::createHolesSampleMesh());
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(sample);
    REQUIRE(holes.size() == 4);

    // Scope = the sphere: its hole is filled, the other three are skipped as outside the scope
    std::vector<std::uint8_t> sphere(static_cast<std::size_t>(sample.numFaces()), 0);
    for (Eigen::Index f = 0; f < sample.numFaces(); ++f) {
        sphere[static_cast<std::size_t>(f)] = sample.V(sample.F(f, 0), 0) > 11.0 && sample.V(sample.F(f, 0), 0) < 17.0 ? 1 : 0;
    }
    for (const mesheal::HoleCrossingPolicy policy :
         {mesheal::HoleCrossingPolicy::IgnoreCrossings, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings}) {
        const mesheal::HoleBatchResult r = fillWithPolicy(sample, policy, sphere);
        REQUIRE(r.holesClosed == 1);
        REQUIRE(r.holesSkipped == 3);
        std::size_t outOfScope = 0;
        for (const mesheal::HoleFillOutcome& o : r.outcomes) {
            outOfScope += o.skipped == mesheal::HoleSkipReason::OutOfScope ? 1 : 0;
        }
        REQUIRE(outOfScope == 3);
        REQUIRE(mesheal::checkOpenEdges(r.mesh).infoCount == 6 + 8 + 24);
        requireFaceMapKeepsGeometry(sample, r);
    }

    // Scope = only the crack's rim faces: Close Crack would move vertices of faces outside it, so it is not used
    const auto crack = std::find_if(holes.begin(), holes.end(), [](const mesheal::HoleLoop& h) { return h.shape == mesheal::HoleShape::Crack; });
    REQUIRE(crack != holes.end());
    std::vector<std::uint8_t> rim(static_cast<std::size_t>(sample.numFaces()), 0);
    for (const Eigen::Index f : crack->faces) {
        rim[static_cast<std::size_t>(f)] = 1;
    }
    const mesheal::HoleBatchResult r = fillWithPolicy(sample, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings, rim);
    const mesheal::HoleFillOutcome& o = r.outcomes[static_cast<std::size_t>(crack - holes.begin())];
    REQUIRE(o.method == mesheal::HoleFillMethod::None);
    REQUIRE(o.attempts.front().method == mesheal::HoleFillMethod::CloseCrack);
    REQUIRE(o.attempts.front().status == "It would change faces outside the scope.");
    REQUIRE(r.mesh.V == sample.V);
    REQUIRE(r.mesh.F == sample.F);

    // A scope of the wrong size changes nothing
    const std::vector<std::uint8_t> wrong(3, 1);
    const mesheal::HoleBatchResult bad = fillWithPolicy(sample, mesheal::HoleCrossingPolicy::IgnoreCrossings, wrong);
    REQUIRE_FALSE(bad.success);
    REQUIRE(bad.mesh.F == sample.F);
}

TEST_CASE("A cancelled batch fill returns the input", "[repair][holes][crossings]") {
    const mesheal::Mesh cup = mesheal::SampleMeshes::createCupWithPostMesh(mesheal::SampleMeshes::CupPost::ThroughOpening);
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(cup);
    std::stop_source stop;
    stop.request_stop();
    mesheal::HoleBatchOptions options;
    options.crossingPolicy = mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings;
    const mesheal::HoleBatchResult r = mesheal::fillHoles(cup, holes, mesheal::HoleFillMethod::None, {}, options, stop.get_token());
    REQUIRE_FALSE(r.success);
    REQUIRE(r.statusMessage == "Cancelled.");
    REQUIRE(r.mesh.F == cup.F);
    REQUIRE(r.mesh.V == cup.V);
}

TEST_CASE("The bunny's holes fill the same way under the always-fill policy", "[repair][holes][crossings]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    const mesheal::HoleBatchResult plain = fillWithPolicy(bunny, mesheal::HoleCrossingPolicy::IgnoreCrossings);
    const mesheal::HoleBatchResult always = fillWithPolicy(bunny, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings);
    REQUIRE(always.holesClosed == 5);
    for (const mesheal::HoleFillOutcome& o : always.outcomes) {
        REQUIRE(o.attempts.size() == 1); // the recommended fill crosses nothing
        REQUIRE(o.crossingsOwnShell + o.crossingsOtherShells == 0);
    }
    REQUIRE(always.mesh.F == plain.mesh.F);
    REQUIRE(always.mesh.V == plain.mesh.V);
    requireFaceMapKeepsGeometry(bunny, always);
}

TEST_CASE("Batch fill crossing policy timings on the bunny", "[.benchmark]") {
    const mesheal::Mesh bunny = mesheal::loadStl(examplePath("stanford_bunny.stl"));
    for (const mesheal::HoleCrossingPolicy policy :
         {mesheal::HoleCrossingPolicy::IgnoreCrossings, mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings}) {
        const auto start = std::chrono::steady_clock::now();
        const mesheal::HoleBatchResult r = fillWithPolicy(bunny, policy);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        WARN("policy " << static_cast<int>(policy) << ": " << ms << " ms, closed " << r.holesClosed);
    }
}
