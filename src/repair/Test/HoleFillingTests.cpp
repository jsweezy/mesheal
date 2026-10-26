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

#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace {

namespace PMP = CGAL::Polygon_mesh_processing;

/// Faces of `mesh` whose centroid fails `drop`, compacted.
ogun::Mesh removeFaces(const ogun::Mesh& mesh, const std::function<bool(const Eigen::Vector3d&)>& drop) {
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d c =
            (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))).transpose() / 3.0;
        if (!drop(c)) {
            keep.push_back(f);
        }
    }
    return ogun::extractFaces(mesh, keep);
}



std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

ogun::CgalSurfaceMesh toCgal(const ogun::Mesh& mesh) {
    ogun::CgalSurfaceMesh sm;
    REQUIRE(ogun::toCgalSurfaceMesh(mesh, sm));
    return sm;
}

std::vector<ogun::CgalHalfedgeIndex> borderCycles(const ogun::CgalSurfaceMesh& sm) {
    std::vector<ogun::CgalHalfedgeIndex> cycles;
    std::vector<bool> seen(sm.number_of_halfedges() + sm.number_of_removed_halfedges(), false);
    for (const ogun::CgalHalfedgeIndex h : sm.halfedges()) {
        if (!sm.is_border(h) || seen[h.idx()]) {
            continue;
        }
        cycles.push_back(h);
        for (const ogun::CgalHalfedgeIndex b : CGAL::halfedges_around_face(h, sm)) {
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

double maxPlaneDistance(const ogun::Mesh& mesh, const std::vector<Eigen::Index>& faces, double z) {
    double worst = 0.0;
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            worst = std::max(worst, std::abs(mesh.V(mesh.F(f, c), 2) - z));
        }
    }
    return worst;
}

std::vector<Eigen::Index> newFaces(const ogun::Mesh& before, const ogun::Mesh& after) {
    return ogun::faceDiff(before, after).added;
}


ogun::Mesh flatHoleBox() {
    return removeFaces(ogun::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });
}

ogun::Mesh sphereCap() {
    return removeFaces(ogun::SampleMeshes::createSphereMesh(24, 32, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
}

ogun::Mesh openCylinder() {
    return removeFaces(ogun::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0), [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
}

bool watertight(const ogun::Mesh& mesh) {
    return ogun::checkOpenEdges(mesh).empty() && ogun::checkNonManifoldEdges(mesh).empty();
}

/// Largest distance between the two surfaces (both directions).
double surfaceDistance(const ogun::Mesh& a, const ogun::Mesh& b) {
    return std::max(ogun::vertexDeviation(a, b).maxCoeff(), ogun::vertexDeviation(b, a).maxCoeff());
}

} // namespace

// -----------------------------------------------------------------------------
// What CGAL's hole filling actually does (pins the assumptions the hole repair is built on)
// -----------------------------------------------------------------------------

TEST_CASE("CGAL triangulate_hole uses the 2D CDT only when asked", "[repair][holes][cgal]") {
    // A flat 2 x 2-cell square hole in the top of a grid cube
    const ogun::Mesh holed = removeFaces(ogun::SampleMeshes::createGridCubeMesh(6, 6.0), [](const Eigen::Vector3d& c) {
        return c.z() > 5.9 && c.x() > 2.0 && c.x() < 4.0 && c.y() > 2.0 && c.y() < 4.0;
    });

    // The CGAL 6.x docs say use_2d_constrained_delaunay_triangulation defaults to true, the code to false
    ogun::CgalSurfaceMesh sm = toCgal(holed);
    Phases byDefault;
    PMP::triangulate_hole(sm, borderCycles(sm).front(), CGAL::parameters::visitor(PhaseVisitor(byDefault)));
    REQUIRE(byDefault.planar == 0);
    REQUIRE(byDefault.quadraticOk); // 3D Delaunay search space (2D for a flat hole: no start call)
    REQUIRE(byDefault.cubic == 0);

    ogun::CgalSurfaceMesh sm2 = toCgal(holed);
    Phases withCdt;
    PMP::triangulate_hole(sm2, borderCycles(sm2).front(),
                          CGAL::parameters::visitor(PhaseVisitor(withCdt)).use_2d_constrained_delaunay_triangulation(true));
    REQUIRE(withCdt.planar == 1);
    REQUIRE(withCdt.planarOk);
    REQUIRE(withCdt.quadratic == 0);
}

TEST_CASE("CGAL hole filling only adds faces and vertices", "[repair][holes][cgal]") {
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const ogun::Mesh holed = removeFaces(sphere, [](const Eigen::Vector3d& c) { return c.z() > 3.5; });
    const std::size_t holedFaces = static_cast<std::size_t>(holed.numFaces());

    for (int method = 0; method < 3; ++method) {
        ogun::CgalSurfaceMesh sm = toCgal(holed);
        std::vector<ogun::CgalFaceIndex> patch;
        std::vector<ogun::CgalVertexIndex> added;
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
            const auto p = sm.point(ogun::CgalVertexIndex(static_cast<std::uint32_t>(v)));
            REQUIRE(Eigen::Vector3d(p.x(), p.y(), p.z()) == Eigen::Vector3d(holed.V.row(v).transpose()));
        }
        for (const ogun::CgalVertexIndex v : added) {
            REQUIRE(static_cast<Eigen::Index>(v.idx()) >= holed.numVertices());
        }
        REQUIRE((method == 0) == added.empty());
        // Original faces keep their index and corners
        for (Eigen::Index f = 0; f < holed.numFaces(); ++f) {
            const ogun::CgalFaceIndex cf(static_cast<std::uint32_t>(f));
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
    const ogun::Mesh cylinder = ogun::SampleMeshes::createCylinderMesh(32, 4, 5.0, 10.0);
    const ogun::Mesh open = removeFaces(cylinder, [](const Eigen::Vector3d& c) { return c.z() > 9.99; });
    const auto fill = [](const ogun::Mesh& mesh, int method) {
        ogun::CgalSurfaceMesh sm = toCgal(mesh);
        const auto h = borderCycles(sm).front();
        const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true);
        if (method == 0) {
            PMP::triangulate_hole(sm, h, np);
        } else if (method == 1) {
            PMP::triangulate_and_refine_hole(sm, h, np);
        } else {
            PMP::triangulate_refine_and_fair_hole(sm, h, np);
        }
        return ogun::fromCgalSurfaceMesh(sm, mesh.fileUnit);
    };
    const ogun::Mesh flat = fill(open, 0);
    const ogun::Mesh refined = fill(open, 1);
    const ogun::Mesh faired = fill(open, 2);
    REQUIRE(maxPlaneDistance(flat, newFaces(open, flat), 10.0) < 1e-12);
    REQUIRE(maxPlaneDistance(refined, newFaces(open, refined), 10.0) < 1e-9);
    // Fairing keeps tangent continuity with the walls, so the cap bulges into a dome
    REQUIRE(maxPlaneDistance(faired, newFaces(open, faired), 10.0) > 1.0);
    REQUIRE(newFaces(open, refined).size() > newFaces(open, flat).size());

    // Missing sphere cap (its rim is a planar circle too): only fairing restores the curvature
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(24, 32, 5.0);
    const ogun::Mesh holed = removeFaces(sphere, [](const Eigen::Vector3d& c) { return c.z() > 4.0; });
    const auto radiusError = [](const ogun::Mesh& mesh, const std::vector<Eigen::Index>& faces) {
        double worst = 0.0;
        for (const Eigen::Index f : faces) {
            const Eigen::RowVector3d c = (mesh.V.row(mesh.F(f, 0)) + mesh.V.row(mesh.F(f, 1)) + mesh.V.row(mesh.F(f, 2))) / 3.0;
            worst = std::max(worst, std::abs(c.norm() - 5.0));
        }
        return worst;
    };
    const ogun::Mesh sphereFlat = fill(holed, 0);
    const ogun::Mesh sphereFaired = fill(holed, 2);
    const double flatError = radiusError(sphereFlat, newFaces(holed, sphereFlat));
    const double fairError = radiusError(sphereFaired, newFaces(holed, sphereFaired));
    WARN("sphere cap radius error: flat " << flatError << ", faired " << fairError);
    REQUIRE(fairError < 0.25 * flatError);
}

TEST_CASE("A visitor exception aborts the cubic search and leaves the mesh untouched", "[repair][holes][cgal]") {
    const ogun::Mesh holed = removeFaces(ogun::SampleMeshes::createSphereMesh(16, 48, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 2.0; });
    ogun::CgalSurfaceMesh sm = toCgal(holed);
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
    const ogun::Mesh cube = ogun::SampleMeshes::createGridCubeMesh(4, 4.0);
    ogun::CgalSurfaceMesh welded = toCgal(cube);
    REQUIRE(PMP::stitch_borders(welded) == 0);

    // A crack 1e-6 wide (the right half of the top moved): no tolerance, nothing stitched
    ogun::Mesh cracked = cube;
    std::vector<Eigen::Index> top;
    std::vector<Eigen::Index> rest;
    for (Eigen::Index f = 0; f < cube.numFaces(); ++f) {
        (cube.N(f, 2) > 0.5 ? top : rest).push_back(f);
    }
    ogun::Mesh split = ogun::extractFaces(cube, rest);
    ogun::Mesh lid = ogun::extractFaces(cube, top);
    ogun::appendMesh(split, lid);  // same positions, separate vertices: an unwelded seam
    ogun::CgalSurfaceMesh seam = toCgal(split);
    REQUIRE(borderCycles(seam).size() == 2);
    REQUIRE(PMP::stitch_borders(seam) == 16);
    REQUIRE(borderCycles(seam).empty());

    lid.V.col(2).array() += 1e-6;
    ogun::Mesh gap = ogun::extractFaces(cube, rest);
    ogun::appendMesh(gap, lid);
    ogun::CgalSurfaceMesh gapped = toCgal(gap);
    REQUIRE(PMP::stitch_borders(gapped) == 0);
}

// Hidden: timings that back the size limits (run in build_release with "[.holes-timing]")
TEST_CASE("Hole filling timings", "[.holes-timing]") {
    for (const int n : {100, 300, 600, 1200, 2400}) {
        const ogun::Mesh holed = removeFaces(ogun::SampleMeshes::createSphereMesh(n / 4, n, 5.0), [](const Eigen::Vector3d& c) { return c.z() > 3.0; });
        for (int method = 0; method < 4; ++method) {
            if (method == 3 && n > 600) {
                continue; // cubic only
            }
            ogun::CgalSurfaceMesh sm = toCgal(holed);
            Phases phases;
            const auto t0 = std::chrono::steady_clock::now();
            std::vector<ogun::CgalFaceIndex> patch;
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
    const ogun::Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    ogun::CgalSurfaceMesh base = toCgal(bunny);
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
            ogun::CgalSurfaceMesh sm = base;
            std::vector<ogun::CgalFaceIndex> patch;
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
    const ogun::Mesh cracked = ogun::separateTouchingContacts(ogun::SampleMeshes::createCrackedBoxMesh());
    REQUIRE(ogun::checkNonManifoldVertices(cracked).empty()); // the pinch at (2, 2, 4) was split on "load"
    const ogun::CategoryResult open = ogun::checkOpenEdges(cracked);
    REQUIRE(open.size() == 6);
    REQUIRE(open.infoCount == 1); // one slit loop: 4 edges on the left, 2 on the right

    // Every triangle on the slit's collinear rim is degenerate: CGAL finds no fill and changes nothing
    for (const bool cdt : {false, true}) {
        ogun::CgalSurfaceMesh sm = toCgal(cracked);
        std::vector<ogun::CgalFaceIndex> patch;
        PMP::triangulate_hole(sm, borderCycles(sm).front(),
                              CGAL::parameters::face_output_iterator(std::back_inserter(patch)).use_2d_constrained_delaunay_triangulation(cdt));
        REQUIRE(patch.empty());
        REQUIRE(borderCycles(sm).size() == 1);
    }

    // Snapping the rim vertices onto the opposite edges closes it without new faces in the gap
    ogun::CgalSurfaceMesh sm = toCgal(cracked);
    const std::size_t faces = sm.number_of_faces();
    const std::size_t snapped = PMP::experimental::snap_borders(sm);
    const std::size_t stitched = PMP::stitch_borders(sm);
    WARN("snap_borders snapped " << snapped << ", stitched " << stitched << ", faces " << faces << " -> " << sm.number_of_faces());
    sm.collect_garbage();
    REQUIRE(borderCycles(sm).empty());
    const ogun::Mesh closed = ogun::fromCgalSurfaceMesh(sm);
    REQUIRE(ogun::checkOpenEdges(closed).empty());
    REQUIRE(ogun::checkNonManifoldEdges(closed).empty());
    REQUIRE(ogun::signedVolume(closed) == Catch::Approx(64.0));
}

TEST_CASE("Soup cleanup after the Ogun load", "[.holes-timing]") {
    for (const char* name : {"stanford_bunny.stl", "Turkey.stl", "self_intersecting_box.stl"}) {
        const ogun::Mesh mesh = ogun::loadStl(examplePath(name));
        std::vector<ogun::CgalPoint3> points;
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
        ogun::CgalSurfaceMesh sm;
        const bool direct = ogun::toCgalSurfaceMesh(mesh, sm);
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
    const ogun::Mesh holed = sphereCap();
    const ogun::HoleLoop hole = ogun::findHoleLoops(holed).front();
    for (const ogun::HoleFillMethod method : {ogun::HoleFillMethod::Flat, ogun::HoleFillMethod::Refine, ogun::HoleFillMethod::Smooth}) {
        CAPTURE(ogun::holeFillMethodLabel(method));
        const ogun::HoleFillResult r = ogun::fillHole(holed, hole, method);
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
        REQUIRE((method == ogun::HoleFillMethod::Flat) == (r.verticesAdded == 0));
        REQUIRE(r.patchArea > 0.0);
        REQUIRE(r.minNewAngleDeg > 0.0);
    }
}

TEST_CASE("The local patch fill matches CGAL on the whole mesh", "[repair][holes]") {
    const ogun::Mesh holed = sphereCap();
    const ogun::HoleLoop hole = ogun::findHoleLoops(holed).front();
    for (const ogun::HoleFillMethod method : {ogun::HoleFillMethod::Flat, ogun::HoleFillMethod::Refine, ogun::HoleFillMethod::Smooth}) {
        CAPTURE(ogun::holeFillMethodLabel(method));
        ogun::CgalSurfaceMesh sm = toCgal(holed);
        const auto h = borderCycles(sm).front();
        const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true);
        if (method == ogun::HoleFillMethod::Flat) {
            PMP::triangulate_hole(sm, h, np);
        } else if (method == ogun::HoleFillMethod::Refine) {
            PMP::triangulate_and_refine_hole(sm, h, np);
        } else {
            PMP::triangulate_refine_and_fair_hole(sm, h, np);
        }
        const ogun::Mesh whole = ogun::fromCgalSurfaceMesh(sm);
        const ogun::HoleFillResult local = ogun::fillHole(holed, hole, method);
        REQUIRE(local.mesh.numFaces() == whole.numFaces());
        REQUIRE(surfaceDistance(local.mesh, whole) < 1e-9);
    }
}

TEST_CASE("Fill Flat keeps a missing end cap planar; Fill Smooth domes it", "[repair][holes]") {
    const ogun::Mesh open = openCylinder();
    const ogun::HoleLoop hole = ogun::findHoleLoops(open).front();
    REQUIRE(ogun::recommendedHoleFill(hole.shape) == ogun::HoleFillMethod::Flat);
    const ogun::HoleFillResult flat = ogun::fillHole(open, hole, ogun::HoleFillMethod::Flat);
    const ogun::HoleFillResult smooth = ogun::fillHole(open, hole, ogun::HoleFillMethod::Smooth);
    REQUIRE(maxPlaneDistance(flat.mesh, flat.addedFaces, 10.0) < 1e-12);
    REQUIRE(maxPlaneDistance(smooth.mesh, smooth.addedFaces, 10.0) > 1.0);
    const double cylinderVolume = std::numbers::pi * 25.0 * 10.0;
    REQUIRE(ogun::signedVolume(flat.mesh) == Catch::Approx(cylinderVolume).epsilon(0.01)); // 32-gon: -0.6%
    REQUIRE(ogun::signedVolume(smooth.mesh) > ogun::signedVolume(flat.mesh));
}

TEST_CASE("Close Crack seals a T-junction crack that every fill refuses", "[repair][holes]") {
    const ogun::Mesh cracked = ogun::separateTouchingContacts(ogun::SampleMeshes::createCrackedBoxMesh());
    const ogun::HoleLoop hole = ogun::findHoleLoops(cracked).front();
    REQUIRE(ogun::recommendedHoleFill(hole.shape) == ogun::HoleFillMethod::CloseCrack);

    const ogun::HoleFillResult flat = ogun::fillHole(cracked, hole, ogun::HoleFillMethod::Flat);
    REQUIRE_FALSE(flat.success);
    REQUIRE(flat.statusMessage.find("Close Crack") != std::string::npos);
    REQUIRE(flat.mesh.F == cracked.F);

    const ogun::HoleFillResult crack = ogun::fillHole(cracked, hole, ogun::HoleFillMethod::CloseCrack);
    REQUIRE(crack.success);
    REQUIRE(crack.closed());
    REQUIRE(watertight(crack.mesh));
    REQUIRE(ogun::checkNonManifoldVertices(crack.mesh).empty());
    REQUIRE(crack.maxVertexMove < 1e-12);         // the T-vertices already lie on the opposite edges
    REQUIRE(crack.removedFaces.size() == 2);      // the two strip faces split at (2, 1) and (2, 3) ...
    REQUIRE(crack.addedFaces.size() == 4);        // ... into four (renumbered faces are not changes)
    REQUIRE(crack.patchArea == Catch::Approx(2.0));
    const ogun::FaceDiff diff = ogun::faceDiff(cracked, crack.mesh);
    REQUIRE(diff.removed == crack.removedFaces);
    REQUIRE(diff.added.size() == 4);
    REQUIRE(ogun::signedVolume(crack.mesh) == Catch::Approx(64.0));
    REQUIRE(crack.mesh.numVertices() == cracked.numVertices() - 1); // the pinch copies merged again
}

TEST_CASE("Close Crack narrows a gap by at most its width and refuses a real hole", "[repair][holes]") {
    // The strip's copy of (2, 2, 4) pulled 0.01 into the strip: a thin wedge-shaped crack
    ogun::Mesh cracked = ogun::separateTouchingContacts(ogun::SampleMeshes::createCrackedBoxMesh());
    std::vector<Eigen::Index> copies;
    for (Eigen::Index v = 0; v < cracked.numVertices(); ++v) {
        if ((cracked.V.row(v) - Eigen::RowVector3d(2.0, 2.0, 4.0)).norm() == 0.0) {
            copies.push_back(v);
        }
    }
    REQUIRE(copies.size() == 2);
    const ogun::HoleLoop before = ogun::findHoleLoops(cracked).front();
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
    const ogun::HoleLoop hole = ogun::findHoleLoops(cracked).front();
    REQUIRE(hole.shape == ogun::HoleShape::Crack);
    REQUIRE(hole.width > 0.0);
    const ogun::HoleFillResult crack = ogun::fillHole(cracked, hole, ogun::HoleFillMethod::CloseCrack);
    REQUIRE(crack.closed());
    REQUIRE(watertight(crack.mesh));
    REQUIRE(crack.maxVertexMove <= 0.01 + 1e-12);
    REQUIRE(crack.maxVertexMove > 0.0);

    const ogun::Mesh square = flatHoleBox();
    const ogun::HoleFillResult refused = ogun::fillHole(square, ogun::findHoleLoops(square).front(), ogun::HoleFillMethod::CloseCrack);
    REQUIRE_FALSE(refused.success);
    REQUIRE(refused.mesh.V == square.V);
}

TEST_CASE("Hole fills refuse open chains and oversized refined patches", "[repair][holes]") {
    const ogun::Mesh touching = ogun::separateTouchingContacts(ogun::SampleMeshes::createTouchingCubesMesh());
    const ogun::HoleLoop chain = ogun::findHoleLoops(touching).front();
    REQUIRE(chain.shape == ogun::HoleShape::OpenChain);
    REQUIRE(ogun::recommendedHoleFill(chain.shape) == ogun::HoleFillMethod::None);
    const ogun::HoleFillResult r = ogun::fillHole(touching, chain, ogun::HoleFillMethod::Flat);
    REQUIRE_FALSE(r.success);
    REQUIRE(r.statusMessage.find("closed loop") != std::string::npos);

    const ogun::Mesh holed = sphereCap();
    ogun::HoleFillSettings settings;
    settings.maxPatchFaces = 10;
    const ogun::HoleFillResult big = ogun::fillHole(holed, ogun::findHoleLoops(holed).front(), ogun::HoleFillMethod::Smooth, {}, settings);
    REQUIRE_FALSE(big.success);
    REQUIRE(big.statusMessage.find("limit 10") != std::string::npos);
    REQUIRE(big.mesh.F == holed.F);
}

TEST_CASE("A hole preview leaves the mesh alone and offsets the patch", "[repair][holes]") {
    const ogun::Mesh holed = flatHoleBox();
    const ogun::HoleLoop hole = ogun::findHoleLoops(holed).front();
    const ogun::HoleFillPreview preview = ogun::previewHoleFill(holed, hole, ogun::HoleFillMethod::Flat);
    REQUIRE(preview.valid);
    REQUIRE(preview.facesAdded == 6); // an 8-vertex rim
    REQUIRE(preview.verticesAdded == 0);
    REQUIRE(preview.openEdgesLeft == 0);
    REQUIRE(preview.sourceFaceIndices.empty());
    REQUIRE(preview.patchArea == Catch::Approx(4.0));
    REQUIRE(preview.previewPatchMesh.numFaces() == 6);
    REQUIRE(preview.previewPatchMesh.V.col(2).minCoeff() > 6.0); // lifted off the top along +z

    const ogun::HoleFillPreview none = ogun::previewHoleFill(holed, hole, ogun::HoleFillMethod::None);
    REQUIRE_FALSE(none.valid);
}

TEST_CASE("A fill that pokes through another shell reports new crossings", "[repair][holes]") {
    // A plate (wider than the cylinder) just above its open top: the smooth dome rises through it
    ogun::Mesh mesh = openCylinder();
    ogun::Mesh plate = ogun::SampleMeshes::createCubeMesh(14.0);
    plate.V.col(0).array() -= 7.0;
    plate.V.col(1).array() -= 7.0;
    plate.V.col(2) = plate.V.col(2) * (0.2 / 14.0);
    plate.V.col(2).array() += 10.5;
    plate.recomputeNormals();
    ogun::appendMesh(mesh, plate);
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(mesh);
    REQUIRE(holes.size() == 1);
    REQUIRE(ogun::fillHole(mesh, holes.front(), ogun::HoleFillMethod::Flat).newCrossings == 0);
    const ogun::HoleFillResult dome = ogun::fillHole(mesh, holes.front(), ogun::HoleFillMethod::Smooth);
    REQUIRE(dome.newCrossings > 0);
}

TEST_CASE("Filling every hole of the Holes sample makes four closed shells", "[repair][holes]") {
    const ogun::Mesh sample = ogun::separateTouchingContacts(ogun::SampleMeshes::createHolesSampleMesh());
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(sample);
    REQUIRE(holes.size() == 4);
    const ogun::HoleBatchResult r = ogun::fillHoles(sample, holes);
    REQUIRE(r.success);
    REQUIRE(r.holesClosed == 4);
    REQUIRE(r.holesFailed == 0);
    REQUIRE(r.holesSkipped == 0);
    REQUIRE(watertight(r.mesh));
    REQUIRE(ogun::checkShells(r.mesh).infoCount == 4);
    REQUIRE(ogun::checkSelfIntersections(r.mesh, ogun::CheckParams{}).empty());
    REQUIRE(ogun::faceDiff(sample, r.mesh).added.size() == r.facesAdded); // incl. the 4 halves of the 2 split strip faces
    REQUIRE(ogun::faceDiff(sample, r.mesh).removed.size() == 2);

    // A size limit skips the big holes; a forced method applies to every hole
    ogun::HoleFillSettings settings;
    settings.maxHoleEdges = 10;
    const ogun::HoleBatchResult small = ogun::fillHoles(sample, holes, ogun::HoleFillMethod::None, settings);
    REQUIRE(small.holesClosed == 2);
    REQUIRE(small.holesSkipped == 2);
    const ogun::HoleBatchResult flat = ogun::fillHoles(sample, holes, ogun::HoleFillMethod::Flat);
    REQUIRE(flat.holesClosed == 3);
    REQUIRE(flat.holesFailed == 1); // the crack
    REQUIRE(flat.statusMessage.find("Close Crack") != std::string::npos);
}

TEST_CASE("Filling the bunny's holes closes it without new crossings", "[repair][holes]") {
    const ogun::Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(bunny);
    const ogun::HoleBatchResult r = ogun::fillHoles(bunny, holes);
    REQUIRE(r.holesClosed == 5);
    REQUIRE(watertight(r.mesh));
    REQUIRE(ogun::checkShells(r.mesh).infoCount == 1);
    REQUIRE(r.mesh.F.topRows(bunny.numFaces()) == bunny.F);
    const ogun::CategoryResult crossings = ogun::checkSelfIntersections(r.mesh, ogun::CheckParams{});
    for (std::size_t e = 0; e < crossings.size(); ++e) {
        REQUIRE(crossings.facesOf(e)[0] < bunny.numFaces()); // only crossings the bunny already had
    }
    REQUIRE(ogun::signedVolume(r.mesh) > 0.0);
}

TEST_CASE("Repair All Holes: Fill each hole in the shells in scope", "[repair][holes]") {
    const ogun::Mesh sample = ogun::separateTouchingContacts(ogun::SampleMeshes::createHolesSampleMesh());
    REQUIRE(ogun::globalRepairMethodsFor(ogun::DefectCategory::OpenEdges).front() == ogun::GlobalRepairMethod::FillHoles);
    REQUIRE(ogun::globalRepairMethodsFor(ogun::DefectCategory::Intersections).size() == ogun::kGlobalRepairMethods.size());

    const ogun::RepairCandidate all = ogun::runGlobalRepair(sample, {}, ogun::GlobalRepairMethod::FillHoles, ogun::GlobalRepairParams{});
    REQUIRE(all.success);
    REQUIRE(all.metrics.watertight());
    REQUIRE(all.metrics.openEdgesBefore == 6 + 8 + 24 + 24);
    REQUIRE(all.metrics.shellsAfter == 4);
    REQUIRE(all.metrics.intersectingFacesAfter == 0);
    REQUIRE(all.removedFaces.size() == 2);   // the crack's two split strip faces
    REQUIRE(all.metrics.unchangedFraction > 0.99);

    // Only the sphere in scope: the other shells and their holes stay as they are
    const ogun::Mesh sphereOnly = ogun::extractFaces(sample, [&] {
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
    ogun::GlobalRepairParams flat;
    flat.holeMethod = ogun::HoleFillMethod::Flat;
    const ogun::RepairCandidate scoped = ogun::runGlobalRepair(sample, inScope, ogun::GlobalRepairMethod::FillHoles, flat);
    REQUIRE(scoped.success);
    REQUIRE(scoped.metrics.watertight());                              // measured on the scope only
    REQUIRE(ogun::checkOpenEdges(scoped.mesh).size() == 6 + 8 + 24);  // the other three holes are untouched
    REQUIRE(scoped.metrics.facesAfter == sphereOnly.numFaces() + 22); // a flat 24-gon
    REQUIRE_FALSE(flat == ogun::GlobalRepairParams{});
}

TEST_CASE("Repair All Holes on the bunny", "[.holes-timing]") {
    const ogun::Mesh bunny = ogun::loadStl(examplePath("stanford_bunny.stl"));
    for (const ogun::GlobalRepairMethod method : ogun::kHoleRepairMethods) {
        const ogun::RepairCandidate c = ogun::runGlobalRepair(bunny, {}, method, ogun::GlobalRepairParams{});
        WARN(ogun::globalRepairMethodLabel(method) << ": " << c.metrics.elapsedMs << " ms, watertight " << c.metrics.watertight()
                                                   << ", open " << c.metrics.openEdgesAfter << ", dV " << c.metrics.volumeChangePercent()
                                                   << "%, max dev " << c.metrics.maxDeviation << ", lost " << c.metrics.maxLost
                                                   << ", unchanged " << c.metrics.unchangedFraction << ", faces " << c.metrics.facesAfter);
    }
}
