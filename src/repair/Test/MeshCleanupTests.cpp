#include "core/Settings.hpp"
#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/MeshCleanup.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <random>
#include <vector>

namespace {

bool closedManifold(const ogun::Mesh& mesh) {
    return ogun::checkOpenEdges(mesh).empty() && ogun::checkNonManifoldEdges(mesh).empty();
}

/// The grid cube with one vertex of face 0 pulled to within `gap` of another corner of that face.
ogun::Mesh cubeWithShortEdge(double gap) {
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(4, 8.0);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    mesh.V.row(b) = mesh.V.row(a) + gap * (mesh.V.row(b) - mesh.V.row(a)).normalized();
    mesh.recomputeNormals();
    return mesh;
}

std::vector<std::uint8_t> facesAbove(const ogun::Mesh& mesh, double z) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const double cz = (mesh.V(mesh.F(f, 0), 2) + mesh.V(mesh.F(f, 1), 2) + mesh.V(mesh.F(f, 2), 2)) / 3.0;
        mask[static_cast<std::size_t>(f)] = cz > z ? 1 : 0;
    }
    return mask;
}

} // namespace

TEST_CASE("Merge by distance welds a split corner and closes the mesh", "[repair][cleanup][merge]") {
    // Face 0 uses a copy of its first vertex, 0.01 away: two open edges
    ogun::Mesh mesh = ogun::SampleMeshes::createCubeMesh(10.0);
    const int corner = mesh.F(0, 0);
    mesh.V.conservativeResize(mesh.numVertices() + 1, 3);
    mesh.V.row(mesh.numVertices() - 1) = mesh.V.row(corner) + Eigen::RowVector3d(0.01, 0.0, 0.0);
    mesh.F(0, 0) = static_cast<int>(mesh.numVertices() - 1);
    REQUIRE_FALSE(closedManifold(mesh));

    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 1);
    REQUIRE(r.degenerateRemoved == 0);
    REQUIRE(r.maxMove == Catch::Approx(0.01));
    REQUIRE(r.mesh.numVertices() == 8);
    REQUIRE(r.mesh.numFaces() == 12);
    REQUIRE(closedManifold(r.mesh));

    // Too small a distance changes nothing
    const ogun::MergeByDistanceResult none = ogun::mergeByDistance(mesh, 0.005);
    REQUIRE(none.verticesMerged == 0);
    REQUIRE(none.mesh.numFaces() == mesh.numFaces());
}

TEST_CASE("Merge by distance collapses a short edge and removes its two faces", "[repair][cleanup][merge]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    REQUIRE(closedManifold(mesh));
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 1);
    REQUIRE(r.degenerateRemoved == 2);
    REQUIRE(r.duplicatesRemoved == 0);
    REQUIRE(r.maxMove <= 0.05);
    REQUIRE(r.mesh.numFaces() == mesh.numFaces() - 2);
    REQUIRE(closedManifold(r.mesh));
}

TEST_CASE("Merge by distance only touches vertices of the faces in scope", "[repair][cleanup][merge]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(mesh.numFaces()), 1);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (mesh.F(f, c) == a || mesh.F(f, c) == b) {
                mask[static_cast<std::size_t>(f)] = 0;
            }
        }
    }
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, 0.05, mask);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 0);
    REQUIRE(r.mesh.V == mesh.V);
    REQUIRE(r.mesh.F == mesh.F);
}

TEST_CASE("Merge by distance never moves a vertex more than the distance", "[repair][cleanup][merge]") {
    // A fine sphere with a large distance: the clusters are centred on their first vertex, not chained
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(32, 48, 5.0);
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(sphere, 0.6);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged > 0);
    REQUIRE(r.maxMove <= 0.6);
    REQUIRE(r.mesh.V.rowwise().norm().maxCoeff() == Catch::Approx(5.0)); // kept vertices never move
}

TEST_CASE("Decimate halves a sphere and keeps it closed", "[repair][cleanup][decimate]") {
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(32, 48, 5.0);
    const ogun::DecimateResult r = ogun::decimate(sphere, 0.5);
    REQUIRE(r.success);
    REQUIRE(r.facesBefore == static_cast<std::size_t>(sphere.numFaces()));
    REQUIRE(static_cast<double>(r.facesAfter) == Catch::Approx(0.5 * static_cast<double>(sphere.numFaces())).epsilon(0.02));
    REQUIRE(r.mesh.numFaces() == static_cast<Eigen::Index>(r.facesAfter));
    REQUIRE(closedManifold(r.mesh));
    REQUIRE(ogun::vertexDeviation(r.mesh, sphere).maxCoeff() < 0.1);

    const ogun::DecimateResult same = ogun::decimate(sphere, 1.0);
    REQUIRE(same.mesh.numFaces() == sphere.numFaces());
}

TEST_CASE("Decimating selected faces keeps every other face and joins them exactly", "[repair][cleanup][decimate]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(8, 8.0);
    const std::vector<std::uint8_t> top = facesAbove(mesh, 7.99); // the top side only
    const auto topCount = static_cast<std::size_t>(std::count(top.begin(), top.end(), 1));
    REQUIRE(topCount == 128);
    const ogun::DecimateResult r = ogun::decimate(mesh, 0.25, top);
    REQUIRE(r.success);
    REQUIRE(r.facesBefore == topCount);
    REQUIRE(r.facesAfter < topCount);
    REQUIRE(r.mesh.numFaces() == mesh.numFaces() - static_cast<Eigen::Index>(topCount - r.facesAfter));
    REQUIRE(closedManifold(r.mesh));
    // Only top faces changed; the flat top stays flat
    const ogun::FaceDiff diff = ogun::faceDiff(mesh, r.mesh);
    for (const Eigen::Index f : diff.removed) {
        REQUIRE(top[static_cast<std::size_t>(f)] == 1);
    }
    REQUIRE(ogun::vertexDeviation(r.mesh, mesh).maxCoeff() < 1e-9);
}

TEST_CASE("Decimate and merge run as one-method operators with the merge distance in file units", "[repair][cleanup][global]") {
    // A centimetre mesh: the distance is 0.05 in file coordinates, so a 0.04 edge collapses and a 0.06 one does not
    ogun::Mesh mesh = cubeWithShortEdge(0.04);
    mesh.fileUnit = ogun::Unit::Centimeter;
    ogun::GlobalRepairParams params;
    REQUIRE(params.mergeDistance == 0.05);
    REQUIRE(params.decimateRatio == 0.5);
    const ogun::RepairCandidate merged = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(merged.success);
    REQUIRE(merged.mesh.numFaces() == mesh.numFaces() - 2);
    REQUIRE(merged.removedFaces.size() >= 2);
    REQUIRE(merged.metrics.watertight());

    ogun::Mesh wider = cubeWithShortEdge(0.06);
    wider.fileUnit = ogun::Unit::Centimeter;
    const ogun::RepairCandidate kept = ogun::runGlobalRepair(wider, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(kept.success);
    REQUIRE(kept.mesh.numFaces() == wider.numFaces());
    REQUIRE(kept.changedFaces.empty());

    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const ogun::RepairCandidate decimated = ogun::runGlobalRepair(sphere, {}, ogun::GlobalRepairMethod::Decimate, params);
    REQUIRE(decimated.success);
    REQUIRE(decimated.metrics.facesAfter < decimated.metrics.facesBefore);
    REQUIRE(decimated.metrics.watertight());
    REQUIRE(decimated.vertexDeviation.size() == decimated.mesh.numVertices());
}

TEST_CASE("Merge keeps touching surfaces apart unless asked", "[repair][merge]") {
    // Cube B's corner (0, 0, 0) sits 0.02 mm from cube A's corner (8, 8, 8): no edge joins them
    ogun::Mesh mesh = ogun::SampleMeshes::createGridCubeMesh(2, 8.0);
    ogun::Mesh other = mesh;
    other.V.rowwise() += Eigen::RowVector3d(8.02, 8.0, 8.0);
    ogun::appendMesh(mesh, other);
    mesh.recomputeNormals();

    ogun::GlobalRepairParams params;
    REQUIRE(params.mergeKeepSurfacesApart);
    const ogun::RepairCandidate apart = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(apart.success);
    REQUIRE(apart.changedFaces.empty());
    REQUIRE(apart.mesh.numVertices() == mesh.numVertices());

    params.mergeKeepSurfacesApart = false;
    const ogun::RepairCandidate glued = ogun::runGlobalRepair(mesh, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(glued.success);
    REQUIRE(glued.mesh.numVertices() == mesh.numVertices() - 1); // the two shells now share a vertex
}

TEST_CASE("Keeping surfaces apart still welds cracks and collapses short edges", "[repair][merge]") {
    // Two open unit squares with a 0.02 crack between them: the crack's four vertices are on open edges
    ogun::Mesh sheets;
    sheets.V.resize(8, 3);
    sheets.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1.02, 0, 0, 2, 0, 0, 2, 1, 0, 1.02, 1, 0;
    sheets.F.resize(4, 3);
    sheets.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    sheets.recomputeNormals();
    const ogun::MergeByDistanceResult crack = ogun::mergeByDistance(sheets, 0.05, {}, true);
    REQUIRE(crack.success);
    REQUIRE(crack.verticesMerged == 2);
    REQUIRE(crack.mesh.numVertices() == 6);

    const ogun::Mesh shortEdge = cubeWithShortEdge(0.02);
    const ogun::MergeByDistanceResult collapsed = ogun::mergeByDistance(shortEdge, 0.05, {}, true);
    REQUIRE(collapsed.verticesMerged == 1);
    REQUIRE(collapsed.degenerateRemoved == 2);
}

TEST_CASE("Merge by distance preserves open edges and cracks when requested", "[repair][merge]") {
    ogun::Mesh sheets;
    sheets.V.resize(8, 3);
    sheets.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1.02, 0, 0, 2, 0, 0, 2, 1, 0, 1.02, 1, 0;
    sheets.F.resize(4, 3);
    sheets.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    sheets.recomputeNormals();

    // With preserveOpenEdges = true (the UI default), the crack across open edges is not merged
    const ogun::MergeByDistanceResult preserved = ogun::mergeByDistance(sheets, 0.05, {}, true, {}, true);
    REQUIRE(preserved.success);
    REQUIRE(preserved.verticesMerged == 0);
    REQUIRE(preserved.mesh.numVertices() == 8);

    // GlobalRepair defaults to mergePreserveOpenEdges = true
    ogun::GlobalRepairParams params;
    REQUIRE(params.mergePreserveOpenEdges);
    const ogun::RepairCandidate candidate = ogun::runGlobalRepair(sheets, {}, ogun::GlobalRepairMethod::MergeVertices, params);
    REQUIRE(candidate.success);
    REQUIRE(candidate.mesh.numVertices() == 8);

    // Short edge collapse still works when preserving open edges because an edge connects the vertices
    const ogun::Mesh shortEdge = cubeWithShortEdge(0.02);
    const ogun::MergeByDistanceResult collapsed = ogun::mergeByDistance(shortEdge, 0.05, {}, true, {}, true);
    REQUIRE(collapsed.verticesMerged == 1);
    REQUIRE(collapsed.degenerateRemoved == 2);
}

TEST_CASE("weldOpenEdges welds open seams and ignores interior edges", "[repair][weld]") {
    ogun::Mesh sheets;
    sheets.V.resize(8, 3);
    sheets.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1.02, 0, 0, 2, 0, 0, 2, 1, 0, 1.02, 1, 0;
    sheets.F.resize(4, 3);
    sheets.F << 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7;
    sheets.recomputeNormals();

    const ogun::MergeByDistanceResult welded = ogun::weldOpenEdges(sheets, 0.05);
    REQUIRE(welded.success);
    REQUIRE(welded.verticesMerged == 2);
    REQUIRE(welded.mesh.numVertices() == 6);

    // Internal edges on closed meshes are completely untouched by weldOpenEdges
    const ogun::Mesh shortEdge = cubeWithShortEdge(0.02);
    const ogun::MergeByDistanceResult untouched = ogun::weldOpenEdges(shortEdge, 0.05);
    REQUIRE(untouched.success);
    REQUIRE(untouched.verticesMerged == 0);
    REQUIRE(untouched.mesh.numFaces() == shortEdge.numFaces());

    // Via GlobalRepair
    ogun::GlobalRepairParams params;
    params.weldDistance = 0.05;
    const ogun::RepairCandidate rep = ogun::runGlobalRepair(sheets, {}, ogun::GlobalRepairMethod::Weld, params);
    REQUIRE(rep.success);
    REQUIRE(rep.mesh.numVertices() == 6);
}

TEST_CASE("Decimate stops at the target face count, not one collapse below it", "[repair][decimate]") {
    const ogun::Mesh box = ogun::SampleMeshes::createGridCubeMesh(1, 10.0); // 12 faces
    REQUIRE(box.numFaces() == 12);
    const ogun::DecimateResult r = ogun::decimate(box, 0.5);
    REQUIRE(r.success);
    REQUIRE(r.facesAfter >= 6);
    const ogun::Mesh sphere = ogun::SampleMeshes::createSphereMesh(16, 24, 5.0);
    const ogun::DecimateResult s = ogun::decimate(sphere, 0.3);
    REQUIRE(s.facesAfter >= static_cast<std::size_t>(std::lround(0.3 * static_cast<double>(sphere.numFaces()))));
}

TEST_CASE("Merge distance is in file coordinates whatever the file unit", "[repair][merge][units]") {
    for (const ogun::Unit unit : {ogun::Unit::Millimeter, ogun::Unit::Centimeter, ogun::Unit::Meter}) {
        ogun::GlobalRepairParams params;
        params.mergeDistance = 0.05;

        ogun::Mesh closer = cubeWithShortEdge(0.04);
        closer.fileUnit = unit;
        const ogun::RepairCandidate merged = ogun::runGlobalRepair(closer, {}, ogun::GlobalRepairMethod::MergeVertices, params);
        REQUIRE(merged.success);
        REQUIRE(merged.mesh.numFaces() == closer.numFaces() - 2);

        ogun::Mesh farther = cubeWithShortEdge(0.06);
        farther.fileUnit = unit;
        const ogun::RepairCandidate kept = ogun::runGlobalRepair(farther, {}, ogun::GlobalRepairMethod::MergeVertices, params);
        REQUIRE(kept.success);
        REQUIRE(kept.mesh.numFaces() == farther.numFaces());
    }
}

// -----------------------------------------------------------------------------
// Coordinate-relative weld (the ToleranceConfig form, Fix All's weld)
// -----------------------------------------------------------------------------

namespace {

const ogun::ToleranceConfig kFixAllWeld{ogun::settings::kDefaultFixAllWeldAbsFloor, ogun::settings::kDefaultFixAllWeldRelTol};

ogun::Mesh meshOf(const std::vector<Eigen::Vector3d>& points, const std::vector<std::array<int, 3>>& faces) {
    ogun::Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Every input vertex still used moved at most its own per-axis tolerance on each axis.
bool movesWithinTolerance(const ogun::Mesh& in, const ogun::MergeByDistanceResult& r, const ogun::ToleranceConfig& tol) {
    for (Eigen::Index v = 0; v < in.numVertices(); ++v) {
        const Eigen::Index to = r.vertexMap[static_cast<std::size_t>(v)];
        if (to < 0) {
            continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const double p = in.V(v, axis);
            const double q = r.mesh.V(to, axis);
            if (std::abs(p - q) > tol.coordinateTolerance(p, q)) {
                return false;
            }
        }
    }
    return true;
}

bool hasRow(const ogun::Mesh& mesh, const Eigen::RowVector3d& p) {
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        if (mesh.V.row(v) == p) {
            return true;
        }
    }
    return false;
}

std::size_t shellCount(const ogun::Mesh& mesh) { return ogun::shellsLargestFirst(mesh).size(); }

} // namespace

TEST_CASE("Fix All's weld tolerance is its own, separate from the load weld", "[repair][merge][weld]") {
    const ogun::ToleranceConfig load{};
    REQUIRE(load.absFloor == 1.0e-9);
    REQUIRE(load.relTol == 1.0e-9);
    REQUIRE(kFixAllWeld.absFloor == 1.0e-6);
    REQUIRE(kFixAllWeld.relTol == 1.0e-6);
}

TEST_CASE("The coordinate-relative weld decides every pair as ToleranceConfig does", "[repair][merge][weld]") {
    std::mt19937_64 rng(20261029);
    std::uniform_real_distribution<double> exponent(-3.0, 9.0);
    std::uniform_real_distribution<double> unit(0.5, 1.0);
    std::uniform_real_distribution<double> factor(0.0, 2.0);
    std::bernoulli_distribution coin(0.5);
    std::size_t merged = 0;
    constexpr int kTrials = 2000;
    for (int trial = 0; trial < kTrials; ++trial) {
        Eigen::Vector3d p;
        Eigen::Vector3d q;
        for (int axis = 0; axis < 3; ++axis) {
            p[axis] = (coin(rng) ? 1.0 : -1.0) * std::pow(10.0, exponent(rng)) * unit(rng);
            const double t = kFixAllWeld.coordinateTolerance(p[axis], p[axis]);
            // Mostly just inside or just outside the tolerance, so both answers and the edge are tested
            const double f = trial % 4 == 0 ? factor(rng) : (coin(rng) ? 0.999 : 1.001) * (trial % 3 == 0 ? 1.0 : 0.6);
            q[axis] = p[axis] + (coin(rng) ? f : -f) * t;
        }
        const Eigen::Vector3d far = p + 10.0 * (p.cwiseAbs() + Eigen::Vector3d::Ones());
        const ogun::Mesh mesh = meshOf({p, q, far}, {{0, 1, 2}});
        const bool expected = kFixAllWeld.verticesMatchWithinTolerance(p, q);
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld);
        REQUIRE(r.success);
        CAPTURE(trial, p.transpose(), q.transpose());
        REQUIRE(r.verticesMerged == (expected ? 1u : 0u));
        merged += r.verticesMerged;
    }
    REQUIRE(merged > kTrials / 5);
    REQUIRE(merged < kTrials - kTrials / 5);
}

TEST_CASE("A pair at the tolerance on every axis merges and one just over on one axis does not", "[repair][merge][weld]") {
    // Binary values so every difference and tolerance is exact: tolerance 2^-20 x 0.5 on each axis
    const ogun::ToleranceConfig tol{0.0, std::ldexp(1.0, -20)};
    const Eigen::Vector3d p(0.5, 0.5, 0.5);
    const double t = tol.coordinateTolerance(0.5, 0.5);
    const Eigen::Vector3d atTolerance = p - Eigen::Vector3d::Constant(t);
    const Eigen::Vector3d far(4.0, 4.0, 4.0);
    const ogun::MergeByDistanceResult at = ogun::mergeByDistance(meshOf({p, atTolerance, far}, {{0, 1, 2}}), tol);
    REQUIRE(at.verticesMerged == 1);
    REQUIRE(at.maxMove == Catch::Approx(std::sqrt(3.0) * t)); // a box, not a ball
    REQUIRE(at.maxTolerance == Catch::Approx(4.0 * std::ldexp(1.0, -20)));

    Eigen::Vector3d over = atTolerance;
    over.y() = p.y() - t * (1.0 + std::ldexp(1.0, -10));
    const ogun::MergeByDistanceResult apart = ogun::mergeByDistance(meshOf({p, over, far}, {{0, 1, 2}}), tol);
    REQUIRE(apart.success);
    REQUIRE(apart.verticesMerged == 0);
}

TEST_CASE("A near pair and a far pair with the same offset: only the far one merges", "[repair][merge][weld]") {
    // Grid cells are about 1e-2 wide here, far wider than the near pair's 1e-6 tolerance: still exact
    constexpr double kOffset = 5.0e-3;
    std::vector<Eigen::Vector3d> points;
    std::vector<std::array<int, 3>> faces;
    for (const double x0 : {0.0, 1.0e4}) {
        const int base = static_cast<int>(points.size());
        for (const double dx : {0.0, kOffset}) {
            points.emplace_back(x0 + dx, 0.0, 0.0);
            points.emplace_back(x0 + dx + 1.0, 0.0, 0.0);
            points.emplace_back(x0 + dx, 1.0, 0.0);
        }
        faces.push_back({base, base + 1, base + 2});
        faces.push_back({base + 3, base + 4, base + 5});
    }
    const ogun::Mesh mesh = meshOf(points, faces);
    const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld);
    REQUIRE(r.success);
    REQUIRE(r.verticesMerged == 3);    // the far copy's three corners
    REQUIRE(r.duplicatesRemoved == 1); // the far copy itself
    REQUIRE(r.faceMap == std::vector<Eigen::Index>{0, 1, 2, -1});
    REQUIRE(r.mesh.V.topRows(6) == mesh.V.topRows(6)); // near triangles untouched
    REQUIRE(r.maxTolerance == Catch::Approx(1.0e-6 * (1.0e4 + 1.0 + kOffset)));
    REQUIRE(movesWithinTolerance(mesh, r, kFixAllWeld));
}

TEST_CASE("The weld handles huge coordinates and skips non-finite vertices", "[repair][merge][weld]") {
    SECTION("1e15 does not overflow the cell index") {
        const Eigen::Vector3d p(1.0e15, -1.0e15, 1.0e15);
        const ogun::Mesh mesh = meshOf({p, p + Eigen::Vector3d(5.0e8, 0.0, 0.0), p + Eigen::Vector3d(1.0e12, 1.0e12, 0.0)},
                                       {{0, 1, 2}});
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 1); // the tolerance is 1e9 there
        REQUIRE(r.degenerateRemoved == 1);

        // The straight-line form at 1e18 with a small distance (cell index 2e19 before widening)
        const Eigen::Vector3d s(1.0e18, 1.0e18, -1.0e18);
        const ogun::Mesh huge = meshOf({s, s + Eigen::Vector3d(128.0, 0.0, 0.0), s + Eigen::Vector3d(0.0, 1.0e12, 0.0)}, {{0, 1, 2}});
        const ogun::MergeByDistanceResult line = ogun::mergeByDistance(huge, 200.0);
        REQUIRE(line.success);
        REQUIRE(line.verticesMerged == 1);
    }
    SECTION("NaN and infinite vertices are never merged") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        const ogun::Mesh mesh = meshOf({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {nan, 0.0, 0.0}, // face 0: a NaN corner
                                        {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {inf, 1.0, 0.0},  // face 1: an infinite corner
                                        {0.0, 0.0, 5.0e-7}, {1.0, 0.0, 5.0e-7}, {0.0, 0.0, 9.0}}, // face 2: near face 0
                                       {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}});
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 2); // 6 -> 0 and 7 -> 1
        REQUIRE(r.mesh.numFaces() == 3);
        REQUIRE(r.vertexMap[6] == r.vertexMap[0]);
        REQUIRE(std::isnan(r.mesh.V(r.vertexMap[2], 0)));

        const ogun::MergeByDistanceResult line = ogun::mergeByDistance(mesh, 1.0e-6);
        REQUIRE(line.success);
        REQUIRE(line.verticesMerged == 2);
    }
}

TEST_CASE("Pinned vertices absorb others but never move, in both weld forms", "[repair][merge][weld]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    const ogun::ToleranceConfig box{0.02, 0.0}; // a 0.02 box: only the short edge's ends match
    for (const int keep : {a, b}) {
        const int moved = keep == a ? b : a;
        std::vector<std::uint8_t> pinned(static_cast<std::size_t>(mesh.numVertices()), 0);
        pinned[static_cast<std::size_t>(keep)] = 1;
        for (const bool relative : {false, true}) {
            CAPTURE(keep, relative);
            const ogun::MergeByDistanceResult r =
                relative ? ogun::mergeByDistance(mesh, box, {}, false, pinned) : ogun::mergeByDistance(mesh, 0.05, {}, false, pinned);
            REQUIRE(r.success);
            REQUIRE(r.verticesMerged == 1);
            REQUIRE(r.degenerateRemoved == 2);
            REQUIRE(hasRow(r.mesh, mesh.V.row(keep)));
            REQUIRE_FALSE(hasRow(r.mesh, mesh.V.row(moved)));
            REQUIRE(r.vertexMap[static_cast<std::size_t>(moved)] == r.vertexMap[static_cast<std::size_t>(keep)]);
            REQUIRE(std::count(r.faceMap.begin(), r.faceMap.end(), -1) == 2);
        }
    }
    // Both ends pinned: nothing merges, and the pair is counted
    std::vector<std::uint8_t> both(static_cast<std::size_t>(mesh.numVertices()), 0);
    both[static_cast<std::size_t>(a)] = 1;
    both[static_cast<std::size_t>(b)] = 1;
    for (const bool relative : {false, true}) {
        const ogun::MergeByDistanceResult r =
            relative ? ogun::mergeByDistance(mesh, box, {}, false, both) : ogun::mergeByDistance(mesh, 0.05, {}, false, both);
        REQUIRE(r.verticesMerged == 0);
        REQUIRE(r.pinnedPairsKept == 1);
        REQUIRE(r.mesh.V == mesh.V);
        REQUIRE(r.mesh.F == mesh.F);
    }
    // A mask of the wrong size is refused
    const std::vector<std::uint8_t> shortMask(3, 1);
    REQUIRE_FALSE(ogun::mergeByDistance(mesh, box, {}, false, shortMask).success);
}

TEST_CASE("The coordinate-relative weld keeps faces outside the scope bit-identical", "[repair][merge][weld]") {
    const ogun::Mesh mesh = cubeWithShortEdge(0.01);
    const int a = mesh.F(0, 0);
    const int b = mesh.F(0, 1);
    const ogun::ToleranceConfig box{0.02, 0.0};
    const auto uses = [&](Eigen::Index f, int v) { return mesh.F(f, 0) == v || mesh.F(f, 1) == v || mesh.F(f, 2) == v; };
    const auto faceUsingOnly = [&](int v, int notV) {
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            if (uses(f, v) && !uses(f, notV)) {
                return f;
            }
        }
        return Eigen::Index{-1};
    };
    for (const int outsideVertex : {a, b}) {
        const Eigen::Index outside = faceUsingOnly(outsideVertex, outsideVertex == a ? b : a);
        REQUIRE(outside >= 0);
        std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 1);
        scope[static_cast<std::size_t>(outside)] = 0;
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, box, scope);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 1); // the vertex shared with the outside face is pinned and absorbs the other
        const Eigen::Index to = r.faceMap[static_cast<std::size_t>(outside)];
        REQUIRE(to >= 0);
        for (int c = 0; c < 3; ++c) {
            REQUIRE(r.mesh.V.row(r.mesh.F(to, c)) == mesh.V.row(mesh.F(outside, c)));
        }
    }
    // The straight-line form (Weld / Merge Vertices) keeps today's rule: the outside face's vertex can move
    for (const int outsideVertex : {a, b}) {
        const Eigen::Index outside = faceUsingOnly(outsideVertex, outsideVertex == a ? b : a);
        std::vector<std::uint8_t> scope(static_cast<std::size_t>(mesh.numFaces()), 1);
        scope[static_cast<std::size_t>(outside)] = 0;
        REQUIRE(ogun::mergeByDistance(mesh, 0.05, scope).verticesMerged == 1);
    }
}

TEST_CASE("The coordinate-relative weld never removes a face outside the scope as a copy", "[repair][merge][weld]") {
    // Face 0 (in scope) lies 5e-7 from face 1 (outside the scope), wound the other way; faces 2 to 4 (in
    // scope) share face 1's corners, so those corners are candidates, pinned, and absorb face 0's
    const std::vector<Eigen::Vector3d> points{
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},                // 0-2: face 1
        {0.0, 0.0, 5.0e-7}, {1.0, 0.0, 5.0e-7}, {0.0, 1.0, 5.0e-7},       // 3-5: face 0
        {-5.0, 0.0, 0.0}, {-5.0, -1.0, 0.0}, {6.0, 0.0, 0.0},             // helpers
        {6.0, -1.0, 0.0}, {0.0, 6.0, 0.0}, {-1.0, 6.0, 0.0}};
    for (const bool sameWay : {false, true}) {
        const std::array<int, 3> copy = sameWay ? std::array<int, 3>{3, 4, 5} : std::array<int, 3>{3, 5, 4};
        const ogun::Mesh mesh = meshOf(points, {copy, {0, 1, 2}, {0, 6, 7}, {1, 8, 9}, {2, 10, 11}});
        const std::vector<std::uint8_t> scope{1, 0, 1, 1, 1};
        CAPTURE(sameWay);
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld, scope);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 3);
        REQUIRE(r.duplicatesRemoved == 1);
        REQUIRE(r.faceMap == std::vector<Eigen::Index>{-1, 0, 1, 2, 3});
        REQUIRE(r.mesh.V.row(r.mesh.F(0, 0)) == mesh.V.row(0));

        // The straight-line form keeps the first copy, or removes both copies of a fold
        const ogun::MergeByDistanceResult line = ogun::mergeByDistance(mesh, 1.0e-6, scope);
        REQUIRE(line.faceMap[1] == -1);
        REQUIRE(line.faceMap[0] == (sameWay ? 0 : -1));
    }
}

TEST_CASE("Fix All's weld closes a hairline seam and leaves wider cracks", "[repair][merge][weld][fixture]") {
    const auto seamClosed = [](const ogun::Mesh& mesh) {
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld, {}, true);
        REQUIRE(r.success);
        REQUIRE(movesWithinTolerance(mesh, r, kFixAllWeld));
        const bool closed = r.verticesMerged == 4 && r.mesh.numFaces() == 20 && closedManifold(r.mesh) && shellCount(r.mesh) == 1;
        REQUIRE((closed || r.verticesMerged == 0));
        return closed;
    };

    const ogun::Mesh seam = ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    REQUIRE(seam.numFaces() == 20);
    REQUIRE(shellCount(seam) == 2);
    REQUIRE(ogun::checkOpenEdges(seam).infoCount == 8); // two 4-edge loops
    REQUIRE(seamClosed(seam));
    REQUIRE_FALSE(seamClosed(ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-5))); // left to Close Crack
    // The load weld's tolerance (1e-9 relative) would not close it
    REQUIRE_FALSE(ogun::mergeByDistance(seam, ogun::ToleranceConfig{}, {}, true).verticesMerged > 0);

    SECTION("(a) scaled by 1e4 about the origin: the same faces") {
        const ogun::Mesh scaled = ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-6, Eigen::Vector3d::Zero(), 1.0e4);
        REQUIRE(seamClosed(scaled));
        const ogun::MergeByDistanceResult original = ogun::mergeByDistance(seam, kFixAllWeld, {}, true);
        const ogun::MergeByDistanceResult big = ogun::mergeByDistance(scaled, kFixAllWeld, {}, true);
        REQUIRE(big.mesh.F == original.mesh.F);
        REQUIRE(big.faceMap == original.faceMap);
    }
    SECTION("(b) placement: a 5e-3 seam welds at 1e4, not at the original place") {
        const Eigen::Vector3d shift = Eigen::Vector3d::Constant(1.0e4 - 10.0);
        REQUIRE(seamClosed(ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-3, shift)));
        REQUIRE_FALSE(seamClosed(ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-3)));
    }
    SECTION("(c) below 1 file unit the floor applies") {
        const Eigen::Vector3d lo = Eigen::Vector3d::Zero();
        const Eigen::Vector3d hi = Eigen::Vector3d::Constant(1.0e-3);
        REQUIRE(seamClosed(ogun::SampleMeshes::createSeamBoxMesh(lo, hi, 0, 5.0e-7)));
        REQUIRE_FALSE(seamClosed(ogun::SampleMeshes::createSeamBoxMesh(lo, hi, 0, 5.0e-6)));
    }
    SECTION("(d) anisotropy: at x = 1000 an x seam welds, a y seam of the same width does not") {
        const Eigen::Vector3d lo(1000.0, 0.0, 0.0);
        const Eigen::Vector3d hi(1001.0, 0.5, 0.5);
        REQUIRE(seamClosed(ogun::SampleMeshes::createSeamBoxMesh(lo, hi, 0, 5.0e-4)));
        REQUIRE_FALSE(seamClosed(ogun::SampleMeshes::createSeamBoxMesh(lo, hi, 1, 5.0e-4)));
    }
    SECTION("(e) 0.9 x the tolerance on all three axes welds, moving about 1.56 x it") {
        const Eigen::Vector3d corner(0.1, 0.2, 0.3);
        const Eigen::Vector3d near = corner + Eigen::Vector3d::Constant(0.9e-6);
        const ogun::Mesh sheets = meshOf({corner, {0.5, 0.2, 0.3}, {0.1, 0.6, 0.3}, near, {-0.3, 0.2, 0.3}, {0.1, -0.2, 0.3}},
                                         {{0, 1, 2}, {3, 4, 5}});
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(sheets, kFixAllWeld, {}, true);
        REQUIRE(r.verticesMerged == 1);
        REQUIRE(r.maxMove / 1.0e-6 == Catch::Approx(0.9 * std::sqrt(3.0)).epsilon(1.0e-6));
    }
    SECTION("(f) two closed cubes within the tolerance are not welded together") {
        const ogun::Mesh cubes = ogun::SampleMeshes::combineMeshes(
            {ogun::SampleMeshes::createBox({10.0, 10.0, 10.0}, {20.0, 20.0, 20.0}),
             ogun::SampleMeshes::createBox({20.0 + 5.0e-6, 10.0, 10.0}, {30.0, 20.0, 20.0})});
        const ogun::MergeByDistanceResult apart = ogun::mergeByDistance(cubes, kFixAllWeld, {}, true);
        REQUIRE(apart.verticesMerged == 0);
        REQUIRE(shellCount(apart.mesh) == 2);
        REQUIRE(ogun::mergeByDistance(cubes, kFixAllWeld, {}, false).verticesMerged == 4); // only Keep surfaces apart stops it
    }
    SECTION("(g) touching cubes and touching contacts are not pinched") {
        const ogun::Mesh touching = ogun::separateTouchingContacts(ogun::SampleMeshes::createTouchingCubesMesh());
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(touching, kFixAllWeld, {}, true);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 0);
        REQUIRE(shellCount(r.mesh) == shellCount(touching));
        REQUIRE(ogun::checkNonManifoldEdges(r.mesh).size() == ogun::checkNonManifoldEdges(touching).size());
    }
}

TEST_CASE("Pinned seam vertices stay put while the other side welds onto them", "[repair][merge][weld][fixture]") {
    const ogun::Mesh seam = ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    std::vector<std::uint8_t> left(static_cast<std::size_t>(seam.numVertices()), 0);
    std::vector<std::uint8_t> right(left.size(), 0);
    for (Eigen::Index v = 0; v < seam.numVertices(); ++v) {
        if (seam.V(v, 0) == 15.0) {
            left[static_cast<std::size_t>(v)] = 1;
        } else if (std::abs(seam.V(v, 0) - 15.0) < 1.0e-4) {
            right[static_cast<std::size_t>(v)] = 1;
        }
    }
    REQUIRE(std::count(left.begin(), left.end(), 1) == 4);
    REQUIRE(std::count(right.begin(), right.end(), 1) == 4);
    for (const auto* pins : {&left, &right}) {
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(seam, kFixAllWeld, {}, true, *pins);
        REQUIRE(r.verticesMerged == 4);
        REQUIRE(closedManifold(r.mesh));
        for (Eigen::Index v = 0; v < seam.numVertices(); ++v) {
            if ((*pins)[static_cast<std::size_t>(v)] != 0) {
                REQUIRE(r.mesh.V.row(r.vertexMap[static_cast<std::size_t>(v)]) == seam.V.row(v));
            }
        }
    }
    std::vector<std::uint8_t> both = left;
    for (std::size_t v = 0; v < both.size(); ++v) {
        both[v] = static_cast<std::uint8_t>(both[v] | right[v]);
    }
    const ogun::MergeByDistanceResult none = ogun::mergeByDistance(seam, kFixAllWeld, {}, true, both);
    REQUIRE(none.verticesMerged == 0);
    REQUIRE(none.pinnedPairsKept == 4);
}

TEST_CASE("A seam along the scope's edge closes onto the side outside the scope", "[repair][merge][weld][fixture]") {
    // Only one half is in scope: the other half's open-edge vertices are used by no in-scope face, yet they
    // absorb the in-scope side, so the seam closes and the outside half stays bit-identical
    const ogun::Mesh seam = ogun::SampleMeshes::createWeldSeamCubeMesh(5.0e-6);
    for (const bool lowerInScope : {true, false}) {
        CAPTURE(lowerInScope);
        std::vector<std::uint8_t> scope(static_cast<std::size_t>(seam.numFaces()), 0);
        for (Eigen::Index f = 0; f < seam.numFaces(); ++f) {
            const double x = (seam.V(seam.F(f, 0), 0) + seam.V(seam.F(f, 1), 0) + seam.V(seam.F(f, 2), 0)) / 3.0;
            scope[static_cast<std::size_t>(f)] = (x < 15.0) == lowerInScope ? 1 : 0;
        }
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(seam, kFixAllWeld, scope, true);
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 4);
        REQUIRE(closedManifold(r.mesh));
        for (Eigen::Index f = 0; f < seam.numFaces(); ++f) {
            if (scope[static_cast<std::size_t>(f)] == 0) {
                const Eigen::Index to = r.faceMap[static_cast<std::size_t>(f)];
                REQUIRE(to >= 0);
                for (int c = 0; c < 3; ++c) {
                    REQUIRE(r.mesh.V.row(r.mesh.F(to, c)) == seam.V.row(seam.F(f, c)));
                }
            }
        }
        // Nothing in scope: the outside sides never weld to each other
        const std::vector<std::uint8_t> nothing(scope.size(), 0);
        REQUIRE(ogun::mergeByDistance(seam, kFixAllWeld, nothing, true).verticesMerged == 0);
    }
}

TEST_CASE("Cube with a visible crack: clean single-shell mesh with only a crack defect, healed by weld", "[repair][cleanup][weld][crack]") {
    const ogun::Mesh mesh = ogun::SampleMeshes::createVisibleCrackCubeMesh(0.04);

    // Verify diagnostics: exactly 1 shell, 6 open edges, and zero other defects
    CHECK(shellCount(mesh) == 1);
    CHECK(ogun::checkOpenEdges(mesh).infoCount == 6);
    CHECK(ogun::checkOpenEdges(mesh).size() == 1);
    CHECK(ogun::checkNonManifoldEdges(mesh).empty());
    CHECK(ogun::checkNonManifoldVertices(mesh).empty());
    CHECK(ogun::checkSelfIntersections(mesh, ogun::CheckParams{}).empty());

    // Hole loops: correctly classified as a Crack
    const std::vector<ogun::HoleLoop> holes = ogun::findHoleLoops(mesh);
    REQUIRE(holes.size() == 1);
    CHECK(holes[0].shape == ogun::HoleShape::Crack);
    CHECK(holes[0].edges.size() == 6);

    // 1. weldOpenEdges with default distance (0.05 mm) closes the crack completely
    const ogun::MergeByDistanceResult welded = ogun::weldOpenEdges(mesh, 0.05);
    REQUIRE(welded.success);
    CHECK(welded.verticesMerged == 2);
    CHECK(closedManifold(welded.mesh));
    CHECK(ogun::checkOpenEdges(welded.mesh).empty());
    CHECK(shellCount(welded.mesh) == 1);

    // 2. mergeByDistance with preserveOpenEdges = true preserves the crack
    const ogun::MergeByDistanceResult preserved = ogun::mergeByDistance(mesh, 0.05, {}, false, {}, true);
    REQUIRE(preserved.success);
    CHECK(preserved.verticesMerged == 0);
    CHECK(ogun::checkOpenEdges(preserved.mesh).infoCount == 6);

    // 3. mergeByDistance with preserveOpenEdges = false closes the crack
    const ogun::MergeByDistanceResult merged = ogun::mergeByDistance(mesh, 0.05, {}, false, {}, false);
    REQUIRE(merged.success);
    CHECK(merged.verticesMerged == 2);
    CHECK(closedManifold(merged.mesh));
    CHECK(ogun::checkOpenEdges(merged.mesh).empty());
}

TEST_CASE("Fix All's weld changes nothing on the bunny and the Turkey", "[repair][merge][weld][.slow]") {
    const std::filesystem::path examples =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples";
    for (const char* name : {"stanford_bunny.stl", "Turkey.stl"}) {
        const ogun::Mesh mesh = ogun::loadStl(examples / name);
        REQUIRE(mesh.numFaces() > 0);
        const auto start = std::chrono::steady_clock::now();
        const ogun::MergeByDistanceResult r = ogun::mergeByDistance(mesh, kFixAllWeld, {}, true);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        REQUIRE(r.success);
        REQUIRE(r.verticesMerged == 0);
        REQUIRE(r.mesh.numFaces() == mesh.numFaces());
        WARN(name << ": Fix All weld T_max " << r.maxTolerance << ", " << ms << " ms, nothing merged");
    }
}
