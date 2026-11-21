#include "repair/FixAll.hpp"
#include "repair/HoleFiller.hpp"

#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "io/SampleMeshes.hpp"
#include "repair/RepairMetrics.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <map>
#include <vector>

namespace {

/// Builds a mesh from faces given by corner points; each face is wound to face away from `inside`.
struct Builder {
    std::vector<Eigen::Vector3d> points;
    std::map<std::array<double, 3>, int> index;
    std::vector<std::array<int, 3>> faces;

    int vertex(const Eigen::Vector3d& p) {
        const auto [it, fresh] = index.emplace(std::array<double, 3>{p.x(), p.y(), p.z()}, static_cast<int>(points.size()));
        if (fresh) {
            points.push_back(p);
        }
        return it->second;
    }

    void tri(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& inside) {
        const bool outward = (b - a).cross(c - a).dot(a - inside) > 0.0;
        faces.push_back(outward ? std::array{vertex(a), vertex(b), vertex(c)} : std::array{vertex(a), vertex(c), vertex(b)});
    }

    void quad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d,
              const Eigen::Vector3d& inside) {
        tri(a, b, c, inside);
        tri(a, c, d, inside);
    }

    [[nodiscard]] mesheal::Mesh mesh() const {
        mesheal::Mesh m;
        m.V.resize(static_cast<Eigen::Index>(points.size()), 3);
        for (std::size_t i = 0; i < points.size(); ++i) {
            m.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
        }
        m.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
        for (std::size_t f = 0; f < faces.size(); ++f) {
            for (int c = 0; c < 3; ++c) {
                m.F(static_cast<Eigen::Index>(f), c) = faces[f][static_cast<std::size_t>(c)];
            }
        }
        m.recomputeNormals();
        return m;
    }
};

/// A square ring (outer side 4, inner side 2, height 2) cut flat at z = 0: its base is open, an outer
/// and an inner opening in one plane. Built from unit cells, each wall face wound away from its cell.
mesheal::Mesh openRing(bool needleWall = false) {
    Builder b;
    const auto solid = [](int i, int j) { return i >= 0 && i < 4 && j >= 0 && j < 4 && !(i >= 1 && i < 3 && j >= 1 && j < 3); };
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!solid(i, j)) {
                continue;
            }
            const Eigen::Vector3d c(i + 0.5, j + 0.5, 1.0);
            const auto p = [&](int x, int y, int z) { return Eigen::Vector3d(i + x, j + y, 2.0 * z); };
            b.quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1), c); // top
            if (!solid(i - 1, j)) {
                b.quad(p(0, 0, 0), p(0, 1, 0), p(0, 1, 1), p(0, 0, 1), c);
            }
            if (!solid(i + 1, j) && needleWall && i == 3) {
                // The outer wall at x = 4 as a fan of needles: rim points every 0.01 fanned to the top corners
                const int steps = 100;
                const auto rim = [&](int k) { return Eigen::Vector3d(4.0, j + k / static_cast<double>(steps), 0.0); };
                for (int k = 0; k < steps; ++k) {
                    b.tri(rim(k), rim(k + 1), k < steps / 2 ? p(1, 0, 1) : p(1, 1, 1), c);
                }
                b.tri(p(1, 0, 1), rim(steps / 2), p(1, 1, 1), c);
            } else if (!solid(i + 1, j)) {
                b.quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1), c);
            }
            if (!solid(i, j - 1)) {
                b.quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1), c);
            }
            if (!solid(i, j + 1)) {
                b.quad(p(0, 1, 0), p(1, 1, 0), p(1, 1, 1), p(0, 1, 1), c);
            }
        }
    }
    return b.mesh();
}

/// A block (4 wide, 2 high, 3 long) with a V groove whose floor comes down to the cut at x = 2, cut flat
/// at z = 0: one opening, split in two by the groove's floor edge lying in the plane.
mesheal::Mesh openGroovedBlock() {
    Builder b;
    const double length = 3.0;
    for (const double side : {0.0, 1.0}) { // the left and the right half, each a convex prism
        const double x0 = side == 0.0 ? 0.0 : 2.0;
        const double x1 = side == 0.0 ? 2.0 : 4.0;
        const double xTop = side == 0.0 ? 1.0 : 3.0; // where the groove's wall meets the top
        const double xOut = side == 0.0 ? 0.0 : 4.0;
        const Eigen::Vector3d inside((x0 + x1) / 2.0 + (side == 0.0 ? -0.4 : 0.4), length / 2.0, 0.8);
        const auto at = [&](double x, double y, double z) { return Eigen::Vector3d(x, y, z); };
        const double grooveBottom = side == 0.0 ? x1 : x0; // x = 2
        b.quad(at(xOut, 0, 0), at(xOut, length, 0), at(xOut, length, 2), at(xOut, 0, 2), inside);             // outer wall
        b.quad(at(xOut, 0, 2), at(xTop, 0, 2), at(xTop, length, 2), at(xOut, length, 2), inside);             // top
        b.quad(at(xTop, 0, 2), at(grooveBottom, 0, 0), at(grooveBottom, length, 0), at(xTop, length, 2), inside); // groove wall
        for (const double y : {0.0, length}) { // end caps: a trapezoid each
            b.quad(at(xOut, y, 0), at(grooveBottom, y, 0), at(xTop, y, 2), at(xOut, y, 2), inside);
        }
    }
    return b.mesh();
}

bool closedManifold(const mesheal::Mesh& mesh) {
    return mesheal::checkOpenEdges(mesh).elements.empty() && mesheal::checkNonManifoldEdges(mesh).elements.empty() &&
           mesheal::checkNonManifoldVertices(mesh).elements.empty();
}

mesheal::HoleBatchResult fixAllFill(const mesheal::Mesh& mesh) {
    mesheal::HoleBatchOptions options;
    options.crossingPolicy = mesheal::HoleCrossingPolicy::AlwaysFillFewestCrossings;
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(mesh);
    return mesheal::fillHoles(mesh, holes, mesheal::HoleFillMethod::None, {}, options);
}

} // namespace

TEST_CASE("Cap Cut Plane caps a ring cut flat and leaves its hole open", "[repair][holes][cutplane]") {
    const mesheal::Mesh ring = openRing();
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(ring);
    REQUIRE(holes.size() == 2);
    for (const mesheal::HoleLoop& hole : holes) {
        CHECK(mesheal::isCutPlaneHole(hole));
        CHECK(mesheal::sharesCutPlane(hole, holes));
        CHECK(mesheal::recommendedHoleFill(hole, holes) == mesheal::HoleFillMethod::CutPlane);
    }

    const mesheal::HoleBatchResult r = fixAllFill(ring);
    REQUIRE(r.success);
    CHECK(r.holesClosed == 2);
    CHECK(closedManifold(r.mesh));
    CHECK(mesheal::checkSelfIntersections(r.mesh, mesheal::CheckParams{}).elements.empty());
    CHECK(mesheal::signedVolume(r.mesh) == Catch::Approx(12.0 * 2.0)); // the ring, not a lid over its hole
    for (Eigen::Index f = 0; f < r.mesh.numFaces(); ++f) {
        const Eigen::Vector3d c = (r.mesh.V.row(r.mesh.F(f, 0)) + r.mesh.V.row(r.mesh.F(f, 1)) + r.mesh.V.row(r.mesh.F(f, 2))) / 3.0;
        const bool inHole = c.z() < 1.0e-9 && c.x() > 1.0 && c.x() < 3.0 && c.y() > 1.0 && c.y() < 3.0;
        CHECK_FALSE(inHole);
    }
}

TEST_CASE("Cap Cut Plane on its own fills only its own opening when the plane holds more", "[repair][holes][cutplane]") {
    const mesheal::Mesh ring = openRing();
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(ring);
    // Either opening caps the whole plane: the same ring base
    for (const mesheal::HoleLoop& hole : holes) {
        const mesheal::HoleFillResult r = mesheal::fillHole(ring, hole, mesheal::HoleFillMethod::CutPlane);
        REQUIRE(r.success);
        CHECK(r.closed());
        CHECK(r.patchArea == Catch::Approx(12.0));
        CHECK(r.newCrossings == 0);
        CHECK(r.verticesAdded == 0);
    }
}

TEST_CASE("A single flat opening keeps its shape's fill; Cap Cut Plane caps it when picked", "[repair][holes][cutplane]") {
    mesheal::Mesh box = mesheal::SampleMeshes::createGridCubeMesh(4, 4.0);
    std::vector<Eigen::Index> keep;
    for (Eigen::Index f = 0; f < box.numFaces(); ++f) {
        if (box.V(box.F(f, 0), 2) > 1.0e-9 || box.V(box.F(f, 1), 2) > 1.0e-9 || box.V(box.F(f, 2), 2) > 1.0e-9) {
            keep.push_back(f);
        }
    }
    const mesheal::Mesh open = mesheal::extractFaces(box, keep);
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(open);
    REQUIRE(holes.size() == 1);
    CHECK(mesheal::isCutPlaneHole(holes.front()));
    CHECK_FALSE(mesheal::sharesCutPlane(holes.front(), holes)); // one flat opening may as well be a missing dome
    CHECK(mesheal::recommendedHoleFill(holes.front(), holes) == mesheal::recommendedHoleFill(holes.front().shape));

    const mesheal::HoleFillResult r = mesheal::fillHole(open, holes.front(), mesheal::HoleFillMethod::CutPlane);
    REQUIRE(r.success);
    CHECK(r.closed());
    CHECK(r.patchArea == Catch::Approx(16.0));
    CHECK(closedManifold(r.mesh));
    CHECK(mesheal::signedVolume(r.mesh) == Catch::Approx(64.0));
}

TEST_CASE("Cap Cut Plane caps both sides of a groove whose floor comes down to the cut", "[repair][holes][cutplane]") {
    const mesheal::Mesh block = openGroovedBlock();
    const std::vector<mesheal::HoleLoop> holes = mesheal::findHoleLoops(block);
    REQUIRE(holes.size() == 1);
    REQUIRE(mesheal::checkNonManifoldEdges(block).elements.empty());

    const mesheal::HoleFillResult r = mesheal::fillHole(block, holes.front(), mesheal::HoleFillMethod::CutPlane);
    REQUIRE(r.success);
    CHECK(r.closed());
    CHECK(r.addedFaces.size() == 4); // two rectangles, one each side of the groove's floor
    CHECK(r.patchArea == Catch::Approx(12.0));
    CHECK(mesheal::signedVolume(r.mesh) == Catch::Approx(2.0 * 3.0 * 3.0)); // two trapezoids (area 3) x length 3

    // Fix All splits the floor edge the two halves now share: two solids touching along an edge
    const mesheal::FixAllResult fixed = mesheal::runFixAll(block, mesheal::FixAllScope::whole(block), mesheal::FixAllParams{});
    CHECK(closedManifold(fixed.mesh));
    CHECK(mesheal::checkSelfIntersections(fixed.mesh, mesheal::CheckParams{}).elements.empty());
    CHECK(mesheal::checkShells(fixed.mesh).infoCount == 2);
    CHECK(mesheal::signedVolume(fixed.mesh) == Catch::Approx(18.0));
}

TEST_CASE("Fix All keeps the rim of a flat opening in place and caps it", "[repair][holes][cutplane][fixall]") {
    // The ring cut flat, its outer wall a fan of needles standing on short rim edges: Heal thin faces would
    // merge the rim vertices (or flip a sliver flat along the outline) unless the rim is pinned
    const mesheal::Mesh ring = openRing(true);
    REQUIRE(mesheal::checkOpenEdges(ring).elements.size() > 0);
    REQUIRE(mesheal::checkNonManifoldEdges(ring).elements.empty());
    REQUIRE(mesheal::runCategoryCheck(mesheal::DefectCategory::ThinFaces, ring, mesheal::CheckParams{}).size() >= 40);
    std::size_t rimBefore = 0;
    for (Eigen::Index v = 0; v < ring.numVertices(); ++v) {
        rimBefore += ring.V(v, 2) == 0.0 ? 1 : 0;
    }

    // One pass: the heal runs before the fill, so the rim the cap closes is the one the heal left
    mesheal::FixAllParams onePass;
    onePass.passes = 1;
    const mesheal::FixAllResult fixed = mesheal::runFixAll(ring, mesheal::FixAllScope::whole(ring), onePass);
    CHECK(closedManifold(fixed.mesh));
    CHECK(mesheal::checkSelfIntersections(fixed.mesh, mesheal::CheckParams{}).elements.empty());
    std::size_t rimAfter = 0;
    for (Eigen::Index v = 0; v < fixed.mesh.numVertices(); ++v) {
        CHECK(fixed.mesh.V(v, 2) >= 0.0);
        rimAfter += fixed.mesh.V(v, 2) == 0.0 ? 1 : 0;
    }
    CHECK(rimAfter == rimBefore); // no rim vertex lifted off the cut, none merged away before the cap
}
