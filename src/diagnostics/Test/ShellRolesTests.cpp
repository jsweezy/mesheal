#include "diagnostics/ShellRoles.hpp"

#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <stop_token>
#include <string>
#include <vector>

namespace {

using mesheal::SampleMeshes;
using mesheal::ShellRole;

std::filesystem::path examplePath(const char* name) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "examples" / name;
}

mesheal::Mesh box(Eigen::Vector3d min, Eigen::Vector3d max, bool inward = false) {
    return SampleMeshes::createBox(min, max, inward);
}

/// Roles from every pair of shells, loaded the way the app loads a file.
mesheal::ShellRoles rolesOf(const mesheal::Mesh& input) {
    const mesheal::Mesh mesh = mesheal::separateTouchingContacts(input);
    return mesheal::classifyShellRoles(mesh, mesheal::computeShellRelations(mesh));
}

std::vector<ShellRole> roleList(const mesheal::ShellRoles& roles) {
    std::vector<ShellRole> list;
    for (const mesheal::ShellRoleInfo& info : roles.shells) {
        list.push_back(info.role);
    }
    return list;
}

mesheal::CategoryResult invertedShells(const mesheal::Mesh& mesh) {
    return mesheal::runCategoryCheck(mesheal::DefectCategory::InvertedShells, mesheal::separateTouchingContacts(mesh),
                                  mesheal::CheckParams{});
}

/// Builds a mesh from vertex rows and face rows.
mesheal::Mesh meshOf(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& faces) {
    mesheal::Mesh mesh;
    mesh.V.resize(static_cast<Eigen::Index>(vertices.size()), 3);
    for (std::size_t v = 0; v < vertices.size(); ++v) {
        mesh.V.row(static_cast<Eigen::Index>(v)) = vertices[v].transpose();
    }
    mesh.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        mesh.F.row(static_cast<Eigen::Index>(f)) << faces[f][0], faces[f][1], faces[f][2];
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Closed octahedron around `c` with half-sizes `r`, wound inward (negative volume).
mesheal::Mesh inwardOctahedron(const Eigen::Vector3d& c, const Eigen::Vector3d& r) {
    const std::vector<Eigen::Vector3d> v = {c + Eigen::Vector3d(r.x(), 0, 0), c - Eigen::Vector3d(r.x(), 0, 0),
                                            c + Eigen::Vector3d(0, r.y(), 0), c - Eigen::Vector3d(0, r.y(), 0),
                                            c + Eigen::Vector3d(0, 0, r.z()), c - Eigen::Vector3d(0, 0, r.z())};
    // Outward would be (+x, +y, +z) counter-clockwise; each face below is reversed
    return meshOf(v, {{0, 4, 2}, {2, 4, 1}, {1, 4, 3}, {3, 4, 0}, {0, 2, 5}, {2, 1, 5}, {1, 3, 5}, {3, 0, 5}});
}

/// Square pyramid wound inward: base [x0, x1] x [y0, y1] at height z0, apex `apex`.
mesheal::Mesh inwardPyramid(double x0, double x1, double y0, double y1, double z0, const Eigen::Vector3d& apex) {
    const std::vector<Eigen::Vector3d> v = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, apex};
    // Outward: base 0,2,1 / 0,3,2 (facing -z), sides 0,1,4 / 1,2,4 / 2,3,4 / 3,0,4; reversed here
    return meshOf(v, {{0, 1, 2}, {0, 2, 3}, {0, 4, 1}, {1, 4, 2}, {2, 4, 3}, {3, 4, 0}});
}

/// Open Mobius strip around the z axis (radius 3, half-width 1), `segments` quads.
mesheal::Mesh mobiusStrip(int segments) {
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i < segments; ++i) {
        const double t = 2.0 * std::numbers::pi * i / segments;
        const Eigen::Vector3d radial(std::cos(t), std::sin(t), 0.0);
        const Eigen::Vector3d across = std::cos(t / 2.0) * radial + std::sin(t / 2.0) * Eigen::Vector3d::UnitZ();
        v.push_back(3.0 * radial - across);
        v.push_back(3.0 * radial + across);
    }
    std::vector<std::array<int, 3>> f;
    for (int i = 0; i < segments; ++i) {
        const int a = 2 * i;
        const int b = 2 * i + 1;
        const bool last = i + 1 == segments;
        const int c = last ? 1 : 2 * (i + 1); // the half twist swaps the sides at the seam
        const int d = last ? 0 : 2 * (i + 1) + 1;
        f.push_back({a, c, b});
        f.push_back({b, c, d});
    }
    return meshOf(v, f);
}

} // namespace

TEST_CASE("Rule O: a cavity is kept, a free-standing inward box is inside out", "[diagnostics][shell-roles]") {
    const mesheal::ShellRoles cavity = rolesOf(SampleMeshes::createCavityBoxMesh());
    REQUIRE(roleList(cavity) == std::vector{ShellRole::Solid, ShellRole::Cavity});
    REQUIRE(cavity.shells[0].winding == 0);
    REQUIRE(cavity.shells[1].winding == 1);
    REQUIRE(cavity.shells[1].related == std::vector<int>{0});
    REQUIRE(cavity.stoppedGroups.empty());

    const mesheal::ShellRoles insideOut = rolesOf(box({0, 0, 0}, {2, 2, 2}, true));
    REQUIRE(roleList(insideOut) == std::vector{ShellRole::InsideOut});
    REQUIRE(insideOut.count(ShellRole::InsideOut) == 1);
}

TEST_CASE("Rule O: an outward box inside a box is a nested solid, not a cavity", "[diagnostics][shell-roles]") {
    // CGAL's parity orientation would make the inner box a cavity
    REQUIRE(roleList(rolesOf(SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({1, 1, 1}, {3, 3, 3})}))) ==
            std::vector{ShellRole::Solid, ShellRole::NestedSolid});
    REQUIRE(roleList(rolesOf(SampleMeshes::createNestedSolidAndCavityMesh())) ==
            std::vector{ShellRole::Solid, ShellRole::Cavity, ShellRole::NestedSolid});
    // Body, six feathers crossing it, the inner box, a separate box
    const mesheal::ShellRoles turkeyLike = rolesOf(SampleMeshes::createTurkeyLikeMesh());
    REQUIRE(turkeyLike.count(ShellRole::Solid) == 8);
    REQUIRE(turkeyLike.shells[7].role == ShellRole::NestedSolid);
    REQUIRE(turkeyLike.shells[7].related == std::vector<int>{0});
    REQUIRE(roleList(rolesOf(SampleMeshes::createOverlapPlusCavityMesh())) ==
            std::vector{ShellRole::Solid, ShellRole::Solid, ShellRole::Cavity});
}

TEST_CASE("Rule O stops at a body written inside out with a shell inside it", "[diagnostics][shell-roles]") {
    const mesheal::ShellRoles roles = rolesOf(SampleMeshes::createCavityBoxMesh(true));
    REQUIRE(roleList(roles) == std::vector{ShellRole::InsideOutWithInner, ShellRole::InsideOutWithInner});
    REQUIRE(roles.stoppedGroups == std::vector<std::vector<int>>{{0, 1}});
    REQUIRE(roles.shells[0].related == std::vector<int>{1});
    REQUIRE(roles.shells[1].related == std::vector<int>{0});
}

TEST_CASE("Rule O does not guess for an inward shell that touches its container", "[diagnostics][shell-roles]") {
    // An inward octahedron whose corner (0, 1, 2.5) rests on the box's wall x = 0
    const mesheal::ShellRoles roles = rolesOf(SampleMeshes::combineMeshes(
        {box({0, 0, 0}, {4, 4, 4}), inwardOctahedron({1.5, 1.0, 2.5}, {1.5, 0.8, 0.8})}));
    REQUIRE(roleList(roles) == std::vector{ShellRole::Solid, ShellRole::Unclassified});
    REQUIRE(roles.shells[1].related == std::vector<int>{0});
    REQUIRE(roles.shells[1].reason.find("touches") != std::string::npos);
}

TEST_CASE("Rule O: islands in a cavity, upright and inside out", "[diagnostics][shell-roles]") {
    const mesheal::ShellRoles island = rolesOf(SampleMeshes::createIslandInCavityMesh());
    REQUIRE(roleList(island) == std::vector{ShellRole::Solid, ShellRole::Cavity, ShellRole::Island});
    REQUIRE(island.shells[2].winding == 0);
    REQUIRE(island.shells[2].related == std::vector<int>{1});

    const mesheal::ShellRoles flipped = rolesOf(SampleMeshes::createIslandInCavityMesh(true));
    REQUIRE(roleList(flipped) == std::vector{ShellRole::Solid, ShellRole::Cavity, ShellRole::InsideOut});
}

TEST_CASE("Rule O: an inward shell that crosses others is inside out only when mostly outside them",
          "[diagnostics][shell-roles]") {
    // Seven of the inward box's eight corners lie outside the solid
    const mesheal::ShellRoles outside =
        rolesOf(SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({3, 3, 3}, {7, 7, 7}, true)}));
    REQUIRE(roleList(outside) == std::vector{ShellRole::Solid, ShellRole::InsideOut});
    REQUIRE(outside.shells[1].related == std::vector<int>{0});

    // An inward pyramid with its base inside the solid and its apex through the top wall
    const mesheal::ShellRoles inside = rolesOf(
        SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), inwardPyramid(1, 3, 1, 3, 1, {2, 2, 5})}));
    REQUIRE(roleList(inside) == std::vector{ShellRole::Solid, ShellRole::Unclassified});
    REQUIRE(inside.shells[1].reason.find("crosses") != std::string::npos);

    // A cavity that the second box's wall cuts through (fixture 3): inside the solid, so not judged
    REQUIRE(roleList(rolesOf(SampleMeshes::createCavityReachedMesh())) ==
            std::vector{ShellRole::Solid, ShellRole::Solid, ShellRole::Unclassified});
}

TEST_CASE("Rule O: open, mis-wound and self-crossing shells are not closed and never containers",
          "[diagnostics][shell-roles]") {
    mesheal::Mesh open = box({0, 0, 0}, {4, 4, 4});
    open.F.conservativeResize(11, 3);
    REQUIRE(roleList(rolesOf(open)) == std::vector{ShellRole::NotClosed});

    // A cavity inside an open box: cavity or inside out cannot be told yet
    mesheal::Mesh openWithCavity = SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({1, 1, 1}, {3, 3, 3}, true)});
    openWithCavity.F.row(0) = openWithCavity.F.row(23); // the outer box loses a face; the cavity loses nothing
    openWithCavity.F.conservativeResize(23, 3);
    const mesheal::ShellRoles openRoles = rolesOf(openWithCavity); // the cavity has more faces now: #1
    REQUIRE(roleList(openRoles) == std::vector{ShellRole::Unclassified, ShellRole::NotClosed});
    REQUIRE(openRoles.shells[0].related == std::vector<int>{1});
    REQUIRE(openRoles.shells[0].reason.find("not closed") != std::string::npos);

    // The Fix All sample: an open body (a hole) around a cavity, and a peg crossing the body
    REQUIRE(roleList(rolesOf(SampleMeshes::createCavityHoleSliversIntersectionsMesh())) ==
            std::vector{ShellRole::NotClosed, ShellRole::Unclassified, ShellRole::Solid});

    mesheal::Mesh misWound = box({0, 0, 0}, {2, 2, 2}, true);
    misWound.F.row(0) = misWound.F.row(0).reverse().eval();
    const mesheal::ShellRoles misWoundRoles = rolesOf(misWound);
    REQUIRE(roleList(misWoundRoles) == std::vector{ShellRole::NotClosed});
    REQUIRE(misWoundRoles.shells[0].reason.find("wound against") != std::string::npos);

    // A Mobius strip is open (one boundary loop) and is never a container
    const mesheal::Mesh mobius = mobiusStrip(24);
    REQUIRE(roleList(rolesOf(mobius)) == std::vector{ShellRole::NotClosed});
    REQUIRE(roleList(rolesOf(SampleMeshes::combineMeshes({mobius, box({-0.5, -0.5, -0.3}, {0.5, 0.5, 0.3}, true)}))) ==
            std::vector{ShellRole::NotClosed, ShellRole::Unclassified});

    // A shell the caller knows to cross itself
    const mesheal::Mesh cavityBox = SampleMeshes::createCavityBoxMesh();
    const std::vector<std::uint8_t> selfCrossing = {1, 0};
    const mesheal::ShellRoles selfRoles =
        mesheal::classifyShellRoles(cavityBox, mesheal::computeShellRelations(cavityBox), selfCrossing);
    REQUIRE(roleList(selfRoles) == std::vector{ShellRole::NotClosed, ShellRole::Unclassified});
}

TEST_CASE("Rule O leaves an inward shell inside two layers of material unclassified", "[diagnostics][shell-roles]") {
    const mesheal::ShellRoles roles = rolesOf(SampleMeshes::combineMeshes(
        {box({0, 0, 0}, {6, 6, 6}), box({1, 1, 1}, {5, 5, 5}), box({2, 2, 2}, {4, 4, 4}, true)}));
    REQUIRE(roleList(roles) == std::vector{ShellRole::Solid, ShellRole::NestedSolid, ShellRole::Unclassified});
    REQUIRE(roles.shells[2].winding == 2);
}

TEST_CASE("Rule O on stopped relations judges nothing", "[diagnostics][shell-roles]") {
    std::stop_source source;
    source.request_stop();
    const mesheal::Mesh mesh = SampleMeshes::createCavityBoxMesh();
    const mesheal::ShellRelations relations = mesheal::computeShellRelations(mesh, mesheal::ShellRelationParams{}, source.get_token());
    REQUIRE_FALSE(relations.complete);
    const mesheal::ShellRoles roles = mesheal::classifyShellRoles(mesh, relations);
    REQUIRE_FALSE(roles.complete);
    REQUIRE(roleList(roles) == std::vector{ShellRole::Unclassified, ShellRole::Unclassified});
}

TEST_CASE("Rule O needs relations only around inward shells", "[diagnostics][shell-roles]") {
    const auto needs = [](const mesheal::Mesh& mesh) {
        return mesheal::shellsRuleONeeds(mesheal::describeShells(mesh, mesheal::shellsLargestFirst(mesh)));
    };
    REQUIRE(needs(SampleMeshes::createTurkeyLikeMesh()).empty());
    REQUIRE(needs(SampleMeshes::createCavityBoxMesh()) == std::vector<int>{0, 1});
    REQUIRE(needs(SampleMeshes::createCavityReachedMesh(mesheal::SampleMeshes::CavityReachedVariant::WithSeparatePair)) ==
            std::vector<int>{0, 2});

    // Inward shells get the same role from those relations as from every pair
    for (const mesheal::Mesh& mesh : {SampleMeshes::createCavityBoxMesh(true), SampleMeshes::createIslandInCavityMesh(true),
                                   SampleMeshes::createNestedSolidAndCavityMesh(true),
                                   SampleMeshes::createCavityReachedMesh(mesheal::SampleMeshes::CavityReachedVariant::WithSeparatePair),
                                   SampleMeshes::combineMeshes({box({0, 0, 0}, {4, 4, 4}), box({3, 3, 3}, {7, 7, 7}, true)})}) {
        const mesheal::ShellRoles all = mesheal::classifyShellRoles(mesh, mesheal::computeShellRelations(mesh));
        mesheal::ShellRelationParams params;
        params.involving = needs(mesh);
        const mesheal::ShellRelations partial = mesheal::computeShellRelations(mesh, params);
        const mesheal::ShellRoles some = mesheal::classifyShellRoles(mesh, partial);
        for (std::size_t s = 0; s < all.shells.size(); ++s) {
            if (partial.shells[s].signedVolume < 0.0) {
                REQUIRE(some.shells[s].role == all.shells[s].role);
            }
        }
    }
}

TEST_CASE("Rule O reasons name the shells they refer to", "[diagnostics][shell-roles]") {
    mesheal::ShellRoleInfo info;
    info.reason = "is wound inward and crosses {list}, near {0}.";
    info.related = {0, 2, 4};
    const auto label = [](int s) { return "#" + std::to_string(s + 1); };
    REQUIRE(mesheal::formatShellRoleReason(info, label) == "is wound inward and crosses #1, #3 and #5, near #1.");
    REQUIRE(std::string(mesheal::shellRoleLabel(ShellRole::InsideOutWithInner)) == "Inside out with inner shells");
}

TEST_CASE("The Inverted shells check counts inside-out shells and warns about the rest", "[diagnostics][shell-roles]") {
    const mesheal::CategoryResult insideOut = invertedShells(box({0, 0, 0}, {2, 2, 2}, true));
    REQUIRE(insideOut.category == mesheal::DefectCategory::InvertedShells);
    REQUIRE(insideOut.size() == 1);
    REQUIRE(insideOut.elements[0].index == 0);
    REQUIRE(insideOut.facesOf(0).size() == 12);
    REQUIRE(insideOut.elements[0].columns[0] == 12.0);
    REQUIRE(insideOut.elements[0].columns[1] == -8.0);
    REQUIRE(insideOut.warnings.empty());

    // A cavity is correct: nothing counted, nothing to warn about
    const mesheal::CategoryResult cavity = invertedShells(SampleMeshes::createCavityBoxMesh());
    REQUIRE(cavity.empty());
    REQUIRE(cavity.warnings.empty());

    const mesheal::CategoryResult stopped = invertedShells(SampleMeshes::createCavityBoxMesh(true));
    REQUIRE(stopped.empty());
    REQUIRE(stopped.warnings.size() == 1);
    REQUIRE(stopped.warnings[0].shells == std::vector<int>{0, 1});
    REQUIRE(stopped.warnings[0].text == "#1 and its inner shell #2 look inside out together. Flip them by hand.");
    REQUIRE(stopped.warnings[0].kind == mesheal::WarningKind::StoppedGroup); // Flip all / Flip outer only

    const mesheal::CategoryResult touching = invertedShells(SampleMeshes::combineMeshes(
        {box({0, 0, 0}, {4, 4, 4}), inwardOctahedron({1.5, 1.0, 2.5}, {1.5, 0.8, 0.8})}));
    REQUIRE(touching.empty());
    REQUIRE(touching.warnings.size() == 1);
    REQUIRE(touching.warnings[0].shells == std::vector<int>{1});
    REQUIRE(touching.warnings[0].text ==
            "#2 is wound inward and touches #1: cavity or inside out? Left as it is.");
    REQUIRE(touching.warnings[0].kind == mesheal::WarningKind::Unclassified); // Flip it

    const mesheal::CategoryResult island = invertedShells(SampleMeshes::createIslandInCavityMesh(true));
    REQUIRE(island.size() == 1);
    REQUIRE(island.elements[0].index == 2);

    const mesheal::CategoryResult sample = invertedShells(SampleMeshes::createCavityHoleSliversIntersectionsMesh());
    REQUIRE(sample.empty());
    REQUIRE(sample.warnings.size() == 1);
    REQUIRE(sample.warnings[0].shells == std::vector<int>{1});

    // Outward shells only: no relations needed, nothing found
    REQUIRE(invertedShells(SampleMeshes::createTurkeyLikeMesh()).empty());
    REQUIRE(invertedShells(SampleMeshes::createSampleDefectiveMesh()).empty());
}

TEST_CASE("Turkey shell roles", "[diagnostics][shell-roles][.slow]") {
    const mesheal::Mesh turkey = mesheal::loadStl(examplePath("Turkey.stl"));
    const mesheal::ShellRoles roles = mesheal::classifyShellRoles(turkey, mesheal::computeShellRelations(turkey));
    REQUIRE(roles.shells.size() == 28);
    REQUIRE(roles.count(ShellRole::InsideOut) == 0);
    REQUIRE(roles.count(ShellRole::Solid) == 27);
    // #28 (70 faces) lies inside #12 and is wound inward: a cavity of #12, kept
    REQUIRE(roles.shells[27].role == ShellRole::Cavity);
    REQUIRE(roles.shells[27].related == std::vector<int>{11});
    REQUIRE(roles.shells[27].winding == 1);

    const auto start = std::chrono::steady_clock::now();
    const mesheal::CategoryResult row = mesheal::runCategoryCheck(mesheal::DefectCategory::InvertedShells, turkey, mesheal::CheckParams{});
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    WARN("Turkey Inverted shells check: " << ms << " ms");
    REQUIRE(row.empty());
    REQUIRE(row.warnings.empty());
}
