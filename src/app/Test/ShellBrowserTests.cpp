#include "app/ShellBrowser.hpp"
#include "io/SampleMeshes.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <numeric>
#include <vector>

namespace {

/// Cubes of the given side at the given min corners, appended in order (one shell each).
ogun::Mesh cubes(const std::vector<std::pair<double, Eigen::Vector3d>>& specs) {
    ogun::Mesh mesh;
    for (const auto& [side, offset] : specs) {
        ogun::Mesh c = ogun::SampleMeshes::createCubeMesh(side, ogun::Unit::Millimeter);
        c.V.rowwise() += offset.transpose();
        const Eigen::Index nv = mesh.V.rows();
        const Eigen::Index nf = mesh.F.rows();
        mesh.V.conservativeResize(nv + c.V.rows(), 3);
        mesh.F.conservativeResize(nf + c.F.rows(), 3);
        mesh.V.bottomRows(c.V.rows()) = c.V;
        mesh.F.bottomRows(c.F.rows()) = c.F.array() + static_cast<int>(nv);
    }
    mesh.recomputeNormals();
    return mesh;
}

/// Three cubes; the second loses a face so the shells sort by size: 12, 11 and 12 faces.
ogun::Mesh threeShells() {
    ogun::Mesh mesh = cubes({{10.0, {0.0, 0.0, 0.0}}, {4.0, {20.0, 0.0, 0.0}}, {2.0, {40.0, 0.0, 0.0}}});
    const Eigen::MatrixXi f = mesh.F;
    mesh.F.resize(f.rows() - 1, 3);
    mesh.F << f.topRows(12), f.middleRows(13, 23);
    mesh.recomputeNormals();
    return mesh;
}

std::vector<std::size_t> identity(std::size_t n) {
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    return order;
}

} // namespace

TEST_CASE("ShellBrowser numbers shells largest first and maps every face", "[app][shells]") {
    const ogun::Mesh mesh = threeShells();
    ogun::ShellBrowser shells;
    shells.rebuild(mesh);
    REQUIRE(shells.size() == 3);
    REQUIRE(shells.multiShell());
    REQUIRE(shells.shell(0).faces.size() == 12);
    REQUIRE(shells.shell(1).faces.size() == 12);
    REQUIRE(shells.shell(2).faces.size() == 11);
    REQUIRE(shells.shell(0).bbox.max.x() == 10.0); // equal sizes keep the file order
    REQUIRE(shells.shell(2).bbox.min.x() == 20.0);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells.shell(s).faces) {
            REQUIRE(shells.shellOfFace(f) == static_cast<int>(s));
        }
    }
    REQUIRE(shells.shellOfFace(-1) == -1);
    REQUIRE_FALSE(shells.scoped());

    ogun::ShellBrowser single;
    single.rebuild(ogun::SampleMeshes::createCubeMesh(1.0, ogun::Unit::Millimeter));
    REQUIRE(single.size() == 1);
    REQUIRE_FALSE(single.multiShell());
}

TEST_CASE("ShellBrowser selection and the selected-else-visible scope", "[app][shells]") {
    ogun::ShellBrowser shells;
    shells.rebuild(threeShells());
    const std::vector<std::size_t> order = identity(3);

    shells.click(1, false, false, order);
    REQUIRE(shells.selectedCount() == 1);
    REQUIRE(shells.scoped());
    REQUIRE(shells.inScope(1));
    REQUIRE_FALSE(shells.inScope(0));
    REQUIRE(shells.inScope(-1)); // a face in no shell is always in scope

    shells.click(0, true, false, order); // Ctrl: add
    REQUIRE(shells.selectedShells() == std::vector<std::size_t>{0, 1});
    shells.click(2, false, true, order); // Shift: anchor (0) .. 2
    REQUIRE(shells.selectedCount() == 3);
    shells.click(1, true, false, order); // Ctrl: remove
    REQUIRE(shells.selectedShells() == std::vector<std::size_t>{0, 2});

    // With nothing selected the scope is the visible shells
    shells.selectNone();
    shells.setVisible(2, false);
    REQUIRE(shells.scoped());
    REQUIRE(shells.inScope(0));
    REQUIRE_FALSE(shells.inScope(2));
    REQUIRE(shells.scopeCount() == 2);
    REQUIRE_FALSE(shells.faceVisible(shells.shell(2).faces.front()));

    shells.hideAll();
    REQUIRE(shells.visibleCount() == 0);
    shells.showAll();
    REQUIRE_FALSE(shells.scoped());

    shells.selectWhere([&](std::size_t s) { return shells.shell(s).faces.size() < 12; });
    REQUIRE(shells.selectedShells() == std::vector<std::size_t>{2});
    shells.invertSelection();
    REQUIRE(shells.selectedShells() == std::vector<std::size_t>{0, 1});
}

TEST_CASE("ShellBrowser Isolate re-targets to a new selection and restores the first view", "[app][shells]") {
    ogun::ShellBrowser shells;
    shells.rebuild(threeShells());
    const std::vector<std::size_t> order = identity(3);
    REQUIRE_FALSE(shells.toggleIsolate()); // nothing selected, nothing to restore

    shells.setVisible(2, false);
    shells.click(0, false, false, order);
    REQUIRE(shells.toggleIsolate());
    REQUIRE(shells.isolated());
    REQUIRE(shells.visibleCount() == 1);

    // Isolating another shell switches to it
    shells.click(1, false, false, order);
    REQUIRE(shells.toggleIsolate());
    REQUIRE(shells.shell(1).visible);
    REQUIRE_FALSE(shells.shell(0).visible);

    // Pressed again with the same selection: back to the view from before the first Isolate
    REQUIRE(shells.toggleIsolate());
    REQUIRE_FALSE(shells.isolated());
    REQUIRE(shells.shell(0).visible);
    REQUIRE(shells.shell(1).visible);
    REQUIRE_FALSE(shells.shell(2).visible);
}

TEST_CASE("ShellBrowser keeps visibility and selection across a rebuild", "[app][shells]") {
    const ogun::Mesh mesh = threeShells();
    ogun::ShellBrowser shells;
    shells.rebuild(mesh);
    const std::vector<std::size_t> order = identity(3);
    shells.click(2, false, false, order);
    shells.setVisible(0, false);
    const ogun::BoundingBox selectedBox = shells.shell(2).bbox;

    // Dropping shell #2 renumbers the others; their state follows them
    const std::vector<Eigen::Index> keep = shells.facesOf(std::array<std::size_t, 2>{0, 2});
    shells.rebuild(ogun::extractFaces(mesh, keep));
    REQUIRE(shells.size() == 2);
    REQUIRE_FALSE(shells.shell(0).visible);
    REQUIRE(shells.shell(1).selected);
    REQUIRE(shells.shell(1).bbox.min == selectedBox.min);
    REQUIRE(shells.selectedCount() == 1);

    // An edited shell (one face fewer, same box) matches by box overlap
    const ogun::Mesh dropped = ogun::extractFaces(mesh, keep);
    std::vector<Eigen::Index> edited(static_cast<std::size_t>(dropped.numFaces()));
    std::iota(edited.begin(), edited.end(), Eigen::Index{0});
    edited.erase(edited.begin() + static_cast<std::ptrdiff_t>(shells.shell(1).faces.front()));
    shells.rebuild(ogun::extractFaces(dropped, edited));
    REQUIRE(shells.size() == 2);
    REQUIRE(shells.shell(1).faces.size() == 10);
    REQUIRE(shells.shell(1).selected);
    REQUIRE_FALSE(shells.shell(0).visible);

    // Reducing down to a single shell unhides it and turns off shell hiding
    const std::vector<Eigen::Index> onlyHidden = shells.facesOf(std::array<std::size_t, 1>{0});
    shells.rebuild(ogun::extractFaces(dropped, onlyHidden));
    REQUIRE(shells.size() == 1);
    REQUIRE_FALSE(shells.multiShell());
    REQUIRE(shells.shell(0).visible);
    REQUIRE(shells.visibleCount() == 1);
    REQUIRE(shells.selectedCount() == 0);
    REQUIRE_FALSE(shells.isolated());

    shells.setVisible(0, false);
    REQUIRE(shells.visibleCount() == 1);
    shells.hideAll();
    REQUIRE(shells.visibleCount() == 1);
    shells.click(0, false, false, identity(1));
    shells.hideSelected();
    REQUIRE(shells.visibleCount() == 1);
    REQUIRE_FALSE(shells.toggleIsolate());
    REQUIRE(shells.visibleCount() == 1);

    shells.clear();
    REQUIRE(shells.size() == 0);
}
