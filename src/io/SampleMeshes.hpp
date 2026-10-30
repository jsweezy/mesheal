#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ogun {

/// Built-in synthetic and defective test mesh generators for unit tests and the GUI.
struct SampleMeshes {
    static constexpr Eigen::Index kCleanCubeVertexCount = 8;
    static constexpr Eigen::Index kCleanCubeFaceCount = 12;

    /// Known defective face indices in `createSampleDefectiveMesh()`:
    static constexpr Eigen::Index kZeroAreaFaceIndex = 12;
    static constexpr Eigen::Index kMicroscopicFaceIndex = 13;
    static constexpr Eigen::Index kNeedleSliverFaceIndex = 14;
    static constexpr Eigen::Index kDefectiveSampleFaceCount = 15;

    /// Generates a clean watertight cube mesh [0, sideLength]^3 with 8 vertices and 12 triangles.
    [[nodiscard]] static Mesh createCubeMesh(
        double sideLength = 10.0,
        Unit unit = Unit::Millimeter);

    /// Generates the `createCubeMesh()` cube with its top corner (s, s, s) pulled down to (s/2, s/2, -s/2),
    /// so the faces around that corner pass through the bottom. Still one closed, manifold,
    /// consistently wound shell: only the self-intersection check reports it
    /// (`examples/self_intersecting_box.stl`).
    [[nodiscard]] static Mesh createSelfIntersectingBoxMesh(
        double sideLength = 10.0,
        Unit unit = Unit::Millimeter);

    /// Generates a closed cube [0, sideLength]^3 whose six sides are `cells` x `cells` grids of
    /// triangle pairs with shared vertices (outward winding).
    [[nodiscard]] static Mesh createGridCubeMesh(
        int cells = 8,
        double sideLength = 8.0,
        Unit unit = Unit::Millimeter);

    /// Generates the 8 x 8 grid cube (side 8) with two intersection islands in one shell:
    ///   - a local fold: top vertex (2, 2, 8) dragged in-plane to (3.5, 2.2, 8), folding its fan over its neighbours;
    ///   - a pass-through: bottom vertex (6, 6, 0) pushed up to (6, 6, 9), through the top wall.
    /// File > Load Sample > "Folded cube" (try the intersection repairs on it).
    [[nodiscard]] static Mesh createFoldedCubeMesh(Unit unit = Unit::Millimeter);

    /// Generates three welded 2 x 2 grid cubes (side 2) and a fin, for the non-manifold repairs:
    ///   - cube A [0, 2]^3 and cube B [2, 4] x [2, 4] x [0, 2] touch along x = y = 2 (two Touching edges);
    ///   - cube C [-2, 0]^3 touches A only at the origin (one non-manifold vertex);
    ///   - a fin triangle on A's top edge (0, 2, 2)-(1, 2, 2) (one Fin edge and two open edges).
    /// File > Load Sample > "Touching cubes".
    [[nodiscard]] static Mesh createTouchingCubesMesh(Unit unit = Unit::Millimeter);

    /// Generates a UV sphere of radius `radius` centred at the origin: `rings` latitude bands, `segments`
    /// around, single-vertex poles, outward winding.
    [[nodiscard]] static Mesh createSphereMesh(
        int rings = 16,
        int segments = 24,
        double radius = 5.0,
        Unit unit = Unit::Millimeter);

    /// Generates a closed cylinder (axis z, base at z = 0) with `segments` around, `bands` rings of quads
    /// along its side and fan end caps around a centre vertex.
    [[nodiscard]] static Mesh createCylinderMesh(
        int segments = 32,
        int bands = 4,
        double radius = 5.0,
        double height = 10.0,
        Unit unit = Unit::Millimeter);

    /// Generates the 4 x 4 grid cube (side 4) with its top rebuilt from two halves that do not share
    /// the seam vertices at (2, 1, 4) and (2, 3, 4): a T-junction crack along x = 2, welded by exact
    /// position. Pass it through `separateTouchingContacts` like a file: the crack then becomes one
    /// zero-width hole of 6 edges.
    [[nodiscard]] static Mesh createCrackedBoxMesh(Unit unit = Unit::Millimeter);

    /// Generates four shells with one hole each, for the hole repairs (File > Load Sample > "Holes"):
    ///   - the cracked box (`createCrackedBoxMesh`): a Crack;
    ///   - a 4 x 4 grid cube (side 4) at x = 6 missing a 2 x 2-cell square of its top: a Flat hole;
    ///   - a sphere (radius 2) at x = 14 missing its top cap: a Curved hole with a planar rim;
    ///   - a cylinder (radius 2, height 4) at x = 20 missing its top end cap: a Flat hole whose rim faces
    ///     meet the cap at a right angle.
    /// Pass it through `separateTouchingContacts` like a file.
    [[nodiscard]] static Mesh createHolesSampleMesh(Unit unit = Unit::Millimeter);

    /// Generates an ASCII STL string representing the 12-triangle cube (36 raw soup vertices).
    [[nodiscard]] static std::string createCubeAsciiStl(double sideLength = 10.0);

    /// Generates a binary STL byte buffer representing the 12-triangle cube (36 raw soup vertices).
    [[nodiscard]] static std::vector<std::uint8_t> createCubeBinaryStl(double sideLength = 10.0);

    /// Generates the closed 12-triangle box from `min` to `max` (the `createCubeMesh` layout), its faces
    /// pointing outward, or inward when `inward` (a cavity's winding, negative volume).
    [[nodiscard]] static Mesh createBox(
        const Eigen::Vector3d& min,
        const Eigen::Vector3d& max,
        bool inward = false,
        Unit unit = Unit::Millimeter);

    /// Concatenates `parts` in order without welding, so each part stays its own shell (as an STL load
    /// leaves shells that only touch). The unit is the first part's.
    [[nodiscard]] static Mesh combineMeshes(const std::vector<Mesh>& parts);

    // Shell-interaction fixtures (boxes, one shell each, listed in shell order: largest first, then in
    // the order given). Pass them through `separateTouchingContacts` like a file.

    /// Box with a cavity: A = [0, 4]^3 outward (volume 64) holding C = [1, 3]^3 wound inward (-8), touching
    /// nothing. `insideOut`: the whole file inside out (A inward, C outward).
    /// File > Load Sample > "Box with cavity" / "Box with cavity, inside out".
    [[nodiscard]] static Mesh createCavityBoxMesh(bool insideOut = false, Unit unit = Unit::Millimeter);

    /// Overlapping boxes with a cavity: A = [0, 4]^3 and B = [3.5, 7.5] x [0.5, 4.5]^2 outward, overlapping
    /// in [3.5, 4] x [0.5, 4]^2 with no coplanar faces; cavity C = [1, 2] x [1, 3]^2 inward inside A only.
    /// File > Load Sample > "Overlapping boxes with cavity".
    [[nodiscard]] static Mesh createOverlapPlusCavityMesh(Unit unit = Unit::Millimeter);

    enum class CavityReachedVariant {
        CrossesWall,   ///< Cavity [3, 3.8] x [1, 3]^2 (volume 3.2): B's wall x = 3.5 crosses it
        InsideOverlap, ///< Cavity [3.6, 3.9] x [1, 2]^2 (volume 0.3), wholly inside A and B
        WithSeparatePair ///< CrossesWall plus a separate pair of overlapping boxes 20 to the right
    };

    /// The boxes A and B of `createOverlapPlusCavityMesh` with a cavity of A that B reaches into.
    /// File > Load Sample > "Overlapping boxes with cavity in the overlap" (CrossesWall).
    [[nodiscard]] static Mesh createCavityReachedMesh(
        CavityReachedVariant variant = CavityReachedVariant::CrossesWall,
        Unit unit = Unit::Millimeter);

    /// Box [0, 4]^3 holding a cavity [0.5, 1.5]^3 (inward) and a nested box [2.5, 3.5]^3 (outward, touching
    /// nothing). `crossingVariant` adds a third inner box [3.2, 3.8] x [2.7, 3.3] x [2.8, 3.4] that crosses
    /// the nested one. File > Load Sample > "Box with nested box and cavity".
    [[nodiscard]] static Mesh createNestedSolidAndCavityMesh(bool crossingVariant = false, Unit unit = Unit::Millimeter);

    /// Box [0, 6]^3 holding a cavity [1, 5]^3 (inward) with an island [2, 4]^3 floating in it (outward,
    /// or inward when `islandInsideOut`).
    [[nodiscard]] static Mesh createIslandInCavityMesh(bool islandInsideOut = false, Unit unit = Unit::Millimeter);

    /// Boxes [0, 2]^3 and [2, 4] x [2, 4] x [0, 2] touching along the edge x = y = 2 only.
    /// File > Load Sample > "Boxes touching along an edge".
    [[nodiscard]] static Mesh createEdgeTouchingBoxesMesh(Unit unit = Unit::Millimeter);

    /// Boxes [0, 2]^3 and [2, 4] x [0.5, 1.5]^2 sharing a 1 x 1 patch of the plane x = 2.
    /// File > Load Sample > "Boxes touching across a face".
    [[nodiscard]] static Mesh createFaceTouchingBoxesMesh(Unit unit = Unit::Millimeter);

    /// A body box [0, 10] x [0, 6]^2, six thin feather boxes crossing its top y = 6, a small box
    /// [3.2, 4.8] x [1.2, 2.8]^2 inside the body touching nothing (outward), and a separate box
    /// [14, 16] x [0, 2]^2: 9 shells. File > Load Sample > "Turkey-like assembly".
    [[nodiscard]] static Mesh createTurkeyLikeMesh(Unit unit = Unit::Millimeter);

    enum class FillCrossesVariant {
        PlateThroughOpenBox, ///< Body, plate, open box: every fill of the open box's hole crosses the plate
        OpenPlate,           ///< The same with one triangle of the plate's far side x = 1 missing (the plate is open too)
        OpenBoxThroughBody   ///< Body [0, 6]^3 and a box [4, 9] x [1, 3]^2 through its side x = 6, open at x = 9
    };

    /// A fill that crosses another shell. PlateThroughOpenBox: a body A = [0, 6]^3, a plate
    /// P = [1, 5]^2 x [2.9, 3.1] inside it (a nested solid) and a box D = [2, 4]^3 inside it with its two
    /// x = 4 triangles missing (a 4-edge hole); the plate passes through D's walls and through its opening,
    /// so every fill of D's hole crosses P. 3 shells: A #1, P #2, D #3.
    /// File > Load Sample > "Box holding a plate through an open box".
    [[nodiscard]] static Mesh createFillCrossesShellMesh(
        FillCrossesVariant variant = FillCrossesVariant::PlateThroughOpenBox,
        Unit unit = Unit::Millimeter);

    /// Three boxes, all crossing the first: [0, 4]^3, [3.5, 7.5] x [0.5, 4.5]^2 and [-3, 0.5] x [1, 3]^2,
    /// the last missing its far side x = -3 (two triangles: a 4-edge hole). 3 shells.
    /// File > Load Sample > "Overlapping boxes, one open".
    [[nodiscard]] static Mesh createOpenShellAmongCubesMesh(Unit unit = Unit::Millimeter);

    /// The sample to try every repair on at once: a 4 x 4 grid box [0, 8]^3 with
    ///   - a hole: one 2 x 2 cell missing from its front y = 0 (x in [4, 6], z in [2, 4]);
    ///   - slivers on its left side x = 0: one Cap (a point 0.01 off a cell diagonal) and two Needles
    ///     (a point 0.01 from a cell corner), each splitting one triangle in three;
    ///   - a fold: top vertex (2, 2, 8) dragged to (5, 2.4, 8), so its faces cross their neighbours;
    ///   - a cavity [1.5, 3.5]^3 (inward) touching nothing;
    ///   - a peg box [6.6, 10.4] x [5.4, 6.6] x [5.3, 6.7] crossing its right side x = 8.
    /// 3 shells. File > Load Sample > "Box with cavity, hole, slivers, and intersections".
    [[nodiscard]] static Mesh createCavityHoleSliversIntersectionsMesh(Unit unit = Unit::Millimeter);

    /// Faces of `createDoubledFacesCubeMesh()`; faces 0 to 13 alone are a closed cube.
    static constexpr Eigen::Index kFlatCapFaceIndex = 13;
    static constexpr Eigen::Index kSameWindingCopyFaceIndex = 14;
    static constexpr Eigen::Index kOppositeWindingCopyFaceIndex = 15;
    static constexpr Eigen::Index kRepeatedVertexFaceIndex = 16;

    /// The cube [0, 10]^3 (closed, outward) with its bottom triangle at the front edge split in two at
    /// the edge's midpoint (5, 0, 0) (faces 0 and 12) and the T-junction closed by a flat cap (0, 8, 1)
    /// lying along that edge (face 13: zero area, three distinct vertices). Then three faces that are
    /// defects only: a copy of top face 2 wound the same way (14), a copy of back face 6 wound the other
    /// way (15) and a face that repeats top corner 6 (16). Degenerate faces: 2 (13, 16); duplicate
    /// faces: 2 (14, 15).
    /// File > Load Sample > "Cube with doubled and collapsed faces".
    [[nodiscard]] static Mesh createDoubledFacesCubeMesh(Unit unit = Unit::Millimeter);

    /// The corner tetrahedron `corner`, `corner` + `edge` along x, y and z (outward, 4 faces).
    [[nodiscard]] static Mesh createTetrahedron(const Eigen::Vector3d& corner, double edge, Unit unit = Unit::Millimeter);

    /// Debris around a cube, one shell each in this order (all 12-face boxes come first, so the Shells
    /// panel numbers them #1 to #5): the cube [0, 10]^3; a cube [12, 12.5] x [0, 0.5]^2 (2.9 % of the
    /// model's bounding-box diagonal, about 30); a cavity [5, 5.01]^3 inside the cube (wound inward);
    /// a tetrahedron 0.01 across whose base lies on the cube's top at (3, 6, 10); and a tetrahedron 0.01
    /// across at (20, 20, 0), far from everything (0.058 %). Only the last is a tiny floating shell.
    /// File > Load Sample > "Cube with tiny floating pieces".
    [[nodiscard]] static Mesh createTinyFloatingShellsMesh(Unit unit = Unit::Millimeter);

    /// Two half-boxes [0, 1]^3 and [1, 2] x [0, 1]^2 meeting at x = 1, welded, with an internal wall: the
    /// left box's own two x = 1 faces (20 faces: 10, 8, then the wall 18 and 19). The right half has no top, so the wall
    /// and the left top are the only faces at the top seam edge: the wall holds that seam shut. The
    /// bottom and the two upright seam edges have three faces each (*Fin / other*); the right top's
    /// three far edges are open. Without the wall the seam opens and the top is one 4-edge hole.
    [[nodiscard]] static Mesh createFinSeamBoxesMesh(Unit unit = Unit::Millimeter);

    /// The box `min` to `max` cut across `axis` (0 = x, 1 = y, 2 = z) at its middle into two open halves
    /// (10 faces each, two shells), the upper half's side faces starting `offset` past the cut: a hairline
    /// seam whose 4 vertex pairs differ by `offset` along `axis` only (offset 0 gives the closed box).
    [[nodiscard]] static Mesh createSeamBoxMesh(
        const Eigen::Vector3d& min,
        const Eigen::Vector3d& max,
        int axis,
        double offset,
        Unit unit = Unit::Millimeter);

    /// The cube [10, 20]^3 cut at x = 15 with a seam `offset` wide (`createSeamBoxMesh`), then every point
    /// p moved to `scale` x p + `translate` (offsets included). Fix All's weld closes the default 5e-6
    /// seam (its tolerance is 1.5e-5 there); the load weld (1.5e-8 there) does not.
    /// File > Load Sample > "Cube with a hairline seam".
    [[nodiscard]] static Mesh createWeldSeamCubeMesh(
        double offset = 5.0e-6,
        const Eigen::Vector3d& translate = Eigen::Vector3d::Zero(),
        double scale = 1.0,
        Unit unit = Unit::Millimeter);

    /// What stands inside the cup of `createCupWithPostMesh`.
    enum class CupPost {
        BelowOpening,      ///< A post of the cup's own surface rising to 0.5 below the opening
        ThroughOpening,    ///< ... rising 1 above the opening: every fill of the opening crosses it
        RodThroughOpening  ///< A low post, plus a separate box rod standing through the opening (a second shell)
    };

    /// A round cup (32 segments) made by turning a profile about the z axis: an outer wall of radius 5
    /// from z = 0 to 10, a lip sloping down and in to the opening (radius 3 at z = 8, a 32-edge hole whose
    /// rim faces slope at 45 degrees, so it is Curved and Fill Smooth is recommended), a bottom and a post
    /// of radius 1 pushed up from the bottom (the same surface, so the same shell). Fill Smooth follows
    /// the lip down into the cup; Fill Flat stays at z = 8. BelowOpening: the post's top is at 7.5, so
    /// Smooth crosses it and Flat does not. ThroughOpening: the post's top is at 9. RodThroughOpening: the
    /// post's top is at 2 and the box [-0.5, 0.5]^2 x [4, 12] stands through the opening, touching nothing.
    /// File > Load Sample > "Cup with a post".
    [[nodiscard]] static Mesh createCupWithPostMesh(CupPost post = CupPost::BelowOpening, Unit unit = Unit::Millimeter);

    /// A cube [0, 1]^3 with 2 x 2 grid sides (48 triangles) read the way a file is: a triangle soup welded
    /// with the load weld (`weldVerticesLocal`, `ToleranceConfig{}`) and `separateTouchingContacts`. In
    /// the soup the top corner (1, 1, 1) is 1e-7 off along y in one top triangle and the bottom corner
    /// (0, 0, 0) 1e-7 off along x in one bottom triangle: two 4-edge cracks, wider than the load weld
    /// (1e-9 here) and narrower than Fix All's weld (1e-6 per axis here). The two triangles of the left
    /// side's upper back cell are wound backwards (2 flipped faces), and one triangle of the right side's
    /// lower front cell is missing (a 3-edge hole away from both cracks). 47 faces, 1 shell.
    /// File > Load Sample > "Cracked cube with flipped faces".
    [[nodiscard]] static Mesh createCrackedFlippedCubeSoup(Unit unit = Unit::Millimeter);

    /// The 2 x 2 grid cube [0, 2]^3 with the top face along the grid edge (1, 0, 2)-(1, 1, 2) split at the
    /// edge's midpoint (1, 0.5, 2), and the T-junction closed by a zero-area cap with three distinct
    /// vertices lying along that edge (the last face). Closed; deleting the cap opens a 3-edge crack.
    [[nodiscard]] static Mesh createCollinearCapCubeMesh(Unit unit = Unit::Millimeter);

    /// A plate [0, 10]^2 x [0, 1] with a square hole [4, 6]^2 in its top, whose rim edge (4, 4)-(6, 4) is the
    /// long edge of a cap (apex (5, 4 - 8.7e-4, 1), angle 179.9 degrees, the 14th face). One 4-edge hole.
    [[nodiscard]] static Mesh createBoundaryCapPlateMesh(Unit unit = Unit::Millimeter);

    /// Generates a clean 12-triangle cube mesh injected with 3 known defective triangles:
    ///   - Face 12 (`kZeroAreaFaceIndex`): collinear zero-area triangle (A = 0, h_min = 0)
    ///   - Face 13 (`kMicroscopicFaceIndex`): microscopic-area triangle (A = 5e-9, h_min ~ 7.07e-5; well
    ///     shaped, so thin only under an advanced height or area floor)
    ///   - Face 14 (`kNeedleSliverFaceIndex`): long needle/sliver triangle (h_min = 2e-4, AR = 50,000)
    [[nodiscard]] static Mesh createSampleDefectiveMesh(Unit unit = Unit::Millimeter);
};

/// Free-function convenience wrapper around `SampleMeshes::createSampleDefectiveMesh()`.
[[nodiscard]] inline Mesh createSampleDefectiveMesh(Unit unit = Unit::Millimeter) {
    return SampleMeshes::createSampleDefectiveMesh(unit);
}

} // namespace ogun
