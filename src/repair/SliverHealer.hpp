#pragma once

#include "diagnostics/ThinFaceChecker.hpp"
#include "mesh/CgalMesh.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <string>

namespace ogun {

/// Shape classification of a thin / degenerate triangle.
enum class SliverType {
    Healthy,      ///< Not a sliver and within all thin-face thresholds.
    Needle,       ///< One ultra-short edge between two long edges (l_min / l_max tiny).
    Cap,          ///< One obtuse ~180 deg vertex lying on (or near) the longest edge.
    Microscopic,  ///< Well-shaped but below the area / altitude thresholds.
    DanglingFlap  ///< Isolated triangle, or zero-area flap with at most one attached edge.
};

/// Local repair strategy applied to a single face.
enum class HealAction {
    None,
    FlipLongestEdge,        ///< Swap the cap's longest edge with the neighbor's diagonal.
    CollapseShortestEdge,   ///< Collapse the shortest edge (needle / microscopic dissolution).
    SnapCollinearCollapse,  ///< Collapse the cap's shorter side-arm into its fixed corner vertex.
    RemoveDegenerateFace    ///< Remove an isolated / dangling degenerate face.
};

[[nodiscard]] const char* sliverTypeLabel(SliverType type) noexcept;
[[nodiscard]] const char* healActionLabel(HealAction action) noexcept;

/// Classification and repair-safety tolerances.
struct SliverHealerSettings {
    /// l_min / l_max below this ratio classifies a face as a Needle.
    double needleEdgeRatio = 0.05;
    /// Maximum interior angle above this classifies a face as a Cap.
    double capAngleDeg = 170.0;
    /// Minimum n_new . n_opp for both post-flip triangles (and n_0 . n_opp for non-degenerate caps).
    double coplanarDot = 0.95;
    /// Minimum n_old . n_new for every surviving face around a collapsed edge.
    double minNormalDot = 0.1;
    /// Dihedral angle above which edges are protected as feature creases in the batch PMP cleanup pass.
    double creaseAngleDeg = 60.0;
    /// Preview patch normal offset as a fraction of the patch bounding-box diagonal.
    double previewOffsetFactor = 1.0e-4;
    /// Number of vertex-adjacent rings extracted for the single-face preview patch.
    int previewRings = 2;
    /// Maximum number of flip / collapse sweeps in `healAllThinFaces` before the PMP cleanup pass.
    int maxBatchIterations = 10;
};

/// Geometric and topological analysis of a single face.
struct SliverAnalysis {
    Eigen::Index faceIndex = -1;
    SliverType type = SliverType::Healthy;
    HealAction recommendedAction = HealAction::None;

    double area = 0.0;
    double minEdgeLength = 0.0;
    double maxEdgeLength = 0.0;
    double edgeRatio = 0.0;    ///< minEdgeLength / maxEdgeLength
    double maxAngleDeg = 0.0;  ///< Interior angle opposite the longest edge.

    int numBorderEdges = 0;
    bool violatesThresholds = false;  ///< True if `ThinFaceChecker` would flag this face.
    bool longestEdgeIsBorder = false;
    bool longestEdgeFlippable = false;  ///< Neighbor across the longest edge is coplanar and forms a convex quad.
};

/// Live 2-ring preview of a single-face repair (the base mesh is never modified).
struct SingleFaceRepairPreview {
    bool valid = false;
    SliverAnalysis analysis;
    HealAction appliedAction = HealAction::None;
    std::string statusMessage;
    Mesh previewPatchMesh;  ///< Healed patch, offset along vertex normals to avoid Z-fighting.
};

/// Outcome of a single-face or batch repair on a full mesh.
struct RepairResults {
    bool success = false;
    std::string statusMessage;
    Mesh mesh;  ///< Repaired, contiguously indexed mesh (the input mesh unchanged on failure).

    int edgeFlips = 0;
    int edgeCollapses = 0;
    int facesRemoved = 0;
    int iterations = 0;
    bool usedPmpCleanup = false;

    std::size_t initialThinFaces = 0;
    std::size_t remainingThinFaces = 0;
};

/// CGAL-powered thin / sliver face classification and repair.
class SliverHealer {
public:
    /// Classifies `face` (Needle / Cap / Microscopic / DanglingFlap / Healthy) and recommends a repair.
    [[nodiscard]] static SliverAnalysis analyzeFace(
        const CgalSurfaceMesh& mesh,
        CgalFaceIndex face,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});

    /// Convenience overload that builds the halfedge mesh from `mesh` first.
    [[nodiscard]] static SliverAnalysis analyzeFace(
        const Mesh& mesh,
        Eigen::Index faceIndex,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});

    /// Applies `action` to `face` in place. Returns false (leaving `mesh` unchanged) if the action is
    /// unsafe (border, link condition, crease, non-convex quad or normal inversion); `statusOut` explains why.
    static bool applyHealAction(
        CgalSurfaceMesh& mesh,
        CgalFaceIndex face,
        HealAction action,
        const SliverHealerSettings& settings = {},
        std::string* statusOut = nullptr);

    /// Heals a copy of the 2-ring patch around `face` with `action` (or the recommended action).
    [[nodiscard]] static SingleFaceRepairPreview previewSingleFaceRepair(
        const CgalSurfaceMesh& mesh,
        CgalFaceIndex face,
        std::optional<HealAction> action = std::nullopt,
        Unit unit = Unit::Millimeter,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});

    /// Convenience overload that builds the halfedge mesh from `mesh` first.
    [[nodiscard]] static SingleFaceRepairPreview previewSingleFaceRepair(
        const Mesh& mesh,
        Eigen::Index faceIndex,
        std::optional<HealAction> action = std::nullopt,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});

    /// Applies `action` to `faceIndex` of the full mesh and returns the compacted result.
    [[nodiscard]] static RepairResults healSingleFace(
        const Mesh& mesh,
        Eigen::Index faceIndex,
        HealAction action,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});

    /// Batch repair: (1) flip coplanar caps and remove dangling flaps, (2) collapse needles,
    /// creased caps and microscopic faces, repeated until convergence, then (3) a
    /// `PMP::remove_almost_degenerate_faces` cleanup pass that preserves feature creases.
    [[nodiscard]] static RepairResults healAllThinFaces(
        const Mesh& mesh,
        const ThinFaceThresholds& thresholds = {},
        const SliverHealerSettings& settings = {});
};

} // namespace ogun
