#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
#include "repair/RepairMetrics.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstddef>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal {

/// Repair strategy for one intersection island (Inspector radio buttons).
enum class IntersectionRepairMethod {
    None,
    FillFlat,         ///< Cut the island (+ rings) out and fill with the boundary vertices only.
    FillSmooth,       ///< Same cut, then a refined patch faired into the surrounding surface.
    CgalLocalRepair,  ///< CGAL's region-growing remove / smooth / fill (`experimental::remove_self_intersections`).
    SelfUnion         ///< Self-union of the island's shell: cut along the crossings, drop the inside.
};

inline constexpr std::array<IntersectionRepairMethod, 4> kIntersectionRepairMethods = {
    IntersectionRepairMethod::FillFlat,
    IntersectionRepairMethod::FillSmooth,
    IntersectionRepairMethod::CgalLocalRepair,
    IntersectionRepairMethod::SelfUnion,
};

[[nodiscard]] const char* intersectionRepairMethodLabel(IntersectionRepairMethod method) noexcept;

struct IntersectionRepairSettings {
    int dilationRings = settings::kDefaultIntersectionDilationRings;
    int cgalMaxSteps = settings::kDefaultCgalRepairMaxSteps;
    double creaseAngleDeg = settings::kDefaultCreaseAngleDeg;
    double smoothCollarMinAngleDeg = settings::kDefaultSmoothCollarMinAngleDeg;
    double previewOffsetFactor = settings::kDefaultPreviewOffsetFactor;
    /// Batch repair (`healIntersectionIslands`): undo a round whose new faces lie farther than this share of the
    /// shell's bounding-box diagonal from the old surface (a fill bulging over a wall). Removed faces are not
    /// measured (a union rightly drops inner parts), nor the volume (overlaps count twice in it). <= 0: off.
    double maxDeviationFraction = settings::kDefaultIslandMaxDeviation;
};

/// Faces of one shell that cross each other or touch crossing faces (one repair unit).
struct IntersectionIsland {
    std::vector<Eigen::Index> faces;  ///< Sorted intersecting faces
    int shell = -1;                   ///< Shell id (index into `checkShells`), -1 if unknown
    bool local = true;                ///< One surface patch after a 1-ring dilation (a fold), else walls crossing
};

struct IslandAnalysis {
    bool local = true;
    double collarMeanDihedralDeg = 0.0;
    double collarMaxDihedralDeg = 0.0;
    IntersectionRepairMethod recommended = IntersectionRepairMethod::None;
};

/// Shell id of every face (faces with no edge neighbour still get their own shell).
[[nodiscard]] std::vector<int> shellIdsOfFaces(const Mesh& mesh);

/// Groups the faces of an Intersections result into islands: faces that cross each other or share
/// a vertex (union-find), so the holes of neighbouring islands never collide. Crossings between
/// different shells are ignored (Merge Shells or a global union handles those). With `faceInScope`
/// only faces in scope are used. Largest island first.
[[nodiscard]] std::vector<IntersectionIsland> clusterIntersectionIslands(
    const Mesh& mesh,
    const CategoryResult& intersections,
    std::span<const int> faceShell,
    std::span<const std::uint8_t> faceInScope = {});

/// Local vs pass-through and the collar shape around the island; picks a recommended method.
[[nodiscard]] IslandAnalysis analyzeIsland(
    const Mesh& mesh,
    const IntersectionIsland& island,
    const IntersectionRepairSettings& settings = {});

/// `recommended` first, then the other methods in the order most likely to help.
[[nodiscard]] std::array<IntersectionRepairMethod, 4> islandFallbackOrder(IntersectionRepairMethod recommended, bool local) noexcept;

/// Result of repairing one island (preview or commit). Only the island's shell is touched.
struct IslandRepairResult {
    bool success = false;
    std::string statusMessage;
    Mesh mesh;                                  ///< Whole repaired mesh (shell order changes; shell moved last)
    std::vector<Eigen::Index> removedFaces;     ///< Faces of the input that are gone
    std::vector<Eigen::Index> addedFaces;       ///< Faces of `mesh` that are new
    std::size_t crossingsBefore = 0;            ///< Intersecting faces inside the shell before
    std::size_t crossingsAfter = 0;             ///< ... and after
    std::size_t newCrossings = 0;               ///< Intersecting faces after that are new faces
    double volumeChange = 0.0;                  ///< Signed volume change of the shell
    double minNewAngleDeg = 0.0;                ///< Smallest interior angle of the new faces (180 if none)
};

/// Repairs `island` with `method`. `params` supplies the intersection tolerance for the re-check.
/// `faceShell` (from `shellIdsOfFaces`) saves recomputing the shells when the caller has them.
[[nodiscard]] IslandRepairResult repairIsland(
    const Mesh& mesh,
    const IntersectionIsland& island,
    IntersectionRepairMethod method,
    const CheckParams& params,
    const IntersectionRepairSettings& settings = {},
    std::span<const int> faceShell = {});

/// Live preview of `repairIsland`: the new faces as a patch offset along their normals plus the
/// faces they replace (for the cut-out display). The base mesh is never modified.
struct IslandRepairPreview {
    bool valid = false;
    IntersectionRepairMethod method = IntersectionRepairMethod::None;
    std::string statusMessage;
    Mesh previewPatchMesh;
    std::vector<Eigen::Index> sourceFaceIndices;
    std::size_t facesRemoved = 0;
    std::size_t facesAdded = 0;
    std::size_t crossingsAfter = 0;
    std::size_t newCrossings = 0;
    double volumeChange = 0.0;
    double minNewAngleDeg = 0.0;
};

[[nodiscard]] IslandRepairPreview previewIslandRepair(
    const Mesh& mesh,
    const IntersectionIsland& island,
    IntersectionRepairMethod method,
    const CheckParams& params,
    const IntersectionRepairSettings& settings = {},
    std::span<const int> faceShell = {});

/// Batch repair of the within-shell islands of the faces in scope (Fix Selected / Repair All
/// "Patch each island"). Each shell is repaired in rounds: every island gets its recommended
/// method, the shell is re-checked, and the islands left get their next fallback. A round that
/// makes the shell worse, or changes its shape past the settings' limits, is undone. With `seedFaces`, only islands containing one of them are
/// repaired in the first round.
struct IslandBatchResult {
    bool success = false;  ///< True if anything changed
    std::string statusMessage;
    Mesh mesh;
    std::size_t islandsBefore = 0;
    std::size_t islandsAfter = 0;
    int rounds = 0;
    int roundsRefusedForShape = 0; ///< Rounds undone because they changed the shape too much
};

[[nodiscard]] IslandBatchResult healIntersectionIslands(
    const Mesh& mesh,
    const CheckParams& params,
    std::span<const std::uint8_t> faceInScope = {},
    std::span<const Eigen::Index> seedFaces = {},
    const IntersectionRepairSettings& settings = {},
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace mesheal
