#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
#include "repair/FixAll.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal {

/// Whole-mesh ("Repair All") strategies for intersecting faces and holes, from surgical to coarse.
enum class GlobalRepairMethod {
    FixAll,          ///< Fix All: every Mesh Health row repaired in passes (`runFixAll`); faces outside the scope kept exactly
    FillHoles,       ///< Every hole gets its recommended fill (or the chosen one); other faces are kept exactly
    HealThinFaces,   ///< Every thin / sliver face healed by edge flips and collapses (`SliverHealer::healAllThinFaces`)
    PatchIslands,    ///< Every within-shell island gets its recommended local repair (+ fallbacks)
    SelfUnion,       ///< Cut along every crossing and keep the outer surface; keeps other faces exactly
    WindingRebuild,      ///< New surface: marching cubes of the winding number on a grid
    AlphaWrap,           ///< New surface: CGAL alpha wrap (always watertight and manifold)
    VoxelHealRemesh,     ///< Repair > Remesh > Voxel Heal & Remesh (Fast): winding-number MC + Taubin + isotropic remeshing
    PrecisionIsosurface, ///< Repair > Remesh > Precision Isosurface (FEA Grade): CGAL make_mesh_3 on w = 0.5 + relaxation
    Decimate,            ///< Mesh > Simplify > Decimate: edge collapse of the faces in scope
    MergeVertices,       ///< Mesh > Modify > Merge by Distance: merge by distance in scope
    Weld                 ///< Repair > Weld / Mesh > Modify > Weld: weld open edges across cracks
};

inline constexpr std::array<GlobalRepairMethod, 4> kGlobalRepairMethods = {
    GlobalRepairMethod::PatchIslands,
    GlobalRepairMethod::SelfUnion,
    GlobalRepairMethod::WindingRebuild,
    GlobalRepairMethod::AlphaWrap,
};

/// Repair All Holes: the surgical fill and the two rebuilds, which also close holes.
inline constexpr std::array<GlobalRepairMethod, 3> kHoleRepairMethods = {
    GlobalRepairMethod::FillHoles,
    GlobalRepairMethod::WindingRebuild,
    GlobalRepairMethod::AlphaWrap,
};

/// Repair All Thin / Sliver Faces: the sliver healer alone (its result is previewed with its volume change).
inline constexpr std::array<GlobalRepairMethod, 1> kThinRepairMethods = {
    GlobalRepairMethod::HealThinFaces,
};

/// The Fix All operator: the repair run and the three rebuilds it is compared with.
inline constexpr std::array<GlobalRepairMethod, 4> kFixAllMethods = {
    GlobalRepairMethod::FixAll,
    GlobalRepairMethod::AlphaWrap,
    GlobalRepairMethod::VoxelHealRemesh,
    GlobalRepairMethod::PrecisionIsosurface,
};

[[nodiscard]] const char* globalRepairMethodLabel(GlobalRepairMethod method) noexcept;

/// Methods offered by the Repair All operator of `target` (Intersections, OpenEdges or ThinFaces).
[[nodiscard]] std::span<const GlobalRepairMethod> globalRepairMethodsFor(DefectCategory target) noexcept;

struct GlobalRepairParams {
    /// Self-union: also merge shells that cross each other (else each shell is united on its own).
    bool unionCrossingShells = true;
    /// Alpha wrap: alpha = bbox diagonal / detail; offset (the wrap's distance outside the input)
    /// = alpha / offset ratio, so it follows alpha.
    double alphaWrapDetail = settings::kDefaultAlphaWrapDetail;
    double alphaWrapOffsetRatio = settings::kDefaultAlphaWrapOffsetRatio;
    /// Decimate: fraction of the faces in scope to keep.
    double decimateRatio = settings::kDefaultDecimateRatio;
    /// Merge vertices: distance in file coordinates (unitless; the file unit does not change it).
    double mergeDistance = settings::kDefaultMergeDistance;
    /// Merge vertices: only along edges and open edges, so touching surfaces / shells are not pinched.
    bool mergeKeepSurfacesApart = settings::kDefaultMergeKeepSurfacesApart;
    /// Merge vertices: preserve open edges and cracks (do not merge across cracks).
    bool mergePreserveOpenEdges = settings::kDefaultMergePreserveOpenEdges;
    /// Weld: distance in file coordinates.
    double weldDistance = settings::kDefaultWeldDistance;
    /// Winding-number rebuild: grid cells along the longest side.
    int windingResolution = settings::kDefaultWindingGridResolution;
    /// Voxel Heal & Remesh / Precision Isosurface: target triangle edge length in file units (<= 0 = maxSpan / 128).
    double remeshTargetSize = 0.0;
    /// Voxel Heal & Remesh: Taubin de-terracing passes (0-10, default 3).
    int voxelSmoothingPasses = settings::kDefaultVoxelSmoothingPasses;
    /// Precision Isosurface: post-extraction tangential relaxation passes (0-10, default 1).
    int isosurfaceSmoothingPasses = settings::kDefaultIsosurfaceSmoothingPasses;
    /// Precision Isosurface: chord distance as a fraction of target mesh size (0.01-1.0, default 0.10).
    double isosurfaceChordRatio = settings::kDefaultIsosurfaceChordRatio;
    /// Rebuilds (winding / wrap / voxel / isosurface): simplify back to the in-scope face count when larger.
    bool simplifyRebuild = false;
    IntersectionRepairSettings islandSettings{};
    /// Fill each hole: the method for every hole (None = each hole's recommended fill) and the limits.
    HoleFillMethod holeMethod = HoleFillMethod::None;
    HoleFillSettings holeSettings{};
    CheckParams checkParams{};
    /// Fix All: its own settings. Its `checkParams`, `holeFill` and `islandRepair` are not used: the run
    /// takes `checkParams`, `holeSettings` and `islandSettings` above, so it agrees with the row tools.
    FixAllParams fixAll{};

    [[nodiscard]] bool operator==(const GlobalRepairParams& other) const noexcept;
};

/// Alpha wrap: alpha for a scope with bounding-box diagonal `diag`, and the offset used with it (alpha / ratio).
[[nodiscard]] double alphaWrapAlpha(const GlobalRepairParams& params, double diag) noexcept;
[[nodiscard]] double alphaWrapOffset(const GlobalRepairParams& params, double alpha) noexcept;

/// Effective target edge length for Voxel Heal & Remesh and Precision Isosurface given longest bbox span `maxSpan`.
[[nodiscard]] double effectiveRemeshTargetSize(const GlobalRepairParams& params, double maxSpan) noexcept;

/// The Mesh-menu operators built on the same panel offer one method each.
inline constexpr std::array<GlobalRepairMethod, 1> kWrapMethods = {GlobalRepairMethod::AlphaWrap};
inline constexpr std::array<GlobalRepairMethod, 1> kVoxelRemeshMethods = {GlobalRepairMethod::VoxelHealRemesh};
inline constexpr std::array<GlobalRepairMethod, 1> kPrecisionIsosurfaceMethods = {GlobalRepairMethod::PrecisionIsosurface};
inline constexpr std::array<GlobalRepairMethod, 1> kDecimateMethods = {GlobalRepairMethod::Decimate};
inline constexpr std::array<GlobalRepairMethod, 1> kMergeMethods = {GlobalRepairMethod::MergeVertices};
inline constexpr std::array<GlobalRepairMethod, 1> kWeldMethods = {GlobalRepairMethod::Weld};

/// Methods that edit the faces in scope in place (the rest of the mesh keeps its vertices), so the
/// scope may be any set of faces, not just whole shells.
[[nodiscard]] constexpr bool editsInPlace(GlobalRepairMethod method) noexcept {
    return method == GlobalRepairMethod::Decimate || method == GlobalRepairMethod::MergeVertices || method == GlobalRepairMethod::Weld;
}

/// The rebuilds *Rebuild shell #k...* offers for one shell Fix All left, alpha wrap first (the suggested
/// one, the only rebuild that always closes the shell and leaves no crossing within it).
inline constexpr std::array<GlobalRepairMethod, 3> kShellRebuildMethods = {
    GlobalRepairMethod::AlphaWrap,
    GlobalRepairMethod::SelfUnion,
    GlobalRepairMethod::WindingRebuild,
};

struct FixAllCandidate;

/// A Fix All result with one of its shells rebuilt (and merged when the merge is on): what a rebuild row
/// adds to the Fix All candidate it builds on.
struct FixAllShellRebuild {
    std::shared_ptr<const FixAllCandidate> base; ///< The Fix All run (its steps, passes and report)
    int shell = -1;              ///< The shell rebuilt: `base->result.report.shells` index (#(shell + 1))
    bool suggested = false;      ///< Alpha wrap: the row tagged *suggested*
    bool closed = false;         ///< The rebuilt shell has no open and no non-manifold edge
    bool merged = false;         ///< The rebuilt shell was merged (or deleted as a nested solid touching nothing)
    std::size_t rebuiltFaces = 0;
    std::string label;           ///< "+ Alpha wrap #12"
    std::string note;            ///< "wrapped, then merged" / "still open: not merged" / "merge refused: ..."
    double rebuildMs = 0.0;      ///< The rebuild alone (the row's time also has the merge and clean-up)
};

/// What a Fix All candidate carries besides the mesh.
struct FixAllCandidate {
    /// The run. Its `mesh` is moved into `RepairCandidate::mesh` (left empty here); its `scope` and the
    /// report's shells are faces of that mesh.
    FixAllResult result;
    FixAllScopeKind scopeKind = FixAllScopeKind::Whole;
    bool onePassRequested = false; ///< *Preview one pass* (or *Run one more pass*): one pass whatever the Passes field says
    /// Shell of each face of the candidate's mesh: an index into `result.report.shells`, which is the
    /// Shells-panel numbering of that mesh (#(k + 1)); -1 for a face in no shell.
    std::vector<int> faceShell;
    /// A rebuild row (*Rebuild shell #k...*): `result` is then the merge and clean-up pass run on the Fix
    /// All result with the shell rebuilt (its report describes the row's mesh), and the Fix All run itself
    /// is `rebuild->base`.
    std::optional<FixAllShellRebuild> rebuild;
};

/// Extra inputs of a Fix All run (method `FixAll` only).
struct FixAllRunInput {
    FixAllScopeKind scopeKind = FixAllScopeKind::Whole; ///< With a non-empty scope mask: faces or whole shells
    FixAllProgress* progress = nullptr;                  ///< Fraction and text (else only `progress` gets the fraction)
    const FixAllTestHooks* hooks = nullptr;              ///< Tests only
};

/// One computed "Repair All" result, ready to preview and apply.
struct RepairCandidate {
    bool success = false;
    GlobalRepairMethod method = GlobalRepairMethod::PatchIslands;
    GlobalRepairParams params{};
    std::string statusMessage;
    Mesh mesh;                              ///< Whole repaired mesh (out-of-scope shells unchanged, listed first)
    std::vector<Eigen::Index> changedFaces; ///< New faces of `mesh` (tinted in the preview)
    std::vector<Eigen::Index> removedFaces; ///< Faces of the input mesh that `mesh` no longer has
    Eigen::VectorXd vertexDeviation;        ///< Distance of each vertex of `mesh` from the original surface
    RepairMetrics metrics;                  ///< Measured on the faces in scope only
    bool cancelled = false;
    std::shared_ptr<const FixAllCandidate> fixAll; ///< Method `FixAll` only (null otherwise or when it failed)
};

/// Runs `method` on the faces in scope (`faceInScope` empty = whole mesh). Shells out of scope are
/// copied unchanged. Polls `stopToken` between stages; reports 0..1 through `progress`. Fix All runs on
/// the whole mesh with the scope (`fixAll` gives its kind, a progress text and test hooks); its metrics
/// are measured on the faces in scope before and after (the carried scope), its counts are the run's own.
[[nodiscard]] RepairCandidate runGlobalRepair(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr,
    const FixAllRunInput* fixAll = nullptr);

/// Fix All's own parameters with the row tools' settings it shares (`checkParams`, `holeSettings`,
/// `islandSettings`), as `runGlobalRepair` runs it.
[[nodiscard]] FixAllParams fixAllRunParams(const GlobalRepairParams& params);

/// One row of *Rebuild shell #k...* (step 19; never run unless asked). `fixAll` is the Fix All candidate
/// computed on `input` (its `mesh` is the result; `fixAll.fixAll` its run) with the scope mask
/// `faceInScope` (empty = whole). The shell `shell` (`report.shells` index of the result) is rebuilt on its
/// own by `method` (alpha wrap with alpha = its diagonal / `alphaWrapDetail`, self-union, or the winding
/// rebuild), so every other shell is kept exactly and a cavity, being its own shell, is never in the
/// rebuild's input. Then one pass of phases 2 and 3 runs on the result with the rebuilt shell: with the
/// merge on (and whole shells in scope) a closed rebuilt shell is merged with the shells it overlaps,
/// under the same plan, cavity rules and merge guards as Fix All's merge (only its own groups), and the
/// clean-up follows; a rebuilt shell that stays open is not merged, and a refused merge keeps it
/// unmerged; the row says why (`FixAllShellRebuild::note`). The row's metrics are measured on the scope
/// against `input`, its counts are the input's and the row's mesh's, its time is the rebuild plus the
/// merge pass. Refused (`success` false, `statusMessage` says why) for a shell not wholly in scope or a
/// kept cavity. Polls `stopToken` (alpha wrap stops mid-run; a union finishes first).
[[nodiscard]] RepairCandidate computeShellRebuildCandidate(
    const Mesh& input,
    std::span<const std::uint8_t> faceInScope,
    const RepairCandidate& fixAll,
    int shell,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

/// The three rows of `kShellRebuildMethods` for `shell` (stops early when stopped).
[[nodiscard]] std::vector<RepairCandidate> computeShellRebuildCandidates(
    const Mesh& input,
    std::span<const std::uint8_t> faceInScope,
    const RepairCandidate& fixAll,
    int shell,
    const GlobalRepairParams& params,
    std::stop_token stopToken = {});

/// The Rebuild side's cavity warning: rule O cavities (`shellsLargestFirst` indices of `mesh`) with a
/// face in scope (`faceInScope` empty = whole). An alpha wrap of the scope fills them: CGAL purges the
/// pockets its flood fill cannot reach from outside (`keep_inner_connected_components` is false), while
/// a per-shell rebuild keeps them (a cavity is its own shell). std::nullopt when stopped.
[[nodiscard]] std::optional<std::vector<int>> cavitiesFilledByRebuild(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    double intersectionTolerance = settings::kDefaultIntersectionTolerance,
    std::stop_token stopToken = {});

} // namespace mesheal
