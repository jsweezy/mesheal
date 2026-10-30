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
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace ogun {

/// Whole-mesh ("Repair All") strategies for intersecting faces and holes, from surgical to coarse.
enum class GlobalRepairMethod {
    FixAll,          ///< Fix All: every Mesh Health row repaired in passes (`runFixAll`); faces outside the scope kept exactly
    FillHoles,       ///< Every hole gets its recommended fill (or the chosen one); other faces are kept exactly
    PatchIslands,    ///< Every within-shell island gets its recommended local repair (+ fallbacks)
    SelfUnion,       ///< Cut along every crossing and keep the outer surface; keeps other faces exactly
    WindingRebuild,  ///< New surface: marching cubes of the winding number on a grid
    AlphaWrap,       ///< New surface: CGAL alpha wrap (always watertight and manifold)
    Decimate,        ///< Mesh > Simplify > Decimate: edge collapse of the faces in scope
    MergeVertices    ///< Mesh > Clean Up > Weld / Merge Vertices: merge by distance in scope
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

/// The Fix All operator: the repair run and the three rebuilds it is compared with.
inline constexpr std::array<GlobalRepairMethod, 4> kFixAllMethods = {
    GlobalRepairMethod::FixAll,
    GlobalRepairMethod::SelfUnion,
    GlobalRepairMethod::WindingRebuild,
    GlobalRepairMethod::AlphaWrap,
};

[[nodiscard]] const char* globalRepairMethodLabel(GlobalRepairMethod method) noexcept;

/// Methods offered by the Repair All operator of `target` (Intersections or OpenEdges).
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
    /// Winding-number rebuild: grid cells along the longest side.
    int windingResolution = settings::kDefaultWindingGridResolution;
    /// Rebuilds (winding / wrap): simplify back to the in-scope face count when larger.
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

/// The Mesh-menu operators built on the same panel offer one method each.
inline constexpr std::array<GlobalRepairMethod, 1> kWrapMethods = {GlobalRepairMethod::AlphaWrap};
inline constexpr std::array<GlobalRepairMethod, 1> kDecimateMethods = {GlobalRepairMethod::Decimate};
inline constexpr std::array<GlobalRepairMethod, 1> kMergeMethods = {GlobalRepairMethod::MergeVertices};

/// Methods that edit the faces in scope in place (the rest of the mesh keeps its vertices), so the
/// scope may be any set of faces, not just whole shells.
[[nodiscard]] constexpr bool editsInPlace(GlobalRepairMethod method) noexcept {
    return method == GlobalRepairMethod::Decimate || method == GlobalRepairMethod::MergeVertices;
}

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

} // namespace ogun
