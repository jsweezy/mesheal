#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/RepairMetrics.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace ogun {

/// Whole-mesh ("Repair All") strategies for intersecting faces and holes, from surgical to coarse.
enum class GlobalRepairMethod {
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

[[nodiscard]] const char* globalRepairMethodLabel(GlobalRepairMethod method) noexcept;

/// Methods offered by the Repair All operator of `target` (Intersections or OpenEdges).
[[nodiscard]] std::span<const GlobalRepairMethod> globalRepairMethodsFor(DefectCategory target) noexcept;

struct GlobalRepairParams {
    /// Self-union: also merge shells that cross each other (else each shell is united on its own).
    bool unionCrossingShells = true;
    /// Alpha wrap: alpha = bbox diagonal / detail; offset = the wrap's distance outside the input
    /// (a length; 0 = automatic, alpha / kDefaultAlphaWrapOffsetRatio).
    double alphaWrapDetail = settings::kDefaultAlphaWrapDetail;
    double alphaWrapOffset = 0.0;
    /// Decimate: fraction of the faces in scope to keep.
    double decimateRatio = settings::kDefaultDecimateRatio;
    /// Merge vertices: distance in millimetres (converted to the mesh's unit).
    double mergeDistanceMm = settings::kDefaultMergeDistanceMm;
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

    [[nodiscard]] bool operator==(const GlobalRepairParams& other) const noexcept;
};

/// Alpha wrap: alpha for a scope with bounding-box diagonal `diag`, and the offset used with it.
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
};

/// Runs `method` on the faces in scope (`faceInScope` empty = whole mesh). Shells out of scope are
/// copied unchanged. Polls `stopToken` between stages; reports 0..1 through `progress`.
[[nodiscard]] RepairCandidate runGlobalRepair(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    GlobalRepairMethod method,
    const GlobalRepairParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace ogun
