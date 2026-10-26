#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
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

/// Whole-mesh ("Repair All") strategies for intersecting faces, from surgical to coarse.
enum class GlobalRepairMethod {
    PatchIslands,    ///< Every within-shell island gets its recommended local repair (+ fallbacks)
    SelfUnion,       ///< Cut along every crossing and keep the outer surface; keeps other faces exactly
    WindingRebuild,  ///< New surface: marching cubes of the winding number on a grid
    AlphaWrap        ///< New surface: CGAL alpha wrap (always watertight and manifold)
};

inline constexpr std::array<GlobalRepairMethod, 4> kGlobalRepairMethods = {
    GlobalRepairMethod::PatchIslands,
    GlobalRepairMethod::SelfUnion,
    GlobalRepairMethod::WindingRebuild,
    GlobalRepairMethod::AlphaWrap,
};

[[nodiscard]] const char* globalRepairMethodLabel(GlobalRepairMethod method) noexcept;

struct GlobalRepairParams {
    /// Self-union: also merge shells that cross each other (else each shell is united on its own).
    bool unionCrossingShells = true;
    /// Alpha wrap: alpha = bbox diagonal / detail; offset = alpha / offset ratio.
    double alphaWrapDetail = settings::kDefaultAlphaWrapDetail;
    double alphaWrapOffsetRatio = settings::kDefaultAlphaWrapOffsetRatio;
    /// Winding-number rebuild: grid cells along the longest side.
    int windingResolution = settings::kDefaultWindingGridResolution;
    /// Rebuilds (winding / wrap): simplify back to the in-scope face count when larger.
    bool simplifyRebuild = false;
    IntersectionRepairSettings islandSettings{};
    CheckParams checkParams{};

    [[nodiscard]] bool operator==(const GlobalRepairParams& other) const noexcept;
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
