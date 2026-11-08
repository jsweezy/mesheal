#pragma once

// Helpers shared by the Fix All engine (FixAll.cpp) and its stages (FixAllStages.cpp). Not a public API.

#include "mesh/Mesh.hpp"
#include "repair/FixAll.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal::fixall_detail {

/// "1 face" / "3 faces".
[[nodiscard]] std::string counted(std::size_t n, const char* one, const char* many);

/// "0.003 mm" (3 significant digits).
[[nodiscard]] std::string formatLength(double value, Unit unit);

/// "121.875 mm^3" (6 significant digits).
[[nodiscard]] std::string formatVolume(double value, Unit unit);

/// Elements of a check's result that touch `scope` (the score's counting rule; open edges: every rim
/// edge of a hole with a rim face in scope).
[[nodiscard]] std::size_t countInScope(const CategoryResult& result, const FixAllScope& scope);

/// "#2", "#2 and #5", "#2, #3 and #5" for shell indices (0-based, Shells-panel order).
[[nodiscard]] std::string shellList(std::span<const int> shells);

/// Old face -> face of `after` with the same corner positions (winding kept, rotation ignored), -1 when
/// none is left: the face map of a stage that re-numbers faces without reporting it.
[[nodiscard]] std::vector<Eigen::Index> faceMapByGeometry(const Mesh& before, const Mesh& after);

/// Scope guard: every face outside `scope` (which matches `before`) is still in `after` at `map[f]`
/// (empty map = same row) with the same corners.
[[nodiscard]] bool outsideScopeKept(const Mesh& before, const Mesh& after, std::span<const Eigen::Index> map,
                                    const FixAllScope& scope);

/// Shell index of every face (-1 for a face in no shell), from a shell list such as `shellsLargestFirst`.
[[nodiscard]] std::vector<int> shellOfFaces(Eigen::Index faceCount, std::span<const std::vector<Eigen::Index>> shells);

/// Open edges (one face) and non-manifold edges (three or more faces) of `mesh`.
struct EdgeDefectCounts {
    std::size_t open = 0;
    std::size_t nonManifold = 0;
};

[[nodiscard]] EdgeDefectCounts countEdgeDefects(const Mesh& mesh);

/// A kept hole fill: the pass and the hole as the fill step named it.
struct FillRecord {
    int pass = 0;
    int hole = 0;
};

/// What the final report needs besides the result (FixAllReport.cpp).
struct FinalReportInput {
    const Mesh& input;
    const FixAllScope& inputScope;
    const FixAllParams& params;
    std::vector<CategoryResult> health; ///< The nine checks of `result.mesh` (countFixAllDefects order)
    std::span<const int> fillOf;        ///< Per face of `result.mesh`: index into `fills` (-1: no fill made it)
    std::span<const FillRecord> fills;
    std::size_t returnedSteps = 0;      ///< `result.steps[0 .. returnedSteps)` led to `result.mesh`
};

/// Steps 18 and 19 on `result.mesh`: fills `result.report`. False when stopped.
[[nodiscard]] bool buildFinalReport(FixAllResult& result, FinalReportInput input, std::stop_token stopToken);

} // namespace mesheal::fixall_detail
