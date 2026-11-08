#pragma once

#include "core/Settings.hpp"
#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal {

/// Repair strategy for one hole (Inspector radio buttons).
enum class HoleFillMethod {
    None,
    Flat,        ///< CGAL `triangulate_hole`: new faces between the rim vertices only (2D CDT for a planar rim)
    Refine,      ///< ... plus interior vertices so the patch matches the density of the faces around it
    Smooth,      ///< ... refined, then faired (bi-Laplacian) so the patch continues the surrounding curvature
    CloseCrack   ///< Snap the rim vertices onto the opposite side and stitch: no faces in the gap
};

inline constexpr std::array<HoleFillMethod, 4> kHoleFillMethods = {
    HoleFillMethod::Flat,
    HoleFillMethod::Refine,
    HoleFillMethod::Smooth,
    HoleFillMethod::CloseCrack,
};

[[nodiscard]] const char* holeFillMethodLabel(HoleFillMethod method) noexcept;

/// Flat holes -> Flat, curved -> Smooth, cracks -> Close Crack, open chains -> None.
[[nodiscard]] HoleFillMethod recommendedHoleFill(HoleShape shape) noexcept;

struct HoleFillSettings {
    int patchRings = settings::kDefaultHolePatchRings;            ///< Face rings copied around the rim for CGAL
    std::size_t maxPatchFaces = settings::kDefaultMaxHolePatchFaces; ///< Refine / Smooth refuse bigger patches
    double crackSnapFactor = settings::kDefaultCrackSnapFactor;   ///< Close Crack moves vertices at most this many gap widths
    int maxHoleEdges = settings::kDefaultMaxHoleEdges;            ///< Batch: skip holes with more rim edges (0 = no limit)
    double previewOffsetFactor = settings::kDefaultPreviewOffsetFactor;
    HoleShapeParams shape{};

    [[nodiscard]] bool operator==(const HoleFillSettings& other) const noexcept;
};

/// Result of filling one hole (preview or commit). Fills only add faces and vertices: every input face
/// and vertex keeps its index. Close Crack also moves rim vertices and replaces the faces it splits.
struct HoleFillResult {
    bool success = false;  ///< The method changed the mesh
    std::string statusMessage;
    Mesh mesh;                              ///< Whole mesh (the input when `success` is false)
    std::vector<Eigen::Index> addedFaces;   ///< Faces of `mesh` that are new (at the end)
    std::vector<Eigen::Index> removedFaces; ///< Faces of the input that are gone (Close Crack)
    std::size_t verticesAdded = 0;
    std::size_t openEdgesLeft = 0;          ///< Rim edges of this hole (or new edges) still open
    double patchArea = 0.0;                 ///< Area of the new faces
    double maxVertexMove = 0.0;             ///< Farthest an input vertex moved (Close Crack)
    double minNewAngleDeg = 180.0;          ///< Smallest interior angle of the new faces
    std::size_t newCrossings = 0;           ///< New faces that cross other faces of their shell
    double elapsedMs = 0.0;

    [[nodiscard]] bool closed() const noexcept {
        return success && openEdgesLeft == 0;
    }
};

/// Fills `hole` (from `findHoleLoops(mesh)`) with `method`. With `checkCrossings`, the new faces are
/// tested against the other faces of their shell (`params` gives the tolerance).
[[nodiscard]] HoleFillResult fillHole(
    const Mesh& mesh,
    const HoleLoop& hole,
    HoleFillMethod method,
    const CheckParams& params = {},
    const HoleFillSettings& settings = {},
    bool checkCrossings = true);

/// Live preview of `fillHole`: the new faces offset along their normals, plus the faces they replace.
struct HoleFillPreview {
    bool valid = false;
    HoleFillMethod method = HoleFillMethod::None;
    std::string statusMessage;
    Mesh previewPatchMesh;
    std::vector<Eigen::Index> sourceFaceIndices;
    std::size_t facesAdded = 0;
    std::size_t facesRemoved = 0;
    std::size_t verticesAdded = 0;
    std::size_t openEdgesLeft = 0;
    double patchArea = 0.0;
    double maxVertexMove = 0.0;
    double minNewAngleDeg = 180.0;
    std::size_t newCrossings = 0;
    double elapsedMs = 0.0;
};

[[nodiscard]] HoleFillPreview previewHoleFill(
    const Mesh& mesh,
    const HoleLoop& hole,
    HoleFillMethod method,
    const CheckParams& params = {},
    const HoleFillSettings& settings = {});

/// What the batch fill does when a fill crosses other faces (a crossing = a new face and another face
/// that the intersection check reports as a pair).
enum class HoleCrossingPolicy {
    IgnoreCrossings,           ///< One fill per hole, crossings not tested (Fix Selected, Repair All "Fill each hole")
    NeverCommitCrossing,       ///< Try the fill chain; keep the first fill that crosses nothing, else leave the hole open
    AlwaysFillFewestCrossings  ///< Try the fill chain; keep the first fill that crosses nothing, else the one that crosses least (Fix All)
};

/// Options of the batch fill beyond the per-hole settings.
struct HoleBatchOptions {
    HoleCrossingPolicy crossingPolicy = HoleCrossingPolicy::IgnoreCrossings;
    CheckParams checkParams{};                 ///< Intersection tolerance of the crossing test (x the mesh's bbox diagonal)
    std::span<const std::uint8_t> faceInScope; ///< One flag per input face; empty = every face. Faces outside stay bit-identical
};

/// Why the batch did not try to fill a hole.
enum class HoleSkipReason : std::uint8_t {
    None,
    OpenChain,  ///< The open edges do not form a closed loop
    TooBig,     ///< More rim edges than `HoleFillSettings::maxHoleEdges`
    NoMethod,   ///< No fill method applies
    OutOfScope  ///< No rim face is in the scope
};

[[nodiscard]] const char* holeSkipReasonLabel(HoleSkipReason reason) noexcept;

/// One fill tried on a hole in the batch.
struct HoleFillAttempt {
    HoleFillMethod method = HoleFillMethod::None;
    bool changed = false;               ///< The method produced a fill
    bool closed = false;                ///< ... that closed every rim edge
    bool outsideScope = false;          ///< It was undone: it would remove a face outside the scope or move one of its vertices
    std::size_t crossingsOwnShell = 0;  ///< Crossings with faces of the hole's own shell (incl. the patch itself)
    std::size_t crossingsOtherShells = 0;
    std::string status;                 ///< Why it failed, or what it added
};

/// What the batch did with one hole.
struct HoleFillOutcome {
    std::size_t hole = 0;           ///< Index into the `holes` passed in
    int shell = -1;                 ///< Shell of the rim (`shellsLargestFirst` of the input: #(shell + 1)); -1 when not tested
    HoleSkipReason skipped = HoleSkipReason::None;
    HoleFillMethod method = HoleFillMethod::None; ///< The fill kept (None: the hole is still open)
    bool closed = false;
    std::size_t facesAdded = 0;
    std::size_t crossingsOwnShell = 0;    ///< Of the fill kept
    std::size_t crossingsOtherShells = 0; ///< Of the fill kept
    std::vector<int> otherShells;         ///< Shells the kept fill crosses (input numbering, ascending; -1 = a face in no shell)
    std::vector<std::size_t> otherShellCrossings; ///< Crossings with each of `otherShells`
    std::vector<HoleFillAttempt> attempts;        ///< Every fill tried, in order (fallbacks tried = size - 1)
    std::string status;
};

/// Batch fill (Fix Selected, Repair All "Fill each hole", Fix All). Each hole gets `method`, or its
/// recommended method when `method` is None; holes over `settings.maxHoleEdges` and open chains are skipped.
/// With a crossing policy other than IgnoreCrossings, a hole tries that method first, then the rest of
/// Smooth, Refine and Flat (in that order).
struct HoleBatchResult {
    bool success = false;  ///< True if any hole changed
    std::string statusMessage;
    Mesh mesh;
    std::size_t holesBefore = 0;
    std::size_t holesClosed = 0;
    std::size_t holesSkipped = 0;   ///< Too big, not a loop or out of scope
    std::size_t holesFailed = 0;    ///< No fill found (or edges left open, or every fill crossed under NeverCommitCrossing)
    std::size_t facesAdded = 0;
    std::vector<Eigen::Index> addedFaces;  ///< Faces of `mesh` that are new
    std::vector<Eigen::Index> faceMap;     ///< Input face -> face of `mesh` (-1 removed by Close Crack); new faces come after
    std::vector<HoleFillOutcome> outcomes; ///< One per hole, in the order of `holes`
    bool crossingsTested = false;          ///< The policy tested crossings (the counts below are meaningful)
    std::size_t holesWithCrossings = 0;    ///< Holes kept filled although their fill crosses something
    std::size_t crossingsOwnShell = 0;
    std::size_t crossingsOtherShells = 0;
    std::vector<int> crossedShells;        ///< Other shells crossed by any kept fill (input numbering, ascending)
};

[[nodiscard]] HoleBatchResult fillHoles(
    const Mesh& mesh,
    std::span<const HoleLoop> holes,
    HoleFillMethod method = HoleFillMethod::None,
    const HoleFillSettings& settings = {},
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

/// `fillHoles` with a crossing policy and a face scope. A fill that would change a face outside the
/// scope (Close Crack moving its vertices) is not used.
[[nodiscard]] HoleBatchResult fillHoles(
    const Mesh& mesh,
    std::span<const HoleLoop> holes,
    HoleFillMethod method,
    const HoleFillSettings& settings,
    const HoleBatchOptions& options,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace mesheal
