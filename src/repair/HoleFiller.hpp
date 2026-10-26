#pragma once

#include "core/Settings.hpp"
#include "diagnostics/HoleLoops.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstddef>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace ogun {

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

/// Batch fill (Fix Selected, Repair All "Fill each hole"). Each hole gets `method`, or its recommended
/// method when `method` is None; holes over `settings.maxHoleEdges` and open chains are skipped.
struct HoleBatchResult {
    bool success = false;  ///< True if any hole changed
    std::string statusMessage;
    Mesh mesh;
    std::size_t holesBefore = 0;
    std::size_t holesClosed = 0;
    std::size_t holesSkipped = 0;   ///< Too big or not a loop
    std::size_t holesFailed = 0;    ///< The method found no fill (or left edges open)
    std::size_t facesAdded = 0;
    std::vector<Eigen::Index> addedFaces;  ///< Faces of `mesh` that are new
};

[[nodiscard]] HoleBatchResult fillHoles(
    const Mesh& mesh,
    std::span<const HoleLoop> holes,
    HoleFillMethod method = HoleFillMethod::None,
    const HoleFillSettings& settings = {},
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace ogun
