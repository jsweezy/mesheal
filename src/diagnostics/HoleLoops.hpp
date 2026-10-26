#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ogun {

/// Shape of a hole, which decides the recommended fill. The order matches the Open edges sub-types.
enum class HoleShape : std::uint8_t {
    Flat,       ///< Planar rim and the faces around it are coplanar with it or meet it at a right angle (a missing CAD face)
    Curved,     ///< Anything else that is a closed loop: the fill should follow the surrounding curvature
    Crack,      ///< The gap is much narrower than the faces around it (unwelded CAD patches, T-junctions)
    OpenChain   ///< The open edges do not form a closed loop (fins, edges next to non-manifold edges): not fillable
};

inline constexpr std::size_t kHoleShapeCount = 4;

[[nodiscard]] const char* holeShapeLabel(HoleShape shape) noexcept;

struct HoleShapeParams {
    double flatTolerance = settings::kDefaultHoleFlatTolerance;
    double flatNeighbourDeg = settings::kDefaultHoleFlatNeighbourDeg;
    double flatNeighbourShare = settings::kDefaultHoleFlatNeighbourShare;
    double crackWidthRatio = settings::kDefaultHoleCrackWidthRatio;
};

/// One hole: a connected set of open edges (rims that share a vertex count as one hole, the same
/// grouping as the "Hole" column of the Open edges row).
struct HoleLoop {
    std::vector<std::array<int, 2>> edges;  ///< Rim edges in hole order (a -> b runs opposite to the face on the edge)
    std::vector<Eigen::Index> faces;        ///< The face on each rim edge
    std::vector<int> vertices;              ///< Distinct rim vertices, sorted
    bool closed = true;                     ///< Every rim vertex has as many rim edges in as out
    double perimeter = 0.0;
    double meanEdgeLength = 0.0;
    double size = 0.0;                      ///< Bounding-box diagonal of the rim
    double area = 0.0;                      ///< Magnitude of the rim's vector area (planar area for a flat hole)
    double width = 0.0;                     ///< Mean gap width: 2 x area / perimeter (radius of a round hole)
    double planeDeviation = 0.0;            ///< Farthest rim vertex from the best-fit plane
    double flatNeighbourShare = 0.0;        ///< Share of rim faces coplanar with or perpendicular to that plane
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< Best-fit plane normal (along the vector area when it is defined)
    HoleShape shape = HoleShape::Curved;

    [[nodiscard]] std::size_t edgeCount() const noexcept {
        return edges.size();
    }
};

/// Every hole of `mesh` in the order of their first open edge in `topo` (the "Hole" numbering of
/// `checkOpenEdges`), with the rim measured and its shape classified.
[[nodiscard]] std::vector<HoleLoop> findHoleLoops(
    const Mesh& mesh,
    const EdgeTopology& topo,
    const HoleShapeParams& params = {});

[[nodiscard]] std::vector<HoleLoop> findHoleLoops(const Mesh& mesh, const HoleShapeParams& params = {});

/// Measures the rim of `hole` (its edges and faces set) and classifies its shape.
void measureHoleLoop(const Mesh& mesh, HoleLoop& hole, const HoleShapeParams& params = {});

/// Index of the hole whose rim contains the undirected edge {a, b}, or -1.
[[nodiscard]] int findHoleOfEdge(std::span<const HoleLoop> holes, int a, int b) noexcept;

} // namespace ogun
