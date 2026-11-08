#include "repair/MeshCleanup.hpp"

#include "core/Constants.hpp"
#include "mesh/CgalMesh.hpp"

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mesheal {

namespace {

bool inScope(std::span<const std::uint8_t> faceInScope, Eigen::Index f) noexcept {
    return faceInScope.empty() || faceInScope[static_cast<std::size_t>(f)] != 0;
}

/// Mesh of `faces` over `V`, keeping only the vertices they use (in index order). `remapOut` receives
/// the old -> new vertex numbers (-1 = unused).
Mesh compactMesh(const Eigen::MatrixXd& V, const std::vector<std::array<int, 3>>& faces, Unit unit,
                 std::vector<int>* remapOut = nullptr) {
    std::vector<int> remap(static_cast<std::size_t>(V.rows()), -1);
    for (const auto& face : faces) {
        for (const int v : face) {
            remap[static_cast<std::size_t>(v)] = 0;
        }
    }
    int used = 0;
    for (int& r : remap) {
        if (r == 0) {
            r = used++;
        }
    }
    Mesh out;
    out.fileUnit = unit;
    out.V.resize(used, 3);
    for (Eigen::Index v = 0; v < V.rows(); ++v) {
        if (const int r = remap[static_cast<std::size_t>(v)]; r >= 0) {
            out.V.row(r) = V.row(v);
        }
    }
    out.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f) {
        for (int c = 0; c < 3; ++c) {
            out.F(static_cast<Eigen::Index>(f), c) = remap[static_cast<std::size_t>(faces[f][static_cast<std::size_t>(c)])];
        }
    }
    out.recomputeNormals();
    if (remapOut != nullptr) {
        *remapOut = std::move(remap);
    }
    return out;
}

struct CellKey {
    std::int64_t x, y, z;
    bool operator==(const CellKey&) const = default;
};

struct CellHash {
    std::size_t operator()(const CellKey& k) const noexcept {
        return static_cast<std::size_t>(k.x * 73856093LL ^ k.y * 19349663LL ^ k.z * 83492791LL);
    }
};

struct TriangleKeyHash {
    std::size_t operator()(const std::array<int, 3>& k) const noexcept {
        return static_cast<std::size_t>(k[0]) * 2654435761u ^ static_cast<std::size_t>(k[1]) * 40503u ^
               static_cast<std::size_t>(k[2]);
    }
};

/// True when `b` lists the corners of `a` in the same cyclic order.
bool sameWinding(const std::array<int, 3>& a, const std::array<int, 3>& b) noexcept {
    for (int s = 0; s < 3; ++s) {
        if (a[0] == b[static_cast<std::size_t>(s)] && a[1] == b[static_cast<std::size_t>((s + 1) % 3)]) {
            return true;
        }
    }
    return false;
}

/// The merge engine shared by both `mergeByDistance` forms. `widthFor(M)` gives the widest distance a
/// matching pair can differ by on any axis when M is the largest |coordinate| of a candidate (the grid
/// cell); `match(p, q)` decides a pair. `protectOutside`: pin every vertex of a face outside the scope
/// and never remove such a face as a copy.
template <typename WidthFor, typename Match>
MergeByDistanceResult mergeVertices(const Mesh& mesh, WidthFor widthFor, Match match,
                                    std::span<const std::uint8_t> faceInScope, bool keepSurfacesApart,
                                    std::span<const std::uint8_t> pinned, bool protectOutside,
                                    bool preserveOpenEdges = false, bool onlyOpenEdges = false) {
    MergeByDistanceResult result;
    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    const auto nf = static_cast<std::size_t>(mesh.numFaces());
    if ((!faceInScope.empty() && faceInScope.size() != nf) || (!pinned.empty() && pinned.size() != nv)) {
        result.statusMessage = "Merge needs one scope flag per face and one pin flag per vertex.";
        return result;
    }

    // Candidates: the finite vertices of the faces in scope. Pins: the caller's, plus (protectOutside)
    // every vertex of a face outside the scope
    std::vector<std::uint8_t> candidate(nv, 0);
    std::vector<std::uint8_t> pin(nv, 0);
    std::vector<std::uint8_t> usedOutside(nv, 0);
    for (std::size_t v = 0; v < pinned.size(); ++v) {
        pin[v] = pinned[v] != 0 ? 1 : 0;
    }
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const bool in = inScope(faceInScope, f);
        for (int c = 0; c < 3; ++c) {
            const auto v = static_cast<std::size_t>(mesh.F(f, c));
            if (in) {
                candidate[v] = 1;
            } else if (protectOutside) {
                pin[v] = 1;
                usedOutside[v] = 1;
            }
        }
    }
    // Keeping surfaces apart or checking cracks: the edges (sorted vertex pairs) and vertices on open edges
    std::vector<std::pair<int, int>> edges;
    std::vector<std::uint8_t> onOpenEdge;
    if (keepSurfacesApart || preserveOpenEdges || onlyOpenEdges) {
        edges.reserve(3 * nf);
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                const int a = mesh.F(f, c);
                const int b = mesh.F(f, (c + 1) % 3);
                edges.emplace_back(std::min(a, b), std::max(a, b));
            }
        }
        std::sort(edges.begin(), edges.end());
        onOpenEdge.assign(nv, 0);
        for (std::size_t i = 0; i < edges.size();) {
            std::size_t j = i;
            while (j < edges.size() && edges[j] == edges[i]) {
                ++j;
            }
            if (j - i == 1) {
                onOpenEdge[static_cast<std::size_t>(edges[i].first)] = 1;
                onOpenEdge[static_cast<std::size_t>(edges[i].second)] = 1;
            }
            i = j;
        }
    }
    if (onlyOpenEdges) {
        for (std::size_t v = 0; v < nv; ++v) {
            if (candidate[v] != 0 && onOpenEdge[v] == 0) {
                candidate[v] = 0;
            }
        }
    }
    // Protected (outside the scope): a vertex used only by faces outside the scope may still absorb an
    // in-scope vertex, so a crack along the scope's edge closes onto the outside side. With surfaces kept
    // apart only its open-edge vertices can (an edge to an in-scope vertex would make it in scope)
    std::vector<std::uint8_t> outsideOnly(nv, 0);
    if (protectOutside && !faceInScope.empty()) {
        for (std::size_t v = 0; v < nv; ++v) {
            if (candidate[v] == 0 && usedOutside[v] != 0 && (!keepSurfacesApart || onOpenEdge[v] != 0)) {
                candidate[v] = 1;
                outsideOnly[v] = 1;
            }
        }
    }
    double largest = 0.0;
    for (std::size_t v = 0; v < nv; ++v) {
        if (candidate[v] == 0) {
            continue;
        }
        const auto row = mesh.V.row(static_cast<Eigen::Index>(v));
        if (!row.allFinite()) {
            candidate[v] = 0;
            continue;
        }
        largest = std::max(largest, row.cwiseAbs().maxCoeff());
    }

    // Grid cells at least as wide as any matching pair's gap on an axis (exact 27-cell search), never
    // so narrow that a cell index could overflow, and widened a little against rounding in x / cell
    result.maxTolerance = widthFor(largest);
    double cell = std::max(result.maxTolerance, largest * kGridMinCellCoordinateShare) * (1.0 + kGridCellRoundingMargin);
    if (!(cell > 0.0)) {
        cell = 1.0; // every candidate is at the origin
    }
    const auto cellOf = [&](std::size_t v) {
        const auto i = static_cast<Eigen::Index>(v);
        return CellKey{static_cast<std::int64_t>(std::floor(mesh.V(i, 0) / cell)),
                       static_cast<std::int64_t>(std::floor(mesh.V(i, 1) / cell)),
                       static_cast<std::int64_t>(std::floor(mesh.V(i, 2) / cell))};
    };
    std::unordered_map<CellKey, std::vector<int>, CellHash> grid;
    for (std::size_t v = 0; v < nv; ++v) {
        if (candidate[v] != 0) {
            grid[cellOf(v)].push_back(static_cast<int>(v));
        }
    }

    const auto mayMerge = [&](int a, int b) {
        if (onlyOpenEdges) {
            return onOpenEdge[static_cast<std::size_t>(a)] != 0 && onOpenEdge[static_cast<std::size_t>(b)] != 0;
        }
        const std::pair<int, int> edge(std::min(a, b), std::max(a, b));
        const bool hasEdge = !edges.empty() && std::binary_search(edges.begin(), edges.end(), edge);
        if (hasEdge) {
            return true;
        }
        const bool touchesOpen = !onOpenEdge.empty() && (onOpenEdge[static_cast<std::size_t>(a)] != 0 || onOpenEdge[static_cast<std::size_t>(b)] != 0);
        if (preserveOpenEdges && touchesOpen) {
            return false;
        }
        if (keepSurfacesApart) {
            return !preserveOpenEdges && !onOpenEdge.empty() && (onOpenEdge[static_cast<std::size_t>(a)] != 0 && onOpenEdge[static_cast<std::size_t>(b)] != 0);
        }
        return true;
    };

    // Each unmerged vertex (pinned ones first, then the rest, each in index order) absorbs the unmerged,
    // unpinned ones that match it
    std::vector<int> target(nv);
    for (std::size_t v = 0; v < nv; ++v) {
        target[v] = static_cast<int>(v);
    }
    std::vector<std::uint8_t> assigned(nv, 0);
    const auto absorb = [&](std::size_t v) {
        assigned[v] = 1;
        const auto iv = static_cast<Eigen::Index>(v);
        const Eigen::Vector3d p = mesh.V.row(iv).transpose();
        const CellKey key = cellOf(v);
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                for (std::int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = grid.find(CellKey{key.x + dx, key.y + dy, key.z + dz});
                    if (it == grid.end()) {
                        continue;
                    }
                    for (const int n : it->second) {
                        const auto un = static_cast<std::size_t>(n);
                        if (assigned[un] != 0 || (outsideOnly[v] != 0 && outsideOnly[un] != 0)) {
                            continue; // two outside vertices are none of the weld's business
                        }
                        const Eigen::Vector3d q = mesh.V.row(n).transpose();
                        if (!match(p, q) || !mayMerge(static_cast<int>(v), n)) {
                            continue;
                        }
                        if (pin[un] != 0) {
                            ++result.pinnedPairsKept;
                            continue;
                        }
                        assigned[un] = 1;
                        target[un] = static_cast<int>(v);
                        ++result.verticesMerged;
                        result.maxMove = std::max(result.maxMove, (q - p).norm());
                    }
                }
            }
        }
    };
    for (const std::uint8_t pass : {std::uint8_t{1}, std::uint8_t{0}}) {
        for (std::size_t v = 0; v < nv; ++v) {
            if (candidate[v] != 0 && assigned[v] == 0 && pin[v] == pass) {
                absorb(v);
            }
        }
    }

    // Remap faces; drop the collapsed ones
    std::vector<std::array<int, 3>> faces;
    std::vector<Eigen::Index> origin; // input face of each remapped face
    std::vector<std::uint8_t> touched;
    faces.reserve(nf);
    origin.reserve(nf);
    touched.reserve(nf);
    result.faceMap.assign(nf, -1);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        std::array<int, 3> face{};
        bool moved = false;
        for (int c = 0; c < 3; ++c) {
            const int v = mesh.F(f, c);
            face[static_cast<std::size_t>(c)] = target[static_cast<std::size_t>(v)];
            moved = moved || face[static_cast<std::size_t>(c)] != v;
        }
        const bool collapsed = face[0] == face[1] || face[1] == face[2] || face[0] == face[2];
        if (collapsed && !(protectOutside && !inScope(faceInScope, f))) {
            ++result.degenerateRemoved;
            continue;
        }
        faces.push_back(face);
        origin.push_back(f);
        touched.push_back(moved ? 1 : 0);
    }

    // Faces that became copies: keep one of a same-winding group, drop folds (both windings) entirely.
    // Protected faces (outside the scope) always stay; the in-scope copies of one go
    std::unordered_map<std::array<int, 3>, std::vector<std::size_t>, TriangleKeyHash> copies;
    for (std::size_t f = 0; f < faces.size(); ++f) {
        std::array<int, 3> key = faces[f];
        std::sort(key.begin(), key.end());
        copies[key].push_back(f);
    }
    const auto isProtected = [&](std::size_t f) { return protectOutside && !inScope(faceInScope, origin[f]); };
    std::vector<std::uint8_t> drop(faces.size(), 0);
    for (const auto& [key, group] : copies) {
        if (group.size() < 2 || std::none_of(group.begin(), group.end(), [&](std::size_t f) { return touched[f] != 0; })) {
            continue;
        }
        const bool fold = std::any_of(group.begin() + 1, group.end(),
                                      [&](std::size_t f) { return !sameWinding(faces[group.front()], faces[f]); });
        const auto protectedIt = std::find_if(group.begin(), group.end(), isProtected);
        const std::size_t keep = fold ? faces.size() : (protectedIt != group.end() ? *protectedIt : group.front());
        for (const std::size_t f : group) {
            if (f != keep && !isProtected(f)) {
                drop[f] = 1;
                ++result.duplicatesRemoved;
            }
        }
    }
    std::vector<std::array<int, 3>> kept;
    kept.reserve(faces.size());
    for (std::size_t f = 0; f < faces.size(); ++f) {
        if (drop[f] == 0) {
            result.faceMap[static_cast<std::size_t>(origin[f])] = static_cast<Eigen::Index>(kept.size());
            kept.push_back(faces[f]);
        }
    }

    std::vector<int> remap;
    result.mesh = compactMesh(mesh.V, kept, mesh.fileUnit, &remap);
    result.vertexMap.resize(nv);
    for (std::size_t v = 0; v < nv; ++v) {
        result.vertexMap[v] = remap[static_cast<std::size_t>(target[v])];
    }
    result.success = true;
    std::array<char, 160> text{};
    if (onlyOpenEdges) {
        std::snprintf(text.data(), text.size(), "Welded %zu open-edge vertices (moved at most %.3g); removed %zu collapsed and %zu duplicate faces.",
                      result.verticesMerged, result.maxMove, result.degenerateRemoved, result.duplicatesRemoved);
    } else {
        std::snprintf(text.data(), text.size(), "Merged %zu vertices (moved at most %.3g); removed %zu collapsed and %zu duplicate faces.",
                      result.verticesMerged, result.maxMove, result.degenerateRemoved, result.duplicatesRemoved);
    }
    result.statusMessage = text.data();
    return result;
}

} // namespace

MergeByDistanceResult mergeByDistance(const Mesh& mesh, double distance, std::span<const std::uint8_t> faceInScope,
                                      bool keepSurfacesApart, std::span<const std::uint8_t> pinned,
                                      bool preserveOpenEdges) {
    if (mesh.empty() || !(distance > 0.0) || !std::isfinite(distance)) {
        MergeByDistanceResult result;
        result.statusMessage = "Merge needs a mesh and a positive distance.";
        return result;
    }
    const double d2 = distance * distance;
    return mergeVertices(
        mesh, [distance](double) { return distance; },
        [d2](const Eigen::Vector3d& p, const Eigen::Vector3d& q) { return (q - p).squaredNorm() <= d2; }, faceInScope,
        keepSurfacesApart, pinned, false, preserveOpenEdges, false);
}

MergeByDistanceResult weldOpenEdges(const Mesh& mesh, double distance, std::span<const std::uint8_t> faceInScope,
                                    std::span<const std::uint8_t> pinned) {
    if (mesh.empty() || !(distance > 0.0) || !std::isfinite(distance)) {
        MergeByDistanceResult result;
        result.statusMessage = "Weld needs a mesh and a positive distance.";
        return result;
    }
    const double d2 = distance * distance;
    return mergeVertices(
        mesh, [distance](double) { return distance; },
        [d2](const Eigen::Vector3d& p, const Eigen::Vector3d& q) { return (q - p).squaredNorm() <= d2; }, faceInScope,
        true, pinned, false, false, true);
}

MergeByDistanceResult mergeByDistance(const Mesh& mesh, const ToleranceConfig& tol, std::span<const std::uint8_t> faceInScope,
                                      bool keepSurfacesApart, std::span<const std::uint8_t> pinned,
                                      bool preserveOpenEdges) {
    const bool valid = std::isfinite(tol.absFloor) && std::isfinite(tol.relTol) && tol.absFloor >= 0.0 && tol.relTol >= 0.0 &&
                       (tol.absFloor > 0.0 || tol.relTol > 0.0);
    if (mesh.empty() || !valid) {
        MergeByDistanceResult result;
        result.statusMessage = "Weld needs a mesh and a tolerance: floor and relative tolerance at least 0, not both 0.";
        return result;
    }
    return mergeVertices(
        mesh, [&tol](double largest) { return std::max(tol.absFloor, tol.relTol * largest); },
        [&tol](const Eigen::Vector3d& p, const Eigen::Vector3d& q) { return tol.verticesMatchWithinTolerance(p, q); },
        faceInScope, keepSurfacesApart, pinned, true, preserveOpenEdges, false);
}

DecimateResult decimate(const Mesh& mesh, double ratio, std::span<const std::uint8_t> faceInScope) {
    namespace SMS = CGAL::Surface_mesh_simplification;
    DecimateResult result;
    if (mesh.empty() || !(ratio > 0.0) || ratio > 1.0) {
        result.statusMessage = "Decimate needs a mesh and a ratio in (0, 1].";
        return result;
    }

    // Vertices used by faces outside the scope must not move or disappear
    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint8_t> pinned(nv, 0);
    std::vector<Eigen::Index> region;
    std::vector<std::array<int, 3>> outside;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (inScope(faceInScope, f)) {
            region.push_back(f);
        } else {
            outside.push_back({mesh.F(f, 0), mesh.F(f, 1), mesh.F(f, 2)});
            for (int c = 0; c < 3; ++c) {
                pinned[static_cast<std::size_t>(mesh.F(f, c))] = 1;
            }
        }
    }
    result.facesBefore = region.size();
    if (region.empty()) {
        result.statusMessage = "No faces in scope.";
        return result;
    }

    // The region as a Surface_mesh that remembers each vertex's original index
    CgalSurfaceMesh surface;
    auto origMap = surface.add_property_map<CgalVertexIndex, int>("v:mesheal_orig", -1).first;
    std::vector<CgalVertexIndex> local(nv);
    std::vector<std::uint8_t> added(nv, 0);
    for (const Eigen::Index f : region) {
        std::array<CgalVertexIndex, 3> tri{};
        for (int c = 0; c < 3; ++c) {
            const auto v = static_cast<std::size_t>(mesh.F(f, c));
            if (added[v] == 0) {
                local[v] = surface.add_vertex(CgalPoint3(mesh.V(static_cast<Eigen::Index>(v), 0),
                                                         mesh.V(static_cast<Eigen::Index>(v), 1),
                                                         mesh.V(static_cast<Eigen::Index>(v), 2)));
                origMap[local[v]] = static_cast<int>(v);
                added[v] = 1;
            }
            tri[static_cast<std::size_t>(c)] = local[v];
        }
        if (surface.add_face(tri[0], tri[1], tri[2]) == CgalSurfaceMesh::null_face()) {
            result.statusMessage = "The faces to decimate are not manifold: fix the non-manifold edges and vertices first.";
            return result;
        }
    }

    // Constrained: border edges (holes, the region's edge) and every edge at a pinned vertex
    auto constrained = surface.add_property_map<CgalEdgeIndex, bool>("e:mesheal_constrained", false).first;
    for (const CgalEdgeIndex e : surface.edges()) {
        const CgalHalfedgeIndex h = surface.halfedge(e);
        const bool pinnedEnd = pinned[static_cast<std::size_t>(origMap[surface.source(h)])] != 0 ||
                               pinned[static_cast<std::size_t>(origMap[surface.target(h)])] != 0;
        constrained[e] = surface.is_border(e) || pinnedEnd;
    }

    const auto targetFaces = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(ratio * static_cast<double>(region.size()))));
    if (targetFaces < region.size()) {
        using Policies = SMS::GarlandHeckbert_plane_policies<CgalSurfaceMesh, CgalKernel>;
        Policies policies(surface);
        const SMS::Constrained_placement<Policies::Get_placement, decltype(constrained)> placement(constrained,
                                                                                                policies.get_placement());
        const SMS::Face_count_stop_predicate<CgalSurfaceMesh> stop(targetFaces + 1); // stops below the threshold
        const SMS::Bounded_normal_change_filter<> filter; // no collapse may flip a face
        SMS::edge_collapse(surface, stop,
                           CGAL::parameters::edge_is_constrained_map(constrained)
                               .get_cost(policies.get_cost())
                               .get_placement(placement)
                               .filter(filter));
        surface.collect_garbage();
    }

    // Reassemble: surviving region vertices keep their original index (with their new position)
    Eigen::MatrixXd V = mesh.V;
    for (const CgalVertexIndex v : surface.vertices()) {
        const CgalPoint3& p = surface.point(v);
        V.row(origMap[v]) = Eigen::RowVector3d(p.x(), p.y(), p.z());
    }
    std::vector<std::array<int, 3>> faces = std::move(outside);
    faces.reserve(faces.size() + surface.number_of_faces());
    for (const CgalFaceIndex f : surface.faces()) {
        std::array<int, 3> tri{};
        std::size_t c = 0;
        for (const CgalVertexIndex v : CGAL::vertices_around_face(surface.halfedge(f), surface)) {
            tri[c++] = origMap[v];
        }
        faces.push_back(tri);
    }
    result.facesAfter = surface.number_of_faces();
    result.mesh = compactMesh(V, faces, mesh.fileUnit);
    result.success = true;
    result.statusMessage = "Decimated " + std::to_string(result.facesBefore) + " faces to " +
                           std::to_string(result.facesAfter) + ".";
    if (result.facesAfter > targetFaces) {
        result.statusMessage += " Fixed borders and the normal-change filter stopped it above the target of " +
                                std::to_string(targetFaces) + ".";
    }
    return result;
}

} // namespace mesheal
