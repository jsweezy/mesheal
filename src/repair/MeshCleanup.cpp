#include "repair/MeshCleanup.hpp"

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
#include <vector>

namespace ogun {

namespace {

bool inScope(std::span<const std::uint8_t> faceInScope, Eigen::Index f) noexcept {
    return faceInScope.empty() || faceInScope[static_cast<std::size_t>(f)] != 0;
}

/// Mesh of `faces` over `V`, keeping only the vertices they use (in index order).
Mesh compactMesh(const Eigen::MatrixXd& V, const std::vector<std::array<int, 3>>& faces, Unit unit) {
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

} // namespace

MergeByDistanceResult mergeByDistance(const Mesh& mesh, double distance, std::span<const std::uint8_t> faceInScope) {
    MergeByDistanceResult result;
    if (mesh.empty() || !(distance > 0.0) || !std::isfinite(distance)) {
        result.statusMessage = "Merge needs a mesh and a positive distance.";
        return result;
    }

    // Candidates: the vertices of the faces in scope, bucketed in a grid of `distance` cells
    const auto nv = static_cast<std::size_t>(mesh.numVertices());
    std::vector<std::uint8_t> candidate(nv, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (inScope(faceInScope, f)) {
            for (int c = 0; c < 3; ++c) {
                candidate[static_cast<std::size_t>(mesh.F(f, c))] = 1;
            }
        }
    }
    const auto cellOf = [&](Eigen::Index v) {
        return CellKey{static_cast<std::int64_t>(std::floor(mesh.V(v, 0) / distance)),
                       static_cast<std::int64_t>(std::floor(mesh.V(v, 1) / distance)),
                       static_cast<std::int64_t>(std::floor(mesh.V(v, 2) / distance))};
    };
    std::unordered_map<CellKey, std::vector<int>, CellHash> grid;
    for (std::size_t v = 0; v < nv; ++v) {
        if (candidate[v] != 0) {
            grid[cellOf(static_cast<Eigen::Index>(v))].push_back(static_cast<int>(v));
        }
    }

    // Each unmerged vertex, in index order, absorbs the unmerged ones within `distance` of it
    std::vector<int> target(nv);
    for (std::size_t v = 0; v < nv; ++v) {
        target[v] = static_cast<int>(v);
    }
    std::vector<std::uint8_t> assigned(nv, 0);
    const double d2 = distance * distance;
    for (std::size_t v = 0; v < nv; ++v) {
        if (candidate[v] == 0 || assigned[v] != 0) {
            continue;
        }
        assigned[v] = 1;
        const CellKey cell = cellOf(static_cast<Eigen::Index>(v));
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                for (std::int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = grid.find(CellKey{cell.x + dx, cell.y + dy, cell.z + dz});
                    if (it == grid.end()) {
                        continue;
                    }
                    for (const int n : it->second) {
                        const auto un = static_cast<std::size_t>(n);
                        if (assigned[un] != 0) {
                            continue;
                        }
                        const double dist2 = (mesh.V.row(n) - mesh.V.row(static_cast<Eigen::Index>(v))).squaredNorm();
                        if (dist2 <= d2) {
                            assigned[un] = 1;
                            target[un] = static_cast<int>(v);
                            ++result.verticesMerged;
                            result.maxMove = std::max(result.maxMove, std::sqrt(dist2));
                        }
                    }
                }
            }
        }
    }

    // Remap faces; drop the collapsed ones
    std::vector<std::array<int, 3>> faces;
    std::vector<std::uint8_t> touched;
    faces.reserve(static_cast<std::size_t>(mesh.numFaces()));
    touched.reserve(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        std::array<int, 3> face{};
        bool moved = false;
        for (int c = 0; c < 3; ++c) {
            const int v = mesh.F(f, c);
            face[static_cast<std::size_t>(c)] = target[static_cast<std::size_t>(v)];
            moved = moved || face[static_cast<std::size_t>(c)] != v;
        }
        if (face[0] == face[1] || face[1] == face[2] || face[0] == face[2]) {
            ++result.degenerateRemoved;
            continue;
        }
        faces.push_back(face);
        touched.push_back(moved ? 1 : 0);
    }

    // Faces that became copies: keep one of a same-winding group, drop folds (both windings) entirely
    std::unordered_map<std::array<int, 3>, std::vector<std::size_t>, TriangleKeyHash> copies;
    for (std::size_t f = 0; f < faces.size(); ++f) {
        std::array<int, 3> key = faces[f];
        std::sort(key.begin(), key.end());
        copies[key].push_back(f);
    }
    std::vector<std::uint8_t> drop(faces.size(), 0);
    for (const auto& [key, group] : copies) {
        if (group.size() < 2 || std::none_of(group.begin(), group.end(), [&](std::size_t f) { return touched[f] != 0; })) {
            continue;
        }
        const bool fold = std::any_of(group.begin() + 1, group.end(),
                                      [&](std::size_t f) { return !sameWinding(faces[group.front()], faces[f]); });
        for (std::size_t i = fold ? 0 : 1; i < group.size(); ++i) {
            drop[group[i]] = 1;
            ++result.duplicatesRemoved;
        }
    }
    std::vector<std::array<int, 3>> kept;
    kept.reserve(faces.size());
    for (std::size_t f = 0; f < faces.size(); ++f) {
        if (drop[f] == 0) {
            kept.push_back(faces[f]);
        }
    }

    result.mesh = compactMesh(mesh.V, kept, mesh.fileUnit);
    result.success = true;
    std::array<char, 160> text{};
    std::snprintf(text.data(), text.size(), "Merged %zu vertices (moved at most %.3g); removed %zu collapsed and %zu duplicate faces.",
                  result.verticesMerged, result.maxMove, result.degenerateRemoved, result.duplicatesRemoved);
    result.statusMessage = text.data();
    return result;
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
    auto origMap = surface.add_property_map<CgalVertexIndex, int>("v:ogun_orig", -1).first;
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
    auto constrained = surface.add_property_map<CgalEdgeIndex, bool>("e:ogun_constrained", false).first;
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
        const SMS::Face_count_stop_predicate<CgalSurfaceMesh> stop(targetFaces);
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

} // namespace ogun
