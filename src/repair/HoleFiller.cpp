#include "repair/HoleFiller.hpp"

#include "core/Constants.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "mesh/CgalMesh.hpp"

#include <Eigen/Geometry>

#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace ogun {

namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

using FaceKey = std::array<int, 3>;

/// Corners rotated so the smallest vertex comes first (keeps the winding).
FaceKey canonical(const FaceKey& f) noexcept {
    const auto first = static_cast<std::size_t>(std::min_element(f.begin(), f.end()) - f.begin());
    return {f[first], f[(first + 1) % 3], f[(first + 2) % 3]};
}

struct FaceKeyHash {
    std::size_t operator()(const FaceKey& k) const noexcept {
        std::size_t h = 0;
        for (const int v : k) {
            h = h * 1000003u ^ std::hash<int>{}(v);
        }
        return h;
    }
};

struct EdgeHash {
    std::size_t operator()(const std::array<int, 2>& e) const noexcept {
        return std::hash<long long>{}((static_cast<long long>(e[0]) << 32) ^ static_cast<long long>(e[1]));
    }
};

struct Cancelled : std::exception {};

/// Aborts CGAL's hole search when a stop is requested. CGAL copies the visitor, so it holds a pointer.
struct StopVisitor : PMP::Hole_filling::Default_visitor {
    const std::stop_token* stop = nullptr;

    void check() const {
        if (stop != nullptr && stop->stop_requested()) {
            throw Cancelled{};
        }
    }
    void quadratic_step() const { check(); }
    void cubic_step() const { check(); }
};

/// The mesh being edited: the input plus appended vertices and faces, removed faces and moved
/// vertices. Input indices never change, so the holes found on the input stay valid.
class Workspace {
public:
    explicit Workspace(const Mesh& mesh)
        : input_(mesh),
          offsets_(static_cast<std::size_t>(mesh.numVertices()) + 1, 0),
          removed_(static_cast<std::size_t>(mesh.numFaces()), 0) {
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                ++offsets_[static_cast<std::size_t>(mesh.F(f, c)) + 1];
            }
        }
        std::partial_sum(offsets_.begin(), offsets_.end(), offsets_.begin());
        around_.resize(offsets_.back());
        std::vector<std::uint32_t> fill(offsets_.begin(), offsets_.end() - 1);
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            for (int c = 0; c < 3; ++c) {
                around_[fill[static_cast<std::size_t>(mesh.F(f, c))]++] = f;
            }
        }
    }

    [[nodiscard]] Eigen::Index inputFaces() const noexcept {
        return input_.numFaces();
    }

    [[nodiscard]] Eigen::Vector3d point(int v) const {
        if (v >= input_.numVertices()) {
            return points_[static_cast<std::size_t>(v - input_.numVertices())];
        }
        const auto it = moved_.find(v);
        return it != moved_.end() ? it->second : Eigen::Vector3d(input_.V.row(v).transpose());
    }

    [[nodiscard]] FaceKey face(Eigen::Index f) const {
        if (f < input_.numFaces()) {
            return {input_.F(f, 0), input_.F(f, 1), input_.F(f, 2)};
        }
        return faces_[static_cast<std::size_t>(f - input_.numFaces())];
    }

    [[nodiscard]] bool alive(Eigen::Index f) const {
        return f < input_.numFaces() ? removed_[static_cast<std::size_t>(f)] == 0
                                     : !removedNew_.contains(f);
    }

    /// Live faces around vertex `v`.
    void facesAround(int v, std::vector<Eigen::Index>& out) const {
        out.clear();
        if (v < input_.numVertices()) {
            const auto i = static_cast<std::size_t>(v);
            for (std::uint32_t k = offsets_[i]; k < offsets_[i + 1]; ++k) {
                if (alive(around_[k])) {
                    out.push_back(around_[k]);
                }
            }
        }
        if (const auto it = extra_.find(v); it != extra_.end()) {
            for (const Eigen::Index f : it->second) {
                if (alive(f)) {
                    out.push_back(f);
                }
            }
        }
    }

    int addVertex(const Eigen::Vector3d& p) {
        points_.push_back(p);
        return static_cast<int>(input_.numVertices() + static_cast<Eigen::Index>(points_.size()) - 1);
    }

    void moveVertex(int v, const Eigen::Vector3d& p) {
        if (v < input_.numVertices()) {
            const auto it = moved_.find(v);
            moveLog_.push_back({v, it != moved_.end(), it != moved_.end() ? it->second : Eigen::Vector3d::Zero()});
            moved_[v] = p;
        } else {
            Eigen::Vector3d& slot = points_[static_cast<std::size_t>(v - input_.numVertices())];
            moveLog_.push_back({v, true, slot});
            slot = p;
        }
    }

    Eigen::Index addFace(const FaceKey& f) {
        const Eigen::Index id = input_.numFaces() + static_cast<Eigen::Index>(faces_.size());
        faces_.push_back(f);
        for (const int v : f) {
            extra_[v].push_back(id);
        }
        return id;
    }

    void removeFace(Eigen::Index f) {
        if (f < input_.numFaces()) {
            removed_[static_cast<std::size_t>(f)] = 1;
        } else {
            removedNew_.insert(f);
        }
        removeLog_.push_back(f);
    }

    /// A point to roll the workspace back to (a fill tried and not kept).
    struct Mark {
        std::size_t points = 0;
        std::size_t faces = 0;
        std::size_t removes = 0;
        std::size_t moves = 0;
    };

    [[nodiscard]] Mark mark() const noexcept {
        return {points_.size(), faces_.size(), removeLog_.size(), moveLog_.size()};
    }

    /// Undoes every change made since `m`.
    void rollback(const Mark& m) {
        for (std::size_t k = moveLog_.size(); k > m.moves; --k) {
            const Move& move = moveLog_[k - 1];
            if (move.v < input_.numVertices()) {
                if (move.had) {
                    moved_[move.v] = move.previous;
                } else {
                    moved_.erase(move.v);
                }
            } else if (const auto i = static_cast<std::size_t>(move.v - input_.numVertices()); i < points_.size()) {
                points_[i] = move.previous;
            }
        }
        moveLog_.resize(m.moves);
        for (std::size_t k = removeLog_.size(); k > m.removes; --k) {
            const Eigen::Index f = removeLog_[k - 1];
            if (f < input_.numFaces()) {
                removed_[static_cast<std::size_t>(f)] = 0;
            } else {
                removedNew_.erase(f);
            }
        }
        removeLog_.resize(m.removes);
        for (std::size_t k = faces_.size(); k > m.faces; --k) {
            for (const int v : faces_[k - 1]) {
                extra_[v].pop_back(); // faces are appended in id order, so each vertex's last entry is this face
            }
        }
        faces_.resize(m.faces);
        points_.resize(m.points);
    }

    /// Input faces removed and input vertices moved since `m`.
    void changesSince(const Mark& m, std::vector<Eigen::Index>& removedInput, std::vector<int>& movedInput) const {
        for (std::size_t k = m.removes; k < removeLog_.size(); ++k) {
            if (removeLog_[k] < input_.numFaces()) {
                removedInput.push_back(removeLog_[k]);
            }
        }
        for (std::size_t k = m.moves; k < moveLog_.size(); ++k) {
            if (moveLog_[k].v < input_.numVertices()) {
                movedInput.push_back(moveLog_[k].v);
            }
        }
    }

    /// Input vertices moved by any fill so far.
    [[nodiscard]] const std::unordered_map<int, Eigen::Vector3d>& movedInputVertices() const noexcept {
        return moved_;
    }

    [[nodiscard]] Eigen::Index totalFaces() const noexcept {
        return input_.numFaces() + static_cast<Eigen::Index>(faces_.size());
    }

    [[nodiscard]] const Mesh& input() const noexcept {
        return input_;
    }

    /// Live faces (input order, then appended) with unreferenced vertices dropped. `faceOf` receives
    /// the output index of every workspace face (-1 when removed).
    [[nodiscard]] Mesh build(std::vector<Eigen::Index>* faceOf = nullptr) const {
        const Eigen::Index total = input_.numFaces() + static_cast<Eigen::Index>(faces_.size());
        const Eigen::Index nv = input_.numVertices() + static_cast<Eigen::Index>(points_.size());
        std::vector<int> vertexOf(static_cast<std::size_t>(nv), -1);
        std::vector<FaceKey> kept;
        kept.reserve(static_cast<std::size_t>(total));
        if (faceOf != nullptr) {
            faceOf->assign(static_cast<std::size_t>(total), -1);
        }
        for (Eigen::Index f = 0; f < total; ++f) {
            if (!alive(f)) {
                continue;
            }
            if (faceOf != nullptr) {
                (*faceOf)[static_cast<std::size_t>(f)] = static_cast<Eigen::Index>(kept.size());
            }
            kept.push_back(face(f));
            for (const int v : kept.back()) {
                vertexOf[static_cast<std::size_t>(v)] = 0;
            }
        }
        int next = 0;
        for (int& slot : vertexOf) {
            slot = slot == 0 ? next++ : -1;
        }
        Mesh out;
        out.fileUnit = input_.fileUnit;
        out.V.resize(next, 3);
        for (int v = 0; v < static_cast<int>(nv); ++v) {
            if (vertexOf[static_cast<std::size_t>(v)] >= 0) {
                out.V.row(vertexOf[static_cast<std::size_t>(v)]) = point(v).transpose();
            }
        }
        out.F.resize(static_cast<Eigen::Index>(kept.size()), 3);
        for (std::size_t f = 0; f < kept.size(); ++f) {
            for (std::size_t c = 0; c < 3; ++c) {
                out.F(static_cast<Eigen::Index>(f), static_cast<Eigen::Index>(c)) = vertexOf[static_cast<std::size_t>(kept[f][c])];
            }
        }
        out.recomputeNormals();
        return out;
    }

private:
    const Mesh& input_;
    std::vector<std::uint32_t> offsets_;
    std::vector<Eigen::Index> around_;
    std::vector<std::uint8_t> removed_;
    std::unordered_set<Eigen::Index> removedNew_;
    std::vector<Eigen::Vector3d> points_;
    std::unordered_map<int, Eigen::Vector3d> moved_;
    std::vector<FaceKey> faces_;
    std::unordered_map<int, std::vector<Eigen::Index>> extra_;

    struct Move {
        int v = 0;
        bool had = false; ///< An input vertex that had moved before (a new vertex always has a position)
        Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    };
    std::vector<Eigen::Index> removeLog_;
    std::vector<Move> moveLog_;
};

/// Corner positions of a face, rotated so the smallest corner comes first (keeps the winding).
using GeometryKey = std::array<double, 9>;

struct GeometryKeyHash {
    std::size_t operator()(const GeometryKey& k) const noexcept {
        std::size_t h = 0;
        for (const double x : k) {
            h = h * 1000003u ^ std::hash<double>{}(x);
        }
        return h;
    }
};

GeometryKey geometryKey(const Workspace& ws, Eigen::Index f) {
    const FaceKey corners = ws.face(f);
    std::array<Eigen::Vector3d, 3> p{ws.point(corners[0]), ws.point(corners[1]), ws.point(corners[2])};
    const auto less = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3);
    };
    const std::size_t first = less(p[1], p[0]) ? (less(p[2], p[1]) ? 2 : 1) : (less(p[2], p[0]) ? 2 : 0);
    GeometryKey key{};
    for (std::size_t c = 0; c < 3; ++c) {
        const Eigen::Vector3d& q = p[(first + c) % 3];
        std::copy(q.data(), q.data() + 3, key.begin() + static_cast<std::ptrdiff_t>(3 * c));
    }
    return key;
}

/// What one fill changed in the workspace.
struct FillReport {
    bool changed = false;
    std::string status;
    std::vector<Eigen::Index> added;    ///< Workspace face ids of new geometry
    std::vector<Eigen::Index> removed;  ///< Workspace face ids of geometry that is gone
    std::size_t verticesAdded = 0;
    std::size_t openEdgesLeft = 0;
    double maxVertexMove = 0.0;
};

/// Faces within `rings` vertex rings of the rim, copied into a halfedge mesh whose vertex and face
/// indices are the local ones.
struct LocalPatch {
    CgalSurfaceMesh sm;
    std::vector<int> globalVertex;
    std::vector<Eigen::Index> globalFace;
    std::unordered_map<int, CgalVertexIndex> localOf;
};

bool extractPatch(const Workspace& ws, const std::vector<int>& rim, int rings, LocalPatch& patch) {
    std::unordered_set<int> seenVertices(rim.begin(), rim.end());
    std::unordered_set<Eigen::Index> seenFaces;
    std::vector<int> frontier = rim;
    std::vector<Eigen::Index> around;
    for (int ring = 0; ring < std::max(rings, 1); ++ring) {
        std::vector<int> next;
        for (const int v : frontier) {
            ws.facesAround(v, around);
            for (const Eigen::Index f : around) {
                if (!seenFaces.insert(f).second) {
                    continue;
                }
                patch.globalFace.push_back(f);
                for (const int w : ws.face(f)) {
                    if (seenVertices.insert(w).second) {
                        next.push_back(w);
                    }
                }
            }
        }
        frontier = std::move(next);
    }
    std::sort(patch.globalFace.begin(), patch.globalFace.end());
    for (const Eigen::Index f : patch.globalFace) {
        FaceKey local{};
        for (std::size_t c = 0; c < 3; ++c) {
            const int g = ws.face(f)[c];
            auto it = patch.localOf.find(g);
            if (it == patch.localOf.end()) {
                const Eigen::Vector3d p = ws.point(g);
                it = patch.localOf.emplace(g, patch.sm.add_vertex(CgalPoint3(p.x(), p.y(), p.z()))).first;
                patch.globalVertex.push_back(g);
            }
            local[c] = static_cast<int>(it->second.idx());
        }
        if (patch.sm.add_face(CgalVertexIndex(static_cast<std::uint32_t>(local[0])), CgalVertexIndex(static_cast<std::uint32_t>(local[1])),
                              CgalVertexIndex(static_cast<std::uint32_t>(local[2]))) == CgalSurfaceMesh::null_face()) {
            return false;
        }
    }
    return true;
}

Eigen::Vector3d toEigen(const CgalPoint3& p) {
    return {CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())};
}

/// Snaps the crack's rim vertices onto the rim edges across the gap and stitches the coincident
/// edges. Only rim vertices move, by at most `limit` (and 0.9 x their shortest edge, like CGAL's default).
std::size_t closeCrack(CgalSurfaceMesh& sm, const std::vector<CgalHalfedgeIndex>& cycles, double limit) {
    using ToleranceTag = CGAL::dynamic_vertex_property_t<double>;
    auto tolerance = get(ToleranceTag(), sm);
    for (const CgalVertexIndex v : sm.vertices()) {
        put(tolerance, v, 0.0);
    }
    for (const CgalHalfedgeIndex h : cycles) {
        for (const CgalHalfedgeIndex b : CGAL::halfedges_around_face(h, sm)) {
            const CgalVertexIndex v = sm.target(b);
            double shortest = std::numeric_limits<double>::infinity();
            for (const CgalHalfedgeIndex a : CGAL::halfedges_around_target(v, sm)) {
                shortest = std::min(shortest, std::sqrt(CGAL::to_double(CGAL::squared_distance(sm.point(sm.source(a)), sm.point(v)))));
            }
            put(tolerance, v, std::min(0.9 * shortest, limit));
        }
    }
    const std::size_t snapped = PMP::experimental::snap_borders(sm, tolerance);
    return snapped + PMP::stitch_borders(sm);
}

/// Applies `method` to `hole` in `ws`. The workspace is unchanged on failure.
FillReport applyFill(Workspace& ws, const HoleLoop& hole, HoleFillMethod method, const HoleFillSettings& settings,
                     const std::stop_token& stopToken) {
    FillReport report;
    if (method == HoleFillMethod::None) {
        report.status = "Pick a fill method first.";
        return report;
    }
    if (!hole.closed) {
        report.status = "These open edges do not form a closed loop (they end at a non-manifold edge): fix the "
                        "non-manifold edges first.";
        return report;
    }
    LocalPatch patch;
    if (!extractPatch(ws, hole.vertices, settings.patchRings, patch)) {
        report.status = "The faces around this hole are not manifold (a fin or a vertex shared by separate fans): "
                        "fix the non-manifold edges and vertices first.";
        return report;
    }
    CgalSurfaceMesh& sm = patch.sm;
    sm.set_recycle_garbage(false);
    const auto nLocalVertices = static_cast<std::uint32_t>(sm.number_of_vertices());

    // The hole's border cycles, and every other border edge of the patch (its outer rim, other holes)
    std::unordered_set<std::uint32_t> holeHalfedges;
    std::vector<CgalHalfedgeIndex> cycles;
    for (const auto& [a, b] : hole.edges) {
        const auto la = patch.localOf.find(a);
        const auto lb = patch.localOf.find(b);
        if (la == patch.localOf.end() || lb == patch.localOf.end()) {
            continue;
        }
        const CgalHalfedgeIndex h = sm.halfedge(la->second, lb->second);
        if (h == CgalSurfaceMesh::null_halfedge() || !sm.is_border(h) || holeHalfedges.count(h.idx()) != 0) {
            continue;
        }
        cycles.push_back(h);
        for (const CgalHalfedgeIndex c : CGAL::halfedges_around_face(h, sm)) {
            holeHalfedges.insert(c.idx());
        }
    }
    if (cycles.empty()) {
        report.status = "This hole is already closed.";
        return report;
    }
    std::unordered_set<std::array<int, 2>, EdgeHash> otherBorder;
    for (const CgalHalfedgeIndex h : sm.halfedges()) {
        if (sm.is_border(h) && holeHalfedges.count(h.idx()) == 0) {
            otherBorder.insert({patch.globalVertex[sm.source(h).idx()], patch.globalVertex[sm.target(h).idx()]});
        }
    }

    StopVisitor visitor;
    visitor.stop = &stopToken;
    const bool skipCubic = hole.edgeCount() > settings::kMaxCubicHoleFillEdges;
    std::vector<CgalFaceIndex> newFaces;
    std::vector<CgalVertexIndex> newVertices;
    bool fairFailed = false;
    try {
        if (method == HoleFillMethod::CloseCrack) {
            const double limit = std::max(settings.crackSnapFactor * hole.width, settings::kMinCrackSnapEdgeShare * hole.meanEdgeLength);
            if (closeCrack(sm, cycles, limit) == 0) {
                report.status = "Nothing within the snap distance (" + std::to_string(limit) +
                                ") across the gap: this hole is not a crack. Try a fill.";
                return report;
            }
        } else {
            for (const CgalHalfedgeIndex h : cycles) {
                const auto np = CGAL::parameters::use_2d_constrained_delaunay_triangulation(true)
                                    .do_not_use_cubic_algorithm(skipCubic)
                                    .visitor(visitor)
                                    .face_output_iterator(std::back_inserter(newFaces))
                                    .vertex_output_iterator(std::back_inserter(newVertices));
                const std::size_t before = newFaces.size();
                if (method == HoleFillMethod::Flat) {
                    PMP::triangulate_hole(sm, h, np);
                } else {
                    PMP::triangulate_and_refine_hole(sm, h, np);
                }
                if (newFaces.size() == before) {
                    report.status = "CGAL found no valid triangulation for this hole";
                    report.status += hole.shape == HoleShape::Crack ? " (every triangle across a zero-width gap is degenerate): try Close Crack."
                                     : skipCubic ? " in its Delaunay search space; the full O(n^3) search is skipped for holes over " +
                                                       std::to_string(settings::kMaxCubicHoleFillEdges) + " edges."
                                                 : ".";
                    return report;
                }
            }
            if (method != HoleFillMethod::Flat && newFaces.size() > settings.maxPatchFaces) {
                report.status = std::string(holeFillMethodLabel(method)) + " would add " + std::to_string(newFaces.size()) +
                                " faces (limit " + std::to_string(settings.maxPatchFaces) +
                                "): the refined patch matches the small rim edges. Use Fill Flat or raise the limit.";
                return report;
            }
            if (method == HoleFillMethod::Smooth) {
                fairFailed = !PMP::fair(sm, newVertices);
            }
        }
    } catch (const Cancelled&) {
        report.status = "Cancelled.";
        return report;
    } catch (const std::exception& e) {
        report.status = std::string("CGAL failed: ") + e.what();
        return report;
    }
    if (stopToken.stop_requested()) {
        report.status = "Cancelled.";
        return report;
    }

    // Map the patch back: new vertices are appended, moved ones updated, faces diffed by corners
    std::unordered_map<Eigen::Index, GeometryKey> oldGeometry;
    for (const Eigen::Index f : patch.globalFace) {
        oldGeometry.emplace(f, geometryKey(ws, f));
    }
    std::vector<int> globalOf(sm.number_of_vertices() + sm.number_of_removed_vertices(), -1);
    std::unordered_set<int> moved;
    for (const CgalVertexIndex v : sm.vertices()) {
        const Eigen::Vector3d p = toEigen(sm.point(v));
        if (v.idx() < nLocalVertices) {
            const int g = patch.globalVertex[v.idx()];
            globalOf[v.idx()] = g;
            const double move = (p - ws.point(g)).norm();
            if (move > 0.0) {
                ws.moveVertex(g, p);
                moved.insert(g);
                report.maxVertexMove = std::max(report.maxVertexMove, move);
            }
        } else {
            globalOf[v.idx()] = ws.addVertex(p);
            ++report.verticesAdded;
        }
    }
    std::unordered_map<FaceKey, int, FaceKeyHash> before;
    for (const Eigen::Index f : patch.globalFace) {
        ++before[canonical(ws.face(f))];
    }
    std::vector<FaceKey> added;
    for (const CgalFaceIndex f : sm.faces()) {
        FaceKey key{};
        std::size_t c = 0;
        for (const CgalVertexIndex v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
            key[c++] = globalOf[v.idx()];
        }
        key = canonical(key);
        const auto it = before.find(key);
        if (it != before.end() && it->second > 0) {
            --it->second;
        } else {
            added.push_back(key);
        }
    }
    std::vector<Eigen::Index> removedIds;
    std::vector<Eigen::Index> addedIds;
    for (const Eigen::Index f : patch.globalFace) {
        const FaceKey corners = ws.face(f);
        const auto it = before.find(canonical(corners));
        if (it != before.end() && it->second > 0) {
            --it->second;
            ws.removeFace(f);
            removedIds.push_back(f);
        } else if (std::any_of(corners.begin(), corners.end(), [&](int v) { return moved.contains(v); })) {
            removedIds.push_back(f); // kept, but its corners moved: reported as replaced
            addedIds.push_back(f);
        }
    }
    for (const FaceKey& key : added) {
        addedIds.push_back(ws.addFace(key));
    }

    // Report geometric changes only: a face renumbered by a vertex merge is not a change
    std::unordered_map<GeometryKey, int, GeometryKeyHash> removedGeometry;
    std::unordered_map<GeometryKey, int, GeometryKeyHash> addedGeometry;
    for (const Eigen::Index f : removedIds) {
        ++removedGeometry[oldGeometry.at(f)];
    }
    for (const Eigen::Index f : addedIds) {
        ++addedGeometry[geometryKey(ws, f)];
    }
    for (const Eigen::Index f : removedIds) {
        const auto it = addedGeometry.find(oldGeometry.at(f));
        if (it != addedGeometry.end() && it->second > 0) {
            --it->second;
        } else {
            report.removed.push_back(f);
        }
    }
    for (const Eigen::Index f : addedIds) {
        const auto it = removedGeometry.find(geometryKey(ws, f));
        if (it != removedGeometry.end() && it->second > 0) {
            --it->second;
        } else {
            report.added.push_back(f);
        }
    }
    for (const CgalHalfedgeIndex h : sm.halfedges()) {
        if (sm.is_border(h) && !otherBorder.contains({globalOf[sm.source(h).idx()], globalOf[sm.target(h).idx()]})) {
            ++report.openEdgesLeft;
        }
    }
    report.changed = !removedIds.empty() || !addedIds.empty();

    const std::string label = holeFillMethodLabel(method);
    if (method == HoleFillMethod::CloseCrack) {
        report.status = "Closed the crack: vertices moved at most " + std::to_string(report.maxVertexMove) + ".";
    } else {
        report.status = label + ": " + std::to_string(report.added.size()) + " new faces, " +
                        std::to_string(report.verticesAdded) + " new vertices.";
    }
    if (fairFailed) {
        report.status += " Fairing failed; the refined patch was kept.";
    }
    if (report.openEdgesLeft > 0) {
        report.status += " " + std::to_string(report.openEdgesLeft) + " edges are still open.";
    }
    return report;
}

double minAngleDeg(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    double best = 180.0;
    for (const Eigen::Index f : faces) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = mesh.V.row(mesh.F(f, c)).transpose();
            const Eigen::Vector3d u = Eigen::Vector3d(mesh.V.row(mesh.F(f, (c + 1) % 3)).transpose()) - p;
            const Eigen::Vector3d v = Eigen::Vector3d(mesh.V.row(mesh.F(f, (c + 2) % 3)).transpose()) - p;
            const double denom = u.norm() * v.norm();
            best = std::min(best, denom > 0.0 ? std::acos(std::clamp(u.dot(v) / denom, -1.0, 1.0)) * kRadToDeg : 0.0);
        }
    }
    return best;
}

double facesArea(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    double area = 0.0;
    for (const Eigen::Index f : faces) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0)).transpose();
        area += 0.5 * (Eigen::Vector3d(mesh.V.row(mesh.F(f, 1)).transpose()) - a)
                          .cross(Eigen::Vector3d(mesh.V.row(mesh.F(f, 2)).transpose()) - a)
                          .norm();
    }
    return area;
}

/// New faces that cross any other face (of any shell): only the faces near the patch are checked,
/// with the tolerance of the whole mesh.
std::size_t countNewCrossings(const Mesh& mesh, const std::vector<Eigen::Index>& added, const CheckParams& params) {
    if (added.empty()) {
        return 0;
    }
    const auto faceBox = [&](Eigen::Index f) {
        Eigen::AlignedBox3d box;
        for (int c = 0; c < 3; ++c) {
            box.extend(Eigen::Vector3d(mesh.V.row(mesh.F(f, c)).transpose()));
        }
        return box;
    };
    const double diagonal = mesh.boundingBox().diagonal();
    Eigen::AlignedBox3d region;
    for (const Eigen::Index f : added) {
        region.extend(faceBox(f));
    }
    const double grow = params.intersectionTolerance * diagonal;
    region.min().array() -= grow;
    region.max().array() += grow;
    std::vector<Eigen::Index> nearby;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (region.intersects(faceBox(f))) {
            nearby.push_back(f);
        }
    }
    const Mesh local = extractFaces(mesh, nearby);
    CheckParams localParams = params;
    const double localDiagonal = local.boundingBox().diagonal();
    if (localDiagonal > 0.0) {
        localParams.intersectionTolerance = params.intersectionTolerance * diagonal / localDiagonal;
    }
    const std::unordered_set<Eigen::Index> isNew(added.begin(), added.end());
    const CategoryResult crossings = checkSelfIntersections(local, localParams);
    std::size_t count = 0;
    for (std::size_t e = 0; e < crossings.size(); ++e) {
        const auto own = crossings.facesOf(e);
        count += !own.empty() && isNew.contains(nearby[static_cast<std::size_t>(own[0])]) ? 1 : 0;
    }
    return count;
}

/// Uniform grid of the input faces' boxes, so the crossing test of a patch looks only at faces near it.
class FaceGrid {
public:
    explicit FaceGrid(const Mesh& mesh) : stamp_(static_cast<std::size_t>(mesh.numFaces()), 0) {
        const Eigen::Index nf = mesh.numFaces();
        if (nf == 0) {
            return;
        }
        for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
            if (mesh.V.row(v).allFinite()) {
                bounds_.extend(Eigen::Vector3d(mesh.V.row(v).transpose()));
            }
        }
        if (bounds_.isEmpty()) {
            return;
        }
        const int perAxis = std::clamp(static_cast<int>(std::ceil(std::cbrt(static_cast<double>(nf)))), 1,
                                       settings::kHoleCrossingGridMaxCellsPerAxis);
        const Eigen::Vector3d extent = bounds_.sizes();
        const double cell = std::max(extent.maxCoeff() / perAxis, std::numeric_limits<double>::min());
        for (int a = 0; a < 3; ++a) {
            cells_[static_cast<std::size_t>(a)] = std::clamp(static_cast<int>(std::ceil(extent[a] / cell)), 1, perAxis);
            cellSize_[a] = extent[a] > 0.0 ? extent[a] / cells_[static_cast<std::size_t>(a)] : 1.0;
        }
        offsets_.assign(static_cast<std::size_t>(cells_[0]) * static_cast<std::size_t>(cells_[1]) * static_cast<std::size_t>(cells_[2]) + 1, 0);
        std::vector<std::array<int, 6>> ranges(static_cast<std::size_t>(nf));
        for (Eigen::Index f = 0; f < nf; ++f) {
            Eigen::AlignedBox3d box;
            for (int c = 0; c < 3; ++c) {
                box.extend(Eigen::Vector3d(mesh.V.row(mesh.F(f, c)).transpose()));
            }
            auto& r = ranges[static_cast<std::size_t>(f)];
            if (!box.min().allFinite() || !box.max().allFinite()) {
                r = {0, -1, 0, -1, 0, -1};
                large_.push_back(f);
                continue;
            }
            r = cellRange(box);
            const long count = static_cast<long>(r[1] - r[0] + 1) * (r[3] - r[2] + 1) * (r[5] - r[4] + 1);
            if (count > settings::kHoleCrossingGridMaxCellsPerFace) {
                large_.push_back(f);
                r = {0, -1, 0, -1, 0, -1};
                continue;
            }
            forEachCell(r, [&](std::size_t c) { ++offsets_[c + 1]; });
        }
        std::partial_sum(offsets_.begin(), offsets_.end(), offsets_.begin());
        faces_.resize(offsets_.back());
        std::vector<std::size_t> fill(offsets_.begin(), offsets_.end() - 1);
        for (Eigen::Index f = 0; f < nf; ++f) {
            forEachCell(ranges[static_cast<std::size_t>(f)], [&](std::size_t c) { faces_[fill[c]++] = f; });
        }
    }

    /// Input faces whose (input) box may meet `region`, each once.
    void query(const Eigen::AlignedBox3d& region, std::vector<Eigen::Index>& out) {
        out.clear();
        if (++generation_ == 0) {
            std::fill(stamp_.begin(), stamp_.end(), 0);
            generation_ = 1;
        }
        const auto take = [&](Eigen::Index f) {
            std::uint32_t& s = stamp_[static_cast<std::size_t>(f)];
            if (s != generation_) {
                s = generation_;
                out.push_back(f);
            }
        };
        if (!offsets_.empty() && region.intersects(bounds_)) {
            forEachCell(cellRange(region), [&](std::size_t c) {
                for (std::size_t k = offsets_[c]; k < offsets_[c + 1]; ++k) {
                    take(faces_[k]);
                }
            });
        }
        for (const Eigen::Index f : large_) {
            take(f);
        }
    }

private:
    [[nodiscard]] std::array<int, 6> cellRange(const Eigen::AlignedBox3d& box) const {
        std::array<int, 6> r{};
        for (int a = 0; a < 3; ++a) {
            const int n = cells_[static_cast<std::size_t>(a)];
            const auto cellOf = [&](double x) {
                const double t = std::floor((x - bounds_.min()[a]) / cellSize_[a]);
                return static_cast<int>(std::clamp(t, 0.0, static_cast<double>(n - 1)));
            };
            r[static_cast<std::size_t>(2 * a)] = cellOf(box.min()[a]);
            r[static_cast<std::size_t>(2 * a + 1)] = cellOf(box.max()[a]);
        }
        return r;
    }

    template <typename Fn>
    void forEachCell(const std::array<int, 6>& r, Fn&& fn) const {
        for (int z = r[4]; z <= r[5]; ++z) {
            for (int y = r[2]; y <= r[3]; ++y) {
                for (int x = r[0]; x <= r[1]; ++x) {
                    fn((static_cast<std::size_t>(z) * static_cast<std::size_t>(cells_[1]) + static_cast<std::size_t>(y)) *
                           static_cast<std::size_t>(cells_[0]) + static_cast<std::size_t>(x));
                }
            }
        }
    }

    Eigen::AlignedBox3d bounds_;
    std::array<int, 3> cells_{1, 1, 1};
    Eigen::Vector3d cellSize_ = Eigen::Vector3d::Ones();
    std::vector<std::size_t> offsets_;
    std::vector<Eigen::Index> faces_;
    std::vector<Eigen::Index> large_;
    std::vector<std::uint32_t> stamp_;
    std::uint32_t generation_ = 0;
};

/// Crossings of one fill, split by shell.
struct FillCrossings {
    std::size_t own = 0;
    std::size_t other = 0;
    std::map<int, std::size_t> perOtherShell;
};

/// Crossings between the fill's new faces (`added`, workspace ids) and every live face near them,
/// including the earlier fills and the patch itself. `shellOf` gives the shell of a workspace face.
FillCrossings countFillCrossings(
    const Workspace& ws,
    FaceGrid& grid,
    const std::vector<Eigen::Index>& added,
    const std::function<int(Eigen::Index)>& shellOf,
    const std::vector<int>& ownShells,
    double tolerance) {
    FillCrossings out;
    if (added.empty()) {
        return out;
    }
    const auto faceBox = [&](Eigen::Index f) {
        Eigen::AlignedBox3d box;
        for (const int v : ws.face(f)) {
            box.extend(ws.point(v));
        }
        return box;
    };
    Eigen::AlignedBox3d region;
    for (const Eigen::Index f : added) {
        region.extend(faceBox(f));
    }
    region.min().array() -= tolerance;
    region.max().array() += tolerance;

    // Candidates: input faces from the grid, faces around moved input vertices, and every appended face
    std::vector<Eigen::Index> candidates;
    grid.query(region, candidates);
    std::vector<Eigen::Index> around;
    for (const auto& [v, p] : ws.movedInputVertices()) {
        ws.facesAround(v, around);
        candidates.insert(candidates.end(), around.begin(), around.end());
    }
    for (Eigen::Index f = ws.inputFaces(); f < ws.totalFaces(); ++f) {
        candidates.push_back(f);
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    const std::unordered_set<Eigen::Index> isNew(added.begin(), added.end());

    std::vector<Eigen::Index> local(added.begin(), added.end());
    for (const Eigen::Index f : candidates) {
        if (!isNew.contains(f) && ws.alive(f) && region.intersects(faceBox(f))) {
            local.push_back(f);
        }
    }
    std::unordered_map<int, int> vertexOf;
    std::vector<Eigen::Vector3d> points;
    Mesh mesh;
    mesh.F.resize(static_cast<Eigen::Index>(local.size()), 3);
    for (std::size_t i = 0; i < local.size(); ++i) {
        const FaceKey corners = ws.face(local[i]);
        for (std::size_t c = 0; c < 3; ++c) {
            const auto [it, inserted] = vertexOf.emplace(corners[c], static_cast<int>(points.size()));
            if (inserted) {
                points.push_back(ws.point(corners[c]));
            }
            mesh.F(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(c)) = it->second;
        }
    }
    mesh.V.resize(static_cast<Eigen::Index>(points.size()), 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        mesh.V.row(static_cast<Eigen::Index>(i)) = points[i].transpose();
    }

    // Each new face is its own group and the rest one group, so old pairs are not tested
    std::vector<int> group(local.size(), 0);
    std::vector<std::uint8_t> interest(added.size() + 1, 1);
    interest[0] = 0;
    for (std::size_t i = 0; i < added.size(); ++i) {
        group[i] = static_cast<int>(i) + 1;
    }
    FacePairQuery query;
    query.faceGroup = group;
    query.groupOfInterest = interest;
    const std::optional<std::vector<FacePairHit>> pairs = findFacePairs(mesh, tolerance, query, {}, nullptr, 1);
    if (!pairs) {
        return out;
    }
    for (const FacePairHit& p : *pairs) {
        if (!p.hit) {
            continue;
        }
        // a < b and the new faces come first, so b is the other face unless both are new
        if (p.b < added.size()) {
            ++out.own;
            continue;
        }
        const int shell = shellOf(local[p.b]);
        if (std::find(ownShells.begin(), ownShells.end(), shell) != ownShells.end()) {
            ++out.own;
        } else {
            ++out.other;
            ++out.perOtherShell[shell];
        }
    }
    return out;
}

/// The fills a hole tries, in order: `first`, then the rest of Smooth, Refine and Flat.
std::vector<HoleFillMethod> fillChain(HoleFillMethod first, HoleCrossingPolicy policy) {
    std::vector<HoleFillMethod> chain;
    if (first != HoleFillMethod::None) {
        chain.push_back(first);
    }
    if (policy != HoleCrossingPolicy::IgnoreCrossings) {
        for (const HoleFillMethod m : {HoleFillMethod::Smooth, HoleFillMethod::Refine, HoleFillMethod::Flat}) {
            if (m != first) {
                chain.push_back(m);
            }
        }
    }
    return chain;
}

/// Displaces each vertex along its area-weighted normal (keeps the preview off the base mesh).
void offsetAlongNormals(Mesh& mesh, double offset) {
    Eigen::MatrixXd normals = Eigen::MatrixXd::Zero(mesh.numVertices(), 3);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0)).transpose();
        const Eigen::Vector3d n = (Eigen::Vector3d(mesh.V.row(mesh.F(f, 1)).transpose()) - a)
                                      .cross(Eigen::Vector3d(mesh.V.row(mesh.F(f, 2)).transpose()) - a);
        for (int c = 0; c < 3; ++c) {
            normals.row(mesh.F(f, c)) += n.transpose();
        }
    }
    for (Eigen::Index v = 0; v < mesh.numVertices(); ++v) {
        const double len = normals.row(v).norm();
        if (len > 0.0) {
            mesh.V.row(v) += offset * normals.row(v) / len;
        }
    }
    mesh.recomputeNormals();
}

} // namespace

const char* holeFillMethodLabel(HoleFillMethod method) noexcept {
    switch (method) {
    case HoleFillMethod::Flat:
        return "Fill Flat";
    case HoleFillMethod::Refine:
        return "Fill & Refine";
    case HoleFillMethod::Smooth:
        return "Fill Smooth";
    case HoleFillMethod::CloseCrack:
        return "Close Crack";
    case HoleFillMethod::None:
        break;
    }
    return "None";
}

HoleFillMethod recommendedHoleFill(HoleShape shape) noexcept {
    switch (shape) {
    case HoleShape::Flat:
        return HoleFillMethod::Flat;
    case HoleShape::Curved:
        return HoleFillMethod::Smooth;
    case HoleShape::Crack:
        return HoleFillMethod::CloseCrack;
    case HoleShape::OpenChain:
        break;
    }
    return HoleFillMethod::None;
}

bool HoleFillSettings::operator==(const HoleFillSettings& other) const noexcept {
    return patchRings == other.patchRings && maxPatchFaces == other.maxPatchFaces &&
           crackSnapFactor == other.crackSnapFactor && maxHoleEdges == other.maxHoleEdges;
}

HoleFillResult fillHole(
    const Mesh& mesh,
    const HoleLoop& hole,
    HoleFillMethod method,
    const CheckParams& params,
    const HoleFillSettings& settings,
    bool checkCrossings) {
    HoleFillResult result;
    result.mesh = mesh;
    const auto start = std::chrono::steady_clock::now();
    Workspace ws(mesh);
    FillReport report = applyFill(ws, hole, method, settings, {});
    result.statusMessage = std::move(report.status);
    if (!report.changed) {
        return result;
    }
    std::vector<Eigen::Index> faceOf;
    result.mesh = ws.build(&faceOf);
    for (const Eigen::Index f : report.added) {
        result.addedFaces.push_back(faceOf[static_cast<std::size_t>(f)]);
    }
    result.removedFaces = std::move(report.removed);
    std::sort(result.removedFaces.begin(), result.removedFaces.end());
    result.verticesAdded = report.verticesAdded;
    result.openEdgesLeft = report.openEdgesLeft;
    result.maxVertexMove = report.maxVertexMove;
    result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    result.patchArea = facesArea(result.mesh, result.addedFaces);
    result.minNewAngleDeg = minAngleDeg(result.mesh, result.addedFaces);
    if (checkCrossings) {
        result.newCrossings = countNewCrossings(result.mesh, result.addedFaces, params);
    }
    result.success = true;
    return result;
}

HoleFillPreview previewHoleFill(
    const Mesh& mesh,
    const HoleLoop& hole,
    HoleFillMethod method,
    const CheckParams& params,
    const HoleFillSettings& settings) {
    HoleFillPreview preview;
    preview.method = method;
    HoleFillResult r = fillHole(mesh, hole, method, params, settings);
    preview.statusMessage = std::move(r.statusMessage);
    if (!r.success) {
        return preview;
    }
    preview.facesAdded = r.addedFaces.size();
    preview.facesRemoved = r.removedFaces.size();
    preview.verticesAdded = r.verticesAdded;
    preview.openEdgesLeft = r.openEdgesLeft;
    preview.patchArea = r.patchArea;
    preview.maxVertexMove = r.maxVertexMove;
    preview.minNewAngleDeg = r.minNewAngleDeg;
    preview.newCrossings = r.newCrossings;
    preview.elapsedMs = r.elapsedMs;
    preview.sourceFaceIndices = std::move(r.removedFaces);
    if (!r.addedFaces.empty()) {
        preview.previewPatchMesh = extractFaces(r.mesh, r.addedFaces);
        offsetAlongNormals(preview.previewPatchMesh, settings.previewOffsetFactor * mesh.boundingBox().diagonal());
    }
    preview.valid = true;
    return preview;
}

const char* holeSkipReasonLabel(HoleSkipReason reason) noexcept {
    switch (reason) {
    case HoleSkipReason::OpenChain:
        return "not a closed loop";
    case HoleSkipReason::TooBig:
        return "too big";
    case HoleSkipReason::NoMethod:
        return "no fill applies";
    case HoleSkipReason::OutOfScope:
        return "outside the scope";
    case HoleSkipReason::None:
        break;
    }
    return "";
}

HoleBatchResult fillHoles(
    const Mesh& mesh,
    std::span<const HoleLoop> holes,
    HoleFillMethod method,
    const HoleFillSettings& settings,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    return fillHoles(mesh, holes, method, settings, HoleBatchOptions{}, std::move(stopToken), progress);
}

HoleBatchResult fillHoles(
    const Mesh& mesh,
    std::span<const HoleLoop> holes,
    HoleFillMethod method,
    const HoleFillSettings& settings,
    const HoleBatchOptions& options,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    HoleBatchResult result;
    result.mesh = mesh;
    result.holesBefore = holes.size();
    result.faceMap.resize(static_cast<std::size_t>(mesh.numFaces()));
    std::iota(result.faceMap.begin(), result.faceMap.end(), Eigen::Index{0});
    const bool scoped = !options.faceInScope.empty();
    if (scoped && options.faceInScope.size() != static_cast<std::size_t>(mesh.numFaces())) {
        result.statusMessage = "The scope does not match the mesh.";
        return result;
    }
    const auto inScope = [&](Eigen::Index f) {
        return !scoped || f >= mesh.numFaces() || options.faceInScope[static_cast<std::size_t>(f)] != 0;
    };
    const bool testCrossings = options.crossingPolicy != HoleCrossingPolicy::IgnoreCrossings;
    result.crossingsTested = testCrossings;

    Workspace ws(mesh);

    // Shell of every input face (Shells-panel order) and of every face a kept fill adds
    std::vector<int> inputShell;
    std::vector<int> newShell;
    std::optional<FaceGrid> grid;
    double tolerance = 0.0;
    if (testCrossings) {
        inputShell.assign(static_cast<std::size_t>(mesh.numFaces()), -1);
        const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
        for (std::size_t s = 0; s < shells.size(); ++s) {
            for (const Eigen::Index f : shells[s]) {
                inputShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
            }
        }
        grid.emplace(mesh);
        tolerance = options.checkParams.intersectionTolerance * mesh.boundingBox().diagonal();
    }
    const auto shellOf = [&](Eigen::Index f) {
        if (f < mesh.numFaces()) {
            return inputShell[static_cast<std::size_t>(f)];
        }
        const auto i = static_cast<std::size_t>(f - mesh.numFaces());
        return i < newShell.size() ? newShell[i] : -1;
    };

    // A fill may not change a face outside the scope: a removed face or a moved vertex of one
    std::vector<Eigen::Index> removedInput;
    std::vector<int> movedInput;
    std::vector<Eigen::Index> around;
    const auto changedOutsideScope = [&](const Workspace::Mark& m) {
        if (!scoped) {
            return false;
        }
        removedInput.clear();
        movedInput.clear();
        ws.changesSince(m, removedInput, movedInput);
        if (std::any_of(removedInput.begin(), removedInput.end(), [&](Eigen::Index f) { return !inScope(f); })) {
            return true;
        }
        for (const int v : movedInput) {
            ws.facesAround(v, around);
            if (std::any_of(around.begin(), around.end(), [&](Eigen::Index f) { return !inScope(f); })) {
                return true;
            }
        }
        return false;
    };

    std::vector<Eigen::Index> added;
    std::string lastFailure;
    bool changed = false;
    std::set<int> crossedShells;
    result.outcomes.reserve(holes.size());
    for (std::size_t i = 0; i < holes.size(); ++i) {
        if (stopToken.stop_requested()) {
            break;
        }
        const HoleLoop& hole = holes[i];
        HoleFillOutcome& outcome = result.outcomes.emplace_back();
        outcome.hole = i;
        const HoleFillMethod first = method == HoleFillMethod::None ? recommendedHoleFill(hole.shape) : method;
        const bool tooBig = settings.maxHoleEdges > 0 && hole.edgeCount() > static_cast<std::size_t>(settings.maxHoleEdges);
        outcome.skipped = !hole.closed                     ? HoleSkipReason::OpenChain
                          : first == HoleFillMethod::None  ? HoleSkipReason::NoMethod
                          : tooBig                         ? HoleSkipReason::TooBig
                          : std::none_of(hole.faces.begin(), hole.faces.end(), inScope) ? HoleSkipReason::OutOfScope
                                                                                         : HoleSkipReason::None;
        if (outcome.skipped != HoleSkipReason::None) {
            outcome.status = std::string("Skipped: ") + holeSkipReasonLabel(outcome.skipped) + ".";
            ++result.holesSkipped;
            continue;
        }

        // The rim's shells: the most common one names the hole; any of them counts as its own shell
        std::vector<int> ownShells;
        if (testCrossings) {
            std::map<int, std::size_t> votes;
            for (const Eigen::Index f : hole.faces) {
                ++votes[shellOf(f)];
            }
            outcome.shell = std::max_element(votes.begin(), votes.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->first;
            for (const auto& [s, n] : votes) {
                ownShells.push_back(s);
            }
        }

        // Try the chain; keep the first fill that closes the hole and crosses nothing, else the best
        const std::vector<HoleFillMethod> chain = fillChain(first, options.crossingPolicy);
        std::optional<std::size_t> kept;
        std::optional<std::size_t> best;
        FillCrossings keptCrossings;
        FillReport keptReport;
        bool cancelled = false;
        bool anyCrossing = false;
        const auto rankOf = [](const HoleFillAttempt& a) {
            return std::pair<int, std::size_t>(a.closed ? 0 : 1, a.crossingsOwnShell + a.crossingsOtherShells);
        };
        for (std::size_t k = 0; k < chain.size(); ++k) {
            HoleFillAttempt& attempt = outcome.attempts.emplace_back();
            attempt.method = chain[k];
            const Workspace::Mark m = ws.mark();
            FillReport report = applyFill(ws, hole, chain[k], settings, stopToken);
            if (stopToken.stop_requested()) {
                ws.rollback(m);
                cancelled = true;
                break;
            }
            attempt.status = report.status;
            if (!report.changed) {
                continue;
            }
            if (changedOutsideScope(m)) {
                ws.rollback(m);
                attempt.outsideScope = true;
                attempt.status = "It would change faces outside the scope.";
                continue;
            }
            attempt.changed = true;
            attempt.closed = report.openEdgesLeft == 0;
            FillCrossings crossings;
            if (testCrossings) {
                crossings = countFillCrossings(ws, *grid, report.added, shellOf, ownShells, tolerance);
                attempt.crossingsOwnShell = crossings.own;
                attempt.crossingsOtherShells = crossings.other;
            }
            if (!testCrossings || (attempt.closed && crossings.own + crossings.other == 0)) {
                kept = k;
                keptCrossings = std::move(crossings);
                keptReport = std::move(report);
                break;
            }
            const bool crosses = crossings.own + crossings.other > 0;
            anyCrossing = anyCrossing || crosses;
            const bool candidate = options.crossingPolicy == HoleCrossingPolicy::AlwaysFillFewestCrossings || !crosses;
            if (candidate && (!best || rankOf(attempt) < rankOf(outcome.attempts[*best]))) {
                best = k;
            }
            ws.rollback(m);
        }
        if (cancelled) {
            break;
        }
        if (!kept && best) {
            // Fills are deterministic: re-applying gives the same patch that was measured
            keptReport = applyFill(ws, hole, outcome.attempts[*best].method, settings, stopToken);
            if (stopToken.stop_requested()) {
                break;
            }
            if (keptReport.changed) {
                kept = *best;
                keptCrossings = countFillCrossings(ws, *grid, keptReport.added, shellOf, ownShells, tolerance);
            }
        }

        if (!kept) {
            ++result.holesFailed;
            if (anyCrossing) {
                outcome.status = "Every fill crosses other faces: left open.";
            } else {
                outcome.status = outcome.attempts.empty() ? "No fill found." : outcome.attempts.back().status;
            }
            lastFailure = outcome.status;
        } else {
            changed = true;
            const HoleFillAttempt& a = outcome.attempts[*kept];
            outcome.method = a.method;
            outcome.closed = keptReport.openEdgesLeft == 0;
            outcome.facesAdded = keptReport.added.size();
            outcome.status = keptReport.status;
            outcome.crossingsOwnShell = keptCrossings.own;
            outcome.crossingsOtherShells = keptCrossings.other;
            for (const auto& [s, n] : keptCrossings.perOtherShell) {
                outcome.otherShells.push_back(s);
                outcome.otherShellCrossings.push_back(n);
                if (s >= 0) {
                    crossedShells.insert(s);
                }
            }
            if (keptCrossings.own + keptCrossings.other > 0) {
                ++result.holesWithCrossings;
                result.crossingsOwnShell += keptCrossings.own;
                result.crossingsOtherShells += keptCrossings.other;
            }
            added.insert(added.end(), keptReport.added.begin(), keptReport.added.end());
            newShell.resize(static_cast<std::size_t>(ws.totalFaces() - mesh.numFaces()), outcome.shell);
            if (outcome.closed) {
                ++result.holesClosed;
            } else {
                ++result.holesFailed;
                lastFailure = keptReport.status;
            }
        }
        if (progress != nullptr) {
            progress->store(static_cast<float>(i + 1) / static_cast<float>(holes.size()));
        }
    }
    if (stopToken.stop_requested()) {
        HoleBatchResult cancelled;
        cancelled.mesh = mesh;
        cancelled.holesBefore = holes.size();
        cancelled.faceMap = std::move(result.faceMap);
        cancelled.statusMessage = "Cancelled.";
        return cancelled;
    }
    std::vector<Eigen::Index> faceOf;
    result.mesh = ws.build(&faceOf);
    std::copy(faceOf.begin(), faceOf.begin() + mesh.numFaces(), result.faceMap.begin());
    for (const Eigen::Index f : added) {
        if (faceOf[static_cast<std::size_t>(f)] >= 0) {
            result.addedFaces.push_back(faceOf[static_cast<std::size_t>(f)]);
        }
    }
    result.crossedShells.assign(crossedShells.begin(), crossedShells.end());
    result.facesAdded = result.addedFaces.size();
    result.success = changed;
    result.statusMessage = "Closed " + std::to_string(result.holesClosed) + " of " + std::to_string(result.holesBefore) +
                           (result.holesBefore == 1 ? " hole" : " holes") + " (" + std::to_string(result.facesAdded) + " new faces)";
    if (result.holesSkipped > 0) {
        result.statusMessage += ", skipped " + std::to_string(result.holesSkipped) +
                                (scoped ? " (too big, not a loop or outside the scope)" : " (too big or not a loop)");
    }
    if (result.holesFailed > 0) {
        result.statusMessage += ", " + std::to_string(result.holesFailed) + " not closed: " + lastFailure;
    }
    result.statusMessage += ".";
    if (result.holesWithCrossings > 0) {
        result.statusMessage += " " + std::to_string(result.holesWithCrossings) +
                                (result.holesWithCrossings == 1 ? " fill crosses" : " fills cross") + " other faces (" +
                                std::to_string(result.crossingsOwnShell) + " in their own shell, " +
                                std::to_string(result.crossingsOtherShells) + " with other shells).";
    }
    return result;
}

} // namespace ogun
