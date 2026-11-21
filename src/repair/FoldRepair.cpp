#include "repair/FoldRepair.hpp"

#include "core/Settings.hpp"

#include "diagnostics/MeshState.hpp"
#include "diagnostics/SelfIntersections.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace mesheal {

namespace {

using Tri = std::array<int, 3>;

[[nodiscard]] std::uint64_t edgeKey(int u, int v) noexcept {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32U) | static_cast<std::uint32_t>(v);
}

/// One proposed change: faces that get a new triangle, or none (removed).
struct Change {
    std::vector<std::pair<int, std::optional<Tri>>> faces;
    int movedVertex = -1; ///< Collapse: the vertex that goes ...
    int ontoVertex = -1;  ///< ... onto this one
    double move = 0.0;
};

/// The faces as an editable list, with what the checks need kept up to date.
class FoldEditor {
public:
    FoldEditor(const Mesh& mesh, double tol) : V_(mesh.V), tol_(tol) {
        faces_.resize(static_cast<std::size_t>(mesh.numFaces()));
        alive_.assign(faces_.size(), 1);
        touched_.assign(faces_.size(), 0);
        vertexFaces_.resize(static_cast<std::size_t>(mesh.numVertices()));
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            const Tri t{mesh.F(f, 0), mesh.F(f, 1), mesh.F(f, 2)};
            faces_[static_cast<std::size_t>(f)] = t;
            for (int k = 0; k < 3; ++k) {
                vertexFaces_[static_cast<std::size_t>(t[static_cast<std::size_t>(k)])].push_back(static_cast<int>(f));
                // A repeated directed edge already: the input is not manifold there; the last face wins
                directed_[edgeKey(t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)])] = static_cast<int>(f);
            }
        }
        lengthFloor_ = 1.0e-9 * mesh.boundingBox().diagonal();
    }

    [[nodiscard]] bool usable(int f) const { return alive_[static_cast<std::size_t>(f)] != 0 && touched_[static_cast<std::size_t>(f)] == 0; }
    [[nodiscard]] const Tri& face(int f) const { return faces_[static_cast<std::size_t>(f)]; }

    /// The candidate changes for the fold (a, b), cheapest move first.
    [[nodiscard]] std::vector<Change> candidates(int a, int b) const {
        std::vector<Change> out;
        // Rotate a to (p, q, x): edge p -> q is the shared one, so b holds q -> p with apex y
        Tri ta = face(a);
        const Tri& tb = face(b);
        int rot = 0;
        for (; rot < 3; ++rot) {
            const int x = ta[2];
            if (std::find(tb.begin(), tb.end(), x) == tb.end() && std::find(tb.begin(), tb.end(), ta[0]) != tb.end() &&
                std::find(tb.begin(), tb.end(), ta[1]) != tb.end()) {
                break;
            }
            std::rotate(ta.begin(), ta.begin() + 1, ta.end());
        }
        if (rot == 3) {
            return out;
        }
        const int p = ta[0];
        const int q = ta[1];
        const int x = ta[2];
        int y = -1;
        for (const int v : tb) {
            if (v != p && v != q) {
                y = v;
            }
        }
        if (y < 0 || y == x) {
            return out;
        }
        // b must run q -> p (consistently wound with a)
        bool wound = false;
        for (int k = 0; k < 3; ++k) {
            wound = wound || (tb[static_cast<std::size_t>(k)] == q && tb[static_cast<std::size_t>((k + 1) % 3)] == p);
        }
        if (wound) {
            Change flip;
            flip.faces.push_back({a, Tri{x, p, y}});
            flip.faces.push_back({b, Tri{y, q, x}});
            out.push_back(std::move(flip));
        }
        for (const auto& [from, onto] : {std::pair{x, p}, std::pair{x, q}, std::pair{y, p}, std::pair{y, q}}) {
            out.push_back(collapse(from, onto));
        }
        std::stable_sort(out.begin(), out.end(), [](const Change& l, const Change& r) { return l.move < r.move; });
        return out;
    }

    /// The crossing pairs left among the faces `change` touches, if it passes every check and leaves fewer
    /// than there are now. Leaves everything as it was.
    [[nodiscard]] std::optional<std::size_t> evaluate(const Change& change, double maxMove) {
        if (change.move > maxMove) {
            return std::nullopt;
        }
        std::vector<std::pair<int, Tri>> old;
        for (const auto& [f, tri] : change.faces) {
            old.emplace_back(f, face(f));
        }
        const std::vector<int> region = regionAround(change);
        const std::size_t before = crossingPairs(region, old);

        // Topology: take the old faces' directed edges out, put the new ones in, each at most once
        for (const auto& [f, tri] : old) {
            eraseEdges(f, tri);
        }
        std::vector<std::pair<int, Tri>> added;
        bool ok = true;
        for (const auto& [f, tri] : change.faces) {
            if (!tri) {
                continue;
            }
            if (!insertable(*tri) || !wellShaped(*tri)) {
                ok = false;
                break;
            }
            insertEdges(f, *tri);
            added.emplace_back(f, *tri);
        }
        if (ok && change.ontoVertex >= 0) {
            ok = oneFan(change.ontoVertex, change);
        }
        for (const auto& [f, tri] : added) {
            eraseEdges(f, tri);
        }
        for (const auto& [f, tri] : old) {
            insertEdges(f, tri);
        }
        if (!ok) {
            return std::nullopt;
        }
        const std::size_t after = crossingPairs(region, added);
        return after < before ? std::optional<std::size_t>(after) : std::nullopt;
    }

    /// Applies `change` (one `evaluate` accepted).
    void commit(const Change& change) {
        std::vector<std::pair<int, Tri>> old;
        for (const auto& [f, tri] : change.faces) {
            old.emplace_back(f, face(f));
            eraseEdges(f, face(f));
        }
        for (const auto& [f, tri] : change.faces) {
            if (tri) {
                insertEdges(f, *tri);
            }
        }
        for (const auto& [f, tri] : change.faces) {
            touched_[static_cast<std::size_t>(f)] = 1;
            if (tri) {
                faces_[static_cast<std::size_t>(f)] = *tri;
            } else {
                alive_[static_cast<std::size_t>(f)] = 0;
            }
        }
        if (change.ontoVertex >= 0) {
            auto& onto = vertexFaces_[static_cast<std::size_t>(change.ontoVertex)];
            for (const int f : vertexFaces_[static_cast<std::size_t>(change.movedVertex)]) {
                if (alive_[static_cast<std::size_t>(f)] != 0 && std::find(onto.begin(), onto.end(), f) == onto.end()) {
                    onto.push_back(f);
                }
            }
            vertexFaces_[static_cast<std::size_t>(change.movedVertex)].clear();
        } else {
            // A flip: every vertex of the old and new triangles lists exactly the changed faces it is on now
            for (const auto& [f, tri] : old) {
                for (const int v : tri) {
                    std::erase(vertexFaces_[static_cast<std::size_t>(v)], f);
                }
            }
            for (const auto& [f, tri] : change.faces) {
                for (const int v : *tri) {
                    vertexFaces_[static_cast<std::size_t>(v)].push_back(f);
                }
            }
        }
        if (gridBuilt_) {
            for (const auto& [f, tri] : change.faces) {
                if (tri) {
                    fileFace(f); // its new box; stale entries are filtered by the box test
                }
            }
        }
    }

    /// The faces left, with the vertices they use.
    [[nodiscard]] Mesh result(const Mesh& input) const {
        std::vector<int> newIndex(static_cast<std::size_t>(input.numVertices()), -1);
        std::vector<Tri> kept;
        for (std::size_t f = 0; f < faces_.size(); ++f) {
            if (alive_[f] != 0) {
                kept.push_back(faces_[f]);
            }
        }
        int next = 0;
        for (const Tri& t : kept) {
            for (const int v : t) {
                if (newIndex[static_cast<std::size_t>(v)] < 0) {
                    newIndex[static_cast<std::size_t>(v)] = next++;
                }
            }
        }
        Mesh out;
        out.fileUnit = input.fileUnit;
        out.V.resize(next, 3);
        for (std::size_t v = 0; v < newIndex.size(); ++v) {
            if (newIndex[v] >= 0) {
                out.V.row(newIndex[v]) = input.V.row(static_cast<Eigen::Index>(v));
            }
        }
        out.F.resize(static_cast<Eigen::Index>(kept.size()), 3);
        for (std::size_t f = 0; f < kept.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                out.F(static_cast<Eigen::Index>(f), k) = newIndex[static_cast<std::size_t>(kept[f][static_cast<std::size_t>(k)])];
            }
        }
        out.recomputeNormals();
        return out;
    }

private:
    [[nodiscard]] Change collapse(int from, int onto) const {
        Change c;
        c.movedVertex = from;
        c.ontoVertex = onto;
        c.move = (V_.row(from) - V_.row(onto)).norm();
        for (const int f : vertexFaces_[static_cast<std::size_t>(from)]) {
            if (alive_[static_cast<std::size_t>(f)] == 0) {
                continue;
            }
            Tri t = face(f);
            if (std::find(t.begin(), t.end(), onto) != t.end()) {
                c.faces.push_back({f, std::nullopt}); // on the collapsed edge: it goes
                continue;
            }
            std::replace(t.begin(), t.end(), from, onto);
            c.faces.push_back({f, t});
        }
        return c;
    }

    void eraseEdges(int f, const Tri& t) {
        for (int k = 0; k < 3; ++k) {
            const auto it = directed_.find(edgeKey(t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)]));
            if (it != directed_.end() && it->second == f) {
                directed_.erase(it);
            }
        }
    }

    void insertEdges(int f, const Tri& t) {
        for (int k = 0; k < 3; ++k) {
            directed_[edgeKey(t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)])] = f;
        }
    }

    /// No repeated vertex, no directed edge already used (that would make an edge non-manifold or wound
    /// against its neighbour), and not the reverse of a face that is there.
    [[nodiscard]] bool insertable(const Tri& t) const {
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) {
            return false;
        }
        for (int k = 0; k < 3; ++k) {
            if (directed_.count(edgeKey(t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)])) != 0) {
                return false;
            }
        }
        const auto r0 = directed_.find(edgeKey(t[1], t[0]));
        const auto r1 = directed_.find(edgeKey(t[2], t[1]));
        const auto r2 = directed_.find(edgeKey(t[0], t[2]));
        return !(r0 != directed_.end() && r1 != directed_.end() && r2 != directed_.end() && r0->second == r1->second &&
                 r1->second == r2->second);
    }

    [[nodiscard]] bool wellShaped(const Tri& t) const {
        const Eigen::Vector3d a = V_.row(t[0]);
        const Eigen::Vector3d b = V_.row(t[1]);
        const Eigen::Vector3d c = V_.row(t[2]);
        const double longest = std::max({(b - a).norm(), (c - b).norm(), (a - c).norm()});
        return longest > 0.0 && (b - a).cross(c - a).norm() / longest > lengthFloor_;
    }

    /// The faces around `v` (as `change` leaves them) form one fan: one ring, or one strip on a border.
    [[nodiscard]] bool oneFan(int v, const Change& change) const {
        std::unordered_map<int, int> next;
        std::unordered_map<int, int> prevCount;
        const auto add = [&](const Tri& t) {
            for (int k = 0; k < 3; ++k) {
                if (t[static_cast<std::size_t>(k)] == v) {
                    const int u = t[static_cast<std::size_t>((k + 1) % 3)];
                    const int w = t[static_cast<std::size_t>((k + 2) % 3)];
                    if (!next.emplace(u, w).second) {
                        return false;
                    }
                    ++prevCount[w];
                }
            }
            return true;
        };
        std::vector<int> seen;
        for (const auto& [f, tri] : change.faces) {
            seen.push_back(f);
            if (tri && !add(*tri)) {
                return false;
            }
        }
        for (const int f : vertexFaces_[static_cast<std::size_t>(v)]) {
            if (alive_[static_cast<std::size_t>(f)] != 0 && std::find(seen.begin(), seen.end(), f) == seen.end() && !add(face(f))) {
                return false;
            }
        }
        if (next.empty()) {
            return true;
        }
        int start = next.begin()->first;
        int starts = 0;
        for (const auto& [u, w] : next) {
            if (prevCount[u] > 1) {
                return false;
            }
            if (prevCount[u] == 0) {
                start = u;
                ++starts;
            }
        }
        if (starts > 1) {
            return false;
        }
        std::size_t steps = 0;
        int at = start;
        while (steps <= next.size()) {
            const auto it = next.find(at);
            if (it == next.end()) {
                break;
            }
            ++steps;
            at = it->second;
            if (at == start) {
                break;
            }
        }
        return steps == next.size();
    }

    /// Alive faces near everything `change` touches (its faces and the faces around their vertices).
    [[nodiscard]] std::vector<int> regionAround(const Change& change) {
        Eigen::AlignedBox3d box;
        for (const auto& [f, tri] : change.faces) {
            for (const int v : face(f)) {
                box.extend(Eigen::Vector3d(V_.row(v)));
            }
        }
        box.min().array() -= tol_;
        box.max().array() += tol_;
        if (!gridBuilt_) {
            buildGrid();
        }
        ++stamp_;
        std::vector<int> region;
        const auto consider = [&](int f) {
            if (seen_[static_cast<std::size_t>(f)] == stamp_ || alive_[static_cast<std::size_t>(f)] == 0) {
                return;
            }
            seen_[static_cast<std::size_t>(f)] = stamp_;
            if (box.intersects(boxOf(face(f)))) {
                region.push_back(f);
            }
        };
        for (const int f : bigFaces_) {
            consider(f);
        }
        const std::array<std::int64_t, 3> lo = cellOf(box.min());
        const std::array<std::int64_t, 3> hi = cellOf(box.max());
        for (std::int64_t x = lo[0]; x <= hi[0]; ++x) {
            for (std::int64_t y = lo[1]; y <= hi[1]; ++y) {
                for (std::int64_t z = lo[2]; z <= hi[2]; ++z) {
                    if (const auto it = grid_.find(cellKey({x, y, z})); it != grid_.end()) {
                        for (const int f : it->second) {
                            consider(f);
                        }
                    }
                }
            }
        }
        std::sort(region.begin(), region.end());
        return region;
    }

    [[nodiscard]] Eigen::AlignedBox3d boxOf(const Tri& t) const {
        Eigen::AlignedBox3d b;
        for (const int v : t) {
            b.extend(Eigen::Vector3d(V_.row(v)));
        }
        return b;
    }

    [[nodiscard]] std::array<std::int64_t, 3> cellOf(const Eigen::Vector3d& p) const {
        std::array<std::int64_t, 3> c{};
        for (int k = 0; k < 3; ++k) {
            c[static_cast<std::size_t>(k)] = static_cast<std::int64_t>(std::floor((p[k] - gridOrigin_[k]) / cellSize_));
        }
        return c;
    }

    [[nodiscard]] static std::uint64_t cellKey(const std::array<std::int64_t, 3>& c) noexcept {
        const auto part = [](std::int64_t v) { return static_cast<std::uint64_t>(v + (1 << 20)) & 0x1FFFFFU; };
        return (part(c[0]) << 42U) | (part(c[1]) << 21U) | part(c[2]);
    }

    /// Files a face in the grid cells its box covers (a face covering very many cells is kept apart).
    void fileFace(int f) {
        const Eigen::AlignedBox3d b = boxOf(face(f));
        const std::array<std::int64_t, 3> lo = cellOf(b.min());
        const std::array<std::int64_t, 3> hi = cellOf(b.max());
        const std::int64_t cells = (hi[0] - lo[0] + 1) * (hi[1] - lo[1] + 1) * (hi[2] - lo[2] + 1);
        if (cells > 64) {
            bigFaces_.push_back(f);
            return;
        }
        for (std::int64_t x = lo[0]; x <= hi[0]; ++x) {
            for (std::int64_t y = lo[1]; y <= hi[1]; ++y) {
                for (std::int64_t z = lo[2]; z <= hi[2]; ++z) {
                    grid_[cellKey({x, y, z})].push_back(f);
                }
            }
        }
    }

    /// A uniform grid of about one face per cell over the whole mesh, built on the first query.
    void buildGrid() {
        Eigen::AlignedBox3d all;
        for (Eigen::Index v = 0; v < V_.rows(); ++v) {
            all.extend(Eigen::Vector3d(V_.row(v)));
        }
        gridOrigin_ = all.min();
        const double volume = std::max(all.volume(), 1.0e-300);
        const double perCell = volume / static_cast<double>(std::max<std::size_t>(faces_.size(), 1));
        cellSize_ = std::max({std::cbrt(perCell), all.diagonal().norm() / 2.0e6, 1.0e-12});
        // A flat or thin mesh has almost no volume: size the cells by its area instead
        const Eigen::Vector3d ext = all.diagonal();
        const double area = std::max({ext[0] * ext[1], ext[1] * ext[2], ext[0] * ext[2], 1.0e-300});
        cellSize_ = std::max(cellSize_, std::sqrt(area / static_cast<double>(std::max<std::size_t>(faces_.size(), 1))));
        seen_.assign(faces_.size(), 0);
        gridBuilt_ = true;
        for (std::size_t f = 0; f < faces_.size(); ++f) {
            if (alive_[f] != 0) {
                fileFace(static_cast<int>(f));
            }
        }
    }

    /// Crossing pairs that involve `local` (each with its own triangle) among `local` and the rest of
    /// `region`, tested exactly as the intersection check tests them.
    [[nodiscard]] std::size_t crossingPairs(const std::vector<int>& region, const std::vector<std::pair<int, Tri>>& local) const {
        std::vector<Tri> tris;
        std::vector<int> group;
        for (std::size_t i = 0; i < local.size(); ++i) {
            tris.push_back(local[i].second);
            group.push_back(static_cast<int>(i) + 1);
        }
        for (const int f : region) {
            if (std::none_of(local.begin(), local.end(), [f](const auto& l) { return l.first == f; })) {
                tris.push_back(face(f));
                group.push_back(0);
            }
        }
        std::unordered_map<int, int> index;
        Mesh sub;
        std::vector<int> used;
        for (const Tri& t : tris) {
            for (const int v : t) {
                if (index.emplace(v, static_cast<int>(used.size())).second) {
                    used.push_back(v);
                }
            }
        }
        sub.V.resize(static_cast<Eigen::Index>(used.size()), 3);
        for (std::size_t i = 0; i < used.size(); ++i) {
            sub.V.row(static_cast<Eigen::Index>(i)) = V_.row(used[i]);
        }
        sub.F.resize(static_cast<Eigen::Index>(tris.size()), 3);
        for (std::size_t f = 0; f < tris.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                sub.F(static_cast<Eigen::Index>(f), k) = index.at(tris[f][static_cast<std::size_t>(k)]);
            }
        }
        FacePairQuery query;
        query.faceGroup = group;
        const std::optional<std::vector<FacePairHit>> hits = findFacePairs(sub, tol_, query, {}, nullptr, 1);
        return hits ? hits->size() : std::numeric_limits<std::size_t>::max();
    }

    const Eigen::MatrixXd& V_;
    double tol_ = 0.0;
    double lengthFloor_ = 0.0;
    std::vector<Tri> faces_;
    std::vector<char> alive_;
    std::vector<char> touched_; ///< Changed this call: its fold, if any, waits for the next call
    std::vector<std::vector<int>> vertexFaces_;
    std::unordered_map<std::uint64_t, int> directed_;
    std::unordered_map<std::uint64_t, std::vector<int>> grid_;
    std::vector<int> bigFaces_;
    Eigen::Vector3d gridOrigin_ = Eigen::Vector3d::Zero();
    double cellSize_ = 1.0;
    std::vector<std::uint32_t> seen_;
    std::uint32_t stamp_ = 0;
    bool gridBuilt_ = false;
};

} // namespace

FoldRepairResult unfoldFaces(Mesh& mesh, const CheckParams& params, double maxMove, std::span<const std::uint8_t> target,
                             std::stop_token stopToken) {
    FoldRepairResult result;
    if (mesh.empty()) {
        return result;
    }
    const std::optional<std::vector<CrossingPair>> pairs = classifyCrossings(mesh, params, stopToken);
    if (!pairs) {
        result.cancelled = true;
        return result;
    }
    const auto isTarget = [&](std::uint32_t f) { return target.empty() || target[f] != 0; };
    std::vector<std::pair<int, int>> folds;
    for (const CrossingPair& pair : *pairs) {
        if (pair.kind == CrossingKind::Folded && isTarget(pair.a) && isTarget(pair.b)) {
            folds.emplace_back(static_cast<int>(pair.a), static_cast<int>(pair.b));
        }
    }
    result.folds = folds.size();
    if (folds.empty()) {
        return result;
    }
    if (folds.size() > settings::kMaxFoldRepairFolds) {
        result.tooMany = true; // a surface folded this often is not a few stray folds: left to the other repairs
        return result;
    }
    const double tol = params.intersectionTolerance * mesh.boundingBox().diagonal();
    FoldEditor editor(mesh, tol);
    for (const auto& [a, b] : folds) {
        if (stopToken.stop_requested()) {
            result.cancelled = true;
            return result;
        }
        if (!editor.usable(a) || !editor.usable(b)) {
            continue;
        }
        // The change that leaves the fewest crossings near it; the smallest move among equals
        const std::vector<Change> changes = editor.candidates(a, b);
        const Change* best = nullptr;
        std::size_t bestLeft = 0;
        for (const Change& change : changes) {
            const std::optional<std::size_t> left = editor.evaluate(change, maxMove);
            if (left && (best == nullptr || *left < bestLeft)) {
                best = &change;
                bestLeft = *left;
            }
        }
        if (best != nullptr) {
            editor.commit(*best);
            ++result.unfolded;
            (best->ontoVertex >= 0 ? result.collapses : result.flips) += 1;
            result.maxMove = std::max(result.maxMove, best->move);
        }
    }
    if (result.unfolded > 0) {
        mesh = editor.result(mesh);
    }
    return result;
}

} // namespace mesheal
