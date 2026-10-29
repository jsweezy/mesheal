#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/TriTriIntersect.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <thread>
#include <vector>

namespace ogun {

namespace {

constexpr std::uint32_t kLeafSize = 4;
constexpr std::size_t kTasksPerThread = 8;
constexpr std::uint32_t kStopPollMask = 4095;

struct Node {
    Eigen::AlignedBox3d box;
    std::uint32_t first = 0; ///< Leaf: first slot in `order`
    std::uint32_t count = 0; ///< Leaf: number of triangles (0 for inner nodes)
    std::uint32_t left = 0;
    std::uint32_t right = 0;

    [[nodiscard]] bool isLeaf() const noexcept {
        return count > 0;
    }
};

using FacePair = FacePairHit;

[[nodiscard]] bool pairLess(const FacePair& x, const FacePair& y) noexcept {
    return x.a != y.a ? x.a < y.a : x.b < y.b;
}

/// A unit of traversal work: `self(a)` when a == b, otherwise `cross(a, b)`.
struct Task {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
};

/// Binary AABB tree over the usable faces, split at the median centroid of the longest axis
/// (like Blender's BLI_kdopbvh). Leaf boxes are grown by the tolerance.
class TriangleTree {
public:
    TriangleTree(const Mesh& mesh, double tol, std::span<const int> faceGroup) {
        const Eigen::Index nv = mesh.numVertices();
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            const int group = faceGroup.empty() ? 0 : faceGroup[static_cast<std::size_t>(f)];
            if (group < 0) {
                continue;
            }
            std::array<int, 3> idx{mesh.F(f, 0), mesh.F(f, 1), mesh.F(f, 2)};
            if (std::any_of(idx.begin(), idx.end(), [nv](int v) { return v < 0 || v >= nv; }) ||
                faceHasRepeatedVertex(mesh, f)) {
                continue;
            }
            Triangle3d tri{mesh.V.row(idx[0]).transpose(), mesh.V.row(idx[1]).transpose(),
                           mesh.V.row(idx[2]).transpose()};
            if (!((tri[1] - tri[0]).cross(tri[2] - tri[0]).squaredNorm() > 0.0)) {
                continue;
            }
            Eigen::AlignedBox3d box(tri[0]);
            box.extend(tri[1]).extend(tri[2]);
            box.min().array() -= tol;
            box.max().array() += tol;
            faces_.push_back(static_cast<std::uint32_t>(f));
            groups_.push_back(group);
            indices_.push_back(idx);
            triangles_.push_back(tri);
            boxes_.push_back(box);
        }
        order_.resize(faces_.size());
        for (std::uint32_t i = 0; i < order_.size(); ++i) {
            order_[i] = i;
        }
        if (!faces_.empty()) {
            nodes_.reserve(2 * faces_.size() / kLeafSize + 1);
            build(0, static_cast<std::uint32_t>(order_.size()));
        }
    }

    [[nodiscard]] bool empty() const noexcept {
        return nodes_.empty();
    }

    [[nodiscard]] std::size_t numTriangles() const noexcept {
        return faces_.size();
    }

    [[nodiscard]] const Node& node(std::uint32_t n) const noexcept {
        return nodes_[n];
    }

    [[nodiscard]] std::uint32_t face(std::uint32_t slot) const noexcept {
        return faces_[order_[slot]];
    }

    [[nodiscard]] const Triangle3d& triangle(std::uint32_t slot) const noexcept {
        return triangles_[order_[slot]];
    }

    [[nodiscard]] int group(std::uint32_t slot) const noexcept {
        return groups_[order_[slot]];
    }

    [[nodiscard]] const std::array<int, 3>& indices(std::uint32_t slot) const noexcept {
        return indices_[order_[slot]];
    }

    [[nodiscard]] const Eigen::AlignedBox3d& box(std::uint32_t slot) const noexcept {
        return boxes_[order_[slot]];
    }

private:
    std::uint32_t build(std::uint32_t begin, std::uint32_t end) {
        const auto index = static_cast<std::uint32_t>(nodes_.size());
        nodes_.emplace_back();
        Eigen::AlignedBox3d box;
        Eigen::AlignedBox3d centroids;
        for (std::uint32_t i = begin; i < end; ++i) {
            box.extend(boxes_[order_[i]]);
            centroids.extend(boxes_[order_[i]].center());
        }
        nodes_[index].box = box;
        if (end - begin <= kLeafSize) {
            nodes_[index].first = begin;
            nodes_[index].count = end - begin;
            return index;
        }
        Eigen::Index axis = 0;
        centroids.sizes().maxCoeff(&axis);
        const std::uint32_t mid = begin + (end - begin) / 2;
        std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                         [this, axis](std::uint32_t x, std::uint32_t y) {
                             return boxes_[x].center()[axis] < boxes_[y].center()[axis];
                         });
        const std::uint32_t left = build(begin, mid);
        const std::uint32_t right = build(mid, end);
        nodes_[index].left = left;
        nodes_[index].right = right;
        return index;
    }

    std::vector<std::uint32_t> faces_;
    std::vector<int> groups_;
    std::vector<std::array<int, 3>> indices_;
    std::vector<Triangle3d> triangles_;
    std::vector<Eigen::AlignedBox3d> boxes_;
    std::vector<std::uint32_t> order_;
    std::vector<Node> nodes_;
};

/// Splits `task` one level into `out`; returns false if it can't be split (kept as is).
bool splitTask(const TriangleTree& tree, const Task& task, std::vector<Task>& out) {
    const Node& a = tree.node(task.a);
    if (task.a == task.b) {
        if (a.isLeaf()) {
            out.push_back(task);
            return false;
        }
        out.push_back({a.left, a.left});
        out.push_back({a.right, a.right});
        out.push_back({a.left, a.right});
        return true;
    }
    const Node& b = tree.node(task.b);
    if (!a.box.intersects(b.box)) {
        return true;
    }
    if (a.isLeaf() && b.isLeaf()) {
        out.push_back(task);
        return false;
    }
    if (b.isLeaf() || (!a.isLeaf() && a.box.volume() >= b.box.volume())) {
        out.push_back({a.left, task.b});
        out.push_back({a.right, task.b});
    } else {
        out.push_back({task.a, b.left});
        out.push_back({task.a, b.right});
    }
    return true;
}

/// Runs traversal tasks on one thread and collects the intersecting face pairs.
class Traverser {
public:
    Traverser(const TriangleTree& tree, double tol, const FacePairQuery& query, const std::stop_token& stopToken)
        : tree_(tree), tol_(tol), query_(query), stopToken_(stopToken) {}

    void run(const Task& task) {
        if (task.a == task.b) {
            self(task.a);
        } else {
            cross(task.a, task.b);
        }
    }

    [[nodiscard]] bool stopped() const noexcept {
        return stopped_;
    }

    [[nodiscard]] std::vector<FacePair>& pairs() noexcept {
        return pairs_;
    }

private:
    void self(std::uint32_t n) {
        const Node& node = tree_.node(n);
        if (node.isLeaf()) {
            for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
                for (std::uint32_t j = i + 1; j < node.first + node.count; ++j) {
                    testPair(i, j);
                }
            }
            return;
        }
        self(node.left);
        self(node.right);
        cross(node.left, node.right);
    }

    void cross(std::uint32_t na, std::uint32_t nb) {
        const Node& a = tree_.node(na);
        const Node& b = tree_.node(nb);
        if (stopped_ || !a.box.intersects(b.box)) {
            return;
        }
        if (a.isLeaf() && b.isLeaf()) {
            for (std::uint32_t i = a.first; i < a.first + a.count; ++i) {
                for (std::uint32_t j = b.first; j < b.first + b.count; ++j) {
                    testPair(i, j);
                }
            }
        } else if (b.isLeaf() || (!a.isLeaf() && a.box.volume() >= b.box.volume())) {
            cross(a.left, nb);
            cross(a.right, nb);
        } else {
            cross(na, b.left);
            cross(na, b.right);
        }
    }

    void testPair(std::uint32_t i, std::uint32_t j) {
        if ((++tests_ & kStopPollMask) == 0 && stopToken_.stop_requested()) {
            stopped_ = true;
        }
        if (stopped_ || !tree_.box(i).intersects(tree_.box(j))) {
            return;
        }
        if (!query_.faceGroup.empty()) {
            const int gi = tree_.group(i);
            const int gj = tree_.group(j);
            if (gi == gj || (!query_.groupOfInterest.empty() && query_.groupOfInterest[static_cast<std::size_t>(gi)] == 0 &&
                             query_.groupOfInterest[static_cast<std::size_t>(gj)] == 0)) {
                return;
            }
        }
        const std::array<int, 3>& vi = tree_.indices(i);
        const std::array<int, 3>& vj = tree_.indices(j);
        int shared = 0;
        for (const int v : vi) {
            shared += static_cast<int>(std::count(vj.begin(), vj.end(), v));
        }
        const TriTriHit hit = intersectTriangles(tree_.triangle(i), tree_.triangle(j), shared, tol_);
        const bool contact = !hit.hit && query_.contacts && trianglesWithin(tree_.triangle(i), tree_.triangle(j), tol_);
        if (hit.hit || contact) {
            const std::uint32_t fi = tree_.face(i);
            const std::uint32_t fj = tree_.face(j);
            pairs_.push_back({std::min(fi, fj), std::max(fi, fj), hit.hit, hit.coplanar && hit.hit, hit.touching || contact});
        }
    }

    const TriangleTree& tree_;
    double tol_;
    FacePairQuery query_;
    std::stop_token stopToken_;
    std::vector<FacePair> pairs_;
    std::uint32_t tests_ = 0;
    bool stopped_ = false;
};

/// One element per face in any pair (most crossings first); `related` holds the faces it crosses.
[[nodiscard]] CategoryResult buildResult(std::vector<FacePair>& pairs) {
    std::vector<FacePair> directed;
    directed.reserve(2 * pairs.size());
    for (const FacePair& p : pairs) {
        directed.push_back(p);
        directed.push_back({p.b, p.a, p.hit, p.coplanar, p.touching});
    }
    std::sort(directed.begin(), directed.end(), pairLess);
    directed.erase(std::unique(directed.begin(), directed.end(),
                               [](const FacePair& x, const FacePair& y) { return x.a == y.a && x.b == y.b; }),
                   directed.end());

    struct Group {
        std::size_t begin;
        std::size_t end;
        bool coplanar;
    };
    std::vector<Group> groups;
    for (std::size_t i = 0; i < directed.size();) {
        Group group{i, i, false};
        while (group.end < directed.size() && directed[group.end].a == directed[i].a) {
            group.coplanar = group.coplanar || directed[group.end].coplanar;
            ++group.end;
        }
        groups.push_back(group);
        i = group.end;
    }
    std::stable_sort(groups.begin(), groups.end(),
                     [](const Group& x, const Group& y) { return x.end - x.begin > y.end - y.begin; });

    CategoryResult result;
    result.category = DefectCategory::Intersections;
    result.elements.reserve(groups.size());
    result.faces.reserve(groups.size());
    result.faceOffsets.reserve(groups.size() + 1);
    result.related.reserve(directed.size());
    result.relatedOffsets.reserve(groups.size() + 1);
    result.relatedOffsets.push_back(0);
    for (const Group& group : groups) {
        const auto crosses = static_cast<double>(group.end - group.begin);
        DefectElement element;
        element.index = directed[group.begin].a;
        element.metric = crosses;
        element.columns = {crosses, static_cast<double>(directed[group.begin].b), 0.0};
        element.subtype = group.coplanar ? 1 : 0;
        result.elements.push_back(element);
        result.faces.push_back(element.index);
        result.faceOffsets.push_back(static_cast<std::uint32_t>(result.faces.size()));
        for (std::size_t k = group.begin; k < group.end; ++k) {
            result.related.push_back(directed[k].b);
        }
        result.relatedOffsets.push_back(static_cast<std::uint32_t>(result.related.size()));
    }
    return result;
}

[[nodiscard]] CategoryResult cancelled() {
    CategoryResult result;
    result.category = DefectCategory::Intersections;
    result.cancelled = true;
    return result;
}

} // namespace

std::optional<std::vector<FacePairHit>> findFacePairs(
    const Mesh& mesh,
    double tol,
    const FacePairQuery& query,
    std::stop_token stopToken,
    std::atomic<float>* progress,
    unsigned maxThreads) {
    if (stopToken.stop_requested()) {
        return std::nullopt;
    }
    std::vector<FacePairHit> pairs;
    if (mesh.empty()) {
        return pairs;
    }
    const TriangleTree tree(mesh, tol, query.faceGroup);
    if (tree.empty()) {
        return pairs;
    }

    unsigned threads = std::max(1u, std::min(maxThreads, std::thread::hardware_concurrency()));
    if (tree.numTriangles() <= settings::kIntersectionSerialFaceCount) {
        threads = 1;
    }

    // Split the root's self-overlap into independent node pairs for the workers to share
    std::vector<Task> tasks{{0, 0}};
    const std::size_t target = kTasksPerThread * threads;
    while (tasks.size() < target) {
        std::vector<Task> next;
        bool split = false;
        for (const Task& task : tasks) {
            split = splitTask(tree, task, next) || split;
        }
        tasks.swap(next);
        if (!split) {
            break;
        }
    }

    std::atomic<std::size_t> nextTask{0};
    std::atomic<std::size_t> doneTasks{0};
    const auto work = [&](Traverser& traverser) {
        for (std::size_t t = nextTask.fetch_add(1); t < tasks.size() && !traverser.stopped();
             t = nextTask.fetch_add(1)) {
            traverser.run(tasks[t]);
            const std::size_t done = doneTasks.fetch_add(1) + 1;
            if (progress != nullptr) {
                progress->store(static_cast<float>(done) / static_cast<float>(tasks.size()));
            }
        }
    };

    std::vector<Traverser> traversers(threads, Traverser(tree, tol, query, stopToken));
    if (threads == 1) {
        work(traversers.front());
    } else {
        std::vector<std::jthread> workers;
        workers.reserve(threads);
        for (unsigned t = 0; t < threads; ++t) {
            workers.emplace_back([&work, &traversers, t] { work(traversers[t]); });
        }
    }

    if (stopToken.stop_requested()) {
        return std::nullopt;
    }
    for (Traverser& traverser : traversers) {
        pairs.insert(pairs.end(), traverser.pairs().begin(), traverser.pairs().end());
    }
    return pairs;
}

CategoryResult checkSelfIntersections(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress,
    unsigned maxThreads) {
    const double tol = mesh.empty() ? 0.0 : params.intersectionTolerance * mesh.boundingBox().diagonal();
    std::optional<std::vector<FacePairHit>> pairs = findFacePairs(mesh, tol, {}, stopToken, progress, maxThreads);
    if (!pairs) {
        return cancelled();
    }
    return buildResult(*pairs);
}

} // namespace ogun
