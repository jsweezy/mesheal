#include "diagnostics/MeshChecks.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace ogun {

namespace {

constexpr Eigen::Index kStopPollMask = 4095;

[[nodiscard]] bool stopRequested(const std::stop_token& stopToken, Eigen::Index iteration) noexcept {
    return (iteration & kStopPollMask) == 0 && stopToken.stop_requested();
}

[[nodiscard]] CategoryResult cancelledResult(DefectCategory category) {
    CategoryResult result;
    result.category = category;
    result.cancelled = true;
    return result;
}

[[nodiscard]] std::size_t toSize(Eigen::Index value) noexcept {
    return static_cast<std::size_t>(value);
}

[[nodiscard]] bool faceIsUsable(const Mesh& mesh, Eigen::Index f) noexcept {
    const Eigen::Index nv = mesh.numVertices();
    for (int c = 0; c < 3; ++c) {
        const int v = mesh.F(f, c);
        if (v < 0 || v >= nv) {
            return false;
        }
    }
    return !faceHasRepeatedVertex(mesh, f);
}

[[nodiscard]] double edgeLength(const Mesh& mesh, const std::array<int, 2>& edge) {
    return (mesh.V.row(edge[0]) - mesh.V.row(edge[1])).norm();
}

[[nodiscard]] double faceArea(const Mesh& mesh, Eigen::Index f) {
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(f, 0)).transpose();
    const Eigen::Vector3d v1 = mesh.V.row(mesh.F(f, 1)).transpose();
    const Eigen::Vector3d v2 = mesh.V.row(mesh.F(f, 2)).transpose();
    return 0.5 * (v1 - v0).cross(v2 - v0).norm();
}

class UnionFind {
public:
    explicit UnionFind(std::size_t n) : parent_(n) {
        std::iota(parent_.begin(), parent_.end(), std::uint32_t{0});
    }

    [[nodiscard]] std::uint32_t find(std::uint32_t x) noexcept {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }

    void unite(std::uint32_t a, std::uint32_t b) noexcept {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);
        }
    }

private:
    std::vector<std::uint32_t> parent_;
};

enum class SortOrder { ByIndex, LowMetricFirst, HighMetricFirst };

/// Collects elements and their faces, then sorts them worst first into a `CategoryResult`.
class ResultBuilder {
public:
    explicit ResultBuilder(DefectCategory category) {
        result_.category = category;
    }

    void add(const DefectElement& element, std::span<const Eigen::Index> faces) {
        elements_.push_back(element);
        offsets_.push_back(static_cast<std::uint32_t>(faces_.size()));
        faces_.insert(faces_.end(), faces.begin(), faces.end());
    }

    void add(const DefectElement& element, Eigen::Index face) {
        add(element, std::span<const Eigen::Index>(&face, 1));
    }

    [[nodiscard]] CategoryResult finish(SortOrder order, std::size_t infoCount = 0) {
        offsets_.push_back(static_cast<std::uint32_t>(faces_.size()));
        std::vector<std::size_t> perm(elements_.size());
        std::iota(perm.begin(), perm.end(), std::size_t{0});
        std::stable_sort(perm.begin(), perm.end(), [&](std::size_t a, std::size_t b) {
            const DefectElement& ea = elements_[a];
            const DefectElement& eb = elements_[b];
            if (order != SortOrder::ByIndex && ea.metric != eb.metric) {
                return order == SortOrder::LowMetricFirst ? ea.metric < eb.metric : ea.metric > eb.metric;
            }
            if (order != SortOrder::ByIndex && ea.columns[0] != eb.columns[0]) {
                return ea.columns[0] < eb.columns[0];
            }
            if (ea.index != eb.index) {
                return ea.index < eb.index;
            }
            return ea.edge[1] < eb.edge[1];
        });

        result_.elements.reserve(perm.size());
        result_.faceOffsets.reserve(perm.size() + 1);
        result_.faces.reserve(faces_.size());
        for (const std::size_t i : perm) {
            result_.elements.push_back(elements_[i]);
            result_.faces.insert(result_.faces.end(), faces_.begin() + offsets_[i], faces_.begin() + offsets_[i + 1]);
            result_.faceOffsets.push_back(static_cast<std::uint32_t>(result_.faces.size()));
        }
        result_.infoCount = infoCount;
        return std::move(result_);
    }

private:
    CategoryResult result_;
    std::vector<DefectElement> elements_;
    std::vector<std::uint32_t> offsets_;
    std::vector<Eigen::Index> faces_;
};

/// Incident faces of edge `e` as mesh face indices.
[[nodiscard]] std::vector<Eigen::Index> edgeFaces(const EdgeTopology& topo, std::size_t e) {
    std::vector<Eigen::Index> faces;
    faces.reserve(topo.faceCount(e));
    for (std::uint32_t i = topo.faceOffsets[e]; i < topo.faceOffsets[e + 1]; ++i) {
        faces.push_back(static_cast<Eigen::Index>(topo.incidentFaces[i]));
    }
    return faces;
}

} // namespace

bool faceHasRepeatedVertex(const Mesh& mesh, Eigen::Index f) noexcept {
    const int a = mesh.F(f, 0);
    const int b = mesh.F(f, 1);
    const int c = mesh.F(f, 2);
    return a == b || b == c || c == a;
}

EdgeTopology buildEdgeTopology(const Mesh& mesh) {
    struct Incidence {
        std::uint64_t key;
        std::uint32_t face;
        std::uint8_t forward;
    };

    EdgeTopology topo;
    if (mesh.empty()) {
        return topo;
    }

    std::vector<Incidence> incidences;
    incidences.reserve(toSize(mesh.numFaces()) * 3);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (!faceIsUsable(mesh, f)) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            const int a = mesh.F(f, c);
            const int b = mesh.F(f, (c + 1) % 3);
            const auto lo = static_cast<std::uint32_t>(std::min(a, b));
            const auto hi = static_cast<std::uint32_t>(std::max(a, b));
            incidences.push_back({(std::uint64_t{lo} << 32) | hi, static_cast<std::uint32_t>(f), a < b ? std::uint8_t{1} : std::uint8_t{0}});
        }
    }
    std::sort(incidences.begin(), incidences.end(), [](const Incidence& x, const Incidence& y) {
        return x.key != y.key ? x.key < y.key : x.face < y.face;
    });

    topo.incidentFaces.reserve(incidences.size());
    topo.incidentForward.reserve(incidences.size());
    topo.edges.reserve(incidences.size() / 2 + 1);
    topo.faceOffsets.reserve(incidences.size() / 2 + 2);
    for (std::size_t i = 0; i < incidences.size(); ++i) {
        if (i == 0 || incidences[i].key != incidences[i - 1].key) {
            if (i != 0) {
                topo.faceOffsets.push_back(static_cast<std::uint32_t>(topo.incidentFaces.size()));
            }
            topo.edges.push_back({static_cast<int>(incidences[i].key >> 32), static_cast<int>(incidences[i].key & 0xffffffffu)});
        }
        topo.incidentFaces.push_back(incidences[i].face);
        topo.incidentForward.push_back(incidences[i].forward);
    }
    if (!topo.edges.empty()) {
        topo.faceOffsets.push_back(static_cast<std::uint32_t>(topo.incidentFaces.size()));
    }
    return topo;
}

CategoryResult checkOpenEdges(const Mesh& mesh, std::stop_token stopToken) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::OpenEdges);
    }

    // Boundary loops: connected components of the open-edge graph
    std::vector<std::size_t> openEdges;
    UnionFind loops(toSize(mesh.numVertices()));
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) == 1) {
            openEdges.push_back(e);
            loops.unite(static_cast<std::uint32_t>(topo.edges[e][0]), static_cast<std::uint32_t>(topo.edges[e][1]));
        }
    }

    std::unordered_map<std::uint32_t, std::size_t> loopIdOfRoot;
    std::vector<std::size_t> loopOfEdge;
    std::vector<std::size_t> loopSizes;
    loopOfEdge.reserve(openEdges.size());
    for (const std::size_t e : openEdges) {
        const std::uint32_t root = loops.find(static_cast<std::uint32_t>(topo.edges[e][0]));
        const auto [it, inserted] = loopIdOfRoot.try_emplace(root, loopSizes.size());
        if (inserted) {
            loopSizes.push_back(0);
        }
        ++loopSizes[it->second];
        loopOfEdge.push_back(it->second);
    }

    ResultBuilder builder(DefectCategory::OpenEdges);
    for (std::size_t i = 0; i < openEdges.size(); ++i) {
        const std::size_t e = openEdges[i];
        DefectElement element;
        element.index = topo.edges[e][0];
        element.edge = topo.edges[e];
        element.metric = static_cast<double>(loopSizes[loopOfEdge[i]]);
        element.columns = {static_cast<double>(loopOfEdge[i] + 1), element.metric, edgeLength(mesh, topo.edges[e])};
        builder.add(element, static_cast<Eigen::Index>(topo.incidentFaces[topo.faceOffsets[e]]));
    }
    return builder.finish(SortOrder::HighMetricFirst, loopSizes.size());
}

CategoryResult checkNonManifoldEdges(const Mesh& mesh, std::stop_token stopToken) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::NonManifoldEdges);
    }

    ResultBuilder builder(DefectCategory::NonManifoldEdges);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        const std::uint32_t count = topo.faceCount(e);
        if (count < 3) {
            continue;
        }
        DefectElement element;
        element.index = topo.edges[e][0];
        element.edge = topo.edges[e];
        element.metric = static_cast<double>(count);
        element.columns = {element.metric, edgeLength(mesh, topo.edges[e]), 0.0};
        builder.add(element, edgeFaces(topo, e));
    }
    return builder.finish(SortOrder::HighMetricFirst);
}

CategoryResult checkNonManifoldVertices(const Mesh& mesh, std::stop_token stopToken) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    if (stopToken.stop_requested() || mesh.empty()) {
        return stopToken.stop_requested() ? cancelledResult(DefectCategory::NonManifoldVertices)
                                          : ResultBuilder(DefectCategory::NonManifoldVertices).finish(SortOrder::ByIndex);
    }

    const auto cornerOf = [&mesh](std::uint32_t f, int v) {
        const auto fi = static_cast<Eigen::Index>(f);
        int c = 0;
        while (c < 2 && mesh.F(fi, c) != v) {
            ++c;
        }
        return f * 3u + static_cast<std::uint32_t>(c);
    };

    // Corners (face, vertex) around a vertex are joined when their faces share an edge through it
    UnionFind fans(toSize(mesh.numFaces()) * 3);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        const std::uint32_t first = topo.incidentFaces[topo.faceOffsets[e]];
        for (std::uint32_t i = topo.faceOffsets[e] + 1; i < topo.faceOffsets[e + 1]; ++i) {
            const std::uint32_t other = topo.incidentFaces[i];
            for (const int v : topo.edges[e]) {
                fans.unite(cornerOf(first, v), cornerOf(other, v));
            }
        }
    }

    // Vertex -> corners (CSR)
    const std::size_t nv = toSize(mesh.numVertices());
    std::vector<std::uint32_t> offsets(nv + 1, 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (faceIsUsable(mesh, f)) {
            for (int c = 0; c < 3; ++c) {
                ++offsets[static_cast<std::size_t>(mesh.F(f, c)) + 1];
            }
        }
    }
    std::partial_sum(offsets.begin(), offsets.end(), offsets.begin());
    std::vector<std::uint32_t> corners(offsets.back());
    std::vector<std::uint32_t> fill(offsets.begin(), offsets.end() - 1);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (stopRequested(stopToken, f)) {
            return cancelledResult(DefectCategory::NonManifoldVertices);
        }
        if (faceIsUsable(mesh, f)) {
            for (int c = 0; c < 3; ++c) {
                const auto v = static_cast<std::size_t>(mesh.F(f, c));
                corners[fill[v]++] = static_cast<std::uint32_t>(f) * 3u + static_cast<std::uint32_t>(c);
            }
        }
    }

    ResultBuilder builder(DefectCategory::NonManifoldVertices);
    std::vector<std::uint32_t> roots;
    std::vector<Eigen::Index> faces;
    for (std::size_t v = 0; v < nv; ++v) {
        if (offsets[v + 1] - offsets[v] < 2) {
            continue;
        }
        roots.clear();
        for (std::uint32_t i = offsets[v]; i < offsets[v + 1]; ++i) {
            roots.push_back(fans.find(corners[i]));
        }
        std::sort(roots.begin(), roots.end());
        const auto fanCount = static_cast<std::size_t>(std::unique(roots.begin(), roots.end()) - roots.begin());
        if (fanCount < 2) {
            continue;
        }
        faces.clear();
        for (std::uint32_t i = offsets[v]; i < offsets[v + 1]; ++i) {
            faces.push_back(static_cast<Eigen::Index>(corners[i] / 3u));
        }
        DefectElement element;
        element.index = static_cast<Eigen::Index>(v);
        element.metric = static_cast<double>(fanCount);
        element.columns = {element.metric, static_cast<double>(faces.size()), 0.0};
        builder.add(element, faces);
    }
    return builder.finish(SortOrder::HighMetricFirst);
}

CategoryResult checkFlippedNormals(const Mesh& mesh, std::stop_token stopToken) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::FlippedNormals);
    }
    const std::size_t nf = toSize(mesh.numFaces());

    // Face adjacency across manifold (two-face) edges; `consistent` = opposite traversal directions
    struct Neighbor {
        std::uint32_t face;
        bool consistent;
    };
    std::vector<std::uint32_t> offsets(nf + 1, 0);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) == 2) {
            ++offsets[topo.incidentFaces[topo.faceOffsets[e]] + 1];
            ++offsets[topo.incidentFaces[topo.faceOffsets[e] + 1] + 1];
        }
    }
    std::partial_sum(offsets.begin(), offsets.end(), offsets.begin());
    std::vector<Neighbor> neighbors(offsets.back());
    std::vector<std::uint32_t> fill(offsets.begin(), offsets.end() - 1);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) != 2) {
            continue;
        }
        const std::uint32_t i = topo.faceOffsets[e];
        const std::uint32_t a = topo.incidentFaces[i];
        const std::uint32_t b = topo.incidentFaces[i + 1];
        const bool consistent = topo.incidentForward[i] != topo.incidentForward[i + 1];
        neighbors[fill[a]++] = {b, consistent};
        neighbors[fill[b]++] = {a, consistent};
    }

    // Propagate a relative orientation over each shell, then flag the minority
    std::vector<std::int8_t> orient(nf, -1);
    std::vector<std::uint32_t> shellOf(nf, 0);
    std::vector<std::array<std::size_t, 2>> shellCounts;
    std::vector<std::uint32_t> queue;
    for (std::size_t seed = 0; seed < nf; ++seed) {
        if (stopRequested(stopToken, static_cast<Eigen::Index>(seed))) {
            return cancelledResult(DefectCategory::FlippedNormals);
        }
        if (orient[seed] >= 0 || !faceIsUsable(mesh, static_cast<Eigen::Index>(seed))) {
            continue;
        }
        const auto shell = static_cast<std::uint32_t>(shellCounts.size());
        shellCounts.push_back({0, 0});
        orient[seed] = 0;
        queue.assign(1, static_cast<std::uint32_t>(seed));
        while (!queue.empty()) {
            const std::uint32_t f = queue.back();
            queue.pop_back();
            shellOf[f] = shell;
            ++shellCounts[shell][static_cast<std::size_t>(orient[f])];
            for (std::uint32_t i = offsets[f]; i < offsets[f + 1]; ++i) {
                const Neighbor& n = neighbors[i];
                if (orient[n.face] < 0) {
                    orient[n.face] = static_cast<std::int8_t>(n.consistent ? orient[f] : 1 - orient[f]);
                    queue.push_back(n.face);
                }
            }
        }
    }

    ResultBuilder builder(DefectCategory::FlippedNormals);
    for (std::size_t f = 0; f < nf; ++f) {
        if (orient[f] < 0) {
            continue;
        }
        const auto& counts = shellCounts[shellOf[f]];
        const std::int8_t flipped = counts[1] <= counts[0] ? std::int8_t{1} : std::int8_t{0};
        if (orient[f] == flipped) {
            DefectElement element;
            element.index = static_cast<Eigen::Index>(f);
            element.columns = {static_cast<double>(shellOf[f] + 1), 0.0, 0.0};
            builder.add(element, element.index);
        }
    }
    return builder.finish(SortOrder::ByIndex);
}

CategoryResult checkShells(const Mesh& mesh, std::stop_token stopToken) {
    const EdgeTopology topo = buildEdgeTopology(mesh);
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::Shells);
    }
    const std::size_t nf = toSize(mesh.numFaces());
    UnionFind pieces(nf);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        for (std::uint32_t i = topo.faceOffsets[e] + 1; i < topo.faceOffsets[e + 1]; ++i) {
            pieces.unite(topo.incidentFaces[topo.faceOffsets[e]], topo.incidentFaces[i]);
        }
    }

    std::unordered_map<std::uint32_t, std::size_t> shellOfRoot;
    std::vector<std::vector<Eigen::Index>> shellFaces;
    std::vector<double> shellArea;
    for (std::size_t f = 0; f < nf; ++f) {
        const auto fi = static_cast<Eigen::Index>(f);
        if (stopRequested(stopToken, fi)) {
            return cancelledResult(DefectCategory::Shells);
        }
        if (!faceIsUsable(mesh, fi)) {
            continue;
        }
        const auto [it, inserted] = shellOfRoot.try_emplace(pieces.find(static_cast<std::uint32_t>(f)), shellFaces.size());
        if (inserted) {
            shellFaces.emplace_back();
            shellArea.push_back(0.0);
        }
        shellFaces[it->second].push_back(fi);
        shellArea[it->second] += faceArea(mesh, fi);
    }

    ResultBuilder builder(DefectCategory::Shells);
    for (std::size_t s = 0; s < shellFaces.size(); ++s) {
        DefectElement element;
        element.index = static_cast<Eigen::Index>(s);
        element.metric = static_cast<double>(shellFaces[s].size());
        element.columns = {element.metric, shellArea[s], 0.0};
        builder.add(element, shellFaces[s]);
    }
    return builder.finish(SortOrder::LowMetricFirst, shellFaces.size());
}

CategoryResult checkThinFaces(const Mesh& mesh, const CheckParams& params, std::stop_token stopToken) {
    const ThinFaceReport report = ThinFaceChecker::check(mesh, params.thinThresholds);
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::ThinFaces);
    }
    ResultBuilder builder(DefectCategory::ThinFaces);
    for (const ThinFaceDefect& defect : report.defects) {
        DefectElement element;
        element.index = defect.faceIndex;
        element.metric = defect.minAltitude;
        element.columns = {defect.minAltitude, defect.area, defect.aspectRatio};
        element.subtype = static_cast<int>(classifyThinDefect(defect, params.needleEdgeRatio, params.capAngleDeg));
        builder.add(element, defect.faceIndex);
    }
    return builder.finish(SortOrder::LowMetricFirst);
}

CategoryResult checkDegenerateFaces(const Mesh& mesh, const CheckParams& params, std::stop_token stopToken) {
    ResultBuilder builder(DefectCategory::Degenerate);
    if (mesh.empty()) {
        return builder.finish(SortOrder::ByIndex);
    }
    const double diag = mesh.boundingBox().diagonal();
    const double areaEps = params.degenerateAreaEpsilon * diag * diag;
    const Eigen::Index nv = mesh.numVertices();

    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (stopRequested(stopToken, f)) {
            return cancelledResult(DefectCategory::Degenerate);
        }
        DefectElement element;
        element.index = f;
        bool indicesValid = true;
        for (int c = 0; c < 3; ++c) {
            indicesValid = indicesValid && mesh.F(f, c) >= 0 && mesh.F(f, c) < nv;
        }
        if (!indicesValid || faceHasRepeatedVertex(mesh, f)) {
            element.subtype = 0;
            builder.add(element, f);
            continue;
        }
        const double area = faceArea(mesh, f);
        if (area <= areaEps) {
            double longest = 0.0;
            for (int c = 0; c < 3; ++c) {
                longest = std::max(longest, edgeLength(mesh, {mesh.F(f, c), mesh.F(f, (c + 1) % 3)}));
            }
            element.subtype = 1;
            element.metric = area;
            element.columns = {area, longest, 0.0};
            builder.add(element, f);
        }
    }
    return builder.finish(SortOrder::ByIndex);
}

CategoryResult checkDuplicateFaces(const Mesh& mesh, std::stop_token stopToken) {
    struct Keyed {
        std::array<int, 3> sorted;
        Eigen::Index face;
    };
    ResultBuilder builder(DefectCategory::Duplicates);
    std::vector<Keyed> keyed;
    keyed.reserve(toSize(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (faceIsUsable(mesh, f)) {
            std::array<int, 3> key{mesh.F(f, 0), mesh.F(f, 1), mesh.F(f, 2)};
            std::sort(key.begin(), key.end());
            keyed.push_back({key, f});
        }
    }
    if (stopToken.stop_requested()) {
        return cancelledResult(DefectCategory::Duplicates);
    }
    std::sort(keyed.begin(), keyed.end(), [](const Keyed& a, const Keyed& b) {
        return a.sorted != b.sorted ? a.sorted < b.sorted : a.face < b.face;
    });

    // Same winding = the copy is a cyclic rotation of the original
    const auto sameWinding = [&mesh](Eigen::Index a, Eigen::Index b) {
        for (int shift = 0; shift < 3; ++shift) {
            if (mesh.F(a, 0) == mesh.F(b, shift) && mesh.F(a, 1) == mesh.F(b, (shift + 1) % 3)) {
                return true;
            }
        }
        return false;
    };

    std::size_t groupStart = 0;
    for (std::size_t i = 1; i < keyed.size(); ++i) {
        if (keyed[i].sorted != keyed[groupStart].sorted) {
            groupStart = i;
            continue;
        }
        DefectElement element;
        element.index = keyed[i].face;
        element.subtype = sameWinding(keyed[groupStart].face, keyed[i].face) ? 0 : 1;
        element.columns = {static_cast<double>(keyed[groupStart].face), 0.0, 0.0};
        builder.add(element, keyed[i].face);
    }
    return builder.finish(SortOrder::ByIndex);
}

} // namespace ogun
