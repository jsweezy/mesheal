#include "diagnostics/MeshState.hpp"

#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellRelations.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <tuple>

namespace mesheal {

std::string_view crossingKindName(CrossingKind kind) noexcept {
    switch (kind) {
    case CrossingKind::Folded:          return "folded";
    case CrossingKind::VertexNeighbour: return "vertex_neighbour";
    case CrossingKind::Coplanar:        return "coplanar";
    case CrossingKind::Touching:        return "touching";
    case CrossingKind::Crossing:        return "crossing";
    }
    return "crossing";
}

std::vector<int> shellIndexOfFaces(const Mesh& mesh) {
    std::vector<int> shellOf(static_cast<std::size_t>(mesh.numFaces()), -1);
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        for (const Eigen::Index f : shells[s]) {
            shellOf[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }
    return shellOf;
}

namespace {

[[nodiscard]] int sharedVertices(const Mesh& mesh, std::uint32_t a, std::uint32_t b) {
    int shared = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            shared += mesh.F(a, i) == mesh.F(b, j) ? 1 : 0;
        }
    }
    return shared;
}

[[nodiscard]] std::vector<CrossingPair> classify(const Mesh& mesh, const std::vector<FacePairHit>& hits,
                                                 const std::vector<int>& shellOf) {
    std::vector<CrossingPair> pairs;
    pairs.reserve(hits.size());
    for (const FacePairHit& hit : hits) {
        if (!hit.hit) {
            continue;
        }
        CrossingPair pair;
        pair.a = std::min(hit.a, hit.b);
        pair.b = std::max(hit.a, hit.b);
        const int shared = sharedVertices(mesh, pair.a, pair.b);
        pair.kind = shared >= 2      ? CrossingKind::Folded
                    : shared == 1    ? CrossingKind::VertexNeighbour
                    : hit.coplanar   ? CrossingKind::Coplanar
                    : hit.touching   ? CrossingKind::Touching
                                     : CrossingKind::Crossing;
        pair.shellA = shellOf[pair.a];
        pair.shellB = shellOf[pair.b];
        pairs.push_back(pair);
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const CrossingPair& x, const CrossingPair& y) { return std::tie(x.a, x.b) < std::tie(y.a, y.b); });
    pairs.erase(std::unique(pairs.begin(), pairs.end(),
                            [](const CrossingPair& x, const CrossingPair& y) { return x.a == y.a && x.b == y.b; }),
                pairs.end());
    return pairs;
}

[[nodiscard]] double signedVolumeOf(const Mesh& mesh) {
    double volume = 0.0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
        const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1));
        const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2));
        volume += a.dot(b.cross(c));
    }
    return volume / 6.0;
}

} // namespace

std::optional<std::vector<CrossingPair>> classifyCrossings(const Mesh& mesh, const CheckParams& params,
                                                          std::stop_token stopToken) {
    const double tol = mesh.empty() ? 0.0 : params.intersectionTolerance * mesh.boundingBox().diagonal();
    const std::optional<std::vector<FacePairHit>> hits = findFacePairs(mesh, tol, {}, stopToken);
    if (!hits) {
        return std::nullopt;
    }
    return classify(mesh, *hits, shellIndexOfFaces(mesh));
}

MeshState measureMeshState(const Mesh& mesh, const CheckParams& params, std::stop_token stopToken) {
    MeshState state;
    state.faces = static_cast<std::size_t>(mesh.numFaces());
    state.vertices = static_cast<std::size_t>(mesh.V.rows());
    if (mesh.empty()) {
        return state;
    }
    const std::vector<int> shellOf = shellIndexOfFaces(mesh);
    state.shells = shellOf.empty() ? 0 : static_cast<std::size_t>(*std::max_element(shellOf.begin(), shellOf.end()) + 1);
    state.volume = signedVolumeOf(mesh);

    const auto count = [&](DefectCategory category, std::size_t& out) {
        const CategoryResult result = runCategoryCheck(category, mesh, params, stopToken);
        state.complete = state.complete && !result.cancelled;
        out = result.size();
    };
    count(DefectCategory::OpenEdges, state.openEdges);
    count(DefectCategory::NonManifoldEdges, state.nonManifoldEdges);
    count(DefectCategory::NonManifoldVertices, state.nonManifoldVertices);
    count(DefectCategory::FlippedNormals, state.flippedFaces);
    count(DefectCategory::InvertedShells, state.invertedShells);
    count(DefectCategory::Degenerate, state.degenerateFaces);
    count(DefectCategory::Duplicates, state.duplicateFaces);
    count(DefectCategory::ThinFaces, state.thinFaces);

    const double tol = params.intersectionTolerance * mesh.boundingBox().diagonal();
    const std::optional<std::vector<FacePairHit>> hits = findFacePairs(mesh, tol, {}, stopToken);
    if (!hits) {
        state.complete = false;
        return state;
    }
    const std::vector<CrossingPair> pairs = classify(mesh, *hits, shellOf);
    std::vector<std::uint32_t> faces;
    faces.reserve(2 * pairs.size());
    for (const CrossingPair& pair : pairs) {
        auto& counts = pair.withinShell() ? state.pairsWithinShell : state.pairsBetweenShells;
        ++counts[static_cast<std::size_t>(pair.kind)];
        faces.push_back(pair.a);
        faces.push_back(pair.b);
    }
    std::sort(faces.begin(), faces.end());
    state.crossingFaces = static_cast<std::size_t>(std::unique(faces.begin(), faces.end()) - faces.begin());
    return state;
}

} // namespace mesheal
