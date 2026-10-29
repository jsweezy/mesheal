#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/SelfIntersections.hpp"
#include "diagnostics/ShellNesting.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>

namespace ogun {

namespace {

[[nodiscard]] Eigen::Vector3d corner(const Mesh& mesh, Eigen::Index f, int c) {
    return mesh.V.row(mesh.F(f, c)).transpose();
}

[[nodiscard]] bool repeatsVertex(const Mesh& mesh, Eigen::Index f) {
    return mesh.F(f, 0) == mesh.F(f, 1) || mesh.F(f, 1) == mesh.F(f, 2) || mesh.F(f, 0) == mesh.F(f, 2);
}

/// Box, closed flag and signed volume of one shell.
void describeShell(const Mesh& mesh, ShellFacts& shell) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    shell.box = {Eigen::Vector3d::Constant(kInf), Eigen::Vector3d::Constant(-kInf)};
    for (const Eigen::Index f : shell.faces) {
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = corner(mesh, f, c);
            shell.box.min = shell.box.min.cwiseMin(p);
            shell.box.max = shell.box.max.cwiseMax(p);
        }
    }
    if (shell.faces.empty()) {
        shell.box = {};
        return;
    }

    // Volume about the box centre, so far-from-origin shells keep their precision
    const Eigen::Vector3d centre = shell.box.center();
    double volume = 0.0;
    std::vector<std::array<int, 2>> edges;
    edges.reserve(3 * shell.faces.size());
    for (const Eigen::Index f : shell.faces) {
        if (repeatsVertex(mesh, f)) {
            continue;
        }
        const Eigen::Vector3d a = corner(mesh, f, 0) - centre;
        const Eigen::Vector3d b = corner(mesh, f, 1) - centre;
        const Eigen::Vector3d c = corner(mesh, f, 2) - centre;
        volume += a.dot(b.cross(c));
        for (int k = 0; k < 3; ++k) {
            const int u = mesh.F(f, k);
            const int v = mesh.F(f, (k + 1) % 3);
            edges.push_back({std::min(u, v), std::max(u, v)});
        }
    }
    shell.signedVolume = volume / 6.0;
    std::sort(edges.begin(), edges.end());
    shell.closed = !edges.empty();
    for (std::size_t i = 0; i < edges.size() && shell.closed;) {
        std::size_t j = i;
        while (j < edges.size() && edges[j] == edges[i]) {
            ++j;
        }
        shell.closed = j - i == 2;
        i = j;
    }
}

/// Shells whose boxes (grown by `tol`) overlap the box of a shell they may be paired with
/// (sweep along x). With `interest` non-empty, a pair needs one shell of interest.
[[nodiscard]] std::vector<std::uint8_t> shellsWithNeighbours(
    const std::vector<ShellFacts>& shells, double tol, const std::vector<std::uint8_t>& interest) {
    const std::size_t n = shells.size();
    std::vector<std::size_t> order;
    order.reserve(n);
    for (std::size_t s = 0; s < n; ++s) {
        if (!shells[s].faces.empty()) {
            order.push_back(s);
        }
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return shells[a].box.min.x() < shells[b].box.min.x(); });
    std::vector<std::uint8_t> paired(n, 0);
    for (std::size_t i = 0; i < order.size(); ++i) {
        const BoundingBox& bi = shells[order[i]].box;
        for (std::size_t j = i + 1; j < order.size(); ++j) {
            const BoundingBox& bj = shells[order[j]].box;
            if (bj.min.x() > bi.max.x() + 2.0 * tol) {
                break;
            }
            const bool overlap = (bi.min.array() <= bj.max.array() + 2.0 * tol).all() &&
                                 (bj.min.array() <= bi.max.array() + 2.0 * tol).all();
            if (overlap && (interest.empty() || interest[order[i]] != 0 || interest[order[j]] != 0)) {
                paired[order[i]] = 1;
                paired[order[j]] = 1;
            }
        }
    }
    return paired;
}

} // namespace

const ShellPairRelation* ShellRelations::pair(int a, int b) const noexcept {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    const auto it = std::lower_bound(pairs.begin(), pairs.end(), std::pair{lo, hi}, [](const ShellPairRelation& r, const std::pair<int, int>& key) {
        return r.a != key.first ? r.a < key.first : r.b < key.second;
    });
    return it != pairs.end() && it->a == lo && it->b == hi ? &*it : nullptr;
}

std::set<std::pair<int, int>> ShellRelations::overlappingPairs() const {
    std::set<std::pair<int, int>> result;
    for (const ShellPairRelation& r : pairs) {
        if (r.crosses() || r.sharesFaceArea()) {
            result.insert({r.a, r.b});
        }
    }
    return result;
}

std::vector<std::vector<Eigen::Index>> shellsLargestFirst(const Mesh& mesh) {
    const CategoryResult result = checkShells(mesh);
    std::vector<std::size_t> order(result.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const std::size_t na = result.facesOf(a).size();
        const std::size_t nb = result.facesOf(b).size();
        return na != nb ? na > nb : result.facesOf(a).front() < result.facesOf(b).front();
    });
    std::vector<std::vector<Eigen::Index>> shells;
    shells.reserve(order.size());
    for (const std::size_t s : order) {
        const auto faces = result.facesOf(s);
        shells.emplace_back(faces.begin(), faces.end());
    }
    return shells;
}

ShellRelations computeShellRelations(
    const Mesh& mesh,
    std::span<const std::vector<Eigen::Index>> shellFaces,
    const ShellRelationParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    ShellRelations result;
    const std::size_t n = shellFaces.size();
    result.shells.resize(n);
    for (std::size_t s = 0; s < n; ++s) {
        result.shells[s].faces = shellFaces[s];
        describeShell(mesh, result.shells[s]);
    }
    if (mesh.empty()) {
        return result;
    }
    result.tolerance = params.intersectionTolerance * mesh.boundingBox().diagonal();

    std::vector<std::uint8_t> interest;
    if (!params.involving.empty()) {
        interest.assign(n, 0);
        for (const int s : params.involving) {
            if (s >= 0 && static_cast<std::size_t>(s) < n) {
                interest[static_cast<std::size_t>(s)] = 1;
            }
        }
    }

    // Only faces of shells whose boxes overlap a possible partner go into the tree
    const std::vector<std::uint8_t> paired = shellsWithNeighbours(result.shells, result.tolerance, interest);
    std::vector<int> faceShell(static_cast<std::size_t>(mesh.numFaces()), -1);
    for (std::size_t s = 0; s < n; ++s) {
        if (paired[s] == 0) {
            continue;
        }
        for (const Eigen::Index f : shellFaces[s]) {
            faceShell[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }

    FacePairQuery query;
    query.faceGroup = faceShell;
    query.groupOfInterest = interest;
    query.contacts = params.contacts;
    const std::optional<std::vector<FacePairHit>> facePairs =
        findFacePairs(mesh, result.tolerance, query, stopToken, progress, params.maxThreads);
    if (!facePairs || stopToken.stop_requested()) {
        result.complete = false;
        return result;
    }

    // Tally the face pairs per shell pair
    std::vector<ShellPairRelation> tallies;
    tallies.reserve(facePairs->size());
    for (const FacePairHit& hit : *facePairs) {
        const int sa = faceShell[hit.a];
        const int sb = faceShell[hit.b];
        ShellPairRelation r;
        r.a = std::min(sa, sb);
        r.b = std::max(sa, sb);
        r.intersectingPairs = hit.hit ? 1 : 0;
        if (hit.hit && hit.coplanar) {
            r.sharedAreaPairs = 1;
        } else if (hit.touching) {
            r.contacts = 1;
        } else {
            r.crossings = 1;
        }
        tallies.push_back(r);
    }
    std::sort(tallies.begin(), tallies.end(), [](const ShellPairRelation& x, const ShellPairRelation& y) {
        return x.a != y.a ? x.a < y.a : x.b < y.b;
    });
    for (const ShellPairRelation& r : tallies) {
        if (result.pairs.empty() || result.pairs.back().a != r.a || result.pairs.back().b != r.b) {
            result.pairs.push_back({r.a, r.b, 0, 0, 0, 0});
        }
        ShellPairRelation& sum = result.pairs.back();
        sum.crossings += r.crossings;
        sum.sharedAreaPairs += r.sharedAreaPairs;
        sum.contacts += r.contacts;
        sum.intersectingPairs += r.intersectingPairs;
    }

    // Nesting from two probes; a shell they disagree on is not nested
    const std::set<std::pair<int, int>> overlapping = result.overlappingPairs();
    const std::vector<std::vector<int>> byVertex =
        findShellContainers(mesh, shellFaces, overlapping, NestingProbe::FirstVertex);
    const std::vector<std::vector<int>> byCentroid =
        findShellContainers(mesh, shellFaces, overlapping, NestingProbe::FirstFaceCentroid);
    for (std::size_t s = 0; s < n; ++s) {
        ShellFacts& shell = result.shells[s];
        if (byVertex[s] == byCentroid[s]) {
            shell.containers = byVertex[s];
            shell.nestedIn = shell.containers.empty() ? -1 : shell.containers.front();
        } else {
            shell.nestingUnsure = true;
        }
    }
    if (progress != nullptr) {
        progress->store(1.0f);
    }
    return result;
}

ShellRelations computeShellRelations(
    const Mesh& mesh,
    const ShellRelationParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    return computeShellRelations(mesh, shells, params, stopToken, progress);
}

} // namespace ogun
