#include "repair/TopologyCleanup.hpp"

#include "core/Constants.hpp"
#include "diagnostics/MeshChecks.hpp"
#include "diagnostics/ReliableNormal.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <optional>

namespace ogun {

namespace {

bool inScope(std::span<const std::uint8_t> faceInScope, Eigen::Index f) noexcept {
    return faceInScope.empty() || faceInScope[static_cast<std::size_t>(f)] != 0;
}

bool indicesValid(const Mesh& mesh, Eigen::Index f) noexcept {
    for (int c = 0; c < 3; ++c) {
        if (mesh.F(f, c) < 0 || mesh.F(f, c) >= mesh.numVertices()) {
            return false;
        }
    }
    return true;
}

std::array<int, 3> sortedKey(const Mesh& mesh, Eigen::Index f) {
    std::array<int, 3> key{mesh.F(f, 0), mesh.F(f, 1), mesh.F(f, 2)};
    std::sort(key.begin(), key.end());
    return key;
}

/// Copy of `mesh` without the faces flagged in `remove`; `V` is kept whole. Fills `faceMap`.
Mesh withoutFaces(const Mesh& mesh, const std::vector<std::uint8_t>& remove, std::vector<Eigen::Index>& faceMap) {
    faceMap.assign(static_cast<std::size_t>(mesh.numFaces()), -1);
    Eigen::Index kept = 0;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (remove[static_cast<std::size_t>(f)] == 0) {
            faceMap[static_cast<std::size_t>(f)] = kept++;
        }
    }
    Mesh out;
    out.fileUnit = mesh.fileUnit;
    out.V = mesh.V;
    out.F.resize(kept, 3);
    out.N = Eigen::MatrixXd::Zero(kept, 3);
    const bool normals = mesh.N.rows() == mesh.F.rows();
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        const Eigen::Index row = faceMap[static_cast<std::size_t>(f)];
        if (row < 0) {
            continue;
        }
        out.F.row(row) = mesh.F.row(f);
        if (normals) {
            out.N.row(row) = mesh.N.row(f);
        } else if (indicesValid(mesh, f)) {
            const Eigen::Vector3d v0 = mesh.V.row(mesh.F(f, 0)).transpose();
            const Eigen::Vector3d e1 = mesh.V.row(mesh.F(f, 1)).transpose() - v0;
            const Eigen::Vector3d e2 = mesh.V.row(mesh.F(f, 2)).transpose() - v0;
            const Eigen::Vector3d n = e1.cross(e2);
            if (n.norm() > 0.0) {
                out.N.row(row) = n.normalized().transpose();
            }
        }
    }
    return out;
}

/// True when face `b` is a cyclic rotation of face `a` (same winding).
bool sameWinding(const Mesh& mesh, Eigen::Index a, Eigen::Index b) noexcept {
    for (int shift = 0; shift < 3; ++shift) {
        if (mesh.F(a, 0) == mesh.F(b, shift) && mesh.F(a, 1) == mesh.F(b, (shift + 1) % 3)) {
            return true;
        }
    }
    return false;
}

/// Neighbour votes for face `f`'s winding: faces outside its vertex set across its three edges that run
/// the shared edge the other way (agree) or the same way (disagree).
std::pair<std::size_t, std::size_t> windingVotes(const Mesh& mesh, const EdgeTopology& topo, Eigen::Index f) {
    const std::array<int, 3> key = sortedKey(mesh, f);
    std::size_t agree = 0;
    std::size_t disagree = 0;
    for (int c = 0; c < 3; ++c) {
        const int a = mesh.F(f, c);
        const int b = mesh.F(f, (c + 1) % 3);
        const std::array<int, 2> edge{std::min(a, b), std::max(a, b)};
        const auto it = std::lower_bound(topo.edges.begin(), topo.edges.end(), edge);
        if (it == topo.edges.end() || *it != edge) {
            continue;
        }
        const auto e = static_cast<std::size_t>(it - topo.edges.begin());
        const bool selfForward = a < b;
        for (std::uint32_t i = topo.faceOffsets[e]; i < topo.faceOffsets[e + 1]; ++i) {
            const auto g = static_cast<Eigen::Index>(topo.incidentFaces[i]);
            if (sortedKey(mesh, g) == key) {
                continue; // a member of the same set
            }
            if ((topo.incidentForward[i] != 0) == selfForward) {
                ++disagree;
            } else {
                ++agree;
            }
        }
    }
    return {agree, disagree};
}

class FaceSets {
public:
    explicit FaceSets(std::size_t n) : parent_(n) {
        std::iota(parent_.begin(), parent_.end(), std::size_t{0});
    }

    std::size_t find(std::size_t x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }

    void unite(std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);
        }
    }

private:
    std::vector<std::size_t> parent_;
};

Eigen::Vector3d faceCross(const Mesh& mesh, Eigen::Index f) {
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(f, 0)).transpose();
    const Eigen::Vector3d e1 = mesh.V.row(mesh.F(f, 1)).transpose() - v0;
    const Eigen::Vector3d e2 = mesh.V.row(mesh.F(f, 2)).transpose() - v0;
    return e1.cross(e2);
}

/// Composes `next` (current rows -> new rows) into `total` (input faces -> current rows).
void composeFaceMap(std::vector<Eigen::Index>& total, const std::vector<Eigen::Index>& next) {
    for (Eigen::Index& row : total) {
        if (row >= 0) {
            row = next[static_cast<std::size_t>(row)];
        }
    }
}

bool reliableFace(const Mesh& mesh, Eigen::Index f) noexcept {
    return hasReliableNormal(mesh.V.row(mesh.F(f, 0)).transpose(), mesh.V.row(mesh.F(f, 1)).transpose(),
                             mesh.V.row(mesh.F(f, 2)).transpose());
}

/// One round of fin / wall detection: the faces to remove (flags), with counts.
struct FinRound {
    std::vector<std::uint8_t> remove;
    std::size_t finEdges = 0;
    std::size_t finFaces = 0;
    std::size_t wallFaces = 0;
    std::size_t sheetsOutOfScope = 0;
    std::size_t removed = 0;
};

FinRound findFinSheets(const Mesh& mesh, std::span<const std::uint8_t> faceInScope) {
    const std::size_t nf = static_cast<std::size_t>(mesh.numFaces());
    FinRound round;
    round.remove.assign(nf, 0);
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const std::vector<std::uint8_t> touching = touchingEdges(mesh, topo);
    std::vector<std::uint8_t> finEdge(topo.numEdges(), 0);
    std::vector<std::uint8_t> finVertex(static_cast<std::size_t>(mesh.numVertices()), 0);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) > 2 && touching[e] == 0) {
            finEdge[e] = 1;
            ++round.finEdges;
            finVertex[static_cast<std::size_t>(topo.edges[e][0])] = 1;
            finVertex[static_cast<std::size_t>(topo.edges[e][1])] = 1;
        }
    }
    if (round.finEdges == 0) {
        return round;
    }

    // Sheets: faces joined across two-face edges (except a sharp bend between two Fin / other edge
    // ends, where a seam goes on) and across the pairs of touching edges
    const double creaseCos = std::cos(settings::kFinSeamCreaseAngleDeg * kDegToRad);
    FaceSets sets(nf);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        const std::uint32_t begin = topo.faceOffsets[e];
        if (topo.faceCount(e) == 2) {
            const auto f = static_cast<Eigen::Index>(topo.incidentFaces[begin]);
            const auto g = static_cast<Eigen::Index>(topo.incidentFaces[begin + 1]);
            if (finVertex[static_cast<std::size_t>(topo.edges[e][0])] != 0 &&
                finVertex[static_cast<std::size_t>(topo.edges[e][1])] != 0) {
                // Sign-free bend (a wall may be wound either way); a fold flat back stays joined (safe side)
                if (reliableFace(mesh, f) && reliableFace(mesh, g) &&
                    std::abs(faceCross(mesh, f).normalized().dot(faceCross(mesh, g).normalized())) < creaseCos) {
                    continue;
                }
            }
            sets.unite(static_cast<std::size_t>(f), static_cast<std::size_t>(g));
        } else if (topo.faceCount(e) > 2 && touching[e] != 0) {
            if (const auto pairs = pairTouchingFaces(mesh, topo, e)) {
                for (const auto& [f, g] : *pairs) {
                    sets.unite(f, g);
                }
            }
        }
    }
    std::vector<double> area(nf, 0.0);
    std::vector<std::uint8_t> sheetInScope(nf, 1);
    for (std::size_t f = 0; f < nf; ++f) {
        const std::size_t s = sets.find(f);
        if (indicesValid(mesh, static_cast<Eigen::Index>(f))) {
            area[s] += 0.5 * faceCross(mesh, static_cast<Eigen::Index>(f)).norm();
        }
        if (!inScope(faceInScope, static_cast<Eigen::Index>(f))) {
            sheetInScope[s] = 0;
        }
    }

    // Decide at each Fin / other edge which sheets stay and which are fins there
    struct AtEdge {
        std::size_t sheet;
        int faces;
        int forward;
    };
    std::vector<std::uint8_t> keeper(nf, 0);
    std::vector<std::uint8_t> minority(nf, 0);
    std::vector<AtEdge> at;
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (finEdge[e] == 0) {
            continue;
        }
        at.clear();
        for (std::uint32_t i = topo.faceOffsets[e]; i < topo.faceOffsets[e + 1]; ++i) {
            const std::size_t s = sets.find(topo.incidentFaces[i]);
            auto it = std::find_if(at.begin(), at.end(), [s](const AtEdge& a) { return a.sheet == s; });
            if (it == at.end()) {
                at.push_back({s, 0, 0});
                it = at.end() - 1;
            }
            ++it->faces;
            it->forward += topo.incidentForward[i] != 0 ? 1 : 0;
        }
        if (at.size() < 2) {
            continue; // one sheet passes through the edge more than once: no safe choice
        }
        std::vector<std::size_t> kept;
        std::size_t closing = 0;
        std::size_t closer = 0;
        bool allSingle = true;
        for (const AtEdge& a : at) {
            if (a.faces == 2 && a.forward == 1) {
                ++closing;
                closer = a.sheet;
            }
            allSingle = allSingle && a.faces == 1;
        }
        if (closing == 1) {
            kept.push_back(closer);
        } else if (closing == 0 && allSingle && at.size() >= 3) {
            std::sort(at.begin(), at.end(), [&](const AtEdge& x, const AtEdge& y) {
                return area[x.sheet] != area[y.sheet] ? area[x.sheet] > area[y.sheet] : x.sheet < y.sheet;
            });
            kept = {at[0].sheet, at[1].sheet};
        } else {
            continue;
        }
        double smallestKept = area[kept.front()];
        for (const std::size_t s : kept) {
            smallestKept = std::min(smallestKept, area[s]);
        }
        bool safe = true;
        for (const AtEdge& a : at) {
            if (std::find(kept.begin(), kept.end(), a.sheet) == kept.end()) {
                safe = safe && area[a.sheet] <= settings::kFinMaxAreaShare * smallestKept;
            }
        }
        if (!safe) {
            continue;
        }
        for (const AtEdge& a : at) {
            const bool isKept = std::find(kept.begin(), kept.end(), a.sheet) != kept.end();
            (isKept ? keeper : minority)[a.sheet] = 1;
        }
    }

    // A sheet goes when it is a fin somewhere, kept nowhere and wholly in scope
    std::vector<std::uint8_t> goes(nf, 0);
    for (std::size_t s = 0; s < nf; ++s) {
        if (minority[s] != 0 && keeper[s] == 0) {
            if (sheetInScope[s] != 0) {
                goes[s] = 1;
            } else {
                ++round.sheetsOutOfScope;
            }
        }
    }
    // Fin (it has an open edge of its own) or internal wall (bounded by Fin / other edges and seams)
    std::vector<std::uint8_t> dangling(nf, 0);
    for (std::size_t e = 0; e < topo.numEdges(); ++e) {
        if (topo.faceCount(e) == 1) {
            const std::size_t s = sets.find(topo.incidentFaces[topo.faceOffsets[e]]);
            dangling[s] = goes[s];
        }
    }
    for (std::size_t f = 0; f < nf; ++f) {
        const std::size_t s = sets.find(f);
        if (goes[s] != 0) {
            round.remove[f] = 1;
            ++round.removed;
            ++(dangling[s] != 0 ? round.finFaces : round.wallFaces);
        }
    }
    return round;
}

/// Bounding box of the vertices that faces use (empty mesh: zero box).
BoundingBox usedVertexBox(const Mesh& mesh) {
    BoundingBox box;
    bool first = true;
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (!indicesValid(mesh, f)) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = mesh.V.row(mesh.F(f, c)).transpose();
            if (first) {
                box.min = box.max = p;
                first = false;
            } else {
                box.min = box.min.cwiseMin(p);
                box.max = box.max.cwiseMax(p);
            }
        }
    }
    return box;
}

bool boxesMeet(const BoundingBox& a, const BoundingBox& b, double tol) noexcept {
    return (a.min.array() <= b.max.array() + 2.0 * tol).all() && (b.min.array() <= a.max.array() + 2.0 * tol).all();
}

} // namespace

RepeatedVertexFacesResult removeRepeatedVertexFaces(const Mesh& mesh, std::span<const std::uint8_t> faceInScope) {
    RepeatedVertexFacesResult result;
    std::vector<std::uint8_t> remove(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (inScope(faceInScope, f) && (!indicesValid(mesh, f) || faceHasRepeatedVertex(mesh, f))) {
            remove[static_cast<std::size_t>(f)] = 1;
            ++result.removed;
        }
    }
    result.mesh = withoutFaces(mesh, remove, result.faceMap);
    return result;
}

DuplicateFacesResult removeDuplicateFaces(const Mesh& mesh, std::span<const std::uint8_t> faceInScope) {
    struct Keyed {
        std::array<int, 3> sorted;
        Eigen::Index face;
    };
    DuplicateFacesResult result;
    std::vector<Keyed> keyed;
    keyed.reserve(static_cast<std::size_t>(mesh.numFaces()));
    for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
        if (indicesValid(mesh, f) && !faceHasRepeatedVertex(mesh, f)) {
            keyed.push_back({sortedKey(mesh, f), f});
        }
    }
    std::sort(keyed.begin(), keyed.end(), [](const Keyed& a, const Keyed& b) {
        return a.sorted != b.sorted ? a.sorted < b.sorted : a.face < b.face;
    });

    std::vector<std::uint8_t> remove(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::optional<EdgeTopology> topo; // built on the first set wound both ways
    std::vector<Eigen::Index> members;
    std::vector<int> winding;
    for (std::size_t start = 0; start < keyed.size();) {
        std::size_t end = start + 1;
        while (end < keyed.size() && keyed[end].sorted == keyed[start].sorted) {
            ++end;
        }
        if (end - start < 2) {
            start = end;
            continue;
        }
        ++result.groups;
        members.clear();
        winding.clear();
        std::size_t plus = 0;
        for (std::size_t i = start; i < end; ++i) {
            members.push_back(keyed[i].face);
            const bool same = sameWinding(mesh, keyed[start].face, keyed[i].face);
            winding.push_back(same ? 1 : -1);
            plus += same ? 1 : 0;
        }
        const std::size_t minus = members.size() - plus;
        start = end;

        // The winding to keep one face of (0: keep none)
        int keep = 1;
        if (minus > 0) {
            if (!topo) {
                topo = buildEdgeTopology(mesh);
            }
            const auto [agree, disagree] = windingVotes(mesh, *topo, members.front());
            keep = agree > disagree ? 1 : disagree > agree ? -1 : plus > minus ? 1 : minus > plus ? -1 : 0;
        }

        // Faces out of scope stay; they must all be of the kept winding
        std::size_t outOfScope = 0;
        bool conflict = false;
        for (std::size_t i = 0; i < members.size(); ++i) {
            if (!inScope(faceInScope, members[i])) {
                ++outOfScope;
                conflict = conflict || winding[i] != keep;
            }
        }
        if (conflict) {
            ++result.groupsSkipped;
            continue;
        }
        bool kept = outOfScope > 0;
        std::size_t& counter = minus > 0 ? result.removedOppositeWinding : result.removedSameWinding;
        for (std::size_t i = 0; i < members.size(); ++i) {
            if (!inScope(faceInScope, members[i])) {
                continue;
            }
            if (!kept && winding[i] == keep) {
                kept = true;
                continue;
            }
            remove[static_cast<std::size_t>(members[i])] = 1;
            ++counter;
        }
    }
    result.mesh = withoutFaces(mesh, remove, result.faceMap);
    return result;
}

FinRemovalResult removeFinFaces(const Mesh& mesh, std::span<const std::uint8_t> faceInScope) {
    FinRemovalResult result;
    result.mesh = mesh;
    result.faceMap.resize(static_cast<std::size_t>(mesh.numFaces()));
    std::iota(result.faceMap.begin(), result.faceMap.end(), Eigen::Index{0});
    std::vector<Eigen::Index> inputFace = result.faceMap; // current row -> input face
    std::vector<std::uint8_t> scope(faceInScope.begin(), faceInScope.end());

    for (int r = 0; r < settings::kFinRemovalMaxRounds; ++r) {
        FinRound round = findFinSheets(result.mesh, scope);
        if (r == 0) {
            result.finEdgesBefore = round.finEdges;
        }
        result.finEdgesLeft = round.finEdges;
        result.sheetsOutOfScope = round.sheetsOutOfScope;
        if (round.removed == 0) {
            break;
        }
        ++result.rounds;
        result.finFaces += round.finFaces;
        result.wallFaces += round.wallFaces;
        std::vector<Eigen::Index> next;
        Mesh trimmed = withoutFaces(result.mesh, round.remove, next);
        std::vector<Eigen::Index> nextInput(static_cast<std::size_t>(trimmed.numFaces()));
        std::vector<std::uint8_t> nextScope(scope.empty() ? 0 : nextInput.size());
        for (std::size_t f = 0; f < next.size(); ++f) {
            if (next[f] < 0) {
                result.removedFaces.push_back(inputFace[f]);
                continue;
            }
            nextInput[static_cast<std::size_t>(next[f])] = inputFace[f];
            if (!scope.empty()) {
                nextScope[static_cast<std::size_t>(next[f])] = scope[f];
            }
        }
        composeFaceMap(result.faceMap, next);
        result.mesh = std::move(trimmed);
        inputFace = std::move(nextInput);
        scope = std::move(nextScope);
        result.finEdgesLeft = 0; // recounted by the next round
        if (r + 1 == settings::kFinRemovalMaxRounds) {
            const EdgeTopology topo = buildEdgeTopology(result.mesh);
            const std::vector<std::uint8_t> touching = touchingEdges(result.mesh, topo);
            for (std::size_t e = 0; e < topo.numEdges(); ++e) {
                result.finEdgesLeft += topo.faceCount(e) > 2 && touching[e] == 0 ? 1 : 0;
            }
        }
    }
    std::sort(result.removedFaces.begin(), result.removedFaces.end());

    // Seams opened: edges of the result with one face that had two or more before
    if (!result.removedFaces.empty()) {
        const EdgeTopology before = buildEdgeTopology(mesh);
        const EdgeTopology after = buildEdgeTopology(result.mesh);
        for (std::size_t e = 0; e < after.numEdges(); ++e) {
            if (after.faceCount(e) != 1) {
                continue;
            }
            const auto it = std::lower_bound(before.edges.begin(), before.edges.end(), after.edges[e]);
            if (it != before.edges.end() && *it == after.edges[e] &&
                before.faceCount(static_cast<std::size_t>(it - before.edges.begin())) >= 2) {
                ++result.seamEdgesOpened;
            }
        }
    }
    return result;
}

double TinyFloatingShellsResult::largestFraction() const noexcept {
    double largest = 0.0;
    for (const TinyFloatingShell& shell : shells) {
        largest = std::max(largest, shell.sizeFraction);
    }
    return largest;
}

TinyFloatingShellsResult findTinyFloatingShells(const Mesh& mesh, const TinyShellSearch& search, std::stop_token stopToken) {
    TinyFloatingShellsResult result;
    result.modelDiagonal = usedVertexBox(mesh).diagonal();
    if (mesh.empty() || !(search.fraction > 0.0) || !(result.modelDiagonal > 0.0)) {
        return result;
    }
    const std::vector<ShellFacts> described =
        search.relations != nullptr ? std::vector<ShellFacts>{} : describeShells(mesh, shellsLargestFirst(mesh));
    const std::vector<ShellFacts>& shells = search.relations != nullptr ? search.relations->shells : described;
    const std::size_t n = shells.size();
    const double tol = search.relations != nullptr ? search.relations->tolerance
                                                   : search.intersectionTolerance * mesh.boundingBox().diagonal();

    // Candidates by size; those whose boxes meet no other shell's float at once
    std::vector<std::uint8_t> allowed(n, search.scopeShells.empty() ? 1 : 0);
    for (const int s : search.scopeShells) {
        if (s >= 0 && static_cast<std::size_t>(s) < n) {
            allowed[static_cast<std::size_t>(s)] = 1;
        }
    }
    std::vector<int> candidates;
    std::vector<int> needRelations;
    std::vector<std::uint8_t> floats(n, 0);
    for (std::size_t s = 0; s < n; ++s) {
        if (allowed[s] == 0 || shells[s].faces.empty() ||
            !(shells[s].box.diagonal() < search.fraction * result.modelDiagonal)) {
            continue;
        }
        candidates.push_back(static_cast<int>(s));
        bool meets = false;
        for (std::size_t t = 0; t < n && !meets; ++t) {
            meets = t != s && !shells[t].faces.empty() && boxesMeet(shells[s].box, shells[t].box, tol);
        }
        if (meets) {
            needRelations.push_back(static_cast<int>(s));
        } else {
            floats[s] = 1;
        }
    }
    if (!needRelations.empty()) {
        ShellRelations computed;
        const ShellRelations* relations = search.relations;
        if (relations == nullptr) {
            std::vector<std::vector<Eigen::Index>> faces(n);
            for (std::size_t s = 0; s < n; ++s) {
                faces[s] = shells[s].faces;
            }
            ShellRelationParams params;
            params.intersectionTolerance = search.intersectionTolerance;
            params.involving = needRelations;
            computed = computeShellRelations(mesh, faces, params, stopToken);
            relations = &computed;
        }
        if (!relations->complete || stopToken.stop_requested()) {
            result.complete = false;
            return result;
        }
        std::vector<std::uint8_t> holdsOne(n, 0);
        for (const ShellFacts& shell : relations->shells) {
            for (const int c : shell.containers) {
                holdsOne[static_cast<std::size_t>(c)] = 1;
            }
        }
        for (const int s : needRelations) {
            const ShellFacts& shell = relations->shells[static_cast<std::size_t>(s)];
            bool meetsOne = false;
            for (const ShellPairRelation& pair : relations->pairs) {
                meetsOne = meetsOne || pair.a == s || pair.b == s;
            }
            floats[static_cast<std::size_t>(s)] = !meetsOne && shell.containers.empty() && !shell.nestingUnsure &&
                                                          holdsOne[static_cast<std::size_t>(s)] == 0
                                                      ? 1
                                                      : 0;
        }
    }
    for (const int s : candidates) {
        if (floats[static_cast<std::size_t>(s)] != 0) {
            const ShellFacts& shell = shells[static_cast<std::size_t>(s)];
            result.shells.push_back({s, shell.faces, shell.box.diagonal() / result.modelDiagonal});
        }
    }
    return result;
}

TinyShellRemovalResult removeTinyFloatingShells(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    double fraction,
    double intersectionTolerance,
    std::stop_token stopToken) {
    TinyShellRemovalResult result;
    TinyShellSearch search;
    search.fraction = fraction;
    search.intersectionTolerance = intersectionTolerance;
    const std::vector<std::vector<Eigen::Index>> shells = faceInScope.empty() ? std::vector<std::vector<Eigen::Index>>{}
                                                                              : shellsLargestFirst(mesh);
    if (!faceInScope.empty()) {
        for (std::size_t s = 0; s < shells.size(); ++s) {
            if (std::all_of(shells[s].begin(), shells[s].end(), [&](Eigen::Index f) { return inScope(faceInScope, f); })) {
                search.scopeShells.push_back(static_cast<int>(s));
            }
        }
    }
    std::vector<std::uint8_t> remove(static_cast<std::size_t>(mesh.numFaces()), 0);
    if (faceInScope.empty() || !search.scopeShells.empty()) {
        result.found = findTinyFloatingShells(mesh, search, stopToken);
    }
    for (const TinyFloatingShell& shell : result.found.shells) {
        for (const Eigen::Index f : shell.faces) {
            remove[static_cast<std::size_t>(f)] = 1;
            ++result.removedFaces;
        }
    }
    result.mesh = withoutFaces(mesh, remove, result.faceMap);
    return result;
}

} // namespace ogun
