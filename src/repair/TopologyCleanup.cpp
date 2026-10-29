#include "repair/TopologyCleanup.hpp"

#include "diagnostics/MeshChecks.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
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

} // namespace ogun
