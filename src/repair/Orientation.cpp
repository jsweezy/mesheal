#include "repair/Orientation.hpp"

#include "diagnostics/MeshChecks.hpp"

#include <algorithm>
#include <array>
#include <numeric>
#include <optional>

namespace mesheal {

namespace {

constexpr std::size_t kStopPollMask = 0xFFFF;

[[nodiscard]] bool inScope(std::span<const std::uint8_t> faceInScope, std::size_t f) noexcept {
    return faceInScope.empty() || faceInScope[f] != 0;
}

[[nodiscard]] bool faceUsable(const Mesh& mesh, Eigen::Index f) noexcept {
    for (int c = 0; c < 3; ++c) {
        if (mesh.F(f, c) < 0 || mesh.F(f, c) >= mesh.numVertices()) {
            return false;
        }
    }
    return !faceHasRepeatedVertex(mesh, f);
}

/// Reverses face f's winding (and its stored normal).
void flipFace(Mesh& mesh, Eigen::Index f) {
    std::swap(mesh.F(f, 1), mesh.F(f, 2));
    if (mesh.N.rows() == mesh.F.rows()) {
        mesh.N.row(f) = -mesh.N.row(f);
    }
}

std::vector<Eigen::Index> identityMap(Eigen::Index faces) {
    std::vector<Eigen::Index> map(static_cast<std::size_t>(faces));
    std::iota(map.begin(), map.end(), Eigen::Index{0});
    return map;
}

void sortUnique(std::vector<int>& values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

struct FacePassResult {
    std::vector<std::uint8_t> flip;          ///< Faces to flip
    std::vector<std::uint8_t> conflictFace;  ///< Faces in a patch that cannot be oriented
    bool stopped = false;
};

/// (a) Propagates the relative winding over each patch joined by two-face edges and marks the minority
/// faces in scope (ties keep the patch's first face), as `checkFlippedNormals` counts them.
FacePassResult markMinorityFaces(const Mesh& mesh, std::span<const std::uint8_t> faceInScope, std::stop_token stopToken) {
    FacePassResult out;
    const EdgeTopology topo = buildEdgeTopology(mesh);
    const auto nf = static_cast<std::size_t>(mesh.numFaces());
    out.flip.assign(nf, 0);
    out.conflictFace.assign(nf, 0);

    struct Neighbor {
        std::uint32_t face;
        bool consistent; ///< The two faces run the shared edge in opposite directions
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

    std::vector<std::int8_t> orient(nf, -1);
    std::vector<std::uint32_t> patchOf(nf, 0);
    std::vector<std::array<std::size_t, 2>> counts;
    std::vector<std::uint8_t> conflict;
    std::vector<std::uint32_t> queue;
    for (std::size_t seed = 0; seed < nf; ++seed) {
        if ((seed & kStopPollMask) == 0 && stopToken.stop_requested()) {
            out.stopped = true;
            return out;
        }
        if (orient[seed] >= 0 || !faceUsable(mesh, static_cast<Eigen::Index>(seed))) {
            continue;
        }
        const auto patch = static_cast<std::uint32_t>(counts.size());
        counts.push_back({0, 0});
        conflict.push_back(0);
        orient[seed] = 0;
        queue.assign(1, static_cast<std::uint32_t>(seed));
        while (!queue.empty()) {
            const std::uint32_t f = queue.back();
            queue.pop_back();
            patchOf[f] = patch;
            ++counts[patch][static_cast<std::size_t>(orient[f])];
            for (std::uint32_t i = offsets[f]; i < offsets[f + 1]; ++i) {
                const Neighbor& n = neighbors[i];
                const auto expected = static_cast<std::int8_t>(n.consistent ? orient[f] : 1 - orient[f]);
                if (orient[n.face] < 0) {
                    orient[n.face] = expected;
                    queue.push_back(n.face);
                } else if (orient[n.face] != expected) {
                    conflict[patch] = 1; // the propagation came round the other way: no consistent winding
                }
            }
        }
    }

    for (std::size_t f = 0; f < nf; ++f) {
        if (orient[f] < 0) {
            continue;
        }
        const std::uint32_t patch = patchOf[f];
        if (conflict[patch] != 0) {
            out.conflictFace[f] = 1;
            continue;
        }
        const std::int8_t minority = counts[patch][1] <= counts[patch][0] ? std::int8_t{1} : std::int8_t{0};
        if (orient[f] == minority && inScope(faceInScope, f)) {
            out.flip[f] = 1;
        }
    }
    return out;
}

} // namespace

OrientShellsResult orientShells(
    const Mesh& mesh,
    std::span<const std::uint8_t> faceInScope,
    const OrientShellsOptions& options,
    std::stop_token stopToken) {
    OrientShellsResult out;
    out.mesh = mesh;
    out.faceMap = identityMap(mesh.numFaces());
    if (mesh.empty()) {
        return out;
    }
    const auto cancelledResult = [&]() {
        OrientShellsResult cancelled;
        cancelled.mesh = mesh;
        cancelled.faceMap = identityMap(mesh.numFaces());
        cancelled.complete = false;
        return cancelled;
    };

    out.shells = options.relations != nullptr ? std::vector<std::vector<Eigen::Index>>{} : shellsLargestFirst(mesh);
    if (options.relations != nullptr) {
        out.shells.reserve(options.relations->shells.size());
        for (const ShellFacts& facts : options.relations->shells) {
            out.shells.push_back(facts.faces);
        }
    }
    std::vector<int> shellOf(static_cast<std::size_t>(mesh.numFaces()), -1);
    for (std::size_t s = 0; s < out.shells.size(); ++s) {
        for (const Eigen::Index f : out.shells[s]) {
            shellOf[static_cast<std::size_t>(f)] = static_cast<int>(s);
        }
    }

    // (a) Faces against their shell's majority
    if (options.flipFaces) {
        const FacePassResult pass = markMinorityFaces(mesh, faceInScope, stopToken);
        if (pass.stopped) {
            return cancelledResult();
        }
        for (Eigen::Index f = 0; f < mesh.numFaces(); ++f) {
            const auto fs = static_cast<std::size_t>(f);
            if (pass.flip[fs] != 0) {
                flipFace(out.mesh, f);
                ++out.flippedFaces;
                if (shellOf[fs] >= 0) {
                    out.shellsWithFlippedFaces.push_back(shellOf[fs]);
                }
            }
            if (pass.conflictFace[fs] != 0 && shellOf[fs] >= 0) {
                out.nonOrientable.push_back(shellOf[fs]);
            }
        }
        sortUnique(out.shellsWithFlippedFaces);
        sortUnique(out.nonOrientable);
    }
    if (!options.reverseInsideOut) {
        return out;
    }

    // (b) Rule O on closed shells, from the winding after the face pass
    const std::vector<ShellFacts> facts = describeShells(out.mesh, out.shells);
    const std::vector<int> needed = shellsRuleONeeds(facts);
    if (needed.empty()) {
        return out; // no closed shell is wound inward: nothing is inside out, no cavity
    }
    ShellRelations relations;
    if (options.relations != nullptr) {
        relations = *options.relations;
        for (std::size_t s = 0; s < facts.size(); ++s) {
            relations.shells[s].closed = facts[s].closed;
            relations.shells[s].oriented = facts[s].oriented;
            relations.shells[s].signedVolume = facts[s].signedVolume;
        }
    } else {
        ShellRelationParams params;
        params.intersectionTolerance = options.intersectionTolerance;
        params.involving = needed;
        relations = computeShellRelations(out.mesh, out.shells, params, stopToken);
    }
    if (!relations.complete || stopToken.stop_requested()) {
        return cancelledResult();
    }
    out.roles = classifyShellRoles(out.mesh, relations);

    // Inside-out shells wholly in scope; those that cross themselves have no well-defined inside
    std::vector<int> candidates;
    for (std::size_t s = 0; s < out.roles.shells.size(); ++s) {
        if (out.roles.shells[s].role != ShellRole::InsideOut) {
            continue;
        }
        const bool whole = std::all_of(out.shells[s].begin(), out.shells[s].end(), [&](Eigen::Index f) {
            return inScope(faceInScope, static_cast<std::size_t>(f));
        });
        (whole ? candidates : out.outOfScope).push_back(static_cast<int>(s));
    }
    if (!candidates.empty()) {
        const std::optional<std::vector<std::uint8_t>> selfCrossing =
            findSelfCrossingShells(out.mesh, out.shells, candidates, relations.tolerance, stopToken);
        if (!selfCrossing) {
            return cancelledResult();
        }
        if (std::any_of(selfCrossing->begin(), selfCrossing->end(), [](std::uint8_t x) { return x != 0; })) {
            out.roles = classifyShellRoles(out.mesh, relations, *selfCrossing);
            std::erase_if(candidates, [&](int s) {
                if ((*selfCrossing)[static_cast<std::size_t>(s)] == 0) {
                    return false;
                }
                out.selfCrossing.push_back(s);
                return true;
            });
            // A shell newly NotClosed can make others unjudgeable: keep only those still inside out
            std::erase_if(candidates, [&](int s) { return out.roles.shells[static_cast<std::size_t>(s)].role != ShellRole::InsideOut; });
            std::erase_if(out.outOfScope, [&](int s) { return out.roles.shells[static_cast<std::size_t>(s)].role != ShellRole::InsideOut; });
        }
    }
    for (const int s : candidates) {
        for (const Eigen::Index f : out.shells[static_cast<std::size_t>(s)]) {
            flipFace(out.mesh, f);
        }
        out.reversedShells.push_back(s);
    }

    for (std::size_t s = 0; s < out.roles.shells.size(); ++s) {
        const ShellRole role = out.roles.shells[s].role;
        if (role == ShellRole::Cavity) {
            out.cavitiesKept.push_back(static_cast<int>(s));
        } else if (role == ShellRole::Unclassified && relations.shells[s].signedVolume < 0.0) {
            out.unclassified.push_back(static_cast<int>(s));
        }
    }
    out.stopped = out.roles.stoppedGroups;
    return out;
}

FlipShellsResult flipShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shellFaceSets) {
    FlipShellsResult out;
    out.mesh = mesh;
    out.faceMap = identityMap(mesh.numFaces());
    std::vector<std::uint8_t> done(static_cast<std::size_t>(mesh.numFaces()), 0);
    for (const std::vector<Eigen::Index>& faces : shellFaceSets) {
        for (const Eigen::Index f : faces) {
            if (f < 0 || f >= mesh.numFaces() || done[static_cast<std::size_t>(f)] != 0) {
                continue;
            }
            done[static_cast<std::size_t>(f)] = 1;
            flipFace(out.mesh, f);
            ++out.flippedFaces;
        }
    }
    return out;
}

} // namespace mesheal
