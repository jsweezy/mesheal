#include "repair/ShellMerge.hpp"

#include "diagnostics/MeshChecks.hpp"
#include "repair/RepairMetrics.hpp"

#include <manifold/manifold.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

namespace mesheal {

namespace {

[[nodiscard]] const char* manifoldErrorText(manifold::Manifold::Error error) noexcept {
    switch (error) {
    case manifold::Manifold::Error::NoError:
        return "no error";
    case manifold::Manifold::Error::NonFiniteVertex:
        return "a vertex is not finite";
    case manifold::Manifold::Error::NotManifold:
        return "it is not closed and consistently oriented (open edges, flipped faces or fins)";
    default:
        return "Manifold could not read it";
    }
}

/// One shell as a Manifold solid, or why Manifold refuses it.
struct ShellSolid {
    std::optional<manifold::Manifold> solid;
    ManifoldAcceptance acceptance;
};

[[nodiscard]] ShellSolid buildSolid(const Mesh& mesh, std::span<const Eigen::Index> faces) {
    ShellSolid out;
    manifold::Manifold solid(extractFaces(mesh, faces).toMeshGL64());
    if (solid.Status() != manifold::Manifold::Error::NoError) {
        out.acceptance.reason = std::string("Manifold cannot read it: ") + manifoldErrorText(solid.Status()) + ".";
        return out;
    }
    out.acceptance.accepted = true;
    out.acceptance.volume = solid.Volume();
    out.acceptance.area = solid.SurfaceArea();
    out.solid = std::move(solid);
    return out;
}

/// Union-find over shell indices.
class Groups {
public:
    explicit Groups(std::size_t n) : parent_(n) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    [[nodiscard]] int root(int s) {
        while (parent_[static_cast<std::size_t>(s)] != s) {
            parent_[static_cast<std::size_t>(s)] = parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(s)])];
            s = parent_[static_cast<std::size_t>(s)];
        }
        return s;
    }

    void link(int a, int b) {
        a = root(a);
        b = root(b);
        if (a != b) {
            parent_[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
        }
    }

private:
    std::vector<int> parent_;
};

[[nodiscard]] bool meets(const ShellPairRelation* pair) noexcept {
    return pair != nullptr && (pair->crossings > 0 || pair->sharedAreaPairs > 0 || pair->contacts > 0);
}

[[nodiscard]] bool holds(const ShellFacts& inner, int shell) {
    return std::find(inner.containers.begin(), inner.containers.end(), shell) != inner.containers.end();
}

} // namespace

ManifoldAcceptance manifoldAccepts(const Mesh& mesh, std::span<const Eigen::Index> shellFaces) {
    return buildSolid(mesh, shellFaces).acceptance;
}

std::vector<CavityReach> findCavityReaches(
    const ShellRelations& relations,
    std::span<const int> operands,
    std::span<const int> hollows,
    std::span<const int> containers) {
    std::vector<CavityReach> reaches;
    for (std::size_t i = 0; i < hollows.size(); ++i) {
        const int h = hollows[i];
        const int container = i < containers.size() ? containers[i] : -1;
        for (const int o : operands) {
            if (o == h) {
                continue;
            }
            if (meets(relations.pair(o, h)) ||
                (o != container && holds(relations.shells[static_cast<std::size_t>(h)], o))) {
                reaches.push_back({o, h});
            }
        }
    }
    return reaches;
}

ShellMergeResult mergeShells(const Mesh& mesh, const ShellMergeInput& input) {
    const auto start = std::chrono::steady_clock::now();
    ShellMergeResult result;
    const int n = static_cast<int>(input.operands.size());
    const int m = static_cast<int>(input.cavities.size());
    result.shellsIn = input.operands.size() + input.cavities.size();
    if (result.shellsIn < 2) {
        result.message = "Select two or more shells to merge.";
        return result;
    }

    // Every operand on its own: Manifold must read it, and it must face outward
    std::vector<ShellSolid> solids;
    solids.reserve(input.operands.size());
    std::vector<std::uint8_t> refused(input.operands.size(), 0);
    const auto refuse = [&](int o, MergeRefusal why, std::string reason, int cavity = -1) {
        refused[static_cast<std::size_t>(o)] = 1;
        result.refusedShells.push_back({o, why, std::move(reason), cavity});
    };
    std::vector<int> hollows;      // Relation indices: operands 0..n-1, cavities n..n+m-1
    std::vector<int> hollowContainers;
    for (int o = 0; o < n; ++o) {
        solids.push_back(buildSolid(mesh, input.operands[static_cast<std::size_t>(o)]));
        const ManifoldAcceptance& a = solids.back().acceptance;
        if (!a.accepted) {
            refuse(o, MergeRefusal::NotManifold, a.reason);
        } else if (!std::isfinite(a.volume) || a.volume == 0.0) {
            refuse(o, MergeRefusal::NoVolume, "It has no volume.");
        } else if (a.volume < 0.0) {
            refuse(o, MergeRefusal::InwardVolume, "It is wound inward (negative volume): fix orientation first.");
            hollows.push_back(o);
            hollowContainers.push_back(-1);
        }
    }
    for (int c = 0; c < m; ++c) {
        hollows.push_back(n + c);
        hollowContainers.push_back(input.cavities[static_cast<std::size_t>(c)].container);
    }
    std::vector<int> candidates;
    for (int o = 0; o < n; ++o) {
        if (refused[static_cast<std::size_t>(o)] == 0) {
            candidates.push_back(o);
        }
    }

    // Hollows: which operands reach into one, and which float inside a cavity
    std::vector<std::uint8_t> inRefusedGroup(result.shellsIn, 0);
    if (!hollows.empty() && !candidates.empty()) {
        std::vector<std::vector<Eigen::Index>> faceLists = input.operands;
        for (const ShellMergeCavity& cavity : input.cavities) {
            faceLists.push_back(cavity.faces);
        }
        ShellRelationParams params;
        params.intersectionTolerance = input.intersectionTolerance;
        params.involving = hollows;
        const ShellRelations near = computeShellRelations(mesh, faceLists, params);
        const std::vector<CavityReach> reaches = findCavityReaches(near, candidates, hollows, hollowContainers);

        if (!reaches.empty()) {
            // Merge groups from every pair, so only the groups that reach a hollow are refused
            params.involving.clear();
            const ShellRelations all = computeShellRelations(mesh, faceLists, params);
            const auto linked = [&](int s) {
                return s >= n || refused[static_cast<std::size_t>(s)] == 0 ||
                       std::find(hollows.begin(), hollows.end(), s) != hollows.end();
            };
            Groups groups(result.shellsIn);
            for (const ShellPairRelation& pair : all.pairs) {
                if (linked(pair.a) && linked(pair.b)) {
                    groups.link(pair.a, pair.b);
                }
            }
            for (int s = 0; s < n + m; ++s) {
                const int container = all.shells[static_cast<std::size_t>(s)].nestedIn;
                if (container >= 0 && linked(s) && linked(container)) {
                    groups.link(s, container);
                }
            }
            for (int c = 0; c < m; ++c) {
                const int container = input.cavities[static_cast<std::size_t>(c)].container;
                if (container >= 0 && container < n) {
                    groups.link(n + c, container);
                }
            }
            for (const CavityReach& reach : reaches) {
                groups.link(reach.operand, reach.hollow);
            }
            std::vector<int> refusedRoots;
            for (const CavityReach& reach : reaches) {
                const int r = groups.root(reach.hollow);
                if (std::find(refusedRoots.begin(), refusedRoots.end(), r) != refusedRoots.end()) {
                    continue;
                }
                refusedRoots.push_back(r);
                RefusedMergeGroup group;
                for (const CavityReach& other : reaches) {
                    if (other.hollow == reach.hollow) {
                        group.reaching.push_back(other.operand);
                    }
                }
                if (reach.hollow >= n) {
                    group.cavity = reach.hollow - n;
                } else {
                    group.inwardOperand = reach.hollow;
                }
                for (int s = 0; s < n + m; ++s) {
                    if (groups.root(s) == r) {
                        inRefusedGroup[static_cast<std::size_t>(s)] = 1;
                        (s < n ? group.operands : group.cavities).push_back(s < n ? s : s - n);
                    }
                }
                result.refusedGroups.push_back(std::move(group));
            }
        }

        // An operand floating inside a cavity whose container is united would be swallowed: keep it out
        const auto united = [&](int o) {
            return o >= 0 && o < n && refused[static_cast<std::size_t>(o)] == 0 && inRefusedGroup[static_cast<std::size_t>(o)] == 0;
        };
        for (const int o : candidates) {
            if (!united(o)) {
                continue;
            }
            for (int c = 0; c < m; ++c) {
                if (inRefusedGroup[static_cast<std::size_t>(n + c)] == 0 &&
                    united(input.cavities[static_cast<std::size_t>(c)].container) &&
                    holds(near.shells[static_cast<std::size_t>(o)], n + c) && !meets(near.pair(o, n + c))) {
                    refuse(o, MergeRefusal::InsideCavity, "It floats inside a cavity that is kept: left as it is.", c);
                    break;
                }
            }
        }
    }
    for (int c = 0; c < m; ++c) {
        if (inRefusedGroup[static_cast<std::size_t>(n + c)] == 0) {
            ++result.cavitiesKept;
            result.cavityVolume += signedVolume(extractFaces(mesh, input.cavities[static_cast<std::size_t>(c)].faces));
        }
    }

    std::vector<int> united;
    for (int o = 0; o < n; ++o) {
        if (refused[static_cast<std::size_t>(o)] == 0 && inRefusedGroup[static_cast<std::size_t>(o)] == 0) {
            united.push_back(o);
        }
    }
    if (united.size() < 2) {
        result.success = true;
        result.message = "Nothing was merged: fewer than two shells could be united.";
        result.mesh = mesh;
        result.firstMergedFace = mesh.numFaces();
        result.shellsOut = result.shellsIn;
        result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return result;
    }

    std::vector<std::uint8_t> merged(static_cast<std::size_t>(mesh.numFaces()), 0);
    std::vector<manifold::Manifold> operands;
    operands.reserve(united.size());
    std::vector<Eigen::Index> inputFaces; // Face of `inputs` -> face of `mesh`
    Mesh inputs;
    inputs.fileUnit = mesh.fileUnit;
    for (const int o : united) {
        const std::vector<Eigen::Index>& faces = input.operands[static_cast<std::size_t>(o)];
        ShellSolid& s = solids[static_cast<std::size_t>(o)];
        result.volumeBefore += s.acceptance.volume;
        result.areaBefore += s.acceptance.area;
        operands.push_back(std::move(*s.solid));
        appendMesh(inputs, extractFaces(mesh, faces));
        inputFaces.insert(inputFaces.end(), faces.begin(), faces.end());
        for (const Eigen::Index f : faces) {
            merged[static_cast<std::size_t>(f)] = 1;
        }
    }

    const manifold::Manifold unionSolid = manifold::Manifold::BatchBoolean(operands, manifold::OpType::Add);
    if (unionSolid.Status() != manifold::Manifold::Error::NoError) {
        result.message = std::string("The union failed: ") + manifoldErrorText(unionSolid.Status()) + ".";
        return result;
    }
    Mesh unionMesh = Mesh::fromMeshGL64(unionSolid.GetMeshGL64(), mesh.fileUnit);
    result.operandsMerged = united.size();
    result.unionVolume = unionSolid.Volume();
    result.volumeAfter = result.unionVolume + result.cavityVolume;
    result.areaAfter = unionSolid.SurfaceArea();
    result.unionShells = checkShells(unionMesh).infoCount;
    result.shellsOut = result.unionShells + (result.shellsIn - united.size());
    if (inputs.numFaces() > 0) {
        const FaceDiff diff = faceDiff(inputs, unionMesh);
        result.unchangedFraction = 1.0 - static_cast<double>(diff.removed.size()) / static_cast<double>(inputs.numFaces());
        result.removedInputFaces.reserve(diff.removed.size());
        for (const Eigen::Index f : diff.removed) {
            result.removedInputFaces.push_back(inputFaces[static_cast<std::size_t>(f)]);
        }
        std::sort(result.removedInputFaces.begin(), result.removedInputFaces.end());
    }

    std::vector<Eigen::Index> kept;
    kept.reserve(merged.size());
    for (std::size_t f = 0; f < merged.size(); ++f) {
        if (merged[f] == 0) {
            kept.push_back(static_cast<Eigen::Index>(f));
        }
    }
    result.mesh = extractFaces(mesh, kept);
    result.firstMergedFace = result.mesh.numFaces();
    appendMesh(result.mesh, unionMesh);
    result.success = true;
    result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return result;
}

ShellMergeResult mergeShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shells) {
    ShellMergeInput input;
    input.operands.assign(shells.begin(), shells.end());
    return mergeShells(mesh, input);
}

std::string describeRefusedGroup(
    const ShellMergeInput& input,
    const RefusedMergeGroup& group,
    const std::function<std::string(int)>& operandLabel,
    const std::function<std::string(int)>& cavityLabel) {
    std::string reacher;
    for (std::size_t i = 0; i < group.reaching.size(); ++i) {
        reacher += (i == 0 ? "" : (i + 1 == group.reaching.size() ? " and " : ", ")) + operandLabel(group.reaching[i]);
    }
    const std::string verb = group.reaching.size() > 1 ? " reach into " : " reaches into ";
    if (group.cavity >= 0) {
        const int container = input.cavities[static_cast<std::size_t>(group.cavity)].container;
        return reacher + verb + "cavity " + cavityLabel(group.cavity) +
               (container >= 0 ? " of " + operandLabel(container) : std::string()) + ": merge them by hand.";
    }
    return reacher + verb + operandLabel(group.inwardOperand) +
           ", which is wound inward: fix its orientation, or merge them by hand.";
}

} // namespace mesheal
