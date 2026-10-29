#include "diagnostics/ShellRoles.hpp"
#include "core/Constants.hpp"
#include "core/Settings.hpp"
#include "diagnostics/ShellNesting.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstddef>
#include <numeric>

namespace ogun {

namespace {

[[nodiscard]] bool boxWithin(const BoundingBox& inner, const BoundingBox& outer, double tol) {
    return (inner.min.array() >= outer.min.array() - tol).all() && (inner.max.array() <= outer.max.array() + tol).all();
}

/// The strict box test of `findNestedShells`.
[[nodiscard]] bool boxStrictlyInside(const BoundingBox& inner, const BoundingBox& outer) {
    return (inner.min.array() > outer.min.array()).all() && (inner.max.array() < outer.max.array()).all();
}

[[nodiscard]] bool pointInBox(const Eigen::Vector3d& p, const BoundingBox& box, double tol) {
    return (p.array() >= box.min.array() - tol).all() && (p.array() <= box.max.array() + tol).all();
}

/// Odd ray-hit count from `p` against the faces of a closed shell (winding independent).
[[nodiscard]] bool insideByParity(const Mesh& mesh, const Eigen::Vector3d& p, const std::vector<Eigen::Index>& faces) {
    const Eigen::Vector3d dir = Eigen::Vector3d(kAlphaX, kAlphaY, kAlphaZ).normalized();
    std::size_t hits = 0;
    for (const Eigen::Index f : faces) {
        hits += rayHitsTriangle(p, dir, mesh.V.row(mesh.F(f, 0)).transpose(), mesh.V.row(mesh.F(f, 1)).transpose(),
                                mesh.V.row(mesh.F(f, 2)).transpose())
                    ? 1
                    : 0;
    }
    return hits % 2 == 1;
}

/// More than `kRuleOOutsideShare` of shell s's vertices (a sample of them on big shells) lie outside every
/// other closed shell. A vertex within the box of a shell that is not closed does not count as outside.
[[nodiscard]] bool mostlyOutside(const Mesh& mesh, const ShellRelations& relations, std::size_t s) {
    std::vector<int> vertices;
    vertices.reserve(3 * relations.shells[s].faces.size());
    for (const Eigen::Index f : relations.shells[s].faces) {
        for (int c = 0; c < 3; ++c) {
            vertices.push_back(mesh.F(f, c));
        }
    }
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    if (vertices.empty()) {
        return false;
    }
    const std::size_t samples = std::min(vertices.size(), settings::kRuleOMaxVertexSamples);
    std::size_t outside = 0;
    for (std::size_t i = 0; i < samples; ++i) {
        const Eigen::Vector3d p = mesh.V.row(vertices[i * vertices.size() / samples]).transpose();
        bool out = true;
        for (std::size_t k = 0; k < relations.shells.size() && out; ++k) {
            const ShellFacts& other = relations.shells[k];
            if (k == s || other.faces.empty() || !pointInBox(p, other.box, relations.tolerance)) {
                continue;
            }
            out = other.closed && !insideByParity(mesh, p, other.faces);
        }
        outside += out ? 1 : 0;
    }
    return static_cast<double>(outside) > settings::kRuleOOutsideShare * static_cast<double>(samples);
}

void setRole(ShellRoleInfo& info, ShellRole role, std::string reason, std::vector<int> related = {}) {
    info.role = role;
    info.reason = std::move(reason);
    info.related = std::move(related);
}

[[nodiscard]] int signOfRole(ShellRole role) noexcept {
    switch (role) {
    case ShellRole::Solid:
    case ShellRole::NestedSolid:
    case ShellRole::Island:
    case ShellRole::InsideOut: // flipped by orientation; it never holds a shell
        return 1;
    case ShellRole::Cavity:
        return -1;
    case ShellRole::InsideOutWithInner:
    case ShellRole::Unclassified:
    case ShellRole::NotClosed:
        break;
    }
    return 0;
}

} // namespace

const char* shellRoleLabel(ShellRole role) noexcept {
    switch (role) {
    case ShellRole::Solid:
        return "Solid";
    case ShellRole::NestedSolid:
        return "Nested solid";
    case ShellRole::Island:
        return "Island";
    case ShellRole::Cavity:
        return "Cavity";
    case ShellRole::InsideOut:
        return "Inside out";
    case ShellRole::InsideOutWithInner:
        return "Inside out with inner shells";
    case ShellRole::Unclassified:
        return "Unclassified";
    case ShellRole::NotClosed:
        return "Not closed";
    }
    return "Unknown";
}

std::size_t ShellRoles::count(ShellRole role) const noexcept {
    return static_cast<std::size_t>(
        std::count_if(shells.begin(), shells.end(), [role](const ShellRoleInfo& s) { return s.role == role; }));
}

std::string formatShellRoleReason(const ShellRoleInfo& info, const std::function<std::string(int)>& label) {
    std::string list;
    for (std::size_t i = 0; i < info.related.size(); ++i) {
        list += (i == 0 ? "" : (i + 1 == info.related.size() ? " and " : ", ")) + label(info.related[i]);
    }
    std::string text = info.reason;
    for (const auto& [key, value] : {std::pair<std::string, std::string>{"{list}", list},
                                     {"{0}", info.related.empty() ? std::string() : label(info.related.front())}}) {
        for (std::size_t at = text.find(key); at != std::string::npos; at = text.find(key, at + value.size())) {
            text.replace(at, key.size(), value);
        }
    }
    return text;
}

std::vector<int> shellsRuleONeeds(std::span<const ShellFacts> shells) {
    std::vector<std::uint8_t> needed(shells.size(), 0);
    for (std::size_t s = 0; s < shells.size(); ++s) {
        if (!shells[s].oriented || shells[s].signedVolume >= 0.0) {
            continue;
        }
        needed[s] = 1;
        for (std::size_t k = 0; k < shells.size(); ++k) {
            if (k != s && !shells[k].faces.empty() && boxWithin(shells[s].box, shells[k].box, 0.0)) {
                needed[k] = 1;
            }
        }
    }
    std::vector<int> result;
    for (std::size_t s = 0; s < shells.size(); ++s) {
        if (needed[s] != 0) {
            result.push_back(static_cast<int>(s));
        }
    }
    return result;
}

ShellRoles classifyShellRoles(const Mesh& mesh, const ShellRelations& relations, std::span<const std::uint8_t> shellSelfIntersects) {
    ShellRoles out;
    const std::vector<ShellFacts>& facts = relations.shells;
    const std::size_t n = facts.size();
    out.shells.resize(n);
    if (!relations.complete) {
        out.complete = false;
        for (ShellRoleInfo& info : out.shells) {
            setRole(info, ShellRole::Unclassified, "was not judged: the shell check was stopped.");
        }
        return out;
    }

    // Who crosses or touches whom, and the nesting tree
    std::vector<std::vector<int>> crossing(n);
    std::vector<std::vector<int>> touching(n);
    for (const ShellPairRelation& r : relations.pairs) {
        auto& list = r.crosses() ? crossing : touching;
        if (r.crosses() || r.sharesFaceArea() || r.contacts > 0) {
            list[static_cast<std::size_t>(r.a)].push_back(r.b);
            list[static_cast<std::size_t>(r.b)].push_back(r.a);
        }
    }
    const auto parent = [&](std::size_t s) { return facts[s].nestingUnsure ? -1 : facts[s].nestedIn; };
    std::vector<std::vector<int>> children(n);
    std::vector<std::size_t> depth(n, 0);
    for (std::size_t s = 0; s < n; ++s) {
        if (const int k = parent(s); k >= 0) {
            children[static_cast<std::size_t>(k)].push_back(static_cast<int>(s));
        }
        for (int k = parent(s); k >= 0 && depth[s] <= n; k = parent(static_cast<std::size_t>(k))) {
            ++depth[s];
        }
    }
    const auto inSubtreeOf = [&](std::size_t s, int root) {
        std::size_t steps = 0;
        for (int k = parent(s); k >= 0 && steps <= n; k = parent(static_cast<std::size_t>(k)), ++steps) {
            if (k == root) {
                return true;
            }
        }
        return false;
    };

    std::vector<std::uint8_t> eligible(n, 0);
    std::vector<std::uint8_t> decided(n, 0);
    for (std::size_t s = 0; s < n; ++s) {
        const bool selfCrossing = s < shellSelfIntersects.size() && shellSelfIntersects[s] != 0;
        if (facts[s].faces.empty() || !facts[s].closed) {
            setRole(out.shells[s], ShellRole::NotClosed, "is not closed (holes or non-manifold edges): rule O needs a closed shell.");
        } else if (!facts[s].oriented) {
            setRole(out.shells[s], ShellRole::NotClosed, "has faces wound against their neighbours: orient its faces first.");
        } else if (selfCrossing) {
            setRole(out.shells[s], ShellRole::NotClosed, "crosses itself: its inside is not well defined.");
        } else {
            eligible[s] = 1;
            continue;
        }
        decided[s] = 1;
    }

    // Shells whose box holds s's box although rule O cannot judge them (open or mis-wound containers)
    const auto withinUnjudged = [&](std::size_t s) {
        for (std::size_t k = 0; k < n; ++k) {
            if (k != s && eligible[k] == 0 && !facts[k].faces.empty() &&
                boxWithin(facts[s].box, facts[k].box, relations.tolerance)) {
                return static_cast<int>(k);
            }
        }
        return -1;
    };
    // A closed shell whose box holds s's box but fails the strict test (s may touch its wall)
    const auto boxTouches = [&](std::size_t s) {
        for (std::size_t k = 0; k < n; ++k) {
            if (k != s && eligible[k] != 0 && boxWithin(facts[s].box, facts[k].box, relations.tolerance) &&
                !boxStrictlyInside(facts[s].box, facts[k].box)) {
                return static_cast<int>(k);
            }
        }
        return -1;
    };

    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return depth[a] < depth[b]; });

    for (const std::size_t s : order) {
        if (decided[s] != 0) {
            continue;
        }
        decided[s] = 1;
        ShellRoleInfo& info = out.shells[s];
        const double volume = facts[s].signedVolume;
        const bool inward = volume < 0.0;
        const int container = parent(s);
        if (volume == 0.0) {
            setRole(info, ShellRole::Unclassified, "encloses no volume: left as it is.");
            continue;
        }
        if (facts[s].nestingUnsure) {
            if (inward) {
                setRole(info, ShellRole::Unclassified,
                        "is wound inward and its inside test is unsure: cavity or inside out? Left as it is.");
            } else {
                setRole(info, ShellRole::Solid, "is wound outward; its inside test is unsure, so it counts as free-standing.");
            }
            continue;
        }

        // Winding of the space it sits in: the signs along its nesting chain
        int w = 0;
        int unknown = -1;
        std::size_t steps = 0;
        for (int k = container; k >= 0 && steps <= n; k = parent(static_cast<std::size_t>(k)), ++steps) {
            const auto ks = static_cast<std::size_t>(k);
            const int sign = decided[ks] != 0 ? signOfRole(out.shells[ks].role) : 0;
            if (sign == 0) {
                unknown = k;
                break;
            }
            w += sign;
        }
        if (unknown >= 0) {
            if (out.shells[static_cast<std::size_t>(unknown)].role == ShellRole::NotClosed) {
                setRole(info, ShellRole::Unclassified,
                        inward ? "is wound inward inside {0}, which is not closed or is mis-wound: cavity or inside out? "
                                 "Repair {0} first."
                               : "lies inside {0}, which is not closed or is mis-wound: left as it is.",
                        {unknown});
            } else {
                setRole(info, ShellRole::Unclassified, "lies inside {0}, whose role is not known: left as it is.", {unknown});
            }
            continue;
        }
        info.winding = w;

        if (!inward) {
            if (container < 0) {
                setRole(info, ShellRole::Solid, "is wound outward and sits in empty space.");
            } else if (w == 0) {
                setRole(info, ShellRole::Island, "is wound outward inside cavity {0}: a solid in the hollow.", {container});
            } else if (w == 1) {
                setRole(info, ShellRole::NestedSolid, "is wound outward inside {0}: material inside material (an overlap).",
                        {container});
            } else {
                setRole(info, ShellRole::Unclassified,
                        "is wound outward inside {0}, inside " + std::to_string(w) + " layers of material: left as it is.",
                        {container});
            }
            continue;
        }

        // Wound inward: no safe answer means no flip
        if (!crossing[s].empty() || !touching[s].empty()) {
            if (!crossing[s].empty() && children[s].empty() && mostlyOutside(mesh, relations, s)) {
                setRole(info, ShellRole::InsideOut,
                        "is wound inward and crosses {list}, but most of it lies outside every other shell: inside out.",
                        crossing[s]);
            } else if (!crossing[s].empty()) {
                setRole(info, ShellRole::Unclassified, "is wound inward and crosses {list}: cavity or inside out? Left as it is.",
                        crossing[s]);
            } else {
                setRole(info, ShellRole::Unclassified, "is wound inward and touches {list}: cavity or inside out? Left as it is.",
                        touching[s]);
            }
            continue;
        }
        if (const int k = withinUnjudged(s); k >= 0) {
            setRole(info, ShellRole::Unclassified,
                    "is wound inward within the box of {0}, which is not closed or is mis-wound: cavity or inside out? "
                    "Repair {0} first.",
                    {k});
            continue;
        }
        if (container < 0) {
            if (const int k = boxTouches(s); k >= 0) {
                setRole(info, ShellRole::Unclassified,
                        "is wound inward and its box touches the box of {0}: cavity or inside out? Left as it is.", {k});
                continue;
            }
        }
        if (w == 1) {
            setRole(info, ShellRole::Cavity, "is wound inward inside {0}: a cavity, kept as it is.", {container});
        } else if (w == 0 && children[s].empty()) {
            if (container < 0) {
                setRole(info, ShellRole::InsideOut, "is wound inward with nothing around it or inside it: inside out.");
            } else {
                setRole(info, ShellRole::InsideOut, "is wound inward inside cavity {0}: an island that is inside out.",
                        {container});
            }
        } else if (w == 0) {
            // Stop and report: the whole part may be inside out, or only this shell
            std::vector<int> group{static_cast<int>(s)};
            for (std::size_t t = 0; t < n; ++t) {
                if (inSubtreeOf(t, static_cast<int>(s))) {
                    group.push_back(static_cast<int>(t));
                }
            }
            setRole(info, ShellRole::InsideOutWithInner,
                    "is wound inward and holds {list}: the whole part may be inside out, or only this shell. Flip by hand.",
                    {group.begin() + 1, group.end()});
            for (std::size_t i = 1; i < group.size(); ++i) {
                const auto t = static_cast<std::size_t>(group[i]);
                decided[t] = 1;
                setRole(out.shells[t], ShellRole::InsideOutWithInner,
                        "lies inside {0}, which is wound inward and holds other shells: flip by hand.",
                        {static_cast<int>(s)});
            }
            out.stoppedGroups.push_back(std::move(group));
        } else {
            setRole(info, ShellRole::Unclassified,
                    "is wound inward inside {0}, where the winding is " + std::to_string(w) + ": left as it is.",
                    {container});
        }
    }
    return out;
}

CategoryResult checkInvertedShells(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken,
    std::atomic<float>* progress) {
    CategoryResult result;
    result.category = DefectCategory::InvertedShells;
    if (stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    if (mesh.empty()) {
        return result;
    }
    const std::vector<std::vector<Eigen::Index>> shells = shellsLargestFirst(mesh);
    const std::vector<int> needed = shellsRuleONeeds(describeShells(mesh, shells));
    if (needed.empty()) {
        return result; // nothing is wound inward
    }
    ShellRelationParams relationParams;
    relationParams.intersectionTolerance = params.intersectionTolerance;
    relationParams.involving = needed;
    const ShellRelations relations = computeShellRelations(mesh, shells, relationParams, stopToken, progress);
    if (!relations.complete || stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    const ShellRoles roles = classifyShellRoles(mesh, relations);

    // Shells-panel numbering: shell k is #(k + 1)
    const auto label = [](int s) { return "#" + std::to_string(s + 1); };
    for (std::size_t s = 0; s < roles.shells.size(); ++s) {
        if (roles.shells[s].role != ShellRole::InsideOut) {
            continue;
        }
        DefectElement element;
        element.index = static_cast<Eigen::Index>(s);
        element.metric = static_cast<double>(shells[s].size());
        element.columns = {static_cast<double>(shells[s].size()), relations.shells[s].signedVolume, 0.0};
        result.elements.push_back(element);
        result.faces.insert(result.faces.end(), shells[s].begin(), shells[s].end());
        result.faceOffsets.push_back(static_cast<std::uint32_t>(result.faces.size()));
    }
    for (const std::vector<int>& group : roles.stoppedGroups) {
        std::string inner;
        for (std::size_t i = 1; i < group.size(); ++i) {
            inner += (i == 1 ? "" : (i + 1 == group.size() ? " and " : ", ")) + label(group[i]);
        }
        result.warnings.push_back({label(group.front()) + " and its inner shell" + (group.size() > 2 ? "s " : " ") + inner +
                                       " look inside out together. Flip them by hand.",
                                   group});
    }
    for (std::size_t s = 0; s < roles.shells.size(); ++s) {
        const ShellRoleInfo& info = roles.shells[s];
        if (info.role == ShellRole::Unclassified && relations.shells[s].signedVolume < 0.0) {
            result.warnings.push_back({label(static_cast<int>(s)) + " " + formatShellRoleReason(info, label),
                                       {static_cast<int>(s)}});
        }
    }
    return result;
}

} // namespace ogun
