#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "mesh/Mesh.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace mesheal {

/// What a shell is, judged by rule O from its signed volume and the winding-independent nesting.
enum class ShellRole : std::uint8_t {
    Solid,              ///< Wound outward, in empty space
    NestedSolid,        ///< Wound outward, inside material (an overlap, e.g. a hidden internal part)
    Island,             ///< Wound outward, inside a cavity (a solid floating in a hollow)
    Cavity,             ///< Wound inward, inside material: a hollow. Never flipped, never filled
    InsideOut,          ///< Wound inward, in empty space, nothing inside it: its winding is wrong
    InsideOutWithInner, ///< In a subtree whose outer shell is wound inward and holds shells: stop and report
    Unclassified,       ///< No safe answer (touches or crosses others, unknown container, odd winding)
    NotClosed           ///< Open, non-manifold, inconsistently wound or self-crossing: rule O does not apply
};

[[nodiscard]] const char* shellRoleLabel(ShellRole role) noexcept;

/// Rule O's verdict on one shell.
struct ShellRoleInfo {
    ShellRole role = ShellRole::Unclassified;
    std::string reason;          ///< Plain words, no shell numbers (callers format `#k` from `related`)
    std::vector<int> related;    ///< Shells the reason refers to (container, crossed or touched shells)
    std::optional<int> winding;  ///< Winding of the space the shell sits in (1 in material, 0 in a void)
};

/// Roles of every shell of a `ShellRelations`, in the same order.
struct ShellRoles {
    std::vector<ShellRoleInfo> shells;
    /// Each stopped subtree (an inward outer shell holding shells): its outer shell first, then the rest.
    std::vector<std::vector<int>> stoppedGroups;
    bool complete = true; ///< False when the relations were stopped: every shell is Unclassified

    [[nodiscard]] std::size_t count(ShellRole role) const noexcept;
};

/// `info.reason` with its shell placeholders filled in: `{0}` is `related[0]`, `{list}` every related shell
/// ("#2, #3 and #5" with `label` = Shells-panel numbering).
[[nodiscard]] std::string formatShellRoleReason(const ShellRoleInfo& info, const std::function<std::string(int)>& label);

/// Shells whose pair relations rule O needs: every closed, consistently wound shell with a negative
/// volume, and every shell whose box holds one of those (its possible containers). Empty when no shell
/// is wound inward (then rule O needs no relations). Pass it as `ShellRelationParams::involving`.
[[nodiscard]] std::vector<int> shellsRuleONeeds(std::span<const ShellFacts> shells);

/// Rule O. Outermost first, each closed, consistently wound shell is judged from its signed volume and
/// the winding `w` of the space it sits in: the sum of the signs of its nesting chain (its smallest
/// container, that one's smallest container, ...). Roots: positive Solid; negative InsideOut, or
/// InsideOutWithInner (with its whole subtree) when shells lie inside it. Nested: w = 1 positive
/// NestedSolid, negative Cavity; w = 0 positive Island, negative judged as a root; other w Unclassified.
/// No safe answer means Unclassified: a negative shell that touches or crosses another shell (except a
/// crossing shell with nothing inside it and more than `settings::kRuleOOutsideShare` of its vertices
/// outside every other closed shell: InsideOut), whose box touches a container's box, that lies within
/// a shell that is not closed, or whose nesting probes disagree; and any shell inside a container whose
/// role is not known. Open, inconsistently wound or self-crossing (`shellSelfIntersects[s] != 0`)
/// shells are NotClosed and never containers.
/// `relations` must hold every pair involving `shellsRuleONeeds(relations.shells)` (all pairs is fine).
/// Roles of positive shells are exact only when `relations` hold every pair.
[[nodiscard]] ShellRoles classifyShellRoles(
    const Mesh& mesh,
    const ShellRelations& relations,
    std::span<const std::uint8_t> shellSelfIntersects = {});

/// The Inverted shells check (Mesh Health): one element per InsideOut shell (Shells-panel numbering),
/// warnings for stopped subtrees and inward Unclassified shells. Relations are computed only for
/// `shellsRuleONeeds`, so a file with no inward shell costs one pass over the faces.
[[nodiscard]] CategoryResult checkInvertedShells(
    const Mesh& mesh,
    const CheckParams& params,
    std::stop_token stopToken = {},
    std::atomic<float>* progress = nullptr);

} // namespace mesheal
