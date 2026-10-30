#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ShellRelations.hpp"
#include "diagnostics/ShellRoles.hpp"
#include "mesh/Mesh.hpp"
#include "repair/FixAll.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace ogun {

/// Fix All's merge plan (phase 2): which shells in scope are united in which groups, which nested solids
/// are deleted, which groups are refused because a shell reaches into a cavity, which shells are skipped,
/// and what is only reported. Shells are numbered as `shellsLargestFirst` (the Shells panel's order):
/// shell k is `#(k + 1)`.

/// Why a shell that overlaps others is not an operand this pass.
enum class MergeSkipReason : std::uint8_t {
    Open,                 ///< It has open edges
    FinEdges,             ///< Three or more of its faces meet at an edge
    Flipped,              ///< Closed, but its faces are not wound consistently
    InsideOut,            ///< Rule O: inside out (phase 1 could not reverse it)
    Unclassified,         ///< Rule O has no safe answer
    NotManifold,          ///< Manifold cannot read it as a solid
    NoVolume,             ///< Its volume is zero
    ContainerNotMergeable ///< A nested solid whose container cannot be merged
};

struct MergePlanSkipped {
    int shell = -1;
    MergeSkipReason why = MergeSkipReason::Open;
    std::string reason;        ///< Plain words, e.g. "it is still open (4 open edges)"
    std::vector<int> overlaps; ///< Shells in scope it crosses, shares a face area with or lies inside
};

/// Shells united into one solid. Kept cavities whose container is in the group pass through.
struct MergePlanGroup {
    std::vector<int> shells;   ///< Operands, ascending
    std::vector<int> cavities; ///< Cavities of its shells, kept bit-identical
};

/// A shell of a group that reaches into a hollow: crosses, shares a face area with or touches its wall,
/// or holds it while not being its container.
struct MergePlanReach {
    int shell = -1;
    int hollow = -1;
};

/// A group in which a shell reaches into a hollow: none of it is merged, and nested solids inside its
/// shells are not deleted (owner rule: stop and report).
struct MergePlanRefusedGroup {
    std::vector<int> shells;   ///< Its outward shells, ascending
    std::vector<int> hollows;  ///< The hollows it reaches or holds
    std::vector<MergePlanReach> reaches;
    std::string reason;        ///< "#6 and #9 reach into cavity #28 of #12, so their group of 27 shells is not merged: ..."
};

/// A cavity (rule O) in scope, or whose container is in scope, that no shell reaches into: kept.
struct MergePlanCavity {
    int shell = -1;
    int container = -1;       ///< -1: not known
    bool passThrough = false; ///< Its container is in a merge group
};

/// An outward shell wholly inside another shell's material that touches nothing: deleted, which gives
/// the union's result without sending the container through Manifold.
struct MergePlanNestedSolid {
    int shell = -1;
    int container = -1;
};

/// Two shells in scope that touch along an edge or at a point only: not merged, reported.
struct MergePlanContact {
    int a = -1;
    int b = -1;
};

enum class MergeOutsideLinkKind : std::uint8_t {
    Crosses,  ///< The selected shell crosses or shares a face area with the unselected one
    InsideOf, ///< The selected shell is a nested solid of the unselected one (not deleted)
    Holds     ///< The unselected shell is a nested solid of the selected one
};

/// A link between a shell in scope and one outside it: left as it is, reported.
struct MergePlanOutsideLink {
    int shell = -1;   ///< In scope
    int outside = -1; ///< Not in scope
    MergeOutsideLinkKind kind = MergeOutsideLinkKind::Crosses;
};

enum class MergePlanLineKind : std::uint8_t {
    Info,   ///< Grey: what will happen
    Warning ///< Orange: something is left for the user
};

/// A line of the plan, in plain words, with the shells it names (for *Show them*).
struct MergePlanLine {
    MergePlanLineKind kind = MergePlanLineKind::Info;
    std::string text;
    std::vector<int> shells;
};

struct MergePlanParams {
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Fraction of the bbox diagonal (`CheckParams`)
};

struct MergePlan {
    bool complete = true;          ///< False when stopped (then nothing else is filled in)
    bool needsWholeShells = false; ///< A face scope: no merge
    bool tickedByDefault = false;  ///< Two shells in scope overlap (`mergeTickedByDefault`)
    std::vector<std::vector<Eigen::Index>> shells; ///< `shellsLargestFirst`
    std::vector<std::uint8_t> shellInScope;        ///< Every face of the shell is in scope
    ShellRelations relations;      ///< Every pair, contacts on, over `shells`
    ShellRoles roles;              ///< Rule O over `relations`
    std::vector<MergePlanGroup> groups;
    std::vector<MergePlanRefusedGroup> refusedGroups;
    std::vector<MergePlanNestedSolid> isolatedNestedSolids;
    std::vector<MergePlanSkipped> skipped;
    std::vector<std::vector<int>> stoppedGroups; ///< Rule O (inside out with inner shells), outer shell first
    std::vector<MergePlanCavity> cavitiesKept;
    std::vector<MergePlanContact> edgeOrPointContacts;
    std::vector<MergePlanOutsideLink> outsideScopeLinks;
    std::vector<MergePlanLine> lines;
    double elapsedMs = 0.0;

    /// Shells that will be united (all groups).
    [[nodiscard]] std::size_t operandCount() const noexcept;
    /// The plan changes the mesh: a group to unite or a nested solid to delete.
    [[nodiscard]] bool hasWork() const noexcept;
};

/// Plans the merge on `scope` of `mesh`. Roles come from rule O (`classifyShellRoles`) over every pair of
/// shells; nesting, contacts and cavities are read from the whole mesh, but only shells wholly in scope
/// are planned. Two eligible shells in scope (closed, consistently wound, no Fin / other edges, outward,
/// read by Manifold) are linked when they cross or share a face area; a nested solid is linked to its
/// container when it crosses, shares an area with or touches any shell (or holds one), and is marked for
/// deletion when it touches nothing. Edge or point contacts are never links. A group (two or more linked
/// shells) in which a shell reaches into a cavity (or an inward shell rule O cannot judge) is refused
/// whole. A face scope gives `needsWholeShells` and nothing else.
[[nodiscard]] MergePlan planMerge(const Mesh& mesh, const FixAllScope& scope, const MergePlanParams& params = {},
                                  std::stop_token stopToken = {});

/// Shells wholly in `scope` (one flag per shell of `shells`).
[[nodiscard]] std::vector<std::uint8_t> shellsInScope(const FixAllScope& scope,
                                                      const std::vector<std::vector<Eigen::Index>>& shells);

/// The default of the *Merge shells* box: two shells in scope overlap. Overlap = crossing faces, a
/// shared face area, or one is a nested solid (rule O) inside the other. An edge or point contact, or a
/// cavity inside its container, is not an overlap. False for a face scope.
[[nodiscard]] bool mergeTickedByDefault(const FixAllScope& scope, const ShellRelations& relations, const ShellRoles& roles,
                                        const std::vector<std::uint8_t>& shellInScope);

/// The same, working out the relations (every pair) and roles. std::nullopt when stopped.
[[nodiscard]] std::optional<bool> mergeTickedByDefault(const Mesh& mesh, const FixAllScope& scope,
                                                       const MergePlanParams& params = {}, std::stop_token stopToken = {});

/// Phase 2's stages for `plan` (worked out on the mesh phase 2 starts from): one step deleting every
/// isolated nested solid (when there is one), then one step per merge group, which unites the group's
/// shells with `mergeShells` (its pass-through cavities kept bit-identical) and checks the merge guards.
/// Refused groups get no stage. The stages find their shells through `FixAllStageContext::planFaceMap`.
[[nodiscard]] std::vector<FixAllStage> fixAllMergeStages(const MergePlan& plan);

/// The merge guard's volume sanity check: the union `unionVolume` of shells of volumes `solidVolumes`
/// (cavities left out) must lie between the largest of them and their sum, and the signed total of the
/// union and its kept cavities, `signedTotal`, must be the union less the cavities' volumes
/// (`cavityVolumes`, negative), each within `kFixAllMergeVolumeRelTolerance` of the sum. Outside these
/// bounds a shell was probably inside out or misread. Empty when it holds, else why not.
[[nodiscard]] std::string mergeVolumeProblem(std::span<const double> solidVolumes, double unionVolume,
                                             std::span<const double> cavityVolumes, double signedTotal);

} // namespace ogun
