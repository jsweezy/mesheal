#pragma once

#include "diagnostics/ShellRelations.hpp"
#include "mesh/Mesh.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace mesheal {

/// A hollow passed to `mergeShells`: its faces are kept as they are and never sent to Manifold.
struct ShellMergeCavity {
    std::vector<Eigen::Index> faces;
    int container = -1; ///< Index into `ShellMergeInput::operands` of the shell it is a hollow of (-1: not an operand)
};

/// What `mergeShells` is given. Roles come from the caller (rule O, `classifyShellRoles`): solids, nested
/// solids and islands are operands; cavities are cavities. Every inward shell near the operands should
/// be passed as a cavity, or the union may turn it into a hollow of shells it was not a hollow of.
struct ShellMergeInput {
    std::vector<std::vector<Eigen::Index>> operands; ///< Shells to unite (faces of each)
    std::vector<ShellMergeCavity> cavities;          ///< Hollows to keep, bit-identical
    double intersectionTolerance = settings::kDefaultIntersectionTolerance; ///< Reach test (fraction of the bbox diagonal)
};

/// Would Manifold take this shell as a solid on its own (closed and consistently oriented)?
struct ManifoldAcceptance {
    bool accepted = false;
    std::string reason;  ///< Why not, in plain words (empty when accepted)
    double volume = 0.0; ///< Signed (negative: wound inward); 0 when not accepted
    double area = 0.0;
};

[[nodiscard]] ManifoldAcceptance manifoldAccepts(const Mesh& mesh, std::span<const Eigen::Index> shellFaces);

/// Why an operand was left out of the union (its faces are kept as they are).
enum class MergeRefusal : std::uint8_t {
    NotManifold,  ///< Manifold cannot read it (open edges, flipped faces, fins, non-finite vertices)
    InwardVolume, ///< Wound inward (negative volume): fix orientation first. Treated as a hollow
    NoVolume,     ///< Its volume is zero
    InsideCavity  ///< It floats inside a kept cavity whose container is united: the union would swallow it
};

struct RefusedMergeShell {
    int operand = -1;    ///< Index into `operands`
    MergeRefusal why = MergeRefusal::NotManifold;
    std::string reason;  ///< Plain words, no shell numbers
    int cavity = -1;     ///< InsideCavity: index into `cavities`
};

/// A merge group (operands and hollows linked by crossings, shared face areas, contacts or nesting)
/// in which an operand reaches into a hollow. None of its operands is united; nothing in it changes.
struct RefusedMergeGroup {
    std::vector<int> operands; ///< Indices into `operands` (including inward-refused ones)
    std::vector<int> cavities; ///< Indices into `cavities`
    std::vector<int> reaching; ///< Operands that reach into the hollow (indices into `operands`)
    int cavity = -1;           ///< The hollow: index into `cavities`, or -1 when it is the inward operand `inwardOperand`
    int inwardOperand = -1;    ///< The hollow when it was passed as an operand (refused as InwardVolume)
};

/// Outcome of `mergeShells`. On failure `mesh` is empty and `message` says why.
struct ShellMergeResult {
    bool success = false;           ///< The call finished and `mesh` is the result (it may be unchanged)
    std::string message;
    Mesh mesh;                      ///< Whole mesh: faces not united first (in order), then the union's
    Eigen::Index firstMergedFace = 0; ///< Faces from here on came from the union
    std::size_t shellsIn = 0;       ///< Operands plus cavities
    std::size_t shellsOut = 0;      ///< What they became: pieces of the union plus every shell left as it is
    std::size_t operandsMerged = 0; ///< Operands united (0: nothing was united and `mesh` equals the input)
    std::size_t unionShells = 0;    ///< Separate pieces of the union (shells that never touched stay apart)
    double volumeBefore = 0.0;      ///< Sum of the united operands' volumes (solids only, overlaps counted twice)
    double unionVolume = 0.0;       ///< Volume of the union
    double volumeAfter = 0.0;       ///< Signed total: the union plus the kept cavities (negative)
    double areaBefore = 0.0;
    double areaAfter = 0.0;
    std::size_t cavitiesKept = 0;   ///< Cavities passed through bit-identical (not in a refused group)
    double cavityVolume = 0.0;      ///< Their signed volume (negative)
    std::vector<RefusedMergeShell> refusedShells;
    std::vector<RefusedMergeGroup> refusedGroups;
    std::vector<Eigen::Index> removedInputFaces; ///< Input faces (original indices) the union did not keep bit-identical
    double unchangedFraction = 0.0; ///< Share of the united operands' faces the union kept bit-identical
    double elapsedMs = 0.0;
};

/// An operand that reaches into a hollow (both are shell indices of the `ShellRelations`).
struct CavityReach {
    int operand = -1;
    int hollow = -1;
};

/// Operands that reach into a hollow: they cross, share a face area with or touch its wall, or hold it
/// while not being its container. `containers[i]` is the container of `hollows[i]` (-1: none known, so
/// every operand that holds it reaches it). `relations` must hold every pair involving the hollows.
[[nodiscard]] std::vector<CavityReach> findCavityReaches(
    const ShellRelations& relations,
    std::span<const int> operands,
    std::span<const int> hollows,
    std::span<const int> containers);

/// Cavity-safe Boolean union with Manifold's `BatchBoolean`. Cavities are left out of the union and
/// kept bit-identical. Operands Manifold cannot read, or wound inward, or with no volume, are left out
/// (`refusedShells`) instead of failing the call; an inward one counts as a hollow. A group in which an
/// operand reaches into a hollow is refused as a whole (`refusedGroups`); other groups are still united.
/// An operand floating inside a kept cavity whose container is united is left as it is. Faces of every
/// other shell are kept as they are.
[[nodiscard]] ShellMergeResult mergeShells(const Mesh& mesh, const ShellMergeInput& input);

/// `mergeShells` with every shell as an operand and no cavities.
[[nodiscard]] ShellMergeResult mergeShells(const Mesh& mesh, std::span<const std::vector<Eigen::Index>> shells);

/// "#2 reaches into cavity #3 of #1: merge them by hand." ("#2 and #4 reach ...") with labels of operands
/// and cavities.
[[nodiscard]] std::string describeRefusedGroup(
    const ShellMergeInput& input,
    const RefusedMergeGroup& group,
    const std::function<std::string(int)>& operandLabel,
    const std::function<std::string(int)>& cavityLabel);

} // namespace mesheal
