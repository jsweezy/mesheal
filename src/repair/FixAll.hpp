#pragma once

#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/IntersectionHealer.hpp"

#include <Eigen/Core>

#include <atomic>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace ogun {

/// Fix All: a headless engine that repairs a mesh in passes. One pass is the whole sequence: phase 1
/// (repair each shell), phase 2 (merge the shells), phase 3 (clean up the merge). The run repeats
/// passes up to `FixAllParams::passes`, judges each pass by `FixAllPriority`, keeps the best one, and
/// stops when the mesh is clean, repeats a state, makes no progress in two passes, reaches the pass
/// limit or the time budget, or is cancelled. Each step only has to pass its hard guards; a step may
/// leave a problem a later step repairs. GUI-free: the controller runs it in the background.

/// The three phases of a pass.
enum class FixAllPhase : std::uint8_t {
    Repair = 1,  ///< Repair each shell
    Merge = 2,   ///< Merge the shells
    Cleanup = 3  ///< Clean up the merged result
};

/// What the user selected: the whole mesh (nothing selected), whole shells, or faces.
enum class FixAllScopeKind : std::uint8_t { Whole, Shells, Faces };

/// The faces Fix All may change, carried through every step: faces a step adds are in scope, faces it
/// removes leave it, and every re-index maps the flags through the step's face map.
struct FixAllScope {
    FixAllScopeKind kind = FixAllScopeKind::Whole;
    std::vector<std::uint8_t> faceInScope; ///< One flag per face
    std::vector<std::uint8_t> pinned;      ///< One flag per vertex: used by a face in scope and a face outside it (never moves)

    /// Every face of `mesh`.
    [[nodiscard]] static FixAllScope whole(const Mesh& mesh);
    /// The flagged faces (one flag per face; empty = every face). A mask of the wrong size gives a
    /// scope that does not `matches` the mesh.
    [[nodiscard]] static FixAllScope fromFaceMask(const Mesh& mesh, FixAllScopeKind kind,
                                                  std::span<const std::uint8_t> faceInScope);

    /// One face flag per face and one pin flag per vertex of `mesh`.
    [[nodiscard]] bool matches(const Mesh& mesh) const noexcept;
    [[nodiscard]] bool inScope(Eigen::Index face) const noexcept;
    [[nodiscard]] std::size_t faceCount() const noexcept;
    [[nodiscard]] bool everyFace() const noexcept;

    /// Carries the scope to `result`, the mesh a step made: `faceMap` maps each old face to its new row
    /// (-1 = removed; empty = the old faces kept their rows). Faces of `result` no old face maps to are
    /// new and in scope. The pins are worked out again from the carried faces.
    void carry(std::span<const Eigen::Index> faceMap, const Mesh& result);
    /// Pins every vertex used by a face in scope and a face outside it.
    void updatePins(const Mesh& mesh);
};

/// Fix All's settings. The thresholds of the rows (thin aspect, intersection tolerance) come from
/// `checkParams`, so Fix All and the per-row tools agree.
struct FixAllParams {
    int passes = settings::kDefaultFixAllPasses;                         ///< 1 = One pass (the pass is returned even when worse)
    double timeBudgetSeconds = settings::kDefaultFixAllTimeBudgetSeconds; ///< 0 = no budget
    bool weld = settings::kDefaultFixAllWeld;
    /// Fix All's own weld tolerance, never the load weld's (`ToleranceConfig{}`).
    ToleranceConfig weldTolerance{settings::kDefaultFixAllWeldAbsFloor, settings::kDefaultFixAllWeldRelTol};
    bool removeTinyShells = settings::kDefaultFixAllTinyShells;
    double tinyShellFraction = settings::kDefaultFixAllTinyShellFraction;
    bool removeFins = settings::kDefaultFixAllFins;
    bool closeCracks = settings::kDefaultFixAllCloseCracks;
    bool healThinFaces = settings::kDefaultFixAllHealThin;
    bool mergeShells = settings::kDefaultFixAllMergeShells;
    CheckParams checkParams{};
    HoleFillSettings holeFill{};                ///< Close Crack and the hole fill (Holes tool settings)
    IntersectionRepairSettings islandRepair{};  ///< The island step (Intersections tool settings)
};

/// The priority score of a mesh state, lexicographic, lower is better. The order follows the repair
/// dependencies: an open or non-manifold shell cannot be oriented, merged or cut, so closing comes
/// first; then orientation, crossings, junk faces and thin faces. Only used to compare two states.
struct FixAllPriority {
    std::size_t closedManifold = 0; ///< Open edges + non-manifold edges + non-manifold vertices
    std::size_t oriented = 0;       ///< Flipped faces + inside-out closed shells (rule O; cavities are not counted)
    std::size_t intersecting = 0;   ///< Faces in scope that cross another face (in scope or not)
    std::size_t junk = 0;           ///< Degenerate + duplicate faces + tiny floating shells (when their removal is on)
    std::size_t thin = 0;           ///< Thin faces (aspect over the limit)

    [[nodiscard]] auto operator<=>(const FixAllPriority&) const = default;
    /// Closed, manifold, oriented, crossing-free and free of junk; thin faces left do not count.
    [[nodiscard]] bool clean() const noexcept {
        return closedManifold == 0 && oriented == 0 && intersecting == 0 && junk == 0;
    }
};

/// The Mesh Health counts the score is built from, restricted to the scope.
struct FixAllCounts {
    std::size_t openEdges = 0;
    std::size_t nonManifoldEdges = 0;
    std::size_t nonManifoldVertices = 0;
    std::size_t flippedFaces = 0;
    std::size_t invertedShells = 0;    ///< Inside-out shells (the Inverted shells row)
    std::size_t intersectingFaces = 0;
    std::size_t degenerateFaces = 0;
    std::size_t duplicateFaces = 0;
    std::size_t thinFaces = 0;
    std::size_t tinyShells = 0;        ///< Tiny floating shells wholly in scope (0 when not counted)
    bool complete = true;              ///< False when stopped

    [[nodiscard]] FixAllPriority priority() const noexcept;
};

/// Runs the Mesh Health checks behind the score (`runCategoryCheck`, never the GUI's runner) and counts
/// the defects that touch the scope: an element counts when one of its faces is in scope (open edges:
/// every rim edge of a hole with a rim face in scope).
[[nodiscard]] FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                              std::stop_token stopToken = {});

/// The same, also counting tiny floating shells (`findTinyFloatingShells` at `tinyShellFraction`, shells
/// wholly in scope; none for a face scope or a fraction of 0) as junk, so a pass that deletes them is
/// progress. The engine passes its fraction when the tiny-shell step is on.
[[nodiscard]] FixAllCounts countFixAllDefects(const Mesh& mesh, const FixAllScope& scope, const CheckParams& params,
                                              double tinyShellFraction, std::stop_token stopToken = {});

/// A hash of the sorted face geometry keys (`faceGeometryKey`): the same surface gives the same hash
/// whatever the vertex or face numbering. Used to notice a pass that returns to an earlier state.
[[nodiscard]] std::uint64_t meshGeometryHash(const Mesh& mesh);

/// What the weld will do at most, worked out on the scope before the run (for the plan line and the note).
struct FixAllWeldPlan {
    double maxTolerance = 0.0;       ///< T_max = max(floor, rel x M), M the largest |coordinate| in scope
    double scopeDiagonal = 0.0;      ///< Bounding-box diagonal of the vertices of the faces in scope
    std::size_t collapsingEdges = 0; ///< Edges in scope whose two ends match within the tolerance (they would collapse)
    bool noteNeeded = false;         ///< T_max above `kFixAllWeldNoteDiagonalShare` of the diagonal

    /// T_max as a share of the scope's diagonal (0 when the diagonal is 0).
    [[nodiscard]] double diagonalShare() const noexcept;
};

[[nodiscard]] FixAllWeldPlan planFixAllWeld(const Mesh& mesh, const FixAllScope& scope, const ToleranceConfig& tolerance);

/// The steps of a pass, in the order they run.
enum class FixAllStageId : std::uint8_t {
    Weld,          ///< Step 0b: weld by distance (Fix All's tolerance, surfaces kept apart)
    DropRepeated,  ///< Step 1: remove faces that repeat a vertex
    Dedupe,        ///< Step 2: remove duplicate faces
    SplitTouching, ///< Step 3: split touching edges and pinched vertices (geometry unchanged)
    TinyShells,    ///< Step 4: delete tiny floating shells (whole shells only)
    Fins,          ///< Step 5: remove fins and internal walls
    Orient,        ///< Step 6: flip faces against their shell's majority; reverse inside-out shells (rule O)
    CloseCracks,   ///< Step 7: Close Crack on the holes shaped like cracks
    Slivers,       ///< Step 8: heal thin faces (flip / collapse, pinned vertices kept)
    FillHoles,     ///< Step 9: fill every hole, the fill that crosses least when every fill crosses
    Islands,       ///< Step 10: patch intersection islands within shells
    OrientAgain,   ///< Step 11: step 6 again (shells closed this pass are judged by rule O here)
    Custom         ///< A stage a test inserts
};

/// A problem a step leaves for a later step (the deferral table).
enum class FixAllDeferredKind : std::uint8_t {
    OpenEdges,           ///< Left for the hole fill (this pass or the next)
    NonManifoldEdges,    ///< Left for the split and fin steps of the next pass
    CrossingsWithinShell,///< Left for the island step
    CrossingsWithShell   ///< A fill's patch crosses another shell: left for the merge, or reported
};

struct FixAllDeferred {
    FixAllDeferredKind kind = FixAllDeferredKind::OpenEdges;
    std::size_t count = 0;
    std::string text; ///< "opened 2 edges: left for the hole fill"
};

/// Shells a report line names, for the controller to number and show: each entry is the faces of one
/// shell, in the numbering of the mesh the step returned (the step's input when it was not kept). The
/// line itself numbers them `#k` in the Shells-panel order of that mesh (`shellsLargestFirst`).
struct FixAllShellNote {
    std::string text;                              ///< "#1 and its inner shell #2 look inside out together: left for you"
    std::vector<std::vector<Eigen::Index>> shells; ///< Faces of each shell named, in the order the text names them
};

/// What a stage sees besides the mesh.
struct FixAllStageContext {
    const FixAllParams& params;
    const FixAllScope& scope; ///< Matches the mesh the stage gets
    int pass = 1;
    FixAllPhase phase = FixAllPhase::Repair;
    std::stop_token stopToken;
};

/// What a stage returns. The engine keeps `mesh` only when `changed` and every guard passes.
struct FixAllStageOutcome {
    Mesh mesh;
    bool changed = false;
    std::vector<Eigen::Index> faceMap; ///< Old face -> new row, -1 = removed; empty = old faces kept their rows (new ones appended)
    std::size_t targetBefore = 0;      ///< The stage's own target (e.g. duplicate faces) before ...
    std::size_t targetAfter = 0;       ///< ... and after: a change that does not lower it is undone (no churn)
    double maxMove = 0.0;              ///< Farthest a vertex moved (straight line)
    std::string detail;                ///< Report line ("Removed 3 collapsed faces")
    std::string failure;               ///< Non-empty: the stage failed, nothing is kept
    std::string skipped;               ///< Non-empty: the stage did not run on this mesh (why)
    std::vector<FixAllDeferred> deferred; ///< Problems the change leaves for later steps (besides those the engine measures)
    std::vector<FixAllShellNote> notes;   ///< Shells the report names (stopped, kept, crossed ...)
    /// Input faces of shells the stage knows are not cavities and may reverse or delete (inside-out shells
    /// it reversed, floating debris it deleted): the cavity guard does not protect them.
    std::vector<Eigen::Index> notCavityFaces;
    bool cancelled = false;
};

/// One step of a pass: a plain function in a table.
struct FixAllStage {
    FixAllStageId id = FixAllStageId::Custom;
    std::string label;            ///< "Weld by distance"
    std::string activity;         ///< Progress text ("welding")
    double progressWeight = settings::kFixAllQuickStageProgressWeight;
    /// Why the stage does not run (turned off, needs whole shells); empty (or no function) = it runs.
    std::function<std::string(const FixAllParams&, const FixAllScope&)> skipReason;
    std::function<FixAllStageOutcome(const Mesh&, const FixAllStageContext&)> run;
    /// It moves vertices (weld, Close Crack, slivers, island patches): the dead-end guard applies. Hole
    /// fills are exempt (a hole is always filled).
    bool movesVertices = false;
};

/// Phase 1's stages in order: weld, drop repeated, dedupe, split touching, tiny shells, fins, orient,
/// close cracks, slivers, fill holes, islands, orient again.
[[nodiscard]] std::vector<FixAllStage> fixAllRepairStages();

/// The hard guard (or failure) that refused a step.
enum class FixAllGuard : std::uint8_t {
    None,
    Failed,      ///< The stage threw or reported a failure
    ValidResult, ///< Bad indices or face map
    NoChurn,     ///< It did not lower its own target
    Scope,       ///< It changed a face outside the scope
    Cavity,      ///< It would lose, open, join or shrink a shell wound inward (a cavity)
    DeadEnd      ///< It moved vertices into a crossing with a shell the merge cannot take
};

enum class FixAllStepStatus : std::uint8_t {
    Applied,   ///< Changed the mesh and was kept
    Unchanged, ///< Found nothing to do
    Refused,   ///< Undone: a guard failed, the stage failed or threw (see `reason`)
    Skipped    ///< Did not run (see `reason`)
};

/// One step of one pass, for the report.
struct FixAllStepReport {
    FixAllStageId stage = FixAllStageId::Custom;
    std::string label;
    int pass = 0;
    FixAllPhase phase = FixAllPhase::Repair;
    FixAllStepStatus status = FixAllStepStatus::Unchanged;
    FixAllGuard guard = FixAllGuard::None; ///< Refused: which guard
    std::string reason;                ///< Why refused or skipped
    std::string detail;                ///< What it did
    std::vector<FixAllDeferred> deferred; ///< Problems it left for a later step
    std::vector<FixAllShellNote> notes;   ///< Shells the step names
    std::size_t countBefore = 0;       ///< Its own target before / after
    std::size_t countAfter = 0;
    std::size_t facesAdded = 0;
    std::size_t facesRemoved = 0;
    double maxMove = 0.0;
    double elapsedMs = 0.0;

    [[nodiscard]] bool accepted() const noexcept {
        return status == FixAllStepStatus::Applied;
    }
};

/// One pass, judged at its end.
struct FixAllPassReport {
    int pass = 0;
    FixAllPriority before; ///< At the start of the pass (the previous pass's end)
    FixAllPriority after;  ///< At its end
    bool newBest = false;  ///< Better than every earlier state, the input included
    bool repeated = false; ///< Its end state was seen before
    std::size_t firstStep = 0; ///< Its steps: `FixAllResult::steps[firstStep .. firstStep + stepCount)`
    std::size_t stepCount = 0;
    double elapsedMs = 0.0;
};

enum class FixAllStopReason : std::uint8_t {
    Clean,         ///< Closed, manifold, oriented, crossing-free, no junk
    RepeatedState, ///< A pass ended in a state seen before (oscillation)
    NoProgress,    ///< Two passes in a row did not beat the best state
    PassLimit,     ///< The passes asked for are done
    TimeBudget,    ///< The time budget ran out
    Cancelled,
    InvalidInput   ///< Bad face indices or a scope that does not match the mesh
};

/// "clean", "repeated state", "no further progress", "pass limit", "time budget", "cancelled", "invalid input".
[[nodiscard]] const char* fixAllStopReasonLabel(FixAllStopReason reason) noexcept;

struct FixAllResult {
    bool success = false;    ///< False for invalid input or cancel (then `mesh` is the input)
    std::string error;
    Mesh mesh;               ///< The best pass's mesh (a full run) or the pass's mesh (One pass)
    FixAllScope scope;       ///< Carried to `mesh`
    std::vector<FixAllStepReport> steps;
    std::vector<FixAllPassReport> passes;
    int passesRun = 0;
    int bestPass = 0;        ///< Pass whose state is returned (0 = the input)
    FixAllStopReason stopReason = FixAllStopReason::PassLimit;
    bool cancelled = false;
    bool onePass = false;
    bool changed = false;            ///< `mesh` differs from the input
    bool temporarilyWorse = false;   ///< One pass only: the pass's score is worse than the input's
    FixAllCounts countsBefore;
    FixAllCounts countsAfter;        ///< Of `mesh`
    FixAllPriority priorityBefore;
    FixAllPriority priorityAfter;
    FixAllWeldPlan weldPlan;         ///< Worked out on the input's scope (step 0)
    double elapsedMs = 0.0;
};

/// Progress of a run for another thread: a fraction that never runs backwards and a line of text
/// ("pass 2 of 3, phase 1, welding...").
class FixAllProgress {
public:
    std::atomic<float> fraction{0.0f};
    /// Called on the running thread after every update (tests; may be empty).
    std::function<void(float, const std::string&)> observer;

    void update(float value, std::string text);
    [[nodiscard]] std::string text() const;

private:
    mutable std::mutex mutex_;
    std::string text_;
};

/// Test hooks: edit a phase's stage list before a pass runs it (remove, replace or insert stages).
struct FixAllTestHooks {
    std::function<void(int pass, FixAllPhase phase, std::vector<FixAllStage>& stages)> adjustStages;
};

/// Runs Fix All on `scope` of `mesh`. Never changes faces outside the scope. A cancelled run returns
/// the input with `cancelled` set.
[[nodiscard]] FixAllResult runFixAll(const Mesh& mesh, const FixAllScope& scope, const FixAllParams& params,
                                     std::stop_token stopToken = {}, FixAllProgress* progress = nullptr,
                                     const FixAllTestHooks* hooks = nullptr);

} // namespace ogun
