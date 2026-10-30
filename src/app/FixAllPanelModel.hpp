#pragma once

#include "app/AppController.hpp"
#include "repair/MergePlan.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ogun {

/// How a row of the Fix All plan can be switched.
enum class FixAllRowSwitch : std::uint8_t {
    Option,   ///< A box the user can tick or untick for this run (`FixAllParams`)
    Always,   ///< Always runs: ticked, no box to change
    Required  ///< Runs only after a merge (row 11): never a real box
};

/// One numbered row of the Fix All plan (Plan state), with its texts. Pure data for the panel.
struct FixAllPlanRow {
    int number = 0;     ///< 0 to 11
    int phase = 1;      ///< 1: repair each shell, 2: merge shells, 3: clean up the merged result
    std::string label;  ///< "Weld vertices within 1e-6 x coordinate"
    std::string found;  ///< The "found now" column: "14 edges", "3 + 12", "?", "checking...", "needs whole shells", or empty
    std::string tooltip;
    FixAllRowSwitch sw = FixAllRowSwitch::Always;
    bool on = true;           ///< The current tick
    bool available = true;    ///< False: greyed, the run skips it (a face scope has no whole shells)
    std::optional<DefectCategory> category; ///< The Mesh Health row whose count is "found now"
    /// Shown as plain text beside a row with no box to tick: "always runs", "runs after a merge", "needs whole
    /// shells". Empty for a row with a box.
    std::string note;
    /// The row has a box the user can change (an `Option` row that is available). Any other row is drawn as text.
    [[nodiscard]] bool hasBox() const noexcept {
        return sw == FixAllRowSwitch::Option && available;
    }
};

/// The 12 rows for the controller's current mesh, scope, plan-time check and Fix All settings.
[[nodiscard]] std::vector<FixAllPlanRow> fixAllPlanRows(AppController& controller);

/// Shells in scope after the planned merge: each group counts as one, each deleted nested solid as none.
[[nodiscard]] std::size_t shellsAfterPlannedMerge(const MergePlan& plan);

/// Shell count of the scope before the planned merge.
[[nodiscard]] std::size_t shellsInPlanScope(const MergePlan& plan);

/// "28 -> 1 shell" (the row 10 count), "nothing to merge", or "-" when the plan has nothing to say.
[[nodiscard]] std::string mergeEstimateText(const MergePlan& plan);

/// The orange note under row 0 when the weld tolerance is large next to the model, else empty.
[[nodiscard]] std::string weldNoteText(const FixAllWeldPlan& weld, Unit unit);

// ---------------------------------------------------------------------------------------------
// Running, Result and Report states
// ---------------------------------------------------------------------------------------------

/// Colour of a step's status dot: green done, orange done with something left or refused, grey skipped.
enum class FixAllDot : std::uint8_t { Green, Orange, Grey };

/// One step of one pass, as the Running / Result lists show it.
struct FixAllStepLine {
    std::size_t index = 0;  ///< Into `FixAllResult::steps`
    int number = 0;         ///< The plan row it belongs to (0 to 11)
    FixAllPhase phase = FixAllPhase::Repair;
    FixAllDot dot = FixAllDot::Green;
    std::string text;       ///< "Welded 14 vertices (max move 0.0002 mm)", "Heal thin faces: refused (...)"
    std::string time;       ///< "6 ms", "1.2 s"
    std::vector<std::string> details; ///< Shown when the line is expanded: reasons, deferred problems, shells named
};

/// One pass: its header and its steps (phase order).
struct FixAllPassLines {
    int pass = 0;
    std::string header;     ///< "Pass 1: best so far, 2.9 s", "Pass 3: worse than pass 2, discarded, 1.1 s"
    bool discarded = false;
    std::vector<FixAllStepLine> steps;
};

/// What a report line lets the user do.
enum class FixAllLineAction : std::uint8_t {
    None,
    ShowThem,      ///< Select the named shells
    ShowCavities,  ///< Select the cavities kept
    FlipGroup,     ///< Inside-out shell with shells inside: *Flip all* / *Flip outer only*
    FlipIt,        ///< One inside-out shell: *Flip it*
    ShowRemoved    ///< Deleted / merged faces: *Show removed* (before Apply only)
};

/// What the lines of a group stand for: one crossing shell pair each, or one shell each.
enum class FixAllLineGroupKind : std::uint8_t { ShellPairs, Shells };

/// One line of the Result / Report state.
struct FixAllResultLine {
    std::string text;
    bool warning = false;           ///< Left for the user: orange
    std::vector<int> shells;        ///< `report.shells` indices the line names (outer shell first for FlipGroup)
    FixAllLineAction action = FixAllLineAction::None;
    /// `report.shells` index of a shell Fix All left that alpha wrap is suggested for: the line gets the
    /// *Rebuild shell #k...* button (before Apply). -1: no button.
    int rebuildShell = -1;
    /// The reason shared by the lines of one repeated situation ("their group reaches into cavity #28 of #12, so it
    /// is not merged ..."), never their full text; `fixAllLineGroups` folds lines with the same kind and key into
    /// one summary. Empty: never grouped.
    std::string groupKey;
    FixAllLineGroupKind groupKind = FixAllLineGroupKind::ShellPairs;
};

/// A repeated situation: `members` index into the lines, `summary` is the one line that stands for them.
struct FixAllLineGroup {
    /// "100 shell pairs among 27 shells still cross: their group reaches into cavity #28 of #12, ..." or
    /// "27 shells still cross other shells: their group reaches ..."
    std::string summary;
    FixAllLineGroupKind kind = FixAllLineGroupKind::ShellPairs;
    std::vector<std::size_t> members;
    std::vector<int> shells;       ///< Every shell the members name, each once
};

/// Groups of at least `kFixAllMinGroup` lines sharing a `groupKind` and `groupKey`, in order of first appearance.
inline constexpr std::size_t kFixAllMinGroup = 3;
[[nodiscard]] std::vector<FixAllLineGroup> fixAllLineGroups(const std::vector<FixAllResultLine>& lines);

/// The plan row number of a stage (0..11; phase 3 steps are row 11).
[[nodiscard]] int fixAllStepNumber(FixAllStageId stage, FixAllPhase phase) noexcept;

/// "6 ms", "1.2 s", "1 min 5 s".
[[nodiscard]] std::string fixAllTimeText(double milliseconds);

/// The step lists of the shown run, one entry per pass. Empty without a run.
[[nodiscard]] std::vector<FixAllPassLines> fixAllPassLines(const AppController& controller, const FixAllResult& result);

/// "2 passes, 18 steps done, 2 refused".
[[nodiscard]] std::string fixAllStepsSummary(const FixAllResult& result);

/// The report in reading order with the action each line offers. `beforeApply` adds the *Show removed* line
/// for the merge's removed faces (they exist only in the original). Each crossing shell pair is said once: a
/// shell's "#1 still crosses #2: ..." line is left out when every pair it names has its own counted line.
/// Repeated situations carry a `groupKey` (the reason) for `fixAllLineGroups`.
[[nodiscard]] std::vector<FixAllResultLine> fixAllResultLines(const FixAllResult& result, bool beforeApply);

/// "pass 2 of 3, phase 1, filling holes..." -> {"Pass 2 of 3", "Phase 1: repair each shell", "filling holes"}.
/// Anything else comes back whole in `.detail` with empty `pass` and `phase`.
struct FixAllRunningText {
    std::string pass;
    std::string phase;
    std::string detail;
};
[[nodiscard]] FixAllRunningText fixAllRunningText(const std::string& progressText);

// ---------------------------------------------------------------------------------------------
// Rebuild side, rebuild rows and the *Rebuild shell #k...* button
// ---------------------------------------------------------------------------------------------

/// The button on a residue line: "Rebuild shell #12..." for `report.shells` index 11.
[[nodiscard]] std::string fixAllRebuildButtonLabel(int reportShell);

/// The table label of a rebuild row ("+ Alpha wrap #12", with "  (suggested)" on the suggested one), or the
/// method's name for any other row.
[[nodiscard]] std::string fixAllRebuildRowLabel(const RepairCandidate& candidate);

/// "Shell #12 rebuilt by alpha wrap and merged." / "... rebuilt by self-union (still open: not merged)."
/// for the Report after a rebuild row was applied.
[[nodiscard]] std::string fixAllRebuiltLine(const FixAllShellRebuild& rebuild);

/// The plain-language explanation shown beside *Rebuild shell #k...*: what alpha wrap is, and what it costs.
[[nodiscard]] std::string fixAllRebuildTradeOffText();

/// The Rebuild side's explanation with the live numbers (`offsetText`, `alphaText`: lengths with their unit).
struct FixAllRebuildCopy {
    std::string headline;               ///< "Replaces the shells in scope with a new surface."
    std::string guarantee;              ///< What is always true of the result
    std::vector<std::string> costs;     ///< The trade-offs, one bullet each
    std::string advice;                 ///< When to use Rebuild, when Repair
};
[[nodiscard]] FixAllRebuildCopy fixAllRebuildCopy(const std::string& offsetText, const std::string& alphaText);

/// "Closed cavity #2 will be filled in." / "Closed cavities #2 and #5 will be filled in." from
/// 0-based shell indices; empty without cavities.
[[nodiscard]] std::string fixAllRebuildCavityText(const std::vector<int>& cavities);

/// "Phase 1: repair each shell" and so on (1 to 3).
[[nodiscard]] const char* fixAllPhaseCaption(int phase) noexcept;

/// "1e-6" style number for labels.
[[nodiscard]] std::string shortNumber(double value);

/// Splits `text` into lines no wider than `width` (as `measure` says), breaking only at spaces, never inside a
/// word or a number ("0.1"), and never next to a word without a letter or digit ("%)", ":", "(<"), so
/// "(< 0.1 %)" and "100 : 1" stay together. A word wider than `width` gets a line of its own. Runs of spaces
/// inside a line are kept; a line break swallows them.
[[nodiscard]] std::vector<std::string> fixAllWrapLines(const std::string& text, float width,
                                                       const std::function<float(std::string_view)>& measure);

} // namespace ogun
