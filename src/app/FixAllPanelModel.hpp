#pragma once

#include "app/AppController.hpp"
#include "repair/MergePlan.hpp"

#include <cstddef>
#include <optional>
#include <string>
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

/// One line of the Result / Report state.
struct FixAllResultLine {
    std::string text;
    bool warning = false;           ///< Left for the user: orange
    std::vector<int> shells;        ///< `report.shells` indices the line names (outer shell first for FlipGroup)
    FixAllLineAction action = FixAllLineAction::None;
};

/// The plan row number of a stage (0..11; phase 3 steps are row 11).
[[nodiscard]] int fixAllStepNumber(FixAllStageId stage, FixAllPhase phase) noexcept;

/// "6 ms", "1.2 s", "1 min 5 s".
[[nodiscard]] std::string fixAllTimeText(double milliseconds);

/// The step lists of the shown run, one entry per pass. Empty without a run.
[[nodiscard]] std::vector<FixAllPassLines> fixAllPassLines(const AppController& controller, const FixAllResult& result);

/// "2 passes, 18 steps done, 2 refused".
[[nodiscard]] std::string fixAllStepsSummary(const FixAllResult& result);

/// The report in reading order with the action each line offers. `beforeApply` adds the *Show removed* line
/// for the merge's removed faces (they exist only in the original).
[[nodiscard]] std::vector<FixAllResultLine> fixAllResultLines(const FixAllResult& result, bool beforeApply);

/// "pass 2 of 3, phase 1, filling holes..." -> {"Pass 2 of 3", "Phase 1: repair each shell", "filling holes"}.
/// Anything else comes back whole in `.detail` with empty `pass` and `phase`.
struct FixAllRunningText {
    std::string pass;
    std::string phase;
    std::string detail;
};
[[nodiscard]] FixAllRunningText fixAllRunningText(const std::string& progressText);

/// "Phase 1: repair each shell" and so on (1 to 3).
[[nodiscard]] const char* fixAllPhaseCaption(int phase) noexcept;

/// "1e-6" style number for labels.
[[nodiscard]] std::string shortNumber(double value);

} // namespace ogun
