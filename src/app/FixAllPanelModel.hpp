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

/// "1e-6" style number for labels.
[[nodiscard]] std::string shortNumber(double value);

} // namespace ogun
