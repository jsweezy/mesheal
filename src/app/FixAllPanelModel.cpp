#include "app/FixAllPanelModel.hpp"

#include "mesh/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace ogun {

namespace {

std::string plural(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

/// The row's count when its Mesh Health result is fresh enough to quote, else nothing.
std::optional<std::size_t> knownCount(const AppController& controller, DefectCategory category) {
    const HealthStatus status = controller.categoryHealth(category).status;
    if (status != HealthStatus::Defects && status != HealthStatus::Ok) {
        return std::nullopt;
    }
    return controller.rowCount(category);
}

std::string sumText(std::optional<std::size_t> a, std::optional<std::size_t> b, bool plus) {
    if (!a || !b) {
        return "?";
    }
    return plus ? std::to_string(*a) + " + " + std::to_string(*b) : std::to_string(*a + *b);
}

} // namespace

std::string shortNumber(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.3g", value);
    std::string text = buffer;
    // "1e-06" -> "1e-6"
    const std::size_t e = text.find('e');
    if (e != std::string::npos) {
        std::size_t digits = e + 2; // after the exponent's sign
        while (digits + 1 < text.size() && text[digits] == '0') {
            text.erase(digits, 1);
        }
        if (text[e + 1] == '+') {
            text.erase(e + 1, 1);
        }
    }
    return text;
}

std::size_t shellsInPlanScope(const MergePlan& plan) {
    return static_cast<std::size_t>(std::count(plan.shellInScope.begin(), plan.shellInScope.end(), std::uint8_t{1}));
}

std::size_t shellsAfterPlannedMerge(const MergePlan& plan) {
    const std::size_t before = shellsInPlanScope(plan);
    std::size_t removed = plan.isolatedNestedSolids.size();
    for (const MergePlanGroup& group : plan.groups) {
        removed += group.shells.empty() ? 0 : group.shells.size() - 1;
    }
    return before > removed ? before - removed : 0;
}

std::string mergeEstimateText(const MergePlan& plan) {
    if (plan.needsWholeShells) {
        return "needs whole shells";
    }
    if (!plan.hasWork()) {
        return "nothing to merge";
    }
    const std::size_t after = shellsAfterPlannedMerge(plan);
    return std::to_string(shellsInPlanScope(plan)) + " -> " + plural(after, "shell", "shells");
}

std::string weldNoteText(const FixAllWeldPlan& weld, Unit unit) {
    if (!weld.noteNeeded) {
        return {};
    }
    char buffer[220];
    std::snprintf(buffer, sizeof(buffer),
                  "The largest weld tolerance is %.3g %s, %.3g %% of the model's size: vertices that far apart are joined. "
                  "Lower it in Options if thin features matter.",
                  weld.maxTolerance, std::string(unitSymbol(unit)).c_str(), 100.0 * weld.diagonalShare());
    return buffer;
}

std::vector<FixAllPlanRow> fixAllPlanRows(AppController& controller) {
    const FixAllParams& p = controller.fixAllParams();
    const FixAllScopeInfo scope = controller.fixAllScope();
    const bool faceScope = scope.kind == FixAllScopeKind::Faces;
    const MergePlan* merge = controller.fixAllMergePlan();
    const FixAllWeldPlan* weld = controller.fixAllWeldPlan();
    const bool checking = controller.fixAllPlanChecking() || (merge == nullptr && weld == nullptr);
    const std::string unit(unitSymbol(controller.fileUnit()));
    const char* const wholeShells = "needs whole shells";

    std::vector<FixAllPlanRow> rows(12);
    for (int i = 0; i < 12; ++i) {
        rows[static_cast<std::size_t>(i)].number = i;
    }
    const auto set = [&](int n, int phase, std::string label, FixAllRowSwitch sw, bool on, std::string tip) -> FixAllPlanRow& {
        FixAllPlanRow& row = rows[static_cast<std::size_t>(n)];
        row.phase = phase;
        row.label = std::move(label);
        row.sw = sw;
        row.on = on;
        row.tooltip = std::move(tip);
        return row;
    };

    // Phase 1
    {
        FixAllPlanRow& r = set(0, 1, "Weld vertices within " + shortNumber(p.weldTolerance.relTol) + " x coordinate",
                               FixAllRowSwitch::Option, p.weld,
                               "Joins vertices that differ by at most " + shortNumber(p.weldTolerance.relTol) +
                                   " of their coordinate on every axis, and always at least " + shortNumber(p.weldTolerance.absFloor) +
                                   " " + unit + ". It closes hairline cracks and removes edges too short to matter, and never presses "
                                   "touching surfaces together. This is separate from the weld done when the file was loaded, which only "
                                   "joins duplicate corners.");
        r.found = weld == nullptr ? "checking..." : (weld->collapsingEdges == 0 ? "0" : plural(weld->collapsingEdges, "edge", "edges"));
    }
    {
        FixAllPlanRow& r = set(1, 1, "Remove collapsed and duplicate faces", FixAllRowSwitch::Always, true,
                               "Deletes faces that use the same vertex twice (collapsed) and faces that repeat another face. "
                               "The two numbers are the Degenerate faces and Duplicate faces rows of Mesh Health. Always runs: "
                               "later steps need clean faces.");
        r.category = DefectCategory::Degenerate;
        r.found = sumText(knownCount(controller, DefectCategory::Degenerate), knownCount(controller, DefectCategory::Duplicates), true);
    }
    {
        FixAllPlanRow& r = set(2, 1, "Separate touching solids", FixAllRowSwitch::Always, true,
                               "Splits edges and vertices where two parts touch, so each part becomes its own closed surface. "
                               "The geometry does not change. The number is the non-manifold edges plus vertices of Mesh Health. "
                               "Always runs: the orientation and merge steps need clean contacts.");
        r.category = DefectCategory::NonManifoldEdges;
        r.found = sumText(knownCount(controller, DefectCategory::NonManifoldEdges),
                          knownCount(controller, DefectCategory::NonManifoldVertices), false);
    }
    {
        FixAllPlanRow& r = set(3, 1, "Delete tiny floating shells (< " + shortNumber(p.tinyShellFraction * 100.0) + " %)",
                               FixAllRowSwitch::Option, p.removeTinyShells,
                               "Deletes whole shells smaller than " + shortNumber(p.tinyShellFraction * 100.0) +
                                   " % of the model's size that touch nothing: dust and stray specks. The Result lists every "
                                   "deletion. Needs whole shells.");
        r.available = !faceScope;
        r.found = faceScope ? wholeShells : "";
    }
    set(4, 1, "Remove fins and internal walls", FixAllRowSwitch::Option, p.removeFins,
        "Deletes faces where three or more faces meet at one edge (a fin or a wall inside a solid). Removing a wall can "
        "open a seam; row 8 fills it.");
    {
        FixAllPlanRow& r = set(5, 1, "Fix normals (cavities stay inward)", FixAllRowSwitch::Always, true,
                               "Turns faces that point against their neighbours around, and reverses shells that are inside "
                               "out. A hollow cavity inside another shell keeps pointing inward. Shells that look inside out "
                               "together with a shell inside them are left for you to flip. Always runs.");
        r.category = DefectCategory::FlippedNormals;
        const std::optional<std::size_t> flipped = knownCount(controller, DefectCategory::FlippedNormals);
        const std::optional<std::size_t> inverted = knownCount(controller, DefectCategory::InvertedShells);
        if (!flipped || !inverted) {
            r.found = "?";
        } else {
            r.found = plural(*flipped, "face", "faces") + (*inverted > 0 ? ", " + plural(*inverted, "shell", "shells") : std::string());
        }
    }
    set(6, 1, "Close cracks", FixAllRowSwitch::Option, p.closeCracks,
        "Closes holes that are really hairline cracks by moving their two sides together by at most a few gap widths.");
    {
        FixAllPlanRow& r = set(7, 1, "Heal thin faces (stretched over " + shortNumber(controller.thresholds().maxAspectRatio) + " : 1)",
                               FixAllRowSwitch::Option, p.healThinFaces,
                               "Flips or collapses the long, thin triangles of the Thin faces row, keeping vertices that lie "
                               "on a boundary of the scope in place. May open a seam; row 8 fills it.");
        r.category = DefectCategory::ThinFaces;
        const std::optional<std::size_t> n = knownCount(controller, DefectCategory::ThinFaces);
        r.found = n ? std::to_string(*n) : "?";
    }
    {
        FixAllPlanRow& r = set(8, 1, "Fill holes", FixAllRowSwitch::Always, true,
                               "Every hole that can be filled is filled. If a patch crosses its own shell, row 9 repairs it; "
                               "if it pokes into another shell, row 10 merges the two. Crossings still left at the end are listed.");
        r.category = DefectCategory::OpenEdges;
        const bool known = knownCount(controller, DefectCategory::OpenEdges).has_value();
        r.found = known ? plural(controller.holeCountInScope(), "hole", "holes") : "?";
    }
    {
        FixAllPlanRow& r = set(9, 1, "Repair intersecting islands", FixAllRowSwitch::Always, true,
                               "Where faces of one shell cross each other, cuts out the crossing patch and fills it again. "
                               "Crossings between different shells are left to row 10.");
        r.category = DefectCategory::Intersections;
        const bool known = knownCount(controller, DefectCategory::Intersections).has_value();
        r.found = known ? plural(controller.intersectionIslandCount(), "island", "islands") : "?";
    }

    // Phase 2 and 3
    {
        FixAllPlanRow& r = set(10, 2, "Merge shells that overlap", FixAllRowSwitch::Option, p.mergeShells,
                               "Unites shells that cross or overlap into one solid. What it destroys: the faces inside other "
                               "shells, a nested solid that touches nothing (deleted), and the separation between parts that may "
                               "be different materials. What it keeps: cavities are always kept. Groups that reach into a cavity, "
                               "inside-out bodies with shells inside, and shells that touch only along an edge are left for you. "
                               "Open shells are skipped and listed.");
        r.available = !faceScope;
        if (faceScope) {
            r.found = wholeShells;
        } else if (merge == nullptr || checking) {
            r.found = "checking...";
        } else {
            r.found = mergeEstimateText(*merge);
        }
    }
    {
        FixAllPlanRow& r = set(11, 3, "Split contacts, fix normals, heal cut slivers, fill holes, repair islands",
                               FixAllRowSwitch::Required, p.mergeShells && !faceScope,
                               "Required after a merge: the union adds thin faces along every cut. Shells it closes or splits off "
                               "are merged by the next pass.");
        r.found = p.mergeShells && !faceScope ? "(required after 10)" : "(skipped: runs only after a merge)";
    }
    for (FixAllPlanRow& row : rows) {
        if (row.sw == FixAllRowSwitch::Always) {
            row.note = "always runs";
        } else if (row.sw == FixAllRowSwitch::Required) {
            row.note = "runs after a merge";
        } else if (!row.available) {
            row.note = "needs whole shells";
        }
    }
    return rows;
}

const char* fixAllPhaseCaption(int phase) noexcept {
    switch (phase) {
    case 1: return "Phase 1: repair each shell";
    case 2: return "Phase 2: merge shells";
    case 3: return "Phase 3: clean up the merged result";
    default: return "";
    }
}

int fixAllStepNumber(FixAllStageId stage, FixAllPhase phase) noexcept {
    if (phase == FixAllPhase::Cleanup) {
        return 11;
    }
    switch (stage) {
    case FixAllStageId::Weld: return 0;
    case FixAllStageId::DropRepeated:
    case FixAllStageId::Dedupe: return 1;
    case FixAllStageId::SplitTouching: return 2;
    case FixAllStageId::TinyShells: return 3;
    case FixAllStageId::Fins: return 4;
    case FixAllStageId::Orient:
    case FixAllStageId::OrientAgain: return 5;
    case FixAllStageId::CloseCracks: return 6;
    case FixAllStageId::Slivers: return 7;
    case FixAllStageId::FillHoles: return 8;
    case FixAllStageId::Islands: return 9;
    case FixAllStageId::PlanMerge:
    case FixAllStageId::DeleteNestedSolids:
    case FixAllStageId::MergeShells: return 10;
    case FixAllStageId::Custom: break;
    }
    return phase == FixAllPhase::Merge ? 10 : 0;
}

std::string fixAllTimeText(double milliseconds) {
    char buffer[40];
    if (milliseconds < 1000.0) {
        std::snprintf(buffer, sizeof(buffer), "%.0f ms", milliseconds);
    } else if (milliseconds < 60000.0) {
        std::snprintf(buffer, sizeof(buffer), "%.1f s", milliseconds / 1000.0);
    } else {
        const int seconds = static_cast<int>(std::lround(milliseconds / 1000.0));
        std::snprintf(buffer, sizeof(buffer), "%d min %d s", seconds / 60, seconds % 60);
    }
    return buffer;
}

namespace {

FixAllStepLine stepLine(const AppController& controller, const FixAllResult& result, std::size_t index) {
    const FixAllStepReport& step = result.steps[index];
    FixAllStepLine line;
    line.index = index;
    line.phase = step.phase;
    line.number = fixAllStepNumber(step.stage, step.phase);
    line.time = step.status == FixAllStepStatus::Skipped ? std::string() : fixAllTimeText(step.elapsedMs);
    const bool warnNote = std::any_of(step.notes.begin(), step.notes.end(), [](const FixAllShellNote& n) { return n.warning; });
    switch (step.status) {
    case FixAllStepStatus::Applied:
        line.dot = step.deferred.empty() && !warnNote ? FixAllDot::Green : FixAllDot::Orange;
        line.text = step.detail.empty() ? step.label : step.detail;
        break;
    case FixAllStepStatus::Unchanged:
        line.dot = FixAllDot::Grey;
        line.text = step.label + ": nothing to do";
        break;
    case FixAllStepStatus::Refused:
        line.dot = FixAllDot::Orange;
        line.text = step.label + ": undone" + (step.reason.empty() ? std::string() : " (" + step.reason + ")");
        break;
    case FixAllStepStatus::Skipped:
        line.dot = FixAllDot::Grey;
        line.text = step.label + ": skipped" + (step.reason.empty() ? std::string() : " (" + step.reason + ")");
        break;
    }
    if (step.status == FixAllStepStatus::Refused && !step.reason.empty()) {
        line.details.push_back("Why: " + step.reason);
    }
    for (const FixAllDeferred& d : step.deferred) {
        line.details.push_back(d.text);
    }
    for (const FixAllShellNote& note : step.notes) {
        line.details.push_back(controller.fixAllNoteText(step, note));
    }
    return line;
}

} // namespace

std::vector<FixAllPassLines> fixAllPassLines(const AppController& controller, const FixAllResult& result) {
    std::vector<FixAllPassLines> out;
    for (const FixAllPassReport& pass : result.passes) {
        FixAllPassLines lines;
        lines.pass = pass.pass;
        const bool kept = result.bestPass == pass.pass;
        lines.discarded = !result.onePass && pass.pass > result.bestPass;
        std::string outcome;
        if (result.onePass) {
            outcome = "one pass";
        } else if (kept && pass.newBest) {
            outcome = "best so far";
        } else if (kept) {
            outcome = "kept";
        } else if (pass.pass < result.bestPass) {
            outcome = pass.newBest ? "best so far" : "kept going";
        } else if (pass.repeated) {
            outcome = "back to an earlier state, discarded";
        } else if (result.bestPass > 0) {
            outcome = "no better than pass " + std::to_string(result.bestPass) + ", discarded";
        } else {
            outcome = "no better than the input, discarded";
        }
        if (pass.groupsMerged > 0) {
            outcome += "; merged " + plural(pass.groupsMerged, "group", "groups");
        }
        lines.header = "Pass " + std::to_string(pass.pass) + ": " + outcome + ", " + fixAllTimeText(pass.elapsedMs);
        const std::size_t end = std::min(result.steps.size(), pass.firstStep + pass.stepCount);
        for (std::size_t i = pass.firstStep; i < end; ++i) {
            lines.steps.push_back(stepLine(controller, result, i));
        }
        out.push_back(std::move(lines));
    }
    return out;
}

std::string fixAllStepsSummary(const FixAllResult& result) {
    std::size_t done = 0;
    std::size_t refused = 0;
    for (const FixAllStepReport& step : result.steps) {
        done += step.status == FixAllStepStatus::Applied ? 1 : 0;
        refused += step.status == FixAllStepStatus::Refused ? 1 : 0;
    }
    std::string text = plural(static_cast<std::size_t>(result.passesRun), "pass", "passes") + ", " + std::to_string(done) + " steps done";
    if (refused > 0) {
        text += ", " + std::to_string(refused) + " undone";
    }
    return text;
}

std::vector<FixAllResultLine> fixAllResultLines(const FixAllResult& result, bool beforeApply) {
    const FixAllFinalReport& report = result.report;
    std::vector<FixAllResultLine> lines;
    if (!report.complete) {
        return lines;
    }
    const auto add = [&](std::string text, bool warning, std::vector<int> shells, FixAllLineAction action) {
        lines.push_back({std::move(text), warning, std::move(shells), action});
    };
    add(report.passesLine, false, {}, FixAllLineAction::None);
    const bool unclean = !result.priorityAfter.clean();
    std::vector<int> leftIn;
    for (const FixAllShellResidue& r : report.residue) {
        if (r.suggestion != FixAllSuggestion::None) {
            leftIn.push_back(r.shell);
        }
    }
    add(report.leftLine, unclean, leftIn, leftIn.empty() ? FixAllLineAction::None : FixAllLineAction::ShowThem);
    if (!report.temporarilyWorseText.empty()) {
        add(report.temporarilyWorseText, true, {}, FixAllLineAction::None);
    }
    add(report.shellsLine, false, {}, FixAllLineAction::None);
    if (!report.cavitiesLine.empty()) {
        add(report.cavitiesLine, false, report.cavitiesKept, FixAllLineAction::ShowCavities);
    }
    add(report.closedLine, report.closedShells < report.shellsAfter, {}, FixAllLineAction::None);
    for (const FixAllReportLine& line : report.groupsNotMerged) {
        const bool flip = line.text.find("look inside out together") != std::string::npos && line.shells.size() >= 2;
        add(line.text, true, line.shells, flip ? FixAllLineAction::FlipGroup : FixAllLineAction::ShowThem);
    }
    // A shell shown as a cavity another group reaches into needs no second "cavity or inside out?" line
    const auto reachedCavity = [&](int shell) {
        return std::any_of(report.groupsNotMerged.begin(), report.groupsNotMerged.end(), [&](const FixAllReportLine& g) {
            return g.text.find("reaches into cavity") != std::string::npos &&
                   std::find(g.shells.begin(), g.shells.end(), shell) != g.shells.end();
        });
    };
    for (const FixAllShellResidue& r : report.residue) {
        if (r.suggestion == FixAllSuggestion::ReportOnly && r.reason.find("cavity or inside out?") != std::string::npos &&
            reachedCavity(r.shell)) {
            continue;
        }
        FixAllLineAction action = r.named.empty() ? FixAllLineAction::None : FixAllLineAction::ShowThem;
        std::vector<int> shells = r.named;
        if (r.reason.find("looks inside out together") != std::string::npos && r.named.size() >= 2) {
            action = FixAllLineAction::FlipGroup;
        } else if (r.reason.find("is still inside out") != std::string::npos) {
            action = FixAllLineAction::FlipIt;
            shells = {r.shell};
        }
        add(r.reason, r.suggestion != FixAllSuggestion::None, std::move(shells), action);
        if (r.suggestion == FixAllSuggestion::AlphaWrap) {
            lines.back().rebuildShell = r.shell;
        }
    }
    for (const FixAllCrossingLeft& left : report.crossingsLeft) {
        add(left.text, true, {left.a, left.b}, FixAllLineAction::ShowThem);
        if (left.fills.empty() && !left.whyNotMerged.empty()) {
            lines.back().groupKey = left.whyNotMerged;
        }
    }
    if (beforeApply && !result.mergeRemovedFaces.empty()) {
        add("The merge removed " + plural(result.mergeRemovedFaces.size(), "face", "faces") +
                " that lay inside other shells, or belonged to a nested solid.",
            false, {}, FixAllLineAction::ShowRemoved);
    }
    const bool manyPasses = result.passesRun > 1;
    for (const FixAllDeletion& deletion : report.deletions) {
        add(deletion.text + (manyPasses ? " (pass " + std::to_string(deletion.pass) + ")" : std::string()), false, {},
            beforeApply && !deletion.inputFaces.empty() ? FixAllLineAction::ShowRemoved : FixAllLineAction::None);
    }
    return lines;
}

std::vector<FixAllLineGroup> fixAllLineGroups(const std::vector<FixAllResultLine>& lines) {
    std::vector<FixAllLineGroup> groups;
    std::vector<std::string> keys;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].groupKey.empty()) {
            continue;
        }
        const auto it = std::find(keys.begin(), keys.end(), lines[i].groupKey);
        const std::size_t g = static_cast<std::size_t>(it - keys.begin());
        if (it == keys.end()) {
            keys.push_back(lines[i].groupKey);
            groups.emplace_back();
        }
        groups[g].members.push_back(i);
        for (const int s : lines[i].shells) {
            if (std::find(groups[g].shells.begin(), groups[g].shells.end(), s) == groups[g].shells.end()) {
                groups[g].shells.push_back(s);
            }
        }
    }
    std::vector<FixAllLineGroup> out;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (groups[g].members.size() >= kFixAllMinGroup) {
            groups[g].summary = std::to_string(groups[g].members.size()) + " shell pairs still cross: " + keys[g] + ".";
            out.push_back(std::move(groups[g]));
        }
    }
    return out;
}

FixAllRunningText fixAllRunningText(const std::string& progressText) {
    FixAllRunningText out;
    int pass = 0;
    int total = 0;
    int phase = 0;
    int used = 0;
    if (std::sscanf(progressText.c_str(), "pass %d of %d, phase %d, %n", &pass, &total, &phase, &used) >= 3 && used > 0) {
        out.pass = "Pass " + std::to_string(pass) + " of " + std::to_string(total);
        out.phase = fixAllPhaseCaption(phase);
        out.detail = progressText.substr(static_cast<std::size_t>(used));
        return out;
    }
    if (std::sscanf(progressText.c_str(), "pass %d of %d, %n", &pass, &total, &used) >= 2 && used > 0) {
        out.pass = "Pass " + std::to_string(pass) + " of " + std::to_string(total);
        out.detail = progressText.substr(static_cast<std::size_t>(used));
        return out;
    }
    out.detail = progressText;
    return out;
}

std::string fixAllRebuildButtonLabel(int reportShell) {
    return "Rebuild shell #" + std::to_string(reportShell + 1) + "...";
}

std::string fixAllRebuildRowLabel(const RepairCandidate& candidate) {
    if (candidate.fixAll != nullptr && candidate.fixAll->rebuild) {
        const FixAllShellRebuild& rebuild = *candidate.fixAll->rebuild;
        return rebuild.label + (rebuild.suggested ? "  (suggested)" : "");
    }
    return globalRepairMethodLabel(candidate.method);
}

std::string fixAllRebuiltLine(const FixAllShellRebuild& rebuild) {
    // The label is "+ Alpha wrap #12": the method is between "+ " and " #"
    std::string method = rebuild.label;
    if (method.starts_with("+ ")) {
        method.erase(0, 2);
    }
    const std::size_t hash = method.rfind(" #");
    if (hash != std::string::npos) {
        method.erase(hash);
    }
    if (!method.empty()) {
        method[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(method[0])));
    }
    const std::string head = "Shell #" + std::to_string(rebuild.shell + 1) + " rebuilt by " + method;
    if (rebuild.merged) {
        return head + " and merged.";
    }
    return head + (rebuild.note.empty() ? "." : " (" + rebuild.note + ").");
}

std::string fixAllRebuildTradeOffText() {
    return "Alpha wrap is another way to fix a shell: instead of repairing its faces, it wraps a new closed surface around it, "
           "which is always watertight and free of crossings. The price: the new surface lies a little outside the old one "
           "(the offset), gaps and pockets narrower than alpha are filled in, closed cavities inside the shell are filled in, "
           "and none of the shell's original faces are kept. Nothing is applied until you pick a row and click Apply; "
           "Cmd/Ctrl+Z undoes it in one step.";
}

FixAllRebuildCopy fixAllRebuildCopy(const std::string& offsetText, const std::string& alphaText) {
    FixAllRebuildCopy copy;
    copy.headline = "Replaces the shells in scope with a new surface.";
    copy.guarantee = "With alpha wrap the result is always watertight, manifold and free of crossings, but no original face "
                     "survives, and:";
    copy.costs = {
        "the surface moves outward by the offset (here " + offsetText + "),",
        "gaps and pockets narrower than alpha (" + alphaText + ") are filled,",
        "closed cavities inside a shell are filled in,",
        "an open sheet becomes a thin closed skin.",
    };
    copy.advice = "Use it for scans, badly broken exports, or large objects whose detail does not matter. Use Repair to keep "
                  "your original faces exactly.";
    return copy;
}

std::string fixAllRebuildCavityText(const std::vector<int>& cavities) {
    if (cavities.empty()) {
        return {};
    }
    if (cavities.size() == 1) {
        return "Closed cavity #" + std::to_string(cavities.front() + 1) + " will be filled in.";
    }
    std::string text = "Closed cavities ";
    for (std::size_t i = 0; i < cavities.size(); ++i) {
        if (i > 0) {
            text += i + 1 == cavities.size() ? " and " : ", ";
        }
        text += "#" + std::to_string(cavities[i] + 1);
    }
    return text + " will be filled in.";
}

} // namespace ogun
