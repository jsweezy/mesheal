#include "app/FixAllPanelModel.hpp"

#include "mesh/Units.hpp"

#include <algorithm>
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
    return rows;
}

} // namespace ogun
