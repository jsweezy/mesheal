#include "app/FixAllPanelModel.hpp"
#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"
#include "mesh/Units.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <string_view>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

constexpr float kFoundColumnWidth = 118.0f;

constexpr const char* kFixAllTooltipFull =
    "Repair every problem in the scope (the selection, or the whole mesh), repeating the sequence up to the number of "
    "passes you set (3 by default), with a preview and before / after numbers. Nothing changes until Apply.  Key: Shift+F5";

/// Text wrapped at the cell or panel edge, breaking only at spaces: never inside a number ("0." / "1 %", which
/// ImGui's own wrapping allows after a full stop) and never next to a sign such as "%)" or ":".
void wordWrapped(const std::string& text) {
    const float width = std::max(ImGui::GetContentRegionAvail().x, 1.0f);
    const auto measure = [](std::string_view part) { return ImGui::CalcTextSize(part.data(), part.data() + part.size()).x; };
    std::string out;
    std::size_t start = 0;
    bool first = true;
    while (start <= text.size()) {
        const std::size_t end = std::min(text.find('\n', start), text.size());
        for (const std::string& line : fixAllWrapLines(text.substr(start, end - start), width, measure)) {
            if (!first) {
                out += '\n';
            }
            first = false;
            out += line;
        }
        start = end + 1;
    }
    ImGui::PushTextWrapPos(-1.0f); // already broken: ImGui must not wrap it again
    ImGui::TextUnformatted(out.c_str(), out.c_str() + out.size());
    ImGui::PopTextWrapPos();
}

void wrappedColored(const ImVec4& color, const std::string& text) {
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    wordWrapped(text);
    ImGui::PopStyleColor();
}

/// Call before a button that follows another button: keeps it on the same line when it fits in the rest of
/// the row, else leaves the cursor on the next line. Sizes buttons to their text, never clips them.
void flowSameLine(const char* nextLabel) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float width = ImGui::CalcTextSize(nextLabel).x + 2.0f * style.FramePadding.x;
    const float right = ImGui::GetWindowPos().x - ImGui::GetScrollX() + ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + width <= right) {
        ImGui::SameLine();
    }
}

/// Muted text that wraps at the cell or panel edge (words are never split unless one is wider than the line).
void mutedWrapped(const std::string& text) {
    wrappedColored(gui::kMutedColor, text);
}

/// "0.006 mm" (the number is in file coordinates; the unit is a label).
std::string lengthWithUnit(double value, Unit unit) {
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.3g %s", value, std::string(unitSymbol(unit)).c_str());
    return buffer;
}

struct RebuildMethodOption {
    GlobalRepairMethod method;
    const char* summary;
    const char* help;
};

/// The Rebuild side's methods, the suggested one first.
constexpr std::array<RebuildMethodOption, 3> kRebuildMethodOptions = {{
    {GlobalRepairMethod::AlphaWrap, "always watertight; moves outward, fills small gaps",
     "Shrink-Wrap (Outer Envelope): a guaranteed watertight, 2-manifold outer casing around the geometry. Every "
     "face is new; pockets and cavities narrower than alpha are filled, and the surface moves outward by the offset. "
     "Best for external shells and zero-volume sheets."},
    {GlobalRepairMethod::VoxelHealRemesh, "uniform 3D grid + Taubin smoothing + isotropic remesh",
     "Voxel Heal & Remesh (Fast): evaluates the winding number on a 3D grid, extracts a watertight isosurface, applies Taubin "
     "de-terracing to remove voxel stair-stepping, and generates clean, uniform equilateral triangles. Preserves internal cavities."},
    {GlobalRepairMethod::PrecisionIsosurface, "exact level-set Delaunay meshing (>= 30 deg angles)",
     "Precision Isosurface (FEA Grade): meshes the exact winding-number level set with Delaunay refinement (guaranteed >= 30 deg "
     "angles) and tangential relaxation. Hugs tight curves and fillets without stair-stepping. Best for CAD and simulation."},
}};

} // namespace

void PolyscopeGui::openFixAllPanel(bool fullRun, bool rebuildSide) {
    if (!controller_.hasMesh()) {
        return;
    }
    fixAllRebuildSide_ = rebuildSide;
    fixAllCavityCache_.valid = false;
    controller_.openFixAll(fullRun ? controller_.preferenceDefaults().fixAll.passes : 1);
    requestSync();
}

// -----------------------------------------------------------------------------
// Mesh Health header: [Check Mesh] [Fix All...]  (One Pass is in the Repair menu)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawFixAllHeaderButtons() {
    ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
    if (ImGui::Button("Fix All...")) {
        openFixAllPanel(true);
    }
    ImGui::PopStyleColor();
    tooltip(kFixAllTooltipFull);
    if (controller_.fixAllRunning()) {
        ImGui::SameLine();
        gui::statusDot(HealthStatus::Running, false);
        tooltip("Fix All is running. Its progress and Cancel are in the Fix All panel and the status bar.");
    }
}

// -----------------------------------------------------------------------------
// Operation: Fix All
// -----------------------------------------------------------------------------

void PolyscopeGui::drawFixAllOperator() {
    if (!controller_.hasMesh()) {
        ImGui::TextDisabled("No mesh loaded.");
        return;
    }
    // The body scrolls in a child region; the action buttons below it stay visible. The footer's height is the one
    // measured on the previous frame. In a short panel the body keeps a usable height and the whole panel scrolls
    // down to the buttons instead.
    static float footerHeight = 60.0f;
    const FixAllPanelState state = controller_.fixAllState();
    const bool footer = state != FixAllPanelState::Running;
    if (footer) {
        const float panel = ImGui::GetContentRegionAvail().y + ImGui::GetScrollY(); // as if not scrolled
        const float minBody = std::max(settings::kFixAllBodyMinShare * panel,
                                       settings::kFixAllBodyMinLines * ImGui::GetTextLineHeightWithSpacing());
        const float body = std::max(panel - footerHeight, minBody);
        ImGui::BeginChild(("##fixAllBody" + std::to_string(static_cast<int>(state))).c_str(), ImVec2(0.0f, body));
    }
    ImGui::PushTextWrapPos(0.0f); // long lines wrap at the panel edge
    switch (state) {
    case FixAllPanelState::Plan:
        drawFixAllPlan();
        break;
    case FixAllPanelState::Running:
        drawFixAllRunning();
        break;
    case FixAllPanelState::Result:
        drawFixAllResult();
        break;
    case FixAllPanelState::Report:
        drawFixAllReport();
        break;
    }
    ImGui::PopTextWrapPos();
    if (footer) {
        ImGui::EndChild();
        const float top = ImGui::GetCursorPosY();
        ImGui::PushTextWrapPos(0.0f);
        drawFixAllFooter();
        ImGui::PopTextWrapPos();
        footerHeight = std::max(ImGui::GetCursorPosY() - top, 0.0f) + ImGui::GetStyle().ItemSpacing.y;
    }
}

void PolyscopeGui::drawFixAllFooter() {
    switch (controller_.fixAllState()) {
    case FixAllPanelState::Plan:
        if (fixAllRebuildSide_) {
            drawFixAllRebuildFooter();
        } else {
            drawFixAllPlanFooter();
        }
        break;
    case FixAllPanelState::Result:
        drawFixAllResultFooter();
        break;
    case FixAllPanelState::Report:
        drawFixAllReportFooter();
        break;
    case FixAllPanelState::Running:
        break;
    }
}

void PolyscopeGui::drawFixAllPlan() {
    if (!fixAllHintSeen_) {
        wrappedColored(gui::kInfoColor, "Preview first: nothing changes until you Apply.");
    }

    const FixAllScopeInfo scope = controller_.fixAllScope();
    ImGui::AlignTextToFramePadding();
    wrappedColored(gui::kInfoColor, "Scope: " + scope.text);
    tooltip("Fix All follows the selection: the selected faces, else the selected shells, else the whole mesh. Hidden shells "
            "are never changed. Everything outside the scope stays exactly as it is. Change the selection to change the scope.");

    drawFixAllSwitch();
    if (fixAllRebuildSide_) {
        drawFixAllRebuildPlan();
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    wordWrapped("Fixes each problem in place, in a safe order, and repeats the whole sequence until the mesh is "
                       "watertight and free of crossings. Faces that are not part of a problem stay exactly as they are.");
    ImGui::PopStyleColor();

    ImGui::SeparatorText("Each pass runs");
    drawFixAllPlanRows();

    // Passes
    ImGui::Spacing();
    int passes = controller_.fixAllParams().passes;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Passes");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputInt("##fixAllPasses", &passes, 1, 1)) {
        controller_.setFixAllPasses(passes);
    }
    tooltip("How many times the whole sequence may repeat (1 to 10). It stops earlier when the mesh is clean or a pass brings no "
            "progress. 1 = one pass. The saved starting value is in Preferences > Fix All.");
    ImGui::SameLine();
    ImGui::TextDisabled("1 = one pass");
    ImGui::TextDisabled("Repeats until clean, or until a pass brings no progress.");

    drawFixAllOptions();
    drawFixAllSettingsUsed();
}

void PolyscopeGui::drawFixAllPlanFooter() {
    const FixAllScopeInfo scope = controller_.fixAllScope();
    ImGui::Separator();
    const bool empty = scope.faces == 0;
    const int shownPasses = controller_.fixAllParams().passes;
    if (!empty) {
        ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
        if (ImGui::Button(shownPasses > 1 ? "Preview Fix All" : "Preview one pass")) {
            fixAllHintSeen_ = true;
            controller_.previewFixAll(shownPasses <= 1);
            requestSync();
        }
        ImGui::PopStyleColor();
        tooltip(shownPasses > 1 ? "Run Fix All with the passes above on a copy of the mesh, in the background, and show the result. "
                                  "Nothing changes until you Apply."
                                : "Run one pass on a copy of the mesh, in the background, and show the result. Nothing changes until "
                                  "you Apply.");
        if (shownPasses > 1) {
            flowSameLine("Preview one pass");
            if (ImGui::Button("Preview one pass")) {
                fixAllHintSeen_ = true;
                controller_.previewFixAll(true);
                requestSync();
            }
            tooltip("Run the sequence once, whatever Passes says, so you can see each step's effect before the next pass.");
        }
        if (scope.kind != FixAllScopeKind::Faces) {
            flowSameLine("Compare with rebuilds");
            if (ImGui::Button("Compare with rebuilds")) {
                fixAllHintSeen_ = true;
                controller_.setGlobalRepairMethod(GlobalRepairMethod::FixAll);
                controller_.compareAllGlobalRepairs();
                requestSync();
            }
            tooltip("Run Fix All and the three rebuilds (self-union, winding-number rebuild, alpha wrap) on the scope and list them "
                    "side by side, so you can see what each keeps and what it costs. Nothing changes until Apply.");
        }
        flowSameLine("Close");
    }
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close without changing the mesh.  Key: Esc");
    if (empty) {
        ImGui::TextDisabled("Nothing is in scope: every shell is hidden.");
    }
}

void PolyscopeGui::drawFixAllPlanRows() {
    const std::vector<FixAllPlanRow> rows = fixAllPlanRows(controller_);
    FixAllParams params = controller_.fixAllParams();
    bool changed = false;
    int phase = 0;
    static constexpr const char* kPhaseNames[] = {"", "Phase 1: repair each shell", "Phase 2: merge shells",
                                                 "Phase 3: clean up the merged result"};
    const FixAllWeldPlan* weld = controller_.fixAllWeldPlan();

    const ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit;
    for (const FixAllPlanRow& row : rows) {
        if (row.phase != phase) {
            phase = row.phase;
            ImGui::TextDisabled("%s", kPhaseNames[phase]);
            if (phase == 1 && ImGui::BeginTable("foundHeader", 2, flags)) {
                ImGui::TableSetupColumn("##step", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("##found", ImGuiTableColumnFlags_WidthFixed, kFoundColumnWidth);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("found now");
                tooltip("What Mesh Health finds now in the scope (? = not checked yet: press Check Mesh). It is only information: the "
                        "run does its own check.");
                ImGui::EndTable();
            }
        }
        ImGui::PushID(row.number);
        if (ImGui::BeginTable("row", 2, flags)) {
            ImGui::TableSetupColumn("##step", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("##found", ImGuiTableColumnFlags_WidthFixed, kFoundColumnWidth);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            const std::string number = std::to_string(row.number);
            std::string tip = row.tooltip;
            if (row.sw == FixAllRowSwitch::Always) {
                tip += "\n\nThis step always runs: it has no switch.";
            } else if (row.sw == FixAllRowSwitch::Required) {
                tip += "\n\nThis step has no box: it runs after a merge and never otherwise.";
            } else if (!row.available) {
                tip += "\n\nA selection of faces has no whole shells, so this step is off. Select shells, or clear the selection.";
            }
            if (row.hasBox()) {
                bool on = row.on;
                if (ImGui::Checkbox("##on", &on)) {
                    switch (row.number) {
                    case 0: params.weld = on; break;
                    case 3: params.removeTinyShells = on; break;
                    case 4: params.removeFins = on; break;
                    case 6: params.closeCracks = on; break;
                    case 7: params.healThinFaces = on; break;
                    case 10: controller_.setFixAllMergeShells(on); break;
                    default: break;
                    }
                    changed = changed || row.number != 10;
                }
                tooltip(tip.c_str());
            } else {
                // No box to change: plain text, never a greyed box
                ImGui::Dummy(ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight()));
            }
            ImGui::SameLine();
            wordWrapped(number + "  " + row.label);
            tooltip(tip.c_str());
            if (!row.hasBox() && row.sw != FixAllRowSwitch::Required && !row.note.empty() && row.note != row.found) {
                // On its own line, under the label: never squeezed into what is left of the label's line
                ImGui::Dummy(ImVec2(ImGui::GetFrameHeight(), 0.0f));
                ImGui::SameLine();
                mutedWrapped("(" + row.note + ")");
            }
            ImGui::TableSetColumnIndex(1);
            if (row.found == "checking...") {
                ImGui::TextColored(gui::kRunningColor, "checking...");
            } else if (row.found == "?" || row.found == "0" || row.found.empty() || !row.available || row.sw == FixAllRowSwitch::Required) {
                ImGui::TextDisabled("%s", row.found.c_str());
            } else {
                ImGui::TextUnformatted(row.found.c_str());
            }
            ImGui::EndTable();
        }
        if (row.number == 0 && weld != nullptr) {
            const std::string note = weldNoteText(*weld, controller_.fileUnit());
            if (!note.empty()) {
                ImGui::Indent();
                wrappedColored(gui::kDefectColor, note);
                ImGui::Unindent();
            }
        }
        if (row.number == 10) {
            drawFixAllMergeNotes();
        }
        ImGui::PopID();
    }
    if (changed) {
        controller_.setFixAllParams(params);
    }
}

void PolyscopeGui::drawFixAllMergeNotes() {
    const FixAllScopeInfo scope = controller_.fixAllScope();
    ImGui::Indent();
    if (scope.kind == FixAllScopeKind::Faces) {
        wordWrapped("Merge needs whole shells: select shells, or clear the selection. Faces around the selection stay "
                           "exactly as they are.");
        ImGui::Unindent();
        return;
    }
    const MergePlan* plan = controller_.fixAllMergePlan();
    if (plan == nullptr || controller_.fixAllPlanChecking()) {
        ImGui::TextColored(gui::kRunningColor, "Checking which shells overlap...");
        ImGui::Unindent();
        return;
    }
    const bool ticked = controller_.fixAllParams().mergeShells;
    if (plan->hasWork()) {
        if (ticked) {
            wordWrapped(controller_.fixAllMergeUserSet() ? "Ticked by you. Untick to keep the shells as separate parts."
                                                                : "Ticked because shells overlap. Untick to keep them as separate parts.");
        } else {
            wordWrapped("Not ticked: the shells stay separate parts. Tick it to merge those that overlap.");
        }
        ImGui::TextDisabled("Removes faces inside other shells; a nested solid touching nothing is deleted; cavities are kept.");
    } else if (ticked && plan->tickedByDefault) {
        wordWrapped("Ticked because shells overlap, but none of them can be merged (see below).");
    }
    for (std::size_t i = 0; i < plan->lines.size(); ++i) {
        const MergePlanLine& line = plan->lines[i];
        ImGui::PushID(static_cast<int>(i));
        if (line.kind == MergePlanLineKind::Warning) {
            wrappedColored(gui::kDefectColor, "(!) " + line.text);
        } else {
            wrappedColored(gui::kMutedColor, line.text);
        }
        if (!line.shells.empty()) {
            if (ImGui::SmallButton("Show them")) {
                controller_.showShells(line.shells);
                requestSync();
            }
            tooltip("Select these shells in the Shells panel and frame them.");
        }
        ImGui::PopID();
    }
    if (!plan->groups.empty()) {
        if (ImGui::SmallButton("Show groups")) {
            std::vector<int> all;
            for (const MergePlanGroup& group : plan->groups) {
                all.insert(all.end(), group.shells.begin(), group.shells.end());
            }
            controller_.showShells(all);
            requestSync();
        }
        tooltip("Select every shell that the merge would unite, so you can see them in the 3D view.");
    }
    ImGui::Unindent();
}

void PolyscopeGui::drawFixAllOptions() {
    if (!ImGui::TreeNode("Options (time budget, tiny-shell size, weld tolerance)")) {
        return;
    }
    FixAllParams params = controller_.fixAllParams();
    bool changed = false;
    const auto field = [&](const char* label, double& value, const char* help) {
        ImGui::SetNextItemWidth(110.0f);
        if (ImGui::InputDouble(label, &value, 0.0, 0.0, "%.6g", ImGuiInputTextFlags_EnterReturnsTrue) ||
            ImGui::IsItemDeactivatedAfterEdit()) {
            changed = true;
        }
        tooltip(help);
    };
    field("Stop after (seconds)", params.timeBudgetSeconds,
          "No new pass starts once a pass has ended past this many seconds. 0 = no limit. Press Enter to apply.");
    double percent = params.tinyShellFraction * 100.0;
    field("Tiny shell size (%)", percent,
          "Floating shells smaller than this share of the whole model's size are deleted (row 3). Press Enter to apply.");
    params.tinyShellFraction = percent / 100.0;
    field("Weld floor (file units)", params.weldTolerance.absFloor,
          "The weld tolerance is at least this many file units on every axis (1e-9 to 1e-3). It is separate from the weld done "
          "when the file was loaded. Press Enter to apply.");
    field("Weld relative tolerance", params.weldTolerance.relTol,
          "The weld tolerance is also this fraction of the coordinate (1e-9 to 1e-3; 0 is not allowed). It is separate from the "
          "weld done when the file was loaded. Press Enter to apply.");
    ImGui::TextDisabled("These apply to this run only. The starting values are in Preferences > Fix All.");
    if (changed) {
        controller_.setFixAllParams(params);
    }
    ImGui::TreePop();
}

void PolyscopeGui::drawFixAllSettingsUsed() {
    if (!ImGui::TreeNode("Settings used (thresholds of each row's tool)")) {
        return;
    }
    ImGui::TextDisabled("Fix All uses the same limits as the tools of the Mesh Health rows. Edit them there.");
    struct Link {
        DefectCategory category;
        std::string text;
        const char* help;
    };
    char buffer[160];
    std::vector<Link> links;
    std::snprintf(buffer, sizeof(buffer), "Thin faces: stretched over %s : 1", withThousands(static_cast<std::size_t>(controller_.thresholds().maxAspectRatio)).c_str());
    links.push_back({DefectCategory::ThinFaces, buffer, "Thin faces row > Settings."});
    std::snprintf(buffer, sizeof(buffer), "Crossings: tolerance %.3g of the model's size", controller_.intersectionTolerance());
    links.push_back({DefectCategory::Intersections, buffer, "Intersecting faces row > Settings."});
    const HoleFillSettings& holes = controller_.holeFillSettings();
    std::snprintf(buffer, sizeof(buffer), "Holes: skip holes with more than %s edges; patches up to %s faces",
                  holes.maxHoleEdges > 0 ? withThousands(static_cast<std::size_t>(holes.maxHoleEdges)).c_str() : "(no limit)",
                  withThousands(holes.maxPatchFaces).c_str());
    links.push_back({DefectCategory::OpenEdges, buffer, "Open edges row > Settings."});
    const IntersectionRepairSettings& islands = controller_.islandRepairSettings();
    std::snprintf(buffer, sizeof(buffer), "Islands: %d rings, %d CGAL steps", islands.dilationRings, islands.cgalMaxSteps);
    links.push_back({DefectCategory::Intersections, buffer, "Intersecting faces row > Settings."});
    std::optional<DefectCategory> open;
    for (std::size_t i = 0; i < links.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        wordWrapped(links[i].text);
        ImGui::SameLine();
        if (ImGui::SmallButton("Edit in tool")) {
            open = links[i].category;
        }
        tooltip((std::string("Opens the tool of this row: ") + links[i].help + " This panel closes; open Fix All again afterwards.").c_str());
        ImGui::PopID();
    }
    if (open) {
        controller_.closeTool();
        controller_.selectCategory(open);
        requestSync();
    }
    ImGui::TreePop();
}


// -----------------------------------------------------------------------------
// Running, Result and Report
// -----------------------------------------------------------------------------

namespace {

HealthStatus dotStatus(FixAllDot dot) {
    switch (dot) {
    case FixAllDot::Green: return HealthStatus::Ok;
    case FixAllDot::Orange: return HealthStatus::Defects;
    case FixAllDot::Grey: return HealthStatus::NotRun;
    }
    return HealthStatus::NotRun;
}

} // namespace

void PolyscopeGui::drawFixAllRunning() {
    const FixAllRunningText text = fixAllRunningText(controller_.fixAllStatusText());
    ImGui::AlignTextToFramePadding();
    gui::statusDot(HealthStatus::Running, false);
    ImGui::SameLine();
    if (controller_.fixAllCancelling()) {
        ImGui::TextColored(gui::kRunningColor, "Cancelling...");
    } else if (!text.pass.empty()) {
        ImGui::TextColored(gui::kRunningColor, "Running  -  %s", text.pass.c_str());
    } else {
        ImGui::TextColored(gui::kRunningColor, "Running...");
    }
    if (!text.phase.empty() && !controller_.fixAllCancelling()) {
        wordWrapped(text.phase);
    }
    if (!text.detail.empty()) {
        wrappedColored(gui::kMutedColor, text.detail);
    }
    ImGui::ProgressBar(controller_.fixAllProgress(), ImVec2(-1.0f, 0.0f));
    if (!controller_.fixAllCancelling()) {
        if (ImGui::Button("Cancel")) {
            controller_.cancelFixAll();
            requestSync();
        }
        tooltip("Stop and keep the mesh as it is. A merge in progress finishes first.");
    }
    ImGui::TextDisabled("Nothing changes until you Apply. The step list appears when the run is done.");
    ImGui::TextDisabled("Cancel takes effect when the current merge returns.");
}

void PolyscopeGui::drawFixAllSteps(const FixAllResult& result) {
    const std::vector<FixAllPassLines> passes = fixAllPassLines(controller_, result);
    if (passes.empty()) {
        return;
    }
    const std::string title = "Passes and steps (" + fixAllStepsSummary(result) + ")";
    if (!ImGui::TreeNode("##steps", "%s", title.c_str())) {
        return;
    }
    for (std::size_t p = 0; p < passes.size(); ++p) {
        const FixAllPassLines& pass = passes[p];
        ImGui::PushID(static_cast<int>(p));
        const ImGuiTreeNodeFlags open = (passes.size() == 1 || p + 1 == passes.size()) ? ImGuiTreeNodeFlags_DefaultOpen : 0;
        if (pass.discarded) {
            ImGui::PushStyleColor(ImGuiCol_Text, gui::kMutedColor);
        }
        const bool expanded = ImGui::TreeNodeEx("##pass", open, "%s", pass.header.c_str());
        if (pass.discarded) {
            ImGui::PopStyleColor();
        }
        tooltip("Click to fold or unfold the steps of this pass.");
        if (expanded) {
            int phase = 0;
            for (const FixAllStepLine& step : pass.steps) {
                if (static_cast<int>(step.phase) != phase) {
                    phase = static_cast<int>(step.phase);
                    ImGui::TextDisabled("%s", fixAllPhaseCaption(phase));
                }
                ImGui::PushID(static_cast<int>(step.index));
                const ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                                 (step.details.empty() ? ImGuiTreeNodeFlags_Leaf : 0);
                const bool detailsOpen = ImGui::TreeNodeEx("##step", flags | ImGuiTreeNodeFlags_NoTreePushOnOpen);
                ImGui::SameLine();
                gui::statusDot(dotStatus(step.dot), false);
                ImGui::SameLine();
                wordWrapped(std::to_string(step.number) + "  " + step.text);
                if (!step.time.empty()) {
                    ImGui::SameLine();
                    ImGui::TextColored(gui::kMutedColor, "%s", step.time.c_str());
                }
                if (detailsOpen) {
                    ImGui::Indent();
                    for (const std::string& detail : step.details) {
                        wrappedColored(gui::kMutedColor, detail);
                    }
                    ImGui::Unindent();
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
        }
        if (expanded) {
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    ImGui::TreePop();
}

void PolyscopeGui::drawFixAllLines(const FixAllResult& result, bool beforeApply) {
    const std::vector<FixAllResultLine> lines = fixAllResultLines(result, beforeApply);
    const std::vector<FixAllLineGroup> groups = fixAllLineGroups(lines);
    const auto groupOf = [&](std::size_t i) -> const FixAllLineGroup* {
        for (const FixAllLineGroup& g : groups) {
            if (std::find(g.members.begin(), g.members.end(), i) != g.members.end()) {
                return &g;
            }
        }
        return nullptr;
    };
    std::vector<const FixAllLineGroup*> drawnGroups;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const FixAllResultLine& line = lines[i];
        const FixAllLineGroup* group = groupOf(i);
        if (group != nullptr && std::find(drawnGroups.begin(), drawnGroups.end(), group) != drawnGroups.end()) {
            continue; // drawn inside its group's list
        }
        if (group != nullptr) {
            // One summary line with a collapsed list of the individual pairs
            drawnGroups.push_back(group);
            ImGui::PushID(static_cast<int>(i));
            wrappedColored(gui::kDefectColor, "(!) " + group->summary);
            if (!beforeApply && ImGui::SmallButton("Show them all")) {
                controller_.fixAllShowShells(group->shells);
                requestSync();
            }
            if (!beforeApply) {
                tooltip("Select every shell named in these pairs in the Shells panel and frame them.");
            }
            if (ImGui::TreeNode("##pairs", "Show the %zu pairs", group->members.size())) {
                for (const std::size_t m : group->members) {
                    ImGui::PushID(static_cast<int>(m));
                    wrappedColored(gui::kDefectColor, lines[m].text);
                    if (!beforeApply && ImGui::SmallButton("Show them")) {
                        controller_.fixAllShowShells(lines[m].shells);
                        requestSync();
                    }
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
            continue;
        }
        ImGui::PushID(static_cast<int>(i));
        if (line.warning) {
            wrappedColored(gui::kDefectColor, "(!) " + line.text);
        } else {
            wordWrapped(line.text);
        }
        if (beforeApply && line.rebuildShell >= 0 && !controller_.fixAllRebuildRunning() && !controller_.globalRepairRunning()) {
            if (ImGui::SmallButton(fixAllRebuildButtonLabel(line.rebuildShell).c_str())) {
                controller_.rebuildFixAllShell(line.rebuildShell);
                requestSync();
            }
            tooltip((fixAllRebuildTradeOffText() + "\n\nThis computes the rebuilds of this shell only (alpha wrap, self-union, "
                     "winding-number rebuild) and adds them as rows below. The rest of the Fix All result is kept exactly.").c_str());
        }
        // Selecting or flipping shells changes the mesh's selection, which would drop an unapplied preview
        const bool shellButtons = !beforeApply;
        switch (line.action) {
        case FixAllLineAction::ShowThem:
            if (shellButtons && ImGui::SmallButton("Show them")) {
                controller_.fixAllShowShells(line.shells);
                requestSync();
            }
            if (shellButtons) {
                tooltip("Select these shells in the Shells panel and frame them.");
            }
            break;
        case FixAllLineAction::ShowCavities:
            if (shellButtons && ImGui::SmallButton("Show")) {
                controller_.fixAllShowShells(line.shells);
                requestSync();
            }
            if (shellButtons) {
                tooltip("Select the cavities that were kept: hollow shells inside a solid, wound inward on purpose.");
            }
            break;
        case FixAllLineAction::FlipGroup:
            if (shellButtons) {
                if (ImGui::SmallButton("Show them")) {
                    controller_.fixAllShowShells(line.shells);
                    requestSync();
                }
                tooltip("Select these shells in the Shells panel and frame them.");
                flowSameLine("Flip all");
                if (ImGui::SmallButton("Flip all")) {
                    controller_.fixAllFlipShells(line.shells);
                    requestSync(true);
                }
                tooltip("Reverse the outer shell and the shells inside it (one undo step). Use it when the whole group is inside out.");
                flowSameLine("Flip outer only");
                if (ImGui::SmallButton("Flip outer only")) {
                    const std::array<int, 1> outer{line.shells.front()};
                    controller_.fixAllFlipShells(outer);
                    requestSync(true);
                }
                tooltip("Reverse only the outer shell (one undo step). Use it when the shells inside are already right.");
            }
            break;
        case FixAllLineAction::FlipIt:
            if (shellButtons) {
                if (ImGui::SmallButton("Show it")) {
                    controller_.fixAllShowShells(line.shells);
                    requestSync();
                }
                tooltip("Select this shell in the Shells panel and frame it.");
                flowSameLine("Flip it");
                if (ImGui::SmallButton("Flip it")) {
                    controller_.fixAllFlipShells(line.shells);
                    requestSync(true);
                }
                tooltip("Reverse this shell (one undo step).");
            }
            break;
        case FixAllLineAction::ShowRemoved:
            if (ImGui::SmallButton("Show removed")) {
                controller_.setGlobalPreviewShow(GlobalPreviewShow::Both);
                requestSync();
            }
            tooltip("Show the original with the removed faces ghosted behind the result (the Both view below). Original shows only "
                    "the original.");
            break;
        case FixAllLineAction::None:
            break;
        }
        ImGui::PopID();
    }
}

void PolyscopeGui::drawFixAllResult() {
    const RepairCandidate* candidate = controller_.fixAllCandidate();
    if (candidate == nullptr || !candidate->success || candidate->fixAll == nullptr) {
        if (candidate != nullptr) {
            ImGui::TextColored(gui::kErrorColor, "Fix All failed: %s", candidate->statusMessage.c_str());
            ImGui::TextDisabled("Nothing was changed.");
        } else {
            ImGui::TextDisabled("No result.");
        }
        return; // its Back to plan / Close are in the footer
    }
    const FixAllResult& result = candidate->fixAll->result;
    const RepairCandidate* shown = controller_.shownGlobalCandidate();
    const bool rebuildRowShown = shown != nullptr && shown->fixAll != nullptr && shown->fixAll->rebuild.has_value();

    ImGui::TextColored(gui::kInfoColor, "Preview ready. Nothing has changed yet.");
    drawGlobalCandidateTable();
    drawFixAllShellRebuildRows();
    drawFixAllRebuildJob();
    ImGui::Spacing();
    if (rebuildRowShown) {
        // The row's own report describes the rebuilt mesh; the steps are those of the Fix All run it builds on
        const FixAllShellRebuild& rebuild = *shown->fixAll->rebuild;
        wrappedColored(rebuild.merged ? gui::kOkColor : gui::kDefectColor, fixAllRebuiltLine(rebuild));
        drawFixAllLines(shown->fixAll->result, true);
        ImGui::TextDisabled("Click the Fix All row to see the report of the repair alone.");
        ImGui::Spacing();
        if (rebuild.base != nullptr) {
            drawFixAllSteps(rebuild.base->result);
        }
    } else if (shown != nullptr && shown->method != GlobalRepairMethod::FixAll) {
        // A whole-scope rebuild row from Compare
        if (shown->success) {
            wordWrapped(std::string(globalRepairMethodLabel(shown->method)) + ": a new surface. No original face is kept (see Kept in the table).");
            if (shown->method == GlobalRepairMethod::AlphaWrap) {
                wrappedColored(gui::kMutedColor, fixAllRebuildTradeOffText());
            }
        }
        if (!shown->statusMessage.empty()) {
            wrappedColored(gui::kMutedColor, shown->statusMessage);
        }
    } else {
        drawFixAllLines(result, true);
        const std::vector<FixAllResultLine> lines = fixAllResultLines(result, true);
        const bool offered = std::any_of(lines.begin(), lines.end(), [](const FixAllResultLine& l) { return l.rebuildShell >= 0; });
        if (offered) {
            ImGui::Spacing();
            wrappedColored(gui::kMutedColor, fixAllRebuildTradeOffText());
        }
        ImGui::TextDisabled("Shell numbers are those of the result. Apply to get Show them and Flip buttons for the shells named.");
        ImGui::Spacing();
        drawFixAllSteps(result);
    }

    drawFixAllPreviewControls();
}

void PolyscopeGui::drawFixAllResultFooter() {
    const RepairCandidate* candidate = controller_.fixAllCandidate();
    ImGui::Separator();
    if (candidate == nullptr || !candidate->success || candidate->fixAll == nullptr) {
        if (ImGui::Button("Back to plan")) {
            controller_.backToFixAllPlan();
            requestSync();
        }
        tooltip("Return to the plan to change settings and preview again.");
        flowSameLine("Close");
        if (ImGui::Button("Close")) {
            controller_.closeTool();
            requestSync();
        }
        tooltip("Close without changing the mesh.  Key: Esc");
        return;
    }
    drawFixAllApplyRow(false);
}

void PolyscopeGui::drawFixAllPreviewControls() {
    ImGui::SeparatorText("Preview");
    int show = static_cast<int>(controller_.globalPreviewShow());
    bool showChanged = ImGui::RadioButton("Result", &show, static_cast<int>(GlobalPreviewShow::Result));
    tooltip("The repaired mesh.");
    ImGui::SameLine();
    showChanged |= ImGui::RadioButton("Original", &show, static_cast<int>(GlobalPreviewShow::Original));
    tooltip("The mesh as it is now.");
    ImGui::SameLine();
    showChanged |= ImGui::RadioButton("Both", &show, static_cast<int>(GlobalPreviewShow::Both));
    tooltip("Both: the faces the repair removes are ghosted behind the result.");
    if (showChanged) {
        controller_.setGlobalPreviewShow(static_cast<GlobalPreviewShow>(show));
        requestSync();
    }
    bool deviation = controller_.colorByDeviation();
    if (ImGui::Checkbox("Color by deviation", &deviation)) {
        controller_.setColorByDeviation(deviation);
        requestSync();
    }
    tooltip("Color the result by its distance from the original surface. Off: the changed faces are tinted.");
    drawWholeMeshPreviewControls(true);
}

void PolyscopeGui::drawFixAllApplyRow(bool inPlan) {
    const RepairCandidate* shown = controller_.shownGlobalCandidate();
    const bool busy = controller_.globalRepairRunning() || controller_.fixAllRebuildRunning();
    // No shown result (or a failed one, or one still being computed): no Apply button at all
    if (shown != nullptr && shown->success && !busy) {
        std::string label = "Apply Fix All";
        std::string help = "Replace the mesh with the shown result. It is one undo step: Undo Fix All, or Ctrl/Cmd+Z, puts everything back.";
        if (shown->fixAll != nullptr && shown->fixAll->rebuild) {
            label = "Apply " + AppController::fixAllRebuildUndoLabel(*shown);
            help = "Replace the mesh with the Fix All result in which this shell is rebuilt (and merged). One undo step: Ctrl/Cmd+Z "
                   "puts everything back.";
        } else if (shown->method != GlobalRepairMethod::FixAll) {
            label = std::string("Apply ") + globalRepairMethodLabel(shown->method);
            help = "Replace the mesh with this new surface. No original face is kept. One undo step: Ctrl/Cmd+Z puts everything back.";
        }
        ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
        if (ImGui::Button(label.c_str()) && controller_.applyGlobalRepair()) {
            requestSync(true);
        }
        ImGui::PopStyleColor();
        tooltip(help.c_str());
    }
    if (!inPlan) {
        if (shown != nullptr && shown->success && !busy) {
            flowSameLine("Back to plan");
        }
        if (ImGui::Button("Back to plan")) {
            controller_.backToFixAllPlan();
            requestSync();
        }
        tooltip("Drop this preview and return to the plan to change settings. The mesh is unchanged.");
        flowSameLine("Close");
    } else if (shown != nullptr && shown->success && !busy) {
        flowSameLine("Close");
    }
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close without changing the mesh.  Key: Esc");
    ImGui::TextDisabled("Apply = 1 undo step.");
}

void PolyscopeGui::drawFixAllShellRebuildRows() {
    const std::vector<RepairCandidate>& rows = controller_.fixAllRebuilds();
    if (rows.empty()) {
        return;
    }
    for (const RepairCandidate& row : rows) {
        const std::string label = fixAllRebuildRowLabel(row);
        if (!row.success) {
            wrappedColored(gui::kDefectColor, label + ": " + row.statusMessage);
        } else if (row.fixAll != nullptr && row.fixAll->rebuild) {
            wrappedColored(gui::kMutedColor, label + ": " + row.fixAll->rebuild->note);
        }
    }
}

void PolyscopeGui::drawFixAllRebuildJob() {
    const bool shellJob = controller_.fixAllRebuildRunning();
    if (!shellJob && !controller_.globalRepairRunning()) {
        return;
    }
    ImGui::AlignTextToFramePadding();
    gui::statusDot(HealthStatus::Running, false);
    ImGui::SameLine();
    bool cancelling = false;
    if (shellJob) {
        const std::string text = controller_.fixAllRebuildStatusText();
        cancelling = text.starts_with("Cancelling");
        ImGui::TextColored(gui::kRunningColor, "%s", text.c_str());
    } else if (controller_.globalRepairCancelling()) {
        cancelling = true;
        ImGui::TextColored(gui::kRunningColor, "Cancelling...");
    } else {
        const std::optional<GlobalRepairMethod> current = controller_.globalRepairCurrentMethod();
        ImGui::TextColored(gui::kRunningColor, "%s...", current ? globalRepairMethodLabel(*current) : "Working");
    }
    ImGui::ProgressBar(controller_.globalRepairProgress(), ImVec2(-1.0f, 0.0f));
    if (!cancelling) {
        if (ImGui::Button("Cancel##rebuild")) {
            controller_.cancelGlobalRepair();
            requestSync();
        }
        tooltip("Stop and keep the mesh as it is. An alpha wrap stops at its next step; a union or a merge in progress finishes first.");
    }
}

// -----------------------------------------------------------------------------
// The Repair / Rebuild switch and the Rebuild side
// -----------------------------------------------------------------------------

void PolyscopeGui::drawFixAllSwitch() {
    // Rebuilds wrap whole shells: a selection of faces has none, so the switch is not offered
    if (controller_.fixAllScope().kind == FixAllScopeKind::Faces) {
        fixAllRebuildSide_ = false;
        return;
    }
    if (controller_.globalRepairRunning() || controller_.fixAllRebuildRunning()) {
        return;
    }
    int side = fixAllRebuildSide_ ? 1 : 0;
    bool changed = ImGui::RadioButton("Repair  keep my geometry", &side, 0);
    tooltip("Fix each problem in place. Faces that are not part of a problem stay exactly as they are. Cavities are kept.");
    // Side by side only when both fit; else one under the other
    const float radioWidth = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x +
                             ImGui::CalcTextSize("Rebuild  new watertight surface").x;
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + radioWidth <=
        ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) {
        ImGui::SameLine();
    }
    changed |= ImGui::RadioButton("Rebuild  new watertight surface", &side, 1);
    tooltip("Replace the shells with a new surface (Shrink-Wrap, Voxel Heal & Remesh, or Precision Isosurface). "
            "All rebuild methods produce a guaranteed closed surface. Compare them with Repair before you choose.");
    if (changed && (side == 1) != fixAllRebuildSide_) {
        controller_.backToFixAllPlan(); // drops the other side's previews: the mesh is unchanged
        fixAllRebuildSide_ = side == 1;
        requestSync();
    }
}

void PolyscopeGui::drawFixAllRebuildPlan() {
    const FixAllScopeInfo scope = controller_.fixAllScope();
    const Unit unit = controller_.fileUnit();
    const bool running = controller_.globalRepairRunning();
    GlobalRepairParams params = controller_.globalRepairParams();
    const double alpha = alphaWrapAlpha(params, controller_.meshBounds().diagonal());
    const double offset = alphaWrapOffset(params, alpha);
    const FixAllRebuildCopy copy = fixAllRebuildCopy(lengthWithUnit(offset, unit), lengthWithUnit(alpha, unit));

    wordWrapped(copy.headline);
    wordWrapped(copy.guarantee);
    for (const std::string& cost : copy.costs) {
        ImGui::Bullet();
        wordWrapped(cost);
    }
    wrappedColored(gui::kMutedColor, copy.advice);

    // Cavities that an alpha wrap of the scope fills: worked out once per mesh and scope, not every frame
    if (!running && (!fixAllCavityCache_.valid || fixAllCavityCache_.meshGeneration != controller_.meshGeneration() ||
                     fixAllCavityCache_.scopeFaces != scope.faces || fixAllCavityCache_.scopeText != scope.text)) {
        fixAllCavityCache_ = {true, controller_.meshGeneration(), scope.faces, scope.text, controller_.fixAllRebuildFilledCavities()};
    }
    const std::string cavityText = fixAllRebuildCavityText(fixAllCavityCache_.cavities);
    if (!cavityText.empty()) {
        wrappedColored(gui::kDefectColor, "(!) " + cavityText);
        tooltip("A cavity is a hollow shell inside a solid (wound inward). Repair keeps it; an alpha wrap fills it in, because the "
                "wrap closes over everything inside the outer surface.");
    }

    ImGui::SeparatorText("Method");
    for (const RebuildMethodOption& option : kRebuildMethodOptions) {
        const std::string label = std::string(globalRepairMethodLabel(option.method)) + "##rebuild";
        if (ImGui::RadioButton(label.c_str(), fixAllRebuildMethod_ == option.method)) {
            fixAllRebuildMethod_ = option.method;
        }
        tooltip(option.help);
        ImGui::Indent();
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        wordWrapped(option.summary);
        ImGui::PopStyleColor();
        if (fixAllRebuildMethod_ == option.method) {
            drawGlobalMethodParams(option.method);
        }
        ImGui::Unindent();
    }

    if (!controller_.globalCandidates().empty()) {
        drawGlobalCandidateTable();
        const RepairCandidate* shown = controller_.shownGlobalCandidate();
        if (shown != nullptr && !shown->statusMessage.empty()) {
            wrappedColored(gui::kMutedColor, shown->statusMessage);
        }
        if (shown != nullptr && !controller_.candidateIsCurrent(*shown)) {
            ImGui::TextColored(gui::kDefectColor, "Computed with other settings: Preview again to update.");
        }
        drawFixAllPreviewControls();
    }
}

void PolyscopeGui::drawFixAllRebuildFooter() {
    const FixAllScopeInfo scope = controller_.fixAllScope();
    const bool running = controller_.globalRepairRunning();
    ImGui::Separator();
    if (running) {
        drawFixAllRebuildJob();
    } else if (scope.faces > 0) {
        ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
        if (ImGui::Button("Preview")) {
            fixAllHintSeen_ = true;
            controller_.setGlobalRepairMethod(fixAllRebuildMethod_);
            controller_.previewGlobalRepair();
            requestSync();
        }
        ImGui::PopStyleColor();
        tooltip("Compute the chosen rebuild on a copy of the mesh, in the background, and show it. Nothing changes until you Apply.");
        flowSameLine("Compare with Repair");
        if (ImGui::Button("Compare with Repair")) {
            fixAllHintSeen_ = true;
            controller_.setGlobalRepairMethod(GlobalRepairMethod::FixAll);
            controller_.compareAllGlobalRepairs();
            requestSync();
        }
        tooltip("Run Fix All and the three rebuilds on the scope and list them side by side: how much of your geometry each keeps "
                "(Kept), how far it moves (Max dev) and what it fills in (Lost). Nothing changes until Apply.");
    } else {
        ImGui::TextDisabled("Nothing is in scope: every shell is hidden.");
    }

    ImGui::Separator();
    drawFixAllApplyRow(true);
}

void PolyscopeGui::drawFixAllReport() {
    const std::optional<FixAllApplied>& applied = controller_.fixAllApplied();
    const FixAllCandidate* run = controller_.fixAllShown();
    if (!applied || run == nullptr) {
        ImGui::TextDisabled("No report.");
        if (ImGui::Button("Close")) {
            controller_.closeTool();
            requestSync();
        }
        return;
    }
    if (applied->undoLabel.empty()) {
        ImGui::TextColored(gui::kOkColor, "Fix All finished: it changed nothing, so there is nothing to undo.");
    } else {
        ImGui::TextColored(gui::kOkColor, "Fix All applied (Ctrl/Cmd+Z to undo).");
    }
    if (run->rebuild) {
        wordWrapped(fixAllRebuiltLine(*run->rebuild));
    }
    const bool current = applied->meshGeneration == controller_.meshGeneration();
    if (current) {
        drawFixAllLines(run->result, false);
    } else {
        for (const FixAllResultLine& line : fixAllResultLines(run->result, false)) {
            if (line.warning) {
                wrappedColored(gui::kDefectColor, "(!) " + line.text);
            } else {
                wordWrapped(line.text);
            }
        }
        ImGui::TextDisabled("The mesh changed after Fix All, so the Show and Flip buttons are hidden. This report is as it was.");
    }
    {
        const std::vector<FixAllResultLine> lines = fixAllResultLines(run->result, false);
        if (std::any_of(lines.begin(), lines.end(), [](const FixAllResultLine& l) { return l.rebuildShell >= 0; })) {
            ImGui::TextDisabled("A shell is rebuilt in the preview, before Apply: undo, preview again and use Rebuild shell #k.... "
                                "Repair > Shrink-Wrap (Outer Envelope)... also wraps shells you select.");
        }
    }
    ImGui::Spacing();
    drawFixAllSteps(run->rebuild && run->rebuild->base != nullptr ? run->rebuild->base->result : run->result);
}

void PolyscopeGui::drawFixAllReportFooter() {
    const std::optional<FixAllApplied>& applied = controller_.fixAllApplied();
    if (!applied || controller_.fixAllShown() == nullptr) {
        return;
    }
    const bool undoable = controller_.fixAllUndoAvailable();
    // Read now: Close (or Undo) below resets `applied`
    const bool undoHint = !undoable && !applied->undoLabel.empty();
    ImGui::Separator();
    if (undoable) {
        if (ImGui::Button("Undo Fix All") && controller_.undoFixAll()) {
            requestSync(true);
        }
        tooltip("Put the mesh back as it was before Fix All (also Ctrl/Cmd+Z).");
        flowSameLine("Run one more pass");
    }
    if (ImGui::Button("Run one more pass")) {
        controller_.runFixAllOneMorePass();
        requestSync();
    }
    tooltip("Preview one more pass of Fix All on the mesh as it is now. Nothing changes until you Apply. Useful when the report "
            "says something is left: some problems need the result of the first pass to be repaired.");
    flowSameLine("Close");
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close this panel.  Key: Esc");
    if (undoHint) {
        ImGui::TextDisabled("Undo Fix All appears again when the edits made after it are undone (Ctrl/Cmd+Z).");
    }
}

} // namespace ogun
