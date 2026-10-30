#include "app/FixAllPanelModel.hpp"
#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

constexpr float kFoundColumnWidth = 118.0f;

constexpr const char* kFixAllTooltipFull =
    "Repair every problem in the scope (the selection, or the whole mesh), repeating the sequence up to the number of "
    "passes you set (3 by default), with a preview and before / after numbers. Nothing changes until Apply.  Key: Shift+F5";
constexpr const char* kFixAllTooltipOnePass =
    "Run the Fix All sequence once and preview it, so you can see each step's effect. Run it again for the next pass. "
    "Nothing changes until Apply.  Key: Alt+Shift+F5";

void wrappedColored(const ImVec4& color, const std::string& text) {
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
}

} // namespace

void PolyscopeGui::openFixAllPanel(bool fullRun) {
    if (!controller_.hasMesh()) {
        return;
    }
    controller_.openFixAll(fullRun ? controller_.preferenceDefaults().fixAll.passes : 1);
    requestSync();
}

// -----------------------------------------------------------------------------
// Mesh Health header: [Check Mesh] [Fix All...] [One Pass...]
// -----------------------------------------------------------------------------

void PolyscopeGui::drawFixAllHeaderButtons() {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float fixWidth = ImGui::CalcTextSize("Fix All...").x + 2.0f * style.FramePadding.x;
    const float oneWidth = ImGui::CalcTextSize("One Pass...").x + 2.0f * style.FramePadding.x;
    const float spinner = controller_.fixAllRunning() ? ImGui::GetTextLineHeight() + style.ItemSpacing.x : 0.0f;
    const float rightEdge = ImGui::GetWindowContentRegionMax().x + ImGui::GetWindowPos().x;
    const float needed = ImGui::GetCursorScreenPos().x + fixWidth + spinner + style.ItemSpacing.x + oneWidth;

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
    // Narrow header: One Pass... moves to a second line
    if (needed <= rightEdge) {
        ImGui::SameLine();
    }
    if (ImGui::Button("One Pass...")) {
        openFixAllPanel(false);
    }
    tooltip(kFixAllTooltipOnePass);
}

// -----------------------------------------------------------------------------
// Operation: Fix All
// -----------------------------------------------------------------------------

void PolyscopeGui::drawFixAllOperator() {
    if (!controller_.hasMesh()) {
        ImGui::TextDisabled("No mesh loaded.");
        return;
    }
    ImGui::PushTextWrapPos(0.0f); // long lines wrap at the panel edge
    switch (controller_.fixAllState()) {
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

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("Fixes each problem in place, in a safe order, and repeats the whole sequence until the mesh is "
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
            ImGui::SameLine();
            if (ImGui::Button("Preview one pass")) {
                fixAllHintSeen_ = true;
                controller_.previewFixAll(true);
                requestSync();
            }
            tooltip("Run the sequence once, whatever Passes says, so you can see each step's effect before the next pass.");
        }
        ImGui::SameLine();
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
            ImGui::TextWrapped("%s  %s", number.c_str(), row.label.c_str());
            tooltip(tip.c_str());
            if (!row.hasBox() && row.sw != FixAllRowSwitch::Required && !row.note.empty() && row.note != row.found) {
                ImGui::SameLine();
                ImGui::TextColored(gui::kMutedColor, "(%s)", row.note.c_str());
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
        ImGui::TextWrapped("Merge needs whole shells: select shells, or clear the selection. Faces around the selection stay "
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
            ImGui::TextWrapped(controller_.fixAllMergeUserSet() ? "Ticked by you. Untick to keep the shells as separate parts."
                                                                : "Ticked because shells overlap. Untick to keep them as separate parts.");
        } else {
            ImGui::TextWrapped("Not ticked: the shells stay separate parts. Tick it to merge those that overlap.");
        }
        ImGui::TextDisabled("Removes faces inside other shells; a nested solid touching nothing is deleted; cavities are kept.");
    } else if (ticked && plan->tickedByDefault) {
        ImGui::TextWrapped("Ticked because shells overlap, but none of them can be merged (see below).");
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
        ImGui::TextWrapped("%s", links[i].text.c_str());
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
        ImGui::TextWrapped("%s", text.phase.c_str());
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
                ImGui::TextWrapped("%d  %s", step.number, step.text.c_str());
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
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const FixAllResultLine& line = lines[i];
        ImGui::PushID(static_cast<int>(i));
        if (line.warning) {
            wrappedColored(gui::kDefectColor, "(!) " + line.text);
        } else {
            ImGui::TextWrapped("%s", line.text.c_str());
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
                ImGui::SameLine();
                if (ImGui::SmallButton("Flip all")) {
                    controller_.fixAllFlipShells(line.shells);
                    requestSync(true);
                }
                tooltip("Reverse the outer shell and the shells inside it (one undo step). Use it when the whole group is inside out.");
                ImGui::SameLine();
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
                ImGui::SameLine();
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
        if (ImGui::Button("Back to plan")) {
            controller_.backToFixAllPlan();
            requestSync();
        }
        tooltip("Return to the plan to change settings and preview again.");
        ImGui::SameLine();
        if (ImGui::Button("Close")) {
            controller_.closeTool();
            requestSync();
        }
        tooltip("Close without changing the mesh.  Key: Esc");
        return;
    }
    const FixAllResult& result = candidate->fixAll->result;

    ImGui::TextColored(gui::kInfoColor, "Preview ready. Nothing has changed yet.");
    drawGlobalCandidateTable();
    ImGui::Spacing();
    drawFixAllLines(result, true);
    ImGui::TextDisabled("Shell numbers are those of the result. Apply to get Show them and Flip buttons for the shells named.");
    ImGui::Spacing();
    drawFixAllSteps(result);

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

    ImGui::Separator();
    ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
    if (ImGui::Button("Apply Fix All") && controller_.applyFixAll()) {
        requestSync(true);
    }
    ImGui::PopStyleColor();
    tooltip("Replace the mesh with the shown result. It is one undo step: Undo Fix All, or Ctrl/Cmd+Z, puts everything back.");
    ImGui::SameLine();
    if (ImGui::Button("Back to plan")) {
        controller_.backToFixAllPlan();
        requestSync();
    }
    tooltip("Drop this preview and return to the plan to change settings. The mesh is unchanged.");
    ImGui::SameLine();
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close without changing the mesh.  Key: Esc");
    ImGui::TextDisabled("Apply = 1 undo step.");
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
    const bool undoable = controller_.fixAllUndoAvailable();
    if (applied->undoLabel.empty()) {
        ImGui::TextColored(gui::kOkColor, "Fix All finished: it changed nothing, so there is nothing to undo.");
    } else {
        ImGui::TextColored(gui::kOkColor, "Fix All applied (Ctrl/Cmd+Z to undo).");
    }
    const bool current = applied->meshGeneration == controller_.meshGeneration();
    if (current) {
        drawFixAllLines(run->result, false);
    } else {
        for (const FixAllResultLine& line : fixAllResultLines(run->result, false)) {
            if (line.warning) {
                wrappedColored(gui::kDefectColor, "(!) " + line.text);
            } else {
                ImGui::TextWrapped("%s", line.text.c_str());
            }
        }
        ImGui::TextDisabled("The mesh changed after Fix All, so the Show and Flip buttons are hidden. This report is as it was.");
    }
    ImGui::Spacing();
    drawFixAllSteps(run->result);

    ImGui::Separator();
    if (undoable) {
        if (ImGui::Button("Undo Fix All") && controller_.undoFixAll()) {
            requestSync(true);
        }
        tooltip("Put the mesh back as it was before Fix All (also Ctrl/Cmd+Z).");
        ImGui::SameLine();
    }
    if (ImGui::Button("Run one more pass")) {
        controller_.runFixAllOneMorePass();
        requestSync();
    }
    tooltip("Preview one more pass of Fix All on the mesh as it is now. Nothing changes until you Apply. Useful when the report "
            "says something is left: some problems need the result of the first pass to be repaired.");
    ImGui::SameLine();
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close this panel.  Key: Esc");
    if (!undoable && !applied->undoLabel.empty()) {
        ImGui::TextDisabled("Undo Fix All appears again when the edits made after it are undone (Ctrl/Cmd+Z).");
    }
}

} // namespace ogun
