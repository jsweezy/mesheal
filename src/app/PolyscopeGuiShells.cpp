#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>
#include <portable-file-dialogs.h>

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

std::string shellLabel(std::size_t shell) {
    return "#" + std::to_string(shell + 1);
}

/// Filled dot on a framed row (after AlignTextToFramePadding). Advances the cursor.
void rowDot(const ImVec4& color) {
    const float lineHeight = ImGui::GetTextLineHeight();
    const float radius = lineHeight * 0.3f;
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    const ImVec2 center(cursor.x + radius + 2.0f, cursor.y + ImGui::GetStyle().FramePadding.y + lineHeight * 0.5f);
    ImGui::GetWindowDrawList()->AddCircleFilled(center, radius, ImGui::GetColorU32(color), 12);
    ImGui::Dummy(ImVec2(2.0f * radius + 4.0f, lineHeight));
}

} // namespace

// -----------------------------------------------------------------------------
// Shells panel (UI_DESIGN.md §16): pick which shells Mesh Health works on
// -----------------------------------------------------------------------------

void PolyscopeGui::drawShellsPanel() {
    bool colorByShell = controller_.colorByShell();
    if (ImGui::Checkbox("Color by shell", &colorByShell)) {
        controller_.setColorByShell(colorByShell);
        requestSync();
    }
    tooltip("Give each shell its own muted color (no reds or oranges, so defect tints stay clear).");
    const float selectWidth = ImGui::CalcTextSize("Select...").x + 2.0f * ImGui::GetStyle().FramePadding.x;
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - selectWidth);
    if (ImGui::Button("Select...")) {
        ImGui::OpenPopup("ShellSelect");
    }
    tooltip("Select all, none, invert, the visible, unhealthy, nested or small shells.");
    drawShellSelectPopup();

    // Footer: a summary only; the actions are in the right-click menu and Mesh > Shells
    const float footer = 2.0f * ImGui::GetTextLineHeightWithSpacing();
    drawShellTable(std::max(80.0f, ImGui::GetContentRegionAvail().y - footer));

    const ShellBrowser& shells = controller_.shells();
    if (shells.selectedCount() == 0) {
        const MouseBinding& shellClick = mouseMapping_.of(MouseAction::SelectShell)[0];
        if (shellClick.bound()) {
            ImGui::TextDisabled("Click a shell, or %s it in 3D.",
                                formatMouseBinding(shellClick, MouseActionKind::Click).c_str());
        } else {
            ImGui::TextDisabled("Click a shell to select it.");
        }
    } else {
        std::size_t faces = 0;
        for (const std::size_t s : shells.selectedShells()) {
            faces += shells.shell(s).faces.size();
        }
        ImGui::Text("%s selected - %s faces", withThousands(shells.selectedCount()).c_str(), withThousands(faces).c_str());
    }
    ImGui::TextDisabled("Right-click a shell for actions.");
    tooltip("Frame (.), Isolate (I), Hide, Hide All / Show All, Delete (Delete key), Save As.");
}

void PolyscopeGui::drawShellTable(float height) {
    const ImGuiIO& io = ImGui::GetIO();
    const ShellBrowser& shells = controller_.shells();
    const std::vector<ShellHealth>& health = controller_.shellHealth();
    const bool known = controller_.shellHealthKnown() && health.size() == shells.size();
    const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                                  ImGuiTableFlags_Sortable | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("ShellTable", 4, flags, ImVec2(0.0f, height))) {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Vis", ImGuiTableColumnFlags_NoSort, 0.0f, 0);
    ImGui::TableSetupColumn("Shell", ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_WidthStretch, 0.0f, 1);
    ImGui::TableSetupColumn("Faces", ImGuiTableColumnFlags_None, 0.0f, 2);
    ImGui::TableSetupColumn("Health", ImGuiTableColumnFlags_None, 0.0f, 3);
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    ImGui::TableSetColumnIndex(0);
    bool allVisible = shells.visibleCount() == shells.size();
    if (ImGui::Checkbox("##AllVisible", &allVisible)) {
        if (allVisible) {
            controller_.showAllShells();
        } else {
            controller_.hideAllShells();
        }
        requestSync();
    }
    tooltip("Show or hide every shell.");
    for (int column = 1; column < 4; ++column) {
        ImGui::TableSetColumnIndex(column);
        ImGui::TableHeader(ImGui::TableGetColumnName(column));
    }

    std::vector<std::size_t> order(shells.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    if (const ImGuiTableSortSpecs* specs = ImGui::TableGetSortSpecs(); specs != nullptr && specs->SpecsCount > 0) {
        const ImGuiTableColumnSortSpecs& spec = specs->Specs[0];
        const bool ascending = spec.SortDirection == ImGuiSortDirection_Ascending;
        const auto key = [&](std::size_t s) -> std::size_t {
            switch (spec.ColumnUserID) {
            case 2:
                return shells.shell(s).faces.size();
            case 3:
                return known ? health[s].total : 0;
            default:
                return s;
            }
        };
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
            return ascending ? key(a) < key(b) : key(a) > key(b);
        });
    }

    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(order.size()));
    while (clipper.Step()) {
        for (int pos = clipper.DisplayStart; pos < clipper.DisplayEnd; ++pos) {
            const std::size_t s = order[static_cast<std::size_t>(pos)];
            const ShellInfo& shell = shells.shell(s);
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(s));

            // The name cell comes first so the visibility checkbox (column 0) stays clickable over it
            ImGui::TableSetColumnIndex(1);
            ImGui::AlignTextToFramePadding();
            if (controller_.colorByShell()) {
                const Eigen::Vector3d c = AppController::shellColor(s);
                const float side = ImGui::GetTextLineHeight() * 0.8f;
                const ImVec2 p = ImGui::GetCursorScreenPos();
                const float pad = (ImGui::GetTextLineHeight() - side) * 0.5f + ImGui::GetStyle().FramePadding.y;
                ImGui::GetWindowDrawList()->AddRectFilled(
                    ImVec2(p.x, p.y + pad), ImVec2(p.x + side, p.y + pad + side),
                    ImGui::GetColorU32(ImVec4(static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()), 1.0f)));
                ImGui::Dummy(ImVec2(side, ImGui::GetTextLineHeight()));
                ImGui::SameLine();
            }
            // Hidden shells are dimmed rather than labelled, so the narrow column never truncates
            if (!shell.visible) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            }
            const bool clicked = ImGui::Selectable(shellLabel(s).c_str(), shell.selected,
                                                   ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap |
                                                       ImGuiSelectableFlags_AllowDoubleClick);
            if (!shell.visible) {
                ImGui::PopStyleColor();
            }
            if (clicked) {
                controller_.clickShell(s, io.KeyCtrl || io.KeySuper, io.KeyShift, order);
                requestSync();
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    focusCameraOnSelection(FocusNormalSide::PreserveViewDirection);
                }
            }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && ImGui::BeginTooltip()) {
                drawShellRowTooltip(s);
                ImGui::EndTooltip();
            }
            if (ImGui::BeginPopupContextItem("ShellContext")) {
                if (!shell.selected) {
                    controller_.clickShell(s, false, false, order);
                    requestSync();
                }
                drawShellActionItems();
                ImGui::EndPopup();
            }

            ImGui::TableSetColumnIndex(0);
            bool visible = shell.visible;
            if (ImGui::Checkbox("##vis", &visible)) {
                controller_.setShellVisible(s, visible);
                requestSync();
            }

            ImGui::TableSetColumnIndex(2);
            ImGui::AlignTextToFramePadding();
            const std::string count = withThousands(shell.faces.size());
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(count.c_str()).x);
            if (shell.visible) {
                ImGui::TextUnformatted(count.c_str());
            } else {
                ImGui::TextDisabled("%s", count.c_str());
            }

            ImGui::TableSetColumnIndex(3);
            ImGui::AlignTextToFramePadding();
            if (!known) {
                ImGui::TextDisabled("-");
            } else {
                const ImVec4 color = health[s].total == 0 ? gui::kOkColor : gui::kDefectColor;
                rowDot(color);
                ImGui::SameLine(0.0f, 2.0f);
                ImGui::TextColored(color, "%s", withThousands(health[s].total).c_str());
            }
            if (shell.nestedIn >= 0) {
                ImGui::SameLine();
                ImGui::TextColored(gui::kRunningColor, "inside %s", shellLabel(static_cast<std::size_t>(shell.nestedIn)).c_str());
            }
            ImGui::PopID();
        }
    }
    ImGui::EndTable();
}

void PolyscopeGui::drawShellRowTooltip(std::size_t shell) {
    const ShellInfo& info = controller_.shells().shell(shell);
    const std::vector<ShellHealth>& health = controller_.shellHealth();
    ImGui::Text("Shell %s: %s faces", shellLabel(shell).c_str(), withThousands(info.faces.size()).c_str());
    if (!controller_.shellHealthKnown() || shell >= health.size()) {
        ImGui::TextDisabled("Not checked yet (Check Mesh, F5).");
    } else {
        const ShellHealth& h = health[shell];
        if (h.total == 0) {
            ImGui::TextColored(gui::kOkColor, "Clean: nothing to fix inside this shell.");
        }
        for (const DefectCategory category : kAllDefectCategories) {
            if (h.counts[categoryIndex(category)] > 0) {
                ImGui::TextColored(gui::kDefectColor, "%s: %s", categoryInfo(category).name,
                                   withThousands(h.counts[categoryIndex(category)]).c_str());
            }
        }
        if (!h.crosses.empty()) {
            std::string list;
            for (const int other : h.crosses) {
                list += (list.empty() ? "" : ", ") + shellLabel(static_cast<std::size_t>(other));
            }
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
            ImGui::TextDisabled("Crosses shell%s %s (%s faces). Merge Shells (a union) removes these.",
                                h.crosses.size() == 1 ? "" : "s", list.c_str(), withThousands(h.between).c_str());
            ImGui::PopTextWrapPos();
        }
    }
    if (info.nestedIn >= 0) {
        ImGui::TextColored(gui::kRunningColor, "Lies inside shell %s without crossing it: often a hidden internal part.",
                           shellLabel(static_cast<std::size_t>(info.nestedIn)).c_str());
    }
    ImGui::TextDisabled("Double-click to frame. Right-click for actions.");
}

void PolyscopeGui::drawShellActionItems() {
    const ShellBrowser& shells = controller_.shells();
    const bool none = shells.selectedCount() == 0;
    if (ImGui::MenuItem("Frame", ".", false, !none)) {
        focusCameraOnSelection(FocusNormalSide::PreserveViewDirection);
    }
    const bool unisolate = shells.isolated() && (none || shells.selectionMatchesVisible());
    if (ImGui::MenuItem(unisolate ? "Un-isolate" : "Isolate", "I", false, shells.isolated() || !none)) {
        toggleIsolateShells();
    }
    tooltip("Show only the selected shells; Mesh Health then covers only them. Press again to go back.");
    if (ImGui::MenuItem("Hide", nullptr, false, !none)) {
        controller_.hideSelectedShells();
        requestSync();
    }
    if (ImGui::MenuItem("Hide All")) {
        controller_.hideAllShells();
        requestSync();
    }
    if (ImGui::MenuItem("Show All")) {
        controller_.showAllShells();
        requestSync();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Delete", "Delete", false, !none)) {
        deleteSelectedShells();
    }
    tooltip("Remove the selected shells from the mesh (undoable).");
    if (ImGui::BeginMenu("Save As", !none)) {
        if (ImGui::MenuItem("Selected shells to one STL...")) {
            saveSelectedShellsDialog(false);
        }
        if (ImGui::MenuItem("Each selected shell to its own STL...")) {
            saveSelectedShellsDialog(true);
        }
        ImGui::EndMenu();
    }
    tooltip("Write the selected shells to new STL files. The open mesh is not changed.");
    if (ImGui::MenuItem("Merge...", nullptr, false, shells.selectedCount() >= 2)) {
        controller_.openMergeShells();
        requestSync();
    }
    tooltip(shells.selectedCount() >= 2
                ? "Boolean union of the selected shells with Manifold: a checklist first, then Merge (undoable)."
                : "Select two or more shells to merge: Cmd/Ctrl+click or Shift+click rows, or Select... > All.");
}

// -----------------------------------------------------------------------------
// Operation: Merge Shells (UI_DESIGN.md §16.3)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawMergeShellsOperator() {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Boolean union of the selected shells with Manifold. Faces where they cross are removed, and "
                        "shells that overlap or touch become one. Shells that do not touch stay separate. Cavities "
                        "(shells wound inward inside a solid) are kept as they are, never filled.");
    ImGui::PopTextWrapPos();
    const MergeChecklist list = controller_.mergeChecklist();
    if (list.shells.size() < 2) {
        ImGui::TextWrapped("Select two or more shells in the Shells panel (Select... > All for everything).");
        if (ImGui::Button("Close")) {
            controller_.closeTool();
            requestSync();
        }
        return;
    }
    ImGui::Text("%s shells selected (%s faces).", withThousands(list.shells.size()).c_str(), withThousands(list.faces).c_str());

    ImGui::Separator();
    ImGui::TextUnformatted("Every shell must be:");
    if (ImGui::BeginTable("MergeChecks", 3, ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("##dot", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##check", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##count", ImGuiTableColumnFlags_WidthFixed);
        for (std::size_t c = 0; c < list.checks.size(); ++c) {
            const MergeCheck& check = list.checks[c];
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(c));
            ImGui::TableSetColumnIndex(0);
            if (!check.known) {
                gui::statusDot(HealthStatus::NotRun, false);
            } else {
                gui::statusDot(check.failing.empty() ? HealthStatus::Ok : HealthStatus::Defects, false);
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(check.label);
            if (!check.blocking) {
                ImGui::SameLine();
                ImGui::TextDisabled("(warning)");
            }
            ImGui::TableSetColumnIndex(2);
            if (!check.known) {
                ImGui::TextDisabled("not checked");
            } else {
                ImGui::Text("%s/%s", withThousands(list.shells.size() - check.failing.size()).c_str(),
                            withThousands(list.shells.size()).c_str());
                if (!check.failing.empty()) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Show")) {
                        controller_.selectShellsWhere([&](std::size_t s) {
                            return std::binary_search(check.failing.begin(), check.failing.end(), s);
                        });
                        controller_.postStatusMessage("Selected the " + withThousands(check.failing.size()) +
                                                      " shell(s) that are not " + check.label +
                                                      ". Isolate (I) and fix them, then select the shells to merge again.");
                        requestSync();
                    }
                    tooltip("Select only the shells that fail this check.");
                }
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    const MergeCheck& selfCrossings = list.checks[MergeChecklist::kNoSelfCrossings];
    ImGui::PushTextWrapPos(0.0f);
    for (const std::string& problem : list.problems) {
        ImGui::TextColored(gui::kDefectColor, "%s", problem.c_str());
    }
    if (!list.cavities.empty()) {
        std::string kept;
        for (std::size_t i = 0; i < list.cavities.size(); ++i) {
            kept += (i == 0 ? "#" : (i + 1 == list.cavities.size() ? " and #" : ", #")) + std::to_string(list.cavities[i] + 1);
        }
        ImGui::TextDisabled("%s %s kept: %s.", withThousands(list.cavities.size()).c_str(),
                            list.cavities.size() == 1 ? "cavity" : "cavities", kept.c_str());
        tooltip("A cavity is a shell wound inward inside a solid: a hollow. The union leaves it out and keeps its "
                "faces exactly as they are.");
    }
    for (const std::size_t island : list.islands) {
        ImGui::TextDisabled("#%zu floats inside a kept cavity: left as it is.", island + 1);
    }
    if (list.problems.empty() && list.operands.size() < 2) {
        ImGui::TextColored(gui::kDefectColor, "Fewer than two selected shells face outward: nothing to unite.");
    }
    if (!selfCrossings.failing.empty()) {
        ImGui::TextColored(gui::kDefectColor, "Shells that cross themselves can still be merged, but the union may "
                                               "be wrong where they do (Repair All Intersections fixes them first).");
    }
    ImGui::PopTextWrapPos();
    if (!selfCrossings.known) {
        ImGui::TextDisabled("Self-crossings are not checked: run Intersecting faces to see them.");
    }
    if (selfCrossings.known) {
        ImGui::TextDisabled("%s faces cross between these shells; the union removes them.",
                            withThousands(list.crossingsBetween).c_str());
    }
    ImGui::TextDisabled("Thin, degenerate and duplicate faces do not block a merge.");
    ImGui::TextDisabled("Gaps between shells are not closed yet: shells that almost touch stay apart.");

    ImGui::Separator();
    const bool running = controller_.mergeRunning();
    ImGui::BeginDisabled(!list.canApply() || running);
    ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
    const std::string label = "Merge " + withThousands(list.operands.size()) + " Shells";
    if (ImGui::Button(label.c_str()) && controller_.mergeSelectedShells()) {
        requestSync(true);
    }
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", running ? "A merge is running (see the status bar)."
                                : list.canApply() ? "Union the selected shells as one undoable step."
                                                  : "Every selected shell must pass the checks above except the warning, "
                                                    "and Mesh Health must have checked them; at least two must face "
                                                    "outward. Use Show to find the ones that fail.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    ImGui::TextDisabled("Merge is one undo step (%s+Z).", gui::kModKey);
}

void PolyscopeGui::drawShellSelectPopup() {
    if (!ImGui::BeginPopup("ShellSelect")) {
        return;
    }
    const ShellBrowser& shells = controller_.shells();
    const std::vector<ShellHealth>& health = controller_.shellHealth();
    const auto select = [this](auto&& predicate) {
        controller_.selectShellsWhere(predicate);
        requestSync();
    };
    if (ImGui::MenuItem("All")) {
        select([](std::size_t) { return true; });
    }
    if (ImGui::MenuItem("None", "Esc")) {
        select([](std::size_t) { return false; });
    }
    if (ImGui::MenuItem("Invert")) {
        select([&](std::size_t s) { return !shells.shell(s).selected; });
    }
    if (ImGui::MenuItem("Visible shells")) {
        select([&](std::size_t s) { return shells.shell(s).visible; });
    }
    ImGui::Separator();
    const bool known = controller_.shellHealthKnown() && health.size() == shells.size();
    if (ImGui::MenuItem("Unhealthy (orange)", nullptr, false, known)) {
        select([&](std::size_t s) { return health[s].total > 0; });
    }
    if (ImGui::MenuItem("Nested inside another shell")) {
        select([&](std::size_t s) { return shells.shell(s).nestedIn >= 0; });
    }
    tooltip("Shells lying inside another shell without crossing it (found by Intersecting faces).");
    ImGui::Separator();
    const std::string small = "Smaller than " + withThousands(static_cast<std::size_t>(smallShellFaces_)) + " faces";
    if (ImGui::MenuItem(small.c_str())) {
        select([&](std::size_t s) { return shells.shell(s).faces.size() < static_cast<std::size_t>(smallShellFaces_); });
    }
    tooltip("Then press Delete to remove small loose pieces.");
    ImGui::SetNextItemWidth(100.0f);
    ImGui::InputInt("N faces", &smallShellFaces_, 0);
    smallShellFaces_ = std::max(smallShellFaces_, 1);
    ImGui::EndPopup();
}

void PolyscopeGui::toggleIsolateShells() {
    if (controller_.toggleIsolateShells()) {
        requestSync();
    }
}

void PolyscopeGui::deleteSelectedShells() {
    deleteShellsRequested_ = true;
}

void PolyscopeGui::deleteSelection() {
    if (controller_.selectedFaceCount() > 0) {
        deleteFacesRequested_ = true;
    } else if (controller_.shells().selectedCount() > 0) {
        deleteShellsRequested_ = true;
    }
}

void PolyscopeGui::saveSelectedShellsDialog(bool eachOwnFile) {
    if (controller_.shells().selectedCount() == 0) {
        return;
    }
    if (eachOwnFile) {
        const std::string dir = pfd::select_folder("Save Each Selected Shell To Folder", "", pfd::opt::none).result();
        if (!dir.empty()) {
            controller_.saveEachSelectedShell(std::filesystem::path(dir), saveFormat_);
        }
        return;
    }
    const std::string suggestion = controller_.shellFileStem() + "_shells.stl";
    const std::string destination = pfd::save_file(
        "Save Selected Shells",
        suggestion,
        {"STL Mesh (*.stl)", "*.stl *.STL", "All Files", "*"},
        pfd::opt::none).result();
    if (destination.empty()) {
        return;
    }
    std::filesystem::path outPath(destination);
    if (!outPath.has_extension()) {
        outPath.replace_extension(".stl");
    }
    controller_.saveSelectedShells(outPath, saveFormat_);
}

} // namespace ogun
