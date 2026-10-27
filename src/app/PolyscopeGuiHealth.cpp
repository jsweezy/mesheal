#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <optional>
#include <span>
#include <string>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

struct HealActionOption {
    HealAction action;
    const char* label;
};

constexpr std::array<HealActionOption, 4> kHealActionOptions = {{
    {HealAction::FlipLongestEdge, "Flip Longest Edge (Best for Coplanar Caps)"},
    {HealAction::CollapseShortestEdge, "Collapse Shortest Edge (Best for Needles)"},
    {HealAction::SnapCollinearCollapse, "Snap Collinear Vertex (Cap on Feature Crease)"},
    {HealAction::RemoveDegenerateFace, "Remove Degenerate Flap (Isolated / Redundant Face)"},
}};

const char* elementColumnName(ElementKind kind) {
    switch (kind) {
    case ElementKind::Face:
        return "Face";
    case ElementKind::Edge:
        return "Edge";
    case ElementKind::Vertex:
        return "Vertex";
    case ElementKind::Shell:
        return "Shell";
    case ElementKind::Hole:
        return "Hole";
    }
    return "#";
}

std::string elementLabel(const CategoryInfo& info, const DefectElement& element) {
    char buffer[64];
    if (info.kind == ElementKind::Edge) {
        std::snprintf(buffer, sizeof(buffer), "%d-%d", element.edge[0], element.edge[1]);
    } else if (info.kind == ElementKind::Shell || info.kind == ElementKind::Hole) {
        std::snprintf(buffer, sizeof(buffer), "#%lld", static_cast<long long>(element.index + 1));
    } else {
        std::snprintf(buffer, sizeof(buffer), "#%lld", static_cast<long long>(element.index));
    }
    return buffer;
}

/// Widest possible element label for this mesh ("#258831", "129429-129430"), so the column fits every row.
std::string widestElementLabel(const CategoryInfo& info, const Mesh& mesh, std::size_t shellCount, std::size_t holeCount) {
    const auto widestDigits = [](Eigen::Index count) {
        return std::string(std::to_string(std::max<Eigen::Index>(count, 1)).size(), '8');
    };
    switch (info.kind) {
    case ElementKind::Edge:
        return widestDigits(mesh.numVertices()) + "-" + widestDigits(mesh.numVertices());
    case ElementKind::Vertex:
        return "#" + widestDigits(mesh.numVertices());
    case ElementKind::Shell:
        return "#" + widestDigits(static_cast<Eigen::Index>(shellCount));
    case ElementKind::Hole:
        return "#" + widestDigits(static_cast<Eigen::Index>(holeCount));
    case ElementKind::Face:
        break;
    }
    return "#" + widestDigits(mesh.numFaces());
}

std::string columnHeader(const CategoryColumn& column, Unit unit) {
    const std::string suffix = gui::dimensionSuffix(column.dimension, unit);
    return suffix.empty() ? std::string(column.name) : std::string(column.name) + " (" + suffix + ")";
}

} // namespace

// -----------------------------------------------------------------------------
// Actions shared by the menu, keys and panels
// -----------------------------------------------------------------------------

void PolyscopeGui::checkMesh() {
    if (!controller_.hasMesh()) {
        return;
    }
    controller_.checkMesh();
    requestSync();
}

void PolyscopeGui::fixAll(DefectCategory category) {
    if ((category == DefectCategory::ThinFaces && controller_.fixAllThinFaces()) ||
        (category == DefectCategory::NonManifoldEdges && controller_.fixAllNonManifoldEdges()) ||
        (category == DefectCategory::NonManifoldVertices && controller_.fixAllNonManifoldVertices())) {
        requestSync(true);
    } else if (category == DefectCategory::Intersections) {
        // Several methods with very different trade-offs: open the operator to compare them first
        controller_.openRepairAllIntersections();
        requestSync();
    } else if (category == DefectCategory::OpenEdges) {
        controller_.openRepairAllHoles();
        requestSync();
    }
}

// -----------------------------------------------------------------------------
// Mesh Health panel (UI_DESIGN.md §5)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawHealthPanel() {
    if (!controller_.hasMesh()) {
        ImGui::TextDisabled("No mesh loaded. Use File > Open or File > Load Sample.");
        return;
    }

    if (ImGui::Button("Check Mesh")) {
        checkMesh();
    }
    tooltip("Run every check below in the background, including rows whose Auto is off.  Key: F5");
    ImGui::SameLine();

    bool anyRun = false;
    for (const CategoryHealth& row : controller_.healthReport().rows) {
        anyRun = anyRun || row.status != HealthStatus::NotRun;
    }
    const std::size_t problems = controller_.totalProblemCount();
    if (controller_.checksRunning()) {
        const std::optional<DefectCategory> current = controller_.currentCheckCategory();
        ImGui::TextColored(gui::kRunningColor, "Checking %s... %d%%",
                           current ? categoryInfo(*current).name : "",
                           static_cast<int>(controller_.checkProgress() * 100.0f));
    } else if (!anyRun) {
        ImGui::TextDisabled("Not checked yet");
    } else if (problems == 0) {
        ImGui::TextColored(gui::kOkColor, "No problems found");
    } else {
        ImGui::TextColored(gui::kDefectColor, "%s problem%s", withThousands(problems).c_str(), problems == 1 ? "" : "s");
    }

    // Only while part of the mesh is out of scope (UI_DESIGN.md §16)
    const ShellBrowser& shells = controller_.shells();
    if (shells.scoped()) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(gui::kInfoColor, "Scope: %s of %s shells (%s)", withThousands(shells.scopeCount()).c_str(),
                           withThousands(shells.size()).c_str(), shells.selectedCount() > 0 ? "selected" : "visible");
        tooltip("Mesh Health counts, lists and Fix All cover only these shells: the selected shells, or the visible "
                "ones when none is selected. Crossings with shells outside the scope are not shown.");
        ImGui::SameLine();
        if (ImGui::SmallButton("All shells")) {
            controller_.selectShellsWhere([](std::size_t) { return false; });
            controller_.showAllShells();
            requestSync();
        }
        tooltip("Clear the shell selection and show every shell.");
    }

    const ImGuiTableFlags tableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("HealthTable", 4, tableFlags)) {
        ImGui::TableSetupColumn("##status", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("count", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        ImGui::TableSetupColumn("Auto", ImGuiTableColumnFlags_WidthFixed);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(2);
        ImGui::TextDisabled("count");
        ImGui::TableSetColumnIndex(3);
        ImGui::TextDisabled("Auto");
        tooltip("Auto rows re-run after every edit. Rows with Auto off turn Stale (?) until Check Mesh or Re-check.");

        std::optional<CategoryGroup> lastGroup;
        for (const DefectCategory category : kAllDefectCategories) {
            const CategoryGroup group = categoryInfo(category).group;
            if (lastGroup != group) {
                lastGroup = group;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("%s", group == CategoryGroup::Topology ? "TOPOLOGY" : "GEOMETRY");
            }
            drawHealthRow(category);
        }
        ImGui::EndTable();
    }

    if (!controller_.selectedCategory()) {
        ImGui::TextDisabled("Click a row to highlight it and open its tool.");
    }
    drawMeshInfo();
}

void PolyscopeGui::drawHealthRow(DefectCategory category) {
    const CategoryInfo& info = categoryInfo(category);
    const CategoryHealth& row = controller_.categoryHealth(category);
    ImGui::TableNextRow();
    ImGui::PushID(static_cast<int>(categoryIndex(category)));

    const HealthStatus status = controller_.rowStatus(category);
    ImGui::TableSetColumnIndex(0);
    gui::statusDot(status, info.infoOnly);

    ImGui::TableSetColumnIndex(1);
    const bool selected = controller_.selectedCategory() == category;
    const bool opensShellsPanel = category == DefectCategory::Shells && controller_.shells().multiShell();
    if (ImGui::Selectable(info.name, selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
        if (opensShellsPanel) {
            // Shells are worked on in the Shells panel, not the Tool slot
            showShells_ = true;
            controller_.setColorByShell(true);
        } else {
            controller_.selectCategory(selected ? std::nullopt : std::optional<DefectCategory>(category));
        }
        requestSync();
    }
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 26.0f);
        ImGui::TextUnformatted(info.description);
        if (category == DefectCategory::OpenEdges || category == DefectCategory::NonManifoldEdges) {
            ImGui::TextDisabled("A hole edge has one triangle, a non-manifold edge three or more. "
                                "Watertight meshes have exactly two on every edge.");
        }
        if (opensShellsPanel) {
            ImGui::TextDisabled("Click to open the Shells panel.");
        }
        ImGui::TextDisabled("Status: %s", healthStatusLabel(row.status));
        if (row.result && row.status != HealthStatus::Running) {
            ImGui::TextDisabled("Last check: %.1f ms", row.result->elapsedMs);
        }
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }

    ImGui::TableSetColumnIndex(2);
    std::string count;
    ImVec4 color = gui::kMutedColor;
    const std::size_t n = controller_.rowCount(category);
    switch (status) {
    case HealthStatus::NotRun:
        count = "-";
        break;
    case HealthStatus::Running:
        count = "...";
        color = gui::kRunningColor;
        break;
    case HealthStatus::Ok:
    case HealthStatus::Defects:
        count = withThousands(n);
        color = info.infoOnly ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : (n == 0 ? gui::kOkColor : gui::kDefectColor);
        break;
    case HealthStatus::Stale:
        count = withThousands(n) + "?";
        break;
    }
    const float textWidth = ImGui::CalcTextSize(count.c_str()).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - textWidth);
    ImGui::TextColored(color, "%s", count.c_str());

    ImGui::TableSetColumnIndex(3);
    bool autoCheck = row.autoCheck;
    if (ImGui::Checkbox("##auto", &autoCheck)) {
        controller_.setAutoCheck(category, autoCheck);
        requestSync();
    }
    tooltip("Auto: re-run this check after every mesh edit.");
    ImGui::PopID();
}

void PolyscopeGui::drawMeshInfo() {
    ImGui::SetNextItemOpen(showMeshInfo_);
    showMeshInfo_ = ImGui::CollapsingHeader("Mesh Info");
    if (!showMeshInfo_) {
        return;
    }
    const Mesh& mesh = controller_.mesh();
    const std::string sym(unitSymbol(controller_.fileUnit()));
    const BoundingBox bbox = mesh.boundingBox();
    const Eigen::Vector3d ext = bbox.extents();
    ImGui::Text("Vertices %s   Triangles %s",
                withThousands(static_cast<std::size_t>(mesh.numVertices())).c_str(),
                withThousands(static_cast<std::size_t>(mesh.numFaces())).c_str());
    ImGui::Text("Size: %.4g x %.4g x %.4g %s", ext.x(), ext.y(), ext.z(), sym.c_str());
    ImGui::Text("Min: (%.4g, %.4g, %.4g)", bbox.min.x(), bbox.min.y(), bbox.min.z());
    ImGui::Text("Max: (%.4g, %.4g, %.4g)", bbox.max.x(), bbox.max.y(), bbox.max.z());
    ImGui::Text("Diagonal: %.4g %s", bbox.diagonal(), sym.c_str());
    const CategoryHealth& shells = controller_.categoryHealth(DefectCategory::Shells);
    if (shells.result && shells.status == HealthStatus::Ok) {
        ImGui::Text("Shells: %s", withThousands(shells.result->infoCount).c_str());
    }
}

// -----------------------------------------------------------------------------
// Tool panel template (UI_DESIGN.md §6)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawToolPanel() {
    const std::optional<DefectCategory> category = controller_.selectedCategory();
    if (category) {
        drawCategoryTool(*category);
    }
}

void PolyscopeGui::drawCategoryTool(DefectCategory category) {
    const CategoryInfo& info = categoryInfo(category);
    const CategoryHealth& row = controller_.categoryHealth(category);

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", info.description);
    ImGui::PopStyleColor();
    switch (row.status) {
    case HealthStatus::NotRun:
        ImGui::TextDisabled("Not checked yet. Press Re-check or Check Mesh (F5).");
        break;
    case HealthStatus::Running:
        ImGui::TextColored(gui::kRunningColor, "Checking...");
        break;
    case HealthStatus::Stale:
        ImGui::TextColored(gui::kDefectColor, "Stale: the mesh changed since this check ran.");
        break;
    case HealthStatus::Ok:
    case HealthStatus::Defects:
        break;
    }

    if (ImGui::CollapsingHeader("Settings")) {
        drawCategorySettings(category);
    }

    const DefectBrowser& browser = controller_.browser();
    const CategoryResult* result = browser.result();

    // Optional sub-type filter with counts
    const std::size_t subtypeCount = categorySubtypeCount(category);
    if (result != nullptr && subtypeCount > 0 && !result->empty()) {
        const std::array<std::size_t, 4> counts = browser.subtypeCounts();
        const std::string all = "All (" + withThousands(browser.scopedCount()) + ")";
        if (ImGui::RadioButton(all.c_str(), browser.subtypeFilter() < 0)) {
            controller_.setSubtypeFilter(-1);
            requestSync();
        }
        for (std::size_t t = 0; t < subtypeCount; ++t) {
            const std::string label = std::string(info.subtypes[t]) + " (" + withThousands(counts[t]) + ")";
            // Same line while the button fits, else wrap
            const float labelWidth = ImGui::CalcTextSize(label.c_str()).x + ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x;
            const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
            if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + labelWidth <= right) {
                ImGui::SameLine();
            }
            if (ImGui::RadioButton(label.c_str(), browser.subtypeFilter() == static_cast<int>(t))) {
                controller_.setSubtypeFilter(static_cast<int>(t));
                requestSync();
            }
        }
    }

    ImGui::Separator();
    drawCategoryActions(category);

    if (result == nullptr) {
        return;
    }
    if (info.infoOnly) {
        ImGui::Text("%s %s.", withThousands(result->infoCount).c_str(), info.elementNoun);
    }
    if (browser.scopedCount() == 0) {
        ImGui::TextColored(gui::kOkColor, "No %s found%s.", info.name,
                           result->empty() ? "" : " in the shells in scope");
        return;
    }
    if (category == DefectCategory::OpenEdges && !controller_.shells().scoped()) {
        ImGui::Text("%s hole%s, %s open edges.", withThousands(result->size()).c_str(), result->size() == 1 ? "" : "s",
                    withThousands(result->infoCount).c_str());
    }

    // Histogram: whenever the metric exists and the rows would not fit the list, or a bin is active
    const std::size_t listCap = controller_.listCap();
    std::size_t histogramTotal = 0;
    for (const std::size_t c : browser.histogram().counts) {
        histogramTotal += c;
    }
    const bool showList = browser.showList(listCap);
    if (!browser.histogram().empty() && (histogramTotal > listCap || browser.binFilter())) {
        drawHistogram();
    }

    if (showList) {
        // Fit the rows (at most kDefectListMaxRows) so the Inspector and its Commit button sit right below
        const float rowHeight = ImGui::GetTextLineHeight() + 2.0f * ImGui::GetStyle().CellPadding.y;
        const std::size_t rows = std::min(browser.visibleRows().size(), settings::kDefectListMaxRows);
        drawDefectTable(rowHeight * static_cast<float>(rows + 1) + 2.0f * ImGui::GetStyle().WindowBorderSize + 2.0f);
    } else {
        drawDefectSummary();
    }

    if (browser.visibleRows().size() > controller_.overlayCap() && info.kind == ElementKind::Face) {
        ImGui::TextDisabled("3D: face tint only (%s > overlay cap %s); tubes for the selected defects.",
                            withThousands(browser.visibleRows().size()).c_str(),
                            withThousands(controller_.overlayCap()).c_str());
    }

    // Inspector only for a focus that passes the current filter (FB-13)
    if (browser.hasFocus() && browser.isVisible(browser.focusedElement())) {
        drawInspector();
    }
}

void PolyscopeGui::drawCategorySettings(DefectCategory category) {
    const std::string sym(unitSymbol(controller_.fileUnit()));
    ImGui::PushItemWidth(110.0f);
    switch (category) {
    case DefectCategory::ThinFaces: {
        ThinFaceThresholds thresh = controller_.thresholds();
        bool changed = false;
        if (ImGui::InputDouble(("Min altitude h_min (" + sym + ")").c_str(), &thresh.minAltitude, 0.0, 0.0, "%.0e",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            thresh.minAltitude = std::max(0.0, thresh.minAltitude);
            changed = true;
        }
        if (ImGui::InputDouble(("Min area (" + sym + "^2)").c_str(), &thresh.minArea, 0.0, 0.0, "%.0e",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            thresh.minArea = std::max(0.0, thresh.minArea);
            changed = true;
        }
        if (ImGui::InputDouble("Max aspect ratio (l_max / h_min)", &thresh.maxAspectRatio, 0.0, 0.0, "%.0f",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            thresh.maxAspectRatio = std::max(1.0, thresh.maxAspectRatio);
            changed = true;
        }
        if (changed) {
            controller_.setThresholds(thresh);
            requestSync();
        }
        ImGui::TextDisabled("Press Enter to apply. Auto re-checks immediately.");
        break;
    }
    case DefectCategory::Degenerate: {
        double eps = controller_.degenerateAreaEpsilon();
        if (ImGui::InputDouble("Zero-area epsilon (x diag^2)", &eps, 0.0, 0.0, "%.0e",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            controller_.setDegenerateAreaEpsilon(eps);
            requestSync();
        }
        gui::helpMarker("A face counts as zero-area when its area is below epsilon times the squared bounding-box diagonal.");
        break;
    }
    case DefectCategory::Intersections: {
        double tolerance = controller_.intersectionTolerance();
        if (ImGui::InputDouble("Tolerance (x diagonal)", &tolerance, 0.0, 0.0, "%.0e",
                               ImGuiInputTextFlags_EnterReturnsTrue)) {
            controller_.setIntersectionTolerance(tolerance);
            requestSync();
        }
        gui::helpMarker("Tolerance as a fraction of the bounding-box diagonal. Faces that only touch, or "
                        "cross by less than this length, are not reported. Neighbours sharing an edge are "
                        "reported only when folded flat onto each other.");
        drawIntersectionRepairSettings();
        break;
    }
    case DefectCategory::OpenEdges:
        ImGui::TextDisabled("One row per hole (a loop of open edges); the sub-type is the hole's shape.");
        drawHoleFillSettings();
        break;
    case DefectCategory::FlippedNormals:
        ImGui::TextDisabled("Faces wound against the majority of their shell are listed.");
        break;
    case DefectCategory::NonManifoldEdges:
    case DefectCategory::NonManifoldVertices:
        drawSplitRepairNote(category);
        break;
    default:
        ImGui::TextDisabled("No settings.");
        break;
    }
    ImGui::PopItemWidth();
}

void PolyscopeGui::drawSplitRepairNote(DefectCategory category) {
    ImGui::PushTextWrapPos(0.0f);
    if (category == DefectCategory::NonManifoldEdges) {
        ImGui::TextDisabled("Fix splits Touching edges: the faces around the edge are paired into closed solids "
                            "and each solid gets its own copies of the edge's vertices. Fin / other edges are skipped.");
    } else {
        ImGui::TextDisabled("Fix gives each fan of triangles its own copy of the vertex.");
    }
    ImGui::TextDisabled("Nothing moves: copies sit at the same position, so volume and shape are unchanged. "
                        "Solids that only touched become separate shells.");
    ImGui::PopTextWrapPos();
    gui::helpMarker("Opening an STL already keeps solids that only touch apart, so Touching edges and "
                    "non-manifold vertices show up here only after an edit creates them. Fins and internal walls "
                    "are real defects and are always listed.");
}

void PolyscopeGui::drawCategoryActions(DefectCategory category) {
    const CategoryInfo& info = categoryInfo(category);
    const CategoryHealth& row = controller_.categoryHealth(category);
    const DefectBrowser& browser = controller_.browser();
    const std::size_t count = row.status == HealthStatus::Defects ? controller_.rowCount(category) : 0;

    if (!info.infoOnly) {
        ImGui::BeginDisabled(!info.hasFixAll || count == 0 || !controller_.browsableResult());
        const bool repairAll = category == DefectCategory::Intersections || category == DefectCategory::OpenEdges;
        const std::string fixAllLabel = (repairAll ? "Repair All " : "Fix All ") + withThousands(count) + (repairAll ? "..." : "");
        ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
        if (ImGui::Button(fixAllLabel.c_str())) {
            fixAll(category);
        }
        ImGui::PopStyleColor();
        ImGui::EndDisabled();
        tooltip(!info.hasFixAll  ? "Fix All for this row is planned (P4)."
                : category == DefectCategory::OpenEdges
                    ? "Compare whole-mesh repairs (fill each hole, winding-number rebuild, alpha wrap) with a preview "
                      "and before / after numbers, then apply one."
                : repairAll      ? "Compare whole-mesh repairs (patch each island, self-union, winding-number rebuild, "
                                   "alpha wrap) with a preview and before / after numbers, then apply one."
                : controller_.shells().scoped() ? "Repair every defect of this type in the shells in scope, in one undoable step."
                                                : "Repair every defect of this type in one undoable step.");
        ImGui::SameLine();

        const bool perDefect = category == DefectCategory::ThinFaces || category == DefectCategory::Intersections ||
                               category == DefectCategory::OpenEdges ||
                               category == DefectCategory::NonManifoldEdges ||
                               category == DefectCategory::NonManifoldVertices;
        const bool canFixSelected = perDefect && browser.selectedCount() > 0;
        ImGui::BeginDisabled(!canFixSelected);
        const std::string fixSelLabel = "Fix Selected (" + withThousands(browser.selectedCount()) + ")";
        if (ImGui::Button(fixSelLabel.c_str())) {
            bool fixed = false;
            switch (category) {
            case DefectCategory::ThinFaces:
                fixed = controller_.fixSelectedThinFaces();
                break;
            case DefectCategory::Intersections:
                fixed = controller_.fixSelectedIntersections();
                break;
            case DefectCategory::OpenEdges:
                fixed = controller_.fixSelectedHoles();
                break;
            case DefectCategory::NonManifoldEdges:
                fixed = controller_.fixSelectedNonManifoldEdges();
                break;
            case DefectCategory::NonManifoldVertices:
                fixed = controller_.fixSelectedNonManifoldVertices();
                break;
            default:
                break;
            }
            if (fixed) {
                requestSync(true);
            }
        }
        ImGui::EndDisabled();
        tooltip(category == DefectCategory::ThinFaces
                    ? "Apply each selected face's recommended repair, as one undoable step."
                : category == DefectCategory::Intersections
                    ? "Repair the islands of the selected faces with their recommended method (falling back to the "
                      "others), as one undoable step. Crossings between shells are skipped."
                : category == DefectCategory::OpenEdges
                    ? "Fill the selected holes, each with its recommended method, as one undoable step. "
                      "Holes over the Max hole edges setting and open chains are skipped."
                : category == DefectCategory::NonManifoldEdges
                    ? "Split the selected Touching edges, as one undoable step. Fin / other edges are skipped."
                : category == DefectCategory::NonManifoldVertices
                    ? "Give each fan of the selected vertices its own copy, as one undoable step."
                    : "Per-defect repairs for this row are planned (P4).");
        ImGui::SameLine();
    }
    ImGui::BeginDisabled(row.status == HealthStatus::Running);
    if (ImGui::Button("Re-check")) {
        controller_.recheckCategory(category);
        requestSync();
    }
    ImGui::EndDisabled();
    tooltip("Re-run this check only (even when its Auto is off).");
}

void PolyscopeGui::drawDefectTable(float height) {
    const ImGuiIO& io = ImGui::GetIO();
    const DefectBrowser& browser = controller_.browser();
    const CategoryResult& result = *browser.result();
    const CategoryInfo& info = categoryInfo(result.category);
    const Unit unit = controller_.fileUnit();
    const bool showSubtype = categorySubtypeCount(result.category) > 0;

    int dataColumns = 0;
    while (dataColumns < 3 && info.columns[static_cast<std::size_t>(dataColumns)].name != nullptr) {
        ++dataColumns;
    }
    const int columns = 1 + dataColumns + (showSubtype ? 1 : 0);
    const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                                  ImGuiTableFlags_Sortable | ImGuiTableFlags_SortTristate | ImGuiTableFlags_Resizable;
    // Fixed width for the longest label: auto-fit only measures the rows on screen, and a width saved for
    // shorter labels would clip "#184666" to "#18466". Keying the table on the label length drops old widths.
    const std::string widest = widestElementLabel(info, controller_.mesh(), controller_.shells().size(), result.size());
    const float labelWidth = ImGui::CalcTextSize(widest.c_str()).x + ImGui::GetStyle().ItemSpacing.x;
    ImGui::PushID(static_cast<int>(widest.size()));
    if (!ImGui::BeginTable("DefectTable", columns, flags, ImVec2(0.0f, height))) {
        ImGui::PopID();
        return;
    }
    ImGui::PushTextWrapPos(-1.0f); // cells clip instead of wrapping (the clipper needs equal row heights)
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn(elementColumnName(info.kind), ImGuiTableColumnFlags_WidthFixed, labelWidth, 3);
    for (int c = 0; c < dataColumns; ++c) {
        const std::string header = columnHeader(info.columns[static_cast<std::size_t>(c)], unit);
        ImGui::TableSetupColumn(header.c_str(), ImGuiTableColumnFlags_None, 0.0f, static_cast<ImGuiID>(c));
    }
    if (showSubtype) {
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_NoSort, 0.0f, 9);
    }
    ImGui::TableHeadersRow();

    if (ImGuiTableSortSpecs* specs = ImGui::TableGetSortSpecs(); specs != nullptr && specs->SpecsDirty) {
        if (specs->SpecsCount > 0) {
            const ImGuiTableColumnSortSpecs& spec = specs->Specs[0];
            controller_.setDefectSort(static_cast<int>(spec.ColumnUserID),
                                      spec.SortDirection == ImGuiSortDirection_Descending);
        } else {
            controller_.setDefectSort(-1, false);
        }
        specs->SpecsDirty = false;
    }

    const std::vector<std::size_t>& rows = browser.visibleRows();
    const std::size_t focusPos = browser.hasFocus() ? browser.visiblePositionOf(browser.focusedElement())
                                                    : DefectBrowser::kNone;
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(rows.size()));
    if (scrollListToFocus_ && focusPos != DefectBrowser::kNone) {
        clipper.IncludeItemByIndex(static_cast<int>(focusPos));
    }
    bool clicked = false;
    std::size_t clickedPos = 0;
    bool doubleClicked = false;
    while (clipper.Step()) {
        for (int r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r) {
            const auto pos = static_cast<std::size_t>(r);
            const std::size_t element = rows[pos];
            const DefectElement& e = result.elements[element];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::PushID(r);
            const std::string label = elementLabel(info, e);
            if (ImGui::Selectable(label.c_str(), browser.isSelected(element),
                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
                clicked = true;
                clickedPos = pos;
                doubleClicked = ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
            }
            if (scrollListToFocus_ && pos == focusPos) {
                ImGui::SetScrollHereY(0.5f);
                scrollListToFocus_ = false;
            }
            for (int c = 0; c < dataColumns; ++c) {
                ImGui::TableSetColumnIndex(1 + c);
                const auto ci = static_cast<std::size_t>(c);
                ImGui::TextUnformatted(gui::formatValue(e.columns[ci], info.columns[ci].dimension).c_str());
            }
            if (showSubtype) {
                ImGui::TableSetColumnIndex(1 + dataColumns);
                if (e.subtype >= 0 && e.subtype < 4) {
                    ImGui::TextUnformatted(info.subtypes[static_cast<std::size_t>(e.subtype)]);
                }
            }
            ImGui::PopID();
        }
    }
    ImGui::PopTextWrapPos();
    ImGui::EndTable();
    ImGui::PopID();
    scrollListToFocus_ = false;

    if (clicked) {
        const bool extend = io.KeyCtrl || io.KeySuper || io.KeyShift;
        controller_.clickDefectRow(clickedPos, io.KeyCtrl || io.KeySuper, io.KeyShift);
        requestSync();
        // A plain click flies to the defect (as '.'), so the hand stays on the mouse; extending the selection
        // keeps the camera unless it is a double-click
        if (!extend || doubleClicked) {
            focusCameraOnSelection(FocusNormalSide::PositiveNormal);
        }
    }
    ImGui::TextDisabled("%s listed. Click to fly to one; %s+click, Shift+click to select more.",
                        withThousands(rows.size()).c_str(), gui::kModKey);
}

void PolyscopeGui::drawDefectSummary() {
    const DefectBrowser& browser = controller_.browser();
    const std::size_t visible = browser.visibleRows().size();
    const std::size_t listCap = controller_.listCap();
    ImGui::Spacing();
    ImGui::TextWrapped(
        "%s defects are highlighted in 3D. That is more than the list cap (%s), so no list is shown. "
        "Use Fix All, click a highlighted face, step through them worst first with Next / Prev "
        "(N / Shift+N), or pick a histogram bin. The list appears at %s or fewer.",
        withThousands(visible).c_str(), withThousands(listCap).c_str(), withThousands(listCap).c_str());

    if (ImGui::Button("< Prev")) {
        stepDefectFocus(-1);
    }
    tooltip("Previous defect.  Key: Shift+N");
    ImGui::SameLine();
    if (ImGui::Button("Next >")) {
        stepDefectFocus(+1);
    }
    tooltip("Next defect, worst first.  Key: N");
    ImGui::SameLine();
    if (browser.hasFocus() && browser.isVisible(browser.focusedElement())) {
        ImGui::Text("Defect %s of %s",
                    withThousands(browser.visiblePositionOf(browser.focusedElement()) + 1).c_str(),
                    withThousands(visible).c_str());
    } else {
        ImGui::TextDisabled("None focused");
    }
}

void PolyscopeGui::drawHistogram() {
    const DefectBrowser& browser = controller_.browser();
    const MetricHistogram& histogram = browser.histogram();
    const CategoryInfo& info = categoryInfo(browser.result()->category);
    const std::string unit = gui::dimensionSuffix(info.metricDimension, controller_.fileUnit());
    const std::size_t bins = histogram.numBins();

    ImGui::Text("%s%s%s%s", info.metricName, unit.empty() ? "" : " (", unit.c_str(), unit.empty() ? "" : ")");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", histogram.logScale ? "log scale, click a bar to filter" : "click a bar to filter");

    // Bars: one InvisibleButton per bin so clicks and hovers work (PlotHistogram reports neither)
    const float width = ImGui::GetContentRegionAvail().x;
    const float height = 64.0f;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float binWidth = width / static_cast<float>(bins);
    const auto maxCount = static_cast<float>(std::max<std::size_t>(histogram.maxCount(), 1));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg));
    const std::optional<BinRange>& active = browser.binFilter();
    for (std::size_t b = 0; b < bins; ++b) {
        const auto bf = static_cast<float>(b);
        ImGui::SetCursorScreenPos(ImVec2(origin.x + bf * binWidth, origin.y));
        ImGui::PushID(static_cast<int>(b));
        if (ImGui::InvisibleButton("bin", ImVec2(std::max(binWidth, 1.0f), height)) && histogram.counts[b] > 0) {
            controller_.selectHistogramBin(b, ImGui::GetIO().KeyShift);
            requestSync();
        }
        const bool hovered = ImGui::IsItemHovered();
        ImGui::PopID();
        if (histogram.counts[b] == 0) {
            continue;
        }
        const float barHeight = std::max(2.0f, height * static_cast<float>(histogram.counts[b]) / maxCount);
        const bool inFilter = active && active->contains(b);
        const ImU32 color = ImGui::GetColorU32(inFilter ? gui::kDefectColor
                                                        : (hovered ? ImVec4(0.75f, 0.75f, 0.75f, 1.0f)
                                                                   : ImVec4(0.55f, 0.60f, 0.70f, 1.0f)));
        draw->AddRectFilled(ImVec2(origin.x + bf * binWidth + 1.0f, origin.y + height - barHeight),
                            ImVec2(origin.x + (bf + 1.0f) * binWidth - 1.0f, origin.y + height), color);
        if (hovered) {
            ImGui::SetTooltip("%s %s: %.2e ... %.2e %s", withThousands(histogram.counts[b]).c_str(),
                              categoryInfo(browser.result()->category).elementNoun,
                              histogram.binLower(b), histogram.binUpper(b), unit.c_str());
        }
    }
    ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + height + 2.0f));
    ImGui::TextDisabled("< worst");
    ImGui::SameLine(width - ImGui::CalcTextSize("best >").x);
    ImGui::TextDisabled("best >");

    if (ImGui::ArrowButton("##binLeft", ImGuiDir_Left)) {
        controller_.stepHistogramBin(-1);
        requestSync();
    }
    tooltip("Select the next non-empty bin to the left (worse).");
    ImGui::SameLine();
    if (ImGui::ArrowButton("##binRight", ImGuiDir_Right)) {
        controller_.stepHistogramBin(+1);
        requestSync();
    }
    tooltip("Select the next non-empty bin to the right (better).");
    ImGui::SameLine();
    if (active) {
        char chip[128];
        std::snprintf(chip, sizeof(chip), "%s %.1e ... %.1e (%s)  x##chip", info.metricName,
                      histogram.binLower(active->first), histogram.binUpper(active->last),
                      withThousands(browser.countInBins(*active)).c_str());
        if (ImGui::SmallButton(chip)) {
            controller_.clearHistogramBinFilter();
            requestSync();
        }
        tooltip("Bin filter: the list, 3D highlight, Next / Prev and Fix Selected use only these. Click to remove.");
    } else {
        ImGui::TextDisabled("No bin selected");
    }
}

void PolyscopeGui::drawInspector() {
    const DefectBrowser& browser = controller_.browser();
    const CategoryResult& result = *browser.result();
    const CategoryInfo& info = categoryInfo(result.category);
    const DefectElement& element = result.elements[browser.focusedElement()];
    const std::string header = std::string("Inspector: ") + elementColumnName(info.kind) + " " +
                               elementLabel(info, element) + "###Inspector";
    if (!ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    if (controller_.inspectorActive()) {
        drawThinFaceInspector();
        return;
    }
    if (result.category == DefectCategory::Intersections) {
        drawIntersectionInspector();
        return;
    }
    if (result.category == DefectCategory::OpenEdges && controller_.holeInspectorActive()) {
        drawHoleInspector();
        return;
    }
    const Unit unit = controller_.fileUnit();
    for (std::size_t c = 0; c < 3 && info.columns[c].name != nullptr; ++c) {
        const std::string suffix = gui::dimensionSuffix(info.columns[c].dimension, unit);
        ImGui::Text("%s: %s %s", info.columns[c].name,
                    gui::formatValue(element.columns[c], info.columns[c].dimension).c_str(), suffix.c_str());
    }
    ImGui::Text("Faces: %s", withThousands(result.facesOf(browser.focusedElement()).size()).c_str());
    if (result.category == DefectCategory::Intersections) {
        drawIntersectionPartners();
    }
    if (result.category == DefectCategory::NonManifoldEdges) {
        ImGui::TextWrapped("%s", element.subtype == 0
                                     ? "Touching: solids meet along this edge. Fix Selected splits it without moving anything."
                                     : "Fin / other: a fin, internal wall, odd face count or mixed winding. Splitting "
                                       "vertices cannot fix it; it needs faces removed or re-oriented (planned).");
    } else if (result.category == DefectCategory::NonManifoldVertices) {
        ImGui::TextWrapped("Fix Selected gives each fan its own copy of this vertex, at the same position.");
    } else {
        ImGui::TextDisabled("Per-defect repairs for this row are planned (P4).");
    }
    if (ImGui::Button("Frame (.)")) {
        focusCameraOnSelection(FocusNormalSide::AutoToggleNormal);
    }
}

void PolyscopeGui::drawIntersectionPartners() {
    constexpr std::size_t kMaxPartnerButtons = 20;
    const std::span<const Eigen::Index> partners = controller_.focusedRelatedFaces();
    ImGui::TextUnformatted("Crosses faces (shown in 3D; click to inspect):");
    std::optional<Eigen::Index> clicked;
    for (std::size_t i = 0; i < std::min(partners.size(), kMaxPartnerButtons); ++i) {
        if (i > 0) {
            ImGui::SameLine();
            if (ImGui::GetContentRegionAvail().x < ImGui::GetFontSize() * 4.0f) {
                ImGui::NewLine();
            }
        }
        const std::string label = "#" + std::to_string(partners[i]);
        if (ImGui::SmallButton(label.c_str())) {
            clicked = partners[i];
        }
    }
    if (partners.size() > kMaxPartnerButtons) {
        ImGui::SameLine();
        ImGui::TextDisabled("+%s more", withThousands(partners.size() - kMaxPartnerButtons).c_str());
    }
    if (clicked && controller_.focusRelatedFace(*clicked)) {
        requestSync();
    }
}

void PolyscopeGui::drawThinFaceInspector() {
    const SliverAnalysis& analysis = controller_.selectedFaceAnalysis();
    const SingleFaceRepairPreview& preview = controller_.repairPreview();

    ImGui::Text("Detected type: %s   Max angle: %.3f deg", sliverTypeLabel(analysis.type), analysis.maxAngleDeg);
    ImGui::TextDisabled("Min/max edge ratio: %.3e", analysis.edgeRatio);
    if (analysis.recommendedAction != HealAction::None) {
        ImGui::TextDisabled("Recommended: %s", healActionLabel(analysis.recommendedAction));
    }
    for (const HealActionOption& option : kHealActionOptions) {
        if (ImGui::RadioButton(option.label, controller_.selectedHealAction() == option.action)) {
            controller_.setSelectedHealAction(option.action);
            requestSync();
        }
    }
    if (!preview.statusMessage.empty()) {
        const bool unsafe = !preview.valid && controller_.selectedHealAction() != HealAction::None;
        ImGui::PushStyleColor(ImGuiCol_Text, unsafe ? gui::kErrorColor : gui::kOkColor);
        ImGui::TextWrapped("%s%s", unsafe ? "Warning: " : "", preview.statusMessage.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::TextDisabled("Preview display options: Display > Repair preview.");

    ImGui::BeginDisabled(controller_.selectedHealAction() == HealAction::None);
    if (ImGui::Button("Commit Repair (R)") && controller_.commitSelectedFaceRepair()) {
        requestSync(true);
    }
    ImGui::EndDisabled();
    tooltip("Apply the chosen strategy to this face (one undo step).");
    ImGui::SameLine();
    if (ImGui::Button("Frame (.)")) {
        focusCameraOnSelection(FocusNormalSide::AutoToggleNormal);
    }
    ImGui::SameLine();
    if (ImGui::Button("Back side (Shift+.)")) {
        focusCameraOnSelection(FocusNormalSide::NegativeNormal);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!controller_.canUndo());
    if (ImGui::Button("Undo")) {
        undoLastModification();
    }
    ImGui::EndDisabled();
    if (controller_.canUndo()) {
        tooltip(("Undo " + controller_.lastUndoActionLabel() + "  (" + gui::modShortcut("Z") + ")").c_str());
    }
}

// -----------------------------------------------------------------------------
// Operation: Units & Scale (UI_DESIGN.md §7.1)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawUnitsAndScaleOperator() {
    constexpr float kComboWidth = 180.0f; // both combos share this width so their rows line up

    ImGui::TextWrapped("File units re-interpret the numbers in the file. No vertex moves.");
    Unit fileUnit = controller_.fileUnit();
    if (gui::unitCombo("File units", fileUnit, kComboWidth)) {
        controller_.setFileUnit(fileUnit);
        controller_.postStatusMessage("File units set to " + std::string(unitDisplayName(fileUnit)) +
                                      "; coordinates unchanged.");
        requestSync();
    }
    tooltip("Same setting as the unit button in the status bar. Not a mesh edit (no undo step).");

    ImGui::Separator();
    ImGui::TextWrapped("Scaling multiplies every vertex coordinate.");
    gui::unitCombo("Scale to", pendingScaleTargetUnit_, kComboWidth);
    const double factor = unitConversionFactor(controller_.fileUnit(), pendingScaleTargetUnit_);
    ImGui::Text("Coordinate multiplier: %.6g", factor);

    ImGui::BeginDisabled(!controller_.hasMesh() || pendingScaleTargetUnit_ == controller_.fileUnit());
    if (ImGui::Button("Apply Scale")) {
        controller_.scaleMeshCoordinatesToUnit(pendingScaleTargetUnit_);
        requestSync(true);
        syncMeshStructureToPolyscope(true);
    }
    ImGui::EndDisabled();
    tooltip("One undo step. Auto checks re-run.");
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        controller_.closeTool();
        requestSync();
    }
}

} // namespace ogun
