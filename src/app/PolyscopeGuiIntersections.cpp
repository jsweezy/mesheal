// Intersecting faces: the per-island Inspector, and the Repair All operator for intersections and holes.

#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

struct IslandMethodOption {
    IntersectionRepairMethod method;
    const char* label;
    const char* help;
};

constexpr std::array<IslandMethodOption, 4> kIslandMethodOptions = {{
    {IntersectionRepairMethod::FillFlat, "Remove & Fill Flat (Best for CAD Folds)",
     "Cuts the island and its surrounding rings out and fills the hole using only the boundary vertices. "
     "Keeps flat areas and creases crisp; the smallest change to the surface."},
    {IntersectionRepairMethod::FillSmooth, "Remove & Fill Smooth (Curved Surfaces)",
     "Same cut, then a refined patch faired into the surrounding surface, so curved areas stay curved."},
    {IntersectionRepairMethod::CgalLocalRepair, "CGAL Local Repair (Grazing Kinks)",
     "CGAL's region-growing repair: smooths, then removes and refills a slowly growing region until the "
     "crossings are gone. Often the gentlest fix for small grazing folds."},
    {IntersectionRepairMethod::SelfUnion, "Cut & Merge Walls (Pass-Through)",
     "Self-union of this island's shell: cuts every face along the crossing curves and drops the parts that "
     "end up inside. Uncrossed faces are kept exactly. Fixes every crossing in the shell at once."},
}};

struct GlobalMethodOption {
    GlobalRepairMethod method;
    const char* summary;
    const char* help;
};

constexpr std::array<GlobalMethodOption, 7> kGlobalMethodOptions = {{
    {GlobalRepairMethod::FillHoles, "keeps every other face exactly",
     "Surgical: each hole gets its recommended fill (Flat holes Fill Flat, Curved holes Fill Smooth, cracks "
     "Close Crack), or the method chosen below. Every existing face is kept; holes over the size limit and open "
     "chains stay open."},
    {GlobalRepairMethod::PatchIslands, "keeps every other face exactly",
     "Surgical: each island (crossings within a shell) gets its recommended per-island repair, falling back to "
     "the others. Crossings between shells are left alone."},
    {GlobalRepairMethod::SelfUnion, "cuts along crossings, drops the inside",
     "Exact cut along every crossing (CGAL autorefinement), then keeps only the outer surface by winding number. "
     "Uncrossed faces are kept exactly; internal walls and pockets are removed; new thin faces can appear along the cuts."},
    {GlobalRepairMethod::WindingRebuild, "new surface from a grid",
     "Marching cubes of the winding number on a regular grid: always closed, merges overlaps and closes "
     "holes (the winding number spans them), but every face is new and detail below the grid size is lost."},
    {GlobalRepairMethod::AlphaWrap, "new surface wrapped around the input",
     "CGAL alpha wrap: guaranteed watertight, 2-manifold and free of crossings. Every face is new; cavities "
     "smaller than alpha and shells nested inside others are lost. Holes narrower than about alpha are spanned; "
     "through bigger ones the wrap follows the inside, and an open sheet becomes a thin closed shell."},
    {GlobalRepairMethod::Decimate, "fewer faces, same shape",
     "Garland-Heckbert edge collapse (CGAL) down to the chosen fraction of the faces. Collapses that would flip a "
     "face are skipped; hole rims and the edge of a face selection never move, so the rest of the mesh is kept "
     "exactly."},
    {GlobalRepairMethod::MergeVertices, "welds close vertices",
     "Merges every vertex within the distance of an earlier one (no vertex moves farther than the distance). "
     "Faces that collapse are removed, as are faces that become copies of another. Closes small gaps between "
     "unwelded patches and removes tiny faces; between shells that only touch it creates non-manifold edges."},
}};

/// Length with the current unit ("0.012 mm").
std::string lengthText(double value, Unit unit) {
    if (!std::isfinite(value)) {
        return "-";
    }
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.3g %s", value, std::string(unitSymbol(unit)).c_str());
    return buffer;
}

std::string percentText(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%+.3g%%", value);
    return buffer;
}

} // namespace

// -----------------------------------------------------------------------------
// Island Inspector (same look as the thin-face Inspector)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawIntersectionInspector() {
    drawIntersectionPartners();
    const IntersectionIsland* island = controller_.focusedIsland();
    if (island == nullptr) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("This face only crosses other shells (or is out of scope). Repair All > Self-union with "
                           "\"Also union shells that cross\" merges those shells, and so does Shells > Merge... (a Manifold union).");
        ImGui::PopStyleColor();
    } else {
        const IslandAnalysis& analysis = controller_.focusedIslandAnalysis();
        ImGui::Text("Island: %s faces, %s", withThousands(island->faces.size()).c_str(),
                    island->local ? "local fold" : "pass-through");
        tooltip(island->local ? "The crossing faces form one surface patch: a fold or kink. Cutting it out leaves one hole."
                              : "Two separate parts of the same shell pass through each other: cutting them out would leave "
                                "two holes, so a self-union is the natural fix.");
        ImGui::TextDisabled("Surrounding surface: mean bend %.1f deg, sharpest %.1f deg", analysis.collarMeanDihedralDeg,
                            analysis.collarMaxDihedralDeg);
        ImGui::TextDisabled("Recommended: %s", intersectionRepairMethodLabel(analysis.recommended));
        for (const IslandMethodOption& option : kIslandMethodOptions) {
            if (ImGui::RadioButton(option.label, controller_.selectedIslandMethod() == option.method)) {
                controller_.setSelectedIslandMethod(option.method);
                requestSync();
            }
            tooltip(option.help);
        }
        drawIslandScorecard();
        ImGui::TextDisabled("Preview display options: Display > Repair preview.");
    }

    const IslandRepairPreview& preview = controller_.islandPreview();
    const bool canCommit = island != nullptr && preview.valid && preview.newCrossings == 0;
    ImGui::BeginDisabled(!canCommit);
    if (ImGui::Button("Commit Repair (R)") && controller_.commitIslandRepair()) {
        requestSync(true);
    }
    ImGui::EndDisabled();
    tooltip(canCommit ? "Apply the chosen method to this island (one undo step)."
                      : "Pick a method whose preview is valid and creates no new crossings.");
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

void PolyscopeGui::drawIslandScorecard() {
    const IslandRepairPreview& preview = controller_.islandPreview();
    if (!preview.valid) {
        if (!preview.statusMessage.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, gui::kErrorColor);
            ImGui::TextWrapped("Not possible: %s", preview.statusMessage.c_str());
            ImGui::PopStyleColor();
        }
        return;
    }
    const Unit unit = controller_.fileUnit();
    ImGui::Text("-%s / +%s faces   dV %.3g %s^3   min angle %.1f deg", withThousands(preview.facesRemoved).c_str(),
                withThousands(preview.facesAdded).c_str(), preview.volumeChange, std::string(unitSymbol(unit)).c_str(),
                preview.minNewAngleDeg);
    tooltip("Faces removed / added, the signed volume change of the shell, and the smallest angle of the new "
            "faces (small angles are new slivers).");
    if (preview.newCrossings > 0) {
        ImGui::TextColored(gui::kErrorColor, "Creates %s new intersecting faces: Commit is disabled.",
                           withThousands(preview.newCrossings).c_str());
    } else if (preview.crossingsAfter > 0) {
        ImGui::TextColored(gui::kOkColor, "No new crossings (%s intersecting faces left elsewhere in the shell).",
                           withThousands(preview.crossingsAfter).c_str());
    } else {
        ImGui::TextColored(gui::kOkColor, "No new crossings; the shell is free of crossings.");
    }
    if (!preview.statusMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("%s", preview.statusMessage.c_str());
        ImGui::PopStyleColor();
    }
}

void PolyscopeGui::drawIntersectionRepairSettings() {
    IntersectionRepairSettings settings = controller_.islandRepairSettings();
    bool changed = false;
    ImGui::SeparatorText("Island repair");
    if (ImGui::SliderInt("Rings cut around an island", &settings.dilationRings, 0, settings::kMaxIntersectionDilationRings)) {
        changed = true;
    }
    tooltip("Remove & Fill: how many rings of faces around the crossing faces are cut away too. More rings give "
            "the fill room to bend smoothly; fewer change less.");
    if (ImGui::InputInt("CGAL repair steps", &settings.cgalMaxSteps, 1, 1, ImGuiInputTextFlags_EnterReturnsTrue)) {
        changed = true;
    }
    tooltip("CGAL Local Repair: how many times the region may grow before it gives up.");
    if (changed) {
        controller_.setIslandRepairSettings(settings);
        requestSync();
    }
}

// -----------------------------------------------------------------------------
// Operation: Repair All Intersections (UI_DESIGN.md §7.2)
// -----------------------------------------------------------------------------

void PolyscopeGui::drawRepairAllOperator() {
    if (!controller_.hasMesh()) {
        ImGui::TextDisabled("No mesh loaded.");
        return;
    }
    const DefectCategory target = controller_.globalRepairTarget();
    const CategoryHealth& row = controller_.categoryHealth(target);
    const ToolSlot slot = controller_.activeTool();
    const bool wrapOnly = slot == ToolSlot::AlphaWrap || slot == ToolSlot::Decimate || slot == ToolSlot::MergeVertices;
    if (slot == ToolSlot::AlphaWrap) {
        ImGui::TextWrapped("Wraps %s in a new surface that is always watertight, manifold and free of "
                           "self-intersections. Every face is new: detail smaller than alpha is lost.",
                           controller_.shells().scoped() ? "the shells in scope" : "the whole mesh");
    } else if (slot == ToolSlot::Decimate || slot == ToolSlot::MergeVertices) {
        const std::size_t inScope = controller_.operatorScopeFaceCount();
        const std::string scope = controller_.selectedFaceCount() > 0
            ? "the " + withThousands(inScope) + (inScope == 1 ? " selected face" : " selected faces")
            : controller_.operatorScopeText();
        ImGui::TextWrapped("%s %s.", slot == ToolSlot::Decimate ? "Decimates" : "Merges close vertices in", scope.c_str());
    } else if (target == DefectCategory::OpenEdges) {
        if (row.status == HealthStatus::Defects || row.status == HealthStatus::Ok) {
            const std::size_t holes = controller_.holeCountInScope();
            ImGui::TextWrapped("%s %s%s.", withThousands(holes).c_str(), holes == 1 ? "hole" : "holes",
                               controller_.shells().scoped() ? " (shells in scope)" : "");
        } else {
            ImGui::TextDisabled("Open edges not checked yet. The preview measures them anyway.");
        }
    } else if (row.status == HealthStatus::Defects || row.status == HealthStatus::Ok) {
        const std::size_t islands = controller_.intersectionIslandCount();
        ImGui::TextWrapped("%s intersecting faces, %s %s within shells%s.",
                           withThousands(controller_.rowCount(DefectCategory::Intersections)).c_str(),
                           withThousands(islands).c_str(), islands == 1 ? "island" : "islands",
                           controller_.shells().scoped() ? " (shells in scope)" : "");
    } else {
        ImGui::TextDisabled("Intersecting faces not checked yet (Re-check in its Tool, or Check Mesh). "
                            "The preview measures them anyway.");
    }
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped(wrapOnly ? "Preview shows the result and its numbers before anything is applied."
                                : "Compare surgical and coarse repairs, then apply the one that suits this model. Every "
                                  "method shows its changes before anything is applied.");
    ImGui::PopStyleColor();

    ImGui::SeparatorText("Method");
    const std::span<const GlobalRepairMethod> methods = controller_.globalRepairMethods();
    for (const GlobalMethodOption& option : kGlobalMethodOptions) {
        if (std::find(methods.begin(), methods.end(), option.method) == methods.end()) {
            continue;
        }
        const std::string label = std::string(globalRepairMethodLabel(option.method)) + "##global";
        if (ImGui::RadioButton(label.c_str(), controller_.globalRepairMethod() == option.method)) {
            controller_.setGlobalRepairMethod(option.method);
            requestSync();
        }
        tooltip(option.help);
        ImGui::SameLine();
        ImGui::TextDisabled("%s", option.summary);
        if (controller_.globalRepairMethod() == option.method) {
            ImGui::Indent();
            drawGlobalMethodParams(option.method);
            ImGui::Unindent();
        }
    }

    const bool running = controller_.globalRepairRunning();
    ImGui::BeginDisabled(running);
    if (ImGui::Button("Preview")) {
        controller_.previewGlobalRepair();
        requestSync();
    }
    tooltip("Compute the selected method on a copy of the mesh (in the background).");
    if (methods.size() > 1) {
        ImGui::SameLine();
        if (ImGui::Button("Compare all")) {
            controller_.compareAllGlobalRepairs();
            requestSync();
        }
        tooltip("Compute every method, one after another, and list them below.");
    }
    ImGui::EndDisabled();
    if (running) {
        ImGui::SameLine();
        const std::optional<GlobalRepairMethod> current = controller_.globalRepairCurrentMethod();
        ImGui::TextColored(gui::kRunningColor, "%s...", current ? globalRepairMethodLabel(*current) : "Working");
    }

    drawGlobalCandidateTable();

    const RepairCandidate* shown = controller_.shownGlobalCandidate();
    ImGui::SeparatorText("Preview");
    int show = static_cast<int>(controller_.globalPreviewShow());
    bool showChanged = ImGui::RadioButton("Result", &show, static_cast<int>(GlobalPreviewShow::Result));
    ImGui::SameLine();
    showChanged |= ImGui::RadioButton("Original", &show, static_cast<int>(GlobalPreviewShow::Original));
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
    if (shown != nullptr) {
        const RepairMetrics& m = shown->metrics;
        ImGui::TextWrapped("%s: %.1f%% of faces unchanged, shells %s -> %s, thin faces %s -> %s, %.2f s.",
                           globalRepairMethodLabel(shown->method), 100.0 * m.unchangedFraction,
                           withThousands(m.shellsBefore).c_str(), withThousands(m.shellsAfter).c_str(),
                           withThousands(m.thinFacesBefore).c_str(), withThousands(m.thinFacesAfter).c_str(),
                           m.elapsedMs / 1000.0);
        if (!shown->statusMessage.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("%s", shown->statusMessage.c_str());
            ImGui::PopStyleColor();
        }
        if (!controller_.candidateIsCurrent(*shown)) {
            ImGui::TextColored(gui::kDefectColor, "Computed with other settings: Preview again to update.");
        }
    }

    ImGui::Separator();
    const bool canApply = shown != nullptr && shown->success && !running;
    ImGui::BeginDisabled(!canApply);
    ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
    const std::string applyLabel = canApply ? std::string("Apply ") + globalRepairMethodLabel(shown->method) : std::string("Apply");
    if (ImGui::Button(applyLabel.c_str()) && controller_.applyGlobalRepair()) {
        requestSync(true);
    }
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    tooltip("Replace the mesh with the shown result (one undo step). The Auto rows and Intersecting faces are re-checked.");
    ImGui::SameLine();
    if (ImGui::Button("Close")) {
        controller_.closeTool();
        requestSync();
    }
    tooltip("Close without changing the mesh.  Key: Esc");
}

void PolyscopeGui::drawGlobalMethodParams(GlobalRepairMethod method) {
    GlobalRepairParams params = controller_.globalRepairParams();
    bool changed = false;
    ImGui::PushItemWidth(140.0f);
    switch (method) {
    case GlobalRepairMethod::FillHoles: {
        constexpr std::array<HoleFillMethod, 5> kChoices = {HoleFillMethod::None, HoleFillMethod::Flat, HoleFillMethod::Refine,
                                                            HoleFillMethod::Smooth, HoleFillMethod::CloseCrack};
        const auto label = [](HoleFillMethod m) { return m == HoleFillMethod::None ? "Recommended per hole" : holeFillMethodLabel(m); };
        if (ImGui::BeginCombo("Fill", label(params.holeMethod))) {
            for (const HoleFillMethod choice : kChoices) {
                if (ImGui::Selectable(label(choice), params.holeMethod == choice)) {
                    params.holeMethod = choice;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        tooltip("Recommended: Flat holes get Fill Flat, Curved holes Fill Smooth, cracks Close Crack. A single method "
                "applies to every hole (holes it cannot close stay open).");
        const HoleFillSettings& holes = controller_.holeFillSettings();
        ImGui::TextDisabled("Limits (Open edges tool settings): max hole edges %s, max refined faces %s",
                            holes.maxHoleEdges > 0 ? withThousands(static_cast<std::size_t>(holes.maxHoleEdges)).c_str() : "none",
                            withThousands(holes.maxPatchFaces).c_str());
        break;
    }
    case GlobalRepairMethod::FixAll:
        break; // Fix All's settings are on its own panel
    case GlobalRepairMethod::PatchIslands:
        ImGui::TextDisabled("Island settings: rings %d, CGAL steps %d",
                            controller_.islandRepairSettings().dilationRings, controller_.islandRepairSettings().cgalMaxSteps);
        break;
    case GlobalRepairMethod::SelfUnion:
        changed |= ImGui::Checkbox("Also union shells that cross each other", &params.unionCrossingShells);
        tooltip("On: crossing shells merge into one. Off: each shell is united on its own and shells stay apart.");
        break;
    case GlobalRepairMethod::WindingRebuild:
        changed |= ImGui::SliderInt("Grid cells (longest side)", &params.windingResolution, settings::kMinWindingGridResolution,
                                    settings::kMaxWindingGridResolution, "%d", ImGuiSliderFlags_Logarithmic);
        tooltip("More cells keep more detail but take longer and give more faces.");
        changed |= ImGui::Checkbox("Simplify to the original face count##winding", &params.simplifyRebuild);
        break;
    case GlobalRepairMethod::Decimate: {
        changed |= ImGui::SliderScalar("Ratio (faces kept)", ImGuiDataType_Double, &params.decimateRatio,
                                       &settings::kMinDecimateRatio, &settings::kMaxDecimateRatio, "%.2f",
                                       ImGuiSliderFlags_Logarithmic);
        tooltip("Fraction of the faces to keep: 0.5 halves them. Ctrl+click (Control, not Cmd) to type a value.");
        const std::size_t faces = controller_.operatorScopeFaceCount();
        ImGui::TextDisabled("%s %s -> about %s", withThousands(faces).c_str(), faces == 1 ? "face" : "faces",
                            withThousands(static_cast<std::size_t>(std::lround(params.decimateRatio * static_cast<double>(faces)))).c_str());
        break;
    }
    case GlobalRepairMethod::MergeVertices: {
        // The number is in file coordinates; the unit is only a label
        double distance = params.mergeDistance;
        const std::string label = "Distance (" + std::string(unitSymbol(controller_.fileUnit())) + ")";
        const double lo = settings::kMinMergeDistance;
        const double hi = settings::kMaxMergeDistance;
        if (ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &distance, &lo, &hi, "%.4g", ImGuiSliderFlags_Logarithmic)) {
            params.mergeDistance = std::clamp(distance, settings::kMinMergeDistance, settings::kMaxMergeDistance);
            changed = true;
        }
        tooltip("Vertices closer than this are merged (default 0.05; 0.0001 to 10, log scale; the unit is only a label). "
                "Ctrl+click (Control, not Cmd) to type a value.");
        changed |= ImGui::Checkbox("Keep surfaces apart", &params.mergeKeepSurfacesApart);
        tooltip("On: two vertices merge only when an edge joins them (a short edge collapses) or both lie "
                "on open edges (a crack is welded). Surfaces and shells that touch stay apart. Off: any "
                "close vertices merge, like Blender's Merge by Distance, which pinches touching surfaces "
                "into non-manifold edges.");
        const double meanEdge = controller_.meanEdgeLength();
        if (meanEdge > 0.0) {
            const double ratio = distance / meanEdge;
            if (ratio < 1.0) {
                ImGui::TextDisabled("mean edge %s (the distance is %.0f%% of it)", lengthText(meanEdge, controller_.fileUnit()).c_str(), 100.0 * ratio);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Text, gui::kDefectColor);
                ImGui::TextWrapped("mean edge %s: the distance is %.3gx that, so %s", lengthText(meanEdge, controller_.fileUnit()).c_str(), ratio,
                                   ratio < settings::kMergeCollapseWarnRatio ? "many edges will collapse" : "whole areas will collapse");
                ImGui::PopStyleColor();
            }
        }
        break;
    }
    case GlobalRepairMethod::AlphaWrap: {
        changed |= ImGui::SliderScalar("Detail (diagonal / alpha)", ImGuiDataType_Double, &params.alphaWrapDetail,
                                       &settings::kMinAlphaWrapDetail, &settings::kMaxAlphaWrapDetail, "%.0f",
                                       ImGuiSliderFlags_Logarithmic);
        const double diag = controller_.meshBounds().diagonal();
        const double alpha = alphaWrapAlpha(params, diag);
        tooltip("Higher detail = smaller alpha = a tighter wrap, but much slower on big meshes. Gaps and holes "
                "narrower than about alpha are closed; wider ones are followed inside.");
        ImGui::TextDisabled("alpha = %s", lengthText(alpha, controller_.fileUnit()).c_str());

        changed |= ImGui::SliderScalar("Offset (alpha / x)", ImGuiDataType_Double, &params.alphaWrapOffsetRatio,
                                       &settings::kMinAlphaWrapOffsetRatio, &settings::kMaxAlphaWrapOffsetRatio, "x = %.0f");
        tooltip("The offset is how far outside the input the wrap lies: offset = alpha / x, so it follows "
                "alpha. CGAL suggests x = 30. A higher x follows the input more closely (the volume grows less) "
                "but makes more faces and takes longer; a lower one rounds edges off. Ctrl+click (Control, not "
                "Cmd) to type a value.");
        ImGui::TextDisabled("offset = %s (alpha / %.0f)", lengthText(alphaWrapOffset(params, alpha), controller_.fileUnit()).c_str(),
                            params.alphaWrapOffsetRatio);
        changed |= ImGui::Checkbox("Simplify to the original face count##wrap", &params.simplifyRebuild);
        break;
    }
    }
    ImGui::PopItemWidth();
    if (changed) {
        controller_.setGlobalRepairParams(params);
    }
}

void PolyscopeGui::drawGlobalCandidateTable() {
    const std::vector<RepairCandidate>& candidates = controller_.globalCandidates();
    if (candidates.empty()) {
        ImGui::TextDisabled("No previews yet.");
        return;
    }
    const Unit unit = controller_.fileUnit();
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_ScrollX |
                                  ImGuiTableFlags_SizingFixedFit;
    // Sized to its rows: with ScrollX the table would otherwise fill the rest of the panel
    const float height = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(candidates.size() + 1) +
                         ImGui::GetStyle().ScrollbarSize + 2.0f * ImGui::GetStyle().CellPadding.y;
    if (!ImGui::BeginTable("RepairCandidates", 10, flags, ImVec2(0.0f, height))) {
        return;
    }
    ImGui::TableSetupColumn("Method");
    ImGui::TableSetupColumn("Shells");
    const bool holes = controller_.globalRepairTarget() == DefectCategory::OpenEdges;
    ImGui::TableSetupColumn("Closed");
    ImGui::TableSetupColumn(holes ? "Open edges" : "Crossings");
    ImGui::TableSetupColumn("dV");
    ImGui::TableSetupColumn("dA");
    ImGui::TableSetupColumn("Max dev");
    ImGui::TableSetupColumn("Lost");
    ImGui::TableSetupColumn("Faces");
    ImGui::TableSetupColumn("Time");
    ImGui::TableHeadersRow();
    const char* headerHelp[] = {
        "Click a row to show that result.",
        "Shells in scope: before -> after (a merge or a deletion lowers it).",
        "Watertight and manifold: no open and no non-manifold edges.",
        holes ? "Open edges after (before)." : "Intersecting faces after (before).",
        "Volume change (sum of signed shell volumes: overlapping shells count twice before a union).",
        "Surface area change.",
        "Farthest point of the result from the original surface: how far new surface strays.",
        "Farthest point of the original from the result: removed walls, filled pockets, lost detail.",
        "Faces after (before) in the shells in scope.",
        "Time to compute the repair.",
    };
    for (int c = 0; c < 10; ++c) {
        ImGui::TableSetColumnIndex(c);
        ImGui::PushID(c);
        ImGui::TableHeader(ImGui::TableGetColumnName(c));
        tooltip(headerHelp[c]);
        ImGui::PopID();
    }

    const RepairCandidate* shown = controller_.shownGlobalCandidate();
    std::optional<GlobalRepairMethod> clicked;
    for (const RepairCandidate& c : candidates) {
        const RepairMetrics& m = c.metrics;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushID(static_cast<int>(c.method));
        const bool current = controller_.candidateIsCurrent(c);
        if (!current || !c.success) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        }
        if (ImGui::Selectable(globalRepairMethodLabel(c.method), shown == &c, ImGuiSelectableFlags_SpanAllColumns)) {
            clicked = c.method;
        }
        if (!c.success) {
            tooltip(c.statusMessage.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted("failed");
        } else {
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%s -> %s", withThousands(m.shellsBefore).c_str(), withThousands(m.shellsAfter).c_str());
            ImGui::TableSetColumnIndex(2);
            if (m.watertight()) {
                ImGui::TextColored(gui::kOkColor, "yes");
            } else {
                ImGui::TextColored(gui::kDefectColor, "no");
                tooltip((withThousands(m.openEdgesAfter) + " open, " + withThousands(m.nonManifoldEdgesAfter) +
                         " non-manifold edges").c_str());
            }
            ImGui::TableSetColumnIndex(3);
            if (holes) {
                ImGui::TextColored(m.openEdgesAfter == 0 ? gui::kOkColor : gui::kDefectColor, "%s (%s)",
                                   withThousands(m.openEdgesAfter).c_str(), withThousands(m.openEdgesBefore).c_str());
                tooltip((withThousands(m.intersectingFacesAfter) + " intersecting faces after (" +
                         withThousands(m.intersectingFacesBefore) + " before)").c_str());
            } else if (m.intersectingFacesAfter == 0) {
                ImGui::TextColored(gui::kOkColor, "0 (%s)", withThousands(m.intersectingFacesBefore).c_str());
            } else {
                ImGui::TextColored(gui::kDefectColor, "%s (%s)", withThousands(m.intersectingFacesAfter).c_str(),
                                   withThousands(m.intersectingFacesBefore).c_str());
            }
            ImGui::TableSetColumnIndex(4);
            ImGui::TextUnformatted(percentText(m.volumeChangePercent()).c_str());
            ImGui::TableSetColumnIndex(5);
            ImGui::TextUnformatted(percentText(m.areaChangePercent()).c_str());
            ImGui::TableSetColumnIndex(6);
            ImGui::TextUnformatted(lengthText(m.maxDeviation, unit).c_str());
            ImGui::TableSetColumnIndex(7);
            ImGui::TextUnformatted(lengthText(m.maxLost, unit).c_str());
            ImGui::TableSetColumnIndex(8);
            ImGui::Text("%s (%s)", withThousands(static_cast<std::size_t>(m.facesAfter)).c_str(),
                        withThousands(static_cast<std::size_t>(m.facesBefore)).c_str());
            ImGui::TableSetColumnIndex(9);
            ImGui::Text("%.2f s", m.elapsedMs / 1000.0);
        }
        if (!current || !c.success) {
            ImGui::PopStyleColor();
        }
        ImGui::PopID();
    }
    ImGui::EndTable();
    if (clicked) {
        controller_.showGlobalCandidate(*clicked);
        requestSync();
    }
}

} // namespace ogun
