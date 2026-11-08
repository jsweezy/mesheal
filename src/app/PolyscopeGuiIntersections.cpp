// Intersecting faces: the per-island Inspector, and the Repair All operator for intersections and holes.

#include "app/FixAllPanelModel.hpp"
#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"
#include "repair/SurfaceRebuild.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

namespace mesheal {

using gui::dimensionSuffix;
using gui::signedPercent;
using gui::signedThousands;
using gui::significant;
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

const char* displayGlobalMethodName(GlobalRepairMethod method) noexcept {
    if (method == GlobalRepairMethod::AlphaWrap) {
        return "Shrink-Wrap (Outer Envelope)";
    }
    return globalRepairMethodLabel(method);
}

std::string lengthText(double value, Unit unit);

// Round distances, 1-2-5 per decade (0.0001, 0.0002, 0.0005, 0.001 ... 10), within [lo, hi]
std::vector<double> roundDistanceSteps(double lo, double hi) {
    std::vector<double> steps;
    for (double decade = std::pow(10.0, std::floor(std::log10(lo))); decade <= hi * 1.0001; decade *= 10.0) {
        for (const double m : {1.0, 2.0, 5.0}) {
            const double v = m * decade;
            if (v >= lo * 0.9999 && v <= hi * 1.0001) {
                steps.push_back(v);
            }
        }
    }
    return steps;
}

// A slider that snaps to round distances, with a box beside it for an exact value. Returns true on change.
bool roundDistanceInput(const std::string& label, double& value, double lo, double hi) {
    const std::vector<double> steps = roundDistanceSteps(lo, hi);
    // Nearest step on a log scale, so a typed value still places the handle sensibly
    std::size_t nearest = 0;
    for (std::size_t i = 1; i < steps.size(); ++i) {
        if (std::abs(std::log(steps[i] / value)) < std::abs(std::log(steps[nearest] / value))) {
            nearest = i;
        }
    }
    const float full = ImGui::CalcItemWidth();
    const float boxWidth = std::min(90.0f, full * 0.4f);
    bool changed = false;
    char shown[32];
    std::snprintf(shown, sizeof(shown), "%g", steps[nearest]);
    int index = static_cast<int>(nearest);
    ImGui::PushItemWidth(full - boxWidth - ImGui::GetStyle().ItemInnerSpacing.x);
    if (ImGui::SliderInt(("##steps" + label).c_str(), &index, 0, static_cast<int>(steps.size()) - 1, shown,
                         ImGuiSliderFlags_NoInput)) {
        value = steps[static_cast<std::size_t>(index)];
        changed = true;
    }
    ImGui::PopItemWidth();
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::PushItemWidth(boxWidth);
    double typed = value;
    if (ImGui::InputDouble(label.c_str(), &typed, 0.0, 0.0, "%.6g", ImGuiInputTextFlags_EnterReturnsTrue) && typed > 0.0) {
        value = std::clamp(typed, lo, hi);
        changed = true;
    }
    ImGui::PopItemWidth();
    return changed;
}

// "shortest edge 0.012 mm": merging or welding past it collapses real edges, not just near-duplicates
void edgeLengthHints(const AppController& controller, double distance, bool warn, bool keepSurfacesApart) {
    const double meanEdge = controller.meanEdgeLength();
    const double shortEdge = controller.shortestEdgeLength();
    if (meanEdge <= 0.0) {
        return;
    }
    const double ratio = distance / meanEdge;
    ImGui::TextDisabled("= %.3gx mean edge (%s)", ratio, lengthText(meanEdge, controller.fileUnit()).c_str());
    if (shortEdge > 0.0) {
        ImGui::TextDisabled("shortest edge in scope %s", lengthText(shortEdge, controller.fileUnit()).c_str());
    }
    if (!warn) {
        return;
    }
    if (ratio >= 1.0) {
        ImGui::PushStyleColor(ImGuiCol_Text, gui::defectColor());
        ImGui::TextWrapped("Above the mean edge, so %s",
                           ratio < settings::kMergeCollapseWarnRatio ? "many edges will collapse" : "whole areas will collapse");
        ImGui::PopStyleColor();
    } else if (shortEdge > 0.0 && distance > shortEdge) {
        const std::size_t shortEdges = controller.edgesShorterThanCount(distance);
        ImGui::TextDisabled("Edges shorter than %s in scope: %s",
                            lengthText(distance, controller.fileUnit()).c_str(), withThousands(shortEdges).c_str());
    } else if (shortEdge > 0.0 && keepSurfacesApart) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Below the shortest edge: with Keep surfaces apart on, nothing will merge.");
        ImGui::PopTextWrapPos();
    }
}

constexpr std::array<GlobalMethodOption, 11> kGlobalMethodOptions = {{
    {GlobalRepairMethod::FillHoles, "keeps every other face exactly",
     "Surgical: each hole gets its recommended fill (Flat holes Fill Flat, Curved holes Fill Smooth, cracks "
     "Close Crack), or the method chosen below. Every existing face is kept; holes over the size limit and open "
     "chains stay open."},
    {GlobalRepairMethod::HealThinFaces, "flips and collapses, then a clean-up pass",
     "Every thin / sliver face gets its recommended heal (edge flip, or collapse of its short edge), repeated "
     "until no more improve, then a clean-up pass that keeps feature creases. Collapses move vertices: check the "
     "volume change and the deviation before applying."},
    {GlobalRepairMethod::PatchIslands, "keeps every other face exactly",
     "Surgical: each island (crossings within a shell) gets its recommended per-island repair, falling back to "
     "the others. Crossings between shells are left alone."},
    {GlobalRepairMethod::SelfUnion, "cuts along crossings, drops the inside",
     "Exact cut along every crossing (CGAL autorefinement), then keeps only the outer surface by winding number. "
     "Uncrossed faces are kept exactly; internal walls and pockets are removed; new thin faces can appear along the cuts."},
    {GlobalRepairMethod::WindingRebuild, "new surface from a grid",
     "Marching cubes of the winding number on a regular grid: always closed, merges overlaps and closes "
     "holes (the winding number spans them), but every face is new and detail below the grid size is lost."},
    {GlobalRepairMethod::AlphaWrap, "watertight outer casing around the geometry",
     "Best for 3D printing external shells and enveloping thin, zero-volume sheets. Creates a guaranteed "
     "watertight outer casing around the geometry. Warning: This will slightly inflate the model and "
     "completely fill in any enclosed internal voids or hollow cavities."},
    {GlobalRepairMethod::VoxelHealRemesh, "uniform 3D grid + Taubin de-terracing + isotropic remesh",
     "Best for organic models, 3D scans, and rapid manifold conversion. Rebuilds the model using a uniform "
     "3D grid to quickly resolve all self-intersections, stitch holes, and generate even, uniform triangles. "
     "Preserves internal hollow cavities. Note: Very sharp mechanical edges may be slightly softened."},
    {GlobalRepairMethod::PrecisionIsosurface, "exact level-set Delaunay meshing (>= 30 deg angles)",
     "Best for engineering parts, CAD exports, and finite element simulation. Traces the exact mathematical "
     "boundary of the volume using Delaunay refinement. Generates high-quality, simulation-ready triangles "
     "(guaranteed >= 30 deg angles) that adaptively shrink to hug tight curves and mechanical fillets without "
     "stair-stepping. Slower than Voxel Remesh."},
    {GlobalRepairMethod::Decimate, "fewer faces, same shape",
     "Garland-Heckbert edge collapse (CGAL) down to the chosen fraction of the faces. Collapses that would flip a "
     "face are skipped; hole rims and the edge of a face selection never move, so the rest of the mesh is kept "
     "exactly."},
    {GlobalRepairMethod::MergeVertices, "merges close vertices",
     "Merges every vertex within the distance of an earlier one (no vertex moves farther than the distance). "
     "Faces that collapse are removed, as are faces that become copies of another. By default preserves open "
     "edges and cracks so seams are not altered; uncheck to allow crack merging."},
    {GlobalRepairMethod::Weld, "welds open edges across cracks",
     "Welds vertices on open boundary edges together across cracks and seams within the distance threshold. "
     "Interior vertices and triangles are never altered or collapsed."},
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
            ImGui::PushStyleColor(ImGuiCol_Text, gui::errorColor());
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
        ImGui::TextColored(gui::errorColor(), "Creates %s new intersecting faces: Commit is disabled.",
                           withThousands(preview.newCrossings).c_str());
    } else if (preview.crossingsAfter > 0) {
        ImGui::TextColored(gui::okColor(), "No new crossings (%s intersecting faces left elsewhere in the shell).",
                           withThousands(preview.crossingsAfter).c_str());
    } else {
        ImGui::TextColored(gui::okColor(), "No new crossings; the shell is free of crossings.");
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
    if (slot == ToolSlot::AlphaWrap) {
        ImGui::TextWrapped("Best for 3D printing external shells and enveloping thin, zero-volume sheets (%s). "
                           "Creates a guaranteed watertight outer casing around the geometry. Warning: This will "
                           "slightly inflate the model and completely fill in any enclosed internal voids or hollow cavities.",
                           controller_.shells().scoped() ? "the shells in scope" : "the whole mesh");
    } else if (slot == ToolSlot::VoxelHealRemesh) {
        ImGui::TextWrapped("Best for organic models, 3D scans, and rapid manifold conversion (%s). Rebuilds the model "
                           "using a uniform 3D grid to quickly resolve all self-intersections, stitch holes, and generate "
                           "even, uniform triangles. Preserves internal hollow cavities. Note: Very sharp mechanical edges "
                           "may be slightly softened.",
                           controller_.shells().scoped() ? "the shells in scope" : "the whole mesh");
    } else if (slot == ToolSlot::PrecisionIsosurface) {
        ImGui::TextWrapped("Best for engineering parts, CAD exports, and finite element simulation (%s). Traces the exact "
                           "mathematical boundary of the volume using Delaunay refinement. Generates high-quality, "
                           "simulation-ready triangles (guaranteed >= 30 deg angles) that adaptively shrink to hug tight "
                           "curves and mechanical fillets without stair-stepping. Slower than Voxel Remesh.",
                           controller_.shells().scoped() ? "the shells in scope" : "the whole mesh");
    }
    if ((slot == ToolSlot::AlphaWrap || slot == ToolSlot::VoxelHealRemesh || slot == ToolSlot::PrecisionIsosurface) &&
        controller_.shells().scoped()) {
        ImGui::PushStyleColor(ImGuiCol_Text, gui::defectColor());
        ImGui::TextWrapped("Scope: %s of %s shells selected (unselected shells are left untouched).",
                           withThousands(controller_.shells().selectedCount()).c_str(),
                           withThousands(controller_.shells().size()).c_str());
        ImGui::PopStyleColor();
        if (ImGui::SmallButton("Use whole mesh")) {
            controller_.showAllShells();
            controller_.selectShellsWhere([](std::size_t) { return false; });
            requestSync();
        }
        tooltip("Clear the shell selection so the rebuild unites and remeshes all shells together.");
    } else if (slot == ToolSlot::Decimate || slot == ToolSlot::MergeVertices || slot == ToolSlot::Weld) {
        const std::size_t inScope = controller_.operatorScopeFaceCount();
        const std::string scope = controller_.selectedFaceCount() > 0
            ? "the " + withThousands(inScope) + (inScope == 1 ? " selected face" : " selected faces")
            : controller_.operatorScopeText();
        ImGui::TextWrapped("%s %s.",
                           slot == ToolSlot::Decimate ? "Decimates"
                           : slot == ToolSlot::Weld   ? "Welds open boundary edges in"
                                                      : "Merges close vertices in",
                           scope.c_str());
    } else if (target == DefectCategory::ThinFaces) {
        if (row.status == HealthStatus::Defects || row.status == HealthStatus::Ok) {
            ImGui::TextWrapped("%s thin / sliver faces%s.", withThousands(controller_.rowCount(DefectCategory::ThinFaces)).c_str(),
                               controller_.shells().scoped() ? " (shells in scope)" : "");
        } else {
            ImGui::TextDisabled("Thin / sliver faces not checked yet. The preview measures them anyway.");
        }
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
    const std::span<const GlobalRepairMethod> methods = controller_.globalRepairMethods();
    if (methods.size() > 1) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("Compare surgical and coarse repairs, then apply the one that suits this model.");
        ImGui::PopStyleColor();
        ImGui::SeparatorText("Method");
    }
    for (const GlobalMethodOption& option : kGlobalMethodOptions) {
        if (std::find(methods.begin(), methods.end(), option.method) == methods.end()) {
            continue;
        }
        if (methods.size() == 1) {
            // A lone method needs no picker: just its settings
            drawGlobalMethodParams(option.method);
            continue;
        }
        const std::string label = std::string(displayGlobalMethodName(option.method)) + "##global";
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
        ImGui::TextColored(gui::runningColor(), "%s...", current ? displayGlobalMethodName(*current) : "Working");
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
    drawWholeMeshPreviewControls(true);
    if (shown != nullptr) {
        const RepairMetrics& m = shown->metrics;
        ImGui::TextWrapped("%s: %.1f%% of faces changed, shells %s -> %s, thin faces %s -> %s, %.2f s.",
                           displayGlobalMethodName(shown->method), 100.0 * (1.0 - m.unchangedFraction),
                           withThousands(m.shellsBefore).c_str(), withThousands(m.shellsAfter).c_str(),
                           withThousands(m.thinFacesBefore).c_str(), withThousands(m.thinFacesAfter).c_str(),
                           m.elapsedMs / 1000.0);
        const std::string volumeUnit = dimensionSuffix(ValueDimension::Volume, controller_.fileUnit());
        const bool bigVolumeChange = std::abs(m.volumeChangePercent()) >= settings::kVolumeChangeWarnPercent;
        ImGui::PushStyleColor(ImGuiCol_Text, bigVolumeChange ? gui::defectColor() : ImGui::GetStyleColorVec4(ImGuiCol_Text));
        ImGui::TextWrapped("Volume %s -> %s %s (%s), faces %s -> %s.", significant(m.volumeBefore).c_str(),
                           significant(m.volumeAfter).c_str(), volumeUnit.c_str(), percentText(m.volumeChangePercent()).c_str(),
                           withThousands(static_cast<std::size_t>(m.facesBefore)).c_str(),
                           withThousands(static_cast<std::size_t>(m.facesAfter)).c_str());
        ImGui::PopStyleColor();
        if (bigVolumeChange) {
            tooltip("A large volume change: the repair may have removed or flattened real geometry (thin walls, "
                    "small features). Compare Result and Original, or Color by deviation, before applying.");
        }
        if (m.intersectingFacesAfter > m.intersectingFacesBefore) {
            ImGui::PushStyleColor(ImGuiCol_Text, gui::defectColor());
            ImGui::TextWrapped("Intersecting faces %s -> %s: the repair makes faces cross.",
                               withThousands(m.intersectingFacesBefore).c_str(), withThousands(m.intersectingFacesAfter).c_str());
            ImGui::PopStyleColor();
        }
        if (!shown->statusMessage.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("%s", shown->statusMessage.c_str());
            ImGui::PopStyleColor();
        }
        if (!controller_.candidateIsCurrent(*shown)) {
            ImGui::TextColored(gui::defectColor(), "Computed with other settings: Preview again to update.");
        }
    }

    ImGui::Separator();
    const bool canApply = shown != nullptr && shown->success && !running;
    ImGui::BeginDisabled(!canApply);
    ImGui::PushStyleColor(ImGuiCol_Button, gui::kFixAllButtonColor);
    const std::string applyLabel = canApply ? std::string("Apply ") + displayGlobalMethodName(shown->method) : std::string("Apply");
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
    case GlobalRepairMethod::HealThinFaces:
        ImGui::TextDisabled("Limits: the Thin / sliver faces tool settings.");
        break;
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
        const std::string label = "Distance (" + std::string(unitSymbol(controller_.fileUnit())) + ")";
        changed |= roundDistanceInput(label, params.mergeDistance, settings::kMinMergeDistance, settings::kMaxMergeDistance);
        tooltip("Vertices closer than this are merged (default 0.05; the unit is only a label). The slider snaps "
                "to round values; type in the box (Enter) for an exact one.");
        changed |= ImGui::Checkbox("Preserve open edges / cracks", &params.mergePreserveOpenEdges);
        tooltip("On (default): open edges and cracks are protected and will not merge across seams. To stitch "
                "open seams, use Mesh > Weld. Off: vertices across open edges can merge.");
        changed |= ImGui::Checkbox("Keep surfaces apart", &params.mergeKeepSurfacesApart);
        tooltip("On (default): two vertices merge only when an edge joins them (a short edge collapses). Surfaces "
                "and shells that touch stay apart. Off: any close vertices merge, like Blender's Merge by Distance, "
                "which pinches touching surfaces into non-manifold edges.");
        edgeLengthHints(controller_, params.mergeDistance, true, params.mergeKeepSurfacesApart);
        break;
    }
    case GlobalRepairMethod::Weld: {
        const std::string label = "Distance (" + std::string(unitSymbol(controller_.fileUnit())) + ")";
        changed |= roundDistanceInput(label, params.weldDistance, settings::kMinWeldDistance, settings::kMaxWeldDistance);
        tooltip("Open boundary vertices closer than this across cracks are welded (default 0.05; the unit is only a "
                "label). The slider snaps to round values; type in the box (Enter) for an exact one.");
        ImGui::TextDisabled("Welds open boundary vertices together across seams and cracks without modifying interior faces.");
        edgeLengthHints(controller_, params.weldDistance, false, false);
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
    case GlobalRepairMethod::VoxelHealRemesh: {
        const BoundingBox bbox = controller_.meshBounds();
        const double maxSpan = std::max(bbox.extents().maxCoeff(), settings::kMinRemeshTargetSize);
        const double minSize = std::max(settings::kMinRemeshTargetSize, maxSpan / static_cast<double>(settings::kMaxRemeshGridCells));
        const double maxSize = std::max(minSize, maxSpan / static_cast<double>(settings::kMinRemeshGridCells));
        double targetSize = effectiveRemeshTargetSize(params, maxSpan);
        const std::string sizeLabel = "Target Mesh Size (" + std::string(unitSymbol(controller_.fileUnit())) + ")##voxel";
        if (ImGui::SliderScalar(sizeLabel.c_str(), ImGuiDataType_Double, &targetSize, &minSize, &maxSize, "%.4g",
                                ImGuiSliderFlags_Logarithmic)) {
            params.remeshTargetSize = std::clamp(targetSize, settings::kMinRemeshTargetSize, settings::kMaxRemeshTargetSize);
            changed = true;
        }
        tooltip("Controls the average triangle edge length and grid detail of the output mesh. Smaller values capture "
                "finer details and sharper corners, but increase triangle count and processing time. "
                "Ctrl+click (Control, not Cmd) to type a value.");
        const std::array<int, 3> grid = estimateVoxelGridDimensions(bbox, targetSize);
        ImGui::TextDisabled("Grid: %d x %d x %d", grid[0], grid[1], grid[2]);

        changed |= ImGui::SliderInt("Smoothing Passes##voxel", &params.voxelSmoothingPasses,
                                    settings::kMinRemeshSmoothingPasses, settings::kMaxRemeshSmoothingPasses);
        tooltip("Controls how many times the mesh is relaxed (volume-preserving Taubin de-terracing + isotropic "
                "relaxation) after extraction. Higher values remove 'stair-step' artifacts and create more uniform "
                "triangles, while 0 preserves raw extracted edges.");

        ImGui::BeginDisabled(true);
        double disabledChordPct = 100.0 * params.isosurfaceChordRatio;
        ImGui::SliderScalar("Curve Fidelity / Chord Tolerance##voxel_disabled", ImGuiDataType_Double, &disabledChordPct,
                            &settings::kMinIsosurfaceChordRatio, &settings::kMaxIsosurfaceChordRatio, "N/A");
        ImGui::EndDisabled();
        tooltip("Disabled for Voxel Heal & Remesh (uses a uniform 3D voxel grid). Select Precision Isosurface (FEA Grade) "
                "for curvature-adaptive chord tolerance.");

        changed |= ImGui::Checkbox("Simplify to the original face count##voxel", &params.simplifyRebuild);
        break;
    }
    case GlobalRepairMethod::PrecisionIsosurface: {
        const BoundingBox bbox = controller_.meshBounds();
        const double maxSpan = std::max(bbox.extents().maxCoeff(), settings::kMinRemeshTargetSize);
        const double minSize = std::max(settings::kMinRemeshTargetSize, maxSpan / static_cast<double>(settings::kMaxRemeshGridCells));
        const double maxSize = std::max(minSize, maxSpan / static_cast<double>(settings::kMinRemeshGridCells));
        double targetSize = effectiveRemeshTargetSize(params, maxSpan);
        const std::string sizeLabel = "Target Mesh Size (" + std::string(unitSymbol(controller_.fileUnit())) + ")##iso";
        if (ImGui::SliderScalar(sizeLabel.c_str(), ImGuiDataType_Double, &targetSize, &minSize, &maxSize, "%.4g",
                                ImGuiSliderFlags_Logarithmic)) {
            params.remeshTargetSize = std::clamp(targetSize, settings::kMinRemeshTargetSize, settings::kMaxRemeshTargetSize);
            changed = true;
        }
        tooltip("Controls the average triangle edge length of the output mesh. Smaller values capture finer details "
                "and sharper corners, but increase triangle count and processing time. "
                "Ctrl+click (Control, not Cmd) to type a value.");

        changed |= ImGui::SliderInt("Smoothing Passes##iso", &params.isosurfaceSmoothingPasses,
                                    settings::kMinRemeshSmoothingPasses, settings::kMaxRemeshSmoothingPasses);
        tooltip("Controls how many times the mesh is relaxed after extraction. Higher values create more uniform "
                "triangles, while 0 preserves raw Delaunay-extracted edges.");

        double chordPct = 100.0 * params.isosurfaceChordRatio;
        const double minPct = 100.0 * settings::kMinIsosurfaceChordRatio;
        const double maxPct = 100.0 * settings::kMaxIsosurfaceChordRatio;
        if (ImGui::SliderScalar("Curve Fidelity / Chord Tolerance##iso", ImGuiDataType_Double, &chordPct,
                                &minPct, &maxPct, "%.0f%%", ImGuiSliderFlags_Logarithmic)) {
            params.isosurfaceChordRatio = std::clamp(chordPct / 100.0,
                                                     settings::kMinIsosurfaceChordRatio,
                                                     settings::kMaxIsosurfaceChordRatio);
            changed = true;
        }
        tooltip("Controls how tightly triangles must hug curved surfaces. Lower values force the mesher to place "
                "smaller, denser triangles along curved walls and fillets while keeping flat areas coarse.");
        ImGui::TextDisabled("chord distance = %s (%.0f%% of target size)",
                            lengthText(targetSize * params.isosurfaceChordRatio, controller_.fileUnit()).c_str(),
                            100.0 * params.isosurfaceChordRatio);

        changed |= ImGui::Checkbox("Simplify to the original face count##iso", &params.simplifyRebuild);
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
    const std::vector<RepairCandidate>& rebuildRows = controller_.fixAllRebuilds(); // Fix All: *Rebuild shell #k...*
    if (candidates.empty()) {
        ImGui::TextDisabled("No previews yet.");
        return;
    }
    const Unit unit = controller_.fileUnit();
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_ScrollX |
                                  ImGuiTableFlags_SizingFixedFit;
    // Sized to its rows: with ScrollX the table would otherwise fill the rest of the panel
    const float height = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(candidates.size() + rebuildRows.size() + 1) +
                         ImGui::GetStyle().ScrollbarSize + 2.0f * ImGui::GetStyle().CellPadding.y;
    if (!ImGui::BeginTable("RepairCandidates", 10, flags, ImVec2(0.0f, height))) {
        return;
    }
    ImGui::TableSetupColumn("Method");
    ImGui::TableSetupColumn("Shells");
    const bool holes = controller_.globalRepairTarget() == DefectCategory::OpenEdges;
    const bool thin = controller_.globalRepairTarget() == DefectCategory::ThinFaces;
    ImGui::TableSetupColumn("Closed");
    ImGui::TableSetupColumn(holes ? "Open edges" : thin ? "Thin" : "Crossings");
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
        holes ? "Open edges after (before)." : thin ? "Thin / sliver faces after (before)." : "Intersecting faces after (before).",
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
    bool clickedRebuild = false;
    const auto drawRow = [&](const RepairCandidate& c, bool rebuildRow) {
        const RepairMetrics& m = c.metrics;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushID(static_cast<int>(c.method) + (rebuildRow ? 1000 : 0));
        const bool current = controller_.candidateIsCurrent(c);
        if (!current || !c.success) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        }
        if (ImGui::Selectable(rebuildRow ? fixAllRebuildRowLabel(c).c_str() : displayGlobalMethodName(c.method), shown == &c,
                              ImGuiSelectableFlags_SpanAllColumns)) {
            clicked = c.method;
            clickedRebuild = rebuildRow;
        }
        if (rebuildRow && c.success && c.fixAll != nullptr && c.fixAll->rebuild) {
            tooltip(c.fixAll->rebuild->note.c_str());
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
                ImGui::TextColored(gui::okColor(), "yes");
            } else {
                ImGui::TextColored(gui::defectColor(), "no");
                tooltip((withThousands(m.openEdgesAfter) + " open, " + withThousands(m.nonManifoldEdgesAfter) +
                         " non-manifold edges").c_str());
            }
            ImGui::TableSetColumnIndex(3);
            if (thin) {
                ImGui::TextColored(m.thinFacesAfter == 0 ? gui::okColor() : gui::defectColor(), "%s (%s)",
                                   withThousands(m.thinFacesAfter).c_str(), withThousands(m.thinFacesBefore).c_str());
            } else if (holes) {
                ImGui::TextColored(m.openEdgesAfter == 0 ? gui::okColor() : gui::defectColor(), "%s (%s)",
                                   withThousands(m.openEdgesAfter).c_str(), withThousands(m.openEdgesBefore).c_str());
                tooltip((withThousands(m.intersectingFacesAfter) + " intersecting faces after (" +
                         withThousands(m.intersectingFacesBefore) + " before)").c_str());
            } else if (m.intersectingFacesAfter == 0) {
                ImGui::TextColored(gui::okColor(), "0 (%s)", withThousands(m.intersectingFacesBefore).c_str());
            } else {
                ImGui::TextColored(gui::defectColor(), "%s (%s)", withThousands(m.intersectingFacesAfter).c_str(),
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
    };
    for (const RepairCandidate& c : candidates) {
        drawRow(c, false);
    }
    for (const RepairCandidate& c : rebuildRows) {
        drawRow(c, true);
    }
    ImGui::EndTable();
    if (clicked) {
        if (clickedRebuild) {
            controller_.showFixAllRebuild(*clicked);
        } else {
            controller_.showGlobalCandidate(*clicked);
        }
        requestSync();
    }
}

} // namespace mesheal
