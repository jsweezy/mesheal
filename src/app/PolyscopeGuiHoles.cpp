// Open edges: the per-hole Inspector and the hole-filling settings of the Open edges tool.

#include "app/GuiWidgets.hpp"
#include "app/PolyscopeGui.hpp"
#include "core/Settings.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace ogun {

using gui::tooltip;
using gui::withThousands;

namespace {

struct HoleMethodOption {
    HoleFillMethod method;
    const char* label;
    const char* help;
};

constexpr std::array<HoleMethodOption, 4> kHoleMethodOptions = {{
    {HoleFillMethod::Flat, "Fill Flat (Missing CAD Faces)",
     "CGAL triangulate_hole: new faces between the rim vertices only, no new vertices. A planar rim is "
     "triangulated in its plane (2D constrained Delaunay), so a missing flat face or end cap comes back flat."},
    {HoleFillMethod::Refine, "Fill & Refine (Match the Mesh Density)",
     "The flat fill plus new interior vertices, so the patch's triangles are about the size of the faces "
     "around it. The patch stays where the flat fill put it."},
    {HoleFillMethod::Smooth, "Fill Smooth (Curved Surfaces, Scans)",
     "The refined patch is faired (a bi-Laplacian solve) so it continues the curvature of the surface around "
     "it. Right for scans and curved CAD faces; it domes a cap whose walls meet it at a right angle."},
    {HoleFillMethod::CloseCrack, "Close Crack (Unwelded Seams, T-junctions)",
     "Snaps the rim vertices onto the rim edges across the gap and stitches them: no faces are added in the "
     "gap. Only for gaps much narrower than the faces around them; vertices move at most the crack snap "
     "distance (Settings)."},
}};

std::string lengthText(double value, Unit unit) {
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.3g %s", value, std::string(unitSymbol(unit)).c_str());
    return buffer;
}

const char* holeShapeHelp(HoleShape shape) {
    switch (shape) {
    case HoleShape::Flat:
        return "The rim lies in a plane and the faces around it are coplanar with it or meet it at a right "
               "angle: a missing flat face or end cap.";
    case HoleShape::Curved:
        return "The rim is not planar, or the surface around it is tilted: the fill should follow the curvature.";
    case HoleShape::Crack:
        return "The gap is much narrower than the faces around it: CAD patches that were not welded, or a "
               "T-junction where one side has vertices the other lacks.";
    case HoleShape::OpenChain:
        return "The open edges do not close into a loop: they end at a non-manifold edge (a fin or wall).";
    }
    return "";
}

} // namespace

void PolyscopeGui::drawHoleInspector() {
    const HoleLoop* hole = controller_.focusedHole();
    if (hole == nullptr) {
        return;
    }
    const Unit unit = controller_.fileUnit();
    ImGui::Text("Hole %s: %s edges, %s", withThousands(controller_.focusedHoleNumber()).c_str(),
                withThousands(hole->edgeCount()).c_str(), holeShapeLabel(hole->shape));
    tooltip(holeShapeHelp(hole->shape));
    ImGui::TextDisabled("Size %s, perimeter %s, gap width %s", lengthText(hole->size, unit).c_str(),
                        lengthText(hole->perimeter, unit).c_str(), lengthText(hole->width, unit).c_str());
    tooltip("Size: bounding-box diagonal of the rim. Gap width: 2 x area / perimeter (the radius of a round "
            "hole, the width of a slit).");
    ImGui::TextDisabled("Rim off its plane by %s; rim faces flat or square to it: %.0f%%",
                        lengthText(hole->planeDeviation, unit).c_str(), 100.0 * hole->flatNeighbourShare);
    tooltip("How the shape was decided: a Flat hole has a planar rim and faces around it that continue the "
            "plane or meet it at a right angle.");
    if (hole->shape == HoleShape::OpenChain) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("%s Fix the Non-manifold edges row first.", holeShapeHelp(hole->shape));
        ImGui::PopStyleColor();
    } else {
        ImGui::TextDisabled("Recommended: %s", holeFillMethodLabel(recommendedHoleFill(hole->shape)));
        for (const HoleMethodOption& option : kHoleMethodOptions) {
            if (ImGui::RadioButton(option.label, controller_.selectedHoleMethod() == option.method)) {
                controller_.setSelectedHoleMethod(option.method);
                requestSync();
            }
            tooltip(option.help);
        }
        drawHoleScorecard();
        ImGui::TextDisabled("Preview display options: Display > Repair preview.");
    }

    const HoleFillPreview& preview = controller_.holePreview();
    const bool canCommit = preview.valid && preview.newCrossings == 0;
    ImGui::BeginDisabled(!canCommit);
    if (ImGui::Button("Commit Repair") && controller_.commitHoleFill()) {
        requestSync(true);
    }
    ImGui::EndDisabled();
    tooltip(canCommit ? "Apply the chosen fill to this hole (one undo step)."
                      : "Pick a method whose preview is valid and crosses no other faces.");
    ImGui::SameLine();
    if (ImGui::Button("Frame (.)")) {
        focusCameraOnSelection(FocusNormalSide::AutoToggleNormal);
    }
    tooltip("Look into the hole along its outward normal; press again for the inside.");
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

void PolyscopeGui::drawHoleScorecard() {
    const HoleFillPreview& preview = controller_.holePreview();
    if (!preview.valid) {
        if (!preview.statusMessage.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, gui::kErrorColor);
            ImGui::TextWrapped("Not possible: %s", preview.statusMessage.c_str());
            ImGui::PopStyleColor();
        }
        return;
    }
    const Unit unit = controller_.fileUnit();
    const std::string sym(unitSymbol(unit));
    ImGui::Text("+%s faces, +%s vertices%s   area %.3g %s^2   min angle %.1f deg", withThousands(preview.facesAdded).c_str(),
                withThousands(preview.verticesAdded).c_str(),
                preview.facesRemoved > 0 ? (", -" + withThousands(preview.facesRemoved) + " faces").c_str() : "",
                preview.patchArea, sym.c_str(), preview.minNewAngleDeg);
    tooltip("New faces and vertices (and faces replaced), the area of the new faces, and their smallest angle "
            "(small angles are new slivers).");
    if (preview.maxVertexMove > 0.0) {
        ImGui::Text("Rim vertices moved at most %s", lengthText(preview.maxVertexMove, unit).c_str());
    }
    if (preview.openEdgesLeft == 0) {
        ImGui::TextColored(gui::kOkColor, "Closes the hole.");
    } else {
        ImGui::TextColored(gui::kDefectColor, "%s edges stay open.", withThousands(preview.openEdgesLeft).c_str());
    }
    if (preview.newCrossings > 0) {
        ImGui::TextColored(gui::kErrorColor, "The patch crosses %s faces: Commit is disabled.",
                           withThousands(preview.newCrossings).c_str());
    } else {
        ImGui::SameLine();
        ImGui::TextColored(gui::kOkColor, "Crosses no other face.");
    }
    tooltip("The new faces are tested against every face near them, in any shell.");
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s (%.0f ms)", preview.statusMessage.c_str(), preview.elapsedMs);
    ImGui::PopStyleColor();
}

void PolyscopeGui::drawHoleFillSettings() {
    HoleFillSettings settings = controller_.holeFillSettings();
    bool changed = false;
    ImGui::SeparatorText("Hole filling");
    changed |= ImGui::SliderInt("Patch rings", &settings.patchRings, 1, settings::kMaxHolePatchRings);
    tooltip("Rings of faces around the rim handed to CGAL with the hole. Fill & Refine and Fill Smooth size and "
            "bend the patch from them; 3 is enough for both.");
    int maxFaces = static_cast<int>(std::min<std::size_t>(settings.maxPatchFaces, 100'000'000));
    if (ImGui::InputInt("Max refined faces", &maxFaces, 1000, 10000, ImGuiInputTextFlags_EnterReturnsTrue)) {
        settings.maxPatchFaces = static_cast<std::size_t>(std::max(maxFaces, 1));
        changed = true;
    }
    tooltip("Fill & Refine / Fill Smooth refuse a hole whose refined patch has more faces: the patch matches the "
            "small rim edges, and fairing a huge patch takes seconds.");
    if (ImGui::InputInt("Max hole edges (0 = all)", &settings.maxHoleEdges, 10, 100, ImGuiInputTextFlags_EnterReturnsTrue)) {
        changed = true;
    }
    tooltip("Fix Selected and Repair All > Fill each hole skip holes with more rim edges, e.g. an open end "
            "that is meant to stay open. The Inspector fills any hole.");
    changed |= ImGui::InputDouble("Crack snap (x gap width)", &settings.crackSnapFactor, 0.5, 1.0, "%.1f",
                                  ImGuiInputTextFlags_EnterReturnsTrue);
    tooltip("Close Crack moves a rim vertex at most this many mean gap widths (and never more than 0.9 of its "
            "shortest edge).");
    if (changed) {
        controller_.setHoleFillSettings(settings);
        requestSync();
    }
}

} // namespace ogun
