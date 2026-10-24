#pragma once

#include "app/AppController.hpp"

#include <string>

namespace ogun {

/// Thin Polyscope + Dear ImGui visual presentation layer bound to `ogun::AppController`.
class PolyscopeGui {
public:
    static constexpr const char* kSurfaceMeshName = "STL Surface Mesh";
    static constexpr const char* kFaceColorQuantityName = "Defect Highlight";
    static constexpr const char* kOverlayCurveName = "Selected Defect Edges";
    static constexpr const char* kOverlayPointName = "Selected Defect Vertices";

    explicit PolyscopeGui(AppController& controller);

    /// Initializes Polyscope, registers callbacks, syncs initial mesh state, and launches the event loop.
    void run();

private:
    /// Per-frame Dear ImGui callback invoked by Polyscope.
    void drawUiFrame();

    void drawFileBarSection();
    void drawUnitsAndScaleSection();
    void drawDiagnosticsSection();
    void drawHighlightAndNavigationSection();
    void drawNavigationHelpWindow();
    void handleViewportMouseNavigation();
    void handleKeyboardShortcuts();

    void processPivotRotate(float dragDeltaX, float dragDeltaY);
    void processPivotTranslate(float dragDeltaX, float dragDeltaY, double effectiveDist);
    void processPivotZoom(double amount, double effectiveDist);

    /// Registers or updates the Polyscope surface mesh, face color quantity, and curve/point overlays.
    void syncMeshStructureToPolyscope(bool resetCameraToHome);
    void syncHighlightOverlaysToPolyscope();
    void syncMeshTransparencyToPolyscope();
    void syncHighlightSizingFromPolyscopeUi();
    void applyEffectiveHighlightSizingToPolyscope(bool forceUpdate = false);

    [[nodiscard]] Eigen::Vector3d currentCameraWorldPosition() const noexcept;
    [[nodiscard]] double currentZoomScale() const noexcept;

    /// Animates the 3D Polyscope camera to focus on the current selection (or whole mesh)
    /// and updates the 3D center of rotation (orbit pivot) to the target centroid.
    void focusCameraOnSelection(FocusNormalSide normalSide = FocusNormalSide::AutoToggleNormal);

    /// Resets the orbit pivot to the full mesh center and flies the camera to the home view.
    void resetCameraAndPivotToHome();

    AppController& controller_;
    Unit pendingScaleTargetUnit_ = Unit::Centimeter;
    double surfaceEdgeWidth_ = 1.0;
    double lastAppliedPointRelRadius_ = -1.0;
    double lastAppliedEdgeRelRadius_ = -1.0;
    float dragDistSinceLastRelease_ = 0.0f;
    bool positionedMainWindowOnce_ = false;
    bool showNavigationHelpWindow_ = false;
};

} // namespace ogun
