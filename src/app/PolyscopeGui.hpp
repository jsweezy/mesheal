#pragma once

#include "app/AppController.hpp"

#include <memory>
#include <string>
#include <vector>

namespace polyscope {
struct PickResult;
} // namespace polyscope

namespace polyscope::render {
class TextureBuffer;
} // namespace polyscope::render

namespace ogun {

/// Thin Polyscope + Dear ImGui visual presentation layer bound to `ogun::AppController`.
class PolyscopeGui {
public:
    static constexpr const char* kSurfaceMeshName = "STL Surface Mesh";
    static constexpr const char* kFaceColorQuantityName = "Defect Highlight";
    static constexpr const char* kOverlayCurveName = "Selected Defect Edges";
    static constexpr const char* kOverlayPointName = "Selected Defect Vertices";
    static constexpr const char* kRepairPreviewName = "Repair Preview Patch";
    static constexpr const char* kDefectFacesName = "Selected Defect Faces";
    static constexpr const char* kOriginalEdgesName = "Original Edges (Before Repair)";

    explicit PolyscopeGui(AppController& controller);

    /// Initializes Polyscope, registers callbacks, syncs initial mesh state, and launches the event loop.
    void run();

private:
    /// Per-frame Dear ImGui callback invoked by Polyscope.
    void drawUiFrame();

    void drawFileBarSection();
    void drawUnitsAndScaleSection();
    void drawDiagnosticsSection();
    void drawMeshRepairSection();
    void drawSingleFaceRepairInspector();
    void drawUndoButton(const char* idSuffix);
    void drawHighlightAndNavigationSection();
    void drawNavigationHelpWindow();
    void handleViewportMouseNavigation();
    void handleKeyboardShortcuts();
    void handleViewportFacePick(const polyscope::PickResult& pickResult);

    void processPivotRotate(float dragDeltaX, float dragDeltaY);
    void processPivotTranslate(float dragDeltaX, float dragDeltaY, double effectiveDist);
    void processPivotZoom(double amount, double effectiveDist);

    /// Registers or updates the Polyscope surface mesh, face color quantity, and curve/point overlays.
    void loadAndApplyApplicationIcon();
    void syncMeshStructureToPolyscope(bool resetCameraToHome);
    void syncHighlightOverlaysToPolyscope();
    void syncMeshTransparencyToPolyscope();
    /// Registers the base mesh without the faces cut out by `AppController::buildBaseMeshDisplay()`.
    void registerBaseMeshToPolyscope(const HighlightOverlayGeometry& overlay);
    void syncDefectOverlaysToPolyscope(const HighlightOverlayGeometry& overlay);
    void syncRepairPreviewToPolyscope();
    /// Sets Polyscope's transparency mode preferred by the active preview display mode.
    void applyTransparencyModeForPreview();
    /// Pushes overlay styles (colors, edge widths, point color) to the overlay structures.
    void applyOverlayAppearanceToPolyscope();
    /// Adopts style edits made in Polyscope's Structures panel and restores edge widths after
    /// Polyscope's "Edges" checkbox re-enables them at its default width.
    void syncStylesFromPolyscopeUi();
    /// Converts a tube / sphere radius edited in Polyscope's UI back into screen pixels.
    bool adoptOverlayRadius(double polyscopeRadius, double& lastApplied, double& valuePx);
    /// Re-registers the modified mesh (after a repair, scale, or undo) without moving the camera.
    void syncModifiedMeshToPolyscope();
    void undoLastModification();
    void applyEffectiveHighlightSizingToPolyscope(bool forceUpdate = false);

    [[nodiscard]] Eigen::Vector3d currentCameraWorldPosition() const noexcept;

    /// Animates the 3D Polyscope camera to focus on the current selection (or whole mesh)
    /// and updates the 3D center of rotation (orbit pivot) to the target centroid.
    void focusCameraOnSelection(FocusNormalSide normalSide = FocusNormalSide::AutoToggleNormal);

    /// Resets the orbit pivot to the full mesh center and flies the camera to the home view.
    void resetCameraAndPivotToHome();

    AppController& controller_;
    std::shared_ptr<polyscope::render::TextureBuffer> appIconTexture_;
    Unit pendingScaleTargetUnit_ = Unit::Centimeter;
    StlFormat exportFormat_ = StlFormat::Binary;
    std::string saveStatusMessage_;
    SurfaceStyle baseStyle_{Eigen::Vector3d::Constant(0.5), Eigen::Vector3d(0.18, 0.20, 0.24)};
    std::string baseMaterial_;
    bool baseEnabled_ = true;
    bool baseStyleCaptured_ = false;
    bool baseIsCut_ = false;
    std::vector<Eigen::Index> baseFaceIndices_;   ///< Mesh face of each drawn base face (when `baseIsCut_`).
    std::vector<Eigen::Index> defectFaceIndices_; ///< Mesh face of each "Selected Defect Faces" face.
    double lastAppliedDefectEdgeRadius_ = -1.0;
    double lastAppliedOriginalEdgeRadius_ = -1.0;
    double lastAppliedPointRadius_ = -1.0;
    float dragDistSinceLastRelease_ = 0.0f;
    bool positionedMainWindowOnce_ = false;
    bool showNavigationHelpWindow_ = false;
};

} // namespace ogun
