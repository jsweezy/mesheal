#pragma once

#include "app/AppController.hpp"

#include <array>
#include <chrono>
#include <cstdint>
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
/// Layout (UI_DESIGN.md §3): menu bar, left column (Display / Navigate / Shells / Structures), right
/// column (Mesh Health + Tool slot) and a status bar, all docked by code every frame.
/// Implementation is split across PolyscopeGui.cpp (Polyscope sync, mouse, keys),
/// PolyscopeGuiShell.cpp (layout, menus, status bar, windows), PolyscopeGuiPanels.cpp
/// (Display / Navigate / Structures), PolyscopeGuiShells.cpp (Shells panel) and
/// PolyscopeGuiHealth.cpp (Mesh Health, Tool slot).
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

    // ---- Shell: layout, menus, status bar, floating windows (PolyscopeGuiShell.cpp) ----
    float drawMenuBar();
    void drawFileMenu();
    void drawEditMenu();
    void drawViewMenu();
    void drawMeshMenu();
    void drawRepairMenu();
    void drawHelpMenu();
    void drawStatusBar(float top, float height);
    void drawLeftColumn(float top, float bottom, float rightColumnLeft);
    void drawRightColumn(float top, float bottom);
    void drawPreferencesWindow();
    void drawNavigationHelpWindow();
    void drawModalPopups();
    void resetLayout();
    void openFileDialog();
    void saveCurrentFile();
    void saveAsDialog(StlFormat format);
    void saveTo(const std::string& path, StlFormat format);
    /// Loads `examples/<exampleFile>`, or the built-in defective cube when `exampleFile` is null.
    void loadSample(const char* exampleFile);

    // ---- Left column panels (PolyscopeGuiPanels.cpp) ----
    void drawDisplayPanel();
    void drawNavigatePanel();
    void drawStructuresPanel();
    void flyToStandardView(int view);
    void toggleGhost();
    void toggleOrthographic();

    // ---- Shells panel (PolyscopeGuiShells.cpp) ----
    void drawShellsPanel();
    void drawShellTable(float height);
    void drawShellRowTooltip(std::size_t shell);
    /// Frame, Isolate, Hide, Delete, Save As... (right-click menu and Mesh > Shells).
    void drawShellActionItems();
    void drawShellSelectPopup();
    void toggleIsolateShells();
    /// Deletes the selected shells at the end of the frame (undoable).
    void deleteSelectedShells();
    void saveSelectedShellsDialog(bool eachOwnFile);

    // ---- Right column: Mesh Health & Tool slot (PolyscopeGuiHealth.cpp) ----
    void drawHealthPanel();
    void drawHealthRow(DefectCategory category);
    void drawMeshInfo();
    void drawToolPanel();
    void drawCategoryTool(DefectCategory category);
    void drawCategorySettings(DefectCategory category);
    void drawSplitRepairNote(DefectCategory category);
    void drawCategoryActions(DefectCategory category);
    void drawDefectTable(float height);
    void drawDefectSummary();
    void drawHistogram();
    void drawInspector();
    void drawThinFaceInspector();
    void drawIntersectionPartners();
    void drawUnitsAndScaleOperator();

    // ---- Intersecting faces: island Inspector & Repair All (PolyscopeGuiIntersections.cpp) ----
    void drawIntersectionInspector();
    void drawIslandScorecard();
    void drawIntersectionRepairSettings();
    void drawHoleInspector();
    void drawHoleScorecard();
    void drawHoleFillSettings();
    void drawRepairAllOperator();
    void drawMergeShellsOperator();
    void drawGlobalMethodParams(GlobalRepairMethod method);
    void drawGlobalCandidateTable();
    void fixAll(DefectCategory category);
    void checkMesh();

    // ---- Viewport input & Polyscope sync (PolyscopeGui.cpp) ----
    void handleViewportMouseNavigation();
    void handleKeyboardShortcuts();
    void handleViewportFacePick(const polyscope::PickResult& pickResult);
    void processPivotRotate(float dragDeltaX, float dragDeltaY);
    void processPivotTranslate(float dragDeltaX, float dragDeltaY, double effectiveDist);
    void processPivotZoom(double amount, double effectiveDist);

    void loadAndApplyApplicationIcon();
    /// Re-registers everything after a new mesh was loaded and frames it.
    void syncMeshStructureToPolyscope(bool resetCameraToHome);
    void syncHighlightOverlaysToPolyscope();
    void syncMeshTransparencyToPolyscope();
    /// Registers the base mesh without the faces cut out by `AppController::buildBaseMeshDisplay()`.
    void registerBaseMeshToPolyscope(const HighlightOverlayGeometry& overlay);
    void applyBaseMeshStyleToPolyscope();
    void syncDefectOverlaysToPolyscope(const HighlightOverlayGeometry& overlay);
    void syncRepairPreviewToPolyscope();
    void syncGlobalRepairPreviewToPolyscope();
    /// Gives every overlay sphere / tube end the radius that draws it at the set pixel size at its own depth.
    void applyPerNodeOverlayRadii(double edgeRadius, double originalEdgeRadius, double pointRadius, bool forceUpdate);
    /// Sets Polyscope's transparency mode preferred by the active preview display mode.
    void applyTransparencyModeForPreview();
    /// Pushes overlay styles (colors, edge widths, point color) to the overlay structures.
    void applyOverlayAppearanceToPolyscope();
    /// Adopts style edits made in the Structures panel and restores edge widths after Polyscope's
    /// "Edges" checkbox re-enables them at its default width.
    void syncStylesFromPolyscopeUi();
    /// Converts a tube / sphere radius edited in Polyscope's UI back into screen pixels.
    bool adoptOverlayRadius(double polyscopeRadius, double& lastApplied, double& valuePx);
    void applyEffectiveHighlightSizingToPolyscope(bool forceUpdate = false);
    void undoLastModification();

    /// Marks the 3D overlays for a rebuild at the end of the frame (`meshChanged`: clear the pick too).
    void requestSync(bool meshChanged = false) noexcept;
    void processPendingSync();

    [[nodiscard]] Eigen::Vector3d currentCameraWorldPosition() const noexcept;

    /// Animates the camera to the current selection (or whole mesh) and moves the orbit pivot there.
    void focusCameraOnSelection(FocusNormalSide normalSide = FocusNormalSide::AutoToggleNormal);
    /// Focuses the next / previous defect and flies the camera to it (N / Shift+N).
    void stepDefectFocus(int direction);

    /// Resets the orbit pivot to the full mesh center and flies the camera to the home view.
    void resetCameraAndPivotToHome();

    AppController& controller_;
    std::shared_ptr<polyscope::render::TextureBuffer> appIconTexture_;

    // Panels & layout (View menu, Reset Layout)
    bool showHealth_ = true;
    bool showTool_ = true;
    bool showDisplay_ = true;
    bool showNavigate_ = true;
    bool showStructures_ = true;
    bool showShells_ = true;           ///< Shells panel (shown only for multi-shell meshes)
    bool structuresCollapsed_ = true;  ///< Structures panel state, to size the Shells panel above it
    bool showMeshInfo_ = false;
    bool showPreferences_ = false;
    bool showNavigationHelpWindow_ = false;
    bool requestAbout_ = false;
    bool requestSaveConfirm_ = false;
    bool layoutResetPending_ = true;
    bool leftColumnAutoCollapsed_ = false;
    float rightColumnWidth_ = settings::kRightColumnDefaultPx;
    float uiScale_ = 1.0f;

    // Status bar message (mirrors the controller's latest message for a few seconds)
    std::uint64_t shownMessageSerial_ = 0;
    std::chrono::steady_clock::time_point messageTime_{};

    // File state
    StlFormat saveFormat_ = StlFormat::Binary;
    std::string overwriteConfirmedFor_;
    Unit pendingScaleTargetUnit_ = Unit::Centimeter;

    // Tool panel
    bool scrollListToFocus_ = false;

    // Shells panel
    int smallShellFaces_ = settings::kDefaultSmallShellFaces;
    bool deleteShellsRequested_ = false;

    // 3D sync
    bool syncPending_ = false;
    bool syncMeshChanged_ = false;
    std::string baseMaterial_;
    bool baseEnabled_ = true;
    bool baseStyleCaptured_ = false;
    bool baseIsCut_ = false;
    std::vector<Eigen::Index> baseFaceIndices_;   ///< Mesh face of each drawn base face (when `baseIsCut_`).
    std::vector<Eigen::Index> defectFaceIndices_; ///< Mesh face of each "Selected Defect Faces" face.
    double lastAppliedDefectEdgeRadius_ = -1.0;
    double lastAppliedOriginalEdgeRadius_ = -1.0;
    double lastAppliedPointRadius_ = -1.0;
    std::vector<Eigen::Vector3d> overlayNodes_;      ///< Points of the defect tubes / spheres (per-node sizing)
    std::vector<Eigen::Vector3d> originalEdgeNodes_; ///< Points of the "Original Edges" tubes
    std::array<double, 22> lastNodeSizingKey_{};     ///< View matrix, FOV, viewport and sizes the node radii fit
    bool nodeRadiiDirty_ = true;
    float dragDistSinceLastRelease_ = 0.0f;
};

} // namespace ogun
