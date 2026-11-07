#pragma once

#include "app/AppController.hpp"
#include "app/FramePacer.hpp"
#include "app/RecentFiles.hpp"
#include "app/UserPreferences.hpp"

#include <polyscope/types.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
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
    static constexpr const char* kPreviewFocusName = "Preview Focus";

    /// `preferencesFile`: where preference changes are saved (empty: not saved). `prefs`: the loaded
    /// preferences, whose controller part is already applied (`loadAndApplyUserPreferences`).
    /// `recentFilesFile`: backing file of File > Open Recent (empty: kept for the session only).
    explicit PolyscopeGui(AppController& controller, std::filesystem::path preferencesFile = {},
                          const UserPreferences& prefs = UserPreferences{}, std::filesystem::path recentFilesFile = {});

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
    void drawStatusBarDelta();
    void drawLeftColumn(float top, float bottom, float rightColumnLeft);
    void drawRightColumn(float top, float bottom);
    void drawPreferencesWindow(float top, float bottom, float rightColumnLeft);
    /// Preferences > Mouse: preset, one row of bindings per action, conflicts, zoom options.
    void drawMousePreferencesTab();
    void drawFixAllPreferencesTab();
    /// Popup editing one binding of the Mouse tab (click-to-capture plus button / modifier controls).
    void drawMouseBindingEditor();
    /// Controller preferences plus the GUI-only ones (UI scale, background color).
    [[nodiscard]] UserPreferences currentPreferences() const;
    /// Applies `prefs` to the controller, the GUI and Polyscope (Preferences > Reset to defaults).
    void applyPreferences(const UserPreferences& prefs);
    /// Saves the preferences when they changed, once no widget is being edited (`force`: even then).
    void savePreferencesIfChanged(bool force = false);
    void drawNavigationHelpWindow();
    void drawAboutWindow();
    void drawModalPopups();
    void resetLayout();
    void openFileDialog();
    /// Loads an STL by path (Open dialog and Open Recent): records it in the recent list on success.
    void openStlFile(const std::filesystem::path& path);
    void drawOpenRecentMenu();
    void saveCurrentFile();
    void saveAsDialog(StlFormat format);
    void saveTo(const std::string& path, StlFormat format);
    /// Loads `examples/<exampleFile>`, or the built-in defective cube when `exampleFile` is null.
    void loadSample(const char* exampleFile);

    // ---- Left column panels (PolyscopeGuiPanels.cpp) ----
    void drawDisplayPanel();
    void drawNavigatePanel();
    void drawStructuresPanel();
    /// View > Polyscope windows, tiled in the free 3D view between the columns when they open.
    void drawPolyscopeWindows(float top, float bottom, float rightColumnLeft);
    void drawCompass(float left, float bottom);
    void drawNoMeshHint(float left, float top, float bottom);
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
    /// Del: the selected faces when there are any, else the selected shells.
    void deleteSelection();
    void saveSelectedShellsDialog(bool eachOwnFile);

    // ---- Right column: Mesh Health & Tool slot (PolyscopeGuiHealth.cpp) ----
    void drawHealthPanel();
    void drawHealthRow(DefectCategory category);
    void drawMeshInfo();
    void drawMeshStatsTable();
    void drawToolPanel();
    void drawCategoryTool(DefectCategory category);
    bool drawCategoryWarnings(const CategoryResult& result); ///< True when a flip button changed the mesh
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
    /// Mesh > Transform > Translate / Rotate (live preview, Apply / Reset / Cancel).
    void drawTransformOperator();
    /// Mesh > Transform > Set Origin to Selection: applies it, then frames the moved mesh.
    void confirmSetOriginToSelection();

    // ---- Intersecting faces: island Inspector & Repair All (PolyscopeGuiIntersections.cpp) ----
    void drawIntersectionInspector();
    void drawIslandScorecard();
    void drawIntersectionRepairSettings();
    void drawHoleInspector();
    void drawHoleScorecard();
    void drawHoleFillSettings();
    void drawRepairAllOperator();

    // ---- Operation: Fix All (PolyscopeGuiFixAll.cpp) ----
    void drawFixAllOperator();
    void drawFixAllPlan();
    void drawFixAllPlanRows();
    void drawFixAllMergeNotes();
    void drawFixAllOptions();
    void drawFixAllSettingsUsed();
    void drawFixAllRunning();
    void drawFixAllResult();
    void drawFixAllReport();
    void drawFixAllFooter();        ///< The action buttons, always visible below the scrolling body
    void drawFixAllPlanFooter();
    void drawFixAllRebuildFooter();
    void drawFixAllResultFooter();
    void drawFixAllReportFooter();
    void drawFixAllSwitch();        ///< Repair / Rebuild radios (hidden for a face scope)
    void drawFixAllRebuildPlan();   ///< The Rebuild side of the Plan state
    void drawFixAllRebuildJob();    ///< Progress and Cancel of a rebuild job (whole scope or one shell)
    void drawFixAllShellRebuildRows(); ///< Notes of the *Rebuild shell #k...* rows under the table
    void drawFixAllApplyRow(bool inPlan); ///< Apply the shown row, Close
    void drawFixAllPreviewControls();
    void drawFixAllSteps(const FixAllResult& result);
    void drawFixAllLines(const FixAllResult& result, bool beforeApply);
    /// The [Fix All...] and [One Pass...] buttons beside Check Mesh; returns after laying them out.
    void drawFixAllHeaderButtons();
    void drawMergeShellsOperator();
    void drawGlobalMethodParams(GlobalRepairMethod method);
    void drawGlobalCandidateTable();
    void fixAll(DefectCategory category);
    void checkMesh();
    /// Opens Operation: Fix All with the saved number of passes (`fullRun`), or with one pass (One Pass...).
    void openFixAllPanel(bool fullRun, bool rebuildSide = false);

    // ---- Face selection tools: box (B), circle (C), Shift+click (PolyscopeGuiSelection.cpp) ----
    enum class FaceSelectMode { None, Box, Circle };
    [[nodiscard]] ScreenView currentScreenView() const;
    /// The enabled Polyscope slice planes that cut the base mesh (world space).
    [[nodiscard]] static std::vector<ClipPlane> currentClipPlanes();
    void startFaceSelectMode(FaceSelectMode mode);
    void exitFaceSelectMode();
    /// Mouse input while a select mode is on (instead of the camera).
    void handleFaceSelectTool();
    void drawFaceSelectOverlay();
    void drawSelectFacesMenu();

    // ---- Viewport input & Polyscope sync (PolyscopeGui.cpp) ----
    void handleViewportMouseNavigation();
    /// Sleeps or waits for input before the frame when nothing needs the full frame rate (idle CPU).
    void paceFrame();
    void handleKeyboardShortcuts();
    /// `shellPick`: Select Shell (on multi-shell meshes), adding / removing it when `shellToggle`.
    void handleViewportFacePick(const polyscope::PickResult& pickResult, bool shellPick, bool shellToggle);
    /// Runs the click action bound to `button` (released without dragging).
    void handleViewportClick(MouseInput button);
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
    /// Registers (or removes, when empty) the "Original edges" curve network.
    void registerOriginalEdges(const HighlightOverlayGeometry& originalEdges);
    /// Result / Original opacity, Original style and Changed faces only of a whole-mesh preview. `inTool`: the
    /// compact set under an operation's Result / Original / Both buttons.
    void drawWholeMeshPreviewControls(bool inTool);
    /// Switches to Simple (Wireframe) or Pretty (Ghosted) transparency while a whole-mesh preview shows Both.
    void applyWholeMeshTransparencyMode();
    /// A click while a whole-mesh preview is open: sets the preview focus to the clicked triangle of the result
    /// or the current mesh. Never opens a defect, so the preview stays as it is.
    void handleWholeMeshPreviewPick(const polyscope::PickResult& pickResult);
    /// Draws (or removes) the dot marking the preview focus.
    void syncPreviewFocusMarker();
    /// Registers the Translate / Rotate preview (the shown faces at their moved positions).
    void syncTransformPreviewToPolyscope();
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
    bool showStructures_ = false;      ///< Structures panel, off until View > Polyscope > Structures
    bool showShells_ = true;           ///< Shells panel (shown only for multi-shell meshes)
    bool structuresCollapsed_ = true;  ///< Structures panel state, to size the Shells panel above it
    bool showCompass_ = false;         ///< View > Compass (saved preference)
    bool showPolySlicePlanes_ = false; ///< View > Polyscope > Slice Planes window
    bool showPolyAppearance_ = false;  ///< ... Appearance (background, transparency, ground plane, ...)
    bool showPolyView_ = false;        ///< ... Camera & Clipping (navigation style, up / front, FOV, clip planes)
    bool showPolyRender_ = false;      ///< ... Render & Debug (frame rate, vsync, redraw)
    bool showMeshInfo_ = false;
    bool showPreferences_ = false;
    bool fixAllHintSeen_ = false; ///< The first-run line of the Fix All panel was shown once
    bool fixAllRebuildSide_ = false; ///< The Fix All panel's switch is on Rebuild (reset when the panel opens)
    GlobalRepairMethod fixAllRebuildMethod_ = GlobalRepairMethod::AlphaWrap; ///< The Rebuild side's method radio
    /// The cavities an alpha wrap of the scope would fill, worked out once per mesh and scope (not every frame).
    struct FixAllCavityCache {
        bool valid = false;
        std::uint64_t meshGeneration = 0;
        std::size_t scopeFaces = 0;
        std::string scopeText;
        std::vector<int> cavities;
    } fixAllCavityCache_;
    bool showNavigationHelpWindow_ = false;
    bool showAboutWindow_ = false;
    bool requestSaveConfirm_ = false;
    bool requestDeleteConfirm_ = false;                      ///< Open the X-key "Delete N selected faces?" popup
    bool requestSetOrigin_ = false;                          ///< Open the Set Origin to Selection confirmation
    Eigen::Vector3d pendingOrigin_ = Eigen::Vector3d::Zero(); ///< Selection centroid shown in that confirmation
    bool layoutResetPending_ = true;
    bool leftColumnAutoCollapsed_ = false;
    float rightColumnWidth_ = settings::kRightColumnDefaultPx;
    float uiScale_ = 1.0f;
    MouseMapping mouseMapping_;                  ///< Preferences > Mouse (saved preference)
    MouseAction editingMouseAction_ = MouseAction::Orbit; ///< Binding open in the Mouse tab's editor
    std::size_t editingMouseSlot_ = 0;

    // Saved preferences (written to `preferencesFile_` whenever a value changes)
    std::filesystem::path preferencesFile_;
    UserPreferences savedPreferences_;  ///< Values last loaded or saved; a difference triggers a save
    std::string preferencesSaveError_;

    RecentFiles recentFiles_;  ///< File > Open Recent

    // Status bar message (mirrors the controller's latest message for a few seconds)
    std::uint64_t shownMessageSerial_ = 0;
    std::chrono::steady_clock::time_point messageTime_{};

    // File state
    StlFormat saveFormat_ = StlFormat::Binary;
    std::string overwriteConfirmedFor_;
    Unit pendingScaleTargetUnit_ = Unit::Centimeter;

    // Tool panel
    bool scrollListToFocus_ = false;

    // Face selection tools
    FaceSelectMode faceSelectMode_ = FaceSelectMode::None;
    float circleRadiusPx_ = settings::kDefaultCircleSelectRadiusPx;
    bool boxDragging_ = false;
    bool boxRemoves_ = false;
    Eigen::Vector2d boxStart_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d lastPaint_ = Eigen::Vector2d::Constant(-1.0e9);

    // Shells panel
    int smallShellFaces_ = settings::kDefaultSmallShellFaces;
    bool deleteShellsRequested_ = false;
    bool deleteHoleRimsRequested_ = false;
    bool deleteFacesRequested_ = false;

    // 3D sync
    bool syncPending_ = false;
    bool syncMeshChanged_ = false;
    std::string baseMaterial_;
    bool baseEnabled_ = true;
    bool baseStyleCaptured_ = false;
    /// Transparency mode forced while a whole-mesh preview shows Both (-1: none, else a WholeMeshOriginalStyle):
    /// Pretty for Ghosted, Simple for Wireframe (Pretty is too slow on a dense wireframe). The user's mode is
    /// restored when the preview leaves Both.
    int wholeMeshTransparencyStyle_ = -1;
    /// Geometry of the drawn whole-mesh result (`kRepairPreviewName`), to map a click back to a triangle.
    Eigen::MatrixXd previewPickV_;
    Eigen::MatrixXi previewPickF_;
    polyscope::TransparencyMode savedTransparencyMode_ = polyscope::TransparencyMode::Pretty;
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
    ViewportClickTracker clickTracker_; ///< Tells viewport clicks from drags
    FramePacer framePacer_;
};

} // namespace ogun
