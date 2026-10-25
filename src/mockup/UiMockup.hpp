#pragma once

#include "diagnostics/MeshHealth.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstddef>
#include <functional>
#include <future>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ogun::mockup {

/// Lifecycle of one Mesh Health row (see UI_DESIGN.md, "Mesh Health panel").
enum class HealthStatus {
    NotRun,
    Running,
    Ok,
    Defects,
    Stale,
};

/// One fake defect: a face index on the real mesh plus made-up metrics for the list columns.
struct MockDefect {
    Eigen::Index faceIndex = -1;
    double metric = 0.0;
    double area = 0.0;
    int subtype = 0;
    bool selected = false;
    int shell = -1;        ///< Shell that owns `faceIndex` (-1: unusable face)
    std::vector<int> partnerShells; ///< Intersections: other shells this face crosses
    bool selfCrossing = false;      ///< Intersections: also crosses a face of its own shell
};

/// One connected piece of the real mesh (from `checkShells`), largest first.
struct MockShell {
    std::vector<Eigen::Index> faces;
    BoundingBox bbox;
    int nestedIn = -1; ///< Shell this one lies inside without crossing it (-1: none)
};

/// Per-shell view and edit state; kept apart from `MockShell` so undo snapshots stay small.
struct ShellFlags {
    bool visible = true;
    bool selected = false;
    bool deleted = false;
    int mergedInto = -1; ///< After a merge: the shell this one became part of
};

/// Which shells the Mesh Health counts, lists and Fix All cover (compared in the Mockup menu).
enum class ScopeModel {
    VisibleShells,
    SelectedShells,
    WholeMesh,
};

/// Defect counts of one shell, by Mesh Health row. Crossings with other shells are kept apart
/// because Merge Shells removes them, so they do not block a per-shell clean-up.
struct ShellHealth {
    std::array<std::size_t, 16> counts{};
    std::size_t between = 0;
    std::size_t total = 0;
};

struct HealthCategory {
    const char* name = "";
    const char* group = "";
    const char* tooltip = "";
    const char* metricLabel = "";
    std::vector<const char*> subtypes; ///< Sub-type filter labels (empty: no filter)
    bool slowCheck = false;
    bool infoOnly = false;
    std::vector<MockDefect> defects;  ///< Sorted worst first.
    std::vector<MockDefect> pending;  ///< What the first simulated check "finds".
    std::size_t infoValue = 0;        ///< Count shown by info-only rows (e.g. shells).
    HealthStatus status = HealthStatus::NotRun;
};

/// What the right-column Tool slot currently shows (one active tool at a time).
enum class ToolSlot {
    None,
    DefectCategory,
    UnitsAndScale,
    Decimate,
    WeldVertices,
    RemoveDegenerate,
    MergeShells,
};

/// Throwaway clickable prototype of the layout proposed in UI_DESIGN.md.
/// Shells and (unless `fakeDefects`) defects come from the real checks; fixes, deletes and merges
/// are simulated. Save As of shells writes real STL files.
class UiMockup {
public:
    UiMockup(Mesh mesh, std::string sourceLabel, bool fakeDefects = false);

    void run();

private:
    struct UndoSnapshot {
        std::string label;
        std::vector<std::vector<MockDefect>> defects;
        std::vector<HealthStatus> statuses;
        std::vector<ShellFlags> shellFlags;
    };

    void drawFrame();
    float drawMenuBar();
    void drawStatusBar(float top, float height);
    void drawLeftColumn(float top, float bottom);
    void drawRightColumn(float top, float bottom);
    void drawDisplayPanelContents();
    void drawNavigationPanelContents();
    void drawStructuresPanelContents();
    void drawHealthPanelContents();
    void drawHealthRow(std::size_t categoryIndex);
    void drawToolPanelContents();
    void drawCategoryTool(HealthCategory& category);
    void drawCategorySettings(std::size_t categoryIndex);
    void drawDefectTable(HealthCategory& category, const std::vector<std::size_t>& visibleRows);
    void drawDefectSummary(HealthCategory& category, const std::vector<std::size_t>& visibleRows);
    void drawInspector(const HealthCategory& category);
    void drawOperatorTool();
    void drawMergeTool();
    void drawPreferencesWindow();
    void drawHelpWindow();
    void handleKeyboardShortcuts();
    void handleViewportClick();

    // Shells panel and shell actions
    void drawShellsPanelContents();
    void drawShellTable(const std::vector<ShellHealth>& health, float height);
    void drawShellRowTooltip(int shell, const ShellHealth& health);
    void drawShellActionItems();
    void drawShellSelectPopup(const std::vector<ShellHealth>& health);
    void drawShellSavePopup();
    void clickShell(int shell, bool toggle, bool range, const std::vector<int>& order);
    void selectShellsWhere(const std::function<bool(int)>& predicate);
    void toggleIsolate();
    void hideSelectedShells();
    void showAllShells();
    void hideAllShells();
    void frameSelectedShells();
    void deleteSelectedShells();
    void saveSelectedShells(bool eachOwnFile);
    void applyMerge(const std::vector<int>& roots);
    void onShellVisibilityChanged();
    void onShellSelectionChanged();
    void pruneOutOfScope();
    void recountSelection();

    void buildShells();
    void startRealChecks();
    void applyRealResults(std::vector<CategoryResult> results);
    void findNestedShells();
    void generateFakeDefects();
    void startCheckAll();
    void startCheckOne(std::size_t categoryIndex);
    void advanceCheckJob();
    void cancelCheckJob();
    void selectCategory(int categoryIndex);
    void fixAll(std::size_t categoryIndex);
    void fixSelected(std::size_t categoryIndex);
    void pushUndo(const std::string& label);
    void undo();
    void stepFocus(int direction);
    void focusDefectRow(std::size_t row, bool flyCamera);
    void openOperator(ToolSlot slot);
    void setStatusMessage(std::string message);
    void mockAction(const char* what);

    void registerMesh();
    void refreshFaceColors();
    void applyBaseStyle();
    void flyToFace(Eigen::Index faceIndex, double normalSign);
    void flyToAxisView(const Eigen::Vector3d& direction);
    void flyToBox(const BoundingBox& box);
    void resetLayout();

    [[nodiscard]] HealthCategory* selectedCategory();
    [[nodiscard]] std::size_t selectedRowCount(const HealthCategory& category) const;
    [[nodiscard]] bool watertightKnown() const;
    [[nodiscard]] bool isWatertight() const;

    [[nodiscard]] int rootShell(int shell) const;
    [[nodiscard]] bool shellLive(int shell) const;
    [[nodiscard]] bool faceShown(Eigen::Index face) const;
    [[nodiscard]] bool shellInScope(int root) const;
    [[nodiscard]] bool crossesOwnShell(const MockDefect& defect) const;
    [[nodiscard]] bool defectInScope(const MockDefect& defect) const;
    [[nodiscard]] bool rowVisible(const HealthCategory& category, std::size_t row) const;
    [[nodiscard]] bool fixableByFixAll(std::size_t categoryIndex, const MockDefect& defect) const;
    [[nodiscard]] std::size_t scopedCount(const HealthCategory& category) const;
    [[nodiscard]] std::size_t fixAllCount(std::size_t categoryIndex) const;
    [[nodiscard]] bool hasShellPanel() const;
    [[nodiscard]] std::size_t liveShellCount() const;
    [[nodiscard]] std::vector<int> liveShells() const;
    [[nodiscard]] std::vector<int> selectedShells() const;
    [[nodiscard]] bool selectionMatchesVisible() const;
    [[nodiscard]] std::vector<std::size_t> shellFaceCounts() const;
    [[nodiscard]] BoundingBox shellBox(int root) const;
    [[nodiscard]] std::vector<ShellHealth> shellHealth() const;
    [[nodiscard]] bool healthKnown() const;
    [[nodiscard]] std::string shellLabel(int shell) const;
    [[nodiscard]] Eigen::RowVector3d shellColor(int root) const;
    [[nodiscard]] Mesh extractShells(const std::vector<int>& roots) const;

    Mesh mesh_;
    std::string sourceLabel_;
    BoundingBox bbox_;
    Unit fileUnit_ = Unit::Millimeter;
    bool modified_ = false;
    bool fakeDefects_ = false;

    // Shells (real, from checkShells) and their view / edit state
    std::vector<MockShell> shells_;
    std::vector<ShellFlags> shellFlags_;
    std::vector<int> faceShell_;              ///< Shell of every face (-1: unusable face)
    std::vector<Eigen::Index> displayFaces_;  ///< Registered Polyscope face -> mesh face
    std::vector<Eigen::Index> faceToRow_;     ///< Mesh face -> registered face (-1: hidden)
    std::set<std::pair<int, int>> crossingPairs_; ///< Shell pairs (low, high) whose faces cross
    std::optional<std::vector<bool>> isolateRestore_; ///< Visibility to restore when un-isolating
    int shellAnchor_ = -1;
    std::size_t selectedShellCount_ = 0;
    bool colorByShell_ = true;
    bool showShells_ = true;
    bool structuresCollapsed_ = true;
    ScopeModel scopeModel_ = ScopeModel::SelectedShells;
    int smallShellFaces_ = 100;
    double mergeGapTolerance_ = 1.0e-8;
    std::future<std::vector<CategoryResult>> realChecks_;
    bool realChecksApplied_ = false;

    std::vector<HealthCategory> categories_;
    int selectedCategory_ = -1;
    ToolSlot toolSlot_ = ToolSlot::None;
    std::ptrdiff_t focusedRow_ = -1;
    std::ptrdiff_t selectionAnchor_ = -1;
    bool scrollToFocus_ = false;
    int subtypeFilter_ = -1;
    std::vector<UndoSnapshot> undoStack_;

    // Simulated background "Check All" job
    bool jobRunning_ = false;
    bool jobSingle_ = false;
    std::size_t jobCategory_ = 0;
    std::chrono::steady_clock::time_point jobCategoryStart_;
    float jobProgress_ = 0.0f;

    // Panel visibility (View menu)
    bool showDisplay_ = true;
    bool showNavigation_ = true;
    bool showStructures_ = true;
    bool showHealth_ = true;
    bool showTool_ = true;
    bool showPreferences_ = false;
    bool showHelp_ = false;
    bool showAbout_ = false;
    bool showHealthInfo_ = true;
    float rightColumnWidth_ = 0.0f;

    // Display panel state (applied to the real Polyscope mesh)
    float baseColor_[3] = {0.62f, 0.66f, 0.72f};
    float categoryColor_[3] = {0.95f, 0.58f, 0.16f};
    float selectedColor_[3] = {0.96f, 0.18f, 0.18f};
    float shellSelectedColor_[3] = {0.40f, 0.70f, 1.00f};
    bool showEdges_ = true;
    bool ghost_ = false;
    float ghostOpacity_ = 0.35f;
    int previewMode_ = 0;
    int overlayCap_ = 5000; ///< Above this, 3D shows face tint only (no edge tubes / vertex spheres).
    int listCap_ = 100;     ///< Above this, the Tool panel shows a summary instead of a defect list.

    // Fake tool/operator parameters
    double sliverMinAltitude_ = 1.0e-4;
    double sliverMinArea_ = 1.0e-8;
    double sliverMaxAspect_ = 1000.0;
    int sliverStrategy_ = 0;
    bool intersectIgnoreAdjacent_ = true;
    double intersectTolerance_ = 1.0e-9;
    int holeMaxEdges_ = 500;
    int holeFillMethod_ = 1;
    int nonManifoldStrategy_ = 0;
    double degenerateAreaEps_ = 1.0e-12;
    float decimateRatio_ = 0.5f;
    bool decimatePreserveBoundary_ = true;
    bool decimatePreserveFeatures_ = true;
    double weldTolerance_ = 1.0e-6;
    int scaleTargetUnit_ = 1;

    std::string statusMessage_;
    std::chrono::steady_clock::time_point statusMessageTime_;
};

} // namespace ogun::mockup
