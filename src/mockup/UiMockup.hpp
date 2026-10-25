#pragma once

#include "mesh/Mesh.hpp"
#include "mesh/Units.hpp"

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <string>
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
};

struct HealthCategory {
    const char* name = "";
    const char* group = "";
    const char* tooltip = "";
    const char* metricLabel = "";
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
};

/// Throwaway clickable prototype of the layout proposed in UI_DESIGN.md.
/// Defect data is fake; only the camera, display styling and Polyscope structures are real.
class UiMockup {
public:
    UiMockup(Mesh mesh, std::string sourceLabel);

    void run();

private:
    struct UndoSnapshot {
        std::string label;
        std::vector<std::vector<MockDefect>> defects;
        std::vector<HealthStatus> statuses;
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
    void drawPreferencesWindow();
    void drawHelpWindow();
    void handleKeyboardShortcuts();
    void handleViewportClick();

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
    void resetLayout();

    [[nodiscard]] HealthCategory* selectedCategory();
    [[nodiscard]] std::size_t selectedRowCount(const HealthCategory& category) const;
    [[nodiscard]] bool watertightKnown() const;
    [[nodiscard]] bool isWatertight() const;

    Mesh mesh_;
    std::string sourceLabel_;
    BoundingBox bbox_;
    Unit fileUnit_ = Unit::Millimeter;
    bool modified_ = false;

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
