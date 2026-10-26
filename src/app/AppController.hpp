#pragma once

#include "app/DefectBrowser.hpp"
#include "app/FaceSelection.hpp"
#include "app/HealthCheckRunner.hpp"
#include "app/MergeJobRunner.hpp"
#include "app/RepairJobRunner.hpp"
#include "app/ShellBrowser.hpp"
#include "core/Settings.hpp"
#include "diagnostics/MeshHealth.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "io/StlWriter.hpp"
#include "mesh/CgalMesh.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"
#include "repair/GlobalRepair.hpp"
#include "repair/HoleFiller.hpp"
#include "repair/IntersectionHealer.hpp"
#include "repair/SliverHealer.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace ogun {

/// Lifecycle of one Mesh Health row (UI_DESIGN.md §5.2).
enum class HealthStatus {
    NotRun,  ///< Never checked on this mesh
    Running, ///< Queued or being checked
    Ok,      ///< Checked, nothing found (or an information row)
    Defects, ///< Checked, defects found
    Stale    ///< The mesh changed since this row last ran (Auto off)
};

[[nodiscard]] const char* healthStatusLabel(HealthStatus status) noexcept;

/// State of one Mesh Health row.
struct CategoryHealth {
    HealthStatus status = HealthStatus::NotRun;
    bool autoCheck = true;
    std::optional<CategoryResult> result; ///< Last result (kept while Stale / Running)

    /// Count shown in the row: elements, or `infoCount` for information rows.
    [[nodiscard]] std::size_t displayCount(DefectCategory category) const noexcept;
};

/// Every Mesh Health row, indexed by `DefectCategory`.
struct MeshHealthReport {
    std::array<CategoryHealth, kDefectCategoryCount> rows{};

    [[nodiscard]] CategoryHealth& operator[](DefectCategory category) noexcept {
        return rows[categoryIndex(category)];
    }
    [[nodiscard]] const CategoryHealth& operator[](DefectCategory category) const noexcept {
        return rows[categoryIndex(category)];
    }
};

/// Defect counts of one shell by Mesh Health row. Faces that cross only other shells are counted
/// apart: Merge Shells (a union) removes them, so they do not block cleaning up one shell.
struct ShellHealth {
    std::array<std::size_t, kDefectCategoryCount> counts{};
    std::size_t between = 0;
    std::size_t total = 0;
    std::vector<int> crosses; ///< Other shells whose faces cross this one (sorted)
};

/// Status-bar watertight badge: green only when open and non-manifold edges both checked to 0.
enum class WatertightState { Unknown, Watertight, NotWatertight };

/// What the right-column Tool slot shows (one tool at a time).
enum class ToolSlot {
    None,
    DefectCategory,
    UnitsAndScale,
    RepairAllIntersections,
    RepairAllHoles,
    AlphaWrap,
    Decimate,
    MergeVertices,
    MergeShells
};

/// One line of the Merge Shells checklist (UI_DESIGN.md §16.3), built from Mesh Health.
struct MergeCheck {
    const char* label = "";
    bool blocking = true;                ///< Manifold rejects such shells (false: a warning only)
    bool known = false;                  ///< The Mesh Health row it needs is checked for this mesh
    std::vector<std::size_t> failing;    ///< Selected shells that fail it
};

/// What Merge Shells would do with the selected shells.
struct MergeChecklist {
    std::vector<std::size_t> shells;     ///< Selected shells
    std::size_t faces = 0;
    /// Closed (open edges), oriented (flipped normals), no Fin / other edges: blocking.
    /// No self-crossings (intersecting faces within a shell): a warning, the result may be wrong there.
    std::array<MergeCheck, 4> checks{};
    std::size_t crossingsBetween = 0;    ///< Faces crossing another selected shell (the union removes them)

    /// Two or more shells and every blocking check known and passed.
    [[nodiscard]] bool canApply() const noexcept;
};

/// What the Repair All preview draws: the candidate, the current mesh, or both (current ghosted).
enum class GlobalPreviewShow { Result, Original, Both };

/// Base-mesh back-face rendering (maps to Polyscope's `BackFacePolicy`).
enum class BackFaceMode { Identical, Darker, Custom, Cull };

[[nodiscard]] const char* backFaceModeLabel(BackFaceMode mode) noexcept;

/// What a viewport click on a face did.
enum class PickOutcome {
    None,            ///< No mesh or invalid face
    FocusedDefect,   ///< The face belongs to the selected row: it is now the focus
    SelectedRow,     ///< No row was selected; the row containing the face was selected and focused
    NotInSelectedRow,///< The face is not a defect of the selected row (status message only)
    PickedFace       ///< No row contains the face: it is highlighted and becomes the orbit pivot
};

/// Which side of a selected facet's normal to align the camera with when framing ('.' / 'Shift+.').
enum class FocusNormalSide {
    PreserveViewDirection, ///< Keep the supplied camera view direction
    PositiveNormal,        ///< Look straight at the facet from its +Normal side
    NegativeNormal,        ///< Look straight at the facet from its -Normal side
    AutoToggleNormal       ///< Frame +Normal first; toggle +Normal <-> -Normal on repeated '.' presses
};

/// 3D visual overlay geometry for highlighting selected defective facets in Polyscope.
struct HighlightOverlayGeometry {
    /// Per-face RGB colors in [0, 1] for the surface mesh (length == `mesh.numFaces()`).
    std::vector<Eigen::Vector3d> faceColors;

    /// Deduplicated 3D vertex positions of the currently highlighted faces.
    std::vector<Eigen::Vector3d> overlayPoints;

    /// Undirected 3D edge index pairs into `overlayPoints` for the Curve Network overlay.
    std::vector<std::array<std::size_t, 2>> overlayEdges;

    /// Selected / focused triangles as index triples into `overlayPoints` (the "Selected Defect Faces" surface).
    std::vector<std::array<std::size_t, 3>> overlayFaces;

    /// Base-mesh face index of each entry in `overlayFaces`.
    std::vector<Eigen::Index> overlayFaceIndices;

    [[nodiscard]] bool hasSelectionOverlay() const noexcept {
        return !overlayPoints.empty();
    }
};

[[nodiscard]] inline Eigen::Vector3d rgbFromArray(const std::array<double, 3>& rgb) noexcept {
    return Eigen::Vector3d(rgb[0], rgb[1], rgb[2]);
}

/// How the single-face repair preview is composited with the base mesh.
enum class PreviewDisplayMode {
    TranslucentBase,       ///< Dim the whole base mesh so the old surface shows through around the opaque patch.
    TranslucentPatch,      ///< Keep the base mesh opaque and draw the patch semi-transparent over it.
    CutoutOriginalEdges    ///< Hide the replaced base faces, draw the opaque patch, and overlay the pre-repair edges.
};

[[nodiscard]] const char* previewDisplayModeLabel(PreviewDisplayMode mode) noexcept;

/// Surface overlay style mirroring Polyscope's surface mesh controls (Color, Edges, Edge Color, Width).
struct SurfaceStyle {
    Eigen::Vector3d color = Eigen::Vector3d::Constant(0.5);
    Eigen::Vector3d edgeColor = Eigen::Vector3d::Zero();
    double edgeWidthPx = settings::kDefaultOverlayEdgeWidthPx;
    bool showEdges = true;

    /// Edge width to hand to Polyscope (0 hides the edges).
    [[nodiscard]] double effectiveEdgeWidth() const noexcept {
        return showEdges ? edgeWidthPx : 0.0;
    }

    /// Adopts an edge width read back from Polyscope's UI. Polyscope's "Edges" checkbox writes exactly
    /// 0 (off) or 1 (on), so re-enabling restores the remembered width instead of resetting it to 1.
    /// Returns true when the style changed (the caller should re-apply `effectiveEdgeWidth()`).
    bool adoptPolyscopeEdgeWidth(double polyscopeWidth) noexcept;
};

/// User-adjustable styles of the base mesh, the defect highlight and the repair preview overlays.
struct OverlayAppearance {
    SurfaceStyle baseMesh{
        rgbFromArray(settings::kDefaultCleanFaceColor),
        rgbFromArray(settings::kDefaultBaseEdgeColor)};
    BackFaceMode backFaceMode = BackFaceMode::Darker;
    Eigen::Vector3d backFaceColor = rgbFromArray(settings::kDefaultBackFaceColor);
    bool smoothShade = false;
    Eigen::Vector3d categoryTintColor = rgbFromArray(settings::kDefaultCategoryTintColor);
    SurfaceStyle selectedFaces{
        rgbFromArray(settings::kDefaultSelectedFaceColor),
        rgbFromArray(settings::kDefaultSelectedEdgeColor)};
    SurfaceStyle previewPatch{
        rgbFromArray(settings::kDefaultPreviewFaceColor),
        rgbFromArray(settings::kDefaultPreviewEdgeColor)};
    /// Pre-repair edges drawn over the patch: thinner and semi-transparent so the patch edges show
    /// through where an edge survives the repair, while removed edges show only this color.
    Eigen::Vector3d originalEdgeColor = rgbFromArray(settings::kDefaultOriginalEdgeColor);
    double originalEdgeWidthPx = settings::kDefaultOriginalEdgeWidthPx;
    double originalEdgeOpacity = settings::kDefaultOriginalEdgeOpacity;
    Eigen::Vector3d pointColor = rgbFromArray(settings::kDefaultSelectedPointColor);
    double pointRadiusPx = settings::kDefaultHighlightPointRadiusPx;
};

/// Base-mesh faces to draw. Highlighted faces (drawn by the "Selected Defect Faces" overlay) and, in the
/// cutout preview mode, the faces replaced by the repair preview are cut out rather than drawn over,
/// so overlays never Z-fight against the base mesh.
struct BaseMeshDisplay {
    bool isCut = false;                    ///< False: draw `mesh.F` unchanged (`F` / `faceIndices` empty).
    Eigen::MatrixXi F;                     ///< Remaining base faces (indexing the unchanged `mesh.V`).
    std::vector<Eigen::Index> faceIndices; ///< Base-mesh face index of each row of `F`.
};

/// Computed 3D camera look-at target, up vector, and bounding box for "Zoom to Selected (.)".
struct CameraFocusTarget {
    bool valid = false;
    BoundingBox selectionBounds{};
    Eigen::Vector3d targetCenter = Eigen::Vector3d::Zero();
    Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d cameraUpDirection = Eigen::Vector3d(0.0, 1.0, 0.0);
    double focusRadius = 1.0;
    int normalSideSign = 0; ///< +1 for +Normal, -1 for -Normal, 0 for custom/multi-face view direction
};

/// Snapshot of the mesh and its inspection context, recorded before every mesh modification.
struct UndoEntry {
    Mesh mesh;
    ThinFaceThresholds thresholds{};
    MeshHealthReport health{};
    std::optional<DefectCategory> selectedCategory;
    Eigen::Index focusedFaceIndex = -1;
    HealAction selectedHealAction = HealAction::None;
    IntersectionRepairMethod selectedIslandMethod = IntersectionRepairMethod::None;
    Eigen::Index focusedElementFace = -1; ///< First face of the focused defect of any kind (e.g. an open edge's face)
    HoleFillMethod selectedHoleMethod = HoleFillMethod::None;
    bool modified = false;
    ShellBrowser shells;
    std::string actionLabel;
};

/// Headless Presentation Model / Controller for the Ogun STL Repair application.
/// Owns all mesh state, unit/tolerance preferences, diagnostics reports, selection state,
/// and 3D camera focus calculations without any OpenGL or Polyscope window dependencies.
class AppController {
public:
    static constexpr double kMinHighlightZoomScale = settings::kMinHighlightZoomScale;
    static constexpr double kReferenceFullMeshDistanceFactor = settings::kReferenceFullMeshDistanceFactor;

    AppController();

    /// Resolves the path to the Ogun application/toolbar icon asset (`assets/icons/ogun.png`
    /// or `assets/icons/ogun.jpeg`), returning an empty path if not found.
    [[nodiscard]] static std::filesystem::path resolveApplicationIconPath() noexcept;

    // -------------------------------------------------------------------------
    // Mesh Loading & Access
    // -------------------------------------------------------------------------

    /// Loads an ASCII or binary STL file from disk using the active `fileUnit()` and `toleranceConfig()`.
    bool loadStlFile(const std::filesystem::path& filepath);

    /// Loads the built-in sample cube mesh containing 3 known thin/degenerate triangles.
    void loadSampleDefectiveMesh();

    /// Path of `examples/<fileName>` in the working directory or the source tree (empty if not found).
    [[nodiscard]] static std::filesystem::path resolveExamplePath(std::string_view fileName);

    /// Loads an STL from `examples/` (File > Load Sample). Returns false if it is missing.
    bool loadSampleFile(std::string_view fileName);

    /// Replaces the current mesh directly. Clears the undo history and the Mesh Health report, then
    /// runs Check Mesh when `checkMeshAfterLoad()` is on.
    void setMesh(Mesh mesh, std::string sourceLabel = "Custom Mesh");

    /// Exports the current mesh as Binary (default) or ASCII STL. Populates `lastErrorMessage()` on failure.
    /// A successful save clears `isModified()` and becomes the `currentFilePath()`.
    bool saveStlFile(const std::filesystem::path& filepath, StlFormat format = StlFormat::Binary);

    /// File the mesh was loaded from or last saved to (empty for samples / custom meshes).
    [[nodiscard]] const std::filesystem::path& currentFilePath() const noexcept {
        return currentFilePath_;
    }

    /// True when the mesh changed since it was loaded or saved.
    [[nodiscard]] bool isModified() const noexcept {
        return modified_;
    }

    [[nodiscard]] bool hasMesh() const noexcept {
        return !mesh_.empty();
    }

    [[nodiscard]] const Mesh& mesh() const noexcept {
        return mesh_;
    }

    [[nodiscard]] const std::string& loadedSourceLabel() const noexcept {
        return loadedSourceLabel_;
    }

    [[nodiscard]] const std::string& lastErrorMessage() const noexcept {
        return lastErrorMessage_;
    }

    // -------------------------------------------------------------------------
    // Units, Explicit Coordinate Scaling & Welding Preferences
    // -------------------------------------------------------------------------

    [[nodiscard]] Unit fileUnit() const noexcept {
        return mesh_.fileUnit;
    }

    /// Re-interprets the active file unit without altering vertex coordinates `mesh.V`.
    /// Also converts physical diagnostic thresholds to match the newly selected unit.
    void setFileUnit(Unit newUnit) noexcept;

    /// Explicitly rescales vertex coordinates `mesh.V` from `fileUnit()` into `targetUnit` (undoable).
    void scaleMeshCoordinatesToUnit(Unit targetUnit);

    [[nodiscard]] const ToleranceConfig& toleranceConfig() const noexcept {
        return toleranceConfig_;
    }

    void setToleranceConfig(const ToleranceConfig& config) noexcept {
        toleranceConfig_ = config;
    }

    // -------------------------------------------------------------------------
    // Mesh Health (Check Mesh, Auto re-checks, background jobs)
    // -------------------------------------------------------------------------

    [[nodiscard]] const ThinFaceThresholds& thresholds() const noexcept {
        return thresholds_;
    }

    /// Updates the thin-face thresholds; the Thin row re-runs (Auto) or turns Stale.
    void setThresholds(const ThinFaceThresholds& thresholds);

    [[nodiscard]] double degenerateAreaEpsilon() const noexcept {
        return degenerateAreaEpsilon_;
    }

    void setDegenerateAreaEpsilon(double epsilon);

    /// Intersection tolerance as a fraction of the bounding-box diagonal.
    [[nodiscard]] double intersectionTolerance() const noexcept {
        return intersectionTolerance_;
    }

    void setIntersectionTolerance(double tolerance);

    [[nodiscard]] const MeshHealthReport& healthReport() const noexcept {
        return health_;
    }

    [[nodiscard]] const CategoryHealth& categoryHealth(DefectCategory category) const noexcept {
        return health_[category];
    }

    /// Queues every row (Auto on or off) for checking.
    void checkMesh();

    /// Queues one row ("Re-check" in its Tool panel).
    void recheckCategory(DefectCategory category);

    /// Turns a row's Auto checkbox on or off. Turning it on re-runs a Stale row.
    void setAutoCheck(DefectCategory category, bool enabled);

    /// Default Auto state of each row for newly loaded meshes (Preferences > Diagnostics).
    [[nodiscard]] bool autoCheckDefault(DefectCategory category) const noexcept {
        return autoCheckDefaults_[categoryIndex(category)];
    }
    void setAutoCheckDefault(DefectCategory category, bool enabled) noexcept {
        autoCheckDefaults_[categoryIndex(category)] = enabled;
    }

    [[nodiscard]] bool checkMeshAfterLoad() const noexcept {
        return checkMeshAfterLoad_;
    }
    void setCheckMeshAfterLoad(bool enabled) noexcept {
        checkMeshAfterLoad_ = enabled;
    }

    /// Cancels queued and running checks (rows go back to Stale or Not run).
    void cancelChecks();

    /// Background mode runs checks on a worker thread (the GUI); otherwise they run inline (tests).
    void setBackgroundChecks(bool enabled);

    [[nodiscard]] bool backgroundChecks() const noexcept {
        return backgroundChecks_;
    }

    /// Applies finished background results. Call once per frame; returns true if any row changed.
    bool pollBackgroundChecks();

    /// Blocks until queued background checks finish and applies them (tests).
    void waitForChecks();

    [[nodiscard]] bool checksRunning() const noexcept;

    /// Overall progress of the running check job in [0, 1].
    [[nodiscard]] float checkProgress() const noexcept {
        return checkRunner_.progress();
    }

    [[nodiscard]] std::optional<DefectCategory> currentCheckCategory() const noexcept {
        return checkRunner_.currentCategory();
    }

    /// Defects across all non-information rows that are checked and not stale (in the shell scope).
    [[nodiscard]] std::size_t totalProblemCount() const noexcept;

    /// Count shown in a Mesh Health row: in-scope elements (UI_DESIGN.md §16), or the information count.
    [[nodiscard]] std::size_t rowCount(DefectCategory category) const noexcept;

    /// Row status with the shell scope applied: a Defects row with nothing in scope reads Ok.
    [[nodiscard]] HealthStatus rowStatus(DefectCategory category) const noexcept;

    /// True when element `element` of the row's current result is in the shell scope.
    [[nodiscard]] bool elementInScope(DefectCategory category, std::size_t element) const noexcept;

    [[nodiscard]] WatertightState watertightState() const noexcept;

    /// Incremented on every mesh change; background results from older generations are dropped.
    [[nodiscard]] std::uint64_t meshGeneration() const noexcept {
        return meshGeneration_;
    }

    // -------------------------------------------------------------------------
    // Selected Row, Filters, Multi-Selection & Focus (Tool panel)
    // -------------------------------------------------------------------------

    [[nodiscard]] std::optional<DefectCategory> selectedCategory() const noexcept {
        return selectedCategory_;
    }

    /// Selects a Mesh Health row (opens its Tool, tints its defects) or deselects with `std::nullopt`.
    void selectCategory(std::optional<DefectCategory> category);

    /// Filters, visible rows, selection and focus of the selected row (read-only; use the methods below).
    [[nodiscard]] const DefectBrowser& browser() const noexcept {
        return browser_;
    }

    /// True when the selected row's defects can be browsed (checked, not running or stale).
    [[nodiscard]] bool browsableResult() const noexcept {
        return browser_.result() != nullptr;
    }

    void setSubtypeFilter(int subtype);
    void selectHistogramBin(std::size_t bin, bool extend);
    void stepHistogramBin(int direction);
    void clearHistogramBinFilter();
    void setDefectSort(int column, bool descending);
    void clickDefectRow(std::size_t visiblePosition, bool toggle, bool range);
    void selectAllVisibleDefects();
    void selectNoDefects();
    void invertDefectSelection();
    void focusDefect(std::size_t element);
    /// Faces related to the focused defect (intersections: the faces it crosses; empty otherwise).
    [[nodiscard]] std::span<const Eigen::Index> focusedRelatedFaces() const noexcept;
    /// Focuses the defect owning face `faceIndex` in the selected row (e.g. an intersection partner).
    bool focusRelatedFace(Eigen::Index faceIndex);
    /// Next (+1) / previous (-1) defect within the filter, worst first. Returns false if none.
    bool focusNextDefect(int direction);

    /// Handles a viewport click on mesh face `faceIndex` (see `PickOutcome`).
    PickOutcome pickFace(Eigen::Index faceIndex);

    /// Staged Esc: clear the defect selection, then close the Tool (deselect the row). False if idle.
    bool escape();

    /// Deselects the row, clears the picked face and closes the Tool (dropping Repair All candidates).
    void clearSelection();

    /// Face shown in the single-face Inspector / framed by '.' (-1 if none).
    [[nodiscard]] Eigen::Index focusedFaceIndex() const noexcept;

    /// True when the focused defect is a thin face the repair Inspector can work on.
    [[nodiscard]] bool inspectorActive() const noexcept {
        return selectedFaceIndex_ >= 0;
    }

    /// Faces drawn by the "Selected Defect Faces" overlay (selected + focused defects, or the picked face).
    [[nodiscard]] std::vector<Eigen::Index> highlightedFaceIndices() const;

    [[nodiscard]] ToolSlot activeTool() const noexcept {
        return activeTool_;
    }

    /// Opens Mesh > Units & Scale in the Tool slot (deselects any row).
    void openUnitsAndScale();

    /// Closes the Tool slot and deselects the row.
    void closeTool();

    [[nodiscard]] std::size_t listCap() const noexcept {
        return listCap_;
    }
    void setListCap(std::size_t cap) noexcept {
        listCap_ = std::max<std::size_t>(cap, 1);
    }

    [[nodiscard]] std::size_t overlayCap() const noexcept {
        return overlayCap_;
    }
    void setOverlayCap(std::size_t cap) noexcept {
        overlayCap_ = cap;
    }

    /// Builds per-face colors and the 3D overlay geometry for the current selection.
    [[nodiscard]] HighlightOverlayGeometry buildHighlightOverlay() const;

    // -------------------------------------------------------------------------
    // Shells (UI_DESIGN.md §16): view state, scope and shell edits
    // -------------------------------------------------------------------------

    /// Shells of the current mesh with their visibility and selection (read-only; use the methods below).
    [[nodiscard]] const ShellBrowser& shells() const noexcept {
        return shells_;
    }

    /// Defects of each shell (empty until a row has a current result).
    [[nodiscard]] const std::vector<ShellHealth>& shellHealth() const noexcept {
        return shellHealth_;
    }

    /// True when some non-information row has a result for the current mesh.
    [[nodiscard]] bool shellHealthKnown() const noexcept;

    void clickShell(std::size_t shell, bool toggle, bool range, std::span<const std::size_t> order);
    void selectShellsWhere(const std::function<bool(std::size_t)>& predicate);
    void setShellVisible(std::size_t shell, bool visible);
    void showAllShells();
    void hideAllShells();
    void hideSelectedShells();
    /// Isolate (I): see `ShellBrowser::toggleIsolate()`. Posts a status message either way.
    bool toggleIsolateShells();

    /// Alt+click in 3D: selects (or with `toggle` flips) the shell owning `faceIndex`.
    bool pickShell(Eigen::Index faceIndex, bool toggle);

    /// Deletes the selected shells (undoable). Refuses to delete every shell.
    bool deleteSelectedShells();

    /// Writes the selected shells to one STL file. The mesh and its file are unchanged.
    bool saveSelectedShells(const std::filesystem::path& path, StlFormat format = StlFormat::Binary);

    /// Writes each selected shell to `directory/<stem>_shell_NN.stl`. Returns the number written.
    std::size_t saveEachSelectedShell(const std::filesystem::path& directory, StlFormat format = StlFormat::Binary);

    /// File name stem used by `saveEachSelectedShell` (the loaded file's stem, or "mesh").
    [[nodiscard]] std::string shellFileStem() const;

    // ---- Merge Shells (AppControllerMerge.cpp) ----

    /// Opens "Operation: Merge Shells" in the Tool slot (deselects any row; keeps the shell selection).
    void openMergeShells();

    /// Checklist for the selected shells, from the current Mesh Health results.
    [[nodiscard]] MergeChecklist mergeChecklist() const;

    /// Unions the selected shells with Manifold (one undo step), in the background when background
    /// checks are on. Returns false (nothing started) when the checklist blocks it.
    bool mergeSelectedShells();

    [[nodiscard]] bool mergeRunning() const noexcept {
        return mergeRunner_.busy();
    }

    /// Drops the running merge's result (Manifold cannot be interrupted, so it finishes unseen).
    void cancelMerge();

    /// Blocks until a background merge is done and applies it (tests).
    void waitForMerge();

    [[nodiscard]] bool colorByShell() const noexcept {
        return colorByShell_;
    }
    void setColorByShell(bool enabled) noexcept {
        colorByShell_ = enabled;
    }

    /// Muted color of shell `shell` (hues green..purple, never a defect color).
    [[nodiscard]] static Eigen::Vector3d shellColor(std::size_t shell) noexcept;

    // -------------------------------------------------------------------------
    // Status messages (status bar)
    // -------------------------------------------------------------------------

    [[nodiscard]] const std::string& statusMessage() const noexcept {
        return statusMessage_;
    }
    [[nodiscard]] bool statusMessageIsError() const noexcept {
        return statusMessageIsError_;
    }
    /// Incremented whenever a new status message is posted.
    [[nodiscard]] std::uint64_t statusMessageSerial() const noexcept {
        return statusMessageSerial_;
    }
    void postStatusMessage(std::string message, bool isError = false);

    // -------------------------------------------------------------------------
    // Display state
    // -------------------------------------------------------------------------

    [[nodiscard]] bool ghostMeshEnabled() const noexcept {
        return ghostMeshEnabled_;
    }

    void setGhostMeshEnabled(bool enabled) noexcept {
        ghostMeshEnabled_ = enabled;
    }

    /// Display > Transparency slider: values below 1 turn Ghost on with that opacity, 1 turns it off.
    void setBaseMeshTransparency(double opacity) noexcept;

    /// Opacity the base mesh is drawn with (1 when Ghost is off).
    [[nodiscard]] double effectiveBaseOpacity() const noexcept {
        return ghostMeshEnabled_ ? ghostMeshOpacity_ : 1.0;
    }

    [[nodiscard]] double ghostMeshOpacity() const noexcept {
        return ghostMeshOpacity_;
    }

    void setGhostMeshOpacity(double opacity) noexcept;

    // -------------------------------------------------------------------------
    // Zoom Scale (Near Clip Plane)
    // -------------------------------------------------------------------------

    /// Reference camera distance corresponding to viewing the full extent of the loaded mesh.
    [[nodiscard]] double referenceFullMeshViewDistance() const noexcept;

    /// Computes the zoom scale factor in `[kMinHighlightZoomScale, 1.0]` for the given camera world position
    /// and vertical field-of-view (in degrees). Returns `1.0` when viewing the full mesh and shrinks
    /// proportionally as the camera approaches the active target (used to shrink the near clip plane).
    [[nodiscard]] double computeHighlightZoomScale(
        const Eigen::Vector3d& cameraPosition,
        double fovDegrees = 45.0) const noexcept;

    // -------------------------------------------------------------------------
    // Overlay Styles & Screen-Space Sizing
    // -------------------------------------------------------------------------

    [[nodiscard]] const OverlayAppearance& overlayAppearance() const noexcept {
        return overlayAppearance_;
    }

    /// Updates overlay styles: colors clamped to [0, 1], edge widths and point radius to the px limits in Settings.
    void setOverlayAppearance(const OverlayAppearance& appearance) noexcept;

    void resetOverlayAppearanceToDefaults() noexcept {
        overlayAppearance_ = OverlayAppearance{};
    }

    /// World-space length spanned by one screen pixel at `viewDistance` from a perspective camera.
    [[nodiscard]] static double worldLengthPerPixel(
        double viewDistance,
        double fovDegrees,
        double viewportHeightPx) noexcept;

    /// World-space length of `pixels` screen pixels at the active selection's distance from the camera.
    /// A tube radius of `edgeWidthPx` pixels matches a surface-mesh wireframe edge of that width,
    /// which Polyscope draws as a band of `edgeWidthPx` pixels on each side of the edge.
    [[nodiscard]] double worldLengthAtSelection(
        double pixels,
        const Eigen::Vector3d& cameraPosition,
        double fovDegrees,
        double viewportHeightPx) const noexcept;

    // -------------------------------------------------------------------------
    // 3D Orbit Pivot, Centroid & Normal-Aligned Camera Framing
    // -------------------------------------------------------------------------

    /// Computes the geometric vertex centroid `(v0 + v1 + v2) / 3` of a single facet.
    [[nodiscard]] Eigen::Vector3d computeFaceCentroid(Eigen::Index faceIndex) const noexcept;

    /// Computes the active selection center: the exact centroid of a focused single face, otherwise
    /// the bounding-box center of the focused / selected / visible defects (or the whole mesh).
    [[nodiscard]] Eigen::Vector3d computeSelectionCentroid() const noexcept;

    /// Computes a unit viewing normal for inspecting `faceIndex`. Uses the triangle's
    /// unit normal `N.row(faceIndex)` when non-zero, or a deterministic unit vector
    /// perpendicular to the longest edge for zero-area collinear triangles.
    [[nodiscard]] Eigen::Vector3d computeFaceInspectionNormal(Eigen::Index faceIndex) const noexcept;

    /// Current 3D center of rotation (orbit pivot) in world coordinates.
    /// Automatically updates to `computeSelectionCentroid()` whenever selection changes,
    /// and shifts with the camera during viewport panning.
    [[nodiscard]] const Eigen::Vector3d& orbitPivot() const noexcept {
        return orbitPivot_;
    }

    void setOrbitPivot(const Eigen::Vector3d& pivot) noexcept;

    /// Shifts the active orbit pivot by `worldDelta` (used when panning the camera).
    void translateOrbitPivot(const Eigen::Vector3d& worldDelta) noexcept;

    /// Resets the orbit pivot to the active selection centroid (or mesh center if none).
    void resetOrbitPivotToSelection() noexcept;

    /// Resets the orbit pivot to the full mesh bounding-box center and clears normal-toggle state.
    void resetOrbitPivotToMeshCenter() noexcept;

    /// Notifies the controller that the user manually orbited the camera, so the next
    /// '.' press will align to `+Normal` rather than toggling to `-Normal`.
    void notifyCameraRotated() noexcept {
        lastFocusedNormalSign_ = 0;
    }

    /// Last normal side sign aligned by `focusOnSelection` (+1 for +N, -1 for -N, 0 if none/orbited).
    [[nodiscard]] int lastFocusedNormalSign() const noexcept {
        return lastFocusedNormalSign_;
    }

    /// Computes the 3D bounding box, look-at target (centroid for a single face), camera position,
    /// and orthogonal up vector for the current selection without mutating toggle state.
    [[nodiscard]] CameraFocusTarget computeCameraFocusTarget(
        const Eigen::Vector3d& currentViewDirection = Eigen::Vector3d(-1.0, -1.0, -1.0),
        FocusNormalSide normalSide = FocusNormalSide::PreserveViewDirection,
        const Eigen::Vector3d& preferredUpDirection = Eigen::Vector3d(0.0, 1.0, 0.0)) const;

    /// Stateful camera focus invoked by '.' (`AutoToggleNormal`), 'Shift+.' (`NegativeNormal`),
    /// or UI buttons. Updates `orbitPivot_` to the target centroid and tracks `+N` <-> `-N` toggling.
    CameraFocusTarget focusOnSelection(
        FocusNormalSide normalSide = FocusNormalSide::AutoToggleNormal,
        const Eigen::Vector3d& currentViewDirection = Eigen::Vector3d(-1.0, -1.0, -1.0),
        const Eigen::Vector3d& preferredUpDirection = Eigen::Vector3d(0.0, 1.0, 0.0));

    // -------------------------------------------------------------------------
    // Single-Face Repair Inspector & Live Preview
    // -------------------------------------------------------------------------

    /// Cap / Needle / Microscopic / Dangling Flap analysis of the selected face (default when none).
    [[nodiscard]] const SliverAnalysis& selectedFaceAnalysis() const noexcept {
        return selectedFaceAnalysis_;
    }

    [[nodiscard]] HealAction selectedHealAction() const noexcept {
        return selectedHealAction_;
    }

    /// Chooses the repair strategy for the selected face and re-evaluates the 2-ring preview.
    /// Ignored unless a thin face is focused (`inspectorActive()`).
    void setSelectedHealAction(HealAction action);

    /// Live 2-ring preview of `selectedHealAction()` on the selected face (invalid with a reason if unsafe).
    [[nodiscard]] const SingleFaceRepairPreview& repairPreview() const noexcept {
        return repairPreview_;
    }

    [[nodiscard]] bool repairPreviewVisible() const noexcept {
        return repairPreviewVisible_;
    }

    void setRepairPreviewVisible(bool visible) noexcept {
        repairPreviewVisible_ = visible;
    }

    [[nodiscard]] PreviewDisplayMode previewDisplayMode() const noexcept {
        return previewDisplayMode_;
    }

    void setPreviewDisplayMode(PreviewDisplayMode mode) noexcept {
        previewDisplayMode_ = mode;
    }

    /// Base mesh opacity in `PreviewDisplayMode::TranslucentBase`.
    [[nodiscard]] double previewBaseOpacity() const noexcept {
        return previewBaseOpacity_;
    }

    void setPreviewBaseOpacity(double opacity) noexcept;

    /// Preview patch opacity in `PreviewDisplayMode::TranslucentPatch`.
    [[nodiscard]] double previewPatchOpacity() const noexcept {
        return previewPatchOpacity_;
    }

    void setPreviewPatchOpacity(double opacity) noexcept;

    /// True when a valid single-defect preview (sliver, island or hole) is visible: the GUI draws
    /// `previewPatchMesh()` and applies the display mode.
    [[nodiscard]] bool repairPreviewShown() const noexcept {
        return repairPreviewVisible_ && (holePreviewActive()     ? holePreview_.valid
                                         : islandPreviewActive() ? islandPreview_.valid
                                                                 : repairPreview_.valid);
    }

    /// Patch of the active single-defect preview (the hole or island preview when one is focused).
    [[nodiscard]] const Mesh& previewPatchMesh() const noexcept {
        return holePreviewActive()     ? holePreview_.previewPatchMesh
               : islandPreviewActive() ? islandPreview_.previewPatchMesh
                                       : repairPreview_.previewPatchMesh;
    }

    /// Base faces the active single-defect preview replaces.
    [[nodiscard]] const std::vector<Eigen::Index>& previewSourceFaces() const noexcept {
        return holePreviewActive()     ? holePreview_.sourceFaceIndices
               : islandPreviewActive() ? islandPreview_.sourceFaceIndices
                                       : repairPreview_.sourceFaceIndices;
    }

    /// Base-mesh faces to draw, cutting out highlighted faces and (in cutout mode) preview-replaced faces.
    [[nodiscard]] BaseMeshDisplay buildBaseMeshDisplay() const;

    /// Pre-repair edges of the faces replaced by the preview (empty unless `CutoutOriginalEdges` is active).
    [[nodiscard]] HighlightOverlayGeometry buildRepairPreviewOriginalEdges() const;

    /// Applies `selectedHealAction()` to the selected face of the full mesh (undoable) and re-runs
    /// diagnostics. Returns false (mesh unchanged) if nothing is selected or the repair is unsafe.
    bool commitSelectedFaceRepair();

    /// Batch heals every thin / sliver face in the shell scope (undoable) and re-runs diagnostics.
    /// Returns false (mesh unchanged) if there is nothing to heal or no face could be healed.
    bool fixAllThinFaces();

    /// Applies each selected thin face's recommended repair as one undoable step.
    bool fixSelectedThinFaces();

    /// Splits every Touching non-manifold edge in the shell scope (one undo step): the solids that touch
    /// get their own copies of the edge's vertices at the same positions. Fin / other edges are skipped.
    bool fixAllNonManifoldEdges();

    /// Splits the selected non-manifold edges (one undo step); Fin / other edges are skipped.
    bool fixSelectedNonManifoldEdges();

    /// Gives each extra fan of every non-manifold vertex in the shell scope its own copy (one undo step).
    bool fixAllNonManifoldVertices();

    /// Splits the selected non-manifold vertices (one undo step).
    bool fixSelectedNonManifoldVertices();

    /// Outcome of the most recent single-face commit or batch repair.
    [[nodiscard]] const std::string& lastRepairStatusMessage() const noexcept {
        return lastRepairStatusMessage_;
    }

    // -------------------------------------------------------------------------
    // Intersecting faces: per-island Inspector (AppControllerIntersections.cpp)
    // -------------------------------------------------------------------------

    /// True when the focused intersecting face belongs to a repairable island (crossings within its shell).
    [[nodiscard]] bool islandInspectorActive() const noexcept {
        return focusedIsland_.has_value();
    }

    /// Island of the focused face (nullptr when `islandInspectorActive()` is false).
    [[nodiscard]] const IntersectionIsland* focusedIsland() const noexcept;

    [[nodiscard]] const IslandAnalysis& focusedIslandAnalysis() const noexcept {
        return focusedIslandAnalysis_;
    }

    [[nodiscard]] IntersectionRepairMethod selectedIslandMethod() const noexcept {
        return selectedIslandMethod_;
    }

    /// Chooses the focused island's repair method and recomputes its preview.
    void setSelectedIslandMethod(IntersectionRepairMethod method);

    /// Live preview and scorecard of `selectedIslandMethod()` on the focused island.
    [[nodiscard]] const IslandRepairPreview& islandPreview() const noexcept {
        return islandPreview_;
    }

    /// Applies the selected method to the focused island (undoable). Refused when the repair would
    /// create new crossings.
    bool commitIslandRepair();

    /// Repairs the islands of the selected intersecting faces with their recommended methods and
    /// fallbacks (one undo step).
    bool fixSelectedIntersections();

    [[nodiscard]] const IntersectionRepairSettings& islandRepairSettings() const noexcept {
        return islandSettings_;
    }

    /// Dilation rings and the CGAL step limit; refreshes the island preview.
    void setIslandRepairSettings(const IntersectionRepairSettings& settings);

    /// Islands of the current Intersections result (in the shell scope); 0 when it is not current.
    [[nodiscard]] std::size_t intersectionIslandCount();

    // -------------------------------------------------------------------------
    // Open edges: per-hole Inspector (AppControllerHoles.cpp)
    // -------------------------------------------------------------------------

    /// True when the focused open edge belongs to a hole (the Inspector shows its fills).
    [[nodiscard]] bool holeInspectorActive() const noexcept {
        return focusedHole_.has_value();
    }

    /// Hole of the focused open edge (nullptr when `holeInspectorActive()` is false).
    [[nodiscard]] const HoleLoop* focusedHole() const noexcept;

    /// 1-based number of the focused hole (the "Hole" column), 0 when none.
    [[nodiscard]] std::size_t focusedHoleNumber() const noexcept {
        return focusedHole_ ? *focusedHole_ + 1 : 0;
    }

    [[nodiscard]] HoleFillMethod selectedHoleMethod() const noexcept {
        return selectedHoleMethod_;
    }

    /// Chooses the focused hole's fill method and recomputes its preview.
    void setSelectedHoleMethod(HoleFillMethod method);

    /// Live preview and scorecard of `selectedHoleMethod()` on the focused hole.
    [[nodiscard]] const HoleFillPreview& holePreview() const noexcept {
        return holePreview_;
    }

    /// Applies the selected method to the focused hole (one undo step). Refused when the fill would
    /// cross other faces.
    bool commitHoleFill();

    /// Fills the holes of the selected open edges with their recommended methods (one undo step).
    bool fixSelectedHoles();

    [[nodiscard]] const HoleFillSettings& holeFillSettings() const noexcept {
        return holeSettings_;
    }

    /// Patch rings, size limits and crack snapping; refreshes the hole preview.
    void setHoleFillSettings(const HoleFillSettings& settings);

    /// Holes of the current mesh with at least one rim face in the shell scope.
    [[nodiscard]] std::size_t holeCountInScope();

    // -------------------------------------------------------------------------
    // Repair All operator for intersections and holes (AppControllerIntersections.cpp)
    // -------------------------------------------------------------------------

    /// Opens "Operation: Repair All Intersections" in the Tool slot (deselects any row).
    void openRepairAllIntersections();

    /// Opens "Operation: Repair All Holes" in the Tool slot (deselects any row).
    void openRepairAllHoles();

    /// Opens "Operation: Alpha Wrap" (Mesh > Clean Up): the same operator with only the wrap.
    void openAlphaWrap();

    // -------------------------------------------------------------------------
    // Face selection: Shift+click, box (B) and circle (C) in the viewport (AppControllerSelection.cpp).
    // Decimate and Merge Vertices work on the selected faces when there are any.
    // -------------------------------------------------------------------------

    [[nodiscard]] std::size_t selectedFaceCount() const noexcept {
        return faceSelectedCount_;
    }

    [[nodiscard]] bool faceSelected(Eigen::Index face) const noexcept {
        return face >= 0 && static_cast<std::size_t>(face) < faceSelected_.size() &&
               faceSelected_[static_cast<std::size_t>(face)] != 0;
    }

    /// Adds (`select`) or removes the faces; returns how many changed.
    std::size_t selectFaces(std::span<const Eigen::Index> faces, bool select);
    void toggleFaceSelected(Eigen::Index face);
    void selectAllFaces();
    /// Returns false when nothing was selected.
    bool clearFaceSelection();

    /// Box / circle selection of the faces whose centre is inside the shape on screen; only the faces
    /// seen there unless `selectThrough()` (Blender's X-ray). Returns how many changed.
    std::size_t selectFacesInRectangle(const ScreenView& view, const Eigen::Vector2d& a, const Eigen::Vector2d& b, bool select);
    std::size_t selectFacesInCircle(const ScreenView& view, const Eigen::Vector2d& center, double radius, bool select);
    /// The first face under a screen point (Shift+click), by a ray cast: dense meshes leave no room
    /// for polyscope's pick, which takes a vertex when the click is near one.
    [[nodiscard]] std::optional<Eigen::Index> faceAtScreenPoint(const ScreenView& view, const Eigen::Vector2d& point) const;

    [[nodiscard]] bool selectThrough() const noexcept {
        return selectThrough_;
    }
    void setSelectThrough(bool through) noexcept {
        selectThrough_ = through;
    }

    /// Faces Decimate / Merge Vertices work on: the selected faces, else the shells in scope (empty = all).
    [[nodiscard]] std::vector<std::uint8_t> operatorScopeMask() const;
    [[nodiscard]] std::size_t operatorScopeFaceCount() const;
    /// "the 1,204 selected faces", "the shells in scope" or "the whole mesh".
    [[nodiscard]] std::string operatorScopeText() const;
    [[nodiscard]] double meanEdgeLength() const;

    /// Opens "Operation: Decimate" (Mesh > Simplify) on the same operator.
    void openDecimate();

    /// Opens "Operation: Merge Vertices" (Mesh > Clean Up > Weld / Merge Vertices) on the same operator.
    void openMergeVertices();

    /// Row the open Repair All operator repairs: Intersections or OpenEdges (Alpha Wrap: Intersections).
    [[nodiscard]] DefectCategory globalRepairTarget() const noexcept {
        return activeTool_ == ToolSlot::RepairAllHoles ? DefectCategory::OpenEdges : DefectCategory::Intersections;
    }

    /// Methods the open operator offers.
    [[nodiscard]] std::span<const GlobalRepairMethod> globalRepairMethods() const noexcept {
        switch (activeTool_) {
        case ToolSlot::AlphaWrap:
            return kWrapMethods;
        case ToolSlot::Decimate:
            return kDecimateMethods;
        case ToolSlot::MergeVertices:
            return kMergeMethods;
        default:
            return globalRepairMethodsFor(globalRepairTarget());
        }
    }

    /// True for the Repair All operators and the Mesh-menu operators built on them.
    [[nodiscard]] bool repairAllOpen() const noexcept {
        return activeTool_ == ToolSlot::RepairAllIntersections || activeTool_ == ToolSlot::RepairAllHoles ||
               activeTool_ == ToolSlot::AlphaWrap || activeTool_ == ToolSlot::Decimate ||
               activeTool_ == ToolSlot::MergeVertices;
    }

    [[nodiscard]] GlobalRepairMethod globalRepairMethod() const noexcept {
        return globalMethod_;
    }

    /// Picks the method; shows its candidate when one is already computed with the current parameters.
    void setGlobalRepairMethod(GlobalRepairMethod method);

    [[nodiscard]] const GlobalRepairParams& globalRepairParams() const noexcept {
        return globalParams_;
    }

    void setGlobalRepairParams(const GlobalRepairParams& params);

    /// Computes the selected method's candidate (background in the GUI, inline in tests).
    void previewGlobalRepair();

    /// Computes a candidate for every method of the open operator, one after another.
    void compareAllGlobalRepairs();

    void cancelGlobalRepair();

    /// Blocks until the background Repair All job is done and collects its candidates (tests).
    void waitForGlobalRepair();

    [[nodiscard]] bool globalRepairRunning() const noexcept {
        return repairRunner_.busy();
    }

    [[nodiscard]] float globalRepairProgress() const noexcept {
        return repairRunner_.progress();
    }

    [[nodiscard]] std::optional<GlobalRepairMethod> globalRepairCurrentMethod() const noexcept {
        return repairRunner_.currentMethod();
    }

    /// Computed candidates for the current mesh, at most one per method (in method order).
    [[nodiscard]] const std::vector<RepairCandidate>& globalCandidates() const noexcept {
        return globalCandidates_;
    }

    /// True when `candidate` was computed with the current parameters.
    [[nodiscard]] bool candidateIsCurrent(const RepairCandidate& candidate) const noexcept {
        return candidate.params == globalParams_;
    }

    /// Candidate drawn in 3D (nullptr when none).
    [[nodiscard]] const RepairCandidate* shownGlobalCandidate() const noexcept;

    void showGlobalCandidate(GlobalRepairMethod method);

    [[nodiscard]] GlobalPreviewShow globalPreviewShow() const noexcept {
        return globalPreviewShow_;
    }

    void setGlobalPreviewShow(GlobalPreviewShow show) noexcept {
        globalPreviewShow_ = show;
    }

    [[nodiscard]] bool colorByDeviation() const noexcept {
        return colorByDeviation_;
    }

    void setColorByDeviation(bool enabled) noexcept {
        colorByDeviation_ = enabled;
    }

    /// True when the GUI should draw the shown candidate (Repair All open, Show is not Original).
    [[nodiscard]] bool globalPreviewShown() const noexcept;

    /// Replaces the mesh with the shown candidate (one undo step) and re-runs the Auto checks.
    bool applyGlobalRepair();

    // -------------------------------------------------------------------------
    // Universal Mesh Modification Undo (Ctrl+Z / Cmd+Z)
    // -------------------------------------------------------------------------

    [[nodiscard]] bool canUndo() const noexcept {
        return !undoStack_.empty();
    }

    /// Label of the modification that `undoLastModification()` would revert (empty if none).
    [[nodiscard]] std::string lastUndoActionLabel() const;

    /// Restores the mesh, thresholds, diagnostics and selection recorded before the last modification.
    bool undoLastModification();

private:
    /// Records the current mesh and inspection context before a modification.
    void pushUndoEntry(std::string actionLabel);
    /// Swaps in a modified mesh: new generation, focus and picks cleared, history kept. The shells are
    /// rebuilt (keeping their view state) unless `shells` restores a snapshot.
    void replaceMeshPreservingHistory(Mesh mesh, std::optional<ShellBrowser> shells = std::nullopt);
    /// Second half of every mesh edit (after `pushUndoEntry`): swap the mesh and re-check.
    void finishMeshEdit(Mesh mesh);
    /// Applies a finished merge (one undo step), selects the merged shells and reports the numbers.
    bool applyMergeResult(FinishedMerge finished);
    /// Applies a finished background merge. Returns true if one arrived.
    bool pollMergeJob();
    /// Shared by the four non-manifold Fix All / Fix Selected actions.
    bool splitNonManifold(DefectCategory category, bool selectedOnly);
    /// Auto rows are queued, other checked rows turn Stale.
    void onMeshEdited();
    /// The given row's parameters changed: re-run (Auto) or mark Stale.
    void onCheckParamsChanged(DefectCategory category);
    void resetHealthReport();
    void markRunning(DefectCategory category) noexcept;
    void launchQueuedChecks();
    void applyCheckResult(CategoryResult result);
    void syncBrowserResult();
    /// The row's result belongs to the current mesh (not a stale or running row's old result).
    [[nodiscard]] const CategoryResult* currentResult(DefectCategory category) const noexcept;
    /// Recomputes one row's scope mask and count.
    void computeScope(DefectCategory category);
    /// Shell visibility or selection changed: recompute every scope and refresh the browser.
    void onShellsChanged();
    /// Hands the selected row's scope mask to the browser (refocusing if the focus was dropped).
    void applyScopeToBrowser();
    /// Recomputes `shellHealth_` and the nested-shell tags from the current results.
    void refreshShellHealth();
    /// Refreshes the Inspector, orbit pivot and normal toggle after the browser focus changed.
    void onFocusChanged();
    [[nodiscard]] CheckParams checkParams() const noexcept;
    [[nodiscard]] std::vector<Eigen::Index> activeFrameFaces() const;
    /// Focused defect (or picked face) when it is a single face; -1 otherwise.
    [[nodiscard]] Eigen::Index focusedSingleFace() const noexcept;
    void analyzeFocusedFace(Eigen::Index faceIndex);
    void rebuildCgalMesh();
    void resetSingleFaceRepairState() noexcept;
    void refreshRepairPreview();
    [[nodiscard]] bool islandPreviewActive() const noexcept {
        return focusedIsland_.has_value() && selectedIslandMethod_ != IntersectionRepairMethod::None;
    }
    /// Islands of the current Intersections result (computed lazily, cleared on every mesh change).
    const std::vector<IntersectionIsland>& intersectionIslands();
    /// Shell id of every face (ShellBrowser numbering).
    [[nodiscard]] std::vector<int> faceShellIds() const;
    /// Per-face scope mask for repairs (empty when the whole mesh is in scope).
    [[nodiscard]] std::vector<std::uint8_t> repairScopeMask() const;
    /// Updates the focused island after the focus changed (Intersections row only).
    void refreshIslandFocus(Eigen::Index face);
    void resetIslandRepairState() noexcept;
    void refreshIslandPreview();
    [[nodiscard]] bool holePreviewActive() const noexcept {
        return focusedHole_.has_value() && selectedHoleMethod_ != HoleFillMethod::None;
    }
    /// Holes of the current mesh (computed lazily, cleared on every mesh change).
    const std::vector<HoleLoop>& holeLoops();
    /// Updates the focused hole after the focus changed (Open edges row only).
    void refreshHoleFocus(bool openEdgeFocus);
    void resetHoleRepairState() noexcept;
    void refreshHolePreview();
    /// Rim faces of the focused hole (empty when none).
    [[nodiscard]] std::span<const Eigen::Index> focusedHoleFaces() const noexcept;
    /// Stops the Repair All job and forgets its candidates (mesh edit, tool closed).
    void discardGlobalCandidates();
    /// Opens the Repair All operator for `slot` with `method` selected.
    void openRepairAll(ToolSlot slot, GlobalRepairMethod method);
    /// Moves finished Repair All candidates into the table. Returns true if any arrived.
    bool pollRepairJobs();
    void queueGlobalRepairs(std::vector<GlobalRepairMethod> methods);
    void storeCandidate(RepairCandidate candidate);

    [[nodiscard]] bool previewUsesCutout() const noexcept;
    /// Appends deduplicated vertices, undirected edges, and (optionally) triangles of `faces` to `overlay`.
    void appendFaceGeometry(
        const std::vector<Eigen::Index>& faces,
        HighlightOverlayGeometry& overlay,
        bool addTriangles = true,
        std::unordered_map<int, std::size_t>* nodeOf = nullptr) const;

    [[nodiscard]] BoundingBox computeActiveSelectionOrMeshBounds() const noexcept;
    [[nodiscard]] static Eigen::Vector3d computeOrthogonalUpVector(
        const Eigen::Vector3d& lookDirection,
        const Eigen::Vector3d& preferredUp) noexcept;

    Mesh mesh_;
    CgalSurfaceMesh cgalMesh_;
    bool cgalMeshValid_ = false;
    std::string loadedSourceLabel_ = "(No mesh loaded)";
    std::filesystem::path currentFilePath_;
    bool modified_ = false;
    std::string lastErrorMessage_;
    ToleranceConfig toleranceConfig_{};
    ThinFaceThresholds thresholds_{};
    double degenerateAreaEpsilon_ = settings::kDefaultDegenerateAreaEpsilon;
    double intersectionTolerance_ = settings::kDefaultIntersectionTolerance;

    MeshHealthReport health_{};
    ShellBrowser shells_;
    std::array<std::vector<std::uint8_t>, kDefectCategoryCount> scopeMasks_{}; ///< Empty: every element in scope
    std::array<std::size_t, kDefectCategoryCount> scopedCounts_{};
    std::vector<ShellHealth> shellHealth_;
    bool colorByShell_ = settings::kDefaultColorByShell;
    std::array<bool, kDefectCategoryCount> autoCheckDefaults_{};
    bool checkMeshAfterLoad_ = settings::kDefaultCheckMeshAfterLoad;
    bool backgroundChecks_ = false;
    std::uint64_t meshGeneration_ = 0;
    HealthCheckRunner checkRunner_;

    std::optional<DefectCategory> selectedCategory_;
    DefectBrowser browser_;
    ToolSlot activeTool_ = ToolSlot::None;
    std::vector<std::uint8_t> faceSelected_;
    std::size_t faceSelectedCount_ = 0;
    bool selectThrough_ = false;
    mutable FaceVisibility faceVisibility_;
    mutable std::uint64_t faceVisibilityGeneration_ = 0;
    /// The ray-cast tree of the current mesh, rebuilt after an edit.
    const FaceVisibility& faceVisibility() const;
    std::size_t listCap_ = settings::kDefaultListCap;
    std::size_t overlayCap_ = settings::kDefaultOverlayCap;
    Eigen::Index pickedFaceIndex_ = -1;   ///< Face clicked with no row containing it
    Eigen::Index selectedFaceIndex_ = -1; ///< Thin face shown in the repair Inspector

    std::string statusMessage_;
    bool statusMessageIsError_ = false;
    std::uint64_t statusMessageSerial_ = 0;

    SliverAnalysis selectedFaceAnalysis_{};
    HealAction selectedHealAction_ = HealAction::None;
    SingleFaceRepairPreview repairPreview_{};
    bool repairPreviewVisible_ = true;
    PreviewDisplayMode previewDisplayMode_ = PreviewDisplayMode::CutoutOriginalEdges;
    double previewBaseOpacity_ = settings::kDefaultPreviewBaseOpacity;
    double previewPatchOpacity_ = settings::kDefaultPreviewPatchOpacity;
    std::string lastRepairStatusMessage_;

    // Intersecting faces: islands, Inspector and Repair All
    std::vector<IntersectionIsland> islands_;
    bool islandsValid_ = false;
    std::optional<std::size_t> focusedIsland_;
    IslandAnalysis focusedIslandAnalysis_{};
    IntersectionRepairMethod selectedIslandMethod_ = IntersectionRepairMethod::None;
    IslandRepairPreview islandPreview_{};
    IntersectionRepairSettings islandSettings_{};
    GlobalRepairMethod globalMethod_ = GlobalRepairMethod::PatchIslands;
    GlobalRepairParams globalParams_{};
    std::vector<RepairCandidate> globalCandidates_;
    std::optional<GlobalRepairMethod> shownCandidate_;
    GlobalPreviewShow globalPreviewShow_ = GlobalPreviewShow::Result;
    bool colorByDeviation_ = false;
    RepairJobRunner repairRunner_;
    MergeJobRunner mergeRunner_;

    // Open edges: holes and the hole Inspector
    std::vector<HoleLoop> holes_;
    bool holesValid_ = false;
    std::optional<std::size_t> focusedHole_;
    HoleFillMethod selectedHoleMethod_ = HoleFillMethod::None;
    HoleFillPreview holePreview_{};
    HoleFillSettings holeSettings_{};
    std::optional<std::size_t> lastFramedHole_; ///< Hole the last `.` framed (for the +N / -N toggle)

    std::vector<UndoEntry> undoStack_;

    Eigen::Vector3d orbitPivot_ = Eigen::Vector3d::Zero();
    Eigen::Index lastFocusedFaceIndex_ = -1;
    int lastFocusedNormalSign_ = 0;

    bool ghostMeshEnabled_ = false;
    double ghostMeshOpacity_ = settings::kDefaultGhostOpacity;

    OverlayAppearance overlayAppearance_{};
};

} // namespace ogun
