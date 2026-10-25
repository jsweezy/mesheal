#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "io/StlWriter.hpp"
#include "mesh/CgalMesh.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"
#include "repair/SliverHealer.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace ogun {

/// Active defect highlight mode in the UI and 3D viewport.
enum class SelectionMode {
    None,
    AllThinFaces,
    SingleFace
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

    /// Highlighted triangles as index triples into `overlayPoints` (the "Selected Defect Faces" surface).
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

/// User-adjustable styles of the defect highlight and repair preview overlays.
struct OverlayAppearance {
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
    bool hadDiagnosticsReport = false;
    SelectionMode selectionMode = SelectionMode::None;
    Eigen::Index selectedFaceIndex = -1;
    HealAction selectedHealAction = HealAction::None;
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

    /// Replaces the current mesh directly. Clears the undo history.
    void setMesh(Mesh mesh, std::string sourceLabel = "Custom Mesh");

    /// Exports the current mesh as Binary (default) or ASCII STL. Populates `lastErrorMessage()` on failure.
    bool saveStlFile(const std::filesystem::path& filepath, StlFormat format = StlFormat::Binary);

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
    // Diagnostics ("Check Manifold")
    // -------------------------------------------------------------------------

    [[nodiscard]] const ThinFaceThresholds& thresholds() const noexcept {
        return thresholds_;
    }

    void setThresholds(const ThinFaceThresholds& thresholds) noexcept {
        thresholds_ = thresholds;
    }

    /// Runs `ThinFaceChecker::check(mesh_, thresholds_)` and updates the active report.
    /// Automatically selects `SelectionMode::AllThinFaces` if any defects are detected.
    const ThinFaceReport& runCheckManifold();

    [[nodiscard]] bool hasDiagnosticsReport() const noexcept {
        return hasDiagnosticsReport_;
    }

    [[nodiscard]] const ThinFaceReport& thinFaceReport() const noexcept {
        return thinFaceReport_;
    }

    // -------------------------------------------------------------------------
    // Defect Selection & 3D Highlight / Navigation State
    // -------------------------------------------------------------------------

    /// Selects the entire "Thin / Near-Zero-Area Faces" defect category (highlights all offending faces).
    void selectAllThinFaces();

    /// Selects and highlights a single facet by its 0-based face index, analyzes it, and builds
    /// the live 2-ring repair preview (choosing the recommended `HealAction` for a newly selected face).
    void selectSingleFace(Eigen::Index faceIndex);

    /// Clears any active facet highlight selection.
    void clearSelection() noexcept;

    [[nodiscard]] SelectionMode selectionMode() const noexcept {
        return selectionMode_;
    }

    [[nodiscard]] Eigen::Index selectedFaceIndex() const noexcept {
        return selectedFaceIndex_;
    }

    [[nodiscard]] const std::vector<Eigen::Index>& highlightedFaceIndices() const noexcept {
        return highlightedFaceIndices_;
    }

    /// Builds per-face surface colors and 3D curve/point overlay geometry for the current selection.
    [[nodiscard]] HighlightOverlayGeometry buildHighlightOverlay() const;

    [[nodiscard]] bool ghostMeshEnabled() const noexcept {
        return ghostMeshEnabled_;
    }

    void setGhostMeshEnabled(bool enabled) noexcept {
        ghostMeshEnabled_ = enabled;
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

    /// Computes the active selection center: exact vertex centroid for `SingleFace`,
    /// or bounding-box center for `AllThinFaces` / `None`.
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
    /// Ignored unless `SelectionMode::SingleFace` is active.
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

    /// True when a valid preview is visible (the GUI should draw the patch and apply the display mode).
    [[nodiscard]] bool repairPreviewShown() const noexcept {
        return repairPreviewVisible_ && repairPreview_.valid;
    }

    /// Base-mesh faces to draw, cutting out highlighted faces and (in cutout mode) preview-replaced faces.
    [[nodiscard]] BaseMeshDisplay buildBaseMeshDisplay() const;

    /// Pre-repair edges of the faces replaced by the preview (empty unless `CutoutOriginalEdges` is active).
    [[nodiscard]] HighlightOverlayGeometry buildRepairPreviewOriginalEdges() const;

    /// Applies `selectedHealAction()` to the selected face of the full mesh (undoable) and re-runs
    /// diagnostics. Returns false (mesh unchanged) if nothing is selected or the repair is unsafe.
    bool commitSelectedFaceRepair();

    /// Batch heals every thin / sliver face (undoable) and re-runs diagnostics.
    /// Returns false (mesh unchanged) if there is nothing to heal or no face could be healed.
    bool fixAllThinFaces();

    /// Outcome of the most recent single-face commit or batch repair.
    [[nodiscard]] const std::string& lastRepairStatusMessage() const noexcept {
        return lastRepairStatusMessage_;
    }

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
    /// Swaps in a modified mesh without clearing the undo history or source label.
    void replaceMeshPreservingHistory(Mesh mesh);
    void rebuildCgalMesh();
    void resetSingleFaceRepairState() noexcept;
    void refreshRepairPreview();

    [[nodiscard]] bool previewUsesCutout() const noexcept;
    /// Appends deduplicated vertices, undirected edges, and triangles of `faces` to `overlay`.
    void appendFaceGeometry(const std::vector<Eigen::Index>& faces, HighlightOverlayGeometry& overlay) const;

    [[nodiscard]] BoundingBox computeActiveSelectionOrMeshBounds() const noexcept;
    [[nodiscard]] static Eigen::Vector3d computeOrthogonalUpVector(
        const Eigen::Vector3d& lookDirection,
        const Eigen::Vector3d& preferredUp) noexcept;

    Mesh mesh_;
    CgalSurfaceMesh cgalMesh_;
    bool cgalMeshValid_ = false;
    std::string loadedSourceLabel_ = "(No mesh loaded)";
    std::string lastErrorMessage_;
    ToleranceConfig toleranceConfig_{};
    ThinFaceThresholds thresholds_{};

    bool hasDiagnosticsReport_ = false;
    ThinFaceReport thinFaceReport_{};

    SelectionMode selectionMode_ = SelectionMode::None;
    Eigen::Index selectedFaceIndex_ = -1;
    std::vector<Eigen::Index> highlightedFaceIndices_;

    SliverAnalysis selectedFaceAnalysis_{};
    HealAction selectedHealAction_ = HealAction::None;
    SingleFaceRepairPreview repairPreview_{};
    bool repairPreviewVisible_ = true;
    PreviewDisplayMode previewDisplayMode_ = PreviewDisplayMode::CutoutOriginalEdges;
    double previewBaseOpacity_ = settings::kDefaultPreviewBaseOpacity;
    double previewPatchOpacity_ = settings::kDefaultPreviewPatchOpacity;
    std::string lastRepairStatusMessage_;

    std::vector<UndoEntry> undoStack_;

    Eigen::Vector3d orbitPivot_ = Eigen::Vector3d::Zero();
    Eigen::Index lastFocusedFaceIndex_ = -1;
    int lastFocusedNormalSign_ = 0;

    bool ghostMeshEnabled_ = false;
    double ghostMeshOpacity_ = 0.25;

    OverlayAppearance overlayAppearance_{};
};

} // namespace ogun
