#pragma once

#include "core/Settings.hpp"
#include "diagnostics/ThinFaceChecker.hpp"
#include "io/SampleMeshes.hpp"
#include "io/StlReader.hpp"
#include "mesh/Mesh.hpp"
#include "mesh/ToleranceConfig.hpp"
#include "mesh/Units.hpp"

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

    [[nodiscard]] bool hasSelectionOverlay() const noexcept {
        return !overlayPoints.empty();
    }
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

/// Headless Presentation Model / Controller for the Ogun STL Repair application.
/// Owns all mesh state, unit/tolerance preferences, diagnostics reports, selection state,
/// and 3D camera focus calculations without any OpenGL or Polyscope window dependencies.
class AppController {
public:
    static constexpr double kDefaultHighlightPointRadius = settings::kDefaultHighlightPointRadius;
    static constexpr double kDefaultHighlightEdgeRadius = settings::kDefaultHighlightEdgeRadius;
    static constexpr double kMinHighlightRelativeRadius = settings::kMinHighlightRelativeRadius;
    static constexpr double kMaxHighlightRelativeRadius = settings::kMaxHighlightRelativeRadius;
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

    /// Replaces the current mesh directly.
    void setMesh(Mesh mesh, std::string sourceLabel = "Custom Mesh");

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

    /// Explicitly rescales vertex coordinates `mesh.V` from `fileUnit()` into `targetUnit`.
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

    /// Selects and highlights a single facet by its 0-based face index.
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
    // Highlight Point & Edge Sizing (Persistent + Dynamic Zoom Scaling)
    // -------------------------------------------------------------------------

    /// Base highlight vertex point radius (relative to full-mesh extent / Polyscope lengthScale).
    [[nodiscard]] double highlightPointRadius() const noexcept {
        return highlightPointRadius_;
    }

    void setHighlightPointRadius(double relativeRadius) noexcept;

    /// Base highlight edge tube radius (relative to full-mesh extent / Polyscope lengthScale).
    [[nodiscard]] double highlightEdgeRadius() const noexcept {
        return highlightEdgeRadius_;
    }

    void setHighlightEdgeRadius(double relativeRadius) noexcept;

    /// Resets highlight point and edge radii back to their default relative values.
    void resetHighlightSizingToDefaults() noexcept;

    /// Whether highlight point and edge radii shrink proportionally as the camera zooms in.
    [[nodiscard]] bool dynamicHighlightSizingEnabled() const noexcept {
        return dynamicHighlightSizingEnabled_;
    }

    void setDynamicHighlightSizingEnabled(bool enabled) noexcept {
        dynamicHighlightSizingEnabled_ = enabled;
    }

    /// Reference camera distance corresponding to viewing the full extent of the loaded mesh.
    [[nodiscard]] double referenceFullMeshViewDistance() const noexcept;

    /// Computes the zoom scale factor in `[kMinHighlightZoomScale, 1.0]` for the given camera world position
    /// and vertical field-of-view (in degrees). Returns `1.0` when viewing the full mesh or when dynamic
    /// sizing is disabled, and shrinks proportionally as the camera approaches the active target.
    [[nodiscard]] double computeHighlightZoomScale(
        const Eigen::Vector3d& cameraPosition,
        double fovDegrees = 45.0) const noexcept;

    /// Effective relative point radius after applying dynamic zoom scaling.
    [[nodiscard]] double effectiveHighlightPointRadius(
        const Eigen::Vector3d& cameraPosition,
        double fovDegrees = 45.0) const noexcept;

    /// Effective relative edge radius after applying dynamic zoom scaling.
    [[nodiscard]] double effectiveHighlightEdgeRadius(
        const Eigen::Vector3d& cameraPosition,
        double fovDegrees = 45.0) const noexcept;

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

private:
    [[nodiscard]] BoundingBox computeActiveSelectionOrMeshBounds() const noexcept;
    [[nodiscard]] static Eigen::Vector3d computeOrthogonalUpVector(
        const Eigen::Vector3d& lookDirection,
        const Eigen::Vector3d& preferredUp) noexcept;

    Mesh mesh_;
    std::string loadedSourceLabel_ = "(No mesh loaded)";
    std::string lastErrorMessage_;
    ToleranceConfig toleranceConfig_{};
    ThinFaceThresholds thresholds_{};

    bool hasDiagnosticsReport_ = false;
    ThinFaceReport thinFaceReport_{};

    SelectionMode selectionMode_ = SelectionMode::None;
    Eigen::Index selectedFaceIndex_ = -1;
    std::vector<Eigen::Index> highlightedFaceIndices_;

    Eigen::Vector3d orbitPivot_ = Eigen::Vector3d::Zero();
    Eigen::Index lastFocusedFaceIndex_ = -1;
    int lastFocusedNormalSign_ = 0;

    bool ghostMeshEnabled_ = false;
    double ghostMeshOpacity_ = 0.25;

    double highlightPointRadius_ = kDefaultHighlightPointRadius;
    double highlightEdgeRadius_ = kDefaultHighlightEdgeRadius;
    bool dynamicHighlightSizingEnabled_ = true;
};

} // namespace ogun
