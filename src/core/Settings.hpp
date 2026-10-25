#pragma once

#include <array>
#include <cstddef>

namespace ogun::settings {

// -----------------------------------------------------------------------------
// 3D Viewport & Defect Highlight Overlay Sizing Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultHighlightPointRadiusPx = 3.0;     // Screen-space radius (px) of highlighted defect vertex spheres
inline constexpr double kMinHighlightPointRadiusPx     = 0.5;     // Minimum defect vertex sphere radius (px)
inline constexpr double kMaxHighlightPointRadiusPx     = 20.0;    // Maximum defect vertex sphere radius (px)
inline constexpr double kMinHighlightZoomScale       = 1.0e-5;    // Minimum zoom factor used to shrink the near clip plane when zoomed in close
inline constexpr double kReferenceFullMeshDistanceFactor = 1.5;   // Multiplier on mesh bounding-box diagonal for nominal camera distance

// -----------------------------------------------------------------------------
// Defect & Repair Preview Overlay Appearance Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultOverlayEdgeWidthPx = 1.0;         // Default screen-space edge width (px) of defect faces/edges and repair preview patch edges
inline constexpr double kMinOverlayEdgeWidthPx     = 0.25;        // Minimum overlay edge width (px)
inline constexpr double kMaxOverlayEdgeWidthPx     = 10.0;        // Maximum overlay edge width (px)
inline constexpr std::array<double, 3> kDefaultSelectedFaceColor = {0.96, 0.18, 0.18}; // RGB fill of the selected defective facet(s)
inline constexpr std::array<double, 3> kDefaultSelectedEdgeColor = {1.00, 0.15, 0.10}; // RGB of the selected defect edge overlay
inline constexpr std::array<double, 3> kDefaultSelectedPointColor = {1.00, 0.85, 0.10}; // RGB of the selected defect vertex spheres
inline constexpr std::array<double, 3> kDefaultPreviewFaceColor  = {0.20, 0.85, 0.30}; // RGB fill of the 2-ring repair preview patch
inline constexpr std::array<double, 3> kDefaultPreviewEdgeColor  = {0.00, 0.00, 0.00}; // RGB wireframe of the 2-ring repair preview patch
inline constexpr std::array<double, 3> kDefaultOriginalEdgeColor = {0.10, 0.30, 0.90}; // RGB of the pre-repair edges drawn over the preview patch
inline constexpr double kDefaultOriginalEdgeWidthPx = 0.5;        // Pre-repair edge width (px); thinner than the patch edges so surviving edges keep a visible patch-colored outline
inline constexpr double kDefaultOriginalEdgeOpacity = 0.5;        // Pre-repair edge opacity so the patch edges show through where an edge survives the repair
inline constexpr double kDefaultPreviewBaseOpacity  = 0.40;       // Base mesh opacity in "Translucent Base" preview mode
inline constexpr double kDefaultPreviewPatchOpacity = 0.50;       // Preview patch opacity in "Translucent Patch" preview mode
inline constexpr double kMinPreviewOpacity          = 0.05;       // Minimum preview opacity

// -----------------------------------------------------------------------------
// Camera Navigation & FOV Bounds
// -----------------------------------------------------------------------------
inline constexpr double kDefaultCameraFovDegrees = 45.0;          // Polyscope default vertical field of view
inline constexpr double kMinCameraFovDegrees     = 5.0;           // Minimum clamped FOV to prevent projection singularity
inline constexpr double kMaxCameraFovDegrees     = 160.0;         // Maximum clamped FOV for wide-angle view
inline constexpr float  kDragIgnoreThreshold     = 0.01f;         // Minimum pixel delta to register viewport camera drag

// -----------------------------------------------------------------------------
// Thin Face Diagnostics Threshold Defaults (referenced to Millimeters)
// -----------------------------------------------------------------------------
inline constexpr double kDefaultThinFaceMinAltitude     = 1.0e-4; // Minimum allowable triangle altitude h_min = 2A / l_max (mm)
inline constexpr double kDefaultThinFaceMinArea         = 1.0e-8; // Minimum allowable triangle surface area A (mm^2)
inline constexpr double kDefaultThinFaceMaxAspectRatio  = 1000.0;// Dimensionless maximum aspect ratio ceiling (l_max / h_min)

// -----------------------------------------------------------------------------
// Sliver Healer Classification & Repair Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultNeedleEdgeRatio      = 0.05;      // l_min / l_max below this ratio classifies face as Needle
inline constexpr double kDefaultCapAngleDeg          = 170.0;     // Maximum interior angle (deg) above which face is classified as Cap
inline constexpr double kDefaultCoplanarDot          = 0.95;      // Minimum normal alignment dot product for coplanar flips
inline constexpr double kDefaultMinNormalDot         = 0.1;       // Minimum n_old . n_new around collapsed edges to prevent inversion
inline constexpr double kDefaultCreaseAngleDeg       = 60.0;      // Dihedral angle (deg) protecting feature edges during PMP cleanup
inline constexpr double kDefaultPreviewOffsetFactor  = 1.0e-4;    // Patch normal offset factor along bbox diagonal (prevents Z-fighting)
inline constexpr int    kDefaultPreviewRings         = 2;         // Adjacent topological vertex rings extracted for preview
inline constexpr int    kDefaultMaxBatchIterations   = 10;        // Max iterative flip/collapse sweeps before PMP cleanup

// -----------------------------------------------------------------------------
// Mesh Health Checks & Large Defect Counts
// -----------------------------------------------------------------------------
inline constexpr double kDefaultDegenerateAreaEpsilon = 1.0e-14; // Face area <= eps * bbox_diag^2 counts as degenerate (relative, unitless)
inline constexpr std::size_t kDefaultListCap      = 100;      // Above this many (filtered) defects the Tool panel shows a summary instead of a list
inline constexpr std::size_t kDefaultOverlayCap   = 5000;     // Above this many highlighted faces only face tint is drawn (no edge tubes / vertex spheres)
inline constexpr std::size_t kHistogramBins       = 24;       // Number of bins in the Tool panel metric histogram
inline constexpr bool   kDefaultCheckMeshAfterLoad = true;    // Run "Check Mesh" automatically after a file or sample is loaded
inline constexpr std::array<double, 3> kDefaultCategoryTintColor = {0.95, 0.58, 0.16}; // RGB tint of every defect of the selected Mesh Health row
inline constexpr std::array<double, 3> kDefaultCleanFaceColor    = {0.70, 0.75, 0.80}; // RGB of base-mesh faces that are not highlighted
inline constexpr std::array<double, 3> kDefaultBaseEdgeColor     = {0.18, 0.20, 0.24}; // RGB of the base-mesh wireframe
inline constexpr std::array<double, 3> kDefaultBackFaceColor     = {0.85, 0.30, 0.55}; // RGB of back faces in the "Custom" back-face mode
inline constexpr double kDefaultGhostOpacity     = 0.25;          // Base-mesh opacity used when Ghost (G) is on
inline constexpr double kMinGhostOpacity         = 0.05;          // Minimum base-mesh opacity

// -----------------------------------------------------------------------------
// GUI Layout (docked panels)
// -----------------------------------------------------------------------------
inline constexpr float kPanelMarginPx          = 6.0f;            // Gap between docked panels and window edges (px)
inline constexpr float kLeftColumnWidthPx      = 290.0f;          // Width of the left column (Display / Navigate / Structures)
inline constexpr float kRightColumnDefaultPx   = 420.0f;          // Default width of the right column (Mesh Health / Tool)
inline constexpr float kRightColumnMinPx       = 340.0f;          // Minimum draggable right-column width
inline constexpr float kRightColumnMaxPx       = 900.0f;          // Maximum draggable right-column width
inline constexpr float kMinViewportWidthPx     = 400.0f;          // Below this 3D-view width the left column auto-collapses
inline constexpr double kStatusMessageSeconds  = 8.0;             // Seconds a status-bar message stays before reverting to "Ready"
inline constexpr float kMinUiScale             = 0.6f;            // Minimum Preferences > Interface UI scale
inline constexpr float kMaxUiScale             = 2.5f;            // Maximum Preferences > Interface UI scale

// -----------------------------------------------------------------------------
// Mesh Modification Undo History
// -----------------------------------------------------------------------------
inline constexpr std::size_t kMaxUndoHistoryDepth    = 50;        // Max undo snapshots kept (oldest full-mesh copies are dropped first)

// -----------------------------------------------------------------------------
// Vertex Welding Tolerance Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultWeldAbsFloorCm = 1.0e-9;          // Absolute tolerance floor in cm (~1 hydrogen atom)
inline constexpr double kDefaultWeldRelTol     = 1.0e-9;          // Coordinate-relative vertex welding tolerance

} // namespace ogun::settings
