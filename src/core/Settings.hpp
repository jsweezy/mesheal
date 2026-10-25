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
// Mesh Modification Undo History
// -----------------------------------------------------------------------------
inline constexpr std::size_t kMaxUndoHistoryDepth    = 50;        // Max undo snapshots kept (oldest full-mesh copies are dropped first)

// -----------------------------------------------------------------------------
// Vertex Welding Tolerance Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultWeldAbsFloorCm = 1.0e-9;          // Absolute tolerance floor in cm (~1 hydrogen atom)
inline constexpr double kDefaultWeldRelTol     = 1.0e-9;          // Coordinate-relative vertex welding tolerance

} // namespace ogun::settings
