#pragma once

#include <cstddef>

namespace ogun::settings {

// -----------------------------------------------------------------------------
// 3D Viewport & Defect Highlight Overlay Sizing Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultHighlightPointRadius = 0.0015;    // Base radius of highlighted defect point spheres (world units)
inline constexpr double kDefaultHighlightEdgeRadius  = 0.0008;    // Base radius of highlighted defect wireframe curves (world units)
inline constexpr double kMinHighlightRelativeRadius  = 1.0e-6;    // Minimum allowable relative radius for highlight elements
inline constexpr double kMaxHighlightRelativeRadius  = 0.1;       // Maximum allowable relative radius for highlight elements
inline constexpr double kMinHighlightZoomScale       = 1.0e-5;    // Minimum dynamic scaling factor when camera zooms in close
inline constexpr double kReferenceFullMeshDistanceFactor = 1.5;   // Multiplier on mesh bounding-box diagonal for nominal camera distance

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
// Vertex Welding Tolerance Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultWeldAbsFloorCm = 1.0e-9;          // Absolute tolerance floor in cm (~1 hydrogen atom)
inline constexpr double kDefaultWeldRelTol     = 1.0e-9;          // Coordinate-relative vertex welding tolerance

} // namespace ogun::settings
