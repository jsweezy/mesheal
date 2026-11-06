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
inline constexpr float  kDefaultMouseZoomSpeed   = 1.0f;          // Preferences > Mouse zoom speed: multiplier of wheel and drag zoom steps (1 = unchanged)
inline constexpr float  kMinMouseZoomSpeed       = 0.1f;          // Slowest zoom speed multiplier
inline constexpr float  kMaxMouseZoomSpeed       = 5.0f;          // Fastest zoom speed multiplier

// -----------------------------------------------------------------------------
// Thin Face Diagnostics Threshold Defaults (file units; the unit is only a label)
// -----------------------------------------------------------------------------
inline constexpr double kDefaultThinFaceMaxAspectRatio  = 100.0;  // A face stretched more than this (l_max / h_min, unitless) is thin
inline constexpr double kDefaultThinFaceMinAltitude     = 0.0;    // Advanced absolute floor: smallest height h_min = 2A / l_max (file units; 0 = off)
inline constexpr double kDefaultThinFaceMinArea         = 0.0;    // Advanced absolute floor: smallest area A (file units^2; 0 = off)
inline constexpr double kNormalFloorCoordinateFactor    = 0x1p-40; // No reliable normal when h_min <= this x the largest |coordinate| of the face (rounding noise)
inline constexpr double kLegacyThinFaceMinAltitude      = 1.0e-4; // Old default height floor: saved preferences equal to it load as 0 (off)
inline constexpr double kLegacyThinFaceMinArea          = 1.0e-8; // Old default area floor: saved preferences equal to it load as 0 (off)
inline constexpr double kLegacyThinFaceMaxAspectRatio   = 1000.0; // Old default aspect limit: saved preferences equal to it load as the new default

// -----------------------------------------------------------------------------
// Sliver Healer Classification & Repair Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultNeedleEdgeRatio      = 0.05;      // l_min / l_max below this ratio classifies face as Needle
inline constexpr double kDefaultCapAngleDeg          = 170.0;     // Maximum interior angle (deg) above which face is classified as Cap
inline constexpr double kDefaultCoplanarDot          = 0.95;      // Minimum normal alignment dot product for coplanar flips
inline constexpr double kDefaultMinNormalDot         = 0.9;       // Minimum n_old . n_new around collapsed edges (about 25 degrees of tilt): 0.1 let a corner snap tilt a wall by 72 degrees and cost 9 % of a CAD part's volume
inline constexpr double kDefaultCreaseAngleDeg       = 60.0;      // Dihedral angle (deg) protecting feature edges during PMP cleanup
inline constexpr double kDefaultPreviewOffsetFactor  = 1.0e-4;    // Patch normal offset factor along bbox diagonal (prevents Z-fighting)
inline constexpr int    kDefaultPreviewRings         = 2;         // Adjacent topological vertex rings extracted for preview
inline constexpr int    kDefaultMaxBatchIterations   = 10;        // Max iterative flip/collapse sweeps before PMP cleanup
inline constexpr double kDefaultClusterMinNormalDot  = 0.5;       // Micro-cluster collapse: minimum n_old . n_new over significant surviving faces
inline constexpr double kDefaultTinyFaceAreaFraction = 1.0e-3;    // Micro-cluster collapse: faces below this fraction of the largest neighbor area ignore the normal test (noise)
inline constexpr int    kDefaultMaxMicroClusterSize  = 64;        // Micro-cluster collapse: maximum vertices gathered around a needle's short edge
inline constexpr double kDefaultFlipFlatDot          = 0.9999;    // Delaunay flip pass: two faces count as one flat patch when n_a . n_b >= this (about 0.8 degrees); creases never flip
inline constexpr double kDefaultHealMaxVolumeChange  = 0.005;     // Safety net: a heal that changes a closed mesh's volume by more than this share (0.5 %) is rejected
inline constexpr double kDefaultHealMaxDeviation     = 0.01;      // Safety net: a heal that moves the surface by more than this share (1 %) of the bounding-box diagonal is rejected

// -----------------------------------------------------------------------------
// Intersecting-Face Repair (per island and Repair All)
// -----------------------------------------------------------------------------
inline constexpr int    kDefaultIntersectionDilationRings = 1;    // Face rings cut away around an island before its hole is filled
inline constexpr int    kMaxIntersectionDilationRings     = 4;    // Upper bound of the dilation-rings setting
inline constexpr int    kDefaultCgalRepairMaxSteps        = 7;    // Region-growing steps of CGAL's local self-intersection repair (CGAL default)
inline constexpr double kDefaultSmoothCollarMinAngleDeg   = 3.0;  // Mean collar dihedral (deg) above which an island sits on a curved surface (Fill Smooth)
inline constexpr int    kMaxIslandRepairRounds            = 4;    // Repair All "Patch each island": rounds of recommended method + fallbacks
inline constexpr double kDefaultAlphaWrapDetail           = 100.0;  // Alpha wrap: alpha = bbox diagonal / detail (larger = tighter, slower)
inline constexpr double kMinAlphaWrapDetail               = 10.0;   // Lowest alpha-wrap detail offered
inline constexpr double kMaxAlphaWrapDetail               = 1000.0; // Highest alpha-wrap detail offered (very slow on big meshes)
inline constexpr double kDefaultAlphaWrapOffsetRatio      = 30.0;   // Alpha wrap: offset = alpha / ratio (CGAL's suggestion)
inline constexpr double kMinAlphaWrapOffsetRatio          = 1.0;    // Lowest offset ratio offered (offset = alpha: loosest wrap)
inline constexpr double kMaxAlphaWrapOffsetRatio          = 200.0;  // Highest offset ratio offered (offset = alpha / 200: tightest, slowest)
inline constexpr double kDefaultDecimateRatio             = 0.5;    // Mesh > Simplify > Decimate: keep this fraction of the faces
inline constexpr double kMinDecimateRatio                 = 0.01;   // Smallest decimation ratio offered
inline constexpr double kMaxDecimateRatio                 = 1.0;    // Largest decimation ratio offered (no change)
inline constexpr double kDefaultMergeDistance           = 0.05;   // Mesh > Merge by Distance: merge distance in file units (unitless)
inline constexpr double kMinMergeDistance               = 1.0e-4; // Merge distance slider minimum (log scale), file units
inline constexpr double kMaxMergeDistance               = 10.0;   // Merge distance slider maximum, file units
inline constexpr double kMergeCollapseWarnRatio           = 5.0;    // Merge distance / mean edge above which "whole areas will collapse"
inline constexpr bool   kDefaultMergeKeepSurfacesApart    = true;   // Merge vertices only along an edge (not pinching touching surfaces)
inline constexpr bool   kDefaultMergePreserveOpenEdges    = true;   // Merge by Distance: preserve open edges and cracks by default
inline constexpr double kDefaultWeldDistance            = 0.05;   // Mesh > Weld: weld distance in file units (unitless)
inline constexpr double kMinWeldDistance                = 1.0e-4; // Weld distance slider minimum (log scale), file units
inline constexpr double kMaxWeldDistance                = 10.0;   // Weld distance slider maximum, file units
inline constexpr int    kDefaultWindingGridResolution     = 128;    // Winding-number rebuild: grid cells along the longest bbox side
inline constexpr int    kMinWindingGridResolution         = 16;     // Lowest winding-number grid resolution offered
inline constexpr int    kMaxWindingGridResolution         = 512;    // Highest winding-number grid resolution offered
inline constexpr int    kDefaultRemeshGridCells           = 128;    // Voxel / Isosurface remesh: default target size = longest bbox span / this cell count
inline constexpr int    kMinRemeshGridCells               = 16;     // Coarsest slider bound for target mesh size (longest bbox span / 16)
inline constexpr int    kMaxRemeshGridCells               = 256;    // Finest slider bound for target mesh size (longest bbox span / 256)
inline constexpr double kMinRemeshTargetSize              = 1.0e-5; // Minimum allowed target mesh size in file coordinates (unitless)
inline constexpr double kMaxRemeshTargetSize              = 1.0e5;  // Maximum allowed target mesh size in file coordinates (unitless)
inline constexpr int    kDefaultVoxelSmoothingPasses      = 3;      // Voxel Heal & Remesh (Fast): default Taubin + isotropic smoothing passes
inline constexpr int    kDefaultIsosurfaceSmoothingPasses = 1;      // Precision Isosurface (FEA Grade): default tangential relaxation passes
inline constexpr int    kMinRemeshSmoothingPasses         = 0;      // Minimum smoothing passes for Voxel Heal & Remesh and Precision Isosurface
inline constexpr int    kMaxRemeshSmoothingPasses         = 10;     // Maximum smoothing passes for Voxel Heal & Remesh and Precision Isosurface
inline constexpr double kDefaultTaubinLambda              = 0.50;   // Taubin smoothing forward Laplacian step factor (removes Marching Cubes terracing)
inline constexpr double kDefaultTaubinMu                  = -0.53;  // Taubin smoothing backward inflation step factor (prevents volume shrinkage)
inline constexpr double kDefaultIsosurfaceChordRatio      = 0.10;   // Precision Isosurface: default chord tolerance as a fraction of target mesh size
inline constexpr double kMinIsosurfaceChordRatio          = 0.05;   // Precision Isosurface: tightest chord tolerance ratio (finest curve tessellation)
inline constexpr double kMaxIsosurfaceChordRatio          = 1.00;   // Precision Isosurface: loosest chord tolerance ratio (1.0 = uniform sizing everywhere)
inline constexpr double kDefaultIsosurfaceMinFacetAngleDeg = 30.0;  // Precision Isosurface: minimum triangle angle (deg) in CGAL Delaunay refinement
inline constexpr double kDefaultIsosurfaceRelativeErrorBound = 1.0e-4; // Precision Isosurface: relative bisection root-finding error bound on W = 0.5

// -----------------------------------------------------------------------------
// Holes (open edges): shape classification and filling
// -----------------------------------------------------------------------------
inline constexpr double kDefaultHoleFlatTolerance     = 1.0e-3; // Flat hole: rim within this fraction of the hole size of its best-fit plane
inline constexpr double kDefaultHoleFlatNeighbourDeg  = 5.0;    // Flat hole: faces on the rim within this angle of the plane (coplanar) or of its normal (a cap)
inline constexpr double kDefaultHoleFlatNeighbourShare = 0.9;   // Flat hole: share of rim faces that must pass the angle test above
inline constexpr double kDefaultHoleCrackWidthRatio   = 0.1;    // Crack: mean gap width (2 x area / perimeter) below this fraction of the mean rim edge
inline constexpr int    kDefaultHolePatchRings        = 3;      // Face rings copied around a hole for its fill (CGAL looks at the rim faces, fairing at 2 rings)
inline constexpr int    kMaxHolePatchRings            = 8;      // Upper bound of the patch-rings setting
inline constexpr int    kDefaultMaxHoleEdges          = 0;      // Fill each hole / Fix Selected skip holes with more rim edges (0 = no limit)
inline constexpr std::size_t kDefaultMaxHolePatchFaces = 50000; // Refine / Smooth refuse a hole whose refined patch has more faces (fairing gets slow)
inline constexpr double kDefaultCrackSnapFactor       = 3.0;    // Close Crack: vertices move at most this many mean gap widths (a wedge is 2x its mean)
inline constexpr double kMinCrackSnapEdgeShare        = 1.0e-3; // Close Crack: ... but at least this share of the mean rim edge (zero-width T-junctions)
inline constexpr std::size_t kMaxCubicHoleFillEdges   = 400;    // Fill Flat / Refine / Smooth: bigger holes skip CGAL's O(n^3) fallback search (n = 600 takes ~1 s)
inline constexpr int    kHoleCrossingGridMaxCellsPerAxis = 128;  // Batch fill crossing test: at most this many grid cells per axis for the faces near a patch
inline constexpr int    kHoleCrossingGridMaxCellsPerFace = 64;   // Batch fill crossing test: a face over more cells than this is tested against every patch instead

// -----------------------------------------------------------------------------
// Mesh Health Checks & Large Defect Counts
// -----------------------------------------------------------------------------
inline constexpr double kDefaultIntersectionTolerance = 1.0e-6;  // Intersection tolerance as a fraction of the bbox diagonal: box growth, on-plane snap, min crossing length
inline constexpr unsigned kIntersectionMaxThreads     = 4;       // Max worker threads of the self-intersection BVH traversal (Blender uses 4)
inline constexpr std::size_t kIntersectionSerialFaceCount = 1024; // Meshes with at most this many faces check intersections on one thread
inline constexpr double kRuleOOutsideShare = 0.5;            // Rule O: an inward shell that crosses others is inside out when more than this share of its vertices lies outside every other closed shell
inline constexpr std::size_t kRuleOMaxVertexSamples = 1001; // Rule O: at most this many vertices (evenly spaced) are ray-tested for that share
inline constexpr double kDefaultFixAllTinyShellFraction = 1.0e-3; // Tiny floating shell: bounding-box diagonal below this fraction (0.1 %) of the whole model's, touching nothing
inline constexpr double kFinMaxAreaShare = 0.5;          // Fin / wall removal: a sheet goes only when its area is at most this share of each sheet kept at its Fin / other edge
inline constexpr double kFinSeamCreaseAngleDeg = 30.0;   // Fin / wall removal: an edge between two Fin / other edge ends whose faces bend more than this (degrees) separates sheets
inline constexpr int    kFinRemovalMaxRounds = 4;        // Fin / wall removal: rounds of detect-and-remove (each removal can settle another edge)
inline constexpr std::size_t kDefaultListCap      = 100;      // Above this many (filtered) defects the Tool panel shows a summary instead of a list
inline constexpr std::size_t kDefectListMaxRows   = 12;       // Tool panel list height in rows; shorter lists shrink to fit, so the Inspector stays on screen
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
inline constexpr float kPreferencesWindowWidthPx  = 620.0f;          // Preferences default size (clamped to the free 3D view)
inline constexpr float kPreferencesWindowHeightPx = 560.0f;
inline constexpr float kRightColumnDefaultPx   = 420.0f;          // Default width of the right column (Mesh Health / Tool)
inline constexpr float kRightColumnMinPx       = 340.0f;          // Minimum draggable right-column width
inline constexpr float kRightColumnMaxPx       = 900.0f;          // Maximum draggable right-column width
inline constexpr float kMinViewportWidthPx     = 400.0f;          // Below this 3D-view width the left column auto-collapses
inline constexpr float kFixAllBodyMinShare     = 0.4f;            // Fix All panel: its scrolling body keeps at least this share of the panel height (the panel scrolls to the buttons below)
inline constexpr float kFixAllBodyMinLines     = 8.0f;            // Fix All panel: ... and at least this many text lines
inline constexpr double kStatusMessageSeconds  = 8.0;             // Seconds a status-bar message stays before reverting to "Ready"
inline constexpr double kVolumeChangeWarnPercent = 1.0;          // Volume change (%) shown as a warning in previews, Mesh Info and the status bar
inline constexpr double kIdleFrameIntervalSeconds = 0.5;          // Longest wait between frames while idle (no input, job or animation)
inline constexpr double kIdleAfterActivitySeconds = 0.75;         // Full frame rate continues this long after the last input, so hovers and tooltips settle
inline constexpr double kFrameBudgetShare      = 0.95;            // Share of 1 / maxFPS slept between active frames (Polyscope's own cap uses 95 %)
inline constexpr float kPolyscopeWindowWidthPx = 330.0f;          // Preferred width of each View > Polyscope window (tiled in the free 3D view)
inline constexpr float kPolyscopeWindowMinWidthPx = 240.0f;       // Narrowest tile before the View > Polyscope windows wrap to another row
inline constexpr std::array<float, 4> kPolyscopeWindowHeightsPx = {230.0f, 420.0f, 520.0f, 200.0f}; // Preferred content height of the Slice Planes, Appearance, Camera & Clipping and Render & Debug windows (at UI scale 1)
inline constexpr float kUiWheelScrollScale     = 0.25f;           // Mouse-wheel / trackpad scroll over ImGui windows (ImGui scrolls 5 lines per tick: this makes about 1)
inline constexpr float kMinUiScale             = 0.6f;            // Minimum Preferences > Interface UI scale
inline constexpr float kMaxUiScale             = 2.5f;            // Maximum Preferences > Interface UI scale
inline constexpr std::array<float, 4> kDefaultBackgroundColor = {1.0f, 1.0f, 1.0f, 0.0f}; // RGBA 3D-view background (Polyscope default; Display > Appearance)

// -----------------------------------------------------------------------------
// Shells panel (UI_DESIGN.md §16)
// -----------------------------------------------------------------------------
inline constexpr bool   kDefaultColorByShell     = true;          // Shells panel "Color by shell" starts on for multi-shell meshes
inline constexpr std::array<double, 3> kDefaultShellSelectedColor = {0.40, 0.70, 1.00}; // RGB tint of selected shells (not a defect color)
inline constexpr std::array<double, 3> kDefaultFaceSelectionColor = {1.00, 0.30, 0.85}; // RGB of faces selected with B / C / Shift+click (magenta: outside the shell hues and defect tints)
inline constexpr float kDefaultCircleSelectRadiusPx = 30.0f;  // Circle select (C) brush radius in screen points
inline constexpr float kMinCircleSelectRadiusPx     = 4.0f;   // Smallest circle-select radius (mouse wheel)
inline constexpr float kMaxCircleSelectRadiusPx     = 300.0f; // Largest circle-select radius (mouse wheel)
inline constexpr double kFaceVisibilityWorkingSize   = 1000.0; // Bounding-box diagonal the visibility rays work at (libigl ray epsilon is absolute)
inline constexpr double kFocusSelectionMinNormalAlignment = 0.3; // "." looks square on at selected faces whose |sum of area normals| / area >= this
inline constexpr double kShellSelectedTintWeight = 0.65;          // Blend weight of the selected-shell tint over the shell color
inline constexpr double kShellHueMin             = 0.28;          // Shell colors use hues [min, min + span]: green..purple, no red/orange/yellow
inline constexpr double kShellHueSpan            = 0.55;          // Width of the shell hue band (HSV hue in [0, 1])
inline constexpr double kShellColorSaturation    = 0.38;          // Shell colors are muted so defect tints stay readable
inline constexpr std::array<double, 2> kShellColorValues = {0.82, 0.70}; // HSV value of even / odd shells
inline constexpr double kShellMatchBoxTolerance  = 1.0e-9;        // Box tolerance (fraction of the mesh diagonal) for "same shell" after an edit
inline constexpr double kShellMatchMinOverlap    = 0.5;           // Min box overlap (IoU) for an edited shell to keep its view state
inline constexpr int    kDefaultSmallShellFaces  = 100;           // Shells panel "Select > Smaller than N faces" default
inline constexpr float  kShellsPanelMinHeightPx  = 160.0f;        // Shells panel never gets shorter than this
inline constexpr float  kStructuresShareOfShellsColumn = 0.4f;    // Expanded Structures takes this share of the column below Navigate

// -----------------------------------------------------------------------------
// Mesh Modification Undo History
// -----------------------------------------------------------------------------
inline constexpr std::size_t kMaxUndoHistoryDepth    = 50;        // Max undo snapshots kept (oldest full-mesh copies are dropped first)

// -----------------------------------------------------------------------------
// Vertex Welding Tolerance Defaults
// -----------------------------------------------------------------------------
inline constexpr double kDefaultWeldAbsFloor = 1.0e-9;            // Absolute tolerance floor in file coordinates (unitless: the file unit never rescales it)
inline constexpr double kDefaultWeldRelTol     = 1.0e-9;          // Coordinate-relative vertex welding tolerance

// Fix All's own weld by distance (crack healing), separate from the load weld above: two vertices match
// when on every axis |p_i - q_i| <= max(floor, rel x max(|p_i|, |q_i|)) (a ToleranceConfig of its own).
inline constexpr double kDefaultFixAllWeldAbsFloor = 1.0e-6;      // Fix All weld: absolute per-axis floor in file coordinates (unitless)
inline constexpr double kDefaultFixAllWeldRelTol   = 1.0e-6;      // Fix All weld: per-axis tolerance relative to the larger coordinate on that axis
inline constexpr double kFixAllWeldToleranceMin    = 1.0e-9;      // Fix All weld: smallest floor or relative tolerance Preferences accepts (rel = 0 is refused)
inline constexpr double kFixAllWeldToleranceMax    = 1.0e-3;      // Fix All weld: largest floor or relative tolerance Preferences accepts
inline constexpr double kFixAllWeldNoteDiagonalShare = 1.0e-4;    // Fix All weld: the plan warns when the largest per-axis tolerance exceeds this share (0.01 %) of the scope's bbox diagonal

// -----------------------------------------------------------------------------
// Fix All (one-click repair in passes: repair each shell, merge, clean up)
// -----------------------------------------------------------------------------
inline constexpr int    kDefaultFixAllPasses = 3;                 // Passes of a full run (1 = One pass); stops earlier when clean or stuck
inline constexpr int    kFixAllMinPasses     = 1;                 // Fewest passes a run may be asked for
inline constexpr int    kFixAllMaxPasses     = 10;                // Most passes a run may be asked for
inline constexpr double kFixAllTinyShellFractionMax = 0.1;        // Largest tiny-shell fraction Preferences accepts (10 % of the bounding-box diagonal)
inline constexpr double kDefaultFixAllTimeBudgetSeconds = 0.0;    // Stop after the pass that ends past this many seconds (0 = no budget)
inline constexpr bool   kDefaultFixAllWeld        = true;         // Weld by distance (its own per-axis tolerance above) runs first in each pass
inline constexpr bool   kDefaultFixAllTinyShells  = true;         // Remove tiny floating shells (below kDefaultFixAllTinyShellFraction, touching nothing)
inline constexpr bool   kDefaultFixAllFins        = true;         // Remove fins and internal walls
inline constexpr bool   kDefaultFixAllCloseCracks = true;         // Close cracks (Holes tool's crack settings)
inline constexpr bool   kDefaultFixAllHealThin    = true;         // Heal thin faces (Thin faces tool's aspect limit)
inline constexpr bool   kDefaultFixAllMergeShells = false;        // Merge shells (phase 2); the panel ticks it when shells in scope overlap
inline constexpr float  kFixAllBaselineProgressShare = 0.10f;     // Progress bar: share of the first check of the mesh
inline constexpr float  kFixAllPassesProgressShare   = 0.85f;     // Progress bar: share of all passes together (split equally between them)
inline constexpr double kFixAllQuickStageProgressWeight = 1.25;   // Progress weight within a pass of each quick phase-1 stage (weld, clean-ups, orient, cracks: 10 % over 8)
inline constexpr double kFixAllHealStageProgressWeight  = 11.25;  // Progress weight within a pass of each healing stage (slivers, holes, islands, orient: 45 % over 4)
inline constexpr double kFixAllMergeProgressWeight      = 20.0;   // Progress weight within a pass of phase 2 (merge), when it runs
inline constexpr double kFixAllCleanupProgressWeight    = 10.0;   // Progress weight within a pass of phase 3 (clean up the merge), when the merge is on
inline constexpr double kFixAllPassCheckProgressWeight  = 15.0;   // Progress weight within a pass of the check that judges it
inline constexpr double kFixAllCavityShrinkShare = 1.0e-3;         // Cavity guard: a step may not shrink a shell wound inward by more than this share of its volume
inline constexpr double kFixAllMergePlanProgressShare = 0.2;       // Share of phase 2's progress weight for planning the merge (the groups share the rest by face count)
inline constexpr double kFixAllMergeVolumeRelTolerance = 1.0e-6;   // Merge guard: volume sanity bounds (max V <= V_union <= sum V) hold within this share of sum V
inline constexpr std::size_t kFixAllReportShellsNamed = 8;       // Final report: shells named in the "Left after N passes" line before "and N more"

} // namespace ogun::settings
