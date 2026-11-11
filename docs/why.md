# Why Mesheal

## The problem

Monte Carlo particle transport codes (MCNP, OpenMC, DAGMC, Serpent, Geant4 and others) and many
other simulation tools trace paths through surface meshes. A ray that crosses a surface must clearly
enter or leave a volume. That only works when the mesh is:

- **Watertight and 2-manifold:** every edge belongs to exactly two triangles, with no holes, fins or
  pinch points.
- **Consistently oriented:** every triangle faces outward from the solid it bounds, and cavities face
  inward.
- **Free of degenerate and sliver triangles:** near-zero-area or extremely stretched triangles cause
  precision failures, lost particles and endless surface-crossing loops.
- **Free of self-intersections:** overlapping faces make it unclear which volume a point is in. How
  much overlap a code can tolerate varies, so Mesheal reports it and leaves the decision to you.
- **In known units:** STL files carry no units, but simulation codes expect a fixed unit system.

Existing tools leave a gap. 3D-printing repair tools aim at slicer tolerance and decide for you: they
remesh, voxelize or patch silently, without showing how much the shape changed. General 3D suites
are powerful but bury mesh diagnostics across several modes and have confusing unit scaling for CAD
and simulation work.

## Options plus information; you decide

Mesheal does not assume what you need. A coarse global fix (rebuild or wrap the whole surface) is
the right answer for some models. Others need a careful fix of individual faces. For each kind of
defect Mesheal offers both. It previews every option before applying it and shows what changes:

- volume and surface area before and after,
- how far the surface moved,
- whether the result is watertight and manifold,
- which defects remain.

Mesheal never rules out a method on your behalf, and every repair is one undo step.

## Units without accidental scaling

Choosing the **File Units** (`mm`, `cm`, `m`, `in`, `ft`) only labels the numbers in the file, so
sizes, areas and tolerances are shown in the right unit. It never moves a vertex. Rescaling the
geometry is a separate, explicit action (**Mesh > Units & Scale**). See
[Units & Preferences](gui/units-and-preferences.md).

## Multi-scale welding

An STL file is a "triangle soup": every triangle stores its own three corners. Mesheal joins equal
corners into shared vertices when it loads the file. Models can mix sub-millimetre detail with
kilometre-scale structures, so the weld tolerance is relative to each coordinate's size, with a tiny
absolute floor:

$$
|p_i - q_i| \le \max\left(\epsilon_{\text{abs}},\; \epsilon_{\text{rel}} \cdot \max(|p_i|, |q_i|)\right) \quad \text{for } i \in \{x, y, z\}
$$

Both default to $10^{-9}$ and can be changed in **Preferences > Units & Tolerance**. The floor
$\epsilon_{\text{abs}}$ is in file coordinates: changing the file units does not change it. The weld never joins two corners of the same triangle, so sliver and
zero-area triangles survive loading and can be inspected. Solids that only touch along an edge or at
a corner are kept apart, so they do not show up as false non-manifold defects.

## Design principles

- **The geometry core has no GUI.** File reading and writing, welding, checks and repairs are a plain
  C++ library. The desktop app, the command line and the Python module all call the same code.
- **Tested.** Every module has Catch2 unit tests, including the application logic behind the GUI.
- **Built on proven libraries:** [CGAL](https://www.cgal.org/) for exact predicates and repair,
  [Manifold](https://github.com/elalish/manifold) for robust booleans,
  [libigl](https://libigl.github.io/) and [Eigen](https://eigen.tuxfamily.org/) for mesh processing,
  and [Polyscope](https://polyscope.run/) with [Dear ImGui](https://github.com/ocornut/imgui) for the
  3D view.
- **GPL-3.0-or-later**, because Mesheal uses CGAL's GPL-licensed *Polygon Mesh Processing* package.
