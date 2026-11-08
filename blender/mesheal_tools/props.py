# SPDX-License-Identifier: GPL-3.0-or-later
"""Scene settings. Defaults and ranges mirror mesheal's src/core/Settings.hpp."""

from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import PropertyGroup


class MeshealSceneProps(PropertyGroup):
    # Scope ------------------------------------------------------------------
    scope: EnumProperty(
        name="Scope",
        description="Which part of the mesh the repair works on",
        items=(
            ("WHOLE", "Whole Mesh", "Repair the whole mesh"),
            ("SELECTED", "Selected Faces", "Repair only the selected faces (Edit Mode selection)"),
        ),
        default="WHOLE",
    )

    # Check ------------------------------------------------------------------
    intersection_tolerance: FloatProperty(
        name="Intersection Tolerance",
        description="Self-intersection tolerance as a fraction of the bounding-box diagonal",
        default=1.0e-6,
        min=0.0,
        soft_max=1.0e-3,
        precision=8,
    )
    thin_max_aspect: FloatProperty(
        name="Max Aspect Ratio",
        description="Faces with a longer-edge to height ratio above this are reported as thin",
        default=100.0,
        min=1.0,
        soft_max=10000.0,
    )

    # Fix All ----------------------------------------------------------------
    fixall_passes: IntProperty(
        name="Passes",
        description="Most repair passes Fix All runs before keeping the best one",
        default=3,
        min=1,
        max=10,
    )
    fixall_time_budget: FloatProperty(
        name="Time Budget",
        description="Stop starting new passes after this many seconds (0 = no limit)",
        default=0.0,
        min=0.0,
        soft_max=600.0,
        subtype="TIME_ABSOLUTE",
        unit="TIME_ABSOLUTE",
    )
    weld: BoolProperty(
        name="Weld Vertices",
        description="Weld vertices that sit within the tolerance of each other",
        default=True,
    )
    weld_tolerance: FloatProperty(
        name="Weld Tolerance",
        description="Per-axis weld tolerance, relative to the larger coordinate",
        default=1.0e-6,
        min=1.0e-9,
        max=1.0e-3,
        precision=9,
    )
    remove_tiny_shells: BoolProperty(
        name="Delete Tiny Shells",
        description="Delete small floating shells",
        default=True,
    )
    tiny_shell_fraction: FloatProperty(
        name="Tiny Shell Size",
        description="Shells smaller than this fraction of the bounding-box diagonal are deleted",
        default=1.0e-3,
        min=0.0,
        max=0.1,
        precision=5,
    )
    remove_fins: BoolProperty(
        name="Remove Fins & Internal Walls",
        description="Remove fin faces and internal walls",
        default=True,
    )
    close_cracks: BoolProperty(
        name="Close Cracks",
        description="Close narrow cracks between open edges",
        default=True,
    )
    heal_thin_faces: BoolProperty(
        name="Heal Thin Faces",
        description="Flip, collapse or remove needle and cap faces",
        default=True,
    )
    merge_shells: BoolProperty(
        name="Merge Overlapping Shells",
        description="Boolean-union shells that overlap each other (cavities are kept)",
        default=False,
    )

    # Remesh -----------------------------------------------------------------
    remesh_method: EnumProperty(
        name="Method",
        description="How to rebuild the surface",
        items=(
            ("VOXEL", "Voxel Heal & Remesh", "Fast voxel rebuild into a new watertight surface"),
            ("ISO", "Precision Isosurface", "Slower, higher-quality isosurface rebuild (FEA grade)"),
            ("ALPHA", "Shrink-Wrap", "Wrap an outer envelope around the mesh"),
        ),
        default="VOXEL",
    )
    remesh_target_size: FloatProperty(
        name="Target Edge Length",
        description="Edge length of the new surface (0 = auto, longest side / 128)",
        default=0.0,
        min=0.0,
        subtype="DISTANCE",
        unit="LENGTH",
    )
    remesh_smoothing: IntProperty(
        name="Smoothing Passes",
        description="Taubin smoothing passes on the rebuilt surface",
        default=3,
        min=0,
        max=10,
    )
    iso_smoothing: IntProperty(
        name="Smoothing Passes",
        description="Tangential relaxation passes on the isosurface",
        default=1,
        min=0,
        max=10,
    )
    iso_chord_ratio: FloatProperty(
        name="Curve Fidelity",
        description="Chord tolerance as a fraction of the target edge length (smaller follows curves closer)",
        default=0.10,
        min=0.05,
        max=1.0,
    )
    alpha_detail: FloatProperty(
        name="Detail",
        description="Bounding-box diagonal divided by the wrap size (higher follows the shape closer)",
        default=100.0,
        min=10.0,
        max=1000.0,
    )
    alpha_offset_ratio: FloatProperty(
        name="Offset",
        description="Wrap offset as a fraction of the wrap size (alpha / x)",
        default=30.0,
        min=1.0,
        max=200.0,
    )
    simplify_rebuild: BoolProperty(
        name="Simplify to Original Face Count",
        description="Reduce the rebuilt surface back to the original face count",
        default=False,
    )

    # Decimate ---------------------------------------------------------------
    decimate_ratio: FloatProperty(
        name="Ratio",
        description="Fraction of faces kept",
        default=0.5,
        min=0.01,
        max=1.0,
        subtype="FACTOR",
    )

    # Merge ------------------------------------------------------------------
    merge_distance: FloatProperty(
        name="Distance",
        description="Merge vertices closer than this",
        default=0.05,
        min=1.0e-4,
        max=10.0,
        precision=4,
        subtype="DISTANCE",
        unit="LENGTH",
    )
    merge_keep_surfaces_apart: BoolProperty(
        name="Keep Surfaces Apart",
        description="Do not merge vertices of surfaces that face each other across a thin gap",
        default=True,
    )
    merge_preserve_open_edges: BoolProperty(
        name="Preserve Open Edges",
        description="Do not merge across open edges and cracks",
        default=True,
    )
