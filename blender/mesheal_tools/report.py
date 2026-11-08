# SPDX-License-Identifier: GPL-3.0-or-later
"""Check results, kept in memory (not saved in the .blend)."""

from typing import NamedTuple, Sequence


class Item(NamedTuple):
    name: str
    count: int
    icon: str
    info_only: bool = False
    indices: Sequence[int] = ()  # polygon indices for "select"


# native Category name -> (label, icon), in panel order
CATEGORIES = {
    "SHELLS": ("Shells", "MESH_ICOSPHERE"),
    "OPEN_EDGES": ("Holes", "MOD_SOLIDIFY"),
    "NON_MANIFOLD_EDGES": ("Non-manifold Edges", "EDGESEL"),
    "NON_MANIFOLD_VERTICES": ("Non-manifold Vertices", "VERTEXSEL"),
    "FLIPPED_NORMALS": ("Flipped Normals", "NORMALS_FACE"),
    "INVERTED_SHELLS": ("Inverted Shells", "ORIENTATION_NORMAL"),
    "INTERSECTIONS": ("Self-intersections", "SELECT_INTERSECT"),
    "THIN_FACES": ("Thin Faces", "MOD_SIMPLIFY"),
    "DEGENERATE": ("Degenerate Faces", "FACESEL"),
    "DUPLICATES": ("Duplicate Faces", "DUPLICATE"),
}

_data: list[Item] = []
_source = {"object": None, "polygons": -1}


def update(items, obj=None, polygons=-1):
    _data[:] = items
    _source["object"] = obj.name if obj is not None else None
    _source["polygons"] = polygons


def get():
    return tuple(_data)


def clear():
    _data.clear()
    _source["object"] = None
    _source["polygons"] = -1


def stale(obj, polygons):
    """True if the results were not made from `obj` with `polygons` faces."""
    return obj is None or obj.name != _source["object"] or polygons != _source["polygons"]
