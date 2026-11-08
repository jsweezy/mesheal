# SPDX-License-Identifier: GPL-3.0-or-later
"""Check results, kept in memory (not saved in the .blend)."""

from typing import NamedTuple, Sequence


class Item(NamedTuple):
    name: str
    count: int
    icon: str
    info_only: bool = False
    indices: Sequence[int] = ()  # face indices for "select"


_data: list[Item] = []


def update(items):
    _data[:] = items


def get():
    return tuple(_data)


def clear():
    _data.clear()


def placeholder_results():
    """Fake results so the panel layout can be designed before mesheal is hooked up."""
    return (
        Item("Shells", 3, "MESH_ICOSPHERE", info_only=True),
        Item("Holes", 2, "MOD_SOLIDIFY", indices=(0, 1)),
        Item("Non-manifold Edges", 4, "EDGESEL", indices=(2,)),
        Item("Non-manifold Vertices", 0, "VERTEXSEL"),
        Item("Flipped Normals", 12, "NORMALS_FACE", indices=(3, 4)),
        Item("Inverted Shells", 0, "ORIENTATION_NORMAL"),
        Item("Self-intersections", 37, "SELECT_INTERSECT", indices=(5,)),
        Item("Thin Faces", 8, "MOD_SIMPLIFY", indices=(0,)),
        Item("Degenerate Faces", 0, "FACESEL"),
        Item("Duplicate Faces", 1, "DUPLICATE", indices=(1,)),
    )
