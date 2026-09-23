"""Regression: ``edges`` reports each edge once, and never confuses located copies.

``edges`` used to be a bare ``TopExp_Explorer`` over ``TopAbs_EDGE``, which yields
one entry per (edge, incident face) pair -- a box came back with 24 entries for its
12 edges. Consumers that need a topology answer (an edge count, a wire frame, a
boundary export) therefore had to collapse the incidences themselves, and the
obvious key, the bare ``TShape`` pointer, is placement-blind: a prism's top rail IS
its base rail instanced at another ``Location``, so every located copy collapsed
into the original and silently disappeared (12 -> 8 for an extruded square,
24 -> 16 for an extruded face with a hole).

``TopExp::MapShapes`` keys on ``TopoDS_Shape::IsSame`` -- TShape AND Location,
orientation-insensitive -- so it answers both halves at the source, which is what
``vertex_points`` has always done. These cases pin the counts on both sides of that
distinction: shared edges collapse, located copies do not.
"""

import pytest

cad = pytest.importorskip("adacpp.cad")

# A unit square in the z=0 plane, and the same outline with a smaller square
# removed -- the outer wire plus one hole wire, 8 edges in the face.
_SQUARE = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]


def _holed_face():
    """A planar face with one interior hole: 2 wires, 8 edges."""
    face = cad.polygon_face(_SQUARE)
    cutter = cad.boolean(
        "DIFFERENCE",
        face,
        cad.extrude_face_along_normal(
            cad.polygon_face([[0.25, 0.25, 0.0], [0.75, 0.25, 0.0], [0.75, 0.75, 0.0], [0.25, 0.75, 0.0]]),
            -1.0,
        ),
    )
    holed = [f for f in cad.faces(cutter) if len(cad.wires(f)) == 2]
    assert len(holed) == 1, "expected the cut to leave exactly one face with a hole"
    return holed[0]


def test_box_reports_twelve_edges_not_twenty_four():
    """Each edge of a box is incident to two faces; it is still ONE edge."""
    box = cad.make_box(2.0, 3.0, 4.0)

    assert len(cad.edges(box)) == 12


def test_face_edges_are_not_collapsed_across_its_wires():
    """An outer wire and a hole wire share no edges, so all 8 survive."""
    face = _holed_face()

    assert len(cad.wires(face)) == 2
    assert len(cad.edges(face)) == 8


def test_extruded_square_keeps_the_located_copies_of_its_rails():
    """A prism's top rail is its base rail at another Location -- a distinct edge.

    4 base + 4 top + 4 verticals = 12. A placement-blind identity drops the top
    four and reports 8.
    """
    prism = cad.extrude_face_along_normal(cad.polygon_face(_SQUARE), 0.5)

    assert len(cad.edges(prism)) == 12


def test_extruded_holed_face_keeps_every_rail():
    """Same as above with a hole: (4 + 4) base + (4 + 4) top + 8 verticals = 24.

    A placement-blind identity reports 16 here.
    """
    prism = cad.extrude_face_along_normal(_holed_face(), 0.5)

    assert len(cad.edges(prism)) == 24


def test_closed_wire_keeps_all_four_of_its_edges():
    """A wire owns each edge once, so uniqueness must leave its edge list alone."""
    wire = cad.build_wire(
        [
            [0, 0, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 1, 0],
            [0, 1, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 0, 0],
        ]
    )

    edges = cad.edges(wire)
    assert len(edges) == 4
    corners = {tuple(round(c, 9) for c in p) for e in edges for p in cad.vertex_points(e)}
    assert corners == {(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)}
