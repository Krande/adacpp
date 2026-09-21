"""Curved-face imprint and grid->B-spline surface fit.

Both replace a step that previously had no native path, so what these pin is
mostly the SHAPE OF THE ANSWER rather than the geometry: an empty sub_faces
entry means "not imprinted, author it whole" and not "the face is gone", and a
fit that lands too far from its own input must decline rather than return a
surface that misses the nodes. Getting either backwards loses geometry silently.
"""

import math

import pytest

cad = pytest.importorskip("adacpp.cad")


def _bottom_face(box):
    """The z-min face of a centred box, picked by bounds rather than by index --
    face order is not part of any contract."""
    zmin = cad.bbox(box)[2]
    for f in cad.faces(box):
        b = cad.bbox(f)
        if abs(b[2] - zmin) < 1e-9 and abs(b[5] - zmin) < 1e-9:
            return f
    raise AssertionError("no z-min face on the box")


# --------------------------------------------------------------------------
# imprint_advanced_faces
# --------------------------------------------------------------------------


def test_curve_across_a_face_splits_it_in_two():
    box = cad.make_box(2.0, 2.0, 2.0)
    face = _bottom_face(box)
    z = cad.bbox(face)[2]
    # A line along y at x=0, spanning the full width, so it cuts rather than grazes.
    curve = [(0.0, -1.5, z), (0.0, 1.5, z)]

    res = cad.imprint_advanced_faces([face], [curve], 1e-6)

    assert res.n_split == 1
    assert res.n_errored == 0 and res.n_invalid == 0
    assert len(res.sub_faces) == 1
    subs = res.sub_faces[0]
    assert len(subs) == 2, "a line across the middle makes exactly two pieces"

    # The pieces tile the original: no area invented, none lost.
    assert math.isclose(sum(cad.area(s) for s in subs), cad.area(face), rel_tol=1e-9)

    # The curve is reported as the edges it became, and only where they bound a face.
    assert len(res.curve_edges) == 1
    edges = res.curve_edges[0]
    assert edges, "the cutting curve should report at least one bounding edge"
    for e in edges:
        assert len(e) == 6
        # The imprint runs along y, so every reported edge stays on x=0 and on the face plane.
        assert abs(e[0]) < 1e-6 and abs(e[3]) < 1e-6
        assert abs(e[2] - z) < 1e-6 and abs(e[5] - z) < 1e-6


def test_a_curve_that_misses_leaves_the_face_whole():
    box = cad.make_box(2.0, 2.0, 2.0)
    face = _bottom_face(box)
    # Far outside the face's bounds in x, so the prefilter drops it.
    curve = [(50.0, -1.5, -1.0), (50.0, 1.5, -1.0)]

    res = cad.imprint_advanced_faces([face], [curve], 1e-6)

    assert res.n_split == 0
    # EMPTY means "not imprinted" -- the caller authors the face whole. This is the
    # distinction that silently drops plates if it is read as "the face vanished".
    assert res.sub_faces == [[]]
    assert res.curve_edges == [[]]


def test_a_curve_only_touching_the_boundary_does_not_split():
    box = cad.make_box(2.0, 2.0, 2.0)
    face = _bottom_face(box)
    b = cad.bbox(face)
    # Along the face's own x-min edge: it lies on the boundary, cutting nothing off.
    curve = [(b[0], b[1], b[2]), (b[0], b[4], b[2])]

    res = cad.imprint_advanced_faces([face], [curve], 1e-6)

    assert res.n_split == 0, "an edge-coincident curve divides no area"
    assert res.sub_faces[0] == []


def test_no_curves_leaves_everything_whole():
    face = _bottom_face(cad.make_box(1.0, 1.0, 1.0))
    res = cad.imprint_advanced_faces([face], [], 1e-6)
    assert res.n_split == 0
    assert res.sub_faces == [[]]
    assert res.curve_edges == []


# --------------------------------------------------------------------------
# fit_bspline_face_from_grid
# --------------------------------------------------------------------------


def _flat_grid(n=4, size=1.0):
    step = size / (n - 1)
    return [[(i * step, j * step, 0.0) for j in range(n)] for i in range(n)]


def test_flat_grid_fits_a_face_of_the_right_extent():
    grid = _flat_grid(4, 2.0)
    face = cad.fit_bspline_face_from_grid(grid, 1e-6)

    assert face is not None
    b = cad.bbox(face)
    assert math.isclose(b[3] - b[0], 2.0, abs_tol=1e-6)
    assert math.isclose(b[4] - b[1], 2.0, abs_tol=1e-6)
    assert math.isclose(cad.area(face), 4.0, rel_tol=1e-6)


def test_curved_grid_fits_through_its_nodes():
    # A parabolic ridge in x -- curvature the fit has to follow, not average away.
    n = 5
    grid = [[(i * 0.5, j * 0.5, 0.2 * (i * 0.5 - 1.0) ** 2) for j in range(n)] for i in range(n)]

    face = cad.fit_bspline_face_from_grid(grid, 1e-6)

    assert face is not None
    b = cad.bbox(face)
    # The surface spans the nodes' own z range rather than flattening to a plane.
    assert b[5] - b[2] > 0.1
    # Curved, so its area exceeds the 2x2 footprint it projects onto.
    assert cad.area(face) > 4.0


def test_degenerate_grids_decline():
    # One row is not a surface.
    assert cad.fit_bspline_face_from_grid([[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]], 1e-6) is None
    # Ragged rows are not a grid.
    ragged = [
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
        [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0)],
    ]
    assert cad.fit_bspline_face_from_grid(ragged, 1e-6) is None
    # Empty.
    assert cad.fit_bspline_face_from_grid([], 1e-6) is None


def test_a_fit_that_cannot_meet_the_tolerance_declines():
    # A sawtooth no smooth surface passes through, asked for at an impossible tolerance.
    n = 6
    grid = [[(i * 0.2, j * 0.2, 1.0 if (i + j) % 2 else -1.0) for j in range(n)] for i in range(n)]

    assert cad.fit_bspline_face_from_grid(grid, 1e-12) is None, (
        "declining is the point: the caller falls back rather than shipping a surface "
        "that misses its own input"
    )
