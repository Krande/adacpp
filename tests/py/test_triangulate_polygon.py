"""cad.triangulate_polygon: planar constrained-Delaunay (detria) of an outline + holes.

The contract param_models relies on (it replaced the `triangle` C library, which has no pyodide
build): the output vertices ARE the input vertices in input order, no Steiner points, triangles
cover the polygon minus its holes, every triangle is CCW, and bad input raises ValueError.
"""

import numpy as np
import pytest

from adacpp import cad


def shoelace(pts) -> float:
    p = np.asarray(pts, dtype=float)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def tri_signed_areas(verts, tris):
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    return 0.5 * ((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))


def check(loops, expected_area):
    verts, tris = cad.triangulate_polygon(loops)
    assert verts.dtype == np.float64 and verts.ndim == 2 and verts.shape[1] == 2
    assert tris.dtype == np.uint32 and tris.ndim == 2 and tris.shape[1] == 3
    # vertex preservation: input order, loop 0 then holes, nothing added
    expected = np.concatenate([np.asarray(lp, dtype=float) for lp in loops])
    np.testing.assert_array_equal(verts, expected)
    assert tris.max() < len(verts)
    areas = tri_signed_areas(verts, tris)
    assert np.all(areas > 0), "every triangle must be CCW"
    assert areas.sum() == pytest.approx(expected_area, rel=1e-12, abs=1e-12)
    # a simple polygon with h holes and n vertices has n + 2h - 2 triangles
    n = len(verts)
    assert len(tris) == n + 2 * (len(loops) - 1) - 2
    # every input vertex is used
    assert set(np.unique(tris)) == set(range(n))
    return verts, tris


SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
L_SHAPE = [(0, 0), (4, 0), (4, 1), (1, 1), (1, 3), (0, 3)]


def comb(teeth=6, w=1.0, depth=5.0):
    """A comb, CCW: a spine y in [0, 1] with `teeth` teeth of width w up to y=depth (very non-convex)."""
    pts = [(0.0, 0.0), ((2 * teeth - 1) * w, 0.0)]
    for i in reversed(range(teeth)):
        x0, x1 = 2 * i * w, 2 * i * w + w
        pts += [(x1, depth), (x0, depth)]
        if i > 0:
            pts += [(x0, 1.0), (x0 - w, 1.0)]
    return pts


def zigzag(n=10, amp=2.0):
    top = [(float(i), 3.0 + (amp if i % 2 else 0.0)) for i in range(n, -1, -1)]
    return [(0.0, 0.0), (float(n), 0.0)] + top


def test_square():
    verts, tris = check([SQUARE], 1.0)
    assert len(verts) == 4 and len(tris) == 2


def test_square_cw_and_closed_and_int_input():
    cw = list(reversed(SQUARE))
    check([cw], 1.0)
    # explicit closing point is tolerated and dropped
    verts, _ = cad.triangulate_polygon([SQUARE + [SQUARE[0]]])
    np.testing.assert_array_equal(verts, np.asarray(SQUARE))
    # integer lists and numpy arrays (incl. strided views) are both accepted
    check([[(0, 0), (2, 0), (2, 2), (0, 2)]], 4.0)
    big = np.asarray(SQUARE * 1, dtype=float)
    padded = np.zeros((4, 4))
    padded[:, ::2] = big
    check([padded[:, ::2]], 1.0)


def test_l_shape():
    check([L_SHAPE], abs(shoelace(L_SHAPE)))


@pytest.mark.parametrize("poly", [comb(), comb(teeth=12, w=0.1, depth=20.0), zigzag(), zigzag(31, 7.5)])
def test_comb_and_zigzag(poly):
    area = shoelace(poly)
    assert area > 0
    check([poly], area)
    check([list(reversed(poly))], area)


def test_square_with_hole():
    outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hole = [(4, 4), (6, 4), (6, 6), (4, 6)]
    verts, tris = check([outer, hole], 96.0)
    # no triangle lies inside the hole
    cent = verts[tris].mean(axis=1)
    inside_hole = (cent[:, 0] > 4) & (cent[:, 0] < 6) & (cent[:, 1] > 4) & (cent[:, 1] < 6)
    assert not inside_hole.any()


def test_two_holes_with_same_winding_as_outline():
    outer = [(0, 0), (20, 0), (20, 10), (0, 10)]
    h1 = [(2, 2), (8, 2), (8, 8), (2, 8)]  # CCW like the outline
    h2 = [(12, 2), (18, 2), (15, 8)]
    check([outer, h1, h2], 200.0 - 36.0 - shoelace(h2))


def test_collinear_boundary_points_kept():
    # extra vertices along the edges must survive (not be merged away) and edges stay unsplit
    poly = [(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2), (0, 2), (0, 1)]
    verts, _ = check([poly], 6.0)
    assert len(verts) == len(poly)


@pytest.mark.parametrize(
    "loops",
    [
        [],
        [[(0, 0), (1, 0)]],
        [[(0, 0), (1, 1), (0, 0)]],
        [[(0, 0), (1, 0), (2, 0), (3, 0)]],  # all collinear
        [[(0, 0), (1, 1), (1, 0), (0, 1)]],  # bow-tie (self-intersecting)
        [[(0, 0), (1, 0), (1, 1), (0, 1)], [(5, 5), (6, 5), (6, 6)]],  # hole outside outline
        [[(0, 0), (4, 0), (4, 4), (0, 4)], [(1, 1), (5, 1), (1, 3)]],  # hole crosses outline
        [[(0, 0), (1, 0), (1, 1), (0, 1)], [(0.2, 0.2), (0.4, 0.2)]],  # degenerate hole
        [[(0, 0), (1, 0), (np.nan, 1), (0, 1)]],
        [[(0, 0, 0), (1, 0, 0), (1, 1, 0)]],  # wrong shape
        [[0.0, 1.0, 2.0]],
        "abc",
    ],
)
def test_value_errors(loops):
    with pytest.raises(ValueError):
        cad.triangulate_polygon(loops)


def test_matches_triangle_library():
    triangle = pytest.importorskip("triangle")
    for poly in (L_SHAPE, comb(), zigzag()):
        pts = np.asarray(poly, dtype=float)
        n = len(pts)
        segs = np.stack([np.arange(n), (np.arange(n) + 1) % n], axis=1)
        ref = triangle.triangulate({"vertices": pts, "segments": segs}, "p")
        verts, tris = cad.triangulate_polygon([poly])
        np.testing.assert_array_equal(np.sort(verts, axis=0), np.sort(ref["vertices"], axis=0))
        rv, rt = ref["vertices"], ref["triangles"]
        assert tri_signed_areas(verts, tris).sum() == pytest.approx(np.abs(tri_signed_areas(rv, rt)).sum())
