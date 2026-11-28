"""A B-spline face bounded by 3D edges only (no 2D p-curves) is a valid face, and sews.

A thickened curved shell is closed by ruled B-spline strips between its two offset faces. Those
strips arrive with 3D boundary curves and no p-curves. ``build_advanced_face_bspline`` used to
return such a face without p-curves: BRepCheck called it invalid, BRepMesh gridded nothing and
``sew_faces`` dropped it, so of the thick shell's six faces only an end face survived the sewing.

The geometry here is synthetic: a cubic arch swept across a width, thickened downwards.
"""

import pytest

cad = pytest.importorskip("adacpp.cad")

# Cubic Bezier arch in the x-z plane (one span, clamped).
ARCH = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.6), (2.0, 0.0, 0.6), (3.0, 0.0, 0.0)]
WIDTH = 1.5
T = 0.025
KNOTS, MULTS = [0.0, 1.0], [4, 4]


def _add(p, d):
    return tuple(a + b for a, b in zip(p, d))


def _bspline_edge(poles):
    """Kind-3 edge record: an untrimmed non-rational cubic over ``poles``."""
    rec = [3, 3, 0, 0, 0.0, 1.0, *poles[0], *poles[-1], len(poles)]
    for p in poles:
        rec.extend(p)
    rec += [len(KNOTS), *KNOTS, *MULTS]
    return rec


def _line(a, b):
    return [0, *a, *b]


def _ruled(rows_a, rows_b):
    """A degree 3x1 surface between two cubic rows, and its pcurve-free boundary loop."""
    ctrl = [[a, b] for a, b in zip(rows_a, rows_b)]
    loop = [
        _bspline_edge(rows_a),
        _line(rows_a[-1], rows_b[-1]),
        _bspline_edge(list(reversed(rows_b))),
        _line(rows_b[0], rows_a[0]),
    ]
    return cad.build_advanced_face_bspline(3, 1, ctrl, KNOTS, [0.0, 1.0], MULTS, [2, 2], [], [loop])


def _arch(dy=0.0, dz=0.0):
    return [_add(p, (0.0, dy, dz)) for p in ARCH]


def _arch_length(n=2000):
    import itertools
    import math

    def at(t):
        b = [(1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t**2 * (1 - t), t**3]
        return [sum(w * p[k] for w, p in zip(b, ARCH)) for k in range(3)]

    pts = [at(i / n) for i in range(n + 1)]
    return sum(math.dist(a, b) for a, b in itertools.pairwise(pts))


def test_ruled_bspline_face_without_pcurves_is_valid():
    face = _ruled(_arch(), _arch(dz=-T))
    assert cad.is_valid(face)
    # The arch lies in the x-z plane, so the strip under it is the flat ribbon of its x-span.
    assert cad.area(face) == pytest.approx((ARCH[-1][0] - ARCH[0][0]) * T, rel=1e-6)


def test_thick_curved_shell_sews_all_six_faces():
    top = _ruled(_arch(), _arch(dy=WIDTH))
    bottom = _ruled(_arch(dz=-T), _arch(dy=WIDTH, dz=-T))
    side_a = _ruled(_arch(), _arch(dz=-T))
    side_b = _ruled(_arch(dy=WIDTH), _arch(dy=WIDTH, dz=-T))
    x0, x1 = ARCH[0], ARCH[-1]
    end_a = cad.polygon_face([x0, _add(x0, (0, WIDTH, 0)), _add(x0, (0, WIDTH, -T)), _add(x0, (0, 0, -T))])
    end_b = cad.polygon_face([x1, _add(x1, (0, 0, -T)), _add(x1, (0, WIDTH, -T)), _add(x1, (0, WIDTH, 0))])
    for f in (top, bottom, side_a, side_b):
        assert cad.is_valid(f)
    shell = cad.sew_faces([top, bottom, side_a, side_b, end_a, end_b])
    length = _arch_length()
    span = ARCH[-1][0] - ARCH[0][0]
    expected = 2 * length * WIDTH + 2 * span * T + 2 * WIDTH * T
    assert cad.area(shell) == pytest.approx(expected, rel=2e-3)
