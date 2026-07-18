"""Regression: a diagonal planar face with holes must not crash the planar-face builder.

``build_advanced_face_planar`` used to run an UNGUARDED FindPlane+Add path
(``BRepBuilderAPI_MakeFace(outer_wire, /*OnlyPlane=*/True)`` then ``Add(hole_wire)``).
When the outer boundary is only *near* planar (import tolerance above
``Precision::Confusion``) and the plane is diagonally oriented, ``MakeFace::Add`` of a
hole wire SIGSEGVs deep in OCC — an uncatchable crash that the guarded declared-plane
fallback never got to handle. The fix builds holed faces on the explicit declared plane
first. This test reconstructs that geometry from scratch (no external model) and asserts
the face builds and meshes.
"""

import math

import pytest

cad = pytest.importorskip("adacpp.cad")

_S2 = math.sqrt(0.5)
# A vertical diagonal plane through the origin: normal [0.7071, -0.7071, 0].
_N = [_S2, -_S2, 0.0]
# In-plane orthonormal basis (u perpendicular to n in the xy-plane, v the z-axis).
_U = [_S2, _S2, 0.0]
_V = [0.0, 0.0, 1.0]


def _p(a, b, off=0.0):
    """Point at (a along u, b along v), pushed `off` off the plane along the normal."""
    return [a * _U[i] + b * _V[i] + off * _N[i] for i in range(3)]


def _line(p, q):
    # kind-0 edge record: [0, x1, y1, z1, x2, y2, z2]
    return [0.0, p[0], p[1], p[2], q[0], q[1], q[2]]


def _loop(points):
    return [_line(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]


def _rect(a0, a1, b0, b1):
    return [_p(a0, b0), _p(a1, b0), _p(a1, b1), _p(a0, b1)]


def test_diagonal_planar_face_with_holes_builds():
    # 9-edge outer boundary, NEAR planar: a few vertices sit ~1e-3 off the plane, modelling the
    # import tolerance that defeats FindPlane. This near-planarity + multiple holes is what crashed.
    e = 1.0e-3
    outer = _loop([
        _p(-6, -4), _p(-2, -4), _p(2, -4, e), _p(6, -4), _p(6, 0, -e),
        _p(6, 4), _p(0, 4, e), _p(-6, 4), _p(-6, 0, -e),
    ])
    # Four coplanar rectangular holes, wound opposite the outer loop.
    holes = [
        _rect(-5, -3, -3, 3),
        _rect(-1, 1, -3, 3),
        _rect(3, 5, -3, -1),
        _rect(3, 5, 1, 3),
    ]
    bounds = [outer] + [list(reversed(_loop(h))) for h in holes]

    # Pre-fix this call SIGSEGVs (takes the whole interpreter down); post-fix it returns a face.
    sh = cad.build_advanced_face_planar([0.0, 0.0, 0.0], _N, _U, bounds)
    assert len(cad.faces(sh)) == 1
    mesh = cad.tessellate(sh, -1.0)
    assert len(mesh.indices) > 0, "holed diagonal planar face produced no triangles"
