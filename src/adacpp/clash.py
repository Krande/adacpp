"""Convex-contact clash detection on coal -- the same C++ as the ``adacpp_clash`` wasm module.

:func:`find_convex_contacts` takes one convex mesh per member (pass convex hulls) and reports every
pair that touches or overlaps within ``security_margin``: the nearest point on each, coal's contact
normal (from the first mesh towards the second), the signed penetration depth (coal's sign: negative
is overlap) and the contact-patch area. Compiled once and called from Python and from the browser, so
a worker and a browser report the same contacts.

:func:`candidate_pairs` (the broad phase) and :func:`patch_area` (one patch's area) are exposed for
testing and for callers that measure patches themselves.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from ._ada_cpp_ext_impl import clash as _clash

DEFAULT_SECURITY_MARGIN = _clash.DEFAULT_SECURITY_MARGIN
DEFAULT_MIN_CONTACT_AREA = _clash.DEFAULT_MIN_CONTACT_AREA
DEFAULT_LINE_CONTACT_THICKNESS = _clash.DEFAULT_LINE_CONTACT_THICKNESS


def find_convex_contacts(
    vertices: Sequence[np.ndarray],
    triangles: Sequence[np.ndarray],
    security_margin: float = DEFAULT_SECURITY_MARGIN,
    min_contact_area: float = DEFAULT_MIN_CONTACT_AREA,
    line_contact_thickness: float = DEFAULT_LINE_CONTACT_THICKNESS,
) -> dict:
    """Every touching / overlapping pair among the meshes ``(vertices[k], triangles[k])``.

    ``vertices[k]`` is ``(n, 3)`` and ``triangles[k]`` ``(m, 3)`` indices into it; an empty mesh is
    allowed and touches nothing. Returns a dict of columns, one row per contact, in ``(i, j)`` order:
    ``pairs`` ``(k, 2)`` int64 with ``i < j``, ``near1`` / ``near2`` / ``normal`` ``(k, 3)``,
    ``penetration_depth`` ``(k,)`` and ``contact_area`` ``(k,)``; plus ``failures`` -- ``(i, j, reason)``
    for pairs coal could not evaluate -- ``candidate_pairs`` and ``below_min_area``.
    """
    if len(vertices) != len(triangles):
        raise ValueError("find_convex_contacts: one triangle array per vertex array")
    vs = [np.ascontiguousarray(np.asarray(v, dtype=np.float64).reshape(-1, 3)) for v in vertices]
    ts = [np.ascontiguousarray(np.asarray(t).reshape(-1, 3), dtype=np.uint32) for t in triangles]
    return _clash.find_convex_contacts(
        vs, ts, float(security_margin), float(min_contact_area), float(line_contact_thickness)
    )


def candidate_pairs(lo: np.ndarray, hi: np.ndarray, security_margin: float) -> list[tuple[int, int]]:
    """Index pairs ``(i, j)``, ``i < j``, sorted, whose boxes come within ``security_margin`` on every axis."""
    lo = np.ascontiguousarray(np.asarray(lo, dtype=np.float64).reshape(-1, 3))
    hi = np.ascontiguousarray(np.asarray(hi, dtype=np.float64).reshape(-1, 3))
    return [tuple(p) for p in _clash.candidate_pairs(lo, hi, float(security_margin))]


def patch_area(points, normal, line_contact_thickness: float = DEFAULT_LINE_CONTACT_THICKNESS) -> float:
    """The area of one contact patch: ``points`` ``(n, 3)`` in the plane with ``normal``."""
    pts = np.ascontiguousarray(np.asarray(points, dtype=np.float64).reshape(-1, 3))
    n = [float(x) for x in np.asarray(normal, dtype=np.float64).reshape(3)]
    return _clash.patch_area(pts, n, float(line_contact_thickness))


__all__ = [
    "DEFAULT_LINE_CONTACT_THICKNESS",
    "DEFAULT_MIN_CONTACT_AREA",
    "DEFAULT_SECURITY_MARGIN",
    "candidate_pairs",
    "find_convex_contacts",
    "patch_area",
]
