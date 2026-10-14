"""Generate tests/fixtures/clash/convex_contact_cases.json -- the parity oracle for adacpp.clash.

The expected contacts come from coal's own Python bindings (conda-forge's coal, built by coal's own
CMake) for the collision and the contact patch, and from :func:`patch_area` below -- numpy plus
scipy's qhull -- for the patch area. So this runs in an env with coal and scipy, not adacpp's:

    pixi exec --spec coal==3.0.4 --spec scipy python tools/gen_clash_cases.py tests/fixtures/clash

Every pair is evaluated (no broad phase), so the fixture also checks that adacpp's broad phase drops
no pair coal would have reported.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import coal
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

SECURITY_MARGIN = 0.01
MIN_CONTACT_AREA = 1e-6
LINE_CONTACT_THICKNESS = 1e-3


def patch_area(shape1, T1, shape2, T2, col_res, col_req, thickness=LINE_CONTACT_THICKNESS) -> float:
    """The summed contact-patch area, by the rules src/clash/convex_contact.cpp implements.

    A patch of 2 points is a line contact: length * thickness. 3+ points are projected into the patch
    plane and measured by qhull; where qhull refuses them as flat, the patch's largest in-plane extent
    * thickness. Fewer than 2 points have no area.
    """
    patch_res = coal.ContactPatchResult()
    coal.computeContactPatch(shape1, T1, shape2, T2, col_res, coal.ContactPatchRequest(col_req), patch_res)
    total = 0.0
    for p in range(patch_res.numContactPatches()):
        patch = patch_res.getContactPatch(p)
        n = patch.size()
        if n < 2:
            continue
        pts = np.vstack([np.array(patch.getPointShape1(k)) for k in range(n)])
        if n == 2:
            total += float(np.linalg.norm(pts[1] - pts[0])) * thickness
            continue
        normal = np.array(patch.getNormal())
        normal /= np.linalg.norm(normal)
        arb = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(arb, normal)) > 0.9:
            arb = np.array([0.0, 1.0, 0.0])
        u1 = np.cross(normal, arb)
        u1 /= np.linalg.norm(u1)
        u2 = np.cross(normal, u1)
        proj = np.column_stack((pts @ u1, pts @ u2))
        try:
            total += float(ConvexHull(proj).volume)  # a 2-d hull's "volume" is its area
        except Exception:  # noqa: BLE001 - qhull refusing flat input is the line-contact case
            total += float(max(proj.max(axis=0) - proj.min(axis=0))) * thickness
    return total


def box(lo, hi, rot=None, about=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    v = np.array([[x, y, z] for z in (lo[2], hi[2]) for y in (lo[1], hi[1]) for x in (lo[0], hi[0])])
    if rot is not None:
        c = np.asarray(about if about is not None else (lo + hi) / 2, float)
        v = Rotation.from_euler("xyz", rot, degrees=True).apply(v - c) + c
    return hull_mesh(v)


def hull_mesh(points):
    """The convex hull as (vertices, outward-wound triangles)."""
    pts = np.asarray(points, float)
    h = ConvexHull(pts)
    used = np.unique(h.simplices)
    remap = {int(o): k for k, o in enumerate(used)}
    verts = pts[used]
    tris = []
    for s, eq in zip(h.simplices, h.equations):
        a, b, c = (remap[int(i)] for i in s)
        n = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        if np.dot(n, eq[:3]) < 0:
            b, c = c, b
        tris.append([a, b, c])
    return verts, np.asarray(tris, dtype=np.uint32)


def to_coal(verts, tris):
    v = coal.StdVec_Vec3s()
    v.extend(np.asarray(verts, float))
    f = coal.StdVec_Triangle()
    for t in tris:
        f.append(coal.Triangle(*[int(i) for i in t]))
    return coal.Convex(v, f)


def oracle(meshes):
    shapes = [to_coal(v, t) if len(v) else None for v, t in meshes]
    T = coal.Transform3s()
    contacts, below = [], 0
    for i in range(len(meshes)):
        for j in range(i + 1, len(meshes)):
            if shapes[i] is None or shapes[j] is None:
                continue
            req = coal.CollisionRequest()
            req.security_margin = SECURITY_MARGIN
            res = coal.CollisionResult()
            coal.collide(shapes[i], T, shapes[j], T, req, res)
            if not res.isCollision():
                continue
            area = patch_area(shapes[i], T, shapes[j], T, res, req)
            if area < MIN_CONTACT_AREA:
                below += 1
                continue
            c = res.getContact(0)
            contacts.append(
                {
                    "pair": [i, j],
                    "near1": [float(x) for x in c.getNearestPoint1()],
                    "near2": [float(x) for x in c.getNearestPoint2()],
                    "normal": [float(x) for x in c.normal],
                    "penetration_depth": float(c.penetration_depth),
                    "contact_area": float(area),
                }
            )
    return contacts, below


def cases():
    yield "face overlap", [box([0, 0, 0], [1, 0.2, 0.2]), box([0.998, 0, 0], [1.3, 0.2, 0.2])]
    yield "gap inside margin", [box([0, 0, 0], [1, 0.2, 0.2]), box([1.005, 0.05, 0.05], [1.3, 0.15, 0.15])]
    yield "apart", [box([0, 0, 0], [1, 0.2, 0.2]), box([1.5, 0, 0], [2, 0.2, 0.2])]
    # A box turned 45 degrees about its long axis, resting edge-down on a slab: a line contact.
    yield "edge on face", [
        box([0, 0, -0.2], [1, 1, 0]),
        box([0.2, 0.5 - 0.0707107, 0.0], [0.8, 0.5 + 0.0707107, 0.1414214], rot=[45, 0, 0]),
    ]
    # A cube balanced on one corner above a slab: a point contact, dropped by the area filter.
    yield "corner on face", [
        box([0, 0, -0.2], [1, 1, 0]),
        box([0.4, 0.4, 0.0], [0.6, 0.6, 0.2], rot=[35.26439, 45, 0], about=[0.5, 0.5, 0.1732051 - 0.0005]),
    ]
    yield "empty mesh", [box([0, 0, 0], [1, 1, 1]), (np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint32))]
    # Members of a small portal frame at plant coordinates (hundreds of metres from the origin): two
    # columns, a beam landing on both, a brace, and a stub overlapping the beam by 3 mm.
    o = np.array([312.0, -148.0, 22.0])
    yield "portal frame at plant coordinates", [
        box(o + [0, 0, 0], o + [0.3, 0.3, 4.0]),
        box(o + [5.7, 0, 0], o + [6.0, 0.3, 4.0]),
        box(o + [0, 0, 4.0], o + [6.0, 0.3, 4.5]),
        box(o + [0.3, 0.1, 0.5], o + [5.7, 0.2, 0.6], rot=[0, -30, 0]),
        box(o + [2.9, 0.3 - 0.003, 4.1], o + [3.1, 1.3, 4.3]),
    ]
    # Twenty-five random rotated prisms and point-cloud hulls in a 3 m cube: many contacts, every kind.
    rng = np.random.default_rng(20261014)
    rnd = []
    for k in range(25):
        c = rng.uniform(0, 3, 3)
        if k % 2:
            half = rng.uniform([0.05, 0.05, 0.3], [0.2, 0.2, 1.0])
            rnd.append(box(c - half, c + half, rot=rng.uniform(-90, 90, 3)))
        else:
            rnd.append(hull_mesh(c + rng.normal(0, 0.25, (20, 3))))
    yield "random cluster", rnd


def main(out_dir: str) -> None:
    out = {
        "schema": "adacpp.clash.convex_contact_cases/1",
        "generator": f"tools/gen_clash_cases.py (coal {coal.__version__} Python bindings + scipy qhull area)",
        "security_margin": SECURITY_MARGIN,
        "min_contact_area": MIN_CONTACT_AREA,
        "line_contact_thickness": LINE_CONTACT_THICKNESS,
        "cases": [],
    }
    for name, meshes in cases():
        contacts, below = oracle(meshes)
        out["cases"].append(
            {
                "name": name,
                "meshes": [
                    {"vertices": np.asarray(v).tolist(), "triangles": np.asarray(t).tolist()} for v, t in meshes
                ],
                "contacts": contacts,
                "below_min_area": below,
            }
        )
        print(f"{name}: {len(meshes)} meshes, {len(contacts)} contacts, {below} below min area")
    path = Path(out_dir) / "convex_contact_cases.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {path}")


if __name__ == "__main__":
    main(sys.argv[1])
