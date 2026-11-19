"""solid_from_polygons / solids_from_polygons_batch: a faceted B-rep made a solid in C++.

All geometry here is synthetic: a cube, and a triangulated bead-like loft (a fillet-shaped section
swept round an arc).
"""

import math
import time

import numpy as np
import pytest

from adacpp import cad

IN, OUT = 0, 1


def cube(size=1.0, origin=(0.0, 0.0, 0.0)):
    """The six faces of an axis-aligned cube, wound outward."""
    ox, oy, oz = origin
    s = size
    p = [(ox + x * s, oy + y * s, oz + z * s) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
    # p index = 4x + 2y + z
    quads = [
        (0, 1, 3, 2),  # x = 0
        (4, 6, 7, 5),  # x = 1
        (0, 4, 5, 1),  # y = 0
        (2, 3, 7, 6),  # y = 1
        (0, 2, 6, 4),  # z = 0
        (1, 5, 7, 3),  # z = 1
    ]
    return [[p[i] for i in q] for q in quads]


def bead(n_stations=26, n_profile=6, leg=8.0, radius=200.0, sweep=None, offset=(0.0, 0.0, 0.0)):
    """A fillet-shaped section (two legs and a bulged face) swept round an arc and triangulated:
    2 * n_profile * (n_stations - 1) side triangles and two n_profile-gon end caps."""
    sweep = math.radians(40) if sweep is None else sweep
    # The section in (radial, up): the root corner, the legs' toes and a bulge between them.
    sec = [(0.0, 0.0), (leg, 0.0)]
    for k in range(1, n_profile - 2):
        t = k / (n_profile - 2)
        a = t * math.pi / 2
        sec.append((leg * (1 - t) + 0.25 * leg * math.sin(a * 2), leg * t + 0.25 * leg * math.sin(a * 2)))
    sec.append((0.0, leg))
    assert len(sec) == n_profile
    ox, oy, oz = offset
    rings = []
    for i in range(n_stations):
        th = sweep * i / (n_stations - 1)
        c, s = math.cos(th), math.sin(th)
        rings.append([(ox + (radius + u) * c, oy + (radius + u) * s, oz + v) for u, v in sec])
    polys = []
    for i in range(n_stations - 1):
        a, b = rings[i], rings[i + 1]
        for k in range(n_profile):
            k1 = (k + 1) % n_profile
            polys.append([a[k], b[k], b[k1]])
            polys.append([a[k], b[k1], a[k1]])
    polys.append(list(reversed(rings[0])))
    polys.append(list(rings[-1]))
    return polys


def maker_volume_reference(polys):
    faces = [cad.polygon_face(p) for p in polys]
    vols = cad.make_volumes_from_faces(faces)
    assert len(vols) == 1
    return vols[0]


def test_cube_direct():
    s = cad.solid_from_polygons(cube(2.0))
    assert cad.is_valid(s)
    assert cad.volume(s) == pytest.approx(8.0, rel=1e-9)
    assert len(cad.faces(s)) == 6
    assert cad.point_in_solid(s, [1.0, 1.0, 1.0]) == IN
    assert cad.point_in_solid(s, [3.0, 1.0, 1.0]) == OUT
    assert cad.solid_from_polygons_route(cube(2.0)) == "direct"


def test_direct_solid_tessellates_and_cuts():
    # Its faces carry no stored pcurves (lines on planes need none): meshing and booleans still
    # work on it, which is what a caller does next.
    s = cad.solid_from_polygons(bead())
    mesh = cad.tessellate(s)
    assert len(mesh.indices) // 3 >= 302
    box = cad.solid_from_polygons(cube(800.0, origin=(-400.0, -400.0, 4.0)))  # everything above z=4
    below = cad.boolean("DIFFERENCE", s, box)
    above = cad.boolean("INTERSECTION", s, box)
    assert 0 < cad.volume(below) < cad.volume(s)
    assert cad.volume(below) + cad.volume(above) == pytest.approx(cad.volume(s), rel=1e-9)


def test_cube_inward_and_mixed_winding_comes_out_outward():
    inward = [list(reversed(q)) for q in cube(2.0)]
    mixed = [list(reversed(q)) if i % 2 else q for i, q in enumerate(cube(2.0))]
    for polys in (inward, mixed):
        s = cad.solid_from_polygons(polys)
        assert cad.is_valid(s)
        assert cad.volume(s) == pytest.approx(8.0, rel=1e-9)  # positive: turned outward
        assert cad.point_in_solid(s, [1.0, 1.0, 1.0]) == IN
        assert cad.solid_from_polygons_route(polys) == "direct"


def test_bead_matches_maker_volume():
    polys = bead()
    assert len(polys) == 2 * 6 * 25 + 2
    s = cad.solid_from_polygons(polys)
    assert cad.is_valid(s)
    assert cad.solid_from_polygons_route(polys) == "direct"
    ref = maker_volume_reference(polys)
    assert cad.volume(s) > 0
    assert cad.volume(s) == pytest.approx(cad.volume(ref), rel=1e-9)
    assert cad.center_of_mass(s) == pytest.approx(cad.center_of_mass(ref), abs=1e-7)


def prism(outline, height):
    """A straight prism over a planar outline in z = 0 (wound either way)."""
    bot = [(x, y, 0.0) for x, y in outline]
    top = [(x, y, height) for x, y in outline]
    n = len(outline)
    sides = [[bot[i], bot[(i + 1) % n], top[(i + 1) % n], top[i]] for i in range(n)]
    return [list(reversed(bot)), top] + sides


def test_concave_cap_prism_direct():
    ell = [(0, 0), (3, 0), (3, 1), (1, 1), (1, 2), (0, 2)]  # an L: area 4
    polys = prism(ell, 2.0)
    assert cad.solid_from_polygons_route(polys) == "direct"
    s = cad.solid_from_polygons(polys)
    assert cad.is_valid(s)
    assert cad.volume(s) == pytest.approx(8.0, rel=1e-12)
    assert cad.point_in_solid(s, [2.0, 1.5, 1.0]) == OUT  # in the L's notch
    assert cad.point_in_solid(s, [0.5, 1.5, 1.0]) == IN


def test_slightly_warped_quad_stays_valid():
    # One top corner a little out of the plane of the other three: the face takes the plane
    # through its centroid and its boundary the tolerance to reach it.
    polys = cube(1.0)
    lift = 2e-5
    polys = [[(x, y, z + lift) if (x, y, z) == (1.0, 1.0, 1.0) else (x, y, z) for x, y, z in q] for q in polys]
    s = cad.solid_from_polygons(polys)
    assert cad.solid_from_polygons_route(polys) == "direct"
    assert cad.is_valid(s)
    assert cad.volume(s) == pytest.approx(1.0, abs=1e-4)


def as_mesh(polys, weld=True):
    """The polygons as (vertices, index loops): shared corners once each, or one copy per face."""
    vertices, faces, index = [], [], {}
    for loop in polys:
        face = []
        for p in loop:
            key = tuple(p) if weld else len(vertices)
            if key not in index:
                index[key] = len(vertices)
                vertices.append(tuple(p))
            face.append(index[key])
        faces.append(face)
    return vertices, faces


def test_long_bead_is_direct_and_linear():
    # The shape of a long lofted bead: ~100 stations of a 4-corner section, ~800 triangles. The
    # sewing a closed shell does not need is what made this kind of input take minutes.
    polys = bead(n_stations=100, n_profile=4)
    assert len(polys) == 2 * 4 * 99 + 2
    assert cad.solid_from_polygons_route(polys) == "direct"
    s = cad.solid_from_polygons(polys)
    assert cad.is_valid(s)
    assert cad.volume(s) == pytest.approx(cad.volume(maker_volume_reference(polys)), rel=1e-9)


@pytest.mark.parametrize("weld", [True, False])
def test_mesh_form_matches_polygon_form(weld):
    polys = bead()
    vertices, faces = as_mesh(polys, weld=weld)
    s = cad.solid_from_mesh(vertices, faces)
    assert cad.is_valid(s)
    assert cad.volume(s) == pytest.approx(cad.volume(cad.solid_from_polygons(polys)), rel=1e-12)
    s2 = cad.solid_from_mesh(np.asarray(vertices), faces)
    assert cad.volume(s2) == pytest.approx(cad.volume(s), rel=1e-12)


def test_mesh_bad_index_raises():
    vertices, faces = as_mesh(cube(1.0))
    faces[2] = [0, 1, 99]
    with pytest.raises(ValueError, match="out of range"):
        cad.solid_from_mesh(vertices, faces)


def test_meshes_batch():
    items = [as_mesh(bead(offset=(30.0 * i, 0.0, 0.0))) for i in range(6)]
    items.append(as_mesh(cube(1.0)[:-1]))
    shapes, errors = cad.solids_from_meshes_batch(items, threads=3)
    assert errors[:6] == [None] * 6 and shapes[6] is None and "no volume" in errors[6]
    for (v, f), s in zip(items[:6], shapes[:6]):
        assert cad.volume(s) == pytest.approx(cad.volume(cad.solid_from_mesh(v, f)), rel=1e-12)


def test_numpy_loops():
    polys = [np.asarray(p, dtype=float) for p in bead(n_stations=5)]
    s = cad.solid_from_polygons(polys)
    assert cad.volume(s) == pytest.approx(cad.volume(cad.solid_from_polygons(bead(n_stations=5))), rel=1e-12)


def test_split_face_falls_back_and_still_closes():
    # One face of the cube split in two where its neighbours are not: the split corner sits on an
    # edge the neighbour runs straight along (a T-junction), so welding alone cannot pair the edges.
    polys = cube(2.0)
    x1 = polys.pop(1)  # x = 1: (4, 6, 7, 5) = (2,0,0) (2,2,0) (2,2,2) (2,0,2)
    assert x1[0] == (2.0, 0.0, 0.0)
    polys.append([(2.0, 0.0, 0.0), (2.0, 1.0, 0.0), (2.0, 1.0, 2.0), (2.0, 0.0, 2.0)])
    polys.append([(2.0, 1.0, 0.0), (2.0, 2.0, 0.0), (2.0, 2.0, 2.0), (2.0, 1.0, 2.0)])
    assert cad.solid_from_polygons_route(polys) == "maker_volume"
    s = cad.solid_from_polygons(polys)
    assert cad.volume(s) == pytest.approx(8.0, rel=1e-6)
    assert cad.point_in_solid(s, [1.0, 1.0, 1.0]) == IN


def test_open_shell_raises():
    polys = cube(2.0)[:-1]  # no lid
    with pytest.raises(ValueError, match="no volume"):
        cad.solid_from_polygons(polys)


def test_no_polygons_raises():
    with pytest.raises(ValueError):
        cad.solid_from_polygons([])
    with pytest.raises(ValueError):
        cad.solid_from_polygons([[(0, 0, 0), (1, 0, 0)]])


def test_batch_equals_one_by_one_and_isolates_failures():
    items = [bead(offset=(30.0 * i, 0.0, 0.0), sweep=math.radians(20 + i)) for i in range(12)]
    items.insert(5, cube(1.0)[:-1])  # an open shell in the middle of the batch
    shapes, errors = cad.solids_from_polygons_batch(items, threads=4)
    assert len(shapes) == len(errors) == len(items)
    assert shapes[5] is None and "no volume" in errors[5]
    for i, polys in enumerate(items):
        if i == 5:
            continue
        assert errors[i] is None
        assert cad.volume(shapes[i]) == pytest.approx(cad.volume(cad.solid_from_polygons(polys)), rel=1e-12)


def test_batch_empty_and_serial():
    assert cad.solids_from_polygons_batch([]) == ([], [])
    shapes, errors = cad.solids_from_polygons_batch([cube(1.0)], threads=1)
    assert errors == [None] and cad.volume(shapes[0]) == pytest.approx(1.0)


def test_batch_parallel_timing():
    # Timing is printed, not asserted: a loaded CI runner must not fail this. The results are.
    items = [bead(offset=(30.0 * i, 0.0, 0.0)) for i in range(48)]
    t0 = time.perf_counter()
    serial, e1 = cad.solids_from_polygons_batch(items, threads=1)
    t1 = time.perf_counter()
    parallel, e2 = cad.solids_from_polygons_batch(items, threads=0)
    t2 = time.perf_counter()
    assert all(e is None for e in e1 + e2)
    for a, b in zip(serial, parallel):
        assert cad.volume(a) == pytest.approx(cad.volume(b), rel=1e-12)
    print(f"\n48 beads: serial {t1 - t0:.3f} s, all cores {t2 - t1:.3f} s")
