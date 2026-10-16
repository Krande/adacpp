"""adacpp.clash: convex-contact clash detection on coal.

The parity cases (tests/fixtures/clash/convex_contact_cases.json) are what coal's own Python bindings
report for the same meshes -- see tools/gen_clash_cases.py. tools/test_clash_wasm.mjs checks the wasm
module against the same file.
"""

import json
import pathlib

import numpy as np
import pytest

clash = pytest.importorskip("adacpp.clash")

CASES = pathlib.Path(__file__).parent.parent / "fixtures" / "clash" / "convex_contact_cases.json"
FIXTURE = json.loads(CASES.read_text(encoding="utf-8"))

# coal's GJK/EPA is iterative; the same sources built by another compiler may round differently in
# the last bits.
TOL = 1e-9
AREA_RTOL = 1e-7


def _run_case(case, hull):
    return clash.find_convex_contacts(
        [np.asarray(m["vertices"], dtype=float).reshape(-1, 3) for m in case["meshes"]],
        [np.asarray(m["triangles"], dtype=np.uint32).reshape(-1, 3) for m in case["meshes"]],
        security_margin=FIXTURE["security_margin"],
        min_contact_area=FIXTURE["min_contact_area"],
        line_contact_thickness=FIXTURE["line_contact_thickness"],
        hull=hull,
    )


def _assert_matches(r, case, tol, area_rtol):
    expected = case["contacts"]
    assert r["failures"] == []
    assert r["below_min_area"] == case["below_min_area"]
    assert [tuple(p) for p in r["pairs"].tolist()] == [tuple(e["pair"]) for e in expected]
    for q, e in enumerate(expected):
        np.testing.assert_allclose(r["near1"][q], e["near1"], rtol=0, atol=tol)
        np.testing.assert_allclose(r["near2"][q], e["near2"], rtol=0, atol=tol)
        np.testing.assert_allclose(r["normal"][q], e["normal"], rtol=0, atol=tol)
        assert r["penetration_depth"][q] == pytest.approx(e["penetration_depth"], abs=tol)
        assert r["contact_area"][q] == pytest.approx(e["contact_area"], rel=area_rtol)
        # The contact-patch polygons, as coal computed them: same patches, same vertices, same order.
        got = r["patches"][q]
        assert len(got) == len(e["patches"])
        for g, w in zip(got, e["patches"]):
            np.testing.assert_allclose(g["normal"], w["normal"], rtol=0, atol=tol)
            assert g["points1"].shape == (len(w["points1"]), 3)
            np.testing.assert_allclose(g["points1"], np.asarray(w["points1"]).reshape(-1, 3), rtol=0, atol=tol)
            np.testing.assert_allclose(g["points2"], np.asarray(w["points2"]).reshape(-1, 3), rtol=0, atol=tol)


def test_a_face_contact_patch_is_the_overlap_polygon():
    # Two unit-cross-section boxes meeting end-on: the patch is their 0.2 x 0.2 shared face.
    face = next(c for c in FIXTURE["cases"] if c["name"] == "face overlap")
    r = _run_case(face, hull=False)
    (patch,) = r["patches"][0]
    assert np.allclose(np.abs(patch["normal"]), (1, 0, 0))
    assert sorted(map(tuple, np.round(patch["points1"][:, 1:], 9).tolist())) == [
        (0.0, 0.0),
        (0.0, 0.2),
        (0.2, 0.0),
        (0.2, 0.2),
    ]
    # The two polygons are the same one, a penetration depth apart along the normal.
    gap = patch["points2"] - patch["points1"]
    assert np.allclose(np.linalg.norm(gap, axis=1), abs(r["penetration_depth"][0]))


@pytest.mark.parametrize("case", FIXTURE["cases"], ids=lambda c: c["name"])
def test_matches_coal_python(case):
    # The fixture's meshes are hulls already: as given, this is coal on coal.
    _assert_matches(_run_case(case, hull=False), case, TOL, AREA_RTOL)


@pytest.mark.parametrize("case", FIXTURE["cases"], ids=lambda c: c["name"])
def test_rehulled_meshes_give_the_same_contacts(case):
    # Taking the hull again re-triangulates coplanar faces, but keeps the input's vertex order -- the
    # one thing coal's contact patch is sensitive to here (a near-parallel edge is a line patch or a
    # point depending on it) -- so the hull of a hull gives exactly the same contacts.
    r = _run_case(case, hull=True)
    assert r["degenerate"] == []
    _assert_matches(r, case, TOL, AREA_RTOL)


def test_convex_hull_of_a_cube_with_inner_points():
    corners = np.array([[x, y, z] for x in (0, 1) for y in (0, 2) for z in (0, 3)], dtype=float)
    inner = np.random.default_rng(1).uniform(0.1, 0.9, (50, 3))
    v, t = clash.convex_hull(np.vstack([inner, corners]))
    assert v.tolist() == corners.tolist()  # the hull's vertices, in input order
    assert t.shape == (12, 3) and t.dtype == np.uint32
    # Outward-wound: every face normal points away from the centre.
    c = v.mean(axis=0)
    n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    assert np.all(np.einsum("ij,ij->i", n, v[t[:, 0]] - c) > 0)
    # Points spanning no volume have no hull.
    flat = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)
    assert clash.convex_hull(flat)[0].shape == (0, 3)


def test_hull_makes_a_non_convex_member_safe():
    # An L-shaped member's vertices only (no triangles). A small box sits in the L's inner corner:
    # clear of the L itself, inside its convex hull -- which is the solid the check measures, so
    # the pair is found rather than silently missed on a non-convex soup.
    L = np.array(
        [[x, y, z] for z in (0, 0.2) for x, y in [(0, 0), (1, 0), (1, 0.2), (0.2, 0.2), (0.2, 1), (0, 1)]], dtype=float
    )
    box = np.array([[x, y, z] for x in (0.4, 0.6) for y in (0.4, 0.6) for z in (0, 0.2)], dtype=float)
    flat = np.array([[5, 5, 0], [6, 5, 0], [5, 6, 0]], dtype=float)
    r = clash.find_convex_contacts([L, box, flat])
    assert r["pairs"].tolist() == [[0, 1]]
    assert r["degenerate"] == [2]


def test_without_hull_triangles_are_required():
    with pytest.raises(ValueError, match="triangles are required"):
        clash.find_convex_contacts([np.zeros((4, 3))], hull=False)


def _all_pairs_within(lo, hi, security_margin):
    """Every pair whose boxes come within the padded margin on every axis -- brute force, no sweep."""
    finite = np.isfinite(lo).all(axis=1) & np.isfinite(hi).all(axis=1)
    scale = float(np.abs(np.concatenate([lo[finite], hi[finite]])).max()) if finite.any() else 0.0
    # The broad phase's padding: the margin (never negative) plus slack for rounding.
    pad = max(float(security_margin), 0.0) + 1e-6 + 1e-9 * scale
    idx = np.flatnonzero(finite)
    return [
        (int(i), int(j))
        for a, i in enumerate(idx)
        for j in idx[a + 1 :]
        if np.all(lo[j] <= hi[i] + pad) and np.all(lo[i] <= hi[j] + pad)
    ]


@pytest.mark.parametrize("margin", [0.0, 0.01, 0.2, -1.0])
def test_candidate_pairs_is_all_pairs(margin):
    rng = np.random.default_rng(7)
    lo = rng.uniform(-50, 50, (300, 3))
    hi = lo + rng.uniform(0, 4, (300, 3))
    lo[[3, 40]] = np.inf  # empty meshes
    hi[[3, 40]] = -np.inf
    assert clash.candidate_pairs(lo, hi, margin) == _all_pairs_within(lo, hi, margin)


def test_patch_area_rules():
    up = [0.0, 0.0, 1.0]
    # Fewer than two points: no area. Two: a line contact, length * thickness.
    assert clash.patch_area(np.zeros((1, 3)), up) == 0.0
    assert clash.patch_area([[0, 0, 0], [0.5, 0, 0]], up, 1e-3) == pytest.approx(0.5e-3)
    # A square patch far from the origin: its area, not cancelled away by the coordinates.
    sq = np.array([[0, 0, 0], [0.1, 0, 0], [0.1, 0.2, 0], [0, 0.2, 0]]) + [312.0, -148.0, 22.0]
    assert clash.patch_area(sq, up) == pytest.approx(0.02, rel=1e-9)
    # Collinear points: qhull would call them flat, so a line contact as long as the patch.
    line = [[0, 0, 0], [0.2, 0, 0], [0.4, 0, 0]]
    assert clash.patch_area(line, up, 1e-3) == pytest.approx(0.4e-3)


def test_malformed_input_raises():
    v = np.zeros((3, 3))
    with pytest.raises(ValueError, match="indexes past"):
        clash.find_convex_contacts([v, v], [np.array([[0, 1, 5]]), np.array([[0, 1, 2]])], hull=False)
    with pytest.raises(ValueError, match="one triangle array"):
        clash.find_convex_contacts([v], [])
