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


@pytest.mark.parametrize("case", FIXTURE["cases"], ids=lambda c: c["name"])
def test_matches_coal_python(case):
    r = clash.find_convex_contacts(
        [np.asarray(m["vertices"], dtype=float).reshape(-1, 3) for m in case["meshes"]],
        [np.asarray(m["triangles"], dtype=np.uint32).reshape(-1, 3) for m in case["meshes"]],
        security_margin=FIXTURE["security_margin"],
        min_contact_area=FIXTURE["min_contact_area"],
        line_contact_thickness=FIXTURE["line_contact_thickness"],
    )
    expected = case["contacts"]
    assert r["failures"] == []
    assert r["below_min_area"] == case["below_min_area"]
    assert [tuple(p) for p in r["pairs"].tolist()] == [tuple(e["pair"]) for e in expected]
    for q, e in enumerate(expected):
        np.testing.assert_allclose(r["near1"][q], e["near1"], rtol=0, atol=TOL)
        np.testing.assert_allclose(r["near2"][q], e["near2"], rtol=0, atol=TOL)
        np.testing.assert_allclose(r["normal"][q], e["normal"], rtol=0, atol=TOL)
        assert r["penetration_depth"][q] == pytest.approx(e["penetration_depth"], abs=TOL)
        assert r["contact_area"][q] == pytest.approx(e["contact_area"], rel=AREA_RTOL)


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
        clash.find_convex_contacts([v, v], [np.array([[0, 1, 5]]), np.array([[0, 1, 2]])])
    with pytest.raises(ValueError, match="one triangle array"):
        clash.find_convex_contacts([v], [])
