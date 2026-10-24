"""adacpp.clash on a viewer GLB: members read from the GLB's element tables, hulled and measured.

Two kinds of input:

* a SYNTHETIC GLB written here, so every rule is checked against a known answer -- members keyed by
  element id across material nodes, node transforms applied, the Y-up convention converted, and the
  contacts identical to ``find_convex_contacts`` on the same vertices;
* a REAL viewer GLB (tests/fixtures/clash/glb_case.glb: files/my_test.ifc through the native IFC ->
  GLB stream, meshopt-compressed) and the contacts recorded for it. tools/test_clash_wasm.mjs runs
  the wasm module on the same bytes against the same record, so the browser and the worker are held
  to one answer.
"""

import json
import pathlib
import struct

import numpy as np
import pytest

clash = pytest.importorskip("adacpp.clash")

FIXTURES = pathlib.Path(__file__).parent.parent / "fixtures" / "clash"
CASE = json.loads((FIXTURES / "glb_case.json").read_text(encoding="utf-8"))


# ── a minimal viewer GLB ─────────────────────────────────────────────────────────────────────


def _box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = [[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    t = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4)]
    t += [(2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]
    return np.asarray(v, dtype=np.float32), np.asarray(t, dtype=np.uint32)


def _write_glb(nodes, hierarchy, scene_roots=None) -> bytes:
    """``nodes``: [(node name, {element id: (vertices, triangles)}, node extras like matrix/translation)].
    One mesh per node; each element is a draw range of that node's index buffer."""
    gltf = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "nodes": [],
        "meshes": [],
        "accessors": [],
        "bufferViews": [],
        "buffers": [],
    }
    extras = {"id_hierarchy": {k: [v, "*"] for k, v in hierarchy.items()}}
    blob = bytearray()

    def view(data: bytes) -> int:
        while len(blob) % 4:
            blob.append(0)
        gltf["bufferViews"].append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(data)})
        blob.extend(data)
        return len(gltf["bufferViews"]) - 1

    for name, elements, node_extra in nodes:
        positions, indices, ranges = [], [], {}
        for eid, (v, t) in elements.items():
            base = sum(len(p) for p in positions)
            ranges[eid] = [len(indices) * 3, len(t) * 3]
            positions.append(v)
            indices.extend((t + base).tolist())
        pos = np.concatenate(positions).astype(np.float32)
        idx = np.asarray(indices, dtype=np.uint32).reshape(-1)
        gltf["accessors"].append(
            {"bufferView": view(pos.tobytes()), "componentType": 5126, "count": len(pos), "type": "VEC3"}
        )
        gltf["accessors"].append(
            {"bufferView": view(idx.tobytes()), "componentType": 5125, "count": len(idx), "type": "SCALAR"}
        )
        gltf["meshes"].append(
            {
                "primitives": [
                    {"attributes": {"POSITION": len(gltf["accessors"]) - 2}, "indices": len(gltf["accessors"]) - 1}
                ]
            }
        )
        gltf["nodes"].append({"name": name, "mesh": len(gltf["meshes"]) - 1, **node_extra})
        extras[f"draw_ranges_{name}"] = ranges
    gltf["scenes"] = [{"nodes": scene_roots or list(range(len(gltf["nodes"]))), "extras": extras}]
    gltf["buffers"] = [{"byteLength": len(blob)}]

    js = json.dumps(gltf).encode()
    js += b" " * (-len(js) % 4)
    blob += b"\0" * (-len(blob) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)
    return (
        struct.pack("<4sII", b"glTF", 2, total)
        + struct.pack("<I4s", len(js), b"JSON")
        + js
        + struct.pack("<I4s", len(blob), b"BIN\0")
        + bytes(blob)
    )


def _frame_glb():
    """A column (element 1) split over two material nodes, a beam (2) landing on it, and a second
    beam (3) sharing the name "beam" -- in a node translated by +x 2 mm into the first beam."""
    col_bottom = _box((0, 0, 0), (0.2, 0.2, 1.5))
    col_top = _box((0, 0, 1.5), (0.2, 0.2, 3.0))
    beam = _box((0.198, 0.05, 2.5), (2.0, 0.15, 2.7))  # 2 mm into the column
    beam2 = _box((-0.002, 0.05, 2.5), (1.0, 0.15, 2.7))  # moved by the node: x + 2.0 -> 1.998 .. 3.0
    nodes = [
        ("node0", {"1": col_bottom, "2": beam}, {}),
        ("node1", {"1": col_top, "3": beam2}, {"translation": [2.0, 0.0, 0.0]}),
    ]
    return _write_glb(nodes, {"1": "col", "2": "beam", "3": "beam"})


# ── synthetic ────────────────────────────────────────────────────────────────────────────────


def test_members_are_elements_merged_across_material_nodes():
    members = clash.glb_members(_frame_glb())
    assert [(i, n) for i, n, _ in members] == [("1", "col"), ("2", "beam"), ("3", "beam")]
    col = members[0][2]
    # Both halves of the column, but the node-1 half moved by its node's translation.
    assert col.shape == (16, 3)
    assert col[:, 2].min() == 0.0 and col[:, 2].max() == pytest.approx(3.0)
    assert sorted(set(np.round(col[:, 0], 6))) == [0.0, 0.2, 2.0, 2.2]
    beam2 = members[2][2]
    assert beam2[:, 0].min() == pytest.approx(1.998) and beam2[:, 0].max() == pytest.approx(3.0)


def test_glb_contacts_equal_the_same_check_on_the_same_vertices():
    glb = _frame_glb()
    r = clash.find_glb_contacts(glb)
    assert r["members"] == [("1", "col"), ("2", "beam"), ("3", "beam")]
    ref = clash.find_convex_contacts([v for _, _, v in clash.glb_members(glb)])
    assert r["pairs"].tolist() == ref["pairs"].tolist()
    for key in ("near1", "near2", "normal", "penetration_depth", "contact_area"):
        assert np.array_equal(r[key], ref[key]), key
    # The column (its hull spans both halves, x 0 .. 2.2) meets both beams; the beams overlap 2 mm.
    assert r["pairs"].tolist() == [[0, 1], [0, 2], [1, 2]]


def test_a_y_up_glb_comes_back_in_the_z_up_model_frame():
    glb = _frame_glb()
    z = clash.glb_members(glb)
    y = clash.glb_members(glb, up_axis="y")
    for (_, _, vz), (_, _, vy) in zip(z, y):
        # The inverse of the Z-up -> Y-up export rotation: (x, y, z) -> (x, -z, y).
        assert np.allclose(vy, np.column_stack([vz[:, 0], -vz[:, 2], vz[:, 1]]))


def test_a_node_matrix_is_applied_like_trs():
    trs = clash.glb_members(
        _write_glb([("n", {"1": _box((0, 0, 0), (1, 1, 1))}, {"translation": [5, 6, 7]})], {"1": "a"})
    )
    mat = clash.glb_members(
        _write_glb(
            [("n", {"1": _box((0, 0, 0), (1, 1, 1))}, {"matrix": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1]})],
            {"1": "a"},
        )
    )
    assert np.array_equal(trs[0][2], mat[0][2])
    assert trs[0][2].min(axis=0).tolist() == [5, 6, 7]


@pytest.mark.parametrize(
    "bad, match",
    [(b"not a glb at all", "not a GLB"), (b"glTF" + b"\x02\0\0\0" + b"\0" * 12, "")],
)
def test_bytes_that_are_not_a_glb_raise(bad, match):
    with pytest.raises(ValueError, match=match):
        clash.find_glb_contacts(bad)


def test_up_axis_is_checked():
    with pytest.raises(ValueError, match="up_axis"):
        clash.glb_members(_frame_glb(), up_axis="x")


# ── the real viewer GLB ──────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("as_path", [True, False])
def test_real_viewer_glb_matches_its_record(as_path):
    glb = FIXTURES / CASE["glb"]
    r = clash.find_glb_contacts(
        glb if as_path else glb.read_bytes(),
        security_margin=CASE["security_margin"],
        min_contact_area=CASE["min_contact_area"],
        line_contact_thickness=CASE["line_contact_thickness"],
    )
    assert [list(m) for m in r["members"]] == CASE["members"]
    assert r["pairs"].tolist() == [c["pair"] for c in CASE["contacts"]]
    assert r["below_min_area"] == CASE["below_min_area"]
    for q, c in enumerate(CASE["contacts"]):
        for key in ("near1", "near2", "normal"):
            np.testing.assert_allclose(r[key][q], c[key], rtol=0, atol=1e-9)
        assert r["penetration_depth"][q] == pytest.approx(c["penetration_depth"], abs=1e-9)
        assert r["contact_area"][q] == pytest.approx(c["contact_area"], rel=1e-7)
        assert len(r["patches"][q]) == len(c["patches"])
        for got, want in zip(r["patches"][q], c["patches"]):
            np.testing.assert_allclose(got["points1"], np.asarray(want["points1"]).reshape(-1, 3), rtol=0, atol=1e-9)
