"""NGEOM TRI_MESH (tag 68): a body that is already triangles goes through the record writers as is.

Lets a model whose objects are meshes (an IfcTriangulatedFaceSet, a body another kernel already
tessellated) share the native GLB / OBJ / STL writers with the analytic records, instead of the
whole model falling back to a Python writer.
"""

import json
import pathlib
import struct

import pytest

cad = pytest.importorskip("adacpp.cad")

TRI_MESH = 68
# A unit tetrahedron, one degenerate triangle appended (dropped by the writer).
POS = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
TRIS = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (0, 3, 2), (1, 1, 2)]


def _blob(pos=POS, tris=TRIS, root_id="tet") -> bytes:
    payload = struct.pack("<i", len(pos)) + b"".join(struct.pack("<3d", *p) for p in pos)
    flat = [i for t in tris for i in t]
    payload += struct.pack("<i", len(flat)) + struct.pack(f"<{len(flat)}i", *flat)
    out = b"ADANGEOM" + struct.pack("<ii", 1, 1)
    out += struct.pack("<ii", TRI_MESH, len(payload)) + payload
    rid = root_id.encode()
    out += struct.pack("<i", 1) + struct.pack("<ii", 0, len(rid)) + rid
    return out


def _glb_json(path) -> dict:
    data = pathlib.Path(path).read_bytes()
    n = struct.unpack_from("<I", data, 12)[0]
    return json.loads(data[20 : 20 + n])


def test_tag_is_advertised():
    assert TRI_MESH in cad.ngeom_mesh_record_tags()


def test_tri_mesh_record_to_glb(tmp_path):
    out = tmp_path / "tet.glb"
    st = cad.stream_ngeom_to_glb([("tet", _blob(), [1.0, 0.0, 0.0, 1.0], None, [[(1, "root"), (2, "tet")]])], str(out))
    assert st["solids_in"] == 1 and st["solids_out"] == 1 and st["solids_skipped"] == 0
    assert st["triangles"] == 4  # the degenerate one is dropped
    j = _glb_json(out)
    acc = j["accessors"]
    prims = [p for m in j["meshes"] for p in m["primitives"]]
    assert sum(acc[p["indices"]]["count"] for p in prims) == 12
    lo = [min(acc[p["attributes"]["POSITION"]]["min"][k] for p in prims) for k in range(3)]
    hi = [max(acc[p["attributes"]["POSITION"]]["max"][k] for p in prims) for k in range(3)]
    assert lo == pytest.approx([0, 0, 0]) and hi == pytest.approx([1, 1, 1])


def test_tri_mesh_unit_scale(tmp_path):
    out = tmp_path / "tet.glb"
    cad.stream_ngeom_to_glb([("tet", _blob(), None, None, None)], str(out), meshopt=False, unit_scale=0.001)
    acc = _glb_json(out)["accessors"]
    assert max(max(a.get("max", [0])) for a in acc if a.get("type") == "VEC3") == pytest.approx(0.001)


def test_tri_mesh_record_to_stl(tmp_path):
    out = tmp_path / "tet.stl"
    st = cad.stream_ngeom_to_mesh([("tet", _blob(), None, None, None)], str(out), "stl")
    assert st["solids_out"] == 1
    assert struct.unpack_from("<I", out.read_bytes(), 80)[0] == 4


def test_tri_mesh_index_out_of_range_is_an_error(tmp_path):
    with pytest.raises(RuntimeError, match="TRI_MESH index out of range"):
        cad.stream_ngeom_to_glb([("bad", _blob(tris=[(0, 1, 9)]), None, None, None)], str(tmp_path / "b.glb"))
