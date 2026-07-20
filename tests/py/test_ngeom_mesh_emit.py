"""NGEOM-blob record streams -> GLB / OBJ / STL (stream_ngeom_to_glb / stream_ngeom_to_mesh).

The ada-object-model MESH export fast path: same record front-end as stream_ngeom_to_step/ifc,
back half of the native STEP->GLB / STEP->mesh tessellate+emit cores. Blobs are produced by the
sibling binding StepNgeomStream from synthetic cube STEP (see test_ngeom_blob_emit), so the tests
are self-contained and the step-source native converters double as the oracle: the records path
must produce the same triangle/vertex counts as stream_step_to_glb / stream_step_to_mesh on the
same file.
"""

import json
import pathlib
import struct

import pytest

cad = pytest.importorskip("adacpp.cad")

from test_ngeom_blob_emit import _RED, _T_IDENT, _T_SHIFT, _records, _two_cube_step  # noqa: E402


@pytest.fixture()
def step_path(tmp_path):
    p = tmp_path / "cubes.stp"
    p.write_text(_two_cube_step())
    return p


@pytest.fixture()
def blobs(step_path):
    out = [(bytes(b), m) for b, m in cad.StepNgeomStream(str(step_path))]
    assert len(out) == 2
    return out


def _glb_json(path) -> dict:
    data = pathlib.Path(path).read_bytes()
    assert data[:4] == b"glTF", "not a GLB container"
    (json_len, json_type) = struct.unpack_from("<II", data, 12)
    assert json_type == 0x4E4F534A  # 'JSON'
    return json.loads(data[20 : 20 + json_len])


def _glb_counts(path) -> tuple[int, int]:
    """(triangles, vertices) summed over every primitive, from the accessor headers (valid for
    raw and meshopt-compressed GLBs alike — EXT_meshopt only re-encodes the buffer bytes)."""
    j = _glb_json(path)
    acc = j["accessors"]
    tris = verts = 0
    for mesh in j.get("meshes", []):
        for prim in mesh["primitives"]:
            tris += acc[prim["indices"]]["count"] // 3
            verts += acc[prim["attributes"]["POSITION"]]["count"]
    return tris, verts


def _stl_tris(path) -> int:
    data = pathlib.Path(path).read_bytes()
    return struct.unpack_from("<I", data, 80)[0]


def _obj_counts(path) -> tuple[int, int]:
    v = f = 0
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                v += 1
            elif line.startswith("f "):
                f += 1
    return f, v


# ---------------------------------------------------------------------------------------------
# GLB


def test_glb_records_match_step_native(blobs, step_path, tmp_path):
    """Records -> GLB carries exactly the geometry the step-source native converter produces."""
    ref = tmp_path / "ref.glb"
    n = cad.stream_step_to_glb(str(step_path), str(ref))
    assert n == 2
    out = tmp_path / "out.glb"
    st = cad.stream_ngeom_to_glb(_records(blobs), str(out))
    assert st["solids_in"] == 2
    assert st["solids_out"] == 2
    assert st["solids_skipped"] == 0
    assert st["instances_out"] == 2
    assert st["faces_dropped"] == 0
    assert _glb_counts(out) == _glb_counts(ref)
    assert st["triangles"] == _glb_counts(out)[0]
    # per-solid draw ranges carry the record names (picking)
    raw = out.read_bytes()
    assert b"cube_a" in raw and b"cube_b" in raw


def test_glb_records_colour(blobs, tmp_path):
    out = tmp_path / "red.glb"
    cad.stream_ngeom_to_glb(_records(blobs, color=_RED), str(out))
    j = _glb_json(out)
    cols = [m["pbrMetallicRoughness"]["baseColorFactor"] for m in j["materials"]]
    assert len(cols) == 1, "same colour must merge into one material"
    r, g, b, a = cols[0]
    assert (round(r, 4), round(g, 4), round(b, 4), a) == (1.0, 0.0, 0.0, 1.0)


def test_glb_records_instances(blobs, tmp_path):
    b, _m = blobs[0]
    out = tmp_path / "inst.glb"
    st = cad.stream_ngeom_to_glb([("box", b, None, [_T_IDENT, _T_SHIFT], None)], str(out))
    assert st["solids_in"] == 1
    assert st["instances_out"] == 2
    tris, _verts = _glb_counts(out)
    assert tris == st["triangles"] == 2 * 12  # both placements baked (12 tris per cube)


def test_glb_records_unit_scale(blobs, tmp_path):
    """unit_scale converts model units to metres: a mm-model cube becomes 0.001 m."""
    out_m = tmp_path / "m.glb"
    out_mm = tmp_path / "mm.glb"
    cad.stream_ngeom_to_glb(_records(blobs), str(out_m))
    cad.stream_ngeom_to_glb(_records(blobs), str(out_mm), unit_scale=0.001)

    def _pos_max(path):
        j = _glb_json(path)
        acc = j["accessors"]
        return max(max(acc[p["attributes"]["POSITION"]]["max"]) for m in j["meshes"] for p in m["primitives"])

    hi_m = _pos_max(out_m)
    hi_mm = _pos_max(out_mm)
    assert hi_m == pytest.approx(4.0)  # cube_b spans x in [3, 4]
    assert hi_mm == pytest.approx(0.004)


def test_glb_generator_and_thread_count_deterministic(blobs, tmp_path):
    """Generator input and any thread count must produce byte-identical output (ordered commits)."""
    recs = _records(blobs, color=_RED)
    a = tmp_path / "a.glb"
    b = tmp_path / "b.glb"
    c = tmp_path / "c.glb"
    cad.stream_ngeom_to_glb(recs, str(a))
    cad.stream_ngeom_to_glb((r for r in recs), str(b))
    cad.stream_ngeom_to_glb(recs, str(c), num_threads=1)
    assert a.read_bytes() == b.read_bytes() == c.read_bytes()


def test_glb_unknown_pipeline_raises(blobs, tmp_path):
    with pytest.raises(RuntimeError, match="pipeline"):
        cad.stream_ngeom_to_glb(_records(blobs), str(tmp_path / "x.glb"), pipeline="no-such-track")


# ---------------------------------------------------------------------------------------------
# OBJ / STL


@pytest.mark.parametrize("fmt", ["obj", "stl"])
def test_mesh_records_match_step_native(blobs, step_path, tmp_path, fmt):
    """Records -> OBJ/STL triangle+vertex counts equal the step-source native converter's."""
    ref = tmp_path / f"ref.{fmt}"
    ref_tris = cad.stream_step_to_mesh(str(step_path), str(ref), fmt)
    out = tmp_path / f"out.{fmt}"
    st = cad.stream_ngeom_to_mesh(_records(blobs), str(out), fmt)
    assert st["solids_in"] == 2
    assert st["solids_out"] == 2
    assert st["faces_dropped"] == 0
    assert st["triangles"] == ref_tris
    if fmt == "stl":
        assert _stl_tris(out) == _stl_tris(ref) == ref_tris
    else:
        assert _obj_counts(out) == _obj_counts(ref)
        assert _obj_counts(out)[0] == ref_tris


def test_mesh_records_instances_and_unit_scale(blobs, tmp_path):
    b, _m = blobs[0]
    out = tmp_path / "inst.stl"
    st = cad.stream_ngeom_to_mesh([("box", b, None, [_T_IDENT, _T_SHIFT], None)], str(out), "stl", unit_scale=0.001)
    assert st["instances_out"] == 2
    assert _stl_tris(out) == st["triangles"] == 2 * 12
    # unit_scale + the +5x shifted instance both baked: max x ~= (1 + 5) * 0.001
    data = out.read_bytes()
    xs = [struct.unpack_from("<f", data, 84 + 50 * i + 12)[0] for i in range(_stl_tris(out))]
    assert max(xs) <= 0.006 + 1e-6
    assert max(xs) > 0.004


def test_mesh_generator_and_thread_count_deterministic(blobs, tmp_path):
    recs = _records(blobs)
    a = tmp_path / "a.obj"
    b = tmp_path / "b.obj"
    c = tmp_path / "c.obj"
    cad.stream_ngeom_to_mesh(recs, str(a), "obj")
    cad.stream_ngeom_to_mesh((r for r in recs), str(b), "obj")
    cad.stream_ngeom_to_mesh(recs, str(c), "obj", num_threads=1)
    assert a.read_bytes() == b.read_bytes() == c.read_bytes()


def test_mesh_bad_fmt_raises(blobs, tmp_path):
    with pytest.raises(RuntimeError, match="fmt"):
        cad.stream_ngeom_to_mesh(_records(blobs), str(tmp_path / "x.ply"), "ply")
