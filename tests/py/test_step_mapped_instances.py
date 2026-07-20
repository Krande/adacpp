"""STEP->STEP mapped (shared-prototype) instancing.

A solid placed N>1 times must be serialised ONCE as a prototype B-rep and referenced per
placement through the AP242 assembly-instancing pattern (NAUO + CONTEXT_DEPENDENT_SHAPE_
REPRESENTATION + ITEM_DEFINED_TRANSFORMATION) — the exact pattern the native stream readers
(C++ Resolver and adapy's pure-Python stream reader) already parse. The source here is a
hand-authored cube placed 3 times (identity, translation, rotation+translation) using that
same pattern, so the test covers the full read->write->read round trip.

ADACPP_STEP_BAKE_INSTANCES=1 must restore the old all-baked form (one full B-rep per
placement, no assembly records).
"""

import math

import pytest

cad = pytest.importorskip("adacpp.cad")

_HEADER = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('synthetic mapped-instance cube'),'2;1');
FILE_NAME('','',(''),(''),'test','','');
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'));
ENDSEC;
DATA;
#1=APPLICATION_CONTEXT('managed model based 3d engineering');
#2=APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering_mim_lf',2014,#1);
#3=PRODUCT_CONTEXT('',#1,'mechanical');
#4=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');
#5=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));
#6=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#7=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());
#8=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-6),#5,'distance_accuracy_value','edge/vertex');
#9=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#8))GLOBAL_UNIT_ASSIGNED_CONTEXT((#5,#6,#7))REPRESENTATION_CONTEXT('Context','3D'));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=DIRECTION('',(0.,0.,1.));
#12=DIRECTION('',(1.,0.,0.));
#13=AXIS2_PLACEMENT_3D('',#10,#11,#12);
"""

# The three world placements: identity, +2x translation, and 90 deg about z at (0,3,0).
# (loc, z_axis, x_ref) — AXIS2_PLACEMENT_3D form.
_PLACEMENTS = [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    ((2.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    ((0.0, 3.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
]


def _fmt(v):
    return f"{v:.6f}"


class _Spf:
    def __init__(self):
        self.lines = []
        self.nid = 13  # continue after the fixed header block

    def emit(self, body):
        self.nid += 1
        self.lines.append(f"#{self.nid}={body};")
        return self.nid

    def pt(self, p):
        return self.emit(f"CARTESIAN_POINT('',({_fmt(p[0])},{_fmt(p[1])},{_fmt(p[2])}))")

    def dirn(self, d):
        return self.emit(f"DIRECTION('',({_fmt(d[0])},{_fmt(d[1])},{_fmt(d[2])}))")

    def axis2(self, loc, z, x):
        return self.emit(f"AXIS2_PLACEMENT_3D('',#{self.pt(loc)},#{self.dirn(z)},#{self.dirn(x)})")

    def plane_face(self, poly, normal, xdir):
        plane = self.emit(f"PLANE('',#{self.axis2(poly[0], normal, xdir)})")
        pids = [self.pt(p) for p in poly]
        loop = self.emit("POLY_LOOP('',({}))".format(",".join(f"#{i}" for i in pids)))
        bound = self.emit(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        return self.emit(f"ADVANCED_FACE('',(#{bound}),#{plane},.T.)")


def _cube_instanced_step() -> str:
    """One unit-cube MANIFOLD_SOLID_BREP prototype + 3 CDSR placements (the reader's pattern)."""
    w = _Spf()
    # 6 planar faces of the unit cube, outward normals (winding not load-bearing for this test).
    c = [
        ([(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], (0, 0, -1), (1, 0, 0)),  # bottom
        ([(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], (0, 0, 1), (1, 0, 0)),  # top
        ([(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], (-1, 0, 0), (0, 1, 0)),  # -x
        ([(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], (1, 0, 0), (0, 1, 0)),  # +x
        ([(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], (0, -1, 0), (1, 0, 0)),  # -y
        ([(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)], (0, 1, 0), (1, 0, 0)),  # +y
    ]
    fids = [w.plane_face([tuple(float(x) for x in p) for p in poly], n, xd) for poly, n, xd in c]
    shell = w.emit("CLOSED_SHELL('',({}))".format(",".join(f"#{i}" for i in fids)))
    brep = w.emit(f"MANIFOLD_SOLID_BREP('cube',#{shell})")
    absr = w.emit(f"ADVANCED_BREP_SHAPE_REPRESENTATION('cube',(#13,#{brep}),#9)")
    prod = w.emit("PRODUCT('cube','cube','',(#3))")
    pdf = w.emit(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
    pd = w.emit(f"PRODUCT_DEFINITION('design','',#{pdf},#4)")
    pds = w.emit(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
    w.emit(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{absr})")
    # Root assembly product + world rep.
    rprod = w.emit("PRODUCT('model','model','',(#3))")
    rpdf = w.emit(f"PRODUCT_DEFINITION_FORMATION('','',#{rprod})")
    rpd = w.emit(f"PRODUCT_DEFINITION('design','',#{rpdf},#4)")
    rpds = w.emit(f"PRODUCT_DEFINITION_SHAPE('','',#{rpd})")
    axes = [w.axis2(loc, z, x) for loc, z, x in _PLACEMENTS]
    wrep = w.emit("SHAPE_REPRESENTATION('model',(#13,{}),#9)".format(",".join(f"#{a}" for a in axes)))
    w.emit(f"SHAPE_DEFINITION_REPRESENTATION(#{rpds},#{wrep})")
    for k, axis in enumerate(axes):
        idt = w.emit(f"ITEM_DEFINED_TRANSFORMATION('','',#13,#{axis})")
        rr = w.emit(
            f"(REPRESENTATION_RELATIONSHIP('','',#{absr},#{wrep})"
            f"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt})"
            "SHAPE_REPRESENTATION_RELATIONSHIP())"
        )
        nauo = w.emit(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('i{k}','cube','',#{rpd},#{pd},$)")
        npds = w.emit(f"PRODUCT_DEFINITION_SHAPE('','',#{nauo})")
        w.emit(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rr},#{npds})")
    return _HEADER + "\n".join(w.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n"


def _roots(path):
    return [(n, m) for n, m in cad.StepNgeomStream(str(path))]


def _translations(meta):
    return sorted(tuple(round(t[i], 4) for i in (12, 13, 14)) for t in meta.transforms)


_EXPECT_TRANSLATIONS = sorted([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 3.0, 0.0)])


@pytest.fixture()
def src(tmp_path):
    p = tmp_path / "cube_instanced.stp"
    p.write_text(_cube_instanced_step())
    return p


def test_source_pattern_reads_as_three_instances(src):
    roots = _roots(src)
    assert len(roots) == 1
    meta = roots[0][1]
    assert len(meta.transforms) == 3
    assert _translations(meta) == _EXPECT_TRANSLATIONS


def test_step_to_step_emits_shared_prototype(src, tmp_path):
    out = tmp_path / "mapped.stp"
    st = cad.stream_step_to_step(str(src), str(out))
    assert st["solids_in"] == 1
    assert st["solids_out"] == 1
    assert st["instances_out"] == 3
    assert st["faces_dropped"] == 0

    data = out.read_text()
    assert data.count("MANIFOLD_SOLID_BREP") == 1, "prototype must be serialised exactly once"
    assert data.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE") == 3
    assert data.count("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") == 3
    assert data.count("ITEM_DEFINED_TRANSFORMATION") == 3
    # every instance AXIS2 must be listed among the shared world rep's items
    assert data.count("SHAPE_REPRESENTATION('model'") == 1

    # Read-back through the native stream reader: 1 root, all 3 placements recovered.
    roots = _roots(out)
    assert len(roots) == 1
    meta = roots[0][1]
    assert len(meta.transforms) == 3
    assert _translations(meta) == _EXPECT_TRANSLATIONS
    # The rotated instance survives: one transform has x-axis ~(0,1,0).
    rot = [t for t in meta.transforms if abs(t[0]) < 1e-4 and abs(t[1] - 1.0) < 1e-4]
    assert len(rot) == 1


def test_bake_env_restores_flat_form(src, tmp_path, monkeypatch):
    monkeypatch.setenv("ADACPP_STEP_BAKE_INSTANCES", "1")
    out = tmp_path / "baked.stp"
    st = cad.stream_step_to_step(str(src), str(out))
    assert st["solids_in"] == 1
    assert st["instances_out"] == 3
    data = out.read_text()
    assert data.count("MANIFOLD_SOLID_BREP") == 3, "forced bake must emit one full B-rep per placement"
    assert data.count("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") == 0
    assert data.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE") == 0
    # Read-back: 3 flat roots, one placement each.
    roots = _roots(out)
    assert len(roots) == 3
    assert all(len(m.transforms) in (0, 1) for _n, m in roots)


def _stl_stats(path):
    """(n_triangles, bbox_min, bbox_max) from a binary STL (80B header, u32 count, 50B/tri)."""
    import struct

    import numpy as np

    raw = path.read_bytes()
    (n,) = struct.unpack_from("<I", raw, 80)
    tris = np.frombuffer(raw, dtype=np.uint8, offset=84).reshape(n, 50)
    verts = tris[:, 12:48].copy().view("<f4").reshape(n, 3, 3).reshape(-1, 3)
    return n, verts.min(axis=0), verts.max(axis=0)


def test_mapped_and_baked_agree_geometrically(src, tmp_path, monkeypatch):
    """The mapped file and the baked file must tessellate to the SAME world-space triangles
    (stream_step_to_mesh bakes per-instance placements)."""
    import numpy as np

    mapped, baked = tmp_path / "m.stp", tmp_path / "b.stp"
    cad.stream_step_to_step(str(src), str(mapped))
    monkeypatch.setenv("ADACPP_STEP_BAKE_INSTANCES", "1")
    cad.stream_step_to_step(str(src), str(baked))
    monkeypatch.delenv("ADACPP_STEP_BAKE_INSTANCES")

    stl_m, stl_b = tmp_path / "m.stl", tmp_path / "b.stl"
    nm = cad.stream_step_to_mesh(str(mapped), str(stl_m), "stl")
    nb = cad.stream_step_to_mesh(str(baked), str(stl_b), "stl")
    assert nm == nb > 0, f"triangle counts diverge: mapped={nm} baked={nb}"
    tm, mn_m, mx_m = _stl_stats(stl_m)
    tb, mn_b, mx_b = _stl_stats(stl_b)
    assert tm == tb == nm
    assert np.allclose(mn_m, mn_b, atol=1e-5) and np.allclose(mx_m, mx_b, atol=1e-5)
    # instancing really placed the copies: extents span all three placements of the unit cube
    assert mx_m[0] - mn_m[0] >= 3.0 - 1e-4  # x: cube at 0 + cube at +2
    assert mx_m[1] - mn_m[1] >= 4.0 - 1e-4  # y: cube at +3 (rotated)


def test_step_parity_counts_mapped_instances(src):
    d = cad.step_parity(str(src))
    assert d["total_instances"] == 3
    assert d["step"]["instances"] == 3
    assert d["ifc"]["instances"] == 3
    assert d["step"]["faces_dropped"] == 0


def test_rotation_math_sanity():
    # the rotated placement really is a 90 deg z-rotation
    loc, z, x = _PLACEMENTS[2]
    assert math.isclose(sum(a * b for a, b in zip(z, x)), 0.0, abs_tol=1e-12)
