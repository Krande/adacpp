"""NGEOM-blob record streams -> STEP / IFC (stream_ngeom_to_step / stream_ngeom_to_ifc).

The ada-object-model export fast path: Python serializes each object's geometry to an NGEOM
blob and hands (name, blob, color, transforms, paths) records to the native emitters, replacing
the per-entity Python writers. Blobs here are produced by the sibling binding StepNgeomStream
(the same NGEOM wire the adapy serializer speaks), so the tests are self-contained: synthetic
cube STEP -> per-solid blobs -> records -> emit -> read back through the native readers.
"""

import pytest

cad = pytest.importorskip("adacpp.cad")

_HEADER = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('synthetic cubes'),'2;1');
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


def _fmt(v):
    return f"{v:.6f}"


class _Spf:
    def __init__(self):
        self.lines = []
        self.nid = 13

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

    def cube(self, name, origin=(0.0, 0.0, 0.0)):
        ox, oy, oz = origin
        c = [
            ([(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], (0, 0, -1), (1, 0, 0)),
            ([(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], (0, 0, 1), (1, 0, 0)),
            ([(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], (-1, 0, 0), (0, 1, 0)),
            ([(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], (1, 0, 0), (0, 1, 0)),
            ([(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], (0, -1, 0), (1, 0, 0)),
            ([(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)], (0, 1, 0), (1, 0, 0)),
        ]
        fids = [
            self.plane_face([(p[0] + ox, p[1] + oy, p[2] + oz) for p in poly], n, xd) for poly, n, xd in c
        ]
        shell = self.emit("CLOSED_SHELL('',({}))".format(",".join(f"#{i}" for i in fids)))
        brep = self.emit(f"MANIFOLD_SOLID_BREP('{name}',#{shell})")
        absr = self.emit(f"ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',(#13,#{brep}),#9)")
        prod = self.emit(f"PRODUCT('{name}','{name}','',(#3))")
        pdf = self.emit(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
        pd = self.emit(f"PRODUCT_DEFINITION('design','',#{pdf},#4)")
        pds = self.emit(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
        self.emit(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{absr})")


def _two_cube_step() -> str:
    w = _Spf()
    w.cube("cube_a")
    w.cube("cube_b", origin=(3.0, 0.0, 0.0))
    return _HEADER + "\n".join(w.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n"


# Two rigid placements for the instancing tests: identity and +5x.
_T_IDENT = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0.0, 0.0, 0.0, 1.0]
_T_SHIFT = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 5.0, 0.0, 0.0, 1.0]

_RED = (1.0, 0.0, 0.0, 1.0)


@pytest.fixture()
def blobs(tmp_path):
    """Per-solid NGEOM blobs for the two cubes (via the native STEP->NGEOM stream)."""
    p = tmp_path / "cubes.stp"
    p.write_text(_two_cube_step())
    out = [(bytes(b), m) for b, m in cad.StepNgeomStream(str(p))]
    assert len(out) == 2
    return out


def _records(blobs, color=None, paths=None):
    return [
        (m.id or f"solid_{i}", b, color, None, paths[i] if paths else None)
        for i, (b, m) in enumerate(blobs)
    ]


def test_step_flat_records(blobs, tmp_path):
    out = tmp_path / "out.stp"
    st = cad.stream_ngeom_to_step(_records(blobs, color=_RED), str(out))
    assert st["solids_in"] == 2
    assert st["solids_out"] == 2
    assert st["instances_out"] == 2
    assert st["faces_dropped"] == 0
    data = out.read_text()
    assert "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING" in data
    assert "SI_UNIT($,.METRE.)" in data
    assert data.count("CLOSED_SHELL") == 2
    assert data.count("MANIFOLD_SOLID_BREP") == 2
    # presentation colour: one STYLED_ITEM chain per solid + the single trailer
    assert data.count("STYLED_ITEM") == 2
    assert data.count("COLOUR_RGB('',1.,0.,0.)") == 2
    assert data.count("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION") == 1
    # read-back through the native stream reader: 2 roots, names preserved
    roots = [(bytes(b), m) for b, m in cad.StepNgeomStream(str(out))]
    assert len(roots) == 2
    assert sorted(m.id for _b, m in roots) == ["cube_a", "cube_b"]


def test_step_assembly_paths(blobs, tmp_path):
    out = tmp_path / "tree.stp"
    paths = [
        [[(1, "deck"), (10, "cube_a")]],
        [[(1, "deck"), (11, "cube_b")]],
    ]
    st = cad.stream_ngeom_to_step(_records(blobs, paths=paths), str(out))
    assert st["solids_out"] == 2
    data = out.read_text()
    # ONE shared assembly node 'deck' (with its own empty SHAPE_REPRESENTATION), both leaves
    # hung off it via the FULL occurrence record (NAUO + identity IDT + complex-RR + CDSR) —
    # the pattern OCC's reader needs to find geometry under the shapeless assembly product.
    assert data.count("PRODUCT('deck'") == 1
    assert data.count("SHAPE_REPRESENTATION('deck'") == 1
    assert data.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE") == 2
    assert data.count("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") == 2
    assert data.count("ITEM_DEFINED_TRANSFORMATION") == 2


def test_step_mapped_instances_from_records(blobs, tmp_path):
    out = tmp_path / "mapped.stp"
    b, m = blobs[0]
    st = cad.stream_ngeom_to_step([("box", b, None, [_T_IDENT, _T_SHIFT], None)], str(out))
    assert st["solids_in"] == 1
    assert st["solids_out"] == 1
    assert st["instances_out"] == 2
    data = out.read_text()
    assert data.count("MANIFOLD_SOLID_BREP") == 1, "prototype must be serialised exactly once"
    assert data.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE") == 2
    assert data.count("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") == 2
    assert data.count("SHAPE_REPRESENTATION('model'") == 1
    # read-back: one root, both placements recovered
    roots = [(bytes(bb), mm) for bb, mm in cad.StepNgeomStream(str(out))]
    assert len(roots) == 1
    tr = sorted(tuple(round(t[i], 4) for i in (12, 13, 14)) for t in roots[0][1].transforms)
    assert tr == [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)]


def test_ifc_flat_records(blobs, tmp_path):
    out = tmp_path / "out.ifc"
    st = cad.stream_ngeom_to_ifc(_records(blobs, color=_RED), str(out))
    assert st["solids_in"] == 2
    assert st["solids_out"] == 2
    assert st["instances_out"] == 2
    assert st["faces_dropped"] == 0
    data = out.read_text()
    assert "FILE_SCHEMA(('IFC4X3_ADD2'))" in data
    assert "IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.)" in data
    assert data.count("IFCCLOSEDSHELL") == 2 or data.count("IfcClosedShell") == 2
    assert data.count("IfcBuildingElementProxy") == 2
    assert data.count("IfcStyledItem") == 2
    assert data.count("IfcColourRgb") == 2
    # read-back through the native IFC->NGEOM stream: both products, none skipped
    s = cad.IfcNgeomStream(str(out))
    roots = [(bytes(b), m) for b, m in s]
    assert len(roots) == 2
    assert s.products_skipped == 0


def test_ifc_spatial_tree_and_mapped(blobs, tmp_path):
    out = tmp_path / "tree.ifc"
    b, m = blobs[0]
    st = cad.stream_ngeom_to_ifc(
        [("box", b, None, [_T_IDENT, _T_SHIFT], [[(1, "deck"), (10, "box")], [(1, "deck"), (10, "box")]])],
        str(out),
    )
    assert st["solids_in"] == 1
    assert st["instances_out"] == 2
    data = out.read_text()
    assert data.count("IfcMappedItem") == 2, "instances must share geometry via IfcMappedItem"
    assert data.count("IfcRepresentationMap") == 1
    assert data.count("IFCSPATIALZONE('") >= 1  # 'deck' zone (root zone is in the header block)
    assert "deck" in data


def test_ifc_parses_with_ifcopenshell(blobs, tmp_path):
    ifcopenshell = pytest.importorskip("ifcopenshell")
    out = tmp_path / "check.ifc"
    cad.stream_ngeom_to_ifc(_records(blobs, color=_RED), str(out))
    f = ifcopenshell.open(str(out))
    assert len(f.by_type("IfcBuildingElementProxy")) == 2
    assert len(f.by_type("IfcClosedShell")) == 2


def test_generator_records_stream(blobs, tmp_path):
    """Records may come from a generator (lazy pull) and must match the one-shot list output."""
    out_list = tmp_path / "list.stp"
    out_gen = tmp_path / "gen.stp"
    recs = _records(blobs, color=_RED)
    cad.stream_ngeom_to_step(recs, str(out_list))
    cad.stream_ngeom_to_step((r for r in recs), str(out_gen))
    assert out_gen.read_bytes() == out_list.read_bytes()


def test_unit_scale_header(blobs, tmp_path):
    out_s = tmp_path / "mm.stp"
    out_i = tmp_path / "mm.ifc"
    cad.stream_ngeom_to_step(_records(blobs), str(out_s), unit_scale=0.001)
    cad.stream_ngeom_to_ifc(_records(blobs), str(out_i), unit_scale=0.001)
    assert "SI_UNIT(.MILLI.,.METRE.)" in out_s.read_text()
    assert "IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.)" in out_i.read_text()


_IFC_SHELL_HEADER = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'test','','');
FILE_SCHEMA(('IFC4X3_ADD2'));
ENDSEC;
DATA;
#1=IFCCARTESIANPOINT((0.,0.,0.));
#2=IFCDIRECTION((0.,0.,1.));
#3=IFCDIRECTION((1.,0.,0.));
#4=IFCAXIS2PLACEMENT3D(#1,#2,#3);
#5=IFCDIRECTION((1.,0.));
#6=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#4,#5);
#7=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);
#8=IFCUNITASSIGNMENT((#7));
#9=IFCPROJECT('0aaaaaaaaaaaaaaaaaaaaa',$,'frag test',$,$,$,$,(#6),#8);
#10=IFCAXIS2PLACEMENT3D(#1,$,$);
#11=IFCLOCALPLACEMENT($,#10);
"""


def test_ifc_body_spf_fragment(blobs, tmp_path):
    """The body-fragment binding: geometry-only SPF, contiguous ids from first_id, and the
    returned body item id is the brep a hand-built typed wrapper can reference."""
    b, m = blobs[0]
    spf, next_id, body, rep_type = cad.ngeom_to_ifc_body_spf(b, 100)
    assert body >= 100
    assert rep_type == "AdvancedBrep"
    ids = [int(ln.split("=")[0][1:]) for ln in spf.strip().splitlines()]
    assert ids[0] == 100, "entity numbering must start at first_id"
    assert ids == sorted(ids) and ids[-1] == next_id - 1, "ids must be contiguous from first_id"
    assert body in ids
    # fragment is geometry-only: no header/product/spatial/style entities
    for kw in ("IfcProject", "IfcShapeRepresentation", "IfcBuildingElementProxy", "IfcStyledItem"):
        assert kw not in spf
    assert "IfcAdvancedBrep" in spf

    # splice into a minimal hand-built SPF shell with a typed wrapper -> ifcopenshell parses it
    ifcopenshell = pytest.importorskip("ifcopenshell")
    wrap = (
        f"#{next_id}=IFCSHAPEREPRESENTATION(#6,'Body','{rep_type}',(#{body}));\n"
        f"#{next_id + 1}=IFCPRODUCTDEFINITIONSHAPE($,$,(#{next_id}));\n"
        f"#{next_id + 2}=IFCPLATE('1aaaaaaaaaaaaaaaaaaaaa',$,'frag_plate',$,$,#11,#{next_id + 1},$,$);\n"
    )
    out = tmp_path / "frag.ifc"
    out.write_text(_IFC_SHELL_HEADER + spf + wrap + "ENDSEC;\nEND-ISO-10303-21;\n")
    f = ifcopenshell.open(str(out))
    (plate,) = f.by_type("IfcPlate")
    (rep,) = plate.Representation.Representations
    assert rep.RepresentationType == "AdvancedBrep"
    assert rep.Items[0].id() == body, "wrapper must reference the returned brep item id"
    assert rep.Items[0].is_a("IfcAdvancedBrep")
    assert len(rep.Items[0].Outer.CfsFaces) == 6


def test_ifc_body_spf_single_root_contract(tmp_path):
    """A multi-root blob is rejected (the adapy writer serializes one solid per blob)."""
    p = tmp_path / "cubes.stp"
    p.write_text(_two_cube_step())
    # stream_step_to_ngeom yields ONE buffer holding both roots
    buf, metas = cad.stream_step_to_ngeom(str(p))
    assert len(metas) == 2
    with pytest.raises(RuntimeError, match="exactly one root"):
        cad.ngeom_to_ifc_body_spf(bytes(buf), 100)


def test_short_records_and_name_override(blobs, tmp_path):
    """2-tuples work; a non-empty record name overrides the blob's root id."""
    out = tmp_path / "short.stp"
    b, _m = blobs[0]
    st = cad.stream_ngeom_to_step([("renamed_cube", b)], str(out))
    assert st["solids_out"] == 1
    roots = [(bytes(bb), mm) for bb, mm in cad.StepNgeomStream(str(out))]
    assert roots[0][1].id == "renamed_cube"
