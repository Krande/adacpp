"""The IFC readers' length unit: the one the project assigns, SI or conversion based.

`unit_scale` is metres per file unit, and every native IFC consumer (the NGEOM stream, the member
scan, IFC -> GLB) scales by it. It used to come from the first IfcSIUnit(.LENGTHUNIT., .METRE.) in
the file, which reads an inch file (an IfcConversionBasedUnit) as metres, and a file that states a
metre unit before the millimetre unit it assigns as metres too. Fixtures are written here.
"""

from __future__ import annotations

import pytest

from adacpp import cad

_HEAD = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#1=IFCCARTESIANPOINT((0.,0.,0.));
#2=IFCDIRECTION((0.,0.,1.));
#3=IFCDIRECTION((1.,0.,0.));
#4=IFCAXIS2PLACEMENT3D(#1,#2,#3);
#6=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#4,$);
"""

_BODY = """#10=IFCLOCALPLACEMENT($,#4);
#20=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,10.,20.);
#21=IFCEXTRUDEDAREASOLID(#20,#4,#2,100.);
#22=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#21));
#23=IFCPRODUCTDEFINITIONSHAPE($,$,(#22));
#24=IFCBUILDINGELEMENTPROXY('0000000000000000000X01',$,'block',$,$,#10,#23,$,$);
ENDSEC;
END-ISO-10303-21;
"""

UNITS = {
    # An inch: 0.0254 of the SI metre.
    "inch": (
        """#7=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);
#8=IFCDIMENSIONALEXPONENTS(1,0,0,0,0,0,0);
#9=IFCMEASUREWITHUNIT(IFCLENGTHMEASURE(0.0254),#7);
#11=IFCCONVERSIONBASEDUNIT(#8,.LENGTHUNIT.,'INCH',#9);
#12=IFCUNITASSIGNMENT((#11));
#13=IFCPROJECT('0000000000000000000001',$,'p',$,$,$,$,(#6),#12);
""",
        0.0254,
    ),
    # A foot stated in millimetres.
    "foot of millimetres": (
        """#7=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#8=IFCDIMENSIONALEXPONENTS(1,0,0,0,0,0,0);
#9=IFCMEASUREWITHUNIT(IFCLENGTHMEASURE(304.8),#7);
#11=IFCCONVERSIONBASEDUNIT(#8,.LENGTHUNIT.,'FOOT',#9);
#12=IFCUNITASSIGNMENT((#11));
#13=IFCPROJECT('0000000000000000000001',$,'p',$,$,$,$,(#6),#12);
""",
        0.3048,
    ),
    # A metre unit first, but the project assigns millimetres.
    "assigned millimetres": (
        """#7=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);
#8=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#12=IFCUNITASSIGNMENT((#8));
#13=IFCPROJECT('0000000000000000000001',$,'p',$,$,$,$,(#6),#12);
""",
        1e-3,
    ),
    # No assignment at all: the first SI length unit, as before.
    "unassigned millimetres": (
        """#7=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#13=IFCPROJECT('0000000000000000000001',$,'p',$,$,$,$,(#6),$);
""",
        1e-3,
    ),
}


@pytest.mark.parametrize("case", list(UNITS))
def test_unit_scale_is_the_assigned_length_unit(tmp_path, case):
    units, scale = UNITS[case]
    path = tmp_path / "unit.ifc"
    path.write_text(_HEAD + units + _BODY)
    stream = cad.IfcNgeomStream(str(path))
    assert stream.unit_scale == pytest.approx(scale)
    assert len(list(stream)) == 1
    assert cad.IfcMemberScan(str(path)).unit_scale == pytest.approx(scale)


def test_member_scan_reports_description_object_type_and_tag(tmp_path):
    units, _ = UNITS["assigned millimetres"]
    body = _BODY.replace(
        "IFCBUILDINGELEMENTPROXY('0000000000000000000X01',$,'block',$,$,#10,#23,$,$)",
        "IFCBUILDINGELEMENTPROXY('0000000000000000000X01',$,'block','a block\\X\\27s side',"
        "'spacer',#10,#23,'T-1',$)",
    )
    path = tmp_path / "attrs.ifc"
    path.write_text(_HEAD + units + body)
    (rec,) = [r for r in cad.IfcMemberScan(str(path)) if r["guid"] == "0000000000000000000X01"]
    assert (rec["description"], rec["object_type"], rec["tag"]) == ("a block's side", "spacer", "T-1")


def test_member_scan_attributes_are_empty_when_unset(tmp_path):
    units, _ = UNITS["assigned millimetres"]
    path = tmp_path / "plain.ifc"
    path.write_text(_HEAD + units + _BODY)
    (rec,) = list(cad.IfcMemberScan(str(path)))
    assert (rec["description"], rec["object_type"], rec["tag"]) == ("", "", "")


def test_a_scaled_instance_transform_is_written_as_the_operator_scale(tmp_path):
    """A reader that folds a millimetre file's unit into the world matrix (so the product sits in
    metres) hands the writer a matrix with a uniform scale. The writer keeps it as the mapped
    item's Scale; normalised axes alone would write the product a thousand times too large."""
    import numpy as np

    units, _ = UNITS["assigned millimetres"]
    src = tmp_path / "mm.ifc"
    src.write_text(_HEAD + units + _BODY)
    ((blob, _meta),) = list(cad.IfcNgeomStream(str(src)))
    scale = np.diag([1e-3, 1e-3, 1e-3, 1.0])
    scale[:3, 3] = (1.0, 2.0, 3.0)
    tf = [float(v) for v in scale.flatten(order="F")]
    dst = tmp_path / "scaled.ifc"
    cad.stream_ngeom_to_ifc([("block", bytes(blob), None, [tf], None)], str(dst), unit_scale=1.0)
    text = dst.read_text()
    assert "IFCCARTESIANTRANSFORMATIONOPERATOR3D(" in text.upper()
    ((_, back),) = list(cad.IfcNgeomStream(str(dst)))
    m = np.asarray(back.transforms[0], dtype=float).reshape(4, 4, order="F")
    assert np.linalg.norm(m[:3, 0]) == pytest.approx(1e-3, rel=1e-5)
    assert m[:3, 3] == pytest.approx([1.0, 2.0, 3.0], abs=1e-6)
