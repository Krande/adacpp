"""`IfcMemberScan`: the facts a member's SECTION and CUTS are rebuilt from, not only its axis.

A consumer that rebuilds a beam from the scan -- rather than drawing it -- needs what a section
reader needs: the profile's curves with their arcs (a rounded corner is part of the section, a
sampled one is a polygon), the parameter set a writer attached to the profile, the half-spaces the
body is clipped by, and the product's own property sets. The fixture is written here, in
millimetres, so every length below also checks the unit scale.
"""

from __future__ import annotations

import json

import pytest

from adacpp import cad

# A clipped hollow beam with rounded corners, a plate, and a beam whose body is a UNION (which is not
# a chain of cuts, so it has no member body to report). Lengths in mm.
SPF = """ISO-10303-21;
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
#7=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#8=IFCUNITASSIGNMENT((#7));
#9=IFCPROJECT('0000000000000000000001',$,'p',$,$,$,$,(#6),#8);
#10=IFCCARTESIANPOINT((1000.25,2000.5,-3000.125));
#11=IFCAXIS2PLACEMENT3D(#10,$,$);
#12=IFCLOCALPLACEMENT($,#11);
#20=IFCCARTESIANPOINTLIST2D(((-40.,-30.),(-37.,-37.),(-30.,-40.),(30.,-40.),(37.,-37.),(40.,-30.),(40.,30.),(37.,37.),(30.,40.),(-30.,40.),(-37.,37.),(-40.,30.)));
#21=IFCINDEXEDPOLYCURVE(#20,(IFCARCINDEX((1,2,3)),IFCLINEINDEX((3,4)),IFCARCINDEX((4,5,6)),IFCLINEINDEX((6,7)),IFCARCINDEX((7,8,9)),IFCLINEINDEX((9,10)),IFCARCINDEX((10,11,12)),IFCLINEINDEX((12,1))),.F.);
#22=IFCCARTESIANPOINTLIST2D(((-34.,-34.),(34.,-34.),(34.,34.),(-34.,34.)));
#23=IFCINDEXEDPOLYCURVE(#22,(IFCLINEINDEX((1,2,3,4,1))),.F.);
#24=IFCARBITRARYPROFILEDEFWITHVOIDS(.AREA.,'RHS80',#21,(#23));
#25=IFCPROPERTYSINGLEVALUE('sec_type',$,IFCTEXT('poly'),$);
#26=IFCPROPERTYSINGLEVALUE('t_w',$,IFCREAL(6.),$);
#27=IFCPROFILEPROPERTIES('SectionBag',$,(#25,#26),#24);
#30=IFCCARTESIANPOINT((0.,0.,0.));
#31=IFCDIRECTION((0.,0.,1.));
#32=IFCDIRECTION((1.,0.,0.));
#33=IFCAXIS2PLACEMENT3D(#30,#31,#32);
#34=IFCDIRECTION((0.,0.,1.));
#35=IFCEXTRUDEDAREASOLID(#24,#33,#34,2500.);
#40=IFCCARTESIANPOINT((0.,0.,100.));
#41=IFCDIRECTION((0.,0.,-1.));
#42=IFCDIRECTION((1.,0.,0.));
#43=IFCAXIS2PLACEMENT3D(#40,#41,#42);
#44=IFCPLANE(#43);
#45=IFCHALFSPACESOLID(#44,.T.);
#46=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#35,#45);
#50=IFCCARTESIANPOINT((0.,0.,2400.));
#51=IFCDIRECTION((0.,0.7071067811865476,0.7071067811865476));
#52=IFCAXIS2PLACEMENT3D(#50,#51,$);
#53=IFCPLANE(#52);
#54=IFCHALFSPACESOLID(#53,.F.);
#55=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#46,#54);
#56=IFCSHAPEREPRESENTATION(#6,'Body','Clipping',(#55));
#57=IFCPRODUCTDEFINITIONSHAPE($,$,(#56));
#58=IFCBEAM('0000000000000000000B01',$,'cut beam',$,$,#12,#57,$,$);
#60=IFCPROPERTYSINGLEVALUE('Ref',$,IFCLABEL('A/1''2'),$);
#61=IFCPROPERTYSINGLEVALUE('Flag',$,IFCBOOLEAN(.T.),$);
#62=IFCPROPERTYSINGLEVALUE('Count',$,IFCINTEGER(3),$);
#63=IFCPROPERTYSINGLEVALUE('Ratio',$,IFCREAL(0.25),$);
#64=IFCPROPERTYLISTVALUE('Tags',$,(IFCLABEL('x'),IFCLABEL('y')),$);
#65=IFCPROPERTYSINGLEVALUE('Empty',$,$,$);
#66=IFCPROPERTYSET('0000000000000000000P01',$,'Properties',$,(#60,#61,#62,#63,#64,#65));
#67=IFCRELDEFINESBYPROPERTIES('0000000000000000000R01',$,$,$,(#58),#66);
#70=IFCCARTESIANPOINTLIST2D(((0.,0.),(400.,0.),(400.,300.),(0.,300.)));
#71=IFCPOLYLINE((#72,#73,#74,#72));
#72=IFCCARTESIANPOINT((0.,0.));
#73=IFCCARTESIANPOINT((400.,0.));
#74=IFCCARTESIANPOINT((0.,300.));
#75=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#71);
#76=IFCEXTRUDEDAREASOLID(#75,#33,#34,10.);
#77=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#76));
#78=IFCPRODUCTDEFINITIONSHAPE($,$,(#77));
#79=IFCPLATE('0000000000000000000L01',$,'plate',$,$,#12,#78,$,$);
#80=IFCCIRCLEHOLLOWPROFILEDEF(.AREA.,'CHS',$,50.,5.);
#81=IFCEXTRUDEDAREASOLID(#80,#33,#34,1000.);
#82=IFCBLOCK(#33,10.,10.,10.);
#83=IFCBOOLEANRESULT(.UNION.,#81,#82);
#84=IFCSHAPEREPRESENTATION(#6,'Body','CSG',(#83));
#85=IFCPRODUCTDEFINITIONSHAPE($,$,(#84));
#86=IFCBEAM('0000000000000000000B02',$,'union beam',$,$,#12,#85,$,$);
#87=IFCEXTRUDEDAREASOLID(#80,#33,#34,1000.);
#88=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#87));
#89=IFCPRODUCTDEFINITIONSHAPE($,$,(#88));
#90=IFCMEMBER('0000000000000000000B03',$,'tube',$,$,#12,#89,$,$);
ENDSEC;
END-ISO-10303-21;
"""


@pytest.fixture(scope="module")
def ifc_path(tmp_path_factory):
    p = tmp_path_factory.mktemp("structure") / "members.ifc"
    p.write_text(SPF, encoding="utf-8")
    return p


@pytest.fixture(scope="module")
def members(ifc_path):
    return {m["name"]: m for m in cad.IfcMemberScan(str(ifc_path))}


def test_a_clipped_beam_is_read_from_the_solid_under_its_cuts(members):
    # Before, a body that was a clipping result had no extrusion to read and the member came back
    # with no profile and no depth -- which a reader then dropped.
    bm = members["cut beam"]
    assert bm["profile_type"] == "IFCARBITRARYPROFILEDEFWITHVOIDS"
    assert bm["profile_name"] == "RHS80"
    assert bm["depth"] == pytest.approx(2.5)


def test_the_cuts_come_innermost_first_with_their_planes_and_flags(members):
    cuts = members["cut beam"]["cuts"]
    assert len(cuts) == 2
    first, second = cuts
    # #46 (the inner clip) was applied first.
    assert first["origin"] == pytest.approx((0.0, 0.0, 0.1))  # metres
    assert first["normal"] == pytest.approx((0.0, 0.0, -1.0))
    assert first["agreement"] is True
    assert second["origin"] == pytest.approx((0.0, 0.0, 2.4))
    assert second["normal"] == pytest.approx((0.0, 0.7071067811865476, 0.7071067811865476))
    assert second["agreement"] is False


def test_a_boolean_that_is_not_a_chain_of_cuts_is_not_read_as_a_member_body(members):
    # A union with a block is a different solid; reading its first operand as the member would
    # report a wrong member rather than a missing one.
    u = members["union beam"]
    assert u["depth"] == 0.0
    assert u["cuts"] == []
    assert u["outer_curve"] is None


def test_the_outer_curve_keeps_its_arcs(members):
    oc = members["cut beam"]["outer_curve"]
    assert oc["kind"] == "indexed"
    assert oc["dim"] == 2
    assert len(oc["points"]) == 12
    assert oc["points"][0] == pytest.approx((-0.04, -0.03))
    assert oc["segments"][0] == ("arc", (1, 2, 3))
    assert oc["segments"][1] == ("line", (3, 4))
    assert sum(1 for kind, _ in oc["segments"] if kind == "arc") == 4


def test_the_voids_are_reported_too(members):
    inner = members["cut beam"]["inner_curves"]
    assert len(inner) == 1
    assert inner[0]["segments"] == [("line", (1, 2, 3, 4, 1))]
    assert inner[0]["points"][2] == pytest.approx((0.034, 0.034))


def test_a_polyline_profile_is_reported_as_its_points(members):
    oc = members["plate"]["outer_curve"]
    assert oc["kind"] == "polyline"
    assert oc["segments"] is None
    assert len(oc["points"]) == 4  # the closing point is part of the polyline as written
    assert oc["points"][1] == pytest.approx((0.4, 0.0))


def test_the_profile_property_bag_is_reported_raw(members):
    bag = members["cut beam"]["profile_properties"]
    assert bag == {"SectionBag": {"sec_type": "poly", "t_w": 6.0}}  # file units: the bag is not geometry


def test_a_parametric_profile_reports_its_lengths(members):
    pp = members["tube"]["profile_params"]
    assert pp == pytest.approx({"Radius": 0.05, "WallThickness": 0.005})


def test_property_sets_keep_their_value_types(members):
    psets = members["cut beam"]["psets"]
    assert psets == {
        "Properties": {
            "Ref": "A/1'2",  # the SPF '' escape undone
            "Flag": True,
            "Count": 3,
            "Ratio": 0.25,
            "Tags": ["x", "y"],
            "Empty": None,
        }
    }
    assert isinstance(psets["Properties"]["Count"], int)
    assert members["plate"]["psets"] == {}


def test_the_double_placement_is_not_rounded_to_float(members):
    pd = members["cut beam"]["placement_d"]
    assert len(pd) == 16
    # File units (mm), like the float matrix; 1000.25 / 2000.5 / -3000.125 survive exactly.
    assert pd[12:15] == [1000.25, 2000.5, -3000.125]
    assert members["cut beam"]["unit_scale"] == pytest.approx(1e-3)
    assert members["cut beam"]["extrude_dir"] == pytest.approx((0.0, 0.0, 1.0))


def test_the_jsonl_scan_carries_the_same_structure(ifc_path, tmp_path):
    out = tmp_path / "m.jsonl"
    cad.scan_ifc_members_to_jsonl(str(ifc_path), str(out))
    lines = out.read_text(encoding="utf-8").splitlines()
    recs = {r["name"]: r for r in (json.loads(x) for x in lines[1:])}
    bm = recs["cut beam"]
    assert bm["psets"]["Properties"]["Ref"] == "A/1'2"
    assert bm["psets"]["Properties"]["Count"] == 3
    assert bm["profile_properties"] == {"SectionBag": {"sec_type": "poly", "t_w": 6.0}}
    assert [c["agreement"] for c in bm["cuts"]] == [True, False]
    assert bm["outer_curve"]["segments"][0] == ["arc", [1, 2, 3]]
    assert recs["plate"]["outer_curve"]["segments"] is None
    assert recs["union beam"]["outer_curve"] is None
    assert recs["tube"]["profile_params"] == pytest.approx({"Radius": 0.05, "WallThickness": 0.005})
