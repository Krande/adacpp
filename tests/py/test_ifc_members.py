"""`IfcMemberScan`: what an IFC says its products ARE, without tessellating any of them.

The geometry reader answers "what does this look like"; a clash check, a quantity take-off and a
tree walk all ask "what is it" -- a beam of this section running from here to there, a plate of
this thickness -- and none of them needs a triangle. These pin the facts that answer sits on:
the member's class, its reference axis in WORLD METRES, its profile, and its depth.

my_test.ifc: one IFCPLATE + six IFCBEAMs, written by adapy's own IFC writer, so it carries both
an `Axis` representation (a two-point polyline) and a `Body` extrusion per beam.
"""

import math

import pytest

import adacpp.cad

IFC = "files/my_test.ifc"


@pytest.fixture(scope="module")
def members():
    scan = adacpp.cad.IfcMemberScan(IFC)
    assert scan.products_total == 7
    assert scan.unit_scale == 1.0  # the file is in metres
    return {m["name"]: m for m in scan}


def test_every_product_is_reported_with_its_class(members):
    assert set(members) == {"pl1", "MyBeam", "bm1", "bm2", "bm3", "bm4", "bm5"}
    assert members["pl1"]["ifc_class"] == "IFCPLATE"
    assert {members[n]["ifc_class"] for n in ("bm1", "bm2", "bm3")} == {"IFCBEAM"}


def test_each_member_carries_its_guid(members):
    # The guid is how a consumer addresses the same product in the geometry stream, the GLB and
    # the file itself -- a name is not unique and may be empty.
    assert members["pl1"]["guid"] == "1cQMCXLayHxgKdv5utukF4"
    assert all(m["guid"] for m in members.values())


def test_a_beam_reports_its_profile(members):
    assert members["bm1"]["profile_name"] == "IPE220"
    assert members["bm1"]["profile_type"] == "IFCISHAPEPROFILEDEF"
    # A box girder is written as an arbitrary profile WITH VOIDS; the type is what a consumer
    # matches a section family on, so it is reported as written rather than normalised here.
    assert members["MyBeam"]["profile_name"] == "BG300x200x10x20"
    assert members["MyBeam"]["profile_type"] == "IFCARBITRARYPROFILEDEFWITHVOIDS"


def test_the_axis_is_the_reference_line_in_world_metres(members):
    # bm1 runs along +X, bm2 along +Y, bm3 along +Z -- from the Axis representation, placed by the
    # product's ObjectPlacement chain, not from the extrusion.
    assert members["bm1"]["p1"] == pytest.approx((0.0, 0.0, 0.0))
    assert members["bm1"]["p2"] == pytest.approx((2.0, 0.0, 0.0))
    assert members["bm2"]["p2"] == pytest.approx((0.0, 2.0, 0.0))
    assert members["bm3"]["p2"] == pytest.approx((0.0, 0.0, 2.0))


def test_a_member_that_does_not_start_at_the_origin_is_placed(members):
    # bm5 spans the top of the frame: a member whose placement is ignored collapses to the origin,
    # which is the failure this pins.
    assert members["bm5"]["p1"] == pytest.approx((0.0, 0.0, 2.0))
    assert members["bm5"]["p2"] == pytest.approx((2.0, 0.0, 2.0))


def test_depth_is_the_extrusion_length(members):
    assert members["bm1"]["depth"] == pytest.approx(2.0)
    # The diagonal: its depth is the length of the member, not its projection.
    assert members["bm4"]["depth"] == pytest.approx(math.sqrt(8.0), rel=1e-6)


def test_a_plate_reports_its_thickness_as_depth(members):
    pl = members["pl1"]
    assert pl["depth"] == pytest.approx(0.01)
    # A plate's outline is an unnamed arbitrary profile -- it has no catalogue section, and saying
    # so honestly beats inventing a name.
    assert pl["profile_type"] == "IFCARBITRARYCLOSEDPROFILEDEF"
    assert pl["profile_name"] == ""


def test_the_placement_is_a_column_major_world_matrix(members):
    pl = members["pl1"]["placement"]
    assert len(pl) == 16
    assert pl[15] == pytest.approx(1.0)  # homogeneous row, so a consumer can multiply it directly


def test_a_scan_is_repeatable(members):
    # Nothing in the FILE is consumed by reading: two scans of one file agree, which is what lets a
    # caller ask for members and geometry independently.
    again = {m["name"]: m for m in adacpp.cad.IfcMemberScan(IFC)}
    assert again["bm1"]["p2"] == pytest.approx(members["bm1"]["p2"])
    assert again.keys() == members.keys()


def test_it_yields_one_product_at_a_time(members):
    # A scan is a STREAM, like the geometry one: the consumer decides what to keep, so a plant-sized
    # file never has to exist as one list of members on either side of the boundary.
    scan = adacpp.cad.IfcMemberScan(IFC)
    assert iter(scan) is scan
    first = next(scan)
    second = next(scan)
    assert first["id"] != second["id"]
    # And it ENDS -- a stream that never raised StopIteration would hang every `for` over it.
    remaining = list(scan)
    assert len(remaining) == 5
    with pytest.raises(StopIteration):
        next(scan)
