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


# ── the swept area: outline + the plane it is swept from ─────────────────────────────────────


def test_a_plate_reports_the_outline_a_plate_actually_is(members):
    # A plate is an outline, a thickness and a plane. The first two were already here; without the
    # third and the boundary itself a consumer can name a plate but cannot rebuild one -- there is
    # no section name that means "this 2 x 2 m rectangle".
    pl = members["pl1"]
    assert [tuple(round(v, 6) for v in p) for p in pl["outline"]] == [
        (0.0, -2.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, -2.0),
    ]
    assert pl["origin"] == pytest.approx((0.0, 0.0, 0.0))
    assert pl["normal"] == pytest.approx((1.0, 0.0, 0.0))  # the extrusion direction
    assert pl["xdir"] == pytest.approx((0.0, 0.0, 1.0))  # where the outline's +x points


def test_a_catalogue_section_is_synthesised_into_the_same_shape(members):
    # An IfcIShapeProfileDef states parameters, not points -- the reader turns them into the same
    # outline the geometry path draws, so a consumer handles one kind of answer rather than two.
    outline = members["bm1"]["outline"]
    assert len(outline) == 12  # an I: two flanges, a web, twelve corners
    xs = [p[0] for p in outline]
    ys = [p[1] for p in outline]
    assert max(xs) - min(xs) == pytest.approx(0.11)  # IPE220 depth
    assert max(ys) - min(ys) == pytest.approx(0.22)  # its width


def test_the_swept_plane_follows_the_member(members):
    # bm2 runs along +Y and bm3 along +Z: the normal is the extrusion direction, so it is how a
    # consumer knows which way a plate faces or a beam runs without re-deriving it from the axis.
    assert members["bm2"]["normal"] == pytest.approx((0.0, 1.0, 0.0))
    assert members["bm3"]["normal"] == pytest.approx((0.0, 0.0, 1.0))
    diagonal = members["bm4"]["normal"]
    assert diagonal == pytest.approx((0.7071067811865476, 0.0, 0.7071067811865476))


def test_outline_points_are_metres_like_everything_else(members):
    # The box girder's outer boundary is 0.3 x 0.2 m; a file in millimetres would report the same
    # numbers, which is the point of normalising once in the reader.
    outline = members["MyBeam"]["outline"]
    xs = [p[0] for p in outline]
    ys = [p[1] for p in outline]
    assert max(xs) - min(xs) == pytest.approx(0.2)
    assert max(ys) - min(ys) == pytest.approx(0.3)


# ── material: the other half of a quantity ───────────────────────────────────────────────────


def test_each_member_reports_the_material_it_is_associated_with(members):
    # A member's MASS is its section times its length times a density, so a consumer computing
    # quantities needs this as much as it needs the section. Per MEMBER, not per file: the plate
    # here is S420 while the beams are S355, and a reader that reported one material for the model
    # would be wrong about most of it.
    assert members["pl1"]["material"] == "S420"
    assert {members[n]["material"] for n in ("bm1", "bm2", "MyBeam")} == {"S355"}


def test_the_stated_properties_come_through_with_their_values(members):
    props = members["bm1"]["material_props"]
    assert props["MassDensity"] == pytest.approx(7850.0)
    assert props["YoungModulus"] == pytest.approx(210e9)
    assert props["PoissonRatio"] == pytest.approx(0.3)
    assert props["YieldStress"] == pytest.approx(355e6)


def test_a_typed_value_is_read_as_its_value_not_its_type(members):
    # IFCPRESSUREMEASURE(355000000.) is not ONE argument -- Part-21 parses the keyword and the
    # list as two adjacent ones. Reading NominalValue as args[2] hands back the type name and
    # never a number, which is exactly how this came out empty on a file stating seven properties
    # per material.
    for value in members["bm1"]["material_props"].values():
        assert not (isinstance(value, str) and value.upper().startswith("IFC"))


def test_text_properties_are_kept_rather_than_dropped(members):
    # Which NAME carries the grade varies by exporter -- this codebase writes "Grade", others
    # write "StrengthGrade" -- so the reader reports what the file says and lets the consumer map
    # the names it knows.
    assert members["bm1"]["material_props"]["Grade"] == "S355"
    assert members["pl1"]["material_props"]["Grade"] == "S420"


# ── joints: the same compiled pass the browser runs ──────────────────────────────────────────


def test_beams_meeting_at_one_node_are_one_joint():
    # Per CONTACT POINT, not per pair. Three beams at a node are ONE joint with three members,
    # which is what a person sees and what a connection spec is written against; a pairwise
    # reading would report three joints and detail the same place three times.
    ms = [
        ("g0", "", (0, 0, 0), (5, 0, 0), 0.1, "I"),
        ("g1", "", (5, 0, 0), (5, 5, 0), 0.1, "I"),
        ("c0", "", (5, 0, 0), (5, 0, 3), 0.1, "I"),
    ]
    joints = adacpp.cad.find_beam_joints(ms)
    assert len(joints) == 1
    assert sorted(joints[0]["members"]) == [0, 1, 2]
    assert joints[0]["centre"] == pytest.approx((5.0, 0.0, 0.0))
    assert joints[0]["type_key"] == "3|BEAM:I:COLUMN+BEAM:I:GIRDER+BEAM:I:GIRDER|perpendicular"


def test_a_mid_span_crossing_is_a_joint():
    # Neither beam's END is near the other; only a box-against-box candidate test finds this pair
    # at all, and it is as real a joint as one at a shared node.
    ms = [
        ("a", "", (-5, 0, 0), (5, 0, 0), 0.1, "I"),
        ("b", "", (0, -5, 0), (0, 5, 0), 0.1, "I"),
    ]
    assert len(adacpp.cad.find_beam_joints(ms)) == 1


def test_one_contact_is_one_joint_when_the_members_only_nearly_meet():
    # The closest point differs per member for a near miss, by up to the out-of-plane tolerance --
    # far beyond point_tol, so taking one side's point would register the contact twice.
    ms = [
        ("a", "", (0, 0, 0), (5, 0, 0), 0.1, "I"),
        ("b", "", (2.5, -1, 0.05), (2.5, 1, 0.05), 0.1, "I"),
    ]
    joints = adacpp.cad.find_beam_joints(ms)
    assert len(joints) == 1
    assert joints[0]["centre"] == pytest.approx((2.5, 0.0, 0.025))


def test_parallel_members_never_join():
    ms = [
        ("a", "", (0, 0, 0), (5, 0, 0), 0.1, "I"),
        ("b", "", (0, 1, 0), (5, 1, 0), 0.1, "I"),
    ]
    assert adacpp.cad.find_beam_joints(ms) == []


def test_a_crossing_far_past_a_member_end_is_not_its_joint():
    # Two non-parallel lines always meet somewhere. Past half a member's own length that meeting
    # is not this member's joint, or every beam in a frame would join every other.
    ms = [
        ("a", "", (0, 0, 0), (1, 0, 0), 0.1, "I"),
        ("b", "", (8, -1, 0), (8, 1, 0), 0.1, "I"),
    ]
    assert adacpp.cad.find_beam_joints(ms) == []


def test_the_out_of_plane_tolerance_decides_a_near_miss():
    def at(dz):
        return [
            ("a", "", (0, 0, 0), (5, 0, 0), 0.1, "I"),
            ("b", "", (2.5, -1, dz), (2.5, 1, dz), 0.1, "I"),
        ]

    assert len(adacpp.cad.find_beam_joints(at(0.05), out_of_plane_tol=0.1)) == 1
    assert adacpp.cad.find_beam_joints(at(0.5), out_of_plane_tol=0.1) == []


def test_member_type_comes_from_the_axis():
    # Column / Girder / Brace, as adapy derives them, because a type key names them.
    vertical = [("c", "", (0, 0, 0), (0, 0, 3), 0.1, "I"), ("g", "", (0, 0, 3), (3, 0, 3), 0.1, "I")]
    key = adacpp.cad.find_beam_joints(vertical)[0]["type_key"]
    assert "COLUMN" in key and "GIRDER" in key
