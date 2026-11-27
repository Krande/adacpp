"""stream_ngeom_to_ifc writes a record's MEMBER description as the structural member it describes.

A product written from its solid alone is an IfcBuildingElementProxy: a reader can draw it and can
not rebuild it. A record carrying a member description is written as the member -- class, axis,
profile with its arcs and property sets, half-space cuts, material, property sets -- and these
check that the member scan reads back exactly what was written (and that ifcopenshell accepts the
file, where it is installed).
"""

from __future__ import annotations

import math

import pytest

from adacpp import cad

_IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
_S = math.sqrt(0.5)


def _rounded_square(half: float, r: float) -> dict:
    c, m = half - r, r * (1 - _S)
    pts = [
        (-half, -c), (-half + m, -half + m), (-c, -half), (c, -half), (half - m, -half + m), (half, -c),
        (half, c), (half - m, half - m), (c, half), (-c, half), (-half + m, half - m), (-half, c),
    ]  # fmt: skip
    segs = [
        ("arc", (1, 2, 3)), ("line", (3, 4)), ("arc", (4, 5, 6)), ("line", (6, 7)),
        ("arc", (7, 8, 9)), ("line", (9, 10)), ("arc", (10, 11, 12)), ("line", (12, 1)),
    ]  # fmt: skip
    return {"kind": "indexed", "points": pts, "segments": segs}


def _beam(**overrides) -> dict:
    placement = list(_IDENTITY)
    placement[12:15] = [1000.123456789012, -2.5, 0.1]
    m = {
        "ifc_class": "IfcBeam",
        "guid": "0000000000000000000B01",
        "name": "beam Æ/1",  # non-ASCII survives the SPF encoding
        "description": "RHS100x6",
        "object_type": "Beam",
        "placement": placement,
        "axis": ((0.0, 0.0, 0.0), (0.0, 0.0, 2.0)),
        "profile": {
            "kind": "curves",
            "name": "RHS100",
            "outer": _rounded_square(0.05, 0.01),
            "inners": [_rounded_square(0.044, 0.004)],
            "properties": {"SectionBag": {"sec_type": "poly", "t_w": 0.006}},
        },
        "extrusion": {
            "origin": (0.0, 0.0, 0.0),
            "axis": (0.0, 0.0, 1.0),
            "ref": (1.0, 0.0, 0.0),
            "direction": (0.0, 0.0, 1.0),
            "depth": 2.0,
        },
        "cuts": [
            {"origin": (0.0, 0.0, 0.1), "normal": (0.0, 0.0, -1.0), "xdir": (1.0, 0.0, 0.0), "agreement": True},
            {"origin": (0.0, 0.0, 1.9), "normal": (0.0, 0.6, 0.8), "agreement": False},
        ],
        "material": {
            "name": "S355",
            "category": "Steel",
            "properties_name": "MaterialMechanical",
            "properties": [
                ("Grade", "S355", "IfcText"),
                ("YieldStress", 355e6, "IfcPressureMeasure"),
                ("MassDensity", 7850.0, "IfcMassDensityMeasure"),
            ],
        },
        "psets": {"Properties": {"Ref": "R-1/2'3", "imported": True, "n": 3, "x": 0.1, "tags": ["a", "b"]}},
        "color": None,
    }
    m.update(overrides)
    return m


def _plate(**overrides) -> dict:
    m = _beam(
        ifc_class="IfcPlate",
        guid="0000000000000000000L01",
        name="plate",
        description=None,
        object_type=None,
        axis=None,
        placement=_IDENTITY,
        profile={
            "kind": "curves",
            "name": None,
            "outer": {"kind": "polyline", "points": [(0, 0), (0.4, 0), (0.4, 0.3), (0, 0.3), (0, 0)]},
            "inners": [],
        },
        extrusion={"origin": (1, 2, 3), "axis": (0, 0, 1), "ref": (1, 0, 0), "direction": (0, 0, 1), "depth": 0.01},
        cuts=[],
        psets={"Properties": {"Ref": "R-1/9"}},
    )
    m.update(overrides)
    return m


@pytest.fixture(scope="module")
def written(tmp_path_factory):
    path = tmp_path_factory.mktemp("member_emit") / "members.ifc"
    tube = _beam(
        guid="0000000000000000000B02",
        name="tube",
        cuts=[],
        psets={},
        profile={"kind": "circle_hollow", "name": "CHS", "radius": 0.05, "wall_thickness": 0.005},
    )
    not_a_member = _beam(guid="0000000000000000000B03", name="no depth")
    not_a_member["extrusion"] = dict(not_a_member["extrusion"], depth=0.0)
    records = [
        ("beam", b"", None, None, None, _beam()),
        ("plate", b"", [0.2, 0.4, 0.6, 1.0], None, None, _plate()),
        ("tube", None, None, None, None, tube),
        ("no depth", b"", None, None, None, not_a_member),
    ]
    stats = cad.stream_ngeom_to_ifc(records, str(path), unit_scale=1.0)
    return path, stats, {m["name"]: m for m in cad.IfcMemberScan(str(path))}


def test_the_writer_says_it_reads_member_descriptions():
    assert cad.ifc_member_records_version() >= 1


def test_every_writable_member_is_counted_and_the_rest_reported(written):
    _, stats, members = written
    assert stats["solids_in"] == 4
    assert stats["solids_out"] == 3
    assert stats["solids_skipped"] == 1  # no depth and no blob: reported, never silent
    assert set(members) == {"beam Æ/1", "plate", "tube"}


def test_a_beam_is_written_as_an_ifc_beam_with_its_axis(written):
    _, _, members = written
    b = members["beam Æ/1"]
    assert b["ifc_class"] == "IFCBEAM"
    assert b["guid"] == "0000000000000000000B01"
    assert b["p1"] == pytest.approx((1000.123456789012, -2.5, 0.1))
    assert b["p2"] == pytest.approx((1000.123456789012, -2.5, 2.1))
    assert b["depth"] == 2.0


def test_reals_round_trip_exactly(written):
    _, _, members = written
    # Shortest round-trip reals: the double comes back bit for bit, not to twelve digits.
    assert members["beam Æ/1"]["placement_d"][12] == 1000.123456789012
    assert members["beam Æ/1"]["outer_curve"]["points"][1][0] == -0.05 + 0.01 * (1 - _S)


def test_the_profile_keeps_its_arcs_voids_name_and_bag(written):
    _, _, members = written
    b = members["beam Æ/1"]
    assert b["profile_type"] == "IFCARBITRARYPROFILEDEFWITHVOIDS"
    assert b["profile_name"] == "RHS100"
    assert b["outer_curve"]["segments"][0] == ("arc", (1, 2, 3))
    assert len(b["inner_curves"]) == 1
    assert b["profile_properties"] == {"SectionBag": {"sec_type": "poly", "t_w": 0.006}}


def test_the_cuts_are_clipping_results_in_order(written):
    _, _, members = written
    cuts = members["beam Æ/1"]["cuts"]
    assert [c["agreement"] for c in cuts] == [True, False]
    assert cuts[0]["origin"] == pytest.approx((0.0, 0.0, 0.1))
    assert cuts[1]["normal"] == pytest.approx((0.0, 0.6, 0.8))
    # A cut stated without an x direction gets one square to its normal.
    x = cuts[1]["xdir"]
    assert sum(a * b for a, b in zip(x, cuts[1]["normal"])) == pytest.approx(0.0, abs=1e-12)


def test_property_sets_and_material_come_through_typed(written):
    _, _, members = written
    b = members["beam Æ/1"]
    assert b["psets"] == {"Properties": {"Ref": "R-1/2'3", "imported": True, "n": 3, "x": 0.1, "tags": ["a", "b"]}}
    assert b["material"] == "S355"
    assert b["material_props"]["Grade"] == "S355"
    assert b["material_props"]["YieldStress"] == 355e6
    # One material, shared: both members name it, the file states it once.
    assert members["plate"]["material"] == "S355"


def test_a_plate_is_its_outline_thickness_and_plane(written):
    _, _, members = written
    p = members["plate"]
    assert p["ifc_class"] == "IFCPLATE"
    assert p["depth"] == 0.01
    assert p["origin"] == pytest.approx((1.0, 2.0, 3.0))
    assert p["outer_curve"]["kind"] == "polyline"
    assert len(p["outer_curve"]["points"]) == 5
    assert p["p1"] is not None and p["cuts"] == []


def test_a_parametric_profile_is_written_as_one(written):
    _, _, members = written
    t = members["tube"]
    assert t["profile_type"] == "IFCCIRCLEHOLLOWPROFILEDEF"
    assert t["profile_params"] == pytest.approx({"Radius": 0.05, "WallThickness": 0.005})


def test_ifcopenshell_reads_the_members(written):
    ifcopenshell = pytest.importorskip("ifcopenshell")
    path, _, _ = written
    f = ifcopenshell.open(str(path))
    (beam,) = [b for b in f.by_type("IfcBeam") if b.GlobalId == "0000000000000000000B01"]
    assert beam.Name == "beam Æ/1"
    reps = {r.RepresentationIdentifier: r for r in beam.Representation.Representations}
    assert reps["Body"].RepresentationType == "Clipping"
    body = reps["Body"].Items[0]
    assert body.is_a("IfcBooleanClippingResult") and body.SecondOperand.AgreementFlag is False
    assert body.FirstOperand.SecondOperand.AgreementFlag is True
    (plate,) = f.by_type("IfcPlate")
    assert [r.RelatingPropertyDefinition.Name for r in plate.IsDefinedBy] == ["Properties"]
    (rel,) = f.by_type("IfcRelAssociatesMaterial")
    assert {o.GlobalId for o in rel.RelatedObjects} == {
        "0000000000000000000B01",
        "0000000000000000000L01",
        "0000000000000000000B02",
    }


def test_an_opening_is_written_as_an_opening_element_voiding_its_member(written, tmp_path):
    # A boolean that is not a half-space (a hole) is an IfcOpeningElement with its own solid and an
    # IfcRelVoidsElement to the member -- not a member of its own, and not part of the member's body.
    path, _, _ = written
    blob = next(bytes(b) for b, m in cad.IfcNgeomStream(str(path)) if m.guid == "0000000000000000000B02")
    shifted = list(_IDENTITY)
    shifted[12:15] = [1.2, 2.1, 3.0]
    plate = _plate(openings=[{"name": "hole", "guid": "0000000000000000000H01", "blob": blob, "placement": shifted}])
    out = tmp_path / "opening.ifc"
    stats = cad.stream_ngeom_to_ifc([("plate", b"", None, None, None, plate)], str(out))
    assert stats["solids_out"] == 1
    text = out.read_text(encoding="utf-8")
    assert text.count("IFCOPENINGELEMENT('0000000000000000000H01'") == 1
    assert text.count("IFCRELVOIDSELEMENT(") == 1
    scanned = list(cad.IfcMemberScan(str(out)))
    assert [m["name"] for m in scanned] == ["plate"]  # the opening is not a product of its own
    ifcopenshell = pytest.importorskip("ifcopenshell")
    f = ifcopenshell.open(str(out))
    (opening,) = f.by_type("IfcOpeningElement")
    assert opening.VoidsElements[0].RelatingBuildingElement.GlobalId == "0000000000000000000L01"
    loc = opening.ObjectPlacement.RelativePlacement.Location.Coordinates
    assert loc == pytest.approx((1.2, 2.1, 3.0))
