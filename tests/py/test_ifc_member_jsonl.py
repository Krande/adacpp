"""The member scan as a FILE, which is the form the browser gets.

embind has no cheap equivalent of "yield one dict per product": a JS object per member crosses the
JS/wasm boundary once per member, and a plant has hundreds of thousands of them. So the wasm build
writes JSONL to an OPFS path instead, and this module pins the thing that would otherwise rot --
that the file and the dicts describe the SAME members, field for field. A browser clash check that
silently saw different members than the server's would be worse than one that did not exist.
"""

from __future__ import annotations

import json

import pytest

from adacpp import cad

IFC = "files/my_test.ifc"


@pytest.fixture(scope="module")
def scanned(tmp_path_factory):
    out = tmp_path_factory.mktemp("jsonl") / "members.jsonl"
    written = cad.scan_ifc_members_to_jsonl(IFC, str(out))
    lines = out.read_text(encoding="utf-8").splitlines()
    return written, json.loads(lines[0]), [json.loads(x) for x in lines[1:]]


def test_the_header_says_what_the_file_is(scanned):
    # A JSONL file found on its own -- in OPFS, in a cache, attached to a job -- has to be able to
    # say what it is and what units it is in without the code that wrote it.
    _, header, _ = scanned
    assert header["schema"] == "adacpp.ifc_members/1"
    assert header["products"] == 7
    assert header["unit_scale"] == pytest.approx(1.0)


def test_every_member_is_one_line(scanned):
    written, header, records = scanned
    assert written == len(records) == header["products"]


def test_the_file_and_the_dicts_are_the_same_members(scanned):
    """The parity that matters: same members, same fields, same values."""
    _, _, records = scanned
    dicts = list(cad.IfcMemberScan(IFC))
    assert len(records) == len(dicts)

    for rec, d in zip(records, dicts):
        assert rec["id"] == d["id"]
        assert rec["guid"] == d["guid"]
        assert rec["name"] == d["name"]
        assert rec["ifc_class"] == d["ifc_class"]
        assert rec["profile_name"] == d["profile_name"]
        assert rec["profile_type"] == d["profile_type"]
        assert rec["depth"] == pytest.approx(d["depth"])
        assert rec["material"] == d["material"]
        assert rec["material_props"].keys() == d["material_props"].keys()
        for k, v in d["material_props"].items():
            if isinstance(v, str):
                assert rec["material_props"][k] == v
            else:
                assert rec["material_props"][k] == pytest.approx(v)
        for key in ("p1", "p2", "origin", "normal", "xdir"):
            if d[key] is None:
                assert rec[key] is None
            else:
                assert rec[key] == pytest.approx(list(d[key]))
        assert rec["placement"] == pytest.approx(list(d["placement"]))
        assert [tuple(pt) for pt in rec["outline"]] == pytest.approx([tuple(pt) for pt in d["outline"]])


def test_a_name_with_json_in_it_does_not_break_the_line(tmp_path):
    """One quote in one product name would otherwise corrupt the whole file.

    CAD names are user text: quotes, backslashes and stray control bytes all occur. JSONL has no
    recovery -- a broken line is a broken record AND a broken parse for everything a reader does
    afterwards -- so the escaping is worth a test of its own rather than trust.
    """
    src = tmp_path / "quoted.ifc"
    with open(IFC, encoding="utf-8", errors="surrogateescape") as fh:
        raw = fh.read()
    src.write_text(raw.replace("'bm1'", "'say \\\\X2\\\\0022 and \\\\ back'"), encoding="utf-8")

    out = tmp_path / "members.jsonl"
    written = cad.scan_ifc_members_to_jsonl(str(src), str(out))
    assert written == 7
    for line in out.read_text(encoding="utf-8").splitlines():
        json.loads(line)  # every line parses, which is the whole claim


def test_coordinates_survive_the_round_trip_at_cad_precision(scanned):
    """The writer prints 12 significant digits, not 17. This is what that has to be worth.

    Shortening the printing is what keeps a plant-scale scan from being mostly digits, but it is
    only safe if what comes back is still the same geometry. A millimetre model spans ~1e5 mm and
    12 digits leaves ~1e-7 mm of slack -- far below any tolerance these files carry, and this pins
    that the claim holds against the scan's own numbers rather than against an argument.
    """
    _, _, records = scanned
    dicts = list(cad.IfcMemberScan(IFC))
    for rec, d in zip(records, dicts):
        for key in ("p1", "p2"):
            if d[key] is None:
                continue
            for written, source in zip(rec[key], d[key]):
                assert written == pytest.approx(source, rel=1e-11, abs=1e-12)
        assert rec["depth"] == pytest.approx(d["depth"], rel=1e-11, abs=1e-12)


def test_an_unwritable_destination_reports_failure_rather_than_pretending(tmp_path):
    assert cad.scan_ifc_members_to_jsonl(IFC, str(tmp_path / "no" / "such" / "dir" / "m.jsonl")) == -1
