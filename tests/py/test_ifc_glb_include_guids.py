"""Subset streaming: stream_ifc_to_glb(include_guids=...) builds one branch of a spatial tree.

The point of the filter is to build a branch WITHOUT first slicing a subset IFC (slicing duplicates
the bytes per build and is slower), so what these tests pin is the contract, not the geometry: an
empty filter is every product, a filter selects exactly its matches, ids that match no body are
skipped rather than fatal, and a filter that matches nothing fails instead of writing an empty GLB.
"""

import pytest

import adacpp.cad

# my_test.ifc: one IFCPLATE + six IFCBEAMs, all geometry-bearing -> 7 products unfiltered.
PLATE = "1cQMCXLayHxgKdv5utukF4"
BEAMS = [
    "1cPZ5wLayHxeFFv5utukF4",
    "1cPbYcLayHxe$Cv5utukF4",
    "1cPsijLayHxgUev5utukF4",
    "1cPv7pLayHxerkv5utukF4",
    "1cPxbBLayHxfhtv5utukF4",
    "1cQ0RLLayHxflGv5utukF4",
]
ABSENT = "NOTAREALGUID0000000000"


@pytest.fixture
def ifc_file(files_dir):
    return str(files_dir / "my_test.ifc")


def test_no_filter_streams_every_product(ifc_file, tmp_path):
    out = tmp_path / "all.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out)) == 7
    assert out.stat().st_size > 0


def test_empty_filter_is_every_product(ifc_file, tmp_path):
    """[] must mean "no filter", not "nothing" — every pre-existing caller relies on it."""
    out = tmp_path / "empty_filter.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out), include_guids=[]) == 7


@pytest.mark.parametrize(
    "guids, expected",
    [([BEAMS[0]], 1), ([PLATE], 1), ([PLATE, BEAMS[0], BEAMS[2]], 3), (BEAMS, 6)],
)
def test_filter_streams_exactly_its_matches(ifc_file, tmp_path, guids, expected):
    out = tmp_path / "subset.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out), include_guids=guids) == expected


def test_subset_is_smaller_than_the_whole(ifc_file, tmp_path):
    whole, part = tmp_path / "whole.glb", tmp_path / "part.glb"
    adacpp.cad.stream_ifc_to_glb(ifc_file, str(whole))
    adacpp.cad.stream_ifc_to_glb(ifc_file, str(part), include_guids=[BEAMS[0]])
    assert part.stat().st_size < whole.stat().st_size


def test_duplicate_ids_do_not_duplicate_products(ifc_file, tmp_path):
    out = tmp_path / "dupes.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out), include_guids=[BEAMS[0]] * 3) == 1


def test_unmatched_id_is_skipped_not_fatal(ifc_file, tmp_path):
    """A spatial branch names containers and curve-only axes too; only the bodies can be streamed."""
    out = tmp_path / "mixed.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out), include_guids=[BEAMS[0], ABSENT]) == 1


def test_filter_matching_nothing_fails_and_writes_no_glb(ifc_file, tmp_path):
    """The silent widen-on-failure this filter exists to prevent: never an empty GLB as success."""
    out = tmp_path / "none.glb"
    assert adacpp.cad.stream_ifc_to_glb(ifc_file, str(out), include_guids=[ABSENT]) == -1
    assert not out.exists()
