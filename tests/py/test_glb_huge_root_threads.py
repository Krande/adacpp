"""The native pool's huge-root path (phase A) writes the serial path's geometry.

A root bigger than one thread's fair share is resolved and tessellated by face slices on every
thread, joined in face order and welded once (huge_root_tess.h). Its output must not depend on the
thread count: the same triangles and vertices per mesh as a 1-thread conversion, which takes no
phase A at all. The fixture is a generated faceted model -- one 2560-face sphere among six boxes --
in STEP and IFC (tests/fixtures/gen_faceted_fixtures.py).
"""

import importlib.util
import json
import pathlib
import struct
from collections import Counter

import pytest

import adacpp.cad

_GEN = pathlib.Path(__file__).parents[1] / "fixtures" / "gen_faceted_fixtures.py"
_spec = importlib.util.spec_from_file_location("gen_faceted_fixtures", _GEN)
gen = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gen)


def glb_meshes(path):
    """(name, triangles, vertices) per primitive, as a multiset, plus the totals."""
    b = path.read_bytes()
    n = struct.unpack_from("<I", b, 12)[0]
    g = json.loads(b[20 : 20 + n])
    acc = g["accessors"]
    per = Counter()
    for m in g["meshes"]:
        for p in m["primitives"]:
            per[(m.get("name", ""), acc[p["indices"]]["count"] // 3, acc[p["attributes"]["POSITION"]]["count"])] += 1
    return per


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    d = tmp_path_factory.mktemp("faceted")
    gen.write_step(d / "huge.stp")
    gen.write_ifc(d / "huge.ifc")
    return d


@pytest.mark.parametrize(
    "ext, convert", [("stp", adacpp.cad.stream_step_to_glb), ("ifc", adacpp.cad.stream_ifc_to_glb)]
)
def test_huge_root_threads_match_serial(models, tmp_path, ext, convert):
    src = str(models / f"huge.{ext}")
    serial, threaded = tmp_path / "t1.glb", tmp_path / "t4.glb"
    assert convert(src, str(serial), num_threads=1) == 7
    assert convert(src, str(threaded), num_threads=4) == 7
    one, four = glb_meshes(serial), glb_meshes(threaded)
    assert sum(k[1] * v for k, v in one.items()) >= 5000  # the sphere is there
    assert one == four
