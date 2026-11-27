"""Shared gmsh test cases for the gmsh-in-pyodide spikes (adapy-plugins#177 step 2d).

One file, run unchanged in three places, so the counts are a comparison basis:
  * natively (conda gmsh 4.15.2 + native adacpp):   python tools/gmsh_spike_cases.py [fixture.stp]
  * in node-pyodide, option A (gmsh inside adacpp's extension): tools/test_gmsh_pyodide.js
  * in node-pyodide, option B (gmsh wheel + shared OCCT side module): same harness, other wheel(s)

tools/gmsh_spike_native.json is the native reference: this script's output in the adapy devstack
image (conda gmsh 4.15.2 h09a0abf_11, occt 8.0.1, adacpp 0.44.0 native, x86_64 Linux).

It needs only `import gmsh` and `import adacpp.cad`; it does not care which binary gmsh.py loads.
`run_all()` returns a JSON-able dict {case: {...counts...}}; every case fails soft (an "error" key)
so one broken path cannot hide the others. The STEP case reads the repo fixture
files/flat_plate_abaqus_10x10_m_wColors.stp (pass its path; the harness copies it to /fixture.stp).

Cases (each starts from a fresh gmsh model):
  rect_transfinite        gmsh only: occ.addRectangle 2 x 1, transfinite 21 x 11 nodes, recombined.
                          Deterministic by construction: 231 nodes, 200 quads.
  plate_native_pointer    the same rectangle built by adacpp (cad.polygon_face), handed to gmsh as a
                          TopoDS_Shape* (cad.to_topods_pointer -> occ.importShapesNativePointer),
                          then meshed exactly as above. Same counts as rect_transfinite or the bridge
                          is broken.
  lshape_recombine_simple adacpp L-shaped plate (6 edges, not transfinite-able) through the pointer
                          bridge, Frontal-Delaunay (Mesh.Algorithm=6), size 0.1,
                          Mesh.RecombineAll=1, Mesh.RecombinationAlgorithm=0 (simple).
  lshape_recombine_blossom the same with RecombinationAlgorithm=1 (gmsh's default, Blossom). Option A
                          builds without Blossom (licence), so this is where wasm and native may differ.
  plate_stiffener_fragment a 2 x 1 plate plus a perpendicular 0.3 high stiffener along y=0.5, both
                          through the pointer bridge, occ.fragment, size 0.1, RecombineAll, alg 6,
                          RecombinationAlgorithm=0.
                          Exercises fragment + shared-edge conformity, as ada's partitioning does.
  size_field              the L plate with a Distance(point)+Threshold background field (0.02 near the
                          re-entrant corner, 0.2 far), Min field, recombined: ada's overrides.py path.
  step_import             occ.importShapes of the STEP fixture (the fallback path; OCC_CAF reader),
                          size 1.0, recombined.
"""

from __future__ import annotations

import json
import sys
import time


def _counts(gmsh) -> dict:
    node_tags, _, _ = gmsh.model.mesh.getNodes()
    types, elem_tags, _ = gmsh.model.mesh.getElements()
    by_type = {}
    for t, tags in zip(types, elem_tags):
        name = gmsh.model.mesh.getElementProperties(t)[0]
        by_type[name] = int(len(tags))
    return {"nodes": int(len(node_tags)), "elements": dict(sorted(by_type.items()))}


def _fresh(gmsh, name: str):
    if gmsh.isInitialized():
        gmsh.finalize()
    gmsh.initialize(interruptible=False)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add(name)


def _transfinite_rect(gmsh, surface_tag: int, nx: int = 21, ny: int = 11):
    # Curves of a 2 x 1 rectangle: the two along x get nx nodes, the two along y ny nodes.
    for dim, ctag in gmsh.model.getBoundary([(2, surface_tag)], oriented=False):
        xmin, ymin, _, xmax, ymax, _ = gmsh.model.getBoundingBox(dim, abs(ctag))
        along_x = (xmax - xmin) > (ymax - ymin)
        gmsh.model.mesh.setTransfiniteCurve(abs(ctag), nx if along_x else ny)
    gmsh.model.mesh.setTransfiniteSurface(surface_tag)
    gmsh.model.mesh.setRecombine(2, surface_tag)


def _surfaces(dimtags):
    return [t for d, t in dimtags if d == 2]


RECT = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
LSHAPE = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 0.6, 0.0), (0.8, 0.6, 0.0), (0.8, 1.2, 0.0), (0.0, 1.2, 0.0)]
STIFFENER = [(0.0, 0.5, 0.0), (2.0, 0.5, 0.0), (2.0, 0.5, 0.3), (0.0, 0.5, 0.3)]


def case_rect_transfinite(gmsh, cad):
    _fresh(gmsh, "rect_transfinite")
    s = gmsh.model.occ.addRectangle(0, 0, 0, 2, 1)
    gmsh.model.occ.synchronize()
    _transfinite_rect(gmsh, s)
    gmsh.model.mesh.generate(2)
    return _counts(gmsh)


def _import_ptr(gmsh, cad, points):
    shape = cad.polygon_face([list(p) for p in points])
    ents = gmsh.model.occ.importShapesNativePointer(cad.to_topods_pointer(shape))
    return shape, ents  # keep `shape` alive until gmsh has copied it


def case_plate_native_pointer(gmsh, cad):
    _fresh(gmsh, "plate_native_pointer")
    _shape, ents = _import_ptr(gmsh, cad, RECT)
    gmsh.model.occ.synchronize()
    surfs = _surfaces(ents)
    for s in surfs:
        _transfinite_rect(gmsh, s)
    gmsh.model.mesh.generate(2)
    out = _counts(gmsh)
    out["imported"] = [list(e) for e in ents]
    return out


def _lshape(gmsh, cad, name: str, recomb_alg: int):
    _fresh(gmsh, name)
    _shape, ents = _import_ptr(gmsh, cad, LSHAPE)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.1)
    gmsh.option.setNumber("Mesh.RecombineAll", 1)
    gmsh.option.setNumber("Mesh.RecombinationAlgorithm", recomb_alg)
    gmsh.model.mesh.generate(2)
    return _counts(gmsh)


def case_lshape_recombine_simple(gmsh, cad):
    return _lshape(gmsh, cad, "lshape_simple", 0)


def case_lshape_recombine_blossom(gmsh, cad):
    return _lshape(gmsh, cad, "lshape_blossom", 1)


def case_plate_stiffener_fragment(gmsh, cad):
    _fresh(gmsh, "plate_stiffener_fragment")
    _a, plate = _import_ptr(gmsh, cad, RECT)
    _b, stiff = _import_ptr(gmsh, cad, STIFFENER)
    out_dimtags, _ = gmsh.model.occ.fragment(plate, stiff)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.1)
    gmsh.option.setNumber("Mesh.RecombineAll", 1)
    gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 0)  # Blossom is measured in lshape_*
    gmsh.model.mesh.generate(2)
    out = _counts(gmsh)
    out["surfaces_after_fragment"] = len(_surfaces(out_dimtags))
    return out


def case_size_field(gmsh, cad):
    _fresh(gmsh, "size_field")
    _shape, ents = _import_ptr(gmsh, cad, LSHAPE)
    gmsh.model.occ.synchronize()
    corner = gmsh.model.getEntitiesInBoundingBox(0.79, 0.59, -0.01, 0.81, 0.61, 0.01, dim=0)
    f_dist = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_dist, "PointsList", [t for _, t in corner])
    f_thr = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thr, "InField", f_dist)
    gmsh.model.mesh.field.setNumber(f_thr, "SizeMin", 0.02)
    gmsh.model.mesh.field.setNumber(f_thr, "SizeMax", 0.2)
    gmsh.model.mesh.field.setNumber(f_thr, "DistMin", 0.05)
    gmsh.model.mesh.field.setNumber(f_thr, "DistMax", 0.5)
    f_min = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(f_min, "FieldsList", [f_thr])
    gmsh.model.mesh.field.setAsBackgroundMesh(f_min)
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.RecombineAll", 1)
    gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 0)
    gmsh.model.mesh.generate(2)
    out = _counts(gmsh)
    out["corner_points"] = len(corner)
    return out


def case_step_import(gmsh, cad, step_path: str | None):
    if not step_path:
        return {"skipped": "no STEP fixture given"}
    _fresh(gmsh, "step_import")
    # Keep the fixture in metres (gmsh's OCC import converts to mm by default: 10 m -> 10000).
    gmsh.option.setString("Geometry.OCCTargetUnit", "M")
    ents = gmsh.model.occ.importShapes(step_path)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.Algorithm", 6)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 1.0)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 1.0)
    gmsh.option.setNumber("Mesh.RecombineAll", 1)
    gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 0)
    gmsh.model.mesh.generate(2)
    out = _counts(gmsh)
    out["imported"] = len(ents)
    return out


CASES = [
    ("rect_transfinite", case_rect_transfinite),
    ("plate_native_pointer", case_plate_native_pointer),
    ("lshape_recombine_simple", case_lshape_recombine_simple),
    ("lshape_recombine_blossom", case_lshape_recombine_blossom),
    ("plate_stiffener_fragment", case_plate_stiffener_fragment),
    ("size_field", case_size_field),
]


def run_all(step_path: str | None = None) -> dict:
    import gmsh

    import adacpp.cad as cad

    import os

    skip = set(filter(None, os.environ.get("GMSH_SPIKE_SKIP", "").split(",")))
    results = {"gmsh_version": gmsh.__version__, "platform": sys.platform}
    for name, fn in CASES + [("step_import", lambda g, c: case_step_import(g, c, step_path))]:
        if name in skip:
            results[name] = {"skipped": "GMSH_SPIKE_SKIP"}
            continue
        print(f"[gmsh_spike_cases] {name}", file=sys.stderr, flush=True)
        t0 = time.perf_counter()
        try:
            results[name] = fn(gmsh, cad)
        except Exception as e:  # report, keep going
            results[name] = {"error": f"{type(e).__name__}: {e}"}
        results[name]["seconds"] = round(time.perf_counter() - t0, 3)
    if gmsh.isInitialized():
        gmsh.finalize()
    return results


if __name__ == "__main__":
    print(json.dumps(run_all(sys.argv[1] if len(sys.argv) > 1 else None), indent=1, sort_keys=True))
