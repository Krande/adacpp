"""The SPIKE's gmsh cases (option B, adapy-plugins#177 step 2d), run unchanged natively and in Pyodide.

  native:  python tools/gmsh_spike_cases.py          (conda gmsh/python-gmsh 4.15.2 + ada-cpp)
  pyodide: tools/test_gmsh_pyodide.js runs run_all() in node-Pyodide over the three wheels

Each case returns node/element counts so the two sides compare as plain JSON.
"""

from __future__ import annotations

import json


def _counts() -> dict:
    import gmsh

    tags, _, _ = gmsh.model.mesh.getNodes()
    types, elem_tags, _ = gmsh.model.mesh.getElements(dim=2)
    by_type = {gmsh.model.mesh.getElementProperties(t)[0]: len(e) for t, e in zip(types, elem_tags)}
    return {"nodes": len(tags), "elements_2d": by_type}


def _transfinite_quads(surfaces: list[int], size: float) -> None:
    """Transfinite + recombine every (four-sided) surface, n per edge from its length (ada's recipe)."""
    import gmsh

    for s in surfaces:
        for _, c in gmsh.model.getBoundary([(2, s)], oriented=False):
            x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(1, abs(c))
            length = max(x1 - x0, y1 - y0, z1 - z0)
            gmsh.model.mesh.setTransfiniteCurve(abs(c), int(round(length / size)) + 1)
        gmsh.model.mesh.setTransfiniteSurface(s)
        gmsh.model.mesh.setRecombine(2, s)


def rect_transfinite() -> dict:
    """occ.addRectangle 10 x 5, transfinite at 0.5, recombine, generate(2): 21 x 11 nodes, 200 quads."""
    import gmsh

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("rect")
        s = gmsh.model.occ.addRectangle(0, 0, 0, 10, 5)
        gmsh.model.occ.synchronize()
        _transfinite_quads([s], 0.5)
        gmsh.model.mesh.generate(2)
        return _counts()
    finally:
        gmsh.finalize()


def _rect_hole(recombine: bool) -> dict:
    import gmsh

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("rect_hole")
        r = gmsh.model.occ.addRectangle(0, 0, 0, 10, 5)
        d = gmsh.model.occ.addDisk(3, 2.5, 0, 1, 1)
        gmsh.model.occ.cut([(2, r)], [(2, d)])
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.Algorithm", 6)
        gmsh.option.setNumber("Mesh.RecombineAll", 1 if recombine else 0)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.5)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.5)
        gmsh.model.mesh.generate(2)
        out = _counts()
        out["nodes_on_curves"] = len(gmsh.model.mesh.getNodes(dim=1, includeBoundary=True)[0])
        out["recombination_algorithm"] = gmsh.option.getNumber("Mesh.RecombinationAlgorithm")
        return out
    finally:
        gmsh.finalize()


def rect_recombine_unstructured() -> dict:
    """Rectangle with a hole, Frontal-Delaunay + RecombineAll (Blossom), size 0.5: not transfinite."""
    return _rect_hole(recombine=True)


def rect_triangles_unstructured() -> dict:
    """The same face, triangles only: separates the 2D triangulation from the recombination."""
    return _rect_hole(recombine=False)


def adacpp_plate() -> dict:
    """The native-pointer bridge: a 10 x 5 face built by adacpp, handed to gmsh as a TopoDS_Shape*."""
    import adacpp.cad as cad
    import gmsh

    box = cad.make_box(10.0, 5.0, 1.0)  # centred on the origin
    # The bottom (z = -0.5) face of the box: a 10 x 5 plate in adacpp's OCCT.
    plate = next(f for f in cad.faces(box) if all(abs(p[2] + 0.5) < 1e-9 for p in cad.vertex_points(f)))
    ptr = cad.to_topods_pointer(plate)

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("plate")
        ents = gmsh.model.occ.importShapesNativePointer(ptr)
        gmsh.model.occ.synchronize()
        surfaces = [t for d, t in ents if d == 2]
        _transfinite_quads(surfaces, 0.5)
        gmsh.model.mesh.generate(2)
        out = _counts()
        out["imported"] = [list(e) for e in ents]
        out["bbox"] = [round(v, 9) for v in gmsh.model.getBoundingBox(-1, -1)]
        return out
    finally:
        gmsh.finalize()


def adacpp_box_fragment() -> dict:
    """adacpp's 10 x 5 x 1 box SOLID through the pointer, fragmented by a gmsh plane at x = 1, then
    every face transfinite at 0.5: OCCT booleans (BOPAlgo) in gmsh on a shape made by adacpp."""
    import adacpp.cad as cad
    import gmsh

    box = cad.make_box(10.0, 5.0, 1.0)
    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("box_fragment")
        ents = gmsh.model.occ.importShapesNativePointer(cad.to_topods_pointer(box))
        cutter = gmsh.model.occ.addRectangle(-1, -4, 0, 5, 8)
        gmsh.model.occ.rotate([(2, cutter)], 0, 0, 0, 0, 1, 0, -3.141592653589793 / 2)
        gmsh.model.occ.translate([(2, cutter)], 1, 0, 0)
        gmsh.model.occ.fragment(ents, [(2, cutter)])
        gmsh.model.occ.synchronize()
        # drop the cutter's leftovers outside the box
        for d, t in gmsh.model.getEntities(2):
            x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(d, t)
            if y0 < -2.6 or y1 > 2.6 or z0 < -0.6 or z1 > 0.6:
                gmsh.model.removeEntities([(d, t)], recursive=True)
        surfaces = [t for _, t in gmsh.model.getEntities(2)]
        _transfinite_quads(surfaces, 0.5)
        gmsh.model.mesh.generate(2)
        out = _counts()
        out["volumes"] = len(gmsh.model.getEntities(3))
        out["surfaces"] = len(surfaces)
        return out
    finally:
        gmsh.finalize()


def build_info() -> dict:
    import gmsh

    gmsh.initialize(interruptible=False)
    try:
        return {"version": gmsh.__version__, "build_options": gmsh.option.getString("General.BuildOptions")}
    finally:
        gmsh.finalize()


CASES = {
    "build_info": build_info,
    "rect_transfinite": rect_transfinite,
    "rect_recombine_unstructured": rect_recombine_unstructured,
    "rect_triangles_unstructured": rect_triangles_unstructured,
    "adacpp_plate": adacpp_plate,
    "adacpp_box_fragment": adacpp_box_fragment,
}


def run_all() -> dict:
    out = {}
    for name, fn in CASES.items():
        try:
            out[name] = fn()
        except Exception as e:  # report, keep going: one broken case must not hide the rest
            out[name] = {"error": f"{type(e).__name__}: {e}"}
    return out


if __name__ == "__main__":
    print(json.dumps(run_all(), indent=1, sort_keys=True))
