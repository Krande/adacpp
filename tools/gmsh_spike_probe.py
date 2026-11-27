"""SPIKE probe: which recombination stage makes wasm and native differ on the holed rectangle.

Runs the rect_recombine_unstructured face under option variants; prints JSON (native) or returns it.
"""

from __future__ import annotations

import json

VARIANTS = {
    "default": {},
    "threads1": {"General.NumThreads": 1},
    "no_topo_opt": {"Mesh.RecombineOptimizeTopology": 0},
    "no_smoothing": {"Mesh.Smoothing": 0},
    "no_topo_no_smooth": {"Mesh.RecombineOptimizeTopology": 0, "Mesh.Smoothing": 0},
    "simple_recombine": {"Mesh.RecombinationAlgorithm": 0},
    "simple_no_topo_no_smooth": {
        "Mesh.RecombinationAlgorithm": 0,
        "Mesh.RecombineOptimizeTopology": 0,
        "Mesh.Smoothing": 0,
    },
}


def run(opts: dict) -> list:
    import gmsh

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        r = gmsh.model.occ.addRectangle(0, 0, 0, 10, 5)
        d = gmsh.model.occ.addDisk(3, 2.5, 0, 1, 1)
        gmsh.model.occ.cut([(2, r)], [(2, d)])
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.Algorithm", 6)
        gmsh.option.setNumber("Mesh.RecombineAll", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.5)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 0.5)
        for k, v in opts.items():
            gmsh.option.setNumber(k, v)
        gmsh.model.mesh.generate(2)
        n = len(gmsh.model.mesh.getNodes()[0])
        types, tags, _ = gmsh.model.mesh.getElements(dim=2)
        return [n, {int(t): len(e) for t, e in zip(types, tags)}]
    finally:
        gmsh.finalize()


def run_all() -> dict:
    return {k: run(v) for k, v in VARIANTS.items()}


if __name__ == "__main__":
    print(json.dumps(run_all(), sort_keys=True))
