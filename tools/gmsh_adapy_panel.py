"""adapy's REAL mesh step on a stiffened panel, with the gmsh in adacpp's pyodide wheel.

The call chain a downstream Sesam mesh step runs:

    options = GmshOptions(); options.Mesh_RecombineAll = 1
    assembly.fem = assembly.to_fem_obj(mesh_size, options=options, use_quads=True)
    assembly.to_fem(name, "sesam", scratch_dir=..., execute=False, overwrite=True)

on a 6 x 3 m, 12 mm plate with three TG200x100x8x12 stiffeners along x and one HP-like girder
across it at x = 3 (beams as line elements on the shell nodes, ada splits the plate along them).

Run unchanged natively and in Pyodide (tools/test_gmsh_pyodide.js --adapy <adapy wheel>):
    python tools/gmsh_adapy_panel.py [out_dir] [mesh_size] [recombination_algorithm]
Prints JSON: node / element counts per type, deck size + sha256, mesh and write seconds.
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys
import time
from collections import Counter


def build_panel():
    import ada
    from ada.materials.metals import CarbonSteel

    mat = ada.Material("S355", CarbonSteel("S355"))
    length, width = 6.0, 3.0
    pl = ada.Plate("pl", [(0.0, 0.0), (length, 0.0), (length, width), (0.0, width)], 0.012, mat=mat)
    objs = [pl]
    for i, y in enumerate((0.75, 1.5, 2.25)):
        objs.append(ada.Beam(f"stiff{i}", (0.0, y, 0.0), (length, y, 0.0), "TG200x100x8x12", mat))
    objs.append(ada.Beam("girder", (3.0, 0.0, 0.0), (3.0, width, 0.0), "TG400x200x10x20", mat))
    part = ada.Part("panel") / objs
    return ada.Assembly("panel_site") / part


# Records that carry the run's date, user or free text: left out of the comparable deck body.
_VOLATILE = {b"DATE", b"IDENT", b"TEXT"}


def _records(data: bytes):
    """Yield (record name, lines) for a Sesam deck: a record starts on a non-blank first column."""
    name, lines = None, []
    for ln in data.splitlines():
        if ln[:1] not in (b" ", b"") and ln.strip():
            if name is not None:
                yield name, lines
            name, lines = ln.split()[0], [ln]
        elif name is not None:
            lines.append(ln)
    if name is not None:
        yield name, lines


def _deck_body(data: bytes) -> bytes:
    return b"\n".join(b"\n".join(lines) for name, lines in _records(data) if name not in _VOLATILE)


def _record_counts(data: bytes) -> Counter:
    return Counter(name.decode("ascii", "replace") for name, _ in _records(data))


def run(out_dir: str = "/tmp/panel_deck", mesh_size: float = 0.25, recomb_alg: int | None = None) -> dict:
    import gmsh
    from ada.fem.meshing import GmshOptions

    a = build_panel()
    options = GmshOptions()
    options.Mesh_RecombineAll = 1
    if recomb_alg is not None:
        options.Mesh_RecombinationAlgorithm = recomb_alg
    t0 = time.perf_counter()
    a.fem = a.to_fem_obj(mesh_size, options=options, use_quads=True)
    t_mesh = time.perf_counter() - t0
    fem = a.fem
    elem_types = Counter(str(getattr(e.type, "value", e.type)) for e in fem.elements)

    t0 = time.perf_counter()
    a.to_fem("panel", "sesam", scratch_dir=out_dir, execute=False, overwrite=True)
    t_write = time.perf_counter() - t0
    decks = sorted(pathlib.Path(out_dir, "panel").glob("*.FEM"))
    deck = decks[0] if decks else None
    data = deck.read_bytes() if deck else b""
    body = _deck_body(data)
    gnodes = sum(1 for ln in data.splitlines() if ln.startswith(b"GNODE"))
    gelmnt = sum(1 for ln in data.splitlines() if ln.startswith(b"GELMNT1"))
    return {
        "gmsh_version": gmsh.__version__,
        "platform": sys.platform,
        "mesh_size": mesh_size,
        "recombination_algorithm": recomb_alg,
        "nodes": len(fem.nodes),
        "elements": len(fem.elements),
        "element_types": dict(sorted(elem_types.items())),
        "deck": deck.name if deck else None,
        "deck_bytes": len(data),
        "deck_sha256": hashlib.sha256(data).hexdigest(),
        "deck_body_sha256": hashlib.sha256(body).hexdigest(),
        "deck_record_counts": dict(sorted(_record_counts(data).items())),
        "deck_GNODE": gnodes,
        "deck_GELMNT1": gelmnt,
        "mesh_seconds": round(t_mesh, 3),
        "write_seconds": round(t_write, 3),
    }


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/panel_deck"
    size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    alg = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] != "-" else None
    print(json.dumps(run(out, size, alg), indent=1, sort_keys=True))
