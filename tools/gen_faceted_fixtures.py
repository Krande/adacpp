"""Write small faceted STEP / IFC models with ONE huge solid among a few small ones.

The huge-root paths -- the native pool's phase A (a root bigger than a thread's fair share, resolved
and tessellated by face slices) and the browser shards' face split -- only run on a solid of at least
2048 faces, and no checked-in fixture has one. These generate it: a faceted UV sphere of
`nu * nv` planar faces (quads, triangles at the poles) plus `nboxes` six-face boxes, as a STEP
(MANIFOLD_SOLID_BREP / FACE_SURFACE / POLY_LOOP / PLANE) and as an IFC4 (IfcBuildingElementProxy /
IfcFacetedBrep / IfcFace / IfcPolyLoop). No dependencies.

    python tools/gen_faceted_fixtures.py <out_dir> [nu nv nboxes]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path


def _sphere(nu: int, nv: int, r: float = 1000.0) -> list[list[tuple[float, float, float]]]:
    def pt(i: int, j: int) -> tuple[float, float, float]:
        th = 2 * math.pi * (i % nu) / nu
        ph = math.pi * j / nv
        return (r * math.sin(ph) * math.cos(th), r * math.sin(ph) * math.sin(th), r * math.cos(ph))

    faces = []
    for j in range(nv):
        for i in range(nu):
            a, b, c, d = pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1)
            if j == 0:
                faces.append([a, c, d])  # north cap: a == b
            elif j == nv - 1:
                faces.append([a, b, c])  # south cap: c == d
            else:
                faces.append([a, b, c, d])
    return faces


def _box(x0: float, size: float = 200.0) -> list[list[tuple[float, float, float]]]:
    s = size
    v = [(x0 + dx * s, dy * s, dz * s) for dz in (0, 1) for dy in (0, 1) for dx in (0, 1)]
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return [[v[k] for k in q] for q in quads]


def _normal(poly: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    nx = ny = nz = 0.0
    for k, (x1, y1, z1) in enumerate(poly):
        x2, y2, z2 = poly[(k + 1) % len(poly)]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / n, ny / n, nz / n)


def _solids(nu: int, nv: int, nboxes: int) -> list[list[list[tuple[float, float, float]]]]:
    return [_sphere(nu, nv)] + [_box(1500.0 + 400.0 * k) for k in range(nboxes)]


def _f(x: float) -> str:
    return f"{x:.6f}"


def write_step(path: Path, nu: int = 64, nv: int = 40, nboxes: int = 6) -> None:
    lines: list[str] = []
    nid = [0]

    def ent(text: str) -> int:
        nid[0] += 1
        lines.append(f"#{nid[0]}={text};")
        return nid[0]

    for solid in _solids(nu, nv, nboxes):
        faces = []
        for poly in solid:
            pts = [ent(f"CARTESIAN_POINT('',({_f(x)},{_f(y)},{_f(z)}))") for x, y, z in poly]
            loop = ent(f"POLY_LOOP('',({','.join(f'#{p}' for p in pts)}))")
            bound = ent(f"FACE_OUTER_BOUND('',#{loop},.T.)")
            n = _normal(poly)
            ref = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
            dot = sum(a * b for a, b in zip(ref, n))
            refd = tuple(a - dot * b for a, b in zip(ref, n))
            rl = math.sqrt(sum(a * a for a in refd))
            refd = tuple(a / rl for a in refd)
            org = ent(f"CARTESIAN_POINT('',({_f(poly[0][0])},{_f(poly[0][1])},{_f(poly[0][2])}))")
            ax = ent(f"DIRECTION('',({_f(n[0])},{_f(n[1])},{_f(n[2])}))")
            rd = ent(f"DIRECTION('',({_f(refd[0])},{_f(refd[1])},{_f(refd[2])}))")
            pl = ent(f"PLANE('',#{ent(f'AXIS2_PLACEMENT_3D({chr(39)}{chr(39)},#{org},#{ax},#{rd})')})")
            faces.append(ent(f"FACE_SURFACE('',(#{bound}),#{pl},.T.)"))
        shell = ent(f"CLOSED_SHELL('',({','.join(f'#{f}' for f in faces)}))")
        ent(f"MANIFOLD_SOLID_BREP('',#{shell})")
    text = (
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('faceted','',(''),(''),'','','');\n"
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n" + "\n".join(lines) + "\nENDSEC;\nEND-ISO-10303-21;\n"
    )
    path.write_text(text)


def write_ifc(path: Path, nu: int = 64, nv: int = 40, nboxes: int = 6) -> None:
    lines: list[str] = []
    nid = [0]

    def ent(text: str) -> int:
        nid[0] += 1
        lines.append(f"#{nid[0]}={text};")
        return nid[0]

    unit = ent("IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.)")
    units = ent(f"IFCUNITASSIGNMENT((#{unit}))")
    origin = ent("IFCCARTESIANPOINT((0.,0.,0.))")
    wcs = ent(f"IFCAXIS2PLACEMENT3D(#{origin},$,$)")
    ctx = ent(f"IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-05,#{wcs},$)")
    ent(f"IFCPROJECT('0faceted0project0000000',$,'faceted',$,$,$,$,(#{ctx}),#{units})")
    for k, solid in enumerate(_solids(nu, nv, nboxes)):
        faces = []
        for poly in solid:
            pts = [ent(f"IFCCARTESIANPOINT(({_f(x)},{_f(y)},{_f(z)}))") for x, y, z in poly]
            loop = ent(f"IFCPOLYLOOP(({','.join(f'#{p}' for p in pts)}))")
            bound = ent(f"IFCFACEOUTERBOUND(#{loop},.T.)")
            faces.append(ent(f"IFCFACE((#{bound}))"))
        shell = ent(f"IFCCLOSEDSHELL(({','.join(f'#{f}' for f in faces)}))")
        brep = ent(f"IFCFACETEDBREP(#{shell})")
        rep = ent(f"IFCSHAPEREPRESENTATION(#{ctx},'Body','Brep',(#{brep}))")
        pds = ent(f"IFCPRODUCTDEFINITIONSHAPE($,$,(#{rep}))")
        place = ent(f"IFCLOCALPLACEMENT($,#{wcs})")
        guid = f"0faceted{k:014d}"
        ent(f"IFCBUILDINGELEMENTPROXY('{guid}',$,'solid{k}',$,$,#{place},#{pds},$,$)")
    text = (
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('faceted','',(''),(''),'','','');\n"
        "FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n" + "\n".join(lines) + "\nENDSEC;\nEND-ISO-10303-21;\n"
    )
    path.write_text(text)


if __name__ == "__main__":
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    args = [int(a) for a in sys.argv[2:5]]
    write_step(out / "faceted_huge.stp", *args)
    write_ifc(out / "faceted_huge.ifc", *args)
    print(out / "faceted_huge.stp", out / "faceted_huge.ifc")
