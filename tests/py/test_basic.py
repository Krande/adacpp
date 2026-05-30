import random

import adacpp
import pytest


def create_box_grid(grid_size):
    # Randomized box sizes
    min_size, max_size = 0.5, 1.0
    origins = []
    dimensions = []

    for x in range(grid_size):
        for y in range(grid_size):
            for z in range(grid_size):
                width = random.uniform(min_size, max_size)
                height = random.uniform(min_size, max_size)
                depth = random.uniform(min_size, max_size)
                origins.append((x * max_size, y * max_size, z * max_size))
                dimensions.append((width, height, depth))

    return origins, dimensions


def test_basic_write_step(tmp_path):
    adacpp.cadit.write_box_to_step(str(tmp_path / "mybox.stp"), (0, 0, 0), (10, 10, 10))


def test_convert_step_to_glb(files_dir, tmp_path):
    adacpp.cadit.conversion.stp_to_glb(
        str(files_dir / "flat_plate_abaqus_10x10_m_wColors.stp"),
        str(tmp_path / "flat_plate_abaqus_10x10_m_wColors.glb"),
    )


def test_basic_write_list_of_boxes_to_step(tmp_path):
    origins, dimensions = create_box_grid(10)
    adacpp.cadit.write_boxes_to_step(str(tmp_path / "myboxes.stp"), origins, dimensions)

    adacpp.cadit.conversion.stp_to_glb(str(tmp_path / "myboxes.stp"), str(tmp_path / "myboxes_convert.glb"))


def test_basic_mesh(tmp_path):
    mesh = adacpp.visit.get_box_mesh((0, 0, 0), (10, 10, 10))
    assert mesh.__class__.__name__ == "Mesh"
    assert hasattr(mesh, "positions")
    assert hasattr(mesh, "normals")
    assert hasattr(mesh, "indices")

    adacpp.cadit.write_mesh_to_gltf(str(tmp_path / "mybox.gltf"), mesh)
    print(mesh)


def test_boxes_mesh_gltf(tmp_path):
    origins, dimensions = create_box_grid(2)
    adacpp.cadit.write_boxes_to_gltf(str(tmp_path / "myboxes.glb"), origins, dimensions)


def test_simple_gmsh(tmp_path):
    adacpp.fem.create_gmesh(str(tmp_path / "my_fem_mesh.msh"))


def test_tess_factory():
    tess_factory = adacpp.visit.TessellateFactory()
    num = 50
    boxes = []
    for x in range(num):
        for y in range(num):
            for z in range(num):
                boxes.append(adacpp.geom.Box((x * 10, y * 10, z * 10), 10, 10, 10))
    box0 = boxes[0]
    assert box0.id == 0
    assert tess_factory.algorithm == adacpp.visit.TessellationAlgorithm.OCCT_DEFAULT
    tess_factory.tessellate()


@pytest.mark.skip(
    reason="ifcopenshell 0.8.5's IfcFile ctor/dtor segfaults (heap corruption) when "
    "libIfcParse.a is statically linked into a downstream consumer — upstream "
    "IfcOpenShell #7131. Our cmake/deps_ifc.cmake already links the *static* "
    "librocksdb.a (not the shared .so) to avoid a second rocksdb instance; the "
    "remaining crash is purely upstream. Re-enable once a newer ifcopenshell build "
    "stops segfaulting on Linux."
)
def test_ifc_file_read(files_dir):
    adacpp.cadit.ifc.read_ifc_file(str(files_dir / "my_test.ifc"))


def test_cad_tessellate_box():
    mesh = adacpp.cad.tessellate_box(2.0, 3.0, 4.0)
    positions = list(mesh.positions)
    assert len(positions) % 3 == 0
    assert len(mesh.indices) % 3 == 0
    assert len(mesh.indices) >= 12 * 3  # at least 12 triangles for a 6-face box

    # Centered axis-aligned box: AABB should be (±dx/2, ±dy/2, ±dz/2).
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    assert min(xs) == -1.0 and max(xs) == 1.0
    assert min(ys) == -1.5 and max(ys) == 1.5
    assert min(zs) == -2.0 and max(zs) == 2.0


def test_cad_mesh_is_visit_mesh():
    """adacpp.cad.Mesh and adacpp.visit.Mesh must be the same class."""
    assert adacpp.cad.Mesh is adacpp.visit.Mesh


def test_cad_make_box_and_tessellate():
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    assert type(box).__name__ == "ShapeHandle"

    mesh = adacpp.cad.tessellate(box)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    assert min(xs) == -1.0 and max(xs) == 1.0
    assert min(ys) == -1.5 and max(ys) == 1.5
    assert min(zs) == -2.0 and max(zs) == 2.0
    assert len(mesh.indices) >= 12 * 3


def test_cad_shape_handle_is_opaque():
    """ShapeHandle must not leak its kernel internals to Python."""
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    public_attrs = [a for a in dir(box) if not a.startswith("_")]
    assert public_attrs == []


def test_cad_make_cylinder():
    cyl = adacpp.cad.make_cylinder(2.0, 5.0)
    mesh = adacpp.cad.tessellate(cyl)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    # Cylinder along +Z, radius 2, base at z=0, top at z=5.
    assert min(zs) == 0.0 and max(zs) == 5.0
    # XY radius is approximately 2 (tessellation chord-deflects slightly inward).
    assert abs(max(xs) - 2.0) < 0.2
    assert abs(min(xs) - -2.0) < 0.2


def test_cad_make_sphere():
    sph = adacpp.cad.make_sphere(3.0)
    mesh = adacpp.cad.tessellate(sph)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    # Sphere centered at origin, radius 3.
    assert abs(max(xs) - 3.0) < 0.3 and abs(min(xs) - -3.0) < 0.3
    assert abs(max(ys) - 3.0) < 0.3 and abs(min(ys) - -3.0) < 0.3
    assert abs(max(zs) - 3.0) < 0.3 and abs(min(zs) - -3.0) < 0.3


def test_cad_bbox_box():
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    bb = adacpp.cad.bbox(box)
    assert tuple(bb) == (-1.0, -1.5, -2.0, 1.0, 1.5, 2.0)


def test_cad_bbox_cylinder():
    cyl = adacpp.cad.make_cylinder(2.0, 5.0)
    bb = adacpp.cad.bbox(cyl)
    # Native OCCT bbox is exact for primitives; +Z cylinder, base at 0.
    assert tuple(bb) == (-2.0, -2.0, 0.0, 2.0, 2.0, 5.0)


def test_cad_bbox_sphere():
    sph = adacpp.cad.make_sphere(3.0)
    bb = adacpp.cad.bbox(sph)
    assert tuple(bb) == (-3.0, -3.0, -3.0, 3.0, 3.0, 3.0)


def test_cad_extruded_curve_profile_is_open_shell():
    # CURVE profile (is_area=False): sweeping a circle wire +Z yields the open
    # lateral cylinder surface — no end caps. Outer circle r=0.5 in XY, depth 2.
    circle = [[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5]]
    shp = adacpp.cad.build_extruded_area_solid(
        circle, [], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0, False
    )
    bb = tuple(round(v, 6) for v in adacpp.cad.bbox(shp))
    assert bb == (-0.5, -0.5, 0.0, 0.5, 0.5, 2.0)
    # Open lateral surface: a single cylindrical face, no caps.
    assert len(adacpp.cad.faces(shp)) == 1


def test_cad_revolved_curve_profile():
    # Revolve a circle wire (r=0.5, centered at x=2 in XY) a quarter turn about
    # the world Z axis through the origin → a curved pipe-elbow surface.
    circle = [[2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5]]
    shp = adacpp.cad.build_revolved_area_solid(
        circle, [], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 90.0, False
    )
    assert adacpp.cad.is_valid(shp)
    bb = adacpp.cad.bbox(shp)
    # Swept quarter-arc of a tube whose centerline radius is 2, tube radius 0.5.
    assert round(bb[3], 6) == 2.5  # +x extent = outer centerline + tube radius
    assert round(bb[4], 6) == 2.5  # +y extent after the 90° sweep


def _signed_mesh_volume(mesh):
    # Signed volume via the divergence theorem; positive iff triangles are
    # consistently wound outward.
    pos = list(mesh.positions)
    idx = list(mesh.indices)
    vol = 0.0
    for t in range(0, len(idx), 3):
        a, b, c = idx[t], idx[t + 1], idx[t + 2]
        ax, ay, az = pos[3 * a], pos[3 * a + 1], pos[3 * a + 2]
        bx, by, bz = pos[3 * b], pos[3 * b + 1], pos[3 * b + 2]
        cx, cy, cz = pos[3 * c], pos[3 * c + 1], pos[3 * c + 2]
        vol += (
            ax * (by * cz - bz * cy)
            - ay * (bx * cz - bz * cx)
            + az * (bx * cy - by * cx)
        )
    return vol / 6.0


def test_cad_tessellation_winding_is_outward():
    # A closed solid must tessellate with outward-facing triangles, i.e. a
    # positive signed mesh volume. Reversed faces (common in solids) need their
    # winding flipped — this guards that fix. A centered box has reversed faces.
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    mesh = adacpp.cad.tessellate(box, -1.0)
    assert _signed_mesh_volume(mesh) > 0.0


def test_cad_fixed_reference_swept_area_solid():
    # Sweep a square profile (in the XY plane at the directrix start) along a
    # straight directrix up +Z by 1.0 → a unit square prism of height 1.
    h = 0.2
    directrix = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]  # line (0,0,0)->(0,0,1)
    profile = [
        [0.0, 0.0, 0.0, 0.0, h, 0.0, 0.0],   # (0,0,0)->(h,0,0)
        [0.0, h, 0.0, 0.0, h, h, 0.0],       # (h,0,0)->(h,h,0)
        [0.0, h, h, 0.0, 0.0, h, 0.0],       # (h,h,0)->(0,h,0)
        [0.0, 0.0, h, 0.0, 0.0, 0.0, 0.0],   # (0,h,0)->(0,0,0)
    ]
    shp = adacpp.cad.build_fixed_reference_swept_area_solid(directrix, profile, [0.0, 0.0, 0.0])
    assert adacpp.cad.is_valid(shp)
    # Square h×h swept length 1 → volume h*h.
    assert round(adacpp.cad.volume(shp), 6) == round(h * h, 6)
    # Solid tessellates outward (positive signed volume).
    assert _signed_mesh_volume(adacpp.cad.tessellate(shp, -1.0)) > 0.0


def test_cad_make_halfspace_cuts_a_box():
    # A half-space at z=0 (solid below) cut from a centered 2³ box removes the
    # top half, leaving a planar cut face on z=0.
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    hs = adacpp.cad.make_halfspace([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], False)
    surfs = adacpp.cad.cut_surfaces(box, [hs], 1e-3, 1e-4)
    assert len(surfs) >= 1
    # Each surface tuple: (type, normal, outer_edges, outer_polyline, inners).
    planar = [s for s in surfs if s[0] == "Plane"]
    assert planar, "expected at least one planar cut face"
    surface_type, normal, outer_edges, outer_polyline, inners = planar[0]
    # The cut face lies on z=0.
    assert all(abs(p[2]) < 1e-6 for p in outer_polyline)
    assert len(outer_polyline) >= 3
    # Edges are classified (a box cut → straight Line edges).
    assert all(e[0] == "Line" for e in outer_edges)


def test_cad_obb_box():
    # Oriented bbox of a centered axis-aligned box: barycenter at origin,
    # half-sizes equal to the box half-extents (axes aligned to world).
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    center, half = adacpp.cad.obb(box)
    assert tuple(round(v, 6) for v in center) == (0.0, 0.0, 0.0)
    assert tuple(sorted(round(v, 6) for v in half)) == (1.0, 1.5, 2.0)


def test_cad_write_glb_bytes_box():
    """Box → glTF binary: header + non-empty body."""
    box = adacpp.cad.make_box(2.0, 3.0, 4.0)
    glb = adacpp.cad.write_glb_bytes(box)
    assert isinstance(glb, bytes)
    assert glb[:4] == b"glTF"
    # GLB version field is little-endian u32 right after the magic.
    assert int.from_bytes(glb[4:8], "little") == 2
    assert len(glb) > 200  # at least header + JSON chunk + BIN chunk


def test_cad_read_step_bytes(files_dir):
    """STEP bytes → ShapeHandle that has actual geometry (non-trivial bbox)."""
    data = (files_dir / "flat_plate_abaqus_10x10_m_wColors.stp").read_bytes()
    handle = adacpp.cad.read_step_bytes(data)
    assert type(handle).__name__ == "ShapeHandle"
    bb = adacpp.cad.bbox(handle)
    # The plate covers a noticeable area in X/Y; sanity-check it's non-empty.
    assert (bb[3] - bb[0]) > 1.0
    assert (bb[4] - bb[1]) > 1.0


def test_cad_step_to_glb_roundtrip(files_dir):
    """End-to-end: STEP bytes → ShapeHandle → glTF bytes."""
    data = (files_dir / "flat_plate_abaqus_10x10_m_wColors.stp").read_bytes()
    handle = adacpp.cad.read_step_bytes(data)
    glb = adacpp.cad.write_glb_bytes(handle)
    assert glb[:4] == b"glTF"
    assert len(glb) > 1000  # converted geometry produces a reasonable file


def test_cad_from_topods_pointer():
    """Native interop: round-trip an adacpp.cadit.occt.TopoDS_Shape into cad."""
    if adacpp.cad.from_topods_pointer is None:
        pytest.skip("from_topods_pointer is native-only")

    # Create a TopoDS_Shape on the C++ side via the existing cadit path, then
    # expose its address (cadit.occt.TopoDS_Shape stores a shared_ptr<TopoDS_Shape>
    # whose underlying pointer is the .get_ptr() — but we already have a Python
    # path that goes the other way. Easier: feed write_box_to_step a temporary
    # path so we know that path lights up the OCCT side, then build via pyocc.
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.gp import gp_Pnt

    pyocc_shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 1.0, 2.0, 3.0).Shape()
    handle = adacpp.cad.from_topods_pointer(int(pyocc_shape.this))

    mesh = adacpp.cad.tessellate(handle)
    positions = list(mesh.positions)
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    # Corner at origin, dims 1×2×3.
    assert min(xs) == 0.0 and max(xs) == 1.0
    assert min(ys) == 0.0 and max(ys) == 2.0
    assert min(zs) == 0.0 and max(zs) == 3.0


# --- shape-algebra verbs (mirror adapy's CadBackend) ---


def test_cad_transform_translates_box():
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)  # centered: [-1, 1]^3
    # top 3 rows of a translation-by-(1,1,1) 4x4, row-major
    moved = adacpp.cad.transform(box, [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1], True)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(moved)) == (0.0, 0.0, 0.0, 2.0, 2.0, 2.0)


def test_cad_boolean_ops():
    a = adacpp.cad.make_box(2.0, 2.0, 2.0)  # [-1, 1]^3
    b = adacpp.cad.transform(adacpp.cad.make_box(2.0, 2.0, 2.0), [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1], True)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(adacpp.cad.boolean("UNION", a, b))) == (-1, -1, -1, 2, 2, 2)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(adacpp.cad.boolean("INTERSECTION", a, b))) == (0, 0, 0, 1, 1, 1)
    # difference is non-empty and bounded by a
    assert adacpp.cad.is_valid(adacpp.cad.boolean("DIFFERENCE", a, b))
    with pytest.raises(Exception):
        adacpp.cad.boolean("NOPE", a, b)


def test_cad_distance():
    a = adacpp.cad.make_box(2.0, 2.0, 2.0)  # ends at x=1
    b = adacpp.cad.transform(adacpp.cad.make_box(2.0, 2.0, 2.0), [1, 0, 0, 5, 0, 1, 0, 0, 0, 0, 1, 0], True)  # x in [4,6]
    assert round(adacpp.cad.distance(a, b), 6) == 3.0


def test_cad_is_valid_and_serialize():
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    assert adacpp.cad.is_valid(box) is True
    s = adacpp.cad.serialize(box)
    assert isinstance(s, str) and "CASCADE Topology" in s[:40] and len(s) > 100


def test_cad_faces_and_vertex_points():
    box = adacpp.cad.make_box(1.0, 1.0, 1.0)
    faces = adacpp.cad.faces(box)
    assert len(faces) == 6
    assert all(type(f).__name__ == "ShapeHandle" for f in faces)
    pts = adacpp.cad.vertex_points(box)
    assert len(pts) == 8  # unique vertices
    assert all(len(p) == 3 for p in pts)


def test_cad_face_plane():
    box = adacpp.cad.make_box(2.0, 2.0, 2.0)
    faces = adacpp.cad.faces(box)
    origin, normal = adacpp.cad.face_plane(faces[0])
    assert len(origin) == 3 and len(normal) == 3
    # a box face normal is axis-aligned (unit on one component)
    assert round(sum(c * c for c in normal), 6) == 1.0
    # a solid (non-face) shape returns None
    assert adacpp.cad.face_plane(box) is None


def test_cad_volume():
    # make_box(2,3,4) is centered; volume = 24
    assert round(adacpp.cad.volume(adacpp.cad.make_box(2.0, 3.0, 4.0)), 6) == 24.0


def test_cad_build_extruded_area_solid():
    # A unit square in XY (4 line edges), extruded +Z by 2 → 1×1×2 box.
    square = [
        [0.0, 0, 0, 0, 1, 0, 0],
        [0.0, 1, 0, 0, 1, 1, 0],
        [0.0, 1, 1, 0, 0, 1, 0],
        [0.0, 0, 1, 0, 0, 0, 0],
    ]
    solid = adacpp.cad.build_extruded_area_solid(square, [], [0, 0, 0], [0, 0, 1], [1, 0, 0], 2.0)
    assert tuple(round(v, 6) for v in adacpp.cad.bbox(solid)) == (0.0, 0.0, 0.0, 1.0, 1.0, 2.0)
    assert round(adacpp.cad.volume(solid), 6) == 2.0

    # Same outer with a 0.5×0.5 inner void at the centre → volume 2 - 0.25*2 = 1.5.
    inner = [
        [0.0, 0.25, 0.25, 0, 0.75, 0.25, 0],
        [0.0, 0.75, 0.25, 0, 0.75, 0.75, 0],
        [0.0, 0.75, 0.75, 0, 0.25, 0.75, 0],
        [0.0, 0.25, 0.75, 0, 0.25, 0.25, 0],
    ]
    hollow = adacpp.cad.build_extruded_area_solid(square, [inner], [0, 0, 0], [0, 0, 1], [1, 0, 0], 2.0)
    assert round(adacpp.cad.volume(hollow), 6) == 1.5
