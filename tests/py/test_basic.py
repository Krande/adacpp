import random

import adacpp


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
