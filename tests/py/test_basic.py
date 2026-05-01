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
