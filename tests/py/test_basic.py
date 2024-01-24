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


def test_basic_write_step():
    adacpp.cadit.write_box_to_step("mybox.stp", (0, 0, 0), (10, 10, 10))


def test_convert_step_to_glb(files_dir):
    adacpp.cadit.conversion.stp_to_glb(str(files_dir / "flat_plate_abaqus_10x10_m_wColors.stp"),
                                       str("temp/flat_plate_abaqus_10x10_m_wColors.glb"))


def test_basic_write_list_of_boxes_to_step():
    origins, dimensions = create_box_grid(10)
    adacpp.cadit.write_boxes_to_step("myboxes.stp", origins, dimensions)

    adacpp.cadit.conversion.stp_to_glb("myboxes.stp", "myboxes_convert.glb")


def test_basic_mesh():
    mesh = adacpp.visit.get_box_mesh((0, 0, 0), (10, 10, 10))
    assert mesh.__class__.__name__ == "Mesh"
    assert hasattr(mesh, "positions")
    assert hasattr(mesh, "normals")
    assert hasattr(mesh, "indices")

    adacpp.cadit.write_mesh_to_gltf("mybox.gltf", mesh)
    print(mesh)


def test_boxes_mesh_gltf():
    origins, dimensions = create_box_grid(10)
    adacpp.cadit.write_boxes_to_gltf("myboxes.glb", origins, dimensions)


def test_simple_gmsh():
    adacpp.fem.create_gmesh("my_fem_mesh.msh")


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
