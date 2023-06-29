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
    adacpp.write_box_to_step("mybox.stp", (0, 0, 0), (10, 10, 10))


def test_basic_write_list_of_boxes_to_step():
    origins, dimensions = create_box_grid(10)
    adacpp.write_boxes_to_step("myboxes.stp", origins, dimensions)


def test_basic_mesh():
    mesh = adacpp.get_box_mesh((0, 0, 0), (10, 10, 10))
    assert mesh.__class__.__name__ == "Mesh"
    assert hasattr(mesh, "positions")
    assert hasattr(mesh, "normals")
    assert hasattr(mesh, "indices")
    print(mesh)
