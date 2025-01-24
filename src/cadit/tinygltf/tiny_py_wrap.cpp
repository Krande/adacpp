#include "tiny_py_wrap.h"
#include "tiny_helpers.h"


void tiny_gltf_module(nb::module_ &m) {
    m.def("write_mesh_to_gltf", &write_to_gltf, "filename"_a, "mesh"_a, "Write a Mesh to GLTF");
    m.def("write_boxes_to_gltf", &write_boxes_to_gltf, "filename"_a, "box_origins"_a, "box_dims"_a, "Write a list of boxes to GLTF");
}
