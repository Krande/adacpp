//
// Created by ofskrand on 24.01.2025.
//
#include "tess_py_wrap.h"

void tess_helper_module(nb::module_ &m) {
    m.def("get_box_mesh", &get_box_mesh, "box_origin"_a, "box_dims"_a, "Write a box to a step file");

    nb::class_<Mesh>(m, "Mesh")
            .def_ro("id", &Mesh::id, "The id of the mesh")
            .def_ro("positions", &Mesh::positions, "The positions of the mesh")
            .def_ro("indices", &Mesh::indices, "The indices of the mesh")
            .def_ro("normals", &Mesh::normals, "The normals of the mesh")
            .def_ro("mesh_type", &Mesh::mesh_type, "The type of mesh", nb::enum_<MeshType>(m, "MeshType"))
            .def_ro("color", &Mesh::color, "The color of the mesh")
            .def_ro("groups", &Mesh::group_reference, "The groups of the mesh",
                    nb::class_<GroupReference>(m, "GroupReference"));

    nb::class_<Color>(m, "Color")
            .def_rw("r", &Color::r)
            .def_rw("g", &Color::g)
            .def_rw("b", &Color::b)
            .def_rw("a", &Color::a);
}