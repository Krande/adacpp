#include "fem_py_wrap.h"
#include "simple_mesh.h"

void gmsh_module(nb::module_ &m) {
    m.def("create_gmesh", &simple_gmesh, "filename"_a, "Write a Mesh to GLTF");
}