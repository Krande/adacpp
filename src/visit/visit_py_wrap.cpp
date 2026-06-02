//
// Created by ofskrand on 24.01.2025.
//
#include "../geom/Mesh.h"
#include "tess_helpers.h"
#include "visit_py_wrap.h"
#include "TessellateFactory.h"

void tess_helper_module(nb::module_ &m) {
    // Mesh / MeshType / Color / GroupReference are registered in adacpp.cad.
    // visit only binds visit-specific functions; types are looked up from the
    // already-registered cad bindings at call time.
    m.def("get_box_mesh", &get_box_mesh, "box_origin"_a, "box_dims"_a, "Write a box to a step file");
}

void tess_module(nb::module_ &m) {
    nb::enum_<TessellationAlgorithm>(m, "TessellationAlgorithm")
            .value("OCCT_DEFAULT", TessellationAlgorithm::OCCT_DEFAULT,
                   "Converts to OCC TopoDS_Shape and uses BRepMesh_IncrementalMesh")
            .value("CGAL_DEFAULT", TessellationAlgorithm::CGAL_DEFAULT, "Uses CGAL's polyhedron_3");

    nb::class_<TessellateFactory>(m, "TessellateFactory")
            .def(nb::init<>())
            .def(nb::init<const TessellationAlgorithm &>())
            .def("tessellate", &TessellateFactory::tessellate)
            .def_rw("algorithm", &TessellateFactory::algorithm, "Tessellation algorithm");

}
