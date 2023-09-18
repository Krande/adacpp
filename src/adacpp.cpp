#include "binding_core.h"
#include "cadit/occt/step_writer.h"
#include "cadit/occt/colors.h"
#include "cadit/ifc/ifcop.h"
#include "cadit/tinygltf/tiny.h"
#include "fem/simple_mesh.h"
#include "geom/geometries.h"
#include "visit/TessellateFactory.h"
#include "visit/tess_helpers.h"

// Define the modules that will be exposed in python
NB_MODULE(_ada_cpp_ext_impl, m) {
    auto cadit_module = m.def_submodule("cadit", "CAD Interoperability toolkit");
        step_writer_module(cadit_module);
        tiny_gltf_module(cadit_module);
    auto ifc_submod = cadit_module.def_submodule("ifc", "IfcOpenShell interface toolkit");
        ifc_module(ifc_submod);
    auto occt_module = cadit_module.def_submodule("occt", "Opencascade interface toolkit");
        occt_color_module(occt_module);
    auto visit_module = m.def_submodule("visit", "Visualization Interoperability toolkit");
        tess_helper_module(visit_module);
        tess_module(visit_module);
    auto fem_module = m.def_submodule("fem", "fem module");
        gmsh_module(fem_module);
    auto geom_module = m.def_submodule("geom", "Geometry module");
        shape_module(geom_module);
}
