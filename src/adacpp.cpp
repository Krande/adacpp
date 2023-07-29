#include "binding_core.h"
#include "exchange/occt/step_writer.h"
#include "tessellation/tess_helpers.h"
#include "exchange/tinygltf/tiny.h"
#include "fem/simple_mesh.h"
#include "tessellation/TessellateFactory.h"


// Define the modules that will be exposed in python
NB_MODULE(_ada_cpp_ext_impl, m) {
    step_writer_module(m);
    geom_module(m);
    geom_module_color(m);
    tiny_gltf_module(m);
    gmsh_module(m);
    tess_module(m);
}