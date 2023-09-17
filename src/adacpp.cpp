#include "binding_core.h"
#include "exchange/occt/step_writer.h"
#include "exchange/occt/colors.h"
#include "tessellation/tess_helpers.h"
#include "exchange/tinygltf/tiny.h"
#include "fem/simple_mesh.h"
#include "tessellation/TessellateFactory.h"
#include "models/geometries.h"

// Define the modules that will be exposed in python
NB_MODULE(_ada_cpp_ext_impl, m) {
    shape_module(m);
    occt_color_module(m);
    step_writer_module(m);
    tess_helper_module(m);
    tiny_gltf_module(m);
    gmsh_module(m);
    tess_module(m);
}