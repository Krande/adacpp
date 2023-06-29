#include "binding_core.h"
#include "exchange/step_writer.h"
#include "geom/tess_helpers.h"

// Define the modules that will be exposed in python
NB_MODULE(_ada_cpp_ext_impl, m) {
    step_writer_module(m);
    geom_module(m);
}