#pragma once

#include "../binding_core.h"

// adacpp.clash: convex-contact clash detection on coal (convex_contact.h). OCC-free, so it is
// registered on every build, the pyodide wheel included.
void clash_module(nb::module_ &m);
