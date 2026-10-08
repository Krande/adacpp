#pragma once

#include "../binding_core.h"

// adacpp.fea: the format-neutral FEA kernels (superposition, derivations, envelopes, AFBL/AFEL I/O).
// OCC-free, so it is registered on every build, the pyodide wheel included.
void fea_module(nb::module_ &m);
