#include "binding_core.h"
#include "cad/cad_py_wrap.h"

#include <exception>
#include <string>

#include <Standard_Failure.hxx>

#ifndef __EMSCRIPTEN__
#include "cadit/occt/occt_py_wrap.h"
#include "cadit/ifc/ifc_py_wrap.h"
#include "cadit/tinygltf/tiny_py_wrap.h"
#include "fem/fem_py_wrap.h"
#include "geom/geom_py_wrap.h"
#include "visit/visit_py_wrap.h"
#endif

// Define the modules that will be exposed in python
NB_MODULE(_ada_cpp_ext_impl, m) {
    // OCCT's Standard_Failure derives from Standard_Transient, NOT std::exception,
    // so nanobind's built-in std::exception translator can't convert it: an OCCT
    // throw escaping a binding becomes an untranslatable SystemError
    // ("nanobind::detail::nb_func_error_except(): exception could not be
    // translated"). Register a translator so every adacpp verb surfaces OCCT
    // failures as a clean Python RuntimeError (with the OCCT type + message)
    // instead. Non-OCCT exceptions fall through to the next translator.
    nb::register_exception_translator([](const std::exception_ptr &p, void * /*payload*/) {
        try {
            std::rethrow_exception(p);
        } catch (const Standard_Failure &e) {
            const char *msg = e.GetMessageString();
            const std::string what = std::string("OCCT ") + e.DynamicType()->Name() + ": " + (msg ? msg : "");
            PyErr_SetString(PyExc_RuntimeError, what.c_str());
        }
    });

    auto cad_sub_module = m.def_submodule("cad", "Backend-agnostic CAD operations");
    cad_module(cad_sub_module);

#ifndef __EMSCRIPTEN__
    auto cadit_module = m.def_submodule("cadit", "CAD Interoperability toolkit");
    step_writer_module(cadit_module);
    tiny_gltf_module(cadit_module);
    auto ifc_submod = cadit_module.def_submodule("ifc", "IfcOpenShell interface toolkit");
    ifc_module(ifc_submod);
    auto occt_sub_module = cadit_module.def_submodule("occt", "Opencascade interface toolkit");
    occt_module(occt_sub_module);
    auto occt_conversion_module = cadit_module.def_submodule("conversion", "OCCT conversion module");
    step_to_glb_module(occt_conversion_module);
    auto visit_module = m.def_submodule("visit", "Visualization Interoperability toolkit");
    tess_helper_module(visit_module);
    tess_module(visit_module);
    auto fem_module = m.def_submodule("fem", "fem module");
    gmsh_module(fem_module);
    auto geom_module = m.def_submodule("geom", "Geometry module");
    shape_module(geom_module);
#endif
}
