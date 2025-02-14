//
// Created by Kristoffer on 24.01.2025.
//

#include "ifc_py_wrap.h"
#include "ifcop.h"

void ifc_module(nb::module_ &m) {
    m.def("read_ifc_file", &read_ifc_file, "file_name"_a, "Read an ifc file");
}