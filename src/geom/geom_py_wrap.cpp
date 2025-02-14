#include "geom_py_wrap.h"
#include "geometries.h"

void shape_module(nb::module_ &m) {
    nb::class_<Shape>(m, "Shape")
            .def_ro("id", &Shape::id);

    nb::class_<Box, Shape>(m, "Box")
            .def(nb::init<std::vector<double>, double, double, double>())
            .def_rw("origin", &Box::origin)
            .def_rw("width", &Box::width)
            .def_rw("length", &Box::length)
            .def_rw("height", &Box::height);
};