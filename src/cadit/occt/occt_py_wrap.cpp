//
// Created by Kristoffer on 24.01.2025.
//


#include <TDF_Label.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Shape.hxx>
#include <Quantity_Color.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include "step_to_glb.h"
#include "step_writer.h"
#include "occt_py_wrap.h"

#include <TopoDS_Solid.hxx>

#include "colors.h"
#include <nanobind/stl/shared_ptr.h>

template <typename T>
std::shared_ptr<T> from_ptr(uintptr_t ptr) {
    return std::shared_ptr<T>(reinterpret_cast<T *>(ptr), [](T *p) {
        // Custom deleter or leave empty if you don't want to delete the object
    });
}

// Main function or other appropriate entry point
void step_to_glb_module(nb::module_& m)
{
    m.def("stp_to_glb", &stp_to_glb, "stp_file"_a, "glb_file"_a, "linearDeflection"_a = 0.1,
        "angularDeflection"_a = 0.5, "relativeDeflection"_a = false,
          "Convert a step file to glb file");
}

// add nanobind wrapping to the function
void occt_module(nb::module_ &m) {
    m.def("setInstanceColorIfAvailable", &setInstanceColorIfAvailable, "color_tool"_a, "lab"_a, "shape"_a, "c"_a,
          "sets a color");

    // Usage for TopoDS_Shape and its derived classes
    nb::class_<TopoDS_Shape>(m, "TopoDS_Shape")
        .def_static("from_ptr", &from_ptr<TopoDS_Shape>);

    nb::class_<TopoDS_Shell, TopoDS_Shape>(m, "TopoDS_Shell")
        .def_static("from_ptr", &from_ptr<TopoDS_Shell>);

    nb::class_<TopoDS_Solid, TopoDS_Shape>(m, "TopoDS_Solid")
        .def_static("from_ptr", &from_ptr<TopoDS_Solid>);

    // XCAFDoc_ColorTool
    nb::class_<XCAFDoc_ColorTool>(m, "XCAFDoc_ColorTool")
            .def("get_ptr", [](XCAFDoc_ColorTool &self) {
                return reinterpret_cast<uintptr_t>(&self);
            })
            .def_static("from_ptr", [](const uintptr_t ptr) {
                return reinterpret_cast<XCAFDoc_ColorTool *>(ptr);
            });

    // TDF_Label
    nb::class_<TDF_Label>(m, "TDF_Label")
            .def("get_ptr", [](TDF_Label &self) {
                return reinterpret_cast<uintptr_t>(&self);
            })
            .def_static("from_ptr", [](const uintptr_t ptr) {
                return reinterpret_cast<TDF_Label *>(ptr);
            });

    // Quantity_Color
    nb::class_<Quantity_Color>(m, "Quantity_Color")
            .def("get_ptr", [](Quantity_Color &self) {
                return reinterpret_cast<uintptr_t>(&self);
            })
            .def_static("from_ptr", [](const uintptr_t ptr) {
                return reinterpret_cast<Quantity_Color *>(ptr);
            });
}


void step_writer_module(nb::module_ &m) {
    m.def("write_box_to_step", &write_box_to_step, "filename"_a, "box_origin"_a, "box_dims"_a,
          "Write a box to a step file");
    m.def("write_boxes_to_step", &write_boxes_to_step, "filename"_a, "box_origins"_a, "box_dims"_a,
          "Write a list of boxes to a step file");
    m.def("step_writer_to_string", &step_writer_to_string, "writer"_a);

    nb::class_<STEPCAFControl_Writer>(m, "STEPCAFControl_Writer")
            .def_static("from_ptr", [](uintptr_t ptr) {
                // Return a shared_ptr to manage the lifetime of the object
                return std::shared_ptr<STEPCAFControl_Writer>(
                        reinterpret_cast<STEPCAFControl_Writer *>(ptr),
                        [](STEPCAFControl_Writer *p) {
                            // Custom deleter (can be empty if you want to avoid deletion)
                            // But in most cases, deleting the object should be safe if it’s properly managed
                            // delete p;
                            // Chose to not delete because this object will always grab the ptr from an existing
                            // swig object and should not be responsible for deleting it
                        }
                );
            });
}
