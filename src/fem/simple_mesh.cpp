//
// Created by Kristoffer on 27/07/2023.
//
#include "simple_mesh.h"
#include <gmsh.h>
#include "../binding_core.h"

void simple_gmesh(std::string filename) {
    gmsh::initialize();
    gmsh::model::add("square");
    gmsh::model::geo::addPoint(0, 0, 0, 0.1, 1);
    gmsh::model::geo::addPoint(1, 0, 0, 0.1, 2);
    gmsh::model::geo::addPoint(1, 1, 0, 0.1, 3);
    gmsh::model::geo::addPoint(0, 1, 0, 0.1, 4);
    gmsh::model::geo::addLine(1, 2, 1);
    gmsh::model::geo::addLine(2, 3, 2);
    gmsh::model::geo::addLine(3, 4, 3);

    // try automatic assignement of tag
    int line4 = gmsh::model::geo::addLine(4, 1);
    gmsh::model::geo::addCurveLoop({1, 2, 3, line4}, 1);
    gmsh::model::geo::addPlaneSurface({1}, 6);
    gmsh::model::geo::synchronize();
    gmsh::model::mesh::generate(2);
    gmsh::write(filename);
};

void gmsh_module(nb::module_ &m) {
    m.def("create_gmesh", &simple_gmesh, "filename"_a, "Write a Mesh to GLTF");
}