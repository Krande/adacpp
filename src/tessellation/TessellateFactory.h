//
// Created by Kristoffer on 29/07/2023.
//

#ifndef ADA_CPP_TESSELLATEFACTORY_H
#define ADA_CPP_TESSELLATEFACTORY_H
#include "../binding_core.h"

// Enum for different tessellation algorithms
enum class TessellationAlgorithm {
    OCCT_DEFAULT = 0, // Converts to OCC TopoDS_Shape and uses BRepMesh_IncrementalMesh
    CGAL_DEFAULT = 1, // Uses CGAL's polyhedron_3

};

class TessellateFactory {
public:
    explicit TessellateFactory(TessellationAlgorithm algorithm = static_cast<TessellationAlgorithm>(0));

    TessellationAlgorithm algorithm;
};

nanobind::class_<TessellateFactory> tess_module(nb::module_ &m);

#endif //ADA_CPP_TESSELLATEFACTORY_H
