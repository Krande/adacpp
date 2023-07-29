//
// Created by Kristoffer on 29/07/2023.
//

#include <vector>
#include <memory>
#include "TessellateFactory.h"
#include "../models/Mesh.h"
#include "ShapeTesselator.h"


// a geometry store that can batch tessellate shapes to meshes using different tessellation algorithms
TessellateFactory::TessellateFactory(TessellationAlgorithm algorithm) {
    this->algorithm = algorithm;

}






