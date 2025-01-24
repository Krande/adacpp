#include "TessellateFactory.h"

// a geometry store that can batch tessellate shapes to meshes using different tessellation algorithms
TessellateFactory::TessellateFactory(TessellationAlgorithm algorithm) {
    this->algorithm = algorithm;
}


