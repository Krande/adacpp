#ifndef ADA_CPP_TESSELLATEFACTORY_H
#define ADA_CPP_TESSELLATEFACTORY_H

#include <iostream>

#if defined(__unix__) || defined(__unix)
#include <cstdint>
#endif


// Enum for different tessellation algorithms
enum class TessellationAlgorithm : uint32_t
{
    OCCT_DEFAULT = 0, // Converts to OCC TopoDS_Shape and uses BRepMesh_IncrementalMesh
    CGAL_DEFAULT = 1, // Uses CGAL's polyhedron_3
};

class TessellateFactory
{
public:
    explicit TessellateFactory(TessellationAlgorithm algorithm = static_cast<TessellationAlgorithm>(0));

    TessellationAlgorithm algorithm;

    void tessellate()
    {
        // print to console which algorithm is used
        std::cout << "Tessellating using algorithm: " << static_cast<int>(algorithm) << std::endl;
    }
};


#endif //ADA_CPP_TESSELLATEFACTORY_H
