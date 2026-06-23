#pragma once
#include <cstdint>
#include <vector>

namespace ngeom {

struct SimplifiedMesh {
    std::vector<float> positions;  // xyz interleaved
    std::vector<uint32_t> indices;
};

// Faithful port of step2glb mesh.rs simplify_meshopt: meshopt_simplify toward
// threshold*index_count within target_error, border LOCKED (shared seams keep shape), then drop
// degenerate triangles and compact to the surviving vertices. With target_error 0.0 this is a
// lossless coplanar-triangle collapse (the cleanup step2glb's merged GLB applies, ~16% on the crane).
SimplifiedMesh meshopt_simplify_mesh(const std::vector<float> &positions,
                                     const std::vector<uint32_t> &indices, float threshold,
                                     float target_error);

}  // namespace ngeom
