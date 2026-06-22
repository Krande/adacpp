// NGEOM libtess2 tessellator — port of step2glb crates/core/src/tessellate.rs onto the
// neutral geometry layer. OCC-free: all surface evaluation goes through the neutral types.
// Spec: dap/plan/v3/notes_libtess2_and_geom_streaming.md (Part A).
#pragma once

#include <cstdint>
#include <vector>

#include "ngeom_topology.h"

namespace adacpp::ngeom {

struct TessParams {
    double deflection = 0.0;   // chord tolerance for boundary discretization (0 => auto)
    double max_angle = 0.35;   // ~20 deg, max turn between boundary samples
};

struct TessMesh {
    std::vector<float> positions;   // flat xyz (3 per vertex)
    std::vector<uint32_t> indices;  // flat (3 per triangle)
    std::vector<float> normals;     // flat xyz (3 per vertex, parallel to positions)
    // [first_index, index_count) ranges per source root, for batched grouping
    struct Group {
        std::string id;
        uint32_t first_index = 0;
        uint32_t index_count = 0;
    };
    std::vector<Group> groups;
};

// Tessellate one neutral face, appending into `out`. Returns true if it produced triangles.
bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &out);

// Tessellate a whole decoded document (all roots), one TessMesh with a Group per root.
TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp);

}  // namespace adacpp::ngeom
