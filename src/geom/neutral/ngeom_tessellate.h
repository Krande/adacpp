// NGEOM libtess2 tessellator — OCC-free: all surface evaluation goes through the neutral types.
// Spec: dap/plan/v3/notes_libtess2_and_geom_streaming.md (Part A).
//
// ATTRIBUTION: this whole NGEOM libtess2 path is a C++ port of step2glb
// (https://github.com/vegarringdal/step2glb), an open-source (MIT) Rust STEP->GLB
// converter. Comments throughout this path cite the original Rust module/function
// names (tessellate.rs, geom.rs, model.rs, mesh.rs, ...) purely as provenance for
// the algorithm being mirrored.
#pragma once

#include <cstdint>
#include <vector>

#include "ngeom_topology.h"

namespace adacpp::ngeom {

struct TessParams {
    double deflection = 0.0; // chord tolerance for boundary discretization (0 => auto)
    double max_angle = 0.35; // ~20 deg, max turn between boundary samples
};

struct TessMesh {
    std::vector<float> positions;  // flat xyz (3 per vertex)
    std::vector<uint32_t> indices; // flat (3 per triangle)
    std::vector<float> normals;    // flat xyz (3 per vertex, parallel to positions)
    // per source root: triangle-index range + vertex range (maps to adacpp GroupReference)
    struct Group {
        std::string id;
        uint32_t first_index = 0;  // flat index into `indices`
        uint32_t index_count = 0;  // number of indices (3 * triangles)
        uint32_t first_vertex = 0; // flat vertex index into `positions`/3
        uint32_t vertex_count = 0;
    };
    std::vector<Group> groups;
};

// Tessellate one neutral face, appending into `out`. Returns true if it produced triangles.
bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &out);

// Tessellate a whole decoded document (all roots), one TessMesh with a Group per root.
TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp);

} // namespace adacpp::ngeom
