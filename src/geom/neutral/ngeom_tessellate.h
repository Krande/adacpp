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

#include "../MeshType.h"
#include "ngeom_tess_track.h"
#include "ngeom_topology.h"

namespace adacpp::ngeom {

struct TessParams {
    double deflection = 0.0; // chord tolerance for boundary discretization (0 => auto)
    double max_angle = 0.35; // ~20 deg, max turn between boundary samples
    int threads = 1;         // >1 tessellates a root's faces across a thread pool. Default 1
                             // (serial) so callers already parallelising across roots/solids
                             // (the STEP->GLB process pool) don't oversubscribe; a single
                             // whole-model call (merge-preview generate) opts into all cores.
    bool weld = true;         // weld coincident vertices + rebuild a shared index buffer with crease-angle
                              // smooth normals (ngeom_weld.h) per root, turning the flat-shaded triangle
                              // soup into a compact indexed mesh (matches OCC density). Off => raw soup.
    double model_scale = 0.0; // model bbox diagonal (world units). 0 => OFF: the fixed max_angle
                              // governs every surface (explicit-global-angle mode). >0 => ADAPTIVE:
                              // the angular ceiling is relaxed for surfaces whose radius is small
                              // relative to model_scale (imperceptible facets), so dense assemblies
                              // of tiny curved features (bolts/pins) don't blow the triangle budget
                              // while large visible surfaces keep the fine max_angle.
    bool capture_face_ranges = false; // record per-face triangle ranges (TessMesh::face_ranges) so the
                                       // GLB writer can emit clickable per-face regions. Opt-in (bloats
                                       // the output) and forces serial face tessellation for stable
                                       // face-order ranges. Off => no per-face bookkeeping.
    // Which tessellation track runs (ngeom_tess_track.h), plus each track's own options. Defaults
    // are today's behaviour, byte-for-byte. Each opts struct is read only on its own track. All POD,
    // so TessParams stays trivially copyable for the per-root/per-face copies.
    TessTrack track = TessTrack::Libtess2;
    Libtess2Opts libtess2;
    CdtOpts cdt;
    HybridOpts hybrid;
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
    // Per-face triangle range within `indices` — populated only when TessParams.capture_face_ranges
    // is set (clickable per-face regions). `first_index` is relative to this mesh; the GLB writer
    // re-bases it against the owning solid's draw range.
    struct FaceRange {
        uint32_t first_index = 0; // flat index into `indices`
        uint32_t index_count = 0; // number of indices (3 * triangles)
        int64_t face_id = 0;      // source entity id (STEP/IFC #id), 0 if unknown
        uint32_t face_seq = 0;    // 0-based face position within the solid
    };
    std::vector<FaceRange> face_ranges;
    // Primitive topology of `indices`: TRIANGLES for solids/faces, LINES for curve-only bodies
    // (index pairs). A single stream blob is one root, so it is homogeneous.
    MeshType mesh_type = MeshType::TRIANGLES;
};

// Tessellate one neutral face, appending into `out`. Returns true if it produced triangles.
bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &out);

// Per-conversion tessellation-health counters (thread-safe across the face/root pools): a face that
// carries a real trim boundary but fails to produce triangles is silently DROPPED geometry — the class
// of bug that otherwise only shows up on visual inspection. Reset before a conversion, read after, so
// the streaming entry points can report a dropped-face count for the audit to flag.
std::uint64_t tess_dropped_faces();
std::uint64_t tess_total_faces();
void reset_tess_face_stats();

// Tessellate a whole decoded document (all roots), one TessMesh with a Group per root.
TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp);

} // namespace adacpp::ngeom
