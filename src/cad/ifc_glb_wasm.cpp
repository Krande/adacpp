// embind entry for the no-pyodide native IFC -> GLB wasm module.
//
// STANDALONE wasm module: the OCC-free, dep-free native IFC pipeline (IfcResolver Part-21 reader +
// libtess2 + meshoptimizer + GLB writer) compiled with emscripten + embind — NO pyodide, NO Python,
// NO OCCT, NO ifcopenshell, NO nanobind. The IFC counterpart of adacpp_step_glb (cad_wasm.cpp); the
// two share the neutral-geometry tessellation + GLB stack and differ only in the front-end reader.
//
// Three verbs, one reader: `ifcToGlb` tessellates the file and `scanMembers` reports the members it
// states (sections, axes, outlines, materials) as JSONL -- the semantic half a clash check or a
// quantity take-off needs, with no tessellation and no server; and `clashJoints` runs the whole
// beam-to-beam joint search over that scan in C++, so an interactive clash check needs no Python
// runtime in the browser at all.
//
// IO model mirrors the STEP module: both `inPath` and `outPath` live in the emscripten file system.
// In a worker, `await Module.opfsMount("/opfs")` (src/wasmio/opfs_sync.js) and pass paths under it so
// a large IFC streams through `pread` (bounded RSS) and the GLB is written back to OPFS. `spillDir` is
// a writable directory (under the OPFS mount, or in-heap) for the per-material spill lanes; pass an
// explicit one (the C++ default mkdtemp("/tmp/...") isn't reliable under WASMFS).
//
// Single-threaded (stream_ifc_to_glb uses threads=1), so this links WITHOUT -pthread and runs on any
// page (no SharedArrayBuffer / cross-origin isolation required).

#include <string>

#include <emscripten/bind.h>

#include "ifc_clash.h"
#include "ifc_member_scan.h"
#include "ifc_to_glb_stream.h"
#if defined(ADACPP_WASM_SHARDS)
#include "ifc_glb_shard.h"
#endif

namespace {

// Returns the triangle count written, or -1 on error. deflection/angular_deg are the libtess2 chordal
// + angular tolerances (2.0 / 20.0 are the adapy production defaults). The GLB is baked to metres via
// the IFC file's unit scale (viewer default), like the native/nanobind path.
long ifc_to_glb(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                double deflection, double angular_deg, bool meshopt) {
#if defined(__EMSCRIPTEN_PTHREADS__)
    // Pin ONE thread: the auto count (navigator.hardwareConcurrency) could exceed the pthread pool,
    // and a blocked caller can't grow it. ifcToGlbThreads is the threaded verb.
    return adacpp::stream_ifc_to_glb(in_path, out_path, deflection, angular_deg, meshopt, spill_dir, 0.0, 1);
#else
    return adacpp::stream_ifc_to_glb(in_path, out_path, deflection, angular_deg, meshopt, spill_dir);
#endif
}

#if defined(__EMSCRIPTEN_PTHREADS__)
// The pthread tier (ADACPP_WASM_PTHREADS builds; cross-origin-isolated page, called off the main
// thread, `threads` <= PTHREAD_POOL_SIZE -- see stepToGlbThreads).
long ifc_to_glb_threads(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                        double deflection, double angular_deg, bool meshopt, int threads) {
    return adacpp::stream_ifc_to_glb(in_path, out_path, deflection, angular_deg, meshopt, spill_dir, 0.0,
                                     threads < 1 ? 1 : threads);
}
#endif

// Scan the IFC's MEMBERS -- beams and plates, their sections, axes, outlines, placements and
// materials -- and write them to `out_path` as JSONL (see ifc_member_scan.h for the record shape).
// Returns the number of members written, or -1 on error.
//
// This is the SEMANTIC half of the same file the ->GLB verb tessellates, and the reader is already
// linked into this module: `ifcToGlb` needs IfcResolver to find the geometry, and the members are
// what it walks to get there. So a browser gets a clash check for the cost of an embind export
// rather than a second pipeline -- and, unlike the ->GLB verb, this one never tessellates anything.
//
// JSONL through a file rather than objects through embind because a plant has hundreds of thousands
// of members: one JS object per member would cross the boundary once per member, while a file keeps
// the scan's streaming property on both sides. Point `out_path` at OPFS and nothing has to fit the
// wasm heap.
long ifc_scan_members(const std::string &in_path, const std::string &out_path) {
    return adacpp::ifc_read::write_members_jsonl(in_path, out_path);
}

// A whole clash check -- read the IFC, find the beam-to-beam joints, classify them -- written to
// `out_path` as JSON. Returns the joint count, or -1 on error.
//
// One pass and one language: the members never materialise as JSON on the way, because the reader
// and the joint finder are both here. This is why the browser does not need a Python runtime for
// it -- the rules ARE the compiled ones the worker runs, through the same header.
long ifc_clash_joints(const std::string &in_path, const std::string &out_path, double out_of_plane_tol,
                      double point_tol) {
    return adacpp::ifc_read::write_beam_joints_json(in_path, out_path, out_of_plane_tol, point_tol);
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_ifc_glb) {
    emscripten::function("ifcToGlb", &ifc_to_glb);
    emscripten::function("scanMembers", &ifc_scan_members);
    emscripten::function("clashJoints", &ifc_clash_joints);
#if defined(__EMSCRIPTEN_PTHREADS__)
    emscripten::function("ifcToGlbThreads", &ifc_to_glb_threads);
#endif
#if defined(ADACPP_WASM_SHARDS)
    // N-worker fallback (no SharedArrayBuffer): see ifc_glb_shard.h for the protocol.
    emscripten::class_<adacpp::IfcGlbShard>("IfcGlbShard")
        .constructor<const std::string &, double, double>()
        .function("rootCount", &adacpp::IfcGlbShard::root_count)
        .function("process", &adacpp::IfcGlbShard::process)
        .function("persist", &adacpp::IfcGlbShard::persist);
    emscripten::function("mergeGlbLanes", &adacpp::merge_ifc_glb_lanes);
#endif
}
