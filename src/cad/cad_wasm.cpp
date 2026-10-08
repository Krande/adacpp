// embind entry for the no-pyodide native STEP -> GLB wasm module.
//
// This is a STANDALONE wasm module: the OCC-free native pipeline (Part-21 reader + libtess2 +
// meshoptimizer + GLB writer) compiled with emscripten + embind — NO pyodide, NO Python, NO OCCT,
// NO nanobind. It is the lightweight counterpart to the pyodide nanobind module.
//
// IO model: both `inPath` and `outPath` live in the emscripten file system (WASMFS). In a worker,
// `await Module.opfsMount("/opfs")` (src/wasmio/opfs_sync.js) and pass paths under it so a multi-GB
// STEP streams through `pread` (bounded RSS) and the GLB is written back to OPFS — none of it has to
// fit in the wasm heap. `spillDir` is a writable directory (under the OPFS mount, or in-heap) for the
// per-material spill lanes.
//
// Single-threaded: step_to_glb_single spawns no std::thread, so this links WITHOUT -pthread and runs
// on any page (no SharedArrayBuffer / cross-origin isolation required). A -pthread variant can come
// later as a perf tier.

#include <string>

#include <emscripten/bind.h>

#include "step_to_glb_st.h"
#if defined(ADACPP_WASM_SHARDS)
#include "step_glb_shard.h"
#endif
#if defined(__EMSCRIPTEN_PTHREADS__)
#include "step_to_glb_stream.h"
#endif

namespace {

// Returns the triangle count written, or -1 on I/O error. deflection/angular_deg are the libtess2
// chordal + angular tolerances (2.0 / 20.0 are the adapy production defaults).
long step_to_glb(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                 double deflection, double angular_deg, bool meshopt) {
    return adacpp::step_to_glb_single(in_path, out_path, spill_dir, deflection, angular_deg, meshopt);
}

#if defined(__EMSCRIPTEN_PTHREADS__)
// The pthread tier (ADACPP_WASM_PTHREADS builds only; needs a cross-origin-isolated page): the native
// server's threaded converter, `threads` workers sharing ONE wasm memory. Must be called off the
// browser main thread (it blocks in join), and `threads` must not exceed the module's
// PTHREAD_POOL_SIZE -- a pool worker can only be spawned by returning to the event loop, which a
// blocked caller never does. Returns the solids written, or -1 on error.
long step_to_glb_threads(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                         double deflection, double angular_deg, bool meshopt, int threads) {
    return adacpp::stream_step_to_glb(in_path, out_path, deflection, angular_deg, threads, meshopt, spill_dir);
}
#endif

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_step_glb) {
    emscripten::function("stepToGlb", &step_to_glb);
#if defined(__EMSCRIPTEN_PTHREADS__)
    emscripten::function("stepToGlbThreads", &step_to_glb_threads);
#endif
#if defined(ADACPP_WASM_SHARDS)
    // N-worker fallback (no SharedArrayBuffer): see step_glb_shard.h for the protocol.
    emscripten::class_<adacpp::StepGlbShard>("StepGlbShard")
        .constructor<const std::string &, double, double>()
        .constructor<const std::string &, const std::string &, double, double>()
        .class_function("prepare", &adacpp::StepGlbShard::prepare)
        .function("rootCount", &adacpp::StepGlbShard::root_count)
        .function("process", &adacpp::StepGlbShard::process)
        .function("hugeCount", &adacpp::StepGlbShard::huge_count)
        .function("hugeFaces", &adacpp::StepGlbShard::huge_faces)
        .function("processHuge", &adacpp::StepGlbShard::process_huge)
        .function("assembleHuge", &adacpp::StepGlbShard::assemble_huge)
        .function("persist", &adacpp::StepGlbShard::persist);
    emscripten::function("mergeGlbLanes", &adacpp::merge_step_glb_lanes);
#endif
}
