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
// on any page (no SharedArrayBuffer / cross-origin isolation required). Several cores come from several
// workers instead, each with its own instance: `StepGlbShard` + `mergeGlbLanes` split one conversion
// across them through a shared (OPFS) directory -- see step_glb_shard.h.

#include <string>

#include <emscripten/bind.h>

#include "step_glb_shard.h"
#include "step_to_glb_st.h"

namespace {

// Returns the triangle count written, or -1 on I/O error. deflection/angular_deg are the libtess2
// chordal + angular tolerances (2.0 / 20.0 are the adapy production defaults).
long step_to_glb(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                 double deflection, double angular_deg, bool meshopt) {
    return adacpp::step_to_glb_single(in_path, out_path, spill_dir, deflection, angular_deg, meshopt);
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_step_glb) {
    emscripten::function("stepToGlb", &step_to_glb);
    // One conversion over N workers (no SharedArrayBuffer): see step_glb_shard.h for the protocol.
    emscripten::register_vector<long>("VectorLong");
    emscripten::class_<adacpp::StepGlbShard>("StepGlbShard")
        .constructor<const std::string &, const std::string &, double, double>()
        .class_function("prepare", &adacpp::StepGlbShard::prepare)
        .function("rootCount", &adacpp::StepGlbShard::root_count)
        .function("planBatches", &adacpp::StepGlbShard::plan_batches)
        .function("process", &adacpp::StepGlbShard::process)
        .function("hugeCount", &adacpp::StepGlbShard::huge_count)
        .function("hugeFaces", &adacpp::StepGlbShard::huge_faces)
        .function("processHuge", &adacpp::StepGlbShard::process_huge)
        .function("assembleHuge", &adacpp::StepGlbShard::assemble_huge)
        .function("persist", &adacpp::StepGlbShard::persist);
    emscripten::function("mergeGlbLanes", &adacpp::merge_step_glb_lanes);
}
