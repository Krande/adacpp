// embind entry for the no-pyodide native STEP -> GLB wasm module.
//
// This is a STANDALONE wasm module: the OCC-free native pipeline (Part-21 reader + libtess2 +
// meshoptimizer + GLB writer) compiled with emscripten + embind — NO pyodide, NO Python, NO OCCT,
// NO nanobind. It is the lightweight counterpart to the pyodide nanobind module.
//
// IO model: both `inPath` and `outPath` live in the emscripten file system. Mount OPFS via WASMFS
// (build with -sWASMFS) and pass OPFS-backed paths so a multi-GB STEP streams through `pread`
// (bounded RSS) and the GLB is written back to OPFS — none of it has to fit in the wasm heap.
// `spillDir` is a writable directory (an OPFS or MEMFS mount) for the per-material spill lanes.
//
// Single-threaded: step_to_glb_single spawns no std::thread, so this links WITHOUT -pthread and runs
// on any page (no SharedArrayBuffer / cross-origin isolation required). A -pthread variant can come
// later as a perf tier.

#include <string>

#include <emscripten/bind.h>
#include <emscripten/wasmfs.h>

#include "step_to_glb_st.h"

namespace {

// Returns the triangle count written, or -1 on I/O error. deflection/angular_deg are the libtess2
// chordal + angular tolerances (2.0 / 20.0 are the adapy production defaults).
long step_to_glb(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                 double deflection, double angular_deg, bool meshopt) {
    return adacpp::step_to_glb_single(in_path, out_path, spill_dir, deflection, angular_deg, meshopt);
}

// Mount the browser's Origin Private File System (OPFS) at `mount_point`, so the STEP, the GLB and the
// spill lanes live in OPFS — the STEP streams through pread (bounded RSS) instead of the wasm heap.
// MUST be called from a Web Worker (OPFS sync access handles are worker-only). Returns 0 on success,
// non-zero if OPFS is unavailable; the caller then falls back to the in-heap WASMFS default.
int mount_opfs(const std::string &mount_point) {
    backend_t opfs = wasmfs_create_opfs_backend();
    if (!opfs)
        return -1;
    return wasmfs_create_directory(mount_point.c_str(), 0777, opfs);
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_step_glb) {
    emscripten::function("stepToGlb", &step_to_glb);
    emscripten::function("mountOpfs", &mount_opfs);
}
