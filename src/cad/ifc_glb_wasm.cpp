// embind entry for the no-pyodide native IFC -> GLB wasm module.
//
// STANDALONE wasm module: the OCC-free, dep-free native IFC pipeline (IfcResolver Part-21 reader +
// libtess2 + meshoptimizer + GLB writer) compiled with emscripten + embind — NO pyodide, NO Python,
// NO OCCT, NO ifcopenshell, NO nanobind. The IFC counterpart of adacpp_step_glb (cad_wasm.cpp); the
// two share the neutral-geometry tessellation + GLB stack and differ only in the front-end reader.
//
// IO model mirrors the STEP module: both `inPath` and `outPath` live in the emscripten file system.
// Mount OPFS via WASMFS (build with -sWASMFS) and pass OPFS-backed paths so a large IFC streams
// through `pread` (bounded RSS) and the GLB is written back to OPFS. `spillDir` is a writable
// directory (OPFS or MEMFS) for the per-material spill lanes; pass an explicit one (the C++ default
// mkdtemp("/tmp/...") isn't reliable under WASMFS).
//
// Single-threaded (stream_ifc_to_glb uses threads=1), so this links WITHOUT -pthread and runs on any
// page (no SharedArrayBuffer / cross-origin isolation required).

#include <string>

#include <emscripten/bind.h>
#include <emscripten/wasmfs.h>

#include "ifc_to_glb_stream.h"

namespace {

// Returns the triangle count written, or -1 on error. deflection/angular_deg are the libtess2 chordal
// + angular tolerances (2.0 / 20.0 are the adapy production defaults). The GLB is baked to metres via
// the IFC file's unit scale (viewer default), like the native/nanobind path.
long ifc_to_glb(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                double deflection, double angular_deg, bool meshopt) {
    return adacpp::stream_ifc_to_glb(in_path, out_path, deflection, angular_deg, meshopt, spill_dir);
}

// Mount the browser's Origin Private File System (OPFS) at `mount_point` so the IFC, the GLB and the
// spill lanes live in OPFS — the IFC streams through pread (bounded RSS) instead of the wasm heap.
// MUST be called from a Web Worker (OPFS sync access handles are worker-only). Returns 0 on success,
// non-zero if OPFS is unavailable; the caller then falls back to the in-heap WASMFS default.
int mount_opfs(const std::string &mount_point) {
    backend_t opfs = wasmfs_create_opfs_backend();
    if (!opfs)
        return -1;
    return wasmfs_create_directory(mount_point.c_str(), 0777, opfs);
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_ifc_glb) {
    emscripten::function("ifcToGlb", &ifc_to_glb);
    emscripten::function("mountOpfs", &mount_opfs);
}
