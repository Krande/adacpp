// embind entry for the no-pyodide native B-rep WRITER wasm module (STEP→IFC + IFC→STEP).
//
// STANDALONE wasm module: the OCC-free, dep-free native B-rep writers (StreamIndex/Resolver +
// IfcResolver readers → ifc_emit/step_emit emitters) compiled with emscripten + embind — NO pyodide,
// NO Python, NO OCCT, NO ifcopenshell, NO nanobind. Shares ONE implementation with the nanobind
// module via brep_file_convert.h (adacpp::brep_convert::write_ifc_file_impl / write_ifc_to_step_impl).
//
// IO mirrors the CAD→GLB modules: both paths live in the emscripten FS. Mount OPFS via WASMFS (build
// with -sWASMFS) and pass OPFS-backed paths so a large STEP/IFC streams through `pread` (bounded RSS)
// and the output is written back to OPFS. Single-threaded (the writers are serial), so this links
// WITHOUT -pthread and runs on any page (no SharedArrayBuffer / cross-origin isolation required).

#include <string>

#include <emscripten/bind.h>
#include <emscripten/wasmfs.h>

#include "brep_file_convert.h"

namespace {

// STEP → IFC. schema = "IFC4X3_ADD2" | "IFC4". Returns the number of solids written (>0 success,
// 0 = nothing emitted / I/O error). deflection/angular_deg only affect face-set fallbacks.
long step_to_ifc(const std::string &in_path, const std::string &out_path, const std::string &schema,
                 double deflection, double angular_deg, long max_solids) {
    adacpp::ifc_emit::FileStats fs =
        adacpp::brep_convert::write_ifc_file_impl(in_path, out_path, schema, deflection, angular_deg, max_solids);
    return fs.solids_out;
}

// IFC → STEP (AP242). Returns the number of products written (>0 success, 0 = nothing / I/O error).
long ifc_to_step(const std::string &in_path, const std::string &out_path, double deflection, double angular_deg,
                 long max_solids) {
    adacpp::ifc_emit::FileStats fs =
        adacpp::brep_convert::write_ifc_to_step_impl(in_path, out_path, deflection, angular_deg, max_solids);
    return fs.solids_out;
}

// Mount the browser's Origin Private File System (OPFS) at `mount_point` (worker-only). Returns 0 on
// success, non-zero if OPFS is unavailable; the caller falls back to the in-heap WASMFS default.
int mount_opfs(const std::string &mount_point) {
    backend_t opfs = wasmfs_create_opfs_backend();
    if (!opfs)
        return -1;
    return wasmfs_create_directory(mount_point.c_str(), 0777, opfs);
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_brep_writer) {
    emscripten::function("stepToIfc", &step_to_ifc);
    emscripten::function("ifcToStep", &ifc_to_step);
    emscripten::function("mountOpfs", &mount_opfs);
}
