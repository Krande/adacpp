#pragma once
// Single-threaded, OCC-free STEP -> GLB for the no-pyodide wasm build (and a native parity oracle).
//
// Differs from stream_step_to_glb_impl ONLY in being serial: NO std::thread (so it links without
// emscripten -pthread / SharedArrayBuffer / COOP-COEP), NO mmap (uses from_file_pread, so the file can
// live in OPFS via WASMFS instead of the wasm heap), and no LPT/profiling. The per-solid body —
// resolve -> tessellate -> unit-scale -> spill -> merge — is the same path as the threaded worker, so a
// given STEP yields a byte-identical GLB (modulo the spill/merge order, which is deterministic here).
//
// Memory stays bounded: each solid is spilled to `spill_dir` and freed; the merge streams the lane
// files into the output. `spill_dir` must exist and be writable (a WASMFS/OPFS mount in wasm, a temp
// dir natively); the lane's spill files are removed when this returns.

#include <string>
#include <vector>

#include "../cadit/step/step_reader.h"
#include "../geom/neutral/ada_ext_schema.h"
#include "../geom/neutral/ngeom_glb.h"
#include "../geom/neutral/ngeom_tessellate.h"

namespace adacpp {

// Returns the number of triangles written, or -1 on I/O error (bad input file or unwritable output).
inline long step_to_glb_single(const std::string &in_path, const std::string &out_path, const std::string &spill_dir,
                               double deflection, double angular_deg, bool meshopt) {
    using namespace adacpp::ngeom;

    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file_pread(in_path); // pread, no mmap
    if (!idx.ok())
        return -1;

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;

    adacpp::step::Resolver r(idx);
    r.build_metadata(idx.lists);

    adacpp::glb::GlbSpillWriter lane(spill_dir, 0);
    long ntri = 0;
    for (long sid : idx.lists.roots) {
        NgeomRoot root = r.resolve_root(sid);
        if (!root.id.empty()) {
            NgeomDoc one;
            one.roots.push_back(std::move(root));
            TessMesh tm = tessellate_doc(one, tp);
            if (!tm.indices.empty()) {
                const long solid_tris = (long) (tm.indices.size() / 3); // before the move below empties it
                const NgeomRoot &rr = one.roots[0];
                adacpp::glb::GlbSolid gs;
                gs.positions = std::move(tm.positions);
                gs.indices = std::move(tm.indices);
                gs.color = {rr.cr, rr.cg, rr.cb, rr.ca};
                gs.transforms = rr.transforms;
                gs.id = rr.id;
                if (!rr.instance_paths.empty() && !rr.instance_paths[0].empty())
                    gs.product_name = rr.instance_paths[0].back().second;
                gs.instance_paths = rr.instance_paths;
                // file length unit -> metres (the viewer's assumption); rotation is unitless.
                const double usc = r.unit_scale();
                if (usc != 1.0) {
                    const float s = (float) usc;
                    for (float &p : gs.positions)
                        p *= s;
                    for (auto &M : gs.transforms) {
                        M[12] *= s;
                        M[13] *= s;
                        M[14] *= s;
                    }
                }
                lane.add(gs); // spilled to disk immediately
                ntri += solid_tris;
            }
        }
        r.clear_geom_cache();
    }
    lane.flush();

    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    std::vector<adacpp::glb::GlbSpillWriter *> lanes{&lane};
    if (!adacpp::glb::write_glb_merged(out_path, lanes, ada_ext, meshopt))
        return -1;
    return ntri;
}

} // namespace adacpp
