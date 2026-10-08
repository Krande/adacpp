#pragma once
// Sharded STEP -> GLB for N independent single-threaded workers that share NO memory -- the browser's
// path to several cores when the page is not cross-origin isolated (no SharedArrayBuffer, so no wasm
// pthreads). Each Web Worker owns its own wasm instance: it opens the file, builds the STEP metadata
// itself, and tessellates whichever root ranges the coordinator hands it into its own spill lane. When
// every range is done, each shard persists its lane (manifest + spill files) into a directory all the
// workers can see (an OPFS directory in the browser) and ONE worker merges the lanes into the GLB
// (glb::merge_persisted_lanes). Dynamic dispatch -- small ranges, the next one to whichever worker is
// idle -- balances the long tail the way the native thread pool's shared counter does.
//
// The per-root body is the single-threaded path's (step_bake_root_single), so the merged GLB carries
// the same solids, materials and hierarchy as the one-worker GLB; only the order of the solids inside
// each material differs (lane order instead of file order).
//
// What every shard pays again: the file scan + metadata (a few % of a STEP conversion) and its own
// copy of whatever it parsed; with the buffered (in-heap) input path, also a copy of the file.

#include <algorithm>
#include <memory>
#include <string>

#include "step_to_glb_st.h"

namespace adacpp {

class StepGlbShard {
public:
    StepGlbShard(const std::string &in_path, double deflection, double angular_deg)
        : idx_(adacpp::step::StreamIndex::from_file_pread(in_path)), r_(idx_) {
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
        if (idx_.ok())
            r_.build_metadata(idx_.lists);
    }
    // Number of root solids (-1 when the file could not be opened). Ranges index into this order.
    long root_count() const {
        return idx_.ok() ? (long) idx_.lists.roots.size() : -1;
    }
    // Tessellate roots [begin, end) into this shard's lane. `lane` must be unique per shard (it names
    // the spill files in `lane_dir`); the first call fixes both. Returns the triangles added.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
        long ntri = 0;
        const auto &roots = idx_.lists.roots;
        for (long i = std::max(0L, begin); i < end && i < (long) roots.size(); ++i)
            ntri += step_bake_root_single(r_, roots[(size_t) i], tp_, *lane_);
        return ntri;
    }
    // Spill everything and write the lane manifest for the merge. False if this shard got no work.
    bool persist() {
        return lane_ && lane_->persist();
    }

private:
    adacpp::step::StreamIndex idx_;
    adacpp::step::Resolver r_;
    ngeom::TessParams tp_;
    std::unique_ptr<adacpp::glb::GlbSpillWriter> lane_;
};

inline long merge_step_glb_lanes(const std::string &lane_dir, int nlanes, const std::string &out_path, bool meshopt) {
    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    return adacpp::glb::merge_persisted_lanes(lane_dir, nlanes, out_path, ada_ext, meshopt);
}

} // namespace adacpp
