#pragma once
// Sharded IFC -> GLB for N independent single-threaded workers (no shared memory): the IFC peer of
// step_glb_shard.h, same protocol (open -> process ranges -> persist -> one worker merges). The per-
// product body is stream_ifc_to_glb's (ifc_bake_product), with the single-threaded module's defaults.
//
// Unlike STEP, every IFC shard repeats a LARGE serial front: the IfcResolver metadata maps and the
// product discovery (proxy_roots) are most of an IFC conversion on the files measured, so N workers
// speed up only the tessellation share. Measure before relying on it.

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "ifc_to_glb_stream.h"

namespace adacpp {

class IfcGlbShard {
public:
    IfcGlbShard(const std::string &in_path, double deflection, double angular_deg)
        : idx_(adacpp::step::StreamIndex::from_file_pread(in_path)), r_(idx_) {
        tp_.libtess2.pin_boundary = true; // stream_ifc_to_glb's default
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * ngeom::PI / 180.0;
        tp_.threads = 1;
        if (idx_.ok()) {
            r_.build_metadata();
            roots_ = r_.proxy_roots();
            usc_ = r_.unit_scale();
        }
    }
    long root_count() const {
        return idx_.ok() ? (long) roots_.size() : -1;
    }
    // Tessellate products [begin, end) into this shard's lane; returns the products that produced
    // triangles.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
        long n = 0;
        for (long i = std::max(0L, begin); i < end && i < (long) roots_.size(); ++i)
            n += ifc_bake_product(r_, roots_[(size_t) i], tp_, usc_, *lane_) ? 1 : 0;
        return n;
    }
    bool persist() {
        return lane_ && lane_->persist();
    }

private:
    adacpp::step::StreamIndex idx_;
    adacpp::ifc_read::IfcResolver r_;
    ngeom::TessParams tp_;
    std::vector<long> roots_;
    double usc_ = 1.0;
    std::unique_ptr<adacpp::glb::GlbSpillWriter> lane_;
};

inline long merge_ifc_glb_lanes(const std::string &lane_dir, int nlanes, const std::string &out_path, bool meshopt) {
    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    return adacpp::glb::merge_persisted_lanes(lane_dir, nlanes, out_path, ada_ext, meshopt);
}

} // namespace adacpp
