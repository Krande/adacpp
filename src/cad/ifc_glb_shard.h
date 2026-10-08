#pragma once
// Sharded IFC -> GLB for N independent single-threaded workers (no shared memory): the IFC peer of
// step_glb_shard.h, same protocol (open -> process ranges -> persist -> one worker merges) and the
// same two ways to open a shard. The per-product body is stream_ifc_to_glb's (ifc_bake_product), with
// the single-threaded module's defaults.
//
// The re-parse open matters more here than for STEP: an IFC shard's serial front -- the IfcResolver
// metadata maps, product discovery (proxy_roots) and the unit scan, each a walk over every statement
// -- is most of an IFC conversion on small and medium files. prepare() + the index open pay it once.

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ifc_to_glb_stream.h"

namespace adacpp {

class IfcGlbShard {
public:
    IfcGlbShard(const std::string &in_path, double deflection, double angular_deg) {
        params(deflection, angular_deg);
        idx_ = std::make_unique<adacpp::step::StreamIndex>(adacpp::step::StreamIndex::from_file_pread(in_path));
        r_ = std::make_unique<adacpp::ifc_read::IfcResolver>(*idx_);
        if (idx_->ok()) {
            r_->build_metadata();
            order_ = r_->proxy_roots();
            usc_ = r_->unit_scale();
            ok_ = true;
        }
    }
    IfcGlbShard(const std::string &in_path, const std::string &index_path, double deflection, double angular_deg) {
        params(deflection, angular_deg);
        adacpp::step::binio::In in(index_path);
        const bool tagged = in && in.u64() == kMagic;
        idx_ = std::make_unique<adacpp::step::StreamIndex>(
            tagged ? adacpp::step::StreamIndex::from_saved(in_path, in)
                   : adacpp::step::StreamIndex::from_file_pread(std::string())); // not ok()
        r_ = std::make_unique<adacpp::ifc_read::IfcResolver>(*idx_);
        if (tagged && idx_->ok() && r_->load_metadata(in)) {
            in.vec(order_);
            in.pod(usc_);
            ok_ = (bool) in;
        }
    }

    // Scan `in_path` once, build the metadata, find the products (heaviest-first when `lpt`) and the
    // unit scale, and save it all to `index_path`. Returns the product count, or -1 on error.
    static long prepare(const std::string &in_path, const std::string &index_path, bool lpt) {
        adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file_pread(in_path);
        if (!idx.ok())
            return -1;
        adacpp::ifc_read::IfcResolver r(idx);
        r.build_metadata();
        std::vector<long> order = r.proxy_roots();
        const double usc = r.unit_scale();
        if (lpt) {
            std::vector<std::pair<size_t, long>> cost;
            cost.reserve(order.size());
            for (long pid : order)
                cost.emplace_back(r.product_cost(pid), pid);
            r.clear_cache();
            std::stable_sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
            for (size_t i = 0; i < cost.size(); ++i)
                order[i] = cost[i].second;
        }
        adacpp::step::binio::Out o(index_path);
        o.u64(kMagic);
        if (!idx.save(o))
            return -1;
        r.save_metadata(o);
        o.vec(order);
        o.pod(usc);
        return o ? (long) order.size() : -1;
    }

    long root_count() const {
        return ok_ ? (long) order_.size() : -1;
    }
    // Tessellate products [begin, end) into this shard's lane; returns the products that produced
    // triangles.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
        long n = 0;
        for (long i = std::max(0L, begin); i < end && i < (long) order_.size(); ++i)
            n += ifc_bake_product(*r_, order_[(size_t) i], tp_, usc_, *lane_) ? 1 : 0;
        return n;
    }
    bool persist() {
        return lane_ && lane_->persist();
    }

private:
    static constexpr uint64_t kMagic = 0x3158444943464949ull; // "IIFCIDX1"
    void params(double deflection, double angular_deg) {
        tp_.libtess2.pin_boundary = true; // stream_ifc_to_glb's default
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * ngeom::PI / 180.0;
        tp_.threads = 1;
    }
    std::unique_ptr<adacpp::step::StreamIndex> idx_; // the resolver keeps a reference: heap-pinned
    std::unique_ptr<adacpp::ifc_read::IfcResolver> r_;
    std::vector<long> order_;
    double usc_ = 1.0;
    bool ok_ = false;
    ngeom::TessParams tp_;
    std::unique_ptr<adacpp::glb::GlbSpillWriter> lane_;
};

inline long merge_ifc_glb_lanes(const std::string &lane_dir, int nlanes, const std::string &out_path, bool meshopt) {
    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    return adacpp::glb::merge_persisted_lanes(lane_dir, nlanes, out_path, ada_ext, meshopt);
}

} // namespace adacpp
