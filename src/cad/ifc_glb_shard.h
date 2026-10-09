#pragma once
// Sharded IFC -> GLB for N independent single-threaded workers (no shared memory): the IFC peer of
// step_glb_shard.h, same protocol and verbs (glb_shard_common.h). The per-product body is
// stream_ifc_to_glb's (ifc_bake_product), with the single-threaded module's defaults.
//
// prepare() matters more here than for STEP: an IFC's serial front -- the IfcResolver metadata maps,
// product discovery (proxy_roots) and the unit scan, each a walk over every statement -- is a large
// share of an IFC conversion, and the index lets every worker skip it.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glb_shard_common.h"
#include "ifc_to_glb_stream.h"

namespace adacpp {

class IfcGlbShard {
public:
    // Open `in_path` with the index prepare() saved. root_count() is -1 when either can't be read.
    IfcGlbShard(const std::string &in_path, const std::string &index_path, double deflection, double angular_deg) {
        tp_.libtess2.pin_boundary = true; // stream_ifc_to_glb's default
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * ngeom::PI / 180.0;
        tp_.threads = 1;
        adacpp::step::binio::In in(index_path);
        const bool tagged = in && in.u64() == kMagic;
        idx_ = std::make_unique<adacpp::step::StreamIndex>(
            tagged ? adacpp::step::StreamIndex::from_saved(in_path, in)
                   : adacpp::step::StreamIndex::from_file_pread(std::string())); // not ok()
        r_ = std::make_unique<adacpp::ifc_read::IfcResolver>(*idx_);
        if (tagged && idx_->ok() && r_->load_metadata(in)) {
            in.vec(order_);
            in.vec(costs_);
            in.vec(huge_);
            in.pod(usc_);
            ok_ = in && order_.size() == costs_.size();
        }
    }

    // Scan `in_path` once, build the metadata, find the products and the unit scale, LPT-order the
    // products and list the huge splittable ones for `nworkers` workers, and save it all to
    // `index_path`. Returns the ordinary product count, or -1 on error.
    static long prepare(const std::string &in_path, const std::string &index_path, int nworkers) {
        adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file_pread(in_path);
        if (!idx.ok())
            return -1;
        adacpp::ifc_read::IfcResolver r(idx);
        r.build_metadata();
        const std::vector<long> roots = r.proxy_roots();
        const double usc = r.unit_scale();
        std::vector<std::pair<size_t, long>> lpt;
        lpt.reserve(roots.size());
        for (long pid : roots)
            lpt.emplace_back(r.product_cost(pid), pid);
        r.clear_cache();
        shard::lpt_sort(lpt);
        const size_t n_huge = shard::count_huge(lpt, nworkers, shard::IFC_HEAVY_FACES);
        std::vector<std::pair<long, uint64_t>> huge; // (product id, face count)
        std::vector<long> order;
        std::vector<uint64_t> costs;
        for (size_t i = 0; i < lpt.size(); ++i) {
            if (i < n_huge) {
                const size_t nf = r.splittable_faces(lpt[i].second);
                if (nf >= shard::HUGE_FACES) {
                    huge.emplace_back(lpt[i].second, (uint64_t) nf);
                    continue;
                }
            }
            order.push_back(lpt[i].second);
            costs.push_back(lpt[i].first);
        }
        adacpp::step::binio::Out o(index_path);
        o.u64(kMagic);
        if (!idx.save(o))
            return -1;
        r.save_metadata(o);
        o.vec(order);
        o.vec(costs);
        o.vec(huge);
        o.pod(usc);
        return o ? (long) order.size() : -1;
    }

    long root_count() const {
        return ok_ ? (long) order_.size() : -1;
    }
    std::vector<long> plan_batches(long nbatches) const {
        return shard::plan_batches(costs_, nbatches);
    }
    // Tessellate products [begin, end) into this shard's lane; returns the products that produced
    // triangles, or -1 when the shard did not open.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!ok_)
            return -1;
        open_lane(lane_dir, lane);
        long n = 0;
        for (long i = std::max(0L, begin); i < end && i < (long) order_.size(); ++i)
            n += ifc_bake_product(*r_, order_[(size_t) i], tp_, usc_, *lane_) ? 1 : 0;
        return n;
    }

    long huge_count() const {
        return ok_ ? (long) huge_.size() : 0;
    }
    long huge_faces(long h) const {
        return h >= 0 && h < huge_count() ? (long) huge_[(size_t) h].second : -1;
    }
    // Resolve + tessellate faces [f0, f1) of huge product `h`, unwelded, into its part file.
    long process_huge(long h, long f0, long f1, const std::string &parts_dir) {
        if (h < 0 || h >= huge_count())
            return -1;
        r_->set_face_window((size_t) f0, (size_t) f1);
        ngeom::NgeomDoc one;
        one.roots.push_back(r_->resolve_product(huge_[(size_t) h].first));
        r_->clear_face_window();
        r_->clear_cache();
        ngeom::TessParams tpu = tp_;
        tpu.weld = false; // welded once over the whole product by assemble_huge
        ngeom::TessMesh tm = ngeom::tessellate_doc(one, tpu);
        return shard::write_part(shard::part_path(parts_dir, h, f0), tm);
    }
    // Join huge product `h`'s parts, weld once and bake it into this shard's lane. Returns 1 when it
    // added triangles, 0 when not, -1 when a part is missing.
    long assemble_huge(long h, long chunk, const std::string &parts_dir, const std::string &lane_dir, int lane) {
        if (h < 0 || h >= huge_count())
            return -1;
        ngeom::TessMesh tm;
        if (!shard::join_parts(parts_dir, h, huge_faces(h), chunk, tp_.weld, tm))
            return -1;
        open_lane(lane_dir, lane);
        // One face is enough to carry the colour, names and the composed object placement: the
        // placement is applied only to a product that resolved geometry.
        ngeom::NgeomRoot rr;
        for (size_t w = 1;; w *= 64) { // widen past unresolvable leading faces
            r_->set_face_window(0, w);
            rr = r_->resolve_product(huge_[(size_t) h].first);
            r_->clear_face_window();
            r_->clear_cache();
            if (!rr.faces.empty() || w >= (size_t) huge_faces(h))
                break;
        }
        return ifc_bake_mesh(rr, std::move(tm), usc_, *lane_) ? 1 : 0;
    }

    bool persist() {
        return lane_ && lane_->persist();
    }

private:
    static constexpr uint64_t kMagic = 0x3258444943464949ull; // "IIFCIDX2"
    void open_lane(const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
    }
    std::unique_ptr<adacpp::step::StreamIndex> idx_; // the resolver keeps a reference: heap-pinned
    std::unique_ptr<adacpp::ifc_read::IfcResolver> r_;
    std::vector<long> order_;
    std::vector<uint64_t> costs_;
    std::vector<std::pair<long, uint64_t>> huge_;
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
