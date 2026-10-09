#pragma once
// Sharded STEP -> GLB for N independent single-threaded workers that share NO memory: the browser's
// path to several cores without cross-origin isolation (no SharedArrayBuffer, so no wasm pthreads).
// Protocol and rationale: glb_shard_common.h. The per-root body is the single-threaded path's
// (step_bake_root_single), so the merged GLB carries the same solids, materials and hierarchy as the
// one-worker GLB; only the order of the solids inside each material differs (lane order).
//
//   coordinator:  StepGlbShard.prepare(in, index, N)            once, on one worker
//   every worker: s = new StepGlbShard(in, index, defl, ang)    loads the index, never scans
//                 s.processHuge(h, f0, f1, partsDir)            face ranges of the huge roots
//                 s.assembleHuge(h, chunk, partsDir, lanes, k)  once a huge root's ranges are all done
//                 s.process(b, e, lanes, k)                     batches from s.planBatches(n)
//                 s.persist()                                   lane k's manifest + spill files
//   one worker:   mergeGlbLanes(lanes, N, out, meshopt)

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glb_shard_common.h"
#include "step_to_glb_st.h"

namespace adacpp {

class StepGlbShard {
public:
    // Open `in_path` with the index prepare() saved. root_count() is -1 when either can't be read.
    StepGlbShard(const std::string &in_path, const std::string &index_path, double deflection, double angular_deg) {
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
        adacpp::step::binio::In in(index_path);
        const bool tagged = in && in.u64() == kMagic;
        idx_ = std::make_unique<adacpp::step::StreamIndex>(
            tagged ? adacpp::step::StreamIndex::from_saved(in_path, in)
                   : adacpp::step::StreamIndex::from_file_pread(std::string())); // not ok()
        r_ = std::make_unique<adacpp::step::Resolver>(*idx_);
        if (tagged && idx_->ok() && r_->load_metadata(in)) {
            in.vec(order_);
            in.vec(costs_);
            in.vec(huge_);
            ok_ = in && order_.size() == costs_.size();
        }
    }

    // Scan `in_path` once, build the metadata, LPT-order the roots and list the huge ones for
    // `nworkers` workers, and save it all to `index_path`. Returns the ordinary root count, -1 on error.
    static long prepare(const std::string &in_path, const std::string &index_path, int nworkers) {
        adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file_pread(in_path);
        if (!idx.ok())
            return -1;
        adacpp::step::Resolver r(idx);
        r.build_metadata(idx.lists);
        std::vector<std::pair<size_t, long>> lpt;
        lpt.reserve(idx.lists.roots.size());
        for (long sid : idx.lists.roots)
            lpt.emplace_back(r.solid_cost_estimate(sid), sid);
        r.clear_geom_cache();
        shard::lpt_sort(lpt);
        const size_t n_huge = shard::count_huge(lpt, nworkers);
        std::vector<std::pair<long, uint64_t>> huge; // (root id, face count)
        std::vector<long> order;
        std::vector<uint64_t> costs;
        for (size_t i = 0; i < lpt.size(); ++i) {
            if (i < n_huge) {
                r.set_face_window(0, 0); // walk the shells, count the faces, build none
                r.resolve_root(lpt[i].second);
                r.clear_face_window();
                r.clear_geom_cache();
                // Split only a B-rep whose faces all come from its shells (a procedural root counts 0).
                if (r.root_faces_seen() >= shard::HUGE_FACES) {
                    huge.emplace_back(lpt[i].second, (uint64_t) r.root_faces_seen());
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
        return o ? (long) order.size() : -1;
    }

    // Ordinary roots (huge ones excluded), heaviest first; -1 when the file or index could not be read.
    long root_count() const {
        return ok_ ? (long) order_.size() : -1;
    }
    // Exclusive ends of about `nbatches` contiguous batches of equal estimated cost over the roots.
    std::vector<long> plan_batches(long nbatches) const {
        return shard::plan_batches(costs_, nbatches);
    }
    // Tessellate roots [begin, end) into this shard's lane. `lane` must be unique per shard (it names
    // the spill files in `lane_dir`); the first call fixes both. Returns the triangles added.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!ok_)
            return -1;
        open_lane(lane_dir, lane);
        long ntri = 0;
        for (long i = std::max(0L, begin); i < end && i < (long) order_.size(); ++i)
            ntri += step_bake_root_single(*r_, order_[(size_t) i], tp_, *lane_);
        return ntri;
    }

    long huge_count() const {
        return ok_ ? (long) huge_.size() : 0;
    }
    long huge_faces(long h) const {
        return h >= 0 && h < huge_count() ? (long) huge_[(size_t) h].second : -1;
    }
    // Resolve + tessellate faces [f0, f1) of huge root `h`, unwelded, into its part file in `parts_dir`.
    // Returns the triangles written, or -1.
    long process_huge(long h, long f0, long f1, const std::string &parts_dir) {
        if (h < 0 || h >= huge_count())
            return -1;
        r_->set_face_window((size_t) f0, (size_t) f1);
        ngeom::NgeomDoc one;
        one.roots.push_back(r_->resolve_root(huge_[(size_t) h].first));
        r_->clear_face_window();
        ngeom::TessParams tpu = tp_;
        tpu.weld = false; // welded once over the whole root by assemble_huge
        ngeom::TessMesh tm = ngeom::tessellate_doc(one, tpu);
        one.roots.clear();
        r_->clear_geom_cache();
        return shard::write_part(shard::part_path(parts_dir, h, f0), tm);
    }
    // Join huge root `h`'s parts (ranges of `chunk` faces), weld once and bake it into this shard's lane
    // (same `lane_dir` / `lane` as process()). Returns the triangles added, or -1 when a part is missing.
    long assemble_huge(long h, long chunk, const std::string &parts_dir, const std::string &lane_dir, int lane) {
        if (h < 0 || h >= huge_count())
            return -1;
        ngeom::TessMesh tm;
        if (!shard::join_parts(parts_dir, h, huge_faces(h), chunk, tp_.weld, tm))
            return -1;
        open_lane(lane_dir, lane);
        r_->set_face_window(0, 0); // colour, transforms and names only
        ngeom::NgeomRoot rr = r_->resolve_root(huge_[(size_t) h].first);
        r_->clear_face_window();
        r_->clear_geom_cache();
        return step_bake_mesh(*r_, rr, std::move(tm), *lane_);
    }

    // Spill everything and write the lane manifest for the merge. False if this shard got no work.
    bool persist() {
        return lane_ && lane_->persist();
    }

private:
    static constexpr uint64_t kMagic = 0x3258444950455453ull; // "STEPIDX2"
    void open_lane(const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
    }
    std::unique_ptr<adacpp::step::StreamIndex> idx_; // the resolver keeps a reference: heap-pinned
    std::unique_ptr<adacpp::step::Resolver> r_;
    std::vector<long> order_;
    std::vector<uint64_t> costs_;
    std::vector<std::pair<long, uint64_t>> huge_; // (root id, face count)
    bool ok_ = false;
    ngeom::TessParams tp_;
    std::unique_ptr<adacpp::glb::GlbSpillWriter> lane_;
};

inline long merge_step_glb_lanes(const std::string &lane_dir, int nlanes, const std::string &out_path, bool meshopt) {
    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    return adacpp::glb::merge_persisted_lanes(lane_dir, nlanes, out_path, ada_ext, meshopt);
}

} // namespace adacpp
