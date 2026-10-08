#pragma once
// Sharded STEP -> GLB for N independent single-threaded workers that share NO memory -- the browser's
// path to several cores when the page is not cross-origin isolated (no SharedArrayBuffer, so no wasm
// pthreads). Each Web Worker owns its own wasm instance and tessellates whichever root ranges the
// coordinator hands it into its own spill lane. When every range is done, each shard persists its lane
// (manifest + spill files) into a directory all the workers can see (an OPFS directory in the browser)
// and ONE worker merges the lanes into the GLB (glb::merge_persisted_lanes). Dynamic dispatch -- small
// ranges, the next one to whichever worker is idle -- balances the long tail the way the native thread
// pool's shared counter does.
//
// Two ways to open a shard:
//   - re-parse: (in_path, deflection, angular) -- every shard scans the file and builds the metadata
//     itself; ranges index the file's root order.
//   - parse once: one worker calls prepare(), which scans, builds the metadata, orders the roots
//     heaviest-first (LPT, the native pool's order) and saves all three to an index file; every shard
//     then opens with (in_path, index_path, deflection, angular) and loads that instead. The scan and
//     the metadata build then happen once per conversion, not once per worker, and the expensive roots
//     start first.
//
// Huge roots (parse-once only). A root bigger than one worker's fair share of the file would pin a
// worker for the conversion's whole tail (469826: one 61k-face solid is 65% of all tessellation). The
// native pool tessellates such a root's faces on every thread (tessellate_doc's face pool); workers
// with no shared memory split it the same way through files: prepare() lists the huge roots apart from
// the ordinary order, any worker takes a FACE RANGE of one (processHuge: it resolves and tessellates
// only those faces, unwelded, into a part file), and once every range is done one worker assembles
// the root (assembleHuge: parts in face order, ONE weld over the whole root -- what the serial path
// does -- then the usual bake into its lane). Same triangles as the serial path, in the same order.
//
// The per-root body is the single-threaded path's (step_bake_root_single), so the merged GLB carries
// the same solids, materials and hierarchy as the one-worker GLB; only the order of the solids inside
// each material differs (lane order instead of file order).

#include <algorithm>
#include <cstdint>
#include <memory>
#include <cstdio>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../geom/neutral/ngeom_weld.h"
#include "step_to_glb_st.h"

namespace adacpp {

class StepGlbShard {
public:
    StepGlbShard(const std::string &in_path, double deflection, double angular_deg) {
        params(deflection, angular_deg);
        idx_ = std::make_unique<adacpp::step::StreamIndex>(adacpp::step::StreamIndex::from_file_pread(in_path));
        r_ = std::make_unique<adacpp::step::Resolver>(*idx_);
        if (idx_->ok()) {
            r_->build_metadata(idx_->lists);
            order_ = idx_->lists.roots;
            ok_ = true;
        }
    }
    StepGlbShard(const std::string &in_path, const std::string &index_path, double deflection, double angular_deg) {
        params(deflection, angular_deg);
        adacpp::step::binio::In in(index_path);
        const bool tagged = in && in.u64() == kMagic;
        idx_ = std::make_unique<adacpp::step::StreamIndex>(
            tagged ? adacpp::step::StreamIndex::from_saved(in_path, in)
                   : adacpp::step::StreamIndex::from_file_pread(std::string())); // not ok()
        r_ = std::make_unique<adacpp::step::Resolver>(*idx_);
        if (tagged && idx_->ok() && r_->load_metadata(in)) {
            in.vec(order_);
            in.vec(huge_);
            ok_ = (bool) in;
        }
    }

    // Scan `in_path` once, build the metadata, order the roots (heaviest-first when `lpt`) and save it
    // all to `index_path` for the parse-once constructor. With `nworkers` > 1, the roots bigger than a
    // worker's fair share are listed as huge (face-split) instead of ordered. Returns the ordinary root
    // count, or -1 on error.
    static long prepare(const std::string &in_path, const std::string &index_path, bool lpt, int nworkers) {
        adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file_pread(in_path);
        if (!idx.ok())
            return -1;
        adacpp::step::Resolver r(idx);
        r.build_metadata(idx.lists);
        std::vector<long> order(idx.lists.roots.begin(), idx.lists.roots.end());
        std::vector<std::pair<long, uint64_t>> huge; // (root id, face count)
        if (lpt || nworkers > 1) {
            std::vector<std::pair<size_t, long>> cost;
            cost.reserve(order.size());
            size_t total = 0;
            for (long sid : order) {
                const size_t c = r.solid_cost_estimate(sid);
                total += c;
                cost.emplace_back(c, sid);
            }
            r.clear_geom_cache();
            std::stable_sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
            // The native pool's huge test (step_to_glb_stream.h): >= 2048 faces AND >= a fair share.
            size_t n_huge = 0;
            if (nworkers > 1) {
                const size_t fair_share = total / (size_t) nworkers;
                while (n_huge < cost.size() && cost[n_huge].first >= 2048 && cost[n_huge].first >= fair_share)
                    ++n_huge;
            }
            for (size_t i = 0; i < n_huge; ++i) {
                r.set_face_window(0, 0); // count the faces; build none
                r.resolve_root(cost[i].second);
                huge.emplace_back(cost[i].second, (uint64_t) r.root_faces_seen());
                r.clear_face_window();
                r.clear_geom_cache();
            }
            order.clear();
            for (size_t i = n_huge; i < cost.size(); ++i)
                order.push_back(cost[i].second);
            if (!lpt) { // keep file order for the ordinary roots
                std::unordered_set<long> big;
                for (const auto &h : huge)
                    big.insert(h.first);
                order.clear();
                for (long sid : idx.lists.roots)
                    if (!big.count(sid))
                        order.push_back(sid);
            }
        }
        adacpp::step::binio::Out o(index_path);
        o.u64(kMagic);
        if (!idx.save(o))
            return -1;
        r.save_metadata(o);
        o.vec(order);
        o.vec(huge);
        return o ? (long) order.size() : -1;
    }

    // Number of roots (-1 when the file or index could not be opened). Ranges index into this order.
    long root_count() const {
        return ok_ ? (long) order_.size() : -1;
    }
    // Tessellate roots [begin, end) into this shard's lane. `lane` must be unique per shard (it names
    // the spill files in `lane_dir`); the first call fixes both. Returns the triangles added.
    long process(long begin, long end, const std::string &lane_dir, int lane) {
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
        long ntri = 0;
        for (long i = std::max(0L, begin); i < end && i < (long) order_.size(); ++i)
            ntri += step_bake_root_single(*r_, order_[(size_t) i], tp_, *lane_);
        return ntri;
    }
    // Huge roots (see the header comment): how many, and each one's face count.
    long huge_count() const {
        return (long) huge_.size();
    }
    long huge_faces(long h) const {
        return h >= 0 && h < (long) huge_.size() ? (long) huge_[(size_t) h].second : -1;
    }
    // Resolve + tessellate faces [f0, f1) of huge root `h`, unwelded, into `<parts_dir>/huge<h>_<f0>.part`.
    // Returns the triangles written, or -1 on a write error.
    long process_huge(long h, long f0, long f1, const std::string &parts_dir) {
        if (h < 0 || h >= (long) huge_.size())
            return -1;
        r_->set_face_window((size_t) f0, (size_t) f1);
        ngeom::NgeomDoc one;
        one.roots.push_back(r_->resolve_root(huge_[(size_t) h].first));
        r_->clear_face_window();
        ngeom::TessParams tpu = tp_;
        tpu.weld = false; // welded once over the whole root at assembly
        ngeom::TessMesh tm = ngeom::tessellate_doc(one, tpu);
        r_->clear_geom_cache();
        adacpp::step::binio::Out o(part_path(parts_dir, h, f0));
        o.vec(tm.positions);
        o.vec(tm.indices);
        o.vec(tm.normals);
        return o ? (long) (tm.indices.size() / 3) : -1;
    }
    // Join huge root `h`'s parts (ranges of `chunk` faces from 0, in `parts_dir`), weld once, bake into
    // this shard's lane (same `lane_dir` / `lane` as process()). The part files are removed. Returns the triangles
    // added, or -1 when a part is missing.
    long assemble_huge(long h, long chunk, const std::string &parts_dir, const std::string &lane_dir, int lane) {
        if (h < 0 || h >= (long) huge_.size() || chunk < 1)
            return -1;
        if (!lane_)
            lane_ = std::make_unique<adacpp::glb::GlbSpillWriter>(lane_dir, lane);
        const long nf = (long) huge_[(size_t) h].second;
        ngeom::TessMesh tm;
        for (long f0 = 0; f0 < nf; f0 += chunk) {
            const std::string path = part_path(parts_dir, h, f0);
            std::vector<float> pos, nrm;
            std::vector<uint32_t> idx;
            {
                adacpp::step::binio::In in(path);
                if (!in)
                    return -1;
                in.vec(pos);
                in.vec(idx);
                in.vec(nrm);
            }
            std::remove(path.c_str());
            const uint32_t base = (uint32_t) (tm.positions.size() / 3);
            tm.positions.insert(tm.positions.end(), pos.begin(), pos.end());
            tm.normals.insert(tm.normals.end(), nrm.begin(), nrm.end());
            tm.indices.reserve(tm.indices.size() + idx.size());
            for (uint32_t i : idx)
                tm.indices.push_back(base + i);
        }
        if (tp_.weld)
            ngeom::weld_mesh(tm.positions, tm.indices, tm.normals);
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
    static constexpr uint64_t kMagic = 0x3158444950455453ull; // "STEPIDX1"
    static std::string part_path(const std::string &dir, long h, long f0) {
        return dir + "/huge" + std::to_string(h) + "_" + std::to_string(f0) + ".part";
    }
    void params(double deflection, double angular_deg) {
        tp_.deflection = deflection;
        tp_.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
    }
    std::unique_ptr<adacpp::step::StreamIndex> idx_; // the resolver keeps a reference: heap-pinned
    std::unique_ptr<adacpp::step::Resolver> r_;
    std::vector<long> order_;
    std::vector<std::pair<long, uint64_t>> huge_; // (root id, face count), face-split across workers
    bool ok_ = false;
    ngeom::TessParams tp_;
    std::unique_ptr<adacpp::glb::GlbSpillWriter> lane_;
};

inline long merge_step_glb_lanes(const std::string &lane_dir, int nlanes, const std::string &out_path, bool meshopt) {
    const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
    return adacpp::glb::merge_persisted_lanes(lane_dir, nlanes, out_path, ada_ext, meshopt);
}

} // namespace adacpp
