// Native pure-C++ IFC -> GLB (no ifcopenshell, no OCC). IfcResolver resolves each product to an
// NgeomRoot (geometry + presentation colour + spatial-structure path + length unit), libtess2
// tessellates it, and the SHARED GLB spill writer (ngeom_glb.h — the same one stream_step_to_glb
// uses) bakes it to metres. So the viewer gets geometry + per-mesh colour + the spatial tree +
// names/guids — a viewer-equivalent GLB (IFC property sets never live in the GLB; they are fetched
// on selection from the source model).
//
// Mirrors stream_step_to_glb but drives IfcResolver (proxy_roots / resolve_product / unit_scale)
// instead of the STEP Resolver. Parallel: a master resolver builds the shared read-only metadata
// (colour + spatial maps) once, LPT-orders products by product_cost, and per-thread worker resolvers
// (copy_metadata_from) tessellate into per-thread lanes — same phase-A huge-prefix + dynamic phase-B
// pool as the STEP core. Curve-only bodies (alignment axes -> GL_LINES) are skipped (GlbSolid is
// triangles-only).
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "effective_concurrency.h"
#include "mem_trim.h"
#include "mem_tune.h"

#include "../geom/neutral/ada_ext_schema.h"
#include "../geom/neutral/ngeom_glb.h"
#include "../geom/neutral/ngeom_tessellate.h"
#include "ifc_reader.h"
#include "step_reader.h" // StreamIndex

namespace adacpp {

inline long stream_ifc_to_glb(const std::string &in_path, const std::string &out_path, double deflection,
                              double angular_deg, bool meshopt, const std::string &spill_dir = "",
                              double model_scale = 0.0, int num_threads = 0) {
    using namespace adacpp::ngeom;
    adacpp::tune_malloc_for_streaming();
    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(in_path);
    if (!idx.ok())
        return -1;
    // Master resolver: build the expensive read-only metadata once (workers share it, never rebuild).
    adacpp::ifc_read::IfcResolver master(idx);
    master.build_metadata();
    std::vector<long> roots = master.proxy_roots();
    const double usc = master.unit_scale(); // metres per file length-unit -> bake to metres

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * PI / 180.0;
    tp.threads = 1;
    tp.model_scale = model_scale;

    int nthreads = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();

    // LPT order (heaviest products first) + huge-prefix detection, mirroring stream_step_to_glb: a
    // product bigger than a thread's fair share of the whole file is resolved serially with the
    // face-level pool so it can't pin one worker (and its resolve+tess memory doesn't stack with the
    // other lanes').
    size_t n_huge = 0;
    if (nthreads > 1 && roots.size() > 1) {
        constexpr size_t HUGE_FACES = 2048;
        std::vector<std::pair<size_t, long>> cost;
        cost.reserve(roots.size());
        size_t total = 0;
        for (long pid : roots) {
            size_t c = master.product_cost(pid);
            total += c;
            cost.emplace_back(c, pid);
        }
        master.clear_cache();
        std::sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (size_t i = 0; i < cost.size(); ++i)
            roots[i] = cost[i].second;
        const size_t fair_share = total / (size_t) nthreads;
        while (n_huge < cost.size() && cost[n_huge].first >= HUGE_FACES && cost[n_huge].first >= fair_share)
            ++n_huge;
    }

    // Spill dir: private mkdtemp (auto-removed) unless the caller supplied one.
    std::string spill;
    bool remove_after = false;
    char tmpl[] = "/tmp/adacpp_ifcglb_XXXXXX";
    if (spill_dir.empty()) {
        if (char *dir = ::mkdtemp(tmpl)) {
            spill = dir;
            remove_after = true;
        }
    } else {
        std::error_code ec;
        std::filesystem::create_directories(spill_dir, ec);
        if (std::filesystem::is_directory(spill_dir, ec))
            spill = spill_dir;
    }
    if (spill.empty())
        return -1;

    std::atomic<long> nwritten{0};
    bool ok = false;
    { // lanes scoped so their temp files are gone before the (maybe) rmdir
        std::deque<adacpp::glb::GlbSpillWriter> lanes;
        for (int t = 0; t < nthreads; ++t)
            lanes.emplace_back(spill, t);
        std::atomic<size_t> next{n_huge};

        // Resolve one product with the caller's resolver, tessellate (tpp.threads>1 => face pool),
        // bake to metres + spill. Shared by the huge-prefix phase and the worker pool.
        auto process = [&](adacpp::ifc_read::IfcResolver &r, adacpp::glb::GlbSpillWriter &lane, size_t i,
                           const TessParams &tpp) {
            NgeomRoot root = r.resolve_product(roots[i]);
            r.clear_cache(); // bounded memory: statement/surface caches don't grow across products
            NgeomDoc one;
            one.roots.push_back(std::move(root));
            TessMesh tm = tessellate_doc(one, tpp);
            if (tm.indices.empty())
                return;
            if (tm.mesh_type == MeshType::LINES)
                return; // curve-only body (alignment axis); GlbSolid is triangles-only
            const NgeomRoot &rr = one.roots[0];
            adacpp::glb::GlbSolid gs;
            gs.positions = std::move(tm.positions);
            gs.indices = std::move(tm.indices);
            gs.color = {rr.cr, rr.cg, rr.cb, rr.ca}; // grey default when !has_color
            gs.transforms = rr.transforms;
            gs.id = rr.id;
            if (!rr.instance_paths.empty() && !rr.instance_paths[0].empty())
                gs.product_name = rr.instance_paths[0].back().second;
            gs.instance_paths = rr.instance_paths;
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
            lane.add(gs);
            nwritten.fetch_add(1, std::memory_order_relaxed);
        };

        // Phase A — the huge prefix, one product at a time with every thread on its faces.
        if (n_huge > 0) {
            TessParams tph = tp;
            tph.threads = nthreads;
            adacpp::ifc_read::IfcResolver r0(idx);
            r0.copy_metadata_from(master);
            // Phase A resolves each huge product single-threaded (only tessellate_doc's face pool is
            // parallel), so bound cache_ mid-shell — without it a single 61k-face IFC product piles
            // parsed statements to ~GB during its one resolve, same failure the STEP glb/mesh paths hit.
            r0.enable_cache_bounding();
            for (size_t i = 0; i < n_huge; ++i)
                process(r0, lanes[0], i, tph);
            adacpp::mem_trim();
        }
        // Phase B — each worker pulls remaining products off the shared counter, resolving with its OWN
        // caches + a copy of the shared metadata, into its lane.
        auto worker = [&](int t) {
            adacpp::ifc_read::IfcResolver r(idx);
            r.copy_metadata_from(master);
            adacpp::glb::GlbSpillWriter &lane = lanes[t];
            int local = 0;
            for (;;) {
                size_t i = next.fetch_add(1, std::memory_order_relaxed);
                if (i >= roots.size())
                    break;
                process(r, lane, i, tp);
                if (++local % 128 == 0)
                    adacpp::mem_trim();
            }
        };
        std::vector<std::thread> pool;
        pool.reserve(nthreads - 1);
        for (int t = 1; t < nthreads; ++t)
            pool.emplace_back(worker, t);
        worker(0);
        for (std::thread &th : pool)
            th.join();

        std::vector<adacpp::glb::GlbSpillWriter *> lane_ptrs;
        for (adacpp::glb::GlbSpillWriter &l : lanes)
            lane_ptrs.push_back(&l);
        const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
        ok = adacpp::glb::write_glb_merged(out_path, lane_ptrs, ada_ext, meshopt);
    }
    if (remove_after)
        ::rmdir(spill.c_str());
    return ok ? nwritten.load() : -1;
}

} // namespace adacpp
