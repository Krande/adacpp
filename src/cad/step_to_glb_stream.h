#pragma once
// Threaded, OCC-free STEP -> GLB core, extracted from cad_py_wrap.cpp so it can be reused by both
// the nanobind Python binding AND the standalone STP2GLB CLI without dragging in OCCT or nanobind.
//
// Stream the .stp with the native reader (offset index + per-statement pread, bounded memory),
// tessellate each solid across `num_threads` worker threads (0 = auto = hardware_concurrency clamped
// to the cgroup cpu quota, see effective_concurrency.h), bake
// each placement's world transform + colour, and write a merge-by-colour GLB matching the adapy
// viewer's structure. `meshopt=true` bakes EXT_meshopt_compression inline. Returns the number of
// solids written, or -1 on I/O error.
//
// Every dependency here is header-only and OCC-free (the native STEP reader + neutral geom layer),
// so this header compiles in any OCC-free target.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mem_trim.h"
#include "mem_tune.h"
#include "effective_concurrency.h"
#include "posix_compat.h"

#include "../cadit/step/step_reader.h"
#include "../geom/neutral/ada_ext_schema.h" // GENERATED from the adapy ADA_EXT_data JSON schema
#include "../geom/neutral/ngeom_glb.h"
#include "../geom/neutral/ngeom_profile.h"
#include "../geom/neutral/ngeom_tessellate.h"

namespace adacpp {

// `spill_dir` controls where each lane's per-material GLB chunks are spilled to disk:
//   - empty  => a private mkdtemp dir under /tmp is created and removed once assembly is done.
//   - set    => that directory is used (created if missing) and is NOT removed afterwards, so a
//               caller can inspect the intermediate spill files. The lane temp files inside it are
//               still cleaned up by GlbSpillWriter's destructor; only the user-supplied dir survives.
inline long stream_step_to_glb(const std::string &in_path, const std::string &out_path, double deflection,
                               double angular_deg, int num_threads, bool meshopt, const std::string &spill_dir = "",
                               double model_scale = 0.0) {
    using namespace adacpp::ngeom;
    adacpp::prof::StepProfiler prof("stream_step_to_glb");
    adacpp::tune_malloc_for_streaming(); // bound streaming peak RSS (mmap/trim tuning) before the pool

    // File-backed offset index: mmap to scan (freed-behind), then pread each statement on demand so
    // the file lives in the OS page cache, not process RSS.
    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(in_path);
    if (!idx.ok())
        return -1;
    prof.phase("scan_index");

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
    tp.model_scale = model_scale; // >0 => adaptive per-surface density; the per-solid tpp copies inherit it

    // Metadata (colour/transform/path maps) once; workers copy these read-only maps.
    adacpp::step::Resolver master(idx);
    master.build_metadata(idx.lists);
    prof.phase("metadata");

    int nthreads = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();

    // LPT scheduling: order roots heaviest-first (by a cheap face-count proxy, no geometry built) so
    // the big solids start while every thread is still busy — the dynamic queue then fills the tail
    // with small ones instead of one thread chewing a giant solid alone at the end.
    //
    // HUGE roots (face count >= threshold) go further: LPT can start them early, but a single
    // 61k-face solid still pins ONE worker for the whole conversion (469826: one solid busy 54 s
    // while the other threads idle after 20 s). Those roots — a prefix of the sorted order — are
    // processed FIRST, one at a time, with tessellate_doc's face-level pool using every thread.
    std::vector<long> roots(idx.lists.roots.begin(), idx.lists.roots.end());
    size_t n_huge = 0;
    if (nthreads > 1) {
        constexpr size_t HUGE_FACES = 2048;
        std::vector<std::pair<size_t, long>> cost;
        cost.reserve(roots.size());
        size_t total_faces = 0;
        for (long sid : roots) {
            size_t fc = master.solid_face_count(sid);
            total_faces += fc;
            cost.emplace_back(fc, sid);
        }
        master.clear_geom_cache();
        std::sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (size_t i = 0; i < cost.size(); ++i)
            roots[i] = cost[i].second;
        // Tail-dominance test, not absolute size: phase A serializes resolve between its
        // face-pool bursts, so routing merely-large solids through it SLOWS a well-balanced
        // file (crane: 7291 solids, many 2-4k faces, pool efficiency already 3.5x -> phase A
        // cost it 26%). Only a root bigger than a thread's fair share of the whole file can
        // outlast the pool no matter how LPT schedules it.
        const size_t fair_share = total_faces / (size_t) nthreads;
        while (n_huge < cost.size() && cost[n_huge].first >= HUGE_FACES && cost[n_huge].first >= fair_share)
            ++n_huge;
        prof.phase("lpt_order");
    }

    // Resolve the spill directory: a private mkdtemp dir (auto-removed) when no spill_dir was given,
    // otherwise the caller's directory (created if missing, left in place afterwards).
    std::string spill;
    bool remove_after = false;
    char tmpl[] = "/tmp/adacpp_glb_XXXXXX";
    if (spill_dir.empty()) {
        if (char *dir = ::mkdtemp(tmpl)) { // unique spill dir (removed after assembly)
            spill = dir;
            remove_after = true;
        }
    } else {
        std::error_code ec;
        std::filesystem::create_directories(spill_dir, ec);
        if (std::filesystem::is_directory(spill_dir, ec))
            spill = spill_dir; // user-supplied: use as-is, do NOT remove
    }
    bool ok = false;
    std::atomic<int> nwritten{0};
    if (!spill.empty()) {
        { // lanes scoped so their temp files are removed before we (maybe) rmdir
            std::deque<adacpp::glb::GlbSpillWriter> lanes;
            for (int t = 0; t < nthreads; ++t)
                lanes.emplace_back(spill, t);
            std::atomic<size_t> next{n_huge};
            const bool tprof = prof.on(); // gate ALL per-solid clock reads -> zero cost in prod
            // One root: resolve with the caller's resolver, tessellate (``tpp.threads`` > 1 runs
            // tessellate_doc's face-level pool), bake + spill into the caller's lane. Shared by the
            // huge-prefix phase and the per-solid worker pool.
            auto process_root = [&](adacpp::step::Resolver &r, adacpp::glb::GlbSpillWriter &lane, size_t i,
                                    const TessParams &tpp) {
                NgeomRoot root = r.resolve_root(roots[i]);
                if (root.id.empty())
                    return;
                size_t fc = root.faces.size();
                NgeomDoc one;
                one.roots.push_back(std::move(root));
                std::chrono::steady_clock::time_point tt0;
                if (tprof)
                    tt0 = std::chrono::steady_clock::now();
                TessMesh tm = tessellate_doc(one, tpp);
                if (tprof && prof.timing())
                    prof.solid_timed(
                        roots[i], fc,
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tt0).count());
                if (tm.indices.empty())
                    return;
                prof.solid(tm.indices.size() / 3);
                const NgeomRoot &rr = one.roots[0];
                adacpp::glb::GlbSolid gs;
                gs.positions = std::move(tm.positions);
                gs.indices = std::move(tm.indices);
                gs.color = {rr.cr, rr.cg, rr.cb, rr.ca}; // grey default when !has_color
                gs.transforms = rr.transforms;
                gs.id = rr.id; // fallback leaf name
                // gid = the solid's own product name (last level of its assembly path);
                // the writer names each placement gid / gid/k+1. Fall back to the solid name.
                if (!rr.instance_paths.empty() && !rr.instance_paths[0].empty())
                    gs.product_name = rr.instance_paths[0].back().second;
                gs.instance_paths = rr.instance_paths; // all placements, parallel to transforms
                // Convert the file's length unit (e.g. mm) to metres — adapy's default unit,
                // and what the viewer assumes. Without this the GLB is e.g. 1000x oversized,
                // which (besides looking wrong) defeats the viewer's edge-overlay spatial-hash
                // vertex welding (only valid for coordinates within ~1e5 units) -> its weldMap
                // overflows V8's 2^24 Map cap ("Map maximum size exceeded"). Scale local
                // positions + each instance transform's translation; rotation is unitless.
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
                nwritten.fetch_add(1, std::memory_order_relaxed);
            };
            // Phase A — the huge prefix, one root at a time BEFORE the pool starts: every thread
            // works the same solid's faces (tessellate_doc face pool), so a lone 61k-face monster
            // no longer pins one worker for the conversion's whole tail.
            if (n_huge > 0) {
                TessParams tph = tp;
                tph.threads = nthreads;
                adacpp::step::Resolver r0(idx);
                r0.copy_metadata_from(master);
                double busy_ms = 0;
                std::chrono::steady_clock::time_point b0;
                if (tprof)
                    b0 = std::chrono::steady_clock::now();
                for (size_t i = 0; i < n_huge; ++i) {
                    process_root(r0, lanes[0], i, tph);
                    r0.clear_geom_cache();
                }
                if (tprof)
                    busy_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count();
                prof.thread_done(-1, busy_ms, n_huge); // tid -1 = the huge-prefix phase
                adacpp::mem_trim();
            }
            // Phase B — each worker pulls the remaining roots off the shared counter (dynamic
            // balancing handles the dense-solid long tail), resolving with its OWN caches + a copy
            // of the shared metadata, into its lane.
            auto worker = [&](int t) {
                adacpp::step::Resolver r(idx);
                r.copy_metadata_from(master);
                adacpp::glb::GlbSpillWriter &lane = lanes[t];
                int local = 0;
                double busy_ms = 0;
                for (;;) {
                    size_t i = next.fetch_add(1, std::memory_order_relaxed);
                    if (i >= roots.size())
                        break;
                    std::chrono::steady_clock::time_point b0;
                    if (tprof)
                        b0 = std::chrono::steady_clock::now();
                    process_root(r, lane, i, tp);
                    r.clear_geom_cache();
                    if (tprof)
                        busy_ms +=
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count();
                    if (++local % 128 == 0)
                        adacpp::mem_trim(); // return per-solid churn to the OS
                }
                prof.thread_done(t, busy_ms, (size_t) local);
            };
            std::vector<std::thread> pool;
            pool.reserve(nthreads - 1);
            for (int t = 1; t < nthreads; ++t)
                pool.emplace_back(worker, t);
            worker(0); // this thread runs lane 0
            for (std::thread &th : pool)
                th.join();
            prof.phase("stream(resolve+tess+spill)");

            std::vector<adacpp::glb::GlbSpillWriter *> lane_ptrs;
            for (adacpp::glb::GlbSpillWriter &l : lanes)
                lane_ptrs.push_back(&l);
            // ADA_EXT_data is produced from the schema-generated struct (ada_ext_schema.h) so it always
            // carries every field the viewer reads (design_objects / simulation_objects lists, version,
            // assembly_guid) — a partial object crashes the viewer on `design_objects.length`. Default =
            // a STEP import: empty design/sim arrays, the schema version, null assembly_guid.
            const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
            ok = adacpp::glb::write_glb_merged(out_path, lane_ptrs, ada_ext, meshopt);
            prof.phase(meshopt ? "write_glb_merged(meshopt)" : "write_glb_merged");
        }
        if (remove_after)
            ::rmdir(spill.c_str());
    }
    prof.note("threads", nthreads);
    return ok ? nwritten.load() : -1;
}

} // namespace adacpp
