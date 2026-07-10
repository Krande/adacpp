#pragma once
// Threaded, OCC-free STEP -> STL / OBJ core, mirroring step_to_glb_stream.h: the SAME native reader
// + parallel libtess2 tessellation, but it bakes each instance's world placement and streams the
// triangles straight to a binary STL or Wavefront OBJ file. Peak memory is O(one solid's mesh +
// per-lane buffer) — no whole-model buffer, no Python round-trip, no GIL.
//
// Mesh containers carry no instancing/materials, so each worker bakes its solids' world triangles
// into a private lane temp file; assembly concatenates the lanes (STL: header+count+raw facets;
// OBJ: all vertices, then faces with a running offset — vertices are unshared, 3 per triangle, so a
// lane's faces are fully determined by its (vertex offset, triangle count)).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "mem_trim.h"
#include "mem_tune.h"
#include "effective_concurrency.h"
#include "posix_compat.h"

#include "../cadit/step/step_reader.h"
#include "../geom/neutral/ngeom_profile.h"
#include "../geom/neutral/ngeom_tessellate.h"

namespace adacpp {

enum class MeshFormat { STL, OBJ };

namespace meshwrite {

inline void bake(const std::array<float, 16> &M, float usc, const float p[3], float out[3]); // fwd

// One worker lane: buffers geometry to a temp file, tracks its triangle count.
//  STL: raw 50-byte little-endian facets (no header), 3 unshared verts/facet — STL has no vertex
//       sharing, so per-triangle is the format's natural form.
//  OBJ: WELDED — each instance's unique tessellated vertices are written once ('v' lines, .objv) and
//       triangles reference them as lane-local 0-based indices stored binary in .objf. The assembly
//       concatenates the 'v' files and rewrites the indices as 'f' lines with a per-lane global base.
//       Avoids the 3x vertex blow-up of unshared per-triangle 'v' lines (crane OBJ 8.25 GB -> ~3 GB,
//       ~6x fewer float-formats).
struct MeshLane {
    std::string path;  // STL: .stlraw ; OBJ: .objv (vertex lines)
    std::string fpath; // OBJ only: .objf (binary uint32 index triples, lane-local 0-based)
    std::FILE *fp = nullptr;
    std::FILE *ffp = nullptr; // OBJ faces
    uint64_t tris = 0;
    uint64_t vcount = 0; // OBJ: running lane-local vertex count
    MeshFormat fmt;
    std::vector<char> buf;
    std::vector<uint32_t> fbuf; // OBJ: pending face indices

    MeshLane(const std::string &dir, int t, MeshFormat f) : fmt(f) {
        std::string base = dir + "/lane_" + std::to_string(t);
        path = base + (f == MeshFormat::STL ? ".stlraw" : ".objv");
        fp = std::fopen(path.c_str(), "wb");
        buf.reserve(1 << 20);
        if (f == MeshFormat::OBJ) {
            fpath = base + ".objf";
            ffp = std::fopen(fpath.c_str(), "wb");
            fbuf.reserve(1 << 18);
        }
    }
    ~MeshLane() {
        if (fp)
            std::fclose(fp);
        if (ffp)
            std::fclose(ffp);
        std::error_code ec;
        std::filesystem::remove(path, ec);
        if (!fpath.empty())
            std::filesystem::remove(fpath, ec);
    }
    void flush() {
        if (!buf.empty() && fp) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
        if (!fbuf.empty() && ffp) {
            std::fwrite(fbuf.data(), sizeof(uint32_t), fbuf.size(), ffp);
            fbuf.clear();
        }
        if (fp)
            std::fflush(fp); // ensure the lane file is complete on disk before assembly reads it
        if (ffp)
            std::fflush(ffp);
    }
    void put(const char *p, size_t n) {
        buf.insert(buf.end(), p, p + n);
        if (buf.size() >= (1 << 20))
            flush();
    }
    // STL per-facet (3 unshared verts). OBJ uses add_instance() instead.
    void facet(const float v0[3], const float v1[3], const float v2[3]) {
        float n[3] = {(v1[1] - v0[1]) * (v2[2] - v0[2]) - (v1[2] - v0[2]) * (v2[1] - v0[1]),
                      (v1[2] - v0[2]) * (v2[0] - v0[0]) - (v1[0] - v0[0]) * (v2[2] - v0[2]),
                      (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0])};
        float ln = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (ln > 0) {
            n[0] /= ln;
            n[1] /= ln;
            n[2] /= ln;
        }
        char rec[50];
        std::memcpy(rec, n, 12);
        std::memcpy(rec + 12, v0, 12);
        std::memcpy(rec + 24, v1, 12);
        std::memcpy(rec + 36, v2, 12);
        rec[48] = 0;
        rec[49] = 0;
        put(rec, 50);
        ++tris;
    }
    // OBJ welded instance: bake + write each unique vertex once, faces as lane-local indices.
    void add_instance(const std::vector<float> &positions, const std::vector<uint32_t> &indices,
                      const std::array<float, 16> &M, float usc) {
        size_t nv = positions.size() / 3;
        char line[96];
        for (size_t j = 0; j < nv; ++j) {
            float w[3];
            bake(M, usc, &positions[3 * j], w);
            int n = std::snprintf(line, sizeof line, "v %.6g %.6g %.6g\n", w[0], w[1], w[2]);
            if (n > 0)
                put(line, (size_t) n);
        }
        for (size_t e = 0; e + 2 < indices.size(); e += 3) {
            fbuf.push_back((uint32_t) (vcount + indices[e]));
            fbuf.push_back((uint32_t) (vcount + indices[e + 1]));
            fbuf.push_back((uint32_t) (vcount + indices[e + 2]));
            ++tris;
            if (fbuf.size() >= (1u << 18)) {
                std::fwrite(fbuf.data(), sizeof(uint32_t), fbuf.size(), ffp);
                fbuf.clear();
            }
        }
        vcount += nv;
    }
};

// world = unit_scale * (M_colmajor * local). M is column-major 4x4 (translation in M[12..14]).
inline void bake(const std::array<float, 16> &M, float usc, const float p[3], float out[3]) {
    out[0] = usc * (M[0] * p[0] + M[4] * p[1] + M[8] * p[2] + M[12]);
    out[1] = usc * (M[1] * p[0] + M[5] * p[1] + M[9] * p[2] + M[13]);
    out[2] = usc * (M[2] * p[0] + M[6] * p[1] + M[10] * p[2] + M[14]);
}

} // namespace meshwrite

// Returns the number of triangles written, or -1 on I/O error.
inline long stream_step_to_mesh(const std::string &in_path, const std::string &out_path, MeshFormat fmt,
                                double deflection, double angular_deg, int num_threads,
                                const std::string &spill_dir = "", double model_scale = 0.0) {
    using namespace adacpp::ngeom;
    static const std::array<float, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    adacpp::prof::StepProfiler prof("stream_step_to_mesh");
    adacpp::tune_malloc_for_streaming(); // bound streaming peak RSS (mmap/trim tuning) before the pool

    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(in_path);
    if (!idx.ok())
        return -1;
    prof.phase("scan_index");

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
    tp.model_scale = model_scale; // >0 => adaptive per-surface density

    adacpp::step::Resolver master(idx);
    master.build_metadata(idx.lists);
    prof.phase("metadata");

    int nthreads = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();

    // LPT: heaviest solids first so big ones start while every thread is busy. HUGE roots
    // (the sorted prefix with >= HUGE_FACES faces) go further: they are processed first, one
    // at a time, with tessellate_doc's face-level pool — see step_to_glb_stream.h.
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
        // Tail-dominance test — see step_to_glb_stream.h: only a root bigger than a thread's
        // fair share of the whole file goes through the serialized face-parallel phase.
        const size_t fair_share = total_faces / (size_t) nthreads;
        while (n_huge < cost.size() && cost[n_huge].first >= HUGE_FACES && cost[n_huge].first >= fair_share)
            ++n_huge;
        prof.phase("lpt_order");
    }

    std::string spill;
    bool remove_after = false;
    char tmpl[] = "/tmp/adacpp_mesh_XXXXXX";
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

    std::deque<meshwrite::MeshLane> lanes;
    for (int t = 0; t < nthreads; ++t)
        lanes.emplace_back(spill, t, fmt);
    std::atomic<size_t> next{n_huge};

    // One root: resolve with the caller's resolver, tessellate (``tpp.threads`` > 1 runs the
    // face-level pool), bake into the caller's lane. Shared by the huge-prefix phase and the pool.
    auto process_root = [&](adacpp::step::Resolver &r, meshwrite::MeshLane &lane, size_t i, const TessParams &tpp) {
        NgeomRoot root = r.resolve_root(roots[i]);
        if (root.id.empty())
            return;
        NgeomDoc one;
        one.roots.push_back(std::move(root));
        TessMesh tm = tessellate_doc(one, tpp);
        if (tm.indices.empty())
            return;
        const NgeomRoot &rr = one.roots[0];
        const float usc = (float) r.unit_scale();
        const std::vector<std::array<float, 16>> &tfs = rr.transforms;
        size_t ninst = tfs.empty() ? 1 : tfs.size();
        for (size_t k = 0; k < ninst; ++k) {
            const std::array<float, 16> &M = tfs.empty() ? kIdentity : tfs[k];
            if (fmt == MeshFormat::OBJ) {
                lane.add_instance(tm.positions, tm.indices, M, usc); // welded
            } else {
                for (size_t e = 0; e + 2 < tm.indices.size(); e += 3) {
                    float w0[3], w1[3], w2[3];
                    meshwrite::bake(M, usc, &tm.positions[3 * tm.indices[e]], w0);
                    meshwrite::bake(M, usc, &tm.positions[3 * tm.indices[e + 1]], w1);
                    meshwrite::bake(M, usc, &tm.positions[3 * tm.indices[e + 2]], w2);
                    lane.facet(w0, w1, w2);
                }
            }
        }
    };

    // Phase A — huge prefix, one root at a time with every thread on its faces.
    if (n_huge > 0) {
        TessParams tph = tp;
        tph.threads = nthreads;
        adacpp::step::Resolver r0(idx);
        r0.copy_metadata_from(master);
        for (size_t i = 0; i < n_huge; ++i) {
            process_root(r0, lanes[0], i, tph);
            r0.clear_geom_cache();
        }
        adacpp::mem_trim();
    }

    // Phase B — per-solid worker pool over the rest.
    auto worker = [&](int t) {
        adacpp::step::Resolver r(idx);
        r.copy_metadata_from(master);
        meshwrite::MeshLane &lane = lanes[t];
        int local = 0;
        for (;;) {
            size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= roots.size())
                break;
            process_root(r, lane, i, tp);
            r.clear_geom_cache();
            if (++local % 128 == 0)
                adacpp::mem_trim();
        }
        lane.flush();
    };

    std::vector<std::thread> pool;
    pool.reserve(nthreads - 1);
    for (int t = 1; t < nthreads; ++t)
        pool.emplace_back(worker, t);
    worker(0);
    for (std::thread &th : pool)
        th.join();
    prof.phase("stream(resolve+tess+bake)");

    uint64_t total = 0;
    for (auto &l : lanes)
        total += l.tris;

    // ── assemble the final file from the lane temp files ──
    bool ok = false;
    std::FILE *out = std::fopen(out_path.c_str(), "wb");
    if (out) {
        auto copy_lane = [&](const std::string &p) {
            std::FILE *in = std::fopen(p.c_str(), "rb");
            if (!in)
                return;
            std::vector<char> b(1 << 20);
            size_t n;
            while ((n = std::fread(b.data(), 1, b.size(), in)) > 0)
                std::fwrite(b.data(), 1, n, out);
            std::fclose(in);
        };
        if (fmt == MeshFormat::STL) {
            char header[80] = {0};
            std::fwrite(header, 1, 80, out);
            uint32_t cnt = (uint32_t) total;
            std::fwrite(&cnt, 4, 1, out);
            for (auto &l : lanes)
                copy_lane(l.path);
        } else { // OBJ: concat welded 'v' files, then faces from the binary index lanes (+global base)
            for (auto &l : lanes)
                copy_lane(l.path); // .objv vertex files, in lane order
            std::vector<char> fb;
            fb.reserve(1 << 20);
            uint64_t base = 0; // verts written by prior lanes (1-based -> +1 at emit)
            for (auto &l : lanes) {
                std::FILE *fin = std::fopen(l.fpath.c_str(), "rb");
                if (fin) {
                    std::vector<uint32_t> ib(3 << 16);
                    size_t got;
                    while ((got = std::fread(ib.data(), sizeof(uint32_t), ib.size(), fin)) >= 3) {
                        for (size_t e = 0; e + 2 < got; e += 3) {
                            char line[64];
                            int n = std::snprintf(line, sizeof line, "f %llu %llu %llu\n",
                                                  (unsigned long long) (base + ib[e] + 1),
                                                  (unsigned long long) (base + ib[e + 1] + 1),
                                                  (unsigned long long) (base + ib[e + 2] + 1));
                            fb.insert(fb.end(), line, line + n);
                        }
                        if (fb.size() >= (1 << 20)) {
                            std::fwrite(fb.data(), 1, fb.size(), out);
                            fb.clear();
                        }
                    }
                    std::fclose(fin);
                }
                base += l.vcount;
            }
            if (!fb.empty())
                std::fwrite(fb.data(), 1, fb.size(), out);
        }
        std::fclose(out);
        ok = true;
    }
    prof.phase("assemble");

    lanes.clear(); // remove lane temp files (destructors)
    if (remove_after)
        ::rmdir(spill.c_str());
    prof.note("threads", nthreads);
    return ok ? (long) total : -1;
}

} // namespace adacpp
