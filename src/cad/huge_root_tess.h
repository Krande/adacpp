#pragma once
// Tessellate ONE huge root on several threads by FACE RANGE -- resolve included. The native converters'
// phase A (stream_step_to_glb / stream_ifc_to_glb) used to resolve a huge root on one thread and only
// tessellate its faces in parallel; the serial resolve was a large share of a huge root's time, and it
// is why merely-large roots could not go through phase A without slowing a balanced file down.
//
// Here every thread resolves its own face slices with its own resolver (the readers' face windows:
// Resolver / IfcResolver::set_face_window) and tessellates them unwelded; the slices are appended in
// face order as soon as the next one in order is done (so at most the in-flight slices exist besides
// the joined mesh), and the joined root is welded once -- what the serial path does. Same triangles
// as the serial path, in the same order; per-face ranges are carried with their positions rebased.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "../geom/neutral/ngeom_tessellate.h"
#include "../geom/neutral/ngeom_weld.h"

namespace adacpp {

struct HugeRootMesh {
    ngeom::TessMesh mesh;  // welded (when tp.weld), face ranges rebased onto the whole root
    ngeom::NgeomRoot meta; // the root's colour, transforms, names and paths (no faces)
};

// `slice(t, lo, hi)` resolves faces [lo, hi) of the root on thread `t`'s resolver and returns the
// NgeomRoot (each thread uses only its own resolver). `nfaces` is the root's face count as the face
// window counts it.
template <class Slice>
HugeRootMesh tessellate_root_by_faces(int nthreads, size_t nfaces, const ngeom::TessParams &tp, Slice slice) {
    HugeRootMesh out;
    nthreads = std::max(1, nthreads);
    // 16 slices per thread: face costs vary a lot across a solid, and smaller slices keep the in-flight
    // part of the root small (a 61k-face solid at 3 threads: peak RSS 1.17 -> 0.94 GB vs 4).
    const size_t nchunks = std::max<size_t>(1, std::min(nfaces, (size_t) nthreads * 16));
    const size_t chunk = (nfaces + nchunks - 1) / nchunks;
    ngeom::TessParams tpu = tp;
    tpu.weld = false; // once, over the joined root
    tpu.threads = 1;  // the slices are the parallelism

    struct Done {
        ngeom::TessMesh tm;
        size_t nresolved = 0; // faces the slice resolved (face_seq continues across slices)
    };
    std::vector<std::optional<Done>> done(nchunks);
    std::mutex mu;
    size_t next_merge = 0, face_base = 0;
    bool have_meta = false;
    std::atomic<size_t> next{0};
    ngeom::TessMesh &m = out.mesh;

    auto append_ready = [&]() { // under mu: append every slice that is next in face order
        while (next_merge < nchunks && done[next_merge]) {
            Done &d = *done[next_merge];
            const uint32_t vbase = (uint32_t) (m.positions.size() / 3);
            const uint32_t ibase = (uint32_t) m.indices.size();
            m.positions.insert(m.positions.end(), d.tm.positions.begin(), d.tm.positions.end());
            m.normals.insert(m.normals.end(), d.tm.normals.begin(), d.tm.normals.end());
            m.indices.reserve(m.indices.size() + d.tm.indices.size());
            for (uint32_t i : d.tm.indices)
                m.indices.push_back(vbase + i);
            for (auto fr : d.tm.face_ranges) {
                fr.first_index += ibase;
                fr.face_seq += (uint32_t) face_base;
                m.face_ranges.push_back(fr);
            }
            face_base += d.nresolved;
            done[next_merge].reset(); // free the slice now that it is in the joined mesh
            ++next_merge;
        }
    };
    auto worker = [&](int t) {
        for (size_t c = next.fetch_add(1); c < nchunks; c = next.fetch_add(1)) {
            const size_t lo = c * chunk, hi = std::min(nfaces, lo + chunk);
            ngeom::NgeomDoc one;
            one.roots.push_back(slice(t, lo, hi));
            Done d;
            d.nresolved = one.roots[0].faces.size();
            d.tm = ngeom::tessellate_doc(one, tpu);
            std::lock_guard<std::mutex> lk(mu);
            // Meta from a slice that resolved faces: IFC composes the object placement onto a product
            // only when it has geometry.
            if (!have_meta && d.nresolved > 0) {
                out.meta = std::move(one.roots[0]);
                out.meta.faces.clear();
                have_meta = true;
            }
            done[c] = std::move(d);
            append_ready();
        }
    };
    std::vector<std::thread> pool;
    pool.reserve((size_t) nthreads - 1);
    for (int t = 1; t < nthreads; ++t)
        pool.emplace_back(worker, t);
    worker(0);
    for (std::thread &th : pool)
        th.join();
    if (tp.weld && !m.indices.empty())
        ngeom::weld_mesh(m.positions, m.indices, m.normals);
    return out;
}

} // namespace adacpp
