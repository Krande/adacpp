#pragma once
// Shared pieces of the sharded STEP/IFC -> GLB path (step_glb_shard.h, ifc_glb_shard.h): N independent
// single-threaded workers, each its own wasm instance with its own memory, that split one conversion
// through files in a directory they all see (an OPFS directory in the browser).
//
//   prepare   one worker scans the file once, builds the metadata, orders the roots heaviest-first
//             (LPT) with their cost estimates, lists the HUGE roots apart, and saves it all to an index
//             file. Every shard opens with that index: no shard scans the file again.
//   batches   plan_batches() cuts the LPT order into contiguous batches of about equal estimated cost,
//             so the expensive roots go out one at a time and the cheap tail in larger groups.
//   huge      a root bigger than one worker's fair share is split by FACE RANGE: any worker resolves and
//             tessellates just its faces, unwelded, into a part file; once every range is done one
//             worker joins the parts in face order and welds once -- what the serial path does -- so
//             the triangles are the serial path's.
//   merge     each shard persists its spill lane; one worker merges the lanes into the GLB.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "../cadit/step/step_binio.h"
#include "../geom/neutral/ngeom_tessellate.h"
#include "../geom/neutral/ngeom_weld.h"

namespace adacpp::shard {

// The native pool's huge-root test (step_to_glb_stream.h): at least this many faces AND at least one
// worker's fair share of the file's total estimated cost.
constexpr size_t HUGE_FACES = 2048;

// Sort (cost, id) heaviest-first, stable so equal costs keep file order.
inline void lpt_sort(std::vector<std::pair<size_t, long>> &cost) {
    std::stable_sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
}

// How many of the heaviest roots (a prefix of the LPT order) are huge for `nworkers` workers: at least
// HUGE_FACES and either a worker's fair share or `heavy` (the IFC rule: see stream_ifc_to_glb).
inline size_t count_huge(const std::vector<std::pair<size_t, long>> &lpt, int nworkers, size_t heavy = (size_t) -1) {
    if (nworkers < 2)
        return 0;
    size_t total = 0;
    for (const auto &c : lpt)
        total += c.first;
    const size_t fair_share = total / (size_t) nworkers;
    size_t n = 0;
    while (n < lpt.size() && lpt[n].first >= HUGE_FACES && (lpt[n].first >= fair_share || lpt[n].first >= heavy))
        ++n;
    return n;
}

// An IFC product this heavy holds ~1 GB while it resolves and tessellates: split it by face too.
constexpr size_t IFC_HEAVY_FACES = 8192;

// Contiguous batch ends over `costs` (in dispatch order): each batch costs about total/nbatches -- a
// root costing more is a batch of its own -- and holds at most n/nbatches roots, because the cost
// estimates only rank roots roughly: a tail batch of many "cheap" roots must not hide a slow one.
// Returns the exclusive end index of each batch.
inline std::vector<long> plan_batches(const std::vector<uint64_t> &costs, long nbatches) {
    std::vector<long> ends;
    if (costs.empty())
        return ends;
    nbatches = std::max(1L, nbatches);
    uint64_t total = 0;
    for (uint64_t c : costs)
        total += c ? c : 1;
    const uint64_t target = std::max<uint64_t>(1, total / (uint64_t) nbatches);
    const size_t max_count = std::max<size_t>(1, (costs.size() + (size_t) nbatches - 1) / (size_t) nbatches);
    // The heaviest roots come first and their estimates are least trustworthy in absolute terms (an
    // IFC product's cost can sit far below the per-batch target even when it is the slowest one), so
    // the count cap ramps from ONE root at the head to max_count over the first quarter of the order.
    const size_t ramp = std::max<size_t>(1, costs.size() / 4);
    uint64_t acc = 0;
    size_t count = 0, start = 0;
    for (size_t i = 0; i < costs.size(); ++i) {
        acc += costs[i] ? costs[i] : 1;
        const size_t cap = std::max<size_t>(1, std::min(max_count, start * max_count / ramp));
        if (acc >= target || ++count >= cap) {
            start = i + 1;
            ends.push_back((long) i + 1);
            acc = 0;
            count = 0;
        }
    }
    if (ends.empty() || ends.back() != (long) costs.size())
        ends.push_back((long) costs.size());
    return ends;
}

inline std::string part_path(const std::string &dir, long h, long f0) {
    return dir + "/huge" + std::to_string(h) + "_" + std::to_string(f0) + ".part";
}

// One face range of a huge root, tessellated unwelded, to its part file. Returns the triangles, or -1
// when the file could not be written.
inline long write_part(const std::string &path, const ngeom::TessMesh &tm) {
    adacpp::step::binio::Out o(path);
    o.vec(tm.positions);
    o.vec(tm.indices);
    o.vec(tm.normals);
    return o ? (long) (tm.indices.size() / 3) : -1;
}

// Join a huge root's parts -- ranges of `chunk` faces from 0 to `nfaces` -- in face order, then weld once
// when `weld`. The part files are removed. False when a part is missing or truncated.
inline bool join_parts(const std::string &dir, long h, long nfaces, long chunk, bool weld, ngeom::TessMesh &tm) {
    if (chunk < 1)
        return false;
    tm = ngeom::TessMesh{};
    for (long f0 = 0; f0 < nfaces; f0 += chunk) {
        const std::string path = part_path(dir, h, f0);
        std::vector<float> pos, nrm;
        std::vector<uint32_t> idx;
        {
            adacpp::step::binio::In in(path);
            if (!in)
                return false;
            in.vec(pos);
            in.vec(idx);
            in.vec(nrm);
            if (!in)
                return false;
        }
        std::remove(path.c_str());
        const uint32_t base = (uint32_t) (tm.positions.size() / 3);
        tm.positions.insert(tm.positions.end(), pos.begin(), pos.end());
        tm.normals.insert(tm.normals.end(), nrm.begin(), nrm.end());
        tm.indices.reserve(tm.indices.size() + idx.size());
        for (uint32_t i : idx)
            tm.indices.push_back(base + i);
    }
    if (weld)
        ngeom::weld_mesh(tm.positions, tm.indices, tm.normals);
    return true;
}

} // namespace adacpp::shard
