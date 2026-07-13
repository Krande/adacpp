// "Crows-nest" tessellation-spike detector over a triangulated mesh (flat positions + indices).
//
// Mirrors the viewer's client-side detector (adapy frontend src/utils/mesh_select/meshStats.ts —
// computeRangeStats) EXACTLY so a Python/audit scan and the browser inspector agree on what counts
// as distorted. A vertex shot out past the mesh body (the tessellation "crows-nest" bug) is an
// OUTLIER: take the robust (median-per-axis) centroid, the median vertex->centroid distance as the
// body scale, then any vertex beyond outlier_k x that median is a spike. spike_tris counts the
// thin/needle triangles (longest-edge^2 / (2*area) > aspect_min) that touch such an outlier.
//
// Kept dependency-free (no OCC / NGEOM types) so it can also compile into the frontend wasm module.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace adacpp::geom {

struct SpikeStats {
    double max_spike = 0.0; // worst vertex distance / median (~1 for a compact body; large for a spike)
    long spike_tris = 0;    // thin triangles touching an outlier vertex
    long triangles = 0;     // total triangles in the mesh
};

// positions: flat xyz (3 floats per vertex). indices: flat (3 per triangle) into the vertices.
inline SpikeStats mesh_spike_stats(const std::vector<float> &pos, const std::vector<uint32_t> &idx,
                                   double aspect_min = 8.0, double outlier_k = 4.0) {
    SpikeStats s;
    s.triangles = static_cast<long>(idx.size() / 3);
    if (idx.size() < 3 || pos.empty())
        return s;

    std::unordered_set<uint32_t> seen;
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        seen.insert(idx[i]);
        seen.insert(idx[i + 1]);
        seen.insert(idx[i + 2]);
    }
    std::vector<uint32_t> verts(seen.begin(), seen.end());
    const size_t n = verts.size();
    if (n == 0)
        return s;

    std::vector<double> xs(n), ys(n), zs(n);
    for (size_t k = 0; k < n; k++) {
        const size_t b = static_cast<size_t>(verts[k]) * 3;
        xs[k] = pos[b];
        ys[k] = pos[b + 1];
        zs[k] = pos[b + 2];
    }
    auto median = [](std::vector<double> a) -> double {
        if (a.empty())
            return 0.0;
        std::sort(a.begin(), a.end());
        const size_t m = a.size() / 2;
        return a.size() % 2 ? a[m] : 0.5 * (a[m - 1] + a[m]);
    };
    const double cx = median(xs), cy = median(ys), cz = median(zs);

    std::vector<double> dists(n);
    for (size_t k = 0; k < n; k++)
        dists[k] = std::sqrt((xs[k] - cx) * (xs[k] - cx) + (ys[k] - cy) * (ys[k] - cy) + (zs[k] - cz) * (zs[k] - cz));
    const double med_dist = median(dists);

    std::unordered_set<uint32_t> outliers;
    if (med_dist > 1e-9) {
        for (size_t k = 0; k < n; k++) {
            const double ratio = dists[k] / med_dist;
            if (ratio > s.max_spike)
                s.max_spike = ratio;
            if (ratio > outlier_k)
                outliers.insert(verts[k]);
        }
    }

    if (!outliers.empty()) {
        for (size_t i = 0; i + 2 < idx.size(); i += 3) {
            const uint32_t ia = idx[i], ib = idx[i + 1], ic = idx[i + 2];
            if (!outliers.count(ia) && !outliers.count(ib) && !outliers.count(ic))
                continue;
            const size_t a = static_cast<size_t>(ia) * 3, b = static_cast<size_t>(ib) * 3,
                         c = static_cast<size_t>(ic) * 3;
            const double abx = pos[b] - pos[a], aby = pos[b + 1] - pos[a + 1], abz = pos[b + 2] - pos[a + 2];
            const double acx = pos[c] - pos[a], acy = pos[c + 1] - pos[a + 1], acz = pos[c + 2] - pos[a + 2];
            const double bcx = pos[c] - pos[b], bcy = pos[c + 1] - pos[b + 1], bcz = pos[c + 2] - pos[b + 2];
            const double crx = aby * acz - abz * acy, cry = abz * acx - abx * acz, crz = abx * acy - aby * acx;
            const double tri_area = 0.5 * std::sqrt(crx * crx + cry * cry + crz * crz);
            const double abl = std::sqrt(abx * abx + aby * aby + abz * abz);
            const double acl = std::sqrt(acx * acx + acy * acy + acz * acz);
            const double bcl = std::sqrt(bcx * bcx + bcy * bcy + bcz * bcz);
            const double emax = std::max(abl, std::max(acl, bcl));
            if (tri_area > 0.0 && emax * emax > aspect_min * 2.0 * tri_area)
                s.spike_tris++;
        }
    }
    return s;
}

} // namespace adacpp::geom
