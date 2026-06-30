// Solidify a tessellated open face shell into a closed thin solid by extruding it along the
// area-weighted average surface normal. Native mirror of adapy's
// ``ada.occ.tessellating._thicken_face_mesh`` — used so a curved plate (PlateCurved) ships a
// solid that matches the OCC prism, instead of a bare one-sided shell. Header-only, no OCC.
//
// Topology contract: the input must be an INDEXED mesh (boundary vertices shared by index, as
// the libtess2 tessellator produces). Boundary edges are detected by vertex index (undirected
// edge used by exactly one triangle) and closed with side walls, so a clean open 2-manifold in
// yields a clean closed 2-manifold out.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "ngeom_tessellate.h" // TessMesh

namespace adacpp::ngeom {

// Extrude the open shell ``m`` by ``thickness`` along its mean surface normal, in place.
// Result = top face + bottom face (translated by thickness*normal, reversed winding) + side
// walls over the boundary loop. No-op if the mesh is empty or has no consistent normal.
inline void thicken_mesh(TessMesh &m, double thickness) {
    const size_t nidx = m.indices.size();
    const size_t ntri = nidx / 3;
    if (ntri == 0)
        return;
    const uint32_t n = static_cast<uint32_t>(m.positions.size() / 3);
    auto P = [&](uint32_t v, int k) { return static_cast<double>(m.positions[v * 3 + k]); };

    // Area-weighted average face normal (cross products are 2*area-scaled, so larger triangles
    // dominate — matching the adapy implementation).
    double nx = 0, ny = 0, nz = 0;
    for (size_t t = 0; t < ntri; ++t) {
        uint32_t a = m.indices[t * 3], b = m.indices[t * 3 + 1], c = m.indices[t * 3 + 2];
        double abx = P(b, 0) - P(a, 0), aby = P(b, 1) - P(a, 1), abz = P(b, 2) - P(a, 2);
        double acx = P(c, 0) - P(a, 0), acy = P(c, 1) - P(a, 1), acz = P(c, 2) - P(a, 2);
        nx += aby * acz - abz * acy;
        ny += abz * acx - abx * acz;
        nz += abx * acy - aby * acx;
    }
    double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nlen < 1e-12)
        return; // closed/degenerate mesh — no consistent normal
    const double vx = nx / nlen * thickness, vy = ny / nlen * thickness, vz = nz / nlen * thickness;

    std::vector<float> pos(m.positions);
    pos.reserve(m.positions.size() * 2);
    for (uint32_t v = 0; v < n; ++v) {
        pos.push_back(static_cast<float>(P(v, 0) + vx));
        pos.push_back(static_cast<float>(P(v, 1) + vy));
        pos.push_back(static_cast<float>(P(v, 2) + vz));
    }

    std::vector<uint32_t> idx;
    idx.reserve(nidx * 3);
    for (size_t i = 0; i < nidx; ++i)
        idx.push_back(m.indices[i]);    // top face
    for (size_t t = 0; t < ntri; ++t) { // bottom face: reversed + offset
        idx.push_back(m.indices[t * 3 + 2] + n);
        idx.push_back(m.indices[t * 3 + 1] + n);
        idx.push_back(m.indices[t * 3 + 0] + n);
    }

    // Boundary = undirected edges used by exactly one triangle (by vertex index).
    auto key = [](uint32_t a, uint32_t b) {
        uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
        return (static_cast<uint64_t>(lo) << 32) | hi;
    };
    std::unordered_map<uint64_t, int> cnt;
    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> ends; // directed, from first sighting
    for (size_t t = 0; t < ntri; ++t) {
        uint32_t v[3] = {m.indices[t * 3], m.indices[t * 3 + 1], m.indices[t * 3 + 2]};
        for (int e = 0; e < 3; ++e) {
            uint32_t a = v[e], b = v[(e + 1) % 3];
            uint64_t k = key(a, b);
            if (cnt.find(k) == cnt.end())
                ends[k] = {a, b};
            cnt[k]++;
        }
    }
    for (auto &kv : cnt) {
        if (kv.second != 1)
            continue;
        uint32_t a = ends[kv.first].first, b = ends[kv.first].second;
        // Two wall triangles spanning top edge (a,b) to bottom edge (a+n,b+n).
        idx.push_back(a);
        idx.push_back(b);
        idx.push_back(b + n);
        idx.push_back(a);
        idx.push_back(b + n);
        idx.push_back(a + n);
    }

    m.positions.swap(pos);
    m.indices.swap(idx);
    m.normals.clear(); // invalidated by the thicken; recompute if a renderer needs them
}

} // namespace adacpp::ngeom
