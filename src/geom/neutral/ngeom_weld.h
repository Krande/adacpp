// Weld a flat-shaded triangle-soup mesh (the tessellator emits 3 independent verts per triangle, each
// carrying that triangle's face normal — emit_tri in ngeom_tessellate.cpp) into a compact INDEXED
// mesh: coincident positions collapse to one shared vertex, and each vertex's normal becomes the
// area-weighted average of its incident faces — but split at a crease angle so hard edges stay sharp.
//
// This is what makes the native GLB match the OCC/ifcopenshell output density: a swept-disk rebar
// drops from 3 verts/tri (soup) to ~0.5 (indexed, smooth-shaded), curved surfaces shade smoothly, and
// box/prism corners keep crisp facets. Applies to any geometry type. Non-triangle (LINES) meshes and
// degenerate input pass through untouched.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "ngeom_math.h" // Vec3

namespace adacpp::ngeom {

// positions/normals: flat xyz (3 per vertex, parallel). indices: flat (3 per triangle). Rewrites all
// three in place to the welded, indexed form. crease_deg: incident faces whose normals differ by more
// than this at a shared position stay as separate vertices (hard edge).
inline void weld_mesh(std::vector<float> &positions, std::vector<uint32_t> &indices,
                      std::vector<float> &normals, double crease_deg = 40.0) {
    const size_t nt = indices.size() / 3;
    const size_t nv = positions.size() / 3;
    if (nt == 0 || nv == 0)
        return;
    const bool have_normals = normals.size() == positions.size();

    // Relative weld tolerance from the bbox diagonal (unit-independent; coincident soup verts are
    // near-identical so any small grid welds them, but this also tolerates fp noise).
    Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (size_t v = 0; v < nv; ++v) {
        double x = positions[3 * v], y = positions[3 * v + 1], z = positions[3 * v + 2];
        lo.x = std::min(lo.x, x); lo.y = std::min(lo.y, y); lo.z = std::min(lo.z, z);
        hi.x = std::max(hi.x, x); hi.y = std::max(hi.y, y); hi.z = std::max(hi.z, z);
    }
    double diag = std::sqrt((hi.x - lo.x) * (hi.x - lo.x) + (hi.y - lo.y) * (hi.y - lo.y) +
                            (hi.z - lo.z) * (hi.z - lo.z));
    const double inv = 1.0 / (diag > 0 ? diag * 1e-6 : 1e-9);
    auto qcoord = [&](double c) -> int64_t { return (int64_t) std::llround(c * inv); };

    // Per-triangle face normal + area (area-weights the vertex-normal average).
    std::vector<Vec3> fnorm(nt);
    std::vector<double> farea(nt);
    for (size_t t = 0; t < nt; ++t) {
        uint32_t ia = indices[3 * t], ib = indices[3 * t + 1], ic = indices[3 * t + 2];
        Vec3 a{positions[3 * ia], positions[3 * ia + 1], positions[3 * ia + 2]};
        Vec3 b{positions[3 * ib], positions[3 * ib + 1], positions[3 * ib + 2]};
        Vec3 c{positions[3 * ic], positions[3 * ic + 1], positions[3 * ic + 2]};
        Vec3 n = (b - a).cross(c - a);
        double len = n.norm();
        farea[t] = 0.5 * len;
        fnorm[t] = len > 1e-30 ? Vec3{n.x / len, n.y / len, n.z / len} : Vec3{0, 0, 1};
    }

    // Group soup corners by quantized position. A dense group id per unique key + a CSR
    // (gstart offsets / corners) layout — one flat uint32 corner array instead of a
    // std::vector-per-key. The old map-of-vectors did one heap allocation PER soup vertex, so a
    // 61k-face solid's 16.9M-vertex soup churned ~2GB of tiny nodes/vectors; CSR holds the same
    // data in ~0.5GB and never allocates per vertex. Bonus: numbering output verts in
    // first-appearance (spatially coherent) order rather than hash order lets meshopt compress the
    // merged GLB markedly better (469826: 130MB -> 83MB). Geometrically identical to the old weld.
    struct KeyHash {
        size_t operator()(const std::array<int64_t, 3> &k) const {
            uint64_t h = 1469598103934665603ull;
            for (int64_t q : k) { h ^= (uint64_t) q; h *= 1099511628211ull; }
            return (size_t) h;
        }
    };
    // First-appearance dense group id per unique quantized position. gid_of[v] indexes the group of
    // soup vertex v; groups are numbered in v order, so the CSR below lists each group's corners in
    // ascending v — the same order the old per-key push_back produced, keeping the greedy crease
    // clustering (and thus the welded normals/topology) identical. Output vertices are numbered in
    // group order rather than the old hash order: geometrically identical, deterministically renumbered.
    std::unordered_map<std::array<int64_t, 3>, uint32_t, KeyHash> gid;
    std::vector<uint32_t> gid_of(nv);
    for (size_t v = 0; v < nv; ++v) {
        std::array<int64_t, 3> key{qcoord(positions[3 * v]), qcoord(positions[3 * v + 1]),
                                   qcoord(positions[3 * v + 2])};
        gid_of[v] = gid.emplace(key, (uint32_t) gid.size()).first->second;
    }
    const size_t ng = gid.size();
    // Counting sort corners into group order: gstart[g] is the start of group g's corner run.
    std::vector<uint32_t> gstart(ng + 1, 0);
    for (size_t v = 0; v < nv; ++v)
        ++gstart[gid_of[v] + 1];
    for (size_t g = 0; g < ng; ++g)
        gstart[g + 1] += gstart[g];
    std::vector<uint32_t> corners(nv);
    {
        std::vector<uint32_t> cur(gstart.begin(), gstart.end() - 1); // per-group write cursor
        for (size_t v = 0; v < nv; ++v)
            corners[cur[gid_of[v]]++] = (uint32_t) v;
    }

    const double crease_cos = std::cos(crease_deg * PI / 180.0);
    std::vector<float> npos, nnrm;
    std::vector<uint32_t> remap(nv);
    npos.reserve(positions.size() / 4);
    nnrm.reserve(positions.size() / 4);
    std::vector<Vec3> cluster_dir; // representative (first face's normal), reused across groups
    std::vector<Vec3> cluster_acc; // area-weighted accumulated normal
    std::vector<int> which;
    std::vector<uint32_t> cluster_vid;
    for (size_t g = 0; g < ng; ++g) {
        const uint32_t cs = gstart[g], nc = gstart[g + 1] - cs;
        // Cluster this position's incident-face normals by crease angle (greedy — a vertex's incident
        // faces are locally coherent, so smooth surfaces make one cluster, a 90-degree edge makes two).
        cluster_dir.clear();
        cluster_acc.clear();
        which.assign(nc, 0);
        for (uint32_t ci = 0; ci < nc; ++ci) {
            size_t t = corners[cs + ci] / 3; // soup: old vertex belongs to exactly one triangle
            const Vec3 &fn = fnorm[t];
            int found = -1;
            for (size_t cl = 0; cl < cluster_dir.size(); ++cl)
                if (cluster_dir[cl].dot(fn) >= crease_cos) { found = (int) cl; break; }
            if (found < 0) {
                found = (int) cluster_dir.size();
                cluster_dir.push_back(fn);
                cluster_acc.push_back(Vec3{0, 0, 0});
            }
            cluster_acc[found] = cluster_acc[found] + fn * farea[t];
            which[ci] = found;
        }
        uint32_t p0 = corners[cs];
        cluster_vid.assign(cluster_dir.size(), 0);
        for (size_t cl = 0; cl < cluster_dir.size(); ++cl) {
            cluster_vid[cl] = (uint32_t) (npos.size() / 3);
            npos.push_back(positions[3 * p0]);
            npos.push_back(positions[3 * p0 + 1]);
            npos.push_back(positions[3 * p0 + 2]);
            if (have_normals) {
                Vec3 n = cluster_acc[cl];
                double l = n.norm();
                Vec3 un = l > 1e-30 ? Vec3{n.x / l, n.y / l, n.z / l} : cluster_dir[cl];
                nnrm.push_back((float) un.x);
                nnrm.push_back((float) un.y);
                nnrm.push_back((float) un.z);
            }
        }
        for (uint32_t ci = 0; ci < nc; ++ci)
            remap[corners[cs + ci]] = cluster_vid[which[ci]];
    }

    for (uint32_t &i : indices)
        i = remap[i];
    positions = std::move(npos);
    if (have_normals)
        normals = std::move(nnrm);
}

} // namespace adacpp::ngeom
