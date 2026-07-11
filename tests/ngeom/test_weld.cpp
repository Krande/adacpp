// Parity test for weld_mesh (ngeom_weld.h): the CSR grouping must produce a mesh geometrically
// identical to the previous map-of-vectors grouping. The old algorithm is reproduced here verbatim
// as reference_weld(); both are run on the same fixtures and compared corner-by-corner (welded
// positions + smoothed/creased normals). Output vertices are renumbered by the rewrite, but every
// triangle reproduces the same original positions and the same per-corner normals, so a direct
// t-th-triangle comparison is exact (weld rewrites indices in place, never reorders triangles).
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <vector>

#include "ngeom_weld.h"

using adacpp::ngeom::Vec3;
using adacpp::ngeom::weld_mesh;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                              \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

// ---- reference weld: the pre-CSR map-of-vectors implementation, verbatim ---------------------------
static void reference_weld(std::vector<float> &positions, std::vector<uint32_t> &indices,
                           std::vector<float> &normals, double crease_deg = 40.0) {
    const size_t nt = indices.size() / 3;
    const size_t nv = positions.size() / 3;
    if (nt == 0 || nv == 0)
        return;
    const bool have_normals = normals.size() == positions.size();
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
    struct KeyHash {
        size_t operator()(const std::array<int64_t, 3> &k) const {
            uint64_t h = 1469598103934665603ull;
            for (int64_t q : k) { h ^= (uint64_t) q; h *= 1099511628211ull; }
            return (size_t) h;
        }
    };
    std::unordered_map<std::array<int64_t, 3>, std::vector<uint32_t>, KeyHash> groups;
    groups.reserve(nv);
    for (size_t v = 0; v < nv; ++v)
        groups[{qcoord(positions[3 * v]), qcoord(positions[3 * v + 1]), qcoord(positions[3 * v + 2])}].push_back(
            (uint32_t) v);
    const double crease_cos = std::cos(crease_deg * 3.14159265358979323846 / 180.0);
    std::vector<float> npos, nnrm;
    std::vector<uint32_t> remap(nv);
    for (auto &[key, corners] : groups) {
        std::vector<Vec3> cluster_dir, cluster_acc;
        std::vector<int> which(corners.size());
        for (size_t ci = 0; ci < corners.size(); ++ci) {
            size_t t = corners[ci] / 3;
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
        uint32_t p0 = corners[0];
        std::vector<uint32_t> cluster_vid(cluster_dir.size());
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
        for (size_t ci = 0; ci < corners.size(); ++ci)
            remap[corners[ci]] = cluster_vid[which[ci]];
    }
    for (uint32_t &i : indices)
        i = remap[i];
    positions = std::move(npos);
    if (have_normals)
        normals = std::move(nnrm);
}

// ---- fixtures --------------------------------------------------------------------------------------

struct Mesh {
    std::vector<float> pos, nrm;
    std::vector<uint32_t> idx;
};

// A flat-shaded triangle soup: one triangle emits 3 fresh verts, each carrying the face normal.
static void add_tri(Mesh &m, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 n = (b - a).cross(c - a);
    double l = n.norm();
    Vec3 fn = l > 1e-30 ? Vec3{n.x / l, n.y / l, n.z / l} : Vec3{0, 0, 1};
    for (const Vec3 &p : {a, b, c}) {
        uint32_t base = (uint32_t) (m.pos.size() / 3);
        m.pos.push_back((float) p.x); m.pos.push_back((float) p.y); m.pos.push_back((float) p.z);
        m.nrm.push_back((float) fn.x); m.nrm.push_back((float) fn.y); m.nrm.push_back((float) fn.z);
        m.idx.push_back(base);
    }
}

static void add_quad(Mesh &m, Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    add_tri(m, a, b, c);
    add_tri(m, a, c, d);
}

// Axis-aligned box as a flat soup (12 tris) — every corner is shared by 3 faces at 90 degrees, so
// welding must SPLIT it into (at least) 3 crease clusters, not average across the sharp edges.
static Mesh make_box(double s) {
    Mesh m;
    Vec3 p000{0, 0, 0}, p100{s, 0, 0}, p110{s, s, 0}, p010{0, s, 0};
    Vec3 p001{0, 0, s}, p101{s, 0, s}, p111{s, s, s}, p011{0, s, s};
    add_quad(m, p000, p010, p110, p100); // z-
    add_quad(m, p001, p101, p111, p011); // z+
    add_quad(m, p000, p100, p101, p001); // y-
    add_quad(m, p010, p011, p111, p110); // y+
    add_quad(m, p000, p001, p011, p010); // x-
    add_quad(m, p100, p110, p111, p101); // x+
    return m;
}

// A tessellated flat plate (grid of coplanar quads) — all incident faces coplanar, so every shared
// vertex welds into ONE smooth cluster; exercises high fan-in (interior verts shared by 6 corners).
static Mesh make_grid(int n, double s) {
    Mesh m;
    double d = s / n;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            Vec3 a{i * d, j * d, 0}, b{(i + 1) * d, j * d, 0}, c{(i + 1) * d, (j + 1) * d, 0}, e{i * d, (j + 1) * d, 0};
            add_quad(m, a, b, c, e);
        }
    return m;
}

// ---- comparison ------------------------------------------------------------------------------------

static bool feq(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }

// Both welds reproduce input triangle t from the same original positions and rewrite indices in place
// (no triangle reordering), so compare corner-by-corner: welded position must equal the ORIGINAL soup
// position, and the two impls' per-corner normals must match. Also require identical dedup vertex count.
static void compare(const char *name, const Mesh &orig) {
    Mesh a = orig, b = orig; // fresh copies (weld rewrites in place)
    weld_mesh(a.pos, a.idx, a.nrm);
    reference_weld(b.pos, b.idx, b.nrm);

    CHECK(a.pos.size() == b.pos.size(), name); // same dedup vertex count
    CHECK(a.idx.size() == b.idx.size() && a.idx.size() == orig.idx.size(), name);

    const size_t nt = orig.idx.size() / 3;
    bool pos_ok = true, nrm_ok = true;
    for (size_t k = 0; k < orig.idx.size(); ++k) {
        uint32_t oc = orig.idx[k]; // original soup corner index
        uint32_t ai = a.idx[k], bi = b.idx[k];
        for (int t = 0; t < 3; ++t) {
            if (!feq(a.pos[3 * ai + t], orig.pos[3 * oc + t]))
                pos_ok = false; // new weld must preserve the original corner position
            if (!feq(a.pos[3 * ai + t], b.pos[3 * bi + t]))
                pos_ok = false;
            if (!feq(a.nrm[3 * ai + t], b.nrm[3 * bi + t]))
                nrm_ok = false; // new normal == reference normal for this corner
        }
    }
    CHECK(pos_ok, name);
    CHECK(nrm_ok, name);
    std::printf("  %-16s tris=%zu  verts: soup=%zu welded=%zu (ref=%zu)  pos=%s nrm=%s\n", name, nt,
                orig.pos.size() / 3, a.pos.size() / 3, b.pos.size() / 3, pos_ok ? "ok" : "DIFF",
                nrm_ok ? "ok" : "DIFF");
}

// Independent sanity checks on the CSR output itself (not just parity vs the old code).
static void semantic_checks() {
    // Flat grid: interior vertices must fully weld (welded count == unique grid nodes).
    Mesh g = make_grid(4, 1.0);
    Mesh w = g;
    weld_mesh(w.pos, w.idx, w.nrm);
    CHECK(w.pos.size() / 3 == 25, "grid welds to (n+1)^2 nodes"); // 5x5 lattice, one smooth cluster each
    // Box: 8 geometric corners, each split into 3 crease clusters -> 24 welded verts.
    Mesh bx = make_box(2.0);
    Mesh wb = bx;
    weld_mesh(wb.pos, wb.idx, wb.nrm);
    CHECK(wb.pos.size() / 3 == 24, "box corners split into 3 crease clusters each");
    // Every box normal axis-aligned unit (crease kept the flat face normals, no averaging across edges).
    bool axis = true;
    for (size_t v = 0; v < wb.nrm.size() / 3; ++v) {
        float x = std::fabs(wb.nrm[3 * v]), y = std::fabs(wb.nrm[3 * v + 1]), z = std::fabs(wb.nrm[3 * v + 2]);
        float mx = std::max(x, std::max(y, z));
        if (!feq(mx, 1.0f, 1e-4f))
            axis = false;
    }
    CHECK(axis, "box welded normals stay axis-aligned (crease preserved)");
}

int main() {
    std::printf("weld_mesh CSR parity + semantics:\n");
    compare("shared-edge", [] { Mesh m; add_quad(m, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}); return m; }());
    compare("box", make_box(2.0));
    compare("grid8", make_grid(8, 3.0));
    compare("grid32", make_grid(32, 10.0)); // 2048 tris, high fan-in, scale stress
    semantic_checks();
    if (g_fail) {
        std::printf("weld: %d checks FAILED\n", g_fail);
        return 1;
    }
    std::printf("weld: all checks passed\n");
    return 0;
}
