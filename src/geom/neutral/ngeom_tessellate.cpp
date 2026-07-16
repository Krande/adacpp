#include "ngeom_tessellate.h"

#include <algorithm>
#include <atomic>
#include <thread>
#include <array>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <utility>
#include <vector>

#include "ngeom_boolean.h" // OCC-free CSG (Manifold) for boolean roots
#include "ngeom_bspline.h"
#include "ngeom_weld.h" // vertex weld + crease-angle smooth normals (per root)
#include "ngeom_surfaces.h"
#include "detria.hpp"   // vendored constrained Delaunay (the `cdt` track)
#include "tesselator.h" // vendored libtess2 (fails soft via setjmp/longjmp -> tessTesselate
                        // returns 0 on any sweep error; no panic to guard against, unlike the original wasm build)

// 1:1 port of crates/core/src/tessellate.rs onto the OCC-free neutral geometry layer.
// Function names and logic mirror the Rust source so the two tessellators stay in lockstep; any
// remaining divergence localizes to the parsing/serialization stage that feeds this layer.
namespace adacpp::ngeom {

using Uv = std::array<double, 2>;
using Tri = std::array<uint32_t, 3>;

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();
const bool FDBG = std::getenv("NGEOM_FDBG") != nullptr;
// NGEOM_TESSDBG: step-by-step tess-path logging (surface params, path, grid dims, pre/post-refine
// triangle counts, all tess params) — for head-to-head comparison with the reference tessellator's face-debug dump.
const bool TESSDBG = std::getenv("NGEOM_TESSDBG") != nullptr;

// Face-merge batch size for the huge single-root face-parallel path (tessellate_one_root). The
// per-face local meshes are held resident until merged; on a 61k-face monster solid the full
// locals[] is a SECOND complete copy of the solid's soup (~0.5 GB with normals), inflating that
// root's transient peak. Processing faces in BATCHES caps the resident local meshes to one batch,
// keeping the merge byte-identical (same face-order append). Defensive memory hygiene — measurably
// trims the face-parallel merge transient, though it is not the dominant term in the 469826 case.
// ADA_TESS_FACE_MERGE_BATCH: batch size (default 4096); 0 disables batching (old all-at-once path).
inline size_t tess_face_merge_batch() {
    if (const char *e = std::getenv("ADA_TESS_FACE_MERGE_BATCH")) {
        char *end = nullptr;
        long v = std::strtol(e, &end, 10);
        if (end != e && v >= 0)
            return (size_t) v;
    }
    return 4096;
}

// ---- diagnostics (env-gated) ----------------------------------------------------------------
[[maybe_unused]] const char *surf_kind(const Surface &s) {
    if (dynamic_cast<const PlaneSurface *>(&s))
        return "plane";
    if (dynamic_cast<const CylinderSurface *>(&s))
        return "cyl";
    if (dynamic_cast<const ConeSurface *>(&s))
        return "cone";
    if (dynamic_cast<const SphereSurface *>(&s))
        return "sphere";
    if (dynamic_cast<const TorusSurface *>(&s))
        return "torus";
    if (dynamic_cast<const BSplineSurface *>(&s))
        return "bspline";
    if (dynamic_cast<const LinearExtrusionSurface *>(&s))
        return "extrusion";
    if (dynamic_cast<const RevolutionSurface *>(&s))
        return "revolution";
    return "?";
}

// ---- UV-inversion residual diagnostic (ADA_TESS_DIAG=<path.csv>) -----------------------------
//
// Measures, per face, how far each boundary point lands from the surface after the UV round-trip:
// r = |surf.point(surf.uv(p)) - p|. This decides whether the shared-edge crack is a real
// surface-vs-edge gap or an inversion bug, and the two have opposite fixes.
//
// The quadric arm is the CONTROL: quadrics invert in closed form, so a large quadric residual
// cannot be numerical — it would mean the boundary genuinely doesn't lie on its surface. B-spline /
// revolution / extrusion invert iteratively and are the suspects (bspline_invert always returns
// true; newton_uv validates with an ABSOLUTE 1e-6).
//
// Off by default and observation-only — it never changes a tessellation decision.
struct TessDiag {
    std::FILE *fh = nullptr;
    std::mutex mu;
    ~TessDiag() {
        if (fh)
            std::fclose(fh);
    }
};

TessDiag &tess_diag() {
    static TessDiag d;
    return d;
}

const char *tess_diag_path() {
    static const char *p = std::getenv("ADA_TESS_DIAG");
    return (p && *p) ? p : nullptr;
}

const bool DIAG = tess_diag_path() != nullptr;

// Per-face accumulator: one row per face, so a 300k-face model stays a manageable CSV.
struct FaceDiag {
    std::vector<double> r;  // residual per boundary point (model units)
    size_t n_uv_fail = 0;   // surf.uv() returned false
    size_t n_collapsed = 0; // ...and the handler collapsed point i onto i-1 (Defect 2 fired)
    const char *path = "?"; // which branch of face_to_mesh took the face
};

// thread_local: DIAG forces threads=1 (see tessellate_one_root), but a stray parallel caller must
// not interleave two faces' points into one row.
FaceDiag &face_diag() {
    static thread_local FaceDiag d;
    return d;
}

void diag_set_path(const char *p) {
    if (DIAG)
        face_diag().path = p;
}

double pct(std::vector<double> &v, double q) {
    if (v.empty())
        return 0.0;
    size_t k = (size_t) (q * (double) (v.size() - 1));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

void diag_flush_face(const Surface &surf, int64_t face_id, uint32_t face_seq, double model_scale) {
    if (!DIAG)
        return;
    FaceDiag &d = face_diag();
    TessDiag &t = tess_diag();
    std::lock_guard<std::mutex> lk(t.mu);
    if (!t.fh) {
        t.fh = std::fopen(tess_diag_path(), "w");
        if (!t.fh)
            return;
        std::fprintf(t.fh, "face_seq,face_id,surf_kind,is_quadric,path_taken,n_bpts,n_uv_fail,n_collapsed,"
                           "r_p50,r_p95,r_max,r_rel_p50,r_rel_p95,r_rel_max,approx_size,model_scale\n");
    }
    double sz = std::max(surf.approx_size(), 1.0);
    double p50 = pct(d.r, 0.50), p95 = pct(d.r, 0.95);
    double mx = d.r.empty() ? 0.0 : *std::max_element(d.r.begin(), d.r.end());
    std::fprintf(t.fh, "%u,%lld,%s,%d,%s,%zu,%zu,%zu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", face_seq,
                 (long long) face_id, surf_kind(surf), (int) surf.is_quadric(), d.path, d.r.size(), d.n_uv_fail,
                 d.n_collapsed, p50, p95, mx, p50 / sz, p95 / sz, mx / sz, sz, model_scale);
    d = FaceDiag{};
}

// ---- refinement profile (ADA_TESS_EDGEPROF=1) -----------------------------------------------
// Reports where tessellation time actually goes. Two findings this settled, kept here because the
// numbers keep being needed:
//   * build_loops3d (edge discretization) is 0.4% of tessellate_doc, and each edge is discretized
//     exactly 2.00x (once per adjacent face). So sharing/caching edge discretization funds nothing
//     -- which is why the planned per-root edge cache was cancelled.
//   * refine passes x tris-scanned is what the boundary freeze inflates (1431 passes / 949k tris
//     scanned vs 812 / 578k unfrozen, for 3.7% more marks). Time tracks tris-scanned almost
//     exactly, which is how WatertightOpts::converged_frac was found.
const bool EDGEPROF = std::getenv("ADA_TESS_EDGEPROF") != nullptr;
std::atomic<uint64_t> g_loops3d_ns{0};
std::atomic<uint64_t> g_tess_ns{0};
std::atomic<uint64_t> g_passes{0};
std::atomic<uint64_t> g_marks{0};
std::atomic<uint64_t> g_scanned{0};
std::atomic<uint64_t> g_conform_ok{0};    // conforming grid applied
std::atomic<uint64_t> g_conform_bail{0};  // ...bailed on the blow-up guard
std::atomic<uint64_t> g_conform_lines{0}; // total conformed lines emitted
std::atomic<uint64_t> g_conform_want{0};  // total lines the loop WANTED

// distinctive surface params, so a face can be matched to the reference output by geometry
void log_surf_params(const Surface &s) {
    if (const auto *c = dynamic_cast<const CylinderSurface *>(&s))
        std::fprintf(stderr, " r=%.3f o=(%.1f,%.1f,%.1f)", c->r, c->f.o.x, c->f.o.y, c->f.o.z);
    else if (const auto *c = dynamic_cast<const ConeSurface *>(&s))
        std::fprintf(stderr, " r0=%.3f semi=%.4f", c->r0, c->semi_angle);
    else if (const auto *c = dynamic_cast<const SphereSurface *>(&s))
        std::fprintf(stderr, " r=%.3f", c->r);
    else if (const auto *c = dynamic_cast<const TorusSurface *>(&s))
        std::fprintf(stderr, " R=%.3f r=%.3f", c->R, c->r);
    else if (const auto *b = dynamic_cast<const BSplineSurface *>(&s))
        std::fprintf(stderr, " nu=%d nv=%d deg=(%d,%d) rational=%d size=%.2f", b->nu, b->nv, b->u_degree, b->v_degree,
                     (int) !b->weights.empty(), b->size());
}

// ---- mesh accumulator with checkpoint/rollback (TriMesh) ----------------------------
struct Mesh {
    TessMesh &m;
    explicit Mesh(TessMesh &mm) : m(mm) {}
    uint32_t base() const {
        return (uint32_t) (m.positions.size() / 3);
    }
    void push_vertex(const Vec3 &p, const Vec3 &n) {
        m.positions.push_back((float) p.x);
        m.positions.push_back((float) p.y);
        m.positions.push_back((float) p.z);
        m.normals.push_back((float) n.x);
        m.normals.push_back((float) n.y);
        m.normals.push_back((float) n.z);
    }
    void push_index(uint32_t i) {
        m.indices.push_back(i);
    }
    std::array<size_t, 2> checkpoint() const {
        return {m.positions.size(), m.indices.size()};
    }
    void rollback(const std::array<size_t, 2> &cp) {
        m.positions.resize(cp[0]);
        m.normals.resize(cp[0]);
        m.indices.resize(cp[1]);
    }
};

// one boundary loop in 3D, FACE_BOUND orientation already applied
struct Loop3 {
    std::vector<Vec3> pts;
};
// A NaN Vec3 marks "no pin" — an interior/synthetic vertex whose 3D position is the surface point,
// not a shared boundary-edge point.
inline Vec3 nan_vec() {
    double q = std::numeric_limits<double>::quiet_NaN();
    return {q, q, q};
}
inline bool is_pinned(const Vec3 &p) {
    return p.x == p.x; // finite (not NaN)
}

// per-loop UV polyline plus winding bookkeeping
struct LoopUv {
    std::vector<Uv> uv;
    std::vector<Vec3> pts3;      // 3D point per uv vertex — the SHARED EDGE discretization point for a
                                 // boundary vertex (NaN => synthetic/no pin). Lets the emitter place
                                 // boundary vertices on the shared edge (watertight) instead of on this
                                 // face's own surface re-projection, which diverges from the neighbour's.
    int w = 0;                   // net winding around the u period
    bool interior_above = false; // interior is on +v side of this winding loop
};

// ---- small geometry helpers ------------------------------------------------------------------
double poly_area(const std::vector<Uv> &c) {
    double a = 0;
    size_t n = c.size();
    for (size_t i = 0; i < n; ++i) {
        const Uv &p = c[i];
        const Uv &q = c[(i + 1) % n];
        a += p[0] * q[1] - q[0] * p[1];
    }
    return a * 0.5;
}

bool point_in_poly(const Uv &pt, const std::vector<Uv> &poly) {
    bool inside = false;
    size_t n = poly.size();
    size_t j = n - 1;
    for (size_t i = 0; i < n; ++i) {
        if (((poly[i][1] > pt[1]) != (poly[j][1] > pt[1])) &&
            (pt[0] <
             (poly[j][0] - poly[i][0]) * (pt[1] - poly[i][1]) / (poly[j][1] - poly[i][1] + 1e-300) + poly[i][0]))
            inside = !inside;
        j = i;
    }
    return inside;
}

bool segments_cross(const Uv &p1, const Uv &p2, const Uv &p3, const Uv &p4) {
    auto o = [](const Uv &a, const Uv &b, const Uv &c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    double d1 = o(p3, p4, p1), d2 = o(p3, p4, p2);
    double d3 = o(p1, p2, p3), d4 = o(p1, p2, p4);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

bool poly_self_intersects(const std::vector<Uv> &c) {
    size_t n = c.size();
    if (n < 4)
        return false;
    for (size_t i = 0; i < n; ++i) {
        const Uv &a1 = c[i];
        const Uv &a2 = c[(i + 1) % n];
        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1)
                continue;
            if (segments_cross(a1, a2, c[j], c[(j + 1) % n]))
                return true;
        }
    }
    return false;
}

// Metric scale mapping UV toward the 3D arc-length metric (metric_scale).
std::pair<double, double> metric_scale(const Surface &s, double umin, double umax, double vmin, double vmax) {
    double du = std::max(umax - umin, 1e-12);
    double dv = std::max(vmax - vmin, 1e-12);
    double uc = (umin + umax) * 0.5, vc = (vmin + vmax) * 0.5;
    double h = 1e-4;
    double lu = (s.point(uc + h * du, vc) - s.point(uc - h * du, vc)).norm() / (2.0 * h * du);
    double lv = (s.point(uc, vc + h * dv) - s.point(uc, vc - h * dv)).norm() / (2.0 * h * dv);
    double size = std::max(std::max(lu * du, lv * dv), 1e-12);
    return {std::max(lu / size, 1e-9 / du), std::max(lv / size, 1e-9 / dv)};
}

// ---- tess2 plumbing --------------------------------------------------------------------------
// Build tess2's flat (scaled) contour, dropping coincident points + closing dup (
// sanitize_contour). Returns empty for <3 distinct points.
// Sanitize the loop into a flat scaled contour AND (when p3 is given) the parallel 3D pin per
// surviving vertex — dropping the SAME coincident/closing-duplicate points from both, so pins stay
// index-aligned with the contour.
//
// The prototype forked this from sanitize_contour; that fork is inverted here — sanitize_contour is
// a one-line delegation with no pins, so the DEFAULT track is identical BY CONSTRUCTION rather than
// by inspecting two copies that must be kept in step.
std::vector<TESSreal> sanitize_contour_pinned(const std::vector<Uv> &l, const std::vector<Vec3> *p3, double su,
                                              double sv, std::vector<Vec3> *pin3_out) {
    if (l.size() < 3)
        return {};
    double w = 0, h = 0;
    for (const Uv &p : l) {
        w = std::max(w, std::abs(p[0] * su));
        h = std::max(h, std::abs(p[1] * sv));
    }
    double eps = 1e-9 * std::max(std::max(w, h), 1.0);
    std::vector<TESSreal> flat;
    std::vector<Vec3> pins;
    flat.reserve(l.size() * 2);
    for (size_t i = 0; i < l.size(); ++i) {
        double x = l[i][0] * su, y = l[i][1] * sv;
        if (flat.size() >= 2) {
            double lx = flat[flat.size() - 2], ly = flat[flat.size() - 1];
            if (std::abs(x - lx) <= eps && std::abs(y - ly) <= eps)
                continue;
        }
        flat.push_back((TESSreal) x);
        flat.push_back((TESSreal) y);
        if (pin3_out)
            pins.push_back(p3 && i < p3->size() ? (*p3)[i] : nan_vec());
    }
    if (flat.size() >= 4) {
        size_t n = flat.size();
        if (std::abs(flat[0] - flat[n - 2]) <= eps && std::abs(flat[1] - flat[n - 1]) <= eps) {
            flat.resize(n - 2);
            if (pin3_out && !pins.empty())
                pins.pop_back();
        }
    }
    if (flat.size() < 6)
        return {};
    if (pin3_out)
        *pin3_out = std::move(pins);
    return flat;
}

std::vector<TESSreal> sanitize_contour(const std::vector<Uv> &l, double su, double sv) {
    return sanitize_contour_pinned(l, nullptr, su, sv, nullptr);
}

struct Tess2Out {
    std::vector<Uv> verts;
    std::vector<Vec3> pin; // parallel to verts: shared-edge 3D point for a boundary vertex, else NaN
    std::vector<Tri> tris;
    bool ok = false;
};

// loops_p3 (parallel to loops_uv, may be null): per-vertex 3D pins so boundary vertices can be
// emitted on the shared edge. When present, out.pin is filled via libtess2's vertex-index map —
// original boundary vertices carry their pin; Steiner/new vertices get NaN. Null (the default track)
// => not one extra byte or branch in the hot loop.
Tess2Out run_tess2(const std::vector<std::vector<Uv>> &loops_uv_in, double su, double sv,
                   const std::vector<std::vector<Vec3>> *loops_p3 = nullptr) {
    Tess2Out out;
    // Re-center the UV loops to their min corner before handing them to libtess2. TESSreal is float,
    // so a face whose parametric coordinates carry a large offset — e.g. a cylinder whose STEP axis
    // placement sits kilometres from the face, giving an axial v ~6e6 — loses its sub-mm real extent
    // to float precision: every vertex snaps to the same value, the loop collapses to a line, tess2
    // returns no triangles and the face silently drops (violating "no geom left behind"). Translating
    // to the origin keeps the true extent representable; the offset is added back to the output verts,
    // so the transform is invisible to callers and composes with any su/sv scaling. (measured: cylinder
    // faces #39996/#40030 on KR_6, v~6e6, dropped -> recovered.)
    double u0 = INF, v0 = INF;
    for (const auto &lp : loops_uv_in)
        for (const Uv &p : lp) {
            u0 = std::min(u0, p[0]);
            v0 = std::min(v0, p[1]);
        }
    if (!std::isfinite(u0))
        u0 = 0.0;
    if (!std::isfinite(v0))
        v0 = 0.0;
    std::vector<std::vector<Uv>> shifted;
    shifted.reserve(loops_uv_in.size());
    for (const auto &lp : loops_uv_in) {
        std::vector<Uv> s;
        s.reserve(lp.size());
        for (const Uv &p : lp)
            s.push_back({p[0] - u0, p[1] - v0});
        shifted.push_back(std::move(s));
    }
    const std::vector<std::vector<Uv>> &loops_uv = shifted;
    TESStesselator *t = tessNewTess(nullptr);
    if (!t)
        return out;
    std::vector<std::vector<TESSreal>> store;
    std::vector<Vec3> flat_p3; // pin per concatenated input vertex, indexable by tessGetVertexIndices
    store.reserve(loops_uv.size());
    for (size_t li = 0; li < loops_uv.size(); ++li) {
        std::vector<Vec3> pins;
        const std::vector<Vec3> *p3 = (loops_p3 && li < loops_p3->size()) ? &(*loops_p3)[li] : nullptr;
        std::vector<TESSreal> flat = sanitize_contour_pinned(loops_uv[li], p3, su, sv, loops_p3 ? &pins : nullptr);
        if (flat.empty())
            continue;
        if (loops_p3)
            flat_p3.insert(flat_p3.end(), pins.begin(), pins.end());
        store.push_back(std::move(flat));
        tessAddContour(t, 2, store.back().data(), sizeof(TESSreal) * 2, (int) store.back().size() / 2);
    }
    if (store.empty() || !tessTesselate(t, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, nullptr)) {
        tessDeleteTess(t);
        return out;
    }
    const TESSreal *tv = tessGetVertices(t);
    const int nv = tessGetVertexCount(t);
    const TESSindex *vi = loops_p3 ? tessGetVertexIndices(t) : nullptr; // output vert -> input index
    const TESSindex *te = tessGetElements(t);
    const int ne = tessGetElementCount(t);
    if (nv <= 0 || ne <= 0) {
        tessDeleteTess(t);
        return out;
    }
    out.verts.reserve(nv);
    if (loops_p3)
        out.pin.reserve(nv);
    for (int i = 0; i < nv; ++i) {
        out.verts.push_back({tv[i * 2] / su + u0, tv[i * 2 + 1] / sv + v0}); // undo the re-centering
        if (loops_p3) {
            TESSindex oi = vi ? vi[i] : TESS_UNDEF;
            out.pin.push_back((oi != TESS_UNDEF && (size_t) oi < flat_p3.size()) ? flat_p3[oi] : nan_vec());
        }
    }
    for (int e = 0; e < ne; ++e) {
        const TESSindex *p = &te[e * 3];
        if (p[0] == TESS_UNDEF || p[1] == TESS_UNDEF || p[2] == TESS_UNDEF)
            continue;
        out.tris.push_back({(uint32_t) p[0], (uint32_t) p[1], (uint32_t) p[2]});
    }
    tessDeleteTess(t); // free the tessellator on the success path too (verts/tris are already copied
                       // out above) — otherwise every successful face leaks its libtess2 buckets,
                       // which accumulates to GBs over a whole model in a single long-lived process.
    // Rust returns Some(verts,tris) whenever tessellate succeeded + ne>0, even if every triangle
    // was UNDEF-filtered (caller then emits nothing rather than retrying shrunk holes). Match it.
    out.ok = true;
    return out;
}

// Retry tess2 with interior holes nudged toward their centroids (tess2_with_shrunk_holes).
Tess2Out tess2_with_shrunk_holes(const std::vector<std::vector<Uv>> &loops_uv, double su, double sv) {
    if (loops_uv.size() < 2)
        return {};
    size_t outer = 0;
    double best = -1;
    for (size_t i = 0; i < loops_uv.size(); ++i) {
        double a = std::abs(poly_area(loops_uv[i]));
        if (a >= best) { // >= : last maximal wins, matching Rust Iterator::max_by
            best = a;
            outer = i;
        }
    }
    for (double frac : {0.01, 0.04, 0.1}) {
        std::vector<std::vector<Uv>> shrunk;
        shrunk.reserve(loops_uv.size());
        for (size_t i = 0; i < loops_uv.size(); ++i) {
            const auto &c = loops_uv[i];
            if (i == outer || c.empty()) {
                shrunk.push_back(c);
                continue;
            }
            double n = (double) c.size();
            double cx = 0, cy = 0;
            for (const Uv &p : c) {
                cx += p[0];
                cy += p[1];
            }
            cx /= n;
            cy /= n;
            std::vector<Uv> s;
            s.reserve(c.size());
            for (const Uv &p : c)
                s.push_back({p[0] + (cx - p[0]) * frac, p[1] + (cy - p[1]) * frac});
            shrunk.push_back(std::move(s));
        }
        Tess2Out r = run_tess2(shrunk, su, sv);
        if (r.ok)
            return r;
    }
    return {};
}

// ---- refinement (refine_uv + delaunay_flip) -----------------------------------------
void delaunay_flip(const std::vector<Uv> &verts, std::vector<Tri> &tris, double su, double sv) {
    auto p = [&](uint32_t i) -> Uv { return {verts[i][0] * su, verts[i][1] * sv}; };
    auto len = [](const Uv &x, const Uv &y) {
        return std::sqrt((x[0] - y[0]) * (x[0] - y[0]) + (x[1] - y[1]) * (x[1] - y[1]));
    };
    auto min_angle = [&](const Uv &a, const Uv &b, const Uv &c) -> double {
        double ab = len(a, b), bc = len(b, c), ca = len(c, a);
        if (ab < 1e-300 || bc < 1e-300 || ca < 1e-300)
            return 0.0;
        auto ang = [](double opp, double s1, double s2) {
            return std::acos(clampd((s1 * s1 + s2 * s2 - opp * opp) / (2.0 * s1 * s2), -1.0, 1.0));
        };
        return std::min(std::min(ang(bc, ab, ca), ang(ca, ab, bc)), ang(ab, bc, ca));
    };
    auto area2 = [](const Uv &a, const Uv &b, const Uv &c) {
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    for (int sweep = 0; sweep < 6; ++sweep) {
        std::map<std::pair<uint32_t, uint32_t>, std::vector<std::pair<size_t, uint32_t>>> edge_map;
        for (size_t ti = 0; ti < tris.size(); ++ti) {
            const Tri &t = tris[ti];
            for (int k = 0; k < 3; ++k) {
                uint32_t i = t[k], j = t[(k + 1) % 3], o = t[(k + 2) % 3];
                edge_map[{std::min(i, j), std::max(i, j)}].push_back({ti, o});
            }
        }
        std::vector<char> dirty(tris.size(), 0);
        int flips = 0;
        for (auto &kv : edge_map) { // std::map => sorted edge order => reproducible
            uint32_t i = kv.first.first, j = kv.first.second;
            auto &adj = kv.second;
            if (adj.size() != 2)
                continue;
            auto [t1, o1] = adj[0];
            auto [t2, o2] = adj[1];
            if (dirty[t1] || dirty[t2] || o1 == o2)
                continue;
            Uv pi = p(i), pj = p(j), po1 = p(o1), po2 = p(o2);
            if (area2(po1, po2, pj) <= 1e-18 || area2(po2, po1, pi) <= 1e-18)
                continue;
            double before = std::min(min_angle(pi, pj, po1), min_angle(pj, pi, po2));
            double after = std::min(min_angle(po1, po2, pj), min_angle(po2, po1, pi));
            if (after > before + 1e-12) {
                tris[t1] = {o1, o2, j};
                tris[t2] = {o2, o1, i};
                dirty[t1] = dirty[t2] = 1;
                ++flips;
            }
        }
        if (flips == 0)
            break;
    }
}

void refine_uv(const Surface &s, std::vector<Uv> &verts, std::vector<Tri> &tris, double max_du, double max_dv,
               double defl, bool have_metric, double su, double sv, double max_angle, bool freeze = false,
               double converged_frac = 0.0) {
    std::vector<Vec3> pts3(verts.size());
    for (size_t i = 0; i < verts.size(); ++i)
        pts3[i] = s.point(verts[i][0], verts[i][1]);
    bool check_dev = dynamic_cast<const PlaneSurface *>(&s) == nullptr;

    // deflection <= 0 means "auto": derive a scale-relative chord tolerance from the surface's
    // characteristic size instead of refining toward zero deviation (dev > 0 splits every curved
    // edge up to the 300k budget — catastrophic on a large B-spline patch, e.g. a coarse 6x6 cubic
    // exploding to ~1M tris). Mirrors the auto-guard the primitive builders already use
    // (tessellate_sphere/cylinder: `defl = tp.deflection > 0 ? tp.deflection : r * 0.01`).
    if (defl <= 0.0)
        defl = std::max(s.approx_size() * 0.005, 1e-4);

    double approx_area = 0;
    for (const Tri &t : tris) {
        Vec3 a = pts3[t[0]], b = pts3[t[1]], c = pts3[t[2]];
        approx_area += (b - a).cross(c - a).norm() * 0.5;
    }
    size_t budget;
    if (defl > 0.0) {
        double raw = 4.0 * approx_area / (defl * defl);
        size_t b = raw > 0 ? (size_t) raw : 0;
        b = std::max(b, tris.size() * 4);
        budget = std::min(std::max(b, (size_t) 2048), (size_t) 300000);
    } else {
        budget = 300000;
    }
    if (TESSDBG)
        std::fprintf(stderr, "TESSDBG     refine_uv approx_area=%.1f budget=%zu max_du=%.5f max_dv=%.5f check_dev=%d\n",
                     approx_area, budget, max_du, max_dv, (int) check_dev);

    auto key = [](uint32_t i, uint32_t j) { return std::make_pair(std::min(i, j), std::max(i, j)); };
    // Undirected edge as one uint64 — hashable without allocating a node per edge.
    auto pack = [](uint32_t i, uint32_t j) { return ((uint64_t) std::min(i, j) << 32) | (uint64_t) std::max(i, j); };

    // Angular refinement: split an edge whose surface normal turns more than the (adaptive) max_angle
    // across it. This is SCALE-INVARIANT, so it captures curved-but-shallow features (a bevel cut into
    // a large solid, whose chord sag is far below `defl` — the chord-deviation test alone misses it
    // entirely on a small model). Uses the SAME adaptive relaxation as the boundary discretization so a
    // tiny curved feature (a bolt on a big assembly) doesn't over-refine — only surfaces at/above ~1%
    // of the model diagonal get the fine angle. Floored by an absolute min 3D edge length so a sharp
    // fillet can't recurse without bound; the 12-pass + budget caps below also bound it.
    const double eff_angle = adaptive_max_angle(s.approx_size(), max_angle);
    const double cos_max_angle = eff_angle > 0.0 ? std::cos(eff_angle) : -2.0;
    const double ang_min_len2 = std::pow(std::max(s.approx_size() * 1e-3, 1e-6), 2.0);

    // Boundary-edge set for the freeze, computed ONCE. An incidence-1 edge is a trim-loop segment;
    // because the freeze never splits one, the set is INVARIANT across passes (refinement only
    // appends interior vertices, so existing index pairs keep their meaning), so there is no reason
    // to rebuild it per pass. (Measured: hoisting this out of the loop is NOT where the freeze's
    // cost was — see converged_frac below for what actually cost +65%.)
    std::unordered_set<uint64_t> boundary;
    if (freeze) {
        std::unordered_map<uint64_t, int> inc0;
        inc0.reserve(tris.size() * 3);
        for (const Tri &t : tris) {
            inc0[pack(t[0], t[1])]++;
            inc0[pack(t[1], t[2])]++;
            inc0[pack(t[2], t[0])]++;
        }
        for (const auto &kv : inc0)
            if (kv.second == 1)
                boundary.insert(kv.first);
    }
    auto is_interior = [&](uint32_t i, uint32_t j) { return !freeze || !boundary.count(pack(i, j)); };

    for (int pass = 0; pass < 12; ++pass) {
        std::set<std::pair<uint32_t, uint32_t>> marked;
        // 0 = no, 1 = parametric, 2 = deviation, 3 = angular
        auto edge_needs_split = [&](uint32_t i, uint32_t j) -> int {
            const Uv &a = verts[i];
            const Uv &b = verts[j];
            if (std::abs(b[0] - a[0]) > max_du || std::abs(b[1] - a[1]) > max_dv)
                return 1;
            if (!check_dev)
                return 0;
            Vec3 pa = pts3[i];
            Vec3 chord = pts3[j] - pa;
            double l2 = chord.dot(chord);
            // Angular split runs BEFORE the chord sub-resolution floor: a small edge across a rounded
            // feature has a large normal turn but a tiny chord sag, so the floor below would wrongly skip it.
            if (max_angle > 0.0 && l2 > ang_min_len2) {
                Vec3 na = s.normal(a[0], a[1]);
                Vec3 nb = s.normal(b[0], b[1]);
                if (na.dot(nb) < cos_max_angle)
                    return 3;
            }
            if (l2 < (4.0 * defl) * (4.0 * defl))
                return 0; // sub-resolution floor
            Vec3 m = s.point((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5);
            Vec3 d = m - pa;
            double dev;
            if (l2 < 1e-300) {
                dev = d.norm();
            } else {
                double tt = clampd(d.dot(chord) / l2, 0.0, 1.0);
                dev = (d - chord * tt).norm();
            }
            return dev > defl ? 2 : 0;
        };

        for (const Tri &t : tris) {
            const std::pair<uint32_t, uint32_t> es[3] = {{t[0], t[1]}, {t[1], t[2]}, {t[2], t[0]}};
            for (auto &e : es) {
                auto k = key(e.first, e.second);
                if (is_interior(e.first, e.second) && !marked.count(k) && edge_needs_split(e.first, e.second) != 0)
                    marked.insert(k);
            }
            // fold detection: split the longest edge of a folded triangle (first 3 passes)
            if (check_dev && pass < 3) {
                Vec3 a = pts3[t[0]], b = pts3[t[1]], c = pts3[t[2]];
                Vec3 gn = (b - a).cross(c - a);
                double lmax = std::max(std::max((b - a).norm(), (c - b).norm()), (a - c).norm());
                if (gn.norm() > 1e-4 * lmax * lmax) {
                    const Uv &ua = verts[t[0]];
                    const Uv &ub = verts[t[1]];
                    const Uv &uc = verts[t[2]];
                    double cu = (ua[0] + ub[0] + uc[0]) / 3.0, cv = (ua[1] + ub[1] + uc[1]) / 3.0;
                    Vec3 sn = s.normal(cu, cv);
                    if (gn.normalized().dot(sn) < 0.5) {
                        double l01 = (b - a).norm(), l12 = (c - b).norm(), l20 = (a - c).norm();
                        std::pair<uint32_t, uint32_t> e;
                        if (l01 >= l12 && l01 >= l20)
                            e = {t[0], t[1]};
                        else if (l12 >= l20)
                            e = {t[1], t[2]};
                        else
                            e = {t[2], t[0]};
                        const Uv &ea = verts[e.first];
                        const Uv &eb = verts[e.second];
                        if (is_interior(e.first, e.second) &&
                            (std::abs(eb[0] - ea[0]) > max_du / 8.0 || std::abs(eb[1] - ea[1]) > max_dv / 8.0))
                            marked.insert(key(e.first, e.second));
                    }
                }
            }
        }

        // NOTE: no early break on empty `marked` — Rust still runs phase-2 (a no-op copy) and the
        // terminal delaunay_flip, then breaks on !split_any below. Keeps the final flip faithful.
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> mid;
        auto midpoint = [&](uint32_t i, uint32_t j) -> uint32_t {
            auto k = key(i, j);
            auto it = mid.find(k);
            if (it != mid.end())
                return it->second;
            const Uv &a = verts[i];
            const Uv &b = verts[j];
            Uv m = {(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5};
            verts.push_back(m);
            pts3.push_back(s.point(m[0], m[1]));
            uint32_t idx = (uint32_t) (verts.size() - 1);
            mid[k] = idx;
            return idx;
        };

        std::vector<Tri> out;
        out.reserve(tris.size());
        bool split_any = false;
        for (const Tri &t : tris) {
            bool s0 = marked.count(key(t[0], t[1])) > 0;
            bool s1 = marked.count(key(t[1], t[2])) > 0;
            bool s2 = marked.count(key(t[2], t[0])) > 0;
            if (!s0 && !s1 && !s2) {
                out.push_back(t);
                continue;
            }
            split_any = true;
            bool h0 = s0, h1 = s1, h2 = s2;
            uint32_t a = s0 ? midpoint(t[0], t[1]) : 0;
            uint32_t b = s1 ? midpoint(t[1], t[2]) : 0;
            uint32_t c = s2 ? midpoint(t[2], t[0]) : 0;
            if (h0 && h1 && h2) {
                out.push_back({t[0], a, c});
                out.push_back({a, t[1], b});
                out.push_back({c, b, t[2]});
                out.push_back({a, b, c});
            } else if (h0 && h1 && !h2) {
                out.push_back({t[0], a, t[2]});
                out.push_back({a, b, t[2]});
                out.push_back({a, t[1], b});
            } else if (!h0 && h1 && h2) {
                out.push_back({t[0], t[1], b});
                out.push_back({t[0], b, c});
                out.push_back({c, b, t[2]});
            } else if (h0 && !h1 && h2) {
                out.push_back({t[0], a, c});
                out.push_back({a, t[1], c});
                out.push_back({c, t[1], t[2]});
            } else if (h0 && !h1 && !h2) {
                out.push_back({t[0], a, t[2]});
                out.push_back({a, t[1], t[2]});
            } else if (!h0 && h1 && !h2) {
                out.push_back({t[0], t[1], b});
                out.push_back({t[0], b, t[2]});
            } else { // !h0 && !h1 && h2
                out.push_back({t[0], t[1], c});
                out.push_back({c, t[1], t[2]});
            }
        }
        tris.swap(out);
        if (have_metric)
            delaunay_flip(verts, tris, su, sv);
        if (EDGEPROF) {
            g_passes.fetch_add(1, std::memory_order_relaxed);
            g_marks.fetch_add(marked.size(), std::memory_order_relaxed);
            g_scanned.fetch_add(tris.size(), std::memory_order_relaxed);
        }
        // Converged-enough exit (watertight track only — it changes which passes run, so it must
        // never touch the default track's output). The freeze slows triangle growth, so the
        // `tris.size() > budget` exit fires much later and the loop keeps rescanning the whole mesh
        // to chase a handful of stragglers: measured 1431 passes / 949k triangles scanned vs 812 /
        // 578k unfrozen, for only 3.7% more marks. Stop once a pass marks a negligible fraction.
        if (freeze && converged_frac > 0.0 && marked.size() * converged_frac < tris.size())
            break;
        if (!split_any || tris.size() > budget)
            break;
    }
}

// Shared tail of every UV tessellation (the GRID paths call this too): drop zero-area tris,
// normalize winding, refine, map to 3D.
//
// `pin` (parallel to the ORIGINAL verts; empty = none) places a boundary vertex at its shared-edge
// point instead of this face's own surface re-projection, so it coincides with the neighbouring
// face's vertex and the per-solid weld stitches the seam watertight. Refinement only APPENDS
// interior vertices, so pins stay index-aligned with the originals. Only POSITION is pinned — the
// normal stays the per-face surface normal, preserving the crease along the seam.
//
// Empty pin (the default track) => `is_pinned` is never consulted and the emit loop is unchanged.
void refine_and_emit(const Surface &s, std::vector<Uv> verts, std::vector<Tri> tris, double su, double sv,
                     const TessParams &tp, bool same_sense, Mesh &mesh, const std::vector<Vec3> &pin = {},
                     bool freeze = false, double converged_frac = 0.0) {
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const Uv &p : verts) {
        umin = std::min(umin, p[0]);
        umax = std::max(umax, p[0]);
        vmin = std::min(vmin, p[1]);
        vmax = std::max(vmax, p[1]);
    }
    double eps_a = 1e-10 * std::max((umax - umin) * (vmax - vmin), 1e-300);
    auto uv_area2 = [&](const Tri &t) {
        const Uv &a = verts[t[0]];
        const Uv &b = verts[t[1]];
        const Uv &c = verts[t[2]];
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    };
    std::vector<Tri> kept;
    kept.reserve(tris.size());
    for (Tri &t : tris) {
        double a2 = uv_area2(t);
        if (std::abs(a2) <= eps_a)
            continue;
        if (a2 < 0.0)
            std::swap(t[1], t[2]);
        kept.push_back(t);
    }
    tris.swap(kept);

    double max_du = s.u_step(tp.deflection, tp.max_angle);
    double max_dv = s.v_step(tp.deflection, tp.max_angle);
    bool needs_dev = dynamic_cast<const PlaneSurface *>(&s) == nullptr;
    size_t pre_refine = tris.size();
    if (std::isfinite(max_du) || std::isfinite(max_dv) || needs_dev) {
        refine_uv(s, verts, tris, max_du, max_dv, tp.deflection, needs_dev, su, sv, tp.max_angle, freeze,
                  converged_frac);
    }
    if (TESSDBG) {
        std::fprintf(stderr, "TESSDBG   refine_and_emit surf=%s", surf_kind(s));
        log_surf_params(s);
        std::fprintf(stderr,
                     " | defl=%.4f max_angle=%.4f u_step=%.5f v_step=%.5f metric_su=%.5f metric_sv=%.5f"
                     " | pre_refine_tris=%zu post_refine_tris=%zu\n",
                     tp.deflection, tp.max_angle, max_du, max_dv, su, sv, pre_refine, tris.size());
    }

    bool flip = !same_sense;
    uint32_t base = mesh.base();
    for (size_t i = 0; i < verts.size(); ++i) {
        const Uv &p = verts[i];
        Vec3 pos = (i < pin.size() && is_pinned(pin[i])) ? pin[i] : s.point(p[0], p[1]);
        Vec3 n = s.normal(p[0], p[1]);
        if (flip)
            n = n * -1.0;
        mesh.push_vertex(pos, n);
    }
    for (const Tri &t : tris) {
        if (flip) {
            mesh.push_index(base + t[0]);
            mesh.push_index(base + t[2]);
            mesh.push_index(base + t[1]);
        } else {
            mesh.push_index(base + t[0]);
            mesh.push_index(base + t[1]);
            mesh.push_index(base + t[2]);
        }
    }
}

// loops_p3 (parallel to loops_uv, may be null): shared-edge pins for the boundary vertices.
// Forward decls: the CDT track sits above these in the file but reuses both — the shared curvature
// grid (as Steiner points) and libtess2 as its per-face fallback.
void uv_grid_res(const Surface &s, int &nu, int &nv);
bool emit_uv_region(const Surface &s, const std::vector<std::vector<Uv>> &loops_uv, const TessParams &tp,
                    bool same_sense, Mesh &mesh, const std::vector<std::vector<Vec3>> *loops_p3,
                    const bool *freeze_override, double conv_override);

// ---- CDT track: one boundary-first path (trim loops as constraints, grid as Steiner) ----------
//
// The reason this exists: libtess2 takes no interior points, which forces a separate UV-grid fast
// path for near-full patches — and that grid tessellates the UV bbox, never sees the trim loop, and
// so has nothing to pin. It is the only remaining crack source. OCC and truck have no grid path at
// all: they classifier-filter grid points and insert them as Steiner points into a CDT whose
// boundary is already constraints. This is that.
//
// detria REFUSES to split a constraint edge (it errors instead), which is exactly the invariant
// pinning needs — a boundary vertex is always a loop vertex, so every boundary vertex can pin.
// artem-ogre/CDT was rejected precisely because it splits them silently. See
// dap/plan/v3/spec_cdt_library_selection.md.
//
// Fail-soft: any error returns ok=false and the caller falls back to libtess2 for THIS FACE only.
std::atomic<uint64_t> g_cdt_fallback_faces{0};
std::atomic<uint64_t> g_cdt_faces{0};

// Even-odd point-in-polygon in UV. Grid points outside the trim region must not be inserted (OCC
// classifier-filters the same way); an outside Steiner point would be triangulated into the hole.
//
// Per-point by default; ADA_TESS_CDT_SCANLINE=1 switches to the per-row variant above, which is
// asymptotically better (O(nv*loop + nu*nv) vs O(nu*nv*loop)) and produces byte-identical output.
// On Ventilator the two are EXACTLY neutral (min 3.73 both, median 3.87 both, 7 interleaved rounds
// on an idle box) because its trim loops are only ~4-40 vertices, so neither term dominates. An
// earlier A/B called the scanline a regression; that was measured under an audit run at load ~6 and
// was noise — do not trust it. The toggle stays so the choice can be settled on a model with large
// loops, where the asymptotics actually bite.
// A/B toggle (ADA_TESS_CDT_SCANLINE=1): per-row crossings instead of per-point. See uv_inside.
const bool CDT_SCANLINE = std::getenv("ADA_TESS_CDT_SCANLINE") != nullptr;

// Per-row even-odd crossings of every trim loop with v = vrow, sorted. O(nv*loop + nu*nv) instead of
// O(nu*nv*loop).
void uv_row_crossings(const std::vector<std::vector<Uv>> &loops, double vrow, std::vector<double> &xs) {
    xs.clear();
    for (const auto &l : loops) {
        if (l.size() < 2)
            continue;
        for (size_t i = 0, j = l.size() - 1; i < l.size(); j = i++) {
            const Uv &a = l[i];
            const Uv &b = l[j];
            if ((a[1] > vrow) != (b[1] > vrow)) {
                double t = (vrow - a[1]) / (b[1] - a[1]);
                xs.push_back(a[0] + t * (b[0] - a[0]));
            }
        }
    }
    std::sort(xs.begin(), xs.end());
}

inline bool uv_inside_row(const std::vector<double> &xs, double u) {
    return (size_t) (std::lower_bound(xs.begin(), xs.end(), u) - xs.begin()) % 2 == 1;
}

bool uv_inside(const std::vector<std::vector<Uv>> &loops, const Uv &p) {
    bool in = false;
    for (const auto &l : loops) {
        for (size_t i = 0, j = l.size() - 1; i < l.size(); j = i++) {
            const Uv &a = l[i];
            const Uv &b = l[j];
            if ((a[1] > p[1]) != (b[1] > p[1])) {
                double t = (p[1] - a[1]) / (b[1] - a[1]);
                if (p[0] < a[0] + t * (b[0] - a[0]))
                    in = !in;
            }
        }
    }
    return in;
}

Tess2Out run_detria(const Surface &surf, const std::vector<std::vector<Uv>> &loops_uv,
                    const std::vector<std::vector<Vec3>> *loops_p3, const TessParams &tp, double su, double sv) {
    Tess2Out out;
    if (loops_uv.empty() || loops_uv[0].size() < 3)
        return out;

    // thread_local so capacity survives across faces: measured zero allocations after the first
    // face, which matters at ~300k faces. detria ALIASES both the point and index arrays (it does
    // not copy), so these must outlive triangulate() — hence thread_local, not locals.
    thread_local detria::Triangulation<detria::PointD, uint32_t> tri;
    thread_local std::vector<detria::PointD> pts;
    thread_local std::vector<std::vector<uint32_t>> loop_idx;
    tri.clear();
    pts.clear();
    loop_idx.clear();

    // Loop vertices FIRST (indices [0,N)), Steiner after ([N,end)) — that keeps `out.pin`
    // index-aligned by construction, with no vertex-index indirection at all.
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (size_t li = 0; li < loops_uv.size(); ++li) {
        std::vector<uint32_t> idx;
        idx.reserve(loops_uv[li].size());
        for (size_t k = 0; k < loops_uv[li].size(); ++k) {
            const Uv &p = loops_uv[li][k];
            umin = std::min(umin, p[0]);
            umax = std::max(umax, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
            idx.push_back((uint32_t) pts.size());
            pts.push_back({p[0] * su, p[1] * sv});
            out.verts.push_back({p[0], p[1]});
            out.pin.push_back((loops_p3 && li < loops_p3->size() && k < (*loops_p3)[li].size()) ? (*loops_p3)[li][k]
                                                                                                : nan_vec());
        }
        if (idx.size() >= 3)
            loop_idx.push_back(std::move(idx));
    }
    if (loop_idx.empty())
        return out;
    const size_t n_boundary = pts.size();

    // Interior Steiner points: the surface's own curvature grid, classifier-filtered to the trim
    // region. Skipping any point that coincides with a loop vertex is not optional — detria's dedup
    // runs BEFORE constraints, so DuplicatePointsFound is the first error you hit in practice.
    if (tp.cdt.grid_steiner) {
        int nu = 0, nv = 0;
        uv_grid_res(surf, nu, nv);
        const double du = umax - umin, dv = vmax - vmin;
        const double eps = 1e-9 * std::max(std::max(du, dv), 1.0);
        // Quantized loop-vertex set, so the coincidence test is O(1) per grid point rather than
        // O(loop). It has to be cheap: the grid runs to 200k points per face (uv_grid_res's cap) and
        // loops carry hundreds of vertices, so the naive scan is the dominant per-face cost.
        // Skipping coincident points is not optional — detria's dedup runs BEFORE constraints, so
        // DuplicatePointsFound is the first error hit in practice, not PointOnConstrainedEdge.
        thread_local std::unordered_set<uint64_t> loop_keys;
        loop_keys.clear();
        const double inv = 1.0 / std::max(eps, 1e-300);
        auto key_of = [&](const Uv &q) {
            return ((uint64_t) (int64_t) std::llround(q[0] * inv) << 32) ^
                   (uint64_t) (uint32_t) (int64_t) std::llround(q[1] * inv);
        };
        for (const auto &l : loops_uv)
            for (const Uv &q : l)
                loop_keys.insert(key_of(q));
        thread_local std::vector<double> xs;
        for (int j = 1; j < nv; ++j) {
            const double vrow = vmin + dv * j / nv;
            if (CDT_SCANLINE) {
                uv_row_crossings(loops_uv, vrow, xs);
                if (xs.empty())
                    continue; // row misses the trim region entirely
            }
            for (int i = 1; i < nu; ++i) {
                Uv g{umin + du * i / nu, vrow};
                if (loop_keys.count(key_of(g)))
                    continue;
                if (!(CDT_SCANLINE ? uv_inside_row(xs, g[0]) : uv_inside(loops_uv, g)))
                    continue;
                pts.push_back({g[0] * su, g[1] * sv});
                out.verts.push_back(g);
                out.pin.push_back(nan_vec()); // interior: takes the surface point in refine_and_emit
            }
        }
    }

    tri.setPoints(pts);
    tri.addOutline(loop_idx[0]);
    for (size_t i = 1; i < loop_idx.size(); ++i)
        tri.addHole(loop_idx[i]);
    if (!tri.triangulate(true)) {
        if (TESSDBG)
            std::fprintf(stderr, "TESSDBG   detria failed: %s\n", tri.getErrorMessage().c_str());
        out.verts.clear();
        out.pin.clear();
        return out;
    }
    tri.forEachTriangle([&](detria::Triangle<uint32_t> t) { out.tris.push_back({t.x, t.y, t.z}); },
                        /*cwTriangles=*/false); // CCW, matching run_tess2's winding
    if (out.tris.empty()) {
        out.verts.clear();
        out.pin.clear();
        return out;
    }
    (void) n_boundary;
    out.ok = true;
    return out;
}

// Trim-respecting emit for the CDT track. Same contract as emit_uv_region.
bool emit_cdt_region(const Surface &s, const std::vector<std::vector<Uv>> &loops_uv, const TessParams &tp,
                     bool same_sense, Mesh &mesh, const std::vector<std::vector<Vec3>> *loops_p3) {
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const auto &l : loops_uv)
        for (const Uv &p : l) {
            umin = std::min(umin, p[0]);
            umax = std::max(umax, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
        }
    auto [su, sv] = metric_scale(s, umin, umax, vmin, vmax);
    g_cdt_faces.fetch_add(1, std::memory_order_relaxed);
    Tess2Out r = run_detria(s, loops_uv, loops_p3, tp, su, sv);
    if (!r.ok) {
        // Per-face fallback, never per-model: a face detria rejects is only as bad as the status quo.
        g_cdt_fallback_faces.fetch_add(1, std::memory_order_relaxed);
        // Still freeze: the face is pinned, so refine_uv splitting its boundary would re-open the
        // seam detria was meant to close. Measured: without this the cdt track goes 0 -> 207 edges.
        return emit_uv_region(s, loops_uv, tp, same_sense, mesh, loops_p3, &tp.cdt.freeze_boundary,
                              tp.cdt.converged_frac);
    }
    refine_and_emit(s, std::move(r.verts), std::move(r.tris), su, sv, tp, same_sense, mesh, r.pin,
                    tp.cdt.freeze_boundary, tp.cdt.converged_frac);
    return true;
}

bool emit_uv_region(const Surface &s, const std::vector<std::vector<Uv>> &loops_uv, const TessParams &tp,
                    bool same_sense, Mesh &mesh, const std::vector<std::vector<Vec3>> *loops_p3 = nullptr,
                    const bool *freeze_override = nullptr, double conv_override = 0.0) {
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const auto &l : loops_uv)
        for (const Uv &p : l) {
            umin = std::min(umin, p[0]);
            umax = std::max(umax, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
        }
    auto [su, sv] = metric_scale(s, umin, umax, vmin, vmax);
    Tess2Out r = run_tess2(loops_uv, su, sv, loops_p3);
    if (!r.ok) {
        // Shrunk-hole retry: no pins (rare degenerate-hole case — the shrunk contour no longer
        // corresponds 1:1 to the shared-edge points, so pinning it would misplace vertices).
        r = tess2_with_shrunk_holes(loops_uv, su, sv);
        if (!r.ok)
            return false;
    }
    refine_and_emit(s, std::move(r.verts), std::move(r.tris), su, sv, tp, same_sense, mesh, r.pin,
                    freeze_override ? *freeze_override : tp.libtess2.freeze_boundary,
                    freeze_override ? conv_override : tp.libtess2.converged_frac);
    return true;
}

// Grid a UV rectangle at ~2 samples/knot-span then refine (tessellate_uv_grid).
// Conforming grid lines: the uniform samples, PLUS the trim loop's own parameters along each side.
// `ring` are the loop's UV points; `along_u` picks which axis we are collecting.
//
// Why this is cheap here and not a redesign: tessellate_full_patch's gate is area_ratio >= 0.995,
// i.e. the trim loop IS the UV rectangle, so its points already lie on the bbox sides and the trim
// edge IS the iso-curve. The crack on these faces is therefore not a surface divergence at all — it
// is a SAMPLING mismatch on a curve both faces agree on: the grid samples the ring at uniform
// parameters while the neighbour samples the same shared edge at the edge-discretization's
// parameters. Putting the loop's parameters into the grid lines removes the mismatch and lets the
// ring pin. The interior stays a structured tensor grid, untouched.
//
// This is NOT the failed annulus (2026-07-13 #3), which ADDED a thin badly-shaped strip; and not
// grid_via_emit, which discards the grid and over-refines (+44% tris, 481 nonmanifold, measured).
std::vector<double> conform_lines(const std::vector<Uv> &ring, bool along_u, double a0, double a1, double b0, double b1,
                                  int n_uniform, int max_ratio, bool &ok) {
    std::vector<double> lines;
    lines.reserve(n_uniform + 1 + ring.size());
    for (int i = 0; i <= n_uniform; ++i)
        lines.push_back(a0 + (a1 - a0) * i / n_uniform);
    // A loop point contributes its parameter only if it lies on a side PERPENDICULAR to this axis —
    // i.e. a point on a b=const side pins a line at its `a`. Points on the other two sides already
    // sit on the a0/a1 lines and would contribute nothing but duplicates.
    const double db = std::abs(b1 - b0);
    const double eps_b = 1e-3 * (db > 0 ? db : 1.0);
    for (const Uv &p : ring) {
        const double a = along_u ? p[0] : p[1];
        const double b = along_u ? p[1] : p[0];
        if (std::abs(b - b0) <= eps_b || std::abs(b - b1) <= eps_b)
            lines.push_back(clampd(a, std::min(a0, a1), std::max(a0, a1)));
    }
    std::sort(lines.begin(), lines.end());
    const double da = std::abs(a1 - a0);
    const double eps_a = 1e-9 * (da > 0 ? da : 1.0);
    lines.erase(std::unique(lines.begin(), lines.end(), [&](double x, double y) { return std::abs(x - y) <= eps_a; }),
                lines.end());
    // Blow-up guard: a tensor grid cannot have a ring denser than its rows — N boundary samples on
    // one side force N columns across the whole patch. Past max_ratio x the uniform line count the
    // union is not worth it; bail and let the caller keep the plain grid.
    ok = (int) lines.size() <= max_ratio * (n_uniform + 1);
    return lines;
}

// `ring_uv`/`ring_p3` (both or neither): the trim loop's UV points and their shared-edge 3D points.
// When given, the grid lines conform to the loop's parameters and the ring vertices are pinned.
// Null (the default track) => the plain uniform grid, unchanged.
// Curvature grid resolution for a surface. Shared by the libtess2 grid fast path and the CDT
// track, which inserts the SAME grid as interior Steiner points instead of tessellating a bbox.
void uv_grid_res(const Surface &s, int &nu, int &nv) {
    if (const auto *b = dynamic_cast<const BSplineSurface *>(&s)) {
        nu = (int) std::clamp((std::max(b->nu - b->u_degree, 1) * 2), 2, 64);
        nv = (int) std::clamp((std::max(b->nv - b->v_degree, 1) * 2), 2, 8192);
    } else {
        nu = 8;
        nv = 8;
    }
    while ((long) nu * nv > 200000) {
        if (nv > nu)
            nv /= 2;
        else
            nu /= 2;
    }
}

bool tessellate_uv_grid(const Surface &s, double u0, double u1, double v0, double v1, const TessParams &tp,
                        bool same_sense, Mesh &mesh, const std::vector<Uv> *ring_uv = nullptr,
                        const std::vector<Vec3> *ring_p3 = nullptr) {
    int nu, nv;
    uv_grid_res(s, nu, nv);
    // Grid lines. Without a ring these are exactly the uniform samples, so the default path is
    // bit-for-bit what it always was.
    std::vector<double> ul, vl;
    if (ring_uv && ring_p3 && tp.track == TessTrack::Libtess2 && tp.libtess2.conforming_grid) {
        bool uok = false, vok = false;
        const int mr = tp.libtess2.conform_max_ratio;
        ul = conform_lines(*ring_uv, true, u0, u1, v0, v1, nu, mr, uok);
        vl = conform_lines(*ring_uv, false, v0, v1, u0, u1, nv, mr, vok);
        if (EDGEPROF) {
            g_conform_want.fetch_add(ul.size() + vl.size(), std::memory_order_relaxed);
            (uok && vok ? g_conform_ok : g_conform_bail).fetch_add(1, std::memory_order_relaxed);
            if (uok && vok)
                g_conform_lines.fetch_add(ul.size() + vl.size(), std::memory_order_relaxed);
        }
        if (!uok || !vok) {
            ul.clear();
            vl.clear();
        }
    }
    if (ul.empty() || vl.empty()) {
        ul.clear();
        vl.clear();
        for (int i = 0; i <= nu; ++i)
            ul.push_back(u0 + (u1 - u0) * i / nu);
        for (int j = 0; j <= nv; ++j)
            vl.push_back(v0 + (v1 - v0) * j / nv);
        ring_uv = nullptr; // no conforming lines => nothing to pin against
    }
    const int gu = (int) ul.size() - 1, gv = (int) vl.size() - 1;
    std::vector<Uv> verts;
    verts.reserve((size_t) (gu + 1) * (gv + 1));
    for (int j = 0; j <= gv; ++j)
        for (int i = 0; i <= gu; ++i)
            verts.push_back({ul[i], vl[j]});
    uint32_t w = (uint32_t) (gu + 1);
    std::vector<Tri> tris;
    tris.reserve((size_t) gu * gv * 2);
    for (uint32_t j = 0; j < (uint32_t) gv; ++j)
        for (uint32_t i = 0; i < (uint32_t) gu; ++i) {
            uint32_t a = j * w + i;
            tris.push_back({a, a + 1, a + w + 1});
            tris.push_back({a, a + w + 1, a + w});
        }
    if (tris.empty())
        return false;
    if (TESSDBG)
        std::fprintf(stderr, "TESSDBG   tessellate_uv_grid nu=%d nv=%d (u=[%.4f,%.4f] v=[%.4f,%.4f]) init_tris=%zu\n",
                     nu, nv, u0, u1, v0, v1, tris.size());
    auto [su, sv] = metric_scale(s, u0, u1, v0, v1);
    // Pin the ring: every grid vertex on the border gets the shared-edge 3D point of the loop vertex
    // sitting at the same UV. The conforming lines put a grid crossing at each loop parameter, so the
    // match is exact; anything unmatched (interior, or a loop point the guard dropped) stays NaN and
    // falls back to the surface point in refine_and_emit.
    std::vector<Vec3> pin;
    if (ring_uv && ring_p3) {
        pin.assign(verts.size(), nan_vec());
        const double tol_u = 1e-9 * std::max(std::abs(u1 - u0), 1.0);
        const double tol_v = 1e-9 * std::max(std::abs(v1 - v0), 1.0);
        for (size_t k = 0; k < ring_uv->size() && k < ring_p3->size(); ++k) {
            const Uv &p = (*ring_uv)[k];
            auto iu = std::lower_bound(ul.begin(), ul.end(), p[0] - tol_u);
            auto iv = std::lower_bound(vl.begin(), vl.end(), p[1] - tol_v);
            if (iu == ul.end() || iv == vl.end())
                continue;
            if (std::abs(*iu - p[0]) > tol_u || std::abs(*iv - p[1]) > tol_v)
                continue;
            const int i = (int) (iu - ul.begin()), j = (int) (iv - vl.begin());
            if (i != 0 && i != gu && j != 0 && j != gv)
                continue; // only the border ring is shared with a neighbour
            pin[(size_t) j * (size_t) w + (size_t) i] = (*ring_p3)[k];
        }
    }
    refine_and_emit(s, std::move(verts), std::move(tris), su, sv, tp, same_sense, mesh, pin,
                    tp.libtess2.freeze_boundary, tp.libtess2.converged_frac);
    return true;
}

// ---- B-spline full-domain gates ---------------------------------------------------
struct Rect {
    double u0, u1, v0, v1;
    bool ok = false;
};

Rect full_patch_rect(const Surface &s, const std::vector<std::vector<Uv>> &contours) {
    const auto *b = dynamic_cast<const BSplineSurface *>(&s);
    if (!b)
        return {};
    if (contours.size() != 1 || contours[0].size() < 3)
        return {};
    double du0, du1, dv0, dv1;
    b->domain(du0, du1, dv0, dv1);
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const Uv &p : contours[0]) {
        umin = std::min(umin, p[0]);
        umax = std::max(umax, p[0]);
        vmin = std::min(vmin, p[1]);
        vmax = std::max(vmax, p[1]);
    }
    double du = umax - umin, dv = vmax - vmin;
    double ddu = std::abs(du1 - du0), ddv = std::abs(dv1 - dv0);
    if (ddu < 1e-12 || ddv < 1e-12 || du < 1e-6 * ddu || dv < 1e-6 * ddv)
        return {};
    // Fill the UV bbox as a full grid ONLY when the trim loop essentially IS that rectangle. A truly
    // untrimmed patch fills its bbox exactly (straight u=const/v=const domain edges lose no area when
    // discretized, so ratio ~1.0); anything materially below that has curved trim edges cutting into
    // the bbox (e.g. a bevel cut to fit its neighbours — ratio ~0.95), and filling the rectangle would
    // replace those trim curves with straight bbox edges. Those must go through the trim-respecting
    // emit_uv_region path instead.
    double area_ratio = std::abs(poly_area(contours[0])) / (du * dv);
    if (area_ratio < 0.995)
        return {};
    return {clampd(umin, du0, du1), clampd(umax, du0, du1), clampd(vmin, dv0, dv1), clampd(vmax, dv0, dv1), true};
}

Rect full_domain_bspline(const Surface &s, const std::vector<std::vector<Uv>> &contours) {
    const auto *b = dynamic_cast<const BSplineSurface *>(&s);
    if (!b)
        return {};
    if (contours.size() != 1 || contours[0].size() < 3)
        return {};
    double du0, du1, dv0, dv1;
    b->domain(du0, du1, dv0, dv1);
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const Uv &p : contours[0]) {
        umin = std::min(umin, p[0]);
        umax = std::max(umax, p[0]);
        vmin = std::min(vmin, p[1]);
        vmax = std::max(vmax, p[1]);
    }
    double ddu = std::abs(du1 - du0), ddv = std::abs(dv1 - dv0);
    if (ddu < 1e-12 || ddv < 1e-12)
        return {};
    if ((umax - umin) >= 0.9 * ddu && (vmax - vmin) >= 0.9 * ddv)
        return {du0, du1, dv0, dv1, true};
    return {};
}

Rect full_wrap_bspline(const Surface &s, const std::vector<std::vector<Uv>> &contours) {
    const auto *b = dynamic_cast<const BSplineSurface *>(&s);
    if (!b)
        return {};
    auto pu = s.u_period();
    auto pv = s.v_period();
    if (!pu && !pv)
        return {};
    double du0, du1, dv0, dv1;
    b->domain(du0, du1, dv0, dv1);
    double umin = INF, umax = -INF, vmin = INF, vmax = -INF;
    for (const auto &c : contours)
        for (const Uv &p : c) {
            umin = std::min(umin, p[0]);
            umax = std::max(umax, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
        }
    double ddu = std::abs(du1 - du0), ddv = std::abs(dv1 - dv0);
    if (ddu < 1e-12 || ddv < 1e-12)
        return {};
    bool u_full = (umax - umin) >= 0.9 * (pu ? *pu : ddu);
    bool v_full = (vmax - vmin) >= 0.9 * (pv ? *pv : ddv);
    if (u_full && v_full)
        return {du0, du1, dv0, dv1, true};
    return {};
}

bool bspline_has_v_pole(const Surface &s) {
    const auto *b = dynamic_cast<const BSplineSurface *>(&s);
    if (!b)
        return false;
    if (b->nu < 2 || b->nv == 0)
        return false;
    double eps = 1e-6 * std::max(b->size(), 1.0);
    auto collapsed = [&](int iv) {
        Vec3 first = b->ctrl[iv];
        for (int iu = 1; iu < b->nu; ++iu)
            if ((b->ctrl[iu * b->nv + iv] - first).norm() >= eps)
                return false;
        return true;
    };
    return collapsed(0) || collapsed(b->nv - 1);
}

// `contours_p3` (optional, parallel to contours): shared-edge pins. When present the grid conforms
// to contours[0] and its ring is pinned — contours[0] IS the rectangle here (area_ratio >= 0.995).
// ---- annulus patch: a structured interior stitched to the PINNED trim loop --------------------
//
// The grid is the right algorithm for a full patch (full_patch_rect gates area_ratio >= 0.995, so
// the trim loop IS the rectangle and a tensor grid is exactly correct, and cheap). Its ONLY defect
// is that its boundary ring sits at uniform parameters, not the loop's, so it cannot pin — which
// makes it the last crack source. detria fixes that but costs ~3-4x per face (crane: 10.4% of faces
// -> +25.6%).
//
// So: keep the grid, INSET it by one cell, and zipper the gap to the loop. The interior stays a
// tensor grid; only the ring is stitched, and the ring's outer vertices ARE the loop's vertices, so
// they pin. O(m+n) triangles, no CDT.
//
// Why this is not 2026-07-13's annulus (which over-refined, ~4k tris/face at ~0.42s/face): that one
// stitched to a loop that, for a full patch, lies essentially ON the rect border — a zero-width
// strip, hence slivers. Insetting by a full cell gives the strip real width. And the over-refinement
// it hit was refinement fighting a frozen boundary, which converged_frac now tames.
//
// Returns false (caller keeps the plain grid) whenever the shape isn't safely stitchable.

// Signed area in UV; used to put both rings in the same winding before zippering (opposite windings
// would cross the strip over itself).
double ring_area(const std::vector<Uv> &r) {
    double a = 0;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
        a += (r[j][0] * r[i][1]) - (r[i][0] * r[j][1]);
    return a * 0.5;
}

// Normalized cumulative arc length per vertex, [0,1). The zipper merges the two rings on this
// parameter, which is what keeps the strip's triangles well-shaped when the rings have very
// different vertex counts (uniform inner ring vs the loop's own discretization).
void ring_params(const std::vector<Uv> &r, std::vector<double> &t) {
    t.assign(r.size(), 0.0);
    double total = 0;
    for (size_t i = 0; i < r.size(); ++i) {
        const Uv &a = r[i];
        const Uv &b = r[(i + 1) % r.size()];
        total += std::hypot(b[0] - a[0], b[1] - a[1]);
        t[i] = total;
    }
    if (total <= 0)
        return;
    for (double &x : t)
        x /= total;
    // t[i] is the param AT THE END of edge i; shift so t[i] is the param of vertex i.
    double last = 0;
    for (size_t i = 0; i < t.size(); ++i) {
        double cur = t[i];
        t[i] = last;
        last = cur;
    }
}

bool tessellate_annulus_patch(const Surface &s, const Rect &r, const std::vector<Uv> &loop_uv,
                              const std::vector<Vec3> &loop_p3, const TessParams &tp, bool same_sense, Mesh &mesh) {
    int nu = 0, nv = 0;
    uv_grid_res(s, nu, nv);
    if (nu < 3 || nv < 3 || loop_uv.size() < 3 || loop_uv.size() != loop_p3.size())
        return false; // too coarse to inset, or no usable loop

    const double du = (r.u1 - r.u0) / nu, dv = (r.v1 - r.v0) / nv;
    // Inset grid: interior nodes only, i in [1,nu-1], j in [1,nv-1].
    const int gu = nu - 2, gv = nv - 2; // cells across the inset grid
    if (gu < 1 || gv < 1)
        return false;

    std::vector<Uv> verts;
    std::vector<Vec3> pin;
    std::vector<Tri> tris;
    verts.reserve((size_t) (gu + 1) * (gv + 1) + loop_uv.size());
    for (int j = 0; j <= gv; ++j)
        for (int i = 0; i <= gu; ++i) {
            verts.push_back({r.u0 + du * (i + 1), r.v0 + dv * (j + 1)});
            pin.push_back(nan_vec()); // interior: the surface point is correct
        }
    const uint32_t w = (uint32_t) (gu + 1);
    for (uint32_t j = 0; j < (uint32_t) gv; ++j)
        for (uint32_t i = 0; i < (uint32_t) gu; ++i) {
            uint32_t a = j * w + i;
            tris.push_back({a, a + 1, a + w + 1});
            tris.push_back({a, a + w + 1, a + w});
        }

    // Inner ring, CCW: bottom L->R, right B->T, top R->L, left T->B.
    std::vector<uint32_t> inner;
    inner.reserve(2 * (gu + gv));
    for (int i = 0; i <= gu; ++i)
        inner.push_back((uint32_t) i);
    for (int j = 1; j <= gv; ++j)
        inner.push_back((uint32_t) (j * w + gu));
    for (int i = gu - 1; i >= 0; --i)
        inner.push_back((uint32_t) (gv * w + i));
    for (int j = gv - 1; j >= 1; --j)
        inner.push_back((uint32_t) (j * w));

    // Outer ring = the trim loop itself, appended as its own vertices and PINNED.
    std::vector<Uv> inner_uv;
    inner_uv.reserve(inner.size());
    for (uint32_t k : inner)
        inner_uv.push_back(verts[k]);
    std::vector<Uv> outer_uv = loop_uv;
    std::vector<Vec3> outer_p3 = loop_p3;
    if ((ring_area(inner_uv) > 0) != (ring_area(outer_uv) > 0)) {
        std::reverse(outer_uv.begin(), outer_uv.end());
        std::reverse(outer_p3.begin(), outer_p3.end());
    }
    const uint32_t obase = (uint32_t) verts.size();
    for (size_t k = 0; k < outer_uv.size(); ++k) {
        verts.push_back(outer_uv[k]);
        pin.push_back(outer_p3[k]); // the shared-edge point — this is what makes the seam close
    }

    // Zipper the strip on normalized arc length. Align the two rings' starts first, otherwise the
    // strip spirals: pick the outer vertex nearest the inner ring's start.
    std::vector<double> ti, to;
    ring_params(inner_uv, ti);
    ring_params(outer_uv, to);
    if (ti.empty() || to.empty())
        return false;
    size_t o0 = 0;
    double best = 1e300;
    for (size_t k = 0; k < outer_uv.size(); ++k) {
        double d = std::hypot(outer_uv[k][0] - inner_uv[0][0], outer_uv[k][1] - inner_uv[0][1]);
        if (d < best) {
            best = d;
            o0 = k;
        }
    }
    const size_t m = inner.size(), n = outer_uv.size();
    auto IN = [&](size_t i) { return inner[i % m]; };
    auto OUT = [&](size_t j) { return (uint32_t) (obase + (o0 + j) % n); };
    auto TI = [&](size_t i) { return ti[i % m] + (double) (i / m); };
    auto TO = [&](size_t j) {
        double t = to[(o0 + j) % n] - to[o0];
        if (t < 0)
            t += 1.0;
        return t + (double) (j / n);
    };
    size_t i = 0, j = 0;
    while (i < m || j < n) {
        const bool adv_inner = (j >= n) || (i < m && TI(i + 1) <= TO(j + 1));
        // Winding must MATCH the interior grid's, which is CCW ({a, a+1, a+w+1}). With both rings
        // CCW and the inner one inside, walking the inner ring forward puts the outer ring on the
        // right, so the naive {IN(i), IN(i+1), OUT(j)} is CLOCKWISE — inverted against the grid.
        // refine_and_emit only normalises winding GLOBALLY (one signed-area flip for the whole
        // face), so a strip wound the other way survives as inverted triangles and shows up as
        // non-manifold edges rather than as a visible flip.
        if (adv_inner) {
            tris.push_back({IN(i), OUT(j), IN(i + 1)});
            ++i;
        } else {
            tris.push_back({IN(i), OUT(j), OUT(j + 1)});
            ++j;
        }
    }

    auto [su, sv] = metric_scale(s, r.u0, r.u1, r.v0, r.v1);
    // Freeze: the outer ring is the shared boundary; splitting it would undo the pin. converged_frac
    // is what makes that affordable (see Libtess2Opts::converged_frac).
    refine_and_emit(s, std::move(verts), std::move(tris), su, sv, tp, same_sense, mesh, pin, true,
                    tp.libtess2.converged_frac);
    return true;
}

bool tessellate_full_patch(const Surface &s, const std::vector<std::vector<Uv>> &contours, const TessParams &tp,
                           bool same_sense, Mesh &mesh, const std::vector<std::vector<Vec3>> *contours_p3 = nullptr) {
    Rect r = full_patch_rect(s, contours);
    if (!r.ok)
        return false;
    const std::vector<Uv> *ring = nullptr;
    const std::vector<Vec3> *ring3 = nullptr;
    if (contours_p3 && !contours_p3->empty() && !contours.empty() && contours[0].size() == (*contours_p3)[0].size()) {
        ring = &contours[0];
        ring3 = &(*contours_p3)[0];
    }
    // Annulus: the grid's structured interior, but stitched to the PINNED trim loop instead of
    // ending at a uniform-parameter ring it cannot pin. Falls through to the plain grid if the face
    // isn't safely stitchable.
    if (tp.libtess2.annulus_patch && ring && ring3 &&
        tessellate_annulus_patch(s, r, *ring, *ring3, tp, same_sense, mesh))
        return true;
    return tessellate_uv_grid(s, r.u0, r.u1, r.v0, r.v1, tp, same_sense, mesh, ring, ring3);
}

// Full-domain tessellation for a closed quadric / B-spline whose only boundary is a slit/seam.
bool tessellate_unbounded(const Surface &s, const TessParams &tp, bool same_sense, Mesh &mesh) {
    double u0, u1, v0, v1;
    if (dynamic_cast<const SphereSurface *>(&s)) {
        u0 = 0;
        u1 = TWO_PI;
        v0 = -PI / 2;
        v1 = PI / 2;
    } else if (dynamic_cast<const TorusSurface *>(&s)) {
        u0 = 0;
        u1 = TWO_PI;
        v0 = 0;
        v1 = TWO_PI;
    } else if (const auto *b = dynamic_cast<const BSplineSurface *>(&s)) {
        b->domain(u0, u1, v0, v1);
        // Full-domain B-spline via the knot-span grid + deflection refinement (budgeted),
        // NOT the raw parameter-step grid below: u/v parameter scales are unrelated, and
        // `dv = min(v_step, du)` explodes when one span is degenerate. Real case (Valve
        // Hall, a 6-face 'DC Voltage Divider' body): a slit-bounded deg-7x7 patch with
        // u-domain 6.5e-8 wide clamped dv to 6.5e-8 over v-range 0.5 -> nv = 7.8M rows
        // -> 62.4M triangles from ONE face, 114 s of a 123 s conversion.
        return tessellate_uv_grid(s, u0, u1, v0, v1, tp, same_sense, mesh);
    } else {
        return false;
    }
    // Angular parameter grid for the closed quadrics (u/v are both angles here, so the
    // isotropy clamp is sound).
    double du = s.u_step(tp.deflection, tp.max_angle);
    double dv = std::min(s.v_step(tp.deflection, tp.max_angle), du);
    int nu = std::max(4, (int) std::ceil((u1 - u0) / du));
    int nv = std::max(3, (int) std::ceil((v1 - v0) / dv));
    uint32_t base = mesh.base();
    for (int j = 0; j <= nv; ++j) {
        double v = v0 + (v1 - v0) * j / nv;
        for (int i = 0; i <= nu; ++i) {
            double u = u0 + (u1 - u0) * i / nu;
            Vec3 n = s.normal(u, v);
            if (!same_sense)
                n = n * -1.0;
            mesh.push_vertex(s.point(u, v), n);
        }
    }
    uint32_t w = (uint32_t) (nu + 1);
    for (uint32_t j = 0; j < (uint32_t) nv; ++j)
        for (uint32_t i = 0; i < (uint32_t) nu; ++i) {
            uint32_t a = base + j * w + i, b = a + 1, c = a + w, d = c + 1;
            if (same_sense) {
                mesh.push_index(a);
                mesh.push_index(b);
                mesh.push_index(d);
                mesh.push_index(a);
                mesh.push_index(d);
                mesh.push_index(c);
            } else {
                mesh.push_index(a);
                mesh.push_index(d);
                mesh.push_index(b);
                mesh.push_index(a);
                mesh.push_index(c);
                mesh.push_index(d);
            }
        }
    return true;
}

// ---- periodic paths ---------------------------------------------------------------
bool tessellate_periodic_winding(const Surface &s, const std::vector<LoopUv> &loops_uv, const TessParams &tp,
                                 bool same_sense, Mesh &mesh) {
    auto pu = s.u_period();
    if (!pu)
        return false;
    std::vector<const LoopUv *> winders;
    for (const LoopUv &l : loops_uv)
        if (l.w != 0)
            winders.push_back(&l);
    if (winders.size() != 1 || std::abs(winders[0]->w) != 1 || winders[0]->uv.size() < 3)
        return false;
    const LoopUv &wl = *winders[0];
    double vmin = INF, vmax = -INF;
    for (const Uv &p : wl.uv) {
        vmin = std::min(vmin, p[1]);
        vmax = std::max(vmax, p[1]);
    }
    if (FDBG)
        std::fprintf(stderr, "FDBG winding %-10s vext=%.4g caps=%d\n", surf_kind(s), vmax - vmin,
                     (int) s.v_caps().has_value());
    if (!(vmax - vmin > 1e-9 * std::max(std::max(std::abs(vmax), std::abs(vmin)), 1.0)))
        return false;

    double cb = -INF, ct = INF;
    if (auto caps = s.v_caps()) {
        cb = caps->first;
        ct = caps->second;
    }
    auto below = [&](double c) { return c + 1e-4 * std::max(std::abs(vmax - c), 1.0); };
    auto above = [&](double c) { return c - 1e-4 * std::max(std::abs(c - vmin), 1.0); };
    double vfar;
    bool cbf = std::isfinite(cb), ctf = std::isfinite(ct);
    if (cbf && ctf) {
        vfar = wl.interior_above ? above(ct) : below(cb);
    } else if (cbf && !ctf) {
        vfar = below(cb);
    } else if (!cbf && ctf) {
        vfar = above(ct);
    } else {
        if (auto pv = s.v_period()) {
            if (vmax - vmin >= *pv - 1e-9)
                return false;
        }
        vfar = wl.interior_above ? vmax : vmin;
    }
    double u0 = wl.uv[0][0];
    double uend = wl.uv.back()[0];
    std::vector<Uv> poly = wl.uv;
    double du = s.u_step(tp.deflection, tp.max_angle);
    int n = (int) std::clamp((int) std::ceil(std::abs(uend - u0) / du), 2, 256);
    for (int k = 0; k <= n; ++k)
        poly.push_back({uend + (u0 - uend) * k / n, vfar});
    std::vector<std::vector<Uv>> all = {poly};
    for (const LoopUv &l : loops_uv)
        if (l.w == 0)
            all.push_back(l.uv);
    return emit_uv_region(s, all, tp, same_sense, mesh);
}

bool tessellate_periodic_band(const Surface &s, const std::vector<LoopUv> &loops_uv, const TessParams &tp,
                              bool same_sense, Mesh &mesh) {
    auto puopt = s.u_period();
    if (!puopt)
        return false;
    double per = *puopt;
    struct BandCurve {
        std::vector<Uv> pts;
        bool interior_above;
    };
    std::vector<BandCurve> curves;
    std::vector<std::vector<Uv>> holes;
    std::optional<double> seam;
    for (const LoopUv &l : loops_uv) {
        if (l.w == 0) {
            holes.push_back(l.uv);
            continue;
        }
        if (std::abs(l.w) != 1 || l.uv.size() < 3)
            return false;
        std::vector<Uv> ext = l.uv;
        ext.push_back({l.uv[0][0] + l.w * per, l.uv[0][1]});
        if (!seam)
            seam = ext[0][0];
        double c = *seam;
        std::optional<std::pair<size_t, Uv>> cut;
        for (size_t i = 0; i + 1 < ext.size() && !cut; ++i) {
            double u0 = ext[i][0], u1 = ext[i + 1][0];
            long k0 = (long) std::floor((std::min(u0, u1) - c) / per);
            for (long k = k0; k <= k0 + 2; ++k) {
                double cv = c + k * per;
                if (std::abs(u0 - cv) < 1e-12) {
                    cut = std::make_pair(i, ext[i]);
                    break;
                }
                if ((u0 - cv) * (u1 - cv) <= 0.0 && std::abs(u1 - u0) > 1e-12) {
                    double t = (cv - u0) / (u1 - u0);
                    if (t >= 0.0 && t <= 1.0) {
                        double v = ext[i][1] + t * (ext[i + 1][1] - ext[i][1]);
                        cut = std::make_pair(i, Uv{cv, v});
                        break;
                    }
                }
            }
        }
        if (!cut)
            return false;
        size_t ci = cut->first;
        Uv cp = cut->second;
        size_t n = l.uv.size();
        std::vector<Uv> open;
        open.push_back(cp);
        for (size_t j = 1; j <= n; ++j) {
            size_t idx = (ci + j) % n;
            double wraps = (double) ((ci + j) / n);
            const Uv &p = l.uv[idx];
            open.push_back({p[0] + wraps * l.w * per, p[1]});
        }
        open.push_back({cp[0] + l.w * per, cp[1]});
        // dedup consecutive
        std::vector<Uv> dd;
        for (const Uv &p : open)
            if (dd.empty() || std::abs(p[0] - dd.back()[0]) >= 1e-12 || std::abs(p[1] - dd.back()[1]) >= 1e-12)
                dd.push_back(p);
        open.swap(dd);
        if (l.w < 0)
            std::reverse(open.begin(), open.end());
        double shift = std::round((open[0][0] - c) / per) * per;
        if (shift != 0.0)
            for (Uv &p : open)
                p[0] -= shift;
        curves.push_back({open, l.interior_above});
    }
    if (curves.empty())
        return false;
    auto mean_v = [](const BandCurve &c) {
        double s = 0;
        for (const Uv &p : c.pts)
            s += p[1];
        return s / c.pts.size();
    };
    std::stable_sort(curves.begin(), curves.end(),
                     [&](const BandCurve &a, const BandCurve &b) { return mean_v(a) < mean_v(b); });
    if (curves.size() % 2 == 1) {
        auto caps = s.v_caps();
        if (!caps)
            return false;
        double c = seam ? *seam : 0.0;
        bool top_wants = !curves.empty() && curves.back().interior_above;
        bool bot_wants = !curves.empty() && !curves.front().interior_above;
        auto cap_line = [&](double vcap) {
            int n = (int) std::clamp((int) std::ceil(per / s.u_step(tp.deflection, tp.max_angle)), 2, 256);
            std::vector<Uv> out;
            for (int i = 0; i <= n; ++i)
                out.push_back({c + per * i / n, vcap});
            return out;
        };
        if (top_wants && std::isfinite(caps->second)) {
            curves.push_back({cap_line(caps->second), false});
            std::stable_sort(curves.begin(), curves.end(),
                             [&](const BandCurve &a, const BandCurve &b) { return mean_v(a) < mean_v(b); });
        } else if (bot_wants && std::isfinite(caps->first)) {
            curves.insert(curves.begin(), {cap_line(caps->first), true});
        } else {
            return false;
        }
    }
    bool any = false;
    for (size_t i = 0; i + 1 < curves.size(); i += 2) {
        const BandCurve &bottom = curves[i];
        const BandCurve &top = curves[i + 1];
        double vb = mean_v(bottom), vt = mean_v(top);
        std::vector<Uv> poly = bottom.pts;
        for (auto it = top.pts.rbegin(); it != top.pts.rend(); ++it)
            poly.push_back(*it);
        std::vector<std::vector<Uv>> contours = {poly};
        for (const auto &h : holes) {
            double hv = 0;
            for (const Uv &p : h)
                hv += p[1];
            hv /= std::max((size_t) 1, h.size());
            if (hv >= std::min(vb, vt) && hv <= std::max(vb, vt)) {
                double hu = 0;
                for (const Uv &p : h)
                    hu += p[0];
                hu /= std::max((size_t) 1, h.size());
                double c = bottom.pts[0][0];
                double shift = std::round((hu - (c + per / 2.0)) / per) * per;
                std::vector<Uv> hh = h;
                if (shift != 0.0)
                    for (Uv &p : hh)
                        p[0] -= shift;
                contours.push_back(hh);
            }
        }
        any |= emit_uv_region(s, contours, tp, same_sense, mesh);
    }
    return any;
}

bool complement_interior(const std::vector<std::vector<Uv>> &contours) {
    if (contours.size() < 2)
        return false;
    auto centroid = [](const std::vector<Uv> &c) -> Uv {
        double n = std::max((size_t) 1, c.size());
        double x = 0, y = 0;
        for (const Uv &p : c) {
            x += p[0];
            y += p[1];
        }
        return {x / n, y / n};
    };
    size_t outer = 0;
    double best = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double a = std::abs(poly_area(contours[i]));
        if (a >= best) { // >= : last maximal wins, matching Rust Iterator::max_by
            best = a;
            outer = i;
        }
    }
    for (size_t k = 0; k < contours.size(); ++k)
        if (k != outer && !point_in_poly(centroid(contours[k]), contours[outer]))
            return true;
    return false;
}

bool tessellate_periodic_complement(const Surface &s, const std::vector<std::vector<Uv>> &contours, double per_u,
                                    const TessParams &tp, bool same_sense, Mesh &mesh) {
    double umin = INF, vmin = INF, vmax = -INF;
    for (const auto &c : contours)
        for (const Uv &p : c) {
            umin = std::min(umin, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
        }
    double pad = (vmax - vmin) * 1e-3 + 1e-6;
    double vlo = vmin - pad, vhi = vmax + pad;
    if (auto caps = s.v_caps()) {
        double m = 1e-4 * std::max(std::abs(vhi - vlo), 1.0);
        if (std::isfinite(caps->first))
            vlo = std::max(vlo, caps->first + m);
        if (std::isfinite(caps->second))
            vhi = std::min(vhi, caps->second - m);
    }
    double u0 = umin - 1e-6;
    std::vector<std::vector<Uv>> all;
    all.push_back({{u0, vlo}, {u0 + per_u, vlo}, {u0 + per_u, vhi}, {u0, vhi}});
    for (const auto &c : contours)
        all.push_back(c);
    return emit_uv_region(s, all, tp, same_sense, mesh);
}

// ---- plane fit fallbacks ----------------------------------------------------------
bool surface_is_planar(const Surface &s) {
    if (dynamic_cast<const PlaneSurface *>(&s))
        return true;
    const auto *b = dynamic_cast<const BSplineSurface *>(&s);
    if (!b)
        return false;
    const auto &cps = b->ctrl;
    if (cps.size() < 3)
        return true;
    Vec3 o = cps[0];
    Vec3 a = cps[0];
    double amax = -1;
    for (const Vec3 &p : cps) {
        double d = (o - p).norm();
        if (d > amax) {
            amax = d;
            a = p;
        }
    }
    Vec3 da = a - o;
    if (da.norm() < 1e-9)
        return true;
    Vec3 normal{0, 0, 0};
    for (const Vec3 &p : cps) {
        Vec3 n = da.cross(p - o);
        if (n.norm() > normal.norm())
            normal = n;
    }
    if (normal.norm() < 1e-12)
        return true;
    Vec3 n = normal.normalized();
    double span = 0;
    for (const Vec3 &p : cps)
        span = std::max(span, (p - o).norm());
    for (const Vec3 &p : cps)
        if (std::abs((p - o).dot(n)) > 1e-3 * span + 1e-6)
            return false;
    return true;
}

std::shared_ptr<PlaneSurface> fit_plane(const std::vector<Loop3> &loops) {
    if (loops.empty() || loops[0].pts.size() < 3)
        return nullptr;
    const auto &outer = loops[0].pts;
    Vec3 n{0, 0, 0}, c{0, 0, 0};
    size_t m = outer.size();
    for (size_t i = 0; i < m; ++i) {
        Vec3 a = outer[i];
        Vec3 b = outer[(i + 1) % m];
        n = n + Vec3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
        c = c + a;
    }
    if (n.norm() < 1e-12)
        return nullptr;
    n = n.normalized();
    c = c * (1.0 / m);
    double span = 0, dev = 0;
    for (const Loop3 &l : loops)
        for (const Vec3 &p : l.pts) {
            dev = std::max(dev, std::abs((p - c).dot(n)));
            span = std::max(span, (p - c).norm());
        }
    if (dev > 1e-3 * std::max(span, 1e-9) + 1e-6)
        return nullptr;
    return std::make_shared<PlaneSurface>(Frame::from_axis_ref(c, n, Vec3{1, 0, 0}));
}

// ---- face mesher (face_to_mesh) -----------------------------------------------------
// returns empty string on success, else a failure reason
const char *face_to_mesh(const Surface &surf, const std::vector<Loop3> &loops3d, const TessParams &tp, bool same_sense,
                         Mesh &mesh) {
    if (surf.is_quadric()) {
        double tol = std::max(surf.approx_size(), 1.0);
        for (const Loop3 &l : loops3d)
            for (const Vec3 &p : l.pts)
                if (surf.point_residual(p) > tol)
                    return "face boundary does not lie on its surface (malformed)";
    }
    auto per_u = surf.u_period();
    auto per_v = surf.v_period();

    double eps_cap = 1e-7 * std::max(surf.approx_size(), 1.0);
    std::vector<Vec3> cap_pts;
    if (per_u) {
        if (auto caps = surf.v_caps()) {
            for (double vv : {caps->first, caps->second})
                if (std::isfinite(vv))
                    cap_pts.push_back(surf.point(0.0, vv));
        }
    }
    auto singular = [&](const Vec3 &p) {
        for (const Vec3 &c : cap_pts)
            if ((p - c).norm() < eps_cap)
                return true;
        return false;
    };

    std::vector<LoopUv> loops_uv;
    std::optional<double> u_ref;
    for (const Loop3 &l : loops3d) {
        size_t n = l.pts.size();
        size_t start = n;
        for (size_t i = 0; i < n; ++i)
            if (!singular(l.pts[i])) {
                start = i;
                break;
            }
        if (start == n)
            start = 0;
        std::vector<Vec3> pts(n);
        for (size_t i = 0; i < n; ++i)
            pts[i] = l.pts[(start + i) % n];

        std::vector<Uv> uv;
        uv.reserve(n);
        std::vector<char> sing;
        sing.reserve(n);
        bool have_hint = false;
        double hu = 0, hv = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            double u = std::numeric_limits<double>::quiet_NaN();
            double v = u;
            bool uv_ok = surf.uv(pts[i], have_hint ? hu : Surface::NAN_HINT, have_hint ? hv : Surface::NAN_HINT, u, v);
            if (DIAG) {
                // Record the residual of the COMPUTED (u,v) — before the failure handler below
                // overwrites it — so the number reflects the inversion, not the recovery.
                FaceDiag &fd = face_diag();
                if (u == u && v == v)
                    fd.r.push_back((surf.point(u, v) - pts[i]).norm());
                if (!uv_ok) {
                    ++fd.n_uv_fail;
                    if (i > 0)
                        ++fd.n_collapsed;
                }
            }
            if (!uv_ok) {
                u = i > 0 ? uv[i - 1][0] : 0.0;
                v = i > 0 ? uv[i - 1][1] : 0.0;
            }
            hu = u;
            hv = v;
            have_hint = true;
            if (i > 0) {
                double pu = uv[i - 1][0], pv = uv[i - 1][1];
                if (per_u) {
                    while (u - pu > *per_u / 2.0)
                        u -= *per_u;
                    while (pu - u > *per_u / 2.0)
                        u += *per_u;
                }
                if (per_v) {
                    while (v - pv > *per_v / 2.0)
                        v -= *per_v;
                    while (pv - v > *per_v / 2.0)
                        v += *per_v;
                }
            }
            uv.push_back({u, v});
            sing.push_back(singular(pts[i]) ? 1 : 0);
        }
        // pts3 is 1:1 with uv here — the shared-edge point behind each boundary vertex. Both faces
        // of an edge discretize it to byte-identical points, so pinning to these makes their
        // boundary vertices coincide. The singular-cap walk below inserts synthetic seam vertices,
        // which get no pin (NaN => the surface point).
        std::vector<Vec3> pts3 = pts;
        // walk a singular point along the cap line at the u-step
        if (per_u) {
            bool any_sing = false;
            for (char x : sing)
                if (x)
                    any_sing = true;
            if (any_sing) {
                double du = surf.u_step(tp.deflection, tp.max_angle);
                std::vector<Uv> out2;
                std::vector<Vec3> p3out;
                out2.reserve(uv.size() + 8);
                p3out.reserve(uv.size() + 8);
                for (size_t i = 0; i < uv.size(); ++i) {
                    out2.push_back(uv[i]);
                    p3out.push_back(pts3[i]);
                    if (sing[i]) {
                        double pu = uv[i][0];
                        double nu = uv[(i + 1) % uv.size()][0];
                        while (nu - pu > *per_u / 2.0)
                            nu -= *per_u;
                        while (pu - nu > *per_u / 2.0)
                            nu += *per_u;
                        int steps = (int) std::clamp((int) std::ceil(std::abs(nu - pu) / du), 1, 256);
                        for (int k = 1; k <= steps; ++k) {
                            out2.push_back({pu + (nu - pu) * k / steps, uv[i][1]});
                            p3out.push_back(nan_vec()); // synthetic seam vertex — surface point, no pin
                        }
                    }
                }
                uv.swap(out2);
                pts3.swap(p3out);
            }
        }
        int w = 0;
        if (per_u && uv.size() >= 3) {
            double uc = uv[0][0];
            double last = uv.back()[0];
            while (uc - last > *per_u / 2.0)
                uc -= *per_u;
            while (last - uc > *per_u / 2.0)
                uc += *per_u;
            w = (int) std::lround((uc - uv[0][0]) / *per_u);
        }
        bool interior_above = (w > 0) == same_sense;
        if (per_u && w == 0) {
            double mean = 0;
            for (const Uv &p : uv)
                mean += p[0];
            mean /= std::max((size_t) 1, uv.size());
            if (!u_ref) {
                u_ref = mean;
            } else {
                double shift = std::round((mean - *u_ref) / *per_u) * *per_u;
                if (shift != 0.0)
                    for (Uv &p : uv)
                        p[0] -= shift;
            }
        }
        loops_uv.push_back({std::move(uv), std::move(pts3), w, interior_above});
    }

    // ISO 10303-42 material-side selection for a single non-winding loop on a periodic surface. STEP
    // defines the face's material via same_sense + FACE_OUTER_BOUND.orientation relative to the
    // surface normal; the loop is INTERIOR-material iff sign(signed UV area) * sign(du x dv . normal)
    // * same_sense > 0, else the COMPLEMENT. run_tess2's TESS_WINDING_ODD always fills the interior,
    // so a complement-material face — e.g. a spherical gasket whose boundary encircles the hose
    // attachment — came out as the small hose-side cap with the gasket itself missing. Route those to
    // the complement tessellation. Measured on KR_6: exactly 2 faces flip (the gaskets); every other
    // quadric face (1925 of them) is interior-material and unchanged.
    if (auto per_u = surf.u_period(); per_u && loops_uv.size() == 1 && loops_uv[0].w == 0
                                      && loops_uv[0].uv.size() >= 3) {
        const auto &L = loops_uv[0].uv;
        double area = 0, uc = 0, vc = 0;
        for (size_t i = 0; i < L.size(); ++i) {
            const Uv &a = L[i], &b = L[(i + 1) % L.size()];
            area += a[0] * b[1] - b[0] * a[1];
            uc += a[0];
            vc += a[1];
        }
        uc /= (double) L.size();
        vc /= (double) L.size();
        const double h = 1e-4;
        double jdot = (surf.point(uc + h, vc) - surf.point(uc - h, vc))
                          .cross(surf.point(uc, vc + h) - surf.point(uc, vc - h))
                          .dot(surf.normal(uc, vc));
        const int A = area > 0 ? 1 : -1, J = jdot > 0 ? 1 : -1, S = same_sense ? 1 : -1;
        if (A * J * S < 0) {
            diag_set_path("iso_complement");
            std::vector<std::vector<Uv>> contour = {L};
            return tessellate_periodic_complement(surf, contour, *per_u, tp, same_sense, mesh)
                       ? nullptr
                       : "iso-complement tessellation failed";
        }
    }

    auto uv_slit = [&](const LoopUv &l) {
        if (l.w != 0)
            return false;
        double area = 0, peri = 0;
        for (size_t i = 0; i < l.uv.size(); ++i) {
            const Uv &a = l.uv[i];
            const Uv &b = l.uv[(i + 1) % l.uv.size()];
            area += a[0] * b[1] - b[0] * a[1];
            peri += std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
        }
        return std::abs(area) * 0.5 < 1e-9 * peri * peri;
    };
    bool all_slit = !loops_uv.empty();
    for (const LoopUv &l : loops_uv)
        if (!uv_slit(l))
            all_slit = false;
    // V-winding band: a doubly-periodic surface (torus/tube) whose boundary sweeps a full minor
    // circle in v while spanning only a short arc in u. The per-point unwrap keeps consecutive v
    // within half a period, so a full wrap DOESN'T fold back — v drifts monotonically past per_v and
    // the UV loop collapses to a near-degenerate vertical sliver. That sliver has ~zero polygon area,
    // so it's misclassified as a slit and tessellated as the WHOLE torus (measured: 118 such faces
    // produced 57x their true surface area on KR_6). The u-winding case is already handled above;
    // this is its v analog. Tessellate the real geometry directly: a grid band over the actual small
    // u-arc x one full minor period.
    if (per_v && !loops_uv.empty()) {
        double umn = INF, umx = -INF, vmn = INF, vmx = -INF;
        for (const auto &lp : loops_uv)
            for (const Uv &q : lp.uv) {
                umn = std::min(umn, q[0]);
                umx = std::max(umx, q[0]);
                vmn = std::min(vmn, q[1]);
                vmx = std::max(vmx, q[1]);
            }
        // dv > ~one full period means the boundary wrapped the minor circle and the v-unwrap drifted
        // (a valid single-valued torus face spans at most one period; a full-circle cross-section
        // reaches ~per_v plus one discretization step of closing overlap, so 1.1x is the safe line —
        // it catches the drifters, which start ~1.12 periods, and leaves genuine full circles alone).
        if (vmx - vmn > 1.1 * *per_v && umx - umn < *per_v) {
            diag_set_path("v_wind_band");
            return tessellate_uv_grid(surf, umn, umx, vmn, vmn + *per_v, tp, same_sense, mesh)
                       ? nullptr
                       : "v-winding band tessellation failed";
        }
    }
    if (all_slit) {
        diag_set_path("unbounded_slit");
        return tessellate_unbounded(surf, tp, same_sense, mesh) ? nullptr : "slit/full-surface tessellation failed";
    }

    if (loops_uv.size() == 1 && bspline_has_v_pole(surf)) {
        std::vector<std::vector<Uv>> contour = {loops_uv[0].uv};
        Rect r = full_wrap_bspline(surf, contour);
        if (r.ok && tessellate_uv_grid(surf, r.u0, r.u1, r.v0, r.v1, tp, same_sense, mesh)) {
            diag_set_path("grid_vpole_wrap");
            return nullptr;
        }
    }

    bool any_wind = false;
    for (const LoopUv &l : loops_uv)
        if (l.w != 0)
            any_wind = true;
    if (any_wind) {
        auto cp = mesh.checkpoint();
        if (tessellate_periodic_band(surf, loops_uv, tp, same_sense, mesh)) {
            diag_set_path("periodic_band");
            return nullptr;
        }
        mesh.rollback(cp);
        diag_set_path("periodic_winding");
        return tessellate_periodic_winding(surf, loops_uv, tp, same_sense, mesh)
                   ? nullptr
                   : "periodic-band (wrap-around) tessellation failed";
    }

    std::vector<std::vector<Uv>> contours;
    std::vector<std::vector<Vec3>> contours_p3; // shared-edge pins parallel to contours
    contours.reserve(loops_uv.size());
    contours_p3.reserve(loops_uv.size());
    for (auto &l : loops_uv) {
        contours.push_back(std::move(l.uv));
        contours_p3.push_back(std::move(l.pts3));
    }
    // Pins are data-guarded: nullptr => the emit path runs the identical code it always did.
    const std::vector<std::vector<Vec3>> *pins =
        (tp.track == TessTrack::Libtess2 && tp.libtess2.pin_boundary) ? &contours_p3 : nullptr;
    const std::vector<std::vector<Vec3>> *cdt_pins = tp.cdt.pin_boundary ? &contours_p3 : nullptr;

    {
        Rect r = full_wrap_bspline(surf, contours);
        if (r.ok && tessellate_uv_grid(surf, r.u0, r.u1, r.v0, r.v1, tp, same_sense, mesh)) {
            diag_set_path("grid_full_wrap");
            return nullptr;
        }
    }
    if (per_u && complement_interior(contours)) {
        diag_set_path("periodic_complement");
        return tessellate_periodic_complement(surf, contours, *per_u, tp, same_sense, mesh)
                   ? nullptr
                   : "periodic-complement tessellation failed";
    }
    {
        Rect r = full_domain_bspline(surf, contours);
        if (r.ok && poly_self_intersects(contours[0]) &&
            tessellate_uv_grid(surf, r.u0, r.u1, r.v0, r.v1, tp, same_sense, mesh)) {
            diag_set_path("grid_full_domain");
            return nullptr;
        }
    }
    // The CDT track has NO grid fast path, by design — that path tessellates the UV bbox, never sees
    // the trim loop, and so cannot pin, which makes it the only remaining crack source. Here the
    // grid becomes interior Steiner points inside a boundary-constrained triangulation instead, so
    // near-full patches and trimmed faces are ONE path and every boundary vertex is a loop vertex.
    // This is what OCC and truck do.
    if (tp.track == TessTrack::Cdt) {
        diag_set_path("emit_cdt_region");
        return emit_cdt_region(surf, contours, tp, same_sense, mesh, cdt_pins)
                   ? nullptr
                   : "CDT tessellation produced no triangles";
    }
    // grid_via_emit: skip the UV-bbox grid so this face goes through the pinned emit path below.
    // full_patch_rect's gate is area_ratio >= 0.995, i.e. the trim loop IS the rectangle, so emit
    // covers the same region -- but with a boundary it can pin.
    if (!(tp.track == TessTrack::Libtess2 && tp.libtess2.grid_via_emit) &&
        tessellate_full_patch(surf, contours, tp, same_sense, mesh, pins ? &contours_p3 : nullptr)) {
        diag_set_path("grid_full_patch");
        return nullptr;
    }
    diag_set_path("emit_uv_region");
    return emit_uv_region(surf, contours, tp, same_sense, mesh, pins) ? nullptr
                                                                      : "UV tessellation produced no triangles";
}

// ---- boundary building from neutral topology (build_loops3d / loop_polyline) --------
std::vector<Loop3> build_loops3d(const FaceSurfaceN &face, const TessParams &tp) {
    std::vector<Loop3> loops;
    auto _t0 = EDGEPROF ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    struct ProfScope {
        bool on;
        std::chrono::steady_clock::time_point t0;
        ~ProfScope() {
            if (on)
                g_loops3d_ns.fetch_add((uint64_t) std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now() - t0)
                                           .count(),
                                       std::memory_order_relaxed);
        }
    } _prof{EDGEPROF, _t0};
    for (const FaceBoundN &b : face.bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> lp = b.loop->discretize(tp.deflection, tp.max_angle);
        if (lp.size() >= 3) {
            if (!b.orientation)
                std::reverse(lp.begin(), lp.end());
            loops.push_back({std::move(lp)});
        }
    }
    return loops;
}

bool boundary_is_degenerate(const FaceSurfaceN &face, const TessParams &tp) {
    std::vector<Vec3> pts;
    double area = 0;
    for (const FaceBoundN &b : face.bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> lp = b.loop->discretize(tp.deflection, tp.max_angle);
        if (lp.size() >= 3) {
            Vec3 c{0, 0, 0};
            for (const Vec3 &p : lp)
                c = c + p;
            c = c * (1.0 / lp.size());
            Vec3 nrm{0, 0, 0};
            for (size_t i = 0; i < lp.size(); ++i)
                nrm = nrm + (lp[i] - c).cross(lp[(i + 1) % lp.size()] - c);
            area += nrm.norm() * 0.5;
        }
        for (const Vec3 &p : lp)
            pts.push_back(p);
    }
    if (pts.empty())
        return false;
    Vec3 lo = pts[0], hi = pts[0];
    for (const Vec3 &p : pts) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    double diag = (hi - lo).norm();
    if (diag > 0.0 && area < 1e-6 * diag * diag)
        return true;
    double mx = 1.0;
    for (const Vec3 &p : pts)
        mx = std::max(mx, p.norm());
    double eps = 1e-7 * mx;
    std::vector<Vec3> distinct;
    for (const Vec3 &p : pts) {
        bool dup = false;
        for (const Vec3 &q : distinct)
            if ((q - p).norm() < eps) {
                dup = true;
                break;
            }
        if (!dup)
            distinct.push_back(p);
    }
    return distinct.size() < 3;
}

} // namespace

namespace {
bool tessellate_face_impl(const FaceSurfaceN &face, const TessParams &tp, TessMesh &outm) {
    // NOTE: do NOT bail on empty bounds — a closed quadric (full sphere/torus) or B-spline patch
    // with no FACE_BOUND tessellates via tessellate_unbounded below (tessellate_face: it
    // never short-circuits on empty bounds; empty loops3d -> tessellate_unbounded).
    if (!face.surface)
        return false;
    Mesh mesh(outm);
    const Surface &surf = *face.surface;
    bool same_sense = face.same_sense;

    std::vector<Loop3> loops3d = build_loops3d(face, tp);
    if (loops3d.empty()) {
        if (tessellate_unbounded(surf, tp, same_sense, mesh))
            return true;
        if (boundary_is_degenerate(face, tp))
            return false; // degenerate, not a failure
        return false;
    }

    // Placeholder-plane face (plain IfcFace / explicit face-set polygon): the declared surface is the
    // z=0 identity plane, so fit the REAL plane from the 3D loop up front. Without this the 3D poly
    // projects validly onto z=0 (face_to_mesh succeeds, so the on-failure re-fit below never runs) and
    // the whole face-set collapses flat.
    std::shared_ptr<PlaneSurface> refit;
    if (face.fit_plane_from_loop) {
        refit = fit_plane(loops3d);
        if (refit)
            same_sense = true; // fitted normal follows the loop winding
    }
    const Surface &use_surf = refit ? *refit : surf;

    auto cp = mesh.checkpoint();
    const char *reason = face_to_mesh(use_surf, loops3d, tp, same_sense, mesh);
    if (!reason)
        return true;

    // recovery: a tess2 failure on a thin curved face is usually a self-intersecting boundary
    // from coarse arc discretization — re-discretize much finer and retry.
    bool reason_tess2 = std::string(reason).find("tess2") != std::string::npos ||
                        std::string(reason).find("UV tessellation") != std::string::npos;
    if (reason_tess2) {
        mesh.rollback(cp);
        for (double div : {8.0, 64.0}) {
            TessParams fine;
            fine.deflection = std::max(tp.deflection / div, 1e-4);
            fine.max_angle = std::max(tp.max_angle / std::sqrt(div), 0.02);
            std::vector<Loop3> fl = build_loops3d(face, fine);
            if (fl.size() != loops3d.size())
                continue;
            auto cp2 = mesh.checkpoint();
            if (!face_to_mesh(use_surf, fl, fine, same_sense, mesh))
                return true;
            mesh.rollback(cp2);
        }
    }
    // a planar face whose declared surface trips the trimmer: re-fit a plane to the loop points
    if (surface_is_planar(surf)) {
        mesh.rollback(cp);
        if (auto fitted = fit_plane(loops3d)) {
            if (!face_to_mesh(*fitted, loops3d, tp, true, mesh))
                return true;
        }
        mesh.rollback(cp);
    }
    if (boundary_is_degenerate(face, tp))
        return false;
    return false;
}
} // namespace

// Per-conversion dropped/total face counters (see ngeom_tessellate.h). Atomic so the face + root pools
// increment safely; reset once single-threaded before a conversion, read after the pools join.
static std::atomic<std::uint64_t> g_dropped_faces{0};
static std::atomic<std::uint64_t> g_total_faces{0};
std::uint64_t tess_dropped_faces() {
    return g_dropped_faces.load(std::memory_order_relaxed);
}
std::uint64_t tess_total_faces() {
    return g_total_faces.load(std::memory_order_relaxed);
}
void reset_tess_face_stats() {
    g_dropped_faces.store(0, std::memory_order_relaxed);
    g_total_faces.store(0, std::memory_order_relaxed);
}

bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &outm) {
    const size_t i0 = outm.indices.size();
    bool ok;
    if (!FDBG && !TESSDBG) {
        ok = tessellate_face_impl(face, tp, outm);
    } else {
        size_t bpts = 0;
        for (const FaceBoundN &b : face.bounds)
            if (b.loop)
                bpts += b.loop->discretize(tp.deflection, tp.max_angle).size();
        if (TESSDBG && face.surface) {
            std::fprintf(stderr, "TESSDBG FACE surf=%s", surf_kind(*face.surface));
            log_surf_params(*face.surface);
            std::fprintf(stderr, " bounds=%zu bpts=%zu same_sense=%d\n", face.bounds.size(), bpts,
                         (int) face.same_sense);
        }
        ok = tessellate_face_impl(face, tp, outm);
        if (FDBG)
            std::fprintf(stderr, "FDBG FACE %-11s bounds=%zu bpts=%zu tris=%zu ok=%d\n",
                         face.surface ? surf_kind(*face.surface) : "?", face.bounds.size(), bpts,
                         (outm.indices.size() - i0) / 3, (int) ok);
        if (TESSDBG)
            std::fprintf(stderr, "TESSDBG FACE-DONE tris=%zu ok=%d\n", (outm.indices.size() - i0) / 3, (int) ok);
    }
    // Health accounting (always on): a face carrying a real trim boundary that yields NO triangles is
    // UV-inversion residual row for this face (ADA_TESS_DIAG only; no-op otherwise). Emitted after
    // the impl so `path` names the branch actually taken, including any retry tier.
    if (DIAG && face.surface) {
        static std::atomic<uint32_t> g_diag_seq{0};
        diag_flush_face(*face.surface, face.src_id, g_diag_seq.fetch_add(1, std::memory_order_relaxed),
                        tls_model_scale());
    }
    // silently dropped geometry — count it. Uses the emitted-triangle delta, not `ok`, since some
    // failures still return true with an empty result.
    g_total_faces.fetch_add(1, std::memory_order_relaxed);
    if (outm.indices.size() == i0 && !face.bounds.empty())
        g_dropped_faces.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

// ---- analytic solid -> boundary mesh (OCC-free; gives the libtess2 path solid support) -------
namespace {
constexpr double TWO_PI = 6.283185307179586;

// flat-shaded triangle with an outward normal derived from the winding; skips degenerates.
void emit_tri(Mesh &mesh, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    Vec3 nrm = (b - a).cross(c - a);
    double l = nrm.norm();
    if (l < 1e-18)
        return;
    nrm = nrm * (1.0 / l);
    uint32_t base = (uint32_t) (mesh.m.positions.size() / 3);
    mesh.push_vertex(a, nrm);
    mesh.push_vertex(b, nrm);
    mesh.push_vertex(c, nrm);
    mesh.push_index(base);
    mesh.push_index(base + 1);
    mesh.push_index(base + 2);
}

// Rodrigues rotation of p about the line (o, unit k) by angle th.
Vec3 rotate_about(const Vec3 &p, const Vec3 &o, const Vec3 &k, double th) {
    Vec3 v = p - o;
    double c = std::cos(th), s = std::sin(th);
    return o + v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c));
}

// ExtrudedAreaSolid -> two caps (profile triangulated in local XY via libtess2, with holes) +
// a quad side band per boundary loop. Profile lives in the local XY plane (z=0); the position
// frame F places the swept solid; d is the local sweep vector (direction * depth).
void tessellate_extrusion(const ExtrusionN &ex, const TessParams &tp, Mesh &mesh) {
    if (!ex.profile)
        return;
    const Frame &F = ex.frame;
    const Vec3 d = ex.direction * ex.depth;

    const auto cp0 = mesh.checkpoint(); // this extrusion's triangle range starts here

    std::vector<std::vector<Uv>> loops_uv;
    for (const FaceBoundN &b : ex.profile->bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> ring = b.loop->discretize(tp.deflection, tp.max_angle);
        std::vector<Uv> uv;
        uv.reserve(ring.size());
        for (const Vec3 &p : ring)
            uv.push_back({p.x, p.y});
        if (uv.size() >= 3)
            loops_uv.push_back(std::move(uv));
    }
    Tess2Out cap = run_tess2(loops_uv, 1.0, 1.0);
    if (cap.ok) {
        for (const Tri &t : cap.tris) {
            const Uv &u0 = cap.verts[t[0]], &u1 = cap.verts[t[1]], &u2 = cap.verts[t[2]];
            // bottom cap (z=0), reversed winding so the normal faces away from the sweep
            emit_tri(mesh, F.to_world(u0[0], u0[1], 0), F.to_world(u2[0], u2[1], 0), F.to_world(u1[0], u1[1], 0));
            // top cap (translated by d)
            emit_tri(mesh, F.to_world(u0[0] + d.x, u0[1] + d.y, d.z), F.to_world(u1[0] + d.x, u1[1] + d.y, d.z),
                     F.to_world(u2[0] + d.x, u2[1] + d.y, d.z));
        }
    }
    for (const FaceBoundN &b : ex.profile->bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> ring = b.loop->discretize(tp.deflection, tp.max_angle);
        size_t n = ring.size();
        if (n > 1 && (ring.front() - ring.back()).norm() < 1e-12) {
            ring.pop_back();
            --n;
        }
        for (size_t i = 0; i < n; ++i) {
            const Vec3 &p0 = ring[i], &p1 = ring[(i + 1) % n];
            Vec3 b0 = F.to_world(p0.x, p0.y, p0.z), b1 = F.to_world(p1.x, p1.y, p1.z);
            Vec3 t0 = F.to_world(p0.x + d.x, p0.y + d.y, p0.z + d.z);
            Vec3 t1 = F.to_world(p1.x + d.x, p1.y + d.y, p1.z + d.z);
            emit_tri(mesh, b0, b1, t1);
            emit_tri(mesh, b0, t1, t0);
        }
    }

    // Outward-facing normals: emit_tri derives each normal from its winding, so if the profile
    // loop's discretized orientation is CW the whole extrusion comes out inside-out (negative
    // signed volume) and shades dark. Flip every triangle in this extrusion's range when that
    // happens — same guard as tessellate_revolve, robust to the incoming loop orientation.
    {
        auto &pos = mesh.m.positions;
        auto &nrm = mesh.m.normals;
        const size_t vbeg = cp0[0] / 3, vend = pos.size() / 3;
        auto Pv = [&](size_t k) -> Vec3 { return {pos[3 * k], pos[3 * k + 1], pos[3 * k + 2]}; };
        double sv = 0.0;
        for (size_t k = vbeg; k + 2 < vend; k += 3)
            sv += Pv(k).dot(Pv(k + 1).cross(Pv(k + 2)));
        if (sv < 0.0) {
            for (size_t k = vbeg; k + 2 < vend; k += 3) {
                for (int c = 0; c < 3; ++c)
                    std::swap(pos[3 * (k + 1) + c], pos[3 * (k + 2) + c]);
                for (int c = 0; c < 9; ++c)
                    nrm[3 * k + c] = -nrm[3 * k + c];
            }
        }
    }
}

// RevolvedAreaSolid -> sweep the profile boundary ring around (axis_origin, axis_dir) by angle.
// Points on the axis trace nothing (their quads collapse), so a profile touching the axis (e.g.
// a cone's apex/centre) closes correctly. Full revolutions need no end caps.
void tessellate_revolve(const RevolveN &rv, const TessParams &tp, Mesh &mesh) {
    if (!rv.profile)
        return;
    const Frame &F = rv.frame;
    const Vec3 axo = rv.axis_origin;
    Vec3 axd = rv.axis_dir;
    if (axd.norm() < 1e-12)
        axd = {0, 0, 1};
    axd = axd.normalized();
    const double ang = (rv.angle != 0.0) ? rv.angle : TWO_PI;

    std::vector<Vec3> ring;
    std::vector<std::vector<Uv>> loops_uv; // all profile loops (outer + holes) for the end caps
    for (const FaceBoundN &b : rv.profile->bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> r = b.loop->discretize(tp.deflection, tp.max_angle);
        std::vector<Uv> uv;
        uv.reserve(r.size());
        for (const Vec3 &p : r)
            uv.push_back({p.x, p.y});
        if (uv.size() >= 3)
            loops_uv.push_back(std::move(uv));
        if (ring.empty())
            ring = std::move(r); // outer loop drives the side walls
    }
    if (ring.size() > 1 && (ring.front() - ring.back()).norm() < 1e-12)
        ring.pop_back();
    const size_t n = ring.size();
    if (n < 2)
        return;

    double rmax = 0;
    for (const Vec3 &p : ring) {
        Vec3 rel = p - axo;
        rmax = std::max(rmax, (rel - axd * axd.dot(rel)).norm());
    }
    double defl = tp.deflection > 0 ? tp.deflection : std::max(rmax * 0.01, 1e-4);
    double step = angle_step(rmax, defl, tp.max_angle);
    int nseg = std::max(3, (int) std::ceil(ang / std::max(step, 1e-6)));

    std::vector<std::vector<Vec3>> R(nseg + 1, std::vector<Vec3>(n));
    for (int j = 0; j <= nseg; ++j) {
        double th = ang * (double) j / (double) nseg;
        for (size_t i = 0; i < n; ++i) {
            Vec3 pr = rotate_about(ring[i], axo, axd, th);
            R[j][i] = F.to_world(pr.x, pr.y, pr.z);
        }
    }
    const auto cp0 = mesh.checkpoint(); // this revolve's triangle range starts here
    for (int j = 0; j < nseg; ++j) {
        const auto &A = R[j];
        const auto &B = R[j + 1];
        for (size_t i = 0; i < n; ++i) {
            size_t i2 = (i + 1) % n;
            emit_tri(mesh, A[i], A[i2], B[i2]);
            emit_tri(mesh, A[i], B[i2], B[i]);
        }
    }

    // Partial revolution (angle < 2pi): cap the two open ends with the triangulated
    // profile at th=0 and th=ang, so a swept beam is a closed solid instead of an open
    // tube. A full turn (cylinder/cone) closes on itself and needs no caps. Mirrors
    // tessellate_sweep's start/end-cap handling (start reversed so its normal faces out).
    if (ang < TWO_PI - 1e-6 && !loops_uv.empty()) {
        auto Pr = [&](double th, double u, double v) -> Vec3 {
            Vec3 pr = rotate_about({u, v, 0.0}, axo, axd, th);
            return F.to_world(pr.x, pr.y, pr.z);
        };
        Tess2Out cap = run_tess2(loops_uv, 1.0, 1.0);
        if (cap.ok) {
            for (const Tri &t : cap.tris) {
                const Uv &u0 = cap.verts[t[0]], &u1 = cap.verts[t[1]], &u2 = cap.verts[t[2]];
                emit_tri(mesh, Pr(0.0, u0[0], u0[1]), Pr(0.0, u2[0], u2[1]), Pr(0.0, u1[0], u1[1]));
                emit_tri(mesh, Pr(ang, u0[0], u0[1]), Pr(ang, u1[0], u1[1]), Pr(ang, u2[0], u2[1]));
            }
        }
    }

    // Outward-facing normals: emit_tri derives each normal from its winding, so if the
    // profile loop was CW the whole revolve comes out inside-out (negative signed volume)
    // and shades dark. Flip every triangle in this revolve's range when that happens —
    // handles walls + caps together regardless of the incoming loop orientation.
    {
        auto &pos = mesh.m.positions;
        auto &nrm = mesh.m.normals;
        const size_t vbeg = cp0[0] / 3, vend = pos.size() / 3;
        auto Pv = [&](size_t k) -> Vec3 { return {pos[3 * k], pos[3 * k + 1], pos[3 * k + 2]}; };
        double sv = 0.0;
        for (size_t k = vbeg; k + 2 < vend; k += 3)
            sv += Pv(k).dot(Pv(k + 1).cross(Pv(k + 2)));
        if (sv < 0.0) {
            for (size_t k = vbeg; k + 2 < vend; k += 3) {
                for (int c = 0; c < 3; ++c)
                    std::swap(pos[3 * (k + 1) + c], pos[3 * (k + 2) + c]);
                for (int c = 0; c < 9; ++c)
                    nrm[3 * k + c] = -nrm[3 * k + c];
            }
        }
    }
}

// FixedReferenceSweptAreaSolid -> sweep the profile along a precomputed field of per-station frames
// (origin + dir_x/dir_y). Side walls connect consecutive profile rings; the first/last stations get
// libtess2 end caps (profile triangulated in local UV, with holes). The directrix analytics (Fresnel
// clothoid + vertical gradient + fixed-reference frame) are evaluated producer-side; no OCC here.
void tessellate_sweep(const SweepN &sw, const TessParams &tp, Mesh &mesh) {
    const size_t ns = sw.origin.size();
    if (!sw.profile || ns < 2)
        return;
    const Frame &F = sw.frame;

    // outer boundary ring (local UV) for the walls + all loops for the cap triangulation
    std::vector<Vec3> ring;
    std::vector<std::vector<Uv>> loops_uv;
    for (const FaceBoundN &b : sw.profile->bounds) {
        if (!b.loop)
            continue;
        std::vector<Vec3> r = b.loop->discretize(tp.deflection, tp.max_angle);
        std::vector<Uv> uv;
        uv.reserve(r.size());
        for (const Vec3 &p : r)
            uv.push_back({p.x, p.y});
        if (uv.size() >= 3)
            loops_uv.push_back(std::move(uv));
        if (ring.empty())
            ring = std::move(r); // outer loop drives the walls
    }
    if (ring.size() > 1 && (ring.front() - ring.back()).norm() < 1e-12)
        ring.pop_back();
    const size_t n = ring.size();
    if (n < 2)
        return;

    // profile (u,v) at station j -> world point
    auto P = [&](size_t j, double u, double v) -> Vec3 {
        Vec3 local = sw.origin[j] + sw.dir_x[j] * u + sw.dir_y[j] * v;
        return F.to_world(local.x, local.y, local.z);
    };

    for (size_t j = 0; j + 1 < ns; ++j) {
        for (size_t i = 0; i < n; ++i) {
            size_t i2 = (i + 1) % n;
            Vec3 a = P(j, ring[i].x, ring[i].y), b = P(j, ring[i2].x, ring[i2].y);
            Vec3 c = P(j + 1, ring[i2].x, ring[i2].y), d = P(j + 1, ring[i].x, ring[i].y);
            emit_tri(mesh, a, b, c);
            emit_tri(mesh, a, c, d);
        }
    }

    Tess2Out cap = run_tess2(loops_uv, 1.0, 1.0);
    if (cap.ok) {
        for (const Tri &t : cap.tris) {
            const Uv &u0 = cap.verts[t[0]], &u1 = cap.verts[t[1]], &u2 = cap.verts[t[2]];
            // start cap (station 0) reversed so its normal faces backward along the sweep
            emit_tri(mesh, P(0, u0[0], u0[1]), P(0, u2[0], u2[1]), P(0, u1[0], u1[1]));
            // end cap (last station)
            emit_tri(mesh, P(ns - 1, u0[0], u0[1]), P(ns - 1, u1[0], u1[1]), P(ns - 1, u2[0], u2[1]));
        }
    }
}

// Sphere primitive -> a UV (lat/long) sphere. Segment counts follow the deflection (angle_step),
// so it honours the requested tolerance; pole quads collapse (emit_tri skips degenerates).
void tessellate_sphere(const SphereN &sp, const TessParams &tp, Mesh &mesh) {
    const Vec3 c = sp.frame.o;
    const double r = sp.radius;
    if (r <= 0)
        return;
    const double PI = TWO_PI * 0.5;
    double defl = tp.deflection > 0 ? tp.deflection : std::max(r * 0.01, 1e-4);
    double step = std::max(angle_step(r, defl, tp.max_angle), 1e-6);
    int nlon = std::max(6, (int) std::ceil(TWO_PI / step));
    int nlat = std::max(3, (int) std::ceil(PI / step));
    auto P = [&](int i, int j) -> Vec3 {
        double lon = TWO_PI * (double) i / nlon, lat = -PI * 0.5 + PI * (double) j / nlat;
        double cl = std::cos(lat);
        return {c.x + r * cl * std::cos(lon), c.y + r * cl * std::sin(lon), c.z + r * std::sin(lat)};
    };
    for (int j = 0; j < nlat; ++j)
        for (int i = 0; i < nlon; ++i) {
            Vec3 a = P(i, j), b = P(i + 1, j), cc = P(i + 1, j + 1), d = P(i, j + 1);
            emit_tri(mesh, a, b, cc);
            emit_tri(mesh, a, cc, d);
        }
}

// Tessellate one boolean operand into its own world-space mesh (recursive for nested booleans).
TessMesh tessellate_solid_item(const SolidItemN &it, const TessParams &tp);

TessMesh tessellate_boolean_item(const BooleanN &bn, const TessParams &tp) {
    TessMesh a = tessellate_solid_item(bn.a, tp);
    TessMesh b = tessellate_solid_item(bn.b, tp);
    TessMesh out;
    mesh_boolean(bn.op, a, b, out); // Manifold (native) / no-op stub (wasm)
    return out;
}

TessMesh tessellate_solid_item(const SolidItemN &it, const TessParams &tp) {
    if (it.boolean)
        return tessellate_boolean_item(*it.boolean, tp);
    TessMesh m;
    if (it.extrusion) {
        Mesh mesh(m);
        tessellate_extrusion(*it.extrusion, tp, mesh);
    } else if (it.revolve) {
        Mesh mesh(m);
        tessellate_revolve(*it.revolve, tp, mesh);
    } else if (it.sweep) {
        Mesh mesh(m);
        tessellate_sweep(*it.sweep, tp, mesh);
    } else {
        for (const auto &f : it.faces)
            if (f)
                tessellate_face(*f, tp, m);
    }
    return m;
}

// Concatenate src into dst, offsetting src's indices by dst's current vertex count.
void append_mesh(TessMesh &dst, const TessMesh &src) {
    uint32_t base = (uint32_t) (dst.positions.size() / 3);
    dst.positions.insert(dst.positions.end(), src.positions.begin(), src.positions.end());
    dst.normals.insert(dst.normals.end(), src.normals.begin(), src.normals.end());
    for (uint32_t i : src.indices)
        dst.indices.push_back(base + i);
}
} // namespace

// Tessellate ONE root's geometry into `out` (no group bookkeeping — the caller records ranges).
static void tessellate_one_root(const NgeomRoot &root, const TessParams &tp, TessMesh &out) {
    // Publish the model scale for this worker thread so the curvature functions (angle_step ->
    // adaptive_max_angle) can relax density on small features without a signature change. Constant
    // for the whole call, so setting it per root (idempotent) is safe under the root/face pools.
    tls_model_scale() = tp.model_scale;
    if (root.extrusion) {
        Mesh m(out);
        tessellate_extrusion(*root.extrusion, tp, m);
    } else if (root.revolve) {
        Mesh m(out);
        tessellate_revolve(*root.revolve, tp, m);
    } else if (root.sweep) {
        Mesh m(out);
        tessellate_sweep(*root.sweep, tp, m);
    } else if (root.boolean) {
        // CSG via Manifold (no-op on wasm until Manifold is wired there).
        append_mesh(out, tessellate_boolean_item(*root.boolean, tp));
    } else if (root.sphere) {
        Mesh m(out);
        tessellate_sphere(*root.sphere, tp, m);
    } else if (!root.polylines.empty()) {
        // Curve-only body -> GL_LINES: emit each polyline's points as a line strip (index pairs).
        out.mesh_type = MeshType::LINES;
        for (const auto &line : root.polylines) {
            if (line.size() < 2)
                continue;
            uint32_t base = (uint32_t) (out.positions.size() / 3);
            for (const Vec3 &p : line) {
                out.positions.push_back((float) p.x);
                out.positions.push_back((float) p.y);
                out.positions.push_back((float) p.z);
                out.normals.push_back(0.0f);
                out.normals.push_back(0.0f);
                out.normals.push_back(1.0f);
            }
            for (uint32_t i = 0; i + 1 < line.size(); ++i) {
                out.indices.push_back(base + i);
                out.indices.push_back(base + i + 1);
            }
        }
    } else {
        if (FDBG) {
            size_t nnull = 0;
            for (const auto &f : root.faces)
                if (!f)
                    ++nnull;
            std::fprintf(stderr, "FDBG ROOT id=%s faces=%zu null=%zu\n", root.id.c_str(), root.faces.size(), nnull);
        }
        // Face-parallel path for a single HUGE face-set root (tp.threads > 1): one
        // 61k-face solid otherwise pins a lone worker for the whole conversion tail
        // (469826: 54 s on one thread while the other three idle after 20 s). Faces
        // are independent (each tessellate_face builds its own libtess2 tessellator);
        // per-face local meshes merged in face order keep the output identical to
        // the serial loop. The 64-face floor keeps small solids on the cheap path.
        // DIAG forces the serial loop: the per-face residual accumulator is thread_local, so a
        // face-parallel run would scatter rows across threads and lose the face-order sequence.
        if (tp.threads > 1 && root.faces.size() >= 64 && !tp.capture_face_ranges && !DIAG) {
            const size_t n = root.faces.size();
            TessParams tpl = tp;
            tpl.threads = 1; // no nested pools inside a face
            // Batch the parallel face tessellation + in-order merge so the resident per-face local
            // meshes never exceed one batch (a 61k-face locals[] is otherwise a full second copy of
            // the solid soup ~0.5 GB — the 469826 obj/stl OOM). Faces within a batch run in parallel;
            // batches (and faces within them) merge in face order, so the output is byte-identical to
            // the old all-at-once path. batch==0 or >=n => single batch (the original behaviour).
            size_t batch = tess_face_merge_batch();
            if (batch == 0 || batch > n)
                batch = n;
            for (size_t b0 = 0; b0 < n; b0 += batch) {
                const size_t b1 = std::min(n, b0 + batch);
                std::vector<TessMesh> locals(b1 - b0);
                std::atomic<size_t> next{b0};
                unsigned nt = std::min<unsigned>((unsigned) tp.threads, (unsigned) (b1 - b0));
                auto face_worker = [&]() {
                    // tls_model_scale is thread_local, set (line above) ONLY on the thread that runs
                    // tessellate_one_root. The SPAWNED face workers start at 0.0 => adaptive density
                    // OFF => full fine tessellation for whatever share of the faces they grab. So a
                    // huge solid's triangle count SCALED WITH THREAD COUNT and varied run-to-run with
                    // scheduling (measured 469826: 12.44M serial -> 14.6M at 4 threads -> 15.85M at 8;
                    // ~+17% at the 3-thread prod pod). Publishing the model scale on THIS worker makes
                    // every face use the adaptive density => tri count is thread-invariant + deterministic.
                    tls_model_scale() = tp.model_scale;
                    for (size_t i = next.fetch_add(1); i < b1; i = next.fetch_add(1))
                        if (root.faces[i])
                            tessellate_face(*root.faces[i], tpl, locals[i - b0]);
                };
                std::vector<std::thread> pool;
                pool.reserve(nt - 1);
                for (unsigned t = 1; t < nt; ++t)
                    pool.emplace_back(face_worker);
                face_worker();
                for (std::thread &th : pool)
                    th.join();
                for (const TessMesh &lm : locals)
                    append_mesh(out, lm);
            } // locals freed here -> next batch's peak is one batch, not all n faces
            return;
        }
        for (size_t fi = 0; fi < root.faces.size(); ++fi) {
            const auto &face = root.faces[fi];
            if (!face)
                continue;
            if (tp.capture_face_ranges) {
                const uint32_t s = (uint32_t) out.indices.size();
                tessellate_face(*face, tp, out);
                const uint32_t c = (uint32_t) out.indices.size() - s;
                if (c > 0)
                    out.face_ranges.push_back({s, c, face->src_id, (uint32_t) fi});
            } else {
                tessellate_face(*face, tp, out);
            }
        }
    }
}

TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp) {
    TessMesh mesh;
    const auto &roots = doc.roots;
    auto _tess_t0 = std::chrono::steady_clock::now();
    struct DocProf {
        bool on;
        std::chrono::steady_clock::time_point t0;
        ~DocProf() {
            if (!on)
                return;
            g_tess_ns.fetch_add(
                (uint64_t) std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
                    .count(),
                std::memory_order_relaxed);
            uint64_t tess = g_tess_ns.load(), loops = g_loops3d_ns.load();
            std::fprintf(stderr,
                         "[EDGEPROF] tessellate_doc=%.3fs  build_loops3d=%.3fs (%.1f%% of tess)\n"
                         "[EDGEPROF] refine passes=%llu  marks=%llu  tris scanned=%llu  marks/pass=%.1f\n"
                         "[EDGEPROF] conforming grid: applied=%llu bailed=%llu  lines wanted=%llu applied=%llu\n"
                         "[EDGEPROF] cdt: faces=%llu fell back to libtess2=%llu\n",
                         tess / 1e9, loops / 1e9, tess ? 100.0 * (double) loops / (double) tess : 0.0,
                         (unsigned long long) g_passes.load(), (unsigned long long) g_marks.load(),
                         (unsigned long long) g_scanned.load(),
                         g_passes.load() ? (double) g_marks.load() / (double) g_passes.load() : 0.0,
                         (unsigned long long) g_conform_ok.load(), (unsigned long long) g_conform_bail.load(),
                         (unsigned long long) g_conform_want.load(), (unsigned long long) g_conform_lines.load(),
                         (unsigned long long) g_cdt_faces.load(), (unsigned long long) g_cdt_fallback_faces.load());
        }
    } _docprof{EDGEPROF, _tess_t0};
    // Opt-in parallelism (tp.threads>1): tessellate ROOTS across a thread pool into per-root local
    // buffers, merged in root order. Roots are independent (each tessellate_face allocates its own
    // libtess2 tessellator, no shared state); merging in order makes the output byte-identical to
    // serial. Per-root parallelism is the right grain for the merge-preview generate, which streams
    // one root PER PLATE (thousands of roots) so every plate is its own pickable BatchMesh group.
    // Default (threads=1) keeps the serial path — the STEP->GLB process pool stays serial per call.
    // Weld each ROOT independently (a shared index buffer + crease-angle smooth normals), then append
    // — per-root keeps group boundaries + picking intact (never merges verts across solids). Skips
    // LINES (curve bodies). tp.weld=false leaves the raw flat-shaded soup.
    auto weld_root = [&](TessMesh &rm) {
        if (tp.weld && rm.mesh_type == MeshType::TRIANGLES && !rm.indices.empty())
            weld_mesh(rm.positions, rm.indices, rm.normals);
    };
    unsigned want = tp.threads > 1 ? (unsigned) tp.threads : 1u;
    if (want <= 1 || roots.size() < 2) {
        for (const NgeomRoot &root : roots) {
            TessMesh rm;
            tessellate_one_root(root, tp, rm);
            weld_root(rm);
            uint32_t first = (uint32_t) mesh.indices.size();
            uint32_t vfirst = (uint32_t) (mesh.positions.size() / 3);
            mesh.mesh_type = rm.mesh_type; // single-root streaming: propagate LINES/TRIANGLES
            append_mesh(mesh, rm);
            for (auto fr : rm.face_ranges) { // re-base per-face ranges onto the merged index buffer
                fr.first_index += first;
                mesh.face_ranges.push_back(fr);
            }
            mesh.groups.push_back({root.id, first, (uint32_t) mesh.indices.size() - first, vfirst,
                                   (uint32_t) (mesh.positions.size() / 3) - vfirst});
        }
        return mesh;
    }
    std::vector<TessMesh> locals(roots.size());
    std::atomic<size_t> next{0};
    unsigned nthreads = std::min<unsigned>(want, (unsigned) roots.size());
    TessParams tp1 = tp;
    tp1.threads = 1; // roots already saturate the pool — no nested face pools per root
    std::vector<std::thread> pool;
    pool.reserve(nthreads);
    for (unsigned t = 0; t < nthreads; ++t)
        pool.emplace_back([&]() {
            for (size_t i = next.fetch_add(1); i < roots.size(); i = next.fetch_add(1)) {
                tessellate_one_root(roots[i], tp1, locals[i]);
                weld_root(locals[i]);
            }
        });
    for (std::thread &th : pool)
        th.join();
    for (size_t i = 0; i < roots.size(); ++i) {
        uint32_t first = (uint32_t) mesh.indices.size();
        uint32_t vfirst = (uint32_t) (mesh.positions.size() / 3);
        append_mesh(mesh, locals[i]);
        for (auto fr : locals[i].face_ranges) { // re-base per-face ranges onto the merged index buffer
            fr.first_index += first;
            mesh.face_ranges.push_back(fr);
        }
        mesh.groups.push_back({roots[i].id, first, (uint32_t) mesh.indices.size() - first, vfirst,
                               (uint32_t) (mesh.positions.size() / 3) - vfirst});
    }
    return mesh;
}

} // namespace adacpp::ngeom
