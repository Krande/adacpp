// Standalone test for the NGEOM decoder (no OCC). Hand-encodes buffers per the spec and
// checks the decoded neutral records. Doubles as a reference for the adapy serializer.
// Build: g++ -std=c++20 -I src/geom/neutral tests/ngeom/test_decode.cpp -o /tmp/ngeom_dec
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ngeom_decode.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)
static bool close(double a, double b, double tol = 1e-7) {
    return std::abs(a - b) <= tol;
}
static bool vclose(const Vec3 &a, const Vec3 &b, double tol = 1e-6) {
    return (a - b).norm() <= tol;
}

// Minimal NGEOM encoder mirroring the spec (the authoritative encoder is the adapy Python
// serializer; this is for tests + as a worked reference).
struct Enc {
    std::vector<uint8_t> recs;
    int count = 0;
    static void put_i32(std::vector<uint8_t> &b, int32_t v) {
        size_t o = b.size();
        b.resize(o + 4);
        std::memcpy(&b[o], &v, 4);
    }
    static void put_f64(std::vector<uint8_t> &b, double v) {
        size_t o = b.size();
        b.resize(o + 8);
        std::memcpy(&b[o], &v, 8);
    }
    static void put_v3(std::vector<uint8_t> &b, Vec3 p) {
        put_f64(b, p.x);
        put_f64(b, p.y);
        put_f64(b, p.z);
    }
    int record(int tag, const std::vector<uint8_t> &payload) {
        int idx = count++;
        put_i32(recs, tag);
        put_i32(recs, (int) payload.size());
        recs.insert(recs.end(), payload.begin(), payload.end());
        return idx;
    }
    std::vector<uint8_t> finish(const std::vector<std::pair<int, std::string>> &roots) {
        std::vector<uint8_t> out;
        const char *magic = "ADANGEOM";
        out.insert(out.end(), magic, magic + 8);
        put_i32(out, (int) NGEOM_VERSION);
        put_i32(out, count);
        out.insert(out.end(), recs.begin(), recs.end());
        put_i32(out, (int) roots.size());
        for (auto &[gi, id] : roots) {
            put_i32(out, gi);
            put_i32(out, (int) id.size());
            out.insert(out.end(), id.begin(), id.end());
        }
        return out;
    }
};

static void test_planar_triangle() {
    Enc e;
    std::vector<uint8_t> p;
    // PLACEMENT3 (origin, +Z, +X)
    p.clear();
    Enc::put_v3(p, {0, 0, 0});
    Enc::put_v3(p, {0, 0, 1});
    Enc::put_v3(p, {1, 0, 0});
    int pl = e.record(tag::PLACEMENT3, p);
    // PLANE
    p.clear();
    Enc::put_i32(p, pl);
    int plane = e.record(tag::PLANE, p);
    // 3 straight EDGE_CURVE (geometry ref = -1)
    Vec3 A{0, 0, 0}, B{1, 0, 0}, C{0, 1, 0};
    auto edge = [&](Vec3 s, Vec3 t) {
        p.clear();
        Enc::put_v3(p, s);
        Enc::put_v3(p, t);
        Enc::put_i32(p, -1); // null geometry
        Enc::put_i32(p, 1);  // same_sense
        return e.record(tag::EDGE_CURVE, p);
    };
    int eAB = edge(A, B), eBC = edge(B, C), eCA = edge(C, A);
    auto oedge = [&](int er) {
        p.clear();
        Enc::put_i32(p, er);
        Enc::put_i32(p, 1); // orientation
        Enc::put_i32(p, 0); // has_pcurve
        Enc::put_i32(p, 0); // has_params
        return e.record(tag::ORIENTED_EDGE, p);
    };
    int oAB = oedge(eAB), oBC = oedge(eBC), oCA = oedge(eCA);
    // EDGE_LOOP
    p.clear();
    Enc::put_i32(p, 3);
    Enc::put_i32(p, oAB);
    Enc::put_i32(p, oBC);
    Enc::put_i32(p, oCA);
    int loop = e.record(tag::EDGE_LOOP, p);
    // FACE_BOUND
    p.clear();
    Enc::put_i32(p, loop);
    Enc::put_i32(p, 1);
    int fb = e.record(tag::FACE_BOUND, p);
    // FACE_SURFACE
    p.clear();
    Enc::put_i32(p, plane);
    Enc::put_i32(p, 1); // same_sense
    Enc::put_i32(p, 1); // n_bounds
    Enc::put_i32(p, fb);
    int face = e.record(tag::FACE_SURFACE, p);

    auto buf = e.finish({{face, "tri"}});
    NgeomDoc doc = decode(buf.data(), buf.size());
    CHECK(doc.roots.size() == 1 && doc.roots[0].id == "tri", "triangle root id");
    CHECK(doc.roots[0].faces.size() == 1, "triangle one face");
    auto &f = *doc.roots[0].faces[0];
    CHECK(f.same_sense && f.bounds.size() == 1, "triangle face attrs");
    CHECK(vclose(f.surface->normal(0, 0), Vec3{0, 0, 1}), "triangle plane normal");
    auto pts = f.bounds[0].loop->discretize(0.01, 0.1);
    CHECK(pts.size() == 3, "triangle loop -> 3 points");
    if (pts.size() == 3) {
        CHECK(vclose(pts[0], A) && vclose(pts[1], B) && vclose(pts[2], C), "triangle loop verts");
    }
}

static void test_planar_disk_with_circle_edge() {
    Enc e;
    std::vector<uint8_t> p;
    p.clear();
    Enc::put_v3(p, {0, 0, 0});
    Enc::put_v3(p, {0, 0, 1});
    Enc::put_v3(p, {1, 0, 0});
    int pl = e.record(tag::PLACEMENT3, p);
    p.clear();
    Enc::put_i32(p, pl);
    int plane = e.record(tag::PLANE, p);
    // CIRCLE radius 2 in the same placement
    p.clear();
    Enc::put_i32(p, pl);
    Enc::put_f64(p, 2.0);
    int circ = e.record(tag::CIRCLE, p);
    // a full-circle edge: start==end at theta=0, geometry = circle
    p.clear();
    Enc::put_v3(p, {2, 0, 0});
    Enc::put_v3(p, {2, 0, 0});
    Enc::put_i32(p, circ);
    Enc::put_i32(p, 1);
    int ec = e.record(tag::EDGE_CURVE, p);
    // oriented edge with params [0, 2pi]
    p.clear();
    Enc::put_i32(p, ec);
    Enc::put_i32(p, 1); // orientation
    Enc::put_i32(p, 0); // has_pcurve
    Enc::put_i32(p, 1); // has_params
    Enc::put_f64(p, 0.0);
    Enc::put_f64(p, TWO_PI);
    int oe = e.record(tag::ORIENTED_EDGE, p);
    p.clear();
    Enc::put_i32(p, 1);
    Enc::put_i32(p, oe);
    int loop = e.record(tag::EDGE_LOOP, p);
    p.clear();
    Enc::put_i32(p, loop);
    Enc::put_i32(p, 1);
    int fb = e.record(tag::FACE_BOUND, p);
    p.clear();
    Enc::put_i32(p, plane);
    Enc::put_i32(p, 1);
    Enc::put_i32(p, 1);
    Enc::put_i32(p, fb);
    int face = e.record(tag::FACE_SURFACE, p);

    auto buf = e.finish({{face, "disk"}});
    NgeomDoc doc = decode(buf.data(), buf.size());
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "disk decoded");
    auto pts = doc.roots[0].faces[0]->bounds[0].loop->discretize(0.001, 0.05);
    CHECK(pts.size() >= 16, "disk circle edge discretized to many points");
    bool on = true;
    for (auto &q : pts)
        if (!close(std::sqrt(q.x * q.x + q.y * q.y), 2.0, 1e-3))
            on = false;
    CHECK(on, "disk loop points on radius 2");
}

static void test_unknown_tag_skipped() {
    // a forward-compat record with an unknown tag must be skipped via nbytes
    Enc e;
    std::vector<uint8_t> p;
    p.clear();
    Enc::put_v3(p, {0, 0, 0});
    Enc::put_v3(p, {0, 0, 1});
    Enc::put_v3(p, {1, 0, 0});
    int pl = e.record(tag::PLACEMENT3, p);
    // unknown tag 9999 with arbitrary payload
    p.clear();
    Enc::put_f64(p, 1.23);
    Enc::put_i32(p, 42);
    e.record(9999, p);
    p.clear();
    Enc::put_i32(p, pl);
    int plane = e.record(tag::PLANE, p);
    // minimal face with no bounds to keep it short
    p.clear();
    Enc::put_i32(p, plane);
    Enc::put_i32(p, 1);
    Enc::put_i32(p, 0);
    int face = e.record(tag::FACE_SURFACE, p);
    auto buf = e.finish({{face, "u"}});
    NgeomDoc doc = decode(buf.data(), buf.size());
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "unknown tag skipped, stream intact");
}

// Exercise the bulk readers (vec3s / f64s / i32s) on arrays large enough that the decoder takes
// the memcpy path: a B-spline surface (control grid + weights + knot multiplicities) and a
// many-point POLY_LOOP. Decoded values must match the hand-encoded ones byte-for-byte.
static void test_bulk_arrays_roundtrip() {
    Enc e;
    std::vector<uint8_t> p;

    // a 4x6 = 24-point control grid (row-major) + per-cp weights
    const int nu = 4, nv = 6, ncp = nu * nv;
    std::vector<Vec3> grid(ncp);
    std::vector<double> w(ncp);
    for (int iu = 0; iu < nu; ++iu)
        for (int iv = 0; iv < nv; ++iv) {
            int k = iu * nv + iv;
            grid[k] = {(double) iu, (double) iv, 0.25 * iu * iv};
            w[k] = 1.0 + 0.01 * k;
        }
    // knots + multiplicities (u then v); sum(mult) is the expanded-knot length
    std::vector<double> uk(20), vk(20);
    std::vector<int> um(20, 1), vm(20, 1);
    for (int i = 0; i < 20; ++i) {
        uk[i] = (double) i;
        vk[i] = 0.5 * i;
    }
    int u_sum = 0, v_sum = 0;
    for (int m : um)
        u_sum += m;
    for (int m : vm)
        v_sum += m;

    p.clear();
    Enc::put_i32(p, 3); // u_degree
    Enc::put_i32(p, 3); // v_degree
    Enc::put_i32(p, 0); // u_closed
    Enc::put_i32(p, 0); // v_closed
    Enc::put_i32(p, 0); // self_intersect
    Enc::put_i32(p, nu);
    Enc::put_i32(p, nv);
    for (const Vec3 &g : grid)
        Enc::put_v3(p, g);
    Enc::put_i32(p, (int) uk.size()); // u_knots: f64s (count + data)
    for (double k : uk)
        Enc::put_f64(p, k);
    for (int m : um) // u_multiplicities: raw i32s (count == u_knots count)
        Enc::put_i32(p, m);
    Enc::put_i32(p, (int) vk.size()); // v_knots: f64s
    for (double k : vk)
        Enc::put_f64(p, k);
    for (int m : vm)
        Enc::put_i32(p, m);
    Enc::put_i32(p, 1); // has_weights
    for (double wi : w) // weights: raw ncp f64
        Enc::put_f64(p, wi);
    int bsurf = e.record(tag::BSPLINE_SURFACE, p);
    // wrap as a no-bounds FACE_SURFACE so it is reachable as a root
    p.clear();
    Enc::put_i32(p, bsurf);
    Enc::put_i32(p, 1); // same_sense
    Enc::put_i32(p, 0); // 0 bounds
    int face = e.record(tag::FACE_SURFACE, p);

    auto buf = e.finish({{face, "bs"}});
    NgeomDoc doc = decode(buf.data(), buf.size());
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "bspline surface root decoded");
    auto *bs = dynamic_cast<BSplineSurface *>(doc.roots[0].faces[0]->surface.get());
    CHECK(bs != nullptr, "decoded surface is a BSplineSurface");
    if (bs) {
        CHECK(bs->nu == nu && bs->nv == nv, "bspline nu/nv");
        CHECK((int) bs->ctrl.size() == ncp, "bspline ctrl count (vec3s)");
        bool grid_ok = bs->ctrl.size() == grid.size();
        for (size_t i = 0; grid_ok && i < grid.size(); ++i)
            grid_ok = vclose(bs->ctrl[i], grid[i]);
        CHECK(grid_ok, "bspline control grid matches (vec3s bulk read)");
        bool w_ok = bs->weights.size() == w.size();
        for (size_t i = 0; w_ok && i < w.size(); ++i)
            w_ok = close(bs->weights[i], w[i]);
        CHECK(w_ok, "bspline weights match (f64s bulk read)");
        // multiplicities feed expand_knots -> expanded length == sum(mult) (i32s bulk read)
        CHECK((int) bs->Uu.size() == u_sum && (int) bs->Uv.size() == v_sum, "knot multiplicities (i32s bulk read)");
    }

    // a 20-point POLY_LOOP -> exercises vec3s on the polygon path
    Enc e2;
    std::vector<Vec3> poly(20);
    for (int i = 0; i < 20; ++i)
        poly[i] = {std::cos(0.3 * i), std::sin(0.3 * i), 0.1 * i};
    p.clear();
    Enc::put_i32(p, (int) poly.size());
    for (const Vec3 &pt : poly)
        Enc::put_v3(p, pt);
    int loop = e2.record(tag::POLY_LOOP, p);
    p.clear();
    Enc::put_i32(p, loop);
    Enc::put_i32(p, 1);
    int fb = e2.record(tag::FACE_BOUND, p);
    p.clear();
    Enc::put_v3(p, {0, 0, 0});
    Enc::put_v3(p, {0, 0, 1});
    Enc::put_v3(p, {1, 0, 0});
    int pl = e2.record(tag::PLACEMENT3, p);
    p.clear();
    Enc::put_i32(p, pl);
    int plane = e2.record(tag::PLANE, p);
    p.clear();
    Enc::put_i32(p, plane);
    Enc::put_i32(p, 1);
    Enc::put_i32(p, 1); // 1 bound
    Enc::put_i32(p, fb);
    int face2 = e2.record(tag::FACE_SURFACE, p);
    auto buf2 = e2.finish({{face2, "pl"}});
    NgeomDoc doc2 = decode(buf2.data(), buf2.size());
    CHECK(doc2.roots.size() == 1 && doc2.roots[0].faces.size() == 1, "poly-loop face decoded");
    const auto &pg = doc2.roots[0].faces[0]->bounds[0].loop->polygon;
    bool poly_ok = pg.size() == poly.size();
    for (size_t i = 0; poly_ok && i < poly.size(); ++i)
        poly_ok = vclose(pg[i], poly[i]);
    CHECK(poly_ok, "20-pt poly-loop matches (vec3s bulk read)");
}

int main() {
    test_planar_triangle();
    test_planar_disk_with_circle_edge();
    test_unknown_tag_skipped();
    test_bulk_arrays_roundtrip();
    if (g_fail == 0)
        std::printf("ngeom decode: ALL PASS\n");
    else
        std::printf("ngeom decode: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
