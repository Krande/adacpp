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
#define CHECK(cond, msg)                                                 \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++g_fail;                                                    \
        }                                                                \
    } while (0)
static bool close(double a, double b, double tol = 1e-7) { return std::abs(a - b) <= tol; }
static bool vclose(const Vec3 &a, const Vec3 &b, double tol = 1e-6) { return (a - b).norm() <= tol; }

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
        put_i32(recs, (int)payload.size());
        recs.insert(recs.end(), payload.begin(), payload.end());
        return idx;
    }
    std::vector<uint8_t> finish(const std::vector<std::pair<int, std::string>> &roots) {
        std::vector<uint8_t> out;
        const char *magic = "ADANGEOM";
        out.insert(out.end(), magic, magic + 8);
        put_i32(out, (int)NGEOM_VERSION);
        put_i32(out, count);
        out.insert(out.end(), recs.begin(), recs.end());
        put_i32(out, (int)roots.size());
        for (auto &[gi, id] : roots) {
            put_i32(out, gi);
            put_i32(out, (int)id.size());
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
        Enc::put_i32(p, -1);  // null geometry
        Enc::put_i32(p, 1);   // same_sense
        return e.record(tag::EDGE_CURVE, p);
    };
    int eAB = edge(A, B), eBC = edge(B, C), eCA = edge(C, A);
    auto oedge = [&](int er) {
        p.clear();
        Enc::put_i32(p, er);
        Enc::put_i32(p, 1);  // orientation
        Enc::put_i32(p, 0);  // has_pcurve
        Enc::put_i32(p, 0);  // has_params
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
    Enc::put_i32(p, 1);  // same_sense
    Enc::put_i32(p, 1);  // n_bounds
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
    Enc::put_i32(p, 1);  // orientation
    Enc::put_i32(p, 0);  // has_pcurve
    Enc::put_i32(p, 1);  // has_params
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
        if (!close(std::sqrt(q.x * q.x + q.y * q.y), 2.0, 1e-3)) on = false;
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

int main() {
    test_planar_triangle();
    test_planar_disk_with_circle_edge();
    test_unknown_tag_skipped();
    if (g_fail == 0)
        std::printf("ngeom decode: ALL PASS\n");
    else
        std::printf("ngeom decode: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
