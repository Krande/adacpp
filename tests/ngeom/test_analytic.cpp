// Standalone unit test for the NGEOM analytic geometry core (no OCC, no nanobind).
// Build: g++ -std=c++20 -I src/geom/neutral tests/ngeom/test_analytic.cpp -o /tmp/ngeom_test
#include <cstdio>
#include <cmath>

#include "ngeom_surfaces.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++g_fail;                                             \
        }                                                         \
    } while (0)

static bool close(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }
static bool vclose(const Vec3 &a, const Vec3 &b, double tol = 1e-7) {
    return (a - b).norm() <= tol;
}

// A non-axis-aligned frame to catch frame bugs.
static Frame tilted() {
    return Frame::from_axis_ref(Vec3{1, 2, 3}, Vec3{1, 1, 1}, Vec3{1, 0, 0});
}

static void test_frame_orthonormal() {
    Frame f = tilted();
    CHECK(close(f.x.norm(), 1.0) && close(f.y.norm(), 1.0) && close(f.z.norm(), 1.0), "frame unit");
    CHECK(close(f.x.dot(f.y), 0.0) && close(f.x.dot(f.z), 0.0) && close(f.y.dot(f.z), 0.0), "frame ortho");
    CHECK(vclose(f.x.cross(f.y), f.z), "frame right-handed");
}

static void test_plane() {
    PlaneSurface s(tilted());
    Vec3 p = s.point(2.5, -1.3);
    double u, v;
    CHECK(s.uv(p, 0, 0, u, v) && close(u, 2.5) && close(v, -1.3), "plane uv roundtrip");
    CHECK(vclose(s.normal(0, 0), tilted().z), "plane normal = z");
}

static void test_cylinder() {
    Frame f = tilted();
    CylinderSurface s(f, 3.0);
    double u0 = 0.7, v0 = 4.2;
    Vec3 p = s.point(u0, v0);
    CHECK(close((p - f.o - f.z * f.z.dot(p - f.o)).norm(), 3.0, 1e-7), "cyl point on radius");
    double u, v;
    CHECK(s.uv(p, 0, 0, u, v) && close(u, u0, 1e-7) && close(v, v0, 1e-7), "cyl uv roundtrip");
    // normal is radial, unit, perpendicular to axis
    Vec3 n = s.normal(u0, v0);
    CHECK(close(n.norm(), 1.0) && close(n.dot(f.z), 0.0, 1e-9), "cyl normal radial unit");
    // normal points outward from the axis through the point
    Vec3 radial = (p - f.o) - f.z * f.z.dot(p - f.o);
    CHECK(n.dot(radial) > 0, "cyl normal outward");
}

static void test_cone() {
    Frame f = tilted();
    ConeSurface s(f, 2.0, 0.4);
    double u0 = 1.1, v0 = 1.5;
    Vec3 p = s.point(u0, v0);
    double u, v;
    CHECK(s.uv(p, 0, 0, u, v) && close(u, u0, 1e-7) && close(v, v0, 1e-7), "cone uv roundtrip");
    Vec3 n = s.normal(u0, v0);
    CHECK(close(n.norm(), 1.0), "cone normal unit");
    // normal perpendicular to both surface tangents (finite-diff cross check)
    CHECK(vclose(n, s.fd_normal(u0, v0), 1e-4) || vclose(n * -1.0, s.fd_normal(u0, v0), 1e-4),
          "cone normal matches FD");
}

static void test_sphere() {
    Frame f = tilted();
    SphereSurface s(f, 5.0);
    double u0 = 2.0, v0 = 0.6;
    Vec3 p = s.point(u0, v0);
    CHECK(close((p - f.o).norm(), 5.0, 1e-7), "sphere point on radius");
    double u, v;
    CHECK(s.uv(p, 0, 0, u, v) && close(u, u0, 1e-7) && close(v, v0, 1e-7), "sphere uv roundtrip");
    Vec3 n = s.normal(u0, v0);
    CHECK(vclose(n, (p - f.o).normalized(), 1e-7), "sphere normal = radial");
}

static void test_torus() {
    Frame f = tilted();
    TorusSurface s(f, 10.0, 2.0);
    double u0 = 1.3, v0 = 2.4;
    Vec3 p = s.point(u0, v0);
    double u, v;
    CHECK(s.uv(p, 0, 0, u, v) && close(u, u0, 1e-6) && close(v, v0, 1e-6), "torus uv roundtrip");
    Vec3 n = s.normal(u0, v0);
    CHECK(close(n.norm(), 1.0), "torus normal unit");
    CHECK(vclose(n, s.fd_normal(u0, v0), 1e-4) || vclose(n * -1.0, s.fd_normal(u0, v0), 1e-4),
          "torus normal matches FD");
}

static void test_circle_discretize() {
    Frame f = tilted();
    CircleCurve c(f, 4.0);
    auto coarse = c.discretize(0.0, TWO_PI, 1.0, 1.0);
    auto fine = c.discretize(0.0, TWO_PI, 0.01, 0.1);
    CHECK(fine.size() > coarse.size(), "finer deflection -> more points");
    // all points on the circle radius
    bool on = true;
    for (auto &p : fine) {
        double rr = ((p - f.o) - f.z * f.z.dot(p - f.o)).norm();
        if (!close(rr, 4.0, 1e-6)) on = false;
    }
    CHECK(on, "circle points on radius");
}

int main() {
    test_frame_orthonormal();
    test_plane();
    test_cylinder();
    test_cone();
    test_sphere();
    test_torus();
    test_circle_discretize();
    if (g_fail == 0)
        std::printf("ngeom analytic: ALL PASS\n");
    else
        std::printf("ngeom analytic: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
