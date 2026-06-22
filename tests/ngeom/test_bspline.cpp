// Standalone unit test for NGEOM B-spline + swept surface evaluation (no OCC).
// Build: g++ -std=c++20 -I src/geom/neutral tests/ngeom/test_bspline.cpp -o /tmp/ngeom_bs
#include <cmath>
#include <cstdio>
#include <memory>

#include "ngeom_bspline.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);     \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static bool close(double a, double b, double tol = 1e-7) { return std::abs(a - b) <= tol; }
static bool vclose(const Vec3 &a, const Vec3 &b, double tol = 1e-6) { return (a - b).norm() <= tol; }

static void test_linear_bspline_curve_is_polyline() {
    // degree 1 -> the curve IS its control polygon
    std::vector<Vec3> cp{{0, 0, 0}, {1, 0, 0}, {2, 1, 0}};
    BSplineCurve c(1, cp, {0, 1, 2}, {2, 1, 2}, {}, false);
    CHECK(vclose(c.point(0.0), cp[0]), "lin bspline t=0 -> cp0");
    CHECK(vclose(c.point(2.0), cp[2]), "lin bspline t=2 -> cp2");
    CHECK(vclose(c.point(0.5), Vec3{0.5, 0, 0}), "lin bspline midpoint");
    CHECK(vclose(c.point(1.5), Vec3{1.5, 0.5, 0}), "lin bspline mid 2nd span");
}

static void test_rational_quarter_circle() {
    // standard NURBS unit quarter circle: deg 2, weights {1, 1/sqrt2, 1}, knots [0,0,0,1,1,1]
    std::vector<Vec3> cp{{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    double w = std::sqrt(0.5);
    BSplineCurve c(2, cp, {0, 1}, {3, 3}, {1, w, 1}, false);
    CHECK(vclose(c.point(0.0), Vec3{1, 0, 0}), "qcirc t=0");
    CHECK(vclose(c.point(1.0), Vec3{0, 1, 0}), "qcirc t=1");
    bool on = true;
    for (int i = 0; i <= 10; ++i) {
        double t = i / 10.0;
        if (!close(c.point(t).norm(), 1.0, 1e-9)) on = false;
    }
    CHECK(on, "qcirc points on unit circle (rational de Boor)");
    CHECK(vclose(c.point(0.5), Vec3{std::sqrt(0.5), std::sqrt(0.5), 0}, 1e-9), "qcirc midpoint = 45deg");
}

static BSplineSurface make_bilinear() {
    BSplineSurface s;
    s.u_degree = s.v_degree = 1;
    s.nu = s.nv = 2;
    // row-major iu*nv+iv: (u0v0)(u0v1)(u1v0)(u1v1); raise one corner -> non-planar bilinear
    s.ctrl = {{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 1, 1}};
    s.Uu = bspline_detail::expand_knots({0, 1}, {2, 2});
    s.Uv = bspline_detail::expand_knots({0, 1}, {2, 2});
    return s;
}

static void test_bilinear_surface() {
    BSplineSurface s = make_bilinear();
    CHECK(vclose(s.point(0, 0), Vec3{0, 0, 0}), "bilinear corner 00");
    CHECK(vclose(s.point(1, 0), Vec3{1, 0, 0}), "bilinear corner 10");
    CHECK(vclose(s.point(1, 1), Vec3{1, 1, 1}), "bilinear corner 11");
    CHECK(vclose(s.point(0.5, 0.5), Vec3{0.5, 0.5, 0.25}), "bilinear center");
    // uv roundtrip via Newton
    Vec3 p = s.point(0.3, 0.7);
    double u, v;
    CHECK(s.uv(p, std::nan(""), std::nan(""), u, v) && close(u, 0.3, 1e-5) && close(v, 0.7, 1e-5),
          "bilinear uv roundtrip (newton + grid seed)");
}

static void test_rational_bspline_surface_is_cylinder() {
    // sweep the rational quarter circle (in xy) along z -> a quarter cylinder; check radius.
    // build a 3x2 rational surface: u = circle (deg2, 3 ctrl), v = line (deg1, 2 ctrl, z 0..5)
    BSplineSurface s;
    s.u_degree = 2;
    s.v_degree = 1;
    s.nu = 3;
    s.nv = 2;
    double w = std::sqrt(0.5);
    // ctrl[iu*nv+iv]
    Vec3 base[3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    s.ctrl = {base[0], base[0] + Vec3{0, 0, 5}, base[1], base[1] + Vec3{0, 0, 5},
              base[2], base[2] + Vec3{0, 0, 5}};
    s.weights = {1, 1, w, w, 1, 1};
    s.Uu = bspline_detail::expand_knots({0, 1}, {3, 3});
    s.Uv = bspline_detail::expand_knots({0, 1}, {2, 2});
    bool on = true;
    for (int i = 0; i <= 6; ++i)
        for (int j = 0; j <= 4; ++j) {
            Vec3 p = s.point(i / 6.0, j / 4.0);
            double rho = std::sqrt(p.x * p.x + p.y * p.y);
            if (!close(rho, 1.0, 1e-9)) on = false;
        }
    CHECK(on, "rational bspline surface quarter-cylinder radius (homogeneous tensor product)");
}

static void test_linear_extrusion() {
    auto line = std::make_shared<LineCurve>(Vec3{2, 0, 0}, Vec3{0, 1, 0});  // x=2, along +y
    LinearExtrusionSurface s(line, Vec3{0, 0, 1}, 10.0);
    CHECK(vclose(s.point(0, 0), Vec3{2, 0, 0}), "extrude base");
    CHECK(vclose(s.point(3, 4), Vec3{2, 3, 4}), "extrude point");
    Vec3 n = s.normal(1, 1);
    CHECK(close(n.norm(), 1.0) && close(n.z, 0.0, 1e-9), "extrude normal perp to dir");
    double u, v;
    Vec3 p = s.point(0.4, 6.0);  // u in the profile's [0,1] range, v in [0,depth]
    CHECK(s.uv(p, std::nan(""), std::nan(""), u, v) && close(u, 0.4, 1e-5) && close(v, 6.0, 1e-5),
          "extrude uv roundtrip");
}

static void test_revolution_makes_cylinder() {
    // revolve a vertical line at radius 3 about the z axis -> cylinder of radius 3
    auto line = std::make_shared<LineCurve>(Vec3{3, 0, 0}, Vec3{0, 0, 1});  // param v = height
    RevolutionSurface s(line, Vec3{0, 0, 0}, Vec3{0, 0, 1});
    bool on = true;
    for (int i = 0; i <= 8; ++i)
        for (int j = 0; j <= 4; ++j) {
            Vec3 p = s.point(TWO_PI * i / 8.0, (double)j);
            double rho = std::sqrt(p.x * p.x + p.y * p.y);
            if (!close(rho, 3.0, 1e-9) || !close(p.z, (double)j, 1e-9)) on = false;
        }
    CHECK(on, "revolution of offset line = cylinder");
    double u, v;
    Vec3 p = s.point(1.0, 0.5);  // v in the profile's [0,1] range
    CHECK(s.uv(p, std::nan(""), std::nan(""), u, v) && close(u, 1.0, 1e-4) && close(v, 0.5, 1e-4),
          "revolution uv roundtrip");
}

int main() {
    test_linear_bspline_curve_is_polyline();
    test_rational_quarter_circle();
    test_bilinear_surface();
    test_rational_bspline_surface_is_cylinder();
    test_linear_extrusion();
    test_revolution_makes_cylinder();
    if (g_fail == 0)
        std::printf("ngeom bspline/swept: ALL PASS\n");
    else
        std::printf("ngeom bspline/swept: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
