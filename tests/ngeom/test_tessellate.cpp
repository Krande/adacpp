// Standalone test for the NGEOM libtess2 tessellator (no OCC). Compiles against the
// vendored libtess2 C sources directly (compile the .c with gcc — C — then link with g++).
// See tests/ngeom/run.sh for the exact build commands.
#include <cmath>
#include <cstdio>
#include <memory>

#include "ngeom_bspline.h"
#include "ngeom_tessellate.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                 \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++g_fail;                                                    \
        }                                                                \
    } while (0)
static bool close(double a, double b, double tol) { return std::abs(a - b) <= tol; }

static double total_area(const TessMesh &m) {
    double area = 0;
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        auto V = [&](uint32_t k) {
            return Vec3{m.positions[k * 3], m.positions[k * 3 + 1], m.positions[k * 3 + 2]};
        };
        Vec3 a = V(m.indices[i]), b = V(m.indices[i + 1]), c = V(m.indices[i + 2]);
        area += 0.5 * (b - a).cross(c - a).norm();
    }
    return area;
}

static std::shared_ptr<LoopN> poly(const std::vector<Vec3> &pts) {
    auto l = std::make_shared<LoopN>();
    l->is_poly = true;
    l->polygon = pts;
    return l;
}

static void test_plane_square_with_hole() {
    auto plane = std::make_shared<PlaneSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    FaceSurfaceN f;
    f.surface = plane;
    f.same_sense = true;
    f.bounds.push_back({poly({{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}}), true});
    f.bounds.push_back({poly({{4, 4, 0}, {6, 4, 0}, {6, 6, 0}, {4, 6, 0}}), true});

    TessMesh m;
    TessParams tp;
    CHECK(tessellate_face(f, tp, m), "plane+hole tessellates");
    CHECK(close(total_area(m), 96.0, 1e-4), "plane+hole area = 100 - 4");
    // all verts on z=0, normals +z
    bool flat = true, nrm = true;
    for (size_t i = 0; i < m.positions.size(); i += 3)
        if (!close(m.positions[i + 2], 0.0, 1e-6)) flat = false;
    for (size_t i = 0; i < m.normals.size(); i += 3)
        if (!close(m.normals[i + 2], 1.0, 1e-6)) nrm = false;
    CHECK(flat, "plane verts on z=0");
    CHECK(nrm, "plane normals +z");
}

static void test_plane_same_sense_false_flips_normal() {
    auto plane = std::make_shared<PlaneSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    FaceSurfaceN f;
    f.surface = plane;
    f.same_sense = false;
    f.bounds.push_back({poly({{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}}), true});
    TessMesh m;
    TessParams tp;
    CHECK(tessellate_face(f, tp, m), "flipped plane tessellates");
    bool down = true;
    for (size_t i = 0; i < m.normals.size(); i += 3)
        if (!close(m.normals[i + 2], -1.0, 1e-6)) down = false;
    CHECK(down, "same_sense=false flips normal to -z");
}

static void test_cylinder_patch() {
    auto cyl = std::make_shared<CylinderSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), 5.0);
    // boundary of a u in [0,1] rad, v in [0,8] patch, sampled on the surface
    std::vector<Vec3> bnd;
    const int N = 16;
    for (int i = 0; i <= N; ++i) bnd.push_back(cyl->point(1.0 * i / N, 0.0));        // bottom arc
    for (int j = 1; j <= N; ++j) bnd.push_back(cyl->point(1.0, 8.0 * j / N));        // right edge
    for (int i = N - 1; i >= 0; --i) bnd.push_back(cyl->point(1.0 * i / N, 8.0));    // top arc
    for (int j = N - 1; j >= 1; --j) bnd.push_back(cyl->point(0.0, 8.0 * j / N));    // left edge
    FaceSurfaceN f;
    f.surface = cyl;
    f.same_sense = true;
    f.bounds.push_back({poly(bnd), true});

    TessMesh m;
    TessParams tp;
    CHECK(tessellate_face(f, tp, m), "cylinder patch tessellates");
    bool on = true;
    for (size_t i = 0; i < m.positions.size(); i += 3) {
        double rho = std::sqrt(m.positions[i] * m.positions[i] + m.positions[i + 1] * m.positions[i + 1]);
        if (!close(rho, 5.0, 1e-3)) on = false;
    }
    CHECK(on, "cylinder patch verts on radius 5");
    double analytic = 5.0 * 1.0 * 8.0;  // r * du(rad) * dv
    CHECK(total_area(m) > 0.9 * analytic && total_area(m) < 1.02 * analytic, "cylinder patch area ~ analytic");
}

static size_t tri_count(const TessMesh &m) { return m.indices.size() / 3; }

static void test_refinement_smooths_curved_face() {
    auto cyl = std::make_shared<CylinderSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), 5.0);
    // a COARSE boundary so the interior triangulation has long edges for refinement to split
    std::vector<Vec3> bnd;
    const int N = 3;
    for (int i = 0; i <= N; ++i) bnd.push_back(cyl->point(1.0 * i / N, 0.0));
    for (int j = 1; j <= N; ++j) bnd.push_back(cyl->point(1.0, 8.0 * j / N));
    for (int i = N - 1; i >= 0; --i) bnd.push_back(cyl->point(1.0 * i / N, 8.0));
    for (int j = N - 1; j >= 1; --j) bnd.push_back(cyl->point(0.0, 8.0 * j / N));
    FaceSurfaceN f;
    f.surface = cyl;
    f.same_sense = true;
    f.bounds.push_back({poly(bnd), true});
    double analytic = 5.0 * 1.0 * 8.0;

    // step2glb-faithful refine_uv always honours the parametric u_step cap + the chord-sag test
    // on a curved surface, so "refinement" is driven by the deflection: a large deflection leaves
    // a coarse mesh, a small one densifies it. (deflection=0 is the degenerate "auto" sentinel —
    // the sag test dev>0 then fires on every curved edge and explodes to the budget, so it is not
    // a meaningful "no refinement" baseline.)
    TessMesh coarse;
    TessParams cp;
    cp.deflection = 5.0;  // very loose chord sag -> few triangles
    tessellate_face(f, cp, coarse);

    TessMesh fine;
    TessParams fp;
    fp.deflection = 0.02;  // tight chord sag -> densified
    tessellate_face(f, fp, fine);

    CHECK(tri_count(fine) > tri_count(coarse), "refinement adds triangles on a curved face");
    CHECK(total_area(fine) > total_area(coarse), "refinement increases area toward analytic");
    CHECK(total_area(fine) > 0.99 * analytic && total_area(fine) < 1.001 * analytic,
          "refined cylinder area ~ analytic (faceting slightly under)");
    bool on = true;
    for (size_t i = 0; i < fine.positions.size(); i += 3) {
        double rho = std::sqrt(fine.positions[i] * fine.positions[i] +
                               fine.positions[i + 1] * fine.positions[i + 1]);
        if (!close(rho, 5.0, 1e-3)) on = false;
    }
    CHECK(on, "refined verts still on radius");
}

static std::shared_ptr<LoopN> circle_loop(const CylinderSurface &c, double v, int n) {
    std::vector<Vec3> pts;
    for (int i = 0; i < n; ++i) pts.push_back(c.point(TWO_PI * i / n, v));
    return poly(pts);
}

static void test_full_cylinder_gridded() {
    auto cyl = std::make_shared<CylinderSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), 5.0);
    FaceSurfaceN f;
    f.surface = cyl;
    f.same_sense = true;
    // two full-circle bounds (bottom v=0, top v=8): zero UV area -> gridded path (1c)
    f.bounds.push_back({circle_loop(*cyl, 0.0, 48), true});
    f.bounds.push_back({circle_loop(*cyl, 8.0, 48), true});
    TessMesh m;
    TessParams tp;
    tp.deflection = 0.02;
    CHECK(tessellate_face(f, tp, m), "full cylinder tessellates (gridded)");
    double analytic = TWO_PI * 5.0 * 8.0;
    CHECK(total_area(m) > 0.99 * analytic && total_area(m) < 1.001 * analytic,
          "full cylinder lateral area ~ 2*pi*r*h");
    bool on = true;
    for (size_t i = 0; i < m.positions.size(); i += 3) {
        double rho = std::sqrt(m.positions[i] * m.positions[i] + m.positions[i + 1] * m.positions[i + 1]);
        if (!close(rho, 5.0, 1e-3)) on = false;
    }
    CHECK(on, "full cylinder verts on radius");
}

static void test_full_sphere_gridded() {
    auto sph = std::make_shared<SphereSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), 5.0);
    // seam meridian loop (u=0): out-and-back -> zero UV area -> gridded path
    std::vector<Vec3> seam;
    const int N = 24;
    for (int i = 0; i <= N; ++i) seam.push_back(sph->point(0.0, -PI / 2 + PI * i / N));
    for (int i = N - 1; i >= 1; --i) seam.push_back(sph->point(0.0, -PI / 2 + PI * i / N));
    FaceSurfaceN f;
    f.surface = sph;
    f.same_sense = true;
    f.bounds.push_back({poly(seam), true});
    TessMesh m;
    TessParams tp;
    tp.deflection = 0.02;
    CHECK(tessellate_face(f, tp, m), "full sphere tessellates (gridded)");
    double analytic = 4.0 * PI * 25.0;
    CHECK(total_area(m) > 0.98 * analytic && total_area(m) < 1.001 * analytic,
          "full sphere area ~ 4*pi*r^2");
    bool on = true;
    for (size_t i = 0; i < m.positions.size(); i += 3) {
        double rr = std::sqrt(m.positions[i] * m.positions[i] + m.positions[i + 1] * m.positions[i + 1] +
                              m.positions[i + 2] * m.positions[i + 2]);
        if (!close(rr, 5.0, 2e-2)) on = false;
    }
    CHECK(on, "full sphere verts on radius");
}

static void test_doc_grouping() {
    NgeomDoc doc;
    auto plane = std::make_shared<PlaneSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    for (int k = 0; k < 2; ++k) {
        auto fs = std::make_shared<FaceSurfaceN>();
        fs->surface = plane;
        fs->same_sense = true;
        fs->bounds.push_back({poly({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}}), true});
        NgeomRoot root;
        root.id = (k == 0) ? "a" : "b";
        root.faces.push_back(fs);
        doc.roots.push_back(root);
    }
    TessMesh m = tessellate_doc(doc, TessParams{});
    CHECK(m.groups.size() == 2 && m.groups[0].id == "a" && m.groups[1].id == "b", "doc groups per root");
    CHECK(m.groups[0].index_count == 6 && m.groups[1].index_count == 6, "each square = 2 tris");
}

static std::shared_ptr<LoopN> cone_circle_loop(const ConeSurface &c, double v, int n) {
    std::vector<Vec3> pts;
    for (int i = 0; i < n; ++i) pts.push_back(c.point(TWO_PI * i / n, v));
    return poly(pts);
}

// Guards the cone AXIAL-v parameterization (step2glb geom.rs): point(u,v).z == v,
// radius_at(v) == r0 + v*tan(a), apex == -r0/tan(a). The previous SLANT-v parameterization
// (z = v*cos(a), radius = r0 + v*sin(a), apex = -r0/sin(a)) over-sampled cones; the cone SHAPE
// is identical under both, so we assert the parameterization directly, plus a faithful frustum.
static void test_cone_axial_v_parameterization() {
    const double a = PI / 6.0;  // 30 deg
    const double tan_a = std::tan(a), r0 = 5.0;
    auto cone = std::make_shared<ConeSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), r0, a);

    Vec3 p = cone->point(0.0, 8.0);
    CHECK(close(p.z, 8.0, 1e-9), "cone point(u,v).z == v (axial-v, not v*cos(a))");
    CHECK(close(std::sqrt(p.x * p.x + p.y * p.y), r0 + 8.0 * tan_a, 1e-9),
          "cone radius_at(v) == r0 + v*tan(a)");
    auto caps = cone->v_caps();
    CHECK(caps && close(caps->first, -r0 / tan_a, 1e-6), "cone apex == -r0/tan(a)");

    // and a full frustum (z in [0,8]) tessellates faithfully to the analytic lateral area
    FaceSurfaceN f;
    f.surface = cone;
    f.same_sense = true;
    f.bounds.push_back({cone_circle_loop(*cone, 0.0, 48), true});
    f.bounds.push_back({cone_circle_loop(*cone, 8.0, 48), true});
    TessMesh m;
    TessParams tp;
    tp.deflection = 0.02;
    CHECK(tessellate_face(f, tp, m), "cone frustum tessellates");
    const double r1 = r0 + 8.0 * tan_a;
    const double slant = std::sqrt((r1 - r0) * (r1 - r0) + 8.0 * 8.0);
    const double analytic = PI * (r0 + r1) * slant;  // frustum lateral area
    CHECK(total_area(m) > 0.99 * analytic && total_area(m) < 1.01 * analytic,
          "cone frustum lateral area ~ pi*(r0+r1)*slant");
    bool on = true;
    for (size_t i = 0; i < m.positions.size(); i += 3) {
        double z = m.positions[i + 2];
        double rho = std::sqrt(m.positions[i] * m.positions[i] + m.positions[i + 1] * m.positions[i + 1]);
        if (!close(rho, r0 + z * tan_a, 2e-2)) on = false;
    }
    CHECK(on, "cone verts satisfy axial-v radius = r0 + z*tan(a)");
}

// Guards the apex-terminated cone: a cone bounded by a SINGLE full-circle loop at constant v, closing
// to its apex on the far side (r0 == 0). The loop has zero v-extent, which the periodic-winding path
// used to reject (vmax-vmin > 0), silently DROPPING the whole cone to zero triangles. It must now
// close the winding to the finite v-cap (the apex) and mesh the full cone. This is the trimmed-conical
// drop class the native path was losing (hundreds of faces per conical-head model).
static void test_cone_apex_single_loop() {
    const double a = PI / 6.0;                 // 30 deg
    const double tan_a = std::tan(a), r0 = 0.0; // apex at v = 0
    const double vtop = 8.0;
    auto cone = std::make_shared<ConeSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}), r0, a);
    // Both orientations that route to the winding path (interior "above" the flat loop) used to drop.
    for (bool same_sense : {true, false})
        for (bool ccw : {true, false}) {
            std::vector<Vec3> pts;
            const int N = 240; // fine like a real deflection-sampled circle edge
            for (int i = 0; i < N; ++i) {
                double u = TWO_PI * i / N;
                pts.push_back(cone->point(ccw ? u : -u, vtop));
            }
            FaceSurfaceN f;
            f.surface = cone;
            f.same_sense = same_sense;
            f.bounds.push_back({poly(pts), true});
            reset_tess_face_stats();
            TessMesh m;
            TessParams tp;
            tp.deflection = 0.02;
            CHECK(tessellate_face(f, tp, m), "apex cone tessellates");
            CHECK(tess_dropped_faces() == 0, "apex cone not dropped");
            CHECK(m.indices.size() > 0, "apex cone produced triangles");
            const double r1 = r0 + vtop * tan_a;
            const double slant = std::sqrt(r1 * r1 + vtop * vtop);
            const double analytic = PI * r1 * slant; // full cone lateral area
            CHECK(total_area(m) > 0.97 * analytic && total_area(m) < 1.03 * analytic,
                  "apex cone lateral area ~ pi*r*slant");
        }
    // per-surface-type drop breakdown: a genuinely un-meshable face (a collapsed loop on a plane, which
    // has no v-cap to recover to) increments the "plane" bucket, and the sum matches tess_dropped_faces.
    {
        reset_tess_face_stats();
        auto plane = std::make_shared<PlaneSurface>(Frame::from_axis_ref({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
        FaceSurfaceN f;
        f.surface = plane;
        f.same_sense = true;
        f.bounds.push_back({poly({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}), true}); // degenerate loop
        TessMesh m;
        TessParams tp;
        tessellate_face(f, tp, m);
        auto by = tess_dropped_by_surface();
        std::uint64_t sum = 0;
        for (auto &kv : by) sum += kv.second;
        CHECK(sum == tess_dropped_faces(), "drop_by_surface sums to dropped_faces");
    }
}

// Guards the B-spline edge boundary fix: an edge whose geometry is a B-spline and which carries
// NO trim params must be sampled over the curve's FULL natural domain (then aligned to the edge
// vertices), NOT collapsed to a 2-point straight chord. (The collapse was the dominant ~2x density
// gap vs step2glb.)
static void test_bspline_edge_samples_natural_domain() {
    // degree-3 clamped B-spline, 5 control points (curvy) -> unique knots [0,0.5,1] mults [4,1,4]
    std::vector<Vec3> cps = {{0, 0, 0}, {2, 4, 0}, {5, 4, 0}, {8, -2, 0}, {10, 1, 0}};
    auto bsp = std::make_shared<BSplineCurve>(3, cps, std::vector<double>{0.0, 0.5, 1.0},
                                              std::vector<int>{4, 1, 4}, std::vector<double>{}, false);
    double lo, hi, per;
    bool periodic;
    bsp->range(lo, hi, periodic, per);

    OrientedEdgeN e;
    e.geometry = bsp;
    e.has_params = false;  // exporters often omit trim params; the fix must NOT bail to a chord
    e.same_sense = true;
    e.orientation = true;
    e.start = e.e_start = bsp->point(lo);
    e.end = e.e_end = bsp->point(hi);

    auto pts = e.discretize(0.05, 0.349);
    CHECK(pts.size() > 8, "B-spline edge sampled over its natural domain (dense, not a 2-pt chord)");
    CHECK((pts.front() - e.start).norm() < 1e-6 && (pts.back() - e.end).norm() < 1e-6,
          "B-spline edge endpoints snapped to the topological vertices");
    // the polyline follows the curve, so it is materially longer than the straight start->end chord
    double plen = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) plen += (pts[i] - pts[i - 1]).norm();
    CHECK(plen > 1.2 * (e.end - e.start).norm(), "B-spline edge polyline follows the curve, not a chord");
}

int main() {
    test_plane_square_with_hole();
    test_plane_same_sense_false_flips_normal();
    test_cylinder_patch();
    test_refinement_smooths_curved_face();
    test_full_cylinder_gridded();
    test_full_sphere_gridded();
    test_cone_axial_v_parameterization();
    test_cone_apex_single_loop();
    test_bspline_edge_samples_natural_domain();
    test_doc_grouping();
    if (g_fail == 0)
        std::printf("ngeom tessellate: ALL PASS\n");
    else
        std::printf("ngeom tessellate: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
