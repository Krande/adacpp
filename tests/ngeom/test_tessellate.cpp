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

int main() {
    test_plane_square_with_hole();
    test_plane_same_sense_false_flips_normal();
    test_cylinder_patch();
    test_doc_grouping();
    if (g_fail == 0)
        std::printf("ngeom tessellate: ALL PASS\n");
    else
        std::printf("ngeom tessellate: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
