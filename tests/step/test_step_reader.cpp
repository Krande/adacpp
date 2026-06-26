// End-to-end test for the native STEP planar-B-rep resolver:
//   STEP text -> Part-21 parse -> NGEOM neutral records -> libtess2 mesh.
// Built by tests/ngeom/run.sh (links ngeom_tessellate.cpp + ngeom_boolean.cpp stub + libtess2).
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "ngeom_surfaces.h"
#include "ngeom_tessellate.h"
#include "step_reader.h"

using namespace adacpp::step;
namespace ng = adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool vclose(const ng::Vec3 &a, double x, double y, double z) {
    return std::abs(a.x - x) < 1e-9 && std::abs(a.y - y) < 1e-9 && std::abs(a.z - z) < 1e-9;
}

// A single planar triangular face wrapped as CLOSED_SHELL / MANIFOLD_SOLID_BREP (static storage,
// so the parsed Instances' string_views stay valid).
static const char *kTriSolid = "ISO-10303-21;\n"
                               "HEADER;\nENDSEC;\n"
                               "DATA;\n"
                               "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
                               "#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
                               "#3=CARTESIAN_POINT('',(0.,1.,0.));\n"
                               "#11=VERTEX_POINT('',#1);\n"
                               "#12=VERTEX_POINT('',#2);\n"
                               "#13=VERTEX_POINT('',#3);\n"
                               "#20=DIRECTION('',(1.,0.,0.));\n"
                               "#21=DIRECTION('',(0.,0.,1.));\n"
                               "#30=VECTOR('',#20,1.);\n"
                               "#31=LINE('',#1,#30);\n"
                               "#41=EDGE_CURVE('',#11,#12,#31,.T.);\n"
                               "#42=EDGE_CURVE('',#12,#13,#31,.T.);\n"
                               "#43=EDGE_CURVE('',#13,#11,#31,.T.);\n"
                               "#51=ORIENTED_EDGE('',*,*,#41,.T.);\n"
                               "#52=ORIENTED_EDGE('',*,*,#42,.T.);\n"
                               "#53=ORIENTED_EDGE('',*,*,#43,.T.);\n"
                               "#60=EDGE_LOOP('',(#51,#52,#53));\n"
                               "#61=FACE_OUTER_BOUND('',#60,.T.);\n"
                               "#70=AXIS2_PLACEMENT_3D('',#1,#21,#20);\n"
                               "#71=PLANE('',#70);\n"
                               "#80=ADVANCED_FACE('tri',(#61),#71,.T.);\n"
                               "#90=CLOSED_SHELL('',(#80));\n"
                               "#100=MANIFOLD_SOLID_BREP('solid',#90);\n"
                               "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_resolve_structure() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kTriSolid, store);

    CHECK(doc.roots.size() == 1, "one solid root");
    if (doc.roots.empty())
        return;
    const ng::NgeomRoot &root = doc.roots[0];
    CHECK(root.id == "solid", "root id from MANIFOLD_SOLID_BREP name");
    CHECK(root.faces.size() == 1, "shell -> one face");
    if (root.faces.empty())
        return;
    const ng::FaceSurfaceN &f = *root.faces[0];
    CHECK(dynamic_cast<const ng::PlaneSurface *>(f.surface.get()) != nullptr, "face surface is a plane");
    CHECK(f.same_sense, "face same_sense");
    CHECK(f.bounds.size() == 1, "one face bound");
    if (f.bounds.empty())
        return;
    const ng::LoopN &lp = *f.bounds[0].loop;
    CHECK(!lp.is_poly && lp.edges.size() == 3, "edge loop with 3 edges");
    if (lp.edges.size() == 3) {
        CHECK(vclose(lp.edges[0].start, 0, 0, 0) && vclose(lp.edges[0].end, 1, 0, 0), "edge0 endpoints (V1->V2)");
        CHECK(vclose(lp.edges[1].start, 1, 0, 0) && vclose(lp.edges[1].end, 0, 1, 0), "edge1 endpoints (V2->V3)");
        CHECK(lp.edges[0].geometry == nullptr, "straight (LINE) edge -> null geometry");
    }
}

static void test_tessellate() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kTriSolid, store);
    ng::TessParams tp;
    tp.deflection = 0.1;
    ng::TessMesh m = ng::tessellate_doc(doc, tp);

    CHECK(m.indices.size() == 3, "one triangle (3 indices)");
    CHECK(m.positions.size() >= 9, "at least 3 vertices");
    CHECK(m.groups.size() == 1 && m.groups[0].id == "solid", "one group, id=solid");
    // every vertex inside the unit triangle's bbox [0,1]^3 (z==0)
    bool inbox = true;
    for (size_t i = 0; i + 2 < m.positions.size(); i += 3) {
        float x = m.positions[i], y = m.positions[i + 1], z = m.positions[i + 2];
        if (x < -1e-5 || x > 1 + 1e-5 || y < -1e-5 || y > 1 + 1e-5 || std::abs(z) > 1e-5)
            inbox = false;
    }
    CHECK(inbox, "vertices within the unit triangle bbox");
}

// Four ADVANCED_FACEs, each on a different analytic surface, sharing one POLY_LOOP bound.
static const char *kCurvedSolid = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
                                  "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
                                  "#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
                                  "#3=CARTESIAN_POINT('',(1.,1.,0.));\n"
                                  "#4=CARTESIAN_POINT('',(0.,1.,0.));\n"
                                  "#5=DIRECTION('',(0.,0.,1.));\n"
                                  "#6=DIRECTION('',(1.,0.,0.));\n"
                                  "#7=AXIS2_PLACEMENT_3D('',#1,#5,#6);\n"
                                  "#10=POLY_LOOP('',(#1,#2,#3,#4));\n"
                                  "#11=FACE_OUTER_BOUND('',#10,.T.);\n"
                                  "#20=CYLINDRICAL_SURFACE('',#7,2.);\n"
                                  "#21=CONICAL_SURFACE('',#7,2.,0.5);\n"
                                  "#22=SPHERICAL_SURFACE('',#7,3.);\n"
                                  "#23=TOROIDAL_SURFACE('',#7,5.,1.);\n"
                                  "#30=ADVANCED_FACE('cyl',(#11),#20,.T.);\n"
                                  "#31=ADVANCED_FACE('con',(#11),#21,.T.);\n"
                                  "#32=ADVANCED_FACE('sph',(#11),#22,.T.);\n"
                                  "#33=ADVANCED_FACE('tor',(#11),#23,.T.);\n"
                                  "#40=CLOSED_SHELL('',(#30,#31,#32,#33));\n"
                                  "#50=MANIFOLD_SOLID_BREP('curved',#40);\n"
                                  "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_curved_surfaces_and_polyloop() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kCurvedSolid, store);
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 4, "curved solid: 4 faces");
    if (doc.roots.empty() || doc.roots[0].faces.size() != 4)
        return;
    auto &fs = doc.roots[0].faces;
    auto *cyl = dynamic_cast<const ng::CylinderSurface *>(fs[0]->surface.get());
    auto *con = dynamic_cast<const ng::ConeSurface *>(fs[1]->surface.get());
    auto *sph = dynamic_cast<const ng::SphereSurface *>(fs[2]->surface.get());
    auto *tor = dynamic_cast<const ng::TorusSurface *>(fs[3]->surface.get());
    CHECK(cyl && std::abs(cyl->r - 2.0) < 1e-9, "cylinder surface + radius");
    CHECK(con && std::abs(con->r0 - 2.0) < 1e-9 && std::abs(con->semi_angle - 0.5) < 1e-9, "cone surface + params");
    CHECK(sph && std::abs(sph->r - 3.0) < 1e-9, "sphere surface + radius");
    CHECK(tor && std::abs(tor->R - 5.0) < 1e-9 && std::abs(tor->r - 1.0) < 1e-9, "torus surface + radii");
    // POLY_LOOP bound
    const ng::LoopN &lp = *fs[0]->bounds[0].loop;
    CHECK(lp.is_poly && lp.polygon.size() == 4, "poly-loop bound with 4 points");
    if (lp.polygon.size() == 4)
        CHECK(vclose(lp.polygon[2], 1, 1, 0), "poly-loop point 2");
}

// A circular disk: a planar face bounded by one full-circle EDGE_CURVE (CIRCLE geometry).
static const char *kDiskSolid = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
                                "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
                                "#2=CARTESIAN_POINT('',(2.,0.,0.));\n"
                                "#3=VERTEX_POINT('',#2);\n"
                                "#4=DIRECTION('',(0.,0.,1.));\n"
                                "#5=DIRECTION('',(1.,0.,0.));\n"
                                "#6=AXIS2_PLACEMENT_3D('',#1,#4,#5);\n"
                                "#7=CIRCLE('',#6,2.);\n"
                                "#8=EDGE_CURVE('',#3,#3,#7,.T.);\n"
                                "#9=ORIENTED_EDGE('',*,*,#8,.T.);\n"
                                "#10=EDGE_LOOP('',(#9));\n"
                                "#11=FACE_OUTER_BOUND('',#10,.T.);\n"
                                "#12=PLANE('',#6);\n"
                                "#13=ADVANCED_FACE('disk',(#11),#12,.T.);\n"
                                "#14=CLOSED_SHELL('',(#13));\n"
                                "#15=MANIFOLD_SOLID_BREP('disk',#14);\n"
                                "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_conic_edge() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kDiskSolid, store);
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "disk: one face");
    if (doc.roots.empty() || doc.roots[0].faces.empty())
        return;
    const ng::LoopN &lp = *doc.roots[0].faces[0]->bounds[0].loop;
    CHECK(!lp.is_poly && lp.edges.size() == 1, "edge loop with one (circular) edge");
    if (!lp.edges.empty()) {
        auto *circ = dynamic_cast<const ng::CircleCurve *>(lp.edges[0].geometry.get());
        CHECK(circ && std::abs(circ->r - 2.0) < 1e-9, "edge geometry is a circle, r=2");
    }
    // tessellate: a disk discretizes to many triangles, all within radius 2
    ng::TessParams tp;
    tp.deflection = 0.02;
    ng::TessMesh m = ng::tessellate_doc(doc, tp);
    CHECK(m.indices.size() > 3, "disk tessellates to many triangles");
    bool inr = m.positions.size() >= 9;
    for (size_t i = 0; i + 2 < m.positions.size(); i += 3) {
        float x = m.positions[i], y = m.positions[i + 1];
        if (std::sqrt(x * x + y * y) > 2.0 + 1e-3)
            inr = false;
    }
    CHECK(inr, "disk vertices within radius 2");
}

int main() {
    test_resolve_structure();
    test_tessellate();
    test_curved_surfaces_and_polyloop();
    test_conic_edge();
    if (g_fail == 0)
        std::printf("step reader: ALL PASS\n");
    else
        std::printf("step reader: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
