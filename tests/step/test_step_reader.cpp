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
    ng::NgeomDoc doc = read_step_planar(kTriSolid, store);

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
    ng::NgeomDoc doc = read_step_planar(kTriSolid, store);
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

int main() {
    test_resolve_structure();
    test_tessellate();
    if (g_fail == 0)
        std::printf("step reader: ALL PASS\n");
    else
        std::printf("step reader: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
