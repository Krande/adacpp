// End-to-end test for the native STEP planar-B-rep resolver:
//   STEP text -> Part-21 parse -> NGEOM neutral records -> libtess2 mesh.
// Built by tests/ngeom/run.sh (links ngeom_tessellate.cpp + ngeom_boolean.cpp stub + libtess2).
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "ngeom_bspline.h"
#include "ngeom_decode.h"
#include "ngeom_encode.h"
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

// One face on a simple (non-rational) B_SPLINE_SURFACE_WITH_KNOTS and one on a rational complex
// record (weights ((1,1),(1,2))) — both a 2x2 degree-1 patch over the unit square.
static const char *kBSplineSurf =
    "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
    "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
    "#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
    "#3=CARTESIAN_POINT('',(0.,1.,0.));\n"
    "#4=CARTESIAN_POINT('',(1.,1.,0.));\n"
    "#10=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.,"
    "(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n"
    "#11=POLY_LOOP('',(#1,#2,#4,#3));\n"
    "#12=FACE_OUTER_BOUND('',#11,.T.);\n"
    "#13=ADVANCED_FACE('s1',(#12),#10,.T.);\n"
    "#20=(BOUNDED_SURFACE()B_SPLINE_SURFACE(1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.)"
    "B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)"
    "RATIONAL_B_SPLINE_SURFACE(((1.,1.),(1.,2.)))GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM('')SURFACE());\n"
    "#21=ADVANCED_FACE('s2',(#12),#20,.T.);\n"
    "#30=CLOSED_SHELL('',(#13,#21));\n"
    "#40=MANIFOLD_SOLID_BREP('bspl',#30);\n"
    "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_bspline_surfaces() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kBSplineSurf, store);
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 2, "bspline solid: 2 faces");
    if (doc.roots.empty() || doc.roots[0].faces.size() != 2)
        return;
    auto *s1 = dynamic_cast<const ng::BSplineSurface *>(doc.roots[0].faces[0]->surface.get());
    CHECK(s1 && s1->u_degree == 1 && s1->v_degree == 1 && s1->nu == 2 && s1->nv == 2, "simple bspline degree/grid");
    CHECK(s1 && s1->weights.empty(), "non-rational -> no weights");
    CHECK(s1 && s1->ctrl.size() == 4 && std::abs(s1->ctrl[3].x - 1.0) < 1e-9 && std::abs(s1->ctrl[3].y - 1.0) < 1e-9,
          "control grid row-major");
    auto *s2 = dynamic_cast<const ng::BSplineSurface *>(doc.roots[0].faces[1]->surface.get());
    CHECK(s2 && s2->weights.size() == 4 && std::abs(s2->weights[3] - 2.0) < 1e-9, "rational complex record weights");
    // a 2x2 degree-1 patch over the unit square tessellates to triangles
    ng::TessParams tp;
    tp.deflection = 0.1;
    ng::TessMesh m = ng::tessellate_doc(doc, tp);
    CHECK(m.indices.size() >= 6, "bspline patches tessellate to triangles");
}

// A planar face whose single boundary edge is a degree-2 B_SPLINE_CURVE_WITH_KNOTS.
static const char *kBSplineEdge = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
                                  "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
                                  "#2=CARTESIAN_POINT('',(0.5,1.,0.));\n"
                                  "#3=CARTESIAN_POINT('',(1.,0.,0.));\n"
                                  "#4=VERTEX_POINT('',#1);\n"
                                  "#5=VERTEX_POINT('',#3);\n"
                                  "#6=B_SPLINE_CURVE_WITH_KNOTS('',2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,"
                                  "(3,3),(0.,1.),.UNSPECIFIED.);\n"
                                  "#7=EDGE_CURVE('',#4,#5,#6,.T.);\n"
                                  "#8=ORIENTED_EDGE('',*,*,#7,.T.);\n"
                                  "#9=EDGE_LOOP('',(#8));\n"
                                  "#10=FACE_OUTER_BOUND('',#9,.T.);\n"
                                  "#11=DIRECTION('',(0.,0.,1.));\n"
                                  "#12=DIRECTION('',(1.,0.,0.));\n"
                                  "#13=AXIS2_PLACEMENT_3D('',#1,#11,#12);\n"
                                  "#14=PLANE('',#13);\n"
                                  "#15=ADVANCED_FACE('be',(#10),#14,.T.);\n"
                                  "#16=CLOSED_SHELL('',(#15));\n"
                                  "#17=MANIFOLD_SOLID_BREP('bedge',#16);\n"
                                  "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_bspline_edge_curve() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kBSplineEdge, store);
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "bspline-edge solid: one face");
    if (doc.roots.empty() || doc.roots[0].faces.empty())
        return;
    const ng::LoopN &lp = *doc.roots[0].faces[0]->bounds[0].loop;
    CHECK(!lp.is_poly && lp.edges.size() == 1, "edge loop with one (bspline) edge");
    if (!lp.edges.empty()) {
        auto *bc = dynamic_cast<const ng::BSplineCurve *>(lp.edges[0].geometry.get());
        CHECK(bc && bc->degree == 2 && bc->ctrl.size() == 3, "edge geometry is a degree-2 bspline, 3 ctrl pts");
    }
}

// The triangle solid + a STYLED_ITEM colour chain (RGB 0.2/0.4/0.6 via the usual
// PRESENTATION_STYLE_ASSIGNMENT -> ... -> FILL_AREA_STYLE_COLOUR -> COLOUR_RGB tree) and a
// millimetre LENGTH_UNIT complex record.
static const char *kColouredSolid = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
                                    "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
                                    "#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
                                    "#3=CARTESIAN_POINT('',(0.,1.,0.));\n"
                                    "#11=VERTEX_POINT('',#1);\n"
                                    "#12=VERTEX_POINT('',#2);\n"
                                    "#13=VERTEX_POINT('',#3);\n"
                                    "#41=EDGE_CURVE('',#11,#12,#11,.T.);\n"
                                    "#42=EDGE_CURVE('',#12,#13,#11,.T.);\n"
                                    "#43=EDGE_CURVE('',#13,#11,#11,.T.);\n"
                                    "#51=ORIENTED_EDGE('',*,*,#41,.T.);\n"
                                    "#52=ORIENTED_EDGE('',*,*,#42,.T.);\n"
                                    "#53=ORIENTED_EDGE('',*,*,#43,.T.);\n"
                                    "#60=EDGE_LOOP('',(#51,#52,#53));\n"
                                    "#61=FACE_OUTER_BOUND('',#60,.T.);\n"
                                    "#70=AXIS2_PLACEMENT_3D('',#1,$,$);\n"
                                    "#71=PLANE('',#70);\n"
                                    "#80=ADVANCED_FACE('',(#61),#71,.T.);\n"
                                    "#90=CLOSED_SHELL('',(#80));\n"
                                    "#100=MANIFOLD_SOLID_BREP('solid',#90);\n"
                                    "#101=COLOUR_RGB('',0.2,0.4,0.6);\n"
                                    "#102=FILL_AREA_STYLE_COLOUR('',#101);\n"
                                    "#103=FILL_AREA_STYLE('',(#102));\n"
                                    "#104=SURFACE_STYLE_FILL_AREA(#103);\n"
                                    "#105=SURFACE_SIDE_STYLE('',(#104));\n"
                                    "#106=SURFACE_STYLE_USAGE(.BOTH.,#105);\n"
                                    "#107=PRESENTATION_STYLE_ASSIGNMENT((#106));\n"
                                    "#108=STYLED_ITEM('',(#107),#100);\n"
                                    "#200=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n"
                                    "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_colour_and_units() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kColouredSolid, store);
    CHECK(std::abs(doc.unit_scale - 0.001) < 1e-12, "millimetre length unit -> 0.001");
    CHECK(doc.roots.size() == 1, "one solid root");
    if (doc.roots.empty())
        return;
    const ng::NgeomRoot &r = doc.roots[0];
    CHECK(r.has_color, "STYLED_ITEM colour resolved onto the solid");
    CHECK(std::abs(r.cr - 0.2) < 1e-6 && std::abs(r.cg - 0.4) < 1e-6 && std::abs(r.cb - 0.6) < 1e-6 &&
              std::abs(r.ca - 1.0) < 1e-6,
          "colour rgba from the style tree");
}

// The triangle solid placed at +10x via a CDSR / SHAPE_REPRESENTATION_RELATIONSHIP /
// ITEM_DEFINED_TRANSFORMATION assembly chain (item_1 = identity in the child rep, item_2 = +10x in
// the parent rep -> world = inv(I) @ T(10,0,0)).
static const char *kAssemblySolid =
    "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
    "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
    "#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
    "#3=CARTESIAN_POINT('',(0.,1.,0.));\n"
    "#11=VERTEX_POINT('',#1);\n"
    "#12=VERTEX_POINT('',#2);\n"
    "#13=VERTEX_POINT('',#3);\n"
    "#41=EDGE_CURVE('',#11,#12,#11,.T.);\n"
    "#42=EDGE_CURVE('',#12,#13,#11,.T.);\n"
    "#43=EDGE_CURVE('',#13,#11,#11,.T.);\n"
    "#51=ORIENTED_EDGE('',*,*,#41,.T.);\n"
    "#52=ORIENTED_EDGE('',*,*,#42,.T.);\n"
    "#53=ORIENTED_EDGE('',*,*,#43,.T.);\n"
    "#60=EDGE_LOOP('',(#51,#52,#53));\n"
    "#61=FACE_OUTER_BOUND('',#60,.T.);\n"
    "#70=AXIS2_PLACEMENT_3D('',#1,$,$);\n"
    "#71=PLANE('',#70);\n"
    "#80=ADVANCED_FACE('',(#61),#71,.T.);\n"
    "#90=CLOSED_SHELL('',(#80));\n"
    "#100=MANIFOLD_SOLID_BREP('solid',#90);\n"
    "#200=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#100),$);\n"
    "#201=SHAPE_REPRESENTATION('child',(),$);\n"
    "#220=SHAPE_REPRESENTATION('root',(),$);\n"
    "#202=SHAPE_REPRESENTATION_RELATIONSHIP('','',#200,#201);\n"
    "#210=CARTESIAN_POINT('',(0.,0.,0.));\n"
    "#211=AXIS2_PLACEMENT_3D('',#210,$,$);\n"
    "#212=CARTESIAN_POINT('',(10.,0.,0.));\n"
    "#213=AXIS2_PLACEMENT_3D('',#212,$,$);\n"
    "#214=ITEM_DEFINED_TRANSFORMATION('','',#211,#213);\n"
    "#215=(REPRESENTATION_RELATIONSHIP('','',#201,#220)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#214)"
    "SHAPE_REPRESENTATION_RELATIONSHIP());\n"
    "#216=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#215,$);\n"
    "ENDSEC;\nEND-ISO-10303-21;\n";

static void test_assembly_transform() {
    std::vector<Instance> store;
    ng::NgeomDoc doc = read_step_brep(kAssemblySolid, store);
    CHECK(doc.roots.size() == 1 && doc.roots[0].faces.size() == 1, "placed solid resolves (1 face)");
    if (doc.roots.empty())
        return;
    const auto &t = doc.roots[0].transforms;
    CHECK(t.size() == 1, "one world placement");
    if (t.size() == 1) {
        const auto &m = t[0]; // column-major: translation in cols 12..14, rotation identity
        CHECK(std::abs(m[12] - 10.0f) < 1e-5 && std::abs(m[13]) < 1e-5 && std::abs(m[14]) < 1e-5, "translation = +10x");
        CHECK(std::abs(m[0] - 1.0f) < 1e-5 && std::abs(m[5] - 1.0f) < 1e-5 && std::abs(m[10] - 1.0f) < 1e-5 &&
                  std::abs(m[15] - 1.0f) < 1e-5,
              "rotation block is identity");
    }
}

// Resolve -> encode -> decode round-trip: the re-encoded buffer must decode back to the same
// neutral structure (validates ngeom_encode.h, incl. B-spline knot RLE).
static void test_ngeom_roundtrip() {
    // curved solid: 4 analytic surfaces survive the round-trip with the right types
    {
        std::vector<Instance> store;
        ng::NgeomDoc doc = read_step_brep(kCurvedSolid, store);
        std::vector<uint8_t> buf = ng::encode(doc);
        ng::NgeomDoc doc2 = ng::decode(buf.data(), buf.size());
        CHECK(doc2.roots.size() == 1 && doc2.roots[0].faces.size() == 4, "roundtrip: 4 faces");
        if (!doc2.roots.empty() && doc2.roots[0].faces.size() == 4) {
            auto &fs = doc2.roots[0].faces;
            CHECK(dynamic_cast<const ng::CylinderSurface *>(fs[0]->surface.get()), "roundtrip cylinder");
            CHECK(dynamic_cast<const ng::ConeSurface *>(fs[1]->surface.get()), "roundtrip cone");
            CHECK(dynamic_cast<const ng::SphereSurface *>(fs[2]->surface.get()), "roundtrip sphere");
            CHECK(dynamic_cast<const ng::TorusSurface *>(fs[3]->surface.get()), "roundtrip torus");
        }
    }
    // bspline solid: degree/grid + rational weights survive (knot RLE inverts expand_knots)
    {
        std::vector<Instance> store;
        ng::NgeomDoc doc = read_step_brep(kBSplineSurf, store);
        std::vector<uint8_t> buf = ng::encode(doc);
        ng::NgeomDoc doc2 = ng::decode(buf.data(), buf.size());
        CHECK(doc2.roots.size() == 1 && doc2.roots[0].faces.size() == 2, "roundtrip: 2 bspline faces");
        if (!doc2.roots.empty() && doc2.roots[0].faces.size() == 2) {
            auto *s1 = dynamic_cast<const ng::BSplineSurface *>(doc2.roots[0].faces[0]->surface.get());
            CHECK(s1 && s1->u_degree == 1 && s1->nu == 2 && s1->nv == 2, "roundtrip bspline degree/grid");
            auto *s2 = dynamic_cast<const ng::BSplineSurface *>(doc2.roots[0].faces[1]->surface.get());
            CHECK(s2 && s2->weights.size() == 4 && std::abs(s2->weights[3] - 2.0) < 1e-9, "roundtrip rational weights");
        }
    }
    // conic edge: the CIRCLE edge geometry survives
    {
        std::vector<Instance> store;
        ng::NgeomDoc doc = read_step_brep(kDiskSolid, store);
        std::vector<uint8_t> buf = ng::encode(doc);
        ng::NgeomDoc doc2 = ng::decode(buf.data(), buf.size());
        CHECK(!doc2.roots.empty() && doc2.roots[0].faces.size() == 1, "roundtrip disk face");
        if (!doc2.roots.empty() && !doc2.roots[0].faces.empty()) {
            const ng::LoopN &lp = *doc2.roots[0].faces[0]->bounds[0].loop;
            CHECK(!lp.edges.empty() && dynamic_cast<const ng::CircleCurve *>(lp.edges[0].geometry.get()),
                  "roundtrip circle edge geometry");
        }
    }
}

int main() {
    test_resolve_structure();
    test_tessellate();
    test_curved_surfaces_and_polyloop();
    test_conic_edge();
    test_bspline_surfaces();
    test_bspline_edge_curve();
    test_colour_and_units();
    test_assembly_transform();
    test_ngeom_roundtrip();
    if (g_fail == 0)
        std::printf("step reader: ALL PASS\n");
    else
        std::printf("step reader: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
