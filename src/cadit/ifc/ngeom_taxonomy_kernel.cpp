// NGEOM taxonomy kernel driver (Part 2b): neutral -> taxonomy -> IfcOpenShell OCC kernel
// -> TopoDS_Shape -> BRepMesh -> Mesh.
//
// We call OpenCascadeKernel::convert(shell, TopoDS_Shape&) DIRECTLY rather than the
// high-level AbstractKernel::convert(...)->ConversionResults path: the latter's
// convert_impl(shell) dereferences shell->instance->...->id(), and our taxonomy items are
// built programmatically (schema-free) so `instance` is null -> segfault. The direct method
// runs IfcOpenShell's full face/wire conversion + healing and never touches `instance`.
#include <Eigen/Dense>
#include <ifcgeom/ConversionSettings.h>
#include <ifcgeom/kernels/cgal/CgalKernel.h>
#include <ifcgeom/kernels/opencascade/OpenCascadeKernel.h>
#include <ifcgeom/taxonomy.h>

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/number_utils.h>

#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <memory>
#include <vector>

#include "../../geom/neutral/ngeom_math.h"
#include "ngeom_taxonomy.h"

namespace geom = ifcopenshell::geometry;

namespace adacpp::ngeom {

namespace {

// gp_Trsf placing local coordinates into world via a Frame (columns x/y/z + origin).
gp_Trsf trsf_from_frame(const Frame &f) {
    gp_Trsf t;
    t.SetValues(f.x.x, f.y.x, f.z.x, f.o.x, f.x.y, f.y.y, f.z.y, f.o.y, f.x.z, f.y.z, f.z.z, f.o.z);
    return t;
}

// Place mesh vertices [v_first..end) from local into world by frame F (positions translated +
// rotated, normals rotated only). Used for cgal extrusions, where convert() ignores the matrix
// and there is no cheap cgal-shape transform.
void place_positions(TessMesh &mesh, size_t v_first, const Frame &F) {
    size_t nv = mesh.positions.size() / 3;
    bool has_n = mesh.normals.size() >= mesh.positions.size();
    for (size_t v = v_first; v < nv; ++v) {
        float *p = &mesh.positions[v * 3];
        Vec3 w = F.to_world(p[0], p[1], p[2]);
        p[0] = (float)w.x;
        p[1] = (float)w.y;
        p[2] = (float)w.z;
        if (has_n) {
            float *q = &mesh.normals[v * 3];
            Vec3 n = F.x * q[0] + F.y * q[1] + F.z * q[2];  // rotate only
            q[0] = (float)n.x;
            q[1] = (float)n.y;
            q[2] = (float)n.z;
        }
    }
}

// BRepMesh a healed OCC shape and append its triangles + smooth per-vertex normals to `out`.
void append_occ_shape(const TopoDS_Shape &shape, double deflection, TessMesh &out) {
    if (shape.IsNull()) return;
    if (deflection <= 0) {
        Bnd_Box box;
        BRepBndLib::Add(shape, box);
        if (!box.IsVoid()) {
            double xm, ym, zm, xM, yM, zM;
            box.Get(xm, ym, zm, xM, yM, zM);
            double diag = std::sqrt((xM - xm) * (xM - xm) + (yM - ym) * (yM - ym) + (zM - zm) * (zM - zm));
            deflection = diag > 0 ? diag * 0.01 : 0.1;
        } else {
            deflection = 0.1;
        }
    }
    BRepMesh_IncrementalMesh(shape, deflection, Standard_False, 0.5, Standard_True);

    uint32_t base = (uint32_t)(out.positions.size() / 3);
    size_t normals_start = out.normals.size();
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face &face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull() || tri->NbNodes() == 0) continue;
        const gp_Trsf &trsf = loc.Transformation();
        bool rev = face.Orientation() == TopAbs_REVERSED;
        uint32_t fbase = (uint32_t)(out.positions.size() / 3);
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i).Transformed(trsf);
            out.positions.push_back((float)p.X());
            out.positions.push_back((float)p.Y());
            out.positions.push_back((float)p.Z());
            out.normals.push_back(0.0f);
            out.normals.push_back(0.0f);
            out.normals.push_back(0.0f);
        }
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int a, b, c;
            tri->Triangle(i).Get(a, b, c);
            if (rev) std::swap(a, c);
            uint32_t ia = fbase + (a - 1), ib = fbase + (b - 1), ic = fbase + (c - 1);
            out.indices.push_back(ia);
            out.indices.push_back(ib);
            out.indices.push_back(ic);
            // accumulate the triangle normal onto its vertices (smooth shading)
            auto P = [&](uint32_t k) {
                return Vec3{out.positions[k * 3], out.positions[k * 3 + 1], out.positions[k * 3 + 2]};
            };
            Vec3 n = (P(ib) - P(ia)).cross(P(ic) - P(ia));
            for (uint32_t k : {ia, ib, ic}) {
                out.normals[k * 3] += (float)n.x;
                out.normals[k * 3 + 1] += (float)n.y;
                out.normals[k * 3 + 2] += (float)n.z;
            }
        }
    }
    // normalize accumulated normals
    for (size_t k = normals_start; k < out.normals.size(); k += 3) {
        double nx = out.normals[k], ny = out.normals[k + 1], nz = out.normals[k + 2];
        double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-20) {
            out.normals[k] = (float)(nx / len);
            out.normals[k + 1] = (float)(ny / len);
            out.normals[k + 2] = (float)(nz / len);
        }
    }
    (void)base;
}

// Extract triangles + per-facet normals from a CGAL polyhedron (exact coords -> double).
// Taken by value so we can triangulate non-convex n-gon facets first: a CGAL extrusion's
// end cap is the (possibly concave) profile as ONE facet — a naive fan over a concave
// polygon spills outside it (the beam-end artifacts). PMP::triangulate_faces produces a
// proper triangulation; after it every facet is a triangle and the loop below is exact.
void append_cgal_shape(cgal_shape_t shape, TessMesh &out) {
    try {
        CGAL::Polygon_mesh_processing::triangulate_faces(shape);
    } catch (...) {
        // fall back to the fan extraction below if triangulation fails
    }
    for (auto f = shape.facets_begin(); f != shape.facets_end(); ++f) {
        std::vector<Vec3> poly;
        auto h = f->facet_begin();
        do {
            const auto &p = h->vertex()->point();
            poly.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
        } while (++h != f->facet_begin());
        if (poly.size() < 3) continue;
        Vec3 n = (poly[1] - poly[0]).cross(poly[2] - poly[0]).normalized();
        for (size_t i = 1; i + 1 < poly.size(); ++i) {
            for (const Vec3 &v : {poly[0], poly[i], poly[i + 1]}) {
                uint32_t idx = (uint32_t)(out.positions.size() / 3);
                out.positions.push_back((float)v.x);
                out.positions.push_back((float)v.y);
                out.positions.push_back((float)v.z);
                out.normals.push_back((float)n.x);
                out.normals.push_back((float)n.y);
                out.normals.push_back((float)n.z);
                out.indices.push_back(idx);
            }
        }
    }
}

}  // namespace

namespace {
bool parse_bool(const std::string &s) {
    return s == "1" || s == "true" || s == "True" || s == "TRUE" || s == "on" || s == "yes";
}
std::string variant_to_string(const geom::Settings::value_variant_t &v) {
    if (const bool *p = boost::get<bool>(&v)) return *p ? "true" : "false";
    if (const int *p = boost::get<int>(&v)) return std::to_string(*p);
    if (const double *p = boost::get<double>(&v)) return std::to_string(*p);
    if (const std::string *p = boost::get<std::string>(&v)) return *p;
    return "";  // non-scalar (set/vector/enum) — not exposed as a simple value
}
// Apply one (name, string-value) override to the kernel settings, parsing the
// string per the setting's declared type. Unknown setting / bad value -> skip.
void apply_setting(geom::Settings &s, const std::string &name, const std::string &val) {
    std::string ty;
    try {
        ty = s.get_type(name);
    } catch (...) {
        return;  // not a known setting
    }
    try {
        if (ty == "bool")
            s.set(name, parse_bool(val));
        else if (ty == "int")
            s.set(name, (int)std::stol(val));
        else if (ty == "double")
            s.set(name, std::stod(val));
        else if (ty == "std::string")
            s.set(name, val);
        // set<>/vector<>/enum settings are not exposed via the simple string API
    } catch (...) {
        // malformed value for the type -> leave the default
    }
}

// --- solid builders (used by revolve + boolean; both bypass ifcopenshell's
// convert_impl, which derefs a null schema instance on our programmatic items) ---
TopoDS_Shape revolve_to_occ(IfcGeom::OpenCascadeKernel &occ, const RevolveN &rv) {
    auto sh = to_taxonomy_shell({rv.profile});
    if (!sh || sh->children.empty()) return {};
    TopoDS_Shape topo_face;
    if (!occ.convert(sh->children[0], topo_face) || topo_face.IsNull()) return {};
    gp_Ax1 ax(gp_Pnt(rv.axis_origin.x, rv.axis_origin.y, rv.axis_origin.z),
              gp_Dir(rv.axis_dir.x, rv.axis_dir.y, rv.axis_dir.z));
    TopoDS_Shape revolved = rv.angle != 0.0 ? BRepPrimAPI_MakeRevol(topo_face, ax, rv.angle).Shape()
                                            : BRepPrimAPI_MakeRevol(topo_face, ax).Shape();
    return BRepBuilderAPI_Transform(revolved, trsf_from_frame(rv.frame), Standard_True).Shape();
}

// convert(extrusion) returns the profile extruded in LOCAL coords -- ifcopenshell applies the
// placement matrix downstream (in the ConversionResult flow we bypass), so place it here.
TopoDS_Shape extrusion_to_occ(IfcGeom::OpenCascadeKernel &occ, const ExtrusionN &ex) {
    auto et = to_taxonomy_extrusion(ex);
    if (!et) return {};
    TopoDS_Shape shape;
    if (!occ.convert(et, shape) || shape.IsNull()) return {};
    return BRepBuilderAPI_Transform(shape, trsf_from_frame(ex.frame), Standard_True).Shape();
}

TopoDS_Shape build_solid_occ(IfcGeom::OpenCascadeKernel &occ, const SolidItemN &it);

TopoDS_Shape boolean_to_occ(IfcGeom::OpenCascadeKernel &occ, const BooleanN &bn) {
    TopoDS_Shape a = build_solid_occ(occ, bn.a);
    if (a.IsNull()) return {};
    TopoDS_Shape b = build_solid_occ(occ, bn.b);
    if (b.IsNull()) return a;  // nothing to combine with -> first operand
    try {
        switch (bn.op) {
            case 1:
                return BRepAlgoAPI_Fuse(a, b).Shape();
            case 2:
                return BRepAlgoAPI_Common(a, b).Shape();
            default:
                return BRepAlgoAPI_Cut(a, b).Shape();  // 0 = difference
        }
    } catch (...) {
        return a;
    }
}

TopoDS_Shape build_solid_occ(IfcGeom::OpenCascadeKernel &occ, const SolidItemN &it) {
    if (it.boolean) return boolean_to_occ(occ, *it.boolean);
    if (it.revolve) return revolve_to_occ(occ, *it.revolve);
    if (it.extrusion) return extrusion_to_occ(occ, *it.extrusion);
    if (!it.faces.empty()) {
        auto sh = to_taxonomy_shell(it.faces);
        TopoDS_Shape s;
        if (sh && occ.convert(sh, s)) return s;
    }
    return {};
}
}  // namespace

std::vector<TaxonomySetting> taxonomy_settings_info() {
    geom::Settings s;
    std::vector<TaxonomySetting> out;
    for (const std::string &name : s.setting_names()) {
        TaxonomySetting info;
        info.name = name;
        try {
            info.type = s.get_type(name);
        } catch (...) {
            info.type = "?";
        }
        try {
            info.default_value = variant_to_string(s.get(name));
        } catch (...) {
        }
        out.push_back(info);
    }
    return out;
}

TessMesh tessellate_via_taxonomy(const NgeomDoc &doc, const std::string &kernel_name, double deflection,
                                 double angular_deg,
                                 const std::vector<std::pair<std::string, std::string>> &overrides) {
    (void)angular_deg;  // the kernels mesh from their own settings
    TessMesh mesh;
    geom::Settings settings;
    // Default: skip the wire self-intersection check. A live gdb profile showed
    // IfcGeom::util::wire_intersections (recursive 2D curve-curve global
    // optimization) dominating ~2/3 of taxonomy time, and our taxonomy shells
    // come from already-validated solids. Callers can re-enable / tune any
    // ifcopenshell setting via `overrides` (e.g. {"precision", "1e-3"}).
    settings.get<geom::settings::NoWireIntersectionCheck>().value = true;
    for (const auto &kv : overrides) apply_setting(settings, kv.first, kv.second);
    const bool use_cgal = (kernel_name == "cgal");
    std::unique_ptr<IfcGeom::OpenCascadeKernel> occ;
    std::unique_ptr<geom::kernels::CgalKernel> cgal;
    // OCC kernel is always created: revolve uses it (MakeRevol + profile-face
    // build) even in cgal mode. The cgal kernel is created additionally for the
    // cgal pipeline (faces/extrusions).
    occ.reset(new IfcGeom::OpenCascadeKernel(settings));
    if (use_cgal) cgal.reset(new geom::kernels::CgalKernel(settings));

    for (const NgeomRoot &root : doc.roots) {
        uint32_t first = (uint32_t)mesh.indices.size();
        uint32_t vfirst = (uint32_t)(mesh.positions.size() / 3);
        try {
            if (root.extrusion) {
                // Swept solid. convert() yields the extrusion in LOCAL coords (the kernels
                // ignore the placement matrix), so place it: occ via a shape transform,
                // cgal via a vertex transform on the emitted mesh.
                if (use_cgal) {
                    size_t vb = mesh.positions.size() / 3;
                    auto ext = to_taxonomy_extrusion(*root.extrusion);
                    cgal_shape_t shape;
                    if (ext && cgal->convert(ext, shape)) append_cgal_shape(shape, mesh);
                    place_positions(mesh, vb, root.extrusion->frame);
                } else {
                    TopoDS_Shape shape = extrusion_to_occ(*occ, *root.extrusion);
                    if (!shape.IsNull()) append_occ_shape(shape, deflection, mesh);
                }
            } else if (root.revolve) {
                // Revolved solid via OCC MakeRevol (ifcopenshell's revolve convert
                // derefs a null schema instance -> segfault on our items).
                TopoDS_Shape shape = revolve_to_occ(*occ, *root.revolve);
                if (!shape.IsNull()) append_occ_shape(shape, deflection, mesh);
            } else if (root.boolean) {
                // CSG boolean: build each operand's TopoDS_Shape and apply
                // BRepAlgoAPI directly (same null-instance reason as revolve).
                TopoDS_Shape shape = boolean_to_occ(*occ, *root.boolean);
                if (!shape.IsNull()) append_occ_shape(shape, deflection, mesh);
            } else if (auto shell = to_taxonomy_shell(root.faces)) {
                if (use_cgal) {
                    cgal_shape_t shape;
                    if (cgal->convert(shell, shape)) append_cgal_shape(shape, mesh);
                } else {
                    TopoDS_Shape shape;
                    if (occ->convert(shell, shape)) append_occ_shape(shape, deflection, mesh);
                }
            }
        } catch (...) {
            // fail-soft: a root the kernel can't build yields no triangles
        }
        mesh.groups.push_back({root.id, first, (uint32_t)mesh.indices.size() - first, vfirst,
                               (uint32_t)(mesh.positions.size() / 3) - vfirst});
    }
    return mesh;
}

}  // namespace adacpp::ngeom
