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

#include <CGAL/number_utils.h>

#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <memory>
#include <vector>

#include "../../geom/neutral/ngeom_math.h"
#include "ngeom_taxonomy.h"

namespace geom = ifcopenshell::geometry;

namespace adacpp::ngeom {

namespace {

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
// Facets may be n-gons (fan-triangulated); vertices are unwelded per facet.
void append_cgal_shape(const cgal_shape_t &shape, TessMesh &out) {
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

TessMesh tessellate_via_taxonomy(const NgeomDoc &doc, const std::string &kernel_name, double deflection,
                                 double angular_deg) {
    (void)angular_deg;  // the kernels mesh from their own settings
    TessMesh mesh;
    geom::Settings settings;
    const bool use_cgal = (kernel_name == "cgal");
    std::unique_ptr<IfcGeom::OpenCascadeKernel> occ;
    std::unique_ptr<geom::kernels::CgalKernel> cgal;
    if (use_cgal)
        cgal.reset(new geom::kernels::CgalKernel(settings));
    else  // "occ" / "hybrid" (hybrid -> occ for now)
        occ.reset(new IfcGeom::OpenCascadeKernel(settings));

    for (const NgeomRoot &root : doc.roots) {
        uint32_t first = (uint32_t)mesh.indices.size();
        uint32_t vfirst = (uint32_t)(mesh.positions.size() / 3);
        auto shell = to_taxonomy_shell(root.faces);
        if (shell) {
            try {
                if (use_cgal) {
                    cgal_shape_t shape;
                    if (cgal->convert(shell, shape)) append_cgal_shape(shape, mesh);
                } else {
                    TopoDS_Shape shape;
                    if (occ->convert(shell, shape)) append_occ_shape(shape, deflection, mesh);
                }
            } catch (...) {
                // fail-soft: a shell the kernel can't build yields no triangles for this root
            }
        }
        mesh.groups.push_back({root.id, first, (uint32_t)mesh.indices.size() - first, vfirst,
                               (uint32_t)(mesh.positions.size() / 3) - vfirst});
    }
    return mesh;
}

}  // namespace adacpp::ngeom
