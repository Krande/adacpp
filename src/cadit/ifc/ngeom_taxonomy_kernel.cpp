// NGEOM taxonomy kernel driver (Part 2b): neutral -> taxonomy -> AbstractKernel::convert
// (OCC / CGAL) -> ConversionResults -> Triangulate -> Representation::Triangulation -> Mesh.
#include <Eigen/Dense>
#include <ifcgeom/ConversionResult.h>
#include <ifcgeom/ConversionSettings.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom/kernels/cgal/CgalKernel.h>
#include <ifcgeom/kernels/opencascade/OpenCascadeKernel.h>
#include <ifcgeom/taxonomy.h>

#include <memory>

#include "../../geom/neutral/ngeom_math.h"
#include "ngeom_taxonomy.h"

namespace geom = ifcopenshell::geometry;
namespace tax = ifcopenshell::geometry::taxonomy;

namespace adacpp::ngeom {

namespace {

// Append a Triangulation's welded verts/faces/normals into `out` (with index base offset).
void append_triangulation(const IfcGeom::Representation::Triangulation &tri, TessMesh &out) {
    const std::vector<double> &v = tri.verts();
    const std::vector<int> &f = tri.faces();
    const std::vector<double> &n = tri.normals();
    uint32_t base = (uint32_t)(out.positions.size() / 3);
    for (double x : v) out.positions.push_back((float)x);
    if (n.size() == v.size())
        for (double x : n) out.normals.push_back((float)x);
    else
        out.normals.resize(out.positions.size(), 0.0f);  // ifc kernel emitted none; leave zero
    for (int idx : f) out.indices.push_back(base + (uint32_t)idx);
}

void tessellate_faces_taxonomy(const std::vector<std::shared_ptr<FaceSurfaceN>> &faces,
                               const std::string &kernel_name, double deflection, double angular,
                               TessMesh &out) {
    tax::shell::ptr shell = to_taxonomy_shell(faces);
    if (!shell) return;

    geom::Settings settings;
    settings.get<geom::settings::MesherLinearDeflection>().value = deflection > 0 ? deflection : 1e-3;
    settings.get<geom::settings::MesherAngularDeflection>().value = angular;

    std::unique_ptr<geom::kernels::AbstractKernel> kernel;
    if (kernel_name == "cgal")
        kernel.reset(new geom::kernels::CgalKernel(settings));
    else  // "occ" (and "hybrid" falls back to occ here; true hybrid needs an IfcFile)
        kernel.reset(new IfcGeom::OpenCascadeKernel(settings));

    IfcGeom::ConversionResults results;
    try {
        if (!kernel->convert(shell, results)) return;
    } catch (...) {
        return;  // fail-soft: a kernel that can't convert this shell yields no triangles
    }

    std::unique_ptr<IfcGeom::Representation::Triangulation> tri(
        IfcGeom::Representation::Triangulation::empty(settings));
    tax::matrix4 place;  // identity placement
    for (auto &cr : results) {
        try {
            cr.Shape()->Triangulate(settings, place, tri.get(), 0, 0);
        } catch (...) {
        }
    }
    append_triangulation(*tri, out);
}

}  // namespace

TessMesh tessellate_via_taxonomy(const NgeomDoc &doc, const std::string &kernel, double deflection,
                                 double angular_deg) {
    TessMesh mesh;
    double ang = angular_deg * PI / 180.0;
    for (const NgeomRoot &root : doc.roots) {
        uint32_t first = (uint32_t)mesh.indices.size();
        uint32_t vfirst = (uint32_t)(mesh.positions.size() / 3);
        tessellate_faces_taxonomy(root.faces, kernel, deflection, ang, mesh);
        mesh.groups.push_back({root.id, first, (uint32_t)mesh.indices.size() - first, vfirst,
                               (uint32_t)(mesh.positions.size() / 3) - vfirst});
    }
    return mesh;
}

}  // namespace adacpp::ngeom
