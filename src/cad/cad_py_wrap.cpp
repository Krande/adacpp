#include "cad_py_wrap.h"
#include "../geom/Color.h"
#include "../geom/GroupReference.h"
#include "../geom/Mesh.h"
#include "../geom/MeshType.h"

#ifndef __EMSCRIPTEN__
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#endif

namespace {

#ifdef __EMSCRIPTEN__

// Stub used while OCCT-wasm is not yet available. Returns a hand-built unit-box
// mesh — same shape semantics as the OCCT path (centered axis-aligned box) so
// callers can write backend-agnostic tests against the AABB / triangle count.
Mesh tessellate_box_impl(float dx, float dy, float dz) {
    const float x = dx * 0.5f;
    const float y = dy * 0.5f;
    const float z = dz * 0.5f;

    std::vector<float> positions = {
        -x, -y, -z,   x, -y, -z,   x,  y, -z,  -x,  y, -z,
        -x, -y,  z,   x, -y,  z,   x,  y,  z,  -x,  y,  z,
    };
    std::vector<uint32_t> faces = {
        0, 1, 2,  0, 2, 3,   // -Z
        4, 6, 5,  4, 7, 6,   // +Z
        0, 4, 5,  0, 5, 1,   // -Y
        3, 2, 6,  3, 6, 7,   // +Y
        0, 3, 7,  0, 7, 4,   // -X
        1, 5, 6,  1, 6, 2,   // +X
    };
    return Mesh(0, std::move(positions), std::move(faces));
}

#else

// Native (OCCT-backed) tessellation. Builds a centered box via BRepPrimAPI_MakeBox,
// triangulates with BRepMesh_IncrementalMesh, then walks the per-face triangulations
// and concatenates them into one flat positions/indices buffer.
Mesh tessellate_box_impl(float dx, float dy, float dz) {
    const gp_Pnt corner(-dx * 0.5, -dy * 0.5, -dz * 0.5);
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(corner, dx, dy, dz).Shape();

    // Linear deflection: a fraction of the bbox diagonal keeps the mesh tight on
    // small boxes without exploding triangle counts on large ones.
    const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    BRepMesh_IncrementalMesh(shape, diag * 0.05);

    std::vector<float> positions;
    std::vector<uint32_t> indices;

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        const uint32_t base = static_cast<uint32_t>(positions.size() / 3);
        const gp_Trsf trsf = loc.Transformation();
        for (Standard_Integer i = 1; i <= tri->NbNodes(); ++i) {
            const gp_Pnt p = tri->Node(i).Transformed(trsf);
            positions.push_back(static_cast<float>(p.X()));
            positions.push_back(static_cast<float>(p.Y()));
            positions.push_back(static_cast<float>(p.Z()));
        }
        for (Standard_Integer i = 1; i <= tri->NbTriangles(); ++i) {
            Standard_Integer n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            indices.push_back(base + static_cast<uint32_t>(n1 - 1));
            indices.push_back(base + static_cast<uint32_t>(n2 - 1));
            indices.push_back(base + static_cast<uint32_t>(n3 - 1));
        }
    }

    return Mesh(0, std::move(positions), std::move(indices));
}

#endif

} // namespace

void cad_module(nb::module_ &m) {
    // Kernel-agnostic mesh / color / group types live in cad — they're the
    // surface every backend (native OCCT, wasm stub, future CGAL) speaks.
    // adacpp.visit re-exports them for back-compat with existing call sites.
    nb::enum_<MeshType>(m, "MeshType")
            .value("POINTS",         MeshType::POINTS)
            .value("LINES",          MeshType::LINES)
            .value("LINE_LOOP",      MeshType::LINE_LOOP)
            .value("LINE_STRIP",     MeshType::LINE_STRIP)
            .value("TRIANGLES",      MeshType::TRIANGLES)
            .value("TRIANGLE_STRIP", MeshType::TRIANGLE_STRIP)
            .value("TRIANGLE_FAN",   MeshType::TRIANGLE_FAN);

    nb::class_<Color>(m, "Color")
            .def_rw("r", &Color::r)
            .def_rw("g", &Color::g)
            .def_rw("b", &Color::b)
            .def_rw("a", &Color::a);

    nb::class_<GroupReference>(m, "GroupReference")
            .def_ro("node_id", &GroupReference::node_id)
            .def_ro("start",   &GroupReference::start)
            .def_ro("length",  &GroupReference::length);

    nb::class_<Mesh>(m, "Mesh")
            .def_ro("id",        &Mesh::id)
            .def_ro("positions", &Mesh::positions)
            .def_ro("indices",   &Mesh::indices)
            .def_ro("normals",   &Mesh::normals)
            .def_ro("edges",     &Mesh::edges)
            .def_ro("mesh_type", &Mesh::mesh_type)
            .def_ro("color",     &Mesh::color)
            .def_ro("groups",    &Mesh::group_reference);

    m.def("tessellate_box", &tessellate_box_impl,
          "dx"_a, "dy"_a, "dz"_a,
          "Return a tessellated axis-aligned box centered at the origin.");
}
