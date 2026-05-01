#include "cad_py_wrap.h"
#include "ShapeHandle.h"
#include "../geom/Color.h"
#include "../geom/GroupReference.h"
#include "../geom/Mesh.h"
#include "../geom/MeshType.h"

#ifndef __EMSCRIPTEN__
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#include <stdexcept>
#endif

namespace {

// ----------------------------------------------------------------------------
// make_box
// ----------------------------------------------------------------------------

ShapeHandle make_box_impl(float dx, float dy, float dz) {
#ifdef __EMSCRIPTEN__
    return ShapeHandle(ShapeHandle::Kind::Box, dx, dy, dz);
#else
    const gp_Pnt corner(-dx * 0.5, -dy * 0.5, -dz * 0.5);
    return ShapeHandle(BRepPrimAPI_MakeBox(corner, dx, dy, dz).Shape());
#endif
}

// ----------------------------------------------------------------------------
// tessellate
// ----------------------------------------------------------------------------

#ifdef __EMSCRIPTEN__

// Wasm stub: dispatches on the kind tag. Will be replaced once a real wasm
// kernel is wired in. ``linear_deflection`` is accepted for API parity but
// unused — the stub is hand-built and exact.
Mesh tessellate_impl(const ShapeHandle &sh, double /*linear_deflection*/) {
    if (sh.kind() == ShapeHandle::Kind::Box) {
        const float x = sh.dx() * 0.5f;
        const float y = sh.dy() * 0.5f;
        const float z = sh.dz() * 0.5f;
        std::vector<float> positions = {
            -x, -y, -z,   x, -y, -z,   x,  y, -z,  -x,  y, -z,
            -x, -y,  z,   x, -y,  z,   x,  y,  z,  -x,  y,  z,
        };
        std::vector<uint32_t> faces = {
            0, 1, 2,  0, 2, 3,
            4, 6, 5,  4, 7, 6,
            0, 4, 5,  0, 5, 1,
            3, 2, 6,  3, 6, 7,
            0, 3, 7,  0, 7, 4,
            1, 5, 6,  1, 6, 2,
        };
        return Mesh(0, std::move(positions), std::move(faces));
    }
    return Mesh(0, {}, {});
}

#else

Mesh tessellate_impl(const ShapeHandle &sh, double linear_deflection) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("tessellate: ShapeHandle is null");
    }

    // Auto deflection: a fraction of the bbox diagonal keeps tessellation tight
    // on small shapes without exploding triangle counts on large ones.
    if (linear_deflection <= 0.0) {
        Bnd_Box bb;
        BRepBndLib::Add(shape, bb);
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double dx = xmax - xmin;
        const double dy = ymax - ymin;
        const double dz = zmax - zmin;
        linear_deflection = std::sqrt(dx * dx + dy * dy + dz * dz) * 0.05;
    }
    BRepMesh_IncrementalMesh(shape, linear_deflection);

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

// Convenience: build a primitive box and tessellate it in one call. Same logic
// as make_box() + tessellate(), kept as a single entry point for callers that
// don't need the intermediate handle.
Mesh tessellate_box_impl(float dx, float dy, float dz) {
    return tessellate_impl(make_box_impl(dx, dy, dz), -1.0);
}

} // namespace

void cad_module(nb::module_ &m) {
    // Kernel-agnostic mesh / color / group types live in cad — they're the
    // surface every backend (native OCCT, wasm stub, future CGAL) speaks.
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

    // Opaque handle: no readable attributes / methods. Callers obtain instances
    // via factory functions (make_box, ...) and pass them to consumers
    // (tessellate, ...). The C++-level shape data is unreachable from Python.
    nb::class_<ShapeHandle>(m, "ShapeHandle");

    m.def("make_box", &make_box_impl,
          "dx"_a, "dy"_a, "dz"_a,
          "Create a centered axis-aligned box ShapeHandle.");

    m.def("tessellate", &tessellate_impl,
          "shape"_a, "linear_deflection"_a = -1.0,
          "Tessellate a shape into a triangle Mesh. "
          "linear_deflection<=0 selects a heuristic based on the shape's bbox.");

    m.def("tessellate_box", &tessellate_box_impl,
          "dx"_a, "dy"_a, "dz"_a,
          "Convenience: build a box and tessellate it in one call.");
}
