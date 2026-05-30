#include "cad_py_wrap.h"
#include "ShapeHandle.h"
#include "../geom/Color.h"
#include "../geom/GroupReference.h"
#include "../geom/Mesh.h"
#include "../geom/MeshType.h"

#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <array>
#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>      // mkstemp, unlink, write, close
#include <utility>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_Triangulation.hxx>
#include <RWGltf_CafWriter.hxx>
#include <RWGltf_WriterTrsfFormat.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <STEPControl_Reader.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace {

// ----------------------------------------------------------------------------
// Primitive factories — single OCCT-backed code path on native + wasm
// ----------------------------------------------------------------------------

ShapeHandle make_box_impl(float dx, float dy, float dz) {
    const gp_Pnt corner(-dx * 0.5, -dy * 0.5, -dz * 0.5);
    return ShapeHandle(BRepPrimAPI_MakeBox(corner, dx, dy, dz).Shape());
}

ShapeHandle make_cylinder_impl(float radius, float height) {
    return ShapeHandle(BRepPrimAPI_MakeCylinder(radius, height).Shape());
}

ShapeHandle make_sphere_impl(float radius) {
    return ShapeHandle(BRepPrimAPI_MakeSphere(radius).Shape());
}

// ----------------------------------------------------------------------------
// tessellate
// ----------------------------------------------------------------------------

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

// Convenience: build a primitive box and tessellate it in one call. Same logic
// as make_box() + tessellate(), kept as a single entry point for callers that
// don't need the intermediate handle.
Mesh tessellate_box_impl(float dx, float dy, float dz) {
    return tessellate_impl(make_box_impl(dx, dy, dz), -1.0);
}

// ----------------------------------------------------------------------------
// pyocc bridge — wrap an existing OCCT TopoDS_Shape (typically created in
// pythonocc-core or another nanobind-bound module) into an adacpp.cad
// ShapeHandle. Pointer must point at a valid C++ TopoDS_Shape; we copy the
// shape (cheap, refcounted value type), so source lifetime is irrelevant
// after this call. Available on both targets — only callers with access to
// a TopoDS_Shape pointer can use it (pythonocc-core on native; potentially
// other adacpp-wasm modules in the future).
// ----------------------------------------------------------------------------

ShapeHandle from_topods_pointer_impl(uintptr_t ptr) {
    if (ptr == 0) {
        throw std::runtime_error("from_topods_pointer: null pointer");
    }
    const TopoDS_Shape *shape = reinterpret_cast<const TopoDS_Shape *>(ptr);
    return ShapeHandle(*shape);
}

// ----------------------------------------------------------------------------
// bbox: axis-aligned bounding box query
// ----------------------------------------------------------------------------

// Returns {xmin, ymin, zmin, xmax, ymax, zmax} — same order as OCCT's
// Bnd_Box::Get and as adapy.occ.utils.get_boundingbox, so this slots in as
// a drop-in replacement for callers using either side.
std::array<double, 6> bbox_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("bbox: ShapeHandle is null");
    }
    // AddOptimal uses geometric extents (BSpline/B-rep aware) for a tight
    // bbox; default Add inflates by shape tolerance (~1e-7) which would
    // surprise callers querying a primitive's natural bbox.
    //
    // useTriangulation=False forces the analytic path — without this, OCCT
    // returns the *mesh* bbox if a triangulation is already cached on the
    // shape, which jitters ±1e-7 for box and ±0.1 for sphere/cylinder
    // depending on tessellation deflection. Callers asking for `bbox(shape)`
    // expect geometric extents, not mesh extents.
    Bnd_Box bb;
    BRepBndLib::AddOptimal(shape, bb,
                           /*useTriangulation=*/Standard_False,
                           /*useShapeTolerance=*/Standard_False);
    if (bb.IsVoid()) {
        throw std::runtime_error("bbox: empty bounding box (shape has no geometry)");
    }
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return {xmin, ymin, zmin, xmax, ymax, zmax};
}

// ----------------------------------------------------------------------------
// STEP read / glTF write — temp-file roundtrip
// ----------------------------------------------------------------------------
//
// OCCT's STEP/glTF readers/writers operate on filenames, not memory streams.
// To present a `bytes -> ShapeHandle` / `ShapeHandle -> bytes` API, we write
// to a temp file under /tmp and let OCCT read/write through that. Under
// pyodide /tmp is MEMFS (in-memory), so this is purely a memcpy detour with
// no real disk I/O — same speed as a memory-stream API would give.

ShapeHandle read_step_bytes_impl(nb::bytes data) {
    char tmpname[] = "/tmp/adacpp_step_XXXXXX";
    const int fd = mkstemp(tmpname);
    if (fd < 0) {
        throw std::runtime_error("read_step_bytes: mkstemp failed");
    }
    const auto written = ::write(fd, data.c_str(), data.size());
    ::close(fd);
    if (written < 0 || static_cast<std::size_t>(written) != data.size()) {
        ::unlink(tmpname);
        throw std::runtime_error("read_step_bytes: failed to materialize temp file");
    }

    STEPControl_Reader reader;
    const IFSelect_ReturnStatus status = reader.ReadFile(tmpname);
    ::unlink(tmpname);
    if (status != IFSelect_RetDone) {
        throw std::runtime_error("read_step_bytes: STEPControl_Reader could not parse the input");
    }
    reader.TransferRoots();
    const TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw std::runtime_error("read_step_bytes: no transferable shape (empty STEP?)");
    }
    return ShapeHandle(shape);
}

nb::bytes write_glb_bytes_impl(const ShapeHandle &sh, double linear_deflection) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("write_glb_bytes: ShapeHandle is null");
    }
    if (linear_deflection <= 0.0) linear_deflection = 0.1;

    // RWGltf needs a triangulation per face; mesh in-place on the shape.
    BRepMesh_IncrementalMesh(shape, linear_deflection,
                              /*relative=*/Standard_False,
                              /*angular=*/0.5,
                              /*parallel=*/Standard_True);

    // Wrap the shape in a CAF document — RWGltf_CafWriter consumes one.
    Handle(TDocStd_Document) doc =
        new TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"));
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    shapeTool->AddShape(shape);

    char tmpname[] = "/tmp/adacpp_glb_XXXXXX";
    const int fd = mkstemp(tmpname);
    if (fd < 0) {
        throw std::runtime_error("write_glb_bytes: mkstemp failed");
    }
    ::close(fd);  // RWGltf_CafWriter opens the file itself by name.

    {
        RWGltf_CafWriter writer(TCollection_AsciiString(tmpname),
                                 /*isBinary=*/Standard_True);
        // Match adapy's gltf_writer.to_gltf() configuration so the produced
        // GLB renders identically in the adapy viewer:
        //   - Z-up source coordinate system (CAD convention) — RWGltf
        //     internally rotates to glTF's Y-up runtime convention so the
        //     viewer doesn't see sideways/upside-down models.
        //   - Compact node transforms: smaller JSON, what the viewer expects.
        writer.ChangeCoordinateSystemConverter()
            .SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
        writer.SetTransformationFormat(RWGltf_WriterTrsfFormat_Compact);

        TColStd_IndexedDataMapOfStringString fileInfo;
        fileInfo.Add(TCollection_AsciiString("Authors"),
                     TCollection_AsciiString("adacpp"));
        const Message_ProgressRange progress;
        if (!writer.Perform(doc, fileInfo, progress)) {
            ::unlink(tmpname);
            throw std::runtime_error("write_glb_bytes: RWGltf_CafWriter::Perform failed");
        }
    }

    std::ifstream f(tmpname, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        ::unlink(tmpname);
        throw std::runtime_error("write_glb_bytes: failed to re-open temp file");
    }
    const auto size = static_cast<std::size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (size > 0) f.read(buffer.data(), size);
    f.close();
    ::unlink(tmpname);

    return nb::bytes(buffer.data(), buffer.size());
}

// ----------------------------------------------------------------------------
// Shape-algebra verbs — mirror adapy's OccBackend (ada/cad/__init__.py) so an
// adacpp/pythonocc backend swap is behaviour-identical. op strings match
// ada.geom.booleans.BoolOpEnum values ("UNION"/"INTERSECTION"/"DIFFERENCE").
// ----------------------------------------------------------------------------

ShapeHandle boolean_impl(const std::string &op, const ShapeHandle &a, const ShapeHandle &b) {
    const TopoDS_Shape &sa = a.topods();
    const TopoDS_Shape &sb = b.topods();
    if (op == "DIFFERENCE")   return ShapeHandle(BRepAlgoAPI_Cut(sa, sb).Shape());
    if (op == "UNION")        return ShapeHandle(BRepAlgoAPI_Fuse(sa, sb).Shape());
    if (op == "INTERSECTION") return ShapeHandle(BRepAlgoAPI_Common(sa, sb).Shape());
    throw std::runtime_error("boolean: unknown op '" + op + "'");
}

// m = the top 3 rows of a 4x4 affine matrix, row-major (12 doubles). The
// implicit bottom row is [0,0,0,1] — same convention as gp_Trsf::SetValues and
// adapy's OccBackend.transform. Lossless for rigid + uniform-scale transforms.
ShapeHandle transform_impl(const ShapeHandle &sh, const std::array<double, 12> &m, bool copy) {
    gp_Trsf trsf;
    trsf.SetValues(m[0], m[1], m[2], m[3],
                   m[4], m[5], m[6], m[7],
                   m[8], m[9], m[10], m[11]);
    return ShapeHandle(BRepBuilderAPI_Transform(sh.topods(), trsf, copy).Shape());
}

double distance_impl(const ShapeHandle &a, const ShapeHandle &b) {
    BRepExtrema_DistShapeShape dss(a.topods(), b.topods());
    if (!dss.IsDone()) {
        throw std::runtime_error("distance: BRepExtrema_DistShapeShape failed");
    }
    return dss.Value();
}

std::string serialize_impl(const ShapeHandle &sh) {
    TopoDS_Shape shape = sh.topods();
    BRepTools::Clean(shape);  // drop cached triangulation → geometry-only string
    std::ostringstream oss;
    BRepTools::Write(shape, oss);
    return oss.str();
}

bool is_valid_impl(const ShapeHandle &sh) {
    return BRepCheck_Analyzer(sh.topods(), Standard_True).IsValid();
}

// Whole list of face sub-shapes — boundary crosses once, not per face.
std::vector<ShapeHandle> faces_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_FACE); exp.More(); exp.Next()) {
        out.emplace_back(exp.Current());
    }
    return out;
}

// Every (unique) vertex coordinate as one list — the per-vertex loop stays in
// the backend. MapShapes deduplicates shared vertices (matches pythonocc's
// TopologyExplorer.vertices()).
std::vector<std::array<double, 3>> vertex_points_impl(const ShapeHandle &sh) {
    std::vector<std::array<double, 3>> out;
    TopTools_IndexedMapOfShape vmap;
    TopExp::MapShapes(sh.topods(), TopAbs_VERTEX, vmap);
    for (Standard_Integer i = 1; i <= vmap.Extent(); ++i) {
        const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vmap(i)));
        out.push_back({p.X(), p.Y(), p.Z()});
    }
    return out;
}

// Planar face → (origin, normal); std::nullopt (→ Python None) if non-planar.
std::optional<std::pair<std::array<double, 3>, std::array<double, 3>>>
face_plane_impl(const ShapeHandle &face_sh) {
    const TopoDS_Shape &s = face_sh.topods();
    if (s.IsNull() || s.ShapeType() != TopAbs_FACE) {
        return std::nullopt;
    }
    BRepAdaptor_Surface surf(TopoDS::Face(s), Standard_True);
    if (surf.GetType() != GeomAbs_Plane) {
        return std::nullopt;
    }
    const gp_Pln pln = surf.Plane();
    const gp_Pnt loc = pln.Location();
    const gp_Dir nrm = pln.Axis().Direction();
    return std::make_pair(std::array<double, 3>{loc.X(), loc.Y(), loc.Z()},
                          std::array<double, 3>{nrm.X(), nrm.Y(), nrm.Z()});
}

} // namespace

void cad_module(nb::module_ &m) {
    // Kernel-agnostic mesh / color / group types live in cad — they're the
    // surface every backend (native OCCT, wasm OCCT, future CGAL) speaks.
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

    m.def("make_cylinder", &make_cylinder_impl,
          "radius"_a, "height"_a,
          "Create a cylinder ShapeHandle along +Z, base on the XY plane.");

    m.def("make_sphere", &make_sphere_impl,
          "radius"_a,
          "Create a sphere ShapeHandle centered at the origin.");

    m.def("tessellate", &tessellate_impl,
          "shape"_a, "linear_deflection"_a = -1.0,
          "Tessellate a shape into a triangle Mesh. "
          "linear_deflection<=0 selects a heuristic based on the shape's bbox.");

    m.def("tessellate_box", &tessellate_box_impl,
          "dx"_a, "dy"_a, "dz"_a,
          "Convenience: build a box and tessellate it in one call.");

    m.def("bbox", &bbox_impl,
          "shape"_a,
          "Axis-aligned bounding box of a shape, returned as "
          "(xmin, ymin, zmin, xmax, ymax, zmax).");

    m.def("from_topods_pointer", &from_topods_pointer_impl,
          "ptr"_a,
          "Wrap an OCCT TopoDS_Shape addressed by a raw pointer (typically "
          "produced by `int(pyocc_shape.this)`) into a ShapeHandle.");

    m.def("read_step_bytes", &read_step_bytes_impl,
          "data"_a,
          "Parse a STEP file from a bytes buffer into a ShapeHandle.");

    m.def("write_glb_bytes", &write_glb_bytes_impl,
          "shape"_a, "linear_deflection"_a = 0.1,
          "Tessellate a ShapeHandle and write a binary glTF (.glb) into "
          "a bytes buffer. linear_deflection<=0 falls back to 0.1.");

    // --- shape-algebra verbs (mirror adapy's CadBackend) ---

    m.def("boolean", &boolean_impl,
          "op"_a, "a"_a, "b"_a,
          "CSG of two shapes. op is one of 'UNION', 'INTERSECTION', "
          "'DIFFERENCE' (a - b).");

    m.def("transform", &transform_impl,
          "shape"_a, "matrix"_a, "copy"_a = true,
          "Apply a 4x4 affine transform (top 3 rows, 12 row-major doubles) to "
          "a shape. copy mirrors BRepBuilderAPI_Transform's copy flag.");

    m.def("distance", &distance_impl,
          "a"_a, "b"_a,
          "Minimal distance between two shapes.");

    m.def("serialize", &serialize_impl,
          "shape"_a,
          "Serialize a shape to a BREP string (triangulation cleaned first).");

    m.def("is_valid", &is_valid_impl,
          "shape"_a,
          "Topological + geometric validity (BRepCheck_Analyzer).");

    m.def("faces", &faces_impl,
          "shape"_a,
          "List of face sub-shapes as ShapeHandles.");

    m.def("vertex_points", &vertex_points_impl,
          "shape"_a,
          "List of unique vertex coordinates as (x, y, z) tuples.");

    m.def("face_plane", &face_plane_impl,
          "face"_a,
          "Planar face's (origin, normal) as ((x,y,z),(x,y,z)), or None if "
          "the face is not planar.");
}
