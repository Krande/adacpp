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
#include <cmath>
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
#include <Bnd_OBB.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
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
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

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

// obb: oriented bounding box query
// ----------------------------------------------------------------------------

// Mirrors OCC.Extend.ShapeFactory.get_oriented_boundingbox (optimal OBB via
// triangulation). Returns the world-space barycenter and the three OBB
// half-sizes; the orientation axes are not exposed (callers reconstruct an
// axis-aligned span from centre +/- half-size, matching the adapy walls path).
std::pair<std::array<double, 3>, std::array<double, 3>> obb_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("obb: ShapeHandle is null");
    }
    Bnd_OBB obb;
    BRepBndLib::AddOBB(shape, obb,
                       /*useTriangulation=*/Standard_True,
                       /*isOptimal=*/Standard_True,
                       /*useShapeTolerance=*/Standard_False);
    if (obb.IsVoid()) {
        throw std::runtime_error("obb: empty bounding box (shape has no geometry)");
    }
    const gp_Pnt c = obb.Center();
    return {{c.X(), c.Y(), c.Z()},
            {obb.XHSize(), obb.YHSize(), obb.ZHSize()}};
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

double volume_impl(const ShapeHandle &sh) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(sh.topods(), props);
    return props.Mass();
}

// Sub-shape lists — boundary crosses once, not per element.
std::vector<ShapeHandle> faces_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_FACE); exp.More(); exp.Next()) {
        out.emplace_back(exp.Current());
    }
    return out;
}

std::vector<ShapeHandle> solids_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_SOLID); exp.More(); exp.Next()) {
        out.emplace_back(exp.Current());
    }
    return out;
}

std::vector<ShapeHandle> edges_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_EDGE); exp.More(); exp.Next()) {
        out.emplace_back(exp.Current());
    }
    return out;
}

// Reverse bridge: expose the address of the wrapped OCCT TopoDS_Shape so an
// ABI-compatible OCCT consumer (e.g. gmsh's importShapesNativePointer, same
// OCCT 7.9.x) can read it — mirrors pythonocc's int(shape.this). The pointer
// is valid only while this ShapeHandle is alive.
uintptr_t to_topods_pointer_impl(const ShapeHandle &sh) {
    return reinterpret_cast<uintptr_t>(&sh.topods());
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

// ----------------------------------------------------------------------------
// Placement-aware primitive builders — direct ports of adapy's
// ada.occ.geom.solids.make_*_from_geom (same OCCT calls → identical shapes),
// so AdacppBackend.build() can construct ada.geom primitives natively without
// any pythonocc dependency. location/axis/ref_dir come from the geometry's
// Axis2Placement3D.
// ----------------------------------------------------------------------------

ShapeHandle build_box_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                           std::array<double, 3> ref_dir, double dx, double dy, double dz) {
    const gp_Ax2 ax2(gp_Pnt(loc[0], loc[1], loc[2]),
                     gp_Dir(axis[0], axis[1], axis[2]),
                     gp_Dir(ref_dir[0], ref_dir[1], ref_dir[2]));
    return ShapeHandle(BRepPrimAPI_MakeBox(ax2, dx, dy, dz).Shape());
}

ShapeHandle build_cylinder_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                                double radius, double height) {
    const gp_Ax2 ax2(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]));
    return ShapeHandle(BRepPrimAPI_MakeCylinder(ax2, radius, height).Shape());
}

ShapeHandle build_sphere_impl(std::array<double, 3> center, double radius) {
    return ShapeHandle(BRepPrimAPI_MakeSphere(gp_Pnt(center[0], center[1], center[2]), radius).Shape());
}

// Polyline wire through a list of 3D points (consecutive straight edges) —
// e.g. a beam's centre-line for line/edge FEM meshing.
ShapeHandle make_wire_impl(const std::vector<std::array<double, 3>> &pts) {
    if (pts.size() < 2) {
        throw std::runtime_error("make_wire: need at least 2 points");
    }
    BRepBuilderAPI_MakeWire wm;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        wm.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(pts[i][0], pts[i][1], pts[i][2]),
                                       gp_Pnt(pts[i + 1][0], pts[i + 1][1], pts[i + 1][2])).Edge());
    }
    wm.Build();
    return ShapeHandle(wm.Wire());
}

ShapeHandle build_cone_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                            double bottom_radius, double height) {
    const gp_Ax2 ax2(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]));
    return ShapeHandle(BRepPrimAPI_MakeCone(ax2, bottom_radius, 0.0, height).Shape());
}

// A profile curve is a list of "edge records" (each a flat list of doubles),
// the first element a kind tag (mirrors ada.geom curve segments):
//   line   = [0, x1,y1,z1, x2,y2,z2]
//   arc    = [1, sx,sy,sz, mx,my,mz, ex,ey,ez]   (start, mid, end)
//   circle = [2, cx,cy,cz, ax,ay,az, r]          (Axis2Placement + radius)
TopoDS_Wire wire_from_edges(const std::vector<std::vector<double>> &edges) {
    BRepBuilderAPI_MakeWire wm;
    for (const auto &e : edges) {
        const int kind = static_cast<int>(std::lround(e[0]));
        if (kind == 0) {  // line
            wm.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(e[1], e[2], e[3]), gp_Pnt(e[4], e[5], e[6])).Edge());
        } else if (kind == 1) {  // 3-point arc
            GC_MakeArcOfCircle arc(gp_Pnt(e[1], e[2], e[3]), gp_Pnt(e[4], e[5], e[6]), gp_Pnt(e[7], e[8], e[9]));
            wm.Add(BRepBuilderAPI_MakeEdge(arc.Value()).Edge());
        } else if (kind == 2) {  // full circle
            const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
            wm.Add(BRepBuilderAPI_MakeEdge(gp_Circ(ax, e[7])).Edge());
        } else {
            throw std::runtime_error("wire_from_edges: unknown edge kind");
        }
    }
    wm.Build();
    return wm.Wire();
}

TopoDS_Shape face_from_edges(const std::vector<std::vector<double>> &edges) {
    return BRepBuilderAPI_MakeFace(wire_from_edges(edges)).Shape();
}

// Place a shape built in the XY frame at an Axis2Placement3D — the gp_Ax3
// change-of-basis + translation (= adapy's transform_shape_to_pos).
TopoDS_Shape place_at(TopoDS_Shape shape, const std::array<double, 3> &loc,
                      const std::array<double, 3> &axis, const std::array<double, 3> &ref_dir) {
    gp_Trsf rot;
    const gp_Ax3 ax_global(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
    const gp_Ax3 ax_local(gp_Pnt(0, 0, 0), gp_Dir(axis[0], axis[1], axis[2]),
                          gp_Dir(ref_dir[0], ref_dir[1], ref_dir[2]));
    rot.SetTransformation(ax_local, ax_global);
    shape = BRepBuilderAPI_Transform(shape, rot, true).Shape();
    gp_Trsf tr;
    tr.SetTranslation(gp_Vec(loc[0], loc[1], loc[2]));
    return BRepBuilderAPI_Transform(shape, tr, true).Shape();
}

// Face-based surface model: a set of polygon faces fused into one shell.
// Port of adapy's make_shell_from_face_based_surface_geom. Each polygon is a
// closed loop of 3D points.
ShapeHandle build_face_based_surface_model_impl(
        const std::vector<std::vector<std::array<double, 3>>> &polygons) {
    TopoDS_Shape result;
    bool first = true;
    for (const auto &poly : polygons) {
        if (poly.size() < 3) continue;
        BRepBuilderAPI_MakeWire wm;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const auto &a = poly[i];
            const auto &b = poly[(i + 1) % poly.size()];
            wm.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(a[0], a[1], a[2]), gp_Pnt(b[0], b[1], b[2])).Edge());
        }
        wm.Build();
        const TopoDS_Shape face = BRepBuilderAPI_MakeFace(wm.Wire()).Shape();
        if (first) {
            result = face;
            first = false;
        } else {
            result = BRepAlgoAPI_Fuse(face, result).Shape();
        }
    }
    if (first) {
        throw std::runtime_error("build_face_based_surface_model: no faces");
    }
    return ShapeHandle(result);
}

// Curve-bounded planar face (shell representation of plates/beams): outer
// profile face minus inner voids, placed. Port of adapy's
// make_shell_from_curve_bounded_plane_geom.
ShapeHandle build_planar_face_impl(
        const std::vector<std::vector<double>> &outer,
        const std::vector<std::vector<std::vector<double>>> &inners,
        std::array<double, 3> loc, std::array<double, 3> axis, std::array<double, 3> ref_dir) {
    TopoDS_Shape face = face_from_edges(outer);
    for (const auto &inner : inners) {
        face = BRepAlgoAPI_Cut(face, face_from_edges(inner)).Shape();
    }
    return ShapeHandle(place_at(face, loc, axis, ref_dir));
}

// Build a swept profile from edge records. AREA → outer face minus inner void
// faces (solid cross-section). CURVE → the outer wire alone: matches OCC's
// make_profile_from_geom non-area path, where cutting a 1-D wire by a disjoint
// inner wire leaves the outer wire unchanged (so sweeping it yields the open
// lateral surface, e.g. a pipe-shell cylinder, not a filled tube).
TopoDS_Shape swept_profile(const std::vector<std::vector<double>> &outer,
                           const std::vector<std::vector<std::vector<double>>> &inners,
                           bool is_area) {
    if (!is_area) {
        return wire_from_edges(outer);
    }
    TopoDS_Shape profile = face_from_edges(outer);
    for (const auto &inner : inners) {
        profile = BRepAlgoAPI_Cut(profile, face_from_edges(inner)).Shape();
    }
    return profile;
}

// Native ExtrudedAreaSolid (beams + plates + pipe-shell straights): port of
// adapy's make_extruded_area_shape_from_geom + make_profile_from_geom. Build
// the profile (AREA face or CURVE wire), prism-extrude +Z by depth, then place
// via the gp_Ax3 change-of-basis + translation (= transform_shape_to_pos).
ShapeHandle build_extruded_area_solid_impl(
        const std::vector<std::vector<double>> &outer,
        const std::vector<std::vector<std::vector<double>>> &inners,
        std::array<double, 3> loc, std::array<double, 3> axis, std::array<double, 3> ref_dir,
        double depth, bool is_area) {
    TopoDS_Shape profile = swept_profile(outer, inners, is_area);
    TopoDS_Shape solid = BRepPrimAPI_MakePrism(profile, gp_Vec(0.0, 0.0, depth)).Shape();
    return ShapeHandle(place_at(solid, loc, axis, ref_dir));
}

// Native RevolvedAreaSolid (pipe-shell elbows): port of adapy's
// make_revolved_area_shape_from_geom. Build the profile (AREA face or CURVE
// wire), place it at the swept_area position, then revolve around the
// world-space axis (axis_loc, axis_dir) by `angle_deg` degrees. No final
// placement — the revolve already lands in world space.
ShapeHandle build_revolved_area_solid_impl(
        const std::vector<std::vector<double>> &outer,
        const std::vector<std::vector<std::vector<double>>> &inners,
        std::array<double, 3> loc, std::array<double, 3> axis, std::array<double, 3> ref_dir,
        std::array<double, 3> axis_loc, std::array<double, 3> axis_dir,
        double angle_deg, bool is_area) {
    TopoDS_Shape profile = swept_profile(outer, inners, is_area);
    profile = place_at(profile, loc, axis, ref_dir);
    const gp_Ax1 rev_axis(gp_Pnt(axis_loc[0], axis_loc[1], axis_loc[2]),
                          gp_Dir(axis_dir[0], axis_dir[1], axis_dir[2]));
    const double angle_rad = angle_deg * 0.017453292519943295;
    TopoDS_Shape shape = BRepPrimAPI_MakeRevol(profile, rev_axis, angle_rad).Shape();
    return ShapeHandle(shape);
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

    m.def("obb", &obb_impl,
          "shape"_a,
          "Oriented bounding box of a shape, returned as "
          "((cx, cy, cz), (hx, hy, hz)) — world-space barycenter and OBB "
          "half-sizes.");

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

    m.def("volume", &volume_impl,
          "shape"_a,
          "Solid volume (BRepGProp::VolumeProperties).");

    m.def("faces", &faces_impl,
          "shape"_a,
          "List of face sub-shapes as ShapeHandles.");

    m.def("solids", &solids_impl,
          "shape"_a,
          "List of solid sub-shapes as ShapeHandles.");

    m.def("edges", &edges_impl,
          "shape"_a,
          "List of edge sub-shapes as ShapeHandles.");

    m.def("to_topods_pointer", &to_topods_pointer_impl,
          "shape"_a,
          "Address of the wrapped OCCT TopoDS_Shape (for ABI-compatible OCCT "
          "consumers like gmsh; mirrors pythonocc's int(shape.this)). Valid "
          "only while the ShapeHandle is alive.");

    m.def("vertex_points", &vertex_points_impl,
          "shape"_a,
          "List of unique vertex coordinates as (x, y, z) tuples.");

    m.def("face_plane", &face_plane_impl,
          "face"_a,
          "Planar face's (origin, normal) as ((x,y,z),(x,y,z)), or None if "
          "the face is not planar.");

    // --- placement-aware primitive builders (ada.geom.solids parity) ---

    m.def("build_box", &build_box_impl,
          "location"_a, "axis"_a, "ref_dir"_a, "dx"_a, "dy"_a, "dz"_a,
          "Box with a corner at `location`, edges along the Axis2Placement3D "
          "frame (axis=Z, ref_dir=X).");

    m.def("build_cylinder", &build_cylinder_impl,
          "location"_a, "axis"_a, "radius"_a, "height"_a,
          "Cylinder with base centre at `location`, along `axis`.");

    m.def("build_sphere", &build_sphere_impl,
          "center"_a, "radius"_a,
          "Sphere centred at `center`.");

    m.def("make_wire", &make_wire_impl,
          "points"_a,
          "Polyline wire through a list of 3D points (consecutive straight "
          "edges).");

    m.def("build_cone", &build_cone_impl,
          "location"_a, "axis"_a, "bottom_radius"_a, "height"_a,
          "Right circular cone (apex radius 0) with base at `location`.");

    m.def("build_extruded_area_solid", &build_extruded_area_solid_impl,
          "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a, "depth"_a,
          "is_area"_a = true,
          "Extruded area solid (beams/plates/pipe-shells): an outer profile "
          "curve + inner void curves (each a list of edge records: "
          "line=[0,p1,p2], arc=[1,start,mid,end], circle=[2,centre,axis,r]), "
          "prism-extruded by `depth` and placed at the Axis2Placement3D frame. "
          "is_area=False sweeps the outer wire alone (open lateral surface).");

    m.def("build_revolved_area_solid", &build_revolved_area_solid_impl,
          "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a,
          "axis_location"_a, "axis_direction"_a, "angle_deg"_a, "is_area"_a = true,
          "Revolved area solid (pipe-shell elbows): build the profile (same "
          "edge-record encoding as build_extruded_area_solid), place it at the "
          "Axis2Placement3D frame, then revolve around the world axis "
          "(axis_location, axis_direction) by angle_deg degrees. is_area=False "
          "revolves the outer wire alone.");

    m.def("build_planar_face", &build_planar_face_impl,
          "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a,
          "Curve-bounded planar face (shell representation): outer profile face "
          "minus inner void faces, placed at the Axis2Placement3D frame. Same "
          "edge-record encoding as build_extruded_area_solid.");

    m.def("build_face_based_surface_model", &build_face_based_surface_model_impl,
          "polygons"_a,
          "Fuse a list of polygon faces (each a closed loop of 3D points) into "
          "one shell.");
}
