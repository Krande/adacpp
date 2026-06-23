#include "cad_py_wrap.h"
#include "ShapeHandle.h"
#include "../geom/Color.h"
#include "../geom/GroupReference.h"
#include "../geom/Mesh.h"
#include "../geom/MeshType.h"
#include "../cadit/occt/static_param_guard.h"
#include "../cadit/occt/step_writer.h"
#include "../cadit/ifc/ngeom_taxonomy.h"
#include "../geom/neutral/ngeom_decode.h"
#include "../geom/neutral/ngeom_tessellate.h"
#include "../geom/neutral/ngeom_meshopt.h"

#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <filesystem>
#include <fstream>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BOPAlgo_Builder.hxx>
#include <BOPAlgo_CellsBuilder.hxx>
#include <BOPAlgo_GlueEnum.hxx>
#include <BOPAlgo_MakerVolume.hxx>
#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_TransitionMode.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dConvert.hxx>
#include <Standard_Failure.hxx>
#include <GeomLProp_SLProps.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt2d.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <GeomAbs_Shape.hxx>
#include <Standard_Type.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <BRepTools_ShapeSet.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_Triangulation.hxx>
#include <RWGltf_CafWriter.hxx>
#include <RWGltf_WriterTrsfFormat.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <STEPControl_Reader.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_State.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_MapOfOrientedShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <Quantity_Color.hxx>
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
// Result structs for face_to_advanced_face (the OCC face → AdvancedFace
// decomposer). Plain data, bound read-only to Python; adapy reassembles them
// into ada.geom AdvancedFace / BSplineSurfaceWithKnots / Pcurve2dBSpline.
// ----------------------------------------------------------------------------

struct PcurveData {
    bool has_pcurve = false;                              // false → no UV curve recoverable
    int degree = 0;
    std::vector<std::array<double, 2>> control_points;    // 2D UV poles
    std::vector<double> knots;
    std::vector<int> multiplicities;
    std::vector<double> weights;                          // empty → non-rational
    bool closed = false;
    std::array<double, 3> start{};                        // 3D edge endpoints
    std::array<double, 3> end{};
};

struct AdvancedFaceData {
    int u_degree = 0, v_degree = 0;
    std::vector<std::vector<std::array<double, 3>>> poles;  // [n_u][n_v]
    std::vector<double> u_knots, v_knots;
    std::vector<int> u_multiplicities, v_multiplicities;
    std::vector<std::vector<double>> weights;              // [n_u][n_v], empty → non-rational
    bool u_closed = false, v_closed = false;
    std::vector<std::vector<PcurveData>> bounds;           // [wire][edge]
};

// One shape extracted from a STEP OCAF document, with its label name + color.
struct StepShapeData {
    ShapeHandle shape;
    std::string name;
    std::array<double, 3> color{0.5, 0.5, 0.5};
    bool has_color = false;
};

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

// Mesh a single shape and APPEND its triangles into the shared buffers,
// offsetting triangle indices by the current vertex base. Shared by the single
// tessellate (one shape) and the batch path (many shapes into one mesh).
static void append_shape_triangles(const TopoDS_Shape &shape_in, double linear_deflection,
                                   std::vector<float> &positions, std::vector<uint32_t> &indices) {
    TopoDS_Shape shape = shape_in;
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

    // BRepMesh grids nothing when a face is missing its 2D p-curves (the parametric
    // representation it triangulates against) — common for imported B-reps: a bspline/NURBS
    // face trimmed by 3D-only boundary curves (IfcPolyline pcurves, IfcIntersectionCurve).
    // The face is valid (exports to STEP) but renders empty. ShapeFix builds the missing
    // p-curves; retry the mesh once. Only fires when the first pass produced no triangles,
    // so there is no happy-path cost. Mirrors OccBackend's tessellate ShapeFix retry.
    {
        bool any_tris = false;
        for (TopExp_Explorer e(shape, TopAbs_FACE); e.More() && !any_tris; e.Next()) {
            TopLoc_Location l;
            const Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(TopoDS::Face(e.Current()), l);
            if (!t.IsNull() && t->NbTriangles() > 0) any_tris = true;
        }
        if (!any_tris) {
            ShapeFix_Shape sf(shape);
            sf.Perform();
            const TopoDS_Shape fixed = sf.Shape();
            if (!fixed.IsNull()) {
                shape = fixed;
                BRepMesh_IncrementalMesh(shape, linear_deflection);
            }
        }
    }

    // Pre-count this shape's nodes/triangles and grow the shared buffers once.
    // Appending without reserve reallocates repeatedly as the mesh grows — the
    // dominant allocation cost. The triangulation was already computed above, so
    // this extra face pass is cheap.
    size_t n_nodes = 0, n_tris = 0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri =
                BRep_Tool::Triangulation(TopoDS::Face(exp.Current()), loc);
        if (tri.IsNull()) continue;
        n_nodes += static_cast<size_t>(tri->NbNodes());
        n_tris += static_cast<size_t>(tri->NbTriangles());
    }
    positions.reserve(positions.size() + n_nodes * 3);
    indices.reserve(indices.size() + n_tris * 3);

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
        // Honor face orientation: OCCT stores each face's triangle nodes wound
        // for the FORWARD surface. A TopAbs_REVERSED face (common in solids,
        // where the surface normal opposes the outward solid normal) needs its
        // winding flipped so the emitted triangles face outward. Without this,
        // reversed faces tessellate with inward normals — e.g. swept/pipe
        // solids come out with a net-negative mesh volume.
        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        for (Standard_Integer i = 1; i <= tri->NbTriangles(); ++i) {
            Standard_Integer n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);
            indices.push_back(base + static_cast<uint32_t>(n1 - 1));
            indices.push_back(base + static_cast<uint32_t>(n2 - 1));
            indices.push_back(base + static_cast<uint32_t>(n3 - 1));
        }
    }
}

Mesh tessellate_impl(const ShapeHandle &sh, double linear_deflection) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("tessellate: ShapeHandle is null");
    }
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    append_shape_triangles(shape, linear_deflection, positions, indices);
    return Mesh(0, std::move(positions), std::move(indices));
}

// Batch: tessellate many shapes into ONE combined mesh in a single Python->C++
// call, demarcated by a GroupReference per input shape (node_id = shape index,
// start/length = the triangle-index range in the shared buffer). This amortizes
// the per-call boundary + Python-loop + object-wrapping cost across all shapes,
// and yields a single zero-copy NumPy buffer ready for one GLB scene. Null
// shapes are recorded as empty (zero-length) groups so the result stays aligned
// with the input list by index.
Mesh tessellate_batch_impl(const std::vector<ShapeHandle> &shapes, double linear_deflection) {
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    std::vector<GroupReference> groups;
    groups.reserve(shapes.size());
    for (size_t i = 0; i < shapes.size(); ++i) {
        const uint32_t start = static_cast<uint32_t>(indices.size());
        const uint32_t vstart = static_cast<uint32_t>(positions.size() / 3);
        const TopoDS_Shape &shape = shapes[i].topods();
        if (!shape.IsNull()) {
            append_shape_triangles(shape, linear_deflection, positions, indices);
        }
        const uint32_t length = static_cast<uint32_t>(indices.size()) - start;
        const uint32_t vlength = static_cast<uint32_t>(positions.size() / 3) - vstart;
        groups.emplace_back(static_cast<int>(i), static_cast<int>(start), static_cast<int>(length),
                            static_cast<int>(vstart), static_cast<int>(vlength));
    }
    Mesh mesh(0, std::move(positions), std::move(indices));
    mesh.group_reference = std::move(groups);
    return mesh;
}

// Convenience: build a primitive box and tessellate it in one call. Same logic
// as make_box() + tessellate(), kept as a single entry point for callers that
// don't need the intermediate handle.
Mesh tessellate_box_impl(float dx, float dy, float dz) {
    return tessellate_impl(make_box_impl(dx, dy, dz), -1.0);
}

// Decode an NGEOM stream buffer (adapy ada.geom serialized per the neutral schema) and
// tessellate every instance through the chosen pipeline into ONE combined Mesh, with a
// GroupReference per root (node_id = root index in serialization order; adapy maps it back
// to its own instance id). pipeline: "libtess2" (OCC-free neutral path) | "occ" | "cgal"
// | "hybrid" (ifcopenshell taxonomy kernels). angular is in DEGREES.
Mesh tessellate_stream_impl(nb::bytes buffer, const std::string &pipeline, double deflection,
                            double angular_deg) {
    using namespace adacpp::ngeom;
    NgeomDoc doc;
    try {
        doc = decode(reinterpret_cast<const uint8_t *>(buffer.c_str()), buffer.size());
    } catch (const std::exception &) {
        return Mesh(0, {}, {});  // malformed buffer -> empty mesh
    }

    TessMesh tm;
    if (pipeline == "libtess2" || pipeline.empty()) {
        TessParams tp;
        tp.deflection = deflection;
        tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
        tm = tessellate_doc(doc, tp);
    } else {
        // taxonomy kernels; accept "occ"/"cgal"/"hybrid" or "taxonomy-<k>"
        std::string kern = pipeline;
        auto dash = kern.find('-');
        if (dash != std::string::npos) kern = kern.substr(dash + 1);
        tm = tessellate_via_taxonomy(doc, kern, deflection, angular_deg);
    }

    std::vector<GroupReference> groups;
    groups.reserve(tm.groups.size());
    for (size_t i = 0; i < tm.groups.size(); ++i) {
        const auto &g = tm.groups[i];
        groups.emplace_back((int)i, (int)g.first_index, (int)g.index_count, (int)g.first_vertex,
                            (int)g.vertex_count);
    }
    Mesh mesh(0, std::move(tm.positions), std::move(tm.indices), {}, std::move(tm.normals));
    mesh.group_reference = std::move(groups);
    return mesh;
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
// to a temp file in the system temp dir and let OCCT read/write through that.
// Under pyodide that dir is MEMFS (in-memory), so this is purely a memcpy
// detour with no real disk I/O — same speed as a memory-stream API would give.

// Allocate a unique, not-yet-existing path in the system temp directory.
// Cross-platform replacement for POSIX mkstemp (no <unistd.h> on Windows).
std::filesystem::path make_temp_path(const char *prefix, const char *suffix) {
    static std::atomic<unsigned long long> counter{0};
    std::random_device rd;
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    for (int attempt = 0; attempt < 100; ++attempt) {
        std::ostringstream name;
        name << prefix << '_' << rd() << '_' << counter.fetch_add(1) << suffix;
        std::filesystem::path candidate = dir / name.str();
        std::error_code ec;
        if (!std::filesystem::exists(candidate, ec)) {
            return candidate;
        }
    }
    throw std::runtime_error("make_temp_path: could not allocate a unique temp file");
}

ShapeHandle read_step_bytes_impl(nb::bytes data) {
    const std::filesystem::path tmp = make_temp_path("adacpp_step", ".stp");
    const std::string tmpname = tmp.string();
    {
        std::ofstream out(tmpname, std::ios::binary);
        if (!out) {
            throw std::runtime_error("read_step_bytes: failed to create temp file");
        }
        out.write(data.c_str(), static_cast<std::streamsize>(data.size()));
        if (!out) {
            std::error_code ec;
            std::filesystem::remove(tmp, ec);
            throw std::runtime_error("read_step_bytes: failed to materialize temp file");
        }
    }

    STEPControl_Reader reader;
    const IFSelect_ReturnStatus status = reader.ReadFile(tmpname.c_str());
    std::error_code ec;
    std::filesystem::remove(tmp, ec);
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

// Recursively collect simple shapes from an OCAF label tree, applying assembly
// component locations and reading each label's name + color. Port of adapy's
// read_step_file_with_names_colors traversal (assembly → components → simple
// shapes + their sub-shapes).
void collect_step_shapes(const Handle(XCAFDoc_ShapeTool) &st, const Handle(XCAFDoc_ColorTool) &ct,
                         const TDF_Label &lab, const TopLoc_Location &loc,
                         std::vector<StepShapeData> &out) {
    auto read_one = [&](const TDF_Label &shape_lab, const TopoDS_Shape &raw) {
        const TopoDS_Shape shape =
                loc.IsIdentity() ? raw : BRepBuilderAPI_Transform(raw, loc.Transformation()).Shape();
        std::string name;
        Handle(TDataStd_Name) nm;
        if (shape_lab.FindAttribute(TDataStd_Name::GetID(), nm)) {
            name = TCollection_AsciiString(nm->Get()).ToCString();
        }
        std::array<double, 3> color{0.5, 0.5, 0.5};
        bool has_color = false;
        Quantity_Color c;
        if (ct->GetColor(shape_lab, XCAFDoc_ColorGen, c) || ct->GetColor(shape_lab, XCAFDoc_ColorSurf, c) ||
            ct->GetColor(shape_lab, XCAFDoc_ColorCurv, c) || ct->GetColor(raw, XCAFDoc_ColorGen, c) ||
            ct->GetColor(raw, XCAFDoc_ColorSurf, c) || ct->GetColor(raw, XCAFDoc_ColorCurv, c)) {
            color = {c.Red(), c.Green(), c.Blue()};
            has_color = true;
        }
        out.push_back(StepShapeData{ShapeHandle(shape), name, color, has_color});
    };

    if (st->IsAssembly(lab)) {
        TDF_LabelSequence comps;
        st->GetComponents(lab, comps);
        for (int i = 1; i <= comps.Length(); ++i) {
            TDF_Label comp = comps.Value(i);
            if (st->IsReference(comp)) {
                TDF_Label ref;
                st->GetReferredShape(comp, ref);
                collect_step_shapes(st, ct, ref, loc * st->GetLocation(comp), out);
            }
        }
    } else if (st->IsSimpleShape(lab)) {
        read_one(lab, st->GetShape(lab));
        TDF_LabelSequence subs;
        st->GetSubShapes(lab, subs);
        for (int i = 1; i <= subs.Length(); ++i) {
            const TDF_Label sub = subs.Value(i);
            read_one(sub, st->GetShape(sub));
        }
    }
}

// Read a STEP file (from bytes) via OCAF, returning each shape with its name +
// color. Port of StepStore + read_step_file_with_names_colors for the adacpp
// doc backend (STEP import with no pythonocc).
std::vector<StepShapeData> read_step_shapes_impl(nb::bytes data, const std::string &unit) {
    // Restore the caller's value on exit — "xstep.cascade.unit" is a process-
    // global Interface_Static parameter; leaving it set leaks into later reads.
    const InterfaceStaticCValGuard cascade_unit_guard("xstep.cascade.unit");
    const std::filesystem::path tmp = make_temp_path("adacpp_step_read", ".stp");
    {
        std::ofstream f(tmp.string(), std::ios::binary);
        f.write(data.c_str(), static_cast<std::streamsize>(data.size()));
    }
    STEPCAFControl_Reader reader;
    reader.SetColorMode(Standard_True);
    reader.SetNameMode(Standard_True);
    // Convert the STEP file's native length unit to `unit` on read — mirrors
    // StepStore's "xstep.cascade.unit" so e.g. a 10 m plate reads as 10.0 not
    // 10000. Set AFTER constructing the reader (its ctor resets the static),
    // before ReadFile — same order as StepStore.create_step_reader.
    Interface_Static::SetCVal("xstep.cascade.unit", unit.c_str());
    if (reader.ReadFile(tmp.string().c_str()) != IFSelect_RetDone) {
        std::filesystem::remove(tmp);
        throw std::runtime_error("read_step_shapes: STEPCAFControl_Reader could not parse the input");
    }
    Handle(TDocStd_Document) doc = new TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"));
    const bool ok = reader.Transfer(doc);
    std::filesystem::remove(tmp);
    if (!ok) throw std::runtime_error("read_step_shapes: transfer to OCAF document failed");

    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) ct = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TDF_LabelSequence free_shapes;
    st->GetFreeShapes(free_shapes);
    std::vector<StepShapeData> out;
    for (int i = 1; i <= free_shapes.Length(); ++i) {
        collect_step_shapes(st, ct, free_shapes.Value(i), TopLoc_Location(), out);
    }
    return out;
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

    const std::filesystem::path tmp = make_temp_path("adacpp_glb", ".glb");
    const std::string tmpname = tmp.string();  // RWGltf_CafWriter opens by name.

    {
        RWGltf_CafWriter writer(TCollection_AsciiString(tmpname.c_str()),
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
            std::error_code ec;
            std::filesystem::remove(tmp, ec);
            throw std::runtime_error("write_glb_bytes: RWGltf_CafWriter::Perform failed");
        }
    }

    std::ifstream f(tmpname, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        std::error_code ec;
        std::filesystem::remove(tmp, ec);
        throw std::runtime_error("write_glb_bytes: failed to re-open temp file");
    }
    const auto size = static_cast<std::size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (size > 0) f.read(buffer.data(), size);
    f.close();
    std::error_code ec;
    std::filesystem::remove(tmp, ec);

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

// Total surface area (GProp). Backend-neutral replacement for tests that
// reach for BRepGProp::SurfaceProperties + GProp_GProps::Mass directly.
double area_impl(const ShapeHandle &sh) {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(sh.topods(), props);
    return props.Mass();
}

// Topological kind of the shape ("solid"/"shell"/"face"/"wire"/"edge"/
// "vertex"/"compound"/"compsolid"). Lets tests classify a built shape without
// isinstance(TopoDS_Solid/Face/Compound) against pythonocc types.
std::string shape_type_impl(const ShapeHandle &sh) {
    switch (sh.topods().ShapeType()) {
        case TopAbs_COMPOUND:  return "compound";
        case TopAbs_COMPSOLID: return "compsolid";
        case TopAbs_SOLID:     return "solid";
        case TopAbs_SHELL:     return "shell";
        case TopAbs_FACE:      return "face";
        case TopAbs_WIRE:      return "wire";
        case TopAbs_EDGE:      return "edge";
        case TopAbs_VERTEX:    return "vertex";
        default:               return "shape";
    }
}

// Underlying geometric surface kind of a face ("plane"/"cylinder"/"cone"/
// "sphere"/"torus"/"bspline"/"bezier"/...). If `sh` is not a face the first
// face found is used. Replaces BRep_Tool::Surface(face)->DynamicType()->Name()
// / IsKind(Geom_BSplineSurface) introspection in tests.
std::string face_surface_type_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &s = sh.topods();
    TopoDS_Face face;
    if (s.ShapeType() == TopAbs_FACE) {
        face = TopoDS::Face(s);
    } else {
        TopExp_Explorer exp(s, TopAbs_FACE);
        if (!exp.More()) throw std::runtime_error("face_surface_type: shape has no face");
        face = TopoDS::Face(exp.Current());
    }
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull()) return "unknown";
    const std::string name = surf->DynamicType()->Name();
    if (name == "Geom_Plane")              return "plane";
    if (name == "Geom_CylindricalSurface") return "cylinder";
    if (name == "Geom_ConicalSurface")     return "cone";
    if (name == "Geom_SphericalSurface")   return "sphere";
    if (name == "Geom_ToroidalSurface")    return "torus";
    if (name == "Geom_BSplineSurface")     return "bspline";
    if (name == "Geom_BezierSurface")      return "bezier";
    if (name == "Geom_SurfaceOfLinearExtrusion") return "linear_extrusion";
    if (name == "Geom_SurfaceOfRevolution")      return "revolution";
    return name;  // fall back to the raw OCCT class name
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
// Topology-kernel ops — the non-manifold core: build solids from a face soup,
// non-manifold merge keeping shared internal faces, free-face (envelope)
// extraction, point-in-solid classification, centre of mass. Native OCCT
// (BOPAlgo / BRepClass3d / BRepGProp), no pythonocc — mirrors adapy's
// OccBackend so both backends agree.
// ----------------------------------------------------------------------------

// Sew a set of faces into a single shell (BRepBuilderAPI_Sewing). Unlike
// make_volumes_from_faces (which partitions space into solids and only suits a
// watertight face soup), this preserves an OPEN surface model as one connected
// shell handle — the IfcShellBasedSurfaceModel / open-shell case where the
// faces are b-spline AdvancedFaces that don't bound a volume. Returns whatever
// the sewer yields (a shell, or a compound of shells/faces if disjoint).
ShapeHandle sew_faces_impl(const std::vector<ShapeHandle> &faces, double tolerance) {
    BRepBuilderAPI_Sewing sewer(tolerance > 0.0 ? tolerance : 1e-6);
    int added = 0;
    for (const auto &f : faces) {
        const TopoDS_Shape s = f.topods();
        if (s.IsNull()) continue;
        sewer.Add(s);
        ++added;
    }
    if (added == 0) throw std::runtime_error("sew_faces: no faces");
    sewer.Perform();
    const TopoDS_Shape sewn = sewer.SewedShape();
    if (sewn.IsNull()) throw std::runtime_error("sew_faces: sewing produced a null shape");
    return ShapeHandle(sewn);
}

std::vector<ShapeHandle> make_volumes_from_faces_impl(const std::vector<ShapeHandle> &faces, double tolerance) {
    BOPAlgo_MakerVolume mv;
    TopTools_ListOfShape args;
    for (const auto &f : faces) args.Append(f.topods());
    mv.SetArguments(args);
    mv.SetIntersect(Standard_True);  // imprint the faces against each other first
    if (tolerance > 0.0) mv.SetFuzzyValue(tolerance);
    mv.Perform();
    if (mv.HasErrors()) throw std::runtime_error("make_volumes_from_faces: BOPAlgo_MakerVolume reported errors");
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(mv.Shape(), TopAbs_SOLID); exp.More(); exp.Next())
        out.emplace_back(exp.Current());
    return out;
}

ShapeHandle non_manifold_merge_impl(const std::vector<ShapeHandle> &shapes, double tolerance, bool glue) {
    BOPAlgo_Builder builder;
    TopTools_ListOfShape args;
    for (const auto &s : shapes) args.Append(s.topods());
    builder.SetArguments(args);
    if (glue) builder.SetGlue(BOPAlgo_GlueShift);  // coincident faces collapse to one shared face
    if (tolerance > 0.0) builder.SetFuzzyValue(tolerance);
    builder.Perform();
    if (builder.HasErrors()) throw std::runtime_error("non_manifold_merge: BOPAlgo_Builder reported errors");
    return ShapeHandle(builder.Shape());
}

// Faithful port of topologic's Topology::Merge over solids: general-fuse the
// solids with BOPAlgo_CellsBuilder, take each operand's region into the result
// (AddToResult) and MakeContainers() to assemble the non-manifold CellComplex —
// each input solid survives as a cell and every interface becomes one shared
// face. Mirrors adapy's OccBackend.merge_cells (unlike make_volumes_from_faces,
// which rebuilds minimal volumes from a face soup and loses operand identity).
std::vector<ShapeHandle> merge_cells_impl(const std::vector<ShapeHandle> &solids, double tolerance) {
    if (solids.empty()) return {};
    BOPAlgo_CellsBuilder cb;
    TopTools_ListOfShape args;
    for (const auto &s : solids) args.Append(s.topods());
    cb.SetArguments(args);
    if (tolerance > 0.0) cb.SetFuzzyValue(tolerance);
    cb.Perform();
    if (cb.HasErrors()) throw std::runtime_error("merge_cells: BOPAlgo_CellsBuilder reported errors");
    for (const auto &s : solids) {
        TopTools_ListOfShape take;
        take.Append(s.topods());
        TopTools_ListOfShape avoid;
        cb.AddToResult(take, avoid);
    }
    cb.MakeContainers();
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(cb.Shape(), TopAbs_SOLID); exp.More(); exp.Next())
        out.emplace_back(exp.Current());
    return out;
}

// Orientation-independent topological identity. A face shared by two cells of a
// non-manifold complex is the SAME underlying TShape referenced twice (with
// opposite orientation), so it hashes equal here while distinct faces differ —
// CellsBuilder/MakerVolume never instance one TShape at multiple placements, so
// the TShape pointer is a stable per-face key. Mirrors adapy OccBackend.face_id;
// lets the cell-graph extractor detect shared faces by true topological identity
// instead of geometry.
int64_t face_id_impl(const ShapeHandle &h) {
    return static_cast<int64_t>(reinterpret_cast<uintptr_t>(h.topods().TShape().get()));
}

// Faces owned by exactly one solid — the outer envelope. Map FACE→SOLID
// ancestors over a compound of the cells and keep the single-owner faces.
std::vector<ShapeHandle> free_faces_impl(const std::vector<ShapeHandle> &solids) {
    BRep_Builder bld;
    TopoDS_Compound comp;
    bld.MakeCompound(comp);
    for (const auto &s : solids) bld.Add(comp, s.topods());
    TopTools_IndexedDataMapOfShapeListOfShape amap;
    TopExp::MapShapesAndAncestors(comp, TopAbs_FACE, TopAbs_SOLID, amap);
    std::vector<ShapeHandle> out;
    for (Standard_Integer i = 1; i <= amap.Extent(); ++i) {
        if (amap.FindFromIndex(i).Extent() == 1)
            out.emplace_back(amap.FindKey(i));
    }
    return out;
}

// Point-in-solid classification. Returns OCCT TopAbs_State as an int:
// IN=0, OUT=1, ON=2, UNKNOWN=3 (adapy's AdacppBackend maps it to Containment).
int point_in_solid_impl(const ShapeHandle &solid, const std::array<double, 3> &pt, double tolerance) {
    BRepClass3d_SolidClassifier clf(solid.topods());
    clf.Perform(gp_Pnt(pt[0], pt[1], pt[2]), tolerance);
    switch (clf.State()) {
        case TopAbs_IN:  return 0;
        case TopAbs_OUT: return 1;
        case TopAbs_ON:  return 2;
        default:         return 3;
    }
}

std::array<double, 3> center_of_mass_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &s = sh.topods();
    GProp_GProps props;
    const TopAbs_ShapeEnum st = s.ShapeType();
    if (st == TopAbs_SOLID || st == TopAbs_COMPSOLID || st == TopAbs_COMPOUND)
        BRepGProp::VolumeProperties(s, props);
    else if (st == TopAbs_SHELL || st == TopAbs_FACE)
        BRepGProp::SurfaceProperties(s, props);
    else
        BRepGProp::LinearProperties(s, props);
    const gp_Pnt c = props.CentreOfMass();
    return {c.X(), c.Y(), c.Z()};
}

std::vector<ShapeHandle> shells_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_SHELL); exp.More(); exp.Next())
        out.emplace_back(exp.Current());
    return out;
}

std::vector<ShapeHandle> wires_impl(const ShapeHandle &sh) {
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(sh.topods(), TopAbs_WIRE); exp.More(); exp.Next())
        out.emplace_back(exp.Current());
    return out;
}

// Ordered boundary vertices. For a FACE, walk its outer wire; for a WIRE, walk
// it directly. BRepTools_WireExplorer yields them in connection order (unlike
// vertex_points, which is unordered) — needed to rebuild a face as a polygon.
// Merge adjacent same-surface (coplanar) faces into single faces. On real
// geometry a cell wall is often split into several coplanar faces; unifying
// makes it one face so shared-face matching between adjacent cells is robust.
ShapeHandle unify_coplanar_faces_impl(const ShapeHandle &sh) {
    ShapeUpgrade_UnifySameDomain unify(sh.topods(), Standard_True, Standard_True, Standard_False);
    unify.Build();
    return ShapeHandle(unify.Shape());
}

std::vector<std::array<double, 3>> wire_points_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &s = sh.topods();
    TopoDS_Wire wire;
    if (s.ShapeType() == TopAbs_FACE)
        wire = BRepTools::OuterWire(TopoDS::Face(s));
    else
        wire = TopoDS::Wire(s);
    std::vector<std::array<double, 3>> out;
    for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next()) {
        const gp_Pnt p = BRep_Tool::Pnt(exp.CurrentVertex());
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
// Build a single TopoDS_Edge from a self-describing flat edge record. The first
// element is a kind tag; the remaining layout depends on the kind. Mirrors
// adapy's make_edge_from_edge (the 3D curve zoo: line/arc/circle/bspline/
// ellipse, full + parametrically-trimmed). Port target so EdgeLoop/OrientedEdge
// boundaries build on adacpp without pythonocc.
//   0 line            : [0, x1,y1,z1, x2,y2,z2]
//   1 arc (3-point)   : [1, sx,sy,sz, mx,my,mz, ex,ey,ez]
//   2 circle (full)   : [2, cx,cy,cz, ax,ay,az, r]
//   5 circle (trimmed): [5, cx,cy,cz, ax,ay,az, r, t_start, t_end]
//   4 ellipse         : [4, cx,cy,cz, ax,ay,az, rdx,rdy,rdz, semi1, semi2,
//                        trim, sx,sy,sz, ex,ey,ez]   (trim=0 → full; pts ignored)
//   3 bspline         : [3, degree, rational, trim, t_start, t_end, n_poles,
//                        <3*n_poles coords>, n_knots, <knots>, <mults>,
//                        <n_poles weights if rational>]
TopoDS_Edge edge_from_record(const std::vector<double> &e) {
    const int kind = static_cast<int>(std::lround(e[0]));
    if (kind == 0) {
        return BRepBuilderAPI_MakeEdge(gp_Pnt(e[1], e[2], e[3]), gp_Pnt(e[4], e[5], e[6])).Edge();
    }
    if (kind == 1) {
        GC_MakeArcOfCircle arc(gp_Pnt(e[1], e[2], e[3]), gp_Pnt(e[4], e[5], e[6]), gp_Pnt(e[7], e[8], e[9]));
        return BRepBuilderAPI_MakeEdge(arc.Value()).Edge();
    }
    if (kind == 2) {
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
        return BRepBuilderAPI_MakeEdge(gp_Circ(ax, e[7])).Edge();
    }
    if (kind == 5) {
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
        return BRepBuilderAPI_MakeEdge(gp_Circ(ax, e[7]), e[8], e[9]).Edge();
    }
    if (kind == 4) {
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]), gp_Dir(e[7], e[8], e[9]));
        const gp_Elips el(ax, e[10], e[11]);
        const bool trim = std::lround(e[12]) != 0;
        if (!trim) return BRepBuilderAPI_MakeEdge(el).Edge();
        return BRepBuilderAPI_MakeEdge(el, gp_Pnt(e[13], e[14], e[15]), gp_Pnt(e[16], e[17], e[18])).Edge();
    }
    if (kind == 3) {
        const int degree = static_cast<int>(std::lround(e[1]));
        const bool rational = std::lround(e[2]) != 0;
        const bool trim = std::lround(e[3]) != 0;
        const double t_start = e[4], t_end = e[5];
        const gp_Pnt p_start(e[6], e[7], e[8]), p_end(e[9], e[10], e[11]);
        std::size_t i = 12;
        const int n_poles = static_cast<int>(std::lround(e[i++]));
        TColgp_Array1OfPnt poles(1, n_poles);
        for (int p = 1; p <= n_poles; ++p) {
            poles.SetValue(p, gp_Pnt(e[i], e[i + 1], e[i + 2]));
            i += 3;
        }
        const int n_knots = static_cast<int>(std::lround(e[i++]));
        TColStd_Array1OfReal knots(1, n_knots);
        for (int k = 1; k <= n_knots; ++k) knots.SetValue(k, e[i++]);
        TColStd_Array1OfInteger mults(1, n_knots);
        for (int k = 1; k <= n_knots; ++k) mults.SetValue(k, static_cast<int>(std::lround(e[i++])));
        Handle(Geom_BSplineCurve) curve;
        if (rational) {
            TColStd_Array1OfReal weights(1, n_poles);
            for (int p = 1; p <= n_poles; ++p) weights.SetValue(p, e[i++]);
            curve = new Geom_BSplineCurve(poles, weights, knots, mults, degree, Standard_False);
        } else {
            curve = new Geom_BSplineCurve(poles, knots, mults, degree, Standard_False);
        }
        if (trim) return BRepBuilderAPI_MakeEdge(curve, t_start, t_end).Edge();
        // No parametric trim: the record's start/end points define the segment of an
        // otherwise-full b-spline curve. Trim by points (OCC projects them onto the curve) —
        // without this the whole curve is used and the edge overshoots the real boundary.
        if (p_start.Distance(p_end) > 1e-9) return BRepBuilderAPI_MakeEdge(curve, p_start, p_end).Edge();
        return BRepBuilderAPI_MakeEdge(curve).Edge();
    }
    throw std::runtime_error("edge_from_record: unknown edge kind " + std::to_string(kind));
}

TopoDS_Wire wire_from_edges(const std::vector<std::vector<double>> &edges) {
    BRepBuilderAPI_MakeWire wm;
    for (const auto &e : edges) {
        wm.Add(edge_from_record(e));
    }
    wm.Build();
    return wm.Wire();
}

TopoDS_Shape face_from_edges(const std::vector<std::vector<double>> &edges) {
    return BRepBuilderAPI_MakeFace(wire_from_edges(edges)).Shape();
}

// Build a wire (open or closed) from a list of edge records. Backs adapy's
// make_wire_from_edge_loop for the active backend.
ShapeHandle build_wire_impl(const std::vector<std::vector<double>> &edges) {
    return ShapeHandle(wire_from_edges(edges));
}

// Wire-filled face (WireFilledFace): interpolate a smooth surface through the
// boundary edges with BRepOffsetAPI_MakeFilling. Port of adapy's
// make_face_from_wire_filled (the SAT exppc-unrecoverable-surface fallback).
ShapeHandle build_filled_face_impl(const std::vector<std::vector<double>> &edges) {
    if (edges.size() < 3) throw std::runtime_error("build_filled_face: need >= 3 boundary edges");
    BRepOffsetAPI_MakeFilling filler;
    for (const auto &e : edges) filler.Add(edge_from_record(e), GeomAbs_C0);
    filler.Build();
    if (!filler.IsDone()) throw std::runtime_error("build_filled_face: MakeFilling failed");
    return ShapeHandle(filler.Shape());
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
// Single planar face from a closed polygon of points (auto-closed). Ports adapy
// OccBackend.polygon_face — used to feed internal divider faces into
// make_volumes_from_faces so a lofted solid partitions into one cell per band.
ShapeHandle polygon_face_impl(const std::vector<std::array<double, 3>> &poly) {
    if (poly.size() < 3) throw std::runtime_error("polygon_face: need at least 3 points");
    BRepBuilderAPI_MakePolygon mp;
    for (const auto &p : poly) mp.Add(gp_Pnt(p[0], p[1], p[2]));
    mp.Close();
    return ShapeHandle(BRepBuilderAPI_MakeFace(mp.Wire(), Standard_True).Face());
}

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

// Build a trimmed face over a B-spline surface (with knots; rational if weights
// supplied). Port of adapy's make_bspline_surface_with_knots + a natural-UV
// MakeFace — the common PlateCurved / loft-derived case where the surface's knot
// range already spans the plate, so no explicit boundary trimming is needed.
// control_points is row-major [num_u][num_v]; weights empty => non-rational.
// Construct a Geom_BSplineSurface from the adapy BSplineSurfaceWithKnots data
// (rational when weights non-empty). Shared by the natural-UV face builder and
// the bounds-trimmed AdvancedFace builder.
Handle(Geom_BSplineSurface) make_bspline_surface(
        int u_degree, int v_degree,
        const std::vector<std::vector<std::array<double, 3>>> &control_points,
        const std::vector<double> &u_knots, const std::vector<double> &v_knots,
        const std::vector<int> &u_mults, const std::vector<int> &v_mults,
        const std::vector<std::vector<double>> &weights) {
    const int num_u = static_cast<int>(control_points.size());
    if (num_u == 0) throw std::runtime_error("make_bspline_surface: empty control point grid");
    const int num_v = static_cast<int>(control_points[0].size());

    TColgp_Array2OfPnt poles(1, num_u, 1, num_v);
    for (int u = 0; u < num_u; ++u) {
        for (int v = 0; v < num_v; ++v) {
            const auto &p = control_points[u][v];
            poles.SetValue(u + 1, v + 1, gp_Pnt(p[0], p[1], p[2]));
        }
    }
    TColStd_Array1OfReal knots_u(1, static_cast<int>(u_knots.size()));
    for (std::size_t i = 0; i < u_knots.size(); ++i) knots_u.SetValue(static_cast<int>(i) + 1, u_knots[i]);
    TColStd_Array1OfReal knots_v(1, static_cast<int>(v_knots.size()));
    for (std::size_t i = 0; i < v_knots.size(); ++i) knots_v.SetValue(static_cast<int>(i) + 1, v_knots[i]);
    TColStd_Array1OfInteger mults_u(1, static_cast<int>(u_mults.size()));
    for (std::size_t i = 0; i < u_mults.size(); ++i) mults_u.SetValue(static_cast<int>(i) + 1, u_mults[i]);
    TColStd_Array1OfInteger mults_v(1, static_cast<int>(v_mults.size()));
    for (std::size_t i = 0; i < v_mults.size(); ++i) mults_v.SetValue(static_cast<int>(i) + 1, v_mults[i]);

    if (!weights.empty()) {
        TColStd_Array2OfReal w(1, num_u, 1, num_v);
        for (int u = 0; u < num_u; ++u)
            for (int v = 0; v < num_v; ++v) w.SetValue(u + 1, v + 1, weights[u][v]);
        return new Geom_BSplineSurface(poles, w, knots_u, knots_v, mults_u, mults_v, u_degree, v_degree,
                                       Standard_False, Standard_False);
    }
    return new Geom_BSplineSurface(poles, knots_u, knots_v, mults_u, mults_v, u_degree, v_degree,
                                   Standard_False, Standard_False);
}

// Build a face edge from a 2D pcurve record (kind 6) laid on `surf`:
//   [6, degree, rational, closed, n_poles, <2*n_poles uv>, n_knots, <knots>,
//    <mults>, <n_poles weights if rational>]
// The 3D parametrization is derived by OCCT from surface(pcurve(t)), so 2D/3D
// stay consistent — the SAT-pcurve path adapy's make_face_from_geom prefers.
TopoDS_Edge edge_from_pcurve(const std::vector<double> &e, const Handle(Geom_Surface) &surf) {
    const int degree = static_cast<int>(std::lround(e[1]));
    const bool rational = std::lround(e[2]) != 0;
    const bool closed = std::lround(e[3]) != 0;
    std::size_t i = 4;
    const int n = static_cast<int>(std::lround(e[i++]));
    TColgp_Array1OfPnt2d poles(1, n);
    for (int p = 1; p <= n; ++p) {
        poles.SetValue(p, gp_Pnt2d(e[i], e[i + 1]));
        i += 2;
    }
    const int nk = static_cast<int>(std::lround(e[i++]));
    TColStd_Array1OfReal knots(1, nk);
    for (int k = 1; k <= nk; ++k) knots.SetValue(k, e[i++]);
    TColStd_Array1OfInteger mults(1, nk);
    for (int k = 1; k <= nk; ++k) mults.SetValue(k, static_cast<int>(std::lround(e[i++])));
    Handle(Geom2d_BSplineCurve) c2d;
    if (rational) {
        TColStd_Array1OfReal w(1, n);
        for (int p = 1; p <= n; ++p) w.SetValue(p, e[i++]);
        c2d = new Geom2d_BSplineCurve(poles, w, knots, mults, degree, closed);
    } else {
        c2d = new Geom2d_BSplineCurve(poles, knots, mults, degree, closed);
    }
    return BRepBuilderAPI_MakeEdge(c2d, surf, c2d->FirstParameter(), c2d->LastParameter()).Edge();
}

ShapeHandle build_bspline_surface_face_impl(
        int u_degree, int v_degree,
        const std::vector<std::vector<std::array<double, 3>>> &control_points,
        const std::vector<double> &u_knots, const std::vector<double> &v_knots,
        const std::vector<int> &u_mults, const std::vector<int> &v_mults,
        const std::vector<std::vector<double>> &weights, double tol) {
    Handle(Geom_BSplineSurface) surf =
            make_bspline_surface(u_degree, v_degree, control_points, u_knots, v_knots, u_mults, v_mults, weights);
    return ShapeHandle(BRepBuilderAPI_MakeFace(surf, tol).Face());
}

// Full bounds-trimmed AdvancedFace over a B-spline surface. Port of adapy's
// make_face_from_geom (SAT-pcurve path): build each boundary wire from its
// edges — a kind-6 record builds the edge from its 2D pcurve laid on the
// surface (3D derived as surface(pcurve(t))), any other kind builds a 3D edge —
// trim the surface to the outer wire, add inner wires as holes, then materialise
// 3D curves (BRepLib::BuildCurves3d). bounds[0] is the outer boundary.
ShapeHandle build_advanced_face_bspline_impl(
        int u_degree, int v_degree,
        const std::vector<std::vector<std::array<double, 3>>> &control_points,
        const std::vector<double> &u_knots, const std::vector<double> &v_knots,
        const std::vector<int> &u_mults, const std::vector<int> &v_mults,
        const std::vector<std::vector<double>> &weights,
        const std::vector<std::vector<std::vector<double>>> &bounds) {
    if (bounds.empty()) throw std::runtime_error("build_advanced_face: no bounds");
    Handle(Geom_BSplineSurface) surf =
            make_bspline_surface(u_degree, v_degree, control_points, u_knots, v_knots, u_mults, v_mults, weights);

    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges) {
            const int kind = static_cast<int>(std::lround(rec[0]));
            wm.Add(kind == 6 ? edge_from_pcurve(rec, surf) : edge_from_record(rec));
        }
        wm.Build();
        if (!wm.IsDone()) throw std::runtime_error("build_advanced_face: wire build failed");
        return wm.Wire();
    };

    BRepBuilderAPI_MakeFace fm(surf, wire_of(bounds[0]));
    for (std::size_t b = 1; b < bounds.size(); ++b) fm.Add(wire_of(bounds[b]));
    if (!fm.IsDone()) throw std::runtime_error("build_advanced_face: MakeFace failed");
    TopoDS_Face face = fm.Face();
    BRepLib::BuildCurves3d(face);  // pcurve-built edges have no 3D curve yet
    return ShapeHandle(face);
}

// Bounds-trimmed AdvancedFace over a PLANE surface (flat SAT/IFC plates). The supporting
// plane is INFERRED from the (planar) boundary wire via MakeFace(wire, OnlyPlane=true),
// which also computes each edge's 2D p-curve — including a b-spline boundary edge — so the
// face is correctly bounded by the wire and meshes. bounds[0] is the outer boundary, the
// rest are holes. loc/axis/ref_dir are accepted for signature parity but unused (the wire
// fixes the plane). Mirrors adapy's make_closed_shell_from_geom AdvancedFace(Plane) path.
ShapeHandle build_advanced_face_planar_impl(
        std::array<double, 3>, std::array<double, 3>, std::array<double, 3>,
        const std::vector<std::vector<std::vector<double>>> &bounds) {
    if (bounds.empty()) throw std::runtime_error("build_advanced_face_planar: no bounds");

    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges) wm.Add(edge_from_record(rec));
        wm.Build();
        if (!wm.IsDone()) throw std::runtime_error("build_advanced_face_planar: wire build failed");
        return wm.Wire();
    };

    BRepBuilderAPI_MakeFace fm(wire_of(bounds[0]), Standard_True);
    for (std::size_t b = 1; b < bounds.size(); ++b) fm.Add(wire_of(bounds[b]));
    if (!fm.IsDone()) throw std::runtime_error("build_advanced_face_planar: MakeFace failed");
    ShapeFix_Face fixer(fm.Face());
    fixer.Perform();
    return ShapeHandle(fixer.Face());
}

// Bounds-trimmed AdvancedFace over an explicitly-positioned analytic surface
// (cylinder / cone / torus). Unlike the planar path the surface is NOT inferred
// from the wire; the boundary wire trims the given surface. The 3D boundary edges
// (LINEs + CIRCLE arcs) carry no UV p-curve, so ShapeFix_Face projects them onto
// the surface and SameParameter reconciles 3D/2D so BRepMesh can grid the face.
// Mirrors adapy's make_closed_shell_from_geom analytic AdvancedFace path.
static ShapeHandle bounds_trimmed_analytic_face(
        const Handle(Geom_Surface) &surf,
        const std::vector<std::vector<std::vector<double>>> &bounds, const char *who) {
    if (bounds.empty()) throw std::runtime_error(std::string(who) + ": no bounds");

    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges) wm.Add(edge_from_record(rec));
        wm.Build();
        if (!wm.IsDone()) throw std::runtime_error(std::string(who) + ": wire build failed");
        return wm.Wire();
    };

    BRepBuilderAPI_MakeFace fm(surf, wire_of(bounds[0]), Standard_True);
    for (std::size_t b = 1; b < bounds.size(); ++b) fm.Add(wire_of(bounds[b]));
    if (!fm.IsDone()) throw std::runtime_error(std::string(who) + ": MakeFace failed");

    TopoDS_Face face = fm.Face();
    ShapeFix_Face fixer(face);
    fixer.Perform();
    face = fixer.Face();
    BRepLib::SameParameter(face, 1.0e-6, Standard_True);
    return ShapeHandle(face);
}

static gp_Ax3 _ax3(const std::array<double, 3> &loc, const std::array<double, 3> &axis,
                   const std::array<double, 3> &ref_dir) {
    return gp_Ax3(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]),
                  gp_Dir(ref_dir[0], ref_dir[1], ref_dir[2]));
}

ShapeHandle build_advanced_face_cylindrical_impl(
        std::array<double, 3> loc, std::array<double, 3> axis,
        std::array<double, 3> ref_dir, double radius,
        const std::vector<std::vector<std::vector<double>>> &bounds) {
    Handle(Geom_CylindricalSurface) surf = new Geom_CylindricalSurface(_ax3(loc, axis, ref_dir), radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_cylindrical");
}

ShapeHandle build_advanced_face_conical_impl(
        std::array<double, 3> loc, std::array<double, 3> axis,
        std::array<double, 3> ref_dir, double radius, double semi_angle,
        const std::vector<std::vector<std::vector<double>>> &bounds) {
    // Geom_ConicalSurface(ax3, semi_angle, ref_radius)
    Handle(Geom_ConicalSurface) surf = new Geom_ConicalSurface(_ax3(loc, axis, ref_dir), semi_angle, radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_conical");
}

ShapeHandle build_advanced_face_toroidal_impl(
        std::array<double, 3> loc, std::array<double, 3> axis,
        std::array<double, 3> ref_dir, double major_radius, double minor_radius,
        const std::vector<std::vector<std::vector<double>>> &bounds) {
    Handle(Geom_ToroidalSurface) surf = new Geom_ToroidalSurface(_ax3(loc, axis, ref_dir), major_radius, minor_radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_toroidal");
}

// Extract the 2D UV pcurve of an edge on a face (BRep_Tool::CurveOnSurface →
// trim → Geom2dConvert::CurveToBSplineCurve), plus the edge's 3D endpoints.
// Port of adapy's _extract_pcurve + _edge_endpoints. has_pcurve=false when OCC
// reports no curve-on-surface or the BSpline conversion fails.
PcurveData extract_pcurve(const TopoDS_Edge &edge, const TopoDS_Face &face) {
    PcurveData pc;
    const gp_Pnt pf = BRep_Tool::Pnt(TopExp::FirstVertex(edge, Standard_True));
    const gp_Pnt pl = BRep_Tool::Pnt(TopExp::LastVertex(edge, Standard_True));
    pc.start = {pf.X(), pf.Y(), pf.Z()};
    pc.end = {pl.X(), pl.Y(), pl.Z()};

    Standard_Real f = 0.0, l = 0.0;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (c2d.IsNull()) return pc;
    Handle(Geom2d_Curve) trimmed = (l > f) ? Handle(Geom2d_Curve)(new Geom2d_TrimmedCurve(c2d, f, l)) : c2d;
    Handle(Geom2d_BSplineCurve) bsp;
    try {
        bsp = Geom2dConvert::CurveToBSplineCurve(trimmed);
    } catch (const Standard_Failure &) {
        return pc;
    }
    if (bsp.IsNull()) return pc;

    pc.degree = bsp->Degree();
    for (int i = 1; i <= bsp->NbPoles(); ++i) {
        const gp_Pnt2d p = bsp->Pole(i);
        pc.control_points.push_back({p.X(), p.Y()});
    }
    for (int i = 1; i <= bsp->NbKnots(); ++i) {
        pc.knots.push_back(bsp->Knot(i));
        pc.multiplicities.push_back(bsp->Multiplicity(i));
    }
    if (bsp->IsRational()) {
        for (int i = 1; i <= bsp->NbPoles(); ++i) pc.weights.push_back(bsp->Weight(i));
    }
    pc.closed = bsp->IsClosed();
    pc.has_pcurve = true;
    return pc;
}

// Decompose a B-spline face into surface params + per-wire ordered edge pcurves.
// Reverse of build_advanced_face_bspline; port of adapy's occ_face_to_ada_face
// (+ get_bsplinesurface_with_knots / get_wires_from_face). Lets the SAT/STEP
// face→AdvancedFace round-trip run on adacpp with no pythonocc.
AdvancedFaceData face_to_advanced_face_impl(const ShapeHandle &sh) {
    const TopoDS_Shape &s = sh.topods();
    TopoDS_Face face;
    if (s.ShapeType() == TopAbs_FACE) {
        face = TopoDS::Face(s);
    } else {
        TopExp_Explorer exp(s, TopAbs_FACE);
        if (!exp.More()) throw std::runtime_error("face_to_advanced_face: shape has no face");
        face = TopoDS::Face(exp.Current());
    }
    Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face));
    if (bs.IsNull()) throw std::runtime_error("face_to_advanced_face: face surface is not a B-spline");

    AdvancedFaceData out;
    out.u_degree = bs->UDegree();
    out.v_degree = bs->VDegree();
    const int nu = bs->NbUPoles(), nv = bs->NbVPoles();
    out.poles.assign(nu, std::vector<std::array<double, 3>>(nv));
    const bool rational = bs->IsURational() || bs->IsVRational();
    if (rational) out.weights.assign(nu, std::vector<double>(nv, 1.0));
    for (int u = 1; u <= nu; ++u) {
        for (int v = 1; v <= nv; ++v) {
            const gp_Pnt p = bs->Pole(u, v);
            out.poles[u - 1][v - 1] = {p.X(), p.Y(), p.Z()};
            if (rational) out.weights[u - 1][v - 1] = bs->Weight(u, v);
        }
    }
    for (int i = 1; i <= bs->NbUKnots(); ++i) {
        out.u_knots.push_back(bs->UKnot(i));
        out.u_multiplicities.push_back(bs->UMultiplicity(i));
    }
    for (int i = 1; i <= bs->NbVKnots(); ++i) {
        out.v_knots.push_back(bs->VKnot(i));
        out.v_multiplicities.push_back(bs->VMultiplicity(i));
    }
    out.u_closed = bs->IsUClosed();
    out.v_closed = bs->IsVClosed();

    for (TopExp_Explorer we(face, TopAbs_WIRE); we.More(); we.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(we.Current());
        std::vector<PcurveData> bound;
        for (BRepTools_WireExplorer ee(wire, face); ee.More(); ee.Next()) {
            bound.push_back(extract_pcurve(ee.Current(), face));
        }
        if (!bound.empty()) out.bounds.push_back(std::move(bound));
    }
    return out;
}

// Prism-extrude a face by `thickness` along its surface normal at the
// parametric centre. Port of adapy's extrude_face_along_normal — gives a curved
// plate (PlateCurved) its thickness. Falls back to the bare face on thickness 0,
// undefined normal, or prism failure (matches adapy's render-something policy).
ShapeHandle extrude_face_along_normal_impl(const ShapeHandle &sh, double thickness) {
    const TopoDS_Shape &shape = sh.topods();
    if (thickness == 0.0) return ShapeHandle(shape);
    TopExp_Explorer exp(shape, TopAbs_FACE);
    if (!exp.More()) return ShapeHandle(shape);
    const TopoDS_Face sub_face = TopoDS::Face(exp.Current());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(sub_face);
    Standard_Real umin = 0.0, umax = 1.0, vmin = 0.0, vmax = 1.0;
    BRepTools::UVBounds(sub_face, umin, umax, vmin, vmax);
    GeomLProp_SLProps props(surf, (umin + umax) / 2.0, (vmin + vmax) / 2.0, 1, 1e-7);
    if (!props.IsNormalDefined()) return ShapeHandle(shape);
    const gp_Dir n = props.Normal();
    const gp_Vec vec(n.X() * thickness, n.Y() * thickness, n.Z() * thickness);
    BRepPrimAPI_MakePrism prism(shape, vec);
    if (!prism.IsDone()) return ShapeHandle(shape);
    return ShapeHandle(prism.Shape());
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

// Native ExtrudedAreaSolidTapered (tapered beams): port of adapy's
// make_extruded_area_shape_tapered_from_geom. Loft (BRepOffsetAPI_ThruSections)
// between the start profile wire (in the XY frame) and the end profile wire
// translated +Z by depth, then place via the gp_Ax3 change-of-basis. Only the
// outer wires are lofted — matches OCC, which takes wires()[0] of each profile
// face (inner voids are not carried through the taper).
ShapeHandle build_extruded_area_solid_tapered_impl(
        const std::vector<std::vector<double>> &outer_start,
        const std::vector<std::vector<double>> &outer_end,
        std::array<double, 3> loc, std::array<double, 3> axis, std::array<double, 3> ref_dir,
        double depth) {
    const TopoDS_Wire wire1 = wire_from_edges(outer_start);
    TopoDS_Wire wire2 = wire_from_edges(outer_end);
    // End profile sits at depth along +Z (identity rotation + Z translation).
    wire2 = TopoDS::Wire(place_at(wire2, {0.0, 0.0, depth}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}));

    BRepOffsetAPI_ThruSections ts(Standard_True);  // is_solid
    ts.AddWire(wire1);
    ts.AddWire(wire2);
    ts.Build();
    if (!ts.IsDone()) {
        throw std::runtime_error("build_extruded_area_solid_tapered: ThruSections failed");
    }
    return ShapeHandle(place_at(ts.Shape(), loc, axis, ref_dir));
}

// Generic loft: thread a ruled (or smooth) solid/shell through a sequence of
// closed polygon section profiles with BRepOffsetAPI_ThruSections. Port of
// adapy's ada.api.loft.loft_profiles. Each profile is a list of 3D points;
// the polygon is closed implicitly (last→first edge added).
ShapeHandle loft_profiles_impl(
        const std::vector<std::vector<std::array<double, 3>>> &profiles,
        bool ruled, bool solid) {
    if (profiles.size() < 2) {
        throw std::runtime_error("loft_profiles: need at least 2 profiles");
    }
    BRepOffsetAPI_ThruSections ts(solid, ruled);
    for (const auto &poly : profiles) {
        if (poly.size() < 2) {
            throw std::runtime_error("loft_profiles: a profile needs at least 2 points");
        }
        BRepBuilderAPI_MakeWire wm;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const auto &a = poly[i];
            const auto &b = poly[(i + 1) % poly.size()];
            wm.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(a[0], a[1], a[2]), gp_Pnt(b[0], b[1], b[2])).Edge());
        }
        wm.Build();
        ts.AddWire(wm.Wire());
    }
    ts.Build();
    if (!ts.IsDone()) {
        throw std::runtime_error("loft_profiles: ThruSections failed");
    }
    return ShapeHandle(ts.Shape());
}

// Boolean-intersect a shape with a finite planar face (cross-section). Port of
// adapy's ada.api.loft.intersect_with_plane: build a `2*size` square face on
// the plane (origin, normal) and common it with the shape. `size` must exceed
// the shape's lateral extent so the result is the full cross-section.
ShapeHandle section_with_plane_impl(
        const ShapeHandle &shape, std::array<double, 3> origin,
        std::array<double, 3> normal, double size) {
    const gp_Pln pln(gp_Pnt(origin[0], origin[1], origin[2]),
                     gp_Dir(normal[0], normal[1], normal[2]));
    const TopoDS_Shape face = BRepBuilderAPI_MakeFace(pln, -size, size, -size, size).Face();
    return ShapeHandle(BRepAlgoAPI_Common(shape.topods(), face).Shape());
}

// Write shapes (with per-shape name + color) to a STEP file via the OCAF/XCAF
// document model (adacpp's bundled OCCT). Backs adapy's StepWriter under the
// adacpp CAD backend, so STEP export needs no pythonocc.
// Serialize a shape to OCCT's BRepTools_ShapeSet text form (FormatNb 2), the
// same string adapy's pyocc path produces via serialize_shape_via_shapeset and
// feeds to ifcopenshell.geom.serialise. Lets the IFC tessellation fallback work
// under the adacpp backend (ifcopenshell's wasm wheel ships no occ_utils, and
// the shape here is an adacpp handle, not a pythonocc TopoDS). The BREP string
// crosses the module boundary as plain text — no OCC object is shared, so the
// two private OCCT copies never interpose.
std::string serialize_brep_impl(const ShapeHandle &sh) {
    BRepTools_ShapeSet ss;
    ss.SetFormatNb(2);
    ss.Add(sh.topods());
    std::ostringstream oss;
    ss.Write(oss);
    return oss.str();
}

void write_step_impl(const std::vector<ShapeHandle> &shapes,
                     const std::vector<std::string> &names,
                     const std::vector<std::array<double, 3>> &colors,
                     const std::string &filename,
                     const std::string &unit, const std::string &schema) {
    std::vector<TopoDS_Shape> tshapes;
    tshapes.reserve(shapes.size());
    for (const auto &s : shapes) tshapes.push_back(s.topods());
    std::vector<Color> cs;
    cs.reserve(colors.size());
    for (const auto &c : colors) {
        cs.emplace_back(static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]), 1.0f);
    }
    write_shapes_to_step(filename, tshapes, names, cs, unit, schema, "Assembly");
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

// Native FixedReferenceSweptAreaSolid (PrimSweep / pipe bends): port of adapy's
// make_fixed_reference_swept_area_shape_from_geom. Sweep the profile *wire*
// (the swept_area outer curve, already positioned in 3D at the directrix
// start) along the directrix spine with BRepOffsetAPI_MakePipeShell, using the
// round-corner transition for clean bends, then make a solid and translate to
// the position location. Inner voids are not swept (matches OCC, which only
// extracts the outer wire from the profile face).
ShapeHandle build_fixed_reference_swept_area_solid_impl(
        const std::vector<std::vector<double>> &directrix,
        const std::vector<std::vector<double>> &profile_outer,
        std::array<double, 3> loc) {
    const TopoDS_Wire spine = wire_from_edges(directrix);
    const TopoDS_Wire profile_wire = wire_from_edges(profile_outer);

    BRepOffsetAPI_MakePipeShell mps(spine);
    mps.SetTransitionMode(BRepBuilderAPI_RoundCorner);
    mps.Add(profile_wire, Standard_True, Standard_False);  // with contact, no correction
    mps.Build();
    mps.MakeSolid();
    TopoDS_Shape solid = mps.Shape();

    gp_Trsf tr;
    tr.SetTranslation(gp_Vec(loc[0], loc[1], loc[2]));
    return ShapeHandle(BRepBuilderAPI_Transform(solid, tr, true).Shape());
}

// Native IfcSweptDiskSolid (pipes / rods): sweep a circular disk (optionally
// annular) along the directrix. Port of adapy's make_swept_disk_solid_from_geom
// — the disk is placed at the spine start, normal to the start tangent, and
// MakePipeShell keeps it perpendicular along the spine; an inner radius is swept
// the same way and cut out. The directrix arrives as edge records (so any curve
// kind adapy can encode — line/arc/composite — works uniformly).
ShapeHandle build_swept_disk_solid_impl(
        const std::vector<std::vector<double>> &directrix,
        double radius,
        double inner_radius) {
    const TopoDS_Wire spine = wire_from_edges(directrix);

    // Start point + tangent of the spine (a circle is rotationally symmetric, so
    // the tangent sign is irrelevant — only the plane it defines matters).
    TopExp_Explorer exp(spine, TopAbs_EDGE);
    if (!exp.More()) throw std::runtime_error("build_swept_disk_solid: empty directrix");
    const TopoDS_Edge first_edge = TopoDS::Edge(exp.Current());
    BRepAdaptor_Curve adaptor(first_edge);
    gp_Pnt p0;
    gp_Vec d0;
    adaptor.D1(adaptor.FirstParameter(), p0, d0);
    if (d0.Magnitude() < 1e-12) throw std::runtime_error("build_swept_disk_solid: degenerate start tangent");
    const gp_Ax2 disk_axis(p0, gp_Dir(d0));

    auto sweep = [&](double r) -> TopoDS_Shape {
        const TopoDS_Edge circ_edge = BRepBuilderAPI_MakeEdge(gp_Circ(disk_axis, r)).Edge();
        const TopoDS_Wire profile = BRepBuilderAPI_MakeWire(circ_edge).Wire();
        BRepOffsetAPI_MakePipeShell mps(spine);
        mps.SetTransitionMode(BRepBuilderAPI_RoundCorner);
        mps.Add(profile, Standard_True, Standard_False);
        mps.Build();
        mps.MakeSolid();
        return mps.Shape();
    };

    TopoDS_Shape solid = sweep(radius);
    if (inner_radius > 0.0) {
        solid = BRepAlgoAPI_Cut(solid, sweep(inner_radius)).Shape();
    }
    return ShapeHandle(solid);
}

// ----------------------------------------------------------------------------
// cut_surfaces — extract polyline boundaries of CSG cut faces (native port of
// adapy's ada.occ.cut_surfaces_occ). Self-contained in adacpp's own OCCT; does
// not touch pythonocc. Returns the same plain-data contract:
//   [(surface_type, (nx,ny,nz),
//     [(edge_type, [(x,y,z)...])...],   # outer edges
//     [(x,y,z)...],                     # outer polyline
//     [[(x,y,z)...]...])]               # inner polylines
// ----------------------------------------------------------------------------

using Pt3 = std::array<double, 3>;
using EdgeData = std::tuple<std::string, std::vector<Pt3>>;
using SurfData = std::tuple<std::string, Pt3, std::vector<EdgeData>,
                            std::vector<Pt3>, std::vector<std::vector<Pt3>>>;

std::string cs_surface_type_name(const TopoDS_Face &face) {
    BRepAdaptor_Surface surf(face, Standard_True);
    switch (surf.GetType()) {
        case GeomAbs_Plane:    return "Plane";
        case GeomAbs_Cylinder: return "Cylinder";
        case GeomAbs_Cone:     return "Cone";
        case GeomAbs_Sphere:   return "Sphere";
        default:               return "Other";
    }
}

Pt3 cs_face_normal(const TopoDS_Face &face) {
    BRepAdaptor_Surface surf(face, Standard_True);
    Pt3 d;
    if (surf.GetType() == GeomAbs_Plane) {
        const gp_Dir n = surf.Plane().Axis().Direction();
        d = {n.X(), n.Y(), n.Z()};
    } else {
        const double um = 0.5 * (surf.FirstUParameter() + surf.LastUParameter());
        const double vm = 0.5 * (surf.FirstVParameter() + surf.LastVParameter());
        gp_Pnt p; gp_Vec du, dv;
        surf.D1(um, vm, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-12) return {0.0, 0.0, 1.0};
        n.Normalize();
        d = {n.X(), n.Y(), n.Z()};
    }
    if (face.Orientation() == TopAbs_REVERSED) { d[0] = -d[0]; d[1] = -d[1]; d[2] = -d[2]; }
    return d;
}

std::string cs_curve_type_name(const BRepAdaptor_Curve &c) {
    switch (c.GetType()) {
        case GeomAbs_Line:         return "Line";
        case GeomAbs_Circle:       return "Circle";
        case GeomAbs_Ellipse:      return "Ellipse";
        case GeomAbs_Hyperbola:    return "Hyperbola";
        case GeomAbs_Parabola:     return "Parabola";
        case GeomAbs_BezierCurve:  return "BezierCurve";
        case GeomAbs_BSplineCurve: return "BSplineCurve";
        case GeomAbs_OffsetCurve:  return "OffsetCurve";
        default:                   return "Other";
    }
}

std::vector<Pt3> cs_edge_to_points(const TopoDS_Edge &edge, double deflection) {
    BRepAdaptor_Curve c(edge);
    std::vector<Pt3> pts;
    if (c.GetType() != GeomAbs_Line) {
        GCPnts_UniformDeflection sampler(c, deflection);
        if (sampler.IsDone() && sampler.NbPoints() >= 2) {
            for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
                const gp_Pnt p = sampler.Value(i);
                pts.push_back({p.X(), p.Y(), p.Z()});
            }
            return pts;
        }
    }
    const gp_Pnt p0 = c.Value(c.FirstParameter());
    const gp_Pnt p1 = c.Value(c.LastParameter());
    pts.push_back({p0.X(), p0.Y(), p0.Z()});
    pts.push_back({p1.X(), p1.Y(), p1.Z()});
    return pts;
}

double cs_point_dist(const Pt3 &a, const Pt3 &b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<EdgeData> cs_wire_to_edges(const TopoDS_Wire &wire, double deflection, double tol) {
    std::vector<EdgeData> edges;
    for (BRepTools_WireExplorer ex(wire); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = ex.Current();
        const std::string etype = cs_curve_type_name(BRepAdaptor_Curve(edge));
        std::vector<Pt3> pts = cs_edge_to_points(edge, deflection);
        if (ex.Orientation() == TopAbs_REVERSED) std::reverse(pts.begin(), pts.end());
        if (!edges.empty() && !pts.empty()) {
            const auto &prev = std::get<1>(edges.back());
            if (cs_point_dist(prev.back(), pts.front()) <= tol) pts.front() = prev.back();
        }
        if (pts.size() >= 2) edges.emplace_back(etype, std::move(pts));
    }
    return edges;
}

std::vector<Pt3> cs_edges_to_polyline(const std::vector<EdgeData> &edges, double tol) {
    std::vector<Pt3> poly;
    for (const auto &e : edges) {
        const auto &pts = std::get<1>(e);
        if (poly.empty()) {
            poly.insert(poly.end(), pts.begin(), pts.end());
        } else if (cs_point_dist(poly.back(), pts.front()) <= tol) {
            poly.insert(poly.end(), pts.begin() + 1, pts.end());
        } else {
            poly.insert(poly.end(), pts.begin(), pts.end());
        }
    }
    if (poly.size() >= 2 && cs_point_dist(poly.front(), poly.back()) <= tol) poly.pop_back();
    return poly;
}

ShapeHandle make_halfspace_impl(std::array<double, 3> origin, std::array<double, 3> normal, bool flip) {
    const gp_Pln pln(gp_Pnt(origin[0], origin[1], origin[2]),
                     gp_Dir(normal[0], normal[1], normal[2]));
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(pln).Face();
    const double off = flip ? -1.0 : 1.0;
    const gp_Pnt ref(origin[0] + normal[0] * off,
                     origin[1] + normal[1] * off,
                     origin[2] + normal[2] * off);
    return ShapeHandle(BRepPrimAPI_MakeHalfSpace(face, ref).Solid());
}

std::vector<SurfData> cut_surfaces_impl(const ShapeHandle &solid_sh,
                                        const std::vector<ShapeHandle> &cutters,
                                        double deflection, double tol) {
    TopoDS_Shape current = solid_sh.topods();

    // Faces descending from the original solid (oriented identity, matching
    // adapy's set semantics). Cut faces are those NOT in this set.
    TopTools_MapOfOrientedShape descendants;
    for (TopExp_Explorer ex(current, TopAbs_FACE); ex.More(); ex.Next())
        descendants.Add(ex.Current());

    for (const ShapeHandle &ch : cutters) {
        BRepAlgoAPI_Cut algo(current, ch.topods());
        algo.Build();
        if (!algo.IsDone()) throw std::runtime_error("cut_surfaces: boolean cut failed");
        TopTools_MapOfOrientedShape next;
        for (TopTools_MapOfOrientedShape::Iterator dit(descendants); dit.More(); dit.Next()) {
            const TopoDS_Shape &f = dit.Value();
            const TopTools_ListOfShape &mod = algo.Modified(f);
            if (!mod.IsEmpty()) {
                for (TopTools_ListOfShape::Iterator mit(mod); mit.More(); mit.Next())
                    next.Add(mit.Value());
            } else if (!algo.IsDeleted(f)) {
                next.Add(f);
            }
        }
        descendants = next;
        current = algo.Shape();
    }

    std::vector<SurfData> out;
    for (TopExp_Explorer ex(current, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face rf = TopoDS::Face(ex.Current());
        if (descendants.Contains(rf)) continue;

        const TopoDS_Wire outer_wire = BRepTools::OuterWire(rf);
        std::vector<EdgeData> outer_edges = cs_wire_to_edges(outer_wire, deflection, tol);
        std::vector<Pt3> outer = cs_edges_to_polyline(outer_edges, tol);
        if (outer.size() < 3) continue;

        std::vector<std::vector<Pt3>> inners;
        for (TopExp_Explorer we(rf, TopAbs_WIRE); we.More(); we.Next()) {
            const TopoDS_Wire w = TopoDS::Wire(we.Current());
            if (w.IsSame(outer_wire)) continue;
            inners.push_back(cs_edges_to_polyline(cs_wire_to_edges(w, deflection, tol), tol));
        }
        out.emplace_back(cs_surface_type_name(rf), cs_face_normal(rf),
                         std::move(outer_edges), std::move(outer), std::move(inners));
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

// step2glb merge cleanup: meshopt_simplify (LockBorder) toward threshold*index_count within
// target_error, then drop degenerate tris + compact. Returns (positions xyz-interleaved, indices).
std::pair<std::vector<float>, std::vector<uint32_t>> meshopt_simplify_mesh_impl(
    const std::vector<float> &positions, const std::vector<uint32_t> &indices, float threshold,
    float target_error) {
    ngeom::SimplifiedMesh r = ngeom::meshopt_simplify_mesh(positions, indices, threshold, target_error);
    return {std::move(r.positions), std::move(r.indices)};
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
            .def_ro("length",  &GroupReference::length)
            .def_ro("vstart",  &GroupReference::vstart)
            .def_ro("vlength", &GroupReference::vlength);

    // Expose the bulk buffers as zero-copy, read-only NumPy views instead of
    // copying each std::vector into a Python list on every access (the old
    // def_ro behaviour via nanobind/stl/vector.h). For a mesh with thousands of
    // triangles this turns an O(n) per-access PyObject allocation into a cheap
    // array view. ``nb::find(self)`` is the owning Python Mesh, passed as the
    // ndarray owner so the underlying vector stays alive while the view exists.
    // Empty vectors are returned as length-0 arrays (data() may be null, which
    // is valid for a zero-element shape).
    nb::class_<Mesh>(m, "Mesh")
            .def_ro("id",        &Mesh::id)
            .def_prop_ro("positions", [](Mesh &self) {
                return nb::ndarray<nb::numpy, const float, nb::ndim<1>>(
                        self.positions.data(), {self.positions.size()}, nb::find(self));
            })
            .def_prop_ro("indices", [](Mesh &self) {
                return nb::ndarray<nb::numpy, const uint32_t, nb::ndim<1>>(
                        self.indices.data(), {self.indices.size()}, nb::find(self));
            })
            .def_prop_ro("normals", [](Mesh &self) {
                return nb::ndarray<nb::numpy, const float, nb::ndim<1>>(
                        self.normals.data(), {self.normals.size()}, nb::find(self));
            })
            .def_prop_ro("edges", [](Mesh &self) {
                return nb::ndarray<nb::numpy, const uint32_t, nb::ndim<1>>(
                        self.edges.data(), {self.edges.size()}, nb::find(self));
            })
            .def_ro("mesh_type", &Mesh::mesh_type)
            .def_ro("color",     &Mesh::color)
            .def_ro("groups",    &Mesh::group_reference);

    nb::class_<PcurveData>(m, "PcurveData")
            .def_ro("has_pcurve",      &PcurveData::has_pcurve)
            .def_ro("degree",          &PcurveData::degree)
            .def_ro("control_points",  &PcurveData::control_points)
            .def_ro("knots",           &PcurveData::knots)
            .def_ro("multiplicities",  &PcurveData::multiplicities)
            .def_ro("weights",         &PcurveData::weights)
            .def_ro("closed",          &PcurveData::closed)
            .def_ro("start",           &PcurveData::start)
            .def_ro("end",             &PcurveData::end);

    nb::class_<AdvancedFaceData>(m, "AdvancedFaceData")
            .def_ro("u_degree",         &AdvancedFaceData::u_degree)
            .def_ro("v_degree",         &AdvancedFaceData::v_degree)
            .def_ro("poles",            &AdvancedFaceData::poles)
            .def_ro("u_knots",          &AdvancedFaceData::u_knots)
            .def_ro("v_knots",          &AdvancedFaceData::v_knots)
            .def_ro("u_multiplicities", &AdvancedFaceData::u_multiplicities)
            .def_ro("v_multiplicities", &AdvancedFaceData::v_multiplicities)
            .def_ro("weights",          &AdvancedFaceData::weights)
            .def_ro("u_closed",         &AdvancedFaceData::u_closed)
            .def_ro("v_closed",         &AdvancedFaceData::v_closed)
            .def_ro("bounds",           &AdvancedFaceData::bounds);

    nb::class_<StepShapeData>(m, "StepShapeData")
            .def_ro("shape",     &StepShapeData::shape)
            .def_ro("name",      &StepShapeData::name)
            .def_ro("color",     &StepShapeData::color)
            .def_ro("has_color", &StepShapeData::has_color);

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

    m.def("tessellate_batch", &tessellate_batch_impl,
          "shapes"_a, "linear_deflection"_a = -1.0,
          "Tessellate many shapes into ONE combined Mesh in a single call, with a "
          "GroupReference per input shape (node_id=index, start/length=triangle-index "
          "range). Amortizes the per-call boundary cost and returns one zero-copy "
          "buffer. linear_deflection<=0 selects a per-shape bbox heuristic.");

    m.def("tessellate_stream", &tessellate_stream_impl,
          "buffer"_a, "pipeline"_a = "libtess2", "deflection"_a = 0.0, "angular_deg"_a = 20.0,
          "Decode an NGEOM stream buffer (adapy ada.geom, neutral schema) and tessellate "
          "every instance into ONE combined Mesh with a GroupReference per root "
          "(node_id = root index). pipeline: 'libtess2' (OCC-free) | 'occ' | 'cgal' | "
          "'hybrid' (ifcopenshell taxonomy kernels). angular_deg in degrees.");

    m.def("meshopt_simplify_mesh", &meshopt_simplify_mesh_impl,
          "positions"_a, "indices"_a, "threshold"_a = 0.75f, "target_error"_a = 0.0f,
          "step2glb merge cleanup: meshopt_simplify (border-locked) toward threshold*index_count "
          "within target_error, then drop degenerate triangles + compact. positions xyz-interleaved "
          "float, indices uint32. Returns (positions, indices). target_error 0.0 = lossless "
          "coplanar-triangle collapse.");

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

    m.def("read_step_shapes", &read_step_shapes_impl, "data"_a, "unit"_a = "M",
          "Read a STEP file (bytes) via OCAF into a list of StepShapeData (shape + "
          "label name + color), converting the file's length unit to `unit` (default M). "
          "Backs adapy's StepStore under the adacpp doc backend.");

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

    m.def("area", &area_impl, "shape"_a, "Total surface area (GProp).");
    m.def("shape_type", &shape_type_impl, "shape"_a,
          "Topological kind: solid/shell/face/wire/edge/vertex/compound/compsolid.");
    m.def("face_surface_type", &face_surface_type_impl, "shape"_a,
          "Geometric surface kind of a face: plane/cylinder/cone/sphere/torus/bspline/...");

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

    m.def("build_extruded_area_solid_tapered", &build_extruded_area_solid_tapered_impl,
          "outer_start"_a, "outer_end"_a, "location"_a, "axis"_a, "ref_dir"_a, "depth"_a,
          "Tapered extruded area solid (tapered beams): loft (ThruSections) between "
          "the start outer profile and the end outer profile (placed +Z by `depth`), "
          "then placed at the Axis2Placement3D frame. Edge records as in "
          "build_extruded_area_solid; only the outer wires are lofted.");

    m.def("loft_profiles", &loft_profiles_impl,
          "profiles"_a, "ruled"_a = true, "solid"_a = true,
          "Loft a ruled (or smooth) solid/shell through >=2 closed polygon "
          "section profiles (each a list of 3D points) via ThruSections.");

    m.def("section_with_plane", &section_with_plane_impl,
          "shape"_a, "origin"_a, "normal"_a, "size"_a = 1000.0,
          "Boolean-intersect `shape` with a finite (2*size square) planar face "
          "at (origin, normal); returns the cross-section.");

    m.def("write_step", &write_step_impl,
          "shapes"_a, "names"_a, "colors"_a, "filename"_a, "unit"_a = "m", "schema"_a = "AP214",
          "Write shapes (with per-shape name + RGB color) to a STEP file with "
          "OCAF names/colors via adacpp's bundled OCCT (no pythonocc needed).");

    m.def("serialize_brep", &serialize_brep_impl, "shape"_a,
          "Serialize a shape to OCCT BRepTools_ShapeSet text (FormatNb 2) — the "
          "BREP string ifcopenshell.geom.serialise consumes for the IFC tessellation "
          "fallback (replaces pythonocc-only ifcopenshell.geom.occ_utils).");

    m.def("build_revolved_area_solid", &build_revolved_area_solid_impl,
          "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a,
          "axis_location"_a, "axis_direction"_a, "angle_deg"_a, "is_area"_a = true,
          "Revolved area solid (pipe-shell elbows): build the profile (same "
          "edge-record encoding as build_extruded_area_solid), place it at the "
          "Axis2Placement3D frame, then revolve around the world axis "
          "(axis_location, axis_direction) by angle_deg degrees. is_area=False "
          "revolves the outer wire alone.");

    m.def("build_fixed_reference_swept_area_solid", &build_fixed_reference_swept_area_solid_impl,
          "directrix"_a, "profile_outer"_a, "location"_a,
          "Fixed-reference swept area solid (PrimSweep / pipe bends): sweep the "
          "profile outer wire along the directrix spine (both in the same "
          "edge-record encoding as build_extruded_area_solid) with "
          "MakePipeShell + round-corner transitions, make a solid, and "
          "translate to `location`.");

    m.def("build_swept_disk_solid", &build_swept_disk_solid_impl,
          "directrix"_a, "radius"_a, "inner_radius"_a = 0.0,
          "Swept-disk solid (IfcSweptDiskSolid — pipes/rods): sweep a circular "
          "(annular when inner_radius>0) disk along the directrix spine (edge "
          "records). The disk is placed at the spine start normal to its tangent "
          "and kept perpendicular by MakePipeShell.");

    m.def("make_halfspace", &make_halfspace_impl,
          "origin"_a, "normal"_a, "flip"_a,
          "Infinite half-space solid bounded by the plane (origin, normal); "
          "`flip` selects which side is solid. Used as a CSG cutter.");

    m.def("cut_surfaces", &cut_surfaces_impl,
          "solid"_a, "cutters"_a, "deflection"_a, "tol"_a,
          "Cut `solid` by each cutter in turn (BRepAlgoAPI_Cut with boolean "
          "history) and return, for every result face originating from a "
          "cutter, plain data: (surface_type, (nx,ny,nz), outer_edges, "
          "outer_polyline, inner_polylines). Curved edges are discretized to "
          "`deflection`; points within `tol` are de-duplicated.");

    m.def("build_bspline_surface_face", &build_bspline_surface_face_impl,
          "u_degree"_a, "v_degree"_a, "control_points"_a, "u_knots"_a, "v_knots"_a,
          "u_mults"_a, "v_mults"_a, "weights"_a, "tol"_a = 1e-6,
          "Trimmed face over a B-spline surface (knots; rational if weights given). "
          "control_points row-major [num_u][num_v]; weights empty => non-rational. "
          "Natural-UV MakeFace (PlateCurved / loft-derived surfaces).");

    m.def("build_advanced_face_bspline", &build_advanced_face_bspline_impl,
          "u_degree"_a, "v_degree"_a, "control_points"_a, "u_knots"_a, "v_knots"_a,
          "u_mults"_a, "v_mults"_a, "weights"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a B-spline surface. bounds[0] is the outer "
          "boundary, the rest are holes; each edge is a 3D edge record or a kind-6 2D "
          "pcurve record laid on the surface. Ports make_face_from_geom (SAT-pcurve path).");

    m.def("build_advanced_face_planar", &build_advanced_face_planar_impl,
          "loc"_a, "axis"_a, "ref_dir"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Plane inferred from the (planar) boundary wire "
          "(MakeFace OnlyPlane). bounds[0] outer, rest holes; 3D edge records. loc/axis/ref_dir "
          "are accepted for parity but unused. Ports make_closed_shell_from_geom's planar path.");

    m.def("build_advanced_face_cylindrical", &build_advanced_face_cylindrical_impl,
          "loc"_a, "axis"_a, "ref_dir"_a, "radius"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_CylindricalSurface. loc is the cylinder "
          "axis base, axis its direction, ref_dir the U reference, radius its radius. bounds[0] "
          "outer, rest holes; 3D edge records (LINE/CIRCLE). Ports make_closed_shell_from_geom's "
          "AdvancedFace(CylindricalSurface) path for tube/pipe walls.");

    m.def("build_advanced_face_conical", &build_advanced_face_conical_impl,
          "loc"_a, "axis"_a, "ref_dir"_a, "radius"_a, "semi_angle"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_ConicalSurface (radius at the axis base, "
          "semi_angle the half-angle). bounds[0] outer, rest holes; 3D edge records.");

    m.def("build_advanced_face_toroidal", &build_advanced_face_toroidal_impl,
          "loc"_a, "axis"_a, "ref_dir"_a, "major_radius"_a, "minor_radius"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_ToroidalSurface (pipe elbows). "
          "bounds[0] outer, rest holes; 3D edge records.");

    m.def("face_to_advanced_face", &face_to_advanced_face_impl, "face"_a,
          "Decompose a B-spline face into AdvancedFaceData (surface poles/knots + per-wire "
          "ordered edge pcurves). Reverse of build_advanced_face_bspline; ports "
          "occ_face_to_ada_face so the SAT/STEP face round-trip runs on adacpp.");

    m.def("extrude_face_along_normal", &extrude_face_along_normal_impl,
          "face"_a, "thickness"_a,
          "Prism-extrude a face by `thickness` along its surface normal (PlateCurved "
          "thickness). Returns the bare face on thickness 0 / undefined normal / failure.");

    m.def("build_wire", &build_wire_impl, "edges"_a,
          "Build a wire from edge records (line/arc/circle/ellipse/bspline, full or "
          "parametrically trimmed). See edge_from_record for the record layout.");

    m.def("build_filled_face", &build_filled_face_impl, "edges"_a,
          "Wire-filled face (WireFilledFace): interpolate a smooth surface through "
          ">=3 boundary edges via BRepOffsetAPI_MakeFilling.");

    m.def("build_planar_face", &build_planar_face_impl,
          "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a,
          "Curve-bounded planar face (shell representation): outer profile face "
          "minus inner void faces, placed at the Axis2Placement3D frame. Same "
          "edge-record encoding as build_extruded_area_solid.");

    m.def("build_face_based_surface_model", &build_face_based_surface_model_impl,
          "polygons"_a,
          "Fuse a list of polygon faces (each a closed loop of 3D points) into "
          "one shell.");

    // --- topology-kernel ops ---
    m.def("make_volumes_from_faces", &make_volumes_from_faces_impl,
          "faces"_a, "tolerance"_a = 1e-6,
          "Partition space into solids from a face soup (BOPAlgo_MakerVolume). "
          "Interior faces come out shared between the two cells they separate.");

    m.def("sew_faces", &sew_faces_impl,
          "faces"_a, "tolerance"_a = 1e-6,
          "Sew faces into one connected shell (BRepBuilderAPI_Sewing). For OPEN "
          "surface models (IfcShellBasedSurfaceModel / open shell) that don't "
          "bound a volume, where make_volumes_from_faces would yield nothing.");

    m.def("polygon_face", &polygon_face_impl,
          "points"_a,
          "Planar face from a closed polygon of >=3 points (auto-closed). Divider "
          "faces for make_volumes_from_faces; ports adapy OccBackend.polygon_face.");

    m.def("non_manifold_merge", &non_manifold_merge_impl,
          "shapes"_a, "tolerance"_a = 1e-6, "glue"_a = true,
          "Non-manifold fuse (BOPAlgo_Builder) keeping coincident faces shared "
          "between adjacent solids rather than dissolving the partition.");

    m.def("merge_cells", &merge_cells_impl,
          "solids"_a, "tolerance"_a = 0.0,
          "Faithful port of topologic Topology::Merge over solids "
          "(BOPAlgo_CellsBuilder + per-operand AddToResult + MakeContainers): "
          "each input solid survives as a cell and every interface becomes one "
          "shared non-manifold face.");

    m.def("face_id", &face_id_impl,
          "face"_a,
          "Orientation-independent topological identity of a face (TShape "
          "pointer). Two cells referencing the same shared non-manifold face "
          "return the same id; distinct faces differ.");

    m.def("free_faces", &free_faces_impl,
          "solids"_a,
          "Faces owned by exactly one solid — the outer envelope (FACE→SOLID "
          "ancestor map over a compound of the cells).");

    m.def("point_in_solid", &point_in_solid_impl,
          "solid"_a, "point"_a, "tolerance"_a = 1e-6,
          "Classify a point against a solid (BRepClass3d_SolidClassifier). "
          "Returns TopAbs_State as int: IN=0, OUT=1, ON=2, UNKNOWN=3.");

    m.def("center_of_mass", &center_of_mass_impl,
          "shape"_a,
          "Centre of mass (x,y,z) — volume props for solids, surface props for "
          "shells/faces, linear props otherwise (BRepGProp).");

    m.def("shells", &shells_impl, "shape"_a, "List of shell sub-shapes.");
    m.def("wires", &wires_impl, "shape"_a, "List of wire sub-shapes.");
    m.def("wire_points", &wire_points_impl, "shape"_a,
          "Ordered boundary vertices of a face's outer wire (or a wire), in "
          "connection order — for rebuilding a face as a polygon.");
    m.def("unify_coplanar_faces", &unify_coplanar_faces_impl, "shape"_a,
          "Merge adjacent same-surface (coplanar) faces into single faces "
          "(ShapeUpgrade_UnifySameDomain).");
}
