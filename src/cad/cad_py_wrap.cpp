#include "cad_py_wrap.h"
#include "ShapeHandle.h"
#include "../geom/Color.h"
#include "../geom/GroupReference.h"
#include "../geom/Mesh.h"
#include "../geom/MeshType.h"
#include "../geom/mesh_spike.h"
#include "../cadit/occt/static_param_guard.h"
#include "../cadit/occt/step_writer.h"
#include "../cadit/ifc/ngeom_taxonomy.h"
#include "../geom/neutral/ngeom_decode.h"
#include "../geom/neutral/ngeom_encode.h"
#include "../geom/neutral/ada_ext_schema.h" // GENERATED from the adapy ADA_EXT_data JSON schema
#include "../geom/neutral/ngeom_glb.h"
#include "../geom/neutral/ngeom_profile.h"
#include "step_to_glb_st.h"      // single-threaded, mmap-free STEP->GLB core (wasm/OPFS + native oracle)
#include "step_to_glb_stream.h"  // threaded OCC-free STEP->GLB core (shared with the STP2GLB CLI)
#include "ifc_to_glb_stream.h"   // native OCC-free IFC->GLB core (IfcResolver)
#include "step_to_mesh_stream.h" // threaded OCC-free STEP->STL/OBJ core (parallel, baked, streaming)
#include "ifc_emit.h"            // native IFC4 advanced-B-rep emitter (Phase 1, native STEP->IFC writer)
#include "step_emit.h"           // native AP242 STEP advanced-B-rep emitter (native STEP->STEP writer)
#include "brep_file_convert.h"   // shared STEP<->IFC file writers (also compiled into the wasm writer)
#include "ifc_reader.h"          // native IFC advanced-B-rep reader -> ng:: (native IFC->STEP path)

#ifndef __EMSCRIPTEN__
#include "../diff/glb_diff_native.h" // GLB model-diff core (summary match + removed overlay); file/mmap-based, native-only
#endif
#include "../geom/neutral/ngeom_tessellate.h"
#include "../geom/neutral/ngeom_meshopt.h"
#include "../cadit/step/step_reader.h"

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
#include <functional>
#include <map>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <tuple>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "posix_compat.h"
#include "mem_trim.h"
#include "mem_tune.h"
#include "effective_concurrency.h"

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
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
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
    bool has_pcurve = false; // false → no UV curve recoverable
    int degree = 0;
    std::vector<std::array<double, 2>> control_points; // 2D UV poles
    std::vector<double> knots;
    std::vector<int> multiplicities;
    std::vector<double> weights; // empty → non-rational
    bool closed = false;
    std::array<double, 3> start{}; // 3D edge endpoints
    std::array<double, 3> end{};
};

struct AdvancedFaceData {
    int u_degree = 0, v_degree = 0;
    std::vector<std::vector<std::array<double, 3>>> poles; // [n_u][n_v]
    std::vector<double> u_knots, v_knots;
    std::vector<int> u_multiplicities, v_multiplicities;
    std::vector<std::vector<double>> weights; // [n_u][n_v], empty → non-rational
    bool u_closed = false, v_closed = false;
    std::vector<std::vector<PcurveData>> bounds; // [wire][edge]
};

// One shape extracted from a STEP OCAF document, with its label name + color.
struct StepShapeData {
    ShapeHandle shape;
    std::string name;
    std::array<double, 3> color{0.5, 0.5, 0.5};
    bool has_color = false;
};

// ----------------------------------------------------------------------------
// Result structs for imprint_planar_faces (the General-Fuse planar imprint).
// Plain data, bound read-only; adapy turns them into ACIS SAT topology for the
// Genie exporter. Mirrors ada.cad.PlanarImprint / ImprintedEdge / ImprintedFace.
// ----------------------------------------------------------------------------

// A straight edge, as indices into ImprintResult::vertices.
struct ImprintedEdge {
    int start = 0;
    int end = 0;
};

struct ImprintedFace {
    std::array<double, 3> origin{};
    std::array<double, 3> normal{};
    std::array<double, 3> ref_direction{};
    // loops[0] is the outer boundary, the rest are holes. Each entry is an
    // (edge index, forward) pair; forward=false means the loop runs that edge
    // from its `end` vertex to its `start`. Every loop winds counter-clockwise
    // about `normal`.
    std::vector<std::vector<std::pair<int, bool>>> loops;
};

struct ImprintResult {
    std::vector<std::array<double, 3>> vertices;
    std::vector<ImprintedEdge> edges;
    std::vector<ImprintedFace> faces;
    // sources[i] = the faces the i-th input outline became.
    std::vector<std::vector<int>> sources;
    // curve_sources[i] = the edges the i-th imprint curve became.
    std::vector<std::vector<int>> curve_sources;
    // Edges that bound no face (an imprint curve with no face under it). Still
    // reported, and still reachable via curve_sources: a consumer needs geometry
    // for them, carried as wire bodies rather than face boundaries.
    std::vector<int> free_edges;
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
    // Angular deflection 0.2 rad (~11.5deg), not OCC's loose 0.5 default: a large-radius
    // arc (e.g. a curved beam swept on a 7 m radius) otherwise facets into a handful of
    // segments — the linear deflection alone can't keep it smooth because the sag tolerance
    // scales with radius. Caps the segment span so revolves/pipes/quadrics read as curved.
    BRepMesh_IncrementalMesh(shape, linear_deflection, /*relative=*/Standard_False,
                             /*angular=*/0.2, /*parallel=*/Standard_True);

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
            if (!t.IsNull() && t->NbTriangles() > 0)
                any_tris = true;
        }
        if (!any_tris) {
            ShapeFix_Shape sf(shape);
            sf.Perform();
            const TopoDS_Shape fixed = sf.Shape();
            if (!fixed.IsNull()) {
                shape = fixed;
                BRepMesh_IncrementalMesh(shape, linear_deflection, /*relative=*/Standard_False,
                                         /*angular=*/0.2, /*parallel=*/Standard_True);
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
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(TopoDS::Face(exp.Current()), loc);
        if (tri.IsNull())
            continue;
        n_nodes += static_cast<size_t>(tri->NbNodes());
        n_tris += static_cast<size_t>(tri->NbTriangles());
    }
    positions.reserve(positions.size() + n_nodes * 3);
    indices.reserve(indices.size() + n_tris * 3);

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull())
            continue;

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
            if (reversed)
                std::swap(n2, n3);
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
Mesh tessellate_stream_impl(nb::object buffer, const std::string &pipeline, double deflection, double angular_deg,
                            nb::dict settings, int threads, double model_scale) {
    using namespace adacpp::ngeom;
    // Accept any buffer-protocol object (bytes, memoryview, the capsule-owned numpy
    // arrays StepNgeomStream/IfcNgeomStream yield) so a lazy ShapeStore blob reaches
    // the kernel with zero copies end-to-end.
    Py_buffer view{};
    if (PyObject_GetBuffer(buffer.ptr(), &view, PyBUF_SIMPLE) != 0) {
        PyErr_Clear();
        return Mesh(0, {}, {}); // not a buffer -> empty mesh
    }
    NgeomDoc doc;
    try {
        doc = decode(static_cast<const uint8_t *>(view.buf), (size_t) view.len);
    } catch (const std::exception &) {
        PyBuffer_Release(&view);
        return Mesh(0, {}, {}); // malformed buffer -> empty mesh
    }
    PyBuffer_Release(&view); // decode copies what it keeps; the view is no longer needed

    // ifcopenshell ConversionSettings overrides (taxonomy paths only). Any
    // python scalar is stringified; the C++ side parses per the setting type.
    std::vector<std::pair<std::string, std::string>> overrides;
    for (auto item : settings) {
        overrides.emplace_back(nb::cast<std::string>(item.first), nb::cast<std::string>(nb::str(item.second)));
    }

    // Route via the track vocabulary (ngeom_tess_track.h) rather than a raw string compare. The old
    // code sent anything that wasn't "libtess2" to the taxonomy branch and then split it on '-',
    // which silently misrouted every NGEOM track added since: "cdt" became taxonomy("cdt"), and
    // a dash-split sent "hybrid-cdt" to taxonomy("cdt"). Wrong geometry, not an error. parse_track
    // also rejects
    // an unknown name outright instead of guessing.
    auto track = parse_track(pipeline);
    if (!track)
        throw std::invalid_argument("unknown tessellation track: '" + pipeline + "' (see tess_tracks())");
    TessMesh tm;
    if (*track == TessTrack::Libtess2 || *track == TessTrack::Cdt) {
        TessParams tp;
        tp.track = *track;
        tp.deflection = deflection;
        tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;
        tp.threads = threads;         // >1 => parallelise a root's faces (opt-in; default serial)
        tp.model_scale = model_scale; // >0 => adaptive per-surface density (0 => fixed max_angle)
        tm = tessellate_doc(doc, tp);
    } else {
        // ifcopenshell taxonomy kernels: occ | cgal | hybrid.
        tm = tessellate_via_taxonomy(doc, track_name(*track), deflection, angular_deg, overrides);
    }

    std::vector<GroupReference> groups;
    groups.reserve(tm.groups.size());
    for (size_t i = 0; i < tm.groups.size(); ++i) {
        const auto &g = tm.groups[i];
        groups.emplace_back((int) i, (int) g.first_index, (int) g.index_count, (int) g.first_vertex,
                            (int) g.vertex_count);
    }
    Mesh mesh(0, std::move(tm.positions), std::move(tm.indices), {}, std::move(tm.normals), tm.mesh_type);
    mesh.group_reference = std::move(groups);
    return mesh;
}

// Per-root metadata accompanying the NGEOM buffer from stream_step_to_ngeom: the data the
// from_step hydrate path needs that the geometry buffer does NOT carry (colour, assembly
// placement matrices, and the assembly path for the part hierarchy). Parallel to the buffer's
// roots (same order).
struct StepRootMeta {
    std::string id;
    std::string guid; // IFC GlobalId (IfcNgeomStream); empty on the STEP path
    bool has_color = false;
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
    std::vector<std::array<float, 16>> transforms;                        // per-instance world matrices
    std::vector<std::vector<std::pair<int, std::string>>> instance_paths; // per-instance (rep_id, name) levels
};

// Hand a just-encoded NGEOM buffer to Python WITHOUT the nb::bytes memcpy: move the vector to the
// heap and expose it as a capsule-owned read-only numpy uint8 array. The consumer (adapy's lazy
// ShapeStore) retains the arriving object as-is, so the buffer is allocated exactly once.
static nb::object ngeom_buffer_to_ndarray(std::vector<uint8_t> &&buf) {
    auto *heap = new std::vector<uint8_t>(std::move(buf));
    // The encoder's growth-doubling leaves real capacity slack (~20% across the crane's
    // 7291 blobs); the consumer retains these long-term, so trade one bounded realloc
    // now for an exact-size resident buffer. Still ahead of nb::bytes: no slack case
    // reallocates nothing, and the vector+copy never coexist with a PyBytes duplicate.
    heap->shrink_to_fit();
    nb::capsule owner(heap, [](void *p) noexcept { delete static_cast<std::vector<uint8_t> *>(p); });
    size_t shape[1] = {heap->size()};
    return nb::cast(nb::ndarray<nb::numpy, const uint8_t, nb::ndim<1>>(heap->data(), 1, shape, owner));
}

// Native STEP -> NGEOM buffer + per-root metadata. Reads the .stp with the native C++ reader,
// re-encodes the resolved neutral records to one NGEOM buffer (decodable by the adapy Python
// deserializer into ada.geom.Geometry), and returns the per-root colour / transforms / assembly
// paths alongside. The geometry sibling of stream_step_to_meshes for the from_step hydrate path.
std::pair<nb::bytes, std::vector<StepRootMeta>> stream_step_to_ngeom_impl(const std::string &path) {
    using namespace adacpp::ngeom;
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return {nb::bytes("", 0), {}};
    std::stringstream ss;
    ss << f.rdbuf();
    std::string src = ss.str();

    std::vector<adacpp::step::Instance> store;
    NgeomDoc doc = adacpp::step::read_step_brep(src, store);
    std::vector<uint8_t> buf = encode(doc);

    std::vector<StepRootMeta> metas;
    metas.reserve(doc.roots.size());
    for (const NgeomRoot &r : doc.roots) {
        StepRootMeta m;
        m.id = r.id;
        m.has_color = r.has_color;
        m.color = {r.cr, r.cg, r.cb, r.ca};
        m.transforms = r.transforms;
        m.instance_paths = r.instance_paths;
        metas.push_back(std::move(m));
    }
    return {nb::bytes(reinterpret_cast<const char *>(buf.data()), buf.size()), std::move(metas)};
}

// Debug/parity hook: build the offset index both ways (mmap scan vs the wasm-safe pread scan) and
// report whether they are identical. Used to validate from_file_pread before it backs the wasm path.
nb::dict step_index_parity_impl(const std::string &path) {
    auto a = adacpp::step::StreamIndex::from_file(path);
    auto b = adacpp::step::StreamIndex::from_file_pread(path);
    nb::dict d;
    d["n_mmap"] = (long) a.ids.size();
    d["n_pread"] = (long) b.ids.size();
    d["ids_match"] = (a.ids == b.ids);
    d["offs_match"] = (a.offs == b.offs);
    d["roots_match"] = (a.lists.roots == b.lists.roots);
    d["units_match"] = (a.lists.units == b.lists.units);
    d["styled_match"] = (a.lists.styled == b.lists.styled);
    d["absr_match"] = (a.lists.absr == b.lists.absr);
    d["srr_match"] = (a.lists.srr == b.lists.srr);
    d["cdsr_match"] = (a.lists.cdsr == b.lists.cdsr);
    d["sdr_match"] = (a.lists.sdr == b.lists.sdr);
    return d;
}

// Streaming NGEOM emitter for the from_step hydrate path: resolve + encode ONE solid per __next__,
// freeing each solid's working set after — so memory stays at the offset index (~12 B/instance) plus
// a single solid, NOT the whole parsed model (stream_step_to_ngeom full-parses + OOMs on large files).
// Python-iterable: each item is (one-root NGEOM buffer, StepRootMeta). The index + resolver live on the
// heap so the resolver's index pointer stays valid even if Python moves the object.
class StepNgeomStream {
public:
    explicit StepNgeomStream(const std::string &path) : prof_("step_ngeom_stream") {
        idx_ = std::make_unique<adacpp::step::StreamIndex>(adacpp::step::StreamIndex::from_file(path));
        prof_.phase("scan_index");
        r_ = std::make_unique<adacpp::step::Resolver>(*idx_);
        r_->build_metadata(idx_->lists);
        // Single-threaded per-solid streaming → safe to bound parse_cache_ on giant shells (the
        // 67 MB single solid in 469826); the multi-threaded mesh/glb path must NOT (race on re-parse).
        r_->enable_parse_cache_bounding();
        prof_.phase("metadata");
    }
    ~StepNgeomStream() {
        // The iterate phase spans the whole consumption window (incl. the Python
        // consumer's time between __next__ calls); resolve_encode_ms is the pure
        // C++ share, so the gap between them is the consumer's own cost.
        if (prof_.on()) {
            prof_.phase("iterate(incl_consumer)");
            prof_.note("resolve_encode_ms", work_ms_);
        }
    }
    nb::tuple next() {
        using namespace adacpp::ngeom;
        // Zero cost when profiling is off: one bool test, no clock reads.
        const bool on = prof_.on();
        auto t0 = on ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        auto charge = [&] {
            if (on)
                work_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
        const auto &roots = idx_->lists.roots;
        while (cursor_ < roots.size()) {
            NgeomRoot root = r_->resolve_root(roots[cursor_++]);
            if (root.id.empty()) {
                r_->clear_geom_cache();
                continue;
            }
            prof_.solid(root.faces.size());
            StepRootMeta m;
            m.id = root.id;
            m.has_color = root.has_color;
            m.color = {root.cr, root.cg, root.cb, root.ca};
            m.transforms = root.transforms;
            m.instance_paths = root.instance_paths;
            NgeomDoc one;
            one.roots.push_back(std::move(root));
            std::vector<uint8_t> buf = encode(one);
            r_->clear_geom_cache();
            nb::tuple out = nb::make_tuple(ngeom_buffer_to_ndarray(std::move(buf)), std::move(m));
            charge();
            return out;
        }
        charge();
        PyErr_SetNone(PyExc_StopIteration);
        throw nb::python_error();
    }

private:
    adacpp::prof::StepProfiler prof_; // declared first → destroyed last (summary sees the notes)
    double work_ms_ = 0;
    std::unique_ptr<adacpp::step::StreamIndex> idx_;
    std::unique_ptr<adacpp::step::Resolver> r_;
    size_t cursor_ = 0;
};

// Streaming per-product IFC -> NGEOM: the IFC sibling of StepNgeomStream, built on the dep-free
// IfcResolver (no ifcopenshell, no OCC). One product is resolved + encoded per __next__ and the
// resolver's statement cache is cleared between products, so memory stays at the offset index plus
// a single product. Yields (one-root NGEOM buffer, StepRootMeta) with meta.guid = the product's
// IFC GlobalId. Geometry is in FILE units — apply .unit_scale (metres per unit) on the consumer
// side. Products the analytic resolver can't represent (tessellated face sets, mixed multi-solid
// reps) are skipped and counted in .products_skipped so the consumer can fall back per-file.
// meta now carries colour (IfcStyledItem -> IfcColourRgb) + the spatial-structure path
// (IfcRelContainedInSpatialStructure + IfcRelAggregates walk -> instance_paths), alongside
// geometry/guid/name/placement — so a native reader gets the full tree + colours.
class IfcNgeomStream {
public:
    explicit IfcNgeomStream(const std::string &path) : prof_("ifc_ngeom_stream") {
        idx_ = std::make_unique<adacpp::step::StreamIndex>(adacpp::step::StreamIndex::from_file(path));
        prof_.phase("scan_index");
        r_ = std::make_unique<adacpp::ifc_read::IfcResolver>(*idx_);
        roots_ = r_->proxy_roots();
        unit_scale_ = r_->unit_scale();
        prof_.phase("metadata");
    }
    ~IfcNgeomStream() {
        if (prof_.on()) {
            prof_.phase("iterate(incl_consumer)");
            prof_.note("resolve_encode_ms", work_ms_);
            prof_.note("products_skipped", (double) skipped_);
        }
    }
    double unit_scale() const {
        return unit_scale_;
    }
    long products_total() const {
        return (long) roots_.size();
    }
    long products_skipped() const {
        return skipped_;
    }
    nb::tuple next() {
        using namespace adacpp::ngeom;
        // Zero cost when profiling is off: one bool test, no clock reads.
        const bool on = prof_.on();
        auto t0 = on ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        auto charge = [&] {
            if (on)
                work_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
        while (cursor_ < roots_.size()) {
            long pid = roots_[cursor_++];
            NgeomRoot root = r_->resolve_product(pid);
            std::string guid = r_->product_guid(pid);
            r_->clear_cache(); // bounded memory: statement/surface caches don't grow across products
            // The encoder now emits analytic solids (extrusion/revolve/boolean/sphere, tags 50-53)
            // in addition to face sets, so a procedural root carries real geometry. Only a root with
            // NO encodable geometry (empty faces + not one of those solids — e.g. an alignment sweep
            // or an unsupported product) still yields nothing; skip it for the kernel fallback.
            bool has_solid =
                root.extrusion || root.revolve || root.boolean || root.sphere || root.sweep || !root.polylines.empty();
            if (root.faces.empty() && !has_solid) {
                // recognized_empty => a degenerate product we DID understand (zero-length curve marker);
                // don't count it as an unsupported skip (that would drive a wasted OCC fallback).
                if (!root.recognized_empty)
                    ++skipped_;
                continue;
            }
            prof_.solid(root.faces.size());
            StepRootMeta m;
            m.id = root.id;
            m.guid = std::move(guid);
            m.has_color = root.has_color;
            m.color = {root.cr, root.cg, root.cb, root.ca};
            m.transforms = root.transforms;
            m.instance_paths = root.instance_paths;
            NgeomDoc one;
            one.roots.push_back(std::move(root));
            std::vector<uint8_t> buf = encode(one);
            nb::tuple out = nb::make_tuple(ngeom_buffer_to_ndarray(std::move(buf)), std::move(m));
            charge();
            return out;
        }
        charge();
        PyErr_SetNone(PyExc_StopIteration);
        throw nb::python_error();
    }

private:
    adacpp::prof::StepProfiler prof_; // declared first → destroyed last (summary sees the notes)
    double work_ms_ = 0;
    std::unique_ptr<adacpp::step::StreamIndex> idx_;
    std::unique_ptr<adacpp::ifc_read::IfcResolver> r_;
    std::vector<long> roots_;
    double unit_scale_ = 1.0;
    long skipped_ = 0;
    size_t cursor_ = 0;
};

// Native STEP -> Mesh: stream the .stp with the native C++ reader (offset index + per-solid lazy
// resolve, no OCC/Python), tessellate each solid as it streams, and append into ONE combined Mesh
// with a GroupReference per root. Memory stays bounded on the parse side (the index + a single
// solid), unlike the naive full-parse. The fully-native counterpart of tessellate_stream.
Mesh stream_step_to_meshes_impl(const std::string &path, const std::string &pipeline, double deflection,
                                double angular_deg) {
    using namespace adacpp::ngeom;
    (void) pipeline; // only the OCC-free libtess2 kernel is wired for the native path
    adacpp::prof::StepProfiler prof("stream_step_to_meshes");

    std::ifstream f(path, std::ios::binary);
    if (!f)
        return Mesh(0, {}, {});
    std::stringstream ss;
    ss << f.rdbuf();
    std::string buf = ss.str();
    prof.phase("read_file");

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * 3.14159265358979323846 / 180.0;

    Mesh out(0, {}, {});
    std::vector<GroupReference> groups;
    adacpp::step::stream_step(buf, [&](const NgeomRoot &root, double) {
        NgeomDoc one;
        one.roots.push_back(root); // shares geometry shared_ptrs
        TessMesh tm = tessellate_doc(one, tp);
        if (tm.indices.empty())
            return;
        prof.solid(tm.indices.size() / 3);
        uint32_t vstart = (uint32_t) (out.positions.size() / 3);
        uint32_t istart = (uint32_t) out.indices.size();
        out.positions.insert(out.positions.end(), tm.positions.begin(), tm.positions.end());
        out.normals.insert(out.normals.end(), tm.normals.begin(), tm.normals.end());
        for (uint32_t ix : tm.indices)
            out.indices.push_back(vstart + ix);
        groups.emplace_back((int) groups.size(), (int) istart, (int) tm.indices.size(), (int) vstart,
                            (int) (tm.positions.size() / 3));
    });
    prof.phase("stream(resolve+tess)");
    out.group_reference = std::move(groups);
    return out;
}

// Native STEP -> GLB file: stream the .stp, tessellate each solid, bake its world transform(s) and
// colour, and write a merge-by-colour GLB matching the adapy viewer's structure. The threaded core
// now lives in step_to_glb_stream.h (adacpp::stream_step_to_glb) so the standalone OCC-free STP2GLB
// CLI can reuse it without nanobind/OCCT. This thin wrapper keeps the existing python binding.
int stream_step_to_glb_impl(const std::string &in_path, const std::string &out_path, double deflection,
                            double angular_deg, int num_threads, bool meshopt, double model_scale, bool face_regions,
                            const std::string &pipeline, bool pin_boundary) {
    return (int) adacpp::stream_step_to_glb(in_path, out_path, deflection, angular_deg, num_threads, meshopt,
                                            /*spill_dir=*/"", model_scale, face_regions, pipeline, pin_boundary);
}

// Native OCC-free IFC -> GLB (IfcResolver: geometry + colour + spatial tree, baked to metres).
// Parallel: LPT-ordered products across `num_threads` workers (0 = cgroup-aware auto). Returns the
// number of products written, or -1 on error.
int stream_ifc_to_glb_impl(const std::string &in_path, const std::string &out_path, double deflection,
                           double angular_deg, bool meshopt, double model_scale, int num_threads,
                           const std::string &pipeline, bool face_regions, bool pin_boundary) {
    return (int) adacpp::stream_ifc_to_glb(in_path, out_path, deflection, angular_deg, meshopt,
                                           /*spill_dir=*/"", model_scale, num_threads, pipeline, face_regions,
                                           pin_boundary);
}

// Threaded OCC-free STEP -> STL / OBJ (same reader + parallel tessellation as the GLB core, but bakes
// world placements and streams triangles to a binary STL or Wavefront OBJ). Returns the triangle
// count, or -1 on error. `fmt` is "stl" or "obj".
long stream_step_to_mesh_impl(const std::string &in_path, const std::string &out_path, const std::string &fmt,
                              double deflection, double angular_deg, int num_threads, double model_scale) {
    adacpp::MeshFormat mf = (fmt == "obj" || fmt == "OBJ") ? adacpp::MeshFormat::OBJ : adacpp::MeshFormat::STL;
    return adacpp::stream_step_to_mesh(in_path, out_path, mf, deflection, angular_deg, num_threads,
                                       /*spill_dir=*/"", model_scale);
}

// GLB model diff: parse two GLBs into per-element summaries, match them, and emit colour ops keyed by
// node_id (== frontend rangeId) + a removed-overlay GLB (ref-only geometry). One model parsed at a
// time (never both full models resident). Returns {ops:[(node_id,status)], removed:[…], added:[…],
// counts:{…}, overlay:bytes}. status: 0=unchanged 1=added 2=removed 3=modified.
// Path-based + mmap'd so the multi-GB GLBs never enter RSS (file stays page-cache-backed) and never
// round-trip through Python bytes. Memory model: summarise scene then ref, ONE mesh node decoded at a
// time (peak = largest material chunk), only the KB-scale summary tables survive to match. The
// removed overlay re-scans the ref and keeps ONLY the removed elements' triangles.
// Native-only: summarize_glb_file is mmap/POSIX-backed (#ifndef __EMSCRIPTEN__ in glb_diff_native.h).
// The browser gets GLB-diff from the standalone adacpp_glb_diff embind module (glb_diff_wasm.cpp).
#ifndef __EMSCRIPTEN__
nb::dict glb_diff_impl(const std::string &scene_path, const std::string &ref_path, const std::string &mode_s,
                       double tol, uint32_t overlay_rgba) {
    using namespace adacpp::gdiff;
    Mode mode = Mode::NameThenCentroid;
    if (mode_s == "byName")
        mode = Mode::ByName;
    else if (mode_s == "byGuid")
        mode = Mode::ByGuid;
    else if (mode_s == "byCentroid")
        mode = Mode::ByCentroid;
    else if (mode_s == "byProperty")
        mode = Mode::ByProperty;

    std::vector<ElementSummary> ss = summarize_glb_file(scene_path);
    std::vector<ElementSummary> rs = summarize_glb_file(ref_path);

    DiffResult res = diff_summaries(ss, rs, mode, tol);

    // overlay: re-scan ref, gather ONLY the removed elements' triangles (bounded by the removed set).
    std::string overlay;
    if (!res.removed_node_ids.empty()) {
        std::unordered_set<std::string> keep(res.removed_node_ids.begin(), res.removed_node_ids.end());
        std::vector<float> tris;
        summarize_glb_file(ref_path, &keep, &tris);
        overlay = write_overlay_glb(tris, overlay_rgba);
    }

    nb::dict d;
    nb::list ops;
    for (const DiffOp &o : res.ops)
        ops.append(nb::make_tuple(o.node_id, (int) o.status));
    d["ops"] = ops;
    nb::list rem, add;
    for (const std::string &id : res.removed_node_ids)
        rem.append(id);
    for (const std::string &id : res.added_node_ids)
        add.append(id);
    d["removed"] = rem;
    d["added"] = add;
    nb::dict counts;
    counts["added"] = res.n_added;
    counts["removed"] = res.n_removed;
    counts["modified"] = res.n_modified;
    counts["unchanged"] = res.n_unchanged;
    d["counts"] = counts;
    d["overlay"] = nb::bytes(overlay.data(), overlay.size());
    return d;
}
#endif // __EMSCRIPTEN__

// Single-threaded, mmap-free STEP -> GLB (the wasm/OPFS core, exercised natively here as a parity
// oracle). Creates a temp spill dir, runs step_to_glb_single, returns the triangle count.
long stream_step_to_glb_st_impl(const std::string &in_path, const std::string &out_path, double deflection,
                                double angular_deg, bool meshopt) {
    std::string tmpl = adacpp::temp_template("adacpp_glbst");
    char *dir = ::mkdtemp(tmpl.data());
    if (!dir)
        return -1;
    long n = adacpp::step_to_glb_single(in_path, out_path, dir, deflection, angular_deg, meshopt);
    ::rmdir(dir);
    return n;
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
std::array<double, 6> bbox_impl(const ShapeHandle &sh, bool optimal = true) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("bbox: ShapeHandle is null");
    }
    Bnd_Box bb;
    if (optimal) {
        // AddOptimal uses geometric extents (BSpline/B-rep aware) for a tight
        // bbox; default Add inflates by shape tolerance (~1e-7) which would
        // surprise callers querying a primitive's natural bbox.
        //
        // useTriangulation=False forces the analytic path — without this, OCCT
        // returns the *mesh* bbox if a triangulation is already cached on the
        // shape, which jitters ±1e-7 for box and ±0.1 for sphere/cylinder
        // depending on tessellation deflection. Callers asking for `bbox(shape)`
        // expect geometric extents, not mesh extents.
        BRepBndLib::AddOptimal(shape, bb,
                               /*useTriangulation=*/Standard_False,
                               /*useShapeTolerance=*/Standard_False);
    } else {
        // Fast path: a loose box without AddOptimal's per-surface refinement.
        // AddOptimal samples every BSpline/B-rep face to tighten the box, which
        // costs milliseconds per curved face — orders of magnitude more than a
        // plain corner-extent Add. Callers that only need a rough extent (e.g. an
        // empty-vs-non-empty probe before tessellation) pass optimal=false and
        // avoid that per-face cost. useTriangulation=False keeps it analytic.
        BRepBndLib::Add(shape, bb, /*useTriangulation=*/Standard_False);
    }
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
    return {{c.X(), c.Y(), c.Z()}, {obb.XHSize(), obb.YHSize(), obb.ZHSize()}};
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
void collect_step_shapes(const Handle(XCAFDoc_ShapeTool) & st, const Handle(XCAFDoc_ColorTool) & ct,
                         const TDF_Label &lab, const TopLoc_Location &loc, std::vector<StepShapeData> &out) {
    auto read_one = [&](const TDF_Label &shape_lab, const TopoDS_Shape &raw) {
        const TopoDS_Shape shape = loc.IsIdentity() ? raw : BRepBuilderAPI_Transform(raw, loc.Transformation()).Shape();
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
    if (!ok)
        throw std::runtime_error("read_step_shapes: transfer to OCAF document failed");

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

// STEP bytes -> GLB bytes in one pass, keeping the OCAF document intact end to end.
//
// The colour-preserving counterpart to read_step_bytes + write_glb_bytes, which lose it twice over:
// read_step_bytes uses the plain STEPControl_Reader (no XCAF layer, so STYLED_ITEM is never even
// looked at) and fuses everything via OneShape(); write_glb_bytes then builds an empty document with
// no ColorTool. The result is a GLB with zero materials, which a viewer renders in its own default
// grey. Handing RWGltf the document STEPCAFControl_Reader produced keeps the names, the assembly
// tree, and both solid-level and per-face colours, without flattening to a shape list (a flat list
// re-emits a solid AND each of its faces, duplicating the geometry).
nb::bytes step_bytes_to_glb_bytes_impl(nb::bytes data, double linear_deflection, double angular_deg,
                                       const std::string &unit) {
    if (linear_deflection <= 0.0)
        linear_deflection = 0.1;
    // Restore the caller's value on exit — "xstep.cascade.unit" is a process-global
    // Interface_Static parameter; leaving it set leaks into later reads.
    const InterfaceStaticCValGuard cascade_unit_guard("xstep.cascade.unit");

    const std::filesystem::path tmp_in = make_temp_path("adacpp_step_glb_in", ".stp");
    {
        std::ofstream f(tmp_in.string(), std::ios::binary);
        f.write(data.c_str(), static_cast<std::streamsize>(data.size()));
    }

    Handle(TDocStd_Document) doc = new TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"));
    {
        STEPCAFControl_Reader reader;
        reader.SetColorMode(Standard_True);
        reader.SetNameMode(Standard_True);
        reader.SetLayerMode(Standard_True);
        // Set AFTER constructing the reader (its ctor resets the static), before ReadFile —
        // same order as read_step_shapes / StepStore.create_step_reader.
        Interface_Static::SetCVal("xstep.cascade.unit", unit.c_str());
        const IFSelect_ReturnStatus st = reader.ReadFile(tmp_in.string().c_str());
        if (st != IFSelect_RetDone) {
            std::error_code ec;
            std::filesystem::remove(tmp_in, ec);
            throw std::runtime_error("step_bytes_to_glb_bytes: STEPCAFControl_Reader could not parse the input");
        }
        if (!reader.Transfer(doc)) {
            std::error_code ec;
            std::filesystem::remove(tmp_in, ec);
            throw std::runtime_error("step_bytes_to_glb_bytes: transfer to OCAF document failed");
        }
    }
    std::error_code ec_in;
    std::filesystem::remove(tmp_in, ec_in);

    // RWGltf needs a triangulation per face; mesh every shape the document holds.
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence labels;
    shapeTool->GetFreeShapes(labels);
    for (int i = 1; i <= labels.Length(); ++i) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels.Value(i));
        if (shape.IsNull())
            continue;
        // MSVC only defines M_PI under _USE_MATH_DEFINES; spell the conversion out.
        constexpr double deg2rad = 3.14159265358979323846 / 180.0;
        BRepMesh_IncrementalMesh(shape, linear_deflection,
                                 /*relative=*/Standard_False, angular_deg * deg2rad,
                                 /*parallel=*/Standard_True);
    }

    const std::filesystem::path tmp = make_temp_path("adacpp_glb", ".glb");
    const std::string tmpname = tmp.string();
    {
        RWGltf_CafWriter writer(TCollection_AsciiString(tmpname.c_str()), /*isBinary=*/Standard_True);
        // Same Z-up + compact-transform configuration as write_glb_bytes, so both writers
        // land in the adapy viewer's orientation.
        writer.ChangeCoordinateSystemConverter().SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
        writer.SetTransformationFormat(RWGltf_WriterTrsfFormat_Compact);
        TColStd_IndexedDataMapOfStringString fileInfo;
        fileInfo.Add(TCollection_AsciiString("Authors"), TCollection_AsciiString("adacpp"));
        const Message_ProgressRange progress;
        if (!writer.Perform(doc, fileInfo, progress)) {
            std::error_code ec;
            std::filesystem::remove(tmp, ec);
            throw std::runtime_error("step_bytes_to_glb_bytes: RWGltf_CafWriter::Perform failed");
        }
    }

    std::ifstream f(tmpname, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        std::error_code ec;
        std::filesystem::remove(tmp, ec);
        throw std::runtime_error("step_bytes_to_glb_bytes: failed to re-open temp file");
    }
    const auto size = static_cast<std::size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (size > 0)
        f.read(buffer.data(), size);
    f.close();
    std::error_code ec;
    std::filesystem::remove(tmp, ec);
    return nb::bytes(buffer.data(), buffer.size());
}

nb::bytes write_glb_bytes_impl(const ShapeHandle &sh, double linear_deflection) {
    const TopoDS_Shape &shape = sh.topods();
    if (shape.IsNull()) {
        throw std::runtime_error("write_glb_bytes: ShapeHandle is null");
    }
    if (linear_deflection <= 0.0)
        linear_deflection = 0.1;

    // RWGltf needs a triangulation per face; mesh in-place on the shape.
    // Angular 0.2 rad (~11.5deg) rather than OCC's loose 0.5 default so large-radius
    // arcs (curved beams, big pipes) stay smooth — see append_shape_triangles.
    BRepMesh_IncrementalMesh(shape, linear_deflection,
                             /*relative=*/Standard_False,
                             /*angular=*/0.2,
                             /*parallel=*/Standard_True);

    // Wrap the shape in a CAF document — RWGltf_CafWriter consumes one.
    Handle(TDocStd_Document) doc = new TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"));
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    shapeTool->AddShape(shape);

    const std::filesystem::path tmp = make_temp_path("adacpp_glb", ".glb");
    const std::string tmpname = tmp.string(); // RWGltf_CafWriter opens by name.

    {
        RWGltf_CafWriter writer(TCollection_AsciiString(tmpname.c_str()),
                                /*isBinary=*/Standard_True);
        // Match adapy's gltf_writer.to_gltf() configuration so the produced
        // GLB renders identically in the adapy viewer:
        //   - Z-up source coordinate system (CAD convention) — RWGltf
        //     internally rotates to glTF's Y-up runtime convention so the
        //     viewer doesn't see sideways/upside-down models.
        //   - Compact node transforms: smaller JSON, what the viewer expects.
        writer.ChangeCoordinateSystemConverter().SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
        writer.SetTransformationFormat(RWGltf_WriterTrsfFormat_Compact);

        TColStd_IndexedDataMapOfStringString fileInfo;
        fileInfo.Add(TCollection_AsciiString("Authors"), TCollection_AsciiString("adacpp"));
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
    if (size > 0)
        f.read(buffer.data(), size);
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
    if (op == "DIFFERENCE")
        return ShapeHandle(BRepAlgoAPI_Cut(sa, sb).Shape());
    if (op == "UNION")
        return ShapeHandle(BRepAlgoAPI_Fuse(sa, sb).Shape());
    if (op == "INTERSECTION")
        return ShapeHandle(BRepAlgoAPI_Common(sa, sb).Shape());
    throw std::runtime_error("boolean: unknown op '" + op + "'");
}

// m = the top 3 rows of a 4x4 affine matrix, row-major (12 doubles). The
// implicit bottom row is [0,0,0,1] — same convention as gp_Trsf::SetValues and
// adapy's OccBackend.transform. Lossless for rigid + uniform-scale transforms.
ShapeHandle transform_impl(const ShapeHandle &sh, const std::array<double, 12> &m, bool copy) {
    gp_Trsf trsf;
    trsf.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
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
    BRepTools::Clean(shape); // drop cached triangulation → geometry-only string
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
    case TopAbs_COMPOUND:
        return "compound";
    case TopAbs_COMPSOLID:
        return "compsolid";
    case TopAbs_SOLID:
        return "solid";
    case TopAbs_SHELL:
        return "shell";
    case TopAbs_FACE:
        return "face";
    case TopAbs_WIRE:
        return "wire";
    case TopAbs_EDGE:
        return "edge";
    case TopAbs_VERTEX:
        return "vertex";
    default:
        return "shape";
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
        if (!exp.More())
            throw std::runtime_error("face_surface_type: shape has no face");
        face = TopoDS::Face(exp.Current());
    }
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull())
        return "unknown";
    const std::string name = surf->DynamicType()->Name();
    if (name == "Geom_Plane")
        return "plane";
    if (name == "Geom_CylindricalSurface")
        return "cylinder";
    if (name == "Geom_ConicalSurface")
        return "cone";
    if (name == "Geom_SphericalSurface")
        return "sphere";
    if (name == "Geom_ToroidalSurface")
        return "torus";
    if (name == "Geom_BSplineSurface")
        return "bspline";
    if (name == "Geom_BezierSurface")
        return "bezier";
    if (name == "Geom_SurfaceOfLinearExtrusion")
        return "linear_extrusion";
    if (name == "Geom_SurfaceOfRevolution")
        return "revolution";
    return name; // fall back to the raw OCCT class name
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
    // BRepBuilderAPI_Sewing's candidate matching is quadratic in the number of
    // free edges, with per-candidate B-spline curve evaluation — a single-body
    // shell of ~5k spline faces (SESAM hull skin) takes >10 min single-threaded.
    // Sewing only stitches shared edges (connectivity); tessellation, entity
    // counting and B-rep export all work face-per-face. Above the cap, return a
    // plain compound of the faces instead of sewing. ADACPP_SEW_MAX_FACES overrides.
    size_t sew_max = 1000;
    if (const char *env = std::getenv("ADACPP_SEW_MAX_FACES"))
        sew_max = (size_t) std::strtoul(env, nullptr, 10);
    size_t n_valid = 0;
    for (const auto &f : faces)
        if (!f.topods().IsNull())
            ++n_valid;
    if (n_valid == 0)
        throw std::runtime_error("sew_faces: no faces");
    if (n_valid > sew_max) {
        TopoDS_Compound comp;
        BRep_Builder builder;
        builder.MakeCompound(comp);
        for (const auto &f : faces)
            if (!f.topods().IsNull())
                builder.Add(comp, f.topods());
        return ShapeHandle(comp);
    }
    BRepBuilderAPI_Sewing sewer(tolerance > 0.0 ? tolerance : 1e-6);
    for (const auto &f : faces) {
        const TopoDS_Shape s = f.topods();
        if (s.IsNull())
            continue;
        sewer.Add(s);
    }
    sewer.Perform();
    const TopoDS_Shape sewn = sewer.SewedShape();
    if (sewn.IsNull())
        throw std::runtime_error("sew_faces: sewing produced a null shape");
    return ShapeHandle(sewn);
}

std::vector<ShapeHandle> make_volumes_from_faces_impl(const std::vector<ShapeHandle> &faces, double tolerance) {
    BOPAlgo_MakerVolume mv;
    TopTools_ListOfShape args;
    for (const auto &f : faces)
        args.Append(f.topods());
    mv.SetArguments(args);
    mv.SetIntersect(Standard_True); // imprint the faces against each other first
    if (tolerance > 0.0)
        mv.SetFuzzyValue(tolerance);
    mv.Perform();
    if (mv.HasErrors())
        throw std::runtime_error("make_volumes_from_faces: BOPAlgo_MakerVolume reported errors");
    std::vector<ShapeHandle> out;
    for (TopExp_Explorer exp(mv.Shape(), TopAbs_SOLID); exp.More(); exp.Next())
        out.emplace_back(exp.Current());
    return out;
}

ShapeHandle non_manifold_merge_impl(const std::vector<ShapeHandle> &shapes, double tolerance, bool glue) {
    BOPAlgo_Builder builder;
    TopTools_ListOfShape args;
    for (const auto &s : shapes)
        args.Append(s.topods());
    builder.SetArguments(args);
    if (glue)
        builder.SetGlue(BOPAlgo_GlueShift); // coincident faces collapse to one shared face
    if (tolerance > 0.0)
        builder.SetFuzzyValue(tolerance);
    builder.Perform();
    if (builder.HasErrors())
        throw std::runtime_error("non_manifold_merge: BOPAlgo_Builder reported errors");
    return ShapeHandle(builder.Shape());
}

// General-Fuse a set of planar outlines against each other and against
// `imprint_curves`, returning the merged topology as plain data.
//
// Unlike non_manifold_merge this keeps the Modified() history, which is what
// maps an input outline to the faces it became — the caller needs that to name
// the faces a plate resolves to. Imprint curves (beam axes) split a face along
// their line where they lie on it, and drop a vertex where they merely cross
// it; they survive the fuse as free edges and are deliberately not reported.
//
// The whole model is imprinted and extracted in one call: crossing the
// Python/C++ boundary per face or per edge would dominate the cost.
ImprintResult imprint_planar_faces_impl(const std::vector<std::vector<std::array<double, 3>>> &outlines,
                                        const std::vector<std::vector<std::array<double, 3>>> &imprint_curves,
                                        double tolerance) {
    ImprintResult out;
    if (outlines.empty())
        return out;

    auto mkface = [](const std::vector<std::array<double, 3>> &pts) {
        if (pts.size() < 3)
            throw std::runtime_error("imprint_planar_faces: an outline needs at least 3 points");
        BRepBuilderAPI_MakePolygon mp;
        for (const auto &p : pts)
            mp.Add(gp_Pnt(p[0], p[1], p[2]));
        mp.Close();
        return TopoDS::Face(BRepBuilderAPI_MakeFace(mp.Wire(), Standard_True).Face());
    };

    const double tol = tolerance > 0.0 ? tolerance : 1e-12;
    // One TopoDS_Edge per segment rather than a wire: BOPAlgo's history is
    // per-argument, and edges give a direct Modified(edge) -> split edges map,
    // which is what resolves a beam axis to the edges it became.
    auto mkedges = [&tol](const std::vector<std::array<double, 3>> &pts) {
        std::vector<TopoDS_Edge> out;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            gp_Pnt a(pts[i][0], pts[i][1], pts[i][2]);
            gp_Pnt b(pts[i + 1][0], pts[i + 1][1], pts[i + 1][2]);
            if (a.Distance(b) <= tol)
                continue; // a zero-length segment would fail the edge build
            out.push_back(BRepBuilderAPI_MakeEdge(a, b).Edge());
        }
        return out;
    };

    std::vector<TopoDS_Face> inputs;
    inputs.reserve(outlines.size());
    for (const auto &o : outlines)
        inputs.push_back(mkface(o));

    // per curve, the argument edges it contributed
    std::vector<std::vector<TopoDS_Edge>> curve_edges;
    std::vector<TopoDS_Edge> cutters;
    for (const auto &c : imprint_curves) {
        curve_edges.push_back(mkedges(c));
        for (const auto &e : curve_edges.back())
            cutters.push_back(e);
    }

    TopoDS_Shape res;
    BOPAlgo_Builder builder;
    const bool fused = inputs.size() + cutters.size() >= 2;
    if (!fused) {
        // General Fuse needs at least two arguments (it raises TooFewArguments
        // otherwise). A lone outline has nothing to imprint against.
        res = inputs[0];
    } else {
        TopTools_ListOfShape args;
        for (const auto &f : inputs)
            args.Append(f);
        for (const auto &w : cutters)
            args.Append(w);
        builder.SetArguments(args);
        if (tolerance > 0.0)
            builder.SetFuzzyValue(tolerance);
        builder.SetRunParallel(Standard_True);
        builder.Perform();
        if (builder.HasErrors())
            throw std::runtime_error("imprint_planar_faces: BOPAlgo_Builder reported errors");
        res = builder.Shape();
    }

    // Index the unique sub-shapes. The map's hasher keys on TShape+Location and
    // ignores orientation, so a FORWARD and a REVERSED use of the same edge
    // collapse to one index — exactly the sharing we want.
    //
    // Index the whole result, faces and free edges alike: an imprint curve with
    // no face under it survives as a free edge, and the caller still needs
    // geometry for it (carried as a wire body). Which is which -> free_edges.
    TopTools_IndexedMapOfShape fmap, vmap, emap;
    TopExp::MapShapes(res, TopAbs_FACE, fmap);
    TopExp::MapShapes(res, TopAbs_VERTEX, vmap);
    TopExp::MapShapes(res, TopAbs_EDGE, emap);

    TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
    TopExp::MapShapesAndAncestors(res, TopAbs_EDGE, TopAbs_FACE, edge_faces);
    for (int i = 1; i <= emap.Extent(); ++i) {
        const TopoDS_Shape &e = emap.FindKey(i);
        if (!edge_faces.Contains(e) || edge_faces.FindFromKey(e).IsEmpty())
            out.free_edges.push_back(i - 1);
    }

    out.vertices.reserve(vmap.Extent());
    for (int i = 1; i <= vmap.Extent(); ++i) {
        gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vmap.FindKey(i)));
        out.vertices.push_back({p.X(), p.Y(), p.Z()});
    }

    out.edges.reserve(emap.Extent());
    for (int i = 1; i <= emap.Extent(); ++i) {
        TopoDS_Edge e = TopoDS::Edge(emap.FindKey(i).Oriented(TopAbs_FORWARD));
        ImprintedEdge rec;
        rec.start = vmap.FindIndex(TopExp::FirstVertex(e, Standard_True)) - 1;
        rec.end = vmap.FindIndex(TopExp::LastVertex(e, Standard_True)) - 1;
        out.edges.push_back(rec);
    }

    out.faces.reserve(fmap.Extent());
    for (int i = 1; i <= fmap.Extent(); ++i) {
        TopoDS_Face f = TopoDS::Face(fmap.FindKey(i));
        BRepAdaptor_Surface surf(f, Standard_True);
        if (surf.GetType() != GeomAbs_Plane)
            throw std::runtime_error("imprint_planar_faces: result contains a non-planar face");
        gp_Pln pln = surf.Plane();
        gp_Pnt loc = pln.Location();
        gp_Dir ax = pln.Axis().Direction();
        gp_Dir xd = pln.XAxis().Direction();

        ImprintedFace rec;
        rec.origin = {loc.X(), loc.Y(), loc.Z()};
        rec.normal = {ax.X(), ax.Y(), ax.Z()};
        rec.ref_direction = {xd.X(), xd.Y(), xd.Z()};
        // A REVERSED face's true outward normal is the opposite of its
        // surface's; flipping here keeps every loop below wound
        // counter-clockwise about the normal we report.
        if (f.Orientation() == TopAbs_REVERSED)
            rec.normal = {-rec.normal[0], -rec.normal[1], -rec.normal[2]};

        TopoDS_Wire outer = BRepTools::OuterWire(f);
        std::vector<TopoDS_Wire> wires;
        for (TopExp_Explorer wexp(f, TopAbs_WIRE); wexp.More(); wexp.Next())
            wires.push_back(TopoDS::Wire(wexp.Current()));
        std::stable_sort(wires.begin(), wires.end(), [&outer](const TopoDS_Wire &a, const TopoDS_Wire &b) {
            return a.IsSame(outer) && !b.IsSame(outer); // outer loop first
        });

        for (const auto &w : wires) {
            std::vector<std::pair<int, bool>> loop;
            for (BRepTools_WireExplorer we(w, f); we.More(); we.Next()) {
                const TopoDS_Edge &edge = we.Current();
                loop.emplace_back(emap.FindIndex(edge) - 1, edge.Orientation() == TopAbs_FORWARD);
            }
            rec.loops.push_back(std::move(loop));
        }
        out.faces.push_back(std::move(rec));
    }

    // The result sub-shapes an argument became, as indices into `index_map`.
    // Only ones present in the map count: an imprint curve with no face under it
    // survives as a free edge, which bounds nothing and is deliberately absent.
    auto history = [&](const TopoDS_Shape &shape, const TopTools_IndexedMapOfShape &index_map) {
        std::vector<int> got;
        if (!fused) {
            if (index_map.Contains(shape))
                got.push_back(index_map.FindIndex(shape) - 1);
            return got;
        }
        const TopTools_ListOfShape &mods = builder.Modified(shape);
        if (!mods.IsEmpty()) {
            for (TopTools_ListOfShape::Iterator it(mods); it.More(); it.Next())
                if (index_map.Contains(it.Value()))
                    got.push_back(index_map.FindIndex(it.Value()) - 1);
        } else if (!builder.IsDeleted(shape) && index_map.Contains(shape)) {
            got.push_back(index_map.FindIndex(shape) - 1); // untouched: passes through as itself
        }
        return got;
    };

    out.sources.reserve(inputs.size());
    for (const auto &f : inputs)
        out.sources.push_back(history(f, fmap));

    out.curve_sources.reserve(curve_edges.size());
    for (const auto &edges_ : curve_edges) {
        std::vector<int> got;
        for (const auto &e : edges_)
            for (int idx : history(e, emap))
                if (std::find(got.begin(), got.end(), idx) == got.end())
                    got.push_back(idx);
        out.curve_sources.push_back(std::move(got));
    }
    return out;
}

// Faithful port of topologic's Topology::Merge over solids: general-fuse the
// solids with BOPAlgo_CellsBuilder, take each operand's region into the result
// (AddToResult) and MakeContainers() to assemble the non-manifold CellComplex —
// each input solid survives as a cell and every interface becomes one shared
// face. Mirrors adapy's OccBackend.merge_cells (unlike make_volumes_from_faces,
// which rebuilds minimal volumes from a face soup and loses operand identity).
std::vector<ShapeHandle> merge_cells_impl(const std::vector<ShapeHandle> &solids, double tolerance) {
    if (solids.empty())
        return {};
    BOPAlgo_CellsBuilder cb;
    TopTools_ListOfShape args;
    for (const auto &s : solids)
        args.Append(s.topods());
    cb.SetArguments(args);
    if (tolerance > 0.0)
        cb.SetFuzzyValue(tolerance);
    cb.Perform();
    if (cb.HasErrors())
        throw std::runtime_error("merge_cells: BOPAlgo_CellsBuilder reported errors");
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
    for (const auto &s : solids)
        bld.Add(comp, s.topods());
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
    case TopAbs_IN:
        return 0;
    case TopAbs_OUT:
        return 1;
    case TopAbs_ON:
        return 2;
    default:
        return 3;
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

ShapeHandle build_box_impl(std::array<double, 3> loc, std::array<double, 3> axis, std::array<double, 3> ref_dir,
                           double dx, double dy, double dz) {
    const gp_Ax2 ax2(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]),
                     gp_Dir(ref_dir[0], ref_dir[1], ref_dir[2]));
    return ShapeHandle(BRepPrimAPI_MakeBox(ax2, dx, dy, dz).Shape());
}

ShapeHandle build_cylinder_impl(std::array<double, 3> loc, std::array<double, 3> axis, double radius, double height) {
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
                                       gp_Pnt(pts[i + 1][0], pts[i + 1][1], pts[i + 1][2]))
                   .Edge());
    }
    wm.Build();
    return ShapeHandle(wm.Wire());
}

ShapeHandle build_cone_impl(std::array<double, 3> loc, std::array<double, 3> axis, double bottom_radius,
                            double height) {
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
// A circle/arc radius is a positive magnitude; gp_Circ throws Standard_ConstructionError on
// radius <= 0, which aborts the entire solid build (one bad boundary edge drops the whole
// solid as UnableToCreateSolidOCCGeom). A negative radius is a sign/serialization artifact —
// radius has no sign — so take its magnitude. A near-zero radius is genuinely degenerate (a
// point, not an edge): raise a clean, identifiable error rather than OCCT's opaque one.
static double circle_radius_or_throw(double r) {
    const double a = std::abs(r);
    if (a < 1e-9)
        throw std::runtime_error("edge_from_record: degenerate circle radius (|radius| < 1e-9)");
    return a;
}

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
        // Full circle. Two layouts (length-detected, back-compatible):
        //   new   [loc(3), axis(3), ref(3), radius, start(3)] (len 14) — ref_direction sets the
        //         angular origin and the start point anchors the closed edge's vertex so a seam
        //         edge connects to it (else the vertex lands on OCC's default x-axis → the
        //         cylinder/torus boundary wire fails to close). Emitted for face-bound edges.
        //   legacy[loc(3), axis(3), radius] (len 8) — full circle, default axes. Still emitted for
        //         closed PROFILE curves (extrude/revolve), where anchoring is irrelevant.
        if (e.size() >= 14) {
            const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]), gp_Dir(e[7], e[8], e[9]));
            const gp_Pnt p_start(e[11], e[12], e[13]);
            return BRepBuilderAPI_MakeEdge(gp_Circ(ax, circle_radius_or_throw(e[10])), p_start, p_start).Edge();
        }
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
        return BRepBuilderAPI_MakeEdge(gp_Circ(ax, circle_radius_or_throw(e[7]))).Edge();
    }
    if (kind == 5) {
        // Trimmed arc. new [loc(3), axis(3), ref(3), radius, t0, t1] (len 13) — ref places the arc
        // endpoints at the right angle so they meet adjacent edges. legacy [loc(3), axis(3),
        // radius, t0, t1] (len 10) — default axes.
        if (e.size() >= 13) {
            const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]), gp_Dir(e[7], e[8], e[9]));
            return BRepBuilderAPI_MakeEdge(gp_Circ(ax, circle_radius_or_throw(e[10])), e[11], e[12]).Edge();
        }
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
        return BRepBuilderAPI_MakeEdge(gp_Circ(ax, circle_radius_or_throw(e[7])), e[8], e[9]).Edge();
    }
    if (kind == 4) {
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]), gp_Dir(e[7], e[8], e[9]));
        const gp_Elips el(ax, e[10], e[11]);
        const bool trim = std::lround(e[12]) != 0;
        if (!trim)
            return BRepBuilderAPI_MakeEdge(el).Edge();
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
        for (int k = 1; k <= n_knots; ++k)
            knots.SetValue(k, e[i++]);
        TColStd_Array1OfInteger mults(1, n_knots);
        for (int k = 1; k <= n_knots; ++k)
            mults.SetValue(k, static_cast<int>(std::lround(e[i++])));
        Handle(Geom_BSplineCurve) curve;
        if (rational) {
            TColStd_Array1OfReal weights(1, n_poles);
            for (int p = 1; p <= n_poles; ++p)
                weights.SetValue(p, e[i++]);
            curve = new Geom_BSplineCurve(poles, weights, knots, mults, degree, Standard_False);
        } else {
            curve = new Geom_BSplineCurve(poles, knots, mults, degree, Standard_False);
        }
        if (trim)
            return BRepBuilderAPI_MakeEdge(curve, t_start, t_end).Edge();
        // No parametric trim: the record's start/end points define the segment of an
        // otherwise-full b-spline curve. Trim by points (OCC projects them onto the curve) —
        // without this the whole curve is used and the edge overshoots the real boundary.
        if (p_start.Distance(p_end) > 1e-9)
            return BRepBuilderAPI_MakeEdge(curve, p_start, p_end).Edge();
        return BRepBuilderAPI_MakeEdge(curve).Edge();
    }
    throw std::runtime_error("edge_from_record: unknown edge kind " + std::to_string(kind));
}

// Build a Geom_Curve (not an edge) from a curve record — used as the generatrix /
// swept curve of a surface of revolution / linear extrusion. Returns the FULL curve
// (the face bounds, not the curve, trim the surface). Handles the curve kinds a
// generatrix realistically takes: B-spline (kind 3, the STEP case), line (kind 0,
// returned as a finite trimmed segment) and full circle (kind 2).
Handle(Geom_Curve) geom_curve_from_record(const std::vector<double> &e) {
    const int kind = static_cast<int>(std::lround(e[0]));
    if (kind == 0) {
        const gp_Pnt a(e[1], e[2], e[3]), b(e[4], e[5], e[6]);
        Handle(Geom_Line) ln = new Geom_Line(a, gp_Dir(gp_Vec(a, b)));
        return new Geom_TrimmedCurve(ln, 0.0, a.Distance(b));
    }
    if (kind == 2) {
        const gp_Ax2 ax(gp_Pnt(e[1], e[2], e[3]), gp_Dir(e[4], e[5], e[6]));
        return new Geom_Circle(ax, circle_radius_or_throw(e[7]));
    }
    if (kind == 3) {
        const int degree = static_cast<int>(std::lround(e[1]));
        const bool rational = std::lround(e[2]) != 0;
        std::size_t i = 12; // skip kind,degree,rational,trim,t0,t1,pstart(3),pend(3)
        const int n_poles = static_cast<int>(std::lround(e[i++]));
        TColgp_Array1OfPnt poles(1, n_poles);
        for (int p = 1; p <= n_poles; ++p) {
            poles.SetValue(p, gp_Pnt(e[i], e[i + 1], e[i + 2]));
            i += 3;
        }
        const int n_knots = static_cast<int>(std::lround(e[i++]));
        TColStd_Array1OfReal knots(1, n_knots);
        for (int k = 1; k <= n_knots; ++k)
            knots.SetValue(k, e[i++]);
        TColStd_Array1OfInteger mults(1, n_knots);
        for (int k = 1; k <= n_knots; ++k)
            mults.SetValue(k, static_cast<int>(std::lround(e[i++])));
        if (rational) {
            TColStd_Array1OfReal weights(1, n_poles);
            for (int p = 1; p <= n_poles; ++p)
                weights.SetValue(p, e[i++]);
            return new Geom_BSplineCurve(poles, weights, knots, mults, degree, Standard_False);
        }
        return new Geom_BSplineCurve(poles, knots, mults, degree, Standard_False);
    }
    throw std::runtime_error("geom_curve_from_record: unsupported generatrix kind " + std::to_string(kind));
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
    if (edges.size() < 3)
        throw std::runtime_error("build_filled_face: need >= 3 boundary edges");
    BRepOffsetAPI_MakeFilling filler;
    for (const auto &e : edges)
        filler.Add(edge_from_record(e), GeomAbs_C0);
    filler.Build();
    if (!filler.IsDone())
        throw std::runtime_error("build_filled_face: MakeFilling failed");
    return ShapeHandle(filler.Shape());
}

// Place a shape built in the XY frame at an Axis2Placement3D — the gp_Ax3
// change-of-basis + translation (= adapy's transform_shape_to_pos).
TopoDS_Shape place_at(TopoDS_Shape shape, const std::array<double, 3> &loc, const std::array<double, 3> &axis,
                      const std::array<double, 3> &ref_dir) {
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
    if (poly.size() < 3)
        throw std::runtime_error("polygon_face: need at least 3 points");
    BRepBuilderAPI_MakePolygon mp;
    for (const auto &p : poly)
        mp.Add(gp_Pnt(p[0], p[1], p[2]));
    mp.Close();
    return ShapeHandle(BRepBuilderAPI_MakeFace(mp.Wire(), Standard_True).Face());
}

ShapeHandle build_face_based_surface_model_impl(const std::vector<std::vector<std::array<double, 3>>> &polygons) {
    TopoDS_Shape result;
    bool first = true;
    for (const auto &poly : polygons) {
        if (poly.size() < 3)
            continue;
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
ShapeHandle build_planar_face_impl(const std::vector<std::vector<double>> &outer,
                                   const std::vector<std::vector<std::vector<double>>> &inners,
                                   std::array<double, 3> loc, std::array<double, 3> axis,
                                   std::array<double, 3> ref_dir) {
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
Handle(Geom_BSplineSurface) make_bspline_surface(int u_degree, int v_degree,
                                                 const std::vector<std::vector<std::array<double, 3>>> &control_points,
                                                 const std::vector<double> &u_knots, const std::vector<double> &v_knots,
                                                 const std::vector<int> &u_mults, const std::vector<int> &v_mults,
                                                 const std::vector<std::vector<double>> &weights) {
    const int num_u = static_cast<int>(control_points.size());
    if (num_u == 0)
        throw std::runtime_error("make_bspline_surface: empty control point grid");
    const int num_v = static_cast<int>(control_points[0].size());

    TColgp_Array2OfPnt poles(1, num_u, 1, num_v);
    for (int u = 0; u < num_u; ++u) {
        for (int v = 0; v < num_v; ++v) {
            const auto &p = control_points[u][v];
            poles.SetValue(u + 1, v + 1, gp_Pnt(p[0], p[1], p[2]));
        }
    }
    TColStd_Array1OfReal knots_u(1, static_cast<int>(u_knots.size()));
    for (std::size_t i = 0; i < u_knots.size(); ++i)
        knots_u.SetValue(static_cast<int>(i) + 1, u_knots[i]);
    TColStd_Array1OfReal knots_v(1, static_cast<int>(v_knots.size()));
    for (std::size_t i = 0; i < v_knots.size(); ++i)
        knots_v.SetValue(static_cast<int>(i) + 1, v_knots[i]);
    TColStd_Array1OfInteger mults_u(1, static_cast<int>(u_mults.size()));
    for (std::size_t i = 0; i < u_mults.size(); ++i)
        mults_u.SetValue(static_cast<int>(i) + 1, u_mults[i]);
    TColStd_Array1OfInteger mults_v(1, static_cast<int>(v_mults.size()));
    for (std::size_t i = 0; i < v_mults.size(); ++i)
        mults_v.SetValue(static_cast<int>(i) + 1, v_mults[i]);

    if (!weights.empty()) {
        TColStd_Array2OfReal w(1, num_u, 1, num_v);
        for (int u = 0; u < num_u; ++u)
            for (int v = 0; v < num_v; ++v)
                w.SetValue(u + 1, v + 1, weights[u][v]);
        return new Geom_BSplineSurface(poles, w, knots_u, knots_v, mults_u, mults_v, u_degree, v_degree, Standard_False,
                                       Standard_False);
    }
    return new Geom_BSplineSurface(poles, knots_u, knots_v, mults_u, mults_v, u_degree, v_degree, Standard_False,
                                   Standard_False);
}

// Build a face edge from a 2D pcurve record (kind 6) laid on `surf`:
//   [6, degree, rational, closed, n_poles, <2*n_poles uv>, n_knots, <knots>,
//    <mults>, <n_poles weights if rational>,
//    <optional tail: 1, t0, t1  |  2, t0, t1, sx, sy, sz, ex, ey, ez>]
// The 3D parametrization is derived by OCCT from surface(pcurve(t)), so 2D/3D
// stay consistent — the SAT-pcurve path adapy's make_face_from_geom prefers.
// The optional tail is the owning edge's trim. SAT pcurves typically span the
// FULL underlying curve, so without a trim the edge overshoots its segment and
// the boundary wire fails to connect. Preferred trim (flag 2): the edge's
// declared 3D vertices, projected point -> surface UV -> pcurve param — exact
// regardless of parameterization. Fallback (flag 1 or failed projection): the
// edge's curve-params, tried directly and negated (per the ACIS SAT spec a
// reversed-sense edge stores its params as (-b, -a) of the true range [a, b]);
// note an ACIS bs2 pcurve is a fit approximation with its own parameterization,
// so curve-params can land slightly off (cm-scale) — hence the projection path.
TopoDS_Edge edge_from_pcurve(const std::vector<double> &e, const Handle(Geom_Surface) & surf) {
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
    for (int k = 1; k <= nk; ++k)
        knots.SetValue(k, e[i++]);
    TColStd_Array1OfInteger mults(1, nk);
    for (int k = 1; k <= nk; ++k)
        mults.SetValue(k, static_cast<int>(std::lround(e[i++])));
    Handle(Geom2d_BSplineCurve) c2d;
    if (rational) {
        TColStd_Array1OfReal w(1, n);
        for (int p = 1; p <= n; ++p)
            w.SetValue(p, e[i++]);
        c2d = new Geom2d_BSplineCurve(poles, w, knots, mults, degree, closed);
    } else {
        c2d = new Geom2d_BSplineCurve(poles, knots, mults, degree, closed);
    }
    const double cf = c2d->FirstParameter(), cl = c2d->LastParameter();
    double lo = cf, hi = cl;
    bool trimmed = false;
    const std::size_t tail = e.size() - i;
    const int flag = tail >= 3 ? static_cast<int>(std::lround(e[i])) : 0;

    if (flag == 2 && tail >= 9) {
        // Geometric trim: declared 3D vertex -> surface UV -> pcurve param.
        const gp_Pnt ps(e[i + 3], e[i + 4], e[i + 5]), pe(e[i + 6], e[i + 7], e[i + 8]);
        try {
            GeomAPI_ProjectPointOnSurf prj_s(ps, surf), prj_e(pe, surf);
            if (prj_s.NbPoints() > 0 && prj_e.NbPoints() > 0) {
                Standard_Real us, vs, ue, ve;
                prj_s.LowerDistanceParameters(us, vs);
                prj_e.LowerDistanceParameters(ue, ve);
                Geom2dAPI_ProjectPointOnCurve p2s(gp_Pnt2d(us, vs), c2d), p2e(gp_Pnt2d(ue, ve), c2d);
                if (p2s.NbPoints() > 0 && p2e.NbPoints() > 0) {
                    const double ta = p2s.LowerDistanceParameter(), tb = p2e.LowerDistanceParameter();
                    lo = std::max(std::min(ta, tb), cf);
                    hi = std::min(std::max(ta, tb), cl);
                    trimmed = hi - lo > 1e-12;
                }
            }
        } catch (const Standard_Failure &) {
            // projection failed — fall through to the param-range trim below
        }
    }
    if (!trimmed && flag >= 1) {
        const double t0 = e[i + 1], t1 = e[i + 2];
        const double a = std::min(t0, t1), b = std::max(t0, t1);
        const double tolp = 1e-9 + 1e-6 * (cl - cf);
        if (a >= cf - tolp && b <= cl + tolp) {
            lo = std::max(a, cf);
            hi = std::min(b, cl);
        } else if (-b >= cf - tolp && -a <= cl + tolp) { // ACIS reversed-sense edge: params negated
            lo = std::max(-b, cf);
            hi = std::min(-a, cl);
        } else {
            lo = cf; // trim doesn't map into this pcurve's range — keep the full range
            hi = cl;
        }
    }
    return BRepBuilderAPI_MakeEdge(c2d, surf, lo, hi).Edge();
}

ShapeHandle build_bspline_surface_face_impl(int u_degree, int v_degree,
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
ShapeHandle build_advanced_face_bspline_impl(int u_degree, int v_degree,
                                             const std::vector<std::vector<std::array<double, 3>>> &control_points,
                                             const std::vector<double> &u_knots, const std::vector<double> &v_knots,
                                             const std::vector<int> &u_mults, const std::vector<int> &v_mults,
                                             const std::vector<std::vector<double>> &weights,
                                             const std::vector<std::vector<std::vector<double>>> &bounds) {
    if (bounds.empty())
        throw std::runtime_error("build_advanced_face: no bounds");
    Handle(Geom_BSplineSurface) surf =
        make_bspline_surface(u_degree, v_degree, control_points, u_knots, v_knots, u_mults, v_mults, weights);

    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges) {
            const int kind = static_cast<int>(std::lround(rec[0]));
            wm.Add(kind == 6 ? edge_from_pcurve(rec, surf) : edge_from_record(rec));
        }
        wm.Build();
        if (!wm.IsDone())
            throw std::runtime_error("build_advanced_face: wire build failed");
        return wm.Wire();
    };

    BRepBuilderAPI_MakeFace fm(surf, wire_of(bounds[0]));
    for (std::size_t b = 1; b < bounds.size(); ++b)
        fm.Add(wire_of(bounds[b]));
    if (!fm.IsDone())
        throw std::runtime_error("build_advanced_face: MakeFace failed");
    TopoDS_Face face = fm.Face();
    BRepLib::BuildCurves3d(face); // pcurve-built edges have no 3D curve yet
    return ShapeHandle(face);
}

// Bounds-trimmed AdvancedFace over a PLANE surface (flat SAT/IFC plates). The face is built on
// the DECLARED plane (loc + axis normal): an only-near-planar boundary wire (import tolerance
// above Precision::Confusion) still trims it, whereas MakeFace(wire) alone runs FindPlane and
// fails ("MakeFace failed") on such wires. Falls back to inferring the plane from the wire when
// the declared normal is degenerate. bounds[0] is the outer boundary, the rest are holes.
ShapeHandle build_advanced_face_planar_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                                            std::array<double, 3> /*ref_dir*/,
                                            const std::vector<std::vector<std::vector<double>>> &bounds) {
    if (bounds.empty())
        throw std::runtime_error("build_advanced_face_planar: no bounds");

    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges)
            wm.Add(edge_from_record(rec));
        wm.Build();
        if (!wm.IsDone())
            throw std::runtime_error("build_advanced_face_planar: wire build failed");
        return wm.Wire();
    };

    const bool have_axis = (std::abs(axis[0]) + std::abs(axis[1]) + std::abs(axis[2])) > 1e-9;

    TopoDS_Face face;
    bool done = false;

    // Build on the EXPLICIT declared plane (loc + axis normal), projecting each wire onto it. This is
    // exception-guarded, unlike the FindPlane path below whose hole-Add can hard-CRASH (SIGSEGV, not a
    // catchable Standard_Failure) when a holed face's plane is diagonally oriented. Factored into a
    // lambda so it can run either FIRST (for holed faces, to sidestep that crash) or as the
    // near-planar fallback for a single-boundary face.
    auto try_declared_plane = [&]() {
        if (done || !have_axis)
            return;
        try {
            const gp_Pln pln(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]));
            BRepBuilderAPI_MakeFace fm(pln, wire_of(bounds[0]), Standard_True);
            for (std::size_t b = 1; b < bounds.size(); ++b)
                fm.Add(wire_of(bounds[b]));
            if (fm.IsDone()) {
                face = fm.Face();
                done = true;
            }
        } catch (const Standard_Failure &) {
            // degenerate declared normal / projection failure → reported below
        }
    };

    // A holed face (bounds.size() > 1) with a valid declared plane builds on that explicit plane
    // FIRST. The FindPlane primary path below adds each hole wire with an unguarded MakeFace::Add,
    // which SIGSEGVs on faces whose plane is diagonally oriented — an uncatchable crash, so the
    // guarded fallback would never be reached if we hit it. For a single-boundary face we keep
    // FindPlane primary: it also builds each edge's 2D p-curve, so a curved-boundary flat plate meshes.
    if (have_axis && bounds.size() > 1)
        try_declared_plane();

    // Primary (single boundary, or no declared axis): infer the plane from the wire — this also builds
    // each edge's 2D p-curve (incl. a B-spline boundary edge), so a curved-boundary flat plate meshes.
    if (!done) {
        BRepBuilderAPI_MakeFace fm(wire_of(bounds[0]), Standard_True);
        for (std::size_t b = 1; b < bounds.size(); ++b)
            fm.Add(wire_of(bounds[b]));
        if (fm.IsDone()) {
            face = fm.Face();
            done = true;
        }
    }
    // Fallback: a only-near-planar wire (import tolerance above Precision::Confusion) defeats FindPlane
    // above — build on the DECLARED plane instead (no-op if the explicit build already ran).
    try_declared_plane();

    if (!done)
        throw std::runtime_error("build_advanced_face_planar: MakeFace failed");
    ShapeFix_Face fixer(face);
    fixer.Perform();
    return ShapeHandle(fixer.Face());
}

// Bounds-trimmed AdvancedFace over an explicitly-positioned analytic surface
// (cylinder / cone / torus). Unlike the planar path the surface is NOT inferred
// from the wire; the boundary wire trims the given surface. The 3D boundary edges
// (LINEs + CIRCLE arcs) carry no UV p-curve, so ShapeFix_Face projects them onto
// the surface and SameParameter reconciles 3D/2D so BRepMesh can grid the face.
// Mirrors adapy's make_closed_shell_from_geom analytic AdvancedFace path.
static ShapeHandle bounds_trimmed_analytic_face(const Handle(Geom_Surface) & surf,
                                                const std::vector<std::vector<std::vector<double>>> &bounds,
                                                const char *who) {
    if (bounds.empty())
        throw std::runtime_error(std::string(who) + ": no bounds");

    // A kind-6 record is a 2D pcurve laid on `surf` (3D derived as surface(pcurve(t))) — this is
    // how a boundary whose edges do NOT lie on the surface (a cylinder trimmed by a diagonal
    // joint cut, whose edges are chords/helices) is put ON the surface so BRepMesh tessellates it
    // curved; without pcurves such a face meshed flat/degenerate. Mirrors build_advanced_face_bspline.
    bool has_pcurve = false;
    auto wire_of = [&](const std::vector<std::vector<double>> &edges) -> TopoDS_Wire {
        BRepBuilderAPI_MakeWire wm;
        for (const auto &rec : edges) {
            if (!rec.empty() && std::lround(rec[0]) == 6) {
                wm.Add(edge_from_pcurve(rec, surf));
                has_pcurve = true;
            } else {
                wm.Add(edge_from_record(rec));
            }
        }
        wm.Build();
        if (!wm.IsDone())
            throw std::runtime_error(std::string(who) + ": wire build failed");
        return wm.Wire();
    };

    BRepBuilderAPI_MakeFace fm(surf, wire_of(bounds[0]), Standard_True);
    for (std::size_t b = 1; b < bounds.size(); ++b)
        fm.Add(wire_of(bounds[b]));
    if (!fm.IsDone())
        throw std::runtime_error(std::string(who) + ": MakeFace failed");
    TopoDS_Face face = fm.Face();

    // pcurve-built edges carry no 3D curve yet — materialise them from surface(pcurve(t)) so
    // sewing/tessellation have real 3D geometry (mirrors the B-spline face path).
    if (has_pcurve)
        BRepLib::BuildCurves3d(face);

    ShapeFix_Face fixer(face);
    fixer.Perform();
    face = fixer.Face();
    // SameParameter is best-effort: an unbounded surface (e.g. Geom_SurfaceOfLinearExtrusion,
    // infinite in V) can make it throw StdFail_NotDone while reconciling pcurves. A face that
    // fails it is still a valid B-rep — BRepMesh rebuilds pcurves on demand — so don't let one
    // step sink the whole face.
    try {
        BRepLib::SameParameter(face, 1.0e-6, Standard_True);
    } catch (const Standard_Failure &) {
        // keep the ShapeFix'd face as-is
    }
    return ShapeHandle(face);
}

static gp_Ax3 _ax3(const std::array<double, 3> &loc, const std::array<double, 3> &axis,
                   const std::array<double, 3> &ref_dir) {
    return gp_Ax3(gp_Pnt(loc[0], loc[1], loc[2]), gp_Dir(axis[0], axis[1], axis[2]),
                  gp_Dir(ref_dir[0], ref_dir[1], ref_dir[2]));
}

ShapeHandle build_advanced_face_cylindrical_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                                                 std::array<double, 3> ref_dir, double radius,
                                                 const std::vector<std::vector<std::vector<double>>> &bounds) {
    Handle(Geom_CylindricalSurface) surf = new Geom_CylindricalSurface(_ax3(loc, axis, ref_dir), radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_cylindrical");
}

ShapeHandle build_advanced_face_conical_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                                             std::array<double, 3> ref_dir, double radius, double semi_angle,
                                             const std::vector<std::vector<std::vector<double>>> &bounds) {
    // Geom_ConicalSurface(ax3, semi_angle, ref_radius)
    Handle(Geom_ConicalSurface) surf = new Geom_ConicalSurface(_ax3(loc, axis, ref_dir), semi_angle, radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_conical");
}

ShapeHandle build_advanced_face_toroidal_impl(std::array<double, 3> loc, std::array<double, 3> axis,
                                              std::array<double, 3> ref_dir, double major_radius, double minor_radius,
                                              const std::vector<std::vector<std::vector<double>>> &bounds) {
    Handle(Geom_ToroidalSurface) surf = new Geom_ToroidalSurface(_ax3(loc, axis, ref_dir), major_radius, minor_radius);
    return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_toroidal");
}

// Bounds-trimmed AdvancedFace over a surface of revolution: revolve the generatrix
// curve (the meridian — a B-spline / line / circle, passed as a curve record) about
// the axis, then trim the resulting surface to the boundary wire(s). Mirrors the
// analytic builders; libtess2 covers this OCC-free (SURF_REVOLUTION), this is the
// OCC AdacppBackend.build path for full-B-rep export (ifc/step).
ShapeHandle
build_advanced_face_surface_of_revolution_impl(std::array<double, 3> axis_loc, std::array<double, 3> axis_dir,
                                               const std::vector<double> &generatrix,
                                               const std::vector<std::vector<std::vector<double>>> &bounds) {
    try {
        Handle(Geom_Curve) gen = geom_curve_from_record(generatrix);
        Handle(Geom_SurfaceOfRevolution) surf = new Geom_SurfaceOfRevolution(
            gen, gp_Ax1(gp_Pnt(axis_loc[0], axis_loc[1], axis_loc[2]), gp_Dir(axis_dir[0], axis_dir[1], axis_dir[2])));
        return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_surface_of_revolution");
    } catch (const Standard_Failure &ex) {
        throw std::runtime_error(std::string("build_advanced_face_surface_of_revolution: ") + ex.GetMessageString());
    }
}

// Bounds-trimmed AdvancedFace over a surface of linear extrusion: extrude the swept
// curve along `direction`, then trim to the boundary wire(s). (libtess2 covers this
// OCC-free via SURF_LIN_EXTRUSION; OCC path for B-rep export.)
ShapeHandle
build_advanced_face_surface_of_linear_extrusion_impl(std::array<double, 3> direction, const std::vector<double> &swept,
                                                     const std::vector<std::vector<std::vector<double>>> &bounds) {
    // Wrap in a catch so an OCC SameParameter/MakeFace failure on the extrusion face
    // surfaces as a clean error (callers fall back / libtess2 covers it OCC-free)
    // instead of propagating a raw OCCT abort.
    // NOTE: OCC face construction over a B-spline-generatrix linear-extrusion surface currently
    // fails (BRepBuilderAPI_MakeFace can't project the 3D boundary wire onto the infinite-V
    // surface; SameParameter / explicit p-curve projection were also tried). Caught + reported
    // cleanly; libtess2 (SURF_LIN_EXTRUSION, OCC-free) is the working path for this surface today.
    try {
        Handle(Geom_Curve) gen = geom_curve_from_record(swept);
        Handle(Geom_SurfaceOfLinearExtrusion) surf =
            new Geom_SurfaceOfLinearExtrusion(gen, gp_Dir(direction[0], direction[1], direction[2]));
        return bounds_trimmed_analytic_face(surf, bounds, "build_advanced_face_surface_of_linear_extrusion");
    } catch (const Standard_Failure &ex) {
        throw std::runtime_error(std::string("build_advanced_face_surface_of_linear_extrusion: ") +
                                 ex.GetMessageString());
    }
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
    if (c2d.IsNull())
        return pc;
    Handle(Geom2d_Curve) trimmed = (l > f) ? Handle(Geom2d_Curve)(new Geom2d_TrimmedCurve(c2d, f, l)) : c2d;
    Handle(Geom2d_BSplineCurve) bsp;
    try {
        bsp = Geom2dConvert::CurveToBSplineCurve(trimmed);
    } catch (const Standard_Failure &) {
        return pc;
    }
    if (bsp.IsNull())
        return pc;

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
        for (int i = 1; i <= bsp->NbPoles(); ++i)
            pc.weights.push_back(bsp->Weight(i));
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
        if (!exp.More())
            throw std::runtime_error("face_to_advanced_face: shape has no face");
        face = TopoDS::Face(exp.Current());
    }
    Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face));
    if (bs.IsNull())
        throw std::runtime_error("face_to_advanced_face: face surface is not a B-spline");

    AdvancedFaceData out;
    out.u_degree = bs->UDegree();
    out.v_degree = bs->VDegree();
    const int nu = bs->NbUPoles(), nv = bs->NbVPoles();
    out.poles.assign(nu, std::vector<std::array<double, 3>>(nv));
    const bool rational = bs->IsURational() || bs->IsVRational();
    if (rational)
        out.weights.assign(nu, std::vector<double>(nv, 1.0));
    for (int u = 1; u <= nu; ++u) {
        for (int v = 1; v <= nv; ++v) {
            const gp_Pnt p = bs->Pole(u, v);
            out.poles[u - 1][v - 1] = {p.X(), p.Y(), p.Z()};
            if (rational)
                out.weights[u - 1][v - 1] = bs->Weight(u, v);
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
        if (!bound.empty())
            out.bounds.push_back(std::move(bound));
    }
    return out;
}

// Prism-extrude a face by `thickness` along its surface normal at the
// parametric centre. Port of adapy's extrude_face_along_normal — gives a curved
// plate (PlateCurved) its thickness. Falls back to the bare face on thickness 0,
// undefined normal, or prism failure (matches adapy's render-something policy).
ShapeHandle extrude_face_along_normal_impl(const ShapeHandle &sh, double thickness) {
    const TopoDS_Shape &shape = sh.topods();
    if (thickness == 0.0)
        return ShapeHandle(shape);
    TopExp_Explorer exp(shape, TopAbs_FACE);
    if (!exp.More())
        return ShapeHandle(shape);
    const TopoDS_Face sub_face = TopoDS::Face(exp.Current());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(sub_face);
    Standard_Real umin = 0.0, umax = 1.0, vmin = 0.0, vmax = 1.0;
    BRepTools::UVBounds(sub_face, umin, umax, vmin, vmax);
    GeomLProp_SLProps props(surf, (umin + umax) / 2.0, (vmin + vmax) / 2.0, 1, 1e-7);
    if (!props.IsNormalDefined())
        return ShapeHandle(shape);
    const gp_Dir n = props.Normal();
    const gp_Vec vec(n.X() * thickness, n.Y() * thickness, n.Z() * thickness);
    BRepPrimAPI_MakePrism prism(shape, vec);
    if (!prism.IsDone())
        return ShapeHandle(shape);
    return ShapeHandle(prism.Shape());
}

// Build a swept profile from edge records. AREA → outer face minus inner void
// faces (solid cross-section). CURVE → the outer wire alone: matches OCC's
// make_profile_from_geom non-area path, where cutting a 1-D wire by a disjoint
// inner wire leaves the outer wire unchanged (so sweeping it yields the open
// lateral surface, e.g. a pipe-shell cylinder, not a filled tube).
TopoDS_Shape swept_profile(const std::vector<std::vector<double>> &outer,
                           const std::vector<std::vector<std::vector<double>>> &inners, bool is_area) {
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
ShapeHandle build_extruded_area_solid_impl(const std::vector<std::vector<double>> &outer,
                                           const std::vector<std::vector<std::vector<double>>> &inners,
                                           std::array<double, 3> loc, std::array<double, 3> axis,
                                           std::array<double, 3> ref_dir, double depth, bool is_area) {
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
ShapeHandle build_extruded_area_solid_tapered_impl(const std::vector<std::vector<double>> &outer_start,
                                                   const std::vector<std::vector<double>> &outer_end,
                                                   std::array<double, 3> loc, std::array<double, 3> axis,
                                                   std::array<double, 3> ref_dir, double depth) {
    const TopoDS_Wire wire1 = wire_from_edges(outer_start);
    TopoDS_Wire wire2 = wire_from_edges(outer_end);
    // End profile sits at depth along +Z (identity rotation + Z translation).
    wire2 = TopoDS::Wire(place_at(wire2, {0.0, 0.0, depth}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}));

    BRepOffsetAPI_ThruSections ts(Standard_True); // is_solid
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
ShapeHandle loft_profiles_impl(const std::vector<std::vector<std::array<double, 3>>> &profiles, bool ruled,
                               bool solid) {
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
ShapeHandle section_with_plane_impl(const ShapeHandle &shape, std::array<double, 3> origin,
                                    std::array<double, 3> normal, double size) {
    const gp_Pln pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(normal[0], normal[1], normal[2]));
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

void write_step_impl(const std::vector<ShapeHandle> &shapes, const std::vector<std::string> &names,
                     const std::vector<std::array<double, 3>> &colors, const std::string &filename,
                     const std::string &unit, const std::string &schema) {
    std::vector<TopoDS_Shape> tshapes;
    tshapes.reserve(shapes.size());
    for (const auto &s : shapes)
        tshapes.push_back(s.topods());
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
ShapeHandle build_revolved_area_solid_impl(const std::vector<std::vector<double>> &outer,
                                           const std::vector<std::vector<std::vector<double>>> &inners,
                                           std::array<double, 3> loc, std::array<double, 3> axis,
                                           std::array<double, 3> ref_dir, std::array<double, 3> axis_loc,
                                           std::array<double, 3> axis_dir, double angle_deg, bool is_area) {
    TopoDS_Shape profile = swept_profile(outer, inners, is_area);
    profile = place_at(profile, loc, axis, ref_dir);
    const gp_Ax1 rev_axis(gp_Pnt(axis_loc[0], axis_loc[1], axis_loc[2]), gp_Dir(axis_dir[0], axis_dir[1], axis_dir[2]));
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
ShapeHandle build_fixed_reference_swept_area_solid_impl(const std::vector<std::vector<double>> &directrix,
                                                        const std::vector<std::vector<double>> &profile_outer,
                                                        std::array<double, 3> loc) {
    const TopoDS_Wire spine = wire_from_edges(directrix);
    const TopoDS_Wire profile_wire = wire_from_edges(profile_outer);

    BRepOffsetAPI_MakePipeShell mps(spine);
    mps.SetTransitionMode(BRepBuilderAPI_RoundCorner);
    mps.Add(profile_wire, Standard_True, Standard_False); // with contact, no correction
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
ShapeHandle build_swept_disk_solid_impl(const std::vector<std::vector<double>> &directrix, double radius,
                                        double inner_radius) {
    const TopoDS_Wire spine = wire_from_edges(directrix);

    // Start point + tangent of the spine (a circle is rotationally symmetric, so
    // the tangent sign is irrelevant — only the plane it defines matters).
    TopExp_Explorer exp(spine, TopAbs_EDGE);
    if (!exp.More())
        throw std::runtime_error("build_swept_disk_solid: empty directrix");
    const TopoDS_Edge first_edge = TopoDS::Edge(exp.Current());
    BRepAdaptor_Curve adaptor(first_edge);
    gp_Pnt p0;
    gp_Vec d0;
    adaptor.D1(adaptor.FirstParameter(), p0, d0);
    if (d0.Magnitude() < 1e-12)
        throw std::runtime_error("build_swept_disk_solid: degenerate start tangent");
    const gp_Ax2 disk_axis(p0, gp_Dir(d0));

    auto sweep = [&](double r) -> TopoDS_Shape {
        const TopoDS_Edge circ_edge = BRepBuilderAPI_MakeEdge(gp_Circ(disk_axis, circle_radius_or_throw(r))).Edge();
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
using SurfData = std::tuple<std::string, Pt3, std::vector<EdgeData>, std::vector<Pt3>, std::vector<std::vector<Pt3>>>;

std::string cs_surface_type_name(const TopoDS_Face &face) {
    BRepAdaptor_Surface surf(face, Standard_True);
    switch (surf.GetType()) {
    case GeomAbs_Plane:
        return "Plane";
    case GeomAbs_Cylinder:
        return "Cylinder";
    case GeomAbs_Cone:
        return "Cone";
    case GeomAbs_Sphere:
        return "Sphere";
    default:
        return "Other";
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
        gp_Pnt p;
        gp_Vec du, dv;
        surf.D1(um, vm, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-12)
            return {0.0, 0.0, 1.0};
        n.Normalize();
        d = {n.X(), n.Y(), n.Z()};
    }
    if (face.Orientation() == TopAbs_REVERSED) {
        d[0] = -d[0];
        d[1] = -d[1];
        d[2] = -d[2];
    }
    return d;
}

std::string cs_curve_type_name(const BRepAdaptor_Curve &c) {
    switch (c.GetType()) {
    case GeomAbs_Line:
        return "Line";
    case GeomAbs_Circle:
        return "Circle";
    case GeomAbs_Ellipse:
        return "Ellipse";
    case GeomAbs_Hyperbola:
        return "Hyperbola";
    case GeomAbs_Parabola:
        return "Parabola";
    case GeomAbs_BezierCurve:
        return "BezierCurve";
    case GeomAbs_BSplineCurve:
        return "BSplineCurve";
    case GeomAbs_OffsetCurve:
        return "OffsetCurve";
    default:
        return "Other";
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
        if (ex.Orientation() == TopAbs_REVERSED)
            std::reverse(pts.begin(), pts.end());
        if (!edges.empty() && !pts.empty()) {
            const auto &prev = std::get<1>(edges.back());
            if (cs_point_dist(prev.back(), pts.front()) <= tol)
                pts.front() = prev.back();
        }
        if (pts.size() >= 2)
            edges.emplace_back(etype, std::move(pts));
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
    if (poly.size() >= 2 && cs_point_dist(poly.front(), poly.back()) <= tol)
        poly.pop_back();
    return poly;
}

ShapeHandle make_halfspace_impl(std::array<double, 3> origin, std::array<double, 3> normal, bool flip) {
    const gp_Pln pln(gp_Pnt(origin[0], origin[1], origin[2]), gp_Dir(normal[0], normal[1], normal[2]));
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(pln).Face();
    const double off = flip ? -1.0 : 1.0;
    const gp_Pnt ref(origin[0] + normal[0] * off, origin[1] + normal[1] * off, origin[2] + normal[2] * off);
    return ShapeHandle(BRepPrimAPI_MakeHalfSpace(face, ref).Solid());
}

std::vector<SurfData> cut_surfaces_impl(const ShapeHandle &solid_sh, const std::vector<ShapeHandle> &cutters,
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
        if (!algo.IsDone())
            throw std::runtime_error("cut_surfaces: boolean cut failed");
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
        if (descendants.Contains(rf))
            continue;

        const TopoDS_Wire outer_wire = BRepTools::OuterWire(rf);
        std::vector<EdgeData> outer_edges = cs_wire_to_edges(outer_wire, deflection, tol);
        std::vector<Pt3> outer = cs_edges_to_polyline(outer_edges, tol);
        if (outer.size() < 3)
            continue;

        std::vector<std::vector<Pt3>> inners;
        for (TopExp_Explorer we(rf, TopAbs_WIRE); we.More(); we.Next()) {
            const TopoDS_Wire w = TopoDS::Wire(we.Current());
            if (w.IsSame(outer_wire))
                continue;
            inners.push_back(cs_edges_to_polyline(cs_wire_to_edges(w, deflection, tol), tol));
        }
        out.emplace_back(cs_surface_type_name(rf), cs_face_normal(rf), std::move(outer_edges), std::move(outer),
                         std::move(inners));
    }
    return out;
}

// Planar face → (origin, normal); std::nullopt (→ Python None) if non-planar.
std::optional<std::pair<std::array<double, 3>, std::array<double, 3>>> face_plane_impl(const ShapeHandle &face_sh) {
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

// merge cleanup: meshopt_simplify (LockBorder) toward threshold*index_count within
// target_error, then drop degenerate tris + compact. Returns (positions xyz-interleaved, indices).
std::pair<std::vector<float>, std::vector<uint32_t>> meshopt_simplify_mesh_impl(const std::vector<float> &positions,
                                                                                const std::vector<uint32_t> &indices,
                                                                                float threshold, float target_error) {
    ngeom::SimplifiedMesh r = ngeom::meshopt_simplify_mesh(positions, indices, threshold, target_error);
    return {std::move(r.positions), std::move(r.indices)};
}

// EXT_meshopt_compression codecs (replace the sdist-only PyPI `meshoptimizer`). The adapy GLB
// packer feeds raw bytes; we wrap the vendored meshoptimizer C encoder/decoder.
nb::bytes meshopt_encode_vertex_buffer_impl(nb::bytes data, size_t count, size_t stride) {
    auto v = ngeom::meshopt_encode_vertices(data.c_str(), count, stride);
    return nb::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}
nb::bytes meshopt_encode_index_sequence_impl(nb::bytes idx, size_t count, size_t vertex_count) {
    auto v = ngeom::meshopt_encode_indices(reinterpret_cast<const uint32_t *>(idx.c_str()), count, vertex_count);
    return nb::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}
nb::bytes meshopt_decode_vertex_buffer_impl(nb::bytes enc, size_t count, size_t stride) {
    auto v =
        ngeom::meshopt_decode_vertices(reinterpret_cast<const unsigned char *>(enc.c_str()), enc.size(), count, stride);
    return nb::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}
nb::bytes meshopt_decode_index_sequence_impl(nb::bytes enc, size_t count, size_t index_size) {
    auto v = ngeom::meshopt_decode_indices(reinterpret_cast<const unsigned char *>(enc.c_str()), enc.size(), count,
                                           index_size);
    return nb::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}

} // namespace

// B-rep file→file writers (STEP↔IFC) moved to a shared dep-free header so the embind wasm writer
// (brep_writer_wasm.cpp) reuses ONE implementation. Brings emit_solid_ifc / emit_spatial_tree /
// ifc_header_block / write_ifc_file_impl / step_header_block / emit_solid_step / emit_step_assembly_tree
// / write_ifc_to_step_impl (+ IfcPath / IfcPath2) into scope for the nb:: writers that still call them.
using namespace adacpp::brep_convert;

// Native IFC writer from NGEOM blobs (the lazy ShapeStore form) + their out-of-band metadata (colour,
// world transforms, spatial paths). The ada-object-model counterpart of write_ifc_file_impl (which
// reads a STEP file): decode each blob to its NgeomRoot(s), re-attach the passed metadata (the blob
// carries geometry + root.id only), and emit the SAME IfcAdvancedBrep + IfcStyledItem + spatial tree.
// This is what makes a fully native Assembly.to_ifc(writer="native") possible — no STEP round-trip.
static adacpp::ifc_emit::FileStats
blobs_to_ifc_impl(const std::vector<nb::bytes> &blobs, const std::vector<std::array<float, 4>> &colors,
                  const std::vector<std::vector<std::array<float, 16>>> &transforms,
                  const std::vector<std::vector<std::vector<std::pair<int, std::string>>>> &paths,
                  const std::string &out_path, const std::string &schema, double unit_scale) {
    using namespace adacpp::ifc_emit;
    FileStats fs;
    fs.unit_scale = unit_scale;
    std::FILE *fp = std::fopen(out_path.c_str(), "wb");
    if (!fp)
        return fs;
    std::string buf;
    buf.reserve(1 << 22);
    auto flush = [&](bool force) {
        if (buf.size() >= (4u << 20) || force) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
    };
    buf += ifc_header_block(schema, unit_scale);
    BrepEmitter em(100, nullptr, 2.0, 20.0);
    std::vector<long> proxies;
    std::vector<IfcPath> proxy_paths;
    long sid = 0;
    for (size_t i = 0; i < blobs.size(); ++i) {
        adacpp::ngeom::NgeomDoc doc =
            adacpp::ngeom::decode(reinterpret_cast<const uint8_t *>(blobs[i].c_str()), blobs[i].size());
        for (adacpp::ngeom::NgeomRoot &root : doc.roots) {
            ++sid;
            ++fs.solids_in;
            if (i < colors.size() && colors[i][3] >= 0.0f) { // alpha<0 sentinel => no colour
                root.has_color = true;
                root.cr = colors[i][0];
                root.cg = colors[i][1];
                root.cb = colors[i][2];
                root.ca = colors[i][3];
            }
            if (i < transforms.size())
                root.transforms = transforms[i];
            if (i < paths.size())
                root.instance_paths = paths[i];
            size_t before = proxies.size();
            emit_solid_ifc(em, buf, root, sid, (uint64_t) sid * 1000u, proxies, &proxy_paths);
            if (proxies.size() > before)
                ++fs.solids_out;
        }
        flush(false);
    }
    emit_spatial_tree(buf, [&]() { return em.alloc_id(); }, proxies, proxy_paths);
    buf += "ENDSEC;\nEND-ISO-10303-21;\n";
    flush(true);
    std::fclose(fp);
    fs.geom = em.stats();
    return fs;
}

// Append `src` to `out`, adding `offset` to every entity id (#N defs + refs) WHOSE id > `keep_below`
// — so references to the shared header block (#1..#keep_below, e.g. #4/#6/#11/#12) are left intact
// while the solid's local ids are shifted to its reserved global block. Skips '#' inside
// single-quoted SPF strings so a '#' in a name/GUID isn't mangled ('' is an escaped quote).
static void renumber_into(std::string &out, const std::string &src, long offset, long keep_below) {
    out.reserve(out.size() + src.size() + src.size() / 8);
    bool in_str = false;
    size_t i = 0, nsz = src.size();
    while (i < nsz) {
        char ch = src[i];
        if (in_str) {
            out += ch;
            if (ch == '\'') {
                if (i + 1 < nsz && src[i + 1] == '\'') {
                    out += '\'';
                    i += 2;
                    continue;
                }
                in_str = false;
            }
            ++i;
            continue;
        }
        if (ch == '\'') {
            in_str = true;
            out += ch;
            ++i;
            continue;
        }
        if (ch == '#' && i + 1 < nsz && src[i + 1] >= '0' && src[i + 1] <= '9') {
            size_t j = i + 1;
            long v = 0;
            while (j < nsz && src[j] >= '0' && src[j] <= '9')
                v = v * 10 + (src[j++] - '0');
            out += '#';
            out += std::to_string(v > keep_below ? v + offset : v);
            i = j;
            continue;
        }
        out += ch;
        ++i;
    }
}

// Phase 3: PARALLEL STEP->IFC. Mirrors the mesh/glb harness — one shared StreamIndex, per-worker
// Resolver (copy_metadata_from), LPT scheduling, per-worker temp lanes. IFC's single global id space
// is handled by giving solid at LPT-index i a DISJOINT id block [K + i*STRIDE, K + (i+1)*STRIDE)
// (STRIDE sized from the max face count + slack; gaps are legal in SPF) — so workers never collide
// and no renumbering pass is needed. NO parse_cache_ bounding (that races multi-threaded — the
// fc37d71 segfault); per-worker clear_geom_cache() per solid bounds memory the safe way. Final pass
// (single-threaded): header + concat lanes + one IfcRelContainedInSpatialStructure over all proxies.
static adacpp::ifc_emit::FileStats write_ifc_file_parallel_impl(const std::string &in_path, const std::string &out_path,
                                                                const std::string &schema, double deflection,
                                                                double angular_deg, int num_threads, long max_solids) {
    using namespace adacpp::ifc_emit;
    using adacpp::ngeom::NgeomRoot;
    FileStats fs;
    adacpp::prof::StepProfiler prof("stream_step_to_ifc(par)");
    adacpp::tune_malloc_for_streaming();
    auto idx = adacpp::step::StreamIndex::from_file(in_path);
    prof.phase("scan_index");
    adacpp::step::Resolver master(idx);
    master.build_metadata(idx.lists);
    fs.unit_scale = master.unit_scale();
    prof.phase("metadata");

    std::vector<long> roots(idx.lists.roots.begin(), idx.lists.roots.end());
    if (max_solids > 0 && (long) roots.size() > max_solids)
        roots.resize(max_solids);
    // LPT order (one cost-estimate pass) — load-balance the heavy solids first.
    {
        std::vector<std::pair<size_t, long>> cost;
        cost.reserve(roots.size());
        for (long sid : roots)
            cost.emplace_back(master.solid_cost_estimate(sid), sid);
        master.clear_geom_cache();
        std::sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (size_t i = 0; i < cost.size(); ++i)
            roots[i] = cost[i].second;
    }
    prof.phase("lpt_order");
    const long K = 13; // ids #1..#13 are the shared header block (Project + root IfcSpatialZone + rel)
    // Robust global id allocation: each solid is emitted with LOCAL ids (1..n), then a contiguous
    // block of n ids is reserved atomically and the solid's text is renumbered by the block base.
    // No STRIDE guessing, no overflow, compact ids — correct regardless of per-face entity counts.
    std::atomic<long> id_counter{K + 1};

    int nth = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();
    if (nth > (int) roots.size())
        nth = std::max(1, (int) roots.size());

    std::string tmpl = adacpp::temp_template("adacpp_ifc");
    char *dir = ::mkdtemp(tmpl.data());
    if (!dir)
        return fs;
    std::string tdir = dir;
    struct Lane {
        std::FILE *fp = nullptr;
        std::string path, buf;
        std::vector<long> proxies;
        std::vector<IfcPath> proxy_paths; // assembly path per proxy (parallel to proxies)
        EmitStats stats;
        long solids_out = 0;
    };
    std::vector<Lane> lanes(nth);
    for (int t = 0; t < nth; ++t) {
        lanes[t].path = tdir + "/lane_" + std::to_string(t) + ".ifc";
        lanes[t].fp = std::fopen(lanes[t].path.c_str(), "wb");
        lanes[t].buf.reserve(1 << 22);
    }
    std::atomic<size_t> next{0};
    std::atomic<long> solids_in{0};

    auto worker = [&](int t) {
        // All profiling costs vanish when ADACPP_STEP_PROFILE is unset: prof.solid()
        // early-returns on one bool, and the per-thread timestamps are guarded here.
        const bool prof_on = prof.on();
        auto w0 = prof_on ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        size_t w_solids = 0;
        adacpp::step::Resolver r(idx);
        r.copy_metadata_from(master);
        Lane &L = lanes[t];
        std::string sb; // per-solid local-id buffer (renumbered into L.buf)
        auto flush = [&](bool force) {
            if (L.buf.size() >= (4u << 20) || force) {
                std::fwrite(L.buf.data(), 1, L.buf.size(), L.fp);
                L.buf.clear();
            }
        };
        int local = 0;
        for (;;) {
            size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= roots.size())
                break;
            NgeomRoot root = r.resolve_root(roots[i]);
            solids_in.fetch_add(1, std::memory_order_relaxed);
            prof.solid(root.faces.size());
            ++w_solids;
            // Emit with LOCAL ids starting ABOVE the shared header block (K+1..), so shared refs
            // (#1..#K) are distinguishable and left intact by renumber. Then reserve a contiguous
            // global block of n ids atomically and shift the solid's local ids into it.
            sb.clear();
            BrepEmitter em(K, nullptr, deflection, angular_deg);
            std::vector<long> local_proxies;
            std::vector<IfcPath> local_paths;
            emit_solid_ifc(em, sb, root, roots[i], (uint64_t) i * 1000u, local_proxies, &local_paths);
            long n = em.current_id() - K; // local ids K+1..K+n
            if (!local_proxies.empty()) {
                long gbase = id_counter.fetch_add(n, std::memory_order_relaxed); // reserve [gbase, gbase+n-1]
                long offset = gbase - (K + 1);                                   // local K+1 -> gbase
                renumber_into(L.buf, sb, offset, K);
                for (long p : local_proxies)
                    L.proxies.push_back(p + offset);
                for (auto &pth : local_paths)
                    L.proxy_paths.push_back(std::move(pth));
                ++L.solids_out;
            }
            const EmitStats &s = em.stats();
            L.stats.faces_in += s.faces_in;
            L.stats.faces_out += s.faces_out;
            L.stats.faces_dropped += s.faces_dropped;
            L.stats.edges_analytic += s.edges_analytic;
            L.stats.edges_polyline_approx += s.edges_polyline_approx;
            L.stats.edges_degenerate += s.edges_degenerate;
            for (const auto &[k, v] : s.drop_reasons)
                L.stats.drop_reasons[k] += v;
            r.clear_geom_cache();
            if (++local % 128 == 0)
                adacpp::mem_trim();
            flush(false);
        }
        if (r.degenerate_faces_skipped_ > 0)
            L.stats.drop_reasons["face:degenerate-skipped(read)"] += r.degenerate_faces_skipped_;
        flush(true);
        std::fclose(L.fp);
        L.fp = nullptr;
        if (prof_on)
            prof.thread_done(
                t, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count(), w_solids);
    };
    std::vector<std::thread> pool;
    pool.reserve(nth - 1);
    for (int t = 1; t < nth; ++t)
        pool.emplace_back(worker, t);
    worker(0);
    for (std::thread &th : pool)
        th.join();
    prof.phase("emit_lanes(parallel)");

    // assemble: header + concat lanes (ids globally disjoint) + containment over all proxies.
    std::FILE *out_fp = std::fopen(out_path.c_str(), "wb");
    if (!out_fp) {
        std::filesystem::remove_all(tdir);
        return fs;
    }
    std::string hdr = ifc_header_block(schema, master.unit_scale());
    std::fwrite(hdr.data(), 1, hdr.size(), out_fp);
    std::vector<long> all_proxies;
    std::vector<IfcPath> all_paths;
    std::vector<char> io(1 << 20);
    for (int t = 0; t < nth; ++t) {
        Lane &L = lanes[t];
        std::FILE *lf = std::fopen(L.path.c_str(), "rb");
        if (lf) {
            size_t n;
            while ((n = std::fread(io.data(), 1, io.size(), lf)) > 0)
                std::fwrite(io.data(), 1, n, out_fp);
            std::fclose(lf);
        }
        all_proxies.insert(all_proxies.end(), L.proxies.begin(), L.proxies.end());
        for (auto &pth : L.proxy_paths)
            all_paths.push_back(std::move(pth));
        fs.solids_out += L.solids_out;
        fs.geom.faces_in += L.stats.faces_in;
        fs.geom.faces_out += L.stats.faces_out;
        fs.geom.faces_dropped += L.stats.faces_dropped;
        fs.geom.edges_analytic += L.stats.edges_analytic;
        fs.geom.edges_polyline_approx += L.stats.edges_polyline_approx;
        fs.geom.edges_degenerate += L.stats.edges_degenerate;
        for (const auto &[k, v] : L.stats.drop_reasons)
            fs.geom.drop_reasons[k] += v;
    }
    fs.solids_in = solids_in.load();
    // Nested IfcElementAssembly tree over all proxies (ids from the shared atomic counter).
    std::string tree;
    emit_spatial_tree(tree, [&]() { return id_counter.fetch_add(1); }, all_proxies, all_paths);
    std::fwrite(tree.data(), 1, tree.size(), out_fp);
    const char *foot = "ENDSEC;\nEND-ISO-10303-21;\n";
    std::fwrite(foot, 1, std::strlen(foot), out_fp);
    std::fclose(out_fp);
    std::filesystem::remove_all(tdir);
    prof.phase("assemble+write");
    return fs;
}

// Native parallel STEP->STEP (AP242). Mirrors the parallel IFC writer: shared StreamIndex, per-worker
// Resolver, LPT, per-worker lanes, local ids -> atomic-reserve -> renumber. Multi-instance B-rep
// solids with rigid placements are emitted ONCE as a shared prototype + per-placement NAUO/CDSR/
// ITEM_DEFINED_TRANSFORMATION references (the AP242 assembly-instancing pattern both native stream
// readers and OCC consume — see emit_step_mapped_instances); everything else keeps the baked form
// (per-instance transform applied to the geometry). ADACPP_STEP_BAKE_INSTANCES=1 forces all-baked.
// nth=1 = serial. NO parse_cache_ bounding (per-worker clear_geom_cache per solid).
// Round-trip-lossless.
static adacpp::ifc_emit::FileStats write_step_file_impl(const std::string &in_path, const std::string &out_path,
                                                        double deflection, double angular_deg, int num_threads,
                                                        long max_solids) {
    using adacpp::ngeom::NgeomRoot;
    using adacpp::step_emit::StepBrepEmitter;
    adacpp::ifc_emit::FileStats fs;
    adacpp::prof::StepProfiler prof("stream_step_to_step");
    adacpp::tune_malloc_for_streaming();
    auto idx = adacpp::step::StreamIndex::from_file(in_path);
    prof.phase("scan_index");
    adacpp::step::Resolver master(idx);
    master.build_metadata(idx.lists);
    fs.unit_scale = master.unit_scale();
    prof.phase("metadata");
    std::vector<long> roots(idx.lists.roots.begin(), idx.lists.roots.end());
    if (max_solids > 0 && (long) roots.size() > max_solids)
        roots.resize(max_solids);
    {
        std::vector<std::pair<size_t, long>> cost;
        cost.reserve(roots.size());
        for (long sid : roots)
            cost.emplace_back(master.solid_cost_estimate(sid), sid);
        master.clear_geom_cache();
        std::sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (size_t i = 0; i < cost.size(); ++i)
            roots[i] = cost[i].second;
    }
    prof.phase("lpt_order");
    // Mapped instancing needs a shared assembly-root block: #14..#17 = root PRODUCT chain, #18 =
    // the world SHAPE_REPRESENTATION (its item list is only known after all lanes ran, so the
    // record itself is written at assemble time — forward references are legal SPF). Ids <= K are
    // protected from renumbering, so reserve them only when instancing can actually occur; a flat
    // source keeps the old K=13 layout byte-for-byte.
    const bool mapped_on = !step_bake_instances_forced() && master.any_multi_instance();
    const long K = mapped_on ? 18 : 13;
    const long ROOT_PD = 16, WORLD_REP = 18, IDENTITY_AXIS = 13;
    std::atomic<long> id_counter{K + 1};
    int nth = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();
    if (nth > (int) roots.size())
        nth = std::max(1, (int) roots.size());
    std::string tmpl = adacpp::temp_template("adacpp_stp");
    char *dir = ::mkdtemp(tmpl.data());
    if (!dir)
        return fs;
    std::string tdir = dir;
    struct Lane {
        std::FILE *fp = nullptr;
        std::string path, buf;
        adacpp::ifc_emit::EmitStats stats;
        long solids_out = 0;
        long instances_out = 0;
        std::vector<long> world_axes; // global ids of per-instance AXIS2s -> the world rep's items
    };
    std::vector<Lane> lanes(nth);
    for (int t = 0; t < nth; ++t) {
        lanes[t].path = tdir + "/lane_" + std::to_string(t) + ".stp";
        lanes[t].fp = std::fopen(lanes[t].path.c_str(), "wb");
        lanes[t].buf.reserve(1 << 22);
    }
    std::atomic<size_t> next{0};
    std::atomic<long> solids_in{0};
    auto worker = [&](int t) {
        // All profiling costs vanish when ADACPP_STEP_PROFILE is unset: prof.solid()
        // early-returns on one bool, and the per-thread timestamps are guarded here.
        const bool prof_on = prof.on();
        auto w0 = prof_on ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        size_t w_solids = 0;
        adacpp::step::Resolver r(idx);
        r.copy_metadata_from(master);
        Lane &L = lanes[t];
        std::string sb;
        auto flush = [&](bool force) {
            if (L.buf.size() >= (4u << 20) || force) {
                std::fwrite(L.buf.data(), 1, L.buf.size(), L.fp);
                L.buf.clear();
            }
        };
        int local = 0;
        for (;;) {
            size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= roots.size())
                break;
            NgeomRoot root = r.resolve_root(roots[i]);
            solids_in.fetch_add(1, std::memory_order_relaxed);
            prof.solid(root.faces.size());
            ++w_solids;
            bool any = false;
            if (mapped_on && step_mapped_eligible(root)) {
                // Shared prototype ONCE (identity frame) + one NAUO/CDSR/IDT reference per placement.
                sb.clear();
                StepBrepEmitter em(K, nullptr, deflection, angular_deg);
                long rep_id = 0;
                long pd = emit_solid_step(em, sb, root, roots[i], &rep_id);
                if (pd) {
                    std::string nm =
                        root.id.empty() ? ("solid_" + std::to_string(roots[i])) : adacpp::ifc_emit::ifc_str(root.id);
                    std::vector<long> axes_local;
                    emit_step_mapped_instances(em, sb, rep_id, pd, nm, roots[i], root.transforms, ROOT_PD, WORLD_REP,
                                               IDENTITY_AXIS, axes_local);
                    long n = em.current_id() - K;
                    long gbase = id_counter.fetch_add(n, std::memory_order_relaxed);
                    long offset = gbase - (K + 1);
                    renumber_into(L.buf, sb, offset, K);
                    for (long a : axes_local)
                        L.world_axes.push_back(a + offset);
                    any = true;
                    L.instances_out += (long) root.transforms.size();
                    const auto &s = em.stats();
                    L.stats.faces_in += s.faces_in;
                    L.stats.faces_out += s.faces_out;
                    L.stats.faces_dropped += s.faces_dropped;
                    L.stats.edges_analytic += s.edges_analytic;
                    L.stats.edges_polyline_approx += s.edges_polyline_approx;
                    L.stats.edges_degenerate += s.edges_degenerate;
                    for (const auto &[rk, rv] : s.drop_reasons)
                        L.stats.drop_reasons[rk] += rv;
                }
            } else {
                // One baked part per instance (or one identity part when flat).
                size_t ninst = root.transforms.empty() ? 1 : root.transforms.size();
                for (size_t k = 0; k < ninst; ++k) {
                    double tf[16];
                    const double *tfp = nullptr;
                    if (!root.transforms.empty()) {
                        // ng:: transforms are column-major glTF; StepBrepEmitter wants row-major.
                        const std::array<float, 16> &M = root.transforms[k];
                        tf[0] = M[0];
                        tf[1] = M[4];
                        tf[2] = M[8];
                        tf[3] = M[12];
                        tf[4] = M[1];
                        tf[5] = M[5];
                        tf[6] = M[9];
                        tf[7] = M[13];
                        tf[8] = M[2];
                        tf[9] = M[6];
                        tf[10] = M[10];
                        tf[11] = M[14];
                        tfp = tf;
                    }
                    sb.clear();
                    StepBrepEmitter em(K, tfp, deflection, angular_deg);
                    bool ok = emit_solid_step(em, sb, root, roots[i]);
                    long n = em.current_id() - K;
                    if (ok) {
                        long gbase = id_counter.fetch_add(n, std::memory_order_relaxed);
                        renumber_into(L.buf, sb, gbase - (K + 1), K);
                        any = true;
                        ++L.instances_out;
                        const auto &s = em.stats();
                        L.stats.faces_in += s.faces_in;
                        L.stats.faces_out += s.faces_out;
                        L.stats.faces_dropped += s.faces_dropped;
                        L.stats.edges_analytic += s.edges_analytic;
                        L.stats.edges_polyline_approx += s.edges_polyline_approx;
                        L.stats.edges_degenerate += s.edges_degenerate;
                        for (const auto &[rk, rv] : s.drop_reasons)
                            L.stats.drop_reasons[rk] += rv;
                    }
                }
            }
            if (any)
                ++L.solids_out;
            r.clear_geom_cache();
            if (++local % 128 == 0)
                adacpp::mem_trim();
            flush(false);
        }
        if (r.degenerate_faces_skipped_ > 0)
            L.stats.drop_reasons["face:degenerate-skipped(read)"] += r.degenerate_faces_skipped_;
        flush(true);
        std::fclose(L.fp);
        L.fp = nullptr;
        if (prof_on)
            prof.thread_done(
                t, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count(), w_solids);
    };
    std::vector<std::thread> pool;
    pool.reserve(nth - 1);
    for (int t = 1; t < nth; ++t)
        pool.emplace_back(worker, t);
    worker(0);
    for (std::thread &th : pool)
        th.join();
    prof.phase("emit_lanes(parallel)");
    std::FILE *out_fp = std::fopen(out_path.c_str(), "wb");
    if (!out_fp) {
        std::filesystem::remove_all(tdir);
        return fs;
    }
    std::string hdr = step_header_block(master.unit_scale());
    std::fwrite(hdr.data(), 1, hdr.size(), out_fp);
    // Shared assembly-root block for mapped instances: root PRODUCT chain (#14..#17) + the world
    // SHAPE_REPRESENTATION #18 listing every instance AXIS2 among its items (deferred until now
    // because the item list spans all lanes; statement order is irrelevant in SPF). Skipped when no
    // solid actually took the mapped path — the unused reserved ids are legal gaps.
    if (mapped_on) {
        std::vector<long> all_axes;
        for (int t = 0; t < nth; ++t)
            all_axes.insert(all_axes.end(), lanes[t].world_axes.begin(), lanes[t].world_axes.end());
        if (!all_axes.empty()) {
            std::string rb;
            rb += "#14=PRODUCT('model','model','',(#3));\n";
            rb += "#15=PRODUCT_DEFINITION_FORMATION('','',#14);\n";
            rb += "#16=PRODUCT_DEFINITION('design','',#15,#4);\n";
            rb += "#17=PRODUCT_DEFINITION_SHAPE('','',#16);\n";
            rb += "#18=SHAPE_REPRESENTATION('model',(#13";
            for (long a : all_axes) {
                rb += ",#";
                rb += std::to_string(a);
            }
            rb += "),#9);\n";
            long sdr_id = id_counter.fetch_add(1, std::memory_order_relaxed);
            rb += "#" + std::to_string(sdr_id) + "=SHAPE_DEFINITION_REPRESENTATION(#17,#18);\n";
            std::fwrite(rb.data(), 1, rb.size(), out_fp);
        }
    }
    std::vector<char> io(1 << 20);
    for (int t = 0; t < nth; ++t) {
        Lane &L = lanes[t];
        if (std::FILE *lf = std::fopen(L.path.c_str(), "rb")) {
            size_t n;
            while ((n = std::fread(io.data(), 1, io.size(), lf)) > 0)
                std::fwrite(io.data(), 1, n, out_fp);
            std::fclose(lf);
        }
        fs.solids_out += L.solids_out;
        fs.instances_out += L.instances_out;
        fs.geom.faces_in += L.stats.faces_in;
        fs.geom.faces_out += L.stats.faces_out;
        fs.geom.faces_dropped += L.stats.faces_dropped;
        fs.geom.edges_analytic += L.stats.edges_analytic;
        fs.geom.edges_polyline_approx += L.stats.edges_polyline_approx;
        fs.geom.edges_degenerate += L.stats.edges_degenerate;
        for (const auto &[k, v] : L.stats.drop_reasons)
            fs.geom.drop_reasons[k] += v;
    }
    fs.solids_in = solids_in.load();
    const char *foot = "ENDSEC;\nEND-ISO-10303-21;\n";
    std::fwrite(foot, 1, std::strlen(foot), out_fp);
    std::fclose(out_fp);
    std::filesystem::remove_all(tdir);
    prof.phase("assemble+write");
    return fs;
}

// Cross-format parity in ONE native parse: resolve every root a single time and run BOTH the
// STEP->IFC and STEP->STEP emitters over it, tallying each format's losslessness (solids/faces
// in-vs-out + drop reasons + placed-instance count) WITHOUT writing any output file. This is the
// fan-out of the two write_*_file_impl above: they each re-parse + re-resolve the whole model and
// serialise multi-GB output just so the parity check can count it; here the expensive per-solid
// resolve_root is shared and nothing is serialised to disk. Bounded memory (one resolved solid +
// a per-solid throwaway buffer per worker), parallel across roots.
struct ParityFormat {
    long solids_out = 0; // roots that emitted at least one solid in this format
    long instances = 0;  // placed occurrences written (proxies / baked parts) — the parity metric
    adacpp::ifc_emit::EmitStats geom;
    void merge(const ParityFormat &o) {
        solids_out += o.solids_out;
        instances += o.instances;
        geom.faces_in += o.geom.faces_in;
        geom.faces_out += o.geom.faces_out;
        geom.faces_dropped += o.geom.faces_dropped;
        geom.edges_analytic += o.geom.edges_analytic;
        geom.edges_polyline_approx += o.geom.edges_polyline_approx;
        geom.edges_degenerate += o.geom.edges_degenerate;
        for (const auto &[k, v] : o.geom.drop_reasons)
            geom.drop_reasons[k] += v;
    }
};

static void accum_geom(adacpp::ifc_emit::EmitStats &dst, const adacpp::ifc_emit::EmitStats &s) {
    dst.faces_in += s.faces_in;
    dst.faces_out += s.faces_out;
    dst.faces_dropped += s.faces_dropped;
    dst.edges_analytic += s.edges_analytic;
    dst.edges_polyline_approx += s.edges_polyline_approx;
    dst.edges_degenerate += s.edges_degenerate;
    for (const auto &[k, v] : s.drop_reasons)
        dst.drop_reasons[k] += v;
}

static void step_parity_impl(const std::string &in_path, double deflection, double angular_deg, int num_threads,
                             long max_solids, long &solids_in_out, long &total_instances_out, double &unit_scale_out,
                             ParityFormat &ifc_out, ParityFormat &step_out) {
    using adacpp::ngeom::NgeomRoot;
    using adacpp::step_emit::StepBrepEmitter;
    using namespace adacpp::ifc_emit;
    auto idx = adacpp::step::StreamIndex::from_file(in_path);
    adacpp::step::Resolver master(idx);
    master.build_metadata(idx.lists);
    unit_scale_out = master.unit_scale();
    std::vector<long> roots(idx.lists.roots.begin(), idx.lists.roots.end());
    if (max_solids > 0 && (long) roots.size() > max_solids)
        roots.resize(max_solids);
    {
        std::vector<std::pair<size_t, long>> cost;
        cost.reserve(roots.size());
        for (long sid : roots)
            cost.emplace_back(master.solid_cost_estimate(sid), sid);
        master.clear_geom_cache();
        std::sort(cost.begin(), cost.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (size_t i = 0; i < cost.size(); ++i)
            roots[i] = cost[i].second;
    }
    int nth = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();
    if (nth > (int) roots.size())
        nth = std::max(1, (int) roots.size());
    const bool bake_forced = step_bake_instances_forced(); // mirror write_step_file_impl's mode
    std::vector<ParityFormat> ifc_lanes(nth), step_lanes(nth);
    std::atomic<size_t> next{0};
    std::atomic<long> solids_in{0};
    std::atomic<long> total_instances{0}; // every placed occurrence in the source (the parity baseline)
    auto worker = [&](int t) {
        adacpp::step::Resolver r(idx);
        r.copy_metadata_from(master);
        ParityFormat &LI = ifc_lanes[t];
        ParityFormat &LS = step_lanes[t];
        std::string sb; // per-solid throwaway (emitters write here; we only read their stats)
        int local = 0;
        for (;;) {
            size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= roots.size())
                break;
            NgeomRoot root = r.resolve_root(roots[i]); // the shared parse — resolved ONCE, both formats
            solids_in.fetch_add(1, std::memory_order_relaxed);
            total_instances.fetch_add((long) (root.transforms.empty() ? 1 : root.transforms.size()),
                                      std::memory_order_relaxed);

            // ── IFC: geometry once + one proxy per instance (IfcMappedItem) ──
            sb.clear();
            BrepEmitter emi(17, nullptr, deflection, angular_deg);
            std::vector<long> proxies;
            emit_solid_ifc(emi, sb, root, roots[i], (uint64_t) i * 1000u, proxies, nullptr);
            if (!proxies.empty())
                ++LI.solids_out;
            LI.instances += (long) proxies.size();
            accum_geom(LI.geom, emi.stats());

            // ── STEP: mirror write_step_file_impl — shared prototype + per-placement references
            // for mapped-eligible solids (geometry counted ONCE, instances = placements), else one
            // baked part per instance ──
            bool any = false;
            if (!bake_forced && step_mapped_eligible(root)) {
                sb.clear();
                StepBrepEmitter ems(13, nullptr, deflection, angular_deg);
                if (emit_solid_step(ems, sb, root, roots[i])) {
                    any = true;
                    LS.instances += (long) root.transforms.size();
                    accum_geom(LS.geom, ems.stats());
                }
            } else {
                size_t ninst = root.transforms.empty() ? 1 : root.transforms.size();
                for (size_t k = 0; k < ninst; ++k) {
                    double tf[16];
                    const double *tfp = nullptr;
                    if (!root.transforms.empty()) {
                        const std::array<float, 16> &M = root.transforms[k];
                        tf[0] = M[0];
                        tf[1] = M[4];
                        tf[2] = M[8];
                        tf[3] = M[12];
                        tf[4] = M[1];
                        tf[5] = M[5];
                        tf[6] = M[9];
                        tf[7] = M[13];
                        tf[8] = M[2];
                        tf[9] = M[6];
                        tf[10] = M[10];
                        tf[11] = M[14];
                        tfp = tf;
                    }
                    sb.clear();
                    StepBrepEmitter ems(13, tfp, deflection, angular_deg);
                    if (emit_solid_step(ems, sb, root, roots[i])) {
                        any = true;
                        ++LS.instances;
                        accum_geom(LS.geom, ems.stats());
                    }
                }
            }
            if (any)
                ++LS.solids_out;

            r.clear_geom_cache();
            if (++local % 128 == 0)
                adacpp::mem_trim();
        }
        // Read-side skips (zero-area `$`-surface faces) — informational, not a
        // faces_dropped leg: the face carries no geometry in the source.
        if (r.degenerate_faces_skipped_ > 0) {
            LI.geom.drop_reasons["face:degenerate-skipped(read)"] += r.degenerate_faces_skipped_;
            LS.geom.drop_reasons["face:degenerate-skipped(read)"] += r.degenerate_faces_skipped_;
        }
    };
    std::vector<std::thread> pool;
    pool.reserve(nth - 1);
    for (int t = 1; t < nth; ++t)
        pool.emplace_back(worker, t);
    worker(0);
    for (std::thread &th : pool)
        th.join();
    for (int t = 0; t < nth; ++t) {
        ifc_out.merge(ifc_lanes[t]);
        step_out.merge(step_lanes[t]);
    }
    solids_in_out = solids_in.load();
    total_instances_out = total_instances.load();
}

// ---------------------------------------------------------------------------------------------
// NGEOM-blob record streams -> STEP / IFC files (the ada-object-model export fast path).
//
// The Python side walks an assembly, serializes each object's solid_geom() to an NGEOM blob
// (adapy serialize_geometries — the same vocabulary tessellate_stream decodes), and hands the
// records to these writers one at a time. Records are consumed LAZILY from any Python iterable
// (list or generator), and each record is decoded + emitted + released before the next is pulled,
// so a 100k-solid model streams at bounded memory. This replaces the ~ms/face per-entity Python
// writers (ifcopenshell / ap242_stream) with the ~µs/face C++ emitters for models whose objects
// all serialize to NGEOM.
//
// Record shape (a tuple/sequence; trailing elements optional):
//   (name, blob [, color_rgba [, transforms [, paths]]])
//     name        str|None — product name; non-empty overrides the blob's root id
//     blob        bytes    — one NGEOM buffer (usually one root; all roots are emitted)
//     color_rgba  4 floats — presentation colour; alpha < 0 (or None) = no colour
//     transforms  list of 16-float column-major (glTF) world placements; None/[] = one identity
//     paths       per-instance assembly path: list of [(rep_id, name), ...] levels, parallel to
//                 transforms (one entry total when transforms is empty)
struct NgeomEmitRecord {
    std::string name;
    nb::bytes blob;
    std::array<float, 4> color{0.f, 0.f, 0.f, -1.f}; // alpha<0 sentinel => no colour
    std::vector<std::array<float, 16>> transforms;
    std::vector<std::vector<std::pair<int, std::string>>> paths;
};

static NgeomEmitRecord parse_ngeom_record(nb::handle item) {
    NgeomEmitRecord rec;
    nb::sequence seq = nb::cast<nb::sequence>(item);
    size_t n = nb::len(seq);
    if (n < 2)
        throw std::runtime_error("NGEOM record needs at least (name, blob)");
    if (!seq[0].is_none())
        rec.name = nb::cast<std::string>(seq[0]);
    rec.blob = nb::cast<nb::bytes>(seq[1]);
    if (n > 2 && !seq[2].is_none())
        rec.color = nb::cast<std::array<float, 4>>(seq[2]);
    if (n > 3 && !seq[3].is_none())
        rec.transforms = nb::cast<std::vector<std::array<float, 16>>>(seq[3]);
    if (n > 4 && !seq[4].is_none())
        rec.paths = nb::cast<std::vector<std::vector<std::pair<int, std::string>>>>(seq[4]);
    return rec;
}

// Attach a record's out-of-band metadata to a decoded root (the blob carries geometry + id only).
static void apply_record_meta(adacpp::ngeom::NgeomRoot &root, const NgeomEmitRecord &rec) {
    if (!rec.name.empty())
        root.id = rec.name;
    if (rec.color[3] >= 0.0f) {
        root.has_color = true;
        root.cr = rec.color[0];
        root.cg = rec.color[1];
        root.cb = rec.color[2];
        root.ca = rec.color[3];
    }
    root.transforms = rec.transforms;
    root.instance_paths = rec.paths;
}

// NGEOM records -> AP242 STEP file. Serial (one running id, no renumber pass): decode each blob,
// emit each root via emit_solid_step — mapped-eligible multi-instance solids as ONE shared
// prototype + per-placement NAUO/CDSR references (write_step_file_impl's pattern), everything else
// baked per instance — then the NAUO assembly tree from the record paths, the mapped-instance root
// block (SPF statement order is irrelevant, so it can trail its references), and the colour trailer.
static adacpp::ifc_emit::FileStats stream_ngeom_to_step_impl(nb::iterable records, const std::string &out_path,
                                                             double unit_scale, double deflection, double angular_deg) {
    using adacpp::ngeom::NgeomRoot;
    using adacpp::step_emit::StepBrepEmitter;
    adacpp::ifc_emit::FileStats fs;
    fs.unit_scale = unit_scale;
    std::FILE *fp = std::fopen(out_path.c_str(), "wb");
    if (!fp)
        return fs;
    std::string hdr = step_header_block(unit_scale);
    std::fwrite(hdr.data(), 1, hdr.size(), fp);
    // Reserve the mapped-instance shared-root ids up front (#14..#17 root PRODUCT chain, #18 world
    // rep — written at the end, only if some solid actually took the mapped path; unused reserved
    // ids are legal SPF gaps and keep this writer renumber-free).
    const long K = 18;
    const long ROOT_PD = 16, WORLD_REP = 18, IDENTITY_AXIS = 13;
    long nid = K;
    std::string buf;
    buf.reserve(1 << 22);
    auto flush = [&](bool force) {
        if (buf.size() >= (4u << 20) || force) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
    };
    const bool bake_forced = step_bake_instances_forced();
    std::vector<long> leaf_pds;       // baked leaves (mapped leaves hang under ROOT_PD instead)
    std::vector<long> leaf_reps;      // parallel: each leaf's SHAPE_REPRESENTATION (CDSR child rep)
    std::vector<IfcPath2> leaf_paths; // parallel: each baked leaf's assembly path
    std::vector<long> styled;         // STYLED_ITEM ids for the presentation trailer
    std::vector<long> world_axes;     // mapped-instance AXIS2 ids -> the world rep's items
    long sid = 0;
    for (nb::handle item : records) {
        NgeomEmitRecord rec = parse_ngeom_record(item);
        adacpp::ngeom::NgeomDoc doc =
            adacpp::ngeom::decode(reinterpret_cast<const uint8_t *>(rec.blob.c_str()), rec.blob.size());
        for (NgeomRoot &root : doc.roots) {
            ++sid;
            ++fs.solids_in;
            apply_record_meta(root, rec);
            std::string nm = root.id.empty() ? ("solid_" + std::to_string(sid)) : adacpp::ifc_emit::ifc_str(root.id);
            bool any = false;
            if (!bake_forced && step_mapped_eligible(root)) {
                StepBrepEmitter em(nid, nullptr, deflection, angular_deg);
                long rep_id = 0, solid_id = 0;
                long pd = emit_solid_step(em, buf, root, sid, &rep_id, &solid_id);
                if (pd) {
                    emit_step_mapped_instances(em, buf, rep_id, pd, nm, sid, root.transforms, ROOT_PD, WORLD_REP,
                                               IDENTITY_AXIS, world_axes);
                    if (root.has_color && solid_id)
                        styled.push_back(emit_step_color(em, buf, solid_id, root.cr, root.cg, root.cb));
                    nid = em.current_id();
                    any = true;
                    fs.instances_out += (long) root.transforms.size();
                    accum_geom(fs.geom, em.stats());
                }
            } else {
                size_t ninst = root.transforms.empty() ? 1 : root.transforms.size();
                for (size_t k = 0; k < ninst; ++k) {
                    double tf[16];
                    const double *tfp = nullptr;
                    if (!root.transforms.empty()) {
                        // ng:: transforms are column-major glTF; StepBrepEmitter wants row-major.
                        const std::array<float, 16> &M = root.transforms[k];
                        tf[0] = M[0];
                        tf[1] = M[4];
                        tf[2] = M[8];
                        tf[3] = M[12];
                        tf[4] = M[1];
                        tf[5] = M[5];
                        tf[6] = M[9];
                        tf[7] = M[13];
                        tf[8] = M[2];
                        tf[9] = M[6];
                        tf[10] = M[10];
                        tf[11] = M[14];
                        tfp = tf;
                    }
                    StepBrepEmitter em(nid, tfp, deflection, angular_deg);
                    long solid_id = 0, rep_id = 0;
                    long pd = emit_solid_step(em, buf, root, sid, &rep_id, &solid_id);
                    if (pd) {
                        if (root.has_color && solid_id)
                            styled.push_back(emit_step_color(em, buf, solid_id, root.cr, root.cg, root.cb));
                        nid = em.current_id();
                        any = true;
                        ++fs.instances_out;
                        leaf_pds.push_back(pd);
                        leaf_reps.push_back(rep_id);
                        leaf_paths.push_back(k < root.instance_paths.size() ? root.instance_paths[k] : IfcPath2{});
                        accum_geom(fs.geom, em.stats());
                    }
                }
            }
            if (any)
                ++fs.solids_out;
            else
                ++fs.products_skipped;
            flush(false);
        }
    }
    fs.products_total = fs.solids_in;
    StepBrepEmitter emtail(nid, nullptr, deflection, angular_deg);
    emit_step_assembly_tree(emtail, buf, leaf_pds, leaf_reps, leaf_paths);
    if (!world_axes.empty()) {
        buf += "#14=PRODUCT('model','model','',(#3));\n";
        buf += "#15=PRODUCT_DEFINITION_FORMATION('','',#14);\n";
        buf += "#16=PRODUCT_DEFINITION('design','',#15,#4);\n";
        buf += "#17=PRODUCT_DEFINITION_SHAPE('','',#16);\n";
        buf += "#18=SHAPE_REPRESENTATION('model',(#13";
        for (long a : world_axes) {
            buf += ",#";
            buf += std::to_string(a);
        }
        buf += "),#9);\n";
        emtail.emit_entity(buf, "SHAPE_DEFINITION_REPRESENTATION(#17,#18)");
    }
    step_color_trailer(emtail, buf, styled);
    buf += "ENDSEC;\nEND-ISO-10303-21;\n";
    flush(true);
    std::fclose(fp);
    return fs;
}

// NGEOM records -> IFC file. The record-stream sibling of blobs_to_ifc_impl (which takes parallel
// lists): same emit_solid_ifc + IfcStyledItem + spatial tree, but records are pulled lazily so a
// generator streams at bounded memory.
static adacpp::ifc_emit::FileStats stream_ngeom_to_ifc_impl(nb::iterable records, const std::string &out_path,
                                                            const std::string &schema, double unit_scale,
                                                            double deflection, double angular_deg) {
    using namespace adacpp::ifc_emit;
    using adacpp::ngeom::NgeomRoot;
    FileStats fs;
    fs.unit_scale = unit_scale;
    std::FILE *fp = std::fopen(out_path.c_str(), "wb");
    if (!fp)
        return fs;
    std::string buf;
    buf.reserve(1 << 22);
    auto flush = [&](bool force) {
        if (buf.size() >= (4u << 20) || force) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
    };
    buf += ifc_header_block(schema, unit_scale);
    BrepEmitter em(100, nullptr, deflection, angular_deg);
    std::vector<long> proxies;
    std::vector<IfcPath> proxy_paths;
    long sid = 0;
    for (nb::handle item : records) {
        NgeomEmitRecord rec = parse_ngeom_record(item);
        adacpp::ngeom::NgeomDoc doc =
            adacpp::ngeom::decode(reinterpret_cast<const uint8_t *>(rec.blob.c_str()), rec.blob.size());
        for (NgeomRoot &root : doc.roots) {
            ++sid;
            ++fs.solids_in;
            apply_record_meta(root, rec);
            size_t before = proxies.size();
            // guid seed strided by 1000 per solid (instances use seed+k) — matches blobs_to_ifc.
            emit_solid_ifc(em, buf, root, sid, (uint64_t) sid * 1000u, proxies, &proxy_paths);
            if (proxies.size() > before)
                ++fs.solids_out;
            else
                ++fs.products_skipped;
            fs.instances_out += (long) (proxies.size() - before);
            flush(false);
        }
    }
    fs.products_total = fs.solids_in;
    emit_spatial_tree(buf, [&]() { return em.alloc_id(); }, proxies, proxy_paths);
    buf += "ENDSEC;\nEND-ISO-10303-21;\n";
    flush(true);
    std::fclose(fp);
    fs.geom = em.stats();
    return fs;
}

// One NGEOM blob -> the SPF text of its IFC geometry-body entity graph ONLY (the IfcAdvancedBrep /
// analytic-solid faces/edges/surfaces lines), numbered from `first_id`. No header/footer, no product
// or spatial entities, no styling — the caller (adapy's streaming IFC writer) splices the fragment
// into its own file and hand-authors the TYPED wrapper (IfcShapeRepresentation -> IfcPlate/...)
// around the returned body item id. This is what lets typed, round-trippable products keep the
// ~µs/face C++ geometry emit (stream_ngeom_to_ifc's proxy wrapper made every product an
// IfcBuildingElementProxy). The body-graph emitter is ifc_emit::BrepEmitter::emit_solid — the same
// machinery emit_solid_ifc wraps — so fragment output is byte-identical to the file writers' bodies.
//
// Returns (spf_text, next_id, body_item_id, rep_type):
//   spf_text     the entity lines, ids first_id..next_id-1 (children before parents)
//   next_id      the next free entity id (pass to a subsequent call to keep numbering contiguous)
//   body_item_id id of the top representation item (the IfcAdvancedBrep / IfcExtrudedAreaSolid / ...),
//                or 0 when the root is unrepresentable OR any face dropped — the caller must then
//                DISCARD spf_text and take its Python fallback (partial geometry is never a success:
//                "no geometry left behind")
//   rep_type     the matching IfcShapeRepresentation.RepresentationType ("AdvancedBrep" /
//                "SweptSolid" / "CSG" / "AdvancedSweptSolid"); "" when body_item_id is 0
// The blob must hold exactly one root (adapy serializes one solid_geom() per object); anything else
// raises. The geometry-body entities are schema-invariant across IFC4/IFC4X3, so no schema parameter.
static nb::tuple ngeom_to_ifc_body_spf_impl(nb::bytes blob, long first_id, double deflection, double angular_deg) {
    adacpp::ngeom::NgeomDoc doc = adacpp::ngeom::decode(reinterpret_cast<const uint8_t *>(blob.c_str()), blob.size());
    if (doc.roots.size() != 1)
        throw std::runtime_error("ngeom_to_ifc_body_spf: blob must hold exactly one root, got " +
                                 std::to_string(doc.roots.size()));
    // BrepEmitter::emit pre-increments, so seed at first_id-1 to number entities from first_id.
    adacpp::ifc_emit::BrepEmitter em(first_id - 1, nullptr, deflection, angular_deg);
    std::string buf, rep_type;
    long body = em.emit_solid(buf, doc.roots[0], rep_type);
    if (body && em.stats().faces_dropped > 0)
        body = 0; // strict: a partially-emitted shell is a failure, not a smaller success
    if (!body)
        return nb::make_tuple(nb::str(""), first_id, (long) 0, nb::str(""));
    return nb::make_tuple(nb::str(buf.c_str(), buf.size()), em.current_id() + 1, body,
                          nb::str(rep_type.c_str(), rep_type.size()));
}

// The shared stats dict for the two NGEOM-record writers (mirrors stream_step_to_step's audit).
static nb::dict ngeom_emit_stats_dict(const adacpp::ifc_emit::FileStats &fs) {
    nb::dict d;
    d["solids_in"] = fs.solids_in;
    d["solids_out"] = fs.solids_out;
    d["instances_out"] = fs.instances_out;
    d["solids_skipped"] = fs.products_skipped;
    d["unit_scale"] = fs.unit_scale;
    d["faces_in"] = fs.geom.faces_in;
    d["faces_out"] = fs.geom.faces_out;
    d["faces_dropped"] = fs.geom.faces_dropped;
    d["edges_analytic"] = fs.geom.edges_analytic;
    d["edges_polyline_approx"] = fs.geom.edges_polyline_approx;
    d["edges_degenerate"] = fs.geom.edges_degenerate;
    nb::dict reasons;
    for (const auto &[k, v] : fs.geom.drop_reasons)
        reasons[k.c_str()] = v;
    d["drop_reasons"] = reasons;
    return d;
}

void cad_module(nb::module_ &m) {
    // Kernel-agnostic mesh / color / group types live in cad — they're the
    // surface every backend (native OCCT, wasm OCCT, future CGAL) speaks.
    nb::enum_<MeshType>(m, "MeshType")
        .value("POINTS", MeshType::POINTS)
        .value("LINES", MeshType::LINES)
        .value("LINE_LOOP", MeshType::LINE_LOOP)
        .value("LINE_STRIP", MeshType::LINE_STRIP)
        .value("TRIANGLES", MeshType::TRIANGLES)
        .value("TRIANGLE_STRIP", MeshType::TRIANGLE_STRIP)
        .value("TRIANGLE_FAN", MeshType::TRIANGLE_FAN);

    nb::class_<Color>(m, "Color")
        .def_rw("r", &Color::r)
        .def_rw("g", &Color::g)
        .def_rw("b", &Color::b)
        .def_rw("a", &Color::a);

    nb::class_<GroupReference>(m, "GroupReference")
        .def_ro("node_id", &GroupReference::node_id)
        .def_ro("start", &GroupReference::start)
        .def_ro("length", &GroupReference::length)
        .def_ro("vstart", &GroupReference::vstart)
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
        .def_ro("id", &Mesh::id)
        .def_prop_ro("positions",
                     [](Mesh &self) {
                         return nb::ndarray<nb::numpy, const float, nb::ndim<1>>(
                             self.positions.data(), {self.positions.size()}, nb::find(self));
                     })
        .def_prop_ro("indices",
                     [](Mesh &self) {
                         return nb::ndarray<nb::numpy, const uint32_t, nb::ndim<1>>(
                             self.indices.data(), {self.indices.size()}, nb::find(self));
                     })
        .def_prop_ro("normals",
                     [](Mesh &self) {
                         return nb::ndarray<nb::numpy, const float, nb::ndim<1>>(self.normals.data(),
                                                                                 {self.normals.size()}, nb::find(self));
                     })
        .def_prop_ro("edges",
                     [](Mesh &self) {
                         return nb::ndarray<nb::numpy, const uint32_t, nb::ndim<1>>(
                             self.edges.data(), {self.edges.size()}, nb::find(self));
                     })
        .def(
            "spike_stats",
            [](Mesh &self, double aspect_min, double outlier_k) {
                // Crows-nest tessellation-spike scan (same detector as the viewer's client-side
                // meshStats.ts) — usable from Python for audits/CI without the browser.
                adacpp::geom::SpikeStats s =
                    adacpp::geom::mesh_spike_stats(self.positions, self.indices, aspect_min, outlier_k);
                nb::dict d;
                d["max_spike"] = s.max_spike;
                d["spike_tris"] = s.spike_tris;
                d["triangles"] = s.triangles;
                return d;
            },
            nb::arg("aspect_min") = 8.0, nb::arg("outlier_k") = 4.0,
            "Crows-nest distortion stats {max_spike, spike_tris, triangles} for this tessellated mesh.")
        .def_ro("mesh_type", &Mesh::mesh_type)
        .def_ro("color", &Mesh::color)
        .def_ro("groups", &Mesh::group_reference);

    // Free-function form: crows-nest distortion scan over raw (positions, indices) numpy arrays — for
    // audits/CI that hold geometry from any source, not only a tessellate_stream Mesh. Same detector
    // as Mesh.spike_stats and the viewer's client-side meshStats.ts.
    m.def(
        "mesh_spike_stats",
        [](nb::ndarray<const float, nb::ndim<1>> positions, nb::ndarray<const uint32_t, nb::ndim<1>> indices,
           double aspect_min, double outlier_k) {
            std::vector<float> p(positions.data(), positions.data() + positions.size());
            std::vector<uint32_t> ix(indices.data(), indices.data() + indices.size());
            adacpp::geom::SpikeStats s = adacpp::geom::mesh_spike_stats(p, ix, aspect_min, outlier_k);
            nb::dict d;
            d["max_spike"] = s.max_spike;
            d["spike_tris"] = s.spike_tris;
            d["triangles"] = s.triangles;
            return d;
        },
        nb::arg("positions"), nb::arg("indices"), nb::arg("aspect_min") = 8.0, nb::arg("outlier_k") = 4.0,
        "Crows-nest distortion stats {max_spike, spike_tris, triangles} for a flat (positions, indices) mesh.");

    nb::class_<StepRootMeta>(m, "StepRootMeta")
        .def_ro("id", &StepRootMeta::id)
        .def_ro("guid", &StepRootMeta::guid)
        .def_ro("has_color", &StepRootMeta::has_color)
        .def_ro("color", &StepRootMeta::color)
        .def_ro("transforms", &StepRootMeta::transforms)
        .def_ro("instance_paths", &StepRootMeta::instance_paths);

    // Inner classes first — ImprintResult exposes vectors of these.
    nb::class_<ImprintedEdge>(m, "ImprintedEdge")
        .def_ro("start", &ImprintedEdge::start)
        .def_ro("end", &ImprintedEdge::end);

    nb::class_<ImprintedFace>(m, "ImprintedFace")
        .def_ro("origin", &ImprintedFace::origin)
        .def_ro("normal", &ImprintedFace::normal)
        .def_ro("ref_direction", &ImprintedFace::ref_direction)
        .def_ro("loops", &ImprintedFace::loops);

    nb::class_<ImprintResult>(m, "ImprintResult")
        .def_ro("vertices", &ImprintResult::vertices)
        .def_ro("edges", &ImprintResult::edges)
        .def_ro("faces", &ImprintResult::faces)
        .def_ro("sources", &ImprintResult::sources)
        .def_ro("curve_sources", &ImprintResult::curve_sources)
        .def_ro("free_edges", &ImprintResult::free_edges);

    nb::class_<PcurveData>(m, "PcurveData")
        .def_ro("has_pcurve", &PcurveData::has_pcurve)
        .def_ro("degree", &PcurveData::degree)
        .def_ro("control_points", &PcurveData::control_points)
        .def_ro("knots", &PcurveData::knots)
        .def_ro("multiplicities", &PcurveData::multiplicities)
        .def_ro("weights", &PcurveData::weights)
        .def_ro("closed", &PcurveData::closed)
        .def_ro("start", &PcurveData::start)
        .def_ro("end", &PcurveData::end);

    nb::class_<AdvancedFaceData>(m, "AdvancedFaceData")
        .def_ro("u_degree", &AdvancedFaceData::u_degree)
        .def_ro("v_degree", &AdvancedFaceData::v_degree)
        .def_ro("poles", &AdvancedFaceData::poles)
        .def_ro("u_knots", &AdvancedFaceData::u_knots)
        .def_ro("v_knots", &AdvancedFaceData::v_knots)
        .def_ro("u_multiplicities", &AdvancedFaceData::u_multiplicities)
        .def_ro("v_multiplicities", &AdvancedFaceData::v_multiplicities)
        .def_ro("weights", &AdvancedFaceData::weights)
        .def_ro("u_closed", &AdvancedFaceData::u_closed)
        .def_ro("v_closed", &AdvancedFaceData::v_closed)
        .def_ro("bounds", &AdvancedFaceData::bounds);

    nb::class_<StepShapeData>(m, "StepShapeData")
        .def_ro("shape", &StepShapeData::shape)
        .def_ro("name", &StepShapeData::name)
        .def_ro("color", &StepShapeData::color)
        .def_ro("has_color", &StepShapeData::has_color);

    // Opaque handle: no readable attributes / methods. Callers obtain instances
    // via factory functions (make_box, ...) and pass them to consumers
    // (tessellate, ...). The C++-level shape data is unreachable from Python.
    nb::class_<ShapeHandle>(m, "ShapeHandle");

    m.def("make_box", &make_box_impl, "dx"_a, "dy"_a, "dz"_a, "Create a centered axis-aligned box ShapeHandle.");

    m.def("make_cylinder", &make_cylinder_impl, "radius"_a, "height"_a,
          "Create a cylinder ShapeHandle along +Z, base on the XY plane.");

    m.def("make_sphere", &make_sphere_impl, "radius"_a, "Create a sphere ShapeHandle centered at the origin.");

    m.def("tessellate", &tessellate_impl, "shape"_a, "linear_deflection"_a = -1.0,
          "Tessellate a shape into a triangle Mesh. "
          "linear_deflection<=0 selects a heuristic based on the shape's bbox.");

    m.def("tessellate_batch", &tessellate_batch_impl, "shapes"_a, "linear_deflection"_a = -1.0,
          "Tessellate many shapes into ONE combined Mesh in a single call, with a "
          "GroupReference per input shape (node_id=index, start/length=triangle-index "
          "range). Amortizes the per-call boundary cost and returns one zero-copy "
          "buffer. linear_deflection<=0 selects a per-shape bbox heuristic.");

    m.def("tessellate_stream", &tessellate_stream_impl, "buffer"_a, "pipeline"_a = "libtess2", "deflection"_a = 0.0,
          "angular_deg"_a = 20.0, "settings"_a = nb::dict(), "threads"_a = 1, "model_scale"_a = 0.0,
          "Decode an NGEOM stream buffer (adapy ada.geom, neutral schema) and tessellate "
          "every instance into ONE combined Mesh with a GroupReference per root "
          "(node_id = root index). pipeline: 'libtess2' (OCC-free) | 'occ' | 'cgal' | "
          "'hybrid' (ifcopenshell taxonomy kernels). angular_deg in degrees. settings: "
          "ifcopenshell ConversionSettings overrides for the taxonomy paths, e.g. "
          "{'no-wire-intersection-check': True, 'precision': 1e-3} — see ifc_taxonomy_settings().");

    // Phase 1 of the native streaming STEP->IFC writer (dap plan/v3/spec_native_streaming_ifc.md):
    // emit ONE solid's IFC4 advanced-B-rep SPF lines from ng:: (no ada.geom). Test/parity entry —
    // the full multi-threaded file writer is later phases.
    m.def(
        "step_emit_ifc_brep",
        [](const std::string &path, size_t index, long start_id) -> std::string {
            auto idx = adacpp::step::StreamIndex::from_file(path);
            adacpp::step::Resolver r(idx);
            r.build_metadata(idx.lists);
            const auto &roots = idx.lists.roots;
            if (index >= roots.size())
                return "";
            adacpp::ngeom::NgeomRoot root = r.resolve_root(roots[index]);
            std::string out;
            adacpp::ifc_emit::BrepEmitter em(start_id);
            em.emit_advanced_brep(out, root);
            return out;
        },
        "path"_a, "index"_a = 0, "start_id"_a = 100,
        "Emit one solid's IFC4 IfcAdvancedBrep SPF lines (string) from the native NGEOM reader. "
        "Phase 1 test entry for the native STEP->IFC writer; returns '' if the solid was skipped.");

    // Phase 2: full single-threaded STEP->IFC file. Returns a dict with the losslessness audit
    // (solids_in/out, faces_in/out/dropped, drop_reasons, edge approximation counts).
    m.def(
        "stream_step_to_ifc",
        [](const std::string &in_path, const std::string &out_path, const std::string &schema, double deflection,
           double angular_deg, long max_solids, int num_threads) -> nb::dict {
            adacpp::ifc_emit::FileStats fs =
                (num_threads == 1) ? write_ifc_file_impl(in_path, out_path, schema, deflection, angular_deg, max_solids)
                                   : write_ifc_file_parallel_impl(in_path, out_path, schema, deflection, angular_deg,
                                                                  num_threads, max_solids);
            nb::dict d;
            d["solids_in"] = fs.solids_in;
            d["unit_scale"] = fs.unit_scale;
            d["solids_out"] = fs.solids_out;
            d["faces_in"] = fs.geom.faces_in;
            d["faces_out"] = fs.geom.faces_out;
            d["faces_dropped"] = fs.geom.faces_dropped;
            d["edges_analytic"] = fs.geom.edges_analytic;
            d["edges_polyline_approx"] = fs.geom.edges_polyline_approx;
            d["edges_degenerate"] = fs.geom.edges_degenerate;
            nb::dict reasons;
            for (const auto &[k, v] : fs.geom.drop_reasons)
                reasons[k.c_str()] = v;
            d["drop_reasons"] = reasons;
            return d;
        },
        "in_path"_a, "out_path"_a, "schema"_a = "IFC4X3_ADD2", "deflection"_a = 2.0, "angular_deg"_a = 20.0,
        "max_solids"_a = 0, "num_threads"_a = 0,
        "Native STEP->IFC: write a full IFC file (header + spatial block + "
        "one IfcAdvancedBrep+proxy per solid). Returns the losslessness audit dict (solids_in/out, "
        "faces_in/out/dropped, drop_reasons). schema: 'IFC4X3_ADD2' | 'IFC4'.");

    m.def(
        "blobs_to_ifc",
        [](const std::vector<nb::bytes> &blobs, const std::vector<std::array<float, 4>> &colors,
           const std::vector<std::vector<std::array<float, 16>>> &transforms,
           const std::vector<std::vector<std::vector<std::pair<int, std::string>>>> &paths, const std::string &out_path,
           const std::string &schema, double unit_scale) -> nb::dict {
            adacpp::ifc_emit::FileStats fs =
                blobs_to_ifc_impl(blobs, colors, transforms, paths, out_path, schema, unit_scale);
            nb::dict d;
            d["solids_in"] = fs.solids_in;
            d["solids_out"] = fs.solids_out;
            d["unit_scale"] = fs.unit_scale;
            d["faces_in"] = fs.geom.faces_in;
            d["faces_out"] = fs.geom.faces_out;
            d["faces_dropped"] = fs.geom.faces_dropped;
            nb::dict reasons;
            for (const auto &[k, v] : fs.geom.drop_reasons)
                reasons[k.c_str()] = v;
            d["drop_reasons"] = reasons;
            return d;
        },
        "blobs"_a, "colors"_a, "transforms"_a, "paths"_a, "out_path"_a, "schema"_a = "IFC4X3_ADD2",
        "unit_scale"_a = 1.0,
        "Native IFC writer from NGEOM blobs (the lazy ShapeStore form) + parallel per-shape metadata: "
        "colors (rgba; alpha<0 => none), transforms (per-shape 4x4 world placements), paths (per-shape "
        "instance_paths). Decodes each blob, re-attaches the metadata, and emits IfcAdvancedBrep + "
        "IfcStyledItem + spatial tree — the fully-native Assembly.to_ifc backend. Returns the audit dict.");

    // NGEOM record streams -> STEP / IFC: the ada-object-model export fast path (Genie-XML ->
    // step/ifc without the per-entity Python writers). Records are consumed lazily from any Python
    // iterable, one blob decoded + emitted + released at a time (bounded memory).
    m.def(
        "stream_ngeom_to_step",
        [](nb::iterable records, const std::string &out_path, double unit_scale, double deflection,
           double angular_deg) -> nb::dict {
            return ngeom_emit_stats_dict(
                stream_ngeom_to_step_impl(records, out_path, unit_scale, deflection, angular_deg));
        },
        "records"_a, "out_path"_a, "unit_scale"_a = 1.0, "deflection"_a = 2.0, "angular_deg"_a = 20.0,
        "Native AP242 STEP writer from a stream of NGEOM records. Each record is a sequence "
        "(name, blob[, color_rgba[, transforms[, paths]]]): name (str|None) overrides the blob's root "
        "id; blob is one NGEOM buffer (adapy serialize_geometries); color_rgba is 4 floats (alpha<0 or "
        "None => no colour, else a STYLED_ITEM presentation colour); transforms is a list of 16-float "
        "column-major world placements (None/[] => one identity instance); paths is the per-instance "
        "assembly path ([(rep_id, name), ...] levels) emitted as a NEXT_ASSEMBLY_USAGE_OCCURRENCE "
        "tree. Multi-instance B-rep solids with rigid placements are written once as a shared "
        "prototype + per-placement NAUO/CDSR references (ADACPP_STEP_BAKE_INSTANCES=1 forces the "
        "all-baked form); everything else is baked per instance. Records may come from a generator — "
        "they are pulled lazily, so a 100k-solid model streams at bounded memory. unit_scale = metres "
        "per model unit (sets the header SI length unit). Returns the losslessness audit dict "
        "(solids_in/out, instances_out, solids_skipped, faces_in/out/dropped, drop_reasons).");

    m.def(
        "stream_ngeom_to_ifc",
        [](nb::iterable records, const std::string &out_path, const std::string &schema, double unit_scale,
           double deflection, double angular_deg) -> nb::dict {
            return ngeom_emit_stats_dict(
                stream_ngeom_to_ifc_impl(records, out_path, schema, unit_scale, deflection, angular_deg));
        },
        "records"_a, "out_path"_a, "schema"_a = "IFC4X3_ADD2", "unit_scale"_a = 1.0, "deflection"_a = 2.0,
        "angular_deg"_a = 20.0,
        "Native IFC writer from a stream of NGEOM records — the record-stream sibling of blobs_to_ifc "
        "(same record shape as stream_ngeom_to_step). Decodes each blob and emits the analytic IFC "
        "solid (IfcAdvancedBrep / IfcExtrudedAreaSolid / ...) + IfcStyledItem colour + the "
        "IfcSpatialZone tree from the record paths; multi-instance solids share geometry via "
        "IfcMappedItem. Records are pulled lazily (generator-friendly, bounded memory). schema: "
        "'IFC4X3_ADD2' | 'IFC4'. Returns the losslessness audit dict.");

    m.def(
        "ngeom_to_ifc_body_spf",
        [](nb::bytes blob, long first_id, double deflection, double angular_deg) -> nb::tuple {
            return ngeom_to_ifc_body_spf_impl(blob, first_id, deflection, angular_deg);
        },
        "blob"_a, "first_id"_a, "deflection"_a = 2.0, "angular_deg"_a = 20.0,
        "Emit ONE NGEOM blob's IFC geometry-body entity graph as an SPF text fragment (no "
        "header/product/spatial entities), numbering entities from first_id. Returns (spf_text, "
        "next_id, body_item_id, rep_type) where body_item_id is the top representation item (the "
        "IfcAdvancedBrep / analytic solid) and rep_type the matching "
        "IfcShapeRepresentation.RepresentationType. body_item_id == 0 (unrepresentable root or any "
        "dropped face) means: discard spf_text and fall back — a partial body is never returned. "
        "The blob must hold exactly one root. The fragment is schema-invariant (IFC4 / IFC4X3). "
        "Used by adapy's streaming IFC writer to keep TYPED products (IfcPlate/IfcBeam wrappers "
        "hand-authored in Python) while the heavy B-rep body graph is emitted at C++ speed.");

    // Native parallel STEP->STEP (AP242). Re-export the analytic B-rep per solid, instances baked,
    // round-trip-lossless. Returns the same losslessness audit dict.
    m.def(
        "stream_step_to_step",
        [](const std::string &in_path, const std::string &out_path, double deflection, double angular_deg,
           long max_solids, int num_threads) -> nb::dict {
            adacpp::ifc_emit::FileStats fs =
                write_step_file_impl(in_path, out_path, deflection, angular_deg, num_threads, max_solids);
            nb::dict d;
            d["solids_in"] = fs.solids_in;
            d["solids_out"] = fs.solids_out;
            d["instances_out"] = fs.instances_out;
            d["unit_scale"] = fs.unit_scale;
            d["faces_in"] = fs.geom.faces_in;
            d["faces_out"] = fs.geom.faces_out;
            d["faces_dropped"] = fs.geom.faces_dropped;
            d["edges_analytic"] = fs.geom.edges_analytic;
            d["edges_polyline_approx"] = fs.geom.edges_polyline_approx;
            d["edges_degenerate"] = fs.geom.edges_degenerate;
            nb::dict reasons;
            for (const auto &[k, v] : fs.geom.drop_reasons)
                reasons[k.c_str()] = v;
            d["drop_reasons"] = reasons;
            return d;
        },
        "in_path"_a, "out_path"_a, "deflection"_a = 2.0, "angular_deg"_a = 20.0, "max_solids"_a = 0,
        "num_threads"_a = 0,
        "Native parallel STEP->STEP (AP242): re-export each solid's analytic B-rep as a "
        "MANIFOLD_SOLID_BREP part, no OCC. Multi-instance B-rep solids with rigid placements are "
        "written once as a shared prototype + per-placement NAUO/CDSR references (AP242 assembly "
        "instancing); other solids are baked per instance. ADACPP_STEP_BAKE_INSTANCES=1 forces the "
        "all-baked form. Returns the losslessness audit dict (instances_out = placed occurrences).");

    // Cross-format parity in ONE parse (the fan-out of stream_step_to_ifc + stream_step_to_step):
    // resolve each root once, run both emitters, count what each format preserves — no file output.
    m.def(
        "step_parity",
        [](const std::string &in_path, double deflection, double angular_deg, long max_solids,
           int num_threads) -> nb::dict {
            long solids_in = 0, total_instances = 0;
            double unit_scale = 1.0;
            ParityFormat ifc_fmt, step_fmt;
            step_parity_impl(in_path, deflection, angular_deg, num_threads, max_solids, solids_in, total_instances,
                             unit_scale, ifc_fmt, step_fmt);
            auto fmt_dict = [](const ParityFormat &f) {
                nb::dict d;
                d["solids_out"] = f.solids_out;
                d["instances"] = f.instances;
                d["faces_in"] = f.geom.faces_in;
                d["faces_out"] = f.geom.faces_out;
                d["faces_dropped"] = f.geom.faces_dropped;
                nb::dict reasons;
                for (const auto &[k, v] : f.geom.drop_reasons)
                    reasons[k.c_str()] = v;
                d["drop_reasons"] = reasons;
                return d;
            };
            nb::dict d;
            d["solids_in"] = solids_in;
            d["total_instances"] = total_instances; // source placed-instance count == the parity baseline
            d["unit_scale"] = unit_scale;
            d["ifc"] = fmt_dict(ifc_fmt);
            d["step"] = fmt_dict(step_fmt);
            return d;
        },
        "in_path"_a, "deflection"_a = 2.0, "angular_deg"_a = 20.0, "max_solids"_a = 0, "num_threads"_a = 0,
        "Cross-format STEP parity in a single native parse: resolve every root once and run both the "
        "STEP->IFC and STEP->STEP emitters over it, returning {solids_in, unit_scale, ifc:{...}, step:{...}} "
        "where each format dict is {solids_out, instances, faces_in, faces_out, faces_dropped, drop_reasons}. "
        "No output file is written — the fan-out of stream_step_to_ifc + stream_step_to_step for the parity "
        "check, so the expensive per-solid resolve is shared and nothing is serialised to disk.");

    // Native IFC->STEP (AP242): read an IFC advanced-B-rep file -> ng:: -> STEP, no OCC.
    m.def(
        "stream_ifc_to_step",
        [](const std::string &in_path, const std::string &out_path, double deflection, double angular_deg,
           long max_solids) -> nb::dict {
            adacpp::ifc_emit::FileStats fs =
                write_ifc_to_step_impl(in_path, out_path, deflection, angular_deg, max_solids);
            nb::dict d;
            d["solids_in"] = fs.solids_in;
            d["solids_out"] = fs.solids_out;
            d["products_total"] = fs.products_total;
            d["products_skipped"] = fs.products_skipped;
            d["unit_scale"] = fs.unit_scale;
            d["faces_in"] = fs.geom.faces_in;
            d["faces_out"] = fs.geom.faces_out;
            d["faces_dropped"] = fs.geom.faces_dropped;
            d["edges_analytic"] = fs.geom.edges_analytic;
            d["edges_polyline_approx"] = fs.geom.edges_polyline_approx;
            d["edges_degenerate"] = fs.geom.edges_degenerate;
            nb::dict reasons;
            for (const auto &[k, v] : fs.geom.drop_reasons)
                reasons[k.c_str()] = v;
            d["drop_reasons"] = reasons;
            return d;
        },
        "in_path"_a, "out_path"_a, "deflection"_a = 2.0, "angular_deg"_a = 20.0, "max_solids"_a = 0,
        "Native IFC->STEP (AP242): read an IFC advanced-B-rep file (IfcAdvancedBrep + analytic "
        "surfaces/curves + IfcMappedItem instancing) and re-export as STEP MANIFOLD_SOLID_BREP parts, "
        "no OCC. Returns the losslessness audit dict.");

    m.def("stream_step_to_meshes", &stream_step_to_meshes_impl, "path"_a, "pipeline"_a = "libtess2",
          "deflection"_a = 0.0, "angular_deg"_a = 20.0,
          "Read a STEP file with the native C++ reader (no OCC, no Python) and tessellate every "
          "solid into ONE combined Mesh with a GroupReference per root (node_id = root index). The "
          "fully-native counterpart of tessellate_stream. pipeline: 'libtess2' is the only kernel "
          "wired for this path. angular_deg in degrees.");

    m.def("stream_step_to_glb", &stream_step_to_glb_impl, "in_path"_a, "out_path"_a, "deflection"_a = 0.0,
          "angular_deg"_a = 20.0, "num_threads"_a = 0, "meshopt"_a = true, "model_scale"_a = 0.0,
          "face_regions"_a = false, "pipeline"_a = "libtess2", "pin_boundary"_a = true,
          "Native STEP -> GLB file: stream the .stp with the native reader (offset index + per-statement "
          "pread, bounded memory), tessellate each solid across num_threads worker threads (0 = auto = "
          "hardware_concurrency clamped to the cgroup cpu quota), each owning a spill lane joined at the "
          "end, bake world transform(s) + "
          "colour, and write a merge-by-colour GLB matching the adapy viewer's structure. meshopt=true "
          "(default) bakes EXT_meshopt_compression inline (no Python re-pack). Returns the number of "
          "solids written (-1 on I/O error, and likewise on an unknown pipeline name). angular_deg in "
          "degrees. pipeline selects the tessellation track by name (see tess_tracks(); '' / 'libtess2' "
          "= default, 'cdt' = watertight); pin_boundary is a libtess2-track option. The threaded core "
          "has taken both since the track vocabulary landed — this binding simply never forwarded them, "
          "so every native conversion silently ran libtess2 no matter what the caller selected.");

    m.def("stream_ifc_to_glb", &stream_ifc_to_glb_impl, "in_path"_a, "out_path"_a, "deflection"_a = 0.0,
          "angular_deg"_a = 20.0, "meshopt"_a = true, "model_scale"_a = 0.0, "num_threads"_a = 0,
          "pipeline"_a = "libtess2", "face_regions"_a = false, "pin_boundary"_a = true,
          "Native IFC -> GLB file (no ifcopenshell, no OCC): IfcResolver resolves each product's "
          "geometry + presentation colour + spatial-structure path, tessellates, baked to "
          "metres into a merge-by-colour GLB matching the viewer. LPT-ordered across num_threads "
          "workers (0=cgroup-aware auto); curve-only bodies (alignment axes) skipped. Returns products "
          "written (-1 on error, and likewise on an unknown pipeline name). pipeline selects the "
          "tessellation track by name (see tess_tracks(); '' / 'libtess2' = default, 'cdt' = "
          "watertight); face_regions bakes per-face clickable regions into scenes[0].extras; "
          "pin_boundary is a libtess2-track option. Same three knobs as stream_step_to_glb and the "
          "same meanings — both feed the same neutral tessellator and the same GLB writer, so none "
          "of them was ever STEP-specific.");

    m.def("stream_step_to_mesh", &stream_step_to_mesh_impl, "in_path"_a, "out_path"_a, "fmt"_a, "deflection"_a = 2.0,
          "angular_deg"_a = 20.0, "num_threads"_a = 0, "model_scale"_a = 0.0,
          "Native STEP -> STL/OBJ file: the SAME native reader + parallel tessellation as "
          "stream_step_to_glb, but bakes each instance's world placement and streams triangles straight "
          "to a binary STL (fmt='stl') or Wavefront OBJ (fmt='obj'). Bounded memory; no Python round-trip. "
          "Returns the triangle count (-1 on I/O error). angular_deg in degrees.");

#ifndef __EMSCRIPTEN__
    m.def("glb_diff", &glb_diff_impl, "scene_path"_a, "ref_path"_a, "mode"_a = "nameThenCentroid", "tol"_a = 1e-3,
          "overlay_rgba"_a = 0xD50000FFu,
          "Diff two GLB FILES (paths, so big GLBs never copy through Python bytes): summarise each one "
          "model at a time (never both, geometry not retained), match by mode "
          "('byName'|'byGuid'|'byCentroid'|'byProperty'|'nameThenCentroid'), and return "
          "{ops:[(node_id,status)], removed:[node_id], added:[node_id], counts:{...}, overlay:bytes}. "
          "status 0=unchanged 1=added 2=removed 3=modified; overlay is a red GLB of the ref-only geometry. "
          "Handles EXT_meshopt_compression (the viewer GLB format).");
#endif // __EMSCRIPTEN__

    m.def("stream_step_to_glb_st", &stream_step_to_glb_st_impl, "in_path"_a, "out_path"_a, "deflection"_a = 2.0,
          "angular_deg"_a = 20.0, "meshopt"_a = false,
          "Single-threaded, mmap-free STEP -> GLB (the no-pyodide wasm/OPFS core, callable natively as a "
          "parity oracle): from_file_pread scan + serial resolve/tessellate/spill + merge, NO std::thread "
          "and NO mmap. Returns the triangle count (-1 on I/O error).");

    m.def("stream_step_to_ngeom", &stream_step_to_ngeom_impl, "path"_a,
          "Read a STEP file with the native C++ reader (no OCC, no Python) and return "
          "(ngeom_buffer, [StepRootMeta]): the resolved geometry re-encoded to ONE NGEOM buffer "
          "(decode with the adapy deserializer into ada.geom.Geometry) plus per-root metadata — "
          "id, colour, per-instance world transforms, and assembly paths — parallel to the "
          "buffer's roots. The geometry sibling of stream_step_to_meshes for the from_step "
          "hydrate path (ada.geom.Geometry + assembly part hierarchy).");

    nb::class_<StepNgeomStream>(m, "StepNgeomStream")
        .def(nb::init<const std::string &>(), "path"_a,
             "Streaming NGEOM emitter: iterate to get (one-solid NGEOM buffer, StepRootMeta) per solid, "
             "with bounded memory (offset index + one solid) — the from_step hydrate path's large-file "
             "counterpart to stream_step_to_ngeom (which full-parses).")
        .def("__iter__", [](nb::object self) { return self; })
        .def("__next__", &StepNgeomStream::next);

    nb::class_<IfcNgeomStream>(m, "IfcNgeomStream")
        .def(nb::init<const std::string &>(), "path"_a,
             "Streaming per-product IFC -> NGEOM (dep-free: no ifcopenshell, no OCC). Iterate to get "
             "(one-root NGEOM buffer, StepRootMeta) per product with bounded memory; meta.guid is the "
             "product GlobalId. Geometry is in FILE units — apply .unit_scale (metres per unit). "
             "Unrepresentable products (tessellated/mixed reps) are skipped and counted in "
             ".products_skipped. v1 carries no colour and no spatial hierarchy.")
        .def("__iter__", [](nb::object self) { return self; })
        .def("__next__", &IfcNgeomStream::next)
        .def_prop_ro("unit_scale", &IfcNgeomStream::unit_scale)
        .def_prop_ro("products_total", &IfcNgeomStream::products_total)
        .def_prop_ro("products_skipped", &IfcNgeomStream::products_skipped);

    m.def("_step_index_parity", &step_index_parity_impl, "path"_a,
          "Debug: build the STEP offset index via mmap scan and via the wasm-safe pread scan, returning "
          "a dict of per-field equality (ids/offs/roots/units/styled/absr/srr/cdsr/sdr).");

    m.def(
        "tess_tracks",
        []() {
            nb::list out;
            for (const auto &t : adacpp::ngeom::track_infos()) {
                nb::dict d;
                d["name"] = t.name;
                d["label"] = t.label;
                d["description"] = t.description;
                d["watertight"] = t.watertight;
                d["default"] = t.is_default;
                d["neutral"] = t.neutral;
                out.append(d);
            }
            return out;
        },
        "Enumerate the tessellation tracks THIS BUILD provides: a list of dicts with name, label, "
        "description, watertight, default, neutral. `name` is the token to pass back as the "
        "`pipeline` argument of stream_step_to_glb / tessellate_stream.\n\n"
        "This is the single source of truth for the track vocabulary: callers must NOT hardcode it. "
        "The list reflects what is actually compiled in (the taxonomy kernels are absent from the "
        "wasm build), so it answers 'what can I run here?', not 'what exists in principle'.\n\n"
        "`neutral` says whether the track meshes NEUTRAL surfaces — the OCC-free path fed by the "
        "native STEP/IFC readers and the NGEOM wire. The taxonomy tracks (occ/cgal/hybrid) are not "
        "neutral: on that path they silently mesh as if no track were selected rather than "
        "erroring, so a caller offering a track choice for a neutral path must filter on this.");

    m.def(
        "ifc_taxonomy_settings",
        []() {
            nb::list out;
            for (const auto &si : adacpp::ngeom::taxonomy_settings_info()) {
                nb::dict d;
                d["name"] = si.name;
                d["type"] = si.type;
                d["default"] = si.default_value;
                out.append(d);
            }
            return out;
        },
        "Enumerate the ifcopenshell taxonomy ConversionSettings (name, type, default) so "
        "Python / the frontend can render + override them via tessellate_stream(settings=...).");

    m.def("meshopt_simplify_mesh", &meshopt_simplify_mesh_impl, "positions"_a, "indices"_a, "threshold"_a = 0.75f,
          "target_error"_a = 0.0f,
          "merge cleanup: meshopt_simplify (border-locked) toward threshold*index_count "
          "within target_error, then drop degenerate triangles + compact. positions xyz-interleaved "
          "float, indices uint32. Returns (positions, indices). target_error 0.0 = lossless "
          "coplanar-triangle collapse.");

    m.def("meshopt_encode_vertex_buffer", &meshopt_encode_vertex_buffer_impl, "data"_a, "count"_a, "stride"_a,
          "EXT_meshopt_compression: encode a vertex buffer (count vertices x stride bytes).");
    m.def("meshopt_encode_index_sequence", &meshopt_encode_index_sequence_impl, "indices"_a, "count"_a,
          "vertex_count"_a,
          "EXT_meshopt_compression: encode an index sequence (count uint32 indices; order-preserving).");
    m.def("meshopt_decode_vertex_buffer", &meshopt_decode_vertex_buffer_impl, "data"_a, "count"_a, "stride"_a,
          "Decode a meshopt vertex buffer back to count x stride raw bytes.");
    m.def("meshopt_decode_index_sequence", &meshopt_decode_index_sequence_impl, "data"_a, "count"_a, "index_size"_a,
          "Decode a meshopt index sequence back to count indices of index_size (2/4) bytes.");

    m.def("tessellate_box", &tessellate_box_impl, "dx"_a, "dy"_a, "dz"_a,
          "Convenience: build a box and tessellate it in one call.");

    m.def("bbox", &bbox_impl, "shape"_a, "optimal"_a = true,
          "Axis-aligned bounding box of a shape, returned as "
          "(xmin, ymin, zmin, xmax, ymax, zmax). optimal=True gives a tight, "
          "BSpline/B-rep-aware box (AddOptimal); optimal=False is a fast loose "
          "box (Add) for rough-extent probes that don't need per-face refinement.");

    m.def("obb", &obb_impl, "shape"_a,
          "Oriented bounding box of a shape, returned as "
          "((cx, cy, cz), (hx, hy, hz)) — world-space barycenter and OBB "
          "half-sizes.");

    m.def("from_topods_pointer", &from_topods_pointer_impl, "ptr"_a,
          "Wrap an OCCT TopoDS_Shape addressed by a raw pointer (typically "
          "produced by `int(pyocc_shape.this)`) into a ShapeHandle.");

    m.def("read_step_bytes", &read_step_bytes_impl, "data"_a,
          "Parse a STEP file from a bytes buffer into a ShapeHandle.");

    m.def("read_step_shapes", &read_step_shapes_impl, "data"_a, "unit"_a = "M",
          "Read a STEP file (bytes) via OCAF into a list of StepShapeData (shape + "
          "label name + color), converting the file's length unit to `unit` (default M). "
          "Backs adapy's StepStore under the adacpp doc backend.");

    m.def("write_glb_bytes", &write_glb_bytes_impl, "shape"_a, "linear_deflection"_a = 0.1,
          "Tessellate a ShapeHandle and write a binary glTF (.glb) into "
          "a bytes buffer. linear_deflection<=0 falls back to 0.1. NOTE: a bare ShapeHandle "
          "carries no colour, so the GLB has NO materials — for STEP input use "
          "step_bytes_to_glb_bytes, which keeps the file's colours.");

    m.def("step_bytes_to_glb_bytes", &step_bytes_to_glb_bytes_impl, "data"_a, "linear_deflection"_a = 0.1,
          "angular_deg"_a = 20.0, "unit"_a = "M",
          "STEP (bytes) -> binary glTF (.glb) (bytes) in one pass via OCAF, keeping the file's "
          "names, assembly tree and colours (both solid-level and per-face) in the GLB's "
          "materials. This is the colour-preserving replacement for read_step_bytes + "
          "write_glb_bytes, which between them drop every colour (plain STEPControl_Reader has no "
          "XCAF layer; the writer builds a document with no ColorTool) and yield a materialless, "
          "uniformly grey model. Converts the file's length unit to `unit` (default M).");

    // --- shape-algebra verbs (mirror adapy's CadBackend) ---

    m.def("boolean", &boolean_impl, "op"_a, "a"_a, "b"_a,
          "CSG of two shapes. op is one of 'UNION', 'INTERSECTION', "
          "'DIFFERENCE' (a - b).");

    m.def("transform", &transform_impl, "shape"_a, "matrix"_a, "copy"_a = true,
          "Apply a 4x4 affine transform (top 3 rows, 12 row-major doubles) to "
          "a shape. copy mirrors BRepBuilderAPI_Transform's copy flag.");

    m.def("distance", &distance_impl, "a"_a, "b"_a, "Minimal distance between two shapes.");

    m.def("serialize", &serialize_impl, "shape"_a, "Serialize a shape to a BREP string (triangulation cleaned first).");

    m.def("is_valid", &is_valid_impl, "shape"_a, "Topological + geometric validity (BRepCheck_Analyzer).");

    m.def("area", &area_impl, "shape"_a, "Total surface area (GProp).");
    m.def("shape_type", &shape_type_impl, "shape"_a,
          "Topological kind: solid/shell/face/wire/edge/vertex/compound/compsolid.");
    m.def("face_surface_type", &face_surface_type_impl, "shape"_a,
          "Geometric surface kind of a face: plane/cylinder/cone/sphere/torus/bspline/...");

    m.def("volume", &volume_impl, "shape"_a, "Solid volume (BRepGProp::VolumeProperties).");

    m.def("faces", &faces_impl, "shape"_a, "List of face sub-shapes as ShapeHandles.");

    m.def("solids", &solids_impl, "shape"_a, "List of solid sub-shapes as ShapeHandles.");

    m.def("edges", &edges_impl, "shape"_a, "List of edge sub-shapes as ShapeHandles.");

    m.def("to_topods_pointer", &to_topods_pointer_impl, "shape"_a,
          "Address of the wrapped OCCT TopoDS_Shape (for ABI-compatible OCCT "
          "consumers like gmsh; mirrors pythonocc's int(shape.this)). Valid "
          "only while the ShapeHandle is alive.");

    m.def("vertex_points", &vertex_points_impl, "shape"_a, "List of unique vertex coordinates as (x, y, z) tuples.");

    m.def("face_plane", &face_plane_impl, "face"_a,
          "Planar face's (origin, normal) as ((x,y,z),(x,y,z)), or None if "
          "the face is not planar.");

    // --- placement-aware primitive builders (ada.geom.solids parity) ---

    m.def("build_box", &build_box_impl, "location"_a, "axis"_a, "ref_dir"_a, "dx"_a, "dy"_a, "dz"_a,
          "Box with a corner at `location`, edges along the Axis2Placement3D "
          "frame (axis=Z, ref_dir=X).");

    m.def("build_cylinder", &build_cylinder_impl, "location"_a, "axis"_a, "radius"_a, "height"_a,
          "Cylinder with base centre at `location`, along `axis`.");

    m.def("build_sphere", &build_sphere_impl, "center"_a, "radius"_a, "Sphere centred at `center`.");

    m.def("make_wire", &make_wire_impl, "points"_a,
          "Polyline wire through a list of 3D points (consecutive straight "
          "edges).");

    m.def("build_cone", &build_cone_impl, "location"_a, "axis"_a, "bottom_radius"_a, "height"_a,
          "Right circular cone (apex radius 0) with base at `location`.");

    m.def("build_extruded_area_solid", &build_extruded_area_solid_impl, "outer"_a, "inners"_a, "location"_a, "axis"_a,
          "ref_dir"_a, "depth"_a, "is_area"_a = true,
          "Extruded area solid (beams/plates/pipe-shells): an outer profile "
          "curve + inner void curves (each a list of edge records: "
          "line=[0,p1,p2], arc=[1,start,mid,end], circle=[2,centre,axis,r]), "
          "prism-extruded by `depth` and placed at the Axis2Placement3D frame. "
          "is_area=False sweeps the outer wire alone (open lateral surface).");

    m.def("build_extruded_area_solid_tapered", &build_extruded_area_solid_tapered_impl, "outer_start"_a, "outer_end"_a,
          "location"_a, "axis"_a, "ref_dir"_a, "depth"_a,
          "Tapered extruded area solid (tapered beams): loft (ThruSections) between "
          "the start outer profile and the end outer profile (placed +Z by `depth`), "
          "then placed at the Axis2Placement3D frame. Edge records as in "
          "build_extruded_area_solid; only the outer wires are lofted.");

    m.def("loft_profiles", &loft_profiles_impl, "profiles"_a, "ruled"_a = true, "solid"_a = true,
          "Loft a ruled (or smooth) solid/shell through >=2 closed polygon "
          "section profiles (each a list of 3D points) via ThruSections.");

    m.def("section_with_plane", &section_with_plane_impl, "shape"_a, "origin"_a, "normal"_a, "size"_a = 1000.0,
          "Boolean-intersect `shape` with a finite (2*size square) planar face "
          "at (origin, normal); returns the cross-section.");

    m.def("write_step", &write_step_impl, "shapes"_a, "names"_a, "colors"_a, "filename"_a, "unit"_a = "m",
          "schema"_a = "AP214",
          "Write shapes (with per-shape name + RGB color) to a STEP file with "
          "OCAF names/colors via adacpp's bundled OCCT (no pythonocc needed).");

    m.def("serialize_brep", &serialize_brep_impl, "shape"_a,
          "Serialize a shape to OCCT BRepTools_ShapeSet text (FormatNb 2) — the "
          "BREP string ifcopenshell.geom.serialise consumes for the IFC tessellation "
          "fallback (replaces pythonocc-only ifcopenshell.geom.occ_utils).");

    m.def("build_revolved_area_solid", &build_revolved_area_solid_impl, "outer"_a, "inners"_a, "location"_a, "axis"_a,
          "ref_dir"_a, "axis_location"_a, "axis_direction"_a, "angle_deg"_a, "is_area"_a = true,
          "Revolved area solid (pipe-shell elbows): build the profile (same "
          "edge-record encoding as build_extruded_area_solid), place it at the "
          "Axis2Placement3D frame, then revolve around the world axis "
          "(axis_location, axis_direction) by angle_deg degrees. is_area=False "
          "revolves the outer wire alone.");

    m.def("build_fixed_reference_swept_area_solid", &build_fixed_reference_swept_area_solid_impl, "directrix"_a,
          "profile_outer"_a, "location"_a,
          "Fixed-reference swept area solid (PrimSweep / pipe bends): sweep the "
          "profile outer wire along the directrix spine (both in the same "
          "edge-record encoding as build_extruded_area_solid) with "
          "MakePipeShell + round-corner transitions, make a solid, and "
          "translate to `location`.");

    m.def("build_swept_disk_solid", &build_swept_disk_solid_impl, "directrix"_a, "radius"_a, "inner_radius"_a = 0.0,
          "Swept-disk solid (IfcSweptDiskSolid — pipes/rods): sweep a circular "
          "(annular when inner_radius>0) disk along the directrix spine (edge "
          "records). The disk is placed at the spine start normal to its tangent "
          "and kept perpendicular by MakePipeShell.");

    m.def("make_halfspace", &make_halfspace_impl, "origin"_a, "normal"_a, "flip"_a,
          "Infinite half-space solid bounded by the plane (origin, normal); "
          "`flip` selects which side is solid. Used as a CSG cutter.");

    m.def("cut_surfaces", &cut_surfaces_impl, "solid"_a, "cutters"_a, "deflection"_a, "tol"_a,
          "Cut `solid` by each cutter in turn (BRepAlgoAPI_Cut with boolean "
          "history) and return, for every result face originating from a "
          "cutter, plain data: (surface_type, (nx,ny,nz), outer_edges, "
          "outer_polyline, inner_polylines). Curved edges are discretized to "
          "`deflection`; points within `tol` are de-duplicated.");

    m.def("build_bspline_surface_face", &build_bspline_surface_face_impl, "u_degree"_a, "v_degree"_a,
          "control_points"_a, "u_knots"_a, "v_knots"_a, "u_mults"_a, "v_mults"_a, "weights"_a, "tol"_a = 1e-6,
          "Trimmed face over a B-spline surface (knots; rational if weights given). "
          "control_points row-major [num_u][num_v]; weights empty => non-rational. "
          "Natural-UV MakeFace (PlateCurved / loft-derived surfaces).");

    m.def("build_advanced_face_bspline", &build_advanced_face_bspline_impl, "u_degree"_a, "v_degree"_a,
          "control_points"_a, "u_knots"_a, "v_knots"_a, "u_mults"_a, "v_mults"_a, "weights"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a B-spline surface. bounds[0] is the outer "
          "boundary, the rest are holes; each edge is a 3D edge record or a kind-6 2D "
          "pcurve record laid on the surface. Ports make_face_from_geom (SAT-pcurve path).");

    m.def("build_advanced_face_planar", &build_advanced_face_planar_impl, "loc"_a, "axis"_a, "ref_dir"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Plane inferred from the (planar) boundary wire "
          "(MakeFace OnlyPlane). bounds[0] outer, rest holes; 3D edge records. loc/axis/ref_dir "
          "are accepted for parity but unused. Ports make_closed_shell_from_geom's planar path.");

    m.def("build_advanced_face_cylindrical", &build_advanced_face_cylindrical_impl, "loc"_a, "axis"_a, "ref_dir"_a,
          "radius"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_CylindricalSurface. loc is the cylinder "
          "axis base, axis its direction, ref_dir the U reference, radius its radius. bounds[0] "
          "outer, rest holes; 3D edge records (LINE/CIRCLE). Ports make_closed_shell_from_geom's "
          "AdvancedFace(CylindricalSurface) path for tube/pipe walls.");

    m.def("build_advanced_face_conical", &build_advanced_face_conical_impl, "loc"_a, "axis"_a, "ref_dir"_a, "radius"_a,
          "semi_angle"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_ConicalSurface (radius at the axis base, "
          "semi_angle the half-angle). bounds[0] outer, rest holes; 3D edge records.");

    m.def("build_advanced_face_toroidal", &build_advanced_face_toroidal_impl, "loc"_a, "axis"_a, "ref_dir"_a,
          "major_radius"_a, "minor_radius"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_ToroidalSurface (pipe elbows). "
          "bounds[0] outer, rest holes; 3D edge records.");

    m.def("build_advanced_face_surface_of_revolution", &build_advanced_face_surface_of_revolution_impl, "axis_loc"_a,
          "axis_dir"_a, "generatrix"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_SurfaceOfRevolution: revolve the generatrix "
          "curve record (B-spline/line/circle) about (axis_loc, axis_dir). bounds[0] outer, rest holes.");

    m.def("build_advanced_face_surface_of_linear_extrusion", &build_advanced_face_surface_of_linear_extrusion_impl,
          "direction"_a, "swept"_a, "bounds"_a,
          "Bounds-trimmed AdvancedFace over a Geom_SurfaceOfLinearExtrusion: extrude the swept "
          "curve record along direction. bounds[0] outer, rest holes; 3D edge records.");

    m.def("face_to_advanced_face", &face_to_advanced_face_impl, "face"_a,
          "Decompose a B-spline face into AdvancedFaceData (surface poles/knots + per-wire "
          "ordered edge pcurves). Reverse of build_advanced_face_bspline; ports "
          "occ_face_to_ada_face so the SAT/STEP face round-trip runs on adacpp.");

    m.def("extrude_face_along_normal", &extrude_face_along_normal_impl, "face"_a, "thickness"_a,
          "Prism-extrude a face by `thickness` along its surface normal (PlateCurved "
          "thickness). Returns the bare face on thickness 0 / undefined normal / failure.");

    m.def("build_wire", &build_wire_impl, "edges"_a,
          "Build a wire from edge records (line/arc/circle/ellipse/bspline, full or "
          "parametrically trimmed). See edge_from_record for the record layout.");

    m.def("build_filled_face", &build_filled_face_impl, "edges"_a,
          "Wire-filled face (WireFilledFace): interpolate a smooth surface through "
          ">=3 boundary edges via BRepOffsetAPI_MakeFilling.");

    m.def("build_planar_face", &build_planar_face_impl, "outer"_a, "inners"_a, "location"_a, "axis"_a, "ref_dir"_a,
          "Curve-bounded planar face (shell representation): outer profile face "
          "minus inner void faces, placed at the Axis2Placement3D frame. Same "
          "edge-record encoding as build_extruded_area_solid.");

    m.def("build_face_based_surface_model", &build_face_based_surface_model_impl, "polygons"_a,
          "Fuse a list of polygon faces (each a closed loop of 3D points) into "
          "one shell.");

    // --- topology-kernel ops ---
    m.def("make_volumes_from_faces", &make_volumes_from_faces_impl, "faces"_a, "tolerance"_a = 1e-6,
          "Partition space into solids from a face soup (BOPAlgo_MakerVolume). "
          "Interior faces come out shared between the two cells they separate.");

    m.def("sew_faces", &sew_faces_impl, "faces"_a, "tolerance"_a = 1e-6,
          "Sew faces into one connected shell (BRepBuilderAPI_Sewing). For OPEN "
          "surface models (IfcShellBasedSurfaceModel / open shell) that don't "
          "bound a volume, where make_volumes_from_faces would yield nothing.");

    m.def("polygon_face", &polygon_face_impl, "points"_a,
          "Planar face from a closed polygon of >=3 points (auto-closed). Divider "
          "faces for make_volumes_from_faces; ports adapy OccBackend.polygon_face.");

    m.def("non_manifold_merge", &non_manifold_merge_impl, "shapes"_a, "tolerance"_a = 1e-6, "glue"_a = true,
          "Non-manifold fuse (BOPAlgo_Builder) keeping coincident faces shared "
          "between adjacent solids rather than dissolving the partition.");

    m.def("imprint_planar_faces", &imprint_planar_faces_impl, "outlines"_a,
          "imprint_curves"_a = std::vector<std::vector<std::array<double, 3>>>(), "tolerance"_a = 1e-6,
          "General-Fuse planar outlines against each other and against imprint_curves "
          "(e.g. beam axes), returning the merged topology as plain data: welded "
          "vertices, shared edges, per-face loops, and sources mapping each input "
          "outline to the faces it became. Ports adapy OccBackend.imprint_planar_faces.");

    m.def("merge_cells", &merge_cells_impl, "solids"_a, "tolerance"_a = 0.0,
          "Faithful port of topologic Topology::Merge over solids "
          "(BOPAlgo_CellsBuilder + per-operand AddToResult + MakeContainers): "
          "each input solid survives as a cell and every interface becomes one "
          "shared non-manifold face.");

    m.def("face_id", &face_id_impl, "face"_a,
          "Orientation-independent topological identity of a face (TShape "
          "pointer). Two cells referencing the same shared non-manifold face "
          "return the same id; distinct faces differ.");

    m.def("free_faces", &free_faces_impl, "solids"_a,
          "Faces owned by exactly one solid — the outer envelope (FACE→SOLID "
          "ancestor map over a compound of the cells).");

    m.def("point_in_solid", &point_in_solid_impl, "solid"_a, "point"_a, "tolerance"_a = 1e-6,
          "Classify a point against a solid (BRepClass3d_SolidClassifier). "
          "Returns TopAbs_State as int: IN=0, OUT=1, ON=2, UNKNOWN=3.");

    m.def("center_of_mass", &center_of_mass_impl, "shape"_a,
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
