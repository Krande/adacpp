// Native IFC4 advanced-B-rep emitter (Phase 1 of the native streaming STEP->IFC writer;
// see dap plan/v3/spec_native_streaming_ifc.md). Emits the SPF lines for ONE solid's geometry
// from the C++ neutral form (ng::NgeomRoot) into a string buffer, allocating entity ids from a
// running counter starting at `start_id`. Children-before-parents, IFC4 positional schema order —
// a port of adapy's ada/cadit/step/write/stream_step_to_ifc.py `_IfcBrepEmitter`, but consuming
// ng:: (not ada.geom), so it is the SAME geometry the mesh/glb streamers already read.
//
// ng:: is tessellation-oriented and lossy for IFC metadata (flat knot vectors, dropped
// curve_form/self_intersect/knot_spec) — so this re-compacts U->(knots,mults) and defaults the
// dropped enums. Geometrically faithful (IfcOpenShell loads + tessellates from degree+ctrl+U+w),
// NOT byte-identical to the STEP-derived Python output. See the spec's "Phase 1 finding".
#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "../geom/neutral/ngeom_bspline.h"
#include "../geom/neutral/ngeom_curves.h"
#include "../geom/neutral/ngeom_surfaces.h"
#include "../geom/neutral/ngeom_topology.h"

namespace adacpp::ifc_emit {

using namespace adacpp::ngeom;

// IFC SPF real: %.12g, uppercase E exponent, always a decimal point (mirrors stream_step_to_ifc._r).
inline std::string ifc_real(double x) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.12g", x);
    std::string s(buf);
    auto epos = s.find_first_of("eE");
    if (epos != std::string::npos) {
        std::string mant = s.substr(0, epos), exp = s.substr(epos + 1);
        if (mant.find('.') == std::string::npos)
            mant += ".";
        return mant + "E" + exp;
    }
    if (s.find('.') == std::string::npos)
        s += ".";
    return s;
}

inline const char *ifc_bool(bool v) {
    return v ? ".T." : ".F.";
}

// Re-compact an expanded flat knot vector into IFC's (distinct knots, multiplicities).
inline void compact_knots(const std::vector<double> &U, std::vector<double> &knots, std::vector<int> &mults) {
    for (size_t i = 0; i < U.size();) {
        size_t j = i;
        while (j < U.size() && U[j] == U[i])
            ++j;
        knots.push_back(U[i]);
        mults.push_back((int) (j - i));
        i = j;
    }
}

// Coverage accounting so "no geometry left behind" is VISIBLE, never silent: every face that fails
// to emit, and every curve that had to be approximated (no analytic IFC entity), is counted with a
// reason. A clean run is faces_in == faces_out, faces_dropped == 0.
struct EmitStats {
    long faces_in = 0, faces_out = 0, faces_dropped = 0;
    long edges_analytic = 0, edges_polyline_approx = 0, edges_degenerate = 0;
    std::unordered_map<std::string, long> drop_reasons; // e.g. "surface:unknown", "loop:empty"
    void drop(const std::string &why) {
        ++faces_dropped;
        ++drop_reasons[why];
    }
};

class BrepEmitter {
  public:
    // `tf` = optional row-major 4x4 baked into every point/dir (instance world placement); null =
    // identity. deflection/angular drive the discretize->IfcPolyline fallback for curves with no
    // analytic IFC entity (hyperbola/parabola/composite/trimmed) — faithful to tolerance, never a
    // wrong straight chord.
    explicit BrepEmitter(long start_id, const double *tf = nullptr, double deflection = 2.0,
                         double angular_deg = 20.0)
        : nid_(start_id), tf_(tf), deflection_(deflection), angular_(angular_deg * PI / 180.0) {}

    long current_id() const {
        return nid_;
    }
    const EmitStats &stats() const {
        return stats_;
    }
    // Public emit for the file writer's wrapper entities (shape rep / proxy / containment).
    long emit_entity(std::string &out, const std::string &body) {
        return emit(out, body);
    }
    // Allocate a fresh entity id WITHOUT emitting (the spatial-tree builder writes its own SPF lines).
    long alloc_id() {
        return ++nid_;
    }

    // Emit the root's faces as one IfcClosedShell -> IfcAdvancedBrep. Returns the IfcAdvancedBrep id,
    // or 0 if any face used non-emittable geometry (solid skipped wholesale, matching the Python).
    long emit_advanced_brep(std::string &out, const NgeomRoot &root) {
        vcache_.clear();
        std::vector<long> face_ids;
        face_ids.reserve(root.faces.size());
        for (const auto &fc : root.faces) {
            long fid = face(out, *fc);
            if (fid)
                face_ids.push_back(fid); // a dropped face is counted in stats_ (no silent skip),
                                         // but never sinks the whole solid — keep every good face
        }
        if (face_ids.empty())
            return 0;
        long shell = emit(out, "IfcClosedShell(" + refs(face_ids) + ")");
        return emit(out, "IfcAdvancedBrep(#" + std::to_string(shell) + ")");
    }

  private:
    long nid_;
    const double *tf_;
    double deflection_, angular_;
    EmitStats stats_;
    std::unordered_map<long long, long> vcache_; // rounded-point key -> IfcVertexPoint id

    long emit(std::string &out, const std::string &body) {
        ++nid_;
        out += "#";
        out += std::to_string(nid_);
        out += "=";
        out += body;
        out += ";\n";
        return nid_;
    }

    Vec3 tp(const Vec3 &p) const {
        if (!tf_)
            return p;
        const double *m = tf_;
        return Vec3{m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
                    m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
    }
    Vec3 td(const Vec3 &v) const {
        if (!tf_)
            return v;
        const double *m = tf_;
        Vec3 r{m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z,
               m[8] * v.x + m[9] * v.y + m[10] * v.z};
        double n = r.norm();
        return n ? Vec3{r.x / n, r.y / n, r.z / n} : v;
    }

    std::string p3(const Vec3 &p) const {
        return "(" + ifc_real(p.x) + "," + ifc_real(p.y) + "," + ifc_real(p.z) + ")";
    }
    long pt(std::string &out, const Vec3 &p) {
        return emit(out, "IfcCartesianPoint(" + p3(tp(p)) + ")");
    }
    long dir(std::string &out, const Vec3 &v) {
        return emit(out, "IfcDirection(" + p3(td(v)) + ")");
    }
    long vec(std::string &out, const Vec3 &v) {
        return emit(out, "IfcVector(#" + std::to_string(dir(out, v)) + ",1.)");
    }
    long vertex(std::string &out, const Vec3 &p) {
        Vec3 w = tp(p);
        long long key = pkey(w);
        auto it = vcache_.find(key);
        if (it != vcache_.end())
            return it->second;
        // emit the point WITHOUT re-baking (already world); use a raw point emit
        ++nid_;
        long pid = nid_;
        // IfcCartesianPoint takes ONE attribute (the coordinate LIST) -> double parens:
        // outer = argument list, inner (from p3) = the coordinates. (already-world: no re-bake)
        out += "#" + std::to_string(pid) + "=IfcCartesianPoint(" + p3(w) + ");\n";
        long vid = emit(out, "IfcVertexPoint(#" + std::to_string(pid) + ")");
        vcache_[key] = vid;
        return vid;
    }
    long axis2(std::string &out, const Frame &f) {
        long loc = pt(out, f.o), ax = dir(out, f.z), rf = dir(out, f.x);
        return emit(out, "IfcAxis2Placement3D(#" + std::to_string(loc) + ",#" + std::to_string(ax) + ",#" +
                             std::to_string(rf) + ")");
    }
    long axis1(std::string &out, const Vec3 &loc, const Vec3 &axis) {
        return emit(out, "IfcAxis1Placement(#" + std::to_string(pt(out, loc)) + ",#" +
                             std::to_string(dir(out, axis)) + ")");
    }

    static std::string refs(const std::vector<long> &ids) {
        std::string s = "(";
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i)
                s += ",";
            s += "#" + std::to_string(ids[i]);
        }
        return s + ")";
    }
    static long long pkey(const Vec3 &p) {
        auto q = [](double v) { return (long long) std::llround(v * 1e6); };
        return (q(p.x) * 73856093LL) ^ (q(p.y) * 19349663LL) ^ (q(p.z) * 83492791LL);
    }

    // -- curves --------------------------------------------------------------
    long bspline_curve(std::string &out, const BSplineCurve &c) {
        std::vector<long> cps;
        cps.reserve(c.ctrl.size());
        for (const auto &p : c.ctrl)
            cps.push_back(pt(out, p));
        std::vector<double> knots;
        std::vector<int> mults;
        compact_knots(c.U, knots, mults);
        std::string common = std::to_string(c.degree) + "," + refs(cps) + ",.UNSPECIFIED.," +
                             ifc_bool(c.closed) + ",.F.," + ilist(mults) + "," + rlist(knots) + ",.UNSPECIFIED.";
        if (!c.weights.empty())
            return emit(out, "IfcRationalBSplineCurveWithKnots(" + common + "," + rlist(c.weights) + ")");
        return emit(out, "IfcBSplineCurveWithKnots(" + common + ")");
    }

    // A bare 3D geometric curve, or 0 if not representable here.
    long curve(std::string &out, const Curve *g) {
        if (const auto *l = dynamic_cast<const LineCurve *>(g))
            return emit(out, "IfcLine(#" + std::to_string(pt(out, l->pnt)) + ",#" +
                                 std::to_string(vec(out, l->dir)) + ")");
        if (const auto *c = dynamic_cast<const CircleCurve *>(g))
            return emit(out, "IfcCircle(#" + std::to_string(axis2(out, c->f)) + "," + ifc_real(c->r) + ")");
        if (const auto *e = dynamic_cast<const EllipseCurve *>(g))
            return emit(out, "IfcEllipse(#" + std::to_string(axis2(out, e->f)) + "," + ifc_real(e->a1) + "," +
                                 ifc_real(e->a2) + ")");
        if (const auto *b = dynamic_cast<const BSplineCurve *>(g))
            return bspline_curve(out, *b);
        if (const auto *p = dynamic_cast<const PolylineCurve *>(g))
            return polyline(out, p->pts);
        return 0; // no analytic IFC entity (hyperbola/parabola/composite/trimmed) -> caller discretizes
    }

    long polyline(std::string &out, const std::vector<Vec3> &pts) {
        std::vector<long> ids;
        ids.reserve(pts.size());
        for (const auto &p : pts)
            ids.push_back(pt(out, p));
        return emit(out, "IfcPolyline(" + refs(ids) + ")");
    }

    // True for curve types with a faithful analytic IFC entity (so no approximation needed).
    static bool analytic_curve(const Curve *g) {
        return !g || dynamic_cast<const LineCurve *>(g) || dynamic_cast<const CircleCurve *>(g) ||
               dynamic_cast<const EllipseCurve *>(g) || dynamic_cast<const BSplineCurve *>(g) ||
               dynamic_cast<const PolylineCurve *>(g);
    }

    long oriented_edge(std::string &out, const OrientedEdgeN &oe) {
        const Curve *g = oe.geometry.get();
        long ec;
        if (analytic_curve(g)) {
            // analytic edge: emit the basis curve in its raw EDGE_CURVE sense; orientation stays on
            // the IfcOrientedEdge. (null/Line -> IfcLine through the raw endpoints.)
            long crv = g ? curve(out, g) : 0;
            if (!crv) {
                Vec3 d = oe.e_end - oe.e_start;
                double n = d.norm();
                Vec3 u = n ? Vec3{d.x / n, d.y / n, d.z / n} : Vec3{0, 0, 1};
                crv = emit(out, "IfcLine(#" + std::to_string(pt(out, oe.e_start)) + ",#" +
                                    std::to_string(vec(out, u)) + ")");
            }
            long v0 = vertex(out, oe.e_start), v1 = vertex(out, oe.e_end);
            ec = emit(out, "IfcEdgeCurve(#" + std::to_string(v0) + ",#" + std::to_string(v1) + ",#" +
                               std::to_string(crv) + "," + ifc_bool(oe.same_sense) + ")");
            ++stats_.edges_analytic;
            return emit(out, "IfcOrientedEdge(*,*,#" + std::to_string(ec) + "," + ifc_bool(oe.orientation) + ")");
        }
        // No analytic IFC entity (hyperbola/parabola/composite/trimmed/...): discretize the edge to a
        // faithful IfcPolyline (tolerance-bounded) with orientation BAKED IN — never a wrong straight
        // chord, never silently dropped. NOTHING LEFT BEHIND. The oriented endpoints + .T./.T. then
        // match the baked traversal.
        std::vector<Vec3> pts = oe.discretize(deflection_, angular_);
        if (pts.size() < 2) {
            pts = {oe.start, oe.end};
            ++stats_.edges_degenerate;
        } else {
            ++stats_.edges_polyline_approx;
        }
        long poly = polyline(out, pts);
        long v0 = vertex(out, oe.start), v1 = vertex(out, oe.end);
        ec = emit(out, "IfcEdgeCurve(#" + std::to_string(v0) + ",#" + std::to_string(v1) + ",#" +
                           std::to_string(poly) + ",.T.)");
        return emit(out, "IfcOrientedEdge(*,*,#" + std::to_string(ec) + ",.T.)");
    }

    long loop(std::string &out, const LoopN &lp) {
        if (lp.is_poly) {
            std::vector<long> pts;
            pts.reserve(lp.polygon.size());
            for (const auto &p : lp.polygon)
                pts.push_back(pt(out, p));
            if (pts.empty())
                return 0;
            return emit(out, "IfcPolyLoop(" + refs(pts) + ")");
        }
        std::vector<long> oe;
        oe.reserve(lp.edges.size());
        for (const auto &e : lp.edges)
            oe.push_back(oriented_edge(out, e));
        if (oe.empty())
            return 0;
        return emit(out, "IfcEdgeLoop(" + refs(oe) + ")");
    }

    // -- surfaces ------------------------------------------------------------
    long bspline_surface(std::string &out, const BSplineSurface &s) {
        std::string grid = "(";
        for (int iu = 0; iu < s.nu; ++iu) {
            if (iu)
                grid += ",";
            std::vector<long> row;
            row.reserve(s.nv);
            for (int iv = 0; iv < s.nv; ++iv)
                row.push_back(pt(out, s.ctrl[iu * s.nv + iv]));
            grid += refs(row);
        }
        grid += ")";
        std::vector<double> uk, vk;
        std::vector<int> um, vm;
        compact_knots(s.Uu, uk, um);
        compact_knots(s.Uv, vk, vm);
        std::string common = std::to_string(s.u_degree) + "," + std::to_string(s.v_degree) + "," + grid +
                             ",.UNSPECIFIED.," + ifc_bool(s.u_closed) + "," + ifc_bool(s.v_closed) + ",.F.," +
                             ilist(um) + "," + ilist(vm) + "," + rlist(uk) + "," + rlist(vk) + ",.UNSPECIFIED.";
        if (!s.weights.empty()) {
            std::string wgrid = "(";
            for (int iu = 0; iu < s.nu; ++iu) {
                if (iu)
                    wgrid += ",";
                std::vector<double> row(s.weights.begin() + iu * s.nv, s.weights.begin() + (iu + 1) * s.nv);
                wgrid += rlist(row);
            }
            wgrid += ")";
            return emit(out, "IfcRationalBSplineSurfaceWithKnots(" + common + "," + wgrid + ")");
        }
        return emit(out, "IfcBSplineSurfaceWithKnots(" + common + ")");
    }

    // A bare profile/basis curve as an IfcCurve: analytic when possible, else sampled to IfcPolyline
    // over its natural range (faithful, never dropped).
    long curve_or_polyline(std::string &out, const Curve *g) {
        if (analytic_curve(g)) {
            long c = g ? curve(out, g) : 0;
            if (c)
                return c;
        }
        if (!g)
            return 0;
        double lo, hi, period;
        bool periodic;
        g->range(lo, hi, periodic, period);
        int n = std::max(2, g->discretize_spans(lo, hi));
        std::vector<Vec3> pts;
        pts.reserve(n + 1);
        for (int i = 0; i <= n; ++i)
            pts.push_back(g->point(lo + (hi - lo) * i / n));
        return polyline(out, pts);
    }

    long surface(std::string &out, const Surface *s, const std::vector<Vec3> &fpts = {}) {
        if (const auto *bs = dynamic_cast<const BSplineSurface *>(s))
            return bspline_surface(out, *bs);
        if (const auto *ex = dynamic_cast<const LinearExtrusionSurface *>(s)) {
            long crv = curve_or_polyline(out, ex->profile.get());
            if (!crv)
                return 0;
            long prof = emit(out, "IfcArbitraryOpenProfileDef(.CURVE.,$,#" + std::to_string(crv) + ")");
            long ed = dir(out, ex->dir);
            return emit(out, "IfcSurfaceOfLinearExtrusion(#" + std::to_string(prof) + ",$,#" +
                                 std::to_string(ed) + "," + ifc_real(ex->depth ? ex->depth : 1.0) + ")");
        }
        if (const auto *rv = dynamic_cast<const RevolutionSurface *>(s)) {
            long crv = curve_or_polyline(out, rv->profile.get());
            if (!crv)
                return 0;
            long prof = emit(out, "IfcArbitraryOpenProfileDef(.CURVE.,$,#" + std::to_string(crv) + ")");
            long ax = axis1(out, rv->axis_loc, rv->axis_dir);
            return emit(out, "IfcSurfaceOfRevolution(#" + std::to_string(prof) + ",$,#" +
                                 std::to_string(ax) + ")");
        }
        if (const auto *pl = dynamic_cast<const PlaneSurface *>(s))
            return emit(out, "IfcPlane(#" + std::to_string(axis2(out, pl->f)) + ")");
        if (const auto *cy = dynamic_cast<const CylinderSurface *>(s))
            return emit(out, "IfcCylindricalSurface(#" + std::to_string(axis2(out, cy->f)) + "," +
                                 ifc_real(cy->r) + ")");
        if (const auto *co = dynamic_cast<const ConeSurface *>(s)) {
            // IFC4/IFC4X3 have NO conical-surface entity -> represent the cone losslessly as an
            // IfcSurfaceOfRevolution of its straight slant generator about the cone axis. The
            // generator must be a BOUNDED curve (IfcArbitraryOpenProfileDef.Curve is IfcBoundedCurve),
            // and the ng:: cone is unbounded — so size the slant segment to the face's axial extent
            // (project the face vertices onto the axis), with a margin. Generator local: along v,
            // radius r0 + v*tan(semi_angle), point (rr,0,v); axis = (f.o, f.z).
            double ta = std::tan(co->semi_angle);
            double vmin = -1.0, vmax = 1.0;
            if (!fpts.empty()) {
                vmin = 1e300;
                vmax = -1e300;
                for (const Vec3 &p : fpts) {
                    double v = co->f.z.dot(p - co->f.o); // axial coordinate
                    vmin = std::min(vmin, v);
                    vmax = std::max(vmax, v);
                }
                double pad = 0.05 * (vmax - vmin) + 1e-6;
                vmin -= pad;
                vmax += pad;
            }
            Vec3 a = co->f.to_world(co->r0 + vmin * ta, 0.0, vmin);
            Vec3 b = co->f.to_world(co->r0 + vmax * ta, 0.0, vmax);
            long gen = polyline(out, {a, b}); // 2-pt bounded slant segment
            long prof = emit(out, "IfcArbitraryOpenProfileDef(.CURVE.,$,#" + std::to_string(gen) + ")");
            long ax = axis1(out, co->f.o, co->f.z);
            return emit(out, "IfcSurfaceOfRevolution(#" + std::to_string(prof) + ",$,#" +
                                 std::to_string(ax) + ")");
        }
        if (const auto *sp = dynamic_cast<const SphereSurface *>(s))
            return emit(out, "IfcSphericalSurface(#" + std::to_string(axis2(out, sp->f)) + "," +
                                 ifc_real(sp->r) + ")");
        if (const auto *to = dynamic_cast<const TorusSurface *>(s))
            return emit(out, "IfcToroidalSurface(#" + std::to_string(axis2(out, to->f)) + "," +
                                 ifc_real(to->R) + "," + ifc_real(to->r) + ")");
        return 0;
    }

    // Gather a face's boundary vertices (world, pre-instance-transform) — used to size unbounded
    // surfaces' generators (the cone-of-revolution) to the actual face extent.
    static void gather_loop_pts(const LoopN &lp, std::vector<Vec3> &out) {
        if (lp.is_poly) {
            out.insert(out.end(), lp.polygon.begin(), lp.polygon.end());
        } else {
            for (const auto &e : lp.edges) {
                out.push_back(e.e_start);
                out.push_back(e.e_end);
            }
        }
    }

    long face(std::string &out, const FaceSurfaceN &fc) {
        ++stats_.faces_in;
        std::vector<Vec3> fpts;
        for (const auto &b : fc.bounds)
            if (b.loop)
                gather_loop_pts(*b.loop, fpts);
        long surf = fc.surface ? surface(out, fc.surface.get(), fpts) : 0;
        if (!surf) {
            stats_.drop(fc.surface ? "surface:unrepresentable" : "surface:null");
            return 0;
        }
        std::vector<long> bounds;
        for (size_t i = 0; i < fc.bounds.size(); ++i) {
            if (!fc.bounds[i].loop) {
                stats_.drop("loop:null");
                return 0;
            }
            long lp = loop(out, *fc.bounds[i].loop);
            if (!lp) {
                stats_.drop("loop:empty");
                return 0;
            }
            const char *kw = (i == 0) ? "IfcFaceOuterBound" : "IfcFaceBound";
            bounds.push_back(emit(out, std::string(kw) + "(#" + std::to_string(lp) + "," +
                                          ifc_bool(fc.bounds[i].orientation) + ")"));
        }
        if (bounds.empty()) {
            stats_.drop("face:no-bounds");
            return 0;
        }
        ++stats_.faces_out;
        return emit(out, "IfcAdvancedFace(" + refs(bounds) + ",#" + std::to_string(surf) + "," +
                             ifc_bool(fc.same_sense) + ")");
    }

    static std::string ilist(const std::vector<int> &v) {
        std::string s = "(";
        for (size_t i = 0; i < v.size(); ++i) {
            if (i)
                s += ",";
            s += std::to_string(v[i]);
        }
        return s + ")";
    }
    static std::string rlist(const std::vector<double> &v) {
        std::string s = "(";
        for (size_t i = 0; i < v.size(); ++i) {
            if (i)
                s += ",";
            s += ifc_real(v[i]);
        }
        return s + ")";
    }
};

// A valid 22-char IfcGloballyUniqueId (IFC base64 alphabet), deterministic from n. Distinct per n.
inline std::string ifc_guid(uint64_t n) {
    static const char *B = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_$";
    uint64_t x = n * 0x9E3779B97F4A7C15ull + 0xD1B54A32D192ED03ull;
    std::string s;
    s.reserve(22);
    for (int i = 0; i < 22; ++i) {
        x = x * 6364136223846793005ull + 1442695040888963407ull;
        // 22 base64 chars encode a 128-bit number: the FIRST char carries only 2 bits (must be 0..3),
        // the remaining 21 carry 6 bits each (21*6 + 2 = 128).
        s += B[(x >> 58) & (i == 0 ? 3u : 63u)];
    }
    return s;
}

// Escape an IFC SPF single-quoted string (only ' needs doubling; keep it simple/ASCII).
inline std::string ifc_str(const std::string &s) {
    std::string o;
    o.reserve(s.size());
    for (char c : s)
        o += (c == '\'') ? std::string("''") : std::string(1, c);
    return o;
}

struct FileStats {
    long solids_in = 0, solids_out = 0;
    double unit_scale = 1.0; // metres per file length-unit (declared in the IFC header)
    EmitStats geom;
};

} // namespace adacpp::ifc_emit
