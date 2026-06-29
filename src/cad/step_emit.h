// Native AP242 STEP (ISO-10303-21) advanced-B-rep emitter — the STEP-syntax sibling of ifc_emit.h.
// Emits one solid's MANIFOLD_SOLID_BREP SPF lines from the C++ neutral form (ng::NgeomRoot) into a
// string buffer, ids from a running counter. Same B-rep traversal as the IFC emitter; STEP entity
// syntax + structural differences ported from adapy's ap242_stream.py: native CONICAL_SURFACE (IFC
// has none), the rational B-spline complex-instance (AIM) form, '' name prefixes, and
// SURFACE_OF_*_EXTRUSION/REVOLUTION taking a bare swept CURVE (not a profile def). Dep-free
// (stdlib + ng:: headers); reuses the dialect-neutral ifc_real / compact_knots from ifc_emit.h.
#pragma once

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "../geom/neutral/ngeom_bspline.h"
#include "../geom/neutral/ngeom_curves.h"
#include "../geom/neutral/ngeom_surfaces.h"
#include "../geom/neutral/ngeom_topology.h"
#include "ifc_emit.h" // ifc_real, ifc_bool, compact_knots, EmitStats

namespace adacpp::step_emit {

using namespace adacpp::ngeom;
using adacpp::ifc_emit::compact_knots;
using adacpp::ifc_emit::EmitStats;
using adacpp::ifc_emit::ifc_bool; // STEP .T./.F. is identical
using adacpp::ifc_emit::ifc_real; // STEP real format is identical

class StepBrepEmitter {
  public:
    // `tf` = optional row-major 4x4 baked into every point/dir (instance world placement); null =
    // identity. STEP v1 bakes per-instance geometry (vs IFC's MAPPED_ITEM) — simplest lossless form.
    explicit StepBrepEmitter(long start_id, const double *tf = nullptr, double deflection = 2.0,
                             double angular_deg = 20.0)
        : nid_(start_id), tf_(tf), deflection_(deflection), angular_(angular_deg * PI / 180.0) {}

    long current_id() const {
        return nid_;
    }
    const EmitStats &stats() const {
        return stats_;
    }
    long emit_entity(std::string &out, const std::string &body) {
        return emit(out, body);
    }
    long alloc_id() {
        return ++nid_;
    }

    // Emit the root's faces as one CLOSED_SHELL -> MANIFOLD_SOLID_BREP; return the brep id (0 if no
    // emittable face). `name` is the SPF name of the brep.
    long emit_manifold_brep(std::string &out, const NgeomRoot &root, const std::string &name) {
        vcache_.clear();
        std::vector<long> face_ids;
        face_ids.reserve(root.faces.size());
        for (const auto &fc : root.faces) {
            long fid = face(out, *fc);
            if (fid)
                face_ids.push_back(fid);
        }
        if (face_ids.empty())
            return 0;
        long shell = emit(out, "CLOSED_SHELL(''," + refs(face_ids) + ")");
        return emit(out, "MANIFOLD_SOLID_BREP('" + name + "',#" + std::to_string(shell) + ")");
    }

  private:
    long nid_;
    const double *tf_;
    double deflection_, angular_;
    EmitStats stats_;
    std::unordered_map<long long, long> vcache_;

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
    static std::string p3(const Vec3 &p) {
        return "(" + ifc_real(p.x) + "," + ifc_real(p.y) + "," + ifc_real(p.z) + ")";
    }
    long pt(std::string &out, const Vec3 &p) {
        return emit(out, "CARTESIAN_POINT(''," + p3(tp(p)) + ")");
    }
    long dir(std::string &out, const Vec3 &v) {
        return emit(out, "DIRECTION(''," + p3(td(v)) + ")");
    }
    long vertex(std::string &out, const Vec3 &p) {
        long long key = pkey(tp(p));
        auto it = vcache_.find(key);
        if (it != vcache_.end())
            return it->second;
        long vid = emit(out, "VERTEX_POINT('',#" + std::to_string(pt(out, p)) + ")");
        vcache_[key] = vid;
        return vid;
    }
    long axis2(std::string &out, const Frame &f) {
        long loc = pt(out, f.o), ax = dir(out, f.z), rf = dir(out, f.x);
        return emit(out, "AXIS2_PLACEMENT_3D('',#" + std::to_string(loc) + ",#" + std::to_string(ax) + ",#" +
                             std::to_string(rf) + ")");
    }
    long axis1(std::string &out, const Vec3 &loc, const Vec3 &axis) {
        return emit(out, "AXIS1_PLACEMENT('',#" + std::to_string(pt(out, loc)) + ",#" +
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

    // -- curves --------------------------------------------------------------
    long bspline_curve(std::string &out, const BSplineCurve &c) {
        std::vector<long> cps;
        cps.reserve(c.ctrl.size());
        for (const auto &p : c.ctrl)
            cps.push_back(pt(out, p));
        std::vector<double> knots;
        std::vector<int> mults;
        compact_knots(c.U, knots, mults);
        std::string cpr = refs(cps);
        if (!c.weights.empty()) {
            // AIM rational complex-instance form (sub-records carry no '' name).
            std::string body = "BOUNDED_CURVE()B_SPLINE_CURVE(" + std::to_string(c.degree) + "," + cpr +
                               ",.UNSPECIFIED.," + ifc_bool(c.closed) + ",.F.)B_SPLINE_CURVE_WITH_KNOTS(" +
                               ilist(mults) + "," + rlist(knots) +
                               ",.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(" +
                               rlist(c.weights) + ")REPRESENTATION_ITEM('')";
            ++nid_;
            out += "#" + std::to_string(nid_) + "=(" + body + ");\n";
            return nid_;
        }
        return emit(out, "B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(c.degree) + "," + cpr +
                             ",.UNSPECIFIED.," + ifc_bool(c.closed) + ",.F.," + ilist(mults) + "," + rlist(knots) +
                             ",.UNSPECIFIED.)");
    }
    long curve(std::string &out, const Curve *g) {
        if (const auto *l = dynamic_cast<const LineCurve *>(g)) {
            long v = emit(out, "VECTOR('',#" + std::to_string(dir(out, l->dir)) + ",1.)");
            return emit(out, "LINE('',#" + std::to_string(pt(out, l->pnt)) + ",#" + std::to_string(v) + ")");
        }
        if (const auto *c = dynamic_cast<const CircleCurve *>(g))
            return emit(out, "CIRCLE('',#" + std::to_string(axis2(out, c->f)) + "," + ifc_real(c->r) + ")");
        if (const auto *e = dynamic_cast<const EllipseCurve *>(g))
            return emit(out, "ELLIPSE('',#" + std::to_string(axis2(out, e->f)) + "," + ifc_real(e->a1) + "," +
                                 ifc_real(e->a2) + ")");
        if (const auto *b = dynamic_cast<const BSplineCurve *>(g))
            return bspline_curve(out, *b);
        if (const auto *p = dynamic_cast<const PolylineCurve *>(g))
            return polyline(out, p->pts);
        return 0;
    }
    long polyline(std::string &out, const std::vector<Vec3> &pts) {
        std::vector<long> ids;
        ids.reserve(pts.size());
        for (const auto &p : pts)
            ids.push_back(pt(out, p));
        return emit(out, "POLYLINE(''," + refs(ids) + ")");
    }
    static bool analytic_curve(const Curve *g) {
        return !g || dynamic_cast<const LineCurve *>(g) || dynamic_cast<const CircleCurve *>(g) ||
               dynamic_cast<const EllipseCurve *>(g) || dynamic_cast<const BSplineCurve *>(g) ||
               dynamic_cast<const PolylineCurve *>(g);
    }
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
    long oriented_edge(std::string &out, const OrientedEdgeN &oe) {
        const Curve *g = oe.geometry.get();
        long ec;
        if (analytic_curve(g)) {
            long crv = g ? curve(out, g) : 0;
            if (!crv) {
                Vec3 d = oe.e_end - oe.e_start;
                double n = d.norm();
                Vec3 u = n ? Vec3{d.x / n, d.y / n, d.z / n} : Vec3{0, 0, 1};
                long v = emit(out, "VECTOR('',#" + std::to_string(dir(out, u)) + ",1.)");
                crv = emit(out, "LINE('',#" + std::to_string(pt(out, oe.e_start)) + ",#" + std::to_string(v) +
                                    ")");
            }
            long v0 = vertex(out, oe.e_start), v1 = vertex(out, oe.e_end);
            ec = emit(out, "EDGE_CURVE('',#" + std::to_string(v0) + ",#" + std::to_string(v1) + ",#" +
                               std::to_string(crv) + "," + ifc_bool(oe.same_sense) + ")");
            ++stats_.edges_analytic;
            return emit(out, "ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + "," + ifc_bool(oe.orientation) + ")");
        }
        std::vector<Vec3> pts = oe.discretize(deflection_, angular_);
        if (pts.size() < 2) {
            pts = {oe.start, oe.end};
            ++stats_.edges_degenerate;
        } else {
            ++stats_.edges_polyline_approx;
        }
        long poly = polyline(out, pts);
        long v0 = vertex(out, oe.start), v1 = vertex(out, oe.end);
        ec = emit(out, "EDGE_CURVE('',#" + std::to_string(v0) + ",#" + std::to_string(v1) + ",#" +
                           std::to_string(poly) + ",.T.)");
        return emit(out, "ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + ",.T.)");
    }
    long loop(std::string &out, const LoopN &lp) {
        if (lp.is_poly) {
            std::vector<long> pts;
            pts.reserve(lp.polygon.size());
            for (const auto &p : lp.polygon)
                pts.push_back(pt(out, p));
            if (pts.empty())
                return 0;
            return emit(out, "POLY_LOOP(''," + refs(pts) + ")");
        }
        std::vector<long> oe;
        oe.reserve(lp.edges.size());
        for (const auto &e : lp.edges)
            oe.push_back(oriented_edge(out, e));
        if (oe.empty())
            return 0;
        return emit(out, "EDGE_LOOP(''," + refs(oe) + ")");
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
        std::string deg = std::to_string(s.u_degree) + "," + std::to_string(s.v_degree);
        std::string flags = ".UNSPECIFIED.," + std::string(ifc_bool(s.u_closed)) + "," + ifc_bool(s.v_closed) + ",.F.";
        std::string knots = ilist(um) + "," + ilist(vm) + "," + rlist(uk) + "," + rlist(vk) + ",.UNSPECIFIED.";
        if (!s.weights.empty()) {
            std::string wgrid = "(";
            for (int iu = 0; iu < s.nu; ++iu) {
                if (iu)
                    wgrid += ",";
                std::vector<double> row(s.weights.begin() + iu * s.nv, s.weights.begin() + (iu + 1) * s.nv);
                wgrid += rlist(row);
            }
            wgrid += ")";
            std::string body = "BOUNDED_SURFACE()B_SPLINE_SURFACE(" + deg + "," + grid + "," + flags +
                               ")B_SPLINE_SURFACE_WITH_KNOTS(" + knots +
                               ")GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(" + wgrid +
                               ")REPRESENTATION_ITEM('')SURFACE()";
            ++nid_;
            out += "#" + std::to_string(nid_) + "=(" + body + ");\n";
            return nid_;
        }
        return emit(out, "B_SPLINE_SURFACE_WITH_KNOTS(''," + deg + "," + grid + "," + flags + "," + knots + ")");
    }
    long surface(std::string &out, const Surface *s) {
        if (const auto *bs = dynamic_cast<const BSplineSurface *>(s))
            return bspline_surface(out, *bs);
        if (const auto *pl = dynamic_cast<const PlaneSurface *>(s))
            return emit(out, "PLANE('',#" + std::to_string(axis2(out, pl->f)) + ")");
        if (const auto *cy = dynamic_cast<const CylinderSurface *>(s))
            return emit(out, "CYLINDRICAL_SURFACE('',#" + std::to_string(axis2(out, cy->f)) + "," +
                                 ifc_real(cy->r) + ")");
        if (const auto *co = dynamic_cast<const ConeSurface *>(s)) // STEP has a native conical surface
            return emit(out, "CONICAL_SURFACE('',#" + std::to_string(axis2(out, co->f)) + "," + ifc_real(co->r0) +
                                 "," + ifc_real(co->semi_angle) + ")");
        if (const auto *sp = dynamic_cast<const SphereSurface *>(s))
            return emit(out, "SPHERICAL_SURFACE('',#" + std::to_string(axis2(out, sp->f)) + "," + ifc_real(sp->r) +
                                 ")");
        if (const auto *to = dynamic_cast<const TorusSurface *>(s))
            return emit(out, "TOROIDAL_SURFACE('',#" + std::to_string(axis2(out, to->f)) + "," + ifc_real(to->R) +
                                 "," + ifc_real(to->r) + ")");
        if (const auto *ex = dynamic_cast<const LinearExtrusionSurface *>(s)) {
            long crv = curve_or_polyline(out, ex->profile.get());
            if (!crv)
                return 0;
            return emit(out, "SURFACE_OF_LINEAR_EXTRUSION('',#" + std::to_string(crv) + ",#" +
                                 std::to_string(dir(out, ex->dir)) + ")");
        }
        if (const auto *rv = dynamic_cast<const RevolutionSurface *>(s)) {
            long crv = curve_or_polyline(out, rv->profile.get());
            if (!crv)
                return 0;
            return emit(out, "SURFACE_OF_REVOLUTION('',#" + std::to_string(crv) + ",#" +
                                 std::to_string(axis1(out, rv->axis_loc, rv->axis_dir)) + ")");
        }
        return 0;
    }
    long face(std::string &out, const FaceSurfaceN &fc) {
        ++stats_.faces_in;
        long surf = fc.surface ? surface(out, fc.surface.get()) : 0;
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
            const char *kw = (i == 0) ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            bounds.push_back(emit(out, std::string(kw) + "('',#" + std::to_string(lp) + "," +
                                          ifc_bool(fc.bounds[i].orientation) + ")"));
        }
        if (bounds.empty()) {
            stats_.drop("face:no-bounds");
            return 0;
        }
        ++stats_.faces_out;
        return emit(out, "ADVANCED_FACE(''," + refs(bounds) + ",#" + std::to_string(surf) + "," +
                             ifc_bool(fc.same_sense) + ")");
    }
};

} // namespace adacpp::step_emit
