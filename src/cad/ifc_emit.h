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

class BrepEmitter {
  public:
    // `tf` = optional row-major 4x4 baked into every point/dir (instance world placement); null =
    // identity. For Phase 1 parity tests pass null (Python solid(transform=None)).
    explicit BrepEmitter(long start_id, const double *tf = nullptr) : nid_(start_id), tf_(tf) {}

    long current_id() const {
        return nid_;
    }

    // Emit the root's faces as one IfcClosedShell -> IfcAdvancedBrep. Returns the IfcAdvancedBrep id,
    // or 0 if any face used non-emittable geometry (solid skipped wholesale, matching the Python).
    long emit_advanced_brep(std::string &out, const NgeomRoot &root) {
        vcache_.clear();
        std::vector<long> face_ids;
        face_ids.reserve(root.faces.size());
        for (const auto &fc : root.faces) {
            long fid = face(out, *fc);
            if (!fid)
                return 0;
            face_ids.push_back(fid);
        }
        if (face_ids.empty())
            return 0;
        long shell = emit(out, "IfcClosedShell(" + refs(face_ids) + ")");
        return emit(out, "IfcAdvancedBrep(#" + std::to_string(shell) + ")");
    }

  private:
    long nid_;
    const double *tf_;
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
        return 0;
    }

    long edge_curve(std::string &out, const OrientedEdgeN &oe) {
        const Curve *g = oe.geometry.get();
        long crv = 0;
        if (g && !dynamic_cast<const LineCurve *>(g))
            crv = curve(out, g);
        if (!crv) {
            // straight (or null) edge: IfcLine through the raw endpoints
            Vec3 d = oe.e_end - oe.e_start;
            double n = d.norm();
            Vec3 u = n ? Vec3{d.x / n, d.y / n, d.z / n} : Vec3{0, 0, 1};
            crv = emit(out, "IfcLine(#" + std::to_string(pt(out, oe.e_start)) + ",#" +
                                std::to_string(vec(out, u)) + ")");
        }
        long v0 = vertex(out, oe.e_start), v1 = vertex(out, oe.e_end);
        return emit(out, "IfcEdgeCurve(#" + std::to_string(v0) + ",#" + std::to_string(v1) + ",#" +
                             std::to_string(crv) + "," + ifc_bool(oe.same_sense) + ")");
    }

    long oriented_edge(std::string &out, const OrientedEdgeN &oe) {
        long ec = edge_curve(out, oe);
        // EdgeStart/EdgeEnd are DERIVED in IfcOrientedEdge (from the referenced edge) -> SPF '*',
        // not '$' (ifcopenshell.validate flags '$' here as "Attribute is derived in subtype").
        return emit(out, "IfcOrientedEdge(*,*,#" + std::to_string(ec) + "," + ifc_bool(oe.orientation) + ")");
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

    long surface(std::string &out, const Surface *s) {
        if (const auto *bs = dynamic_cast<const BSplineSurface *>(s))
            return bspline_surface(out, *bs);
        if (const auto *pl = dynamic_cast<const PlaneSurface *>(s))
            return emit(out, "IfcPlane(#" + std::to_string(axis2(out, pl->f)) + ")");
        if (const auto *cy = dynamic_cast<const CylinderSurface *>(s))
            return emit(out, "IfcCylindricalSurface(#" + std::to_string(axis2(out, cy->f)) + "," +
                                 ifc_real(cy->r) + ")");
        if (const auto *co = dynamic_cast<const ConeSurface *>(s))
            return emit(out, "IfcConicalSurface(#" + std::to_string(axis2(out, co->f)) + "," +
                                 ifc_real(co->r0) + "," + ifc_real(co->semi_angle) + ")");
        if (const auto *sp = dynamic_cast<const SphereSurface *>(s))
            return emit(out, "IfcSphericalSurface(#" + std::to_string(axis2(out, sp->f)) + "," +
                                 ifc_real(sp->r) + ")");
        if (const auto *to = dynamic_cast<const TorusSurface *>(s))
            return emit(out, "IfcToroidalSurface(#" + std::to_string(axis2(out, to->f)) + "," +
                                 ifc_real(to->R) + "," + ifc_real(to->r) + ")");
        return 0;
    }

    long face(std::string &out, const FaceSurfaceN &fc) {
        long surf = surface(out, fc.surface.get());
        if (!surf)
            return 0;
        std::vector<long> bounds;
        for (size_t i = 0; i < fc.bounds.size(); ++i) {
            if (!fc.bounds[i].loop)
                return 0;
            long lp = loop(out, *fc.bounds[i].loop);
            if (!lp)
                return 0;
            const char *kw = (i == 0) ? "IfcFaceOuterBound" : "IfcFaceBound";
            bounds.push_back(emit(out, std::string(kw) + "(#" + std::to_string(lp) + "," +
                                          ifc_bool(fc.bounds[i].orientation) + ")"));
        }
        if (bounds.empty())
            return 0;
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

} // namespace adacpp::ifc_emit
