// NGEOM buffer decoder: uint8 stream -> neutral records (spec §2-§6). Header-only.
//
// Single-pass: records appear in dependency order so `ref` indices always point at an
// already-decoded earlier record. Every record is framed [tag:i32][nbytes:i32][payload];
// after decoding a record we advance the cursor by nbytes, so unknown tags are skipped and
// minor parse drift cannot desync the stream (spec §2/§7).
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "ngeom_bspline.h"
#include "ngeom_topology.h"

namespace adacpp::ngeom {

namespace tag {
enum : int {
    PLACEMENT3 = 1,
    PLACEMENT1 = 2,
    LINE = 10,
    POLYLINE = 11,
    HYPERBOLA = 12,
    PARABOLA = 13,
    COMPOSITE_CURVE = 14,
    CIRCLE = 20,
    ELLIPSE = 21,
    BSPLINE_CURVE = 22,
    TRIMMED_CURVE = 24,
    PCURVE2D = 25,
    PLANE = 40,
    CYLINDER = 41,
    CONE = 42,
    SPHERE = 43,
    TORUS = 44,
    BSPLINE_SURFACE = 45,
    SURF_LIN_EXTRUSION = 46,
    SURF_REVOLUTION = 47,
    EXTRUDED_AREA_SOLID = 50,
    REVOLVED_AREA_SOLID = 51,
    BOOLEAN_RESULT = 52,
    SPHERE_SOLID = 53,
    EDGE_CURVE = 60,
    ORIENTED_EDGE = 61,
    EDGE_LOOP = 62,
    POLY_LOOP = 63,
    FACE_BOUND = 64,
    FACE_SURFACE = 65,
    CONNECTED_FACE_SET = 66,
};
}

constexpr uint32_t NGEOM_VERSION = 1;

class Reader {
public:
    Reader(const uint8_t *data, size_t n) : p_(data), end_(data + n) {}
    int32_t i32() {
        int32_t v = 0;
        need(4);
        std::memcpy(&v, p_, 4);
        p_ += 4;
        return v;
    }
    double f64() {
        double v = 0;
        need(8);
        std::memcpy(&v, p_, 8);
        p_ += 8;
        return v;
    }
    Vec3 vec3() {
        double x = f64(), y = f64(), z = f64();
        return {x, y, z};
    }
    std::vector<double> f64s(int n) {
        std::vector<double> v(n);
        for (int i = 0; i < n; ++i)
            v[i] = f64();
        return v;
    }
    std::vector<int> i32s(int n) {
        std::vector<int> v(n);
        for (int i = 0; i < n; ++i)
            v[i] = i32();
        return v;
    }
    std::string str(int n) {
        need(n);
        std::string s((const char *) p_, n);
        p_ += n;
        return s;
    }
    const uint8_t *cursor() const {
        return p_;
    }
    void seek(const uint8_t *to) {
        p_ = to;
    }
    void skip(size_t n) {
        need(n);
        p_ += n;
    }

private:
    void need(size_t n) {
        if (p_ + n > end_)
            throw std::runtime_error("NGEOM: truncated buffer");
    }
    const uint8_t *p_;
    const uint8_t *end_;
};

namespace decode_detail {

struct Slot {
    int tag = 0;
    Frame frame; // PLACEMENT3 / PLACEMENT1 (z = axis)
    std::shared_ptr<Curve> curve;
    std::shared_ptr<Surface> surface;
    std::shared_ptr<LoopN> loop;
    std::shared_ptr<FaceBoundN> bound;
    std::shared_ptr<FaceSurfaceN> face;
    std::shared_ptr<ConnectedFaceSetN> cfs;
    std::shared_ptr<ExtrusionN> extrusion;
    std::shared_ptr<RevolveN> revolve;
    std::shared_ptr<BooleanN> boolean;
    std::shared_ptr<SphereN> sphere;
    std::shared_ptr<OrientedEdgeN> oedge;
    // EDGE_CURVE intermediate
    Vec3 e_start, e_end;
    std::shared_ptr<Curve> e_geom;
    bool e_same_sense = true;
    bool is_edge = false;
};

} // namespace decode_detail

// Decode an NGEOM buffer into neutral records + roots. Throws std::runtime_error on a
// malformed header / truncated buffer.
inline NgeomDoc decode(const uint8_t *data, size_t n) {
    using namespace decode_detail;
    Reader r(data, n);
    if (r.str(8) != "ADANGEOM")
        throw std::runtime_error("NGEOM: bad magic");
    uint32_t version = (uint32_t) r.i32();
    if (version > NGEOM_VERSION)
        throw std::runtime_error("NGEOM: unsupported version");
    int record_count = r.i32();
    std::vector<Slot> S(record_count);

    auto frame_at = [&](int idx) -> Frame { return S.at(idx).frame; };
    auto curve_at = [&](int idx) -> std::shared_ptr<Curve> { return S.at(idx).curve; };
    // Resolve a boolean operand record into a SolidItemN (any supported solid).
    auto solid_item_at = [&](int idx) -> SolidItemN {
        const Slot &s = S.at(idx);
        SolidItemN it;
        if (s.extrusion)
            it.extrusion = s.extrusion;
        else if (s.revolve)
            it.revolve = s.revolve;
        else if (s.boolean)
            it.boolean = s.boolean;
        else if (s.cfs)
            it.faces = s.cfs->faces;
        else if (s.face)
            it.faces = {s.face};
        return it;
    };
    auto surface_at = [&](int idx) -> std::shared_ptr<Surface> { return S.at(idx).surface; };

    for (int ri = 0; ri < record_count; ++ri) {
        int t = r.i32();
        int nbytes = r.i32();
        const uint8_t *payload = r.cursor();
        Slot &slot = S[ri];
        slot.tag = t;
        switch (t) {
        case tag::PLACEMENT3: {
            Vec3 loc = r.vec3(), axis = r.vec3(), ref = r.vec3();
            slot.frame = Frame::from_axis_ref(loc, axis, ref);
            break;
        }
        case tag::PLACEMENT1: {
            Vec3 loc = r.vec3(), axis = r.vec3();
            slot.frame = Frame::from_axis_ref(loc, axis, Vec3{1, 0, 0});
            break;
        }
        case tag::LINE: {
            Vec3 pnt = r.vec3(), dir = r.vec3();
            slot.curve = std::make_shared<LineCurve>(pnt, dir);
            break;
        }
        case tag::POLYLINE: {
            int np = r.i32();
            std::vector<Vec3> pts(np);
            for (int i = 0; i < np; ++i)
                pts[i] = r.vec3();
            slot.curve = std::make_shared<PolylineCurve>(std::move(pts));
            break;
        }
        case tag::HYPERBOLA: {
            int pl = r.i32();
            double sa = r.f64(), si = r.f64();
            slot.curve = std::make_shared<HyperbolaCurve>(frame_at(pl), sa, si);
            break;
        }
        case tag::PARABOLA: {
            int pl = r.i32();
            double fd = r.f64();
            slot.curve = std::make_shared<ParabolaCurve>(frame_at(pl), fd);
            break;
        }
        case tag::COMPOSITE_CURVE: {
            int ns = r.i32();
            std::vector<CompositeCurveN::Seg> segs(ns);
            for (int i = 0; i < ns; ++i) {
                int cref = r.i32();
                int ss = r.i32();
                segs[i] = {curve_at(cref), ss != 0};
            }
            slot.curve = std::make_shared<CompositeCurveN>(std::move(segs));
            break;
        }
        case tag::CIRCLE: {
            int pl = r.i32();
            double rad = r.f64();
            slot.curve = std::make_shared<CircleCurve>(frame_at(pl), rad);
            break;
        }
        case tag::ELLIPSE: {
            int pl = r.i32();
            double a1 = r.f64(), a2 = r.f64();
            slot.curve = std::make_shared<EllipseCurve>(frame_at(pl), a1, a2);
            break;
        }
        case tag::BSPLINE_CURVE: {
            int deg = r.i32();
            r.i32(); // closed
            r.i32(); // self_intersect
            int nc = r.i32();
            std::vector<Vec3> cp(nc);
            for (int i = 0; i < nc; ++i)
                cp[i] = r.vec3();
            int nk = r.i32();
            std::vector<double> kn = r.f64s(nk);
            std::vector<int> mu = r.i32s(nk);
            int has_w = r.i32();
            std::vector<double> w = has_w ? r.f64s(nc) : std::vector<double>{};
            bool closed = false; // captured above but unused for eval
            slot.curve =
                std::make_shared<BSplineCurve>(deg, std::move(cp), std::move(kn), std::move(mu), std::move(w), closed);
            break;
        }
        case tag::TRIMMED_CURVE: {
            int basis = r.i32();
            double t1 = r.f64(), t2 = r.f64();
            int sense = r.i32();
            r.i32(); // master representation
            slot.curve = std::make_shared<TrimmedCurve>(curve_at(basis), t1, t2, sense != 0);
            break;
        }
        case tag::PLANE: {
            slot.surface = std::make_shared<PlaneSurface>(frame_at(r.i32()));
            break;
        }
        case tag::CYLINDER: {
            int pl = r.i32();
            slot.surface = std::make_shared<CylinderSurface>(frame_at(pl), r.f64());
            break;
        }
        case tag::CONE: {
            int pl = r.i32();
            double rad = r.f64(), a = r.f64();
            slot.surface = std::make_shared<ConeSurface>(frame_at(pl), rad, a);
            break;
        }
        case tag::SPHERE: {
            int pl = r.i32();
            slot.surface = std::make_shared<SphereSurface>(frame_at(pl), r.f64());
            break;
        }
        case tag::TORUS: {
            int pl = r.i32();
            double R = r.f64(), rr = r.f64();
            slot.surface = std::make_shared<TorusSurface>(frame_at(pl), R, rr);
            break;
        }
        case tag::BSPLINE_SURFACE: {
            auto s = std::make_shared<BSplineSurface>();
            s->u_degree = r.i32();
            s->v_degree = r.i32();
            r.i32(); // u_closed
            r.i32(); // v_closed
            r.i32(); // self_intersect
            s->nu = r.i32();
            s->nv = r.i32();
            int ncp = s->nu * s->nv;
            s->ctrl.resize(ncp);
            for (int i = 0; i < ncp; ++i)
                s->ctrl[i] = r.vec3();
            int nuk = r.i32();
            std::vector<double> uk = r.f64s(nuk);
            std::vector<int> um = r.i32s(nuk);
            int nvk = r.i32();
            std::vector<double> vk = r.f64s(nvk);
            std::vector<int> vm = r.i32s(nvk);
            int has_w = r.i32();
            if (has_w)
                s->weights = r.f64s(ncp);
            s->Uu = bspline_detail::expand_knots(uk, um);
            s->Uv = bspline_detail::expand_knots(vk, vm);
            slot.surface = s;
            break;
        }
        case tag::SURF_LIN_EXTRUSION: {
            int sc = r.i32();
            r.i32(); // placement (ignored for eval)
            Vec3 dir = r.vec3();
            double depth = r.f64();
            slot.surface = std::make_shared<LinearExtrusionSurface>(curve_at(sc), dir, depth);
            break;
        }
        case tag::SURF_REVOLUTION: {
            int sc = r.i32();
            int ax = r.i32();
            r.i32(); // optional placement ref (or -1)
            Frame af = frame_at(ax);
            slot.surface = std::make_shared<RevolutionSurface>(curve_at(sc), af.o, af.z);
            break;
        }
        case tag::EDGE_CURVE: {
            slot.is_edge = true;
            slot.e_start = r.vec3();
            slot.e_end = r.vec3();
            int g = r.i32();
            slot.e_same_sense = r.i32() != 0; // kept: closed circles need it for direction
            slot.e_geom = (g >= 0) ? curve_at(g) : nullptr;
            break;
        }
        case tag::ORIENTED_EDGE: {
            int eref = r.i32();
            int orientation = r.i32();
            int has_pc = r.i32();
            if (has_pc)
                r.i32(); // pcurve ref (ignored: tessellator projects 3D->UV)
            int has_params = r.i32();
            double ts = 0, te = 0;
            if (has_params) {
                ts = r.f64();
                te = r.f64();
            }
            const Slot &es = S.at(eref);
            auto oe = std::make_shared<OrientedEdgeN>();
            if (orientation) {
                oe->start = es.e_start;
                oe->end = es.e_end;
            } else {
                oe->start = es.e_end;
                oe->end = es.e_start;
            }
            oe->e_start = es.e_start;
            oe->e_end = es.e_end;
            oe->same_sense = es.e_same_sense;
            oe->orientation = orientation != 0;
            oe->geometry = es.e_geom;
            oe->has_params = has_params != 0;
            oe->t_start = ts;
            oe->t_end = te;
            slot.oedge = oe;
            break;
        }
        case tag::EDGE_LOOP: {
            auto lp = std::make_shared<LoopN>();
            lp->is_poly = false;
            int ne = r.i32();
            lp->edges.reserve(ne);
            for (int i = 0; i < ne; ++i)
                lp->edges.push_back(*S.at(r.i32()).oedge);
            slot.loop = lp;
            break;
        }
        case tag::POLY_LOOP: {
            auto lp = std::make_shared<LoopN>();
            lp->is_poly = true;
            int np = r.i32();
            lp->polygon.reserve(np);
            for (int i = 0; i < np; ++i)
                lp->polygon.push_back(r.vec3());
            slot.loop = lp;
            break;
        }
        case tag::FACE_BOUND: {
            auto b = std::make_shared<FaceBoundN>();
            b->loop = S.at(r.i32()).loop;
            b->orientation = r.i32() != 0;
            slot.bound = b;
            break;
        }
        case tag::FACE_SURFACE: {
            auto f = std::make_shared<FaceSurfaceN>();
            f->surface = surface_at(r.i32());
            f->same_sense = r.i32() != 0;
            int nb = r.i32();
            f->bounds.reserve(nb);
            for (int i = 0; i < nb; ++i)
                f->bounds.push_back(*S.at(r.i32()).bound);
            slot.face = f;
            break;
        }
        case tag::CONNECTED_FACE_SET: {
            auto c = std::make_shared<ConnectedFaceSetN>();
            int nf = r.i32();
            c->faces.reserve(nf);
            for (int i = 0; i < nf; ++i)
                c->faces.push_back(S.at(r.i32()).face);
            slot.cfs = c;
            break;
        }
        case tag::EXTRUDED_AREA_SOLID: {
            auto ex = std::make_shared<ExtrusionN>();
            ex->profile = S.at(r.i32()).face; // profile FACE_SURFACE
            ex->frame = frame_at(r.i32());    // PLACEMENT3
            ex->direction = r.vec3();
            ex->depth = r.f64();
            slot.extrusion = ex;
            break;
        }
        case tag::REVOLVED_AREA_SOLID: {
            auto rv = std::make_shared<RevolveN>();
            rv->profile = S.at(r.i32()).face; // profile FACE_SURFACE
            rv->frame = frame_at(r.i32());    // PLACEMENT3 (position)
            Frame ax = frame_at(r.i32());     // PLACEMENT1 (axis: o=loc, z=dir)
            rv->axis_origin = ax.o;
            rv->axis_dir = ax.z;
            rv->angle = r.f64();
            slot.revolve = rv;
            break;
        }
        case tag::BOOLEAN_RESULT: {
            auto bn = std::make_shared<BooleanN>();
            bn->op = r.i32();
            bn->a = solid_item_at(r.i32());
            bn->b = solid_item_at(r.i32());
            slot.boolean = bn;
            break;
        }
        case tag::SPHERE_SOLID: {
            auto sp = std::make_shared<SphereN>();
            sp->frame = frame_at(r.i32()); // centre placement
            sp->radius = r.f64();
            slot.sphere = sp;
            break;
        }
        default:
            break; // unknown tag -> skipped via nbytes below
        }
        // advance to the next record regardless of how much the case parsed (spec §2)
        r.seek(payload + nbytes);
    }

    // roots trailer
    NgeomDoc doc;
    int root_count = r.i32();
    doc.roots.reserve(root_count);
    for (int i = 0; i < root_count; ++i) {
        int gidx = r.i32();
        int id_len = r.i32();
        std::string id = r.str(id_len);
        NgeomRoot root;
        root.id = std::move(id);
        const Slot &gs = S.at(gidx);
        if (gs.extrusion) {
            root.extrusion = gs.extrusion;
        } else if (gs.revolve) {
            root.revolve = gs.revolve;
        } else if (gs.boolean) {
            root.boolean = gs.boolean;
        } else if (gs.sphere) {
            root.sphere = gs.sphere;
        } else if (gs.face) {
            root.faces.push_back(gs.face);
        } else if (gs.cfs) {
            root.faces = gs.cfs->faces;
        }
        doc.roots.push_back(std::move(root));
    }
    return doc;
}

} // namespace adacpp::ngeom
