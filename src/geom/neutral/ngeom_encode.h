// NGEOM encoder: neutral records (NgeomDoc) -> the NGEOM byte buffer (spec v1). The inverse of
// ngeom_decode.h and a C++ peer of the adapy Python serializer — so encode(decode(buf)) and
// encode(native-read doc) both produce buffers the decoder reads back.
//
// Records are emitted in dependency order (children before parents) with a pointer memo, so every
// ref points at an earlier record (single-pass-decodable). B-spline knot vectors are stored
// EXPANDED in the eval types (BSplineCurve::U / BSplineSurface::Uu,Uv); run-length-encoding them
// recovers the (knots, multiplicities) the wire format wants (expand_knots just repeats each knot
// `mult` times).
#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ngeom_decode.h" // tag enum, NGEOM_VERSION, neutral types

namespace adacpp::ngeom {

class Encoder {
public:
    std::vector<uint8_t> encode(const NgeomDoc &doc) {
        records_.clear();
        memo_.clear();
        std::vector<std::pair<int, std::string>> roots;
        for (const NgeomRoot &root : doc.roots) {
            // Analytic solids (extrusion/revolve/boolean/sphere) encode to tags 50-53 so a faces-less
            // procedural root carries real geometry — previously only CONNECTED_FACE_SET was emitted,
            // so such a root became an empty buffer and the IfcNgeomStream consumer dropped it.
            int gi = -1;
            if (root.extrusion || root.revolve || root.boolean || root.sphere)
                gi = solid_root(root);
            if (gi < 0) {
                std::vector<uint8_t> body;
                put_i32(body, (int) root.faces.size());
                for (const auto &f : root.faces)
                    put_i32(body, face(f));
                gi = add(tag::CONNECTED_FACE_SET, std::move(body));
            }
            roots.emplace_back(gi, root.id);
        }
        std::vector<uint8_t> out;
        const char *magic = "ADANGEOM";
        out.insert(out.end(), magic, magic + 8);
        put_i32(out, (int) NGEOM_VERSION);
        put_i32(out, (int) records_.size());
        for (auto &[t, payload] : records_) {
            put_i32(out, t);
            put_i32(out, (int) payload.size());
            out.insert(out.end(), payload.begin(), payload.end());
        }
        put_i32(out, (int) roots.size());
        for (auto &[gi, id] : roots) {
            put_i32(out, gi);
            put_i32(out, (int) id.size());
            out.insert(out.end(), id.begin(), id.end());
        }
        return out;
    }

private:
    std::vector<std::pair<int, std::vector<uint8_t>>> records_;
    std::unordered_map<const void *, int> memo_;

    static void put_i32(std::vector<uint8_t> &b, int32_t v) {
        size_t o = b.size();
        b.resize(o + 4);
        std::memcpy(&b[o], &v, 4);
    }
    static void put_f64(std::vector<uint8_t> &b, double v) {
        size_t o = b.size();
        b.resize(o + 8);
        std::memcpy(&b[o], &v, 8);
    }
    static void put_v3(std::vector<uint8_t> &b, const Vec3 &p) {
        put_f64(b, p.x);
        put_f64(b, p.y);
        put_f64(b, p.z);
    }
    int add(int t, std::vector<uint8_t> payload) {
        int idx = (int) records_.size();
        records_.emplace_back(t, std::move(payload));
        return idx;
    }
    // PLACEMENT3 stores (location, axis=z, ref=x) — from_axis_ref rebuilds the same frame.
    int placement3(const Frame &f) {
        std::vector<uint8_t> b;
        put_v3(b, f.o);
        put_v3(b, f.z);
        put_v3(b, f.x);
        return add(tag::PLACEMENT3, std::move(b));
    }
    // PLACEMENT1 stores (location, axis) — the revolution axis (spec §5 SURF_REVOLUTION).
    int placement1(const Vec3 &loc, const Vec3 &axis) {
        std::vector<uint8_t> b;
        put_v3(b, loc);
        put_v3(b, axis);
        return add(tag::PLACEMENT1, std::move(b));
    }
    // --- analytic solids (tags 50-53) — layouts match ngeom_decode.h + the adapy Python codec ---
    int extrusion_rec(const std::shared_ptr<ExtrusionN> &ex) {
        std::vector<uint8_t> b;
        put_i32(b, face(ex->profile)); // profile FACE_SURFACE
        put_i32(b, placement3(ex->frame));
        put_v3(b, ex->direction);
        put_f64(b, ex->depth);
        return add(tag::EXTRUDED_AREA_SOLID, std::move(b));
    }
    int revolve_rec(const std::shared_ptr<RevolveN> &rv) {
        std::vector<uint8_t> b;
        put_i32(b, face(rv->profile));
        put_i32(b, placement3(rv->frame));
        put_i32(b, placement1(rv->axis_origin, rv->axis_dir)); // PLACEMENT1 axis
        put_f64(b, rv->angle);
        return add(tag::REVOLVED_AREA_SOLID, std::move(b));
    }
    int sphere_rec(const std::shared_ptr<SphereN> &sp) {
        std::vector<uint8_t> b;
        put_i32(b, placement3(sp->frame)); // centre placement
        put_f64(b, sp->radius);
        return add(tag::SPHERE_SOLID, std::move(b));
    }
    int boolean_rec(const std::shared_ptr<BooleanN> &bn) {
        std::vector<uint8_t> b;
        put_i32(b, bn->op);
        put_i32(b, solid_item_rec(bn->a));
        put_i32(b, solid_item_rec(bn->b));
        return add(tag::BOOLEAN_RESULT, std::move(b));
    }
    // One boolean operand: nested solid, or a shell (CONNECTED_FACE_SET — solid_item_at reads it back
    // as .faces). Sweep operands aren't emittable here (their baked-frame form is out of scope) -> -1.
    int solid_item_rec(const SolidItemN &it) {
        if (it.extrusion)
            return extrusion_rec(it.extrusion);
        if (it.revolve)
            return revolve_rec(it.revolve);
        if (it.boolean)
            return boolean_rec(it.boolean);
        if (!it.faces.empty()) {
            std::vector<uint8_t> b;
            put_i32(b, (int) it.faces.size());
            for (const auto &f : it.faces)
                put_i32(b, face(f));
            return add(tag::CONNECTED_FACE_SET, std::move(b));
        }
        return -1;
    }
    int solid_root(const NgeomRoot &root) {
        if (root.extrusion)
            return extrusion_rec(root.extrusion);
        if (root.revolve)
            return revolve_rec(root.revolve);
        if (root.boolean)
            return boolean_rec(root.boolean);
        if (root.sphere)
            return sphere_rec(root.sphere);
        return -1; // sweep (tag 54) not emitted here — falls back to faces
    }
    static void rle_knots(const std::vector<double> &U, std::vector<double> &knots, std::vector<int> &mults) {
        for (size_t i = 0; i < U.size();) {
            size_t j = i + 1;
            while (j < U.size() && U[j] == U[i])
                ++j;
            knots.push_back(U[i]);
            mults.push_back((int) (j - i));
            i = j;
        }
    }

    int curve(const std::shared_ptr<Curve> &c) {
        if (!c)
            return -1;
        auto it = memo_.find(c.get());
        if (it != memo_.end())
            return it->second;
        int idx = -1;
        std::vector<uint8_t> b;
        if (auto *ci = dynamic_cast<CircleCurve *>(c.get())) {
            put_i32(b, placement3(ci->f));
            put_f64(b, ci->r);
            idx = add(tag::CIRCLE, std::move(b));
        } else if (auto *e = dynamic_cast<EllipseCurve *>(c.get())) {
            put_i32(b, placement3(e->f));
            put_f64(b, e->a1);
            put_f64(b, e->a2);
            idx = add(tag::ELLIPSE, std::move(b));
        } else if (auto *l = dynamic_cast<LineCurve *>(c.get())) {
            put_v3(b, l->pnt);
            put_v3(b, l->dir);
            idx = add(tag::LINE, std::move(b));
        } else if (auto *pl = dynamic_cast<PolylineCurve *>(c.get())) {
            put_i32(b, (int) pl->pts.size());
            for (const auto &p : pl->pts)
                put_v3(b, p);
            idx = add(tag::POLYLINE, std::move(b));
        } else if (auto *bs = dynamic_cast<BSplineCurve *>(c.get())) {
            put_i32(b, bs->degree);
            put_i32(b, bs->closed ? 1 : 0);
            put_i32(b, 0);
            put_i32(b, (int) bs->ctrl.size());
            for (const auto &p : bs->ctrl)
                put_v3(b, p);
            std::vector<double> kn;
            std::vector<int> mu;
            rle_knots(bs->U, kn, mu);
            put_i32(b, (int) kn.size());
            for (double k : kn)
                put_f64(b, k);
            for (int m : mu)
                put_i32(b, m);
            put_i32(b, bs->weights.empty() ? 0 : 1);
            for (double w : bs->weights)
                put_f64(b, w);
            idx = add(tag::BSPLINE_CURVE, std::move(b));
        }
        memo_[c.get()] = idx;
        return idx;
    }

    int surface(const std::shared_ptr<Surface> &s) {
        if (!s)
            return -1;
        auto it = memo_.find(s.get());
        if (it != memo_.end())
            return it->second;
        int idx = -1;
        std::vector<uint8_t> b;
        if (auto *p = dynamic_cast<PlaneSurface *>(s.get())) {
            put_i32(b, placement3(p->f));
            idx = add(tag::PLANE, std::move(b));
        } else if (auto *cy = dynamic_cast<CylinderSurface *>(s.get())) {
            put_i32(b, placement3(cy->f));
            put_f64(b, cy->r);
            idx = add(tag::CYLINDER, std::move(b));
        } else if (auto *co = dynamic_cast<ConeSurface *>(s.get())) {
            put_i32(b, placement3(co->f));
            put_f64(b, co->r0);
            put_f64(b, co->semi_angle);
            idx = add(tag::CONE, std::move(b));
        } else if (auto *sp = dynamic_cast<SphereSurface *>(s.get())) {
            put_i32(b, placement3(sp->f));
            put_f64(b, sp->r);
            idx = add(tag::SPHERE, std::move(b));
        } else if (auto *to = dynamic_cast<TorusSurface *>(s.get())) {
            put_i32(b, placement3(to->f));
            put_f64(b, to->R);
            put_f64(b, to->r);
            idx = add(tag::TORUS, std::move(b));
        } else if (auto *bs = dynamic_cast<BSplineSurface *>(s.get())) {
            put_i32(b, bs->u_degree);
            put_i32(b, bs->v_degree);
            put_i32(b, bs->u_closed ? 1 : 0);
            put_i32(b, bs->v_closed ? 1 : 0);
            put_i32(b, 0);
            put_i32(b, bs->nu);
            put_i32(b, bs->nv);
            for (const auto &p : bs->ctrl)
                put_v3(b, p);
            std::vector<double> uk, vk;
            std::vector<int> um, vm;
            rle_knots(bs->Uu, uk, um);
            rle_knots(bs->Uv, vk, vm);
            put_i32(b, (int) uk.size());
            for (double k : uk)
                put_f64(b, k);
            for (int m : um)
                put_i32(b, m);
            put_i32(b, (int) vk.size());
            for (double k : vk)
                put_f64(b, k);
            for (int m : vm)
                put_i32(b, m);
            put_i32(b, bs->weights.empty() ? 0 : 1);
            for (double w : bs->weights)
                put_f64(b, w);
            idx = add(tag::BSPLINE_SURFACE, std::move(b));
        } else if (auto *le = dynamic_cast<LinearExtrusionSurface *>(s.get())) {
            // SURF_LIN_EXTRUSION: i32(swept_curve) i32(position=-1/None) v3(dir) f64(depth).
            // The neutral model carries no position frame; -1 decodes to None (== adapy's
            // SurfaceOfLinearExtrusion.position) — the trimming face bounds carry the extent.
            put_i32(b, curve(le->profile));
            put_i32(b, -1);
            put_v3(b, le->dir);
            put_f64(b, le->depth);
            idx = add(tag::SURF_LIN_EXTRUSION, std::move(b));
        } else if (auto *rv = dynamic_cast<RevolutionSurface *>(s.get())) {
            // SURF_REVOLUTION: i32(swept_curve) i32(axis PLACEMENT1) i32(-1 reserved).
            put_i32(b, curve(rv->profile));
            put_i32(b, placement1(rv->axis_loc, rv->axis_dir));
            put_i32(b, -1);
            idx = add(tag::SURF_REVOLUTION, std::move(b));
        }
        memo_[s.get()] = idx;
        return idx;
    }

    int oriented_edge(const OrientedEdgeN &oe) {
        std::vector<uint8_t> ec;
        put_v3(ec, oe.e_start);
        put_v3(ec, oe.e_end);
        put_i32(ec, curve(oe.geometry)); // -1 when null (straight edge)
        put_i32(ec, oe.same_sense ? 1 : 0);
        int eref = add(tag::EDGE_CURVE, std::move(ec));
        std::vector<uint8_t> b;
        put_i32(b, eref);
        put_i32(b, oe.orientation ? 1 : 0);
        put_i32(b, 0); // has_pcurve
        put_i32(b, oe.has_params ? 1 : 0);
        if (oe.has_params) {
            put_f64(b, oe.t_start);
            put_f64(b, oe.t_end);
        }
        return add(tag::ORIENTED_EDGE, std::move(b));
    }

    int loop(const std::shared_ptr<LoopN> &lp) {
        auto it = memo_.find(lp.get());
        if (it != memo_.end())
            return it->second;
        int idx;
        if (lp->is_poly) {
            std::vector<uint8_t> b;
            put_i32(b, (int) lp->polygon.size());
            for (const auto &p : lp->polygon)
                put_v3(b, p);
            idx = add(tag::POLY_LOOP, std::move(b));
        } else {
            std::vector<int> refs;
            refs.reserve(lp->edges.size());
            for (const auto &e : lp->edges)
                refs.push_back(oriented_edge(e));
            std::vector<uint8_t> b;
            put_i32(b, (int) refs.size());
            for (int r : refs)
                put_i32(b, r);
            idx = add(tag::EDGE_LOOP, std::move(b));
        }
        memo_[lp.get()] = idx;
        return idx;
    }

    int face_bound(const FaceBoundN &fb) {
        int lp = loop(fb.loop);
        std::vector<uint8_t> b;
        put_i32(b, lp);
        put_i32(b, fb.orientation ? 1 : 0);
        return add(tag::FACE_BOUND, std::move(b));
    }

    int face(const std::shared_ptr<FaceSurfaceN> &f) {
        if (!f)
            return -1;
        auto it = memo_.find(f.get());
        if (it != memo_.end())
            return it->second;
        int surf = surface(f->surface);
        std::vector<int> bounds;
        bounds.reserve(f->bounds.size());
        for (const auto &b : f->bounds)
            bounds.push_back(face_bound(b));
        std::vector<uint8_t> body;
        put_i32(body, surf);
        put_i32(body, f->same_sense ? 1 : 0);
        put_i32(body, (int) bounds.size());
        for (int b : bounds)
            put_i32(body, b);
        int idx = add(tag::FACE_SURFACE, std::move(body));
        memo_[f.get()] = idx;
        return idx;
    }
};

// Encode a whole document (its roots' faces) into one NGEOM buffer.
inline std::vector<uint8_t> encode(const NgeomDoc &doc) {
    Encoder e;
    return e.encode(doc);
}

} // namespace adacpp::ngeom
