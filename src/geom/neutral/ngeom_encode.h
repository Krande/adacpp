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
            std::vector<uint8_t> body;
            put_i32(body, (int) root.faces.size());
            for (const auto &f : root.faces)
                put_i32(body, face(f));
            roots.emplace_back(add(tag::CONNECTED_FACE_SET, std::move(body)), root.id);
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
