// STEP (Part-21) -> NGEOM neutral records resolver. First slice: planar B-rep.
//
// Consumes a parsed entity map (id -> Instance, from step_part21.h) and builds an
// adacpp::ngeom::NgeomDoc. Neutral records are populated to match ngeom_decode.h field-for-field,
// so a natively-read solid tessellates identically to the same solid round-tripped through the
// adapy Python serializer + the NGEOM decoder (the parity oracle).
//
// Supported now: MANIFOLD_SOLID_BREP / CLOSED_SHELL / OPEN_SHELL / ADVANCED_FACE / FACE_SURFACE /
// FACE_OUTER_BOUND / FACE_BOUND / EDGE_LOOP / POLY_LOOP / ORIENTED_EDGE / EDGE_CURVE /
// VERTEX_POINT / AXIS2_PLACEMENT_3D / CARTESIAN_POINT / DIRECTION; surfaces PLANE /
// CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE / TOROIDAL_SURFACE; edge curves
// CIRCLE / ELLIPSE (LINE -> null, straight through endpoints, matching the Python serializer's
// geom=-1 for Line). B-splines, assembly transforms, colours and units come in later slices.
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ngeom_topology.h" // pulls ngeom_curves.h / ngeom_surfaces.h / ngeom_math.h
#include "step_part21.h"

namespace adacpp::step {

namespace ng = adacpp::ngeom;

class Resolver {
public:
    explicit Resolver(const std::unordered_map<long, const Instance *> &by_id) : m_(by_id) {}

    // Find every supported solid root and resolve it. Roots are ordered by ascending entity id
    // (the map is unordered) for deterministic, parity-comparable output.
    ng::NgeomDoc build() {
        std::vector<long> root_ids;
        for (const auto &[id, in] : m_)
            if (!in->complex && in->type == "MANIFOLD_SOLID_BREP")
                root_ids.push_back(id);
        std::sort(root_ids.begin(), root_ids.end());

        ng::NgeomDoc doc;
        doc.roots.reserve(root_ids.size());
        for (long id : root_ids) {
            const Instance *in = inst(id);
            if (!in || in->args.size() < 2 || !in->args[1].is_ref())
                continue;
            ng::NgeomRoot root;
            root.id = solid_name(in, id);
            shell_into(in->args[1].i, root.faces);
            doc.roots.push_back(std::move(root));
        }
        return doc;
    }

private:
    const std::unordered_map<long, const Instance *> &m_;
    std::unordered_map<long, std::shared_ptr<ng::Surface>> surf_cache_;
    std::unordered_map<long, std::shared_ptr<ng::Curve>> curve_cache_;
    std::unordered_map<long, std::shared_ptr<ng::FaceSurfaceN>> face_cache_;

    const Instance *inst(long id) const {
        auto it = m_.find(id);
        return it == m_.end() ? nullptr : it->second;
    }

    // A Part-21 boolean enum is .T. / .F.; anything but explicit F is true (matches the readers).
    static bool enum_true(const Value &v) {
        return !(v.kind == Kind::Enum && v.s == "F");
    }

    static ng::Vec3 vec3_of(const Value &list) {
        ng::Vec3 r{0, 0, 0};
        if (list.kind == Kind::List) {
            if (list.items.size() > 0)
                r.x = list.items[0].as_double();
            if (list.items.size() > 1)
                r.y = list.items[1].as_double();
            if (list.items.size() > 2)
                r.z = list.items[2].as_double();
        }
        return r;
    }

    std::string solid_name(const Instance *in, long id) const {
        if (!in->args.empty() && in->args[0].kind == Kind::Str && !in->args[0].s.empty())
            return unescape(in->args[0].s);
        return "#" + std::to_string(id);
    }

    // CARTESIAN_POINT / DIRECTION (coord list at arg 1) or VERTEX_POINT (ref at arg 1).
    ng::Vec3 point(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.size() < 2)
            return {};
        if (in->type == "VERTEX_POINT" && in->args[1].is_ref())
            return point(in->args[1].i);
        return vec3_of(in->args[1]);
    }

    // AXIS2_PLACEMENT_3D('',#loc,#axis?,#ref?) -> Frame. axis/ref may be $ (defaults +Z / +X).
    ng::Frame placement(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.size() < 2)
            return {};
        ng::Vec3 loc = in->args[1].is_ref() ? point(in->args[1].i) : ng::Vec3{0, 0, 0};
        ng::Vec3 axis{0, 0, 1};
        if (in->args.size() > 2 && in->args[2].is_ref())
            axis = point(in->args[2].i);
        ng::Vec3 ref{1, 0, 0};
        if (in->args.size() > 3 && in->args[3].is_ref())
            ref = point(in->args[3].i);
        return ng::Frame::from_axis_ref(loc, axis, ref);
    }

    // Analytic surfaces. Each STEP entity is `(name, #position, <params...>)` where #position is
    // an AXIS2_PLACEMENT_3D. B-spline surfaces come in a later slice.
    std::shared_ptr<ng::Surface> surface(long id) {
        auto c = surf_cache_.find(id);
        if (c != surf_cache_.end())
            return c->second;
        std::shared_ptr<ng::Surface> s;
        const Instance *in = inst(id);
        if (in && in->args.size() >= 2 && in->args[1].is_ref()) {
            ng::Frame fr = placement(in->args[1].i);
            std::string_view t = in->type;
            if (t == "PLANE")
                s = std::make_shared<ng::PlaneSurface>(fr);
            else if (t == "CYLINDRICAL_SURFACE" && in->args.size() >= 3)
                s = std::make_shared<ng::CylinderSurface>(fr, in->args[2].as_double());
            else if (t == "CONICAL_SURFACE" && in->args.size() >= 4)
                s = std::make_shared<ng::ConeSurface>(fr, in->args[2].as_double(), in->args[3].as_double());
            else if (t == "SPHERICAL_SURFACE" && in->args.size() >= 3)
                s = std::make_shared<ng::SphereSurface>(fr, in->args[2].as_double());
            else if (t == "TOROIDAL_SURFACE" && in->args.size() >= 4)
                s = std::make_shared<ng::TorusSurface>(fr, in->args[2].as_double(), in->args[3].as_double());
        }
        surf_cache_[id] = s;
        return s;
    }

    // Edge geometry. CIRCLE/ELLIPSE resolve to conic curves (arc discretization); LINE (and
    // anything not yet supported) -> null, so the edge discretizes straight through its endpoints
    // — matching the Python serializer, which emits geom=-1 for Line. B-splines come later.
    std::shared_ptr<ng::Curve> curve(long id) {
        auto c = curve_cache_.find(id);
        if (c != curve_cache_.end())
            return c->second;
        std::shared_ptr<ng::Curve> cv;
        const Instance *in = inst(id);
        if (in && in->args.size() >= 2 && in->args[1].is_ref()) {
            ng::Frame fr = placement(in->args[1].i);
            std::string_view t = in->type;
            if (t == "CIRCLE" && in->args.size() >= 3)
                cv = std::make_shared<ng::CircleCurve>(fr, in->args[2].as_double());
            else if (t == "ELLIPSE" && in->args.size() >= 4)
                cv = std::make_shared<ng::EllipseCurve>(fr, in->args[2].as_double(), in->args[3].as_double());
        }
        curve_cache_[id] = cv;
        return cv;
    }

    // ORIENTED_EDGE('',*,*,#edge,orient) wrapping EDGE_CURVE('',#v1,#v2,#geom,sense).
    // Field population mirrors ngeom_decode.h's ORIENTED_EDGE case exactly.
    ng::OrientedEdgeN oriented_edge(long id) {
        ng::OrientedEdgeN oe;
        const Instance *in = inst(id);
        if (!in || in->args.size() < 5)
            return oe;
        bool orientation = enum_true(in->args[4]);
        const Instance *ec = in->args[3].is_ref() ? inst(in->args[3].i) : nullptr;
        if (ec && ec->type == "EDGE_CURVE" && ec->args.size() >= 5) {
            if (ec->args[1].is_ref())
                oe.e_start = point(ec->args[1].i);
            if (ec->args[2].is_ref())
                oe.e_end = point(ec->args[2].i);
            if (ec->args[3].is_ref())
                oe.geometry = curve(ec->args[3].i);
            oe.same_sense = enum_true(ec->args[4]);
        }
        oe.orientation = orientation;
        oe.has_params = false;
        if (orientation) {
            oe.start = oe.e_start;
            oe.end = oe.e_end;
        } else {
            oe.start = oe.e_end;
            oe.end = oe.e_start;
        }
        return oe;
    }

    // EDGE_LOOP('',(#oe,...)) or POLY_LOOP('',(#pt,...)).
    std::shared_ptr<ng::LoopN> loop(long id) {
        auto lp = std::make_shared<ng::LoopN>();
        const Instance *in = inst(id);
        if (in && in->type == "POLY_LOOP" && in->args.size() > 1 && in->args[1].kind == Kind::List) {
            lp->is_poly = true;
            for (const Value &p : in->args[1].items)
                if (p.is_ref())
                    lp->polygon.push_back(point(p.i));
            return lp;
        }
        lp->is_poly = false;
        if (in && in->type == "EDGE_LOOP" && in->args.size() > 1 && in->args[1].kind == Kind::List)
            for (const Value &e : in->args[1].items)
                if (e.is_ref())
                    lp->edges.push_back(oriented_edge(e.i));
        return lp;
    }

    // FACE_OUTER_BOUND / FACE_BOUND('',#loop,orient).
    ng::FaceBoundN face_bound(long id) {
        ng::FaceBoundN b;
        const Instance *in = inst(id);
        if (!in || in->args.size() < 3)
            return b;
        if (in->args[1].is_ref())
            b.loop = loop(in->args[1].i);
        b.orientation = enum_true(in->args[2]);
        return b;
    }

    // ADVANCED_FACE / FACE_SURFACE('',(#bound,...),#surface,same_sense).
    std::shared_ptr<ng::FaceSurfaceN> face(long id) {
        auto c = face_cache_.find(id);
        if (c != face_cache_.end())
            return c->second;
        auto f = std::make_shared<ng::FaceSurfaceN>();
        const Instance *in = inst(id);
        if (in && (in->type == "ADVANCED_FACE" || in->type == "FACE_SURFACE") && in->args.size() >= 4) {
            if (in->args[1].kind == Kind::List)
                for (const Value &b : in->args[1].items)
                    if (b.is_ref())
                        f->bounds.push_back(face_bound(b.i));
            if (in->args[2].is_ref())
                f->surface = surface(in->args[2].i);
            f->same_sense = enum_true(in->args[3]);
        }
        face_cache_[id] = f;
        return f;
    }

    // CLOSED_SHELL / OPEN_SHELL('',(#face,...)) -> append its faces.
    void shell_into(long id, std::vector<std::shared_ptr<ng::FaceSurfaceN>> &out) {
        const Instance *in = inst(id);
        if (!in || (in->type != "CLOSED_SHELL" && in->type != "OPEN_SHELL"))
            return;
        if (in->args.size() > 1 && in->args[1].kind == Kind::List)
            for (const Value &fr : in->args[1].items)
                if (fr.is_ref())
                    out.push_back(face(fr.i));
    }
};

// Convenience: parse a whole STEP buffer and resolve it. Holds the parsed instances (their
// string_views point into `buf`, which the caller must keep alive for the returned doc's strings).
inline ng::NgeomDoc read_step_brep(std::string_view buf, std::vector<Instance> &store) {
    store.clear();
    scan_instances(buf, [&](const Instance &in) { store.push_back(in); });
    std::unordered_map<long, const Instance *> by_id;
    by_id.reserve(store.size() * 2);
    for (const Instance &in : store)
        by_id[in.id] = &in;
    Resolver r(by_id);
    return r.build();
}

} // namespace adacpp::step
