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
// CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE / TOROIDAL_SURFACE /
// B_SPLINE_SURFACE_WITH_KNOTS (+ rational complex records); edge curves CIRCLE / ELLIPSE /
// B_SPLINE_CURVE_WITH_KNOTS (+ rational) (LINE -> null, straight through endpoints, matching the
// Python serializer's geom=-1 for Line). Metadata: per-solid colour (STYLED_ITEM -> COLOUR_RGB,
// BFS style tree) on NgeomRoot, and the file length-unit scale (LENGTH_UNIT/SI_UNIT) on NgeomDoc.
// Assembly transforms (CDSR/SRR/MAPPED_ITEM world matrices, which also fold in the unit scale)
// come in the next slice.
#pragma once

#include <algorithm>
#include <cctype>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ngeom_bspline.h"  // BSplineCurve / BSplineSurface / expand_knots
#include "ngeom_topology.h" // pulls ngeom_curves.h / ngeom_surfaces.h / ngeom_math.h
#include "step_part21.h"

namespace adacpp::step {

namespace ng = adacpp::ngeom;

class Resolver {
public:
    explicit Resolver(const std::unordered_map<long, const Instance *> &by_id) : m_(by_id) {}

    // Find every supported root and resolve it. Roots are ordered by ascending entity id (the map
    // is unordered) for deterministic, parity-comparable output. Root types mirror the Python
    // reader: MANIFOLD_SOLID_BREP (-> its shell) and SHELL_BASED_SURFACE_MODEL (-> its shells'
    // faces; surface/no-thickness shapes, e.g. FEA-exported plates).
    ng::NgeomDoc build() {
        std::vector<long> root_ids;
        for (const auto &[id, in] : m_)
            if (!in->complex && (in->type == "MANIFOLD_SOLID_BREP" || in->type == "SHELL_BASED_SURFACE_MODEL"))
                root_ids.push_back(id);
        std::sort(root_ids.begin(), root_ids.end());

        build_colour_map();
        ng::NgeomDoc doc;
        doc.unit_scale = detect_unit_scale();
        doc.roots.reserve(root_ids.size());
        for (long id : root_ids) {
            const Instance *in = inst(id);
            if (!in || in->args.size() < 2)
                continue;
            ng::NgeomRoot root;
            root.id = solid_name(in, id);
            if (in->type == "MANIFOLD_SOLID_BREP") {
                if (in->args[1].is_ref())
                    shell_into(in->args[1].i, root.faces); // arg1 = the CLOSED_SHELL
            } else if (in->args[1].kind == Kind::List) {
                for (const Value &sh : in->args[1].items) // arg1 = list of shells
                    if (sh.is_ref())
                        shell_into(sh.i, root.faces);
            }
            // STYLED_ITEM colour usually targets the solid root itself.
            auto cit = colour_map_.find(id);
            if (cit != colour_map_.end()) {
                root.has_color = true;
                root.cr = cit->second.r;
                root.cg = cit->second.g;
                root.cb = cit->second.b;
                root.ca = cit->second.a;
            }
            doc.roots.push_back(std::move(root));
        }
        return doc;
    }

private:
    const std::unordered_map<long, const Instance *> &m_;
    std::unordered_map<long, std::shared_ptr<ng::Surface>> surf_cache_;
    std::unordered_map<long, std::shared_ptr<ng::Curve>> curve_cache_;
    std::unordered_map<long, std::shared_ptr<ng::FaceSurfaceN>> face_cache_;

    struct RGBA {
        float r, g, b, a;
    };
    std::unordered_map<long, RGBA> colour_map_; // STYLED_ITEM geometric-item id -> colour

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

    // --- B-spline helpers ----------------------------------------------------------------
    static std::vector<double> reals(const Value &v) {
        std::vector<double> r;
        if (v.kind == Kind::List) {
            r.reserve(v.items.size());
            for (const Value &x : v.items)
                r.push_back(x.as_double());
        }
        return r;
    }
    static std::vector<int> ints(const Value &v) {
        std::vector<int> r;
        if (v.kind == Kind::List) {
            r.reserve(v.items.size());
            for (const Value &x : v.items)
                r.push_back((int) x.i);
        }
        return r;
    }
    // Find a named sub-record of a complex instance (e.g. "RATIONAL_B_SPLINE_SURFACE").
    static const std::vector<Value> *sub(const Instance *in, std::string_view name) {
        for (const auto &[n, a] : in->subs)
            if (n == name)
                return &a;
        return nullptr;
    }

    // Build a BSplineCurve from raw Part-21 args. closed is forced false to match ngeom_decode.h
    // (the decoder reads B_SPLINE_CURVE.closed_curve but ignores it for evaluation).
    std::shared_ptr<ng::Curve> build_bspline_curve(long deg, const Value &cp, const Value &mults, const Value &knots,
                                                   const Value *weights) {
        std::vector<ng::Vec3> control;
        if (cp.kind == Kind::List) {
            control.reserve(cp.items.size());
            for (const Value &r : cp.items)
                if (r.is_ref())
                    control.push_back(point(r.i));
        }
        std::vector<double> w = weights ? reals(*weights) : std::vector<double>{};
        return std::make_shared<ng::BSplineCurve>((int) deg, std::move(control), reals(knots), ints(mults),
                                                  std::move(w), false);
    }

    // Build a BSplineSurface from raw Part-21 args (grid = list of u-rows of control-point refs;
    // optional weights = same-shaped grid of reals). u_closed/v_closed are left default, matching
    // ngeom_decode.h (which reads but ignores them).
    std::shared_ptr<ng::Surface> build_bspline_surface(long u_deg, long v_deg, const Value &grid, const Value &u_mults,
                                                       const Value &v_mults, const Value &u_knots, const Value &v_knots,
                                                       const Value *weights) {
        auto s = std::make_shared<ng::BSplineSurface>();
        s->u_degree = (int) u_deg;
        s->v_degree = (int) v_deg;
        if (grid.kind != Kind::List || grid.items.empty())
            return s;
        s->nu = (int) grid.items.size();
        s->nv = grid.items[0].kind == Kind::List ? (int) grid.items[0].items.size() : 0;
        s->ctrl.resize((size_t) s->nu * s->nv);
        for (int iu = 0; iu < s->nu; ++iu) {
            const Value &row = grid.items[iu];
            if (row.kind != Kind::List)
                continue;
            for (int iv = 0; iv < s->nv && iv < (int) row.items.size(); ++iv)
                if (row.items[iv].is_ref())
                    s->ctrl[(size_t) iu * s->nv + iv] = point(row.items[iv].i);
        }
        s->Uu = ng::bspline_detail::expand_knots(reals(u_knots), ints(u_mults));
        s->Uv = ng::bspline_detail::expand_knots(reals(v_knots), ints(v_mults));
        if (weights && weights->kind == Kind::List) {
            s->weights.resize((size_t) s->nu * s->nv);
            for (int iu = 0; iu < s->nu && iu < (int) weights->items.size(); ++iu) {
                const Value &row = weights->items[iu];
                if (row.kind != Kind::List)
                    continue;
                for (int iv = 0; iv < s->nv && iv < (int) row.items.size(); ++iv)
                    s->weights[(size_t) iu * s->nv + iv] = row.items[iv].as_double();
            }
        }
        return s;
    }

    // A rational B-spline is a complex record splitting data across sub-types: B_SPLINE_SURFACE
    // (degrees, grid, flags), B_SPLINE_SURFACE_WITH_KNOTS (mults, knots), RATIONAL_B_SPLINE_SURFACE
    // (weights). Sub args have NO leading name string. Same shape for curves.
    std::shared_ptr<ng::Surface> bspline_surface_complex(const Instance *in) {
        const auto *bs = sub(in, "B_SPLINE_SURFACE");
        const auto *bk = sub(in, "B_SPLINE_SURFACE_WITH_KNOTS");
        const auto *rat = sub(in, "RATIONAL_B_SPLINE_SURFACE");
        if (!bs || !bk || bs->size() < 3 || bk->size() < 4)
            return nullptr;
        return build_bspline_surface((*bs)[0].i, (*bs)[1].i, (*bs)[2], (*bk)[0], (*bk)[1], (*bk)[2], (*bk)[3],
                                     (rat && !rat->empty()) ? &(*rat)[0] : nullptr);
    }
    std::shared_ptr<ng::Curve> bspline_curve_complex(const Instance *in) {
        const auto *bc = sub(in, "B_SPLINE_CURVE");
        const auto *bk = sub(in, "B_SPLINE_CURVE_WITH_KNOTS");
        const auto *rat = sub(in, "RATIONAL_B_SPLINE_CURVE");
        if (!bc || !bk || bc->size() < 2 || bk->size() < 2)
            return nullptr;
        return build_bspline_curve((*bc)[0].i, (*bc)[1], (*bk)[0], (*bk)[1],
                                   (rat && !rat->empty()) ? &(*rat)[0] : nullptr);
    }

    // Analytic + B-spline surfaces. Analytic entities are `(name, #position, <params...>)` with
    // #position an AXIS2_PLACEMENT_3D; B_SPLINE_SURFACE_WITH_KNOTS and rational complex records
    // carry their own data.
    std::shared_ptr<ng::Surface> surface(long id) {
        auto c = surf_cache_.find(id);
        if (c != surf_cache_.end())
            return c->second;
        std::shared_ptr<ng::Surface> s;
        const Instance *in = inst(id);
        if (in && in->complex) {
            s = bspline_surface_complex(in);
        } else if (in) {
            std::string_view t = in->type;
            if (t == "B_SPLINE_SURFACE_WITH_KNOTS" && in->args.size() >= 12)
                s = build_bspline_surface(in->args[1].i, in->args[2].i, in->args[3], in->args[8], in->args[9],
                                          in->args[10], in->args[11], nullptr);
            else if (in->args.size() >= 2 && in->args[1].is_ref()) {
                ng::Frame fr = placement(in->args[1].i);
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
        }
        surf_cache_[id] = s;
        return s;
    }

    // Edge geometry. CIRCLE/ELLIPSE -> conic curves; B_SPLINE_CURVE_WITH_KNOTS + rational complex
    // records -> B-spline curves; LINE (and anything unsupported) -> null, so the edge discretizes
    // straight through its endpoints (matches the Python serializer's geom=-1 for Line).
    std::shared_ptr<ng::Curve> curve(long id) {
        auto c = curve_cache_.find(id);
        if (c != curve_cache_.end())
            return c->second;
        std::shared_ptr<ng::Curve> cv;
        const Instance *in = inst(id);
        if (in && in->complex) {
            cv = bspline_curve_complex(in);
        } else if (in) {
            std::string_view t = in->type;
            if (t == "B_SPLINE_CURVE_WITH_KNOTS" && in->args.size() >= 8)
                cv = build_bspline_curve(in->args[1].i, in->args[2], in->args[6], in->args[7], nullptr);
            else if (in->args.size() >= 2 && in->args[1].is_ref()) {
                ng::Frame fr = placement(in->args[1].i);
                if (t == "CIRCLE" && in->args.size() >= 3)
                    cv = std::make_shared<ng::CircleCurve>(fr, in->args[2].as_double());
                else if (t == "ELLIPSE" && in->args.size() >= 4)
                    cv = std::make_shared<ng::EllipseCurve>(fr, in->args[2].as_double(), in->args[3].as_double());
            }
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

    // --- presentation colours (STYLED_ITEM -> COLOUR_RGB) ---------------------------------
    static void collect_refs(const std::vector<Value> &args, std::vector<long> &out) {
        for (const Value &v : args) {
            if (v.is_ref())
                out.push_back(v.i);
            else if (v.kind == Kind::List)
                collect_refs(v.items, out);
        }
    }
    static RGBA predefined_colour(std::string_view name, bool &found) {
        std::string n;
        n.reserve(name.size());
        for (char c : name)
            n.push_back((char) std::tolower((unsigned char) c));
        found = true;
        if (n == "red")
            return {1, 0, 0, 1};
        if (n == "green")
            return {0, 1, 0, 1};
        if (n == "blue")
            return {0, 0, 1, 1};
        if (n == "yellow")
            return {1, 1, 0, 1};
        if (n == "magenta")
            return {1, 0, 1, 1};
        if (n == "cyan")
            return {0, 1, 1, 1};
        if (n == "black")
            return {0, 0, 0, 1};
        if (n == "white")
            return {1, 1, 1, 1};
        found = false;
        return {0.5f, 0.5f, 0.5f, 1};
    }

    // BFS an entity's style reference tree for the first COLOUR_RGB / pre-defined colour and the
    // first SURFACE_STYLE_TRANSPARENT (alpha = 1 - transparency). Mirrors the Python _find_colour.
    bool find_colour(long ref_id, RGBA &out) {
        std::deque<std::pair<long, int>> q;
        q.push_back({ref_id, 0});
        std::unordered_set<long> seen;
        bool have_rgb = false, have_t = false;
        float r = 0, g = 0, b = 0, transp = 0;
        while (!q.empty()) {
            auto [rid, d] = q.front();
            q.pop_front();
            if (d > 12 || seen.count(rid))
                continue;
            seen.insert(rid);
            const Instance *rec = inst(rid);
            if (!rec || rec->complex)
                continue;
            std::string_view t = rec->type;
            if (t == "COLOUR_RGB" && !have_rgb && rec->args.size() >= 4) {
                r = (float) rec->args[1].as_double();
                g = (float) rec->args[2].as_double();
                b = (float) rec->args[3].as_double();
                have_rgb = true;
            } else if ((t == "DRAUGHTING_PRE_DEFINED_COLOUR" || t == "PRE_DEFINED_COLOUR") && !have_rgb) {
                if (!rec->args.empty() && rec->args[0].kind == Kind::Str) {
                    bool ok = false;
                    RGBA c = predefined_colour(rec->args[0].s, ok);
                    if (ok) {
                        r = c.r;
                        g = c.g;
                        b = c.b;
                        have_rgb = true;
                    }
                }
            } else if (t == "SURFACE_STYLE_TRANSPARENT" && !have_t) {
                for (const Value &v : rec->args)
                    if (v.kind == Kind::Real || v.kind == Kind::Int) {
                        transp = (float) v.as_double();
                        have_t = true;
                        break;
                    }
            } else {
                std::vector<long> refs;
                collect_refs(rec->args, refs);
                for (long c : refs)
                    q.push_back({c, d + 1});
            }
            if (have_rgb && have_t)
                break;
        }
        if (!have_rgb)
            return false;
        out = {r, g, b, have_t ? std::clamp(1.0f - transp, 0.0f, 1.0f) : 1.0f};
        return true;
    }

    // STYLED_ITEM('',(styles),#item) -> colour_map_[item id]. Search only the style refs.
    void build_colour_map() {
        for (const auto &[id, in] : m_) {
            if (in->complex || in->type != "STYLED_ITEM" || in->args.size() < 3)
                continue;
            const Value &item = in->args[2];
            if (!item.is_ref() || colour_map_.count(item.i))
                continue;
            std::vector<long> style_refs;
            if (in->args[1].kind == Kind::List)
                collect_refs(in->args[1].items, style_refs);
            else if (in->args[1].is_ref())
                style_refs.push_back(in->args[1].i);
            for (long rid : style_refs) {
                RGBA c{};
                if (find_colour(rid, c)) {
                    colour_map_[item.i] = c;
                    break;
                }
            }
        }
    }

    // --- length unit -> metres (LENGTH_UNIT / SI_UNIT) ------------------------------------
    static double si_prefix_factor(std::string_view p) {
        if (p == "EXA")
            return 1e18;
        if (p == "PETA")
            return 1e15;
        if (p == "TERA")
            return 1e12;
        if (p == "GIGA")
            return 1e9;
        if (p == "MEGA")
            return 1e6;
        if (p == "KILO")
            return 1e3;
        if (p == "HECTO")
            return 1e2;
        if (p == "DECA")
            return 1e1;
        if (p == "DECI")
            return 1e-1;
        if (p == "CENTI")
            return 1e-2;
        if (p == "MILLI")
            return 1e-3;
        if (p == "MICRO")
            return 1e-6;
        if (p == "NANO")
            return 1e-9;
        if (p == "PICO")
            return 1e-12;
        return 1.0;
    }
    // SI_UNIT args: (prefix?, .METRE.). Returns the scale to metres, or <0 if not a length unit.
    static double si_length_scale(const std::vector<Value> &a) {
        std::string_view prefix, unit;
        if (a.size() >= 2 && a[1].kind == Kind::Enum) {
            prefix = a[0].kind == Kind::Enum ? a[0].s : std::string_view{};
            unit = a[1].s;
        } else if (a.size() == 1 && a[0].kind == Kind::Enum) {
            unit = a[0].s;
        }
        if (unit != "METRE" && unit != "METER")
            return -1.0;
        return si_prefix_factor(prefix);
    }
    // The lowest-id LENGTH_UNIT in the model -> its SI scale to metres (default 1.0). The
    // ubiquitous form is a complex record ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
    // a plain top-level SI_UNIT('',prefix,.METRE.) is also handled. Lowest-id for determinism
    // without materializing/sorting all ids.
    double detect_unit_scale() {
        long best_id = 0;
        double best = 1.0;
        bool found = false;
        for (const auto &[id, in] : m_) {
            double s = -1.0;
            if (in->complex) {
                if (sub(in, "LENGTH_UNIT"))
                    if (const auto *si = sub(in, "SI_UNIT"))
                        s = si_length_scale(*si);
            } else if (in->type == "SI_UNIT" && !in->args.empty()) {
                std::vector<Value> a(in->args.begin() + 1, in->args.end()); // drop the name
                s = si_length_scale(a);
            }
            if (s > 0 && (!found || id < best_id)) {
                best_id = id;
                best = s;
                found = true;
            }
        }
        return best;
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
