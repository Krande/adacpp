// Native IFC advanced-B-rep reader -> ng:: neutral geometry, for the native IFC->STEP path. Parses an
// IFC4/IFC4X3 SPF file (same Part-21 grammar as STEP, so it reuses the STEP reader's StreamIndex +
// parse_statement) and resolves the geometry resource entities (IfcAdvancedBrep + analytic surfaces/
// curves + IfcMappedItem instancing) into ng::NgeomRoot — the inverse of ifc_emit.h. Dep-free
// (stdlib + ng:: + step_reader's Part-21). Scope: analytic B-rep IFC (the precise-geometry interop
// case + this codebase's own STEP->IFC output); tessellated/CSG IFC is out of scope (no faces -> the
// solid is skipped, counted, never silently mangled).
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../cadit/step/step_part21.h"
#include "../cadit/step/step_reader.h" // StreamIndex
#include "../geom/neutral/ngeom_bspline.h"
#include "../geom/neutral/ngeom_curves.h"
#include "../geom/neutral/ngeom_surfaces.h"
#include "../geom/neutral/ngeom_topology.h"

namespace adacpp::ifc_read {

using namespace adacpp::ngeom;
using adacpp::step::Instance;
using adacpp::step::StreamIndex;
using adacpp::step::Value;

inline bool iequals(std::string_view a, const char *b) {
    size_t n = 0;
    for (; b[n]; ++n) {
        if (n >= a.size())
            return false;
        char ca = a[n], cb = b[n];
        if (ca >= 'a' && ca <= 'z')
            ca -= 32;
        if (cb >= 'a' && cb <= 'z')
            cb -= 32;
        if (ca != cb)
            return false;
    }
    return n == a.size();
}

class IfcResolver {
  public:
    explicit IfcResolver(const StreamIndex &idx) : idx_(idx) {}

    // Root products: scan for IfcBuildingElementProxy (or any product carrying an AdvancedBrep). Each
    // yields one NgeomRoot (geometry + per-instance world transforms from IfcMappedItem).
    std::vector<long> proxy_roots() {
        std::vector<long> roots;
        std::string scratch;
        for (long id : idx_.ids) {
            if (is_product_type(type_of(id, scratch)))
                roots.push_back(id);
        }
        return roots;
    }

    // The geometry-bearing IfcElement subtypes (cheap type-name test — avoids parsing every entity on
    // a giant file). Anything not listed (or with unrepresentable geometry) is skipped -> OCC fallback.
    static bool is_product_type(std::string_view t) {
        static const char *kinds[] = {
            "IFCBUILDINGELEMENTPROXY", "IFCMECHANICALFASTENER", "IFCELEMENTASSEMBLY", "IFCFASTENER",
            "IFCDISCRETEACCESSORY", "IFCBUILDINGELEMENTPART", "IFCBEAM", "IFCCOLUMN", "IFCMEMBER",
            "IFCPLATE", "IFCWALL", "IFCWALLSTANDARDCASE", "IFCSLAB", "IFCFOOTING", "IFCPILE", "IFCROOF",
            "IFCSTAIR", "IFCSTAIRFLIGHT", "IFCRAMP", "IFCRAMPFLIGHT", "IFCRAILING", "IFCCOVERING",
            "IFCCURTAINWALL", "IFCDOOR", "IFCWINDOW", "IFCCHIMNEY", "IFCSHADINGDEVICE", "IFCPIPESEGMENT",
            "IFCPIPEFITTING", "IFCDUCTSEGMENT", "IFCDUCTFITTING", "IFCFLOWSEGMENT", "IFCFLOWFITTING",
            "IFCFLOWTERMINAL", "IFCFLOWCONTROLLER", "IFCDISTRIBUTIONELEMENT", "IFCENERGYCONVERSIONDEVICE",
            "IFCREINFORCINGBAR", "IFCREINFORCINGMESH", "IFCTENDON", "IFCTENDONANCHOR", "IFCFURNISHINGELEMENT",
            "IFCFURNITURE", "IFCSYSTEMFURNITUREELEMENT", "IFCCIVILELEMENT", "IFCGEOGRAPHICELEMENT",
            // *StandardCase / *ElementedCase product subtypes (NB: the *TYPE entities are NOT products)
            "IFCBEAMSTANDARDCASE", "IFCCOLUMNSTANDARDCASE", "IFCMEMBERSTANDARDCASE", "IFCPLATESTANDARDCASE",
            "IFCSLABSTANDARDCASE", "IFCSLABELEMENTEDCASE", "IFCWALLELEMENTEDCASE", "IFCDOORSTANDARDCASE",
            "IFCWINDOWSTANDARDCASE"};
        for (const char *k : kinds)
            if (iequals(t, k))
                return true;
        return false;
    }

    // The file's length unit as metres-per-unit, from the first IfcSIUnit(*,.LENGTHUNIT.,prefix,.METRE.).
    double unit_scale() {
        std::string scratch;
        for (long id : idx_.ids) {
            std::string_view t = type_of(id, scratch);
            if (!iequals(t, "IFCSIUNIT"))
                continue;
            const Instance *in = inst(id); // (Dimensions, UnitType, Prefix, Name)
            if (!in || in->args.size() < 4)
                continue;
            bool is_len = in->args[1].kind == adacpp::step::Kind::Enum && in->args[1].s == "LENGTHUNIT";
            bool is_metre = in->args[3].kind == adacpp::step::Kind::Enum && in->args[3].s == "METRE";
            if (!is_len || !is_metre)
                continue;
            if (in->args[2].kind != adacpp::step::Kind::Enum)
                return 1.0; // no prefix -> METRE
            std::string_view pf = in->args[2].s;
            if (pf == "MILLI")
                return 1e-3;
            if (pf == "CENTI")
                return 1e-2;
            if (pf == "DECI")
                return 1e-1;
            if (pf == "KILO")
                return 1e3;
            if (pf == "MICRO")
                return 1e-6;
            return 1.0;
        }
        return 1.0;
    }

    // Build the NgeomRoot for a product id (faces + name + per-instance transforms). Empty faces => the
    // product had no resolvable advanced-brep (skipped by the caller).
    NgeomRoot resolve_product(long pid) {
        NgeomRoot root;
        solid_src_ = 0;
        mixed_ = false;
        const Instance *p = inst(pid);
        if (!p)
            return root;
        root.id = name_of(*p);
        // IfcBuildingElementProxy.Representation (arg 6) -> IfcProductDefinitionShape.Representations
        // (arg 2) -> IfcShapeRepresentation.Items (arg 3).
        long rep = ref_arg(*p, 6);
        const Instance *pds = inst(rep);
        if (!pds || pds->args.size() < 3)
            return root;
        for (const Value &srref : pds->args[2].items) {
            const Instance *sr = inst(srref.i);
            if (!sr || sr->args.size() < 4)
                continue;
            for (const Value &item : sr->args[3].items) {
                if (!item.is_ref())
                    continue;
                resolve_item(item.i, root);
            }
        }
        // A product mixing >1 distinct solid (or brep+procedural) doesn't fit the single-solid root
        // model — leave it for OCC rather than emit partial geometry.
        if (mixed_) {
            root.faces.clear();
            root.extrusion = nullptr;
            root.revolve = nullptr;
        }
        // Compose the product's world placement (IfcLocalPlacement chain) onto every instance — the
        // geometry rep is in the element's local frame; ObjectPlacement positions it in the world.
        if (!root.faces.empty() || root.extrusion || root.revolve) {
            std::array<float, 16> objp = object_placement(ref_arg(*p, 5)); // ObjectPlacement = arg 5
            if (!is_identity(objp)) {
                if (root.transforms.empty())
                    root.transforms.push_back(objp);
                else
                    for (auto &m : root.transforms)
                        m = mat_mul(objp, m);
            }
        }
        // A revolve under a non-rigid (scale/shear) instance is no longer a revolve (the circular sweep
        // distorts) and can't be carried by REVOLVED_AREA_SOLID -> drop it to OCC rather than emit wrong.
        if (root.revolve)
            for (const auto &m : root.transforms)
                if (!is_rigid(m)) {
                    root.revolve = nullptr;
                    break;
                }
        return root;
    }

  private:
    const StreamIndex &idx_;
    std::unordered_map<long, std::pair<std::string, Instance>> cache_;
    std::unordered_map<long, std::shared_ptr<Surface>> surf_cache_;
    std::string pread_scratch_;
    long solid_src_ = 0;  // entity id of the one solid this product carries (mapped instances share it)
    bool mixed_ = false;  // product has >1 distinct solid / mixes brep+procedural -> skip (OCC)

    const Instance *inst(long id) {
        if (id <= 0)
            return nullptr;
        auto it = cache_.find(id);
        if (it != cache_.end())
            return &it->second.second;
        std::string_view stmt = idx_.statement_bytes(id, pread_scratch_);
        if (stmt.empty())
            return nullptr;
        auto &slot = cache_[id];
        slot.first.assign(stmt.data(), stmt.size());
        if (!adacpp::step::parse_statement(slot.first, slot.second)) {
            cache_.erase(id);
            return nullptr;
        }
        return &slot.second;
    }
    // Cheap type token of #id without a full parse.
    std::string_view type_of(long id, std::string &scratch) {
        std::string_view s = idx_.statement_bytes(id, scratch);
        size_t eq = s.find('=');
        if (eq == std::string_view::npos)
            return {};
        size_t p = eq + 1;
        while (p < s.size() && (s[p] == ' ' || s[p] == '\t')) // SPF allows whitespace after '='
            ++p;
        if (p < s.size() && s[p] == '(')
            return {}; // complex instance — not a root type we scan
        size_t q = p;
        while (q < s.size() && (std::isalnum((unsigned char) s[q]) || s[q] == '_'))
            ++q;
        return s.substr(p, q - p);
    }
    static long ref_arg(const Instance &in, size_t i) {
        return (i < in.args.size() && in.args[i].is_ref()) ? in.args[i].i : 0;
    }
    static double ad(const Instance *in, size_t i) { // numeric arg (0 if absent/$)
        return (in && i < in->args.size() &&
                (in->args[i].kind == adacpp::step::Kind::Real || in->args[i].kind == adacpp::step::Kind::Int))
                   ? in->args[i].as_double()
                   : 0.0;
    }
    static std::string name_of(const Instance &in) {
        // rooted entities: GlobalId, OwnerHistory, Name(arg2) ...
        if (in.args.size() > 2 && in.args[2].kind == adacpp::step::Kind::Str)
            return std::string(in.args[2].s);
        return {};
    }

    // -- geometry -----------------------------------------------------------
    Vec3 point(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.empty() || !in->args[0].is_list())
            return {0, 0, 0};
        const auto &c = in->args[0].items;
        return Vec3{c.size() > 0 ? c[0].as_double() : 0.0, c.size() > 1 ? c[1].as_double() : 0.0,
                    c.size() > 2 ? c[2].as_double() : 0.0};
    }
    Vec3 dir(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.empty() || !in->args[0].is_list())
            return {0, 0, 1};
        const auto &c = in->args[0].items;
        Vec3 v{c.size() > 0 ? c[0].as_double() : 0.0, c.size() > 1 ? c[1].as_double() : 0.0,
               c.size() > 2 ? c[2].as_double() : 0.0};
        return v;
    }
    // IfcAxis2Placement3D(Location, Axis, RefDirection) -> Frame.
    Frame axis2(long id) {
        Frame f;
        const Instance *in = inst(id);
        if (!in)
            return f;
        if (in->args.size() > 0 && in->args[0].is_ref())
            f.o = point(in->args[0].i);
        Vec3 z{0, 0, 1}, x{1, 0, 0};
        if (in->args.size() > 1 && in->args[1].is_ref())
            z = dir(in->args[1].i);
        if (in->args.size() > 2 && in->args[2].is_ref())
            x = dir(in->args[2].i);
        return Frame::from_axis_ref(f.o, z, x);
    }
    static std::vector<double> reals(const Value &list) {
        std::vector<double> v;
        if (list.is_list())
            for (const Value &e : list.items)
                v.push_back(e.as_double());
        return v;
    }
    static std::vector<int> ints(const Value &list) {
        std::vector<int> v;
        if (list.is_list())
            for (const Value &e : list.items)
                v.push_back((int) e.i);
        return v;
    }
    std::vector<Vec3> point_list(const Value &list) {
        std::vector<Vec3> v;
        if (list.is_list())
            for (const Value &e : list.items)
                if (e.is_ref())
                    v.push_back(point(e.i));
        return v;
    }

    std::shared_ptr<Curve> curve(long id) {
        const Instance *in = inst(id);
        if (!in)
            return nullptr;
        std::string_view t = in->type;
        if (iequals(t, "IFCLINE")) {
            Vec3 p = point(ref_arg(*in, 0));
            const Instance *vec = inst(ref_arg(*in, 1)); // IfcVector(Orientation, Magnitude)
            Vec3 d = vec ? dir(ref_arg(*vec, 0)) : Vec3{0, 0, 1};
            return std::make_shared<LineCurve>(p, d);
        }
        if (iequals(t, "IFCCIRCLE"))
            return std::make_shared<CircleCurve>(axis2(ref_arg(*in, 0)), in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCELLIPSE"))
            return std::make_shared<EllipseCurve>(axis2(ref_arg(*in, 0)), in->args.size() > 1 ? in->args[1].as_double() : 0.0,
                                                  in->args.size() > 2 ? in->args[2].as_double() : 0.0);
        if (iequals(t, "IFCPOLYLINE"))
            return std::make_shared<PolylineCurve>(point_list(in->args.empty() ? Value{} : in->args[0]));
        if (iequals(t, "IFCBSPLINECURVEWITHKNOTS") || iequals(t, "IFCRATIONALBSPLINECURVEWITHKNOTS"))
            return bspline_curve(*in);
        return nullptr;
    }
    // IfcBSplineCurveWithKnots(Degree,CPs,Form,Closed,SelfIntersect,Mults,Knots,Spec[,Weights]).
    std::shared_ptr<Curve> bspline_curve(const Instance &in) {
        int deg = in.args.size() > 0 ? (int) in.args[0].i : 1;
        std::vector<Vec3> cps = point_list(in.args.size() > 1 ? in.args[1] : Value{});
        std::vector<int> mult = ints(in.args.size() > 5 ? in.args[5] : Value{});
        std::vector<double> knots = reals(in.args.size() > 6 ? in.args[6] : Value{});
        std::vector<double> w;
        if (iequals(in.type, "IFCRATIONALBSPLINECURVEWITHKNOTS") && in.args.size() > 8)
            w = reals(in.args[8]);
        bool closed = in.args.size() > 3 && !(in.args[3].kind == adacpp::step::Kind::Enum && in.args[3].s == "F");
        return std::make_shared<BSplineCurve>(deg, std::move(cps), std::move(knots), std::move(mult), std::move(w),
                                              closed);
    }

    std::shared_ptr<Surface> surface(long id) {
        auto cit = surf_cache_.find(id);
        if (cit != surf_cache_.end())
            return cit->second;
        std::shared_ptr<Surface> s = build_surface(id);
        surf_cache_[id] = s;
        return s;
    }
    std::shared_ptr<Surface> build_surface(long id) {
        const Instance *in = inst(id);
        if (!in)
            return nullptr;
        std::string_view t = in->type;
        if (iequals(t, "IFCPLANE"))
            return std::make_shared<PlaneSurface>(axis2(ref_arg(*in, 0)));
        if (iequals(t, "IFCCYLINDRICALSURFACE"))
            return std::make_shared<CylinderSurface>(axis2(ref_arg(*in, 0)), in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCSPHERICALSURFACE"))
            return std::make_shared<SphereSurface>(axis2(ref_arg(*in, 0)), in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCTOROIDALSURFACE"))
            return std::make_shared<TorusSurface>(axis2(ref_arg(*in, 0)), in->args.size() > 1 ? in->args[1].as_double() : 0.0,
                                                  in->args.size() > 2 ? in->args[2].as_double() : 0.0);
        if (iequals(t, "IFCSURFACEOFLINEAREXTRUSION")) {
            // (SweptCurve=IfcProfileDef, Position, ExtrudedDirection, Depth)
            std::shared_ptr<Curve> prof = profile_curve(ref_arg(*in, 0));
            Vec3 d = dir(ref_arg(*in, 2));
            double depth = in->args.size() > 3 ? in->args[3].as_double() : 1.0;
            if (prof)
                return std::make_shared<LinearExtrusionSurface>(prof, d, depth);
            return nullptr;
        }
        if (iequals(t, "IFCSURFACEOFREVOLUTION")) {
            // (SweptCurve, Position, AxisPosition=IfcAxis1Placement)
            std::shared_ptr<Curve> prof = profile_curve(ref_arg(*in, 0));
            const Instance *ax = inst(ref_arg(*in, 2));
            Vec3 loc = ax ? point(ref_arg(*ax, 0)) : Vec3{0, 0, 0};
            Vec3 ad = ax ? dir(ref_arg(*ax, 1)) : Vec3{0, 0, 1};
            if (prof)
                return std::make_shared<RevolutionSurface>(prof, loc, ad);
            return nullptr;
        }
        if (iequals(t, "IFCBSPLINESURFACEWITHKNOTS") || iequals(t, "IFCRATIONALBSPLINESURFACEWITHKNOTS"))
            return bspline_surface(*in);
        return nullptr;
    }
    std::shared_ptr<Curve> profile_curve(long profile_id) {
        // IfcArbitraryOpenProfileDef(.CURVE., $, Curve) -> arg 2 is the curve. Else treat as a curve.
        const Instance *in = inst(profile_id);
        if (!in)
            return nullptr;
        if (iequals(in->type, "IFCARBITRARYOPENPROFILEDEF"))
            return curve(ref_arg(*in, 2));
        return curve(profile_id);
    }
    // IfcBSplineSurfaceWithKnots(UDeg,VDeg,CPgrid,Form,UClosed,VClosed,SI,UMult,VMult,UKnots,VKnots,Spec[,Weights]).
    std::shared_ptr<Surface> bspline_surface(const Instance &in) {
        int ud = in.args.size() > 0 ? (int) in.args[0].i : 1;
        int vd = in.args.size() > 1 ? (int) in.args[1].i : 1;
        std::vector<std::vector<Vec3>> grid;
        if (in.args.size() > 2 && in.args[2].is_list())
            for (const Value &row : in.args[2].items)
                grid.push_back(point_list(row));
        int nu = (int) grid.size(), nv = nu ? (int) grid[0].size() : 0;
        std::vector<Vec3> cps;
        cps.reserve(nu * nv);
        for (auto &row : grid)
            for (auto &p : row)
                cps.push_back(p);
        std::vector<int> um = ints(in.args.size() > 7 ? in.args[7] : Value{});
        std::vector<int> vm = ints(in.args.size() > 8 ? in.args[8] : Value{});
        std::vector<double> uk = reals(in.args.size() > 9 ? in.args[9] : Value{});
        std::vector<double> vk = reals(in.args.size() > 10 ? in.args[10] : Value{});
        std::vector<double> w; // flatten the rational weight grid (nu x nv) row-major
        if (iequals(in.type, "IFCRATIONALBSPLINESURFACEWITHKNOTS") && in.args.size() > 11 && in.args[11].is_list())
            for (const Value &row : in.args[11].items)
                if (row.is_list())
                    for (const Value &e : row.items)
                        w.push_back(e.as_double());
        auto s = std::make_shared<BSplineSurface>();
        s->u_degree = ud;
        s->v_degree = vd;
        s->nu = nu;
        s->nv = nv;
        s->ctrl = std::move(cps);
        s->Uu = bspline_detail::expand_knots(uk, um); // compact (knots,mults) -> flat
        s->Uv = bspline_detail::expand_knots(vk, vm);
        s->weights = std::move(w);
        s->u_closed = in.args.size() > 4 && !(in.args[4].kind == adacpp::step::Kind::Enum && in.args[4].s == "F");
        s->v_closed = in.args.size() > 5 && !(in.args[5].kind == adacpp::step::Kind::Enum && in.args[5].s == "F");
        return s;
    }

    OrientedEdgeN oriented_edge(long id) {
        OrientedEdgeN oe;
        const Instance *in = inst(id); // IfcOrientedEdge(*,*,EdgeElement,Orientation)
        if (!in)
            return oe;
        bool orient = !(in->args.size() > 3 && in->args[3].kind == adacpp::step::Kind::Enum && in->args[3].s == "F");
        const Instance *ec = inst(ref_arg(*in, 2)); // IfcEdgeCurve(Start,End,Geometry,SameSense)
        if (!ec)
            return oe;
        const Instance *v0 = inst(ref_arg(*ec, 0)), *v1 = inst(ref_arg(*ec, 1));
        Vec3 p0 = v0 ? point(ref_arg(*v0, 0)) : Vec3{0, 0, 0};
        Vec3 p1 = v1 ? point(ref_arg(*v1, 0)) : Vec3{0, 0, 0};
        bool same = !(ec->args.size() > 3 && ec->args[3].kind == adacpp::step::Kind::Enum && ec->args[3].s == "F");
        oe.geometry = curve(ref_arg(*ec, 2));
        oe.e_start = p0;
        oe.e_end = p1;
        oe.same_sense = same;
        oe.orientation = orient;
        oe.start = orient ? (same ? p0 : p1) : (same ? p1 : p0);
        oe.end = orient ? (same ? p1 : p0) : (same ? p0 : p1);
        return oe;
    }
    std::shared_ptr<LoopN> loop(long id) {
        auto lp = std::make_shared<LoopN>();
        const Instance *in = inst(id);
        if (!in)
            return nullptr;
        if (iequals(in->type, "IFCPOLYLOOP")) {
            lp->is_poly = true;
            lp->polygon = point_list(in->args.empty() ? Value{} : in->args[0]);
            return lp->polygon.empty() ? nullptr : lp;
        }
        if (iequals(in->type, "IFCEDGELOOP")) {
            if (in->args.empty() || !in->args[0].is_list())
                return nullptr;
            for (const Value &oeref : in->args[0].items)
                if (oeref.is_ref())
                    lp->edges.push_back(oriented_edge(oeref.i));
            return lp->edges.empty() ? nullptr : lp;
        }
        return nullptr;
    }
    std::shared_ptr<FaceSurfaceN> face(long id) {
        const Instance *in = inst(id); // IfcAdvancedFace(Bounds, FaceSurface, SameSense)
        if (!in || !iequals(in->type, "IFCADVANCEDFACE"))
            return nullptr;
        auto fc = std::make_shared<FaceSurfaceN>();
        fc->surface = surface(ref_arg(*in, 1));
        if (!fc->surface)
            return nullptr;
        fc->same_sense = !(in->args.size() > 2 && in->args[2].kind == adacpp::step::Kind::Enum && in->args[2].s == "F");
        if (in->args.empty() || !in->args[0].is_list())
            return nullptr;
        for (const Value &bref : in->args[0].items) {
            const Instance *b = inst(bref.i); // IfcFaceOuterBound/IfcFaceBound(Bound, Orientation)
            if (!b)
                return nullptr;
            FaceBoundN fb;
            fb.loop = loop(ref_arg(*b, 0));
            if (!fb.loop)
                return nullptr;
            fb.orientation = !(b->args.size() > 1 && b->args[1].kind == adacpp::step::Kind::Enum && b->args[1].s == "F");
            // outer bound first
            if (iequals(b->type, "IFCFACEOUTERBOUND"))
                fc->bounds.insert(fc->bounds.begin(), fb);
            else
                fc->bounds.push_back(fb);
        }
        return fc->bounds.empty() ? nullptr : fc;
    }
    // An IfcProfileDef -> a planar profile FaceSurfaceN (local XY, z=0, poly loop) for an extrusion's
    // SweptArea. Supports IfcRectangleProfileDef + IfcArbitraryClosedProfileDef (polygonal OuterCurve);
    // returns null for parametric/curved profiles not yet covered (-> the product is skipped -> OCC).
    std::shared_ptr<FaceSurfaceN> profile_face(long pid) {
        const Instance *in = inst(pid);
        if (!in)
            return nullptr;
        std::vector<Vec3> poly;
        if (iequals(in->type, "IFCRECTANGLEPROFILEDEF")) {
            // (ProfileType, ProfileName, Position, XDim, YDim) — centred on Position (or origin if $).
            double hx = (in->args.size() > 3 ? in->args[3].as_double() : 0.0) / 2;
            double hy = (in->args.size() > 4 ? in->args[4].as_double() : 0.0) / 2;
            if (hx <= 0 || hy <= 0)
                return nullptr;
            poly = {{-hx, -hy, 0}, {hx, -hy, 0}, {hx, hy, 0}, {-hx, hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCARBITRARYCLOSEDPROFILEDEF")) {
            poly = curve_points2d(ref_arg(*in, 2)); // (ProfileType, ProfileName, OuterCurve)
        } else if (iequals(in->type, "IFCISHAPEPROFILEDEF")) {
            // (.., Position, OverallWidth, OverallDepth, WebThickness, FlangeThickness, FilletRadius, ...)
            // Centred on the bounding box; fillets ignored (sharp corners). Outline CCW from bottom-right.
            double bx = ad(in, 3) / 2, hy = ad(in, 4) / 2, wx = ad(in, 5) / 2, tf = ad(in, 6);
            if (bx <= 0 || hy <= 0 || wx <= 0 || tf <= 0)
                return nullptr;
            double fy = hy - tf;
            poly = {{bx, -hy, 0},  {bx, -fy, 0},  {wx, -fy, 0},  {wx, fy, 0},   {bx, fy, 0},   {bx, hy, 0},
                    {-bx, hy, 0},  {-bx, fy, 0},  {-wx, fy, 0},  {-wx, -fy, 0}, {-bx, -fy, 0}, {-bx, -hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCTSHAPEPROFILEDEF")) {
            // (.., Position, Depth, FlangeWidth, WebThickness, FlangeThickness, ...). Flange at +y (top),
            // web hangs to -y; centred on the bounding box.
            double hy = ad(in, 3) / 2, fx = ad(in, 4) / 2, wx = ad(in, 5) / 2, tf = ad(in, 6);
            if (hy <= 0 || fx <= 0 || wx <= 0 || tf <= 0)
                return nullptr;
            double fy = hy - tf;
            poly = {{wx, -hy, 0}, {wx, fy, 0},  {fx, fy, 0},   {fx, hy, 0},
                    {-fx, hy, 0}, {-fx, fy, 0}, {-wx, fy, 0},  {-wx, -hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCCIRCLEPROFILEDEF")) {
            // (.., Position, Radius). 64-gon (vertices on the axes -> bbox == analytic 2R).
            double rad = ad(in, 3);
            if (rad <= 0)
                return nullptr;
            const int N = 64;
            for (int i = 0; i < N; ++i) {
                double a = 2.0 * PI * i / N;
                poly.push_back({rad * std::cos(a), rad * std::sin(a), 0});
            }
            apply_placement2d(ref_arg(*in, 2), poly);
        } else {
            return nullptr;
        }
        if (poly.size() < 3)
            return nullptr;
        auto prof = std::make_shared<FaceSurfaceN>();
        prof->surface = std::make_shared<PlaneSurface>(Frame{});
        prof->same_sense = true;
        auto lp = std::make_shared<LoopN>();
        lp->is_poly = true;
        lp->polygon = std::move(poly);
        FaceBoundN fb;
        fb.loop = lp;
        fb.orientation = true;
        prof->bounds.push_back(fb);
        return prof;
    }
    // OuterCurve (IfcPolyline of IfcCartesianPoint, or IfcIndexedPolyCurve over an IfcCartesianPointList2D)
    // -> the profile's 2D polygon (z=0), drop the closing duplicate point.
    std::vector<Vec3> curve_points2d(long cid) {
        std::vector<Vec3> pts;
        const Instance *in = inst(cid);
        if (!in)
            return pts;
        if (iequals(in->type, "IFCPOLYLINE")) {
            if (!in->args.empty() && in->args[0].is_list())
                for (const Value &pr : in->args[0].items)
                    if (pr.is_ref()) {
                        Vec3 p = point(pr.i);
                        pts.push_back({p.x, p.y, 0});
                    }
        } else if (iequals(in->type, "IFCINDEXEDPOLYCURVE")) {
            const Instance *pl = inst(ref_arg(*in, 0)); // IfcCartesianPointList2D(CoordList)
            if (pl && !pl->args.empty() && pl->args[0].is_list())
                for (const Value &row : pl->args[0].items)
                    if (row.is_list() && row.items.size() >= 2)
                        pts.push_back({row.items[0].as_double(), row.items[1].as_double(), 0});
        }
        if (pts.size() > 1 && (pts.front() - pts.back()).norm() < 1e-9)
            pts.pop_back();
        return pts;
    }
    // Translate/rotate a profile polygon by an IfcAxis2Placement2D (Location, RefDirection); $ = identity.
    void apply_placement2d(long pid, std::vector<Vec3> &poly) {
        const Instance *in = inst(pid);
        if (!in)
            return;
        Vec3 loc = (in->args.size() > 0 && in->args[0].is_ref()) ? point(in->args[0].i) : Vec3{0, 0, 0};
        double cx = 1, sx = 0;
        if (in->args.size() > 1 && in->args[1].is_ref()) {
            Vec3 rd = dir(in->args[1].i);
            double n = std::hypot(rd.x, rd.y);
            if (n > 1e-12) {
                cx = rd.x / n;
                sx = rd.y / n;
            }
        }
        for (Vec3 &p : poly) {
            double x = p.x, y = p.y;
            p = {loc.x + cx * x - sx * y, loc.y + sx * x + cx * y, 0};
        }
    }
    // One procedural solid per root. False (+ flags mixed_) if a brep or a DIFFERENT solid is already
    // present; mapped instances of the same solid (same id) are allowed (their transforms accumulate).
    bool claim_solid(NgeomRoot &root, long id) {
        if (!root.faces.empty()) {
            mixed_ = true;
            return false;
        }
        if ((root.extrusion || root.revolve) && solid_src_ != id) {
            mixed_ = true;
            return false;
        }
        solid_src_ = id;
        return true;
    }
    // Append an IfcShapeRepresentation item's geometry to root: IfcAdvancedBrep, IfcFacetedBrep,
    // IfcShellBasedSurfaceModel, IfcExtrudedAreaSolid, IfcRevolvedAreaSolid, IfcMappedItem.
    void resolve_item(long id, NgeomRoot &root) {
        const Instance *in = inst(id);
        if (!in)
            return;
        std::string_view t = in->type;
        if (iequals(t, "IFCADVANCEDBREP") || iequals(t, "IFCFACETEDBREP")) {
            if (root.extrusion || root.revolve) { // brep mixed with a procedural solid -> skip
                mixed_ = true;
                return;
            }
            const Instance *shell = inst(ref_arg(*in, 0)); // IfcClosedShell(CfsFaces)
            if (shell && !shell->args.empty() && shell->args[0].is_list())
                for (const Value &fref : shell->args[0].items)
                    if (fref.is_ref())
                        if (auto f = face(fref.i))
                            root.faces.push_back(f);
        } else if (iequals(t, "IFCEXTRUDEDAREASOLID")) {
            // (SweptArea, Position, ExtrudedDirection, Depth). One solid per root; a 2nd distinct solid
            // (or brep already present) => mixed -> skip. Mapped instances share the same id.
            if (!claim_solid(root, id))
                return;
            auto prof = profile_face(ref_arg(*in, 0));
            if (!prof)
                return; // unsupported profile -> product yields no geometry -> OCC fallback
            auto ex = std::make_shared<ExtrusionN>();
            ex->profile = prof;
            ex->frame = (in->args.size() > 1 && in->args[1].is_ref()) ? axis2(in->args[1].i) : Frame{};
            ex->direction = (in->args.size() > 2 && in->args[2].is_ref()) ? dir(in->args[2].i) : Vec3{0, 0, 1};
            ex->depth = in->args.size() > 3 ? in->args[3].as_double() : 0.0;
            root.extrusion = ex;
        } else if (iequals(t, "IFCREVOLVEDAREASOLID")) {
            // (SweptArea, Position, Axis=IfcAxis1Placement, Angle[rad]). Axis is local to Position.
            if (!claim_solid(root, id))
                return;
            auto prof = profile_face(ref_arg(*in, 0));
            if (!prof)
                return;
            auto rv = std::make_shared<RevolveN>();
            rv->profile = prof;
            rv->frame = (in->args.size() > 1 && in->args[1].is_ref()) ? axis2(in->args[1].i) : Frame{};
            const Instance *ax = inst(ref_arg(*in, 2)); // IfcAxis1Placement(Location, Axis)
            rv->axis_origin = (ax && ax->args.size() > 0 && ax->args[0].is_ref()) ? point(ax->args[0].i) : Vec3{0, 0, 0};
            rv->axis_dir = (ax && ax->args.size() > 1 && ax->args[1].is_ref()) ? dir(ax->args[1].i) : Vec3{0, 0, 1};
            rv->angle = in->args.size() > 3 ? in->args[3].as_double() : 0.0;
            root.revolve = rv;
        } else if (iequals(t, "IFCMAPPEDITEM")) {
            // (MappingSource=IfcRepresentationMap, MappingTarget=IfcCartesianTransformationOperator3D)
            const Instance *rm = inst(ref_arg(*in, 0));
            if (rm && rm->args.size() > 1) {
                // RepresentationMap.MappedRepresentation (arg 1) -> IfcShapeRepresentation.Items.
                const Instance *sr = inst(ref_arg(*rm, 1));
                if (sr && sr->args.size() > 3 && sr->args[3].is_list())
                    for (const Value &it : sr->args[3].items)
                        if (it.is_ref())
                            resolve_item(it.i, root); // appends the brep faces (shared geometry)
            }
            // transform operator -> a world-placement matrix appended to root.transforms.
            std::array<float, 16> M = op_matrix(ref_arg(*in, 1));
            root.transforms.push_back(M);
        }
    }
    // IfcCartesianTransformationOperator3D(Axis1,Axis2,LocalOrigin,Scale,Axis3[,Scale2,Scale3])
    // -> column-major glTF M. Scale (arg3) scales all axes; the 3DNonUniform subtype adds Scale2(arg5),
    // Scale3(arg6) so the axes scale independently. Default scale 1.
    std::array<float, 16> op_matrix(long id) {
        std::array<float, 16> M = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        const Instance *in = inst(id);
        if (!in)
            return M;
        auto unit = [](Vec3 v) {
            double n = v.norm();
            return n > 1e-12 ? Vec3{v.x / n, v.y / n, v.z / n} : v;
        };
        // The transform operator's axes are DIRECTIONs (orientation only) — normalize before scaling,
        // else a non-unit ratio like (1,1,0) leaks an extra |.|=sqrt(2) into the scale.
        Vec3 ax{1, 0, 0}, ay{0, 1, 0}, az{0, 0, 1}, o{0, 0, 0};
        if (in->args.size() > 0 && in->args[0].is_ref())
            ax = unit(dir(in->args[0].i));
        if (in->args.size() > 1 && in->args[1].is_ref())
            ay = unit(dir(in->args[1].i));
        if (in->args.size() > 2 && in->args[2].is_ref())
            o = point(in->args[2].i);
        if (in->args.size() > 4 && in->args[4].is_ref())
            az = unit(dir(in->args[4].i));
        auto scale_at = [&](size_t i, double dflt) {
            return (i < in->args.size() && (in->args[i].kind == adacpp::step::Kind::Real ||
                                            in->args[i].kind == adacpp::step::Kind::Int))
                       ? in->args[i].as_double()
                       : dflt;
        };
        double s1 = scale_at(3, 1.0), s2 = s1, s3 = s1;
        if (iequals(in->type, "IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM")) {
            s2 = scale_at(5, 1.0);
            s3 = scale_at(6, 1.0);
        }
        ax = ax * s1;
        ay = ay * s2;
        az = az * s3;
        // column-major glTF: col0=ax, col1=ay, col2=az, col3=origin
        M = {(float) ax.x, (float) ax.y, (float) ax.z, 0.0f, (float) ay.x, (float) ay.y, (float) ay.z, 0.0f,
             (float) az.x, (float) az.y, (float) az.z, 0.0f, (float) o.x,  (float) o.y,  (float) o.z,  1.0f};
        return M;
    }
    // 4x4 column-major (glTF) multiply R = A*B.
    static std::array<float, 16> mat_mul(const std::array<float, 16> &A, const std::array<float, 16> &B) {
        std::array<float, 16> R{};
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) {
                float s = 0;
                for (int k = 0; k < 4; ++k)
                    s += A[k * 4 + r] * B[c * 4 + k];
                R[c * 4 + r] = s;
            }
        return R;
    }
    static bool is_identity(const std::array<float, 16> &M) {
        static const std::array<float, 16> I = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        for (int i = 0; i < 16; ++i)
            if (std::abs(M[i] - I[i]) > 1e-9f)
                return false;
        return true;
    }
    // Orthonormal 3x3 (rotation only, no scale/shear) — column-major glTF columns are unit length.
    static bool is_rigid(const std::array<float, 16> &M) {
        double c0 = std::sqrt((double) M[0] * M[0] + (double) M[1] * M[1] + (double) M[2] * M[2]);
        double c1 = std::sqrt((double) M[4] * M[4] + (double) M[5] * M[5] + (double) M[6] * M[6]);
        double c2 = std::sqrt((double) M[8] * M[8] + (double) M[9] * M[9] + (double) M[10] * M[10]);
        return std::abs(c0 - 1) < 1e-4 && std::abs(c1 - 1) < 1e-4 && std::abs(c2 - 1) < 1e-4;
    }
    // IfcAxis2Placement3D -> column-major glTF matrix.
    std::array<float, 16> axis2_mat(long id) {
        Frame f = axis2(id);
        return {(float) f.x.x, (float) f.x.y, (float) f.x.z, 0.0f, (float) f.y.x, (float) f.y.y, (float) f.y.z, 0.0f,
                (float) f.z.x, (float) f.z.y, (float) f.z.z, 0.0f, (float) f.o.x, (float) f.o.y, (float) f.o.z, 1.0f};
    }
    // IfcLocalPlacement(PlacementRelTo, RelativePlacement) -> world matrix (recurse up the chain).
    std::array<float, 16> object_placement(long id, int depth = 0) {
        static const std::array<float, 16> I = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        const Instance *in = inst(id);
        if (!in || depth > 64 || !iequals(in->type, "IFCLOCALPLACEMENT"))
            return I;
        std::array<float, 16> rel = (in->args.size() > 1 && in->args[1].is_ref()) ? axis2_mat(in->args[1].i) : I;
        if (in->args.size() > 0 && in->args[0].is_ref())
            return mat_mul(object_placement(in->args[0].i, depth + 1), rel);
        return rel;
    }
};

} // namespace adacpp::ifc_read
