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
#include <unordered_set>
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
        // A geometric product is any IfcProduct whose Representation (arg 6 in every IfcProduct
        // subtype) refs an IfcProductDefinitionShape. Resolving by representation — instead of the
        // old cheap name allowlist, which missed subtypes like IfcSanitaryTerminal / MEP / furniture
        // terminals — catches every product the ifcopenshell reader would, with no schema.
        std::unordered_set<long> pds; // IfcProductDefinitionShape ids
        std::string scratch;
        for (long id : idx_.ids)
            if (iequals(type_of(id, scratch), "IFCPRODUCTDEFINITIONSHAPE"))
                pds.insert(id);
        std::vector<long> roots;
        for (long id : idx_.ids) {
            std::string_view t = type_of(id, scratch);
            if (is_product_type(t)) { // fast path for the common structural types (no full parse)
                roots.push_back(id);
                continue;
            }
            const Instance *in = inst(id);
            if (in && in->args.size() > 6 && in->args[6].is_ref() && pds.count(in->args[6].i))
                roots.push_back(id);
        }
        return roots;
    }

    // The geometry-bearing IfcElement subtypes (cheap type-name test — avoids parsing every entity on
    // a giant file). Anything not listed (or with unrepresentable geometry) is skipped -> OCC fallback.
    static bool is_product_type(std::string_view t) {
        static const char *kinds[] = {
            "IFCBUILDINGELEMENTPROXY", "IFCMECHANICALFASTENER", "IFCELEMENTASSEMBLY", "IFCFASTENER",
            "IFCDISCRETEACCESSORY", "IFCBUILDINGELEMENTPART", "IFCBEAM", "IFCCOLUMN", "IFCMEMBER", "IFCPLATE",
            "IFCWALL", "IFCWALLSTANDARDCASE", "IFCSLAB", "IFCFOOTING", "IFCPILE", "IFCROOF", "IFCSTAIR",
            "IFCSTAIRFLIGHT", "IFCRAMP", "IFCRAMPFLIGHT", "IFCRAILING", "IFCCOVERING", "IFCCURTAINWALL", "IFCDOOR",
            "IFCWINDOW", "IFCCHIMNEY", "IFCSHADINGDEVICE", "IFCPIPESEGMENT", "IFCPIPEFITTING", "IFCDUCTSEGMENT",
            "IFCDUCTFITTING", "IFCFLOWSEGMENT", "IFCFLOWFITTING", "IFCFLOWTERMINAL", "IFCFLOWCONTROLLER",
            "IFCDISTRIBUTIONELEMENT", "IFCENERGYCONVERSIONDEVICE", "IFCREINFORCINGBAR", "IFCREINFORCINGMESH",
            "IFCTENDON", "IFCTENDONANCHOR", "IFCFURNISHINGELEMENT", "IFCFURNITURE", "IFCSYSTEMFURNITUREELEMENT",
            "IFCCIVILELEMENT", "IFCGEOGRAPHICELEMENT",
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

    // Radians per model plane-angle unit. IfcSIUnit RADIAN => 1; an IfcConversionBasedUnit for
    // PLANEANGLEUNIT (DEGREE/GRAD) carries the exact ratio in its ConversionFactor
    // (IfcMeasureWithUnit.ValueComponent) -> use it so trimmed-circle parameters expressed in
    // degrees (curve-parameters-in-degrees.ifc) sample the same arc as the radian model.
    double plane_angle_scale() {
        if (angle_scale_ > 0)
            return angle_scale_;
        angle_scale_ = 1.0; // default: radian
        std::string scratch;
        for (long id : idx_.ids) {
            std::string_view t = type_of(id, scratch);
            if (!iequals(t, "IFCCONVERSIONBASEDUNIT"))
                continue;
            const Instance *in = inst(id); // (Dimensions, UnitType, Name, ConversionFactor)
            if (!in || in->args.size() < 4 ||
                !(in->args[1].kind == adacpp::step::Kind::Enum && in->args[1].s == "PLANEANGLEUNIT"))
                continue;
            const Instance *mwu = inst(ref_arg(*in, 3)); // IfcMeasureWithUnit(ValueComponent, UnitComponent)
            if (mwu)                                     // ValueComponent = IfcPlaneAngleMeasure(x): [Keyword, List[Real]]
                for (const Value &a : mwu->args) {
                    if (numeric(a)) {
                        angle_scale_ = a.as_double();
                        break;
                    }
                    if (a.is_list() && !a.items.empty() && numeric(a.items[0])) {
                        angle_scale_ = a.items[0].as_double();
                        break;
                    }
                }
            break;
        }
        return angle_scale_;
    }

    // Presentation colour. IfcStyledItem(Item, Styles, Name): Item=arg0 is the geometry rep item id
    // (the same id resolve_item consumes), Styles=arg1. Build a one-time map rep-item-id -> rgba by
    // resolving each styled item's style tree to its IfcColourRgb (+ transparency). Persistent across
    // products (clear_cache() must not wipe it). Mirrors the STEP reader's colour_map_/find_colour.
    void build_colour_map() {
        if (colour_map_built_)
            return;
        colour_map_built_ = true;
        std::string scratch;
        for (long id : idx_.ids) {
            if (!iequals(type_of(id, scratch), "IFCSTYLEDITEM"))
                continue;
            const Instance *si = inst(id);
            if (!si || si->args.empty() || !si->args[0].is_ref())
                continue;
            std::array<float, 4> rgba{0.5f, 0.5f, 0.5f, 1.0f};
            if (resolve_styles_color(si->args.size() > 1 ? si->args[1] : Value{}, rgba))
                colour_map_[si->args[0].i] = rgba;
        }
    }
    // BFS over a Styles ref-tree to the first IfcColourRgb (r/g/b = args[1..3]) + alpha from the
    // IfcSurfaceStyleShading/Rendering transparency (arg1). Walks IfcSurfaceStyle (styles=arg2),
    // IfcPresentationStyleAssignment (2x3 wrapper), etc. by collecting every ref arg — so it handles
    // IFC2x3 and IFC4/4x3 without schema-specific level walking.
    bool resolve_styles_color(const Value &styles, std::array<float, 4> &rgba) {
        std::vector<long> stack;
        auto push_refs = [&](const Value &v) {
            if (v.is_ref())
                stack.push_back(v.i);
            else if (v.is_list())
                for (const Value &e : v.items)
                    if (e.is_ref())
                        stack.push_back(e.i);
        };
        push_refs(styles);
        std::unordered_set<long> seen;
        bool found = false;
        int guard = 0;
        while (!stack.empty() && guard++ < 500) {
            long id = stack.back();
            stack.pop_back();
            if (!seen.insert(id).second)
                continue;
            const Instance *in = inst(id);
            if (!in)
                continue;
            std::string_view t = in->type;
            if (iequals(t, "IFCCOLOURRGB")) {
                rgba[0] = (float) ad(in, 1);
                rgba[1] = (float) ad(in, 2);
                rgba[2] = (float) ad(in, 3);
                found = true;
                continue;
            }
            if (iequals(t, "IFCSURFACESTYLESHADING") || iequals(t, "IFCSURFACESTYLERENDERING")) {
                if (in->args.size() > 1 && numeric(in->args[1]))
                    rgba[3] = 1.0f - (float) in->args[1].as_double(); // Transparency -> alpha
                if (!in->args.empty() && in->args[0].is_ref())
                    stack.push_back(in->args[0].i); // SurfaceColour
                continue;
            }
            for (const Value &a : in->args)
                push_refs(a); // IfcSurfaceStyle / PresentationStyleAssignment / StyledRepresentation ...
        }
        return found;
    }
    // Look up a rep item's colour, unwrapping an IfcMappedItem to its mapped representation's items
    // (the style may sit on the shared inner geometry).
    bool find_item_colour(long iid, std::array<float, 4> &rgba, int depth = 0) {
        auto it = colour_map_.find(iid);
        if (it != colour_map_.end()) {
            rgba = it->second;
            return true;
        }
        if (depth > 4)
            return false;
        const Instance *in = inst(iid);
        if (!in)
            return false;
        if (iequals(in->type, "IFCMAPPEDITEM")) {
            const Instance *rm = inst(ref_arg(*in, 0)); // IfcRepresentationMap
            if (rm && rm->args.size() > 1) {
                const Instance *sr = inst(ref_arg(*rm, 1)); // MappedRepresentation
                if (sr && sr->args.size() > 3 && sr->args[3].is_list())
                    for (const Value &m : sr->args[3].items)
                        if (m.is_ref() && find_item_colour(m.i, rgba, depth + 1))
                            return true;
            }
        } else if (iequals(in->type, "IFCBOOLEANCLIPPINGRESULT") || iequals(in->type, "IFCBOOLEANRESULT")) {
            // (Operator, FirstOperand, SecondOperand) — the style usually rides the base operand.
            for (int ai : {1, 2}) {
                long op = ref_arg(*in, (size_t) ai);
                if (op > 0 && find_item_colour(op, rgba, depth + 1))
                    return true;
            }
        }
        return false;
    }

    // Spatial hierarchy. IfcRelContainedInSpatialStructure(.., RelatedElements=arg4 list,
    // RelatingStructure=arg5) puts products in a storey/space; IfcRelAggregates(.., RelatingObject=arg4,
    // RelatedObjects=arg5 list) nests storey->building->site->project (and sub-assemblies). Build both
    // reverse maps once (persistent; clear_cache must not wipe them) so a product can walk up to root.
    void build_rel_maps() {
        if (rel_maps_built_)
            return;
        rel_maps_built_ = true;
        std::string scratch;
        for (long id : idx_.ids) {
            std::string_view t = type_of(id, scratch);
            if (iequals(t, "IFCRELCONTAINEDINSPATIALSTRUCTURE")) {
                const Instance *in = inst(id);
                if (!in || in->args.size() < 6)
                    continue;
                long structure = ref_arg(*in, 5);
                if (in->args[4].is_list())
                    for (const Value &e : in->args[4].items)
                        if (e.is_ref())
                            contained_of_[e.i] = structure;
            } else if (iequals(t, "IFCRELAGGREGATES")) {
                const Instance *in = inst(id);
                if (!in || in->args.size() < 6)
                    continue;
                long parent = ref_arg(*in, 4);
                if (in->args[5].is_list())
                    for (const Value &e : in->args[5].items)
                        if (e.is_ref())
                            parent_of_[e.i] = parent;
            }
        }
    }
    // Root-first (id, name) levels from IfcProject down to the product — the assembly path the adapy
    // consumer turns into the scene tree. One path (IFC products are single-instance here).
    std::vector<std::pair<int, std::string>> product_path(long pid, const Instance &p) {
        build_rel_maps();
        auto next_up = [&](long id) -> long {
            auto a = parent_of_.find(id);
            if (a != parent_of_.end())
                return a->second;
            auto c = contained_of_.find(id);
            return c != contained_of_.end() ? c->second : 0;
        };
        std::vector<std::pair<int, std::string>> path;
        path.push_back({(int) pid, name_or_guid(p)}); // deepest level = the product itself
        std::unordered_set<long> seen{pid};
        long up = next_up(pid);
        int guard = 0;
        while (up > 0 && seen.insert(up).second && guard++ < 64) {
            const Instance *s = inst(up);
            path.push_back({(int) up, s ? name_or_guid(*s) : std::string()});
            up = next_up(up);
        }
        std::reverse(path.begin(), path.end()); // root-first
        return path;
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
        root.id = name_or_guid(*p);
        // IfcBuildingElementProxy.Representation (arg 6) -> IfcProductDefinitionShape.Representations
        // (arg 2) -> IfcShapeRepresentation.Items (arg 3).
        long rep = ref_arg(*p, 6);
        const Instance *pds = inst(rep);
        if (!pds || pds->args.size() < 3) {
            // No (resolvable) Representation -> an intentionally geometry-less container/spatial product
            // (e.g. an IfcElementAssembly aggregating rebars, reached via the is_product_type fast path).
            // Recognized, not an unsupported skip -> the stream won't waste an OCC fallback on it.
            root.recognized_empty = true;
            return root;
        }
        // IfcShapeRepresentation: (ContextOfItems, RepresentationIdentifier, RepresentationType, Items).
        // A product may carry several reps: "Body"/"Facetation"/"Tessellation" are the 3D shape;
        // "Axis" is a reference curve (render only when there is NO body — an alignment segment IS
        // its axis); "FootPrint"/"Annotation"/"Profile"/"Plan"/"Box" are 2D and never rendered in
        // 3D. Pick the right class so a beam's Axis polyline doesn't collide with its Body extrusion,
        // and an alignment takes its 3D gradient Axis rather than its 2D FootPrint.
        auto rep_id = [](const Instance *sr) -> std::string_view {
            if (sr->args.size() > 1 && sr->args[1].kind == adacpp::step::Kind::Str)
                return sr->args[1].s;
            return {};
        };
        auto is_2d_only = [](std::string_view id) {
            return id == "FootPrint" || id == "Annotation" || id == "Profile" || id == "Plan" ||
                   id == "Box";
        };
        auto is_axis = [](std::string_view id) { return id == "Axis"; };
        bool has_body = false;
        for (const Value &srref : pds->args[2].items) {
            const Instance *sr = inst(srref.i);
            if (sr && sr->args.size() >= 4) {
                std::string_view id = rep_id(sr);
                if (!is_2d_only(id) && !is_axis(id)) {
                    has_body = true;
                    break;
                }
            }
        }
        std::vector<long> item_ids; // the rep-item ids (IfcStyledItem keys colour on these)
        for (const Value &srref : pds->args[2].items) {
            const Instance *sr = inst(srref.i);
            if (!sr || sr->args.size() < 4)
                continue;
            std::string_view id = rep_id(sr);
            if (is_2d_only(id))
                continue; // 2D reps are never rendered in 3D
            if (has_body && is_axis(id))
                continue; // a body exists -> the Axis is only a reference line, skip it
            for (const Value &item : sr->args[3].items) {
                if (!item.is_ref())
                    continue;
                item_ids.push_back(item.i);
                resolve_item(item.i, root);
            }
        }
        // Presentation colour (IfcStyledItem -> IfcSurfaceStyle -> IfcColourRgb) — keyed on the rep
        // item id, so it rides the same StepRootMeta.has_color/color rails the STEP path already uses.
        build_colour_map();
        if (!colour_map_.empty()) {
            std::array<float, 4> rgba;
            long keys[] = {solid_src_, 0};
            for (long iid : item_ids)
                if (find_item_colour(iid, rgba)) {
                    root.has_color = true;
                    root.cr = rgba[0];
                    root.cg = rgba[1];
                    root.cb = rgba[2];
                    root.ca = rgba[3];
                    break;
                }
            if (!root.has_color)
                for (long k : keys)
                    if (k > 0 && find_item_colour(k, rgba)) {
                        root.has_color = true;
                        root.cr = rgba[0];
                        root.cg = rgba[1];
                        root.cb = rgba[2];
                        root.ca = rgba[3];
                        break;
                    }
        }
        // Spatial-structure path (Project -> Site -> Storey -> product), for the scene tree. Only emit
        // when there's a real ancestor chain (size>1); a bare product level carries no hierarchy.
        {
            auto path = product_path(pid, *p);
            if (path.size() > 1)
                root.instance_paths.push_back(std::move(path));
        }
        // A product mixing >1 distinct solid (or brep+procedural) doesn't fit the single-solid root
        // model — leave it for OCC rather than emit partial geometry.
        if (mixed_) {
            root.faces.clear();
            root.extrusion = nullptr;
            root.revolve = nullptr;
            root.boolean = nullptr;
        }
        // Compose the product's world placement (IfcLocalPlacement chain) onto every instance — the
        // geometry rep is in the element's local frame; ObjectPlacement positions it in the world.
        if (!root.faces.empty() || root.extrusion || root.revolve || root.boolean) {
            std::array<float, 16> objp = object_placement(ref_arg(*p, 5)); // ObjectPlacement = arg 5
            if (!is_identity(objp)) {
                if (root.transforms.empty())
                    root.transforms.push_back(objp);
                else
                    for (auto &m : root.transforms)
                        m = mat_mul(objp, m);
            }
        }
        // A revolve/boolean under a non-rigid (scale/shear) instance distorts in a way the analytic
        // forms can't carry -> drop it to OCC rather than emit wrong geometry.
        if (root.revolve || root.boolean)
            for (const auto &m : root.transforms)
                if (!is_rigid(m)) {
                    root.revolve = nullptr;
                    root.boolean = nullptr;
                    break;
                }
        return root;
    }

    // The product's IFC GlobalId (arg 0 of any rooted entity). Empty when unparsable.
    std::string product_guid(long pid) {
        const Instance *p = inst(pid);
        if (p && !p->args.empty() && p->args[0].kind == adacpp::step::Kind::Str)
            return std::string(p->args[0].s);
        return {};
    }

    // Drop the statement/surface caches — called between products by the streaming
    // per-product consumer (IfcNgeomStream) so memory stays bounded on large files.
    void clear_cache() {
        cache_.clear();
        surf_cache_.clear();
    }

private:
    const StreamIndex &idx_;
    std::unordered_map<long, std::pair<std::string, Instance>> cache_;
    std::unordered_map<long, std::shared_ptr<Surface>> surf_cache_;
    std::string pread_scratch_;
    long solid_src_ = 0;      // entity id of the one solid this product carries (mapped instances share it)
    bool mixed_ = false;      // product has >1 distinct solid / mixes brep+procedural -> skip (OCC)
    double angle_scale_ = 0.0; // radians per model plane-angle unit (0 => not yet computed)
    std::unordered_map<long, std::array<float, 4>> colour_map_; // rep-item id -> rgba (IfcStyledItem)
    bool colour_map_built_ = false;
    std::unordered_map<long, long> contained_of_; // product -> containing spatial element (IfcRelContained…)
    std::unordered_map<long, long> parent_of_;    // child -> aggregating parent (IfcRelAggregates)
    bool rel_maps_built_ = false;

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
    // Display/picking id for a rooted entity: its Name (arg2) if set, else its GlobalId (arg0). Rebar
    // and assemblies frequently carry only a GlobalId (empty Name) -> naming the tree/picking by Name
    // alone yields blank nodes + empty selection; the Python from_ifc GLB keys on the guid instead, so
    // this mirrors it. Empty only when the entity has neither.
    static std::string name_or_guid(const Instance &in) {
        std::string n = name_of(in);
        if (!n.empty())
            return n;
        if (!in.args.empty() && in.args[0].kind == adacpp::step::Kind::Str)
            return std::string(in.args[0].s);
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
            return std::make_shared<CircleCurve>(axis2(ref_arg(*in, 0)),
                                                 in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCELLIPSE"))
            return std::make_shared<EllipseCurve>(axis2(ref_arg(*in, 0)),
                                                  in->args.size() > 1 ? in->args[1].as_double() : 0.0,
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
            return std::make_shared<CylinderSurface>(axis2(ref_arg(*in, 0)),
                                                     in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCSPHERICALSURFACE"))
            return std::make_shared<SphereSurface>(axis2(ref_arg(*in, 0)),
                                                   in->args.size() > 1 ? in->args[1].as_double() : 0.0);
        if (iequals(t, "IFCTOROIDALSURFACE"))
            return std::make_shared<TorusSurface>(axis2(ref_arg(*in, 0)),
                                                  in->args.size() > 1 ? in->args[1].as_double() : 0.0,
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
        const Instance *in = inst(id);
        if (!in)
            return nullptr;
        // IfcAdvancedFace / IfcFaceSurface (Bounds, FaceSurface, SameSense) carry an analytic surface;
        // a plain IfcFace (Bounds) is a bare polygon — placeholder plane, the tessellator fits the
        // real plane from the poly loop (face-based surface models, shells).
        bool analytic = iequals(in->type, "IFCADVANCEDFACE") || iequals(in->type, "IFCFACESURFACE");
        bool plain = iequals(in->type, "IFCFACE");
        if (!analytic && !plain)
            return nullptr;
        auto fc = std::make_shared<FaceSurfaceN>();
        if (analytic) {
            fc->surface = surface(ref_arg(*in, 1));
            if (!fc->surface)
                return nullptr;
            fc->same_sense =
                !(in->args.size() > 2 && in->args[2].kind == adacpp::step::Kind::Enum && in->args[2].s == "F");
        } else {
            fc->surface = std::make_shared<PlaneSurface>(Frame{});
            fc->same_sense = true;
        }
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
            fb.orientation =
                !(b->args.size() > 1 && b->args[1].kind == adacpp::step::Kind::Enum && b->args[1].s == "F");
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
        } else if (iequals(in->type, "IFCROUNDEDRECTANGLEPROFILEDEF")) {
            // (ProfileType, Name, Position, XDim, YDim, RoundingRadius) — a rectangle whose four
            // corners are quarter-circle arcs of RoundingRadius (matches the adapy Python reader).
            double hx = ad(in, 3) / 2, hy = ad(in, 4) / 2, r = ad(in, 5);
            if (hx <= 0 || hy <= 0)
                return nullptr;
            r = std::min(r, std::min(hx, hy));
            if (r <= 1e-12) {
                poly = {{-hx, -hy, 0}, {hx, -hy, 0}, {hx, hy, 0}, {-hx, hy, 0}};
            } else {
                auto corner = [&](double cx, double cy, double a0) { // quarter arc, centre (cx,cy), CCW
                    int n = std::max(2, (int) std::ceil(0.25 * 64));
                    for (int i = 0; i <= n; ++i) {
                        double t = a0 + (PI / 2) * i / n;
                        poly.push_back({cx + r * std::cos(t), cy + r * std::sin(t), 0});
                    }
                };
                corner(hx - r, -(hy - r), -PI / 2); // bottom-right
                corner(hx - r, hy - r, 0);          // top-right
                corner(-(hx - r), hy - r, PI / 2);  // top-left
                corner(-(hx - r), -(hy - r), PI);   // bottom-left
            }
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCDERIVEDPROFILEDEF")) {
            // (ProfileType, Name, ParentProfile, Operator=IfcCartesianTransformationOperator2D, Label)
            auto base = profile_face(ref_arg(*in, 2));
            if (!base || base->bounds.empty())
                return nullptr;
            double c = 1, s = 0, ox = 0, oy = 0, sc = 1;
            const Instance *op = inst(ref_arg(*in, 3));
            if (op) {
                if (!op->args.empty() && op->args[0].is_ref()) {
                    Vec3 a = dir(op->args[0].i);
                    double nn = std::hypot(a.x, a.y);
                    if (nn > 1e-12) {
                        c = a.x / nn;
                        s = a.y / nn;
                    }
                }
                if (op->args.size() > 2 && op->args[2].is_ref()) {
                    Vec3 o = point(op->args[2].i);
                    ox = o.x;
                    oy = o.y;
                }
                if (op->args.size() > 3 && numeric(op->args[3]))
                    sc = op->args[3].as_double();
            }
            for (auto &b : base->bounds)
                if (b.loop && b.loop->is_poly)
                    for (Vec3 &p : b.loop->polygon) {
                        double x = p.x * sc, y = p.y * sc;
                        p = {ox + c * x - s * y, oy + s * x + c * y, 0};
                    }
            return base;
        } else if (iequals(in->type, "IFCARBITRARYCLOSEDPROFILEDEF")) {
            poly = curve_points2d(ref_arg(*in, 2)); // (ProfileType, ProfileName, OuterCurve)
        } else if (iequals(in->type, "IFCISHAPEPROFILEDEF")) {
            // (.., Position, OverallWidth, OverallDepth, WebThickness, FlangeThickness, FilletRadius, ...)
            // Centred on the bounding box; fillets ignored (sharp corners). Outline CCW from bottom-right.
            double bx = ad(in, 3) / 2, hy = ad(in, 4) / 2, wx = ad(in, 5) / 2, tf = ad(in, 6);
            if (bx <= 0 || hy <= 0 || wx <= 0 || tf <= 0)
                return nullptr;
            double fy = hy - tf;
            poly = {{bx, -hy, 0}, {bx, -fy, 0}, {wx, -fy, 0}, {wx, fy, 0},   {bx, fy, 0},   {bx, hy, 0},
                    {-bx, hy, 0}, {-bx, fy, 0}, {-wx, fy, 0}, {-wx, -fy, 0}, {-bx, -fy, 0}, {-bx, -hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCTSHAPEPROFILEDEF")) {
            // (.., Position, Depth, FlangeWidth, WebThickness, FlangeThickness, ...). Flange at +y (top),
            // web hangs to -y; centred on the bounding box.
            double hy = ad(in, 3) / 2, fx = ad(in, 4) / 2, wx = ad(in, 5) / 2, tf = ad(in, 6);
            if (hy <= 0 || fx <= 0 || wx <= 0 || tf <= 0)
                return nullptr;
            double fy = hy - tf;
            poly = {{wx, -hy, 0}, {wx, fy, 0},  {fx, fy, 0},  {fx, hy, 0},
                    {-fx, hy, 0}, {-fx, fy, 0}, {-wx, fy, 0}, {-wx, -hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCCIRCLEPROFILEDEF")) {
            // (.., Position, Radius). 64-gon (vertices on the axes -> bbox == analytic 2R).
            double rad = ad(in, 3);
            if (rad <= 0)
                return nullptr;
            poly = circle_poly(rad);
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCUSHAPEPROFILEDEF")) {
            // (.., Position, Depth, FlangeWidth, WebThickness, FlangeThickness, ...). Channel opening +x.
            double hy = ad(in, 3) / 2, w = ad(in, 4), tw = ad(in, 5), tf = ad(in, 6);
            if (hy <= 0 || w <= 0 || tw <= 0 || tf <= 0)
                return nullptr;
            double x0 = -w / 2, x1 = w / 2, fy = hy - tf, wx = -w / 2 + tw;
            poly = {{x0, -hy, 0}, {x1, -hy, 0}, {x1, -fy, 0}, {wx, -fy, 0},
                    {wx, fy, 0},  {x1, fy, 0},  {x1, hy, 0},  {x0, hy, 0}};
            apply_placement2d(ref_arg(*in, 2), poly);
        } else if (iequals(in->type, "IFCCIRCLEHOLLOWPROFILEDEF")) {
            // (.., Position, Radius, WallThickness) -> outer + inner circle (void).
            double R = ad(in, 3), tw = ad(in, 4);
            if (R <= 0 || tw <= 0 || tw >= R)
                return nullptr;
            std::vector<Vec3> outer = circle_poly(R), inner = circle_poly(R - tw);
            apply_placement2d(ref_arg(*in, 2), outer);
            apply_placement2d(ref_arg(*in, 2), inner);
            return make_profile(std::move(outer), {std::move(inner)});
        } else if (iequals(in->type, "IFCRECTANGLEHOLLOWPROFILEDEF")) {
            // (.., Position, XDim, YDim, WallThickness, ...) -> outer + inner rectangle (void).
            double hx = ad(in, 3) / 2, hy = ad(in, 4) / 2, t = ad(in, 5);
            if (hx <= 0 || hy <= 0 || t <= 0 || t >= hx || t >= hy)
                return nullptr;
            std::vector<Vec3> outer = {{-hx, -hy, 0}, {hx, -hy, 0}, {hx, hy, 0}, {-hx, hy, 0}};
            std::vector<Vec3> inner = {
                {-hx + t, -hy + t, 0}, {hx - t, -hy + t, 0}, {hx - t, hy - t, 0}, {-hx + t, hy - t, 0}};
            apply_placement2d(ref_arg(*in, 2), outer);
            apply_placement2d(ref_arg(*in, 2), inner);
            return make_profile(std::move(outer), {std::move(inner)});
        } else if (iequals(in->type, "IFCARBITRARYPROFILEDEFWITHVOIDS")) {
            // (.., OuterCurve, InnerCurves[list]) -> outer polygon + void polygons.
            std::vector<Vec3> outer = curve_points2d(ref_arg(*in, 2));
            std::vector<std::vector<Vec3>> holes;
            if (in->args.size() > 3 && in->args[3].is_list())
                for (const Value &cr : in->args[3].items)
                    if (cr.is_ref()) {
                        auto h = curve_points2d(cr.i);
                        if (h.size() >= 3)
                            holes.push_back(std::move(h));
                    }
            return make_profile(std::move(outer), std::move(holes));
        } else {
            return nullptr;
        }
        return make_poly_profile(std::move(poly));
    }
    // Wrap a 2D outer polygon + optional hole polygons (z=0) as a planar profile FaceSurfaceN. Holes are
    // reversed (opposite winding) so the cap triangulates the void and the swept side bands face right.
    static std::shared_ptr<FaceSurfaceN> make_profile(std::vector<Vec3> outer,
                                                      std::vector<std::vector<Vec3>> holes = {}) {
        if (outer.size() < 3)
            return nullptr;
        auto prof = std::make_shared<FaceSurfaceN>();
        prof->surface = std::make_shared<PlaneSurface>(Frame{});
        prof->same_sense = true;
        auto add = [&](std::vector<Vec3> poly) {
            auto lp = std::make_shared<LoopN>();
            lp->is_poly = true;
            lp->polygon = std::move(poly);
            FaceBoundN fb;
            fb.loop = lp;
            fb.orientation = true;
            prof->bounds.push_back(fb);
        };
        add(std::move(outer));
        for (auto &h : holes)
            if (h.size() >= 3) {
                std::reverse(h.begin(), h.end());
                add(std::move(h));
            }
        return prof;
    }
    static std::shared_ptr<FaceSurfaceN> make_poly_profile(std::vector<Vec3> poly) {
        return make_profile(std::move(poly));
    }
    static std::vector<Vec3> circle_poly(double r, int n = 64) {
        std::vector<Vec3> p;
        p.reserve(n);
        for (int i = 0; i < n; ++i) {
            double a = 2.0 * PI * i / n;
            p.push_back({r * std::cos(a), r * std::sin(a), 0});
        }
        return p;
    }
    // A 3-point (start, mid, end) circular arc in the z=0 plane, discretized to a polyline (matches
    // the adapy Python reader's IfcArcIndex -> ArcLine, which the tessellator later rings). Falls
    // back to the chord [a, m, b] when the three points are collinear (degenerate circumcircle).
    static std::vector<Vec3> arc_poly_2d(const Vec3 &a, const Vec3 &m, const Vec3 &b) {
        double ax = a.x, ay = a.y, mx = m.x, my = m.y, bx = b.x, by = b.y;
        double d = 2.0 * (ax * (my - by) + mx * (by - ay) + bx * (ay - my));
        if (std::abs(d) < 1e-12)
            return {a, m, b};
        double a2 = ax * ax + ay * ay, m2 = mx * mx + my * my, b2 = bx * bx + by * by;
        double ux = (a2 * (my - by) + m2 * (by - ay) + b2 * (ay - my)) / d;
        double uy = (a2 * (bx - mx) + m2 * (ax - bx) + b2 * (mx - ax)) / d;
        double r = std::hypot(ax - ux, ay - uy);
        double ta = std::atan2(ay - uy, ax - ux);
        double tm = std::atan2(my - uy, mx - ux);
        double tb = std::atan2(by - uy, bx - ux);
        auto wrap = [](double x) {
            while (x < 0)
                x += 2.0 * PI;
            while (x >= 2.0 * PI)
                x -= 2.0 * PI;
            return x;
        };
        double dab = wrap(tb - ta), dam = wrap(tm - ta);
        double sweep = (dam <= dab) ? dab : dab - 2.0 * PI; // the direction a->b passing through m
        int n = std::max(2, (int) std::ceil(std::abs(sweep) / (2.0 * PI) * 64.0));
        std::vector<Vec3> out;
        out.reserve(n + 1);
        for (int i = 0; i <= n; ++i) {
            double t = ta + sweep * i / n;
            out.push_back({ux + r * std::cos(t), uy + r * std::sin(t), 0});
        }
        return out;
    }
    // OuterCurve (IfcPolyline of IfcCartesianPoint, or IfcIndexedPolyCurve over an IfcCartesianPointList2D)
    // -> the profile's 2D polygon (z=0), drop the closing duplicate point.
    std::vector<Vec3> curve_points2d(long cid) {
        std::vector<Vec3> pts;
        const Instance *in = inst(cid);
        if (!in)
            return pts;
        auto push = [&](const Vec3 &p) {
            if (pts.empty() || (pts.back() - p).norm() > 1e-9)
                pts.push_back(p);
        };
        if (iequals(in->type, "IFCPOLYLINE")) {
            if (!in->args.empty() && in->args[0].is_list())
                for (const Value &pr : in->args[0].items)
                    if (pr.is_ref()) {
                        Vec3 p = point(pr.i);
                        push({p.x, p.y, 0});
                    }
        } else if (iequals(in->type, "IFCINDEXEDPOLYCURVE")) {
            const Instance *pl = inst(ref_arg(*in, 0)); // IfcCartesianPointList2D(CoordList)
            std::vector<Vec3> coords{{0, 0, 0}};        // 1-based: coords[0] is a placeholder
            if (pl && !pl->args.empty() && pl->args[0].is_list())
                for (const Value &row : pl->args[0].items)
                    if (row.is_list() && row.items.size() >= 2)
                        coords.push_back({row.items[0].as_double(), row.items[1].as_double(), 0});
            auto at = [&](long i1) -> Vec3 {
                return (i1 >= 1 && (size_t) i1 < coords.size()) ? coords[i1] : Vec3{0, 0, 0};
            };
            // Segments (arg 1): a list of typed [Keyword, ((indices))] pairs. IfcLineIndex is a
            // polyline through ALL its points; IfcArcIndex is a 3-point circular arc (discretized).
            // Ignoring them (the old behaviour) collapsed every arc to the chord between listed
            // points — sharp corners on filleted profiles. Absent Segments -> the raw coord polygon.
            const Value *segs = (in->args.size() > 1 && in->args[1].is_list()) ? &in->args[1] : nullptr;
            if (segs && !segs->items.empty()) {
                for (size_t k = 0; k + 1 < segs->items.size(); k += 2) {
                    const Value &kw = segs->items[k];
                    const Value &al = segs->items[k + 1];
                    if (kw.kind != adacpp::step::Kind::Keyword || !al.is_list() || al.items.empty() ||
                        !al.items[0].is_list())
                        continue;
                    const std::vector<Value> &ix = al.items[0].items;
                    if (iequals(kw.s, "IFCARCINDEX") && ix.size() == 3) {
                        for (const Vec3 &q : arc_poly_2d(at(ix[0].i), at(ix[1].i), at(ix[2].i)))
                            push(q);
                    } else {
                        for (const Value &iv : ix)
                            push(at(iv.kind == adacpp::step::Kind::Int ? iv.i : (long) iv.as_double()));
                    }
                }
            } else {
                for (size_t i = 1; i < coords.size(); ++i)
                    push(coords[i]);
            }
        } else if (iequals(in->type, "IFCCOMPOSITECURVE")) {
            // (Segments=list of IfcCompositeCurveSegment, SelfIntersect). Each segment is
            // (Transition, SameSense, ParentCurve); sample each parent (line/arc/polyline) and
            // reverse it when SameSense=.F. so the outline stays continuous. This is how a curved
            // profile outline (e.g. a semicircle of a trimmed arc + a closing trimmed line) is built.
            if (!in->args.empty() && in->args[0].is_list())
                for (const Value &sref : in->args[0].items) {
                    if (!sref.is_ref())
                        continue;
                    const Instance *seg = inst(sref.i);
                    if (!seg || seg->args.size() < 3 || !seg->args[2].is_ref())
                        continue;
                    bool same = !(seg->args[1].kind == adacpp::step::Kind::Enum && seg->args[1].s == "F");
                    std::vector<Vec3> sp = curve_points2d(seg->args[2].i);
                    if (!same)
                        std::reverse(sp.begin(), sp.end());
                    for (const Vec3 &q : sp)
                        push(q);
                }
        } else if (iequals(in->type, "IFCTRIMMEDCURVE")) {
            for (const Vec3 &q : trimmed_curve_points2d(*in))
                push(q);
        }
        if (pts.size() > 1 && (pts.front() - pts.back()).norm() < 1e-9)
            pts.pop_back();
        return pts;
    }
    // Sample an IfcTrimmedCurve (BasisCurve, Trim1, Trim2, SenseAgreement, MasterRepresentation) as a
    // 2D profile-outline polyline. Supports an IfcLine basis (straight span) and an IfcCircle basis
    // (arc; PARAMETER trims are angles in the model plane-angle unit). CARTESIAN trims give the
    // endpoints directly. SenseAgreement=.F. reverses the traversal direction.
    std::vector<Vec3> trimmed_curve_points2d(const Instance &in) {
        std::vector<Vec3> out;
        const Instance *basis = inst(ref_arg(in, 0));
        if (!basis)
            return out;
        bool sense = !(in.args.size() > 3 && in.args[3].kind == adacpp::step::Kind::Enum && in.args[3].s == "F");
        // Trim1/Trim2 are SETs of IfcTrimmingSelect: an IfcParameterValue ([Keyword, List[Real]] or a
        // bare number) and/or an IfcCartesianPoint ref.
        auto read_trim = [&](const Value &set, double &param, bool &has_p, Vec3 &cart, bool &has_c) {
            has_p = has_c = false;
            if (!set.is_list())
                return;
            for (const Value &e : set.items) {
                if (numeric(e)) {
                    param = e.as_double();
                    has_p = true;
                } else if (e.is_list() && !e.items.empty() && numeric(e.items[0])) {
                    param = e.items[0].as_double();
                    has_p = true;
                } else if (e.is_ref()) {
                    const Instance *cp = inst(e.i);
                    if (cp && iequals(cp->type, "IFCCARTESIANPOINT")) {
                        cart = point(e.i);
                        has_c = true;
                    }
                }
            }
        };
        double t1 = 0, t2 = 0;
        bool hp1 = false, hp2 = false, hc1 = false, hc2 = false;
        Vec3 c1{}, c2{};
        if (in.args.size() > 1)
            read_trim(in.args[1], t1, hp1, c1, hc1);
        if (in.args.size() > 2)
            read_trim(in.args[2], t2, hp2, c2, hc2);
        if (iequals(basis->type, "IFCLINE")) {
            // IfcLine(Pnt, Dir=IfcVector(Orientation, Magnitude)); P(u) = Pnt + u*Magnitude*Orientation.
            Vec3 p0 = point(ref_arg(*basis, 0));
            const Instance *vec = inst(ref_arg(*basis, 1));
            Vec3 od = vec ? dir(ref_arg(*vec, 0)) : Vec3{1, 0, 0};
            double mag = (vec && vec->args.size() > 1 && numeric(vec->args[1])) ? vec->args[1].as_double() : 1.0;
            auto at = [&](double u, bool hc, const Vec3 &cp) -> Vec3 {
                return hc ? cp : Vec3{p0.x + u * mag * od.x, p0.y + u * mag * od.y, 0};
            };
            Vec3 a = at(t1, hc1, c1), b = at(t2, hc2, c2);
            if (!sense)
                std::swap(a, b);
            out = {a, b};
        } else if (iequals(basis->type, "IFCCIRCLE")) {
            Vec3 center{0, 0, 0};
            double phi0 = 0.0, r = ad(basis, 1);
            const Instance *pos = inst(ref_arg(*basis, 0)); // IfcAxis2Placement2D(Location, RefDirection)
            if (pos) {
                if (pos->args.size() > 0 && pos->args[0].is_ref())
                    center = point(pos->args[0].i);
                if (pos->args.size() > 1 && pos->args[1].is_ref()) {
                    Vec3 rd = dir(pos->args[1].i);
                    phi0 = std::atan2(rd.y, rd.x);
                }
            }
            double sc = plane_angle_scale();
            double a1 = t1 * sc, a2 = t2 * sc; // angles measured from the placement's local X axis
            if (sense) {                       // CCW from a1 to a2
                while (a2 <= a1 + 1e-9)
                    a2 += 2 * PI;
            } else { // CW from a1 to a2
                while (a2 >= a1 - 1e-9)
                    a2 -= 2 * PI;
            }
            int n = std::max(2, (int) std::ceil(std::abs(a2 - a1) / (PI / 32)));
            for (int i = 0; i <= n; ++i) {
                double a = phi0 + a1 + (a2 - a1) * i / n;
                out.push_back({center.x + r * std::cos(a), center.y + r * std::sin(a), 0});
            }
        }
        return out;
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
        if ((root.extrusion || root.revolve || root.boolean || root.sweep) && solid_src_ != id) {
            mixed_ = true;
            return false;
        }
        solid_src_ = id;
        return true;
    }
    static bool solid_ok(const SolidItemN &it) {
        return it.extrusion || it.revolve || it.boolean || it.sweep || !it.faces.empty();
    }
    // Resolve an IfcSolidModel / IfcCsgPrimitive3D / IfcBooleanResult / IfcHalfSpaceSolid into a
    // SolidItemN (the boolean-operand form). `ref*` = a sibling-operand bbox to bound a half-space.
    SolidItemN resolve_solid_item(long id, const Vec3 *refmin = nullptr, const Vec3 *refmax = nullptr) {
        SolidItemN out;
        const Instance *in = inst(id);
        if (!in)
            return out;
        std::string_view t = in->type;
        if (iequals(t, "IFCEXTRUDEDAREASOLID")) {
            auto prof = profile_face(ref_arg(*in, 0));
            if (!prof)
                return out;
            auto ex = std::make_shared<ExtrusionN>();
            ex->profile = prof;
            ex->frame = (in->args.size() > 1 && in->args[1].is_ref()) ? axis2(in->args[1].i) : Frame{};
            ex->direction = (in->args.size() > 2 && in->args[2].is_ref()) ? dir(in->args[2].i) : Vec3{0, 0, 1};
            ex->depth = in->args.size() > 3 ? in->args[3].as_double() : 0.0;
            out.extrusion = ex;
        } else if (iequals(t, "IFCREVOLVEDAREASOLID")) {
            auto prof = profile_face(ref_arg(*in, 0));
            if (!prof)
                return out;
            auto rv = std::make_shared<RevolveN>();
            rv->profile = prof;
            rv->frame = (in->args.size() > 1 && in->args[1].is_ref()) ? axis2(in->args[1].i) : Frame{};
            const Instance *ax = inst(ref_arg(*in, 2));
            rv->axis_origin =
                (ax && ax->args.size() > 0 && ax->args[0].is_ref()) ? point(ax->args[0].i) : Vec3{0, 0, 0};
            rv->axis_dir = (ax && ax->args.size() > 1 && ax->args[1].is_ref()) ? dir(ax->args[1].i) : Vec3{0, 0, 1};
            rv->angle = in->args.size() > 3 ? in->args[3].as_double() : 0.0;
            out.revolve = rv;
        } else if (iequals(t, "IFCBLOCK")) {
            double X = ad(in, 1), Y = ad(in, 2), Z = ad(in, 3);
            if (X <= 0 || Y <= 0 || Z <= 0)
                return out;
            auto ex = std::make_shared<ExtrusionN>();
            ex->profile = make_poly_profile({{0, 0, 0}, {X, 0, 0}, {X, Y, 0}, {0, Y, 0}});
            ex->frame = axis2(ref_arg(*in, 0));
            ex->direction = {0, 0, 1};
            ex->depth = Z;
            out.extrusion = ex->profile ? ex : nullptr;
        } else if (iequals(t, "IFCRIGHTCIRCULARCYLINDER")) {
            double H = ad(in, 1), R = ad(in, 2);
            if (H <= 0 || R <= 0)
                return out;
            auto ex = std::make_shared<ExtrusionN>();
            ex->profile = make_poly_profile(circle_poly(R));
            ex->frame = axis2(ref_arg(*in, 0));
            ex->direction = {0, 0, 1};
            ex->depth = H;
            out.extrusion = ex->profile ? ex : nullptr;
        } else if (iequals(t, "IFCSPHERE")) {
            double R = ad(in, 1);
            if (R <= 0)
                return out;
            std::vector<Vec3> poly;
            const int n = 32;
            for (int i = 0; i <= n; ++i) {
                double a = -PI / 2 + PI * i / n;
                poly.push_back({R * std::cos(a), R * std::sin(a), 0});
            }
            auto rv = std::make_shared<RevolveN>();
            rv->profile = make_poly_profile(std::move(poly));
            rv->frame = axis2(ref_arg(*in, 0));
            rv->axis_origin = {0, 0, 0};
            rv->axis_dir = {0, 1, 0};
            rv->angle = 2.0 * PI;
            out.revolve = rv->profile ? rv : nullptr;
        } else if (iequals(t, "IFCRIGHTCIRCULARCONE")) {
            double H = ad(in, 1), R = ad(in, 2);
            if (H <= 0 || R <= 0)
                return out;
            auto rv = std::make_shared<RevolveN>();
            rv->profile = make_poly_profile({{0, 0, 0}, {R, 0, 0}, {0, H, 0}});
            Frame pos = axis2(ref_arg(*in, 0));
            Frame F;
            F.o = pos.o;
            F.x = pos.x;
            F.y = pos.z;
            F.z = pos.x.cross(pos.z);
            rv->frame = F;
            rv->axis_origin = {0, 0, 0};
            rv->axis_dir = {0, 1, 0};
            rv->angle = 2.0 * PI;
            out.revolve = rv->profile ? rv : nullptr;
        } else if (iequals(t, "IFCCSGSOLID")) {
            return resolve_solid_item(ref_arg(*in, 0), refmin, refmax);
        } else if (iequals(t, "IFCBOOLEANRESULT") || iequals(t, "IFCBOOLEANCLIPPINGRESULT")) {
            out.boolean = mk_boolean(in);
        } else if (iequals(t, "IFCADVANCEDBREP") || iequals(t, "IFCFACETEDBREP")) {
            const Instance *shell = inst(ref_arg(*in, 0));
            if (shell && !shell->args.empty() && shell->args[0].is_list())
                for (const Value &fref : shell->args[0].items)
                    if (fref.is_ref())
                        if (auto f = face(fref.i))
                            out.faces.push_back(f);
        } else if (iequals(t, "IFCHALFSPACESOLID")) {
            if (auto ex = mk_halfspace(in, refmin, refmax))
                out.extrusion = ex;
        } else if (iequals(t, "IFCPOLYGONALFACESET")) {
            // (Coordinates=IfcCartesianPointList3D, Closed, Faces=IfcIndexedPolygonalFace[], PnIndex)
            std::vector<Vec3> pts = point_list_3d(ref_arg(*in, 0));
            if (in->args.size() > 2 && in->args[2].is_list())
                for (const Value &fref : in->args[2].items)
                    if (fref.is_ref()) {
                        const Instance *pf = inst(fref.i); // IfcIndexedPolygonalFace(CoordIndex[, Inner])
                        if (!pf || pf->args.empty() || !pf->args[0].is_list())
                            continue;
                        std::vector<Vec3> poly;
                        for (const Value &iv : pf->args[0].items)
                            push_pt(poly, pts, iv);
                        if (auto f = make_profile(poly))
                            out.faces.push_back(f);
                    }
        } else if (iequals(t, "IFCTRIANGULATEDFACESET")) {
            // (Coordinates, Normals, Closed, CoordIndex=list of (i,j,k) triples, PnIndex)
            std::vector<Vec3> pts = point_list_3d(ref_arg(*in, 0));
            if (in->args.size() > 3 && in->args[3].is_list())
                for (const Value &tri : in->args[3].items)
                    if (tri.is_list() && tri.items.size() >= 3) {
                        std::vector<Vec3> poly;
                        for (const Value &iv : tri.items)
                            push_pt(poly, pts, iv);
                        if (poly.size() >= 3)
                            if (auto f = make_profile(poly))
                                out.faces.push_back(f);
                    }
        } else if (iequals(t, "IFCSHELLBASEDSURFACEMODEL")) { // (SbsmBoundary = shells)
            if (!in->args.empty() && in->args[0].is_list())
                for (const Value &sh : in->args[0].items)
                    if (sh.is_ref())
                        add_shell_faces(sh.i, out);
        } else if (iequals(t, "IFCFACEBASEDSURFACEMODEL")) { // (FbsmFaces = IfcConnectedFaceSet[])
            if (!in->args.empty() && in->args[0].is_list())
                for (const Value &cfs : in->args[0].items)
                    if (cfs.is_ref())
                        add_shell_faces(cfs.i, out);
        } else if (iequals(t, "IFCCONNECTEDFACESET") || iequals(t, "IFCOPENSHELL") ||
                   iequals(t, "IFCCLOSEDSHELL")) {
            add_shell_faces(id, out);
        } else if (iequals(t, "IFCADVANCEDFACE") || iequals(t, "IFCFACESURFACE") || iequals(t, "IFCFACE")) {
            if (auto f = face(id)) // a bare face as the representation item
                out.faces.push_back(f);
        } else if (iequals(t, "IFCSWEPTDISKSOLID")) {
            // (Directrix, Radius, InnerRadius, StartParam, EndParam) — a disk/annulus swept along the
            // 3D directrix (rebar, pipes). Trim params ignored (full directrix).
            std::vector<Vec3> dp = directrix_points(ref_arg(*in, 0));
            double radius = ad(in, 1);
            double inner = (in->args.size() > 2 && (in->args[2].kind == adacpp::step::Kind::Real ||
                                                    in->args[2].kind == adacpp::step::Kind::Int))
                               ? in->args[2].as_double()
                               : 0.0;
            out.sweep = mk_swept_disk(dp, radius, inner);
        } else if (iequals(t, "IFCFIXEDREFERENCESWEPTAREASOLID")) {
            // (SweptArea, Position, Directrix, StartParam, EndParam, FixedReference)
            auto prof = profile_face(ref_arg(*in, 0));
            Frame pos = axis2(ref_arg(*in, 1));
            std::vector<Vec3> dp = directrix_points(ref_arg(*in, 2));
            Vec3 fref = (in->args.size() > 5 && in->args[5].is_ref()) ? dir(in->args[5].i) : Vec3{0, 0, 1};
            out.sweep = mk_fixed_ref_swept(prof, dp, pos, fref);
        } else if (iequals(t, "IFCSECTIONEDSOLIDHORIZONTAL")) {
            // (Directrix, CrossSections, CrossSectionPositions, FixedAxisVertical). Uniform sections
            // (all CrossSections identical) sweep like a fixed-vertical-reference solid; varying
            // sections aren't a single SweepN -> skipped.
            std::vector<Vec3> dp = directrix_points(ref_arg(*in, 0));
            long sec0 = -1;
            bool uniform = true;
            if (in->args.size() > 1 && in->args[1].is_list())
                for (const Value &s : in->args[1].items)
                    if (s.is_ref()) {
                        if (sec0 < 0)
                            sec0 = s.i;
                        else if (s.i != sec0)
                            uniform = false;
                    }
            if (uniform && sec0 >= 0)
                out.sweep = mk_fixed_ref_swept(profile_face(sec0), dp, Frame{}, Vec3{0, 0, 1});
        }
        return out;
    }
    // Fixed-reference sweep: the profile's local-x tracks a FIXED reference direction (projected
    // perpendicular to the tangent), not a rotation-minimising frame. Used by
    // IfcFixedReferenceSweptAreaSolid + (with a vertical reference) IfcSectionedSolidHorizontal.
    std::shared_ptr<SweepN> mk_fixed_ref_swept(std::shared_ptr<FaceSurfaceN> profile, const std::vector<Vec3> &pts,
                                               const Frame &pos, Vec3 fref) {
        if (!profile || pts.size() < 2)
            return nullptr;
        int n = (int) pts.size();
        Vec3 fr = fref.norm() > 1e-9 ? fref.normalized() : Vec3{0, 0, 1};
        auto sw = std::make_shared<SweepN>();
        sw->frame = pos;
        sw->profile = profile;
        sw->origin.resize(n);
        sw->dir_x.resize(n);
        sw->dir_y.resize(n);
        for (int i = 0; i < n; ++i) {
            Vec3 tv = (i == 0) ? pts[1] - pts[0] : (i == n - 1) ? pts[n - 1] - pts[n - 2] : pts[i + 1] - pts[i - 1];
            Vec3 t = tv.norm() > 1e-12 ? tv.normalized() : Vec3{1, 0, 0};
            Vec3 dx = fr - t * t.dot(fr);
            if (dx.norm() < 1e-9) { // reference parallel to the tangent — any perpendicular will do
                Vec3 up = std::abs(t.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
                dx = up - t * t.dot(up);
            }
            dx = dx.normalized();
            sw->origin[i] = pts[i];
            sw->dir_x[i] = dx;
            sw->dir_y[i] = t.cross(dx);
        }
        return sw;
    }

    // ---- IFC alignment curve sampling (port of adapy ngeom._alignment_sweep) ----------------------
    // Normalised Fresnel S(t)=∫sin(πu²/2), C(t)=∫cos(πu²/2) via power series (clothoid transitions).
    static void alignment_fresnel(double t, double &S, double &C) {
        double hp = PI / 2.0, t4 = t * t * t * t;
        C = 0.0;
        double term = t;
        for (int n = 0; n < 200; ++n) {
            double c = term / (4 * n + 1);
            C += c;
            if (std::abs(c) < 1e-18 && n > 2)
                break;
            term *= -(hp * hp) * t4 / ((2 * n + 1) * (2 * n + 2));
        }
        S = 0.0;
        term = hp * t * t * t;
        for (int n = 0; n < 200; ++n) {
            double s = term / (4 * n + 3);
            S += s;
            if (std::abs(s) < 1e-18 && n > 2)
                break;
            term *= -(hp * hp) * t4 / ((2 * n + 2) * (2 * n + 3));
        }
    }
    static bool numeric(const Value &v) {
        return v.kind == adacpp::step::Kind::Real || v.kind == adacpp::step::Kind::Int;
    }
    // Parent curve at arc length p (its own 2D frame) -> (point2d, unit tangent2d), as z=0 Vec3.
    bool alignment_parent_eval(long cid, double p, double seg_len, Vec3 &P, Vec3 &T) {
        const Instance *in = inst(cid);
        if (!in)
            return false;
        std::string_view t = in->type;
        if (iequals(t, "IFCLINE")) {
            P = {p, 0, 0};
            T = {1, 0, 0};
            return true;
        }
        if (iequals(t, "IFCCIRCLE")) {
            double r = ad(in, 1);
            if (std::abs(r) < 1e-12)
                return false;
            double th = p / r;
            P = {r * std::cos(th), r * std::sin(th), 0};
            T = {-std::sin(th), std::cos(th), 0};
            return true;
        }
        if (iequals(t, "IFCCLOTHOID")) {
            double A = ad(in, 1), scale = std::abs(A) * std::sqrt(PI);
            if (scale < 1e-12) {
                P = {p, 0, 0};
                T = {1, 0, 0};
                return true;
            }
            double tt = p / scale, S, C;
            alignment_fresnel(tt, S, C);
            double sgn = A < 0 ? -1.0 : 1.0, ph = PI * tt * tt / 2.0;
            P = {scale * C, sgn * scale * S, 0};
            T = Vec3{std::cos(ph), sgn * std::sin(ph), 0}.normalized();
            return true;
        }
        if (iequals(t, "IFCCOSINESPIRAL")) {
            double A1 = ad(in, 1);
            double A0 = (in->args.size() > 2 && numeric(in->args[2])) ? in->args[2].as_double() : 0.0;
            double L = std::abs(seg_len);
            auto theta = [&](double s) {
                double th = (A0 != 0.0) ? s / A0 : 0.0;
                if (L > 0.0 && A1 != 0.0)
                    th += (L / (PI * A1)) * std::sin(PI * s / L);
                return th;
            };
            int n = std::max(32, (int) (std::abs(p) / 0.1) + 1);
            double px = 0, py = 0, dx = p / n;
            for (int i = 1; i <= n; ++i) {
                double th = theta(p * i / n), thm = theta(p * (i - 1) / n);
                px += (std::cos(th) + std::cos(thm)) * 0.5 * dx;
                py += (std::sin(th) + std::sin(thm)) * 0.5 * dx;
            }
            double thp = theta(p);
            P = {px, py, 0};
            T = {std::cos(thp), std::sin(thp), 0};
            return true;
        }
        return false;
    }
    // Global (point2d, tangent2d) at local_len along an IfcCurveSegment (parent placed by its
    // IfcAxis2Placement2D, arc length advancing in the sign of SegmentLength).
    bool alignment_seg_eval(const Instance *seg, double local_len, Vec3 &gp, Vec3 &gt) {
        long placement = -1, parent = -1;
        std::vector<double> meas;
        for (const Value &a : seg->args) {
            if (a.is_ref()) {
                if (placement < 0)
                    placement = a.i;
                else
                    parent = a.i;
            } else if (a.is_list() && !a.items.empty() && numeric(a.items[0]))
                meas.push_back(a.items[0].as_double());
        }
        if (parent < 0 || meas.size() < 2)
            return false;
        double seg_start = meas[0], seg_length = meas[1], sgn = seg_length >= 0 ? 1.0 : -1.0;
        Vec3 P, T, P0, T0;
        if (!alignment_parent_eval(parent, seg_start + sgn * local_len, seg_length, P, T))
            return false;
        alignment_parent_eval(parent, seg_start, seg_length, P0, T0);
        if (sgn < 0) {
            T = T * -1.0;
            T0 = T0 * -1.0;
        }
        // placement IfcAxis2Placement2D(Location, RefDirection)
        Vec3 o{0, 0, 0}, rd{1, 0, 0};
        const Instance *pl = inst(placement);
        if (pl) {
            if (!pl->args.empty() && pl->args[0].is_ref())
                o = point(pl->args[0].i);
            if (pl->args.size() > 1 && pl->args[1].is_ref())
                rd = dir(pl->args[1].i);
        }
        double rn = std::hypot(rd.x, rd.y);
        if (rn > 1e-12) {
            rd.x /= rn;
            rd.y /= rn;
        }
        double tn = std::hypot(T0.x, T0.y);
        Vec3 t0 = tn > 1e-12 ? Vec3{T0.x / tn, T0.y / tn, 0} : Vec3{1, 0, 0};
        // R = 2x2 rotation taking t0 -> rd
        double c = t0.x * rd.x + t0.y * rd.y, s = t0.x * rd.y - t0.y * rd.x;
        auto rot = [&](const Vec3 &v) { return Vec3{c * v.x - s * v.y, s * v.x + c * v.y, 0}; };
        Vec3 dp = rot(P - P0);
        gp = {o.x + dp.x, o.y + dp.y, 0};
        gt = rot(T).normalized();
        return true;
    }
    // Sample the IfcCurveSegments of a composite/base curve -> (cumulative arc length s, 2D point).
    void alignment_sample(long curve_id, int n_per, std::vector<double> &s_out, std::vector<Vec3> &p_out) {
        const Instance *in = inst(curve_id);
        if (!in || in->args.empty() || !in->args[0].is_list())
            return;
        double s_acc = 0.0;
        for (const Value &sref : in->args[0].items) {
            if (!sref.is_ref())
                continue;
            const Instance *seg = inst(sref.i);
            if (!seg || !iequals(seg->type, "IFCCURVESEGMENT"))
                continue;
            double L = 0.0;
            std::vector<double> meas;
            for (const Value &a : seg->args)
                if (a.is_list() && !a.items.empty() && numeric(a.items[0]))
                    meas.push_back(a.items[0].as_double());
            if (meas.size() >= 2)
                L = std::abs(meas[1]);
            if (L < 1e-9)
                continue;
            for (int i = 0; i <= n_per; ++i) {
                Vec3 gp, gt;
                if (alignment_seg_eval(seg, L * i / n_per, gp, gt)) {
                    s_out.push_back(s_acc + L * i / n_per);
                    p_out.push_back(gp);
                }
            }
            s_acc += L;
        }
    }
    // A composite/segmented alignment curve of IfcCurveSegments -> planar 3D directrix (z=0).
    std::vector<Vec3> alignment_planar_points(long cid) {
        std::vector<double> s;
        std::vector<Vec3> p;
        alignment_sample(cid, 24, s, p);
        return p;
    }
    // IfcGradientCurve(Segments=vertical gradient, SelfIntersect, BaseCurve=horizontal, EndPoint) ->
    // 3D directrix: horizontal (x,y) from BaseCurve + z(s) interpolated from the vertical gradient.
    std::vector<Vec3> alignment_gradient_points(long cid) {
        const Instance *in = inst(cid);
        if (!in)
            return {};
        long base = (in->args.size() > 2 && in->args[2].is_ref()) ? in->args[2].i : -1;
        std::vector<double> hs;
        std::vector<Vec3> hp;
        if (base >= 0)
            alignment_sample(base, 24, hs, hp); // horizontal (x,y) along s
        else
            alignment_sample(cid, 24, hs, hp);  // SegmentedReferenceCurve: sample its own segments
        // vertical gradient z(s): the gradient's own segments give (distance, height) points
        std::vector<double> vs;
        std::vector<Vec3> vp;
        if (iequals(in->type, "IFCGRADIENTCURVE"))
            alignment_sample(cid, 24, vs, vp); // segments (arg 0) are the vertical profile
        std::vector<Vec3> out;
        out.reserve(hp.size());
        for (size_t i = 0; i < hp.size(); ++i) {
            double z = 0.0;
            if (!vp.empty()) { // interpolate height over the vertical profile's (x=distance, y=height)
                double q = hs[i];
                z = vp.front().y;
                for (size_t k = 0; k + 1 < vp.size(); ++k)
                    if ((vp[k].x <= q && q <= vp[k + 1].x) || (vp[k + 1].x <= q && q <= vp[k].x)) {
                        double dxk = vp[k + 1].x - vp[k].x;
                        z = std::abs(dxk) < 1e-12 ? vp[k].y
                                                  : vp[k].y + (vp[k + 1].y - vp[k].y) * (q - vp[k].x) / dxk;
                        break;
                    } else if (q > vp[k + 1].x)
                        z = vp[k + 1].y;
            }
            out.push_back({hp[i].x, hp[i].y, z});
        }
        return out;
    }
    // A 3-point 3D circular arc (fit the plane through the points, circumcircle, sweep a->b via m).
    static std::vector<Vec3> arc_poly_3d(const Vec3 &a, const Vec3 &m, const Vec3 &b) {
        Vec3 ab = b - a, am = m - a, nrm = ab.cross(am);
        if (nrm.norm() < 1e-12)
            return {a, m, b};
        Vec3 n = nrm.normalized();
        double abab = ab.dot(ab), amam = am.dot(am), abam = ab.dot(am);
        double d = 2.0 * (abab * amam - abam * abam);
        if (std::abs(d) < 1e-18)
            return {a, m, b};
        Vec3 c = a + ab * ((amam * (abab - abam)) / d) + am * ((abab * (amam - abam)) / d);
        double r = (a - c).norm();
        Vec3 u = (a - c).normalized(), v = n.cross(u);
        auto ang = [&](const Vec3 &p) { Vec3 rp = p - c; return std::atan2(rp.dot(v), rp.dot(u)); };
        double ta = ang(a), tm = ang(m), tb = ang(b);
        auto wrap = [](double x) {
            while (x < 0)
                x += 2 * PI;
            while (x >= 2 * PI)
                x -= 2 * PI;
            return x;
        };
        double dab = wrap(tb - ta), dam = wrap(tm - ta);
        double sweep = (dam <= dab) ? dab : dab - 2 * PI;
        int ns = std::max(2, (int) std::ceil(std::abs(sweep) / (2 * PI) * 64));
        std::vector<Vec3> out;
        out.reserve(ns + 1);
        for (int i = 0; i <= ns; ++i) {
            double th = ta + sweep * i / ns;
            out.push_back(c + u * (r * std::cos(th)) + v * (r * std::sin(th)));
        }
        return out;
    }
    // A 3D directrix curve -> ordered polyline. Covers IfcPolyline, IfcIndexedPolyCurve (3D, arc-aware),
    // and IfcCompositeCurve (chained parent curves). Other bases yield empty (-> the sweep is skipped).
    std::vector<Vec3> directrix_points(long cid) {
        const Instance *in = inst(cid);
        if (!in)
            return {};
        std::string_view t = in->type;
        auto push = [](std::vector<Vec3> &pts, const Vec3 &p) {
            if (pts.empty() || (pts.back() - p).norm() > 1e-9)
                pts.push_back(p);
        };
        if (iequals(t, "IFCPOLYLINE"))
            return point_list(in->args.empty() ? Value{} : in->args[0]);
        if (iequals(t, "IFCINDEXEDPOLYCURVE")) {
            std::vector<Vec3> coords = point_list_3d(ref_arg(*in, 0)), pts;
            auto at = [&](long i) { return (i >= 1 && (size_t) i < coords.size()) ? coords[i] : Vec3{0, 0, 0}; };
            const Value *segs = (in->args.size() > 1 && in->args[1].is_list()) ? &in->args[1] : nullptr;
            if (segs && !segs->items.empty()) {
                for (size_t k = 0; k + 1 < segs->items.size(); k += 2) {
                    const Value &kw = segs->items[k], &al = segs->items[k + 1];
                    if (kw.kind != adacpp::step::Kind::Keyword || !al.is_list() || al.items.empty() ||
                        !al.items[0].is_list())
                        continue;
                    const auto &ix = al.items[0].items;
                    if (iequals(kw.s, "IFCARCINDEX") && ix.size() == 3) {
                        for (const Vec3 &q : arc_poly_3d(at(ix[0].i), at(ix[1].i), at(ix[2].i)))
                            push(pts, q);
                    } else
                        for (const Value &iv : ix)
                            push(pts, at(iv.kind == adacpp::step::Kind::Int ? iv.i : (long) iv.as_double()));
                }
            } else
                for (size_t i = 1; i < coords.size(); ++i)
                    push(pts, coords[i]);
            return pts;
        }
        // Alignment directrixes: an IfcGradientCurve (horizontal composite + vertical gradient) or a
        // composite/segmented curve of IfcCurveSegments (line/arc/clothoid/cosine-spiral transitions).
        if (iequals(t, "IFCGRADIENTCURVE") || iequals(t, "IFCSEGMENTEDREFERENCECURVE"))
            return alignment_gradient_points(cid);
        if (iequals(t, "IFCCURVESEGMENT")) { // a single alignment segment (IfcAlignmentSegment axis)
            double L = 0.0;
            std::vector<double> meas;
            for (const Value &a : in->args)
                if (a.is_list() && !a.items.empty() && numeric(a.items[0]))
                    meas.push_back(a.items[0].as_double());
            if (meas.size() >= 2)
                L = std::abs(meas[1]);
            std::vector<Vec3> pts;
            if (L > 1e-9)
                for (int i = 0; i <= 24; ++i) {
                    Vec3 gp, gt;
                    if (alignment_seg_eval(in, L * i / 24, gp, gt))
                        pts.push_back(gp);
                }
            return pts;
        }
        if ((iequals(t, "IFCCOMPOSITECURVE")) && !in->args.empty() && in->args[0].is_list()) {
            // alignment composite: its segments are IfcCurveSegment (not IfcCompositeCurveSegment)
            for (const Value &sref : in->args[0].items)
                if (sref.is_ref()) {
                    const Instance *s0 = inst(sref.i);
                    if (s0 && iequals(s0->type, "IFCCURVESEGMENT"))
                        return alignment_planar_points(cid);
                    break;
                }
        }
        if (iequals(t, "IFCCOMPOSITECURVE") && !in->args.empty() && in->args[0].is_list()) {
            std::vector<Vec3> pts;
            for (const Value &sref : in->args[0].items)
                if (sref.is_ref()) {
                    const Instance *seg = inst(sref.i); // IfcCompositeCurveSegment(Transition,Same,ParentCurve)
                    if (seg && seg->args.size() > 2 && seg->args[2].is_ref())
                        for (const Vec3 &p : directrix_points(seg->args[2].i))
                            push(pts, p);
                }
            return pts;
        }
        return {};
    }
    // Disk/annulus swept along a directrix -> SweepN with rotation-minimising (parallel-transport)
    // per-station frames, so the circular profile doesn't twist. Mirrors adapy's _sweep_frames.
    std::shared_ptr<SweepN> mk_swept_disk(const std::vector<Vec3> &pts, double radius, double inner) {
        if (pts.size() < 2 || radius <= 0)
            return nullptr;
        int n = (int) pts.size();
        std::vector<Vec3> tan(n);
        for (int i = 0; i < n; ++i) {
            Vec3 tv = (i == 0) ? pts[1] - pts[0] : (i == n - 1) ? pts[n - 1] - pts[n - 2] : pts[i + 1] - pts[i - 1];
            tan[i] = tv.norm() > 1e-12 ? tv.normalized() : Vec3{0, 0, 1};
        }
        auto sw = std::make_shared<SweepN>();
        sw->frame = Frame{}; // origin/dir_x/dir_y are already world
        sw->origin.resize(n);
        sw->dir_x.resize(n);
        sw->dir_y.resize(n);
        Vec3 up = std::abs(tan[0].z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
        Vec3 dx = (up - tan[0] * tan[0].dot(up)).normalized();
        for (int i = 0; i < n; ++i) {
            if (i > 0) { // parallel-transport dx across the tangent turn (Rodrigues)
                Vec3 axis = tan[i - 1].cross(tan[i]);
                double s = axis.norm(), cth = tan[i - 1].dot(tan[i]);
                if (s > 1e-9) {
                    axis = axis * (1.0 / s);
                    double ang = std::atan2(s, cth);
                    dx = dx * std::cos(ang) + axis.cross(dx) * std::sin(ang) +
                         axis * (axis.dot(dx) * (1 - std::cos(ang)));
                }
            }
            Vec3 ortho = dx - tan[i] * tan[i].dot(dx);
            if (ortho.norm() < 1e-9) { // dx drifted parallel to the tangent — reseed perpendicular
                Vec3 up = std::abs(tan[i].z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
                ortho = up - tan[i] * tan[i].dot(up);
            }
            dx = ortho.normalized();
            sw->origin[i] = pts[i];
            sw->dir_x[i] = dx;
            sw->dir_y[i] = tan[i].cross(dx);
        }
        std::vector<std::vector<Vec3>> holes;
        if (inner > 1e-9)
            holes.push_back(circle_poly(inner));
        sw->profile = make_profile(circle_poly(radius), holes);
        return sw->profile ? sw : nullptr;
    }
    // IfcCartesianPointList3D(CoordList) -> 1-based point array (index 0 is a placeholder).
    std::vector<Vec3> point_list_3d(long id) {
        std::vector<Vec3> pts{{0, 0, 0}};
        const Instance *pl = inst(id);
        if (pl && !pl->args.empty() && pl->args[0].is_list())
            for (const Value &row : pl->args[0].items)
                if (row.is_list() && row.items.size() >= 3)
                    pts.push_back({row.items[0].as_double(), row.items[1].as_double(), row.items[2].as_double()});
        return pts;
    }
    static void push_pt(std::vector<Vec3> &poly, const std::vector<Vec3> &pts, const Value &iv) {
        long ix = (iv.kind == adacpp::step::Kind::Int) ? iv.i : (long) iv.as_double(); // 1-based CoordIndex
        if (ix >= 1 && (size_t) ix < pts.size())
            poly.push_back(pts[ix]);
    }
    // A shell (IfcClosedShell/IfcOpenShell/IfcConnectedFaceSet): CfsFaces (arg 0) is a list of IfcFace.
    void add_shell_faces(long shell_id, SolidItemN &out) {
        const Instance *sh = inst(shell_id);
        if (sh && !sh->args.empty() && sh->args[0].is_list())
            for (const Value &fref : sh->args[0].items)
                if (fref.is_ref())
                    if (auto f = face(fref.i))
                        out.faces.push_back(f);
    }
    // IfcBooleanResult(Operator, FirstOperand, SecondOperand) -> ng::BooleanN (null if an operand can't
    // be resolved). op: 0 difference / 1 union / 2 intersection. The 1st operand bounds a half-space 2nd.
    std::shared_ptr<BooleanN> mk_boolean(const Instance *in) {
        if (!in || in->args.size() < 3)
            return nullptr;
        auto bn = std::make_shared<BooleanN>();
        std::string_view op =
            (in->args[0].kind == adacpp::step::Kind::Enum) ? in->args[0].s : std::string_view("DIFFERENCE");
        bn->op = (op == "UNION") ? 1 : (op == "INTERSECTION") ? 2 : 0;
        bn->a = resolve_solid_item(ref_arg(*in, 1));
        if (!solid_ok(bn->a))
            return nullptr;
        Vec3 amin, amax;
        bool hb = solid_item_bbox(bn->a, amin, amax);
        bn->b = resolve_solid_item(ref_arg(*in, 2), hb ? &amin : nullptr, hb ? &amax : nullptr);
        if (!solid_ok(bn->b))
            return nullptr;
        return bn;
    }
    // Loose world bbox of a SolidItemN (over-estimate is fine — only used to size a half-space box).
    bool solid_item_bbox(const SolidItemN &it, Vec3 &mn, Vec3 &mx) {
        std::vector<Vec3> pts;
        if (it.extrusion && it.extrusion->profile && !it.extrusion->profile->bounds.empty() &&
            it.extrusion->profile->bounds[0].loop) {
            const auto &ex = *it.extrusion;
            Vec3 d = ex.direction * ex.depth;
            for (const Vec3 &p : ex.profile->bounds[0].loop->polygon) {
                pts.push_back(ex.frame.to_world(p.x, p.y, 0));
                pts.push_back(ex.frame.to_world(p.x + d.x, p.y + d.y, d.z));
            }
        } else if (it.revolve && it.revolve->profile && !it.revolve->profile->bounds.empty() &&
                   it.revolve->profile->bounds[0].loop) {
            const auto &rv = *it.revolve;
            Vec3 axd = rv.axis_dir.norm() > 1e-9 ? rv.axis_dir.normalized() : Vec3{0, 1, 0};
            double rmax = 0;
            std::vector<Vec3> w;
            for (const Vec3 &p : rv.profile->bounds[0].loop->polygon) {
                Vec3 rel = p - rv.axis_origin;
                rmax = std::max(rmax, (rel - axd * axd.dot(rel)).norm());
                w.push_back(rv.frame.to_world(p.x, p.y, p.z));
            }
            Vec3 e{rmax, rmax, rmax}; // expand for the swept circle (loose)
            for (const Vec3 &p : w) {
                pts.push_back(p + e);
                pts.push_back(p - e);
            }
        } else if (it.boolean) {
            return solid_item_bbox(it.boolean->a, mn, mx); // result is contained in operand a
        } else if (!it.faces.empty()) {
            for (const auto &f : it.faces)
                for (const auto &b : f->bounds)
                    if (b.loop) {
                        if (b.loop->is_poly)
                            for (const Vec3 &p : b.loop->polygon)
                                pts.push_back(p);
                        else
                            for (const auto &e : b.loop->edges) {
                                pts.push_back(e.start);
                                pts.push_back(e.end);
                            }
                    }
        }
        if (pts.empty())
            return false;
        mn = mx = pts[0];
        for (const Vec3 &p : pts) {
            mn.x = std::min(mn.x, p.x);
            mn.y = std::min(mn.y, p.y);
            mn.z = std::min(mn.z, p.z);
            mx.x = std::max(mx.x, p.x);
            mx.y = std::max(mx.y, p.y);
            mx.z = std::max(mx.z, p.z);
        }
        return true;
    }
    // IfcHalfSpaceSolid(BaseSurface=IfcPlane, AgreementFlag) -> a finite box (extrusion) on the material
    // side of the plane, sized to cover the reference bbox so the boolean DIFFERENCE clips correctly.
    std::shared_ptr<ExtrusionN> mk_halfspace(const Instance *in, const Vec3 *refmin, const Vec3 *refmax) {
        if (!refmin || !refmax)
            return nullptr;                            // no reference extent -> can't bound a half-space -> skip (OCC)
        const Instance *plane = inst(ref_arg(*in, 0)); // IfcPlane(Position)
        if (!plane)
            return nullptr;
        Frame pf = axis2(ref_arg(*plane, 0)); // o on the plane, z = normal
        bool agree = !(in->args.size() > 1 && in->args[1].kind == adacpp::step::Kind::Enum && in->args[1].s == "F");
        Vec3 hd = agree ? Vec3{-pf.z.x, -pf.z.y, -pf.z.z} : pf.z; // material side (sign validated vs OCC)
        Vec3 c{(refmin->x + refmax->x) / 2, (refmin->y + refmax->y) / 2, (refmin->z + refmax->z) / 2};
        double S = (*refmax - *refmin).norm() * 1.5 + 1e-6;
        Vec3 cp = c - pf.z * pf.z.dot(c - pf.o); // project the centre onto the cutting plane
        Frame F;
        F.o = cp;
        F.z = hd.normalized();
        Vec3 t = std::abs(F.z.dot(pf.x)) < 0.9 ? pf.x : pf.y;
        F.x = (t - F.z * F.z.dot(t)).normalized();
        F.y = F.z.cross(F.x);
        auto ex = std::make_shared<ExtrusionN>();
        ex->profile = make_poly_profile({{-S, -S, 0}, {S, -S, 0}, {S, S, 0}, {-S, S, 0}});
        ex->frame = F;
        ex->direction = {0, 0, 1}; // local Z = hd, into the material
        ex->depth = S;
        return ex->profile ? ex : nullptr;
    }
    // Append an IfcShapeRepresentation item's geometry to root. Brep faces concatenate across items; a
    // procedural solid (extrusion/revolve/CSG primitive/boolean) is one-per-root. IfcMappedItem places.
    void resolve_item(long id, NgeomRoot &root) {
        const Instance *in = inst(id);
        if (!in)
            return;
        if (iequals(in->type, "IFCMAPPEDITEM")) {
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
            return;
        }
        // Curve-only body (alignment axis / reference curve / a bare curve rep item) -> a polyline
        // rendered as GL_LINES. Reuse the directrix sampler (handles gradient/segmented/composite/
        // polyline/indexed + alignment clothoid/cant).
        if (iequals(in->type, "IFCGRADIENTCURVE") || iequals(in->type, "IFCSEGMENTEDREFERENCECURVE") ||
            iequals(in->type, "IFCCOMPOSITECURVE") || iequals(in->type, "IFCPOLYLINE") ||
            iequals(in->type, "IFCINDEXEDPOLYCURVE") || iequals(in->type, "IFCTRIMMEDCURVE") ||
            iequals(in->type, "IFCCURVESEGMENT")) {
            std::vector<Vec3> pts = directrix_points(id);
            if (pts.size() >= 2)
                root.polylines.push_back(std::move(pts));
            else
                root.recognized_empty = true; // recognized curve, but degenerate (e.g. zero-length segment)
            return;
        }
        SolidItemN it = resolve_solid_item(id);
        if (!solid_ok(it))
            return; // unsupported geometry -> product yields nothing here -> OCC fallback
        if (!it.faces.empty() && !it.extrusion && !it.revolve && !it.boolean) {
            if (root.extrusion || root.revolve || root.boolean) { // brep mixed with a procedural solid
                mixed_ = true;
                return;
            }
            for (auto &f : it.faces) // brep faces concatenate across items
                root.faces.push_back(f);
        } else if (claim_solid(root, id)) {
            root.extrusion = it.extrusion;
            root.revolve = it.revolve;
            root.boolean = it.boolean;
            root.sweep = it.sweep;
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
            return (i < in->args.size() &&
                    (in->args[i].kind == adacpp::step::Kind::Real || in->args[i].kind == adacpp::step::Kind::Int))
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
