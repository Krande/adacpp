// A structural MEMBER as IFC: the product a reader rebuilds a beam or a plate from, not only the
// solid it looks like.
//
// WHY THIS IS NOT THE GEOMETRY PATH. The NGEOM emitter (ifc_emit.h) writes what a product looks
// like -- an analytic solid inside an IfcBuildingElementProxy -- which is all a viewer needs and
// none of what a model reader needs: the class (IfcBeam, IfcPlate), the reference axis, the
// profile as a profile (its arcs, its name, the parameter set its writer attached to it), the
// half-spaces it is cut by as half-spaces, its material and its property sets. A reader handed
// the proxy can draw the member and cannot rebuild it. This emitter writes the member the way a
// model writer does, from a description of the member rather than of its solid -- so the file a
// native write produces is the file a model reader on either side (ifcopenshell or the native
// member scan) reads back into the same member.
//
// Dep-free (stdlib only) so the embind/wasm writer can take it as-is. Entity layouts are IFC4 /
// IFC4X3 (identical for every entity written here).
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#if __has_include(<charconv>)
#include <charconv>
#endif

namespace adacpp::ifc_member_emit {

// A property value as the member's writer types it.
struct Value {
    enum class Kind { None, Real, Int, Bool, Text, List } kind = Kind::None;
    double d = 0.0;
    long long i = 0;
    bool b = false;
    std::string s;
    std::vector<Value> items;
    std::string ifc_type; //!< explicit IFC value type ("IFCPRESSUREMEASURE"); empty = by kind
};
using PropSet = std::vector<std::pair<std::string, Value>>;
using PropSets = std::vector<std::pair<std::string, PropSet>>;

// A profile boundary, in the member scan's own form (ifc_reader.h MemberInfo::ProfileCurve).
struct Curve {
    std::string kind;                          //!< "polyline" | "indexed"
    std::vector<std::array<double, 3>> points; //!< profile frame, model units
    int dim = 2;
    std::vector<std::pair<bool, std::vector<int>>> segments; //!< (is_arc, 1-based indices)
    bool has_segments = false;
};

struct Profile {
    std::string kind = "curves"; //!< "curves" | "circle" | "circle_hollow"
    std::string name;
    bool has_name = false;
    Curve outer;
    std::vector<Curve> inners;
    double radius = 0.0, wall_thickness = 0.0;
    PropSets properties; //!< IfcProfileProperties sets (e.g. a writer's section parameters)
};

struct Cut {
    std::array<double, 3> origin{0, 0, 0}, normal{0, 0, 1}, xdir{1, 0, 0};
    bool has_xdir = false;
    bool agreement = true;
};

struct Material {
    std::string name, category, properties_name;
    bool has_category = false;
    PropSet properties;
};

struct Member {
    std::string ifc_class = "IfcBeam";
    std::string guid, name, description, object_type;
    bool has_description = false, has_object_type = false;
    std::array<double, 16> placement{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; //!< column-major
    bool has_axis = false;
    std::array<double, 3> axis_p1{0, 0, 0}, axis_p2{0, 0, 0};
    Profile profile;
    std::array<double, 3> ext_origin{0, 0, 0}, ext_axis{0, 0, 1}, ext_ref{1, 0, 0}, ext_dir{0, 0, 1};
    double depth = 0.0;
    std::vector<Cut> cuts;
    bool has_material = false;
    Material material;
    PropSets psets;
    bool has_color = false;
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
};

// A real that reads back as the same double: the shortest round-trip form where the standard
// library has it, %.17g otherwise. IFC SPF wants a decimal point and an uppercase exponent.
inline std::string real(double x) {
    char buf[40];
    size_t n = 0;
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
    auto r = std::to_chars(buf, buf + sizeof(buf) - 2, x);
    n = (size_t) (r.ptr - buf);
#else
    n = (size_t) std::snprintf(buf, sizeof(buf) - 2, "%.17g", x);
#endif
    std::string s(buf, n);
    for (char &c : s)
        if (c == 'e')
            c = 'E';
    auto epos = s.find('E');
    std::string mant = epos == std::string::npos ? s : s.substr(0, epos);
    std::string exp = epos == std::string::npos ? std::string() : s.substr(epos);
    if (mant.find('.') == std::string::npos && mant.find("inf") == std::string::npos &&
        mant.find("nan") == std::string::npos)
        mant += ".";
    return mant + exp;
}

// An SPF string literal (without the quotes): '' for ', \\ for \, and \X2\...\X0\ (UTF-16 hex) for
// anything outside printable ASCII -- the encoding ifcopenshell and the native reader decode.
inline std::string text(const std::string &s) {
    std::string o;
    o.reserve(s.size());
    auto hex4 = [](unsigned v) {
        static const char *H = "0123456789ABCDEF";
        std::string h(4, '0');
        for (int k = 3; k >= 0; --k, v >>= 4)
            h[k] = H[v & 15];
        return h;
    };
    for (size_t i = 0; i < s.size();) {
        unsigned char c = (unsigned char) s[i];
        if (c >= 0x20 && c < 0x7F) {
            if (c == '\'')
                o += "''";
            else if (c == '\\')
                o += "\\\\";
            else
                o += (char) c;
            ++i;
            continue;
        }
        // Decode one UTF-8 code point (an invalid byte is taken as Latin-1).
        unsigned cp = c;
        size_t len = 1;
        if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
            cp = ((c & 0x1F) << 6) | (s[i + 1] & 0x3F);
            len = 2;
        } else if ((c & 0xF0) == 0xE0 && i + 2 < s.size()) {
            cp = ((c & 0x0F) << 12) | ((s[i + 1] & 0x3F) << 6) | (s[i + 2] & 0x3F);
            len = 3;
        } else if ((c & 0xF8) == 0xF0 && i + 3 < s.size()) {
            cp = ((c & 0x07) << 18) | ((s[i + 1] & 0x3F) << 12) | ((s[i + 2] & 0x3F) << 6) | (s[i + 3] & 0x3F);
            len = 4;
        }
        i += len;
        o += "\\X2\\";
        if (cp >= 0x10000) {
            cp -= 0x10000;
            o += hex4(0xD800 + (cp >> 10)) + hex4(0xDC00 + (cp & 0x3FF));
        } else {
            o += hex4(cp);
        }
        o += "\\X0\\";
    }
    return o;
}

// Writes members into an SPF DATA section through a caller-owned id allocator. Materials are
// shared by name: one IfcMaterial (and its property set) per name, one IfcRelAssociatesMaterial
// per material over every member that names it, written by finish().
class MemberWriter {
public:
    // `next_id` hands out fresh entity ids; `ctx` is the geometric representation context id;
    // `rel_to` the IfcLocalPlacement members are placed relative to (0 = none); `guid_of(n)` makes
    // a deterministic IfcGloballyUniqueId for the n-th generated relationship/set.
    template <class NextId, class GuidOf>
    MemberWriter(NextId next_id, GuidOf guid_of, long ctx, long rel_to)
        : next_id_(next_id), guid_of_(guid_of), ctx_(ctx), rel_to_(rel_to) {}

    // Emit one member; returns the product id, or 0 when the member is not writable as a member
    // (no profile curve, no depth) -- the caller then writes the product its other way.
    long emit(std::string &out, const Member &m) {
        if (m.depth <= 0.0 || !member_class(m.ifc_class))
            return 0;
        long prof = profile(out, m.profile);
        if (!prof)
            return 0;
        for (const auto &[set_name, props] : m.profile.properties) {
            if (props.empty())
                continue;
            std::vector<long> ids = properties(out, props);
            e(out, "IFCPROFILEPROPERTIES('" + text(set_name) + "',$," + refs(ids) + "," + ref(prof) + ")");
        }
        long pos = axis2(out, m.ext_origin, m.ext_axis, m.ext_ref);
        long body = e(out, "IFCEXTRUDEDAREASOLID(" + ref(prof) + "," + ref(pos) + "," + ref(dir(out, m.ext_dir)) + "," +
                               real(m.depth) + ")");
        if (m.has_color) {
            auto R = [](float v) { return real((double) v); };
            long col = e(out, "IFCCOLOURRGB($," + R(m.color[0]) + "," + R(m.color[1]) + "," + R(m.color[2]) + ")");
            long sh = e(out, "IFCSURFACESTYLESHADING(" + ref(col) + "," + R(1.0f - m.color[3]) + ")");
            long ss = e(out, "IFCSURFACESTYLE($,.BOTH.,(" + ref(sh) + "))");
            e(out, "IFCSTYLEDITEM(" + ref(body) + ",(" + ref(ss) + "),$)");
        }
        long item = body;
        for (const Cut &c : m.cuts) {
            std::array<double, 3> x = c.has_xdir ? c.xdir : perpendicular(c.normal);
            long plane = e(out, "IFCPLANE(" + ref(axis2(out, c.origin, c.normal, x)) + ")");
            long hs = e(out, "IFCHALFSPACESOLID(" + ref(plane) + "," + (c.agreement ? ".T." : ".F.") + ")");
            item = e(out, "IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.," + ref(item) + "," + ref(hs) + ")");
        }
        std::vector<long> reps;
        if (m.has_axis) {
            long a = e(out, "IFCCARTESIANPOINT(" + p3(m.axis_p1) + ")");
            long b = e(out, "IFCCARTESIANPOINT(" + p3(m.axis_p2) + ")");
            long pl = e(out, "IFCPOLYLINE((" + ref(a) + "," + ref(b) + "))");
            reps.push_back(e(out, "IFCSHAPEREPRESENTATION(" + ref(ctx_) + ",'Axis','Curve3D',(" + ref(pl) + "))"));
        }
        // 'Clipping' names what the items are; strict readers drop a body whose type says otherwise.
        reps.push_back(e(out, "IFCSHAPEREPRESENTATION(" + ref(ctx_) + ",'Body','" +
                                  (m.cuts.empty() ? "SweptSolid" : "Clipping") + "',(" + ref(item) + "))"));
        long pds = e(out, "IFCPRODUCTDEFINITIONSHAPE($,$," + refs(reps) + ")");
        const auto &M = m.placement;
        long place = axis2(out, {M[12], M[13], M[14]}, {M[8], M[9], M[10]}, {M[0], M[1], M[2]});
        long lp = e(out, "IFCLOCALPLACEMENT(" + (rel_to_ ? ref(rel_to_) : std::string("$")) + "," + ref(place) + ")");
        long product =
            e(out, upper(m.ifc_class) + "('" + text(m.guid) + "',$,'" + text(m.name) + "'," +
                       opt_text(m.has_description, m.description) + "," + opt_text(m.has_object_type, m.object_type) +
                       "," + ref(lp) + "," + ref(pds) + ",$,$)");
        for (const auto &[set_name, props] : m.psets) {
            if (props.empty())
                continue; // IfcPropertySet.HasProperties is SET [1:?]
            std::vector<long> ids = properties(out, props);
            long ps =
                e(out, "IFCPROPERTYSET('" + guid_of_(gseq_++) + "',$,'" + text(set_name) + "',$," + refs(ids) + ")");
            e(out,
              "IFCRELDEFINESBYPROPERTIES('" + guid_of_(gseq_++) + "',$,$,$,(" + ref(product) + ")," + ref(ps) + ")");
        }
        if (m.has_material && !m.material.name.empty())
            material_users_[material(out, m.material)].push_back(product);
        return product;
    }

    // The material associations, one relationship per material over all its members.
    void finish(std::string &out) {
        for (const auto &[mat, users] : material_users_)
            e(out, "IFCRELASSOCIATESMATERIAL('" + guid_of_(gseq_++) + "',$,$,$," + refs(users) + "," + ref(mat) + ")");
        material_users_.clear();
    }

private:
    std::function<long()> next_id_;
    std::function<std::string(uint64_t)> guid_of_;
    long ctx_, rel_to_;
    uint64_t gseq_ = 0;
    std::map<std::string, long> materials_;
    std::map<long, std::vector<long>> material_users_;

    long e(std::string &out, const std::string &body) {
        long id = next_id_();
        out += "#" + std::to_string(id) + "=" + body + ";\n";
        return id;
    }
    static std::string ref(long id) {
        return "#" + std::to_string(id);
    }
    static std::string refs(const std::vector<long> &ids) {
        std::string s = "(";
        for (size_t i = 0; i < ids.size(); ++i)
            s += (i ? ",#" : "#") + std::to_string(ids[i]);
        return s + ")";
    }
    // The element classes written here: each has the same nine attributes in IFC4 and IFC4X3
    // (GlobalId .. PredefinedType), which is the layout emit() writes.
    static bool member_class(const std::string &cls) {
        static const char *ok[] = {"IFCBEAM",  "IFCCOLUMN", "IFCMEMBER",
                                   "IFCPLATE", "IFCSLAB",   "IFCBUILDINGELEMENTPROXY"};
        const std::string u = upper(cls);
        for (const char *k : ok)
            if (u == k)
                return true;
        return false;
    }
    static std::string upper(std::string s) {
        for (char &c : s)
            if (c >= 'a' && c <= 'z')
                c = (char) (c - 32);
        return s;
    }
    static std::string opt_text(bool has, const std::string &s) {
        return has ? "'" + text(s) + "'" : std::string("$");
    }
    static std::string p3(const std::array<double, 3> &p) {
        return "(" + real(p[0]) + "," + real(p[1]) + "," + real(p[2]) + ")";
    }
    static std::array<double, 3> perpendicular(const std::array<double, 3> &n) {
        // Any unit vector square to n: the plane's own x is not part of a half-space.
        std::array<double, 3> a =
            std::abs(n[0]) < 0.9 ? std::array<double, 3>{1, 0, 0} : std::array<double, 3>{0, 1, 0};
        double d = a[0] * n[0] + a[1] * n[1] + a[2] * n[2];
        std::array<double, 3> x{a[0] - d * n[0], a[1] - d * n[1], a[2] - d * n[2]};
        double l = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        return l > 0 ? std::array<double, 3>{x[0] / l, x[1] / l, x[2] / l} : std::array<double, 3>{1, 0, 0};
    }
    long dir(std::string &out, const std::array<double, 3> &v) {
        return e(out, "IFCDIRECTION(" + p3(v) + ")");
    }
    long axis2(std::string &out, const std::array<double, 3> &o, const std::array<double, 3> &z,
               const std::array<double, 3> &x) {
        long loc = e(out, "IFCCARTESIANPOINT(" + p3(o) + ")");
        return e(out, "IFCAXIS2PLACEMENT3D(" + ref(loc) + "," + ref(dir(out, z)) + "," + ref(dir(out, x)) + ")");
    }

    long curve(std::string &out, const Curve &c) {
        if (c.points.size() < 2)
            return 0;
        const bool three = c.dim == 3;
        auto coords = [&](const std::array<double, 3> &p) {
            return "(" + real(p[0]) + "," + real(p[1]) + (three ? "," + real(p[2]) : std::string()) + ")";
        };
        if (c.kind == "polyline") {
            std::vector<long> ids;
            for (const auto &p : c.points)
                ids.push_back(e(out, "IFCCARTESIANPOINT(" + coords(p) + ")"));
            return e(out, "IFCPOLYLINE(" + refs(ids) + ")");
        }
        if (c.kind != "indexed")
            return 0;
        std::string list = "(";
        for (size_t i = 0; i < c.points.size(); ++i)
            list += (i ? "," : "") + coords(c.points[i]);
        list += ")";
        long pl = e(out, std::string(three ? "IFCCARTESIANPOINTLIST3D(" : "IFCCARTESIANPOINTLIST2D(") + list + ",$)");
        std::string segs = "$";
        if (c.has_segments) {
            segs = "(";
            for (size_t i = 0; i < c.segments.size(); ++i) {
                const auto &[arc, ix] = c.segments[i];
                if (i)
                    segs += ",";
                segs += arc ? "IFCARCINDEX((" : "IFCLINEINDEX((";
                for (size_t k = 0; k < ix.size(); ++k)
                    segs += (k ? "," : "") + std::to_string(ix[k]);
                segs += "))";
            }
            segs += ")";
        }
        return e(out, "IFCINDEXEDPOLYCURVE(" + ref(pl) + "," + segs + ",.F.)");
    }

    long profile(std::string &out, const Profile &p) {
        const std::string name = p.has_name ? "'" + text(p.name) + "'" : std::string("$");
        if (p.kind == "circle")
            return p.radius > 0 ? e(out, "IFCCIRCLEPROFILEDEF(.AREA.," + name + ",$," + real(p.radius) + ")") : 0;
        if (p.kind == "circle_hollow")
            return p.radius > 0 && p.wall_thickness > 0
                       ? e(out, "IFCCIRCLEHOLLOWPROFILEDEF(.AREA.," + name + ",$," + real(p.radius) + "," +
                                    real(p.wall_thickness) + ")")
                       : 0;
        long outer = curve(out, p.outer);
        if (!outer)
            return 0;
        std::vector<long> inner;
        for (const Curve &c : p.inners)
            if (long id = curve(out, c))
                inner.push_back(id);
        if (inner.empty())
            return e(out, "IFCARBITRARYCLOSEDPROFILEDEF(.AREA.," + name + "," + ref(outer) + ")");
        return e(out, "IFCARBITRARYPROFILEDEFWITHVOIDS(.AREA.," + name + "," + ref(outer) + "," + refs(inner) + ")");
    }

    static std::string value(const Value &v) {
        using K = Value::Kind;
        auto typed = [&](const char *by_kind, const std::string &lit) {
            return (v.ifc_type.empty() ? std::string(by_kind) : upper(v.ifc_type)) + "(" + lit + ")";
        };
        switch (v.kind) {
        case K::Real:
            return typed("IFCREAL", real(v.d));
        case K::Int:
            return typed("IFCINTEGER", std::to_string(v.i));
        case K::Bool:
            return typed("IFCBOOLEAN", v.b ? ".T." : ".F.");
        case K::Text:
            return typed("IFCTEXT", "'" + text(v.s) + "'");
        default:
            return "IFCTEXT('')"; // what adapy's own writer states for a value it has none of
        }
    }

    std::vector<long> properties(std::string &out, const PropSet &props) {
        std::vector<long> ids;
        for (const auto &[name, v] : props) {
            if (v.kind == Value::Kind::List) {
                std::string items = "(";
                for (size_t i = 0; i < v.items.size(); ++i)
                    items += (i ? "," : "") + value(v.items[i]);
                items += ")";
                ids.push_back(e(out, "IFCPROPERTYLISTVALUE('" + text(name) + "',$," + items + ",$)"));
            } else {
                ids.push_back(e(out, "IFCPROPERTYSINGLEVALUE('" + text(name) + "',$," + value(v) + ",$)"));
            }
        }
        return ids;
    }

    long material(std::string &out, const Material &mat) {
        auto it = materials_.find(mat.name);
        if (it != materials_.end())
            return it->second;
        long id = e(out, "IFCMATERIAL('" + text(mat.name) + "',$," + opt_text(mat.has_category, mat.category) + ")");
        if (!mat.properties.empty()) {
            std::vector<long> ids = properties(out, mat.properties);
            const std::string set_name =
                mat.properties_name.empty() ? std::string("$") : "'" + text(mat.properties_name) + "'";
            e(out, "IFCMATERIALPROPERTIES(" + set_name + ",$," + refs(ids) + "," + ref(id) + ")");
        }
        materials_[mat.name] = id;
        return id;
    }
};

} // namespace adacpp::ifc_member_emit
