// The IFC member scan as a FILE: one JSON object per member, one member per line.
//
// WHY A FILE. The scan itself streams -- one product resolved at a time, caches dropped between --
// and the nanobind binding preserves that by yielding one dict per `next()`. embind has no
// equivalent that stays cheap: a JS object per member crosses the JS/wasm boundary once per member,
// and a plant has hundreds of thousands of them. Writing JSONL to an OPFS path keeps the streaming
// property on both sides -- the writer holds one member, the reader holds one line -- and reuses the
// IO model the ->GLB modules already use, where the caller names an input path and an output path
// and neither file ever has to fit the wasm heap.
//
// THE SHAPE IS THE PYTHON SHAPE. Field names and units match `IfcMemberScan.next()`'s dict key for
// key, because they describe the same thing: a consumer that moves a clash check from the server to
// the browser should not find the members renamed on arrival.

#ifndef ADACPP_IFC_MEMBER_SCAN_H
#define ADACPP_IFC_MEMBER_SCAN_H

#include <fstream>
#include <string>

#include "../cadit/step/step_reader.h"
#include "../geom/neutral/json_write.h"
#include "ifc_reader.h"

namespace adacpp::ifc_read {

namespace member_json_detail {

inline void vec3(std::string &o, const char *key, const double *v, bool present) {
    o += "\"";
    o += key;
    o += "\":";
    if (!present) {
        o += "null";
        return;
    }
    o += "[";
    o += jsonw::num(v[0]);
    o += ",";
    o += jsonw::num(v[1]);
    o += ",";
    o += jsonw::num(v[2]);
    o += "]";
}

inline void str_field(std::string &o, const char *key, const std::string &v) {
    o += "\"";
    o += key;
    o += "\":\"";
    o += jsonw::escape(v);
    o += "\"";
}

} // namespace member_json_detail

//: The record schema, written as the first line of every scan so a file found on its own says what
//: it is. Bump when a field changes meaning -- adding one does not need a bump, since a reader takes
//: the fields it knows.
constexpr const char *MEMBER_SCAN_SCHEMA = "adacpp.ifc_members/1";

// One member as a single-line JSON object. Coordinates are in METRES and world space, as the
// nanobind scan reports them -- the unit scale is already applied, and the header line carries it
// only so a consumer can tell what the source file stated.
inline std::string member_to_json(const MemberInfo &mi) {
    using namespace member_json_detail;
    std::string o;
    o.reserve(512);
    o += "{\"id\":";
    o += std::to_string(mi.id);
    o += ",";
    str_field(o, "guid", mi.guid);
    o += ",";
    str_field(o, "name", mi.name);
    o += ",";
    str_field(o, "ifc_class", mi.ifc_class);
    o += ",";
    str_field(o, "profile_name", mi.profile_name);
    o += ",";
    str_field(o, "profile_type", mi.profile_type);
    o += ",\"depth\":";
    o += jsonw::num(mi.depth);
    o += ",";

    vec3(o, "p1", mi.p1.data(), mi.has_axis);
    o += ",";
    vec3(o, "p2", mi.p2.data(), mi.has_axis);
    o += ",";

    o += "\"placement\":[";
    for (size_t i = 0; i < mi.placement.size(); ++i) {
        if (i)
            o += ",";
        o += jsonw::num(mi.placement[i]);
    }
    o += "],\"outline\":[";
    for (size_t i = 0; i < mi.outline.size(); ++i) {
        if (i)
            o += ",";
        o += "[";
        o += jsonw::num(mi.outline[i][0]);
        o += ",";
        o += jsonw::num(mi.outline[i][1]);
        o += "]";
    }
    o += "],";

    vec3(o, "origin", mi.pos_origin.data(), mi.has_position);
    o += ",";
    vec3(o, "normal", mi.pos_axis.data(), mi.has_position);
    o += ",";
    vec3(o, "xdir", mi.pos_ref.data(), mi.has_position);
    o += ",";

    str_field(o, "material", mi.material);
    o += ",\"material_props\":{";
    bool first = true;
    for (const auto &kv : mi.material_props) {
        if (!first)
            o += ",";
        first = false;
        o += "\"";
        o += jsonw::escape(kv.first);
        o += "\":";
        o += jsonw::num(kv.second);
    }
    for (const auto &kv : mi.material_text_props) {
        if (!first)
            o += ",";
        first = false;
        o += "\"";
        o += jsonw::escape(kv.first);
        o += "\":\"";
        o += jsonw::escape(kv.second);
        o += "\"";
    }
    o += "}}";
    return o;
}

// Scan `in_path` and write the members to `out_path` as JSONL. Returns the number of MEMBER lines
// written (the header line is not counted), or -1 if the output could not be opened.
//
// Bounded on both ends: one member is held at a time and the stream is flushed by the ofstream's own
// buffer, so a plant-scale IFC costs the index plus one record rather than a model in memory.
inline long write_members_jsonl(const std::string &in_path, const std::string &out_path) {
    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out)
        return -1;

    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(in_path);
    IfcResolver r(idx);
    const std::vector<long> roots = r.proxy_roots();

    out << "{\"schema\":\"" << MEMBER_SCAN_SCHEMA << "\",\"unit_scale\":" << jsonw::num(r.unit_scale())
        << ",\"products\":" << roots.size() << "}\n";

    long written = 0;
    for (long pid : roots) {
        MemberInfo mi = r.product_member(pid);
        r.clear_cache(); // statement/surface caches must not grow across products
        out << member_to_json(mi) << "\n";
        ++written;
    }
    out.flush();
    if (!out)
        return -1;
    return written;
}

} // namespace adacpp::ifc_read

#endif // ADACPP_IFC_MEMBER_SCAN_H
