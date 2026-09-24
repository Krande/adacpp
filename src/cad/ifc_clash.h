// A clash check straight from an IFC, in one pass and one language.
//
// The browser's route. `IfcMemberScan` already states what the members of an IFC ARE, and
// `clash_joints.h` already decides which of them meet; putting the two together HERE means a
// browser check never materialises the members as JSON on the way -- it reads the file and writes
// the joints, with nothing crossing a language boundary in between.
//
// The joints are written as JSON in the shape adapy's `ada.clash/result@1` document uses for its
// `joints` array, so a consumer parses one thing whichever route produced it.

#ifndef ADACPP_IFC_CLASH_H
#define ADACPP_IFC_CLASH_H

#include <fstream>
#include <string>
#include <vector>

#include "../geom/neutral/json_write.h"
#include "clash_joints.h"
#include "ifc_reader.h"

namespace adacpp::ifc_read {

namespace clash_detail {

// Half the largest cross-section dimension, from the profile outline the scan reports. This only
// pads the candidate box, so an absent outline (a member whose profile the reader could not build)
// costs a slightly tighter filter and never a wrong answer.
inline double reach_of(const MemberInfo &mi) {
    double lo0 = 0, hi0 = 0, lo1 = 0, hi1 = 0;
    bool first = true;
    for (const auto &pt : mi.outline) {
        if (first) {
            lo0 = hi0 = pt[0];
            lo1 = hi1 = pt[1];
            first = false;
            continue;
        }
        lo0 = std::min(lo0, pt[0]);
        hi0 = std::max(hi0, pt[0]);
        lo1 = std::min(lo1, pt[1]);
        hi1 = std::max(hi1, pt[1]);
    }
    return first ? 0.0 : std::max(hi0 - lo0, hi1 - lo1) / 2.0;
}

// The section FAMILY a type key may name, from the profile entity the file used. Never the full
// designation ("IPE220"), which no connection spec compares against.
inline std::string family_of(const MemberInfo &mi) {
    const std::string &t = mi.profile_type;
    if (t == "IFCISHAPEPROFILEDEF")
        return "I";
    if (t == "IFCRECTANGLEHOLLOWPROFILEDEF" || t == "IFCRECTANGLEPROFILEDEF")
        return "BOX";
    if (t == "IFCCIRCLEHOLLOWPROFILEDEF" || t == "IFCCIRCLEPROFILEDEF")
        return "CIRC";
    if (t == "IFCTSHAPEPROFILEDEF")
        return "TG";
    if (t == "IFCUSHAPEPROFILEDEF")
        return "CHANNEL";
    if (t == "IFCLSHAPEPROFILEDEF")
        return "ANGULAR";
    return t.empty() ? "" : "POLY";
}

} // namespace clash_detail

/** The beams an IFC states, as the clash finder wants them. */
inline std::vector<clash::Member> ifc_beam_members(const std::string &ifc_path) {
    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(ifc_path);
    IfcResolver r(idx);
    std::vector<clash::Member> out;
    for (long pid : r.proxy_roots()) {
        MemberInfo mi = r.product_member(pid);
        r.clear_cache();
        // Beams only: the plate passes need surface distances from a CAD kernel, which this
        // module deliberately does not have. A check run here says so rather than reporting the
        // plate joints it never looked for as absent.
        if (!mi.has_axis)
            continue;
        const std::string &c = mi.ifc_class;
        if (c != "IFCBEAM" && c != "IFCCOLUMN" && c != "IFCMEMBER")
            continue;
        clash::Member m;
        m.name = mi.name.empty() ? mi.guid : mi.name;
        m.guid = mi.guid;
        m.p1 = mi.p1;
        m.p2 = mi.p2;
        m.reach = clash_detail::reach_of(mi);
        m.section = clash_detail::family_of(mi);
        out.push_back(std::move(m));
    }
    return out;
}

/** Read `ifc_path`, find its beam-to-beam joints, write them to `out_path` as JSON.
 *
 *  Returns the number of joints, or -1 if the output could not be written.
 */
inline long write_beam_joints_json(const std::string &ifc_path, const std::string &out_path,
                                   double out_of_plane_tol = clash::DEFAULT_OUT_OF_PLANE_TOL,
                                   double point_tol = clash::DEFAULT_POINT_TOL) {
    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out)
        return -1;

    const std::vector<clash::Member> members = ifc_beam_members(ifc_path);
    const std::vector<clash::Joint> joints = clash::find_beam_joints(members, out_of_plane_tol, point_tol);

    out << "{\"schema\":\"adacpp.clash_joints/1\",\"beams\":" << members.size() << ",\"joints\":[";
    for (size_t k = 0; k < joints.size(); ++k) {
        const clash::Joint &j = joints[k];
        const bool has_angle = j.members.size() >= 2;
        const double angle = has_angle ? clash::angle_between(members[j.members[0]], members[j.members[1]]) : 0.0;
        if (k)
            out << ",";
        out << "{\"origin\":\"" << j.origin << "\",\"centre\":[" << jsonw::num(j.centre[0]) << ","
            << jsonw::num(j.centre[1]) << "," << jsonw::num(j.centre[2])
            << "],\"angle_deg\":" << (has_angle ? jsonw::num(angle) : std::string("null")) << ",\"type_key\":\""
            << jsonw::escape(clash::type_key_for(members, j.members, angle, has_angle)) << "\",\"members\":[";
        for (size_t i = 0; i < j.members.size(); ++i) {
            const clash::Member &m = members[j.members[i]];
            if (i)
                out << ",";
            out << "{\"name\":\"" << jsonw::escape(m.name) << "\",\"guid\":\"" << jsonw::escape(m.guid)
                << "\",\"kind\":\"BEAM\",\"section\":\"" << jsonw::escape(m.section) << "\",\"member_type\":\""
                << jsonw::escape(clash::member_type_of(m)) << "\"}";
        }
        out << "]}";
    }
    out << "]}\n";
    out.flush();
    if (!out)
        return -1;
    return (long) joints.size();
}

} // namespace adacpp::ifc_read

#endif // ADACPP_IFC_CLASH_H
