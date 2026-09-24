// Finding the JOINTS in a set of members -- the geometric half of a clash check, in C++.
//
// WHY HERE. The rules that decide "these members meet, and this is what kind of meeting it is"
// have to give the same answer on a worker and in a browser, or a joint a user picks in one place
// cannot be detailed in the other (the joint id is a hash of member names and the pass that found
// it, and a detail hand-off re-derives joints from those ids). Running the same COMPILED code in
// both is the only arrangement where that holds by construction rather than by discipline -- and
// it is fast enough to run while a panel is open, which a Python runtime shipped to the browser
// is not.
//
// WHAT IS HERE AND WHAT IS NOT. The beam-to-beam pass, which is pure vector arithmetic on member
// axes and needs no CAD kernel. The two PLATE passes need surface distances from a kernel and are
// deliberately absent: a check run here reports that they did not run, which is a different answer
// from "there are no plate joints" and the result document has always distinguished the two.
//
// The arithmetic mirrors adapy's `core/clash_check.py` + `core/vector_utils.py` deliberately and
// exactly -- `intersect_calc`'s least-squares s/t, `is_parallel`'s |sin(angle)| test, the half-
// length overrun rejection, and the point-tolerance node merge that groups several beams meeting
// at one place into ONE joint rather than three pairs.

#ifndef ADACPP_CLASH_JOINTS_H
#define ADACPP_CLASH_JOINTS_H

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace adacpp::clash {

//: adapy's `Config().general_point_tol`, the default both `is_parallel` and the node merge take.
constexpr double DEFAULT_POINT_TOL = 1e-5;
//: adapy's `Connections.find(out_of_plane_tol=...)` default.
constexpr double DEFAULT_OUT_OF_PLANE_TOL = 0.1;

struct Member {
    std::string name;
    std::string guid;
    std::array<double, 3> p1{0, 0, 0};
    std::array<double, 3> p2{0, 0, 0};
    //! Half the largest cross-section dimension: how far the member's SOLID reaches from its axis.
    //! Used only to widen the candidate box, never to decide a joint.
    double reach = 0.0;
    //! The section FAMILY ("I", "BOX", "HP", ...) -- what a type key is allowed to name, never the
    //! full designation.
    std::string section;
};

struct Joint {
    std::vector<size_t> members; //!< indices into the input, in the order the passes found them
    std::array<double, 3> centre{0, 0, 0};
    std::string origin = "beam-beam";
};

namespace detail {

inline std::array<double, 3> sub(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline double dot(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline double norm(const std::array<double, 3> &a) {
    return std::sqrt(dot(a, a));
}

// |sin(angle between)| < tol -- adapy's `is_parallel`. Written from the cross product rather than
// acos+sin, which is the same number without the round trip through an angle.
inline bool is_parallel(const std::array<double, 3> &ab, const std::array<double, 3> &cd, double tol) {
    const std::array<double, 3> cross = {
        ab[1] * cd[2] - ab[2] * cd[1],
        ab[2] * cd[0] - ab[0] * cd[2],
        ab[0] * cd[1] - ab[1] * cd[0],
    };
    const double denom = norm(ab) * norm(cd);
    if (denom <= 0.0)
        return true; // a zero-length member is parallel to everything; it is also not a joint
    return norm(cross) / denom < tol;
}

// s, t with A + s*AB = C + t*CD in the least-squares sense -- adapy's `intersect_calc`, which
// solves the 3x2 system with lstsq. For a full-rank system the normal equations give the same
// answer, and rank deficiency is exactly the parallel case the caller rejects first.
inline bool intersect_calc(const std::array<double, 3> &a, const std::array<double, 3> &c,
                           const std::array<double, 3> &ab, const std::array<double, 3> &cd, double &s, double &t) {
    const double m00 = dot(ab, ab), m01 = -dot(ab, cd), m11 = dot(cd, cd);
    const std::array<double, 3> vec = sub(c, a);
    const double b0 = dot(ab, vec), b1 = -dot(cd, vec);
    const double det = m00 * m11 - m01 * m01;
    if (std::abs(det) < 1e-300)
        return false;
    s = (b0 * m11 - m01 * b1) / det;
    t = (m00 * b1 - m01 * b0) / det;
    return true;
}

} // namespace detail

/** Every beam-to-beam joint among `members`.
 *
 *  One joint per CONTACT POINT, not per pair: three beams meeting at a node are one joint with
 *  three members, which is what a person sees and what a connection spec is written against.
 *  Points within `point_tol` of each other are the same point.
 */
inline std::vector<Joint> find_beam_joints(const std::vector<Member> &members,
                                           double out_of_plane_tol = DEFAULT_OUT_OF_PLANE_TOL,
                                           double point_tol = DEFAULT_POINT_TOL) {
    using namespace detail;
    std::vector<Joint> joints;
    const size_t n = members.size();
    if (n < 2)
        return joints;

    // Axis-aligned box per member, padded by its own reach -- the candidate filter. It only has to
    // admit every pair that could touch; the decision below is exact, so a generous box costs time
    // and never an answer.
    std::vector<std::array<double, 6>> box(n);
    for (size_t i = 0; i < n; ++i) {
        const Member &m = members[i];
        const double pad = m.reach + point_tol;
        for (int k = 0; k < 3; ++k) {
            box[i][k] = std::min(m.p1[k], m.p2[k]) - pad;
            box[i][k + 3] = std::max(m.p1[k], m.p2[k]) + pad;
        }
    }
    auto overlaps = [&](size_t i, size_t j) {
        for (int k = 0; k < 3; ++k)
            if (box[i][k] > box[j][k + 3] || box[j][k] > box[i][k + 3])
                return false;
        return true;
    };

    // Contact points, merged by point_tol. Linear scan: a node count is small next to a member
    // count, and it keeps the merge order identical to adapy's `Nodes.add`, which matters because
    // the ORDER beams join a node in is the order they appear in the joint.
    std::vector<std::array<double, 3>> nodes;
    std::vector<std::vector<size_t>> at_node;

    auto node_for = [&](const std::array<double, 3> &p) -> size_t {
        for (size_t k = 0; k < nodes.size(); ++k) {
            if (norm(sub(nodes[k], p)) <= point_tol)
                return k;
        }
        nodes.push_back(p);
        at_node.emplace_back();
        return nodes.size() - 1;
    };
    auto join = [&](size_t node, size_t member) {
        std::vector<size_t> &list = at_node[node];
        if (std::find(list.begin(), list.end(), member) == list.end())
            list.push_back(member);
    };

    for (size_t i = 0; i < n; ++i) {
        const Member &b1 = members[i];
        const std::array<double, 3> ab = sub(b1.p2, b1.p1);
        const double len1 = norm(ab);
        if (len1 <= 0.0)
            continue;
        // Each unordered pair ONCE. Evaluating both directions would register two contact points
        // for a single near-miss -- one on each member's own line, up to `out_of_plane_tol` apart
        // and so too far to merge -- and report one physical contact as two joints.
        for (size_t j = i + 1; j < n; ++j) {
            if (!overlaps(i, j))
                continue;
            const Member &b2 = members[j];
            const std::array<double, 3> cd = sub(b2.p2, b2.p1);
            const double len2 = norm(cd);
            if (len2 <= 0.0 || is_parallel(ab, cd, point_tol))
                continue;

            double s = 0, t = 0;
            if (!intersect_calc(b1.p1, b2.p1, ab, cd, s, t))
                continue;
            const std::array<double, 3> ab_ = {b1.p1[0] + s * ab[0], b1.p1[1] + s * ab[1], b1.p1[2] + s * ab[2]};
            const std::array<double, 3> cd_ = {b2.p1[0] + t * cd[0], b2.p1[1] + t * cd[1], b2.p1[2] + t * cd[2]};
            if (norm(sub(ab_, cd_)) > out_of_plane_tol)
                continue; // the lines pass each other rather than meeting

            // How far PAST an end the intersection lies. Beyond half a member's own length it is
            // not this member's joint -- two beams in the same plane always "intersect" somewhere.
            if ((std::abs(t) - 1.0) * len2 > len2 / 2.0 || (std::abs(s) - 1.0) * len1 > len1 / 2.0)
                continue;

            // The MIDPOINT of the two closest points. For lines that actually meet the two are
            // the same point and this is exactly that point; for a near miss inside the tolerance
            // it is the honest centre of the contact rather than an arbitrary one of its sides.
            const std::array<double, 3> contact = {(ab_[0] + cd_[0]) / 2.0, (ab_[1] + cd_[1]) / 2.0,
                                                   (ab_[2] + cd_[2]) / 2.0};
            const size_t node = node_for(contact);
            join(node, i);
            join(node, j);
        }
    }

    for (size_t k = 0; k < nodes.size(); ++k) {
        if (at_node[k].size() < 2)
            continue;
        Joint j;
        j.members = at_node[k];
        // BY NAME, not by discovery order. A joint of three or more members has no single angle,
        // and the consumer takes the one between its first two -- so the order the pairs happened
        // to be walked in would decide the joint's angle bucket, and with it its TYPE KEY. Sorting
        // makes that choice reproducible: the same model yields the same key on any run, any
        // platform and either runtime. It is still only one pair's angle, which is a limitation of
        // the question rather than of the ordering.
        std::sort(j.members.begin(), j.members.end(),
                  [&](size_t a, size_t b) { return members[a].name < members[b].name; });
        j.centre = nodes[k];
        joints.push_back(std::move(j));
    }
    return joints;
}

// ── classification ──────────────────────────────────────────────────────────────────────────
//
// The same closed vocabulary adapy's `clash/classify.py` builds a type key from. Nothing here may
// name a fabrication process or a vendor system; a key is member kinds, section families, member
// types and an angle bucket, sorted so it is stable.

inline std::string angle_bucket(double angle_deg, bool has_angle) {
    if (!has_angle)
        return "unknown"; // a real value: a contact with no pair of axes has no angle to take
    double folded = std::fmod(angle_deg + 180.0, 360.0);
    if (folded < 0)
        folded += 360.0;
    folded = std::abs(folded - 180.0);
    if (folded <= 15.0 || folded >= 165.0)
        return "parallel";
    if (folded >= 75.0 && folded <= 105.0)
        return "perpendicular";
    return "skew";
}

/** "Column" | "Girder" | "Brace", from the axis alone -- adapy's `Beam.member_type`. */
inline std::string member_type_of(const Member &m) {
    using namespace detail;
    const std::array<double, 3> x = sub(m.p2, m.p1);
    const double len = norm(x);
    if (len <= 0.0)
        return "";
    const std::array<double, 3> up = {0.0, 0.0, 1.0};
    if (is_parallel(x, up, 1e-1))
        return "Column";
    if (x[2] == 0.0)
        return "Girder";
    return "Brace";
}

/** The angle between two members' axes, in degrees. */
inline double angle_between(const Member &a, const Member &b) {
    using namespace detail;
    const std::array<double, 3> u = sub(a.p2, a.p1), v = sub(b.p2, b.p1);
    const double du = norm(u), dv = norm(v);
    if (du <= 0.0 || dv <= 0.0)
        return 0.0;
    double c = dot(u, v) / (du * dv);
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

inline std::string member_token(const Member &m) {
    std::string token = "BEAM";
    if (!m.section.empty())
        token += ":" + m.section;
    const std::string mt = member_type_of(m);
    if (!mt.empty()) {
        std::string upper = mt;
        for (char &c : upper)
            c = (char) std::toupper((unsigned char) c);
        token += ":" + upper;
    }
    return token;
}

/** `<n>|<member token>+…|<angle bucket>` -- adapy's `type_key_for`, sorted and stable. */
inline std::string type_key_for(const std::vector<Member> &members, const std::vector<size_t> &idx, double angle_deg,
                                bool has_angle) {
    std::vector<std::string> tokens;
    tokens.reserve(idx.size());
    for (size_t i : idx)
        tokens.push_back(member_token(members[i]));
    std::sort(tokens.begin(), tokens.end());
    std::string joined;
    for (size_t k = 0; k < tokens.size(); ++k) {
        if (k)
            joined += "+";
        joined += tokens[k];
    }
    return std::to_string(idx.size()) + "|" + joined + "|" + angle_bucket(angle_deg, has_angle);
}

} // namespace adacpp::clash

#endif // ADACPP_CLASH_JOINTS_H
