// Convex-contact clash detection on a viewer GLB: the members' geometry exactly as the viewer shows it.
//
// WHY THE GLB. A clash check measures solids, and the one geometry a browser and a worker can both
// hold, byte for byte, is the GLB the viewer loads -- a derived GLB for a file, a built or published
// GLB for an asset node. Reading members out of those bytes here, in C++ compiled for both, means the
// browser (embind) and the worker (nanobind) measure the same triangles with the same code, so a
// clash found in one is the clash the other finds.
//
// WHAT A MEMBER IS. The viewer's GLB is merged by material: one mesh node per material, and per node a
// `draw_ranges_<node>` table in the scene extras mapping an element id to its [start, count) slice of
// that node's index buffer, with `id_hierarchy[id] = [name, parent]` naming it. An element split over
// several materials appears in several nodes under the SAME id, so members are keyed by id (names may
// repeat; ids do not) and gathered across nodes. This is the mapping the viewer's picking uses.
//
// COORDINATES. Positions are taken through each node's world transform (matrix or TRS, down from the
// scene's roots), then from the GLB's up axis to Z-up -- the model's own frame, which is what a
// connection builder cuts members in. adapy's own GLBs are Z-up already ("z", the default); a
// provider's mesh delivery declares "y" when it follows glTF's Y-up convention.
//
// Header-only: it needs nlohmann json and meshoptimizer, which every target that links it already
// compiles (the nanobind module, the adacpp_clash wasm module).

#ifndef ADACPP_CLASH_GLB_CONTACTS_H
#define ADACPP_CLASH_GLB_CONTACTS_H

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../diff/glb_diff_native.h" // resolve_accessor: raw or EXT_meshopt_compression bufferViews
#include "convex_contact.h"

namespace adacpp::clash {

/** One member read out of a GLB: its element id, its name, its vertices (xyz, Z-up model frame). */
struct GlbMember {
    std::string id;
    std::string name;
    std::vector<double> vertices;
};

struct GlbContactResult {
    std::vector<GlbMember> members;    //!< in first-appearance order; contact indices refer to this
    ConvexContactResult contacts;
};

namespace glb_detail {

using Mat4 = std::array<double, 16>; // column-major, as glTF writes it

inline Mat4 identity() {
    return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

inline Mat4 mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + row] * b[c * 4 + k];
            r[c * 4 + row] = s;
        }
    return r;
}

inline Mat4 local_matrix(const gdiff::njson &node) {
    if (node.contains("matrix") && node["matrix"].is_array() && node["matrix"].size() == 16) {
        Mat4 m{};
        for (int k = 0; k < 16; ++k)
            m[k] = node["matrix"][k].get<double>();
        return m;
    }
    // T * R * S
    double t[3] = {0, 0, 0}, q[4] = {0, 0, 0, 1}, s[3] = {1, 1, 1};
    if (node.contains("translation"))
        for (int k = 0; k < 3; ++k)
            t[k] = node["translation"][k].get<double>();
    if (node.contains("rotation"))
        for (int k = 0; k < 4; ++k)
            q[k] = node["rotation"][k].get<double>();
    if (node.contains("scale"))
        for (int k = 0; k < 3; ++k)
            s[k] = node["scale"][k].get<double>();
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                         2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    Mat4 m{};
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            m[c * 4 + row] = r[c * 3 + row] * s[c];
    m[12] = t[0];
    m[13] = t[1];
    m[14] = t[2];
    m[15] = 1;
    return m;
}

inline bool is_identity(const Mat4 &m) {
    const Mat4 i = identity();
    for (int k = 0; k < 16; ++k)
        if (m[k] != i[k])
            return false;
    return true;
}

} // namespace glb_detail

/** The members of a viewer GLB with their vertices in the Z-up model frame.
 *
 *  `up_axis` is the GLB's up axis: "z" (adapy's GLBs; no conversion) or "y" (glTF's convention: the
 *  inverse of the Z-up -> Y-up export rotation, (x, y, z) -> (x, -z, y)). A member's vertices are the
 *  distinct vertices its triangles use, in first-use order. Throws std::invalid_argument for bytes
 *  that are not a GLB the viewer could have loaded. */
inline std::vector<GlbMember> glb_members(const unsigned char *data, std::size_t size,
                                          const std::string &up_axis = "z") {
    using gdiff::njson;
    if (up_axis != "z" && up_axis != "y")
        throw std::invalid_argument("up_axis must be \"z\" or \"y\", got \"" + up_axis + "\"");
    if (size < 20 || std::memcmp(data, "glTF", 4) != 0)
        throw std::invalid_argument("not a GLB (no glTF magic)");
    std::uint32_t json_len;
    std::memcpy(&json_len, data + 12, 4);
    if (std::size_t(20) + json_len > size)
        throw std::invalid_argument("truncated GLB (JSON chunk runs past the end)");
    const bool has_bin = std::size_t(20) + json_len + 8 <= size;
    const unsigned char *bin = has_bin ? data + 20 + json_len + 8 : nullptr;

    njson j;
    try {
        const char *jbeg = reinterpret_cast<const char *>(data + 20);
        j = njson::parse(jbeg, jbeg + json_len);
    } catch (const std::exception &e) {
        throw std::invalid_argument(std::string("GLB JSON does not parse: ") + e.what());
    }
    std::vector<GlbMember> members;
    if (!j.contains("scenes") || !j.contains("nodes") || !j.contains("meshes") || !j.contains("accessors") ||
        !j.contains("bufferViews"))
        return members;
    const njson &scene0 = j["scenes"][j.value("scene", 0)];
    if (!scene0.contains("extras"))
        return members; // not a viewer GLB: no element table, so no members to name
    const njson &extras = scene0["extras"];
    const njson &nodes = j["nodes"];
    const njson &meshes = j["meshes"];
    const njson &accessors = j["accessors"];
    const njson &bufferViews = j["bufferViews"];

    // World matrix per node, down from the scene's roots.
    std::vector<glb_detail::Mat4> world(nodes.size(), glb_detail::identity());
    std::vector<int> stack;
    std::vector<glb_detail::Mat4> parent_of_stack;
    if (scene0.contains("nodes"))
        for (const njson &r : scene0["nodes"]) {
            stack.push_back(r.get<int>());
            parent_of_stack.push_back(glb_detail::identity());
        }
    while (!stack.empty()) {
        const int n = stack.back();
        const glb_detail::Mat4 parent = parent_of_stack.back();
        stack.pop_back();
        parent_of_stack.pop_back();
        if (n < 0 || std::size_t(n) >= nodes.size())
            continue;
        world[n] = glb_detail::mul(parent, glb_detail::local_matrix(nodes[n]));
        if (nodes[n].contains("children"))
            for (const njson &c : nodes[n]["children"]) {
                stack.push_back(c.get<int>());
                parent_of_stack.push_back(world[n]);
            }
    }

    std::unordered_map<std::string, std::string> name_of;
    if (extras.contains("id_hierarchy"))
        for (auto it = extras["id_hierarchy"].begin(); it != extras["id_hierarchy"].end(); ++it)
            if (it.value().is_array() && !it.value().empty() && it.value()[0].is_string())
                name_of[it.key()] = it.value()[0].get<std::string>();

    std::unordered_map<std::string, int> node_idx;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i].contains("name") && nodes[i]["name"].is_string())
            node_idx[nodes[i]["name"].get<std::string>()] = int(i);

    std::unordered_map<std::string, std::size_t> member_of; // element id -> index into `members`
    const bool y_up = up_axis == "y";
    const std::string prefix = "draw_ranges_";
    for (auto kit = extras.begin(); kit != extras.end(); ++kit) {
        if (kit.key().rfind(prefix, 0) != 0 || !kit.value().is_object())
            continue;
        auto nit = node_idx.find(kit.key().substr(prefix.size()));
        if (nit == node_idx.end())
            continue;
        const njson &node = nodes[nit->second];
        if (!node.contains("mesh"))
            continue;
        const njson &prim = meshes[node["mesh"].get<int>()]["primitives"][0];
        if (!prim.contains("indices") || !prim["attributes"].contains("POSITION"))
            continue;
        if (!bin)
            throw std::invalid_argument("GLB has meshes but no BIN chunk");
        const njson &pacc = accessors[prim["attributes"]["POSITION"].get<int>()];
        if (pacc.value("componentType", 5126) != 5126 || pacc.value("normalized", false))
            throw std::invalid_argument("GLB positions are not float32 (quantized meshes are not supported)");
        const njson &iacc = accessors[prim["indices"].get<int>()];
        const int icomp = iacc.value("componentType", 5125);
        const std::size_t isz = icomp == 5125 ? 4 : icomp == 5123 ? 2 : 1;
        gdiff::ByteView posv = gdiff::resolve_accessor(pacc, bufferViews, bin, 12, false);
        gdiff::ByteView idxv = gdiff::resolve_accessor(iacc, bufferViews, bin, isz, false);
        const std::size_t idx_count = iacc.at("count").get<std::size_t>();
        const std::size_t pos_count = pacc.at("count").get<std::size_t>();
        const glb_detail::Mat4 &m = world[nit->second];
        const bool identity = glb_detail::is_identity(m);

        auto read_idx = [&](std::size_t i) -> std::uint32_t {
            const unsigned char *p = idxv.ptr + i * idxv.stride;
            if (isz == 4) {
                std::uint32_t v;
                std::memcpy(&v, p, 4);
                return v;
            }
            if (isz == 2) {
                std::uint16_t v;
                std::memcpy(&v, p, 2);
                return v;
            }
            return *p;
        };

        for (auto rit = kit.value().begin(); rit != kit.value().end(); ++rit) {
            const njson &rng = rit.value();
            if (!rng.is_array() || rng.size() < 2)
                continue;
            const std::size_t start = rng[0].get<std::size_t>(), count = rng[1].get<std::size_t>();
            if (count == 0 || start + count > idx_count)
                continue;
            const std::string &id = rit.key();
            auto [mit, fresh] = member_of.emplace(id, members.size());
            if (fresh) {
                auto name = name_of.find(id);
                members.push_back({id, name != name_of.end() ? name->second : id, {}});
            }
            GlbMember &member = members[mit->second];
            // Distinct vertices only, in first-use order: an element's triangles share most of them.
            std::unordered_set<std::uint32_t> seen;
            for (std::size_t i = start; i < start + count; ++i) {
                const std::uint32_t vi = read_idx(i);
                if (vi >= pos_count)
                    throw std::invalid_argument("GLB index past its positions (element " + id + ")");
                if (!seen.insert(vi).second)
                    continue;
                float f[3];
                std::memcpy(f, posv.ptr + std::size_t(vi) * posv.stride, 12);
                double p[3] = {f[0], f[1], f[2]};
                if (!identity) {
                    const double q[3] = {m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
                                         m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
                                         m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]};
                    p[0] = q[0];
                    p[1] = q[1];
                    p[2] = q[2];
                }
                if (y_up) {
                    const double y = p[1];
                    p[1] = -p[2];
                    p[2] = y;
                }
                member.vertices.insert(member.vertices.end(), {p[0], p[1], p[2]});
            }
        }
    }
    return members;
}

/** Every touching / overlapping pair of members of a viewer GLB (see glb_members). The hull of each
 *  member is taken (`options.hull` is forced on: a GLB element is a triangle soup, not a hull). */
inline GlbContactResult find_glb_contacts(const unsigned char *data, std::size_t size,
                                          ConvexContactOptions options = {}, const std::string &up_axis = "z") {
    GlbContactResult out;
    out.members = glb_members(data, size, up_axis);
    std::vector<MeshView> views(out.members.size());
    for (std::size_t k = 0; k < out.members.size(); ++k)
        views[k] = {out.members[k].vertices.data(), out.members[k].vertices.size() / 3, nullptr, 0};
    options.hull = true;
    out.contacts = find_convex_contacts(views, options);
    return out;
}

} // namespace adacpp::clash

#endif // ADACPP_CLASH_GLB_CONTACTS_H
