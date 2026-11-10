#include "glb_scene.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <tuple>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#ifdef __EMSCRIPTEN__
// The renderer does not texture: no image decoder in the browser build (load_glb_bytes installs a no-op
// image loader on every host anyway).
#define TINYGLTF_NO_STB_IMAGE
#else
#define STB_IMAGE_IMPLEMENTATION
#endif
#include <tiny_gltf.h>

#include <meshoptimizer.h>

namespace adacpp::render {

namespace {

using Mat4 = std::array<float, 16>;

Mat4 mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + row] * b[c * 4 + k];
            r[c * 4 + row] = s;
        }
    return r;
}

Mat4 node_matrix(const tinygltf::Node &n) {
    Mat4 m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    if (n.matrix.size() == 16) {
        for (int i = 0; i < 16; ++i)
            m[i] = float(n.matrix[i]);
        return m;
    }
    double t[3] = {0, 0, 0}, s[3] = {1, 1, 1}, q[4] = {0, 0, 0, 1};
    for (size_t i = 0; i < n.translation.size() && i < 3; ++i)
        t[i] = n.translation[i];
    for (size_t i = 0; i < n.scale.size() && i < 3; ++i)
        s[i] = n.scale[i];
    for (size_t i = 0; i < n.rotation.size() && i < 4; ++i)
        q[i] = n.rotation[i];
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                         2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            m[c * 4 + row] = float(r[c * 3 + row] * s[c]);
    m[12] = float(t[0]);
    m[13] = float(t[1]);
    m[14] = float(t[2]);
    return m;
}

// Read an accessor as floats (`comps` per element), honouring stride, component type and normalisation.
std::vector<float> read_floats(const tinygltf::Model &model, int accessor_index, int comps) {
    const auto &acc = model.accessors.at(accessor_index);
    const auto &view = model.bufferViews.at(acc.bufferView);
    const auto &buf = model.buffers.at(view.buffer);
    const int ncomp = tinygltf::GetNumComponentsInType(acc.type);
    const int csize = tinygltf::GetComponentSizeInBytes(acc.componentType);
    const size_t stride = view.byteStride ? view.byteStride : size_t(ncomp * csize);
    const uint8_t *base = buf.data.data() + view.byteOffset + acc.byteOffset;
    std::vector<float> out(acc.count * comps, 1.0f);
    for (size_t i = 0; i < acc.count; ++i) {
        const uint8_t *e = base + i * stride;
        for (int c = 0; c < std::min(ncomp, comps); ++c) {
            float v = 0.0f;
            switch (acc.componentType) {
            case TINYGLTF_COMPONENT_TYPE_FLOAT:
                v = reinterpret_cast<const float *>(e)[c];
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                v = e[c] / (acc.normalized || comps == 4 ? 255.0f : 1.0f);
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                v = reinterpret_cast<const uint16_t *>(e)[c] / (acc.normalized || comps == 4 ? 65535.0f : 1.0f);
                break;
            default:
                throw std::runtime_error("adacpp.render: unsupported accessor component type");
            }
            out[i * comps + c] = v;
        }
    }
    return out;
}

std::vector<uint32_t> read_indices(const tinygltf::Model &model, int accessor_index) {
    const auto &acc = model.accessors.at(accessor_index);
    const auto &view = model.bufferViews.at(acc.bufferView);
    const auto &buf = model.buffers.at(view.buffer);
    const int csize = tinygltf::GetComponentSizeInBytes(acc.componentType);
    const size_t stride = view.byteStride ? view.byteStride : size_t(csize);
    const uint8_t *base = buf.data.data() + view.byteOffset + acc.byteOffset;
    std::vector<uint32_t> out(acc.count);
    for (size_t i = 0; i < acc.count; ++i) {
        const uint8_t *e = base + i * stride;
        switch (acc.componentType) {
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            out[i] = *e;
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            out[i] = *reinterpret_cast<const uint16_t *>(e);
            break;
        default:
            out[i] = *reinterpret_cast<const uint32_t *>(e);
        }
    }
    return out;
}

uint32_t pack_rgba8(const float *c) {
    auto q = [](float v) { return uint32_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); };
    return q(c[0]) | (q(c[1]) << 8) | (q(c[2]) << 16) | (q(c[3]) << 24);
}

// The model's objects while the scene is read (GlbLoadResult::objects): filled in the order the GLB gives
// them, then put parents first by finish().
class ObjectTable {
public:
    // adapy's "id_hierarchy": {id: [name, parent id]}, the parent of a root being "*".
    void add_hierarchy(const tinygltf::Value &hierarchy) {
        std::vector<std::string> parents;
        for (const auto &id : hierarchy.Keys()) {
            const auto &v = hierarchy.Get(id);
            const int32_t at = by_id(id);
            parents.resize(objects_.size());
            if (!v.IsArray() || v.ArrayLen() < 1)
                continue;
            objects_[at].name = text(v.Get(0));
            if (v.ArrayLen() > 1)
                parents[at] = text(v.Get(1));
        }
        for (size_t i = 0; i < parents.size(); ++i)
            if (auto it = index_.find(parents[i]); it != index_.end() && it->second != int32_t(i))
                objects_[i].parent = it->second;
        sort_by_id_ = true;
    }

    // The object with this id; one the hierarchy does not list becomes a root named by its id.
    int32_t by_id(const std::string &id) {
        auto [it, fresh] = index_.try_emplace(id, int32_t(objects_.size()));
        if (fresh) {
            GlbObject o;
            o.id = o.name = id;
            objects_.push_back(std::move(o));
        }
        return it->second;
    }

    // The object of a glTF node, under its parent node's.
    int32_t for_node(const tinygltf::Model &model, int node_index, int32_t parent) {
        auto [it, fresh] = index_.try_emplace(std::to_string(node_index), int32_t(objects_.size()));
        if (fresh) {
            const auto &node = model.nodes.at(node_index);
            GlbObject o;
            o.id = it->first;
            o.name = !node.name.empty() || node.mesh < 0 ? node.name : model.meshes.at(node.mesh).name;
            o.parent = parent;
            objects_.push_back(std::move(o));
        }
        return it->second;
    }

    void add_segment(int32_t object, uint32_t instance, uint32_t segment, const float bounds[6], uint32_t triangles) {
        GlbObject &o = objects_[object];
        o.segments.push_back({instance, segment});
        o.triangles += triangles;
        for (int k = 0; k < 3; ++k) {
            o.bounds[k] = std::min(o.bounds[k], bounds[k]);
            o.bounds[3 + k] = std::max(o.bounds[3 + k], bounds[3 + k]);
        }
    }

    // Moves the objects into `res` parents first, and points every geometry's segments at their new places.
    void finish(GlbLoadResult &res) {
        const size_t n = objects_.size();
        std::vector<int32_t> order(n);
        for (size_t i = 0; i < n; ++i)
            order[i] = int32_t(i);
        if (sort_by_id_)
            std::sort(order.begin(), order.end(),
                      [&](int32_t a, int32_t b) { return id_less(objects_[a].id, objects_[b].id); });
        std::vector<std::vector<int32_t>> children(n);
        std::vector<int32_t> roots;
        for (int32_t i : order)
            (objects_[i].parent >= 0 ? children[objects_[i].parent] : roots).push_back(i);
        std::vector<int32_t> place(n, -1), stack;
        int32_t next = 0;
        auto descend = [&](int32_t root) {
            stack.assign(1, root);
            while (!stack.empty()) {
                const int32_t i = stack.back();
                stack.pop_back();
                if (place[i] >= 0)
                    continue;
                place[i] = next++;
                for (auto c = children[i].rbegin(); c != children[i].rend(); ++c)
                    stack.push_back(*c);
            }
        };
        for (int32_t r : roots)
            descend(r);
        // What no root reaches is a cycle of parents: cut it at its first object.
        for (int32_t i : order)
            if (place[i] < 0) {
                objects_[i].parent = -1;
                descend(i);
            }
        res.objects.resize(n);
        for (size_t i = 0; i < n; ++i) {
            GlbObject &o = objects_[i];
            if (o.parent >= 0)
                o.parent = place[o.parent];
            res.objects[place[i]] = std::move(o);
        }
        // Children sit after their parent: one pass from the end sums every subtree into its root.
        for (size_t i = n; i-- > 0;) {
            const GlbObject &o = res.objects[i];
            if (o.parent < 0)
                continue;
            GlbObject &up = res.objects[o.parent];
            up.triangles += o.triangles;
            for (int k = 0; k < 3; ++k) {
                up.bounds[k] = std::min(up.bounds[k], o.bounds[k]);
                up.bounds[3 + k] = std::max(up.bounds[3 + k], o.bounds[3 + k]);
            }
        }
        for (GlbGeometry &g : res.loaded)
            for (int32_t &object : g.segment_objects)
                if (object >= 0)
                    object = place[object];
        objects_.clear();
        index_.clear();
    }

private:
    static std::string text(const tinygltf::Value &v) {
        if (v.IsString())
            return v.Get<std::string>();
        if (v.IsInt())
            return std::to_string(v.Get<int>());
        if (v.IsReal())
            return std::to_string(int64_t(v.Get<double>()));
        return {};
    }

    // Numbers by value ("9" before "10"), anything else as text after them.
    static bool id_less(const std::string &a, const std::string &b) {
        auto digits = [](const std::string &s) {
            return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
        };
        const bool da = digits(a), db = digits(b);
        if (da != db)
            return da;
        if (da && a.size() != b.size())
            return a.size() < b.size();
        return a < b;
    }

    std::vector<GlbObject> objects_;
    std::unordered_map<std::string, int32_t> index_;
    bool sort_by_id_ = false;
};

struct RangeSegment {
    uint32_t start, count;
    int32_t object; // ObjectTable index; -1 = a gap no range covers
};

// adapy writes each merged node's per-object index ranges into the scene extras as
// "draw_ranges_<node name>": {object id: [start, count]}. They become the geometry's segments, as in the
// viewer, so clusters and culling here behave as they do in the browser.
std::vector<RangeSegment> segments_from_extras(const tinygltf::Value *ranges, size_t index_count,
                                               ObjectTable &objects) {
    struct Range {
        uint32_t start, count;
        const std::string *id;
    };
    std::vector<Range> rs;
    std::vector<std::string> keys;
    if (ranges && ranges->IsObject()) {
        keys = ranges->Keys();
        for (const auto &key : keys) {
            const auto &v = ranges->Get(key);
            if (!v.IsArray() || v.ArrayLen() < 2)
                continue;
            auto num = [](const tinygltf::Value &x) {
                return x.IsInt() ? double(x.Get<int>()) : x.IsReal() ? x.Get<double>() : 0.0;
            };
            rs.push_back({uint32_t(num(v.Get(0))), uint32_t(num(v.Get(1))), &key});
        }
    }
    std::stable_sort(rs.begin(), rs.end(), [](const Range &a, const Range &b) {
        return a.start != b.start ? a.start < b.start : a.count < b.count;
    });
    std::vector<RangeSegment> out;
    uint32_t cursor = 0;
    for (const Range &r : rs) {
        if (r.start >= index_count || r.count == 0)
            continue;
        if (r.start > cursor)
            out.push_back({cursor, r.start - cursor, -1});
        const uint32_t c = std::min<uint32_t>(r.count, uint32_t(index_count) - r.start);
        out.push_back({r.start, c, objects.by_id(*r.id)});
        cursor = std::max(cursor, r.start + c);
    }
    if (!out.empty() && cursor < index_count)
        out.push_back({cursor, uint32_t(index_count) - cursor, -1});
    return out;
}

// A value inside JSON text, found by skipping over what is in the way: the scene extras of a large model
// run to megabytes, and a host wants them as they were written, not parsed and written again.
struct JsonSpan {
    const char *begin = nullptr, *end = nullptr;
    explicit operator bool() const {
        return begin != nullptr;
    }
};

const char *json_skip_space(const char *p, const char *end) {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
        ++p;
    return p;
}

// `p` at the opening quote; the position after the closing one, or null.
const char *json_skip_string(const char *p, const char *end) {
    for (++p; p < end; ++p) {
        if (*p == '\\')
            ++p;
        else if (*p == '"')
            return p + 1;
    }
    return nullptr;
}

// The position after the value starting at `p`, or null when the text ends inside it.
const char *json_skip_value(const char *p, const char *end) {
    if (p >= end)
        return nullptr;
    if (*p == '"')
        return json_skip_string(p, end);
    if (*p != '{' && *p != '[') {
        while (p < end && *p != ',' && *p != '}' && *p != ']' && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r')
            ++p;
        return p;
    }
    int depth = 0;
    while (p < end) {
        if (*p == '"') {
            p = json_skip_string(p, end);
            if (!p)
                return nullptr;
            continue;
        }
        if (*p == '{' || *p == '[')
            ++depth;
        else if ((*p == '}' || *p == ']') && --depth == 0)
            return p + 1;
        ++p;
    }
    return nullptr;
}

JsonSpan json_member(JsonSpan object, std::string_view key) {
    if (!object || object.end - object.begin < 2 || *object.begin != '{')
        return {};
    const char *p = object.begin + 1, *end = object.end;
    while (true) {
        p = json_skip_space(p, end);
        if (p >= end || *p != '"')
            return {};
        const char *key_end = json_skip_string(p, end);
        if (!key_end)
            return {};
        const bool match = std::string_view(p + 1, size_t(key_end - p - 2)) == key;
        p = json_skip_space(key_end, end);
        if (p >= end || *p != ':')
            return {};
        const char *value = json_skip_space(p + 1, end);
        p = json_skip_value(value, end);
        if (!p)
            return {};
        if (match)
            return {value, p};
        p = json_skip_space(p, end);
        if (p >= end || *p != ',')
            return {};
        ++p;
    }
}

JsonSpan json_element(JsonSpan array, size_t index) {
    if (!array || array.end - array.begin < 2 || *array.begin != '[')
        return {};
    const char *p = array.begin + 1, *end = array.end;
    for (size_t i = 0;; ++i) {
        const char *value = json_skip_space(p, end);
        if (value >= end || *value == ']')
            return {};
        p = json_skip_value(value, end);
        if (!p)
            return {};
        if (i == index)
            return {value, p};
        p = json_skip_space(p, end);
        if (p >= end || *p != ',')
            return {};
        ++p;
    }
}

// GlbLoadResult::extras and ::ada_ext_data, from the GLB as it was handed in.
void read_passthrough(const std::vector<unsigned char> &glb, GlbLoadResult &res) {
    auto u32 = [&](size_t at) {
        uint32_t v;
        std::memcpy(&v, glb.data() + at, 4);
        return v;
    };
    if (glb.size() < 20 || u32(0) != 0x46546C67u || u32(16) != 0x4E4F534Au || 20 + size_t(u32(12)) > glb.size())
        return;
    const char *begin = reinterpret_cast<const char *>(glb.data()) + 20;
    JsonSpan root{json_skip_space(begin, begin + u32(12)), begin + u32(12)};
    size_t scene = 0;
    if (const JsonSpan s = json_member(root, "scene"))
        scene = size_t(std::max(0L, std::strtol(std::string(s.begin, s.end).c_str(), nullptr, 10)));
    if (const JsonSpan e = json_member(json_element(json_member(root, "scenes"), scene), "extras"))
        res.extras.assign(e.begin, e.end);
    if (const JsonSpan e = json_member(json_member(root, "extensions"), "ADA_EXT_data"))
        res.ada_ext_data.assign(e.begin, e.end);
}

// EXT_meshopt_compression / KHR_meshopt_compression (what adapy's GLB writer emits with compression on):
// the compressed bytes sit in the BIN chunk, each compressed bufferView names a "fallback" buffer with no
// data that it decodes into. tinygltf knows neither, so the GLB is rewritten before parsing: every view is
// decoded into a region appended to BIN, pointed at it, and the fallback buffers dropped. A GLB without
// the extension comes back unchanged.
std::vector<unsigned char> decode_meshopt_glb(std::vector<unsigned char> glb) {
    auto u32 = [&](size_t at) {
        uint32_t v;
        std::memcpy(&v, glb.data() + at, 4);
        return v;
    };
    if (glb.size() < 20 || u32(0) != 0x46546C67u) // "glTF"
        return glb;
    const uint32_t json_len = u32(12);
    if (u32(16) != 0x4E4F534Au || 20 + size_t(json_len) > glb.size()) // "JSON"
        return glb;
    // Parse as chars: an unsigned-char range needs std::char_traits<unsigned char>, which libc++ 19 dropped.
    const char *json_begin = reinterpret_cast<const char *>(glb.data()) + 20;
    nlohmann::json doc = nlohmann::json::parse(json_begin, json_begin + json_len);
    const char *ext_name = nullptr;
    for (const char *name : {"EXT_meshopt_compression", "KHR_meshopt_compression"})
        for (const auto &e : doc.value("extensionsUsed", nlohmann::json::array()))
            if (e == name)
                ext_name = name;
    if (!ext_name)
        return glb;

    std::vector<unsigned char> bin;
    const size_t bin_at = 20 + size_t((json_len + 3) & ~3u);
    if (bin_at + 8 <= glb.size() && u32(bin_at + 4) == 0x004E4942u) { // "BIN\0"
        const uint32_t bin_len = u32(bin_at);
        bin.assign(glb.begin() + bin_at + 8, glb.begin() + bin_at + 8 + bin_len);
    }
    glb.clear();
    glb.shrink_to_fit();

    // Where each fallback buffer lands in the new BIN.
    auto &buffers = doc["buffers"];
    std::vector<int64_t> base(buffers.size(), -1);
    std::vector<int> remap(buffers.size(), 0);
    int kept = 0;
    size_t out_len = (bin.size() + 15) & ~size_t(15);
    for (size_t b = 0; b < buffers.size(); ++b) {
        const auto &ext = buffers[b].value("extensions", nlohmann::json::object());
        if (ext.contains(ext_name) && ext[ext_name].value("fallback", false)) {
            base[b] = int64_t(out_len);
            out_len = (out_len + buffers[b].value("byteLength", size_t(0)) + 15) & ~size_t(15);
        } else {
            remap[b] = kept++;
        }
    }
    if (out_len > 0xFFFFFFF0u)
        throw std::runtime_error("adacpp.render: decoded GLB exceeds 4 GB");
    bin.resize(out_len, 0);

    for (auto &bv : doc["bufferViews"]) {
        const size_t b = bv.value("buffer", size_t(0));
        if (bv.contains("extensions") && bv["extensions"].contains(ext_name)) {
            const auto &mx = bv["extensions"][ext_name];
            if (mx.value("buffer", 0) != 0)
                throw std::runtime_error("adacpp.render: meshopt data outside the GLB's BIN chunk");
            const size_t src_at = mx.value("byteOffset", size_t(0)), src_len = mx.value("byteLength", size_t(0));
            const size_t count = mx.value("count", size_t(0)), stride = mx.value("byteStride", size_t(0));
            if (base[b] < 0 || src_at + src_len > bin.size())
                throw std::runtime_error("adacpp.render: malformed meshopt bufferView");
            unsigned char *dst = bin.data() + base[b] + bv.value("byteOffset", size_t(0));
            const unsigned char *src = bin.data() + src_at;
            const std::string mode = mx.value("mode", "ATTRIBUTES");
            int rc;
            if (mode == "ATTRIBUTES")
                rc = meshopt_decodeVertexBuffer(dst, count, stride, src, src_len);
            else if (mode == "TRIANGLES")
                rc = meshopt_decodeIndexBuffer(dst, count, stride, src, src_len);
            else
                rc = meshopt_decodeIndexSequence(dst, count, stride, src, src_len);
            if (rc != 0)
                throw std::runtime_error("adacpp.render: meshopt decode failed (" + mode + ")");
            const std::string filter = mx.value("filter", "NONE");
            if (filter == "OCTAHEDRAL")
                meshopt_decodeFilterOct(dst, count, stride);
            else if (filter == "QUATERNION")
                meshopt_decodeFilterQuat(dst, count, stride);
            else if (filter == "EXPONENTIAL")
                meshopt_decodeFilterExp(dst, count, stride);
            else if (filter == "COLOR")
                meshopt_decodeFilterColor(dst, count, stride);
            bv["extensions"].erase(ext_name);
            if (bv["extensions"].empty())
                bv.erase("extensions");
        }
        if (base[b] >= 0) {
            bv["buffer"] = 0;
            bv["byteOffset"] = size_t(base[b]) + bv.value("byteOffset", size_t(0));
        } else {
            bv["buffer"] = remap[b];
        }
    }
    nlohmann::json kept_buffers = nlohmann::json::array();
    for (size_t b = 0; b < buffers.size(); ++b)
        if (base[b] < 0)
            kept_buffers.push_back(buffers[b]);
    if (kept_buffers.empty())
        kept_buffers.push_back(nlohmann::json::object());
    kept_buffers[0]["byteLength"] = bin.size();
    doc["buffers"] = kept_buffers;
    for (const char *list : {"extensionsUsed", "extensionsRequired"}) {
        if (!doc.contains(list))
            continue;
        nlohmann::json rest = nlohmann::json::array();
        for (const auto &e : doc[list])
            if (e != ext_name)
                rest.push_back(e);
        doc[list] = rest;
    }

    std::string json = doc.dump();
    json.resize((json.size() + 3) & ~size_t(3), ' ');
    std::vector<unsigned char> out;
    out.reserve(28 + json.size() + bin.size());
    auto put = [&](uint32_t v) {
        const auto *p = reinterpret_cast<const unsigned char *>(&v);
        out.insert(out.end(), p, p + 4);
    };
    put(0x46546C67u);
    put(2);
    put(uint32_t(28 + json.size() + bin.size()));
    put(uint32_t(json.size()));
    put(0x4E4F534Au);
    out.insert(out.end(), json.begin(), json.end());
    put(uint32_t(bin.size()));
    put(0x004E4942u);
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

void normalize(float v[3]) {
    float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l > 0)
        for (int i = 0; i < 3; ++i)
            v[i] /= l;
}

InstanceDesc material_desc(const tinygltf::Model &model, int material, bool vertex_colors) {
    InstanceDesc idesc;
    idesc.flags |= kClipped; // a loaded model is cut by the section planes, when any are set
    // Faces sit just behind the GLB's own edge lines, so those draw cleanly on them (triangles only).
    idesc.flags |= kDepthBack;
    if (vertex_colors)
        idesc.flags |= kVertexColors;
    // No material: glTF's default material, as three.js' GLTFLoader builds it (metallic 1, roughness 1).
    idesc.metalness = 1.0f;
    idesc.roughness = 1.0f;
    if (material >= 0) {
        const auto &mat = model.materials.at(material);
        const auto &f = mat.pbrMetallicRoughness.baseColorFactor;
        for (int i = 0; i < 4 && i < int(f.size()); ++i)
            idesc.color[i] = float(f[i]);
        // glTF defaults (metallic 1, roughness 1) are what three.js' GLTFLoader uses too.
        idesc.metalness = float(mat.pbrMetallicRoughness.metallicFactor);
        idesc.roughness = float(mat.pbrMetallicRoughness.roughnessFactor);
        if (mat.doubleSided)
            idesc.flags |= kDoubleSided;
        if (mat.alphaMode == "BLEND" && idesc.color[3] < 1.0f)
            idesc.flags |= kTransparent;
        if (mat.extensions.count("KHR_materials_unlit"))
            idesc.flags |= kUnlit;
    }
    return idesc;
}

// Every primitive of one material and topology, merged into one geometry in world space, as the viewer
// batches a model (CustomBatchedMesh): a model of 15k objects is then a handful of draws instead of 15k.
// Each primitive -- or each adapy draw range in it -- stays a segment, so picking still names the object.
class Batcher {
public:
    Batcher(Renderer &r, const tinygltf::Model &model, GlbLoadResult &res, const GlbOptions &options,
            ObjectTable &objects)
        : r_(r), model_(model), res_(res), options_(options), objects_(objects) {}

    // `object`: what the primitive is when no draw ranges split it (-1 = nothing).
    void add(const tinygltf::Primitive &prim, const Mat4 &world, const tinygltf::Value *ranges, int32_t object) {
        Topology topo;
        if (prim.mode == TINYGLTF_MODE_TRIANGLES || prim.mode == -1)
            topo = Topology::Triangles;
        else if (prim.mode == TINYGLTF_MODE_LINE)
            topo = Topology::Lines;
        else if (prim.mode == TINYGLTF_MODE_POINTS)
            topo = Topology::Points;
        else
            return; // strips/fans/loops: not produced by adapy's writers
        auto pos_it = prim.attributes.find("POSITION");
        if (pos_it == prim.attributes.end())
            return;
        const auto positions = read_floats(model_, pos_it->second, 3);
        const size_t nv = positions.size() / 3;
        std::vector<float> normals;
        if (auto it = prim.attributes.find("NORMAL"); it != prim.attributes.end())
            normals = read_floats(model_, it->second, 3);
        std::vector<float> colors;
        if (auto it = prim.attributes.find("COLOR_0"); it != prim.attributes.end())
            colors = read_floats(model_, it->second, 4);
        std::vector<uint32_t> indices;
        if (prim.indices >= 0) {
            indices = read_indices(model_, prim.indices);
        } else {
            indices.resize(nv);
            for (uint32_t i = 0; i < nv; ++i)
                indices[i] = i;
        }

        // Coloured and plain lines draw differently (members vs wireframe): never in one batch.
        const bool colored = !colors.empty() && topo != Topology::Triangles;
        Batch &b = batches_[{int(topo), prim.material, colored}];
        b.topology = topo;
        b.material = prim.material;
        if (b.vertex_count() + nv > kBatchMaxVertices || b.indices.size() + indices.size() > kBatchMaxIndices)
            flush(b);

        // Normals take the inverse transpose (the cofactor matrix, up to scale); a mirroring transform also
        // flips the winding, as glTF requires.
        const Mat4 &m = world;
        const float cof[9] = {m[5] * m[10] - m[6] * m[9], m[6] * m[8] - m[4] * m[10], m[4] * m[9] - m[5] * m[8],
                              m[2] * m[9] - m[1] * m[10], m[0] * m[10] - m[2] * m[8], m[1] * m[8] - m[0] * m[9],
                              m[1] * m[6] - m[2] * m[5],  m[2] * m[4] - m[0] * m[6],  m[0] * m[5] - m[1] * m[4]};
        const float det = m[0] * cof[0] + m[4] * cof[1] + m[8] * cof[2];
        const float sign = det < 0.0f ? -1.0f : 1.0f;
        const uint32_t base = uint32_t(b.vertex_count());
        for (size_t v = 0; v < nv; ++v) {
            const float *p = &positions[v * 3];
            for (int k = 0; k < 3; ++k) {
                const float w = m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k];
                b.positions.push_back(w);
                res_.bounds[k] = std::min(res_.bounds[k], w);
                res_.bounds[3 + k] = std::max(res_.bounds[3 + k], w);
            }
            float n[3] = {0, 0, 0}; // zero: the shader derives a flat normal
            if (!normals.empty()) {
                const float *s = &normals[v * 3];
                for (int k = 0; k < 3; ++k)
                    n[k] = sign * (cof[k * 3] * s[0] + cof[k * 3 + 1] * s[1] + cof[k * 3 + 2] * s[2]);
                normalize(n);
            }
            b.normals.insert(b.normals.end(), n, n + 3);
            b.colors.push_back(colors.empty() ? 0xFFFFFFFFu : pack_rgba8(&colors[v * 4]));
        }
        b.any_normals |= !normals.empty();
        b.any_colors |= !colors.empty();

        const uint32_t first = uint32_t(b.indices.size());
        const bool flip = det < 0.0f && topo == Topology::Triangles;
        for (size_t i = 0; i < indices.size(); ++i) {
            // Mirrored triangle: swap its 2nd and 3rd corner.
            size_t src = i;
            if (flip && i - i % 3 + 2 < indices.size() && i % 3 != 0)
                src = i % 3 == 1 ? i + 1 : i - 1;
            b.indices.push_back(base + indices[src]);
        }
        auto segments = topo == Topology::Triangles ? segments_from_extras(ranges, indices.size(), objects_)
                                                    : std::vector<RangeSegment>{};
        if (segments.empty())
            segments = {{0, uint32_t(indices.size()), object}};
        for (const RangeSegment &seg : segments) {
            b.segments.insert(b.segments.end(), {first + seg.start, seg.count});
            b.segment_objects.push_back(seg.object);
        }
        res_.segments += segments.size();
        res_.vertices += nv;
        if (topo == Topology::Triangles)
            res_.triangles += indices.size() / 3;
    }

    void finish() {
        for (auto &[key, b] : batches_)
            flush(b);
    }

private:
    static constexpr size_t kBatchMaxVertices = 2'000'000; // 48 MB of vertex data, well inside 128 MB bindings
    static constexpr size_t kBatchMaxIndices = 12'000'000;

    struct Batch {
        Topology topology = Topology::Triangles;
        int material = -1;
        std::vector<float> positions, normals;
        std::vector<uint32_t> colors, indices, segments;
        std::vector<int32_t> segment_objects; // per segment, its ObjectTable index or -1
        bool any_normals = false, any_colors = false;
        size_t vertex_count() const {
            return positions.size() / 3;
        }
    };

    void flush(Batch &b) {
        if (b.positions.empty())
            return;
        GeometryDesc gd;
        gd.topology = b.topology;
        gd.positions = b.positions.data();
        gd.vertex_count = b.vertex_count();
        gd.normals = b.any_normals ? b.normals.data() : nullptr;
        gd.colors = b.any_colors ? b.colors.data() : nullptr;
        gd.indices = b.indices.data();
        gd.index_count = b.indices.size();
        gd.segments = b.segments.data();
        gd.segment_count = b.segments.size() / 2;
        const uint32_t geom = r_.add_geometry(gd);
        const uint32_t inst = r_.add_instance(geom);
        InstanceDesc desc = material_desc(model_, b.material, b.any_colors);
        if (b.topology == Topology::Lines) {
            desc.line_width = b.any_colors ? options_.member_width : options_.line_width;
            if (!b.any_colors && options_.line_color[3] > 0.0f)
                desc.color = options_.line_color;
        } else if (b.topology == Topology::Points) {
            desc.line_width = options_.point_size;
        } else if (options_.double_sided) {
            desc.flags |= kDoubleSided;
        }
        r_.set_instance(inst, desc);
        res_.geometries++;
        res_.instances++;
        GlbGeometry rec;
        rec.geometry = geom;
        rec.instance = inst;
        rec.topology = b.topology;
        rec.vertex_count = uint32_t(b.vertex_count());
        rec.vertex_colors = b.any_colors;
        if (options_.keep_positions) {
            rec.positions = b.positions;
            rec.indices = b.indices;
        }
        // Each object's share of the batch: its segments, and the bounds and triangles they cover.
        for (size_t k = 0; k < b.segment_objects.size(); ++k) {
            if (b.segment_objects[k] < 0)
                continue;
            const uint32_t start = b.segments[k * 2], count = b.segments[k * 2 + 1];
            float bounds[6] = {3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
            for (uint32_t i = start; i < start + count; ++i) {
                const float *p = &b.positions[size_t(b.indices[i]) * 3];
                for (int c = 0; c < 3; ++c) {
                    bounds[c] = std::min(bounds[c], p[c]);
                    bounds[3 + c] = std::max(bounds[3 + c], p[c]);
                }
            }
            objects_.add_segment(b.segment_objects[k], inst, uint32_t(k), bounds,
                                 b.topology == Topology::Triangles ? count / 3 : 0);
        }
        rec.segment_objects = b.segment_objects;
        res_.loaded.push_back(std::move(rec));
        if (b.topology == Topology::Triangles && options_.feature_edge_angle_deg > 0.0f)
            add_feature_edges(b);
        const Topology topo = b.topology;
        const int material = b.material;
        b = Batch{};
        b.topology = topo;
        b.material = material;
    }

    // The batch's feature edges as a line overlay (GlbOptions::feature_edge_angle_deg), drawn like the
    // GLB's own uncoloured lines; the faces already sit behind coplanar lines (kDepthBack).
    void add_feature_edges(const Batch &b) {
        const std::vector<float> lines = feature_edges(b.positions, b.indices, options_.feature_edge_angle_deg);
        if (lines.empty())
            return;
        GeometryDesc gd;
        gd.topology = Topology::Lines;
        gd.positions = lines.data();
        gd.vertex_count = lines.size() / 3;
        const uint32_t geom = r_.add_geometry(gd);
        const uint32_t inst = r_.add_instance(geom);
        InstanceDesc desc;
        desc.flags = kClipped | kUnlit;
        desc.color = options_.line_color[3] > 0.0f ? options_.line_color : std::array<float, 4>{0, 0, 0, 1};
        desc.line_width = options_.line_width;
        r_.set_instance(inst, desc);
        res_.geometries++;
        res_.instances++;
        res_.feature_edges += lines.size() / 6;
        GlbGeometry rec;
        rec.geometry = geom;
        rec.instance = inst;
        rec.topology = Topology::Lines;
        rec.vertex_count = uint32_t(lines.size() / 3);
        rec.feature_edges = true;
        rec.segment_objects = {-1};
        res_.loaded.push_back(std::move(rec));
    }

    Renderer &r_;
    const tinygltf::Model &model_;
    GlbLoadResult &res_;
    GlbOptions options_;
    ObjectTable &objects_;
    std::map<std::tuple<int, int, bool>, Batch> batches_;
};

struct Walk {
    Batcher &batcher;
    const tinygltf::Model &model;
    const tinygltf::Value &scene_extras;
    ObjectTable &objects;
    bool node_objects; // every node is an object (no adapy extras to name them)

    void node(int node_index, const Mat4 &parent, int32_t parent_object) {
        const auto &n = model.nodes.at(node_index);
        const Mat4 world = mul(parent, node_matrix(n));
        const int32_t object = node_objects ? objects.for_node(model, node_index, parent_object) : -1;
        if (n.mesh >= 0) {
            const auto &prims = model.meshes.at(n.mesh).primitives;
            const std::string key = "draw_ranges_" + n.name;
            const tinygltf::Value *ranges = prims.size() == 1 && scene_extras.IsObject() && scene_extras.Has(key)
                                                ? &scene_extras.Get(key)
                                                : nullptr;
            for (const auto &prim : prims)
                batcher.add(prim, world, ranges, object);
        }
        for (int child : n.children)
            node(child, world, object);
    }
};

} // namespace

GlbLoadResult load_glb(Renderer &renderer, const std::string &path, const GlbOptions &options) {
    std::vector<unsigned char> bytes;
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("adacpp.render: cannot open " + path);
        bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    return load_glb_bytes(renderer, std::move(bytes), options, path);
}

GlbLoadResult load_glb_bytes(Renderer &renderer, std::vector<unsigned char> bytes, const GlbOptions &options,
                             const std::string &name) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err, warn;
    // Images are never needed: the renderer does not texture.
    loader.SetImageLoader([](tinygltf::Image *, const int, std::string *, std::string *, int, int,
                             const unsigned char *, int, void *) { return true; },
                          nullptr);
    GlbLoadResult res;
    read_passthrough(bytes, res);
    bytes = decode_meshopt_glb(std::move(bytes));
    const bool ok = loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), unsigned(bytes.size()));
    bytes.clear();
    bytes.shrink_to_fit();
    if (!ok)
        throw std::runtime_error("adacpp.render: failed to read " + name + ": " + err);
    for (const auto &ext : model.extensionsRequired)
        if (ext == "KHR_draco_mesh_compression")
            throw std::runtime_error("adacpp.render: " + name + " requires " + ext + ", not supported");

    // glTF y-up -> z-up: rotate +90 degrees about x.
    const Mat4 root = options.y_up_to_z_up ? Mat4{1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1}
                                           : Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const int scene = model.defaultScene >= 0 ? model.defaultScene : 0;
    const tinygltf::Value no_extras;
    const tinygltf::Value &extras = model.scenes.empty() ? no_extras : model.scenes.at(scene).extras;
    // adapy names the objects in the scene extras; without them the nodes are the objects.
    ObjectTable objects;
    bool adapy = false;
    if (extras.IsObject()) {
        if (extras.Has("id_hierarchy") && extras.Get("id_hierarchy").IsObject()) {
            objects.add_hierarchy(extras.Get("id_hierarchy"));
            adapy = true;
        }
        for (const auto &key : extras.Keys())
            adapy |= key.rfind("draw_ranges_", 0) == 0;
    }
    res.node_objects = !adapy;
    Batcher batcher(renderer, model, res, options, objects);
    Walk walk{batcher, model, extras, objects, res.node_objects};
    if (!model.scenes.empty()) {
        for (int n : model.scenes.at(scene).nodes)
            walk.node(n, root, -1);
    } else {
        for (size_t n = 0; n < model.nodes.size(); ++n)
            walk.node(int(n), root, -1);
    }
    batcher.finish();
    objects.finish(res);
    return res;
}

std::vector<float> feature_edges(const std::vector<float> &positions, const std::vector<uint32_t> &indices,
                                 float angle_deg) {
    // Weld by position (a tessellated GLB is a triangle soup), quantised to 1e-4 model units as the viewer
    // does (frontend utils/mesh_select/edgeCore.ts).
    struct Key {
        int64_t x, y, z;
        bool operator==(const Key &o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct KeyHash {
        size_t operator()(const Key &k) const {
            return size_t(k.x * 73856093) ^ size_t(k.y * 19349663) ^ size_t(k.z * 83492791);
        }
    };
    const size_t nv = positions.size() / 3;
    std::unordered_map<Key, uint32_t, KeyHash> weld;
    weld.reserve(nv);
    std::vector<uint32_t> id(nv);
    for (size_t v = 0; v < nv; ++v) {
        const Key k{std::llround(positions[v * 3] * 1e4), std::llround(positions[v * 3 + 1] * 1e4),
                    std::llround(positions[v * 3 + 2] * 1e4)};
        id[v] = weld.emplace(k, uint32_t(v)).first->second;
    }

    const size_t nt = indices.size() / 3;
    std::vector<float> normal(nt * 3, 0.0f);
    for (size_t t = 0; t < nt; ++t) {
        const float *a = &positions[id[indices[t * 3]] * 3], *b = &positions[id[indices[t * 3 + 1]] * 3],
                    *c = &positions[id[indices[t * 3 + 2]] * 3];
        const float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float n[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
        normalize(n);
        std::copy(n, n + 3, &normal[t * 3]);
    }

    // Edge (lo << 32 | hi) -> the first face on it, how many faces share it, and (once a second face
    // arrives) whether the two meet at more than the angle.
    struct Use {
        uint32_t first, count;
        bool sharp;
    };
    std::unordered_map<uint64_t, Use> edges;
    edges.reserve(nt * 2);
    std::vector<uint64_t> order; // first-seen order, so the output is deterministic
    const float cos_angle = std::cos(angle_deg * 3.14159265358979f / 180.0f);
    for (size_t t = 0; t < nt; ++t) {
        const uint32_t v[3] = {id[indices[t * 3]], id[indices[t * 3 + 1]], id[indices[t * 3 + 2]]};
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2])
            continue; // collapsed by the weld
        for (int e = 0; e < 3; ++e) {
            const uint32_t lo = std::min(v[e], v[(e + 1) % 3]), hi = std::max(v[e], v[(e + 1) % 3]);
            const uint64_t key = (uint64_t(lo) << 32) | hi;
            auto [it, fresh] = edges.try_emplace(key, Use{uint32_t(t), 0, false});
            if (fresh)
                order.push_back(key);
            it->second.count++;
            if (it->second.count == 2) {
                const float *n1 = &normal[it->second.first * 3], *n2 = &normal[t * 3];
                it->second.sharp = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] <= cos_angle;
            }
        }
    }
    std::vector<float> out;
    for (const uint64_t key : order) {
        const Use &u = edges[key];
        // boundary (1) and non-manifold (>2) edges always; shared ones when the faces meet at the angle
        const bool show = u.count != 2 || u.sharp;
        if (!show)
            continue;
        for (const uint32_t v : {uint32_t(key >> 32), uint32_t(key & 0xFFFFFFFFu)})
            out.insert(out.end(), &positions[v * 3], &positions[v * 3] + 3);
    }
    return out;
}

Camera look_at(const float eye[3], const float target[3], std::array<float, 3> up, float fov_y_deg, float aspect,
               const std::array<float, 16> &fit) {
    float zc[3] = {eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]};
    normalize(zc);
    float xc[3] = {up[1] * zc[2] - up[2] * zc[1], up[2] * zc[0] - up[0] * zc[2], up[0] * zc[1] - up[1] * zc[0]};
    normalize(xc);
    float yc[3] = {zc[1] * xc[2] - zc[2] * xc[1], zc[2] * xc[0] - zc[0] * xc[2], zc[0] * xc[1] - zc[1] * xc[0]};
    Camera cam{};
    auto &v = cam.view;
    v = {xc[0], yc[0], zc[0], 0, xc[1], yc[1], zc[1], 0, xc[2], yc[2], zc[2], 0, 0, 0, 0, 1};
    v[12] = -(xc[0] * eye[0] + xc[1] * eye[1] + xc[2] * eye[2]);
    v[13] = -(yc[0] * eye[0] + yc[1] * eye[1] + yc[2] * eye[2]);
    v[14] = -(zc[0] * eye[0] + zc[1] * eye[1] + zc[2] * eye[2]);
    const float near_z = 0.05f, far_z = 2000.0f;
    const float f = 1.0f / std::tan(fov_y_deg * 3.14159265f / 360.0f);
    auto &p = cam.proj;
    p = {};
    p[0] = f / aspect;
    p[5] = f;
    p[10] = (far_z + near_z) / (near_z - far_z);
    p[11] = -1.0f;
    p[14] = 2.0f * far_z * near_z / (near_z - far_z);
    (void) fit;
    return cam;
}

Camera frame_bounds(const std::array<float, 6> &b, std::array<float, 3> direction, std::array<float, 3> up,
                    float fov_y_deg, float aspect, float margin) {
    float center[3] = {0.5f * (b[0] + b[3]), 0.5f * (b[1] + b[4]), 0.5f * (b[2] + b[5])};
    const float dx = b[3] - b[0], dy = b[4] - b[1], dz = b[5] - b[2];
    const float radius = std::max(0.5f * std::sqrt(dx * dx + dy * dy + dz * dz), 1e-3f);
    const float fov = fov_y_deg * 3.14159265f / 180.0f;
    const float fov_min = aspect < 1.0f ? 2.0f * std::atan(std::tan(fov * 0.5f) * aspect) : fov;
    const float dist = margin * radius / std::sin(fov_min * 0.5f);

    float d[3] = {direction[0], direction[1], direction[2]};
    normalize(d);
    float eye[3] = {center[0] + d[0] * dist, center[1] + d[1] * dist, center[2] + d[2] * dist};
    // Right-handed look-at: camera z axis points from the target to the eye.
    float zc[3] = {d[0], d[1], d[2]};
    float xc[3] = {up[1] * zc[2] - up[2] * zc[1], up[2] * zc[0] - up[0] * zc[2], up[0] * zc[1] - up[1] * zc[0]};
    normalize(xc);
    float yc[3] = {zc[1] * xc[2] - zc[2] * xc[1], zc[2] * xc[0] - zc[0] * xc[2], zc[0] * xc[1] - zc[1] * xc[0]};
    Camera cam{};
    auto &v = cam.view;
    v = {xc[0], yc[0], zc[0], 0, xc[1], yc[1], zc[1], 0, xc[2], yc[2], zc[2], 0, 0, 0, 0, 1};
    v[12] = -(xc[0] * eye[0] + xc[1] * eye[1] + xc[2] * eye[2]);
    v[13] = -(yc[0] * eye[0] + yc[1] * eye[1] + yc[2] * eye[2]);
    v[14] = -(zc[0] * eye[0] + zc[1] * eye[1] + zc[2] * eye[2]);

    const float near_z = std::max(dist - radius * 1.5f, dist * 1e-3f), far_z = dist + radius * 1.5f;
    const float f = 1.0f / std::tan(fov * 0.5f);
    auto &p = cam.proj;
    p = {};
    p[0] = f / aspect;
    p[5] = f;
    p[10] = (far_z + near_z) / (near_z - far_z);
    p[11] = -1.0f;
    p[14] = 2.0f * far_z * near_z / (near_z - far_z);
    return cam;
}

} // namespace adacpp::render
