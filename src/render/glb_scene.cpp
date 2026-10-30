#include "glb_scene.h"

#ifndef __EMSCRIPTEN__

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <stdexcept>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#define STB_IMAGE_IMPLEMENTATION
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

// adapy writes each merged node's per-object index ranges into the scene extras as
// "draw_ranges_<node name>": {object id: [start, count]}. They become the geometry's segments, as in the
// viewer, so clusters and culling here behave as they do in the browser.
std::vector<uint32_t> segments_from_extras(const tinygltf::Value *ranges, size_t index_count) {
    std::vector<std::pair<uint32_t, uint32_t>> rs;
    if (ranges && ranges->IsObject()) {
        for (const auto &key : ranges->Keys()) {
            const auto &v = ranges->Get(key);
            if (!v.IsArray() || v.ArrayLen() < 2)
                continue;
            auto num = [](const tinygltf::Value &x) {
                return x.IsInt() ? double(x.Get<int>()) : x.IsReal() ? x.Get<double>() : 0.0;
            };
            rs.emplace_back(uint32_t(num(v.Get(0))), uint32_t(num(v.Get(1))));
        }
    }
    std::sort(rs.begin(), rs.end());
    std::vector<uint32_t> out;
    uint32_t cursor = 0;
    for (const auto &[start, count] : rs) {
        if (start >= index_count || count == 0)
            continue;
        if (start > cursor)
            out.insert(out.end(), {cursor, start - cursor});
        const uint32_t c = std::min<uint32_t>(count, uint32_t(index_count) - start);
        out.insert(out.end(), {start, c});
        cursor = std::max(cursor, start + c);
    }
    if (!out.empty() && cursor < index_count)
        out.insert(out.end(), {cursor, uint32_t(index_count) - cursor});
    return out;
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
    nlohmann::json doc = nlohmann::json::parse(glb.begin() + 20, glb.begin() + 20 + json_len);
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
    if (vertex_colors)
        idesc.flags |= kVertexColors;
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
    Batcher(Renderer &r, const tinygltf::Model &model, GlbLoadResult &res, float line_width, float point_size)
        : r_(r), model_(model), res_(res), line_width_(line_width), point_size_(point_size) {}

    void add(const tinygltf::Primitive &prim, const Mat4 &world, const tinygltf::Value *ranges) {
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

        Batch &b = batches_[{int(topo), prim.material}];
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
            for (int k = 0; k < 3; ++k)
                b.positions.push_back(m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k]);
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
        auto segments =
            topo == Topology::Triangles ? segments_from_extras(ranges, indices.size()) : std::vector<uint32_t>{};
        if (segments.empty())
            segments = {0, uint32_t(indices.size())};
        for (size_t k = 0; k < segments.size(); k += 2)
            b.segments.insert(b.segments.end(), {first + segments[k], segments[k + 1]});
        res_.segments += segments.size() / 2;
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
        const uint32_t inst = r_.add_instance(r_.add_geometry(gd));
        InstanceDesc desc = material_desc(model_, b.material, b.any_colors);
        if (b.topology == Topology::Lines)
            desc.line_width = line_width_;
        else if (b.topology == Topology::Points)
            desc.line_width = point_size_;
        r_.set_instance(inst, desc);
        res_.geometries++;
        res_.instances++;
        const Topology topo = b.topology;
        const int material = b.material;
        b = Batch{};
        b.topology = topo;
        b.material = material;
    }

    Renderer &r_;
    const tinygltf::Model &model_;
    GlbLoadResult &res_;
    float line_width_, point_size_;
    std::map<std::pair<int, int>, Batch> batches_;
};

void walk(Batcher &batcher, const tinygltf::Model &model, int node_index, const Mat4 &parent,
          const tinygltf::Value &scene_extras) {
    const auto &node = model.nodes.at(node_index);
    const Mat4 world = mul(parent, node_matrix(node));
    if (node.mesh >= 0) {
        const auto &prims = model.meshes.at(node.mesh).primitives;
        const std::string key = "draw_ranges_" + node.name;
        const tinygltf::Value *ranges =
            prims.size() == 1 && scene_extras.IsObject() && scene_extras.Has(key) ? &scene_extras.Get(key) : nullptr;
        for (const auto &prim : prims)
            batcher.add(prim, world, ranges);
    }
    for (int child : node.children)
        walk(batcher, model, child, world, scene_extras);
}

} // namespace

GlbLoadResult load_glb(Renderer &renderer, const std::string &path, bool y_up_to_z_up, float line_width,
                       float point_size) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err, warn;
    // Images are never needed: the renderer does not texture.
    loader.SetImageLoader([](tinygltf::Image *, const int, std::string *, std::string *, int, int,
                             const unsigned char *, int, void *) { return true; },
                          nullptr);
    std::vector<unsigned char> bytes;
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("adacpp.render: cannot open " + path);
        bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    bytes = decode_meshopt_glb(std::move(bytes));
    const bool ok = loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), unsigned(bytes.size()));
    bytes.clear();
    bytes.shrink_to_fit();
    if (!ok)
        throw std::runtime_error("adacpp.render: failed to read " + path + ": " + err);
    for (const auto &ext : model.extensionsRequired)
        if (ext == "KHR_draco_mesh_compression")
            throw std::runtime_error("adacpp.render: " + path + " requires " + ext + ", not supported");

    // glTF y-up -> z-up: rotate +90 degrees about x.
    const Mat4 root = y_up_to_z_up ? Mat4{1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1}
                                   : Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    GlbLoadResult res;
    Batcher batcher(renderer, model, res, line_width, point_size);
    const int scene = model.defaultScene >= 0 ? model.defaultScene : 0;
    if (!model.scenes.empty()) {
        for (int n : model.scenes.at(scene).nodes)
            walk(batcher, model, n, root, model.scenes.at(scene).extras);
    } else {
        for (size_t n = 0; n < model.nodes.size(); ++n)
            walk(batcher, model, int(n), root, tinygltf::Value());
    }
    batcher.finish();
    return res;
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

#endif
