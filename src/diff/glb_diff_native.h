#pragma once
// Native (non-wasm) GLB reader for the diff core: parse a GLB into per-element ParsedElement records
// using the vendored tinygltf (only in the OCC/native build). Keeps glb_diff.h portable — the wasm
// target supplies its own ParsedElement list from a tinygltf-free reader and reuses the same
// portable diff logic. v1: uncompressed GLBs (the diff contract).

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <tiny_gltf.h>

#include "../geom/neutral/ngeom_meshopt.h" // EXT_meshopt_compression decode
#include "glb_diff.h"

namespace adacpp {
namespace gdiff {

inline uint32_t pad4(uint32_t n) {
    return (4 - (n & 3u)) & 3u;
}

// tinygltf rejects a meshopt GLB: the EXT_meshopt_compression fallback buffer declares the DECODED
// size, which exceeds the stored BIN bytes. Clamp every JSON ``byteLength`` that exceeds the BIN
// chunk size down to it so tinygltf loads (only the fallback buffer's exceeds it; meshopt
// bufferViews are read via their extension's count/stride, not byteLength, so clamping is safe).
inline std::string patch_buffer_bytelength(const std::string &glb) {
    const unsigned char *d = (const unsigned char *) glb.data();
    if (glb.size() < 20 || std::memcmp(d, "glTF", 4) != 0)
        return glb;
    uint32_t json_len;
    std::memcpy(&json_len, d + 12, 4);
    size_t json_end = 20 + json_len;
    if (json_end + 8 > glb.size())
        return glb;
    uint32_t bin_len;
    std::memcpy(&bin_len, d + json_end, 4); // following chunk header = BIN length
    std::string json(glb.begin() + 20, glb.begin() + json_end);

    const std::string key = "\"byteLength\"";
    std::string nj;
    nj.reserve(json.size());
    bool changed = false;
    size_t i = 0;
    while (i < json.size()) {
        size_t k = json.find(key, i);
        if (k == std::string::npos) {
            nj.append(json, i, json.size() - i);
            break;
        }
        size_t ns = json.find(':', k) + 1;
        while (ns < json.size() && json[ns] == ' ')
            ++ns;
        size_t ne = ns;
        while (ne < json.size() && json[ne] >= '0' && json[ne] <= '9')
            ++ne;
        size_t val = 0;
        for (size_t j = ns; j < ne; ++j)
            val = val * 10 + (json[j] - '0');
        nj.append(json, i, ns - i); // through the key, colon, and any spaces
        if (val > bin_len) {
            nj += std::to_string((unsigned) bin_len);
            changed = true;
        } else {
            nj.append(json, ns, ne - ns);
        }
        i = ne;
    }
    if (!changed)
        return glb; // uncompressed GLB — nothing to patch

    nj.append(pad4((uint32_t) nj.size()), ' ');
    const char *bin = glb.data() + json_end + 8;
    uint32_t njl = (uint32_t) nj.size();
    uint32_t total = 12 + 8 + njl + 8 + bin_len;
    std::string out;
    out.reserve(total);
    auto u32 = [&](uint32_t v) { out.append((const char *) &v, 4); };
    out.append("glTF", 4);
    u32(2);
    u32(total);
    u32(njl);
    u32(0x4E4F534Au); // "JSON"
    out.append(nj);
    u32(bin_len);
    u32(0x004E4942u); // "BIN\0"
    out.append(bin, bin_len);
    return out;
}

// Decoded-bufferView cache: a bufferView with EXT_meshopt_compression is decoded once (the viewer's
// crane GLBs are meshopt-compressed); raw bufferViews are read in place. ``stride`` is the element
// byte-stride to walk with.
using BvCache = std::unordered_map<int, std::vector<unsigned char>>;
struct BvView {
    const unsigned char *data;
    size_t stride;
};

// ``bin`` is the GLB's embedded BIN chunk (we read buffer data directly from it rather than
// model.buffers[].data: a meshopt no-fallback GLB declares the buffer's DECODED byteLength, which
// exceeds the stored bytes, so tinygltf refuses to populate buffers — but it parses the JSON fine).
inline BvView resolve_bufferview(const tinygltf::Model &m, int bv_idx, size_t elem_size, BvCache &cache,
                                 const unsigned char *bin) {
    const tinygltf::BufferView &bv = m.bufferViews[bv_idx];
    auto eit = bv.extensions.find("EXT_meshopt_compression");
    if (eit != bv.extensions.end()) {
        const tinygltf::Value &e = eit->second;
        size_t bo = (size_t) e.Get("byteOffset").GetNumberAsInt();
        size_t bl = (size_t) e.Get("byteLength").GetNumberAsInt();
        size_t cnt = (size_t) e.Get("count").GetNumberAsInt();
        size_t stride = (size_t) e.Get("byteStride").GetNumberAsInt();
        std::string mode = e.Has("mode") ? e.Get("mode").Get<std::string>() : std::string("ATTRIBUTES");
        auto cit = cache.find(bv_idx);
        if (cit == cache.end()) {
            const unsigned char *enc = bin + bo;
            std::vector<unsigned char> dec = (mode == "ATTRIBUTES")
                                                 ? ::ngeom::meshopt_decode_vertices(enc, bl, cnt, stride)
                                                 : ::ngeom::meshopt_decode_indices(enc, bl, cnt, stride);
            cit = cache.emplace(bv_idx, std::move(dec)).first;
        }
        return {cit->second.data(), stride};
    }
    return {bin + bv.byteOffset, bv.byteStride ? (size_t) bv.byteStride : elem_size};
}

// hash the MODIFIED-relevant metadata fields (section/material/thickness/type), order-stable.
inline uint64_t meta_signature(const tinygltf::Value &m) {
    if (!m.IsObject())
        return 0;
    std::string s;
    for (const char *k : {"type", "section", "material", "thickness"}) {
        if (m.Has(k)) {
            const tinygltf::Value &v = m.Get(k);
            s += k;
            s += "=";
            if (v.IsString())
                s += v.Get<std::string>();
            else if (v.IsNumber())
                s += std::to_string(v.GetNumberAsDouble());
            s += ";";
        }
    }
    return std::hash<std::string>{}(s);
}

// Read a float vec accessor (POSITION) into a flat buffer; decodes meshopt if present.
inline void read_accessor_floats(const tinygltf::Model &model, int acc_idx, std::vector<float> &out, int &ncomp,
                                 BvCache &cache, const unsigned char *bin) {
    const tinygltf::Accessor &acc = model.accessors[acc_idx];
    ncomp = (acc.type == TINYGLTF_TYPE_VEC3) ? 3 : (acc.type == TINYGLTF_TYPE_VEC2 ? 2 : 1);
    BvView v = resolve_bufferview(model, acc.bufferView, (size_t) ncomp * 4, cache, bin);
    const unsigned char *base = v.data + acc.byteOffset;
    out.resize(acc.count * (size_t) ncomp);
    for (size_t i = 0; i < acc.count; ++i)
        std::memcpy(&out[i * ncomp], base + i * v.stride, (size_t) ncomp * 4);
}

inline void read_accessor_indices(const tinygltf::Model &model, int acc_idx, std::vector<uint32_t> &out,
                                  BvCache &cache, const unsigned char *bin) {
    const tinygltf::Accessor &acc = model.accessors[acc_idx];
    const size_t isz = (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) ? 4 : 2;
    BvView v = resolve_bufferview(model, acc.bufferView, isz, cache, bin);
    const unsigned char *base = v.data + acc.byteOffset;
    out.resize(acc.count);
    for (size_t i = 0; i < acc.count; ++i) {
        if (isz == 4) {
            uint32_t x;
            std::memcpy(&x, base + i * v.stride, 4);
            out[i] = x;
        } else {
            uint16_t x;
            std::memcpy(&x, base + i * v.stride, 2);
            out[i] = x;
        }
    }
}

// Parse a GLB into per-element ParsedElement records (mirrors adapy diff.py parse_elements).
inline std::vector<ParsedElement> parse_glb_elements(const std::string &glb) {
    std::vector<ParsedElement> out;
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    // Ignore the bool: a meshopt no-fallback GLB fails buffer-load (decoded byteLength > stored
    // bytes) but the JSON (scenes/nodes/meshes/accessors/bufferViews/extensions) is parsed first.
    std::string patched = patch_buffer_bytelength(glb);
    bool ok = loader.LoadBinaryFromMemory(&model, &err, &warn, (const unsigned char *) patched.data(), patched.size());
    if (!ok || model.scenes.empty() || model.bufferViews.empty() || model.buffers.empty())
        return out;
    const unsigned char *bin = model.buffers[0].data.data();
    const tinygltf::Scene &scene = model.scenes[model.defaultScene >= 0 ? model.defaultScene : 0];
    const tinygltf::Value &extras = scene.extras;
    if (!extras.IsObject())
        return out;

    // id_hierarchy[node_id] = [name, parent]
    std::unordered_map<std::string, std::string> name_of;
    if (extras.Has("id_hierarchy")) {
        const tinygltf::Value &ih = extras.Get("id_hierarchy");
        for (const std::string &nid : ih.Keys()) {
            const tinygltf::Value &arr = ih.Get(nid);
            if (arr.IsArray() && arr.ArrayLen() >= 1 && arr.Get(0).IsString())
                name_of[nid] = arr.Get(0).Get<std::string>();
        }
    }

    // ADA_EXT_data: object_guids {name:guid}, object_metadata {name:{...}}
    std::unordered_map<std::string, std::string> guid_of, etype_of;
    std::unordered_map<std::string, uint64_t> meta_of;
    auto ext_it = model.extensions.find("ADA_EXT_data");
    if (ext_it != model.extensions.end() && ext_it->second.IsObject()) {
        for (const char *grp : {"design_objects", "simulation_objects"}) {
            if (!ext_it->second.Has(grp))
                continue;
            const tinygltf::Value &lst = ext_it->second.Get(grp);
            if (!lst.IsArray())
                continue;
            for (int i = 0; i < lst.ArrayLen(); ++i) {
                const tinygltf::Value &obj = lst.Get(i);
                if (obj.Has("object_guids")) {
                    const tinygltf::Value &g = obj.Get("object_guids");
                    for (const std::string &nm : g.Keys())
                        if (g.Get(nm).IsString())
                            guid_of[nm] = g.Get(nm).Get<std::string>();
                }
                if (obj.Has("object_metadata")) {
                    const tinygltf::Value &md = obj.Get("object_metadata");
                    for (const std::string &nm : md.Keys()) {
                        const tinygltf::Value &m = md.Get(nm);
                        meta_of[nm] = meta_signature(m);
                        if (m.IsObject() && m.Has("type") && m.Get("type").IsString())
                            etype_of[nm] = m.Get("type").Get<std::string>();
                    }
                }
            }
        }
    }

    // node name -> (positions, indices). meshopt bufferViews decoded once via the cache.
    BvCache bv_cache;
    std::unordered_map<std::string, std::pair<std::vector<float>, std::vector<uint32_t>>> node_geo;
    for (const tinygltf::Node &node : model.nodes) {
        if (node.mesh < 0)
            continue;
        const tinygltf::Primitive &prim = model.meshes[node.mesh].primitives[0];
        auto pit = prim.attributes.find("POSITION");
        if (pit == prim.attributes.end() || prim.indices < 0)
            continue;
        std::vector<float> pos;
        int nc = 0;
        read_accessor_floats(model, pit->second, pos, nc, bv_cache, bin);
        std::vector<uint32_t> idx;
        read_accessor_indices(model, prim.indices, idx, bv_cache, bin);
        node_geo[node.name] = {std::move(pos), std::move(idx)};
    }

    // draw_ranges_<node> = {node_id: [start, count]}  -> one ParsedElement per node_id
    for (const std::string &key : extras.Keys()) {
        if (key.rfind("draw_ranges_", 0) != 0)
            continue;
        std::string node_name = key.substr(std::string("draw_ranges_").size());
        auto git = node_geo.find(node_name);
        if (git == node_geo.end())
            continue;
        const std::vector<float> &pos = git->second.first;
        const std::vector<uint32_t> &idx = git->second.second;
        const tinygltf::Value &ranges = extras.Get(key);
        for (const std::string &nid : ranges.Keys()) {
            const tinygltf::Value &rng = ranges.Get(nid);
            if (!rng.IsArray() || rng.ArrayLen() < 2)
                continue;
            size_t start = (size_t) rng.Get(0).GetNumberAsInt();
            size_t count = (size_t) rng.Get(1).GetNumberAsInt();
            if (count == 0 || start + count > idx.size())
                continue;
            ParsedElement e;
            e.node_id = nid;
            auto nit = name_of.find(nid);
            e.name = (nit != name_of.end()) ? nit->second : nid;
            auto git2 = guid_of.find(e.name);
            if (git2 != guid_of.end())
                e.guid = git2->second;
            auto eit = etype_of.find(e.name);
            if (eit != etype_of.end())
                e.etype = eit->second;
            auto mit = meta_of.find(e.name);
            if (mit != meta_of.end())
                e.meta_sig = mit->second;
            e.tris.reserve(count * 3);
            for (size_t i = 0; i < count; ++i) {
                uint32_t vi = idx[start + i];
                e.tris.push_back(pos[3 * vi]);
                e.tris.push_back(pos[3 * vi + 1]);
                e.tris.push_back(pos[3 * vi + 2]);
            }
            out.push_back(std::move(e));
        }
    }
    return out;
}

// Minimal single-colour GLB from a set of elements' triangles (unshared verts). For the removed
// overlay. rgba packed 0xRRGGBBAA.
inline std::string write_overlay_glb(const std::vector<const ParsedElement *> &elems, uint32_t rgba) {
    std::vector<float> pos;
    for (const ParsedElement *e : elems)
        pos.insert(pos.end(), e->tris.begin(), e->tris.end());
    const uint32_t nverts = (uint32_t) (pos.size() / 3);
    std::vector<uint32_t> idx(nverts);
    for (uint32_t i = 0; i < nverts; ++i)
        idx[i] = i;

    // bbox for the accessor min/max (required for POSITION)
    float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
    for (uint32_t i = 0; i < nverts; ++i)
        for (int k = 0; k < 3; ++k) {
            mn[k] = std::min(mn[k], pos[3 * i + k]);
            mx[k] = std::max(mx[k], pos[3 * i + k]);
        }
    if (nverts == 0) {
        mn[0] = mn[1] = mn[2] = mx[0] = mx[1] = mx[2] = 0;
    }

    const size_t pos_bytes = pos.size() * 4, idx_bytes = idx.size() * 4;
    const size_t idx_off = pos_bytes; // bin layout: positions then indices
    const double r = ((rgba >> 24) & 0xff) / 255.0, g = ((rgba >> 16) & 0xff) / 255.0;
    const double b = ((rgba >> 8) & 0xff) / 255.0, a = (rgba & 0xff) / 255.0;

    std::ostringstream js;
    js << "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
       << "\"nodes\":[{\"mesh\":0}],"
       << "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1,\"material\":0}]}],"
       << "\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorFactor\":[" << r << "," << g << "," << b << ","
       << a << "],\"metallicFactor\":0.1,\"roughnessFactor\":0.8}}],"
       << "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":" << nverts
       << ",\"type\":\"VEC3\",\"min\":[" << mn[0] << "," << mn[1] << "," << mn[2] << "],\"max\":[" << mx[0] << ","
       << mx[1] << "," << mx[2] << "]},{\"bufferView\":1,\"componentType\":5125,\"count\":" << idx.size()
       << ",\"type\":\"SCALAR\"}],"
       << "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":" << pos_bytes
       << ",\"target\":34962},{\"buffer\":0,\"byteOffset\":" << idx_off << ",\"byteLength\":" << idx_bytes
       << ",\"target\":34963}],"
       << "\"buffers\":[{\"byteLength\":" << (pos_bytes + idx_bytes) << "}]}";
    std::string json = js.str();
    json.append(pad4((uint32_t) json.size()), ' ');

    std::string bin;
    bin.resize(pos_bytes + idx_bytes);
    std::memcpy(&bin[0], pos.data(), pos_bytes);
    std::memcpy(&bin[pos_bytes], idx.data(), idx_bytes);
    bin.append(pad4((uint32_t) bin.size()), '\0');

    const uint32_t json_len = (uint32_t) json.size(), bin_len = (uint32_t) bin.size();
    const uint32_t total = 12 + 8 + json_len + 8 + bin_len;
    std::string glb;
    glb.reserve(total);
    auto u32 = [&](uint32_t v) { glb.append((const char *) &v, 4); };
    glb.append("glTF", 4);
    u32(2);
    u32(total);
    u32(json_len);
    u32(0x4E4F534A); // "JSON"
    glb.append(json);
    u32(bin_len);
    u32(0x004E4942); // "BIN\0"
    glb.append(bin);
    return glb;
}

} // namespace gdiff
} // namespace adacpp
