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

#include "glb_diff.h"

namespace adacpp {
namespace gdiff {

inline uint32_t pad4(uint32_t n) {
    return (4 - (n & 3u)) & 3u;
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

// Read a scalar/vec accessor into a flat double buffer (handles float/uint16/uint32; v1 = no meshopt).
inline void read_accessor_floats(const tinygltf::Model &model, int acc_idx, std::vector<float> &out, int &ncomp) {
    const tinygltf::Accessor &acc = model.accessors[acc_idx];
    const tinygltf::BufferView &bv = model.bufferViews[acc.bufferView];
    const tinygltf::Buffer &buf = model.buffers[bv.buffer];
    ncomp = (acc.type == TINYGLTF_TYPE_VEC3) ? 3 : (acc.type == TINYGLTF_TYPE_VEC2 ? 2 : 1);
    size_t off = bv.byteOffset + acc.byteOffset;
    out.resize(acc.count * ncomp);
    const unsigned char *p = buf.data.data() + off;
    for (size_t i = 0; i < acc.count * (size_t) ncomp; ++i) {
        float f;
        std::memcpy(&f, p + i * 4, 4);
        out[i] = f;
    }
}

inline void read_accessor_indices(const tinygltf::Model &model, int acc_idx, std::vector<uint32_t> &out) {
    const tinygltf::Accessor &acc = model.accessors[acc_idx];
    const tinygltf::BufferView &bv = model.bufferViews[acc.bufferView];
    const tinygltf::Buffer &buf = model.buffers[bv.buffer];
    size_t off = bv.byteOffset + acc.byteOffset;
    const unsigned char *p = buf.data.data() + off;
    out.resize(acc.count);
    if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
        for (size_t i = 0; i < acc.count; ++i) {
            uint32_t v;
            std::memcpy(&v, p + i * 4, 4);
            out[i] = v;
        }
    } else { // UNSIGNED_SHORT
        for (size_t i = 0; i < acc.count; ++i) {
            uint16_t v;
            std::memcpy(&v, p + i * 2, 2);
            out[i] = v;
        }
    }
}

// Parse a GLB into per-element ParsedElement records (mirrors adapy diff.py parse_elements).
inline std::vector<ParsedElement> parse_glb_elements(const std::string &glb) {
    std::vector<ParsedElement> out;
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    if (!loader.LoadBinaryFromMemory(&model, &err, &warn, (const unsigned char *) glb.data(), glb.size()))
        return out;
    if (model.scenes.empty())
        return out;
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

    // node name -> (positions, indices)
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
        read_accessor_floats(model, pit->second, pos, nc);
        std::vector<uint32_t> idx;
        read_accessor_indices(model, prim.indices, idx);
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
