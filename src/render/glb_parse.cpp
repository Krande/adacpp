#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// tinygltf's implementation is compiled here, by the first include of glb_gltf.h.
#define TINYGLTF_IMPLEMENTATION
#ifndef __EMSCRIPTEN__
#define STB_IMAGE_IMPLEMENTATION
#endif
#include "glb_gltf.h"

#ifdef ADACPP_RENDER_DRACO
#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/mesh/mesh.h>
#endif

#include <meshoptimizer.h>

#include "glb_parse.h"

namespace adacpp::render::detail {

namespace {

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

} // namespace

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
            case TINYGLTF_COMPONENT_TYPE_BYTE: {
                const float b = float(reinterpret_cast<const int8_t *>(e)[c]);
                v = acc.normalized ? std::max(b / 127.0f, -1.0f) : b;
                break;
            }
            case TINYGLTF_COMPONENT_TYPE_SHORT: {
                const float b = float(reinterpret_cast<const int16_t *>(e)[c]);
                v = acc.normalized ? std::max(b / 32767.0f, -1.0f) : b;
                break;
            }
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

bool read_draco(const tinygltf::Model &model, const tinygltf::Primitive &prim, std::vector<float> &positions,
                std::vector<float> &normals, std::vector<float> &colors, std::vector<uint32_t> &indices) {
    const auto ext = prim.extensions.find("KHR_draco_mesh_compression");
    if (ext == prim.extensions.end())
        return false;
#ifdef ADACPP_RENDER_DRACO
    const tinygltf::Value &view_index = ext->second.Get("bufferView"), &ids = ext->second.Get("attributes");
    if (!view_index.IsInt() || !ids.IsObject())
        throw std::runtime_error("adacpp.render: malformed KHR_draco_mesh_compression");
    const auto &view = model.bufferViews.at(view_index.Get<int>());
    const auto &buf = model.buffers.at(view.buffer);
    if (view.byteOffset + view.byteLength > buf.data.size())
        throw std::runtime_error("adacpp.render: Draco data outside its buffer");
    draco::DecoderBuffer in;
    in.Init(reinterpret_cast<const char *>(buf.data.data()) + view.byteOffset, view.byteLength);
    draco::Decoder decoder;
    auto decoded = decoder.DecodeMeshFromBuffer(&in);
    if (!decoded.ok())
        throw std::runtime_error("adacpp.render: Draco decode failed: " + decoded.status().error_msg_string());
    const std::unique_ptr<draco::Mesh> mesh = std::move(decoded).value();
    const size_t points = mesh->num_points();

    // `comps` floats per point, as read_floats gives them: a missing component is 1, colours are 0..1.
    auto attribute = [&](const char *name, int comps, bool color, std::vector<float> &out) {
        if (!ids.Has(name) || !ids.Get(name).IsInt())
            return;
        const draco::PointAttribute *att = mesh->GetAttributeByUniqueId(uint32_t(ids.Get(name).Get<int>()));
        if (!att)
            throw std::runtime_error(std::string("adacpp.render: Draco data has no ") + name);
        const int have = std::min<int>(att->num_components(), comps);
        float scale = 1.0f;
        if (color && !att->normalized()) {
            if (att->data_type() == draco::DT_UINT8)
                scale = 1.0f / 255.0f;
            else if (att->data_type() == draco::DT_UINT16)
                scale = 1.0f / 65535.0f;
        }
        out.assign(points * comps, 1.0f);
        float v[4];
        for (draco::PointIndex i(0); i < points; ++i) {
            if (!att->ConvertValue<float>(att->mapped_index(i), int8_t(have), v))
                throw std::runtime_error(std::string("adacpp.render: Draco ") + name + " cannot be read");
            for (int c = 0; c < have; ++c)
                out[size_t(i.value()) * comps + c] = v[c] * scale;
        }
    };
    attribute("POSITION", 3, false, positions);
    attribute("NORMAL", 3, false, normals);
    attribute("COLOR_0", 4, true, colors);
    if (positions.empty())
        throw std::runtime_error("adacpp.render: Draco data has no POSITION");
    indices.resize(size_t(mesh->num_faces()) * 3);
    for (draco::FaceIndex f(0); f < mesh->num_faces(); ++f)
        for (int c = 0; c < 3; ++c)
            indices[size_t(f.value()) * 3 + c] = mesh->face(f)[c].value();
    return true;
#else
    (void) positions, (void) normals, (void) colors, (void) indices;
    // The uncompressed copy a GLB may carry beside the Draco data is read like any primitive.
    const auto pos = prim.attributes.find("POSITION");
    if (pos != prim.attributes.end() && model.accessors.at(pos->second).bufferView >= 0)
        return false;
    throw std::runtime_error("adacpp.render: KHR_draco_mesh_compression is not supported by this build");
#endif
}

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

tinygltf::Model parse_glb(std::vector<unsigned char> bytes, const std::string &name) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err, warn;
    // Images are never needed: the renderer does not texture.
    loader.SetImageLoader([](tinygltf::Image *, const int, std::string *, std::string *, int, int,
                             const unsigned char *, int, void *) { return true; },
                          nullptr);
    bytes = decode_meshopt_glb(std::move(bytes));
    const bool ok = loader.LoadBinaryFromMemory(&model, &err, &warn, bytes.data(), unsigned(bytes.size()));
    bytes.clear();
    bytes.shrink_to_fit();
    if (!ok)
        throw std::runtime_error("adacpp.render: failed to read " + name + ": " + err);
#ifndef ADACPP_RENDER_DRACO
    for (const auto &ext : model.extensionsRequired)
        if (ext == "KHR_draco_mesh_compression")
            throw std::runtime_error("adacpp.render: " + name + " requires " + ext + ", not supported by this build");
#endif
    return model;
}

} // namespace adacpp::render::detail
