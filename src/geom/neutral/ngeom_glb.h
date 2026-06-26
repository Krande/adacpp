// Native GLB writer for the NGEOM/libtess2 path — a C++ peer of the adapy hand-authored
// glb_spill.py writer, producing the same glTF structure the adapy viewer expects (merge-by-colour
// materials, per-material POSITION+indices accessors, root + per-material nodes, PBR look matching
// step2glb). Hand-authored JSON + GLB framing, no external dependency, OCC-free (wasm-safe).
//
// This first slice merges in RAM (bakes per-instance world transforms into the vertices). A
// disk-spilling variant + ADA_EXT metadata + picking draw-ranges follow; the streaming STEP reader
// feeds solids in one at a time so the full pipeline stays memory-bounded.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace adacpp::glb {

// One tessellated solid to place in the GLB: local-space mesh + colour + per-instance world
// transforms (column-major / glTF order; empty => a single identity instance).
struct GlbSolid {
    std::vector<float> positions; // flat xyz (local)
    std::vector<uint32_t> indices;
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
    std::vector<std::array<float, 16>> transforms;
    std::string id;
};

namespace glb_detail {

inline std::string fnum(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.9g", v);
    return b;
}
inline uint32_t pad4(uint32_t n) {
    return (4 - (n & 3u)) & 3u;
}

struct MatBuf {
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
    std::vector<float> pos;
    std::vector<uint32_t> idx;
    uint32_t idx_max = 0;
    float lo[3] = {1e30f, 1e30f, 1e30f};
    float hi[3] = {-1e30f, -1e30f, -1e30f};
};

constexpr float IDENT[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

inline void xform(const float *M, float x, float y, float z, float &ox, float &oy, float &oz) {
    ox = M[0] * x + M[4] * y + M[8] * z + M[12]; // column-major M * [x y z 1]
    oy = M[1] * x + M[5] * y + M[9] * z + M[13];
    oz = M[2] * x + M[6] * y + M[10] * z + M[14];
}

inline std::string base_color_factor(const std::array<float, 4> &c) {
    return "[" + fnum(c[0]) + "," + fnum(c[1]) + "," + fnum(c[2]) + "," + fnum(c[3]) + "]";
}

} // namespace glb_detail

// Merge `solids` by colour (baking transforms) and write a GLB to `path`. Returns false on I/O
// error. An empty input writes a valid empty-scene GLB.
inline bool write_glb(const std::string &path, const std::vector<GlbSolid> &solids) {
    using namespace glb_detail;

    // colour-keyed merge (quantize to 1e-4 so near-equal colours share a material)
    std::map<std::array<int, 4>, MatBuf> mats;
    for (const GlbSolid &s : solids) {
        if (s.indices.empty() || s.positions.size() < 3)
            continue;
        std::array<int, 4> key = {(int) std::lround(s.color[0] * 1e4), (int) std::lround(s.color[1] * 1e4),
                                  (int) std::lround(s.color[2] * 1e4), (int) std::lround(s.color[3] * 1e4)};
        MatBuf &m = mats[key];
        m.color = s.color;
        size_t ninst = s.transforms.empty() ? 1 : s.transforms.size();
        for (size_t t = 0; t < ninst; ++t) {
            const float *M = s.transforms.empty() ? IDENT : s.transforms[t].data();
            uint32_t base = (uint32_t) (m.pos.size() / 3);
            for (size_t i = 0; i + 2 < s.positions.size(); i += 3) {
                float ox, oy, oz;
                xform(M, s.positions[i], s.positions[i + 1], s.positions[i + 2], ox, oy, oz);
                m.pos.push_back(ox);
                m.pos.push_back(oy);
                m.pos.push_back(oz);
                m.lo[0] = std::min(m.lo[0], ox);
                m.lo[1] = std::min(m.lo[1], oy);
                m.lo[2] = std::min(m.lo[2], oz);
                m.hi[0] = std::max(m.hi[0], ox);
                m.hi[1] = std::max(m.hi[1], oy);
                m.hi[2] = std::max(m.hi[2], oz);
            }
            for (uint32_t ix : s.indices) {
                uint32_t v = base + ix;
                m.idx.push_back(v);
                m.idx_max = std::max(m.idx_max, v);
            }
        }
    }

    // Drop empties; fix a deterministic material order (std::map is key-sorted).
    std::vector<const MatBuf *> ms;
    for (const auto &[k, m] : mats)
        if (!m.idx.empty() && !m.pos.empty())
            ms.push_back(&m);

    // Build JSON arrays + the BIN layout (per material: indices then POSITION, each 4-byte padded).
    std::ostringstream bv, acc, meshes, materials;
    uint32_t bin_off = 0;
    for (size_t i = 0; i < ms.size(); ++i) {
        const MatBuf &m = *ms[i];
        uint32_t vcount = (uint32_t) (m.pos.size() / 3);
        uint32_t idx_bytes = (uint32_t) m.idx.size() * 4;
        uint32_t pos_bytes = vcount * 3 * 4;
        int bv_idx = (int) i * 2, bv_pos = (int) i * 2 + 1;
        int acc_idx = (int) i * 2, acc_pos = (int) i * 2 + 1;
        // indices bufferView/accessor
        bv << (i ? "," : "") << "{\"buffer\":0,\"byteOffset\":" << bin_off << ",\"byteLength\":" << idx_bytes << "}";
        bin_off += idx_bytes + pad4(idx_bytes);
        acc << (i ? "," : "") << "{\"bufferView\":" << bv_idx << ",\"componentType\":5125,\"count\":" << m.idx.size()
            << ",\"type\":\"SCALAR\",\"min\":[0],\"max\":[" << m.idx_max << "]}";
        // POSITION bufferView/accessor
        bv << ",{\"buffer\":0,\"byteOffset\":" << bin_off << ",\"byteLength\":" << pos_bytes << "}";
        bin_off += pos_bytes + pad4(pos_bytes);
        acc << ",{\"bufferView\":" << bv_pos << ",\"componentType\":5126,\"count\":" << vcount
            << ",\"type\":\"VEC3\",\"min\":[" << fnum(m.lo[0]) << "," << fnum(m.lo[1]) << "," << fnum(m.lo[2])
            << "],\"max\":[" << fnum(m.hi[0]) << "," << fnum(m.hi[1]) << "," << fnum(m.hi[2]) << "]}";
        meshes << (i ? "," : "") << "{\"primitives\":[{\"attributes\":{\"POSITION\":" << acc_pos
               << "},\"indices\":" << acc_idx << ",\"mode\":4,\"material\":" << i << "}]}";
        materials << (i ? "," : "") << "{\"pbrMetallicRoughness\":{\"baseColorFactor\":" << base_color_factor(m.color)
                  << ",\"metallicFactor\":0.1,\"roughnessFactor\":0.7},\"doubleSided\":true"
                  << (m.color[3] < 1.0f ? ",\"alphaMode\":\"BLEND\"" : "") << "}";
    }
    uint32_t bin_len = bin_off;

    std::ostringstream nodes;
    nodes << "{\"name\":\"root\"";
    if (!ms.empty()) {
        nodes << ",\"children\":[";
        for (size_t i = 0; i < ms.size(); ++i)
            nodes << (i ? "," : "") << (i + 1);
        nodes << "]";
    }
    nodes << "}";
    for (size_t i = 0; i < ms.size(); ++i)
        nodes << ",{\"mesh\":" << i << "}";

    std::ostringstream js;
    js << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"adacpp-native-glb\"},\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[0],\"extras\":{}}],\"nodes\":[" << nodes.str() << "],"
       << "\"buffers\":[{\"byteLength\":" << bin_len << "}],\"bufferViews\":[" << bv.str() << "],"
       << "\"accessors\":[" << acc.str() << "],\"meshes\":[" << meshes.str() << "],\"materials\":[" << materials.str()
       << "]}";
    std::string json = js.str();
    uint32_t jpad = pad4((uint32_t) json.size());
    json.append(jpad, ' '); // pad JSON chunk with spaces
    uint32_t json_len = (uint32_t) json.size();
    uint32_t total = 12 + 8 + json_len + 8 + bin_len;

    std::ofstream out(path, std::ios::binary);
    if (!out)
        return false;
    auto u32 = [&](uint32_t v) { out.write(reinterpret_cast<const char *>(&v), 4); };
    u32(0x46546C67);
    u32(2);
    u32(total); // "glTF", version 2, total
    u32(json_len);
    u32(0x4E4F534A); // JSON chunk
    out.write(json.data(), json.size());
    u32(bin_len);
    u32(0x004E4942); // BIN chunk
    static const char zeros[4] = {0, 0, 0, 0};
    for (const MatBuf *mp : ms) {
        const MatBuf &m = *mp;
        uint32_t idx_bytes = (uint32_t) m.idx.size() * 4;
        out.write(reinterpret_cast<const char *>(m.idx.data()), idx_bytes);
        out.write(zeros, pad4(idx_bytes));
        uint32_t pos_bytes = (uint32_t) m.pos.size() * 4;
        out.write(reinterpret_cast<const char *>(m.pos.data()), pos_bytes);
        out.write(zeros, pad4(pos_bytes));
    }
    return (bool) out;
}

} // namespace adacpp::glb
