// Native GLB writer for the NGEOM/libtess2 path — a C++ peer of adapy's hand-authored glb_spill.py.
// Produces the glTF structure the viewer expects: merge-by-colour materials, per-material POSITION+
// indices accessors, root + per-material nodes, the step2glb PBR look, and GLB chunk framing. Hand-
// authored JSON, no dependency, OCC-free (wasm-safe). Bakes per-instance world transforms.
//
// Two writers share the framing (glb_write_framed): write_glb (in-RAM merge, for small inputs) and
// GlbSpillWriter (+ write_glb_merged) which spills each material to disk per "lane" and assembles by
// streaming the lane files — so a large model never holds the merged buffers in RAM, and parallel
// workers each own a lane (no shared-file contention). The merge re-offsets each lane's indices by
// its cumulative vertex base.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

constexpr float IDENT[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

inline void xform(const float *M, float x, float y, float z, float &ox, float &oy, float &oz) {
    ox = M[0] * x + M[4] * y + M[8] * z + M[12]; // column-major M * [x y z 1]
    oy = M[1] * x + M[5] * y + M[9] * z + M[13];
    oz = M[2] * x + M[6] * y + M[10] * z + M[14];
}

// Per-material header (everything the glTF JSON + accessors need, minus the raw bytes).
struct MatHeader {
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
    uint32_t vert_count = 0;
    uint32_t index_count = 0;
    uint32_t idx_max = 0;
    float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
};

// Build the glTF JSON + GLB framing for the given material headers and stream the BIN via
// `write_bin(out)`, which must write exactly [indices(padded) + positions(padded)] per material in
// order. The caller supplies the bytes (from RAM buffers or spilled files).
template <class BinWriter>
inline bool glb_write_framed(const std::string &path, const std::vector<MatHeader> &mats, BinWriter &&write_bin) {
    std::ostringstream bv, acc, meshes, materials, nodes;
    uint32_t bin_off = 0;
    for (size_t i = 0; i < mats.size(); ++i) {
        const MatHeader &m = mats[i];
        uint32_t idx_bytes = m.index_count * 4, pos_bytes = m.vert_count * 3 * 4;
        bv << (i ? "," : "") << "{\"buffer\":0,\"byteOffset\":" << bin_off << ",\"byteLength\":" << idx_bytes << "}";
        acc << (i ? "," : "") << "{\"bufferView\":" << i * 2 << ",\"componentType\":5125,\"count\":" << m.index_count
            << ",\"type\":\"SCALAR\",\"min\":[0],\"max\":[" << m.idx_max << "]}";
        bin_off += idx_bytes + pad4(idx_bytes);
        bv << ",{\"buffer\":0,\"byteOffset\":" << bin_off << ",\"byteLength\":" << pos_bytes << "}";
        acc << ",{\"bufferView\":" << i * 2 + 1 << ",\"componentType\":5126,\"count\":" << m.vert_count
            << ",\"type\":\"VEC3\",\"min\":[" << fnum(m.lo[0]) << "," << fnum(m.lo[1]) << "," << fnum(m.lo[2])
            << "],\"max\":[" << fnum(m.hi[0]) << "," << fnum(m.hi[1]) << "," << fnum(m.hi[2]) << "]}";
        bin_off += pos_bytes + pad4(pos_bytes);
        meshes << (i ? "," : "") << "{\"primitives\":[{\"attributes\":{\"POSITION\":" << i * 2 + 1
               << "},\"indices\":" << i * 2 << ",\"mode\":4,\"material\":" << i << "}]}";
        materials << (i ? "," : "") << "{\"pbrMetallicRoughness\":{\"baseColorFactor\":[" << fnum(m.color[0]) << ","
                  << fnum(m.color[1]) << "," << fnum(m.color[2]) << "," << fnum(m.color[3])
                  << "],\"metallicFactor\":0.1,\"roughnessFactor\":0.7},\"doubleSided\":true"
                  << (m.color[3] < 1.0f ? ",\"alphaMode\":\"BLEND\"" : "") << "}";
    }
    uint32_t bin_len = bin_off;
    nodes << "{\"name\":\"root\"";
    if (!mats.empty()) {
        nodes << ",\"children\":[";
        for (size_t i = 0; i < mats.size(); ++i)
            nodes << (i ? "," : "") << (i + 1);
        nodes << "]";
    }
    nodes << "}";
    for (size_t i = 0; i < mats.size(); ++i)
        nodes << ",{\"mesh\":" << i << "}";

    std::ostringstream js;
    js << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"adacpp-native-glb\"},\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[0],\"extras\":{}}],\"nodes\":[" << nodes.str() << "],"
       << "\"buffers\":[{\"byteLength\":" << bin_len << "}],\"bufferViews\":[" << bv.str() << "],"
       << "\"accessors\":[" << acc.str() << "],\"meshes\":[" << meshes.str() << "],\"materials\":[" << materials.str()
       << "]}";
    std::string json = js.str();
    json.append(pad4((uint32_t) json.size()), ' ');
    uint32_t json_len = (uint32_t) json.size();
    uint32_t total = 12 + 8 + json_len + 8 + bin_len;

    std::ofstream out(path, std::ios::binary);
    if (!out)
        return false;
    auto u32 = [&](uint32_t v) { out.write(reinterpret_cast<const char *>(&v), 4); };
    u32(0x46546C67);
    u32(2);
    u32(total);
    u32(json_len);
    u32(0x4E4F534Au);
    out.write(json.data(), json.size());
    u32(bin_len);
    u32(0x004E4942u);
    write_bin(out);
    return (bool) out;
}

inline std::array<int, 4> colour_key(const std::array<float, 4> &c) {
    return {(int) std::lround(c[0] * 1e4), (int) std::lround(c[1] * 1e4), (int) std::lround(c[2] * 1e4),
            (int) std::lround(c[3] * 1e4)};
}

} // namespace glb_detail

// In-RAM merge-by-colour writer (for small/medium inputs + tests).
inline bool write_glb(const std::string &path, const std::vector<GlbSolid> &solids) {
    using namespace glb_detail;
    struct Buf {
        std::array<float, 4> color;
        std::vector<float> pos;
        std::vector<uint32_t> idx;
        uint32_t idx_max = 0;
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    };
    std::map<std::array<int, 4>, Buf> mats;
    for (const GlbSolid &s : solids) {
        if (s.indices.empty() || s.positions.size() < 3)
            continue;
        Buf &m = mats[colour_key(s.color)];
        m.color = s.color;
        size_t ninst = s.transforms.empty() ? 1 : s.transforms.size();
        for (size_t t = 0; t < ninst; ++t) {
            const float *M = s.transforms.empty() ? IDENT : s.transforms[t].data();
            uint32_t base = (uint32_t) (m.pos.size() / 3);
            for (size_t i = 0; i + 2 < s.positions.size(); i += 3) {
                float o[3];
                xform(M, s.positions[i], s.positions[i + 1], s.positions[i + 2], o[0], o[1], o[2]);
                for (int k = 0; k < 3; ++k) {
                    m.pos.push_back(o[k]);
                    m.lo[k] = std::min(m.lo[k], o[k]);
                    m.hi[k] = std::max(m.hi[k], o[k]);
                }
            }
            for (uint32_t ix : s.indices) {
                uint32_t v = base + ix;
                m.idx.push_back(v);
                m.idx_max = std::max(m.idx_max, v);
            }
        }
    }
    std::vector<MatHeader> hdrs;
    std::vector<const Buf *> bufs;
    for (const auto &[k, m] : mats)
        if (!m.idx.empty()) {
            MatHeader h;
            h.color = m.color;
            h.vert_count = (uint32_t) (m.pos.size() / 3);
            h.index_count = (uint32_t) m.idx.size();
            h.idx_max = m.idx_max;
            for (int k2 = 0; k2 < 3; ++k2) {
                h.lo[k2] = m.lo[k2];
                h.hi[k2] = m.hi[k2];
            }
            hdrs.push_back(h);
            bufs.push_back(&m);
        }
    static const char z[4] = {0, 0, 0, 0};
    return glb_write_framed(path, hdrs, [&](std::ofstream &out) {
        for (const Buf *b : bufs) {
            uint32_t ib = (uint32_t) b->idx.size() * 4, pb = (uint32_t) b->pos.size() * 4;
            out.write(reinterpret_cast<const char *>(b->idx.data()), ib);
            out.write(z, pad4(ib));
            out.write(reinterpret_cast<const char *>(b->pos.data()), pb);
            out.write(z, pad4(pb));
        }
    });
}

// Disk-spilling writer: one "lane" (one producer / worker). add() bakes + appends each material's
// verts/indices to per-material temp files; the merge (write_glb_merged) streams them into the GLB.
// Keeps only per-material metadata in RAM, never the merged buffers.
class GlbSpillWriter {
public:
    struct MatLane {
        std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
        std::string pos_path, idx_path;
        std::ofstream pos, idx;
        uint32_t vert_count = 0, index_count = 0, idx_max = 0;
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    };

    GlbSpillWriter(std::string dir, int lane) : dir_(std::move(dir)), lane_(lane) {}
    ~GlbSpillWriter() {
        remove_files();
    }

    void add(const GlbSolid &s) {
        using namespace glb_detail;
        if (s.indices.empty() || s.positions.size() < 3)
            return;
        std::array<int, 4> key = colour_key(s.color);
        MatLane &m = mats_[key];
        if (m.pos_path.empty())
            open(m, key);
        m.color = s.color;
        size_t ninst = s.transforms.empty() ? 1 : s.transforms.size();
        for (size_t t = 0; t < ninst; ++t) {
            const float *M = s.transforms.empty() ? IDENT : s.transforms[t].data();
            uint32_t base = m.vert_count;
            for (size_t i = 0; i + 2 < s.positions.size(); i += 3) {
                float o[3];
                xform(M, s.positions[i], s.positions[i + 1], s.positions[i + 2], o[0], o[1], o[2]);
                m.pos.write(reinterpret_cast<const char *>(o), 12);
                for (int k = 0; k < 3; ++k) {
                    m.lo[k] = std::min(m.lo[k], o[k]);
                    m.hi[k] = std::max(m.hi[k], o[k]);
                }
                ++m.vert_count;
            }
            for (uint32_t ix : s.indices) {
                uint32_t v = base + ix;
                m.idx.write(reinterpret_cast<const char *>(&v), 4);
                m.idx_max = std::max(m.idx_max, v);
                ++m.index_count;
            }
        }
    }
    void flush() {
        for (auto &[k, m] : mats_) {
            m.pos.flush();
            m.idx.flush();
        }
    }
    const std::map<std::array<int, 4>, MatLane> &mats() const {
        return mats_;
    }

private:
    void open(MatLane &m, const std::array<int, 4> &key) {
        char tag[64];
        std::snprintf(tag, sizeof tag, "/glb_l%d_%d_%d_%d_%d", lane_, key[0], key[1], key[2], key[3]);
        m.pos_path = dir_ + tag + ".pos";
        m.idx_path = dir_ + tag + ".idx";
        m.pos.open(m.pos_path, std::ios::binary);
        m.idx.open(m.idx_path, std::ios::binary);
    }
    void remove_files() {
        for (auto &[k, m] : mats_) {
            m.pos.close();
            m.idx.close();
            if (!m.pos_path.empty())
                std::remove(m.pos_path.c_str());
            if (!m.idx_path.empty())
                std::remove(m.idx_path.c_str());
        }
    }
    std::string dir_;
    int lane_;
    std::map<std::array<int, 4>, MatLane> mats_;
};

// Merge N lanes (one per worker) into a GLB. Per material: positions are concatenated across lanes;
// each lane's indices are re-offset by its cumulative vertex base in that material. Streams the lane
// files into the BIN chunk — never materializes the merged buffers.
inline bool write_glb_merged(const std::string &path, const std::vector<GlbSpillWriter *> &lanes) {
    using namespace glb_detail;
    for (GlbSpillWriter *l : lanes)
        l->flush();
    // union of material keys across lanes, with summed counts
    std::map<std::array<int, 4>, MatHeader> hdr;
    for (GlbSpillWriter *l : lanes)
        for (const auto &[key, m] : l->mats()) {
            MatHeader &h = hdr[key];
            h.color = m.color;
            if (h.index_count == 0)
                for (int k = 0; k < 3; ++k) {
                    h.lo[k] = m.lo[k];
                    h.hi[k] = m.hi[k];
                }
            else
                for (int k = 0; k < 3; ++k) {
                    h.lo[k] = std::min(h.lo[k], m.lo[k]);
                    h.hi[k] = std::max(h.hi[k], m.hi[k]);
                }
            h.idx_max = std::max(h.idx_max, m.idx_max + h.vert_count); // after re-offset
            h.vert_count += m.vert_count;
            h.index_count += m.index_count;
        }
    std::vector<std::array<int, 4>> keys;
    std::vector<MatHeader> hdrs;
    for (const auto &[key, h] : hdr)
        if (h.index_count) {
            keys.push_back(key);
            hdrs.push_back(h);
        }
    static const char z[4] = {0, 0, 0, 0};
    return glb_write_framed(path, hdrs, [&](std::ofstream &out) {
        for (const std::array<int, 4> &key : keys) {
            // indices: each lane re-offset by the cumulative vertex base, padded once at the end
            uint32_t base = 0, idx_bytes = 0;
            for (GlbSpillWriter *l : lanes) {
                auto it = l->mats().find(key);
                if (it == l->mats().end())
                    continue;
                const auto &m = it->second;
                std::ifstream f(m.idx_path, std::ios::binary);
                uint32_t v;
                while (f.read(reinterpret_cast<char *>(&v), 4)) {
                    uint32_t ov = v + base;
                    out.write(reinterpret_cast<const char *>(&ov), 4);
                }
                base += m.vert_count;
                idx_bytes += m.index_count * 4;
            }
            out.write(z, pad4(idx_bytes));
            // positions: raw concat across lanes
            uint32_t pos_bytes = 0;
            for (GlbSpillWriter *l : lanes) {
                auto it = l->mats().find(key);
                if (it == l->mats().end())
                    continue;
                const auto &m = it->second;
                std::ifstream f(m.pos_path, std::ios::binary);
                out << f.rdbuf();
                pos_bytes += m.vert_count * 12;
            }
            out.write(z, pad4(pos_bytes));
        }
    });
}

} // namespace adacpp::glb
