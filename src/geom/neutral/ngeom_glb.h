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
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ngeom_meshopt.h" // EXT_meshopt_compression codecs (vendored meshoptimizer)

namespace adacpp::glb {

// One tessellated solid to place in the GLB: local-space mesh + colour + per-instance world
// transforms (column-major / glTF order; empty => a single identity instance).
// One clickable face region within a solid — the triangle range RELATIVE to the solid's own index
// buffer (the viewer adds it to the solid's draw-range start), plus the source face id + sequence.
// Emitted into scenes[0].extras as face_ranges_node<m> only when face-region capture is requested.
struct FaceSub {
    uint32_t start = 0;  // first index, relative to the solid's draw range
    uint32_t length = 0; // index count (3 * triangles)
    int64_t face_id = 0; // source entity id (STEP/IFC #id), 0 if unknown
    uint32_t seq = 0;    // 0-based face position within the solid
    // Per-face presentation colour (STEP per-face styling). has_color=false => inherit the solid's base
    // colour. Used only by split_solid_by_face_colour to bucket a solid's faces into per-colour GlbSolids;
    // ignored by the picking/draw-range machinery downstream.
    bool has_color = false;
    float cr = 0.5f, cg = 0.5f, cb = 0.5f, ca = 1.0f;
};

struct GlbSolid {
    std::vector<float> positions; // flat xyz (local)
    std::vector<uint32_t> indices;
    std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
    std::vector<std::array<float, 16>> transforms;
    std::vector<FaceSub> face_ranges; // per-face regions (relative to this solid); empty unless captured
    std::string id;                   // solid's own name (fallback leaf name)
    std::string product_name;         // the solid's product name (the picking leaf name `gid`); "" => use id
    // Per-instance assembly path (root-first (rep_id, product_name) levels, last level = the solid's
    // own product), parallel to transforms. The writer emits ONE pickable leaf per instance — named
    // gid (k==0) / gid/k+1 — parented under path[:-1] (the solid's own level collapses into the leaf),
    // matching the Python scene builder 1:1.
    std::vector<std::vector<std::pair<int, std::string>>> instance_paths;
};

// Per-instance pickable-leaf name + parent path (the writer emits one per world placement). gid is the
// solid's product name (its own name as fallback); placements after the first are gid/k+1.
inline std::string instance_leaf_name(const GlbSolid &s, size_t t) {
    const std::string &gid = s.product_name.empty() ? s.id : s.product_name;
    return t == 0 ? gid : gid + "/" + std::to_string(t + 1);
}
// Collapse the solid's own (last) product level into the leaf ONLY for a single-placement solid (Python
// collapse_leaf, gated on n_inst==1): the leaf then IS the product node. A multi-placement solid keeps
// the full path so its product becomes a group node holding the gid / gid/k+1 instance leaves.
inline std::vector<std::pair<int, std::string>> instance_parent_path(const GlbSolid &s, size_t t) {
    if (t >= s.instance_paths.size() || s.instance_paths[t].empty())
        return {};
    const auto &p = s.instance_paths[t];
    size_t ninst = s.transforms.empty() ? 1 : s.transforms.size();
    size_t keep = (ninst == 1) ? p.size() - 1 : p.size();
    return {p.begin(), p.begin() + keep};
}

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

inline std::string json_escape(const std::string &s) {
    std::string o;
    o.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
        case '"':
            o += "\\\"";
            break;
        case '\\':
            o += "\\\\";
            break;
        case '\n':
            o += "\\n";
            break;
        case '\r':
            o += "\\r";
            break;
        case '\t':
            o += "\\t";
            break;
        default:
            if ((unsigned char) c < 0x20)
                o += ' ';
            else
                o += c;
        }
    }
    return o;
}

// Per-solid draw range within a material's merged index buffer (index units), for picking. `path` is
// the solid's root-first STEP assembly chain (rep_id, product_name) — used to build the id_hierarchy
// tree (empty => the solid is a direct child of the scene root).
struct PartRange {
    std::string id;
    uint32_t start, length;
    std::vector<std::pair<int, std::string>> path;
    std::vector<FaceSub> faces; // per-face regions relative to [start,length); empty unless captured
};

// Build the scenes[0].extras content: id_hierarchy {nid:[name,parent]} (the full STEP product tree —
// assembly nodes shared by path prefix, one pickable LEAF per instance/placement named by product;
// parent = "*" for roots) + draw_ranges_node<m> {leaf_nid:[start,len]}. Node ids are assigned here so
// the draw ranges and the hierarchy agree. Each PartRange = one placement (its parent path is the
// assembly above the solid; the solid's own level is collapsed into the leaf), 1:1 with the Python path.
inline std::string build_scene_extras(const std::vector<std::vector<PartRange>> &per_mat) {
    // Assembly nodes are shared by PATH PREFIX (the full root-first rep_id chain), not by rep_id alone
    // — so a sub-assembly instanced under two different parents yields two nodes (a tree, like the
    // Python asm_nodes keyed by prefix), not one shared DAG node.
    std::map<std::vector<int>, int> prefix_nid;
    std::ostringstream hier;
    int next = 0;
    bool first = true;
    auto emit = [&](int nid, const std::string &name, int parent) {
        if (!first)
            hier << ",";
        first = false;
        hier << "\"" << nid << "\":[\"" << json_escape(name) << "\",";
        if (parent < 0)
            hier << "\"*\"";
        else
            hier << parent; // parent node id (int), matching graph.to_json_hierarchy
        hier << "]";
    };
    // Synthetic root node (matches the Python GraphStore root: node id 0, name "root", parent "*").
    // Everything hangs under it, so the native tree depth lines up with Python's 1:1.
    const int root_nid = next++;
    emit(root_nid, "root", -1);
    std::vector<std::vector<int>> solid_nid(per_mat.size()); // per material/part -> the solid's node id
    for (size_t m = 0; m < per_mat.size(); ++m) {
        solid_nid[m].resize(per_mat[m].size());
        for (size_t k = 0; k < per_mat[m].size(); ++k) {
            const PartRange &r = per_mat[m][k];
            int parent = root_nid; // under the synthetic root
            std::vector<int> prefix;
            for (const auto &[rid, nm] : r.path) {
                prefix.push_back(rid);
                auto it = prefix_nid.find(prefix);
                if (it == prefix_nid.end()) {
                    int nid = next++;
                    prefix_nid[prefix] = nid;
                    emit(nid, nm, parent);
                    parent = nid;
                } else {
                    parent = it->second;
                }
            }
            int snid = next++;
            solid_nid[m][k] = snid;
            emit(snid, r.id, parent);
        }
    }
    std::ostringstream ranges;
    for (size_t m = 0; m < per_mat.size(); ++m) {
        ranges << ",\"draw_ranges_node" << m << "\":{";
        for (size_t k = 0; k < per_mat[m].size(); ++k) {
            if (k)
                ranges << ",";
            ranges << "\"" << solid_nid[m][k] << "\":[" << per_mat[m][k].start << "," << per_mat[m][k].length << "]";
        }
        ranges << "}";
    }
    // Per-face clickable regions (opt-in): face_ranges_node<m> {solid_nid:[[start,len,face_id,seq],...]}
    // where start/len are RELATIVE to that solid's draw range (the viewer adds the draw-range start).
    // Emitted only when some part carries face regions, so normal GLBs are unchanged.
    std::ostringstream franges;
    bool any_faces = false;
    for (const auto &pm : per_mat)
        for (const PartRange &r : pm)
            if (!r.faces.empty()) {
                any_faces = true;
                break;
            }
    if (any_faces) {
        for (size_t m = 0; m < per_mat.size(); ++m) {
            franges << ",\"face_ranges_node" << m << "\":{";
            bool firstk = true;
            for (size_t k = 0; k < per_mat[m].size(); ++k) {
                const PartRange &r = per_mat[m][k];
                if (r.faces.empty())
                    continue;
                if (!firstk)
                    franges << ",";
                firstk = false;
                franges << "\"" << solid_nid[m][k] << "\":[";
                for (size_t j = 0; j < r.faces.size(); ++j) {
                    const FaceSub &fs = r.faces[j];
                    if (j)
                        franges << ",";
                    franges << "[" << fs.start << "," << fs.length << "," << fs.face_id << "," << fs.seq << "]";
                }
                franges << "]";
            }
            franges << "}";
        }
    }
    std::ostringstream e;
    e << "\"id_hierarchy\":{" << hier.str() << "}" << ranges.str() << franges.str();
    return e.str();
}

// Build the glTF JSON + GLB framing. `write_bin(out)` writes [indices(padded)+positions(padded)] per
// material in order. `scene_extras` is the scenes[0].extras body (id_hierarchy + draw_ranges, may be
// empty); `ada_ext` is the ADA_EXT_data extension body (may be empty -> extension omitted).
template <class BinWriter>
inline bool glb_write_framed(const std::string &path, const std::vector<MatHeader> &mats,
                             const std::string &scene_extras, const std::string &ada_ext, BinWriter &&write_bin) {
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
        meshes << (i ? "," : "") << "{\"name\":\"node" << i
               << "\",\"primitives\":[{\"attributes\":{\"POSITION\":" << i * 2 + 1 << "},\"indices\":" << i * 2
               << ",\"mode\":4,\"material\":" << i << "}]}";
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
        nodes << ",{\"name\":\"node" << i << "\",\"mesh\":" << i << "}";

    std::ostringstream js;
    js << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"adacpp-native-glb\"},\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[0],\"extras\":{" << scene_extras << "}}],\"nodes\":[" << nodes.str() << "],"
       << "\"buffers\":[{\"byteLength\":" << bin_len << "}],\"bufferViews\":[" << bv.str() << "],"
       << "\"accessors\":[" << acc.str() << "],\"meshes\":[" << meshes.str() << "],\"materials\":[" << materials.str()
       << "]";
    if (!ada_ext.empty())
        js << ",\"extensionsUsed\":[\"ADA_EXT_data\"],\"extensions\":{\"ADA_EXT_data\":" << ada_ext << "}";
    js << "}";
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

// EXT_meshopt_compression framing (the in-writer meshopt option). For each material `provide(i, pos,
// idx)` fills the uncompressed buffers (from RAM or read from spill — one material at a time, so the
// whole model is never resident); they are encoded with the vendored meshoptimizer — indices via the
// INDICES codec (order-preserving, so scene.extras draw-range offsets stay valid) + positions via
// ATTRIBUTES — then emitted as buffer 0 = compressed data, buffer 1 = fallback (no bytes), bufferViews
// carrying the EXT extension. Round-trips every buffer; returns false on any encode/verify failure so
// the caller falls back to the raw writer. Peak memory = all compressed buffers + one material's raw.
template <class ProvideIdx, class ProvidePos>
inline bool glb_write_framed_meshopt(const std::string &path, const std::vector<MatHeader> &hdrs,
                                     ProvideIdx &&provide_idx, ProvidePos &&provide_pos,
                                     const std::string &scene_extras, const std::string &ada_ext) {
    auto a4 = [](uint32_t n) { return (n + 3u) & ~3u; };
    struct Enc {
        uint32_t uidx_off = 0, upos_off = 0, cidx_off = 0, cpos_off = 0, cidx_len = 0, cpos_len = 0;
    };
    std::vector<Enc> enc(hdrs.size());
    uint32_t coff = 0, uoff = 0;
    static const char z[4] = {0, 0, 0, 0};
    // Per-buffer encode-roundtrip self-check budget. The verify decodes the just-encoded buffer into
    // a FULL second copy (di/dv) purely to memcmp — which doubles peak RSS on a big single-material
    // model (Valve Hall STEP->GLB peaked ~3.1 GB, ~1 GB of it these decode copies) and pushed the
    // capped worker OOM. Verify buffers up to the budget (keeps CI/fixtures + typical models checked);
    // skip it for the oversized buffers where the copy is the memory problem. ADA_MESHOPT_VERIFY_MAX_BYTES
    // overrides (0 = never verify, e.g. a memory-pinned worker). Default 256 MiB.
    size_t verify_max = (size_t) 256u << 20;
    if (const char *e = std::getenv("ADA_MESHOPT_VERIFY_MAX_BYTES")) {
        char *end = nullptr;
        unsigned long long v = std::strtoull(e, &end, 10);
        if (end != e)
            verify_max = (size_t) v;
    }
    // Spill each material's compressed regions to a temp file as we go (one material resident at a
    // time) rather than accumulating every material's compressed buffers — peak memory ~one material.
    std::string tmp = path + ".moptmp";
    {
        std::ofstream tf(tmp, std::ios::binary);
        if (!tf)
            return false;
        for (size_t i = 0; i < hdrs.size(); ++i) {
            const MatHeader &h = hdrs[i];
            // Indices FIRST — load, encode, free — before positions are ever materialised, so a single
            // large merged material never holds its positions AND indices resident at once. That doubled
            // peak RSS (Valve Hall STEP->GLB merges to ONE material of ~1.4 GB) and OOM'd the capped
            // worker. Peak is now max(idx, pos) per material, not their sum.
            std::vector<uint32_t> idx;
            provide_idx(i, idx);
            auto cidx = ::ngeom::meshopt_encode_indices(idx.data(), idx.size(), h.vert_count);
            if (cidx.empty())
                return false;
            // verify (small buffers only — the decode is a full second copy) before freeing indices
            if ((size_t) idx.size() * 4 <= verify_max) {
                auto di = ::ngeom::meshopt_decode_indices(cidx.data(), cidx.size(), idx.size(), 4);
                if (di.size() != idx.size() * 4 || std::memcmp(di.data(), idx.data(), di.size()) != 0)
                    return false;
            }
            std::vector<uint32_t>().swap(idx);
            std::vector<float> pos;
            provide_pos(i, pos);
            auto cpos = ::ngeom::meshopt_encode_vertices(pos.data(), h.vert_count, 12);
            if (cpos.empty())
                return false;
            if ((size_t) h.vert_count * 12 <= verify_max) {
                auto dv = ::ngeom::meshopt_decode_vertices(cpos.data(), cpos.size(), h.vert_count, 12);
                if (dv.size() != (size_t) h.vert_count * 12 || std::memcmp(dv.data(), pos.data(), dv.size()) != 0)
                    return false;
            }
            std::vector<float>().swap(pos);
            enc[i].uidx_off = uoff;
            uoff = a4(uoff + h.index_count * 4);
            enc[i].upos_off = uoff;
            uoff = a4(uoff + h.vert_count * 12);
            enc[i].cidx_off = coff;
            enc[i].cidx_len = (uint32_t) cidx.size();
            tf.write(reinterpret_cast<const char *>(cidx.data()), cidx.size());
            tf.write(z, pad4((uint32_t) cidx.size()));
            coff = a4(coff + (uint32_t) cidx.size());
            enc[i].cpos_off = coff;
            enc[i].cpos_len = (uint32_t) cpos.size();
            tf.write(reinterpret_cast<const char *>(cpos.data()), cpos.size());
            tf.write(z, pad4((uint32_t) cpos.size()));
            coff = a4(coff + (uint32_t) cpos.size());
        }
        if (!tf) {
            std::remove(tmp.c_str());
            return false;
        }
    }
    uint32_t comp_len = coff, uncomp_len = uoff;

    std::ostringstream bv, acc, meshes, materials, nodes;
    for (size_t i = 0; i < hdrs.size(); ++i) {
        const MatHeader &h = hdrs[i];
        const Enc &e = enc[i];
        bv << (i ? "," : "") << "{\"buffer\":1,\"byteOffset\":" << e.uidx_off << ",\"byteLength\":" << h.index_count * 4
           << ",\"extensions\":{\"EXT_meshopt_compression\":{\"buffer\":0,\"byteOffset\":" << e.cidx_off
           << ",\"byteLength\":" << e.cidx_len << ",\"byteStride\":4,\"mode\":\"INDICES\",\"count\":" << h.index_count
           << "}}}";
        bv << ",{\"buffer\":1,\"byteOffset\":" << e.upos_off << ",\"byteLength\":" << h.vert_count * 12
           << ",\"extensions\":{\"EXT_meshopt_compression\":{\"buffer\":0,\"byteOffset\":" << e.cpos_off
           << ",\"byteLength\":" << e.cpos_len
           << ",\"byteStride\":12,\"mode\":\"ATTRIBUTES\",\"count\":" << h.vert_count << "}}}";
        acc << (i ? "," : "") << "{\"bufferView\":" << i * 2 << ",\"componentType\":5125,\"count\":" << h.index_count
            << ",\"type\":\"SCALAR\",\"min\":[0],\"max\":[" << h.idx_max << "]}";
        acc << ",{\"bufferView\":" << i * 2 + 1 << ",\"componentType\":5126,\"count\":" << h.vert_count
            << ",\"type\":\"VEC3\",\"min\":[" << fnum(h.lo[0]) << "," << fnum(h.lo[1]) << "," << fnum(h.lo[2])
            << "],\"max\":[" << fnum(h.hi[0]) << "," << fnum(h.hi[1]) << "," << fnum(h.hi[2]) << "]}";
        meshes << (i ? "," : "") << "{\"name\":\"node" << i
               << "\",\"primitives\":[{\"attributes\":{\"POSITION\":" << i * 2 + 1 << "},\"indices\":" << i * 2
               << ",\"mode\":4,\"material\":" << i << "}]}";
        materials << (i ? "," : "") << "{\"pbrMetallicRoughness\":{\"baseColorFactor\":[" << fnum(h.color[0]) << ","
                  << fnum(h.color[1]) << "," << fnum(h.color[2]) << "," << fnum(h.color[3])
                  << "],\"metallicFactor\":0.1,\"roughnessFactor\":0.7},\"doubleSided\":true"
                  << (h.color[3] < 1.0f ? ",\"alphaMode\":\"BLEND\"" : "") << "}";
    }
    nodes << "{\"name\":\"root\"";
    if (!hdrs.empty()) {
        nodes << ",\"children\":[";
        for (size_t i = 0; i < hdrs.size(); ++i)
            nodes << (i ? "," : "") << (i + 1);
        nodes << "]";
    }
    nodes << "}";
    for (size_t i = 0; i < hdrs.size(); ++i)
        nodes << ",{\"name\":\"node" << i << "\",\"mesh\":" << i << "}";

    std::ostringstream js;
    js << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"adacpp-native-glb\"},\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[0],\"extras\":{" << scene_extras << "}}],\"nodes\":[" << nodes.str()
       << "],\"buffers\":[{\"byteLength\":" << comp_len << "},{\"byteLength\":" << uncomp_len
       << ",\"extensions\":{\"EXT_meshopt_compression\":{\"fallback\":true}}}],\"bufferViews\":[" << bv.str()
       << "],\"accessors\":[" << acc.str() << "],\"meshes\":[" << meshes.str() << "],\"materials\":[" << materials.str()
       << "],\"extensionsUsed\":[\"EXT_meshopt_compression\"" << (ada_ext.empty() ? "" : ",\"ADA_EXT_data\"")
       << "],\"extensionsRequired\":[\"EXT_meshopt_compression\"]";
    if (!ada_ext.empty())
        js << ",\"extensions\":{\"ADA_EXT_data\":" << ada_ext << "}";
    js << "}";
    std::string json = js.str();
    json.append(pad4((uint32_t) json.size()), ' ');
    uint32_t json_len = (uint32_t) json.size();
    uint32_t total = 12 + 8 + json_len + 8 + comp_len;

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
    u32(comp_len);
    u32(0x004E4942u);
    { // stream the spilled compressed BIN (buffer 0) into the output through a bounded buffer
        std::ifstream tf(tmp, std::ios::binary);
        std::vector<char> buf(1u << 20);
        while (tf) {
            tf.read(buf.data(), (std::streamsize) buf.size());
            std::streamsize got = tf.gcount();
            if (got > 0)
                out.write(buf.data(), got);
        }
    }
    std::remove(tmp.c_str());
    return (bool) out;
}

} // namespace glb_detail

// Split one tessellated solid into one GlbSolid per DISTINCT face colour, so a per-face-styled solid
// (STEP AP214 OVER_RIDING_STYLED_ITEM) becomes one glTF primitive/material per colour under the existing
// merge-by-colour scheme. Each face's effective colour is its own captured colour, else the solid's base
// colour (`s.color`). Requirements this preserves:
//   * Solids whose faces all share one colour return a SINGLE GlbSolid (no extra primitives) — identical
//     to the pre-split output except the base colour is upgraded to that shared face colour when present.
//   * face_ranges stay valid: each sub-solid's ranges are rebased onto its own compacted index buffer and
//     still tile it contiguously, so the face_ranges_node<m> picking contract holds per material.
//   * All geometry is preserved: the split only runs when the captured face ranges cover the whole index
//     buffer (they do on the capture path); otherwise it degrades to a single primitive (base colour).
inline std::vector<GlbSolid> split_solid_by_face_colour(const GlbSolid &s) {
    if (s.face_ranges.empty())
        return {s}; // no per-face styling captured -> unchanged
    auto eff = [&](const FaceSub &f) -> std::array<float, 4> {
        return f.has_color ? std::array<float, 4>{f.cr, f.cg, f.cb, f.ca} : s.color;
    };
    std::map<std::array<int, 4>, size_t> group_of; // colour key -> group index (first-appearance order)
    std::vector<std::array<float, 4>> group_col;
    size_t covered = 0;
    for (const FaceSub &f : s.face_ranges) {
        covered += f.length;
        auto k = glb_detail::colour_key(eff(f));
        if (group_of.emplace(k, group_of.size()).second)
            group_col.push_back(eff(f));
    }
    // Single distinct colour (or ranges don't fully tile the buffer -> can't safely re-slice): keep one
    // primitive, adopting the shared face colour when the faces all override the base.
    if (group_col.size() <= 1 || covered != s.indices.size()) {
        GlbSolid one = s;
        if (group_col.size() == 1)
            one.color = group_col[0];
        return {one};
    }
    std::vector<GlbSolid> out(group_col.size());
    const size_t nv = s.positions.size() / 3;
    std::vector<std::vector<uint32_t>> remap(group_col.size()); // per group: old vertex -> new (compacted)
    for (size_t g = 0; g < group_col.size(); ++g) {
        out[g].color = group_col[g];
        out[g].transforms = s.transforms;
        out[g].id = s.id;
        out[g].product_name = s.product_name;
        out[g].instance_paths = s.instance_paths;
        remap[g].assign(nv, 0xFFFFFFFFu);
    }
    for (const FaceSub &f : s.face_ranges) {
        size_t g = group_of[glb_detail::colour_key(eff(f))];
        GlbSolid &o = out[g];
        std::vector<uint32_t> &rm = remap[g];
        uint32_t new_start = (uint32_t) o.indices.size();
        uint32_t end = f.start + f.length;
        for (uint32_t k = f.start; k < end && k < s.indices.size(); ++k) {
            uint32_t ov = s.indices[k];
            uint32_t nvid = (ov < rm.size()) ? rm[ov] : 0xFFFFFFFFu;
            if (nvid == 0xFFFFFFFFu) {
                nvid = (uint32_t) (o.positions.size() / 3);
                if (ov < rm.size())
                    rm[ov] = nvid;
                o.positions.push_back(s.positions[3 * ov]);
                o.positions.push_back(s.positions[3 * ov + 1]);
                o.positions.push_back(s.positions[3 * ov + 2]);
            }
            o.indices.push_back(nvid);
        }
        o.face_ranges.push_back({new_start, f.length, f.face_id, f.seq, f.has_color, f.cr, f.cg, f.cb, f.ca});
    }
    return out;
}

// In-RAM merge-by-colour writer (for small/medium inputs + tests). meshopt=true bakes
// EXT_meshopt_compression directly (falls back to raw on any encode failure).
inline bool write_glb(const std::string &path, const std::vector<GlbSolid> &solids, bool meshopt = false) {
    using namespace glb_detail;
    struct Buf {
        std::array<float, 4> color;
        std::vector<float> pos;
        std::vector<uint32_t> idx;
        uint32_t idx_max = 0;
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        std::vector<PartRange> parts; // per-solid index ranges (picking)
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
            uint32_t part_start = (uint32_t) m.idx.size();
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
            // One pickable leaf per placement (1:1 with the Python scene builder). Face regions are
            // relative to the solid's index buffer, identical for every placement, so each leaf carries
            // the same list (the viewer re-bases each against that leaf's own draw-range start).
            m.parts.push_back({instance_leaf_name(s, t), part_start, (uint32_t) m.idx.size() - part_start,
                               instance_parent_path(s, t), s.face_ranges});
        }
    }
    std::vector<MatHeader> hdrs;
    std::vector<const Buf *> bufs;
    std::vector<std::vector<PartRange>> per_mat;
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
            per_mat.push_back(m.parts);
        }
    std::string extras = build_scene_extras(per_mat);
    if (meshopt && glb_write_framed_meshopt(
                       path, hdrs, [&](size_t i, std::vector<uint32_t> &idx) { idx = bufs[i]->idx; },
                       [&](size_t i, std::vector<float> &pos) { pos = bufs[i]->pos; }, extras, ""))
        return true;
    static const char z[4] = {0, 0, 0, 0};
    return glb_write_framed(path, hdrs, extras, "", [&](std::ofstream &out) {
        for (const Buf *b : bufs) {
            uint32_t ib = (uint32_t) b->idx.size() * 4, pb = (uint32_t) b->pos.size() * 4;
            out.write(reinterpret_cast<const char *>(b->idx.data()), ib);
            out.write(z, pad4(ib));
            out.write(reinterpret_cast<const char *>(b->pos.data()), pb);
            out.write(z, pad4(pb));
        }
    });
}

// Per-material buffer/spill threshold (bytes, pos+idx combined). Below it a material stays entirely in
// RAM and never touches the disk — most conversions (small/medium models) then do ZERO spill writes,
// which is the dominant (~70%) source of the audit's nvme write pressure. A material that grows past it
// (a huge solid / dense model) spills to disk and streams from there, keeping peak RAM bounded. Env
// ADA_GLB_SPILL_THRESHOLD_MB (default 96); 0 forces the always-spill behaviour.
inline size_t glb_spill_threshold_bytes() {
    if (const char *e = std::getenv("ADA_GLB_SPILL_THRESHOLD_MB")) {
        char *end = nullptr;
        long v = std::strtol(e, &end, 10);
        if (end != e && v >= 0)
            return (size_t) v * 1024 * 1024;
    }
    return (size_t) 96 * 1024 * 1024;
}

// Disk-spilling writer: one "lane" (one producer / worker). add() bakes + appends each material's
// verts/indices; small materials stay in an in-RAM buffer, large ones spill to per-material temp files.
// The merge (write_glb_merged) reads each material from its buffer or file. Keeps only per-material
// metadata + (for small models) the raw bytes in RAM, never the merged buffers.
class GlbSpillWriter {
public:
    struct MatLane {
        std::array<float, 4> color{0.5f, 0.5f, 0.5f, 1.0f};
        std::string pos_path, idx_path; // set only once this material has spilled to disk
        std::ofstream pos, idx;
        std::string pos_buf, idx_buf; // in-RAM bytes while !spilled (then flushed to file + cleared)
        bool spilled = false;
        uint32_t vert_count = 0, index_count = 0, idx_max = 0;
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        struct Part {
            std::string id;
            uint32_t index_count;
            std::vector<std::pair<int, std::string>> path;
            std::vector<FaceSub> faces; // per-face regions relative to this part; empty unless captured
        };
        std::vector<Part> parts; // per-solid (id, index_count, assembly path) in add order

        // Yield this material's index bytes as uint32 chunks — from the RAM buffer or streamed from the
        // spill file — so the merge readers don't care which. f(const uint32_t*, count).
        template <class F> void for_idx(F f) const {
            if (!spilled) {
                if (!idx_buf.empty())
                    f(reinterpret_cast<const uint32_t *>(idx_buf.data()), idx_buf.size() / 4);
                return;
            }
            std::ifstream xf(idx_path, std::ios::binary);
            std::vector<uint32_t> rb(1u << 16);
            while (xf) {
                xf.read(reinterpret_cast<char *>(rb.data()), (std::streamsize) (rb.size() * 4));
                size_t got = (size_t) (xf.gcount() / 4);
                if (got)
                    f(rb.data(), got);
            }
        }
        // Yield this material's position bytes as raw chunks. f(const char*, nbytes).
        template <class F> void for_pos(F f) const {
            if (!spilled) {
                if (!pos_buf.empty())
                    f(pos_buf.data(), pos_buf.size());
                return;
            }
            std::ifstream pf(pos_path, std::ios::binary);
            std::vector<char> b(1u << 16);
            while (pf) {
                pf.read(b.data(), (std::streamsize) b.size());
                std::streamsize got = pf.gcount();
                if (got > 0)
                    f(b.data(), (size_t) got);
            }
        }
    };

    GlbSpillWriter(std::string dir, int lane)
        : dir_(std::move(dir)), lane_(lane), threshold_(glb_spill_threshold_bytes()) {}
    ~GlbSpillWriter() {
        remove_files();
    }

    void add(const GlbSolid &s) {
        using namespace glb_detail;
        if (s.indices.empty() || s.positions.size() < 3)
            return;
        std::array<int, 4> key = colour_key(s.color);
        MatLane &m = mats_[key];
        m.color = s.color;
        size_t ninst = s.transforms.empty() ? 1 : s.transforms.size();
        for (size_t t = 0; t < ninst; ++t) {
            const float *M = s.transforms.empty() ? IDENT : s.transforms[t].data();
            uint32_t base = m.vert_count;
            uint32_t inst_before = m.index_count;
            for (size_t i = 0; i + 2 < s.positions.size(); i += 3) {
                float o[3];
                xform(M, s.positions[i], s.positions[i + 1], s.positions[i + 2], o[0], o[1], o[2]);
                append(m, key, /*is_pos=*/true, reinterpret_cast<const char *>(o), 12);
                for (int k = 0; k < 3; ++k) {
                    m.lo[k] = std::min(m.lo[k], o[k]);
                    m.hi[k] = std::max(m.hi[k], o[k]);
                }
                ++m.vert_count;
            }
            for (uint32_t ix : s.indices) {
                uint32_t v = base + ix;
                append(m, key, /*is_pos=*/false, reinterpret_cast<const char *>(&v), 4);
                m.idx_max = std::max(m.idx_max, v);
                ++m.index_count;
            }
            // One pickable leaf per placement (1:1 with the Python scene builder). Face regions are
            // relative to the solid's index buffer (same for every placement); each leaf keeps its own copy.
            m.parts.push_back(
                {instance_leaf_name(s, t), m.index_count - inst_before, instance_parent_path(s, t), s.face_ranges});
        }
    }
    void flush() {
        for (auto &[k, m] : mats_)
            if (m.spilled) {
                m.pos.flush();
                m.idx.flush();
            }
    }
    const std::map<std::array<int, 4>, MatLane> &mats() const {
        return mats_;
    }

private:
    // Append bytes to a material's pos/idx — into the RAM buffer, or the spill file once spilled. The
    // threshold bounds the LANE's total in-RAM bytes (across all its materials), not one material's, so
    // a many-material model can't pile up N thresholds of RAM on the memory-capped worker: crossing it
    // spills the material currently being appended (freeing its buffer) and, if still over, the rest.
    void append(MatLane &m, const std::array<int, 4> &key, bool is_pos, const char *p, size_t n) {
        if (m.spilled) {
            (is_pos ? m.pos : m.idx).write(p, (std::streamsize) n);
            return;
        }
        (is_pos ? m.pos_buf : m.idx_buf).append(p, n);
        buffered_ += n;
        if (threshold_ && buffered_ > threshold_) {
            spill(m, key); // the hot material first
            for (auto &[k2, m2] : mats_) {
                if (buffered_ <= threshold_)
                    break;
                if (!m2.spilled && !(m2.pos_buf.empty() && m2.idx_buf.empty()))
                    spill(m2, k2);
            }
        }
    }
    // Move a material from RAM to disk: open its files, write the accumulated buffers, free the RAM.
    void spill(MatLane &m, const std::array<int, 4> &key) {
        char tag[64];
        std::snprintf(tag, sizeof tag, "/glb_l%d_%d_%d_%d_%d", lane_, key[0], key[1], key[2], key[3]);
        m.pos_path = dir_ + tag + ".pos";
        m.idx_path = dir_ + tag + ".idx";
        m.pos.open(m.pos_path, std::ios::binary);
        m.idx.open(m.idx_path, std::ios::binary);
        m.pos.write(m.pos_buf.data(), (std::streamsize) m.pos_buf.size());
        m.idx.write(m.idx_buf.data(), (std::streamsize) m.idx_buf.size());
        buffered_ -= (m.pos_buf.size() + m.idx_buf.size());
        std::string().swap(m.pos_buf);
        std::string().swap(m.idx_buf);
        m.spilled = true;
    }
    void remove_files() {
        for (auto &[k, m] : mats_) {
            if (!m.spilled)
                continue;
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
    size_t threshold_;
    size_t buffered_ = 0; // total in-RAM buffer bytes across all not-yet-spilled materials in this lane
    std::map<std::array<int, 4>, MatLane> mats_;
};

// Merge N lanes (one per worker) into a GLB. Per material: positions are concatenated across lanes;
// each lane's indices are re-offset by its cumulative vertex base in that material. Streams the lane
// files into the BIN chunk — never materializes the merged buffers.
inline bool write_glb_merged(const std::string &path, const std::vector<GlbSpillWriter *> &lanes,
                             const std::string &ada_ext = "", bool meshopt = false) {
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
    // Per-solid draw ranges: index offset accumulates across lanes (same order as the BIN concat),
    // so each solid's [start,length] addresses the material's merged index buffer (picking).
    std::vector<std::vector<PartRange>> per_mat;
    per_mat.reserve(keys.size());
    for (const std::array<int, 4> &key : keys) {
        std::vector<PartRange> parts;
        uint32_t off = 0;
        for (GlbSpillWriter *l : lanes) {
            auto it = l->mats().find(key);
            if (it == l->mats().end())
                continue;
            for (const auto &part : it->second.parts) {
                parts.push_back({part.id, off, part.index_count, part.path, part.faces});
                off += part.index_count;
            }
        }
        per_mat.push_back(std::move(parts));
    }
    std::string extras = build_scene_extras(per_mat);
    // meshopt: read one material at a time from the spill lanes (positions concatenated, indices
    // re-offset by the cumulative vertex base across lanes — exactly the raw merge, into vectors),
    // encode + bake EXT_meshopt_compression. Falls through to the raw writer on any failure.
    // Two providers (indices, positions) read one attribute at a time from the spill lanes so the
    // meshopt writer never holds a material's positions AND indices resident together — see the
    // idx-first ordering in glb_write_framed_meshopt. Indices are re-offset by the cumulative vertex
    // base across lanes (the raw-merge order); positions are a plain concat.
    auto provide_idx = [&](size_t mi, std::vector<uint32_t> &idx) {
        const std::array<int, 4> &key = keys[mi];
        uint32_t vbase = 0;
        for (GlbSpillWriter *l : lanes) {
            auto it = l->mats().find(key);
            if (it == l->mats().end())
                continue;
            const auto &m = it->second;
            m.for_idx([&](const uint32_t *rb, size_t got) {
                for (size_t k = 0; k < got; ++k)
                    idx.push_back(rb[k] + vbase);
            });
            vbase += m.vert_count;
        }
    };
    auto provide_pos = [&](size_t mi, std::vector<float> &pos) {
        const std::array<int, 4> &key = keys[mi];
        for (GlbSpillWriter *l : lanes) {
            auto it = l->mats().find(key);
            if (it == l->mats().end())
                continue;
            const auto &m = it->second;
            size_t old = pos.size();
            pos.resize(old + (size_t) m.vert_count * 3);
            char *dst = reinterpret_cast<char *>(pos.data() + old);
            m.for_pos([&](const char *b, size_t n) {
                std::memcpy(dst, b, n);
                dst += n;
            });
        }
    };
    if (meshopt && glb_write_framed_meshopt(path, hdrs, provide_idx, provide_pos, extras, ada_ext))
        return true;
    static const char z[4] = {0, 0, 0, 0};
    return glb_write_framed(path, hdrs, extras, ada_ext, [&](std::ofstream &out) {
        for (const std::array<int, 4> &key : keys) {
            // indices: each lane re-offset by the cumulative vertex base, padded once at the end
            uint32_t base = 0, idx_bytes = 0;
            for (GlbSpillWriter *l : lanes) {
                auto it = l->mats().find(key);
                if (it == l->mats().end())
                    continue;
                const auto &m = it->second;
                // Re-offset in bulk (a chunk at a time) instead of per-uint32 read/add/write — there are
                // tens of millions of indices, so per-element overhead dominates. Chunks come from the
                // material's RAM buffer or its spill file (for_idx hides which).
                std::vector<uint32_t> ibuf;
                m.for_idx([&](const uint32_t *rb, size_t got) {
                    ibuf.assign(rb, rb + got);
                    for (size_t i = 0; i < got; ++i)
                        ibuf[i] += base;
                    out.write(reinterpret_cast<const char *>(ibuf.data()), (std::streamsize) (got * 4));
                });
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
                m.for_pos([&](const char *b, size_t n) { out.write(b, (std::streamsize) n); });
                pos_bytes += m.vert_count * 12;
            }
            out.write(z, pad4(pos_bytes));
        }
    });
}

} // namespace adacpp::glb
