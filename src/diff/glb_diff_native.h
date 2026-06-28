#pragma once
// Streaming GLB summariser for the diff core. NO tinygltf, NO whole-model load: parse only the small
// glTF JSON (nlohmann), mmap the BIN chunk (file-backed — the multi-GB GLB never enters RSS), and
// decode ONE mesh node at a time (the native GLB is merged-by-material → one node per material), so
// peak RSS is the largest single material chunk + the KB-scale summary table, independent of file
// size. EXT_meshopt_compression is decoded per-bufferView. The buffer-pointer core
// (`summarize_glb_buf`) is portable (nlohmann + meshopt only) so the wasm target reuses it with a
// fetched/streamed buffer instead of an mmap.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef __EMSCRIPTEN__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <json.hpp>
#include <meshoptimizer.h> // raw decode-into-buffer (decode straight into a disk-backed mmap)

#include "../geom/neutral/ngeom_meshopt.h" // EXT_meshopt_compression decode (RAM fallback / wasm)
#include "glb_diff.h"

namespace adacpp {
namespace gdiff {

using njson = nlohmann::json;

// Only spill a meshopt-decoded bufferView to disk when it exceeds this — small/common nodes decode in
// RAM (fast); a larger (>512 MiB) material chunk goes disk-backed so it can't blow RSS. Tuned so the
// disk path's msync+page-fault overhead is only paid when RAM decode would actually be dangerous.
constexpr size_t kDiskDecodeThreshold = 512ull << 20; // 512 MiB

inline uint32_t pad4(uint32_t n) {
    return (4 - (n & 3u)) & 3u;
}

// hash of {type, section.name, material.name, thickness} for the MODIFIED test.
inline uint64_t meta_signature(const njson &m) {
    if (!m.is_object())
        return 0;
    std::string s;
    auto add = [&](const char *k, const njson &v) {
        s += k;
        s += '=';
        if (v.is_string())
            s += v.get<std::string>();
        else if (v.is_number())
            s += std::to_string(v.get<double>());
        else if (v.is_object() && v.contains("name") && v["name"].is_string())
            s += v["name"].get<std::string>();
        s += ';';
    };
    for (const char *k : {"type", "section", "material", "thickness"})
        if (m.contains(k))
            add(k, m[k]);
    return std::hash<std::string>{}(s);
}

// A readable byte source + element stride. ``owner`` keeps the backing alive: a disk-backed mmap
// (DiskBuf) or a RAM vector for a meshopt-decoded bufferView, or null when ``ptr`` points straight
// into the input mmap (raw bufferView).
struct ByteView {
    const unsigned char *ptr = nullptr;
    size_t stride = 0;
    std::shared_ptr<void> owner;
};

#ifndef __EMSCRIPTEN__
// A decoded bufferView living in an unlinked temp file, mmap'd. The decode writes the whole buffer,
// but it's FILE-backed so the kernel writes dirty pages back to disk (RSS stays bounded by the dirty
// limit, not the buffer size) — this is what lets a single >2 GB material node be summarised without
// holding it in RAM. Reads during folding page-fault from disk; pages are reclaimable under pressure.
struct DiskBuf {
    void *m = nullptr;
    size_t size = 0;
    int fd = -1;
    ~DiskBuf() {
        if (m && m != MAP_FAILED)
            ::munmap(m, size);
        if (fd >= 0)
            ::close(fd);
    }
};

inline std::shared_ptr<DiskBuf> decode_to_disk(const unsigned char *enc, size_t enc_size, size_t count,
                                               size_t stride, bool is_index) {
    size_t dsize = count * stride;
    if (dsize == 0)
        return nullptr;
    char tmpl[] = "/tmp/adacpp_diffdec_XXXXXX";
    int fd = ::mkstemp(tmpl);
    if (fd < 0)
        return nullptr;
    ::unlink(tmpl); // disk space freed on close; no path left behind
    auto db = std::make_shared<DiskBuf>();
    db->fd = fd;
    if (::ftruncate(fd, (off_t) dsize) != 0)
        return nullptr;
    void *m = ::mmap(nullptr, dsize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED)
        return nullptr;
    db->m = m;
    db->size = dsize;
    int rc = is_index ? ::meshopt_decodeIndexSequence(m, count, stride, enc, enc_size)
                      : ::meshopt_decodeVertexBuffer(m, count, stride, enc, enc_size);
    if (rc != 0)
        return nullptr;
    ::msync(m, dsize, MS_SYNC);       // force decoded bytes out to the temp file
    ::madvise(m, dsize, MADV_DONTNEED); // drop them from RSS; folding re-faults what it touches
    return db;
}
#endif

// Resolve an accessor to a ByteView. meshopt bufferViews are decoded — disk-backed when ``disk`` (so
// a huge node never sits in RAM), else into a RAM vector (small models / wasm). Raw bufferViews point
// straight into the mmap'd BIN.
inline ByteView resolve_accessor(const njson &acc, const njson &bvs, const unsigned char *bin, size_t elem,
                                 bool disk) {
    const njson &bv = bvs[acc.at("bufferView").get<int>()];
    size_t acc_off = acc.value("byteOffset", (size_t) 0);
    auto ext = bv.find("extensions");
    if (ext != bv.end() && ext->contains("EXT_meshopt_compression")) {
        const njson &e = (*ext)["EXT_meshopt_compression"];
        size_t bo = e.at("byteOffset").get<size_t>();
        size_t bl = e.at("byteLength").get<size_t>();
        size_t cnt = e.at("count").get<size_t>();
        size_t stride = e.at("byteStride").get<size_t>();
        bool is_index = e.value("mode", std::string("ATTRIBUTES")) != "ATTRIBUTES";
#ifndef __EMSCRIPTEN__
        // Disk-spill only the huge nodes; everything else decodes in RAM (no msync/page-fault cost).
        if (disk && cnt * stride > kDiskDecodeThreshold) {
            if (auto db = decode_to_disk(bin + bo, bl, cnt, stride, is_index))
                return {(const unsigned char *) db->m + acc_off, stride, db};
            // decode_to_disk failed (no /tmp etc.) -> fall through to RAM
        }
#endif
        auto vec = std::make_shared<std::vector<unsigned char>>(
            is_index ? ::ngeom::meshopt_decode_indices(bin + bo, bl, cnt, stride)
                     : ::ngeom::meshopt_decode_vertices(bin + bo, bl, cnt, stride));
        return {vec->data() + acc_off, stride, vec};
    }
    size_t stride = (bv.contains("byteStride") && !bv["byteStride"].is_null()) ? bv["byteStride"].get<size_t>() : elem;
    return {bin + bv.value("byteOffset", (size_t) 0) + acc_off, stride, nullptr};
}

// Summarise a GLB from an in-memory buffer (portable: native mmap or wasm fetched bytes). Folds one
// ElementSummary per draw-range; if ``keep`` is set, also gathers world-tris for those node_ids.
inline std::vector<ElementSummary> summarize_glb_buf(const unsigned char *data, size_t size,
                                                     const std::unordered_set<std::string> *keep = nullptr,
                                                     std::vector<float> *keep_tris = nullptr, bool disk = false) {
    std::vector<ElementSummary> out;
    if (size < 20 || std::memcmp(data, "glTF", 4) != 0)
        return out;
    uint32_t json_len;
    std::memcpy(&json_len, data + 12, 4);
    if ((size_t) 20 + json_len + 8 > size)
        return out;
    const unsigned char *bin = data + 20 + json_len + 8; // skip the BIN chunk's 8-byte header

    njson j;
    try {
        const char *jbeg = (const char *) (data + 20);
        j = njson::parse(jbeg, jbeg + json_len);
    } catch (...) {
        return out;
    }
    if (!j.contains("scenes") || !j.contains("bufferViews") || !j.contains("accessors") || !j.contains("nodes") ||
        !j.contains("meshes"))
        return out;

    const njson &accessors = j["accessors"];
    const njson &bufferViews = j["bufferViews"];
    const njson &nodes = j["nodes"];
    const njson &meshes = j["meshes"];
    const njson &scene0 = j["scenes"][j.value("scene", 0)];
    if (!scene0.contains("extras"))
        return out;
    const njson &extras = scene0["extras"];

    // name_of from id_hierarchy[node_id] = [name, parent]
    std::unordered_map<std::string, std::string> name_of;
    if (extras.contains("id_hierarchy"))
        for (auto it = extras["id_hierarchy"].begin(); it != extras["id_hierarchy"].end(); ++it)
            if (it.value().is_array() && !it.value().empty() && it.value()[0].is_string())
                name_of[it.key()] = it.value()[0].get<std::string>();

    // guid / etype / meta from extensions.ADA_EXT_data.{design,simulation}_objects
    std::unordered_map<std::string, std::string> guid_of, etype_of;
    std::unordered_map<std::string, uint64_t> meta_of;
    if (j.contains("extensions") && j["extensions"].contains("ADA_EXT_data")) {
        const njson &ext = j["extensions"]["ADA_EXT_data"];
        for (const char *key : {"design_objects", "simulation_objects"}) {
            if (!ext.contains(key))
                continue;
            for (const njson &obj : ext[key]) {
                if (obj.contains("object_guids"))
                    for (auto it = obj["object_guids"].begin(); it != obj["object_guids"].end(); ++it)
                        if (it.value().is_string())
                            guid_of[it.key()] = it.value().get<std::string>();
                if (obj.contains("object_metadata"))
                    for (auto it = obj["object_metadata"].begin(); it != obj["object_metadata"].end(); ++it) {
                        meta_of[it.key()] = meta_signature(it.value());
                        if (it.value().is_object() && it.value().contains("type") && it.value()["type"].is_string())
                            etype_of[it.key()] = it.value()["type"].get<std::string>();
                    }
            }
        }
    }

    // node name -> index, to resolve draw_ranges_<nodeName>
    std::unordered_map<std::string, int> node_idx;
    for (size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i].contains("name") && nodes[i]["name"].is_string())
            node_idx[nodes[i]["name"].get<std::string>()] = (int) i;

    // One node (= one material chunk) decoded at a time, folded, then freed.
    for (auto kit = extras.begin(); kit != extras.end(); ++kit) {
        const std::string &key = kit.key();
        if (key.rfind("draw_ranges_", 0) != 0)
            continue;
        auto nit = node_idx.find(key.substr(std::string("draw_ranges_").size()));
        if (nit == node_idx.end())
            continue;
        const njson &node = nodes[nit->second];
        if (!node.contains("mesh"))
            continue;
        const njson &prim = meshes[node["mesh"].get<int>()]["primitives"][0];
        if (!prim.contains("indices") || !prim["attributes"].contains("POSITION"))
            continue;

        const njson &iacc = accessors[prim["indices"].get<int>()];
        const size_t isz = (iacc.value("componentType", 5125) == 5125) ? 4 : 2;
        ByteView posv = resolve_accessor(accessors[prim["attributes"]["POSITION"].get<int>()], bufferViews, bin, 12,
                                         disk);
        ByteView idxv = resolve_accessor(iacc, bufferViews, bin, isz, disk);
        const size_t idx_count = iacc.at("count").get<size_t>();

        auto read_idx = [&](size_t i) -> uint32_t {
            if (isz == 4) {
                uint32_t v;
                std::memcpy(&v, idxv.ptr + i * idxv.stride, 4);
                return v;
            }
            uint16_t v;
            std::memcpy(&v, idxv.ptr + i * idxv.stride, 2);
            return v;
        };
        auto read_pos = [&](uint32_t vi, float o[3]) { std::memcpy(o, posv.ptr + (size_t) vi * posv.stride, 12); };

        for (auto rit = kit.value().begin(); rit != kit.value().end(); ++rit) {
            const njson &rng = rit.value();
            if (!rng.is_array() || rng.size() < 2)
                continue;
            size_t start = rng[0].get<size_t>(), count = rng[1].get<size_t>();
            if (count == 0 || start + count > idx_count)
                continue;

            ElementSummary s;
            s.node_id = rit.key();
            auto nameit = name_of.find(s.node_id);
            s.name = (nameit != name_of.end()) ? nameit->second : s.node_id;
            auto g = guid_of.find(s.name);
            if (g != guid_of.end())
                s.guid = g->second;
            auto e = etype_of.find(s.name);
            if (e != etype_of.end())
                s.etype = e->second;
            auto mt = meta_of.find(s.name);
            if (mt != meta_of.end())
                s.meta_sig = mt->second;

            const size_t ntri = count / 3;
            s.tri_count = (uint32_t) ntri;
            double cx = 0, cy = 0, cz = 0, area = 0;
            std::array<double, 3> mn{1e300, 1e300, 1e300}, mx{-1e300, -1e300, -1e300};
            const bool want = keep && keep_tris && keep->count(s.node_id);
            for (size_t t = 0; t < ntri; ++t) {
                float v[3][3];
                for (int k = 0; k < 3; ++k)
                    read_pos(read_idx(start + 3 * t + k), v[k]);
                for (int k = 0; k < 3; ++k) {
                    cx += v[k][0];
                    cy += v[k][1];
                    cz += v[k][2];
                    for (int d = 0; d < 3; ++d) {
                        mn[d] = std::min(mn[d], (double) v[k][d]);
                        mx[d] = std::max(mx[d], (double) v[k][d]);
                    }
                    if (want) {
                        keep_tris->push_back(v[k][0]);
                        keep_tris->push_back(v[k][1]);
                        keep_tris->push_back(v[k][2]);
                    }
                }
                double ux = v[1][0] - v[0][0], uy = v[1][1] - v[0][1], uz = v[1][2] - v[0][2];
                double wx = v[2][0] - v[0][0], wy = v[2][1] - v[0][1], wz = v[2][2] - v[0][2];
                double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
                area += 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
            }
            if (ntri) {
                const double nv = (double) (ntri * 3);
                s.centroid = {cx / nv, cy / nv, cz / nv};
                s.bbox_min = mn;
                s.bbox_max = mx;
                s.area = area;
            }
            out.push_back(std::move(s));
        }
        // Drop the BIN pages touched for this node from RSS before the next one, so resident memory
        // stays bounded by the LARGEST single node — not the (multi-GB) file. Decoded temps free as
        // pos_owned/idx_owned leave scope. (no-op on wasm, where ``data`` is a JS buffer not an mmap)
#ifndef __EMSCRIPTEN__
        ::madvise(const_cast<unsigned char *>(data), size, MADV_DONTNEED);
#endif
    }
    return out;
}

#ifndef __EMSCRIPTEN__
// Native wrapper: mmap the GLB so the file stays page-cache/file-backed (out of the RSS the worker
// cap measures) and the encoded bytes are dropped after the scan.
inline std::vector<ElementSummary> summarize_glb_file(const std::string &path,
                                                      const std::unordered_set<std::string> *keep = nullptr,
                                                      std::vector<float> *keep_tris = nullptr) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0)
        return {};
    struct stat st;
    if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
        ::close(fd);
        return {};
    }
    size_t sz = (size_t) st.st_size;
    void *m = ::mmap(nullptr, sz, PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd);
    if (m == MAP_FAILED)
        return {};
    ::madvise(m, sz, MADV_SEQUENTIAL);
    auto out = summarize_glb_buf((const unsigned char *) m, sz, keep, keep_tris, /*disk=*/true);
    ::madvise(m, sz, MADV_DONTNEED);
    ::munmap(m, sz);
    return out;
}
#endif // __EMSCRIPTEN__

// Minimal single-colour GLB from a flat triangle-soup (9 floats per tri, unshared verts). For the
// removed overlay. rgba packed 0xRRGGBBAA.
inline std::string write_overlay_glb(const std::vector<float> &pos, uint32_t rgba) {
    const uint32_t nverts = (uint32_t) (pos.size() / 3);
    std::vector<uint32_t> idx(nverts);
    for (uint32_t i = 0; i < nverts; ++i)
        idx[i] = i;

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

    const uint32_t bin_len = (uint32_t) (pos_bytes + idx_bytes);
    const uint32_t total = 12 + 8 + (uint32_t) json.size() + 8 + bin_len;
    std::string out;
    out.reserve(total);
    auto u32 = [&](uint32_t v) { out.append((const char *) &v, 4); };
    out.append("glTF", 4);
    u32(2);
    u32(total);
    u32((uint32_t) json.size());
    u32(0x4E4F534Au); // "JSON"
    out.append(json);
    u32(bin_len);
    u32(0x004E4942u); // "BIN\0"
    if (!pos.empty())
        out.append((const char *) pos.data(), pos_bytes);
    if (!idx.empty())
        out.append((const char *) idx.data(), idx_bytes);
    return out;
}

} // namespace gdiff
} // namespace adacpp
