#pragma once
// Streaming-friendly GLB model diff core (OCC-free). Drop-in producer for adapy's diff @utility:
// compute a per-element summary table for two GLBs, match them, and emit (a) a colour-op list keyed
// by the GLB draw-range node_id (== frontend rangeId) and (b) the node_ids whose geometry is unique
// to the ref (for an overlay GLB). Peak memory is one model at a time (never both), and only the
// per-element SUMMARY (centroid/area/bbox), not whole geometry, is retained for matching.
//
// Portability: the GLB -> arrays parse (`parse_glb`) is the only native-specific part (tinygltf,
// non-wasm build); the diff logic (`summarize` / `diff_summaries` / `collect_overlay_tris`) is plain
// portable C++ so the wasm target can swap in a tinygltf-free reader + feed the same arrays.
//
// v1 targets UNCOMPRESSED GLBs — the diff contract (adapy diff.py reads raw accessors; the worker
// applies meshopt compression only AFTER conversion). EXT_meshopt_compression decode is a follow-up.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace adacpp {
namespace gdiff {

// ── per-element summary (the only thing kept for matching) ───────────────────────────────────────
struct ElementSummary {
    std::string node_id; // GLB draw-range key == frontend rangeId
    std::string name;    // id_hierarchy[node_id][0]
    std::string guid;    // ADA_EXT_data object_guids[name] (may be empty)
    std::string etype;   // object_metadata[name].type (may be empty)
    std::array<double, 3> centroid{0, 0, 0};
    std::array<double, 3> bbox_min{0, 0, 0};
    std::array<double, 3> bbox_max{0, 0, 0};
    double area = 0.0;     // summed triangle area
    uint32_t tri_count = 0;
    uint64_t meta_sig = 0; // hash of {section, material, thickness} for the MODIFIED test
};

enum class Status : uint8_t { Unchanged = 0, Added = 1, Removed = 2, Modified = 3 };
enum class Mode : uint8_t { ByName, ByGuid, ByCentroid, ByProperty, NameThenCentroid };

struct DiffOp {
    std::string node_id; // SCENE node_id
    Status status;
};
struct DiffResult {
    std::vector<DiffOp> ops;                    // added / modified / unchanged (scene node_ids)
    std::vector<std::string> removed_node_ids;  // REF-only node_ids (overlay extraction)
    std::vector<std::string> added_node_ids;    // SCENE-only node_ids (optional standalone overlay)
    uint32_t n_added = 0, n_removed = 0, n_modified = 0, n_unchanged = 0;
};

// ── parsed GLB geometry + identity (filled by the native tinygltf parser below; the wasm target
// supplies an equivalent struct from its own reader) ─────────────────────────────────────────────
struct ParsedElement {
    std::string node_id, name, guid, etype;
    uint64_t meta_sig = 0;
    // the element's triangle vertices in world space (3 floats * 3 verts per tri), already gathered
    // from its draw range — used for summary + overlay. Held transiently per model.
    std::vector<float> tris; // size = 9 * tri_count
};

// ── portable diff logic ──────────────────────────────────────────────────────────────────────────
inline ElementSummary summarize_one(const ParsedElement &e) {
    ElementSummary s;
    s.node_id = e.node_id;
    s.name = e.name;
    s.guid = e.guid;
    s.etype = e.etype;
    s.meta_sig = e.meta_sig;
    const size_t ntri = e.tris.size() / 9;
    s.tri_count = (uint32_t) ntri;
    if (ntri == 0)
        return s;
    double cx = 0, cy = 0, cz = 0, area = 0;
    std::array<double, 3> mn{1e300, 1e300, 1e300}, mx{-1e300, -1e300, -1e300};
    const float *p = e.tris.data();
    for (size_t t = 0; t < ntri; ++t) {
        const float *a = p + 9 * t, *b = a + 3, *c = a + 6;
        for (int v = 0; v < 3; ++v) {
            const float *q = a + 3 * v;
            for (int k = 0; k < 3; ++k) {
                cx += (k == 0) ? q[0] : 0;
                cy += (k == 1) ? q[1] : 0;
                cz += (k == 2) ? q[2] : 0;
                mn[k] = std::min(mn[k], (double) q[k]);
                mx[k] = std::max(mx[k], (double) q[k]);
            }
        }
        // triangle area = 0.5 * |(b-a) x (c-a)|
        double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
        double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        area += 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
    }
    const double nv = (double) (ntri * 3);
    s.centroid = {cx / nv, cy / nv, cz / nv};
    s.bbox_min = mn;
    s.bbox_max = mx;
    s.area = area;
    return s;
}

inline std::string _centroid_key(const std::array<double, 3> &c, double tol) {
    if (tol <= 0)
        tol = 1e-6;
    auto q = [&](double v) { return (long long) std::llround(v / tol); };
    return std::to_string(q(c[0])) + "," + std::to_string(q(c[1])) + "," + std::to_string(q(c[2]));
}

inline std::string _match_key(const ElementSummary &e, Mode mode, double tol) {
    switch (mode) {
    case Mode::ByGuid:
        return e.guid.empty() ? ("n:" + e.name) : ("g:" + e.guid);
    case Mode::ByCentroid:
        return _centroid_key(e.centroid, tol);
    case Mode::ByName:
    case Mode::ByProperty:
    case Mode::NameThenCentroid:
    default:
        return e.name;
    }
}

// Multi-match: group ref by key, consume one ref element per matching scene element (so N instances
// of a shared product name match N ref instances). Returns a per-key bucket of unused ref indices.
struct _Buckets {
    std::unordered_map<std::string, std::vector<size_t>> by_key;
    std::unordered_map<std::string, size_t> cursor; // next unused index within a key's vector

    explicit _Buckets(const std::vector<ElementSummary> &ref, Mode mode, double tol) {
        for (size_t i = 0; i < ref.size(); ++i)
            by_key[_match_key(ref[i], mode, tol)].push_back(i);
    }
    // pop the next unused ref index for ``key``, or SIZE_MAX if none left.
    size_t take(const std::string &key) {
        auto it = by_key.find(key);
        if (it == by_key.end())
            return SIZE_MAX;
        size_t &cur = cursor[key];
        return cur < it->second.size() ? it->second[cur++] : SIZE_MAX;
    }
};

// Match scene vs ref into a DiffResult. NameThenCentroid: name first, then centroid for the leftovers.
inline DiffResult diff_summaries(const std::vector<ElementSummary> &scene,
                                 const std::vector<ElementSummary> &ref, Mode mode, double tol) {
    DiffResult r;
    const bool check_props = (mode == Mode::ByProperty);
    std::vector<char> ref_used(ref.size(), 0);
    std::vector<size_t> scene_unmatched;

    _Buckets ref_buckets(ref, mode, tol);
    for (size_t si = 0; si < scene.size(); ++si) {
        size_t ri = ref_buckets.take(_match_key(scene[si], mode, tol));
        if (ri == SIZE_MAX) {
            if (mode == Mode::NameThenCentroid) {
                scene_unmatched.push_back(si);
                continue;
            }
            r.ops.push_back({scene[si].node_id, Status::Added});
            r.added_node_ids.push_back(scene[si].node_id);
            ++r.n_added;
        } else {
            ref_used[ri] = 1;
            if (check_props && scene[si].meta_sig != ref[ri].meta_sig) {
                r.ops.push_back({scene[si].node_id, Status::Modified});
                ++r.n_modified;
            } else {
                r.ops.push_back({scene[si].node_id, Status::Unchanged});
                ++r.n_unchanged;
            }
        }
    }

    if (mode == Mode::NameThenCentroid && !scene_unmatched.empty()) {
        // second pass: centroid-match the leftovers against still-unused ref elements
        std::vector<ElementSummary> ref_left;
        std::vector<size_t> ref_left_idx;
        for (size_t i = 0; i < ref.size(); ++i)
            if (!ref_used[i]) {
                ref_left.push_back(ref[i]);
                ref_left_idx.push_back(i);
            }
        _Buckets cent(ref_left, Mode::ByCentroid, tol);
        for (size_t si : scene_unmatched) {
            size_t li = cent.take(_centroid_key(scene[si].centroid, tol));
            if (li != SIZE_MAX) {
                ref_used[ref_left_idx[li]] = 1;
                r.ops.push_back({scene[si].node_id, Status::Unchanged});
                ++r.n_unchanged;
            } else {
                r.ops.push_back({scene[si].node_id, Status::Added});
                r.added_node_ids.push_back(scene[si].node_id);
                ++r.n_added;
            }
        }
    }

    for (size_t i = 0; i < ref.size(); ++i)
        if (!ref_used[i]) {
            r.removed_node_ids.push_back(ref[i].node_id);
            ++r.n_removed;
        }
    return r;
}

} // namespace gdiff
} // namespace adacpp
