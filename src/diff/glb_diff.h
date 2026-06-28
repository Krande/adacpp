#pragma once
// Streaming-friendly GLB model diff core (OCC-free). Drop-in producer for adapy's diff @utility:
// compute a per-element summary table for two GLBs, match them, and emit (a) a colour-op list keyed
// by the GLB draw-range node_id (== frontend rangeId) and (b) the node_ids whose geometry is unique
// to the ref (for an overlay GLB). Peak memory is one model at a time (never both), and only the
// per-element SUMMARY (centroid/area/bbox), not whole geometry, is retained for matching.
//
// Portability: the GLB -> ElementSummary fold (`summarize_glb` in glb_diff_native.h) is the only
// native-specific part (tinygltf, non-wasm build); the diff logic (`diff_summaries`) here is plain
// portable C++ so the wasm target can swap in a tinygltf-free reader and feed the same summaries.

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

// ── portable diff logic ──────────────────────────────────────────────────────────────────────────
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
