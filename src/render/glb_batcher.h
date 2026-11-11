// Primitives into renderer geometries (internal): the batches a loaded model is drawn as.
#pragma once

#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

#include "glb_animation.h"
#include "glb_gltf.h"
#include "glb_math.h"
#include "glb_objects.h"
#include "glb_scene.h"
#include "renderer.h"

namespace adacpp::render::detail {

// Every primitive of one material and topology, merged into one geometry in world space, as the viewer
// batches a model (CustomBatchedMesh): a model of 15k objects is then a handful of draws instead of 15k.
// Each primitive -- or each adapy draw range in it -- stays a segment, so picking still names the object.
// What an animation moves (GlbOptions::animations) is batched per node instead, in the node's own space.
class Batcher {
public:
    Batcher(Renderer &r, const tinygltf::Model &model, GlbLoadResult &res, const GlbOptions &options,
            ObjectTable &objects, AnimationBuilder *animation)
        : r_(r), model_(model), res_(res), options_(options), objects_(objects), animation_(animation) {}

    // `object`: what the primitive is when no draw ranges split it (-1 = nothing). `animated`: the node kept
    // apart (AnimationBuilder) whose instance the primitive joins, -1 to merge it at `world`.
    void add(const tinygltf::Primitive &prim, const Mat4 &world, const tinygltf::Value *ranges, int32_t object,
             int32_t animated = -1);

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
        std::vector<int32_t> segment_objects; // per segment, its ObjectTable index or -1
        bool any_normals = false, any_colors = false;
        int32_t animated = -1; // the AnimationBuilder node whose space the vertices are in
        size_t vertex_count() const {
            return positions.size() / 3;
        }
    };

    void flush(Batch &b);
    // The batch's feature edges as a line overlay (GlbOptions::feature_edge_angle_deg), drawn like the
    // GLB's own uncoloured lines; the faces already sit behind coplanar lines (kDepthBack).
    void add_feature_edges(const Batch &b);

    Renderer &r_;
    const tinygltf::Model &model_;
    GlbLoadResult &res_;
    GlbOptions options_;
    ObjectTable &objects_;
    AnimationBuilder *animation_; // null unless GlbOptions::animations found something to animate
    // By (animated node or -1, topology, material, coloured lines): the merged batches first.
    std::map<std::tuple<int, int, int, bool>, Batch> batches_;
};

} // namespace adacpp::render::detail
