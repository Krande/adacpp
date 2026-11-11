// The animated part of a GLB (internal): what GlbAnimations plays, and its builder while the scene is read.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "glb_gltf.h"
#include "glb_math.h"
#include "glb_scene.h"

namespace adacpp::render {

// What GlbAnimations plays: the nodes kept apart, parents first, and each animation's channels.
struct GlbAnimations::Data {
    using Mat4 = detail::Mat4;
    enum Path { Translation = 0, Rotation = 1, Scale = 2 };
    enum Interpolation { Linear = 0, Step = 1, CubicSpline = 2 };
    struct Node {
        int32_t gltf_node = -1;
        int32_t parent = -1;           // into `nodes`; -1 = under the merged part of the scene, at `base`
        Mat4 base = detail::kIdentity; // the world transform of what is above it (parent -1)
        Mat4 rest = detail::kIdentity; // its own transform as the GLB holds it
        double t[3] = {0, 0, 0};       // the same, for a channel to replace
        float r[4] = {0, 0, 0, 1}, s[3] = {1, 1, 1};
        std::vector<uint32_t> instances;
    };
    struct Channel {
        uint32_t node = 0; // into `nodes`
        Path path = Translation;
        Interpolation interpolation = Linear;
        std::vector<float> times;  // seconds, ascending
        std::vector<float> values; // 3 or 4 per keyframe; CUBICSPLINE: in-tangent, value, out-tangent
    };
    struct Clip {
        std::string name;
        float duration = 0.0f;
        std::vector<Channel> channels;
    };
    std::vector<Node> nodes;
    std::vector<int32_t> gltf_nodes; // nodes[i].gltf_node
    std::vector<Clip> clips;
    std::vector<Mat4> world; // per node, as the last apply() left it (GlbOptions::offset in)

    // `width` values of `ch` at `time` into `out`.
    static void sample(const Channel &ch, float time, float *out);
    // The world matrix of the first `count` nodes with `clip` (null = at rest) at `time`, into `out`.
    void pose(const Clip *clip, float time, size_t count, std::vector<Mat4> &out) const;
};

namespace detail {

// The animated part of the scene while it is read: GlbAnimations::Data in the making, and the poses the
// bounds are taken over.
class AnimationBuilder {
public:
    using Data = GlbAnimations::Data;
    static constexpr int kBoundsSamples = 33; // per animation, evenly spaced over its duration

    // Reads every animation's translation / rotation / scale channels; `targeted()` then names the nodes.
    explicit AnimationBuilder(const tinygltf::Model &model);

    bool targeted(int gltf_node) const {
        return targeted_[gltf_node] != 0;
    }

    // A node to keep apart, under `parent` (one of these, or -1 with what is above it at `base`). Parents
    // come before their children. Returns its index.
    int32_t add_node(const tinygltf::Node &n, int gltf_node, int32_t parent, const Mat4 &base);

    int gltf_node(int32_t node) const {
        return data_->nodes[node].gltf_node;
    }

    // The node's world transform at rest.
    Mat4 rest_world(int32_t node) {
        return poses()[0][node];
    }

    void add_instance(int32_t node, uint32_t instance) {
        data_->nodes[node].instances.push_back(instance);
    }

    // `box` (min xyz, max xyz in the node's space) over the rest pose and every sampled pose, added to `out`.
    void grow(float out[6], int32_t node, const float box[6]);

    // The animations, when anything is animated.
    GlbAnimations finish();

private:
    // The poses of the nodes added so far: at rest, then each animation's samples. While the scene is read
    // the channels still name glTF nodes, so they are mapped as they are sampled.
    const std::vector<std::vector<Mat4>> &poses();

    std::shared_ptr<Data> data_ = std::make_shared<Data>();
    std::vector<char> targeted_;
    std::vector<int32_t> place_; // glTF node -> index among the nodes kept apart, -1 = not one
    std::vector<std::vector<Mat4>> poses_;
};

} // namespace detail

} // namespace adacpp::render
