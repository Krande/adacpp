#include "glb_animation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "glb_parse.h"

namespace adacpp::render {

using detail::compose;
using detail::mul;

void GlbAnimations::Data::sample(const Channel &ch, float time, float *out) {
    const int width = ch.path == Rotation ? 4 : 3;
    const size_t n = ch.times.size();
    const bool cubic = ch.interpolation == CubicSpline;
    const size_t stride = size_t(width) * (cubic ? 3 : 1), value_at = cubic ? size_t(width) : 0;
    auto key = [&](size_t k) { return ch.values.data() + k * stride + value_at; };
    size_t k1 = size_t(std::upper_bound(ch.times.begin(), ch.times.end(), time) - ch.times.begin());
    if (k1 == 0 || k1 >= n || ch.interpolation == Step) {
        std::copy_n(key(k1 == 0 ? 0 : k1 - 1), width, out);
    } else {
        const size_t k0 = k1 - 1;
        const float dt = ch.times[k1] - ch.times[k0];
        const float u = dt > 0.0f ? (time - ch.times[k0]) / dt : 0.0f;
        const float *a = key(k0), *b = key(k1);
        if (cubic) {
            const float u2 = u * u, u3 = u2 * u;
            const float *out_tangent = a + width, *in_tangent = b - width;
            for (int c = 0; c < width; ++c)
                out[c] = (2 * u3 - 3 * u2 + 1) * a[c] + dt * (u3 - 2 * u2 + u) * out_tangent[c] +
                         (-2 * u3 + 3 * u2) * b[c] + dt * (u3 - u2) * in_tangent[c];
        } else if (ch.path == Rotation) {
            // Spherical, the short way round; near-parallel quaternions fall back to a straight blend.
            float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
            const float sign = d < 0.0f ? -1.0f : 1.0f;
            d = std::min(d * sign, 1.0f);
            float wa = 1.0f - u, wb = u;
            if (d < 0.9995f) {
                const float theta = std::acos(d), inv = 1.0f / std::sin(theta);
                wa = std::sin((1.0f - u) * theta) * inv;
                wb = std::sin(u * theta) * inv;
            }
            for (int c = 0; c < 4; ++c)
                out[c] = wa * a[c] + wb * sign * b[c];
        } else {
            for (int c = 0; c < width; ++c)
                out[c] = a[c] + (b[c] - a[c]) * u;
        }
    }
    if (ch.path == Rotation) {
        const float l = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
        if (l > 0.0f)
            for (int c = 0; c < 4; ++c)
                out[c] /= l;
    }
}

void GlbAnimations::Data::pose(const Clip *clip, float time, size_t count, std::vector<Mat4> &out) const {
    struct Trs {
        float t[3], r[4], s[3];
        bool moved;
    };
    std::vector<Trs> trs(clip ? count : 0);
    if (clip) {
        for (size_t i = 0; i < count; ++i) {
            const Node &n = nodes[i];
            std::copy_n(n.t, 3, trs[i].t);
            std::copy_n(n.r, 4, trs[i].r);
            std::copy_n(n.s, 3, trs[i].s);
            trs[i].moved = false;
        }
        for (const Channel &ch : clip->channels) {
            if (ch.node >= count || ch.times.empty())
                continue;
            Trs &x = trs[ch.node];
            sample(ch, time, ch.path == Translation ? x.t : ch.path == Rotation ? x.r : x.s);
            x.moved = true;
        }
    }
    out.resize(count);
    for (size_t i = 0; i < count; ++i) {
        const Node &n = nodes[i];
        const Mat4 local = clip && trs[i].moved ? compose(trs[i].t, trs[i].r, trs[i].s) : n.rest;
        out[i] = mul(n.parent >= 0 ? out[n.parent] : n.base, local);
    }
}

size_t GlbAnimations::count() const {
    return data_ ? data_->clips.size() : 0;
}

const std::string &GlbAnimations::name(size_t animation) const {
    if (animation >= count())
        throw std::out_of_range("adacpp.render: unknown animation " + std::to_string(animation));
    return data_->clips[animation].name;
}

float GlbAnimations::duration(size_t animation) const {
    if (animation >= count())
        throw std::out_of_range("adacpp.render: unknown animation " + std::to_string(animation));
    return data_->clips[animation].duration;
}

void GlbAnimations::apply(Renderer &renderer, int animation, float time) const {
    if (!data_)
        return;
    if (animation >= int(data_->clips.size()))
        throw std::out_of_range("adacpp.render: unknown animation " + std::to_string(animation));
    data_->pose(animation < 0 ? nullptr : &data_->clips[animation], time, data_->nodes.size(), data_->world);
    for (size_t i = 0; i < data_->nodes.size(); ++i)
        for (const uint32_t instance : data_->nodes[i].instances)
            renderer.set_instance_model(instance, data_->world[i].data());
}

const std::vector<int32_t> &GlbAnimations::nodes() const {
    static const std::vector<int32_t> none;
    return data_ ? data_->gltf_nodes : none;
}

std::array<float, 16> GlbAnimations::node_matrix(size_t i) const {
    if (!data_ || i >= data_->world.size())
        throw std::out_of_range("adacpp.render: unknown animated node " + std::to_string(i));
    return data_->world[i];
}

namespace detail {

AnimationBuilder::AnimationBuilder(const tinygltf::Model &model) : targeted_(model.nodes.size(), 0) {
    for (const auto &anim : model.animations) {
        Data::Clip clip;
        clip.name = anim.name;
        for (const auto &ch : anim.channels) {
            Data::Path path;
            if (ch.target_path == "translation")
                path = Data::Translation;
            else if (ch.target_path == "rotation")
                path = Data::Rotation;
            else if (ch.target_path == "scale")
                path = Data::Scale;
            else
                continue; // morph weights
            if (ch.target_node < 0 || ch.target_node >= int(model.nodes.size()) || ch.sampler < 0 ||
                ch.sampler >= int(anim.samplers.size()))
                continue;
            const auto &smp = anim.samplers[ch.sampler];
            if (smp.input < 0 || smp.output < 0)
                continue;
            Data::Channel out;
            out.node = uint32_t(ch.target_node); // a glTF node for now: place() makes it an index
            out.path = path;
            out.interpolation = smp.interpolation == "STEP"          ? Data::Step
                                : smp.interpolation == "CUBICSPLINE" ? Data::CubicSpline
                                                                     : Data::Linear;
            const int width = path == Data::Rotation ? 4 : 3;
            out.times = read_floats(model, smp.input, 1);
            out.values = read_floats(model, smp.output, width);
            const size_t per_key = size_t(width) * (out.interpolation == Data::CubicSpline ? 3 : 1);
            const size_t keys = std::min(out.times.size(), out.values.size() / per_key);
            if (keys == 0)
                continue;
            out.times.resize(keys);
            out.values.resize(keys * per_key);
            clip.duration = std::max(clip.duration, out.times.back());
            targeted_[ch.target_node] = 1;
            clip.channels.push_back(std::move(out));
        }
        data_->clips.push_back(std::move(clip));
    }
}

int32_t AnimationBuilder::add_node(const tinygltf::Node &n, int gltf_node, int32_t parent, const Mat4 &base) {
    Data::Node node;
    node.gltf_node = gltf_node;
    node.parent = parent;
    node.base = base;
    node.rest = node_matrix(n);
    for (size_t i = 0; i < n.translation.size() && i < 3; ++i)
        node.t[i] = float(n.translation[i]);
    for (size_t i = 0; i < n.rotation.size() && i < 4; ++i)
        node.r[i] = float(n.rotation[i]);
    for (size_t i = 0; i < n.scale.size() && i < 3; ++i)
        node.s[i] = float(n.scale[i]);
    const int32_t index = int32_t(data_->nodes.size());
    if (size_t(gltf_node) >= place_.size())
        place_.resize(gltf_node + 1, -1);
    place_[gltf_node] = index;
    data_->nodes.push_back(std::move(node));
    data_->gltf_nodes.push_back(gltf_node);
    poses_.clear(); // taken again, with this node, when next asked for
    return index;
}

void AnimationBuilder::grow(float out[6], int32_t node, const float box[6]) {
    for (const std::vector<Mat4> &pose : poses())
        grow_bounds(out, pose[node], box);
}

GlbAnimations AnimationBuilder::finish() {
    if (data_->nodes.empty() || data_->clips.empty())
        return {};
    // Channels name their node by its place among the nodes kept apart; one aimed at a node outside
    // the scene moves nothing.
    for (Data::Clip &clip : data_->clips) {
        std::vector<Data::Channel> kept;
        for (Data::Channel &ch : clip.channels) {
            if (ch.node >= place_.size() || place_[ch.node] < 0)
                continue;
            ch.node = uint32_t(place_[ch.node]);
            kept.push_back(std::move(ch));
        }
        clip.channels = std::move(kept);
    }
    data_->pose(nullptr, 0.0f, data_->nodes.size(), data_->world);
    return GlbAnimations(data_);
}

const std::vector<std::vector<Mat4>> &AnimationBuilder::poses() {
    if (!poses_.empty())
        return poses_;
    const size_t count = data_->nodes.size();
    poses_.emplace_back();
    data_->pose(nullptr, 0.0f, count, poses_.back());
    for (const Data::Clip &clip : data_->clips) {
        Data::Clip mapped;
        for (const Data::Channel &ch : clip.channels)
            if (ch.node < place_.size() && place_[ch.node] >= 0) {
                mapped.channels.push_back(ch);
                mapped.channels.back().node = uint32_t(place_[ch.node]);
            }
        if (mapped.channels.empty())
            continue;
        for (int k = 0; k < kBoundsSamples; ++k) {
            poses_.emplace_back();
            data_->pose(&mapped, clip.duration * float(k) / float(kBoundsSamples - 1), count, poses_.back());
        }
    }
    return poses_;
}

} // namespace detail

} // namespace adacpp::render
