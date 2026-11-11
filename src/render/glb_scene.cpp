#include "glb_scene.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "glb_animation.h"
#include "glb_batcher.h"
#include "glb_gltf.h"
#include "glb_math.h"
#include "glb_objects.h"
#include "glb_parse.h"

namespace adacpp::render {

using namespace detail;

namespace {

struct Walk {
    Batcher &batcher;
    const tinygltf::Model &model;
    const tinygltf::Value &scene_extras;
    ObjectTable &objects;
    bool node_objects; // every node is an object (no adapy extras to name them)
    AnimationBuilder *animation = nullptr;

    // `animated`: the node kept apart that this one hangs under, -1 while in the merged part of the scene.
    void node(int node_index, const Mat4 &parent, int32_t parent_object, int32_t animated = -1) {
        const auto &n = model.nodes.at(node_index);
        const Mat4 world = mul(parent, node_matrix(n));
        const int32_t object = node_objects ? objects.for_node(model, node_index, parent_object) : -1;
        if (animation && (animated >= 0 || animation->targeted(node_index)))
            animated = animation->add_node(n, node_index, animated, parent);
        if (n.mesh >= 0) {
            const auto &prims = model.meshes.at(n.mesh).primitives;
            const std::string key = "draw_ranges_" + n.name;
            const tinygltf::Value *ranges = prims.size() == 1 && scene_extras.IsObject() && scene_extras.Has(key)
                                                ? &scene_extras.Get(key)
                                                : nullptr;
            for (const auto &prim : prims)
                batcher.add(prim, world, ranges, object, animated);
        }
        for (int child : n.children)
            node(child, world, object, animated);
    }
};

// Above every node: glTF y-up -> z-up (+90 degrees about x) when asked, then the offset.
Mat4 root_matrix(bool y_up_to_z_up, const std::array<double, 3> &offset) {
    Mat4 m = y_up_to_z_up ? Mat4{1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1} : kIdentity;
    for (int k = 0; k < 3; ++k)
        m[12 + k] = offset[k];
    return m;
}

// The default scene's nodes; every node for a GLB without scenes.
std::vector<int> scene_roots(const tinygltf::Model &model) {
    if (!model.scenes.empty())
        return model.scenes.at(model.defaultScene >= 0 ? model.defaultScene : 0).nodes;
    std::vector<int> all(model.nodes.size());
    for (size_t n = 0; n < all.size(); ++n)
        all[n] = int(n);
    return all;
}

// What a primitive of a node at `world` adds to the box glb_bounds_bytes reports.
void grow_primitive_bounds(const tinygltf::Model &model, const tinygltf::Primitive &prim, const Mat4 &world,
                           double out[6]) {
    if (prim.mode != -1 && prim.mode != TINYGLTF_MODE_TRIANGLES && prim.mode != TINYGLTF_MODE_LINE &&
        prim.mode != TINYGLTF_MODE_POINTS)
        return; // not drawn (Batcher::add)
    const auto pos = prim.attributes.find("POSITION");
    if (pos == prim.attributes.end())
        return;
    const auto &acc = model.accessors.at(pos->second);
    double box[6] = {3.4e38, 3.4e38, 3.4e38, -3.4e38, -3.4e38, -3.4e38};
    // min / max as glTF requires them of positions; what is read is what a load reads otherwise.
    if (acc.minValues.size() == 3 && acc.maxValues.size() == 3 && !acc.normalized &&
        acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
        for (int k = 0; k < 3; ++k) {
            box[k] = acc.minValues[k];
            box[3 + k] = acc.maxValues[k];
        }
    } else {
        std::vector<float> positions, normals, colors;
        std::vector<uint32_t> indices;
        if (!read_draco(model, prim, positions, normals, colors, indices)) {
            if (acc.bufferView < 0)
                return;
            positions = read_floats(model, pos->second, 3);
        }
        if (positions.empty())
            return;
        for (size_t i = 0; i < positions.size(); ++i) {
            box[i % 3] = std::min(box[i % 3], double(positions[i]));
            box[3 + i % 3] = std::max(box[3 + i % 3], double(positions[i]));
        }
    }
    grow_bounds(out, world, box);
}

void grow_node_bounds(const tinygltf::Model &model, int node_index, const Mat4 &parent, double out[6]) {
    const auto &n = model.nodes.at(node_index);
    const Mat4 world = mul(parent, node_matrix(n));
    if (n.mesh >= 0)
        for (const auto &prim : model.meshes.at(n.mesh).primitives)
            grow_primitive_bounds(model, prim, world, out);
    for (int child : n.children)
        grow_node_bounds(model, child, world, out);
}

} // namespace

GlbLoadResult load_glb(Renderer &renderer, const std::string &path, const GlbOptions &options) {
    std::vector<unsigned char> bytes;
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("adacpp.render: cannot open " + path);
        bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    return load_glb_bytes(renderer, std::move(bytes), options, path);
}

GlbLoadResult load_glb_bytes(Renderer &renderer, std::vector<unsigned char> bytes, const GlbOptions &options,
                             const std::string &name) {
    GlbLoadResult res;
    read_passthrough(bytes, res);
    const tinygltf::Model model = parse_glb(std::move(bytes), name);

    const Mat4 root = root_matrix(options.y_up_to_z_up, options.offset);
    const int scene = model.defaultScene >= 0 ? model.defaultScene : 0;
    const tinygltf::Value no_extras;
    const tinygltf::Value &extras = model.scenes.empty() ? no_extras : model.scenes.at(scene).extras;
    // adapy names the objects in the scene extras; without them the nodes are the objects.
    ObjectTable objects;
    bool adapy = false;
    if (extras.IsObject()) {
        if (extras.Has("id_hierarchy") && extras.Get("id_hierarchy").IsObject()) {
            objects.add_hierarchy(extras.Get("id_hierarchy"));
            adapy = true;
        }
        for (const auto &key : extras.Keys())
            adapy |= key.rfind("draw_ranges_", 0) == 0;
    }
    res.node_objects = !adapy;
    std::unique_ptr<AnimationBuilder> animation;
    if (options.animations && !model.animations.empty())
        animation = std::make_unique<AnimationBuilder>(model);
    Batcher batcher(renderer, model, res, options, objects, animation.get());
    Walk walk{batcher, model, extras, objects, res.node_objects, animation.get()};
    for (int n : scene_roots(model))
        walk.node(n, root, -1);
    batcher.finish();
    objects.finish(res);
    if (animation)
        res.animations = animation->finish();
    return res;
}

std::array<double, 6> glb_bounds_bytes(const std::vector<unsigned char> &bytes, bool y_up_to_z_up) {
    const tinygltf::Model model = parse_glb(bytes, "GLB");
    std::array<double, 6> out{3.4e38, 3.4e38, 3.4e38, -3.4e38, -3.4e38, -3.4e38};
    const Mat4 root = root_matrix(y_up_to_z_up, {0, 0, 0});
    for (int n : scene_roots(model))
        grow_node_bounds(model, n, root, out.data());
    return out;
}

Camera look_at(const float eye[3], const float target[3], std::array<float, 3> up, float fov_y_deg, float aspect,
               const std::array<float, 16> &fit) {
    float zc[3] = {eye[0] - target[0], eye[1] - target[1], eye[2] - target[2]};
    normalize(zc);
    float xc[3] = {up[1] * zc[2] - up[2] * zc[1], up[2] * zc[0] - up[0] * zc[2], up[0] * zc[1] - up[1] * zc[0]};
    normalize(xc);
    float yc[3] = {zc[1] * xc[2] - zc[2] * xc[1], zc[2] * xc[0] - zc[0] * xc[2], zc[0] * xc[1] - zc[1] * xc[0]};
    Camera cam{};
    auto &v = cam.view;
    v = {xc[0], yc[0], zc[0], 0, xc[1], yc[1], zc[1], 0, xc[2], yc[2], zc[2], 0, 0, 0, 0, 1};
    v[12] = -(xc[0] * eye[0] + xc[1] * eye[1] + xc[2] * eye[2]);
    v[13] = -(yc[0] * eye[0] + yc[1] * eye[1] + yc[2] * eye[2]);
    v[14] = -(zc[0] * eye[0] + zc[1] * eye[1] + zc[2] * eye[2]);
    const float near_z = 0.05f, far_z = 2000.0f;
    const float f = 1.0f / std::tan(fov_y_deg * 3.14159265f / 360.0f);
    auto &p = cam.proj;
    p = {};
    p[0] = f / aspect;
    p[5] = f;
    p[10] = (far_z + near_z) / (near_z - far_z);
    p[11] = -1.0f;
    p[14] = 2.0f * far_z * near_z / (near_z - far_z);
    (void) fit;
    return cam;
}

Camera frame_bounds(const std::array<float, 6> &b, std::array<float, 3> direction, std::array<float, 3> up,
                    float fov_y_deg, float aspect, float margin) {
    float center[3] = {0.5f * (b[0] + b[3]), 0.5f * (b[1] + b[4]), 0.5f * (b[2] + b[5])};
    const float dx = b[3] - b[0], dy = b[4] - b[1], dz = b[5] - b[2];
    const float radius = std::max(0.5f * std::sqrt(dx * dx + dy * dy + dz * dz), 1e-3f);
    const float fov = fov_y_deg * 3.14159265f / 180.0f;
    const float fov_min = aspect < 1.0f ? 2.0f * std::atan(std::tan(fov * 0.5f) * aspect) : fov;
    const float dist = margin * radius / std::sin(fov_min * 0.5f);

    float d[3] = {direction[0], direction[1], direction[2]};
    normalize(d);
    float eye[3] = {center[0] + d[0] * dist, center[1] + d[1] * dist, center[2] + d[2] * dist};
    // Right-handed look-at: camera z axis points from the target to the eye.
    float zc[3] = {d[0], d[1], d[2]};
    float xc[3] = {up[1] * zc[2] - up[2] * zc[1], up[2] * zc[0] - up[0] * zc[2], up[0] * zc[1] - up[1] * zc[0]};
    normalize(xc);
    float yc[3] = {zc[1] * xc[2] - zc[2] * xc[1], zc[2] * xc[0] - zc[0] * xc[2], zc[0] * xc[1] - zc[1] * xc[0]};
    Camera cam{};
    auto &v = cam.view;
    v = {xc[0], yc[0], zc[0], 0, xc[1], yc[1], zc[1], 0, xc[2], yc[2], zc[2], 0, 0, 0, 0, 1};
    v[12] = -(xc[0] * eye[0] + xc[1] * eye[1] + xc[2] * eye[2]);
    v[13] = -(yc[0] * eye[0] + yc[1] * eye[1] + yc[2] * eye[2]);
    v[14] = -(zc[0] * eye[0] + zc[1] * eye[1] + zc[2] * eye[2]);

    const float near_z = std::max(dist - radius * 1.5f, dist * 1e-3f), far_z = dist + radius * 1.5f;
    const float f = 1.0f / std::tan(fov * 0.5f);
    auto &p = cam.proj;
    p = {};
    p[0] = f / aspect;
    p[5] = f;
    p[10] = (far_z + near_z) / (near_z - far_z);
    p[11] = -1.0f;
    p[14] = 2.0f * far_z * near_z / (near_z - far_z);
    return cam;
}

} // namespace adacpp::render
