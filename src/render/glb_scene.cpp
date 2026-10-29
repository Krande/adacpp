#include "glb_scene.h"

#ifndef __EMSCRIPTEN__

#include <cmath>
#include <stdexcept>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#define STB_IMAGE_IMPLEMENTATION
#include <tiny_gltf.h>

namespace adacpp::render {

namespace {

using Mat4 = std::array<float, 16>;

Mat4 mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + row] * b[c * 4 + k];
            r[c * 4 + row] = s;
        }
    return r;
}

Mat4 node_matrix(const tinygltf::Node &n) {
    Mat4 m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    if (n.matrix.size() == 16) {
        for (int i = 0; i < 16; ++i)
            m[i] = float(n.matrix[i]);
        return m;
    }
    double t[3] = {0, 0, 0}, s[3] = {1, 1, 1}, q[4] = {0, 0, 0, 1};
    for (size_t i = 0; i < n.translation.size() && i < 3; ++i)
        t[i] = n.translation[i];
    for (size_t i = 0; i < n.scale.size() && i < 3; ++i)
        s[i] = n.scale[i];
    for (size_t i = 0; i < n.rotation.size() && i < 4; ++i)
        q[i] = n.rotation[i];
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                         2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            m[c * 4 + row] = float(r[c * 3 + row] * s[c]);
    m[12] = float(t[0]);
    m[13] = float(t[1]);
    m[14] = float(t[2]);
    return m;
}

// Read an accessor as floats (`comps` per element), honouring stride, component type and normalisation.
std::vector<float> read_floats(const tinygltf::Model &model, int accessor_index, int comps) {
    const auto &acc = model.accessors.at(accessor_index);
    const auto &view = model.bufferViews.at(acc.bufferView);
    const auto &buf = model.buffers.at(view.buffer);
    const int ncomp = tinygltf::GetNumComponentsInType(acc.type);
    const int csize = tinygltf::GetComponentSizeInBytes(acc.componentType);
    const size_t stride = view.byteStride ? view.byteStride : size_t(ncomp * csize);
    const uint8_t *base = buf.data.data() + view.byteOffset + acc.byteOffset;
    std::vector<float> out(acc.count * comps, 1.0f);
    for (size_t i = 0; i < acc.count; ++i) {
        const uint8_t *e = base + i * stride;
        for (int c = 0; c < std::min(ncomp, comps); ++c) {
            float v = 0.0f;
            switch (acc.componentType) {
            case TINYGLTF_COMPONENT_TYPE_FLOAT:
                v = reinterpret_cast<const float *>(e)[c];
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                v = e[c] / (acc.normalized || comps == 4 ? 255.0f : 1.0f);
                break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                v = reinterpret_cast<const uint16_t *>(e)[c] / (acc.normalized || comps == 4 ? 65535.0f : 1.0f);
                break;
            default:
                throw std::runtime_error("adacpp.render: unsupported accessor component type");
            }
            out[i * comps + c] = v;
        }
    }
    return out;
}

std::vector<uint32_t> read_indices(const tinygltf::Model &model, int accessor_index) {
    const auto &acc = model.accessors.at(accessor_index);
    const auto &view = model.bufferViews.at(acc.bufferView);
    const auto &buf = model.buffers.at(view.buffer);
    const int csize = tinygltf::GetComponentSizeInBytes(acc.componentType);
    const size_t stride = view.byteStride ? view.byteStride : size_t(csize);
    const uint8_t *base = buf.data.data() + view.byteOffset + acc.byteOffset;
    std::vector<uint32_t> out(acc.count);
    for (size_t i = 0; i < acc.count; ++i) {
        const uint8_t *e = base + i * stride;
        switch (acc.componentType) {
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            out[i] = *e;
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            out[i] = *reinterpret_cast<const uint16_t *>(e);
            break;
        default:
            out[i] = *reinterpret_cast<const uint32_t *>(e);
        }
    }
    return out;
}

uint32_t pack_rgba8(const float *c) {
    auto q = [](float v) { return uint32_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); };
    return q(c[0]) | (q(c[1]) << 8) | (q(c[2]) << 16) | (q(c[3]) << 24);
}

void add_primitive(Renderer &r, const tinygltf::Model &model, const tinygltf::Primitive &prim, const Mat4 &world,
                   GlbLoadResult &res) {
    Topology topo;
    if (prim.mode == TINYGLTF_MODE_TRIANGLES || prim.mode == -1)
        topo = Topology::Triangles;
    else if (prim.mode == TINYGLTF_MODE_LINE)
        topo = Topology::Lines;
    else if (prim.mode == TINYGLTF_MODE_POINTS)
        topo = Topology::Points;
    else
        return; // strips/fans/loops: not produced by adapy's writers
    auto pos_it = prim.attributes.find("POSITION");
    if (pos_it == prim.attributes.end())
        return;
    const auto positions = read_floats(model, pos_it->second, 3);
    std::vector<float> normals;
    if (auto it = prim.attributes.find("NORMAL"); it != prim.attributes.end())
        normals = read_floats(model, it->second, 3);
    std::vector<uint32_t> colors;
    if (auto it = prim.attributes.find("COLOR_0"); it != prim.attributes.end()) {
        const auto c = read_floats(model, it->second, 4);
        colors.resize(c.size() / 4);
        for (size_t i = 0; i < colors.size(); ++i)
            colors[i] = pack_rgba8(&c[i * 4]);
    }
    std::vector<uint32_t> indices;
    if (prim.indices >= 0)
        indices = read_indices(model, prim.indices);

    GeometryDesc gd;
    gd.topology = topo;
    gd.positions = positions.data();
    gd.vertex_count = positions.size() / 3;
    gd.normals = normals.empty() ? nullptr : normals.data();
    gd.colors = colors.empty() ? nullptr : colors.data();
    gd.indices = indices.empty() ? nullptr : indices.data();
    gd.index_count = indices.size();
    const uint32_t geom = r.add_geometry(gd);
    const uint32_t inst = r.add_instance(geom);

    InstanceDesc idesc;
    idesc.model = world;
    if (!colors.empty())
        idesc.flags |= kVertexColors;
    if (prim.material >= 0) {
        const auto &mat = model.materials.at(prim.material);
        const auto &f = mat.pbrMetallicRoughness.baseColorFactor;
        for (int i = 0; i < 4 && i < int(f.size()); ++i)
            idesc.color[i] = float(f[i]);
        // glTF defaults (metallic 1, roughness 1) are what three.js' GLTFLoader uses too.
        idesc.metalness = float(mat.pbrMetallicRoughness.metallicFactor);
        idesc.roughness = float(mat.pbrMetallicRoughness.roughnessFactor);
        if (mat.doubleSided)
            idesc.flags |= kDoubleSided;
        if (mat.alphaMode == "BLEND" && idesc.color[3] < 1.0f)
            idesc.flags |= kTransparent;
        if (mat.extensions.count("KHR_materials_unlit"))
            idesc.flags |= kUnlit;
    }
    r.set_instance(inst, idesc);
    res.geometries++;
    res.instances++;
    res.vertices += gd.vertex_count;
    if (topo == Topology::Triangles)
        res.triangles += (indices.empty() ? gd.vertex_count : indices.size()) / 3;
}

void walk(Renderer &r, const tinygltf::Model &model, int node_index, const Mat4 &parent, GlbLoadResult &res) {
    const auto &node = model.nodes.at(node_index);
    const Mat4 world = mul(parent, node_matrix(node));
    if (node.mesh >= 0)
        for (const auto &prim : model.meshes.at(node.mesh).primitives)
            add_primitive(r, model, prim, world, res);
    for (int child : node.children)
        walk(r, model, child, world, res);
}

void normalize(float v[3]) {
    float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l > 0)
        for (int i = 0; i < 3; ++i)
            v[i] /= l;
}

} // namespace

GlbLoadResult load_glb(Renderer &renderer, const std::string &path, bool y_up_to_z_up) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err, warn;
    // Images are never needed: the renderer does not texture.
    loader.SetImageLoader([](tinygltf::Image *, const int, std::string *, std::string *, int, int,
                             const unsigned char *, int, void *) { return true; },
                          nullptr);
    const bool ok = loader.LoadBinaryFromFile(&model, &err, &warn, path);
    if (!ok)
        throw std::runtime_error("adacpp.render: failed to read " + path + ": " + err);
    for (const auto &ext : model.extensionsRequired)
        if (ext == "EXT_meshopt_compression" || ext == "KHR_meshopt_compression" || ext == "KHR_draco_mesh_compression")
            throw std::runtime_error("adacpp.render: " + path + " requires " + ext + ", not supported yet");

    // glTF y-up -> z-up: rotate +90 degrees about x.
    const Mat4 root = y_up_to_z_up ? Mat4{1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1}
                                   : Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    GlbLoadResult res;
    const int scene = model.defaultScene >= 0 ? model.defaultScene : 0;
    if (!model.scenes.empty()) {
        for (int n : model.scenes.at(scene).nodes)
            walk(renderer, model, n, root, res);
    } else {
        for (size_t n = 0; n < model.nodes.size(); ++n)
            walk(renderer, model, int(n), root, res);
    }
    return res;
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

#endif
