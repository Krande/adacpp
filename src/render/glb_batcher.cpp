#include "glb_batcher.h"

#include <algorithm>
#include <cmath>

#include "glb_parse.h"

namespace adacpp::render {

namespace detail {

namespace {

uint32_t pack_rgba8(const float *c) {
    auto q = [](float v) { return uint32_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); };
    return q(c[0]) | (q(c[1]) << 8) | (q(c[2]) << 16) | (q(c[3]) << 24);
}

InstanceDesc material_desc(const tinygltf::Model &model, int material, bool vertex_colors) {
    InstanceDesc idesc;
    idesc.flags |= kClipped; // a loaded model is cut by the section planes, when any are set
    // Faces sit just behind the GLB's own edge lines, so those draw cleanly on them (triangles only).
    idesc.flags |= kDepthBack;
    if (vertex_colors)
        idesc.flags |= kVertexColors;
    // No material: glTF's default material, as three.js' GLTFLoader builds it (metallic 1, roughness 1).
    idesc.metalness = 1.0f;
    idesc.roughness = 1.0f;
    if (material >= 0) {
        const auto &mat = model.materials.at(material);
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
    return idesc;
}

} // namespace

void Batcher::add(const tinygltf::Primitive &prim, const Mat4 &world, const tinygltf::Value *ranges, int32_t object,
                  int32_t animated) {
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
    std::vector<float> positions, normals, colors;
    std::vector<uint32_t> indices;
    const bool draco = read_draco(model_, prim, positions, normals, colors, indices);
    if (!draco) {
        positions = read_floats(model_, pos_it->second, 3);
        if (auto it = prim.attributes.find("NORMAL"); it != prim.attributes.end())
            normals = read_floats(model_, it->second, 3);
        if (auto it = prim.attributes.find("COLOR_0"); it != prim.attributes.end())
            colors = read_floats(model_, it->second, 4);
    }
    const size_t nv = positions.size() / 3;
    if (!draco && prim.indices >= 0) {
        indices = read_indices(model_, prim.indices);
    } else if (!draco) {
        indices.resize(nv);
        for (uint32_t i = 0; i < nv; ++i)
            indices[i] = i;
    }

    // Coloured and plain lines draw differently (members vs wireframe): never in one batch.
    const bool colored = !colors.empty() && topo != Topology::Triangles;
    Batch &b = batches_[{animated, int(topo), prim.material, colored}];
    b.topology = topo;
    b.material = prim.material;
    b.animated = animated;
    if (b.vertex_count() + nv > kBatchMaxVertices || b.indices.size() + indices.size() > kBatchMaxIndices)
        flush(b);

    // Normals take the inverse transpose (the cofactor matrix, up to scale); a mirroring transform also
    // flips the winding, as glTF requires.
    // An animated node's vertices stay in its space; only a mirroring rest pose still turns the winding.
    const Mat4 &m = animated >= 0 ? kIdentity : world;
    const float cof[9] = {m[5] * m[10] - m[6] * m[9], m[6] * m[8] - m[4] * m[10], m[4] * m[9] - m[5] * m[8],
                          m[2] * m[9] - m[1] * m[10], m[0] * m[10] - m[2] * m[8], m[1] * m[8] - m[0] * m[9],
                          m[1] * m[6] - m[2] * m[5],  m[2] * m[4] - m[0] * m[6],  m[0] * m[5] - m[1] * m[4]};
    const float det = m[0] * cof[0] + m[4] * cof[1] + m[8] * cof[2];
    const float sign = det < 0.0f ? -1.0f : 1.0f;
    const bool mirrored = animated >= 0 ? det3(world) < 0.0f : det < 0.0f;
    const uint32_t base = uint32_t(b.vertex_count());
    for (size_t v = 0; v < nv; ++v) {
        const float *p = &positions[v * 3];
        for (int k = 0; k < 3; ++k) {
            const float w = m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k];
            b.positions.push_back(w);
            if (animated < 0) { // an animated batch joins the bounds once flushed, over its motion
                res_.bounds[k] = std::min(res_.bounds[k], w);
                res_.bounds[3 + k] = std::max(res_.bounds[3 + k], w);
            }
        }
        float n[3] = {0, 0, 0}; // zero: the shader derives a flat normal
        if (!normals.empty()) {
            const float *s = &normals[v * 3];
            for (int k = 0; k < 3; ++k)
                n[k] = sign * (cof[k * 3] * s[0] + cof[k * 3 + 1] * s[1] + cof[k * 3 + 2] * s[2]);
            normalize(n);
        }
        b.normals.insert(b.normals.end(), n, n + 3);
        b.colors.push_back(colors.empty() ? 0xFFFFFFFFu : pack_rgba8(&colors[v * 4]));
    }
    b.any_normals |= !normals.empty();
    b.any_colors |= !colors.empty();

    const uint32_t first = uint32_t(b.indices.size());
    const bool flip = mirrored && topo == Topology::Triangles;
    for (size_t i = 0; i < indices.size(); ++i) {
        // Mirrored triangle: swap its 2nd and 3rd corner.
        size_t src = i;
        if (flip && i - i % 3 + 2 < indices.size() && i % 3 != 0)
            src = i % 3 == 1 ? i + 1 : i - 1;
        b.indices.push_back(base + indices[src]);
    }
    auto segments = topo == Topology::Triangles ? segments_from_extras(ranges, indices.size(), objects_)
                                                : std::vector<RangeSegment>{};
    if (segments.empty())
        segments = {{0, uint32_t(indices.size()), object}};
    for (const RangeSegment &seg : segments) {
        b.segments.insert(b.segments.end(), {first + seg.start, seg.count});
        b.segment_objects.push_back(seg.object);
    }
    res_.segments += segments.size();
    res_.vertices += nv;
    if (topo == Topology::Triangles)
        res_.triangles += indices.size() / 3;
}

void Batcher::flush(Batch &b) {
    if (b.positions.empty())
        return;
    GeometryDesc gd;
    gd.topology = b.topology;
    gd.positions = b.positions.data();
    gd.vertex_count = b.vertex_count();
    gd.normals = b.any_normals ? b.normals.data() : nullptr;
    gd.colors = b.any_colors ? b.colors.data() : nullptr;
    gd.indices = b.indices.data();
    gd.index_count = b.indices.size();
    gd.segments = b.segments.data();
    gd.segment_count = b.segments.size() / 2;
    const uint32_t geom = r_.add_geometry(gd);
    const uint32_t inst = r_.add_instance(geom);
    InstanceDesc desc = material_desc(model_, b.material, b.any_colors);
    if (b.topology == Topology::Lines) {
        desc.line_width = b.any_colors ? options_.member_width : options_.line_width;
        if (!b.any_colors && options_.line_color[3] > 0.0f)
            desc.color = options_.line_color;
    } else if (b.topology == Topology::Points) {
        desc.line_width = options_.point_size;
    } else if (options_.double_sided) {
        desc.flags |= kDoubleSided;
    } else if (options_.section_caps && !(desc.flags & kDoubleSided)) {
        desc.flags |= kCapped;
    }
    if (b.animated >= 0) {
        desc.model = animation_->rest_world(b.animated);
        animation_->add_instance(b.animated, inst);
    }
    r_.set_instance(inst, desc);
    res_.geometries++;
    res_.instances++;
    GlbGeometry rec;
    rec.geometry = geom;
    rec.instance = inst;
    rec.topology = b.topology;
    rec.vertex_count = uint32_t(b.vertex_count());
    rec.vertex_colors = b.any_colors;
    if (b.animated >= 0) {
        rec.node = animation_->gltf_node(b.animated);
        float local[6] = {3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
        for (size_t v = 0; v < b.positions.size(); ++v) {
            local[v % 3] = std::min(local[v % 3], b.positions[v]);
            local[3 + v % 3] = std::max(local[3 + v % 3], b.positions[v]);
        }
        animation_->grow(res_.bounds.data(), b.animated, local);
    }
    if (options_.keep_positions) {
        rec.positions = b.positions;
        rec.indices = b.indices;
    }
    // Each object's share of the batch: its segments, and the bounds and triangles they cover.
    for (size_t k = 0; k < b.segment_objects.size(); ++k) {
        if (b.segment_objects[k] < 0)
            continue;
        const uint32_t start = b.segments[k * 2], count = b.segments[k * 2 + 1];
        float bounds[6] = {3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
        for (uint32_t i = start; i < start + count; ++i) {
            const float *p = &b.positions[size_t(b.indices[i]) * 3];
            for (int c = 0; c < 3; ++c) {
                bounds[c] = std::min(bounds[c], p[c]);
                bounds[3 + c] = std::max(bounds[3 + c], p[c]);
            }
        }
        if (b.animated >= 0 && count > 0) {
            float moved[6] = {3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
            animation_->grow(moved, b.animated, bounds);
            std::copy(moved, moved + 6, bounds);
        }
        objects_.add_segment(b.segment_objects[k], inst, uint32_t(k), bounds,
                             b.topology == Topology::Triangles ? count / 3 : 0);
    }
    rec.segment_objects = b.segment_objects;
    res_.loaded.push_back(std::move(rec));
    if (b.topology == Topology::Triangles && options_.feature_edge_angle_deg > 0.0f)
        add_feature_edges(b);
    const Topology topo = b.topology;
    const int material = b.material;
    const int32_t animated = b.animated;
    b = Batch{};
    b.topology = topo;
    b.material = material;
    b.animated = animated;
}

void Batcher::add_feature_edges(const Batch &b) {
    const std::vector<float> lines = feature_edges(b.positions, b.indices, options_.feature_edge_angle_deg);
    if (lines.empty())
        return;
    GeometryDesc gd;
    gd.topology = Topology::Lines;
    gd.positions = lines.data();
    gd.vertex_count = lines.size() / 3;
    const uint32_t geom = r_.add_geometry(gd);
    const uint32_t inst = r_.add_instance(geom);
    InstanceDesc desc;
    desc.flags = kClipped | kUnlit;
    desc.color = options_.line_color[3] > 0.0f ? options_.line_color : std::array<float, 4>{0, 0, 0, 1};
    desc.line_width = options_.line_width;
    if (b.animated >= 0) {
        desc.model = animation_->rest_world(b.animated);
        animation_->add_instance(b.animated, inst);
    }
    r_.set_instance(inst, desc);
    res_.geometries++;
    res_.instances++;
    res_.feature_edges += lines.size() / 6;
    GlbGeometry rec;
    if (b.animated >= 0)
        rec.node = animation_->gltf_node(b.animated);
    rec.geometry = geom;
    rec.instance = inst;
    rec.topology = Topology::Lines;
    rec.vertex_count = uint32_t(lines.size() / 3);
    rec.feature_edges = true;
    rec.segment_objects = {-1};
    res_.loaded.push_back(std::move(rec));
}

} // namespace detail

using detail::normalize;

std::vector<float> feature_edges(const std::vector<float> &positions, const std::vector<uint32_t> &indices,
                                 float angle_deg) {
    // Weld by position (a tessellated GLB is a triangle soup), quantised to 1e-4 model units as the viewer
    // does (frontend utils/mesh_select/edgeCore.ts).
    struct Key {
        int64_t x, y, z;
        bool operator==(const Key &o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct KeyHash {
        size_t operator()(const Key &k) const {
            return size_t(k.x * 73856093) ^ size_t(k.y * 19349663) ^ size_t(k.z * 83492791);
        }
    };
    const size_t nv = positions.size() / 3;
    std::unordered_map<Key, uint32_t, KeyHash> weld;
    weld.reserve(nv);
    std::vector<uint32_t> id(nv);
    for (size_t v = 0; v < nv; ++v) {
        const Key k{std::llround(positions[v * 3] * 1e4), std::llround(positions[v * 3 + 1] * 1e4),
                    std::llround(positions[v * 3 + 2] * 1e4)};
        id[v] = weld.emplace(k, uint32_t(v)).first->second;
    }

    const size_t nt = indices.size() / 3;
    std::vector<float> normal(nt * 3, 0.0f);
    for (size_t t = 0; t < nt; ++t) {
        const float *a = &positions[id[indices[t * 3]] * 3], *b = &positions[id[indices[t * 3 + 1]] * 3],
                    *c = &positions[id[indices[t * 3 + 2]] * 3];
        const float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float n[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
        normalize(n);
        std::copy(n, n + 3, &normal[t * 3]);
    }

    // Edge (lo << 32 | hi) -> the first face on it, how many faces share it, and (once a second face
    // arrives) whether the two meet at more than the angle.
    struct Use {
        uint32_t first, count;
        bool sharp;
    };
    std::unordered_map<uint64_t, Use> edges;
    edges.reserve(nt * 2);
    std::vector<uint64_t> order; // first-seen order, so the output is deterministic
    const float cos_angle = std::cos(angle_deg * 3.14159265358979f / 180.0f);
    for (size_t t = 0; t < nt; ++t) {
        const uint32_t v[3] = {id[indices[t * 3]], id[indices[t * 3 + 1]], id[indices[t * 3 + 2]]};
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2])
            continue; // collapsed by the weld
        for (int e = 0; e < 3; ++e) {
            const uint32_t lo = std::min(v[e], v[(e + 1) % 3]), hi = std::max(v[e], v[(e + 1) % 3]);
            const uint64_t key = (uint64_t(lo) << 32) | hi;
            auto [it, fresh] = edges.try_emplace(key, Use{uint32_t(t), 0, false});
            if (fresh)
                order.push_back(key);
            it->second.count++;
            if (it->second.count == 2) {
                const float *n1 = &normal[it->second.first * 3], *n2 = &normal[t * 3];
                it->second.sharp = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] <= cos_angle;
            }
        }
    }
    std::vector<float> out;
    for (const uint64_t key : order) {
        const Use &u = edges[key];
        // boundary (1) and non-manifold (>2) edges always; shared ones when the faces meet at the angle
        const bool show = u.count != 2 || u.sharp;
        if (!show)
            continue;
        for (const uint32_t v : {uint32_t(key >> 32), uint32_t(key & 0xFFFFFFFFu)})
            out.insert(out.end(), &positions[v * 3], &positions[v * 3] + 3);
    }
    return out;
}

} // namespace adacpp::render
