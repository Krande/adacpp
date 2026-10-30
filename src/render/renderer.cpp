#include "renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <meshoptimizer.h>

#include "shaders.h"

namespace adacpp::render {

namespace {

constexpr uint32_t kHasMorph = 1u << 6; // internal: set when the instance's geometry carries a morph target
constexpr uint32_t kConeOk = 1u << 9;   // internal: cone culling is exact for this instance (GPU culling)
constexpr uint32_t kPoints = 1u << 10;  // internal: a points instance, for vs_wide
constexpr size_t kInstanceBytes = 112;  // WGSL `Instance`
constexpr size_t kItemWords = 16;       // WGSL `Item` (cull shader)
constexpr size_t kCullFrameBytes = 208; // WGSL `CullFrame`
constexpr size_t kCounters = 8;         // cull counters, see kCullWGSL
// Two-pass occlusion culling costs ~0.35 ms a frame where nothing is hidden (an extra render pass, the
// multisampled targets stored and reloaded, the depth pyramid) and saves tens of ms where most of a big
// model is (a 62M-triangle crane: 93 -> 17 ms). Below this many triangles the CPU culls instead.
constexpr uint64_t kOcclusionMinTriangles = 2'000'000;
constexpr WGPUTextureFormat kDepthFormat = WGPUTextureFormat_Depth24Plus;
// Minimum triangles per clustering batch (runs of consecutive segments); see build_clusters. Measured on a
// 1M-triangle / 25k-object frame model: larger batches fill clusters better, 8192 keeps load time low.
constexpr size_t kBatchTriangles = 8192;
constexpr size_t kClusterMinTriangles = 16;

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

bool invert(const Mat4 &m, Mat4 &out) {
    Mat4 inv{};
    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] +
             m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] -
             m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] +
             m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] -
              m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] -
             m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] +
             m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] -
             m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] +
              m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] +
             m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] -
             m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] +
              m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] -
              m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] -
             m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] -
              m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];
    float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (std::fabs(det) < 1e-30f)
        return false;
    for (int i = 0; i < 16; ++i)
        out[i] = inv[i] / det;
    return true;
}

float det3(const Mat4 &m) {
    return m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) +
           m[8] * (m[1] * m[6] - m[5] * m[2]);
}

// The six clip planes of `mvp` in the space `mvp` maps from, normalised so distances come out in that
// space's units. WebGPU clip volume: -w <= x,y <= w, 0 <= z <= w.
std::array<std::array<float, 4>, 6> frustum_planes(const Mat4 &m) {
    auto row = [&](int r) { return std::array<float, 4>{m[r], m[4 + r], m[8 + r], m[12 + r]}; };
    auto r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
    std::array<std::array<float, 4>, 6> p{};
    for (int i = 0; i < 4; ++i) {
        p[0][i] = r3[i] + r0[i];
        p[1][i] = r3[i] - r0[i];
        p[2][i] = r3[i] + r1[i];
        p[3][i] = r3[i] - r1[i];
        p[4][i] = r2[i];
        p[5][i] = r3[i] - r2[i];
    }
    for (auto &pl : p) {
        float len = std::sqrt(pl[0] * pl[0] + pl[1] * pl[1] + pl[2] * pl[2]);
        if (len > 0.0f)
            for (float &v : pl)
                v /= len;
    }
    return p;
}

struct ClusterCull {
    float center[3];
    float radius;
    float axis[3];
    float cutoff;
    float morph_extent; // max |morph displacement| over the cluster's vertices
    uint32_t triangles;
    uint32_t seg_first, seg_last; // the segments its triangles belong to (a contiguous run)
};

struct Range {
    uint32_t start, count;
};

// A triangle by its (sorted) vertices: maps a meshlet's triangles back to their original index.
struct TriKey {
    uint32_t a, b, c;
    bool operator==(const TriKey &o) const {
        return a == o.a && b == o.b && c == o.c;
    }
};
struct TriKeyHash {
    size_t operator()(const TriKey &k) const {
        uint64_t h = k.a * 0x9E3779B97F4A7C15ull;
        h ^= (uint64_t(k.b) + 0x632BE59BD9B4E019ull + (h << 6) + (h >> 2));
        h ^= (uint64_t(k.c) + 0x94D049BB133111EBull + (h << 6) + (h >> 2));
        return size_t(h);
    }
};
TriKey tri_key(uint32_t a, uint32_t b, uint32_t c) {
    if (a > b)
        std::swap(a, b);
    if (b > c)
        std::swap(b, c);
    if (a > b)
        std::swap(a, b);
    return {a, b, c};
}

constexpr WGPUTextureFormat kPickIdFormat = WGPUTextureFormat_RGBA32Uint;
constexpr WGPUTextureFormat kPickPosFormat = WGPUTextureFormat_RGBA32Float;

} // namespace

struct Renderer::Impl {
    struct Geometry {
        Topology topology = Topology::Triangles;
        uint32_t vertex_count = 0;
        uint32_t list_count = 0; // lines/points: index count
        bool has_colors = false;
        bool has_morph = false;
        std::vector<ClusterCull> clusters;
        std::vector<Range> segments;
        uint64_t triangles = 0;
        float aabb[6] = {0, 0, 0, 0, 0, 0};
        float sphere[4] = {0, 0, 0, 0};
        float morph_extent = 0.0f;
        BufferRef vtx, colors, morph, prim;
        BindGroupRef bind;
    };
    struct Instance {
        uint32_t geometry = 0;
        InstanceDesc desc;
        std::vector<uint32_t> segstate; // 2 per segment
        uint32_t id = 0, gpu_index = 0, seg_base = 0, vis_base = 0, vis_count = 0;
    };

    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat color_format;

    BindGroupLayoutRef frame_layout, geom_layout;
    PipelineLayoutRef pipeline_layout;
    ShaderModuleRef shader;
    std::string shader_source;
    RenderPipelineRef tri_double, tri_single, tri_blend, lines, lines_blend, points;
    RenderPipelineRef wide, wide_blend;         // lines / points wider than a pixel, as quads (vs_wide)
    RenderPipelineRef pick_double, pick_single; // id + world-position targets, 1 sample
    BufferRef dummy;

    // GPU culling: per-(instance, cluster) items, last frame's visibility, the early / late visible lists and
    // their indirect draw args, and a depth pyramid (HZB) of the early pass.
    BindGroupLayoutRef cull_layout, hzb_init_layout, hzb_reduce_layout;
    PipelineLayoutRef cull_pipeline_layout, hzb_init_pipeline_layout, hzb_reduce_pipeline_layout;
    ShaderModuleRef cull_shader;
    ComputePipelineRef cull_early, cull_late, hzb_init, hzb_reduce;
    BufferRef cull_frame_buf, items_buf, history_buf, vis_early_buf, vis_late_buf, args_early_buf, args_late_buf;
    BufferRef counters_buf;
    size_t items_cap = 0, history_cap = 0, vis_gpu_cap = 0, args_cap = 0;
    std::vector<uint32_t> args_template;
    uint32_t item_count = 0;
    uint64_t scene_triangles = 0;
    BindGroupRef frame_bind_early, frame_bind_late, cull_bind_early, cull_bind_late, hzb_init_bind;
    std::vector<BindGroupRef> hzb_reduce_binds;
    TextureRef hzb_tex;
    TextureViewRef hzb_view;
    std::vector<TextureViewRef> hzb_mips;
    uint32_t hzb_levels = 0;
    bool cull_binds_dirty = true;

    BufferRef frame_buf, instance_buf, segstate_buf, visible_buf;
    size_t instance_cap = 0, segstate_cap = 0, visible_cap = 0;
    BindGroupRef frame_bind, pick_frame_bind;
    BufferRef pick_frame_buf;
    TextureRef pick_id_tex, pick_pos_tex, pick_depth_tex;
    TextureViewRef pick_id_view, pick_pos_view, pick_depth_view;

    TextureRef msaa_tex, depth_tex;
    TextureViewRef msaa_view, depth_view;
    uint32_t target_w = 0, target_h = 0;

    std::vector<std::unique_ptr<Geometry>> geometries;
    std::vector<std::unique_ptr<Instance>> instances;
    std::vector<Instance *> order; // live instances in GPU order
    std::vector<uint32_t> visible;

    bool structure_dirty = true, instances_dirty = true, segstate_dirty = true;

    Mat4 view{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    Mat4 proj{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    bool ortho = false;
    float clear[4] = {0.2235f, 0.2235f, 0.2235f, 1.0f}; // #393939, the viewer's background
    float ambient = 1.5707964f;                         // the viewer's AmbientLight(0xffffff, PI / 2)
    float diffuse = 1.4f;                               // its camera-tracking DirectionalLight(0xffffff, 1.4)
    bool cull_frustum = true, cull_cone = true, cull_occlusion = true;

    Impl(WGPUDevice dev, WGPUTextureFormat fmt) : device(dev), queue(wgpuDeviceGetQueue(dev)), color_format(fmt) {
        wgpuDeviceAddRef(device);
        create_layouts();
        create_pipelines();
        dummy = make_buffer(16, WGPUBufferUsage_Storage, "adacpp.dummy");
        frame_buf = make_buffer(112, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst, "adacpp.frame");
        pick_frame_buf = make_buffer(112, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst, "adacpp.pick_frame");
        create_pick_targets();
        create_cull();
    }
    ~Impl() {
        geometries.clear();
        instances.clear();
        wgpuQueueRelease(queue);
        wgpuDeviceRelease(device);
    }

    BufferRef make_buffer(size_t size, WGPUBufferUsage usage, const char *label) {
        WGPUBufferDescriptor d = WGPU_BUFFER_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.size = std::max<size_t>(16, (size + 3) & ~size_t(3));
        d.usage = usage;
        return BufferRef(wgpuDeviceCreateBuffer(device, &d));
    }

    BufferRef upload(const void *data, size_t bytes, const char *label) {
        auto b = make_buffer(bytes, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst, label);
        if (bytes)
            wgpuQueueWriteBuffer(queue, b.get(), 0, data, (bytes + 3) & ~size_t(3));
        return b;
    }

    void create_layouts() {
        WGPUBindGroupLayoutEntry fe[4] = {WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT,
                                          WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT};
        for (uint32_t i = 0; i < 4; ++i) {
            fe[i].binding = i;
            fe[i].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
            fe[i].buffer.type = i == 0 ? WGPUBufferBindingType_Uniform : WGPUBufferBindingType_ReadOnlyStorage;
        }
        for (uint32_t i = 1; i < 4; ++i)
            fe[i].visibility = WGPUShaderStage_Vertex;
        WGPUBindGroupLayoutDescriptor fd = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        fd.label = sv("adacpp.frame_layout");
        fd.entryCount = 4;
        fd.entries = fe;
        frame_layout.reset(wgpuDeviceCreateBindGroupLayout(device, &fd));

        WGPUBindGroupLayoutEntry ge[4] = {WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT,
                                          WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT};
        for (uint32_t i = 0; i < 4; ++i) {
            ge[i].binding = i;
            ge[i].visibility = WGPUShaderStage_Vertex;
            ge[i].buffer.type = WGPUBufferBindingType_ReadOnlyStorage;
        }
        WGPUBindGroupLayoutDescriptor gd = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        gd.label = sv("adacpp.geometry_layout");
        gd.entryCount = 4;
        gd.entries = ge;
        geom_layout.reset(wgpuDeviceCreateBindGroupLayout(device, &gd));

        WGPUBindGroupLayout layouts[2] = {frame_layout.get(), geom_layout.get()};
        WGPUPipelineLayoutDescriptor pd = WGPU_PIPELINE_LAYOUT_DESCRIPTOR_INIT;
        pd.label = sv("adacpp.pipeline_layout");
        pd.bindGroupLayoutCount = 2;
        pd.bindGroupLayouts = layouts;
        pipeline_layout.reset(wgpuDeviceCreatePipelineLayout(device, &pd));

        shader_source = kShaderWGSL;
        const std::string token = "{{CLUSTER_VERTS}}";
        shader_source.replace(shader_source.find(token), token.size(), std::to_string(kClusterMaxTriangles * 3));
        WGPUShaderSourceWGSL src = WGPU_SHADER_SOURCE_WGSL_INIT;
        src.code = sv(shader_source);
        WGPUShaderModuleDescriptor sd = WGPU_SHADER_MODULE_DESCRIPTOR_INIT;
        sd.nextInChain = &src.chain;
        sd.label = sv("adacpp.shader");
        shader.reset(wgpuDeviceCreateShaderModule(device, &sd));
    }

    RenderPipelineRef make_pipeline(const char *label, const char *vs, WGPUPrimitiveTopology topo, WGPUCullMode cull,
                                    bool blend, bool depth_bias, const char *fs_entry = "fs_main") {
        WGPUBlendState bs = WGPU_BLEND_STATE_INIT;
        bs.color.srcFactor = WGPUBlendFactor_SrcAlpha;
        bs.color.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
        bs.color.operation = WGPUBlendOperation_Add;
        bs.alpha.srcFactor = WGPUBlendFactor_One;
        bs.alpha.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
        bs.alpha.operation = WGPUBlendOperation_Add;

        WGPUColorTargetState ct = WGPU_COLOR_TARGET_STATE_INIT;
        ct.format = color_format;
        ct.blend = blend ? &bs : nullptr;
        ct.writeMask = WGPUColorWriteMask_All;

        WGPUFragmentState fs = WGPU_FRAGMENT_STATE_INIT;
        fs.module = shader.get();
        fs.entryPoint = sv(fs_entry);
        fs.targetCount = 1;
        fs.targets = &ct;

        WGPUDepthStencilState ds = WGPU_DEPTH_STENCIL_STATE_INIT;
        ds.format = kDepthFormat;
        ds.depthWriteEnabled = blend ? WGPUOptionalBool_False : WGPUOptionalBool_True;
        ds.depthCompare = WGPUCompareFunction_LessEqual;
        if (depth_bias) {
            // Push faces back a little so feature edges drawn on top of them win the depth test (what
            // three.js does with polygonOffset on the CAD materials).
            ds.depthBias = 2;
            ds.depthBiasSlopeScale = 1.5f;
        }

        WGPURenderPipelineDescriptor d = WGPU_RENDER_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.layout = pipeline_layout.get();
        d.vertex.module = shader.get();
        d.vertex.entryPoint = sv(vs);
        d.primitive.topology = topo;
        d.primitive.frontFace = WGPUFrontFace_CCW;
        d.primitive.cullMode = cull;
        d.depthStencil = &ds;
        d.multisample.count = kSampleCount;
        d.multisample.mask = 0xFFFFFFFFu;
        d.fragment = &fs;
        return RenderPipelineRef(wgpuDeviceCreateRenderPipeline(device, &d));
    }

    RenderPipelineRef make_pick_pipeline(const char *label, WGPUCullMode cull) {
        WGPUColorTargetState ct[2] = {WGPU_COLOR_TARGET_STATE_INIT, WGPU_COLOR_TARGET_STATE_INIT};
        ct[0].format = kPickIdFormat;
        ct[0].writeMask = WGPUColorWriteMask_All;
        ct[1].format = kPickPosFormat;
        ct[1].writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState fs = WGPU_FRAGMENT_STATE_INIT;
        fs.module = shader.get();
        fs.entryPoint = sv("fs_pick");
        fs.targetCount = 2;
        fs.targets = ct;
        WGPUDepthStencilState ds = WGPU_DEPTH_STENCIL_STATE_INIT;
        ds.format = kDepthFormat;
        ds.depthWriteEnabled = WGPUOptionalBool_True;
        ds.depthCompare = WGPUCompareFunction_Less;
        WGPURenderPipelineDescriptor d = WGPU_RENDER_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.layout = pipeline_layout.get();
        d.vertex.module = shader.get();
        d.vertex.entryPoint = sv("vs_tri");
        d.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        d.primitive.frontFace = WGPUFrontFace_CCW;
        d.primitive.cullMode = cull;
        d.depthStencil = &ds;
        d.multisample.count = 1;
        d.multisample.mask = 0xFFFFFFFFu;
        d.fragment = &fs;
        return RenderPipelineRef(wgpuDeviceCreateRenderPipeline(device, &d));
    }

    void create_pick_targets() {
        auto make = [&](WGPUTextureFormat fmt, WGPUTextureUsage usage, TextureRef &tex, TextureViewRef &view,
                        const char *label) {
            WGPUTextureDescriptor d = WGPU_TEXTURE_DESCRIPTOR_INIT;
            d.label = sv(label);
            d.size = {1, 1, 1};
            d.format = fmt;
            d.usage = usage;
            tex.reset(wgpuDeviceCreateTexture(device, &d));
            view.reset(wgpuTextureCreateView(tex.get(), nullptr));
        };
        const auto readable = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc;
        make(kPickIdFormat, readable, pick_id_tex, pick_id_view, "adacpp.pick_id");
        make(kPickPosFormat, readable, pick_pos_tex, pick_pos_view, "adacpp.pick_pos");
        make(kDepthFormat, WGPUTextureUsage_RenderAttachment, pick_depth_tex, pick_depth_view, "adacpp.pick_depth");
    }

    void create_pipelines() {
        const auto tri = WGPUPrimitiveTopology_TriangleList;
        tri_double = make_pipeline("adacpp.tri_double", "vs_tri", tri, WGPUCullMode_None, false, true);
        tri_single = make_pipeline("adacpp.tri_single", "vs_tri", tri, WGPUCullMode_Back, false, true);
        tri_blend = make_pipeline("adacpp.tri_blend", "vs_tri", tri, WGPUCullMode_None, true, true);
        lines =
            make_pipeline("adacpp.lines", "vs_list", WGPUPrimitiveTopology_LineList, WGPUCullMode_None, false, false);
        lines_blend = make_pipeline("adacpp.lines_blend", "vs_list", WGPUPrimitiveTopology_LineList, WGPUCullMode_None,
                                    true, false);
        points =
            make_pipeline("adacpp.points", "vs_list", WGPUPrimitiveTopology_PointList, WGPUCullMode_None, false, false);
        wide = make_pipeline("adacpp.wide", "vs_wide", tri, WGPUCullMode_None, false, false, "fs_disc");
        wide_blend = make_pipeline("adacpp.wide_blend", "vs_wide", tri, WGPUCullMode_None, true, false, "fs_disc");
        pick_double = make_pick_pipeline("adacpp.pick_double", WGPUCullMode_None);
        pick_single = make_pick_pipeline("adacpp.pick_single", WGPUCullMode_Back);
    }

    BindGroupRef make_frame_bind(WGPUBuffer frame, WGPUBuffer vis, const char *label) {
        WGPUBindGroupEntry e[4] = {WGPU_BIND_GROUP_ENTRY_INIT, WGPU_BIND_GROUP_ENTRY_INIT, WGPU_BIND_GROUP_ENTRY_INIT,
                                   WGPU_BIND_GROUP_ENTRY_INIT};
        WGPUBuffer bufs[4] = {frame, instance_buf.get(), segstate_buf.get(), vis};
        for (uint32_t i = 0; i < 4; ++i) {
            e[i].binding = i;
            e[i].buffer = bufs[i];
            e[i].size = wgpuBufferGetSize(bufs[i]);
        }
        WGPUBindGroupDescriptor d = WGPU_BIND_GROUP_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.layout = frame_layout.get();
        d.entryCount = 4;
        d.entries = e;
        return BindGroupRef(wgpuDeviceCreateBindGroup(device, &d));
    }

    BindGroupLayoutRef make_layout(const char *label, std::vector<WGPUBindGroupLayoutEntry> entries) {
        for (uint32_t i = 0; i < entries.size(); ++i) {
            entries[i].binding = i;
            entries[i].visibility = WGPUShaderStage_Compute;
        }
        WGPUBindGroupLayoutDescriptor d = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.entryCount = entries.size();
        d.entries = entries.data();
        return BindGroupLayoutRef(wgpuDeviceCreateBindGroupLayout(device, &d));
    }

    ComputePipelineRef make_compute(const char *entry, const BindGroupLayoutRef &layout, PipelineLayoutRef &pl) {
        if (!pl) {
            WGPUBindGroupLayout l = layout.get();
            WGPUPipelineLayoutDescriptor pd = WGPU_PIPELINE_LAYOUT_DESCRIPTOR_INIT;
            pd.bindGroupLayoutCount = 1;
            pd.bindGroupLayouts = &l;
            pl.reset(wgpuDeviceCreatePipelineLayout(device, &pd));
        }
        WGPUComputePipelineDescriptor d = WGPU_COMPUTE_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(entry);
        d.layout = pl.get();
        d.compute.module = cull_shader.get();
        d.compute.entryPoint = sv(entry);
        return ComputePipelineRef(wgpuDeviceCreateComputePipeline(device, &d));
    }

    void create_cull() {
        auto buffer = [](WGPUBufferBindingType type) {
            WGPUBindGroupLayoutEntry e = WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT;
            e.buffer.type = type;
            return e;
        };
        auto texture = [](WGPUTextureSampleType type, bool multisampled) {
            WGPUBindGroupLayoutEntry e = WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT;
            e.texture.sampleType = type;
            e.texture.viewDimension = WGPUTextureViewDimension_2D;
            e.texture.multisampled = multisampled;
            return e;
        };
        WGPUBindGroupLayoutEntry store = WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT;
        store.storageTexture.access = WGPUStorageTextureAccess_WriteOnly;
        store.storageTexture.format = WGPUTextureFormat_R32Float;
        store.storageTexture.viewDimension = WGPUTextureViewDimension_2D;
        const auto ro = WGPUBufferBindingType_ReadOnlyStorage, rw = WGPUBufferBindingType_Storage;
        cull_layout = make_layout("adacpp.cull_layout", {buffer(WGPUBufferBindingType_Uniform), buffer(ro), buffer(ro),
                                                         buffer(ro), buffer(rw), buffer(rw), buffer(rw), buffer(rw),
                                                         texture(WGPUTextureSampleType_UnfilterableFloat, false)});
        hzb_init_layout = make_layout("adacpp.hzb_init_layout", {texture(WGPUTextureSampleType_Depth, true), store});
        hzb_reduce_layout =
            make_layout("adacpp.hzb_reduce_layout", {texture(WGPUTextureSampleType_UnfilterableFloat, false), store});

        std::string src_text = kCullWGSL;
        const std::string token = "{{CLUSTER_VERTS}}";
        src_text.replace(src_text.find(token), token.size(), std::to_string(kClusterMaxTriangles * 3));
        WGPUShaderSourceWGSL src = WGPU_SHADER_SOURCE_WGSL_INIT;
        src.code = sv(src_text);
        WGPUShaderModuleDescriptor sd = WGPU_SHADER_MODULE_DESCRIPTOR_INIT;
        sd.nextInChain = &src.chain;
        sd.label = sv("adacpp.cull_shader");
        cull_shader.reset(wgpuDeviceCreateShaderModule(device, &sd));
        cull_early = make_compute("cs_early", cull_layout, cull_pipeline_layout);
        cull_late = make_compute("cs_late", cull_layout, cull_pipeline_layout);
        hzb_init = make_compute("hzb_init", hzb_init_layout, hzb_init_pipeline_layout);
        hzb_reduce = make_compute("hzb_reduce", hzb_reduce_layout, hzb_reduce_pipeline_layout);

        cull_frame_buf =
            make_buffer(kCullFrameBytes, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst, "adacpp.cull_frame");
        counters_buf =
            make_buffer(kCounters * 4, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst | WGPUBufferUsage_CopySrc,
                        "adacpp.cull_counters");
    }

    BindGroupRef make_bind(const BindGroupLayoutRef &layout, std::vector<WGPUBindGroupEntry> e, const char *label) {
        for (uint32_t i = 0; i < e.size(); ++i)
            e[i].binding = i;
        WGPUBindGroupDescriptor d = WGPU_BIND_GROUP_DESCRIPTOR_INIT;
        d.label = sv(label);
        d.layout = layout.get();
        d.entryCount = e.size();
        d.entries = e.data();
        return BindGroupRef(wgpuDeviceCreateBindGroup(device, &d));
    }
    static WGPUBindGroupEntry buffer_entry(WGPUBuffer b) {
        WGPUBindGroupEntry e = WGPU_BIND_GROUP_ENTRY_INIT;
        e.buffer = b;
        e.size = wgpuBufferGetSize(b);
        return e;
    }
    static WGPUBindGroupEntry view_entry(WGPUTextureView v) {
        WGPUBindGroupEntry e = WGPU_BIND_GROUP_ENTRY_INIT;
        e.textureView = v;
        return e;
    }

    // The depth pyramid for a w x h target: level 0 at full resolution, halving (rounded down) to 1 x 1.
    void create_hzb(uint32_t w, uint32_t h) {
        hzb_levels = 1;
        while ((std::max(w, h) >> hzb_levels) > 0)
            ++hzb_levels;
        WGPUTextureDescriptor d = WGPU_TEXTURE_DESCRIPTOR_INIT;
        d.label = sv("adacpp.hzb");
        d.size = {w, h, 1};
        d.format = WGPUTextureFormat_R32Float;
        d.mipLevelCount = hzb_levels;
        d.usage = WGPUTextureUsage_StorageBinding | WGPUTextureUsage_TextureBinding;
        hzb_tex.reset(wgpuDeviceCreateTexture(device, &d));
        hzb_view.reset(wgpuTextureCreateView(hzb_tex.get(), nullptr));
        hzb_mips.clear();
        for (uint32_t l = 0; l < hzb_levels; ++l) {
            WGPUTextureViewDescriptor vd = WGPU_TEXTURE_VIEW_DESCRIPTOR_INIT;
            vd.baseMipLevel = l;
            vd.mipLevelCount = 1;
            hzb_mips.emplace_back(wgpuTextureCreateView(hzb_tex.get(), &vd));
        }
        hzb_init_bind = make_bind(hzb_init_layout, {view_entry(depth_view.get()), view_entry(hzb_mips[0].get())},
                                  "adacpp.hzb_init");
        hzb_reduce_binds.clear();
        for (uint32_t l = 1; l < hzb_levels; ++l)
            hzb_reduce_binds.push_back(make_bind(
                hzb_reduce_layout, {view_entry(hzb_mips[l - 1].get()), view_entry(hzb_mips[l].get())}, "adacpp.hzb"));
        cull_binds_dirty = true;
    }

    void create_cull_binds() {
        auto make = [&](const BufferRef &vis, const BufferRef &args, const char *label) {
            return make_bind(cull_layout,
                             {buffer_entry(cull_frame_buf.get()), buffer_entry(instance_buf.get()),
                              buffer_entry(segstate_buf.get()), buffer_entry(items_buf.get()),
                              buffer_entry(history_buf.get()), buffer_entry(vis.get()), buffer_entry(args.get()),
                              buffer_entry(counters_buf.get()), view_entry(hzb_view.get())},
                             label);
        };
        cull_bind_early = make(vis_early_buf, args_early_buf, "adacpp.cull_early");
        cull_bind_late = make(vis_late_buf, args_late_buf, "adacpp.cull_late");
        cull_binds_dirty = false;
    }

    void bind_geometry(Geometry &g) {
        WGPUBindGroupEntry e[4] = {WGPU_BIND_GROUP_ENTRY_INIT, WGPU_BIND_GROUP_ENTRY_INIT, WGPU_BIND_GROUP_ENTRY_INIT,
                                   WGPU_BIND_GROUP_ENTRY_INIT};
        WGPUBuffer bufs[4] = {g.vtx.get(), g.has_colors ? g.colors.get() : dummy.get(),
                              g.has_morph ? g.morph.get() : dummy.get(), g.prim.get()};
        for (uint32_t i = 0; i < 4; ++i) {
            e[i].binding = i;
            e[i].buffer = bufs[i];
            e[i].offset = 0;
            e[i].size = wgpuBufferGetSize(bufs[i]);
        }
        WGPUBindGroupDescriptor d = WGPU_BIND_GROUP_DESCRIPTOR_INIT;
        d.label = sv("adacpp.geometry");
        d.layout = geom_layout.get();
        d.entryCount = 4;
        d.entries = e;
        g.bind.reset(wgpuDeviceCreateBindGroup(device, &d));
    }

    // Build the cluster records + vertex lists + packed triangles for one triangle geometry.
    void build_clusters(Geometry &g, const GeometryDesc &desc, const std::vector<uint32_t> &indices,
                        std::vector<uint32_t> &prim) {
        const float *pos = desc.positions;
        const size_t nv = desc.vertex_count;

        // The segment of every triangle: clusters may span several (small) segments, so hiding and
        // per-object colour are resolved per triangle.
        const size_t tri_total = indices.size() / 3;
        std::vector<uint32_t> tri_segment(tri_total, 0);
        for (uint32_t s = 0; s < g.segments.size(); ++s) {
            const Range r = g.segments[s];
            for (size_t t = r.start / 3; t < (size_t(r.start) + r.count) / 3 && t < tri_total; ++t)
                tri_segment[t] = s;
        }

        // Batches: runs of consecutive segments of at least kBatchTriangles (a large segment is a batch on
        // its own). Clustering a batch instead of each segment keeps clusters full on models of many tiny
        // objects (beams average ~20 triangles), where per-segment clusters were ~17% occupied.
        std::vector<Range> batches;
        for (const Range r : g.segments) {
            if (r.count < 3)
                continue;
            const bool contiguous = !batches.empty() && batches.back().start + batches.back().count == r.start;
            if (contiguous && batches.back().count / 3 < kBatchTriangles)
                batches.back().count += r.count;
            else
                batches.push_back(r);
        }

        std::vector<uint32_t> all_vertices, all_triangles; // per triangle: packed, original index, segment
        std::vector<meshopt_Meshlet> meshlets;
        std::vector<unsigned int> mv;
        std::vector<unsigned char> mt;
        std::unordered_map<TriKey, uint32_t, TriKeyHash> tri_lookup;
        for (const Range r : batches) {
            const unsigned int *idx = indices.data() + r.start;
            const size_t count = r.count - r.count % 3;
            // Meshlet building reorders triangles; remember where each came from (picking, segment).
            tri_lookup.clear();
            tri_lookup.reserve(count / 3);
            for (uint32_t t = 0; t < count / 3; ++t)
                tri_lookup.emplace(tri_key(idx[t * 3], idx[t * 3 + 1], idx[t * 3 + 2]), r.start / 3 + t);
            const size_t max_m = meshopt_buildMeshletsBound(count, kClusterMaxVertices, kClusterMinTriangles);
            meshlets.resize(max_m);
            mv.resize(max_m * kClusterMaxVertices);
            mt.resize(max_m * kClusterMaxTriangles * 3);
            // Spatial builder: compact clusters whatever the triangle order, so frustum culling stays
            // precise when a batch spans many objects (the order-based scan builder made clusters that
            // joined distant objects: 3x more triangles drawn in a close-up).
            size_t n = meshopt_buildMeshletsSpatial(meshlets.data(), mv.data(), mt.data(), idx, count, pos, nv,
                                                    sizeof(float) * 3, kClusterMaxVertices, kClusterMinTriangles,
                                                    kClusterMaxTriangles, 0.5f);
            for (size_t m = 0; m < n; ++m) {
                const meshopt_Meshlet &ml = meshlets[m];
                meshopt_Bounds b = meshopt_computeMeshletBounds(&mv[ml.vertex_offset], &mt[ml.triangle_offset],
                                                                ml.triangle_count, pos, nv, sizeof(float) * 3);
                ClusterCull c{};
                for (int k = 0; k < 3; ++k) {
                    c.center[k] = b.center[k];
                    c.axis[k] = b.cone_axis[k];
                }
                c.radius = b.radius;
                c.cutoff = b.cone_cutoff;
                c.triangles = ml.triangle_count;
                c.seg_first = UINT32_MAX;
                c.seg_last = 0;
                if (desc.morph) {
                    float ext = 0.0f;
                    for (uint32_t v = 0; v < ml.vertex_count; ++v) {
                        const size_t vi = mv[ml.vertex_offset + v];
                        float d2 = 0.0f;
                        for (int k = 0; k < 3; ++k) {
                            float d = desc.morph[vi * 3 + k];
                            d2 += d * d;
                        }
                        ext = std::max(ext, d2);
                    }
                    c.morph_extent = std::sqrt(ext);
                }
                // GPU record: vertex offset / triangle offset are patched once the sections are laid out.
                prim.push_back(uint32_t(all_vertices.size()));
                prim.push_back(uint32_t(all_triangles.size()));
                prim.push_back(ml.triangle_count);
                prim.push_back(0);
                for (uint32_t v = 0; v < ml.vertex_count; ++v)
                    all_vertices.push_back(mv[ml.vertex_offset + v]);
                const unsigned int *mvp = &mv[ml.vertex_offset];
                for (uint32_t t = 0; t < ml.triangle_count; ++t) {
                    const unsigned char *tri = &mt[ml.triangle_offset + t * 3];
                    auto it = tri_lookup.find(tri_key(mvp[tri[0]], mvp[tri[1]], mvp[tri[2]]));
                    const uint32_t orig = it != tri_lookup.end() ? it->second : 0xFFFFFFFFu;
                    const uint32_t seg = orig < tri_total ? tri_segment[orig] : tri_segment[r.start / 3];
                    all_triangles.push_back(uint32_t(tri[0]) | (uint32_t(tri[1]) << 8) | (uint32_t(tri[2]) << 16));
                    all_triangles.push_back(orig);
                    all_triangles.push_back(seg);
                    c.seg_first = std::min(c.seg_first, seg);
                    c.seg_last = std::max(c.seg_last, seg);
                }
                g.clusters.push_back(c);
                g.triangles += ml.triangle_count;
            }
        }
        const uint32_t vbase = uint32_t(prim.size());
        const uint32_t tbase = vbase + uint32_t(all_vertices.size());
        for (size_t c = 0; c < g.clusters.size(); ++c) {
            prim[c * 4] += vbase;
            prim[c * 4 + 1] += tbase;
        }
        prim.insert(prim.end(), all_vertices.begin(), all_vertices.end());
        prim.insert(prim.end(), all_triangles.begin(), all_triangles.end());
    }

    uint32_t add_geometry(const GeometryDesc &desc) {
        if (!desc.positions || desc.vertex_count == 0)
            throw std::invalid_argument("adacpp.render: geometry without positions");
        auto g = std::make_unique<Geometry>();
        g->topology = desc.topology;
        g->vertex_count = uint32_t(desc.vertex_count);

        std::vector<uint32_t> indices;
        if (desc.indices) {
            indices.assign(desc.indices, desc.indices + desc.index_count);
        } else {
            indices.resize(desc.vertex_count);
            for (uint32_t i = 0; i < desc.vertex_count; ++i)
                indices[i] = i;
        }
        if (desc.segments && desc.segment_count) {
            for (size_t s = 0; s < desc.segment_count; ++s) {
                uint32_t start = desc.segments[s * 2], count = desc.segments[s * 2 + 1];
                start = std::min<uint32_t>(start, uint32_t(indices.size()));
                count = std::min<uint32_t>(count, uint32_t(indices.size()) - start);
                g->segments.push_back({start, count});
            }
        } else {
            g->segments.push_back({0, uint32_t(indices.size())});
        }

        // Interleaved position + normal; a zero normal tells the shader to derive a flat one.
        std::vector<float> vtx(desc.vertex_count * 6, 0.0f);
        float lo[3] = {INFINITY, INFINITY, INFINITY}, hi[3] = {-INFINITY, -INFINITY, -INFINITY};
        for (size_t v = 0; v < desc.vertex_count; ++v) {
            for (int k = 0; k < 3; ++k) {
                float p = desc.positions[v * 3 + k];
                vtx[v * 6 + k] = p;
                lo[k] = std::min(lo[k], p);
                hi[k] = std::max(hi[k], p);
                if (desc.normals)
                    vtx[v * 6 + 3 + k] = desc.normals[v * 3 + k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            g->aabb[k] = lo[k];
            g->aabb[3 + k] = hi[k];
            g->sphere[k] = 0.5f * (lo[k] + hi[k]);
        }
        g->sphere[3] = 0.5f * std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                        (hi[2] - lo[2]) * (hi[2] - lo[2]));
        g->vtx = upload(vtx.data(), vtx.size() * sizeof(float), "adacpp.vtx");

        if (desc.colors) {
            g->colors = upload(desc.colors, desc.vertex_count * sizeof(uint32_t), "adacpp.colors");
            g->has_colors = true;
        }
        if (desc.morph) {
            g->morph = upload(desc.morph, desc.vertex_count * 3 * sizeof(float), "adacpp.morph");
            g->has_morph = true;
            float ext = 0.0f;
            for (size_t v = 0; v < desc.vertex_count; ++v) {
                float d2 = 0.0f;
                for (int k = 0; k < 3; ++k)
                    d2 += desc.morph[v * 3 + k] * desc.morph[v * 3 + k];
                ext = std::max(ext, d2);
            }
            g->morph_extent = std::sqrt(ext);
        }

        std::vector<uint32_t> prim;
        if (desc.topology == Topology::Triangles) {
            build_clusters(*g, desc, indices, prim);
        } else {
            // Index list, then the segment of each index entry (vs_list reads it to drop hidden segments).
            g->list_count = uint32_t(indices.size());
            prim = std::move(indices);
            prim.resize(size_t(g->list_count) * 2, 0);
            for (uint32_t s = 0; s < g->segments.size(); ++s) {
                const Range r = g->segments[s];
                for (uint32_t i = r.start; i < r.start + r.count && i < g->list_count; ++i)
                    prim[g->list_count + i] = s;
            }
        }
        g->prim = upload(prim.data(), prim.size() * sizeof(uint32_t), "adacpp.prim");
        bind_geometry(*g);

        for (uint32_t id = 0; id < geometries.size(); ++id) {
            if (!geometries[id]) {
                geometries[id] = std::move(g);
                return id;
            }
        }
        geometries.push_back(std::move(g));
        return uint32_t(geometries.size() - 1);
    }

    Geometry &geometry(uint32_t id) {
        if (id >= geometries.size() || !geometries[id])
            throw std::out_of_range("adacpp.render: unknown geometry " + std::to_string(id));
        return *geometries[id];
    }
    Instance &instance(uint32_t id) {
        if (id >= instances.size() || !instances[id])
            throw std::out_of_range("adacpp.render: unknown instance " + std::to_string(id));
        return *instances[id];
    }

    void ensure_targets(uint32_t w, uint32_t h) {
        if (w == target_w && h == target_h && msaa_tex)
            return;
        target_w = w;
        target_h = h;
        auto make = [&](WGPUTextureFormat fmt, TextureRef &tex, TextureViewRef &view, const char *label) {
            WGPUTextureDescriptor d = WGPU_TEXTURE_DESCRIPTOR_INIT;
            d.label = sv(label);
            d.size = {w, h, 1};
            d.format = fmt;
            d.sampleCount = kSampleCount;
            // Depth also feeds the occlusion pyramid.
            d.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
            tex.reset(wgpuDeviceCreateTexture(device, &d));
            view.reset(wgpuTextureCreateView(tex.get(), nullptr));
        };
        make(color_format, msaa_tex, msaa_view, "adacpp.msaa");
        make(kDepthFormat, depth_tex, depth_view, "adacpp.depth");
        create_hzb(w, h);
    }

    void rebuild_structure() {
        order.clear();
        uint32_t seg_total = 0, vis_total = 0;
        for (auto &inst : instances) {
            if (!inst)
                continue;
            Geometry &g = geometry(inst->geometry);
            inst->gpu_index = uint32_t(order.size());
            inst->seg_base = seg_total;
            inst->vis_base = vis_total;
            seg_total += uint32_t(g.segments.size());
            vis_total += uint32_t(g.clusters.size());
            order.push_back(inst.get());
        }
        visible.assign(size_t(vis_total) * 2, 0);

        auto grow = [&](BufferRef &buf, size_t &cap, size_t need, WGPUBufferUsage usage, const char *label) {
            if (need <= cap && buf)
                return false;
            cap = std::max<size_t>(need + need / 2, 256);
            buf = make_buffer(cap, usage, label);
            return true;
        };
        const auto st = WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst;
        bool rebind = !frame_bind;
        rebind |= grow(instance_buf, instance_cap, order.size() * kInstanceBytes, st, "adacpp.instances");
        rebind |= grow(segstate_buf, segstate_cap, size_t(seg_total) * 8, st, "adacpp.segstate");
        rebind |= grow(visible_buf, visible_cap, size_t(vis_total) * 8, st, "adacpp.visible");
        rebind |= grow(items_buf, items_cap, size_t(vis_total) * kItemWords * 4, st, "adacpp.cull_items");
        rebind |= grow(history_buf, history_cap, size_t(vis_total) * 4, st, "adacpp.cull_history");
        // Early / late pairs share a capacity.
        if (grow(vis_late_buf, vis_gpu_cap, size_t(vis_total) * 8, st, "adacpp.visible_late")) {
            vis_early_buf = make_buffer(vis_gpu_cap, st, "adacpp.visible_early");
            rebind = true;
        }
        const auto args_usage = st | WGPUBufferUsage_Indirect;
        if (grow(args_late_buf, args_cap, order.size() * 16, args_usage, "adacpp.args_late")) {
            args_early_buf = make_buffer(args_cap, args_usage, "adacpp.args_early");
            rebind = true;
        }
        if (rebind) {
            frame_bind = make_frame_bind(frame_buf.get(), visible_buf.get(), "adacpp.frame");
            pick_frame_bind = make_frame_bind(pick_frame_buf.get(), visible_buf.get(), "adacpp.pick_frame");
            frame_bind_early = make_frame_bind(frame_buf.get(), vis_early_buf.get(), "adacpp.frame_early");
            frame_bind_late = make_frame_bind(frame_buf.get(), vis_late_buf.get(), "adacpp.frame_late");
            cull_binds_dirty = true;
        }

        // Cull items, one per (instance, cluster) in visible-region order, and the draw args each frame
        // starts from. Last frame's visibility no longer lines up: start from "nothing drawn", so the first
        // frame after a change is drawn by the late pass alone.
        std::vector<uint32_t> items(size_t(vis_total) * kItemWords, 0);
        args_template.assign(order.size() * 4, 0);
        for (Instance *inst : order) {
            const Geometry &g = geometry(inst->geometry);
            uint32_t *a = args_template.data() + inst->gpu_index * 4;
            a[1] = 1;
            a[2] = inst->vis_base * kClusterMaxTriangles * 3;
            for (uint32_t c = 0; c < g.clusters.size(); ++c) {
                const ClusterCull &cl = g.clusters[c];
                uint32_t *rec = items.data() + size_t(inst->vis_base + c) * kItemWords;
                const float f[9] = {cl.center[0], cl.center[1], cl.center[2], cl.radius, cl.morph_extent,
                                    cl.axis[0],   cl.axis[1],   cl.axis[2],   cl.cutoff};
                std::memcpy(rec, f, 16);
                std::memcpy(rec + 4, f + 5, 16);
                std::memcpy(rec + 8, f + 4, 4);
                rec[9] = inst->gpu_index;
                rec[10] = c;
                rec[11] = cl.triangles;
                rec[12] = cl.seg_first;
                rec[13] = cl.seg_last;
            }
        }
        item_count = vis_total;
        scene_triangles = 0;
        for (Instance *inst : order)
            scene_triangles += geometry(inst->geometry).triangles;
        if (!items.empty()) {
            wgpuQueueWriteBuffer(queue, items_buf.get(), 0, items.data(), items.size() * 4);
            const std::vector<uint32_t> zeros(vis_total, 0);
            wgpuQueueWriteBuffer(queue, history_buf.get(), 0, zeros.data(), zeros.size() * 4);
        }
        structure_dirty = false;
        instances_dirty = true;
        segstate_dirty = true;
    }

    void upload_instances() {
        constexpr size_t words = kInstanceBytes / 4;
        std::vector<uint32_t> data(order.size() * words, 0);
        for (Instance *inst : order) {
            const Geometry &g = geometry(inst->geometry);
            uint32_t *rec = data.data() + inst->gpu_index * words;
            std::memcpy(rec, inst->desc.model.data(), 64);
            std::memcpy(rec + 16, inst->desc.color.data(), 16);
            uint32_t flags = inst->desc.flags;
            if (g.has_morph && inst->desc.morph_influence != 0.0f)
                flags |= kHasMorph;
            if (g.topology != Topology::Triangles)
                flags |= kUnlit;
            if (!g.has_colors)
                flags &= ~uint32_t(kVertexColors);
            if (cone_exact(*inst, g))
                flags |= kConeOk;
            rec[20] = flags;
            rec[21] = inst->seg_base;
            std::memcpy(rec + 22, &inst->desc.morph_influence, 4);
            rec[23] = inst->vis_base;
            float material[3] = {inst->desc.metalness, inst->desc.roughness, inst->desc.specular ? 1.0f : 0.0f};
            if (g.topology != Topology::Triangles) {
                material[0] = inst->desc.line_width;
                if (g.topology == Topology::Points)
                    rec[20] |= kPoints;
            }
            std::memcpy(rec + 24, material, 12);
            rec[27] = g.list_count; // lines/points: where the per-index segment ids start in `prim`
        }
        if (!data.empty())
            wgpuQueueWriteBuffer(queue, instance_buf.get(), 0, data.data(), data.size() * 4);
        instances_dirty = false;
    }

    // Cone culling on the GPU tests in world space, which is exact only for single-sided, unmorphed geometry
    // under a rotation + uniform scale with positive determinant.
    static bool cone_exact(const Instance &inst, const Geometry &g) {
        if (inst.desc.flags & (kDoubleSided | kTransparent))
            return false;
        if (g.has_morph && inst.desc.morph_influence != 0.0f)
            return false;
        const Mat4 &m = inst.desc.model;
        if (det3(m) <= 0.0f)
            return false;
        float len[3];
        for (int c = 0; c < 3; ++c)
            len[c] = std::sqrt(m[c * 4] * m[c * 4] + m[c * 4 + 1] * m[c * 4 + 1] + m[c * 4 + 2] * m[c * 4 + 2]);
        const float tol = 1e-4f * len[0];
        return std::fabs(len[1] - len[0]) <= tol && std::fabs(len[2] - len[0]) <= tol;
    }

    // The instance's bounding sphere is inside or crossing the frustum `planes` (world space).
    bool instance_in_view(const Instance &inst, const std::array<std::array<float, 4>, 6> &planes) const {
        const Geometry &g = *geometries[inst.geometry];
        const Mat4 &m = inst.desc.model;
        float c[3], scale = 0.0f;
        for (int k = 0; k < 3; ++k) {
            c[k] = m[k] * g.sphere[0] + m[4 + k] * g.sphere[1] + m[8 + k] * g.sphere[2] + m[12 + k];
            scale = std::max(scale, m[k * 4] * m[k * 4] + m[k * 4 + 1] * m[k * 4 + 1] + m[k * 4 + 2] * m[k * 4 + 2]);
        }
        float r = g.sphere[3];
        if (g.has_morph && inst.desc.morph_influence != 0.0f)
            r += std::fabs(inst.desc.morph_influence) * g.morph_extent;
        r *= std::sqrt(scale);
        for (const auto &p : planes)
            if (p[0] * c[0] + p[1] * c[1] + p[2] * c[2] + p[3] < -r)
                return false;
        return true;
    }

    void upload_segstate() {
        std::vector<uint32_t> data;
        for (Instance *inst : order)
            data.insert(data.end(), inst->segstate.begin(), inst->segstate.end());
        if (!data.empty())
            wgpuQueueWriteBuffer(queue, segstate_buf.get(), 0, data.data(), data.size() * 4);
        segstate_dirty = false;
    }

    // Cull one triangle instance's clusters into its region of `visible`.
    void cull(Instance &inst, const Mat4 &view_proj, const float cam_world[3], RenderStats &st) {
        const Geometry &g = geometry(inst.geometry);
        const Mat4 &model = inst.desc.model;
        const Mat4 mvp = mul(view_proj, model);
        const auto planes = frustum_planes(mvp);
        const bool morph = g.has_morph && inst.desc.morph_influence != 0.0f;
        const float morph_scale = morph ? std::fabs(inst.desc.morph_influence) : 0.0f;

        // Whole-geometry sphere first: most instances are entirely in or out.
        bool all_inside = true;
        if (cull_frustum) {
            const float r = g.sphere[3] + morph_scale * g.morph_extent;
            for (const auto &p : planes) {
                float d = p[0] * g.sphere[0] + p[1] * g.sphere[1] + p[2] * g.sphere[2] + p[3];
                if (d < -r) {
                    inst.vis_count = 0;
                    return;
                }
                if (d < r)
                    all_inside = false;
            }
        }

        const bool cone =
            cull_cone && !ortho && !morph && !(inst.desc.flags & (kDoubleSided | kTransparent)) && det3(model) > 0.0f;
        float cam[3] = {0, 0, 0};
        if (cone) {
            Mat4 inv;
            if (invert(model, inv)) {
                for (int k = 0; k < 3; ++k)
                    cam[k] =
                        inv[k] * cam_world[0] + inv[4 + k] * cam_world[1] + inv[8 + k] * cam_world[2] + inv[12 + k];
            }
        }
        const uint32_t *seg = inst.segstate.data();
        uint32_t *out = visible.data() + size_t(inst.vis_base) * 2;
        uint32_t n = 0;
        for (uint32_t c = 0; c < g.clusters.size(); ++c) {
            const ClusterCull &cl = g.clusters[c];
            // Culled only when every segment in it is hidden; the shader drops hidden triangles of the rest.
            bool any_visible = false;
            for (uint32_t sg = cl.seg_first; sg <= cl.seg_last && !any_visible; ++sg)
                any_visible = !(seg[sg * 2 + 1] & kSegmentHidden);
            if (!any_visible)
                continue;
            if (cull_frustum && !all_inside) {
                const float r = cl.radius + morph_scale * cl.morph_extent;
                bool out_of_view = false;
                for (const auto &p : planes) {
                    if (p[0] * cl.center[0] + p[1] * cl.center[1] + p[2] * cl.center[2] + p[3] < -r) {
                        out_of_view = true;
                        break;
                    }
                }
                if (out_of_view)
                    continue;
            }
            if (cone) {
                const float d[3] = {cl.center[0] - cam[0], cl.center[1] - cam[1], cl.center[2] - cam[2]};
                const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                if (d[0] * cl.axis[0] + d[1] * cl.axis[1] + d[2] * cl.axis[2] >= cl.cutoff * len + cl.radius)
                    continue;
            }
            out[n * 2] = inst.gpu_index;
            out[n * 2 + 1] = c;
            ++n;
            st.triangles_drawn += cl.triangles;
        }
        inst.vis_count = n;
        st.clusters_drawn += n;
    }

    // view x projection in WebGPU clip space: OpenGL clip z [-w, w] -> [0, w], z' = 0.5 z + 0.5 w.
    Mat4 webgpu_view_proj() const {
        Mat4 proj_wgpu = proj;
        for (int c = 0; c < 4; ++c)
            proj_wgpu[c * 4 + 2] = 0.5f * proj[c * 4 + 2] + 0.5f * proj[c * 4 + 3];
        return mul(proj_wgpu, view);
    }

    // The Frame uniform for `view_proj` into `buf`; returns the camera position in `cam`.
    void write_frame(WGPUBuffer buf, const Mat4 &view_proj, float cam[3]) {
        float fwd[3];
        for (int i = 0; i < 3; ++i) {
            cam[i] = -(view[i * 4 + 0] * view[12] + view[i * 4 + 1] * view[13] + view[i * 4 + 2] * view[14]);
            fwd[i] = -view[i * 4 + 2];
        }
        float frame[28] = {};
        std::memcpy(frame, view_proj.data(), 64);
        frame[16] = cam[0];
        frame[17] = cam[1];
        frame[18] = cam[2];
        frame[19] = ortho ? 0.0f : 1.0f;
        frame[20] = fwd[0];
        frame[21] = fwd[1];
        frame[22] = fwd[2];
        frame[24] = ambient;
        frame[25] = diffuse;
        frame[26] = float(std::max(target_w, 1u));
        frame[27] = float(std::max(target_h, 1u));
        wgpuQueueWriteBuffer(queue, buf, 0, frame, sizeof(frame));
    }

    struct PendingPick {
        BufferRef readback;
        std::vector<uint32_t> ids; // instance id by GPU index, as of the pick
        std::function<void(const PickResult &)> done;
    };

    void pick(uint32_t px, uint32_t py, uint32_t w, uint32_t h, std::function<void(const PickResult &)> done) {
        if (w == 0 || h == 0 || px >= w || py >= h) {
            done(PickResult{});
            return;
        }
        if (structure_dirty)
            rebuild_structure();
        if (instances_dirty)
            upload_instances();
        if (segstate_dirty)
            upload_segstate();

        // Pick matrix: scale + shift clip space so the clicked pixel fills the 1x1 target.
        const float cx = 2.0f * (float(px) + 0.5f) / float(w) - 1.0f;
        const float cy = 1.0f - 2.0f * (float(py) + 0.5f) / float(h);
        Mat4 pm{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        pm[0] = float(w);
        pm[5] = float(h);
        pm[12] = -float(w) * cx;
        pm[13] = -float(h) * cy;
        const Mat4 view_proj = mul(pm, webgpu_view_proj());
        float cam[3];
        write_frame(pick_frame_buf.get(), view_proj, cam);

        RenderStats scratch;
        for (Instance *inst : order) {
            inst->vis_count = 0;
            const Geometry &g = geometry(inst->geometry);
            if (g.topology == Topology::Triangles && !(inst->desc.flags & (kHidden | kNoPick)))
                cull(*inst, view_proj, cam, scratch);
        }
        // The main pass that used this buffer is already submitted; the next render re-culls into it.
        if (!visible.empty())
            wgpuQueueWriteBuffer(queue, visible_buf.get(), 0, visible.data(), visible.size() * 4);

        WGPUCommandEncoderDescriptor ed = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device, &ed);
        WGPURenderPassColorAttachment ca[2] = {WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT,
                                               WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT};
        ca[0].view = pick_id_view.get();
        ca[1].view = pick_pos_view.get();
        for (auto &a : ca) {
            a.loadOp = WGPULoadOp_Clear;
            a.storeOp = WGPUStoreOp_Store;
            a.clearValue = {0, 0, 0, 0};
        }
        WGPURenderPassDepthStencilAttachment da = WGPU_RENDER_PASS_DEPTH_STENCIL_ATTACHMENT_INIT;
        da.view = pick_depth_view.get();
        da.depthLoadOp = WGPULoadOp_Clear;
        da.depthStoreOp = WGPUStoreOp_Discard;
        da.depthClearValue = 1.0f;
        WGPURenderPassDescriptor rp = WGPU_RENDER_PASS_DESCRIPTOR_INIT;
        rp.label = sv("adacpp.pick");
        rp.colorAttachmentCount = 2;
        rp.colorAttachments = ca;
        rp.depthStencilAttachment = &da;
        WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(enc, &rp);
        wgpuRenderPassEncoderSetBindGroup(pass, 0, pick_frame_bind.get(), 0, nullptr);
        WGPURenderPipeline bound = nullptr;
        for (Instance *inst : order) {
            if (inst->vis_count == 0)
                continue;
            const Geometry &g = geometry(inst->geometry);
            WGPURenderPipeline p =
                (inst->desc.flags & (kDoubleSided | kTransparent)) ? pick_double.get() : pick_single.get();
            if (p != bound) {
                wgpuRenderPassEncoderSetPipeline(pass, p);
                bound = p;
            }
            wgpuRenderPassEncoderSetBindGroup(pass, 1, g.bind.get(), 0, nullptr);
            wgpuRenderPassEncoderDraw(pass, inst->vis_count * kClusterMaxTriangles * 3, 1,
                                      inst->vis_base * kClusterMaxTriangles * 3, 0);
        }
        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);

        auto *pending = new PendingPick();
        pending->done = std::move(done);
        pending->ids.reserve(order.size());
        for (Instance *inst : order)
            pending->ids.push_back(inst->id);
        // Two texels, each in its own 256-byte row (bytesPerRow must be a multiple of 256).
        pending->readback = make_buffer(512, WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead, "adacpp.pick_readback");
        WGPUExtent3D one = {1, 1, 1};
        WGPUTexelCopyTextureInfo src = WGPU_TEXEL_COPY_TEXTURE_INFO_INIT;
        WGPUTexelCopyBufferInfo dst = WGPU_TEXEL_COPY_BUFFER_INFO_INIT;
        dst.buffer = pending->readback.get();
        dst.layout.bytesPerRow = 256;
        dst.layout.rowsPerImage = 1;
        src.texture = pick_id_tex.get();
        dst.layout.offset = 0;
        wgpuCommandEncoderCopyTextureToBuffer(enc, &src, &dst, &one);
        src.texture = pick_pos_tex.get();
        dst.layout.offset = 256;
        wgpuCommandEncoderCopyTextureToBuffer(enc, &src, &dst, &one);
        WGPUCommandBufferDescriptor cd = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
        WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, &cd);
        wgpuQueueSubmit(queue, 1, &cmd);
        wgpuCommandBufferRelease(cmd);
        wgpuCommandEncoderRelease(enc);

        WGPUBufferMapCallbackInfo cb = WGPU_BUFFER_MAP_CALLBACK_INFO_INIT;
#ifdef __EMSCRIPTEN__
        cb.mode = WGPUCallbackMode_AllowSpontaneous;
#else
        cb.mode = WGPUCallbackMode_AllowProcessEvents;
#endif
        cb.callback = [](WGPUMapAsyncStatus status, WGPUStringView, void *ud1, void *) {
            std::unique_ptr<PendingPick> p(static_cast<PendingPick *>(ud1));
            PickResult r;
            if (status == WGPUMapAsyncStatus_Success) {
                const auto *bytes =
                    static_cast<const uint8_t *>(wgpuBufferGetConstMappedRange(p->readback.get(), 0, 512));
                uint32_t id[4];
                float pos[4];
                std::memcpy(id, bytes, 16);
                std::memcpy(pos, bytes + 256, 16);
                wgpuBufferUnmap(p->readback.get());
                if (id[0] != 0 && id[0] - 1 < p->ids.size()) {
                    r.hit = true;
                    r.instance = p->ids[id[0] - 1];
                    r.triangle = id[1];
                    r.segment = id[2];
                    r.world[0] = pos[0];
                    r.world[1] = pos[1];
                    r.world[2] = pos[2];
                }
            }
            p->done(r);
        };
        cb.userdata1 = pending;
        wgpuBufferMapAsync(pending->readback.get(), WGPUMapMode_Read, 0, 512, cb);
    }

    // One render pass. `clear` starts from the background (else loads what an earlier pass left), `resolve`
    // ends the frame into `target`.
    WGPURenderPassEncoder begin_pass(WGPUCommandEncoder enc, WGPUTextureView target, bool clear_targets, bool resolve,
                                     const char *label) {
        WGPURenderPassColorAttachment ca = WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT;
        ca.view = msaa_view.get();
        ca.resolveTarget = resolve ? target : nullptr;
        ca.loadOp = clear_targets ? WGPULoadOp_Clear : WGPULoadOp_Load;
        ca.storeOp = resolve ? WGPUStoreOp_Discard : WGPUStoreOp_Store;
        ca.clearValue = {clear[0], clear[1], clear[2], clear[3]};
        WGPURenderPassDepthStencilAttachment da = WGPU_RENDER_PASS_DEPTH_STENCIL_ATTACHMENT_INIT;
        da.view = depth_view.get();
        da.depthLoadOp = clear_targets ? WGPULoadOp_Clear : WGPULoadOp_Load;
        da.depthStoreOp = resolve ? WGPUStoreOp_Discard : WGPUStoreOp_Store;
        da.depthClearValue = 1.0f;
        WGPURenderPassDescriptor rp = WGPU_RENDER_PASS_DESCRIPTOR_INIT;
        rp.label = sv(label);
        rp.colorAttachmentCount = 1;
        rp.colorAttachments = &ca;
        rp.depthStencilAttachment = &da;
        return wgpuCommandEncoderBeginRenderPass(enc, &rp);
    }

    // Draws triangle instances (opaque or transparent) and/or line / point instances into `pass`.
    // `args`: the GPU-culled indirect args, else the CPU visible counts.
    struct Drawer {
        Impl &r;
        WGPURenderPassEncoder pass;
        RenderStats &st;
        WGPURenderPipeline bound = nullptr;
        void use(const RenderPipelineRef &p) {
            if (bound != p.get()) {
                wgpuRenderPassEncoderSetPipeline(pass, p.get());
                bound = p.get();
            }
        }
        void tris(bool transparent, WGPUBuffer args, const std::vector<char> *in_view) {
            for (Instance *inst : r.order) {
                const Geometry &g = r.geometry(inst->geometry);
                if (g.topology != Topology::Triangles || (inst->desc.flags & kHidden) || g.clusters.empty())
                    continue;
                if (bool(inst->desc.flags & kTransparent) != transparent)
                    continue;
                if (args ? !(*in_view)[inst->gpu_index] : inst->vis_count == 0)
                    continue;
                use(transparent ? r.tri_blend : (inst->desc.flags & kDoubleSided) ? r.tri_double : r.tri_single);
                wgpuRenderPassEncoderSetBindGroup(pass, 1, g.bind.get(), 0, nullptr);
                if (args)
                    wgpuRenderPassEncoderDrawIndirect(pass, args, uint64_t(inst->gpu_index) * 16);
                else
                    wgpuRenderPassEncoderDraw(pass, inst->vis_count * kClusterMaxTriangles * 3, 1,
                                              inst->vis_base * kClusterMaxTriangles * 3, 0);
                st.draw_calls++;
            }
        }
        void lists(bool transparent) {
            for (Instance *inst : r.order) {
                const Geometry &g = r.geometry(inst->geometry);
                if (g.topology == Topology::Triangles || (inst->desc.flags & kHidden))
                    continue;
                if (bool(inst->desc.flags & kTransparent) != transparent)
                    continue;
                wgpuRenderPassEncoderSetBindGroup(pass, 1, g.bind.get(), 0, nullptr);
                if (inst->desc.line_width > 1.0f) {
                    use(transparent ? r.wide_blend : r.wide);
                    const uint32_t items = g.topology == Topology::Points ? g.list_count : g.list_count / 2;
                    wgpuRenderPassEncoderDraw(pass, items * 6, 1, 0, inst->gpu_index);
                } else {
                    if (g.topology == Topology::Points)
                        use(r.points);
                    else
                        use(transparent ? r.lines_blend : r.lines);
                    wgpuRenderPassEncoderDraw(pass, g.list_count, 1, 0, inst->gpu_index);
                }
                st.draw_calls++;
            }
        }
    };

    static void end_pass(WGPURenderPassEncoder pass) {
        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);
    }

    void submit(WGPUCommandEncoder enc) {
        WGPUCommandBufferDescriptor cd = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
        WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, &cd);
        wgpuQueueSubmit(queue, 1, &cmd);
        wgpuCommandBufferRelease(cmd);
        wgpuCommandEncoderRelease(enc);
    }

    void render(WGPUTextureView target, uint32_t w, uint32_t h, RenderStats &st) {
        if (w == 0 || h == 0)
            return;
        ensure_targets(w, h);
        if (structure_dirty)
            rebuild_structure();
        if (instances_dirty)
            upload_instances();
        if (segstate_dirty)
            upload_segstate();

        const Mat4 view_proj = webgpu_view_proj();
        float cam[3];
        write_frame(frame_buf.get(), view_proj, cam);

        st = RenderStats{};
        for (Instance *inst : order) {
            const Geometry &g = geometry(inst->geometry);
            st.instances++;
            st.clusters_total += g.clusters.size();
            st.triangles_total += g.triangles;
        }
        for (auto &g : geometries)
            st.geometries += g ? 1 : 0;

        WGPUCommandEncoderDescriptor ed = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device, &ed);
        if (cull_occlusion && item_count > 0 && scene_triangles >= kOcclusionMinTriangles)
            encode_gpu_culled(enc, target, view_proj, cam, st);
        else
            encode_cpu_culled(enc, target, view_proj, cam, st);
        submit(enc);
    }

    void encode_cpu_culled(WGPUCommandEncoder enc, WGPUTextureView target, const Mat4 &view_proj, const float cam[3],
                           RenderStats &st) {
        auto t0 = std::chrono::steady_clock::now();
        for (Instance *inst : order) {
            const Geometry &g = geometry(inst->geometry);
            inst->vis_count = 0;
            if (g.topology == Topology::Triangles && !(inst->desc.flags & kHidden))
                cull(*inst, view_proj, cam, st);
        }
        if (!visible.empty())
            wgpuQueueWriteBuffer(queue, visible_buf.get(), 0, visible.data(), visible.size() * 4);
        st.cull_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        WGPURenderPassEncoder pass = begin_pass(enc, target, true, true, "adacpp.main");
        wgpuRenderPassEncoderSetBindGroup(pass, 0, frame_bind.get(), 0, nullptr);
        Drawer d{*this, pass, st};
        d.tris(false, nullptr, nullptr);
        d.lists(false);
        d.tris(true, nullptr, nullptr);
        d.lists(true);
        end_pass(pass);
    }

    // Two-pass occlusion culling, see kCullWGSL. All on the GPU; the CPU only drops whole instances outside
    // the frustum so their (empty) indirect draws are not encoded.
    void encode_gpu_culled(WGPUCommandEncoder enc, WGPUTextureView target, const Mat4 &view_proj, const float cam[3],
                           RenderStats &st) {
        auto t0 = std::chrono::steady_clock::now();
        st.gpu_culled = true;
        if (cull_binds_dirty)
            create_cull_binds();
        const auto planes = frustum_planes(view_proj);
        uint32_t frame[kCullFrameBytes / 4] = {};
        std::memcpy(frame, view_proj.data(), 64);
        for (int p = 0; p < 6; ++p)
            std::memcpy(frame + 16 + p * 4, planes[p].data(), 16);
        const float cam_pos[4] = {cam[0], cam[1], cam[2], ortho ? 0.0f : 1.0f};
        std::memcpy(frame + 40, cam_pos, 16);
        const float size[2] = {float(target_w), float(target_h)};
        std::memcpy(frame + 44, size, 8);
        frame[48] = hzb_levels;
        frame[49] = (cull_frustum ? 1u : 0u) | (cull_cone ? 2u : 0u) | 4u;
        frame[50] = item_count;
        wgpuQueueWriteBuffer(queue, cull_frame_buf.get(), 0, frame, sizeof(frame));
        wgpuQueueWriteBuffer(queue, args_early_buf.get(), 0, args_template.data(), args_template.size() * 4);
        wgpuQueueWriteBuffer(queue, args_late_buf.get(), 0, args_template.data(), args_template.size() * 4);
        const uint32_t zeros[kCounters] = {};
        wgpuQueueWriteBuffer(queue, counters_buf.get(), 0, zeros, sizeof(zeros));

        std::vector<char> in_view(order.size(), 0);
        for (Instance *inst : order)
            in_view[inst->gpu_index] = !cull_frustum || instance_in_view(*inst, planes);
        st.cull_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        const uint32_t groups = (item_count + 63) / 64;
        const uint32_t groups_x = std::min(groups, 65535u), groups_y = (groups + groups_x - 1) / groups_x;
        WGPUComputePassDescriptor cd = WGPU_COMPUTE_PASS_DESCRIPTOR_INIT;
        cd.label = sv("adacpp.cull_early");
        WGPUComputePassEncoder cp = wgpuCommandEncoderBeginComputePass(enc, &cd);
        wgpuComputePassEncoderSetPipeline(cp, cull_early.get());
        wgpuComputePassEncoderSetBindGroup(cp, 0, cull_bind_early.get(), 0, nullptr);
        wgpuComputePassEncoderDispatchWorkgroups(cp, groups_x, groups_y, 1);
        wgpuComputePassEncoderEnd(cp);
        wgpuComputePassEncoderRelease(cp);

        WGPURenderPassEncoder pass = begin_pass(enc, target, true, false, "adacpp.early");
        wgpuRenderPassEncoderSetBindGroup(pass, 0, frame_bind_early.get(), 0, nullptr);
        Drawer early{*this, pass, st};
        early.tris(false, args_early_buf.get(), &in_view);
        end_pass(pass);

        cd.label = sv("adacpp.hzb_cull_late");
        cp = wgpuCommandEncoderBeginComputePass(enc, &cd);
        wgpuComputePassEncoderSetPipeline(cp, hzb_init.get());
        wgpuComputePassEncoderSetBindGroup(cp, 0, hzb_init_bind.get(), 0, nullptr);
        wgpuComputePassEncoderDispatchWorkgroups(cp, (target_w + 7) / 8, (target_h + 7) / 8, 1);
        wgpuComputePassEncoderSetPipeline(cp, hzb_reduce.get());
        for (uint32_t l = 1; l < hzb_levels; ++l) {
            const uint32_t lw = std::max(1u, target_w >> l), lh = std::max(1u, target_h >> l);
            wgpuComputePassEncoderSetBindGroup(cp, 0, hzb_reduce_binds[l - 1].get(), 0, nullptr);
            wgpuComputePassEncoderDispatchWorkgroups(cp, (lw + 7) / 8, (lh + 7) / 8, 1);
        }
        wgpuComputePassEncoderSetPipeline(cp, cull_late.get());
        wgpuComputePassEncoderSetBindGroup(cp, 0, cull_bind_late.get(), 0, nullptr);
        wgpuComputePassEncoderDispatchWorkgroups(cp, groups_x, groups_y, 1);
        wgpuComputePassEncoderEnd(cp);
        wgpuComputePassEncoderRelease(cp);

        pass = begin_pass(enc, target, false, true, "adacpp.late");
        wgpuRenderPassEncoderSetBindGroup(pass, 0, frame_bind_late.get(), 0, nullptr);
        Drawer late{*this, pass, st};
        late.tris(false, args_late_buf.get(), &in_view);
        late.lists(false);
        late.tris(true, args_late_buf.get(), &in_view);
        late.lists(true);
        end_pass(pass);
    }

    struct PendingStats {
        BufferRef readback;
        RenderStats stats;
        std::function<void(const RenderStats &)> done;
    };

    void read_cull_stats(const RenderStats &last, std::function<void(const RenderStats &)> done) {
        if (!last.gpu_culled) {
            done(last);
            return;
        }
        auto *pending = new PendingStats();
        pending->stats = last;
        pending->done = std::move(done);
        pending->readback =
            make_buffer(kCounters * 4, WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead, "adacpp.cull_readback");
        WGPUCommandEncoderDescriptor ed = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device, &ed);
        wgpuCommandEncoderCopyBufferToBuffer(enc, counters_buf.get(), 0, pending->readback.get(), 0, kCounters * 4);
        submit(enc);
        WGPUBufferMapCallbackInfo cb = WGPU_BUFFER_MAP_CALLBACK_INFO_INIT;
#ifdef __EMSCRIPTEN__
        cb.mode = WGPUCallbackMode_AllowSpontaneous;
#else
        cb.mode = WGPUCallbackMode_AllowProcessEvents;
#endif
        cb.callback = [](WGPUMapAsyncStatus status, WGPUStringView, void *ud1, void *) {
            std::unique_ptr<PendingStats> p(static_cast<PendingStats *>(ud1));
            if (status == WGPUMapAsyncStatus_Success) {
                uint32_t c[kCounters];
                std::memcpy(c, wgpuBufferGetConstMappedRange(p->readback.get(), 0, sizeof(c)), sizeof(c));
                wgpuBufferUnmap(p->readback.get());
                p->stats.clusters_drawn = uint64_t(c[0]) + c[2];
                p->stats.triangles_drawn = uint64_t(c[1]) + c[3];
                p->stats.clusters_occluded = c[4];
            }
            p->done(p->stats);
        };
        cb.userdata1 = pending;
        wgpuBufferMapAsync(pending->readback.get(), WGPUMapMode_Read, 0, kCounters * 4, cb);
    }
};

Renderer::Renderer(WGPUDevice device, WGPUTextureFormat color_format)
    : impl_(std::make_unique<Impl>(device, color_format)) {}
Renderer::~Renderer() = default;

uint32_t Renderer::add_geometry(const GeometryDesc &desc) {
    return impl_->add_geometry(desc);
}

void Renderer::update_colors(uint32_t geometry, const uint32_t *colors, size_t count) {
    auto &g = impl_->geometry(geometry);
    if (count < g.vertex_count)
        throw std::invalid_argument("adacpp.render: update_colors needs one colour per vertex");
    if (!g.has_colors) {
        g.colors = impl_->upload(colors, size_t(g.vertex_count) * 4, "adacpp.colors");
        g.has_colors = true;
        impl_->bind_geometry(g);
        impl_->instances_dirty = true; // the vertex-colour flag is masked by has_colors
    } else {
        wgpuQueueWriteBuffer(impl_->queue, g.colors.get(), 0, colors, size_t(g.vertex_count) * 4);
    }
}

void Renderer::update_morph(uint32_t geometry, const float *morph, size_t vertex_count) {
    auto &g = impl_->geometry(geometry);
    if (vertex_count < g.vertex_count)
        throw std::invalid_argument("adacpp.render: update_morph needs one displacement per vertex");
    float ext = 0.0f;
    for (size_t v = 0; v < g.vertex_count; ++v) {
        float d2 =
            morph[v * 3] * morph[v * 3] + morph[v * 3 + 1] * morph[v * 3 + 1] + morph[v * 3 + 2] * morph[v * 3 + 2];
        ext = std::max(ext, d2);
    }
    // Per-cluster extents would need the cluster vertex lists on the CPU; bound every cluster by the
    // geometry-wide maximum instead (conservative: never culls a visible cluster).
    g.morph_extent = std::sqrt(ext);
    for (auto &c : g.clusters)
        c.morph_extent = g.morph_extent;
    if (!g.has_morph) {
        g.morph = impl_->upload(morph, size_t(g.vertex_count) * 12, "adacpp.morph");
        g.has_morph = true;
        impl_->bind_geometry(g);
    } else {
        wgpuQueueWriteBuffer(impl_->queue, g.morph.get(), 0, morph, size_t(g.vertex_count) * 12);
    }
    impl_->instances_dirty = true;
}

void Renderer::remove_geometry(uint32_t geometry) {
    impl_->geometry(geometry);
    for (auto &inst : impl_->instances)
        if (inst && inst->geometry == geometry)
            inst.reset();
    impl_->geometries[geometry].reset();
    impl_->structure_dirty = true;
}

uint32_t Renderer::segment_count(uint32_t geometry) const {
    return uint32_t(impl_->geometry(geometry).segments.size());
}

uint32_t Renderer::add_instance(uint32_t geometry) {
    auto &g = impl_->geometry(geometry);
    auto inst = std::make_unique<Impl::Instance>();
    inst->geometry = geometry;
    inst->segstate.assign(g.segments.size() * 2, 0);
    impl_->structure_dirty = true;
    auto &list = impl_->instances;
    for (uint32_t id = 0; id < list.size(); ++id) {
        if (!list[id]) {
            inst->id = id;
            list[id] = std::move(inst);
            return id;
        }
    }
    const uint32_t id = uint32_t(list.size());
    inst->id = id;
    list.push_back(std::move(inst));
    return id;
}

void Renderer::set_instance(uint32_t instance, const InstanceDesc &desc) {
    impl_->instance(instance).desc = desc;
    impl_->instances_dirty = true;
}

void Renderer::set_segment_states(uint32_t instance, const uint32_t *packed, size_t segment_count) {
    auto &inst = impl_->instance(instance);
    const size_t n = std::min(segment_count * 2, inst.segstate.size());
    std::copy(packed, packed + n, inst.segstate.begin());
    impl_->segstate_dirty = true;
}

void Renderer::remove_instance(uint32_t instance) {
    impl_->instance(instance);
    impl_->instances[instance].reset();
    impl_->structure_dirty = true;
}

void Renderer::clear() {
    impl_->instances.clear();
    impl_->geometries.clear();
    impl_->structure_dirty = true;
}

void Renderer::set_camera(const float view[16], const float proj_gl[16], bool orthographic) {
    std::copy(view, view + 16, impl_->view.begin());
    std::copy(proj_gl, proj_gl + 16, impl_->proj.begin());
    impl_->ortho = orthographic;
}

void Renderer::set_background(float r, float g, float b, float a) {
    impl_->clear[0] = r;
    impl_->clear[1] = g;
    impl_->clear[2] = b;
    impl_->clear[3] = a;
}

void Renderer::set_lighting(float ambient, float diffuse) {
    impl_->ambient = ambient;
    impl_->diffuse = diffuse;
}

void Renderer::set_culling(bool frustum, bool cone, bool occlusion) {
    impl_->cull_frustum = frustum;
    impl_->cull_cone = cone;
    impl_->cull_occlusion = occlusion;
}

void Renderer::read_cull_stats(std::function<void(const RenderStats &)> done) {
    impl_->read_cull_stats(stats_, std::move(done));
}

void Renderer::render(WGPUTextureView target, uint32_t width, uint32_t height) {
    impl_->render(target, width, height, stats_);
}

void Renderer::pick(uint32_t x, uint32_t y, uint32_t width, uint32_t height,
                    std::function<void(const PickResult &)> done) {
    impl_->pick(x, y, width, height, std::move(done));
}

std::array<float, 6> Renderer::bounds() const {
    std::array<float, 6> b{INFINITY, INFINITY, INFINITY, -INFINITY, -INFINITY, -INFINITY};
    bool any = false;
    for (auto &inst : impl_->instances) {
        if (!inst || (inst->desc.flags & kHidden))
            continue;
        const auto &g = *impl_->geometries[inst->geometry];
        const auto &m = inst->desc.model;
        for (int corner = 0; corner < 8; ++corner) {
            float p[3] = {g.aabb[(corner & 1) ? 3 : 0], g.aabb[(corner & 2) ? 4 : 1], g.aabb[(corner & 4) ? 5 : 2]};
            for (int k = 0; k < 3; ++k) {
                float w = m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k];
                b[k] = std::min(b[k], w);
                b[3 + k] = std::max(b[3 + k], w);
            }
        }
        any = true;
    }
    if (!any)
        b.fill(0.0f);
    return b;
}

} // namespace adacpp::render
