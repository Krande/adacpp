#include "effects_gpu.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "effects_shaders.h"
#include "renderer.h"

namespace adacpp::render::detail {

namespace {

BufferRef make_buffer(WGPUDevice device, size_t size, WGPUBufferUsage usage, const char *label) {
    WGPUBufferDescriptor d = WGPU_BUFFER_DESCRIPTOR_INIT;
    d.label = sv(label);
    d.size = std::max<size_t>(16, (size + 15) & ~size_t(15));
    d.usage = usage;
    return BufferRef(wgpuDeviceCreateBuffer(device, &d));
}

ShaderModuleRef make_shader(WGPUDevice device, const std::string &code, const char *label) {
    WGPUShaderSourceWGSL src = WGPU_SHADER_SOURCE_WGSL_INIT;
    src.code = sv(code);
    WGPUShaderModuleDescriptor sd = WGPU_SHADER_MODULE_DESCRIPTOR_INIT;
    sd.nextInChain = &src.chain;
    sd.label = sv(label);
    return ShaderModuleRef(wgpuDeviceCreateShaderModule(device, &sd));
}

TextureRef make_texture(WGPUDevice device, uint32_t w, uint32_t h, uint32_t mips, WGPUTextureFormat format,
                        WGPUTextureUsage usage, const char *label) {
    WGPUTextureDescriptor d = WGPU_TEXTURE_DESCRIPTOR_INIT;
    d.label = sv(label);
    d.size = {w, h, 1};
    d.format = format;
    d.mipLevelCount = mips;
    d.usage = usage;
    return TextureRef(wgpuDeviceCreateTexture(device, &d));
}

TextureViewRef mip_view(const TextureRef &t, uint32_t mip, uint32_t count = 1) {
    WGPUTextureViewDescriptor vd = WGPU_TEXTURE_VIEW_DESCRIPTOR_INIT;
    vd.baseMipLevel = mip;
    vd.mipLevelCount = count;
    return TextureViewRef(wgpuTextureCreateView(t.get(), &vd));
}

WGPUBindGroupEntry buffer_entry(uint32_t binding, WGPUBuffer b) {
    WGPUBindGroupEntry e = WGPU_BIND_GROUP_ENTRY_INIT;
    e.binding = binding;
    e.buffer = b;
    e.size = wgpuBufferGetSize(b);
    return e;
}
WGPUBindGroupEntry view_entry(uint32_t binding, WGPUTextureView v) {
    WGPUBindGroupEntry e = WGPU_BIND_GROUP_ENTRY_INIT;
    e.binding = binding;
    e.textureView = v;
    return e;
}
WGPUBindGroupEntry sampler_entry(uint32_t binding, WGPUSampler s) {
    WGPUBindGroupEntry e = WGPU_BIND_GROUP_ENTRY_INIT;
    e.binding = binding;
    e.sampler = s;
    return e;
}

BindGroupRef make_bind(WGPUDevice device, WGPUBindGroupLayout layout, const std::vector<WGPUBindGroupEntry> &e,
                       const char *label) {
    WGPUBindGroupDescriptor d = WGPU_BIND_GROUP_DESCRIPTOR_INIT;
    d.label = sv(label);
    d.layout = layout;
    d.entryCount = e.size();
    d.entries = e.data();
    return BindGroupRef(wgpuDeviceCreateBindGroup(device, &d));
}

// RGBA half floats of `rgb` (3 floats per texel), alpha 1.
std::vector<uint16_t> to_half_rgba(const std::vector<float> &rgb, size_t texels) {
    std::vector<uint16_t> out(texels * 4);
    const uint16_t one = fx::float_to_half(1.0f);
    for (size_t i = 0; i < texels; ++i) {
        for (int k = 0; k < 3; ++k)
            out[i * 4 + k] = fx::float_to_half(std::min(rgb[i * 3 + k], 65000.0f));
        out[i * 4 + 3] = one;
    }
    return out;
}

void write_half(WGPUQueue queue, WGPUTexture tex, uint32_t mip, uint32_t w, uint32_t h,
                const std::vector<uint16_t> &texels) {
    WGPUTexelCopyTextureInfo dst = WGPU_TEXEL_COPY_TEXTURE_INFO_INIT;
    dst.texture = tex;
    dst.mipLevel = mip;
    WGPUTexelCopyBufferLayout layout = WGPU_TEXEL_COPY_BUFFER_LAYOUT_INIT;
    layout.bytesPerRow = w * 8;
    layout.rowsPerImage = h;
    const WGPUExtent3D size = {w, h, 1};
    wgpuQueueWriteTexture(queue, &dst, texels.data(), texels.size() * 2, &layout, &size);
}

void submit(WGPUQueue queue, WGPUCommandEncoder enc) {
    WGPUCommandBufferDescriptor cd = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, &cd);
    wgpuQueueSubmit(queue, 1, &cmd);
    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(enc);
}

constexpr uint32_t kEnvWidth = 512, kEnvHeight = 256; // the source, and the prefiltered radiance's level 0
constexpr uint32_t kSpecMips = 6;                     // roughness 0, 0.2, ..., 1
constexpr uint32_t kIrrWidth = 32, kIrrHeight = 16;
constexpr uint32_t kIrrSourceMip = 4; // 32 x 16 of the source
constexpr uint32_t kBrdfSize = 64;

} // namespace

// ---------------------------------------------------------------------------------------------------------------

struct Environment::Impl {
    WGPUDevice device;
    WGPUQueue queue;
    BindGroupLayoutRef layout;
    SamplerRef sampler;
    TextureRef dummy;
    TextureViewRef dummy_view;
    BindGroupRef dummy_bind, bind;
    fx::EnvImage image; // empty: the studio
    bool dirty = true, built = false;

    ShaderModuleRef shader;
    ComputePipelineRef prefilter, irradiance, brdf;
    TextureRef spec, irr, lut;
    TextureViewRef spec_view, irr_view, lut_view;

    Impl(WGPUDevice d, WGPUQueue q) : device(d), queue(q) {
        WGPUBindGroupLayoutEntry e[4] = {WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT,
                                         WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT};
        for (uint32_t i = 0; i < 4; ++i) {
            e[i].binding = i;
            e[i].visibility = WGPUShaderStage_Fragment;
            if (i == 0) {
                e[i].sampler.type = WGPUSamplerBindingType_Filtering;
            } else {
                e[i].texture.sampleType = WGPUTextureSampleType_Float;
                e[i].texture.viewDimension = WGPUTextureViewDimension_2D;
            }
        }
        WGPUBindGroupLayoutDescriptor ld = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        ld.label = sv("adacpp.env_layout");
        ld.entryCount = 4;
        ld.entries = e;
        layout.reset(wgpuDeviceCreateBindGroupLayout(device, &ld));

        WGPUSamplerDescriptor sd = WGPU_SAMPLER_DESCRIPTOR_INIT;
        sd.label = sv("adacpp.env_sampler");
        sd.addressModeU = WGPUAddressMode_Repeat; // equirectangular: wraps around in azimuth
        sd.addressModeV = WGPUAddressMode_ClampToEdge;
        sd.magFilter = WGPUFilterMode_Linear;
        sd.minFilter = WGPUFilterMode_Linear;
        sd.mipmapFilter = WGPUMipmapFilterMode_Linear;
        sampler.reset(wgpuDeviceCreateSampler(device, &sd));

        dummy = make_texture(device, 1, 1, 1, WGPUTextureFormat_RGBA16Float,
                             WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst, "adacpp.env_dummy");
        write_half(queue, dummy.get(), 0, 1, 1, std::vector<uint16_t>(4, 0));
        dummy_view.reset(wgpuTextureCreateView(dummy.get(), nullptr));
        dummy_bind = make_bind(device, layout.get(),
                               {sampler_entry(0, sampler.get()), view_entry(1, dummy_view.get()),
                                view_entry(2, dummy_view.get()), view_entry(3, dummy_view.get())},
                               "adacpp.env_dummy");
    }

    ComputePipelineRef compute(const char *entry) {
        WGPUComputePipelineDescriptor d = WGPU_COMPUTE_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(entry);
        d.layout = nullptr; // "auto": each entry point binds what it uses
        d.compute.module = shader.get();
        d.compute.entryPoint = sv(entry);
        return ComputePipelineRef(wgpuDeviceCreateComputePipeline(device, &d));
    }

    void dispatch(WGPUCommandEncoder enc, const ComputePipelineRef &p, const std::vector<WGPUBindGroupEntry> &e,
                  uint32_t w, uint32_t h) {
        WGPUBindGroupLayout l = wgpuComputePipelineGetBindGroupLayout(p.get(), 0);
        BindGroupRef bg = make_bind(device, l, e, "adacpp.env_pass");
        wgpuBindGroupLayoutRelease(l);
        WGPUComputePassDescriptor cd = WGPU_COMPUTE_PASS_DESCRIPTOR_INIT;
        WGPUComputePassEncoder cp = wgpuCommandEncoderBeginComputePass(enc, &cd);
        wgpuComputePassEncoderSetPipeline(cp, p.get());
        wgpuComputePassEncoderSetBindGroup(cp, 0, bg.get(), 0, nullptr);
        wgpuComputePassEncoderDispatchWorkgroups(cp, (w + 7) / 8, (h + 7) / 8, 1);
        wgpuComputePassEncoderEnd(cp);
        wgpuComputePassEncoderRelease(cp);
    }

    void build() {
        if (!shader) {
            shader = make_shader(device, kEnvWGSL, "adacpp.env_shader");
            prefilter = compute("cs_prefilter");
            irradiance = compute("cs_irradiance");
            brdf = compute("cs_brdf");
        }
        const auto store = WGPUTextureUsage_StorageBinding | WGPUTextureUsage_TextureBinding;
        WGPUCommandEncoderDescriptor ed = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device, &ed);
        std::vector<BufferRef> params; // alive until the submit below
        auto param = [&](float a, float b, float c) {
            const float p[4] = {a, b, c, 0};
            params.push_back(make_buffer(device, 16, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst, "adacpp.env"));
            wgpuQueueWriteBuffer(queue, params.back().get(), 0, p, 16);
            return params.back().get();
        };
        if (!lut) {
            lut =
                make_texture(device, kBrdfSize, kBrdfSize, 1, WGPUTextureFormat_RGBA16Float, store, "adacpp.env_brdf");
            lut_view.reset(wgpuTextureCreateView(lut.get(), nullptr));
            dispatch(enc, brdf, {view_entry(3, lut_view.get())}, kBrdfSize, kBrdfSize);
        }

        // The source with its mip chain, filtered on the CPU (box), as half floats.
        fx::EnvImage img =
            image.width ? fx::resample(image, kEnvWidth, kEnvHeight) : fx::studio_environment(kEnvWidth, kEnvHeight);
        uint32_t mips = 1;
        while ((std::max(kEnvWidth, kEnvHeight) >> mips) > 0)
            ++mips;
        TextureRef source = make_texture(device, kEnvWidth, kEnvHeight, mips, WGPUTextureFormat_RGBA16Float,
                                         WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst, "adacpp.env_src");
        for (uint32_t m = 0; m < mips; ++m) {
            if (m > 0)
                img = fx::resample(img, std::max(1u, img.width / 2), std::max(1u, img.height / 2));
            write_half(queue, source.get(), m, img.width, img.height,
                       to_half_rgba(img.rgb, size_t(img.width) * img.height));
        }
        TextureViewRef source_view(wgpuTextureCreateView(source.get(), nullptr));

        spec = make_texture(device, kEnvWidth, kEnvHeight, kSpecMips, WGPUTextureFormat_RGBA16Float, store,
                            "adacpp.env_radiance");
        for (uint32_t m = 0; m < kSpecMips; ++m) {
            TextureViewRef out = mip_view(spec, m);
            const float rough = float(m) / float(kSpecMips - 1);
            dispatch(enc, prefilter,
                     {buffer_entry(0, param(rough, 0, m == 0 ? 1.0f : 256.0f)), view_entry(1, source_view.get()),
                      sampler_entry(2, sampler.get()), view_entry(3, out.get())},
                     std::max(1u, kEnvWidth >> m), std::max(1u, kEnvHeight >> m));
        }
        spec_view.reset(wgpuTextureCreateView(spec.get(), nullptr));
        irr = make_texture(device, kIrrWidth, kIrrHeight, 1, WGPUTextureFormat_RGBA16Float, store,
                           "adacpp.env_irradiance");
        irr_view.reset(wgpuTextureCreateView(irr.get(), nullptr));
        dispatch(enc, irradiance,
                 {buffer_entry(0, param(0, float(kIrrSourceMip), 0)), view_entry(1, source_view.get()),
                  view_entry(3, irr_view.get())},
                 kIrrWidth, kIrrHeight);
        submit(queue, enc);
        bind = make_bind(device, layout.get(),
                         {sampler_entry(0, sampler.get()), view_entry(1, spec_view.get()),
                          view_entry(2, irr_view.get()), view_entry(3, lut_view.get())},
                         "adacpp.env");
        dirty = false;
        built = true;
    }
};

Environment::Environment(WGPUDevice device, WGPUQueue queue) : impl_(std::make_unique<Impl>(device, queue)) {}
Environment::~Environment() = default;
WGPUBindGroupLayout Environment::layout() const {
    return impl_->layout.get();
}
WGPUBindGroup Environment::bind() const {
    return impl_->built ? impl_->bind.get() : impl_->dummy_bind.get();
}
void Environment::set_image(fx::EnvImage image) {
    impl_->image = std::move(image);
    impl_->dirty = true;
}
void Environment::ensure_built() {
    if (impl_->dirty || !impl_->built)
        impl_->build();
}
bool Environment::built() const {
    return impl_->built;
}
float Environment::max_lod() const {
    return float(kSpecMips - 1);
}

// ---------------------------------------------------------------------------------------------------------------

struct PostChain::Impl {
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat target_format;
    BindGroupLayoutRef layout;
    PipelineLayoutRef pipeline_layout;
    ShaderModuleRef shader;
    RenderPipelineRef prefilter, down, up, composite;
    SamplerRef sampler;
    BufferRef params;
    TextureRef dummy;
    TextureViewRef dummy_view;

    uint32_t width = 0, height = 0, levels = 0;
    TextureRef hdr, bloom;
    TextureViewRef hdr_view;
    std::vector<TextureViewRef> bloom_mips;
    BindGroupRef prefilter_bind, composite_bind, composite_nobloom_bind;
    std::vector<BindGroupRef> down_binds, up_binds; // down i: mip i -> i + 1; up i: mip i + 1 -> i

    Impl(WGPUDevice d, WGPUQueue q, WGPUTextureFormat f) : device(d), queue(q), target_format(f) {
        WGPUBindGroupLayoutEntry e[4] = {WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT,
                                         WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT};
        for (uint32_t i = 0; i < 4; ++i) {
            e[i].binding = i;
            e[i].visibility = WGPUShaderStage_Fragment;
        }
        e[0].buffer.type = WGPUBufferBindingType_Uniform;
        e[1].texture.sampleType = WGPUTextureSampleType_Float;
        e[1].texture.viewDimension = WGPUTextureViewDimension_2D;
        e[2].sampler.type = WGPUSamplerBindingType_Filtering;
        e[3].texture.sampleType = WGPUTextureSampleType_Float;
        e[3].texture.viewDimension = WGPUTextureViewDimension_2D;
        WGPUBindGroupLayoutDescriptor ld = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        ld.label = sv("adacpp.post_layout");
        ld.entryCount = 4;
        ld.entries = e;
        layout.reset(wgpuDeviceCreateBindGroupLayout(device, &ld));
        WGPUBindGroupLayout l = layout.get();
        WGPUPipelineLayoutDescriptor pd = WGPU_PIPELINE_LAYOUT_DESCRIPTOR_INIT;
        pd.bindGroupLayoutCount = 1;
        pd.bindGroupLayouts = &l;
        pipeline_layout.reset(wgpuDeviceCreatePipelineLayout(device, &pd));
        shader = make_shader(device, kPostWGSL, "adacpp.post_shader");

        WGPUSamplerDescriptor sd = WGPU_SAMPLER_DESCRIPTOR_INIT;
        sd.label = sv("adacpp.post_sampler");
        sd.addressModeU = WGPUAddressMode_ClampToEdge;
        sd.addressModeV = WGPUAddressMode_ClampToEdge;
        sd.magFilter = WGPUFilterMode_Linear;
        sd.minFilter = WGPUFilterMode_Linear;
        sampler.reset(wgpuDeviceCreateSampler(device, &sd));
        params = make_buffer(device, 32, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst, "adacpp.post");
        dummy = make_texture(device, 1, 1, 1, kFormat, WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
                             "adacpp.post_dummy");
        write_half(queue, dummy.get(), 0, 1, 1, std::vector<uint16_t>(4, 0));
        dummy_view.reset(wgpuTextureCreateView(dummy.get(), nullptr));

        prefilter = pipeline("fs_prefilter", kFormat, false);
        down = pipeline("fs_down", kFormat, false);
        up = pipeline("fs_up", kFormat, true);
        composite = pipeline("fs_composite", target_format, false);
    }

    RenderPipelineRef pipeline(const char *fragment, WGPUTextureFormat format, bool additive) {
        WGPUBlendState bs = WGPU_BLEND_STATE_INIT;
        bs.color.srcFactor = WGPUBlendFactor_One;
        bs.color.dstFactor = WGPUBlendFactor_One;
        bs.color.operation = WGPUBlendOperation_Add;
        bs.alpha.srcFactor = WGPUBlendFactor_Zero;
        bs.alpha.dstFactor = WGPUBlendFactor_One;
        bs.alpha.operation = WGPUBlendOperation_Add;
        WGPUColorTargetState ct = WGPU_COLOR_TARGET_STATE_INIT;
        ct.format = format;
        ct.blend = additive ? &bs : nullptr;
        ct.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState fs = WGPU_FRAGMENT_STATE_INIT;
        fs.module = shader.get();
        fs.entryPoint = sv(fragment);
        fs.targetCount = 1;
        fs.targets = &ct;
        WGPURenderPipelineDescriptor d = WGPU_RENDER_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(fragment);
        d.layout = pipeline_layout.get();
        d.vertex.module = shader.get();
        d.vertex.entryPoint = sv("vs_full");
        d.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        d.multisample.count = 1;
        d.multisample.mask = 0xFFFFFFFFu;
        d.fragment = &fs;
        return RenderPipelineRef(wgpuDeviceCreateRenderPipeline(device, &d));
    }

    BindGroupRef bind(WGPUTextureView src, WGPUTextureView bloom_view) {
        return make_bind(device, layout.get(),
                         {buffer_entry(0, params.get()), view_entry(1, src), sampler_entry(2, sampler.get()),
                          view_entry(3, bloom_view)},
                         "adacpp.post_pass");
    }

    void ensure(uint32_t w, uint32_t h) {
        if (w == width && h == height && hdr)
            return;
        width = w;
        height = h;
        hdr = make_texture(device, w, h, 1, kFormat,
                           WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding, "adacpp.hdr");
        hdr_view.reset(wgpuTextureCreateView(hdr.get(), nullptr));
        const uint32_t bw = std::max(1u, w / 2), bh = std::max(1u, h / 2);
        levels = 1;
        while (levels < 6 && (std::min(bw, bh) >> levels) >= 4)
            ++levels;
        bloom = make_texture(device, bw, bh, levels, kFormat,
                             WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding, "adacpp.bloom");
        bloom_mips.clear();
        for (uint32_t m = 0; m < levels; ++m)
            bloom_mips.push_back(mip_view(bloom, m));
        prefilter_bind = bind(hdr_view.get(), dummy_view.get());
        composite_bind = bind(hdr_view.get(), bloom_mips[0].get());
        composite_nobloom_bind = bind(hdr_view.get(), dummy_view.get());
        down_binds.clear();
        up_binds.clear();
        for (uint32_t m = 0; m + 1 < levels; ++m) {
            down_binds.push_back(bind(bloom_mips[m].get(), dummy_view.get()));
            up_binds.push_back(bind(bloom_mips[m + 1].get(), dummy_view.get()));
        }
    }

    void pass(WGPUCommandEncoder enc, WGPUTextureView target, bool load, const RenderPipelineRef &p,
              const BindGroupRef &bg) {
        WGPURenderPassColorAttachment ca = WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT;
        ca.view = target;
        ca.loadOp = load ? WGPULoadOp_Load : WGPULoadOp_Clear;
        ca.storeOp = WGPUStoreOp_Store;
        ca.clearValue = {0, 0, 0, 0};
        WGPURenderPassDescriptor rp = WGPU_RENDER_PASS_DESCRIPTOR_INIT;
        rp.label = sv("adacpp.post");
        rp.colorAttachmentCount = 1;
        rp.colorAttachments = &ca;
        WGPURenderPassEncoder pe = wgpuCommandEncoderBeginRenderPass(enc, &rp);
        wgpuRenderPassEncoderSetPipeline(pe, p.get());
        wgpuRenderPassEncoderSetBindGroup(pe, 0, bg.get(), 0, nullptr);
        wgpuRenderPassEncoderDraw(pe, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(pe);
        wgpuRenderPassEncoderRelease(pe);
    }

    void encode(WGPUCommandEncoder enc, WGPUTextureView target, const PostSettings &s) {
        const bool bloom_on = s.bloom && s.bloom_strength > 0.0f;
        const float p[8] = {std::max(s.exposure, 0.0f),
                            std::max(s.bloom_threshold, 0.0f),
                            std::clamp(s.bloom_knee, 0.0f, 1.0f),
                            std::max(s.bloom_strength, 0.0f),
                            std::clamp(s.bloom_radius, 0.0f, 1.0f),
                            float(uint32_t(s.tone_mapping)),
                            bloom_on ? 1.0f : 0.0f,
                            0.0f};
        wgpuQueueWriteBuffer(queue, params.get(), 0, p, sizeof(p));
        if (bloom_on) {
            pass(enc, bloom_mips[0].get(), false, prefilter, prefilter_bind);
            for (uint32_t m = 0; m + 1 < levels; ++m)
                pass(enc, bloom_mips[m + 1].get(), false, down, down_binds[m]);
            for (uint32_t m = levels - 1; m-- > 0;)
                pass(enc, bloom_mips[m].get(), true, up, up_binds[m]);
        }
        pass(enc, target, false, composite, bloom_on ? composite_bind : composite_nobloom_bind);
    }
};

PostChain::PostChain(WGPUDevice device, WGPUQueue queue, WGPUTextureFormat target_format)
    : impl_(std::make_unique<Impl>(device, queue, target_format)) {}
PostChain::~PostChain() = default;
void PostChain::ensure(uint32_t width, uint32_t height) {
    impl_->ensure(width, height);
}
WGPUTextureView PostChain::hdr_view() const {
    return impl_->hdr_view.get();
}
void PostChain::encode(WGPUCommandEncoder enc, WGPUTextureView target, const PostSettings &post) {
    impl_->encode(enc, target, post);
}

// ---------------------------------------------------------------------------------------------------------------

namespace {
constexpr size_t kEmitterBytes = 144; // WGSL `Emitter`
constexpr size_t kPFrameBytes = 160;  // WGSL `PFrame`
} // namespace

uint32_t Particles::capacity(const EmitterDesc &e) {
    if (!(e.rate > 0.0f) || !(e.lifetime > 0.0f))
        return 0;
    const double n = std::ceil(double(e.rate) * double(e.lifetime)) + 1.0;
    return uint32_t(std::min<double>(n, Renderer::kMaxParticlesPerEmitter));
}

bool Particles::alive(const EmitterDesc &e, double time) {
    return e.enabled && capacity(e) > 0 && time >= e.start && time <= e.end + double(e.lifetime);
}

struct Particles::Impl {
    WGPUDevice device;
    WGPUQueue queue;
    BindGroupLayoutRef layout;
    PipelineLayoutRef pipeline_layout;
    ShaderModuleRef shader;
    RenderPipelineRef additive, alpha;
    WGPUTextureFormat color = WGPUTextureFormat_Undefined, depth = WGPUTextureFormat_Undefined;
    uint32_t samples = 0;
    BufferRef frame, emitters;
    size_t cap = 0;
    BindGroupRef bind;
    struct Draw {
        uint32_t index, count;
        ParticleBlend blend;
    };
    std::vector<Draw> draws;

    Impl(WGPUDevice d, WGPUQueue q) : device(d), queue(q) {
        WGPUBindGroupLayoutEntry e[2] = {WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT, WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT};
        e[0].binding = 0;
        e[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        e[0].buffer.type = WGPUBufferBindingType_Uniform;
        e[1].binding = 1;
        e[1].visibility = WGPUShaderStage_Vertex;
        e[1].buffer.type = WGPUBufferBindingType_ReadOnlyStorage;
        WGPUBindGroupLayoutDescriptor ld = WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT;
        ld.label = sv("adacpp.particle_layout");
        ld.entryCount = 2;
        ld.entries = e;
        layout.reset(wgpuDeviceCreateBindGroupLayout(device, &ld));
        WGPUBindGroupLayout l = layout.get();
        WGPUPipelineLayoutDescriptor pd = WGPU_PIPELINE_LAYOUT_DESCRIPTOR_INIT;
        pd.bindGroupLayoutCount = 1;
        pd.bindGroupLayouts = &l;
        pipeline_layout.reset(wgpuDeviceCreatePipelineLayout(device, &pd));
        std::string code = kParticleWGSL;
        const std::string token = "{{MAX_PER_EMITTER}}";
        code.replace(code.find(token), token.size(), std::to_string(Renderer::kMaxParticlesPerEmitter));
        shader = make_shader(device, code, "adacpp.particle_shader");
        frame = make_buffer(device, kPFrameBytes, WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst,
                            "adacpp.particle_frame");
    }

    RenderPipelineRef pipeline(const char *fragment, bool add) {
        WGPUBlendState bs = WGPU_BLEND_STATE_INIT;
        if (add) {
            bs.color.srcFactor = WGPUBlendFactor_One;
            bs.color.dstFactor = WGPUBlendFactor_One;
            bs.alpha.srcFactor = WGPUBlendFactor_Zero;
            bs.alpha.dstFactor = WGPUBlendFactor_One;
        } else {
            bs.color.srcFactor = WGPUBlendFactor_SrcAlpha;
            bs.color.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
            bs.alpha.srcFactor = WGPUBlendFactor_One;
            bs.alpha.dstFactor = WGPUBlendFactor_OneMinusSrcAlpha;
        }
        bs.color.operation = WGPUBlendOperation_Add;
        bs.alpha.operation = WGPUBlendOperation_Add;
        WGPUColorTargetState ct = WGPU_COLOR_TARGET_STATE_INIT;
        ct.format = color;
        ct.blend = &bs;
        ct.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState fs = WGPU_FRAGMENT_STATE_INIT;
        fs.module = shader.get();
        fs.entryPoint = sv(fragment);
        fs.targetCount = 1;
        fs.targets = &ct;
        WGPUDepthStencilState ds = WGPU_DEPTH_STENCIL_STATE_INIT;
        ds.format = depth;
        ds.depthWriteEnabled = WGPUOptionalBool_False;
        ds.depthCompare = WGPUCompareFunction_GreaterEqual; // reversed depth: nearer or equal
        WGPURenderPipelineDescriptor d = WGPU_RENDER_PIPELINE_DESCRIPTOR_INIT;
        d.label = sv(fragment);
        d.layout = pipeline_layout.get();
        d.vertex.module = shader.get();
        d.vertex.entryPoint = sv("vs_particle");
        d.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        d.primitive.cullMode = WGPUCullMode_None;
        d.depthStencil = &ds;
        d.multisample.count = samples;
        d.multisample.mask = 0xFFFFFFFFu;
        d.fragment = &fs;
        return RenderPipelineRef(wgpuDeviceCreateRenderPipeline(device, &d));
    }

    bool prepare(const std::vector<const EmitterDesc *> &list, const std::array<float, 16> &view,
                 const std::array<float, 16> &proj, double time, bool linear) {
        draws.clear();
        std::vector<uint32_t> data;
        for (const EmitterDesc *e : list) {
            if (!Particles::alive(*e, time))
                continue;
            const uint32_t index = uint32_t(draws.size());
            draws.push_back({index, Particles::capacity(*e), e->blend});
            const float dir_len = std::sqrt(e->direction[0] * e->direction[0] + e->direction[1] * e->direction[1] +
                                            e->direction[2] * e->direction[2]);
            // Times relative to the emitter's start where precision matters (k / rate), absolute otherwise.
            const float end = float(std::min(e->end, 1e30));
            const float f[32] = {e->origin[0],
                                 e->origin[1],
                                 e->origin[2],
                                 float(e->start),
                                 e->velocity[0],
                                 e->velocity[1],
                                 e->velocity[2],
                                 end,
                                 dir_len > 0 ? e->direction[0] : 0.0f,
                                 dir_len > 0 ? e->direction[1] : 0.0f,
                                 dir_len > 0 ? e->direction[2] : 1.0f,
                                 std::cos(std::clamp(e->spread, 0.0f, 3.14159265f)),
                                 e->gravity[0],
                                 e->gravity[1],
                                 e->gravity[2],
                                 std::max(e->drag, 0.0f),
                                 e->speed,
                                 std::clamp(e->speed_jitter, 0.0f, 1.0f),
                                 e->lifetime,
                                 std::clamp(e->lifetime_jitter, 0.0f, 0.95f),
                                 e->color_start[0] * e->intensity_start,
                                 e->color_start[1] * e->intensity_start,
                                 e->color_start[2] * e->intensity_start,
                                 e->alpha_start,
                                 e->color_end[0] * e->intensity_end,
                                 e->color_end[1] * e->intensity_end,
                                 e->color_end[2] * e->intensity_end,
                                 e->alpha_end,
                                 e->size_start,
                                 e->size_end,
                                 std::max(e->stretch, 0.0f),
                                 std::max(e->radius, 0.0f)};
            const size_t at = data.size();
            data.resize(at + kEmitterBytes / 4);
            std::memcpy(data.data() + at, f, sizeof(f));
            const float rate = e->rate;
            std::memcpy(&data[at + 32], &rate, 4);
            data[at + 33] = Particles::capacity(*e);
            data[at + 34] = e->seed * 0x9E3779B9u + 0x7F4A7C15u;
            data[at + 35] = uint32_t(e->blend);
        }
        if (draws.empty())
            return false;
        if (data.size() * 4 > cap || !emitters) {
            cap = std::max<size_t>(data.size() * 4 * 2, kEmitterBytes * 8);
            emitters = make_buffer(device, cap, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst, "adacpp.emitters");
            bind = make_bind(device, layout.get(), {buffer_entry(0, frame.get()), buffer_entry(1, emitters.get())},
                             "adacpp.particles");
        }
        wgpuQueueWriteBuffer(queue, emitters.get(), 0, data.data(), data.size() * 4);
        float pf[kPFrameBytes / 4] = {};
        std::memcpy(pf, view.data(), 64);
        std::memcpy(pf + 16, proj.data(), 64);
        pf[32] = float(time);
        const uint32_t lin = linear ? 1u : 0u;
        std::memcpy(pf + 36, &lin, 4);
        wgpuQueueWriteBuffer(queue, frame.get(), 0, pf, sizeof(pf));
        return true;
    }

    uint32_t draw(WGPURenderPassEncoder pass, WGPUTextureFormat c, WGPUTextureFormat d, uint32_t s) {
        if (draws.empty())
            return 0;
        if (c != color || d != depth || s != samples || !additive) {
            color = c;
            depth = d;
            samples = s;
            additive = pipeline("fs_additive", true);
            alpha = pipeline("fs_alpha", false);
        }
        wgpuRenderPassEncoderSetBindGroup(pass, 0, bind.get(), 0, nullptr);
        uint32_t calls = 0;
        // Smoke first, so the sparks glow through it.
        for (int pass_blend = 1; pass_blend >= 0; --pass_blend) {
            bool bound = false;
            for (const Draw &dr : draws) {
                if (uint32_t(dr.blend) != uint32_t(pass_blend))
                    continue;
                if (!bound) {
                    wgpuRenderPassEncoderSetPipeline(pass, pass_blend ? alpha.get() : additive.get());
                    bound = true;
                }
                wgpuRenderPassEncoderDraw(pass, 6, dr.count, 0, dr.index * Renderer::kMaxParticlesPerEmitter);
                ++calls;
            }
        }
        return calls;
    }
};

Particles::Particles(WGPUDevice device, WGPUQueue queue) : impl_(std::make_unique<Impl>(device, queue)) {}
Particles::~Particles() = default;
bool Particles::prepare(const std::vector<const EmitterDesc *> &emitters, const std::array<float, 16> &view,
                        const std::array<float, 16> &proj, double time, bool linear) {
    return impl_->prepare(emitters, view, proj, time, linear);
}
uint32_t Particles::draw(WGPURenderPassEncoder pass, WGPUTextureFormat color, WGPUTextureFormat depth,
                         uint32_t samples) {
    return impl_->draw(pass, color, depth, samples);
}

} // namespace adacpp::render::detail
