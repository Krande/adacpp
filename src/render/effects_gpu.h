// GPU side of the realistic visuals (effects.h), kept out of renderer.cpp: the environment's textures and
// their prefiltering (Environment), the HDR target with bloom and tone mapping (PostChain), and particle
// emitters (Particles). Internal to the renderer.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "effects.h"
#include "gpu.h"

namespace adacpp::render::detail {

// Image-based lighting: bind group 2 of the main pipelines. Until an environment is built, the group holds
// 1 x 1 black stand-ins, so the layout is always satisfied and a scene without IBL allocates nothing else.
class Environment {
public:
    Environment(WGPUDevice device, WGPUQueue queue);
    ~Environment();
    WGPUBindGroupLayout layout() const;
    WGPUBindGroup bind() const;
    // The image to light with (a copy, resampled at build); empty = the procedural studio.
    void set_image(fx::EnvImage image);
    // Builds the prefiltered textures if the image changed since the last build (or none was built yet).
    void ensure_built();
    bool built() const;
    // The last mip of the prefiltered radiance (roughness 1).
    float max_lod() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// The HDR target the scene is resolved into, the bloom chain on it, and the composite into the host's target.
class PostChain {
public:
    static constexpr WGPUTextureFormat kFormat = WGPUTextureFormat_RGBA16Float;
    PostChain(WGPUDevice device, WGPUQueue queue, WGPUTextureFormat target_format);
    ~PostChain();
    void ensure(uint32_t width, uint32_t height);
    WGPUTextureView hdr_view() const;
    void encode(WGPUCommandEncoder enc, WGPUTextureView target, const PostSettings &post);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Particle emitters, drawn instanced in the main pass after the transparent instances.
class Particles {
public:
    Particles(WGPUDevice device, WGPUQueue queue);
    ~Particles();
    // This frame's emitters (those alive at `time`) into the GPU; returns whether any is drawn. `view`, `proj`:
    // column-major, proj in the renderer's clip space (WebGPU, reversed depth).
    bool prepare(const std::vector<const EmitterDesc *> &emitters, const std::array<float, 16> &view,
                 const std::array<float, 16> &proj, double time, bool linear);
    // Draws what prepare() set up; the pipelines are made for these formats the first time (and again when
    // they change). Leaves its own bind group 0 set. Returns the draw calls.
    uint32_t draw(WGPURenderPassEncoder pass, WGPUTextureFormat color, WGPUTextureFormat depth, uint32_t samples);
    // The particles an emitter draws at once (its instanced draw's count).
    static uint32_t capacity(const EmitterDesc &e);
    // Whether the emitter has particles alive at `time`.
    static bool alive(const EmitterDesc &e, double time);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace adacpp::render::detail
