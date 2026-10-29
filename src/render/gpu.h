// The one place that knows which WebGPU implementation the renderer is built against.
//
// Native builds use wgpu-native (conda-forge `wgpu-native` + `webgpu-headers`, the standard webgpu.h at
// <webgpu.h> plus wgpu-native's extensions in <wgpu.h>). The browser build uses Dawn's emdawnwebgpu
// emscripten port, which maps the same standard header (at <webgpu/webgpu.h>) onto navigator.gpu. Both
// follow the webgpu-headers 1.0 API, so everything outside this file is written once against webgpu.h.
#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

#ifdef __EMSCRIPTEN__
#include <webgpu/webgpu.h>
#else
#include <webgpu.h>
#include <wgpu.h>
#endif

namespace adacpp::render {

inline WGPUStringView sv(std::string_view s) {
    return WGPUStringView{s.data(), s.size()};
}
inline WGPUStringView sv(const char *s) {
    return WGPUStringView{s, std::strlen(s)};
}

// RAII for the handful of handle types the renderer owns. WebGPU handles are reference counted; Release
// drops ours.
#define ADACPP_RENDER_HANDLE(Type, Release)                                                                            \
    struct Type##Ref {                                                                                                 \
        WGPU##Type h = nullptr;                                                                                        \
        Type##Ref() = default;                                                                                         \
        explicit Type##Ref(WGPU##Type v) : h(v) {}                                                                     \
        Type##Ref(const Type##Ref &) = delete;                                                                         \
        Type##Ref &operator=(const Type##Ref &) = delete;                                                              \
        Type##Ref(Type##Ref &&o) noexcept : h(o.h) {                                                                   \
            o.h = nullptr;                                                                                             \
        }                                                                                                              \
        Type##Ref &operator=(Type##Ref &&o) noexcept {                                                                 \
            if (this != &o) {                                                                                          \
                reset();                                                                                               \
                h = o.h;                                                                                               \
                o.h = nullptr;                                                                                         \
            }                                                                                                          \
            return *this;                                                                                              \
        }                                                                                                              \
        ~Type##Ref() {                                                                                                 \
            reset();                                                                                                   \
        }                                                                                                              \
        void reset(WGPU##Type v = nullptr) {                                                                           \
            if (h)                                                                                                     \
                Release(h);                                                                                            \
            h = v;                                                                                                     \
        }                                                                                                              \
        WGPU##Type get() const {                                                                                       \
            return h;                                                                                                  \
        }                                                                                                              \
        explicit operator bool() const {                                                                               \
            return h != nullptr;                                                                                       \
        }                                                                                                              \
    };

ADACPP_RENDER_HANDLE(Buffer, wgpuBufferRelease)
ADACPP_RENDER_HANDLE(BindGroup, wgpuBindGroupRelease)
ADACPP_RENDER_HANDLE(BindGroupLayout, wgpuBindGroupLayoutRelease)
ADACPP_RENDER_HANDLE(PipelineLayout, wgpuPipelineLayoutRelease)
ADACPP_RENDER_HANDLE(RenderPipeline, wgpuRenderPipelineRelease)
ADACPP_RENDER_HANDLE(ShaderModule, wgpuShaderModuleRelease)
ADACPP_RENDER_HANDLE(Texture, wgpuTextureRelease)
ADACPP_RENDER_HANDLE(TextureView, wgpuTextureViewRelease)

#undef ADACPP_RENDER_HANDLE

} // namespace adacpp::render
