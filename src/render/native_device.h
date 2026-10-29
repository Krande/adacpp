// Headless native WebGPU device (wgpu-native) and an offscreen colour target with PNG-ready readback.
#pragma once

#ifndef __EMSCRIPTEN__

#include <cstdint>
#include <string>
#include <vector>

#include "gpu.h"

namespace adacpp::render {

struct AdapterInfo {
    std::string vendor, architecture, device, description, backend;
};

class NativeDevice {
public:
    // `backend`: "auto", "vulkan", "gl", "metal", "dx12". Falls back to whatever adapter is available.
    explicit NativeDevice(const std::string &backend = "auto");
    ~NativeDevice();
    NativeDevice(const NativeDevice &) = delete;
    NativeDevice &operator=(const NativeDevice &) = delete;

    WGPUDevice device() const {
        return device_;
    }
    const AdapterInfo &info() const {
        return info_;
    }
    // Block until all submitted GPU work has finished.
    void wait_idle();
    // Run pending AllowProcessEvents callbacks (async readbacks such as Renderer::pick).
    void process_events();

private:
    WGPUInstance instance_ = nullptr;
    WGPUAdapter adapter_ = nullptr;
    WGPUDevice device_ = nullptr;
    AdapterInfo info_;
};

class OffscreenTarget {
public:
    static constexpr WGPUTextureFormat kFormat = WGPUTextureFormat_RGBA8Unorm;
    OffscreenTarget(NativeDevice &dev, uint32_t width, uint32_t height);
    WGPUTextureView view() const {
        return view_.get();
    }
    uint32_t width() const {
        return width_;
    }
    uint32_t height() const {
        return height_;
    }
    // Tightly packed RGBA8 rows, top row first.
    std::vector<uint8_t> read_rgba();

private:
    NativeDevice &dev_;
    uint32_t width_, height_;
    TextureRef texture_;
    TextureViewRef view_;
};

} // namespace adacpp::render

#endif
