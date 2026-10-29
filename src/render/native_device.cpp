#include "native_device.h"

#ifndef __EMSCRIPTEN__

#include <cstdio>
#include <stdexcept>

namespace adacpp::render {

namespace {

std::string to_string(WGPUStringView s) {
    return s.data ? std::string(s.data, s.length == WGPU_STRLEN ? std::strlen(s.data) : s.length) : std::string();
}

const char *backend_name(WGPUBackendType t) {
    switch (t) {
    case WGPUBackendType_Vulkan:
        return "vulkan";
    case WGPUBackendType_Metal:
        return "metal";
    case WGPUBackendType_D3D12:
        return "dx12";
    case WGPUBackendType_OpenGL:
        return "gl";
    case WGPUBackendType_OpenGLES:
        return "gles";
    default:
        return "other";
    }
}

} // namespace

NativeDevice::NativeDevice(const std::string &backend) {
    WGPUInstanceExtras extras = {};
    extras.chain.sType = (WGPUSType) WGPUSType_InstanceExtras;
    if (backend == "vulkan")
        extras.backends = WGPUInstanceBackend_Vulkan;
    else if (backend == "gl")
        extras.backends = WGPUInstanceBackend_GL;
    else if (backend == "metal")
        extras.backends = WGPUInstanceBackend_Metal;
    else if (backend == "dx12")
        extras.backends = WGPUInstanceBackend_DX12;
    else
        extras.backends = WGPUInstanceBackend_All;
    WGPUInstanceDescriptor idesc = WGPU_INSTANCE_DESCRIPTOR_INIT;
    idesc.nextInChain = &extras.chain;
    instance_ = wgpuCreateInstance(&idesc);
    if (!instance_)
        throw std::runtime_error("adacpp.render: wgpuCreateInstance failed");

    struct AdapterResult {
        WGPUAdapter adapter = nullptr;
        std::string message;
        bool done = false;
    } ar;
    WGPURequestAdapterOptions opts = WGPU_REQUEST_ADAPTER_OPTIONS_INIT;
    opts.powerPreference = WGPUPowerPreference_HighPerformance;
    WGPURequestAdapterCallbackInfo acb = WGPU_REQUEST_ADAPTER_CALLBACK_INFO_INIT;
    acb.mode = WGPUCallbackMode_AllowProcessEvents;
    acb.callback = [](WGPURequestAdapterStatus status, WGPUAdapter adapter, WGPUStringView message, void *ud1, void *) {
        auto *r = static_cast<AdapterResult *>(ud1);
        if (status == WGPURequestAdapterStatus_Success)
            r->adapter = adapter;
        else
            r->message = to_string(message);
        r->done = true;
    };
    acb.userdata1 = &ar;
    wgpuInstanceRequestAdapter(instance_, &opts, acb);
    while (!ar.done)
        wgpuInstanceProcessEvents(instance_);
    if (!ar.adapter)
        throw std::runtime_error("adacpp.render: no WebGPU adapter (" + ar.message + ")");
    adapter_ = ar.adapter;

    WGPUAdapterInfo ainfo = WGPU_ADAPTER_INFO_INIT;
    wgpuAdapterGetInfo(adapter_, &ainfo);
    info_.vendor = to_string(ainfo.vendor);
    info_.architecture = to_string(ainfo.architecture);
    info_.device = to_string(ainfo.device);
    info_.description = to_string(ainfo.description);
    info_.backend = backend_name(ainfo.backendType);
    wgpuAdapterInfoFreeMembers(ainfo);

    // Large models put more than the default 128 MiB in one storage binding; ask for what the adapter has.
    WGPULimits limits = WGPU_LIMITS_INIT;
    wgpuAdapterGetLimits(adapter_, &limits);
    WGPULimits required = WGPU_LIMITS_INIT;
    required.maxStorageBufferBindingSize = limits.maxStorageBufferBindingSize;
    required.maxBufferSize = limits.maxBufferSize;

    struct DeviceResult {
        WGPUDevice device = nullptr;
        std::string message;
        bool done = false;
    } dr;
    WGPUDeviceDescriptor ddesc = WGPU_DEVICE_DESCRIPTOR_INIT;
    ddesc.label = sv("adacpp.render");
    ddesc.requiredLimits = &required;
    // Never throw through this callback: it is called from inside wgpu-native (C ABI).
    ddesc.uncapturedErrorCallbackInfo.callback = [](WGPUDevice const *, WGPUErrorType type, WGPUStringView message,
                                                    void *, void *) {
        std::fprintf(stderr, "adacpp.render: WebGPU error %d: %s\n", int(type), to_string(message).c_str());
    };
    WGPURequestDeviceCallbackInfo dcb = WGPU_REQUEST_DEVICE_CALLBACK_INFO_INIT;
    dcb.mode = WGPUCallbackMode_AllowProcessEvents;
    dcb.callback = [](WGPURequestDeviceStatus status, WGPUDevice device, WGPUStringView message, void *ud1, void *) {
        auto *r = static_cast<DeviceResult *>(ud1);
        if (status == WGPURequestDeviceStatus_Success)
            r->device = device;
        else
            r->message = to_string(message);
        r->done = true;
    };
    dcb.userdata1 = &dr;
    wgpuAdapterRequestDevice(adapter_, &ddesc, dcb);
    while (!dr.done)
        wgpuInstanceProcessEvents(instance_);
    if (!dr.device)
        throw std::runtime_error("adacpp.render: device request failed (" + dr.message + ")");
    device_ = dr.device;
}

NativeDevice::~NativeDevice() {
    if (device_)
        wgpuDeviceRelease(device_);
    if (adapter_)
        wgpuAdapterRelease(adapter_);
    if (instance_)
        wgpuInstanceRelease(instance_);
}

void NativeDevice::wait_idle() {
    wgpuDevicePoll(device_, true, nullptr);
}

void NativeDevice::process_events() {
    wgpuDevicePoll(device_, true, nullptr);
    wgpuInstanceProcessEvents(instance_);
}

OffscreenTarget::OffscreenTarget(NativeDevice &dev, uint32_t width, uint32_t height)
    : dev_(dev), width_(width), height_(height) {
    WGPUTextureDescriptor d = WGPU_TEXTURE_DESCRIPTOR_INIT;
    d.label = sv("adacpp.offscreen");
    d.size = {width, height, 1};
    d.format = kFormat;
    d.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc;
    texture_.reset(wgpuDeviceCreateTexture(dev.device(), &d));
    view_.reset(wgpuTextureCreateView(texture_.get(), nullptr));
}

std::vector<uint8_t> OffscreenTarget::read_rgba() {
    const uint32_t row = width_ * 4;
    const uint32_t padded = (row + 255) & ~255u; // bytesPerRow must be a multiple of 256
    WGPUBufferDescriptor bd = WGPU_BUFFER_DESCRIPTOR_INIT;
    bd.label = sv("adacpp.readback");
    bd.size = uint64_t(padded) * height_;
    bd.usage = WGPUBufferUsage_CopyDst | WGPUBufferUsage_MapRead;
    BufferRef buf(wgpuDeviceCreateBuffer(dev_.device(), &bd));

    WGPUCommandEncoderDescriptor ed = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
    WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(dev_.device(), &ed);
    WGPUTexelCopyTextureInfo src = WGPU_TEXEL_COPY_TEXTURE_INFO_INIT;
    src.texture = texture_.get();
    WGPUTexelCopyBufferInfo dst = WGPU_TEXEL_COPY_BUFFER_INFO_INIT;
    dst.buffer = buf.get();
    dst.layout.bytesPerRow = padded;
    dst.layout.rowsPerImage = height_;
    WGPUExtent3D extent = {width_, height_, 1};
    wgpuCommandEncoderCopyTextureToBuffer(enc, &src, &dst, &extent);
    WGPUCommandBufferDescriptor cd = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, &cd);
    WGPUQueue queue = wgpuDeviceGetQueue(dev_.device());
    wgpuQueueSubmit(queue, 1, &cmd);
    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(enc);

    bool done = false;
    WGPUMapAsyncStatus status = WGPUMapAsyncStatus_Error;
    WGPUBufferMapCallbackInfo mcb = WGPU_BUFFER_MAP_CALLBACK_INFO_INIT;
    mcb.mode = WGPUCallbackMode_AllowProcessEvents;
    mcb.callback = [](WGPUMapAsyncStatus s, WGPUStringView, void *ud1, void *ud2) {
        *static_cast<WGPUMapAsyncStatus *>(ud1) = s;
        *static_cast<bool *>(ud2) = true;
    };
    mcb.userdata1 = &status;
    mcb.userdata2 = &done;
    wgpuBufferMapAsync(buf.get(), WGPUMapMode_Read, 0, bd.size, mcb);
    while (!done)
        wgpuDevicePoll(dev_.device(), true, nullptr);
    wgpuQueueRelease(queue);
    if (status != WGPUMapAsyncStatus_Success)
        throw std::runtime_error("adacpp.render: readback map failed");

    const auto *mapped = static_cast<const uint8_t *>(wgpuBufferGetConstMappedRange(buf.get(), 0, bd.size));
    std::vector<uint8_t> out(size_t(row) * height_);
    for (uint32_t y = 0; y < height_; ++y)
        std::memcpy(out.data() + size_t(y) * row, mapped + size_t(y) * padded, row);
    wgpuBufferUnmap(buf.get());
    return out;
}

} // namespace adacpp::render

#endif
