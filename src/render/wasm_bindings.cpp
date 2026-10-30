// Browser entry point: embind API over Renderer, drawing into a <canvas> through emdawnwebgpu.
//
// The page requests the GPUDevice itself (so it can ask for the adapter's limits and fall back when
// WebGPU is unavailable) and hands it over as `preinitializedWebGPUDevice` when instantiating the module.
// Bulk data crosses as pointers into the wasm heap: the caller _malloc()s, fills HEAPF32/HEAPU32 and
// passes the byte address, then frees it. Every call copies what it needs before returning.
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>
#include <string>

#include "renderer.h"

extern "C" WGPUDevice emscripten_webgpu_get_device(void);

namespace adacpp::render {

namespace {

template <typename T> const T *ptr(uintptr_t p) {
    return p ? reinterpret_cast<const T *>(p) : nullptr;
}

WGPUTextureFormat parse_format(const std::string &f) {
    if (f == "rgba8unorm")
        return WGPUTextureFormat_RGBA8Unorm;
    return WGPUTextureFormat_BGRA8Unorm;
}

} // namespace

class WebRenderer {
public:
    WebRenderer(const std::string &canvas_selector, const std::string &format)
        : format_(parse_format(format)), device_(emscripten_webgpu_get_device()) {
        instance_ = wgpuCreateInstance(nullptr);
        WGPUEmscriptenSurfaceSourceCanvasHTMLSelector sel = WGPU_EMSCRIPTEN_SURFACE_SOURCE_CANVAS_HTML_SELECTOR_INIT;
        sel.selector = WGPUStringView{selector_storage(canvas_selector), canvas_selector.size()};
        WGPUSurfaceDescriptor sd = WGPU_SURFACE_DESCRIPTOR_INIT;
        sd.nextInChain = &sel.chain;
        surface_ = wgpuInstanceCreateSurface(instance_, &sd);
        renderer_ = std::make_unique<Renderer>(device_, format_);
    }
    ~WebRenderer() {
        renderer_.reset();
        if (surface_) {
            wgpuSurfaceUnconfigure(surface_);
            wgpuSurfaceRelease(surface_);
        }
        if (instance_)
            wgpuInstanceRelease(instance_);
        if (device_)
            wgpuDeviceRelease(device_);
    }

    void resize(uint32_t width, uint32_t height) {
        if (width == width_ && height == height_)
            return;
        width_ = width;
        height_ = height;
        if (!width || !height)
            return;
        WGPUSurfaceConfiguration c = WGPU_SURFACE_CONFIGURATION_INIT;
        c.device = device_;
        c.format = format_;
        c.usage = WGPUTextureUsage_RenderAttachment;
        c.width = width;
        c.height = height;
        c.alphaMode = WGPUCompositeAlphaMode_Opaque;
        c.presentMode = WGPUPresentMode_Fifo;
        wgpuSurfaceConfigure(surface_, &c);
    }

    void render() {
        if (!width_ || !height_)
            return;
        WGPUSurfaceTexture st = WGPU_SURFACE_TEXTURE_INIT;
        wgpuSurfaceGetCurrentTexture(surface_, &st);
        if (!st.texture)
            return;
        WGPUTextureView view = wgpuTextureCreateView(st.texture, nullptr);
        renderer_->render(view, width_, height_);
        wgpuTextureViewRelease(view);
        wgpuTextureRelease(st.texture);
    }

    uint32_t add_geometry(uint32_t topology, uintptr_t positions, uint32_t vertex_count, uintptr_t normals,
                          uintptr_t colors, uintptr_t morph, uintptr_t indices, uint32_t index_count,
                          uintptr_t segments, uint32_t segment_count) {
        GeometryDesc d;
        d.topology = static_cast<Topology>(topology);
        d.positions = ptr<float>(positions);
        d.vertex_count = vertex_count;
        d.normals = ptr<float>(normals);
        d.colors = ptr<uint32_t>(colors);
        d.morph = ptr<float>(morph);
        d.indices = ptr<uint32_t>(indices);
        d.index_count = index_count;
        d.segments = ptr<uint32_t>(segments);
        d.segment_count = segment_count;
        return renderer_->add_geometry(d);
    }
    void update_colors(uint32_t geometry, uintptr_t colors, uint32_t count) {
        renderer_->update_colors(geometry, ptr<uint32_t>(colors), count);
    }
    void update_morph(uint32_t geometry, uintptr_t morph, uint32_t vertex_count) {
        renderer_->update_morph(geometry, ptr<float>(morph), vertex_count);
    }
    void remove_geometry(uint32_t geometry) {
        renderer_->remove_geometry(geometry);
    }
    uint32_t segment_count(uint32_t geometry) const {
        return renderer_->segment_count(geometry);
    }

    uint32_t add_instance(uint32_t geometry) {
        return renderer_->add_instance(geometry);
    }
    void set_instance(uint32_t instance, uintptr_t matrix, float r, float g, float b, float a, uint32_t flags,
                      float morph_influence, float metalness, float roughness, bool specular) {
        InstanceDesc d;
        const float *m = ptr<float>(matrix);
        std::copy(m, m + 16, d.model.begin());
        d.color = {r, g, b, a};
        d.flags = flags;
        d.morph_influence = morph_influence;
        d.metalness = metalness;
        d.roughness = roughness;
        d.specular = specular;
        renderer_->set_instance(instance, d);
    }
    void set_segment_states(uint32_t instance, uintptr_t packed, uint32_t count) {
        renderer_->set_segment_states(instance, ptr<uint32_t>(packed), count);
    }
    void remove_instance(uint32_t instance) {
        renderer_->remove_instance(instance);
    }
    void clear() {
        renderer_->clear();
    }

    void set_camera(uintptr_t view, uintptr_t proj, bool orthographic) {
        renderer_->set_camera(ptr<float>(view), ptr<float>(proj), orthographic);
    }
    void set_background(float r, float g, float b) {
        renderer_->set_background(r, g, b, 1.0f);
    }
    void set_lighting(float ambient, float diffuse) {
        renderer_->set_lighting(ambient, diffuse);
    }
    void set_culling(bool frustum, bool cone, bool occlusion) {
        renderer_->set_culling(frustum, cone, occlusion);
    }

    // GPU pick at canvas pixel (x, y) of a width x height drawing buffer; `callback(result)` runs once the
    // readback lands: {hit, instance, segment, triangle, world: [x, y, z]}.
    void pick(uint32_t x, uint32_t y, uint32_t width, uint32_t height, emscripten::val callback) {
        renderer_->pick(x, y, width, height, [callback](const PickResult &r) {
            auto o = emscripten::val::object();
            o.set("hit", r.hit);
            o.set("instance", r.instance);
            o.set("segment", r.segment);
            o.set("triangle", r.triangle);
            auto w = emscripten::val::array();
            for (int i = 0; i < 3; ++i)
                w.call<void>("push", r.world[i]);
            o.set("world", w);
            callback(o);
        });
    }

    emscripten::val stats() const {
        return stats_object(renderer_->stats());
    }

    // The last frame's stats with the GPU culling counts read back; `callback(stats)` once they land.
    void read_cull_stats(emscripten::val callback) {
        renderer_->read_cull_stats([callback](const RenderStats &s) { callback(stats_object(s)); });
    }

private:
    static emscripten::val stats_object(const RenderStats &s) {
        auto o = emscripten::val::object();
        o.set("geometries", s.geometries);
        o.set("instances", s.instances);
        o.set("clustersTotal", double(s.clusters_total));
        o.set("clustersDrawn", double(s.clusters_drawn));
        o.set("trianglesTotal", double(s.triangles_total));
        o.set("trianglesDrawn", double(s.triangles_drawn));
        o.set("clustersOccluded", double(s.clusters_occluded));
        o.set("gpuCulled", s.gpu_culled);
        o.set("drawCalls", s.draw_calls);
        o.set("cullMs", s.cull_ms);
        return o;
    }

    const char *selector_storage(const std::string &s) {
        selector_ = s;
        return selector_.c_str();
    }

    std::string selector_;
    WGPUTextureFormat format_;
    WGPUDevice device_ = nullptr;
    WGPUInstance instance_ = nullptr;
    WGPUSurface surface_ = nullptr;
    std::unique_ptr<Renderer> renderer_;
    uint32_t width_ = 0, height_ = 0;
};

} // namespace adacpp::render

EMSCRIPTEN_BINDINGS(adacpp_render) {
    using adacpp::render::WebRenderer;
    emscripten::class_<WebRenderer>("WebRenderer")
        .constructor<const std::string &, const std::string &>()
        .function("resize", &WebRenderer::resize)
        .function("render", &WebRenderer::render)
        .function("addGeometry", &WebRenderer::add_geometry)
        .function("updateColors", &WebRenderer::update_colors)
        .function("updateMorph", &WebRenderer::update_morph)
        .function("removeGeometry", &WebRenderer::remove_geometry)
        .function("segmentCount", &WebRenderer::segment_count)
        .function("addInstance", &WebRenderer::add_instance)
        .function("setInstance", &WebRenderer::set_instance)
        .function("setSegmentStates", &WebRenderer::set_segment_states)
        .function("removeInstance", &WebRenderer::remove_instance)
        .function("clear", &WebRenderer::clear)
        .function("setCamera", &WebRenderer::set_camera)
        .function("setBackground", &WebRenderer::set_background)
        .function("setLighting", &WebRenderer::set_lighting)
        .function("setCulling", &WebRenderer::set_culling)
        .function("pick", &WebRenderer::pick)
        .function("stats", &WebRenderer::stats)
        .function("readCullStats", &WebRenderer::read_cull_stats);
}
