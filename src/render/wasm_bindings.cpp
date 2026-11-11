// Browser entry point: embind API over Renderer, drawing into a <canvas> through emdawnwebgpu.
//
// The page requests the GPUDevice itself (so it can ask for the adapter's limits and fall back when
// WebGPU is unavailable) and hands it over as `preinitializedWebGPUDevice` when instantiating the module.
// Bulk data crosses as pointers into the wasm heap: the caller _malloc()s, fills HEAPF32/HEAPU32 and
// passes the byte address, then frees it. Every call copies what it needs before returning.
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "glb_scene.h"
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

// A JS typed array holding a copy of `data` (a view of the heap would dangle once `data` goes).
template <typename T> emscripten::val typed_copy(const char *type, const std::vector<T> &data) {
    return emscripten::val::global(type).new_(emscripten::typed_memory_view(data.size(), data.data()));
}

// A JS array of strings in one crossing: per-element calls cost too much for tens of thousands of names.
template <typename Get> emscripten::val string_array(const std::vector<GlbObject> &objects, Get get) {
    std::string json = "[";
    for (const GlbObject &o : objects) {
        if (json.size() > 1)
            json += ',';
        json += '"';
        for (const unsigned char c : get(o)) {
            if (c == '"' || c == '\\') {
                json += '\\';
                json += char(c);
            } else if (c < 0x20) {
                static const char hex[] = "0123456789abcdef";
                json += "\\u00";
                json += hex[c >> 4];
                json += hex[c & 15];
            } else {
                json += char(c);
            }
        }
        json += '"';
    }
    json += ']';
    return emscripten::val::global("JSON").call<emscripten::val>("parse", json);
}

// The world box of a GLB (its bytes at `bytes`, `length` long) without loading it: [min x, y, z, max x, y, z]
// as doubles, as load_glb_bytes would place it with no offset (glb_bounds_bytes; min > max when nothing
// is drawable). For choosing loadGlb's `offset` before the load.
emscripten::val glb_bounds(uintptr_t bytes, uint32_t length, bool y_up) {
    const auto *p = ptr<unsigned char>(bytes);
    const std::array<double, 6> b = glb_bounds_bytes(std::vector<unsigned char>(p, p + length), y_up);
    emscripten::val out = emscripten::val::array();
    for (double v : b)
        out.call<void>("push", v);
    return out;
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
        renderer_->camera().set_viewport(width, height); // what the camera's pixel arguments are measured in
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

    // A GLB (its bytes at `bytes`, `length` long) added to the scene through the same reader the Python
    // offscreen renderer uses (glb_scene.h), so a model draws as its poster does. `opts` (all optional):
    // {yUp, lineWidth, memberWidth, pointSize, lineColor: [r, g, b, a] linear, doubleSided, sectionCaps,
    //  featureEdgeAngleDeg, keepPositions, animations, offset: [x, y, z] (GlbOptions::offset, as doubles;
    //  bounds, positions and matrices below include it)}. Returns {bounds: [6], featureEdges, geometries:
    // [{geometry, instance, topology, vertexCount, vertexColors, featureEdges, node, segmentObjects:
    // Int32Array, positions?: Float32Array, indices?: Uint32Array}]} (positions and indices with
    // keepPositions; node: the glTF node placing an animated geometry, else -1), and the
    // model's objects (GlbLoadResult::objects) as tables, row i being object i:
    //   objects: {count, ids: string[], names: string[], parents: Int32Array, bounds: Float32Array (6 per
    //             object), triangles: Uint32Array, segmentOffsets: Uint32Array (count + 1),
    //             segments: Uint32Array (instance, segment pairs)}
    // Object i's own segments are pairs segmentOffsets[i] .. segmentOffsets[i + 1] of `segments`; a
    // geometry's segmentObjects[s] is the object segment s draws (-1 = none), which is how a pick's
    // (instance, segment) names an object. nodeObjects: the objects are glTF nodes, not adapy ids. extras /
    // adaExtData: the scene extras and the ADA_EXT_data extension as JSON text ("" when absent).
    // With `animations` (GlbOptions::animations): animationSet, the handle setAnimationTime takes (-1 when
    // the GLB has no animations), animations: [{name, duration}] and animatedNodes: Int32Array, the glTF
    // nodes kept apart, in the order animationNodeMatrices reports them.
    emscripten::val load_glb(uintptr_t bytes, uint32_t length, emscripten::val opts) {
        auto num = [&](const char *k, float d) {
            const emscripten::val v = opts[k];
            return v.isNumber() ? v.as<float>() : d;
        };
        auto flag = [&](const char *k) {
            const emscripten::val v = opts[k];
            return !v.isUndefined() && !v.isNull() && v.as<bool>();
        };
        GlbOptions o;
        if (!opts.isUndefined() && !opts.isNull()) {
            o.y_up_to_z_up = flag("yUp");
            o.line_width = num("lineWidth", 1.0f);
            o.member_width = num("memberWidth", 1.0f);
            o.point_size = num("pointSize", 1.0f);
            o.double_sided = flag("doubleSided");
            o.section_caps = flag("sectionCaps");
            o.feature_edge_angle_deg = num("featureEdgeAngleDeg", 0.0f);
            o.keep_positions = flag("keepPositions");
            o.animations = flag("animations");
            const emscripten::val c = opts["lineColor"];
            if (c.isArray())
                for (int i = 0; i < 4; ++i)
                    o.line_color[i] = c[i].as<float>();
            const emscripten::val offset = opts["offset"];
            if (offset.isArray())
                for (int i = 0; i < 3; ++i)
                    o.offset[i] = offset[i].as<double>();
        }
        const auto *p = ptr<unsigned char>(bytes);
        GlbLoadResult r = load_glb_bytes(*renderer_, std::vector<unsigned char>(p, p + length), o);
        emscripten::val out = emscripten::val::object();
        emscripten::val bounds = emscripten::val::array();
        for (float b : r.bounds)
            bounds.call<void>("push", b);
        out.set("bounds", bounds);
        out.set("featureEdges", double(r.feature_edges));
        emscripten::val list = emscripten::val::array();
        for (const GlbGeometry &g : r.loaded) {
            emscripten::val e = emscripten::val::object();
            e.set("geometry", g.geometry);
            e.set("instance", g.instance);
            e.set("topology", uint32_t(g.topology));
            e.set("vertexCount", g.vertex_count);
            e.set("vertexColors", g.vertex_colors);
            e.set("featureEdges", g.feature_edges);
            e.set("node", g.node);
            e.set("segmentObjects", typed_copy("Int32Array", g.segment_objects));
            if (!g.positions.empty())
                e.set("positions", typed_copy("Float32Array", g.positions));
            if (!g.indices.empty())
                e.set("indices", typed_copy("Uint32Array", g.indices));
            list.call<void>("push", e);
        }
        out.set("geometries", list);

        const size_t n = r.objects.size();
        std::vector<int32_t> parents(n);
        std::vector<float> object_bounds(n * 6);
        std::vector<uint32_t> triangles(n), offsets(n + 1, 0), segments;
        for (size_t i = 0; i < n; ++i) {
            const GlbObject &o = r.objects[i];
            parents[i] = o.parent;
            std::copy(o.bounds.begin(), o.bounds.end(), object_bounds.begin() + i * 6);
            triangles[i] = o.triangles;
            for (const auto &pair : o.segments)
                segments.insert(segments.end(), pair.begin(), pair.end());
            offsets[i + 1] = uint32_t(segments.size() / 2);
        }
        emscripten::val objects = emscripten::val::object();
        objects.set("count", double(n));
        objects.set("ids", string_array(r.objects, [](const GlbObject &o) -> const std::string & { return o.id; }));
        objects.set("names", string_array(r.objects, [](const GlbObject &o) -> const std::string & { return o.name; }));
        objects.set("parents", typed_copy("Int32Array", parents));
        objects.set("bounds", typed_copy("Float32Array", object_bounds));
        objects.set("triangles", typed_copy("Uint32Array", triangles));
        objects.set("segmentOffsets", typed_copy("Uint32Array", offsets));
        objects.set("segments", typed_copy("Uint32Array", segments));
        out.set("objects", objects);
        out.set("nodeObjects", r.node_objects);
        out.set("extras", r.extras);
        out.set("adaExtData", r.ada_ext_data);
        emscripten::val clips = emscripten::val::array();
        for (size_t i = 0; i < r.animations.count(); ++i) {
            emscripten::val c = emscripten::val::object();
            c.set("name", r.animations.name(i));
            c.set("duration", r.animations.duration(i));
            clips.call<void>("push", c);
        }
        out.set("animations", clips);
        out.set("animatedNodes", typed_copy("Int32Array", r.animations.nodes()));
        int set = -1;
        if (r.animations.count() > 0) {
            // A freed slot first, so a page that loads and unloads models does not grow the list.
            for (size_t i = 0; i < animation_sets_.size() && set < 0; ++i)
                if (animation_sets_[i].count() == 0)
                    set = int(i);
            if (set < 0) {
                set = int(animation_sets_.size());
                animation_sets_.emplace_back();
            }
            animation_sets_[set] = r.animations;
        }
        out.set("animationSet", set);
        return out;
    }

    // The animations of a loadGlb with `animations` (its animationSet). setAnimationTime poses the instances
    // that load kept apart: `animation` -1 = the rest pose, else that animation at `seconds`, held at its
    // first / last keyframe outside its range (the page wraps the time to loop). Not blended: one animation
    // of a set at a time.
    uint32_t animation_count(int set) const {
        return uint32_t(animation_set(set).count());
    }
    std::string animation_name(int set, uint32_t animation) const {
        return animation_set(set).name(animation);
    }
    float animation_duration(int set, uint32_t animation) const {
        return animation_set(set).duration(animation);
    }
    void set_animation_time(int set, int animation, float seconds) {
        animation_set(set).apply(*renderer_, animation, seconds);
    }
    // Float32Array, 16 per node of the load's animatedNodes: each one's world matrix as last posed.
    emscripten::val animation_node_matrices(int set) const {
        const GlbAnimations &a = animation_set(set);
        std::vector<float> out;
        for (size_t i = 0; i < a.nodes().size(); ++i) {
            const auto m = a.node_matrix(i);
            out.insert(out.end(), m.begin(), m.end());
        }
        return typed_copy("Float32Array", out);
    }
    // Forget a set (its instances are the page's to remove); clear() forgets them all.
    void remove_animations(int set) {
        animation_set(set);
        animation_sets_[set] = GlbAnimations();
    }

    uint32_t add_geometry(uint32_t topology, uintptr_t positions, uint32_t vertex_count, uintptr_t normals,
                          uintptr_t colors, uintptr_t morph, uintptr_t indices, uint32_t index_count,
                          uintptr_t segments, uint32_t segment_count, uintptr_t distances) {
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
        d.distances = ptr<float>(distances);
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
                      float morph_influence, float metalness, float roughness, bool specular, float line_width,
                      float dash, float gap) {
        InstanceDesc d;
        const float *m = ptr<float>(matrix);
        std::copy(m, m + 16, d.model.begin());
        d.color = {r, g, b, a};
        d.flags = flags;
        d.morph_influence = morph_influence;
        d.metalness = metalness;
        d.roughness = roughness;
        d.specular = specular;
        d.line_width = line_width;
        d.dash = dash;
        d.gap = gap;
        renderer_->set_instance(instance, d);
    }
    // The instance's matrix alone (16 floats in the heap), the rest of setInstance's values kept.
    void set_instance_matrix(uint32_t instance, uintptr_t matrix) {
        renderer_->set_instance_model(instance, ptr<float>(matrix));
    }
    // Float32Array(16): the instance's matrix as it is now (where an animation has put it).
    emscripten::val instance_matrix(uint32_t instance) const {
        const auto &m = renderer_->instance_desc(instance).model;
        return emscripten::val::global("Float32Array").new_(emscripten::typed_memory_view(16, m.data()));
    }
    void set_segment_states(uint32_t instance, uintptr_t packed, uint32_t count) {
        renderer_->set_segment_states(instance, ptr<uint32_t>(packed), count);
    }
    void remove_instance(uint32_t instance) {
        renderer_->remove_instance(instance);
    }
    void clear() {
        renderer_->clear();
        animation_sets_.clear();
    }

    void set_camera(uintptr_t view, uintptr_t proj, bool orthographic) {
        renderer_->set_camera(ptr<float>(view), ptr<float>(proj), orthographic);
    }

    // The renderer's own camera (camera_controller.h). The page maps its pointer, wheel and key events to
    // these; with setCameraControlled(true) render() and pick() draw from it and setCamera() is ignored.
    // Pixels are drawing-buffer pixels as in pick() (x right, y down, origin top-left), angles degrees.
    void set_camera_controlled(bool controlled) {
        renderer_->set_camera_controlled(controlled);
    }
    void camera_orbit(double dx, double dy) {
        renderer_->camera().orbit(dx, dy);
    }
    void camera_pan(double dx, double dy) {
        renderer_->camera().pan(dx, dy);
    }
    // Below 1 moves in, above 1 moves out.
    void camera_zoom(double factor) {
        renderer_->camera().zoom(factor);
    }
    void camera_zoom_at(double factor, double x, double y) {
        renderer_->camera().zoom_at(factor, x, y);
    }
    void camera_roll(double angle_deg) {
        renderer_->camera().roll(angle_deg);
    }
    // Frame the box (min xyz, max xyz); `margin` 0 = the default (1.15); `seconds` 0 = at once.
    void camera_fit(float x0, float y0, float z0, float x1, float y1, float z1, double margin, double seconds) {
        renderer_->camera().fit({x0, y0, z0, x1, y1, z1}, margin, seconds);
    }
    // "top" | "bottom" | "front" | "back" | "left" | "right" | "iso"; false for any other name.
    bool camera_view(const std::string &name, double seconds) {
        return renderer_->camera().set_view(name, seconds);
    }
    void camera_set_orientation(double azimuth_deg, double elevation_deg, double roll_deg, double seconds) {
        renderer_->camera().set_orientation(azimuth_deg, elevation_deg, roll_deg, seconds);
    }
    // `ortho_height` 0 = what the perspective view shows at the target.
    void camera_set_pose(double ex, double ey, double ez, double tx, double ty, double tz, double ux, double uy,
                         double uz, double fov_deg, bool orthographic, double ortho_height, double seconds) {
        CameraPose p;
        p.eye = {ex, ey, ez};
        p.target = {tx, ty, tz};
        p.up = {ux, uy, uz};
        p.fov_deg = fov_deg;
        p.orthographic = orthographic;
        p.ortho_height = ortho_height;
        renderer_->camera().set_pose(p, seconds);
    }
    // {eye: [3], target: [3], up: [3], fov, orthographic, orthoHeight, azimuth, elevation, roll, distance, near,
    // far}: the first six are what cameraSetPose takes, then the same pose as the controller holds it, and the
    // clip planes the projection uses (cameraSetClipRange's, or the automatic ones).
    emscripten::val camera_pose() const {
        const CameraController &c = renderer_->camera();
        const CameraPose p = c.pose();
        auto vec = [](const std::array<double, 3> &v) {
            auto a = emscripten::val::array();
            for (double x : v)
                a.call<void>("push", x);
            return a;
        };
        auto o = emscripten::val::object();
        o.set("eye", vec(p.eye));
        o.set("target", vec(p.target));
        o.set("up", vec(p.up));
        o.set("fov", p.fov_deg);
        o.set("orthographic", p.orthographic);
        o.set("orthoHeight", p.ortho_height);
        o.set("azimuth", c.azimuth());
        o.set("elevation", c.elevation());
        o.set("roll", c.roll_angle());
        o.set("distance", c.distance());
        const std::array<double, 2> clip = c.clip_range();
        o.set("near", clip[0]);
        o.set("far", clip[1]);
        return o;
    }
    // {view, proj}: the controller's matrices for the canvas as it is now, as Float32Array(16), column-major,
    // the projection in OpenGL clip space (what setCamera takes), for a page that projects points itself.
    emscripten::val camera_matrices() const {
        const CameraController &c = renderer_->camera();
        const std::array<float, 16> view = c.view(), proj = c.projection(c.aspect());
        auto o = emscripten::val::object();
        o.set("view", emscripten::val::global("Float32Array").new_(emscripten::typed_memory_view(16, view.data())));
        o.set("proj", emscripten::val::global("Float32Array").new_(emscripten::typed_memory_view(16, proj.data())));
        return o;
    }
    // Near and far planes for the projection to use as they are; (0, 0) = the automatic ones that wrap the
    // scene (the default). Either projection; clip space stays OpenGL's, z in [-1, 1].
    void camera_set_clip_range(double near_z, double far_z) {
        renderer_->camera().set_clip_range(near_z, far_z);
    }
    void camera_set_orthographic(bool orthographic) {
        renderer_->camera().set_orthographic(orthographic);
    }
    void camera_set_fov(double fov_deg) {
        renderer_->camera().set_fov(fov_deg);
    }
    // "z" (the default) or "y".
    void camera_set_up_axis(const std::string &axis) {
        renderer_->camera().set_up_axis(axis == "y" || axis == "Y" ? UpAxis::Y : UpAxis::Z);
    }
    // Inertia of orbit and pan, 0 (off) to below 1; needs cameraUpdate every frame.
    void camera_set_damping(double inertia) {
        renderer_->camera().set_damping(inertia);
    }
    void camera_set_pivot(double x, double y, double z) {
        renderer_->camera().set_orbit_pivot({x, y, z});
    }
    void camera_clear_pivot() {
        renderer_->camera().clear_orbit_pivot();
    }
    // Advance a transition and the damping by `dt` seconds; true while the page should keep drawing.
    bool camera_update(double dt) {
        return renderer_->camera().update(dt);
    }
    bool camera_moving() const {
        return renderer_->camera().moving();
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
    // `planes`: `count` x (nx, ny, nz, constant) floats in the wasm heap.
    void set_clip_planes(uintptr_t planes, uint32_t count) {
        renderer_->set_clip_planes(ptr<float>(planes), count);
    }

    // Fill the cut through kCapped instances (see Renderer::set_section_caps).
    void set_section_caps(bool enabled) {
        renderer_->set_section_caps(enabled);
    }
    // One linear RGB colour for every cap; `a` 0 = each instance's base colour.
    void set_section_cap_color(float r, float g, float b, float a) {
        renderer_->set_section_cap_color(r, g, b, a);
    }

    // Sprite textures: `rgba` points at width x height RGBA8 texels in the wasm heap, top row first,
    // sRGB-encoded with straight alpha (ImageData of a canvas).
    uint32_t add_texture(uintptr_t rgba, uint32_t width, uint32_t height) {
        return renderer_->add_texture(ptr<uint8_t>(rgba), width, height);
    }
    void update_texture(uint32_t texture, uintptr_t rgba, uint32_t width, uint32_t height) {
        renderer_->update_texture(texture, ptr<uint8_t>(rgba), width, height);
    }
    void remove_texture(uint32_t texture) {
        renderer_->remove_texture(texture);
    }
    // As addTexture, without a mip chain: for an image replaced often (updateTexture is then one upload,
    // in place when the size is unchanged), drawn near its own resolution or with InstanceFlag Nearest.
    uint32_t add_plain_texture(uintptr_t rgba, uint32_t width, uint32_t height) {
        return renderer_->add_texture(ptr<uint8_t>(rgba), width, height, false);
    }

    // Textured quads in the world (Renderer QuadDesc): unlit, alpha-blended, never picked.
    uint32_t add_quad() {
        return renderer_->add_quad();
    }
    // `matrix` (16 floats in the heap, column-major) places the unit square (x, y in [-0.5, 0.5], z = 0),
    // the texture's top row at y = +0.5; `texture` 0xFFFFFFFF = none (the tint alone); tint linear rgb with
    // `a` the opacity; `flags` are InstanceFlags: Hidden, Clipped, OnTop (no depth test), DepthOffset and
    // Nearest (1 << 19); `renderOrder` as three.js'.
    void set_quad(uint32_t quad, uint32_t texture, uintptr_t matrix, float r, float g, float b, float a, uint32_t flags,
                  float render_order) {
        QuadDesc d;
        d.texture = texture;
        const float *m = ptr<float>(matrix);
        std::copy(m, m + 16, d.model.begin());
        d.color = {r, g, b, a};
        d.flags = flags;
        d.render_order = render_order;
        renderer_->set_quad(quad, d);
    }
    void remove_quad(uint32_t quad) {
        renderer_->remove_quad(quad);
    }

    uint32_t add_sprite() {
        return renderer_->add_sprite();
    }
    // three.js' Sprite: `texture` 0xFFFFFFFF = none (a quad of the tint), world anchor, sprite.scale,
    // sprite.center, material.rotation, tint (linear rgb, opacity), SpriteFlags, renderOrder.
    void set_sprite(uint32_t sprite, uint32_t texture, float x, float y, float z, float scale_x, float scale_y,
                    float center_x, float center_y, float rotation, float r, float g, float b, float a, uint32_t flags,
                    float render_order) {
        SpriteDesc d;
        d.texture = texture;
        d.position = {x, y, z};
        d.scale = {scale_x, scale_y};
        d.center = {center_x, center_y};
        d.rotation = rotation;
        d.color = {r, g, b, a};
        d.flags = flags;
        d.render_order = render_order;
        renderer_->set_sprite(sprite, d);
    }
    void remove_sprite(uint32_t sprite) {
        renderer_->remove_sprite(sprite);
    }

    // GPU pick at canvas pixel (x, y) of a width x height drawing buffer; `callback(result)` runs once the
    // readback lands: {hit, instance, segment, triangle, topology, world: [x, y, z]}; `topology` 0 = a
    // triangle, 2 = a point, whose index (position in the index list) is in `triangle`.
    void pick(uint32_t x, uint32_t y, uint32_t width, uint32_t height, emscripten::val callback) {
        renderer_->pick(x, y, width, height, [callback](const PickResult &r) {
            auto o = emscripten::val::object();
            o.set("hit", r.hit);
            o.set("instance", r.instance);
            o.set("segment", r.segment);
            o.set("triangle", r.triangle);
            o.set("topology", r.topology);
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

    const GlbAnimations &animation_set(int set) const {
        if (set < 0 || size_t(set) >= animation_sets_.size() || animation_sets_[set].count() == 0)
            throw std::out_of_range("adacpp.render: unknown animation set " + std::to_string(set));
        return animation_sets_[set];
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
    std::vector<GlbAnimations> animation_sets_; // by loadGlb's animationSet; an empty one is a free slot
    uint32_t width_ = 0, height_ = 0;
};

} // namespace adacpp::render

EMSCRIPTEN_BINDINGS(adacpp_render) {
    using adacpp::render::WebRenderer;
    emscripten::function("glbBounds", &adacpp::render::glb_bounds);
    emscripten::class_<WebRenderer>("WebRenderer")
        .constructor<const std::string &, const std::string &>()
        .function("resize", &WebRenderer::resize)
        .function("render", &WebRenderer::render)
        .function("loadGlb", &WebRenderer::load_glb)
        .function("addGeometry", &WebRenderer::add_geometry)
        .function("updateColors", &WebRenderer::update_colors)
        .function("updateMorph", &WebRenderer::update_morph)
        .function("removeGeometry", &WebRenderer::remove_geometry)
        .function("segmentCount", &WebRenderer::segment_count)
        .function("addInstance", &WebRenderer::add_instance)
        .function("setInstance", &WebRenderer::set_instance)
        .function("setInstanceMatrix", &WebRenderer::set_instance_matrix)
        .function("instanceMatrix", &WebRenderer::instance_matrix)
        .function("animationCount", &WebRenderer::animation_count)
        .function("animationName", &WebRenderer::animation_name)
        .function("animationDuration", &WebRenderer::animation_duration)
        .function("setAnimationTime", &WebRenderer::set_animation_time)
        .function("animationNodeMatrices", &WebRenderer::animation_node_matrices)
        .function("removeAnimations", &WebRenderer::remove_animations)
        .function("setSegmentStates", &WebRenderer::set_segment_states)
        .function("removeInstance", &WebRenderer::remove_instance)
        .function("clear", &WebRenderer::clear)
        .function("setCamera", &WebRenderer::set_camera)
        .function("setCameraControlled", &WebRenderer::set_camera_controlled)
        .function("cameraOrbit", &WebRenderer::camera_orbit)
        .function("cameraPan", &WebRenderer::camera_pan)
        .function("cameraZoom", &WebRenderer::camera_zoom)
        .function("cameraZoomAt", &WebRenderer::camera_zoom_at)
        .function("cameraRoll", &WebRenderer::camera_roll)
        .function("cameraFit", &WebRenderer::camera_fit)
        .function("cameraView", &WebRenderer::camera_view)
        .function("cameraSetOrientation", &WebRenderer::camera_set_orientation)
        .function("cameraSetPose", &WebRenderer::camera_set_pose)
        .function("cameraPose", &WebRenderer::camera_pose)
        .function("cameraMatrices", &WebRenderer::camera_matrices)
        .function("cameraSetClipRange", &WebRenderer::camera_set_clip_range)
        .function("cameraSetOrthographic", &WebRenderer::camera_set_orthographic)
        .function("cameraSetFov", &WebRenderer::camera_set_fov)
        .function("cameraSetUpAxis", &WebRenderer::camera_set_up_axis)
        .function("cameraSetDamping", &WebRenderer::camera_set_damping)
        .function("cameraSetPivot", &WebRenderer::camera_set_pivot)
        .function("cameraClearPivot", &WebRenderer::camera_clear_pivot)
        .function("cameraUpdate", &WebRenderer::camera_update)
        .function("cameraMoving", &WebRenderer::camera_moving)
        .function("setBackground", &WebRenderer::set_background)
        .function("setLighting", &WebRenderer::set_lighting)
        .function("setCulling", &WebRenderer::set_culling)
        .function("setClipPlanes", &WebRenderer::set_clip_planes)
        .function("setSectionCaps", &WebRenderer::set_section_caps)
        .function("setSectionCapColor", &WebRenderer::set_section_cap_color)
        .function("addTexture", &WebRenderer::add_texture)
        .function("updateTexture", &WebRenderer::update_texture)
        .function("removeTexture", &WebRenderer::remove_texture)
        .function("addPlainTexture", &WebRenderer::add_plain_texture)
        .function("addQuad", &WebRenderer::add_quad)
        .function("setQuad", &WebRenderer::set_quad)
        .function("removeQuad", &WebRenderer::remove_quad)
        .function("addSprite", &WebRenderer::add_sprite)
        .function("setSprite", &WebRenderer::set_sprite)
        .function("removeSprite", &WebRenderer::remove_sprite)
        .function("pick", &WebRenderer::pick)
        .function("stats", &WebRenderer::stats)
        .function("readCullStats", &WebRenderer::read_cull_stats);
}
