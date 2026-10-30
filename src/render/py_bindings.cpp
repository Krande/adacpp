// adacpp._render: the WebGPU renderer for Python, headless (wgpu-native). adapy wraps it
// (ada.visit.rendering.adacpp_offscreen) with camera presets and the GLB -> image entry points.
#include <cstring>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "glb_scene.h"
#include "native_device.h"
#include "overlay_json.h"
#include "renderer.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace adacpp::render {
namespace {

template <class T> using Array = nb::ndarray<const T, nb::c_contig, nb::device::cpu>;

// One device, renderer and colour target: render() draws the scene and reads it back.
class OffscreenRenderer {
public:
    OffscreenRenderer(uint32_t width, uint32_t height, const std::string &backend)
        : device_(backend), renderer_(device_.device(), OffscreenTarget::kFormat),
          target_(std::make_unique<OffscreenTarget>(device_, width, height)) {}

    nb::dict adapter() const {
        const auto &i = device_.info();
        nb::dict d;
        d["vendor"] = i.vendor;
        d["architecture"] = i.architecture;
        d["device"] = i.device;
        d["description"] = i.description;
        d["backend"] = i.backend;
        return d;
    }

    void resize(uint32_t width, uint32_t height) {
        if (width != target_->width() || height != target_->height())
            target_ = std::make_unique<OffscreenTarget>(device_, width, height);
    }
    std::array<uint32_t, 2> size() const {
        return {target_->width(), target_->height()};
    }

    nb::dict load_glb(const std::string &path, bool y_up, float line_width, float member_width, float point_size,
                      std::optional<std::array<float, 4>> line_color, bool double_sided) {
        GlbOptions o;
        o.y_up_to_z_up = y_up;
        o.line_width = line_width;
        o.member_width = member_width;
        o.point_size = point_size;
        if (line_color)
            o.line_color = *line_color;
        o.double_sided = double_sided;
        GlbLoadResult r;
        {
            nb::gil_scoped_release release;
            r = adacpp::render::load_glb(renderer_, path, o);
        }
        nb::dict d;
        d["geometries"] = r.geometries;
        d["instances"] = r.instances;
        d["vertices"] = r.vertices;
        d["triangles"] = r.triangles;
        d["segments"] = r.segments;
        return d;
    }

    // One geometry + one instance of it; returns the instance id. `colors`: linear RGBA8 per vertex.
    // `depth_offset`: triangles drawn in front of coplanar faces (an overlay on a surface).
    // Lines: `distances` along the line per vertex, drawn dashed with `dash` / `gap` (same units) when dash > 0.
    uint32_t add_mesh(Array<float> positions, std::optional<Array<uint32_t>> indices,
                      std::optional<Array<float>> normals, std::optional<Array<uint8_t>> colors,
                      std::optional<Array<float>> morph, std::optional<Array<uint32_t>> segments,
                      const std::string &topology, std::array<float, 4> color, float metalness, float roughness,
                      bool specular, bool double_sided, bool unlit, float line_width, float morph_influence,
                      bool morph_relative, std::optional<Array<float>> model, bool depth_offset,
                      std::optional<Array<float>> distances, float dash, float gap) {
        const size_t nv = positions.size() / 3;
        if (positions.size() % 3 != 0 || nv == 0)
            throw std::invalid_argument("positions: expected (N, 3) floats");
        auto check = [&](size_t got, size_t want, const char *what) {
            if (got != want)
                throw std::invalid_argument(std::string(what) + ": expected " + std::to_string(want) + " values, got " +
                                            std::to_string(got));
        };
        GeometryDesc gd;
        if (topology == "triangles")
            gd.topology = Topology::Triangles;
        else if (topology == "lines")
            gd.topology = Topology::Lines;
        else if (topology == "points")
            gd.topology = Topology::Points;
        else
            throw std::invalid_argument("topology must be 'triangles', 'lines' or 'points'");
        gd.positions = positions.data();
        gd.vertex_count = nv;
        if (normals) {
            check(normals->size(), nv * 3, "normals");
            gd.normals = normals->data();
        }
        if (colors) {
            check(colors->size(), nv * 4, "colors");
            gd.colors = reinterpret_cast<const uint32_t *>(colors->data()); // RGBA bytes = packed little-endian
        }
        if (morph) {
            check(morph->size(), nv * 3, "morph");
            gd.morph = morph->data();
        }
        if (indices) {
            gd.indices = indices->data();
            gd.index_count = indices->size();
        }
        if (segments) {
            if (segments->size() % 2 != 0)
                throw std::invalid_argument("segments: expected (S, 2) [start, count] pairs");
            gd.segments = segments->data();
            gd.segment_count = segments->size() / 2;
        }
        if (distances) {
            check(distances->size(), nv, "distances");
            gd.distances = distances->data();
        }
        uint32_t geom, inst;
        InstanceDesc d;
        {
            nb::gil_scoped_release release;
            geom = renderer_.add_geometry(gd);
            inst = renderer_.add_instance(geom);
        }
        geometry_of_[inst] = geom;
        d.color = color;
        d.metalness = metalness;
        d.roughness = roughness;
        d.specular = specular;
        d.line_width = line_width;
        d.dash = dash;
        d.gap = gap;
        d.morph_influence = morph_influence;
        d.flags |= kClipped; // cut by set_clip_planes, when any are set
        if (depth_offset)
            d.flags |= kDepthOffset;
        if (gd.colors)
            d.flags |= kVertexColors;
        if (double_sided)
            d.flags |= kDoubleSided;
        if (unlit)
            d.flags |= kUnlit;
        if (morph_relative)
            d.flags |= kMorphRelative;
        if (color[3] < 1.0f)
            d.flags |= kTransparent;
        if (model) {
            check(model->size(), 16, "model");
            std::memcpy(d.model.data(), model->data(), 64);
        }
        renderer_.set_instance(inst, d);
        descs_[inst] = d;
        return inst;
    }

    void set_morph_influence(uint32_t instance, float influence) {
        InstanceDesc &d = desc(instance);
        d.morph_influence = influence;
        renderer_.set_instance(instance, d);
    }
    void set_visible(uint32_t instance, bool visible) {
        InstanceDesc &d = desc(instance);
        d.flags = visible ? (d.flags & ~uint32_t(kHidden)) : (d.flags | kHidden);
        renderer_.set_instance(instance, d);
    }
    void update_colors(uint32_t instance, Array<uint8_t> colors) {
        renderer_.update_colors(geometry(instance), reinterpret_cast<const uint32_t *>(colors.data()),
                                colors.size() / 4);
    }
    void update_morph(uint32_t instance, Array<float> morph) {
        renderer_.update_morph(geometry(instance), morph.data(), morph.size() / 3);
    }
    void remove(uint32_t instance) {
        const uint32_t geom = geometry(instance);
        renderer_.remove_instance(instance);
        renderer_.remove_geometry(geom);
        geometry_of_.erase(instance);
        descs_.erase(instance);
    }
    void clear() {
        renderer_.clear();
        geometry_of_.clear();
        descs_.clear();
    }

    // Column-major 4x4s; `proj` in the OpenGL convention (clip z in [-1, 1]), as three.js builds it.
    void set_camera(Array<float> view, Array<float> proj, bool orthographic) {
        if (view.size() != 16 || proj.size() != 16)
            throw std::invalid_argument("view / proj: expected 16 floats each (column-major 4x4)");
        renderer_.set_camera(view.data(), proj.data(), orthographic);
        std::memcpy(view_, view.data(), sizeof(view_));
    }
    // The gizmo / legend drawn over every render (overlay_json.h for the spec); None removes it.
    void set_overlay(std::optional<std::string> spec_json) {
        overlay_.reset();
        if (spec_json)
            overlay_ = std::make_unique<Overlay>(overlay_from_json(*spec_json));
    }
    void set_background(float r, float g, float b, float a) {
        renderer_.set_background(r, g, b, a);
    }
    void set_lighting(float ambient, float headlight) {
        renderer_.set_lighting(ambient, headlight);
    }
    void set_culling(bool frustum, bool cone, bool occlusion) {
        renderer_.set_culling(frustum, cone, occlusion);
    }
    // Section planes as (nx, ny, nz, constant) rows, three.js' Plane; they cut instances added with clipped=True.
    void set_clip_planes(Array<float> planes) {
        if (planes.size() % 4 != 0)
            throw std::invalid_argument("planes: expected (n, 4) floats");
        renderer_.set_clip_planes(planes.data(), planes.size() / 4);
    }
    std::array<float, 6> bounds() const {
        return renderer_.bounds();
    }

    // (H, W, 4) uint8, top row first.
    nb::ndarray<nb::numpy, uint8_t, nb::shape<-1, -1, 4>> render() {
        std::vector<uint8_t> rgba;
        {
            nb::gil_scoped_release release;
            renderer_.render(target_->view(), target_->width(), target_->height());
            rgba = target_->read_rgba();
            if (overlay_)
                draw_overlay(rgba.data(), target_->width(), target_->height(), view_, *overlay_);
        }
        auto *data = new std::vector<uint8_t>(std::move(rgba));
        nb::capsule owner(data, [](void *p) noexcept { delete static_cast<std::vector<uint8_t> *>(p); });
        return nb::ndarray<nb::numpy, uint8_t, nb::shape<-1, -1, 4>>(data->data(),
                                                                     {target_->height(), target_->width(), 4}, owner);
    }

    nb::bytes render_png() {
        std::string png;
        {
            nb::gil_scoped_release release;
            renderer_.render(target_->view(), target_->width(), target_->height());
            auto rgba = target_->read_rgba();
            if (overlay_)
                draw_overlay(rgba.data(), target_->width(), target_->height(), view_, *overlay_);
            stbi_write_png_to_func(
                [](void *ctx, void *data, int n) {
                    static_cast<std::string *>(ctx)->append(static_cast<const char *>(data), size_t(n));
                },
                &png, int(target_->width()), int(target_->height()), 4, rgba.data(), int(target_->width() * 4));
        }
        return nb::bytes(png.data(), png.size());
    }

    nb::dict pick(uint32_t x, uint32_t y) {
        PickResult hit;
        {
            nb::gil_scoped_release release;
            bool done = false;
            renderer_.pick(x, y, target_->width(), target_->height(), [&](const PickResult &r) {
                hit = r;
                done = true;
            });
            while (!done)
                device_.process_events();
        }
        nb::dict d;
        d["hit"] = hit.hit;
        d["instance"] = hit.instance;
        d["segment"] = hit.segment;
        d["triangle"] = hit.triangle;
        d["world"] = std::array<float, 3>{hit.world[0], hit.world[1], hit.world[2]};
        return d;
    }

    // The last frame's stats, with the GPU culling counts read back.
    nb::dict stats() {
        RenderStats s;
        {
            nb::gil_scoped_release release;
            bool done = false;
            renderer_.read_cull_stats([&](const RenderStats &r) {
                s = r;
                done = true;
            });
            while (!done)
                device_.process_events();
        }
        nb::dict d;
        d["geometries"] = s.geometries;
        d["instances"] = s.instances;
        d["clusters_total"] = s.clusters_total;
        d["clusters_drawn"] = s.clusters_drawn;
        d["clusters_occluded"] = s.clusters_occluded;
        d["triangles_total"] = s.triangles_total;
        d["triangles_drawn"] = s.triangles_drawn;
        d["draw_calls"] = s.draw_calls;
        d["gpu_culled"] = s.gpu_culled;
        return d;
    }

private:
    uint32_t geometry(uint32_t instance) const {
        auto it = geometry_of_.find(instance);
        if (it == geometry_of_.end())
            throw std::out_of_range("unknown instance " + std::to_string(instance) + " (not added with add_mesh)");
        return it->second;
    }
    InstanceDesc &desc(uint32_t instance) {
        auto it = descs_.find(instance);
        if (it == descs_.end())
            throw std::out_of_range("unknown instance " + std::to_string(instance) + " (not added with add_mesh)");
        return it->second;
    }

    NativeDevice device_;
    Renderer renderer_;
    std::unique_ptr<OffscreenTarget> target_;
    std::map<uint32_t, uint32_t> geometry_of_;
    std::map<uint32_t, InstanceDesc> descs_;
    float view_[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    std::unique_ptr<Overlay> overlay_;
};

} // namespace
} // namespace adacpp::render

NB_MODULE(_render, m) {
    using adacpp::render::OffscreenRenderer;
    m.doc() = "adacpp WebGPU renderer, headless (wgpu-native)";
    nb::class_<OffscreenRenderer>(m, "OffscreenRenderer")
        .def(nb::init<uint32_t, uint32_t, const std::string &>(), "width"_a, "height"_a, "backend"_a = "auto")
        .def_prop_ro("adapter", &OffscreenRenderer::adapter)
        .def_prop_ro("size", &OffscreenRenderer::size)
        .def("resize", &OffscreenRenderer::resize, "width"_a, "height"_a)
        .def("load_glb", &OffscreenRenderer::load_glb, "path"_a, "y_up"_a = false, "line_width"_a = 1.0f,
             "member_width"_a = 1.0f, "point_size"_a = 1.0f, "line_color"_a = nb::none(), "double_sided"_a = false)
        .def("add_mesh", &OffscreenRenderer::add_mesh, "positions"_a, "indices"_a = nb::none(),
             "normals"_a = nb::none(), "colors"_a = nb::none(), "morph"_a = nb::none(), "segments"_a = nb::none(),
             "topology"_a = "triangles", "color"_a = std::array<float, 4>{1, 1, 1, 1}, "metalness"_a = 0.0f,
             "roughness"_a = 1.0f, "specular"_a = true, "double_sided"_a = false, "unlit"_a = false,
             "line_width"_a = 1.0f, "morph_influence"_a = 0.0f, "morph_relative"_a = true, "model"_a = nb::none(),
             "depth_offset"_a = false, "distances"_a = nb::none(), "dash"_a = 0.0f, "gap"_a = 0.0f)
        .def("set_morph_influence", &OffscreenRenderer::set_morph_influence, "instance"_a, "influence"_a)
        .def("set_visible", &OffscreenRenderer::set_visible, "instance"_a, "visible"_a)
        .def("update_colors", &OffscreenRenderer::update_colors, "instance"_a, "colors"_a)
        .def("update_morph", &OffscreenRenderer::update_morph, "instance"_a, "morph"_a)
        .def("remove", &OffscreenRenderer::remove, "instance"_a)
        .def("clear", &OffscreenRenderer::clear)
        .def("set_camera", &OffscreenRenderer::set_camera, "view"_a, "proj"_a, "orthographic"_a = false)
        .def("set_overlay", &OffscreenRenderer::set_overlay, "spec_json"_a.none(),
             "Gizmo / colour legend drawn over every render: a JSON spec (see adacpp src/render/overlay_json.h), "
             "or None.")
        .def("set_background", &OffscreenRenderer::set_background, "r"_a, "g"_a, "b"_a, "a"_a = 1.0f)
        .def("set_lighting", &OffscreenRenderer::set_lighting, "ambient"_a, "headlight"_a)
        .def("set_culling", &OffscreenRenderer::set_culling, "frustum"_a = true, "cone"_a = true, "occlusion"_a = true)
        .def("set_clip_planes", &OffscreenRenderer::set_clip_planes, "planes"_a)
        .def("bounds", &OffscreenRenderer::bounds)
        .def("render", &OffscreenRenderer::render)
        .def("render_png", &OffscreenRenderer::render_png)
        .def("pick", &OffscreenRenderer::pick, "x"_a, "y"_a)
        .def("stats", &OffscreenRenderer::stats);
}
