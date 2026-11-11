// The parts of the renderer that need a GPU to check: GLB animations (glb_scene.h GlbAnimations), feature edges
// per object, the GLB offset and glb_bounds_bytes, Draco decoding in the GLB reader, and textured quads (renderer.h QuadDesc). Every GLB is
// built here, in memory, and every image is drawn into an offscreen target and read back. Built and registered with
// ctest by src/render/CMakeLists.txt (pixi run -e render test-render); skipped where there is no adapter to draw with.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "glb_scene.h"
#include "native_device.h"
#include "renderer.h"

#ifdef ADACPP_RENDER_DRACO
#include <draco/compression/encode.h>
#include <draco/mesh/triangle_soup_mesh_builder.h>
#endif

using namespace adacpp::render;

namespace {

int failures = 0, checks = 0;

void check(bool ok, const char *what, int line) {
    ++checks;
    if (!ok) {
        ++failures;
        std::fprintf(stderr, "FAILED line %d: %s\n", line, what);
    }
}
#define CHECK(cond) check((cond), #cond, __LINE__)

using Mat4 = std::array<float, 16>;
using Bytes = std::vector<unsigned char>;

bool close(float a, float b, float tol = 1e-4f) {
    return std::fabs(a - b) <= tol;
}

// A GLB put together by hand: binary data goes in through view() / floats() / indices(), the rest of the
// document (scenes, nodes, meshes, ...) as JSON text.
struct GlbBuilder {
    Bytes bin;
    std::string views, accessors;
    int view_count = 0, accessor_count = 0;

    int view(const void *data, size_t bytes) {
        while (bin.size() % 4)
            bin.push_back(0);
        const size_t at = bin.size();
        const auto *p = static_cast<const unsigned char *>(data);
        bin.insert(bin.end(), p, p + bytes);
        views += std::string(view_count ? "," : "") + "{\"buffer\":0,\"byteOffset\":" + std::to_string(at) +
                 ",\"byteLength\":" + std::to_string(bytes) + "}";
        return view_count++;
    }
    // `view` -1: an accessor that only describes data held elsewhere (a Draco-compressed primitive's).
    // `extra`: more members of the accessor, each with its leading comma.
    int accessor(int view, int component_type, size_t count, const char *type, const std::string &extra = "") {
        accessors += std::string(accessor_count ? "," : "") + "{" +
                     (view >= 0 ? "\"bufferView\":" + std::to_string(view) + "," : std::string()) +
                     "\"componentType\":" + std::to_string(component_type) + ",\"count\":" + std::to_string(count) +
                     ",\"type\":\"" + type + "\"" + extra + "}";
        return accessor_count++;
    }
    int floats(const std::vector<float> &v, const char *type, int comps) {
        return accessor(view(v.data(), v.size() * 4), 5126, v.size() / comps, type);
    }
    // VEC3 positions with the min and max glTF asks of them.
    int bounded_positions(const std::vector<float> &v) {
        float lo[3] = {3.4e38f, 3.4e38f, 3.4e38f}, hi[3] = {-3.4e38f, -3.4e38f, -3.4e38f};
        for (size_t i = 0; i < v.size(); ++i) {
            lo[i % 3] = std::min(lo[i % 3], v[i]);
            hi[i % 3] = std::max(hi[i % 3], v[i]);
        }
        char extra[256];
        std::snprintf(extra, sizeof extra, ",\"min\":[%.9g,%.9g,%.9g],\"max\":[%.9g,%.9g,%.9g]", lo[0], lo[1], lo[2],
                      hi[0], hi[1], hi[2]);
        return accessor(view(v.data(), v.size() * 4), 5126, v.size() / 3, "VEC3", extra);
    }
    int indices(const std::vector<uint32_t> &v) {
        return accessor(view(v.data(), v.size() * 4), 5125, v.size(), "SCALAR");
    }
    Bytes finish(const std::string &rest) {
        while (bin.size() % 4)
            bin.push_back(0);
        std::string json =
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":" + std::to_string(bin.size()) +
            "}],\"bufferViews\":[" + views + "],\"accessors\":[" + accessors + "]," + rest + "}";
        while (json.size() % 4)
            json += ' ';
        Bytes out;
        auto put = [&](uint32_t v) {
            const auto *p = reinterpret_cast<const unsigned char *>(&v);
            out.insert(out.end(), p, p + 4);
        };
        put(0x46546C67u);
        put(2);
        put(uint32_t(28 + json.size() + bin.size()));
        put(uint32_t(json.size()));
        put(0x4E4F534Au);
        out.insert(out.end(), json.begin(), json.end());
        put(uint32_t(bin.size()));
        put(0x004E4942u);
        out.insert(out.end(), bin.begin(), bin.end());
        return out;
    }
};

// A renderer drawing into a size x size image, looking straight down the z axis at the rectangle
// x in [x0, x1], y in [y0, y1] (x right, y up): an orthographic camera at z = 10.
struct Stage {
    static constexpr uint32_t kSize = 256;
    NativeDevice &dev;
    Renderer r;
    OffscreenTarget target;
    float x0, x1, y0, y1;
    std::vector<uint8_t> image;

    Stage(NativeDevice &d, float x0_, float x1_, float y0_, float y1_)
        : dev(d), r(d.device(), OffscreenTarget::kFormat), target(d, kSize, kSize), x0(x0_), x1(x1_), y0(y0_), y1(y1_) {
        const float n = 0.1f, f = 100.0f;
        const Mat4 view{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -10, 1};
        Mat4 proj{};
        proj[0] = 2 / (x1 - x0);
        proj[5] = 2 / (y1 - y0);
        proj[10] = -2 / (f - n);
        proj[12] = -(x1 + x0) / (x1 - x0);
        proj[13] = -(y1 + y0) / (y1 - y0);
        proj[14] = -(f + n) / (f - n);
        proj[15] = 1;
        r.set_camera(view.data(), proj.data(), true);
        r.set_background(10 / 255.0f, 20 / 255.0f, 30 / 255.0f);
    }
    void draw() {
        r.render(target.view(), kSize, kSize);
        image = target.read_rgba();
    }
    std::array<uint32_t, 2> pixel_of(float x, float y) const {
        return {uint32_t((x - x0) / (x1 - x0) * kSize), uint32_t((y1 - y) / (y1 - y0) * kSize)};
    }
    // The pixel of the last draw() showing world (x, y).
    std::array<int, 4> at(float x, float y) const {
        const auto p = pixel_of(x, y);
        const uint8_t *c = &image[(size_t(p[1]) * kSize + p[0]) * 4];
        return {c[0], c[1], c[2], c[3]};
    }
    bool shows(float x, float y, int red, int green, int blue, int tol = 2) const {
        const auto c = at(x, y);
        const bool ok = std::abs(c[0] - red) <= tol && std::abs(c[1] - green) <= tol && std::abs(c[2] - blue) <= tol;
        if (!ok)
            std::fprintf(stderr, "  pixel at (%g, %g) is %d %d %d, expected %d %d %d\n", x, y, c[0], c[1], c[2], red,
                         green, blue);
        return ok;
    }
    bool background(float x, float y) const {
        return shows(x, y, 10, 20, 30, 0);
    }
    PickResult pick(float x, float y) {
        const auto p = pixel_of(x, y);
        PickResult hit;
        bool done = false;
        r.pick(p[0], p[1], kSize, kSize, [&](const PickResult &res) {
            hit = res;
            done = true;
        });
        while (!done)
            dev.process_events();
        return hit;
    }
};

const float kS = std::sqrt(0.5f);

// Three squares, 0.8 wide, in the z = 0 plane: "base" at the origin, "arm" at x = 2 and, under it, "hand" a
// unit further out. Animations:
//   0 "move"    arm slides to y = 4 over 2 s (LINEAR); hand turns a quarter about z over 1 s
//   1 "step"    arm jumps to y = 2 at 1 s (STEP)
//   2 "spline"  arm eases to y = 2 over 1 s (CUBICSPLINE, flat tangents)
// `root` ("x,y,z"): all of it hangs under a node with that translation instead.
Bytes animated_glb(const std::string &root = "") {
    GlbBuilder b;
    const float h = 0.4f;
    const int positions = b.floats({-h, -h, 0, h, -h, 0, h, h, 0, -h, h, 0}, "VEC3", 3);
    const int indices = b.indices({0, 1, 2, 0, 2, 3});
    const int t02 = b.floats({0, 2}, "SCALAR", 1);
    const int slide = b.floats({2, 0, 0, 2, 4, 0}, "VEC3", 3);
    const int t01 = b.floats({0, 1}, "SCALAR", 1);
    const int turn = b.floats({0, 0, 0, 1, 0, 0, kS, kS}, "VEC4", 4);
    const int jump = b.floats({2, 0, 0, 2, 2, 0}, "VEC3", 3);
    const int ease = b.floats({0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0}, "VEC3", 3);
    CHECK(positions == 0 && indices == 1 && t02 == 2 && slide == 3 && t01 == 4 && turn == 5 && jump == 6 && ease == 7);
    const std::string scene =
        root.empty() ? R"("scene":0,"scenes":[{"nodes":[0,1]}],)" : R"("scene":0,"scenes":[{"nodes":[3]}],)";
    const std::string root_node =
        root.empty() ? "" : R"(,{"name":"root","translation":[)" + root + R"(],"children":[0,1]})";
    return b.finish(scene + R"("nodes":[{"name":"base","mesh":0},
                 {"name":"arm","mesh":0,"translation":[2,0,0],"children":[2]},
                 {"name":"hand","mesh":0,"translation":[1,0,0]})" +
                    root_node + R"(],
        "meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1,"material":0}]}],
        "materials":[{"pbrMetallicRoughness":{"baseColorFactor":[1,0,0,1]},"doubleSided":true,
                      "extensions":{"KHR_materials_unlit":{}}}],
        "extensionsUsed":["KHR_materials_unlit"],
        "animations":[
          {"name":"move",
           "channels":[{"sampler":0,"target":{"node":1,"path":"translation"}},
                       {"sampler":1,"target":{"node":2,"path":"rotation"}}],
           "samplers":[{"input":2,"output":3,"interpolation":"LINEAR"},{"input":4,"output":5}]},
          {"name":"step",
           "channels":[{"sampler":0,"target":{"node":1,"path":"translation"}}],
           "samplers":[{"input":4,"output":6,"interpolation":"STEP"}]},
          {"name":"spline",
           "channels":[{"sampler":0,"target":{"node":1,"path":"translation"}}],
           "samplers":[{"input":4,"output":7,"interpolation":"CUBICSPLINE"}]}])");
}

const GlbGeometry *geometry_of_node(const GlbLoadResult &res, int node) {
    for (const GlbGeometry &g : res.loaded)
        if (g.node == node)
            return &g;
    return nullptr;
}

int object_by_id(const GlbLoadResult &res, const std::string &id) {
    for (size_t i = 0; i < res.objects.size(); ++i)
        if (res.objects[i].id == id)
            return int(i);
    return -1;
}

void test_animation(NativeDevice &dev) {
    const Bytes glb = animated_glb();

    // Without the option the reader does what it always did: one merged batch, nothing to play.
    {
        Stage s(dev, -1, 7, -2, 6);
        const GlbLoadResult res = load_glb_bytes(s.r, glb);
        CHECK(res.geometries == 1 && res.instances == 1 && res.triangles == 6);
        CHECK(res.animations.count() == 0 && res.animations.nodes().empty());
        CHECK(res.loaded.size() == 1 && res.loaded[0].node == -1);
        CHECK(res.loaded[0].segment_objects.size() == 3);
        CHECK(close(res.bounds[0], -0.4f) && close(res.bounds[3], 3.4f) && close(res.bounds[4], 0.4f));
        CHECK(s.r.instance_desc(res.loaded[0].instance).model ==
              Mat4({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
        s.draw();
        CHECK(s.shows(0, 0, 255, 0, 0) && s.shows(2, 0, 255, 0, 0) && s.shows(3, 0, 255, 0, 0));
        CHECK(s.background(1, 0) && s.background(3, 4));
    }

    Stage s(dev, -1, 7, -2, 6);
    GlbOptions options;
    options.animations = true;
    const GlbLoadResult res = load_glb_bytes(s.r, glb, options);
    const GlbAnimations &anim = res.animations;
    CHECK(res.geometries == 3 && res.instances == 3 && res.triangles == 6);
    CHECK(anim.count() == 3);
    CHECK(anim.name(0) == "move" && anim.name(1) == "step" && anim.name(2) == "spline");
    CHECK(close(anim.duration(0), 2) && close(anim.duration(1), 1) && close(anim.duration(2), 1));
    CHECK(anim.nodes() == std::vector<int32_t>({1, 2}));
    const GlbGeometry *base = geometry_of_node(res, -1), *arm = geometry_of_node(res, 1),
                      *hand = geometry_of_node(res, 2);
    CHECK(base && arm && hand);
    if (!base || !arm || !hand)
        return;
    auto model = [&](const GlbGeometry *g) { return s.r.instance_desc(g->instance).model; };
    auto origin = [&](const GlbGeometry *g, float x, float y) {
        const Mat4 m = model(g);
        const bool ok = close(m[12], x) && close(m[13], y) && close(m[14], 0);
        if (!ok)
            std::fprintf(stderr, "  node %d is at (%g, %g, %g), expected (%g, %g, 0)\n", g->node, m[12], m[13], m[14],
                         x, y);
        return ok;
    };

    // Loaded in the rest pose, and drawn as the merged model is.
    CHECK(origin(base, 0, 0) && origin(arm, 2, 0) && origin(hand, 3, 0));
    s.draw();
    CHECK(s.shows(0, 0, 255, 0, 0) && s.shows(2, 0, 255, 0, 0) && s.shows(3, 0, 255, 0, 0));
    CHECK(s.background(1, 0) && s.background(3, 4));

    // LINEAR translation; the child follows its parent and turns by slerp.
    anim.apply(s.r, 0, 0.5f);
    CHECK(origin(base, 0, 0) && origin(arm, 2, 1) && origin(hand, 3, 1));
    CHECK(close(model(hand)[0], kS) && close(model(hand)[1], kS) && close(model(hand)[4], -kS));
    CHECK(close(model(arm)[0], 1) && close(model(arm)[1], 0));
    anim.apply(s.r, 0, 1.0f);
    CHECK(origin(arm, 2, 2) && origin(hand, 3, 2));
    CHECK(close(model(hand)[0], 0) && close(model(hand)[1], 1) && close(model(hand)[4], -1) &&
          close(model(hand)[5], 0));
    const auto hand_matrix = anim.node_matrix(1);
    CHECK(hand_matrix == model(hand) && anim.node_matrix(0) == model(arm));
    // Past the end, and before the start, the nearest keyframe holds.
    anim.apply(s.r, 0, 9.0f);
    CHECK(origin(arm, 2, 4) && origin(hand, 3, 4) && close(model(hand)[1], 1));
    anim.apply(s.r, 0, -1.0f);
    CHECK(origin(arm, 2, 0) && origin(hand, 3, 0) && close(model(hand)[0], 1));

    // The instances moved are the ones drawn and picked, and a pick still names the object.
    anim.apply(s.r, 0, 2.0f);
    s.draw();
    CHECK(s.shows(0, 0, 255, 0, 0) && s.shows(2, 4, 255, 0, 0) && s.shows(3, 4, 255, 0, 0));
    CHECK(s.background(2, 0) && s.background(3, 0));
    CHECK(res.node_objects && res.objects.size() == 3);
    const int arm_object = object_by_id(res, "1"), hand_object = object_by_id(res, "2");
    CHECK(arm_object >= 0 && hand_object >= 0);
    if (arm_object < 0 || hand_object < 0)
        return;
    CHECK(res.objects[hand_object].parent == arm_object && res.objects[hand_object].name == "hand");
    CHECK(res.objects[hand_object].segments.size() == 1 && res.objects[hand_object].segments[0][0] == hand->instance);
    PickResult hit = s.pick(3, 4);
    CHECK(hit.hit && hit.instance == hand->instance);
    CHECK(hit.hit && hit.segment < hand->segment_objects.size() && hand->segment_objects[hit.segment] == hand_object);
    CHECK(close(hit.world[0], 3, 0.05f) && close(hit.world[1], 4, 0.05f) && close(hit.world[2], 0, 0.05f));
    hit = s.pick(2, 4);
    CHECK(hit.hit && hit.instance == arm->instance && arm->segment_objects[hit.segment] == arm_object);
    CHECK(!s.pick(3, 0).hit);

    // STEP holds the earlier keyframe; a node the animation does not name is at rest.
    anim.apply(s.r, 1, 0.99f);
    CHECK(origin(arm, 2, 0) && origin(hand, 3, 0) && close(model(hand)[0], 1));
    anim.apply(s.r, 1, 1.0f);
    CHECK(origin(arm, 2, 2) && origin(hand, 3, 2) && close(model(hand)[0], 1));
    // CUBICSPLINE with flat tangents: 3u^2 - 2u^3 of the way.
    anim.apply(s.r, 2, 0.25f);
    CHECK(origin(arm, 2, 2 * 0.15625f) && origin(hand, 3, 2 * 0.15625f));
    anim.apply(s.r, 2, 0.5f);
    CHECK(origin(arm, 2, 1));
    // -1: back to rest.
    anim.apply(s.r, -1, 0.0f);
    CHECK(origin(base, 0, 0) && origin(arm, 2, 0) && origin(hand, 3, 0) && close(model(hand)[0], 1));
    bool threw = false;
    try {
        anim.apply(s.r, 3, 0.0f);
    } catch (const std::out_of_range &) {
        threw = true;
    }
    CHECK(threw);

    // Bounds cover the motion: the arm reaches y = 4, the hand beside it, turning.
    CHECK(close(res.bounds[0], -0.4f) && close(res.bounds[1], -0.4f, 0.2f));
    CHECK(res.bounds[3] >= 3.4f && res.bounds[3] < 3.8f && res.bounds[4] >= 4.4f && res.bounds[4] < 4.8f);
    const auto &hb = res.objects[hand_object].bounds, &ab = res.objects[arm_object].bounds;
    CHECK(hb[0] > 2.3f && hb[1] <= -0.4f && hb[3] >= 3.4f && hb[4] >= 4.4f && hb[4] < 4.8f);
    CHECK(ab[0] <= 1.6f && ab[3] >= 3.4f && ab[4] >= 4.4f); // with the hand below it
    const auto &bb = res.objects[object_by_id(res, "0")].bounds;
    CHECK(close(bb[0], -0.4f) && close(bb[3], 0.4f) && close(bb[4], 0.4f));

    // Feature edges of an animated node ride with it.
    Stage e(dev, -1, 7, -2, 6);
    options.feature_edge_angle_deg = 30.0f;
    const GlbLoadResult with_edges = load_glb_bytes(e.r, glb, options);
    int edge_overlays = 0;
    for (const GlbGeometry &g : with_edges.loaded)
        if (g.feature_edges && g.node == 2) {
            ++edge_overlays;
            with_edges.animations.apply(e.r, 0, 2.0f);
            const Mat4 m = e.r.instance_desc(g.instance).model;
            CHECK(close(m[12], 3) && close(m[13], 4));
        }
    CHECK(edge_overlays == 1);
}

// Two unit squares side by side in one primitive, x in [-1, 0] and [0, 1], made objects "a" and "b" by
// draw ranges: each object's feature edges are its own outline, a segment of the overlay naming it.
void test_feature_edge_objects(NativeDevice &dev) {
    GlbBuilder b;
    const int pos = b.floats({-1, -1, 0, 0, -1, 0, 0, 1, 0, -1, 1, 0, 0, -1, 0, 1, -1, 0, 1, 1, 0, 0, 1, 0}, "VEC3", 3);
    const int idx = b.indices({0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7});
    const Bytes glb = b.finish(
        R"("scene":0,"scenes":[{"nodes":[0],"extras":{"id_hierarchy":{"a":["a","*"],"b":["b","*"]},
            "draw_ranges_n":{"a":[0,6],"b":[6,6]}}}],"nodes":[{"name":"n","mesh":0}],
        "meshes":[{"primitives":[{"attributes":{"POSITION":)" +
        std::to_string(pos) + R"(},"indices":)" + std::to_string(idx) + "}]}]");
    Stage s(dev, -2, 2, -2, 2);
    GlbOptions options;
    options.feature_edge_angle_deg = 30.0f;
    options.line_color = {0, 0, 0, 1};
    options.line_width = 6.0f;
    const GlbLoadResult res = load_glb_bytes(s.r, glb, options);
    const int a = object_by_id(res, "a"), b_ = object_by_id(res, "b");
    const GlbGeometry *faces = nullptr, *edges = nullptr;
    for (const GlbGeometry &g : res.loaded)
        (g.feature_edges ? edges : faces) = &g;
    CHECK(faces && edges);
    if (!faces || !edges)
        return;
    // The shared edge at x = 0 is on both outlines.
    CHECK(res.feature_edges == 8);
    CHECK((edges->segment_objects == std::vector<int32_t>{a, b_}));
    CHECK(res.objects[a].segments.size() == 1 && res.objects[a].segments[0][0] == faces->instance);

    InstanceDesc hidden = s.r.instance_desc(faces->instance);
    hidden.flags |= kHidden;
    s.r.set_instance(faces->instance, hidden);
    s.draw();
    CHECK(s.shows(-1, -0.5f, 0, 0, 0) && s.shows(1, -0.5f, 0, 0, 0));
    // Hiding "a" hides its outline; b's stays.
    const uint32_t states[4] = {0, kSegmentHidden, 0, 0};
    s.r.set_segment_states(edges->instance, states, 2);
    s.draw();
    CHECK(s.background(-1, -0.5f) && s.shows(1, -0.5f, 0, 0, 0) && s.shows(0, -0.5f, 0, 0, 0));
}

// A translation far from the origin, as text for a GLB and as the double it parses to.
const char *kFarText = "1234567.891,-2345678.123,345678.5";
const double kFar[3] = {1234567.891, -2345678.123, 345678.5};
const std::vector<float> kFarTriangle = {0.3127f, 0.6473f, 0.1189f, 1.4381f, 0.2219f,
                                         0.0713f, 0.5531f, 1.3097f, 0.9257f};

// One triangle near the origin of a node at kFar; `bounded`: its positions carry min and max.
Bytes far_glb(bool bounded) {
    GlbBuilder b;
    const int pos = bounded ? b.bounded_positions(kFarTriangle) : b.floats(kFarTriangle, "VEC3", 3);
    const int idx = b.indices({0, 1, 2});
    return b.finish(
        std::string(R"("scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"name":"far","mesh":0,"translation":[)") +
        kFarText + R"(]}],"meshes":[{"primitives":[{"attributes":{"POSITION":)" + std::to_string(pos) +
        R"(},"indices":)" + std::to_string(idx) + "}]}]");
}

// glTF y-up -> z-up, as GlbOptions::y_up_to_z_up turns the model.
std::array<double, 3> z_up(double x, double y, double z) {
    return {x, -z, y};
}

void test_offset(NativeDevice &dev) {
    // Moved to the origin before anything is a float: the triangle keeps its own coordinates. Loaded where
    // it is and moved back afterwards, in float, it is off by up to half the float spacing out there.
    {
        Stage s(dev, -1, 2, -1, 2);
        GlbOptions o;
        o.keep_positions = true;
        o.offset = {-kFar[0], -kFar[1], -kFar[2]};
        const GlbLoadResult res = load_glb_bytes(s.r, far_glb(false), o);
        Stage t(dev, -1, 2, -1, 2);
        GlbOptions plain;
        plain.keep_positions = true;
        const GlbLoadResult far = load_glb_bytes(t.r, far_glb(false), plain);
        CHECK(res.loaded.size() == 1 && far.loaded.size() == 1);
        if (res.loaded.size() != 1 || far.loaded.size() != 1)
            return;
        const std::vector<float> &p = res.loaded[0].positions, &q = far.loaded[0].positions;
        CHECK(p.size() == 9 && q.size() == 9);
        double worst = 0, worst_float = 0;
        for (size_t i = 0; i < p.size() && i < q.size(); ++i) {
            worst = std::max(worst, std::fabs(double(p[i]) - kFarTriangle[i]));
            worst_float = std::max(worst_float, std::fabs(double(q[i] - float(kFar[i % 3])) - kFarTriangle[i]));
        }
        std::printf("offset: worst error %g with the offset, %g moved afterwards in float\n", worst, worst_float);
        CHECK(worst < 1e-6);
        CHECK(worst_float > 1e-2);
        // Bounds are in the offset frame.
        CHECK(close(res.bounds[0], 0.3127f, 1e-6f) && close(res.bounds[1], 0.2219f, 1e-6f) &&
              close(res.bounds[2], 0.0713f, 1e-6f) && close(res.bounds[3], 1.4381f, 1e-6f) &&
              close(res.bounds[4], 1.3097f, 1e-6f) && close(res.bounds[5], 0.9257f, 1e-6f));
        CHECK(res.objects.size() == 1 && res.objects[0].bounds == res.bounds);
        s.draw();
        const auto c = s.at(0.768f, 0.726f); // the triangle's centroid
        CHECK(c[0] != 10 || c[1] != 20 || c[2] != 30);
    }

    // y-up: the offset is in the turned (z-up) frame.
    {
        Stage s(dev, -1, 2, -1, 2);
        GlbOptions o;
        o.keep_positions = true;
        o.y_up_to_z_up = true;
        const auto t = z_up(kFar[0], kFar[1], kFar[2]);
        o.offset = {-t[0], -t[1], -t[2]};
        const GlbLoadResult res = load_glb_bytes(s.r, far_glb(true), o);
        CHECK(res.loaded.size() == 1 && res.loaded[0].positions.size() == 9);
        double worst = 0;
        if (res.loaded.size() == 1)
            for (size_t v = 0; v < 3 && v * 3 + 2 < res.loaded[0].positions.size(); ++v) {
                const auto e = z_up(kFarTriangle[v * 3], kFarTriangle[v * 3 + 1], kFarTriangle[v * 3 + 2]);
                for (int k = 0; k < 3; ++k)
                    worst = std::max(worst, std::fabs(double(res.loaded[0].positions[v * 3 + k]) - e[k]));
            }
        CHECK(worst < 1e-6);
    }

    // glb_bounds_bytes: exact in double, from the accessors' min / max or from the data without them.
    for (const bool bounded : {false, true})
        for (const bool y_up : {false, true}) {
            const auto b = glb_bounds_bytes(far_glb(bounded), y_up);
            std::array<double, 6> e{3.4e38, 3.4e38, 3.4e38, -3.4e38, -3.4e38, -3.4e38};
            for (size_t v = 0; v < 3; ++v) {
                const double x = double(kFarTriangle[v * 3]) + kFar[0], y = double(kFarTriangle[v * 3 + 1]) + kFar[1],
                             z = double(kFarTriangle[v * 3 + 2]) + kFar[2];
                const auto w = y_up ? z_up(x, y, z) : std::array<double, 3>{x, y, z};
                for (int k = 0; k < 3; ++k) {
                    e[k] = std::min(e[k], w[k]);
                    e[3 + k] = std::max(e[3 + k], w[k]);
                }
            }
            double worst = 0;
            for (int k = 0; k < 6; ++k)
                worst = std::max(worst, std::fabs(b[k] - e[k]));
            if (worst >= 1e-6)
                std::fprintf(stderr, "  glb_bounds (bounded %d, y_up %d) off by %g\n", bounded, y_up, worst);
            CHECK(worst < 1e-6);
            // ... and what load_glb reports, with the offset as without it.
            for (const bool offset : {false, true}) {
                Stage s(dev, -1, 1, -1, 1);
                GlbOptions o;
                o.y_up_to_z_up = y_up;
                if (offset)
                    o.offset = {-b[0], -b[1], -b[2]};
                const GlbLoadResult res = load_glb_bytes(s.r, far_glb(bounded), o);
                for (int k = 0; k < 6; ++k)
                    CHECK(close(res.bounds[k], float(b[k] + o.offset[k % 3]), offset ? 1e-6f : 0.25f));
            }
        }
    {
        const Bytes glb = animated_glb();
        Stage s(dev, -1, 1, -1, 1);
        const GlbLoadResult res = load_glb_bytes(s.r, glb);
        const auto b = glb_bounds_bytes(glb, false);
        for (int k = 0; k < 6; ++k)
            CHECK(close(res.bounds[k], float(b[k]), 1e-6f));
        // At rest, whatever the animations do.
        CHECK(close(float(b[3]), 3.4f, 1e-6f) && close(float(b[4]), 0.4f, 1e-6f));
    }
    {
        // Nothing drawable (a strip is not read): inverted.
        GlbBuilder g;
        const int pos = g.floats(kFarTriangle, "VEC3", 3);
        const auto b = glb_bounds_bytes(g.finish(R"("scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],
            "meshes":[{"primitives":[{"attributes":{"POSITION":)" +
                                                 std::to_string(pos) + R"(},"mode":5}]}])"),
                                        false);
        CHECK(b[0] > b[3] && b[1] > b[4] && b[2] > b[5]);
    }

    // The animated path: the offset is in every node's world matrix before it is a float.
    Stage s(dev, -1, 7, -2, 6);
    GlbOptions options;
    options.animations = true;
    options.offset = {-kFar[0], -kFar[1], -kFar[2]};
    const GlbLoadResult res = load_glb_bytes(s.r, animated_glb(kFarText), options);
    const GlbGeometry *base = geometry_of_node(res, -1), *arm = geometry_of_node(res, 1),
                      *hand = geometry_of_node(res, 2);
    CHECK(base && arm && hand && res.animations.count() == 3);
    if (!base || !arm || !hand || res.animations.count() != 3)
        return;
    auto at = [&](const GlbGeometry *g, float x, float y) {
        const Mat4 m = s.r.instance_desc(g->instance).model;
        const bool ok = close(m[12], x, 1e-6f) && close(m[13], y, 1e-6f) && close(m[14], 0, 1e-6f);
        if (!ok)
            std::fprintf(stderr, "  node %d is at (%.9g, %.9g, %.9g), expected (%g, %g, 0)\n", g->node, m[12], m[13],
                         m[14], x, y);
        return ok;
    };
    CHECK(at(arm, 2, 0) && at(hand, 3, 0));
    CHECK(close(res.bounds[0], -0.4f, 1e-6f) && res.bounds[4] >= 4.4f && res.bounds[4] < 4.8f);
    res.animations.apply(s.r, 0, 1.0f);
    CHECK(at(arm, 2, 2) && at(hand, 3, 2));
    CHECK(res.animations.node_matrix(0) == s.r.instance_desc(arm->instance).model);
    res.animations.apply(s.r, 0, 2.0f);
    s.draw();
    CHECK(s.shows(0, 0, 255, 0, 0) && s.shows(2, 4, 255, 0, 0) && s.shows(3, 4, 255, 0, 0));
    CHECK(s.background(2, 0) && s.background(3, 0));
}

#ifdef ADACPP_RENDER_DRACO
// A bumpy n x n grid over [-2, 2]^2, as an indexed mesh.
void grid(int n, std::vector<float> &positions, std::vector<uint32_t> &indices) {
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const float x = -2 + 4.0f * i / n, y = -2 + 4.0f * j / n;
            positions.insert(positions.end(), {x, y, 0.5f * std::sin(x * 2) * std::cos(y * 2)});
        }
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const uint32_t a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
            indices.insert(indices.end(), {a, b, d, a, d, c});
        }
}

const char *kGridScene = R"("scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"name":"grid","mesh":0}],
    "materials":[{"pbrMetallicRoughness":{"baseColorFactor":[0.2,0.6,0.9,1],"metallicFactor":0},"doubleSided":true}],)";

void test_draco(NativeDevice &dev) {
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    grid(16, positions, indices);

    GlbBuilder plain;
    const int p = plain.floats(positions, "VEC3", 3), i = plain.indices(indices);
    const Bytes plain_glb = plain.finish(
        std::string(kGridScene) + "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":" + std::to_string(p) +
        "},\"indices\":" + std::to_string(i) + ",\"material\":0}]}]");

    draco::TriangleSoupMeshBuilder builder;
    builder.Start(int(indices.size() / 3));
    const int att = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    for (size_t f = 0; f < indices.size() / 3; ++f)
        builder.SetAttributeValuesForFace(att, draco::FaceIndex(uint32_t(f)), &positions[indices[f * 3] * 3],
                                          &positions[indices[f * 3 + 1] * 3], &positions[indices[f * 3 + 2] * 3]);
    const std::unique_ptr<draco::Mesh> mesh = builder.Finalize();
    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
    draco::EncoderBuffer encoded;
    CHECK(encoder.EncodeMeshToBuffer(*mesh, &encoded).ok());
    CHECK(encoded.size() > 0 && encoded.size() < positions.size() * 4);

    auto draco_glb = [&](const char *data, size_t size, int unique_id) {
        GlbBuilder b;
        const int view = b.view(data, size);
        const int pos = b.accessor(-1, 5126, mesh->num_points(), "VEC3");
        const int idx = b.accessor(-1, 5125, mesh->num_faces() * 3, "SCALAR");
        return b.finish(
            std::string(kGridScene) +
            "\"extensionsUsed\":[\"KHR_draco_mesh_compression\"],"
            "\"extensionsRequired\":[\"KHR_draco_mesh_compression\"],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":" +
            std::to_string(pos) + "},\"indices\":" + std::to_string(idx) +
            ",\"material\":0,\"extensions\":{\"KHR_draco_mesh_compression\":{\"bufferView\":" + std::to_string(view) +
            ",\"attributes\":{\"POSITION\":" + std::to_string(unique_id) + "}}}}]}]");
    };
    const int unique_id = int(mesh->attribute(att)->unique_id());

    Stage a(dev, -2.5f, 2.5f, -2.5f, 2.5f), b(dev, -2.5f, 2.5f, -2.5f, 2.5f);
    const GlbLoadResult plain_res = load_glb_bytes(a.r, plain_glb);
    const GlbLoadResult draco_res = load_glb_bytes(b.r, draco_glb(encoded.data(), encoded.size(), unique_id));
    CHECK(plain_res.triangles == 512 && draco_res.triangles == 512);
    CHECK(draco_res.vertices == 17 * 17 && draco_res.geometries == 1);
    for (int k = 0; k < 6; ++k)
        CHECK(close(plain_res.bounds[k], draco_res.bounds[k], 1e-3f));
    // Without the accessors' min / max, glb_bounds_bytes reads the Draco data.
    const auto draco_bounds = glb_bounds_bytes(draco_glb(encoded.data(), encoded.size(), unique_id), false);
    const auto plain_bounds = glb_bounds_bytes(plain_glb, false);
    for (int k = 0; k < 6; ++k)
        CHECK(close(float(draco_bounds[k]), draco_res.bounds[k], 1e-6f) &&
              close(float(plain_bounds[k]), plain_res.bounds[k], 1e-6f));
    CHECK(draco_res.objects.size() == 1 && draco_res.objects[0].triangles == 512);
    // The same picture, but for the 14-bit positions.
    a.draw();
    b.draw();
    size_t differing = 0, drawn = 0;
    for (size_t px = 0; px < a.image.size(); px += 4) {
        int d = 0;
        for (int c = 0; c < 3; ++c)
            d = std::max(d, std::abs(int(a.image[px + c]) - int(b.image[px + c])));
        differing += d > 8;
        drawn += a.image[px] != 10 || a.image[px + 1] != 20 || a.image[px + 2] != 30;
    }
    CHECK(drawn > a.image.size() / 4 / 2);
    CHECK(differing < a.image.size() / 4 / 200);
    const PickResult hit = b.pick(0.3f, -0.7f);
    CHECK(hit.hit && close(hit.world[0], 0.3f, 0.05f) && close(hit.world[1], -0.7f, 0.05f));

    // Data that is not Draco, and an attribute the data does not have, are errors, not crashes.
    for (int broken = 0; broken < 2; ++broken) {
        Stage c(dev, -1, 1, -1, 1);
        bool threw = false;
        try {
            if (broken == 0) {
                const std::string junk(64, 'x');
                load_glb_bytes(c.r, draco_glb(junk.data(), junk.size(), unique_id));
            } else {
                load_glb_bytes(c.r, draco_glb(encoded.data(), encoded.size(), unique_id + 7));
            }
        } catch (const std::runtime_error &) {
            threw = true;
        }
        CHECK(threw);
    }
}
#endif

Mat4 scaled(float sx, float sy, float tx = 0, float ty = 0, float tz = 0) {
    return {sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, 1, 0, tx, ty, tz, 1};
}

void test_quads(NativeDevice &dev) {
    Stage s(dev, -3, 3, -3, 3);
    // Top row first: red, green; then blue, white.
    const uint8_t rgbw[16] = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255};
    const uint32_t tex = s.r.add_texture(rgbw, 2, 2, false);
    const uint32_t quad = s.r.add_quad();
    QuadDesc d;
    d.texture = tex;
    d.model = scaled(4, 4); // x, y in [-2, 2]
    d.flags = kNearest;
    s.r.set_quad(quad, d);

    // The texture's top row lies along +y, its left column along -x.
    s.draw();
    CHECK(s.shows(-1, 1, 255, 0, 0) && s.shows(1, 1, 0, 255, 0) && s.shows(-1, -1, 0, 0, 255) &&
          s.shows(1, -1, 255, 255, 255));
    CHECK(s.shows(-0.1f, 0.1f, 255, 0, 0) && s.shows(0.1f, -0.1f, 255, 255, 255)); // nearest: hard edges
    CHECK(s.background(-2.5f, 0) && s.background(0, 2.5f));
    CHECK(s.r.stats().draw_calls == 1);
    CHECK(!s.pick(-1, 1).hit); // never picked
    const auto bounds = s.r.bounds();
    CHECK(close(bounds[0], -2) && close(bounds[1], -2) && close(bounds[3], 2) && close(bounds[4], 2));

    // The matrix places it: a negative y scale puts the first row at the low edge.
    d.model = scaled(2, -2, 1, 1);
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(0.5f, 0.5f, 255, 0, 0) && s.shows(1.5f, 1.5f, 255, 255, 255) && s.background(-1, -1));
    d.model = scaled(4, 4);

    // Opacity and tint: blended over the background.
    d.color = {1, 1, 1, 0.5f};
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(-1, 1, 133, 10, 15, 2) && s.shows(1, -1, 133, 138, 143, 2));
    d.color = {0, 1, 0, 1};
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(-1, 1, 0, 0, 0) && s.shows(1, 1, 0, 255, 0) && s.shows(1, -1, 0, 255, 0));
    d.color = {1, 1, 1, 1};

    // Depth: a magenta plate above the left half hides a depth-tested quad and not a kOnTop one.
    const float plate[] = {-2, -2, 1, 0, -2, 1, 0, 2, 1, -2, 2, 1};
    const uint32_t plate_indices[] = {0, 1, 2, 0, 2, 3};
    GeometryDesc gd;
    gd.positions = plate;
    gd.vertex_count = 4;
    gd.indices = plate_indices;
    gd.index_count = 6;
    const uint32_t geometry = s.r.add_geometry(gd);
    const uint32_t instance = s.r.add_instance(geometry);
    InstanceDesc id;
    id.color = {1, 0, 1, 1};
    id.flags = kUnlit | kDoubleSided;
    s.r.set_instance(instance, id);
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(-1, 1, 255, 0, 255) && s.shows(-1, -1, 255, 0, 255) && s.shows(1, 1, 0, 255, 0));
    d.flags = kNearest | kOnTop;
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(-1, 1, 255, 0, 0) && s.shows(-1, -1, 0, 0, 255) && s.shows(1, 1, 0, 255, 0));
    // The quad writes no depth: a transparent plate drawn before it is still blended under it, and an
    // instance drawn on top stays on top.
    id.flags = kUnlit | kDoubleSided | kOnTop;
    s.r.set_instance(instance, id);
    d.flags = kNearest;
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.shows(-1, 1, 255, 0, 255) && s.shows(1, 1, 0, 255, 0));
    id.flags = kUnlit | kDoubleSided | kHidden;
    s.r.set_instance(instance, id);

    // Section planes cut a kClipped quad only.
    const float plane[4] = {1, 0, 0, 0}; // keeps x > 0
    s.r.set_clip_planes(plane, 1);
    s.draw();
    CHECK(s.shows(-1, 1, 255, 0, 0) && s.shows(1, 1, 0, 255, 0));
    d.flags = kNearest | kClipped;
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.background(-1, 1) && s.background(-1, -1) && s.shows(1, 1, 0, 255, 0) && s.shows(1, -1, 255, 255, 255));
    s.r.set_clip_planes(nullptr, 0);
    d.flags = kNearest | kHidden;
    s.r.set_quad(quad, d);
    s.draw();
    CHECK(s.background(-1, 1) && s.background(1, -1));
    CHECK(s.r.stats().draw_calls == 0);
    d.flags = kNearest;
    s.r.set_quad(quad, d);

    // New pixels, in place (the same size) and as a new image (another size); the id stays.
    const uint8_t yellow[16] = {255, 255, 0, 255, 255, 255, 0, 255, 255, 255, 0, 255, 255, 255, 0, 255};
    s.r.update_texture(tex, yellow, 2, 2);
    s.draw();
    CHECK(s.shows(-1, 1, 255, 255, 0) && s.shows(1, -1, 255, 255, 0));
    std::vector<uint8_t> wide(4 * 1 * 4, 255);
    for (int x = 0; x < 4; ++x) {
        wide[x * 4] = 0;
        wide[x * 4 + 1] = uint8_t(x < 2 ? 0 : 255);
    }
    s.r.update_texture(tex, wide.data(), 4, 1);
    s.draw();
    CHECK(s.shows(-1.5f, 0, 0, 0, 255) && s.shows(-0.5f, 1, 0, 0, 255) && s.shows(0.5f, -1, 0, 255, 255) &&
          s.shows(1.5f, 0, 0, 255, 255));
    for (int frame = 0; frame < 3; ++frame) { // every frame: each write is what the next draw shows
        const uint8_t v = uint8_t(60 * (frame + 1));
        const uint8_t grey[16] = {v, v, v, 255, v, v, v, 255, v, v, v, 255, v, v, v, 255};
        s.r.update_texture(tex, grey, 4, 1);
        s.draw();
        CHECK(s.shows(-1.5f, 0, v, v, v) && s.shows(1.5f, 0, v, v, v));
    }

    // Filtered or nearest: a black and a white texel, looked at just beside where they meet.
    const uint8_t bw[8] = {0, 0, 0, 255, 255, 255, 255, 255};
    for (const bool mipmaps : {false, true}) {
        const uint32_t t2 = s.r.add_texture(bw, 2, 1, mipmaps);
        d.texture = t2;
        d.flags = kNearest;
        s.r.set_quad(quad, d);
        s.draw();
        CHECK(s.shows(-0.2f, 0, 0, 0, 0) && s.shows(0.2f, 0, 255, 255, 255));
        d.flags = 0;
        s.r.set_quad(quad, d);
        s.draw();
        const int left = s.at(-0.2f, 0)[0], right = s.at(0.2f, 0)[0];
        CHECK(left > 100 && left < right && right < 240);
        CHECK(s.shows(-1.8f, 0, 0, 0, 0, 12) && s.shows(1.8f, 0, 255, 255, 255, 12));
        s.r.remove_texture(t2);
        s.draw(); // a quad whose texture is gone is not drawn
        CHECK(s.background(1.8f, 0));
    }

    // No texture: the tint alone. Draw order: render_order, then back to front.
    d.texture = kNoTexture;
    d.flags = 0;
    d.color = {1, 0, 0, 1};
    d.model = scaled(2, 2, 0, 0, 0);
    s.r.set_quad(quad, d);
    const uint32_t over = s.r.add_quad();
    QuadDesc o;
    o.color = {0, 0, 1, 1};
    o.model = scaled(2, 2, 0.5f, 0, -1); // farther from the camera
    s.r.set_quad(over, o);
    s.draw();
    CHECK(s.shows(-0.8f, 0, 255, 0, 0) && s.shows(0.2f, 0, 255, 0, 0) && s.shows(1.3f, 0, 0, 0, 255));
    o.flags = kOnTop;
    s.r.set_quad(over, o);
    s.draw();
    CHECK(s.shows(-0.8f, 0, 255, 0, 0) && s.shows(0.2f, 0, 0, 0, 255) && s.shows(1.3f, 0, 0, 0, 255));
    o.flags = 0;
    o.model = scaled(2, 2, 0.5f, 0, 0); // coplanar: the later render_order wins
    o.render_order = -1;
    s.r.set_quad(over, o);
    s.draw();
    CHECK(s.shows(0.2f, 0, 255, 0, 0));
    o.render_order = 1;
    s.r.set_quad(over, o);
    s.draw();
    CHECK(s.shows(0.2f, 0, 0, 0, 255));

    s.r.remove_quad(over);
    s.r.remove_quad(quad);
    s.draw();
    CHECK(s.background(0, 0));
    CHECK(s.r.add_quad() == 0); // ids are reused
    s.r.clear();
    s.draw();
    CHECK(s.background(0, 0));
}

} // namespace

int main() {
    std::unique_ptr<NativeDevice> dev;
    try {
        dev = std::make_unique<NativeDevice>();
    } catch (const std::exception &e) {
        std::printf("SKIPPED: no adapter to draw with (%s)\n", e.what());
        return 77;
    }
    std::printf("adapter: %s (%s)\n", dev->info().device.c_str(), dev->info().backend.c_str());
    test_animation(*dev);
    test_feature_edge_objects(*dev);
    test_offset(*dev);
#ifdef ADACPP_RENDER_DRACO
    test_draco(*dev);
#else
    std::printf("built without Draco: its test is skipped\n");
#endif
    test_quads(*dev);
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
