// adacpp_render_effects_demo: the realistic visuals (effects.h) on a small generic scene -- metal parts under the
// studio environment, a flickering point light, a spark emitter with a little smoke, and a strip cooling down
// from a glow -- rendered headless to PNG frames, and timed.
//
//   adacpp_render_effects_demo <out_dir> [--size WxH] [--frames N] [--fps F] [--bench N] [--backend B]
//
// Writes <out_dir>/plain.png (the same scene with every effect off), <out_dir>/effects.png (t = 1.5 s) and,
// with --frames, <out_dir>/frame_000.png ... (t from 0 at --fps). --bench N times N frames of each
// configuration (GPU work waited for) and prints the means as JSON.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "glb_scene.h"
#include "native_device.h"
#include "renderer.h"

using namespace adacpp::render;

namespace {

struct Mesh {
    std::vector<float> positions, normals;
    std::vector<uint32_t> indices;
    void quad(const std::array<float, 3> &o, const std::array<float, 3> &u, const std::array<float, 3> &v,
              const std::array<float, 3> &n) {
        const uint32_t b = uint32_t(positions.size() / 3);
        for (int c = 0; c < 4; ++c) {
            const float a = (c == 1 || c == 2) ? 1.0f : 0.0f, d = c >= 2 ? 1.0f : 0.0f;
            for (int k = 0; k < 3; ++k) {
                positions.push_back(o[k] + a * u[k] + d * v[k]);
                normals.push_back(n[k]);
            }
        }
        indices.insert(indices.end(), {b, b + 1, b + 2, b, b + 2, b + 3});
    }
};

Mesh box(float x0, float y0, float z0, float x1, float y1, float z1) {
    Mesh m;
    const float dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
    m.quad({x0, y0, z1}, {dx, 0, 0}, {0, dy, 0}, {0, 0, 1});
    m.quad({x0, y1, z0}, {dx, 0, 0}, {0, -dy, 0}, {0, 0, -1});
    m.quad({x0, y0, z0}, {dx, 0, 0}, {0, 0, dz}, {0, -1, 0});
    m.quad({x1, y1, z0}, {-dx, 0, 0}, {0, 0, dz}, {0, 1, 0});
    m.quad({x1, y0, z0}, {0, dy, 0}, {0, 0, dz}, {1, 0, 0});
    m.quad({x0, y1, z0}, {0, -dy, 0}, {0, 0, dz}, {-1, 0, 0});
    return m;
}

// A tube along x from x0 to x1 with radius r around (y, z), smooth normals, capped.
Mesh cylinder_x(float x0, float x1, float y, float z, float r, int n = 64) {
    Mesh m;
    for (int i = 0; i <= n; ++i) {
        const float a = 2.0f * 3.14159265f * i / n, c = std::cos(a), s = std::sin(a);
        for (float x : {x0, x1}) {
            m.positions.insert(m.positions.end(), {x, y + r * c, z + r * s});
            m.normals.insert(m.normals.end(), {0, c, s});
        }
    }
    for (uint32_t i = 0; i < uint32_t(n); ++i) {
        const uint32_t a = i * 2, b = a + 1, c = a + 2, d = a + 3;
        m.indices.insert(m.indices.end(), {a, c, d, a, d, b});
    }
    for (int side = 0; side < 2; ++side) {
        const float x = side ? x1 : x0, nx = side ? 1.0f : -1.0f;
        const uint32_t centre = uint32_t(m.positions.size() / 3);
        m.positions.insert(m.positions.end(), {x, y, z});
        m.normals.insert(m.normals.end(), {nx, 0, 0});
        for (int i = 0; i <= n; ++i) {
            const float a = 2.0f * 3.14159265f * i / n;
            m.positions.insert(m.positions.end(), {x, y + r * std::cos(a), z + r * std::sin(a)});
            m.normals.insert(m.normals.end(), {nx, 0, 0});
        }
        for (uint32_t i = 0; i < uint32_t(n); ++i)
            if (side)
                m.indices.insert(m.indices.end(), {centre, centre + 1 + i, centre + 2 + i});
            else
                m.indices.insert(m.indices.end(), {centre, centre + 2 + i, centre + 1 + i});
    }
    return m;
}

Mesh sphere(float cx, float cy, float cz, float r, int n = 48) {
    Mesh m;
    for (int j = 0; j <= n; ++j) {
        const float th = 3.14159265f * j / n;
        for (int i = 0; i <= 2 * n; ++i) {
            const float ph = 3.14159265f * i / n;
            const float x = std::sin(th) * std::cos(ph), y = std::sin(th) * std::sin(ph), z = std::cos(th);
            m.positions.insert(m.positions.end(), {cx + r * x, cy + r * y, cz + r * z});
            m.normals.insert(m.normals.end(), {x, y, z});
        }
    }
    const uint32_t row = uint32_t(2 * n + 1);
    for (uint32_t j = 0; j < uint32_t(n); ++j)
        for (uint32_t i = 0; i < uint32_t(2 * n); ++i) {
            const uint32_t a = j * row + i, b = a + 1, c = a + row, d = c + 1;
            m.indices.insert(m.indices.end(), {a, c, d, a, d, b});
        }
    return m;
}

uint32_t add(Renderer &r, const Mesh &m, std::array<float, 4> color, float metal, float rough) {
    GeometryDesc g;
    g.positions = m.positions.data();
    g.vertex_count = m.positions.size() / 3;
    g.normals = m.normals.data();
    g.indices = m.indices.data();
    g.index_count = m.indices.size();
    const uint32_t inst = r.add_instance(r.add_geometry(g));
    InstanceDesc d;
    d.color = color;
    d.metalness = metal;
    d.roughness = rough;
    r.set_instance(inst, d);
    return inst;
}

struct Scene {
    uint32_t strip = 0, light = 0, sparks = 0, smoke = 0, ramp = 0;
};

constexpr std::array<float, 3> kTorch{0.55f, -0.05f, 0.06f};

// The parts, and every effect (left on).
Scene build(Renderer &r) {
    Scene s;
    add(r, box(-1.2f, -0.8f, -0.04f, 1.2f, 0.8f, 0.0f), {0.32f, 0.33f, 0.35f, 1}, 1.0f, 0.55f); // a steel plate
    add(r, cylinder_x(-0.9f, 0.9f, 0.4f, 0.16f, 0.16f), {0.78f, 0.78f, 0.8f, 1}, 1.0f, 0.22f);  // a polished pipe
    add(r, sphere(-0.55f, -0.35f, 0.2f, 0.2f), {0.95f, 0.95f, 0.95f, 1}, 1.0f, 0.06f);          // chrome
    add(r, box(0.75f, -0.6f, 0.0f, 1.05f, -0.3f, 0.3f), {0.95f, 0.64f, 0.54f, 1}, 1.0f, 0.35f); // copper
    add(r, box(-0.25f, 0.05f, 0.0f, 0.05f, 0.25f, 0.45f), {0.6f, 0.05f, 0.05f, 1}, 0.0f, 0.5f); // painted
    // A strip laid along the plate, cooling down from a glow.
    s.strip = add(r, box(-0.6f, -0.08f, 0.0f, kTorch[0], -0.02f, 0.025f), {0.3f, 0.3f, 0.32f, 1}, 1.0f, 0.4f);
    EmissiveRampDesc ramp;
    ramp.hot_color = {1.0f, 0.62f, 0.25f};
    ramp.hot_intensity = 12.0f;
    ramp.cool_color = {0.7f, 0.08f, 0.01f};
    ramp.cool_intensity = 0.0f;
    ramp.start = 0.0;
    ramp.duration = 4.0f;
    ramp.curve = 0.45f;
    s.ramp = r.add_emissive_ramp();
    r.set_emissive_ramp(s.ramp, ramp);
    InstanceDesc d = r.instance_desc(s.strip);
    d.emissive_ramp = s.ramp;
    r.set_instance(s.strip, d);

    PointLightDesc l;
    l.position = {kTorch[0], kTorch[1], kTorch[2] + 0.03f};
    l.color = {0.75f, 0.85f, 1.0f};
    l.intensity = 1.2f;
    l.flicker = 0.55f;
    l.flicker_rate = 18.0f;
    l.seed = 11;
    s.light = r.add_point_light();
    r.set_point_light(s.light, l);

    EmitterDesc e;
    e.origin = kTorch;
    e.direction = {0.2f, 0.0f, 1.0f};
    e.spread = 1.1f;
    e.speed = 1.1f;
    e.speed_jitter = 0.6f;
    e.rate = 600.0f;
    e.lifetime = 0.55f;
    e.gravity = {0, 0, -9.81f};
    e.drag = 0.6f;
    e.size_start = 0.008f;
    e.size_end = 0.003f;
    e.stretch = 0.025f;
    e.seed = 5;
    e.start = 0.0;
    s.sparks = r.add_emitter();
    r.set_emitter(s.sparks, e);
    EmitterDesc smoke = smoke_emitter({kTorch[0], kTorch[1], kTorch[2] + 0.05f});
    smoke.size_start = 0.04f;
    smoke.size_end = 0.35f;
    smoke.alpha_start = 0.12f;
    s.smoke = r.add_emitter();
    r.set_emitter(s.smoke, smoke);
    return s;
}

void effects_on(Renderer &r, bool bloom) {
    PostSettings p;
    p.hdr = true;
    p.tone_mapping = ToneMapping::Aces;
    p.exposure = 1.0f;
    p.bloom = bloom;
    p.bloom_strength = 0.5f;
    r.set_post(p);
    EnvironmentDesc env;
    env.enabled = true;
    env.intensity = 1.0f;
    env.rotation = 0.6f;
    r.set_environment(env);
    r.set_lighting(0.2f, 0.4f);
}

void effects_off(Renderer &r) {
    r.set_post(PostSettings{});
    r.set_environment(EnvironmentDesc{});
    r.set_lighting(1.5707964f, 1.4f); // the viewer's defaults
}

bool save(const std::string &path, std::vector<uint8_t> rgba, uint32_t w, uint32_t h) {
    if (!stbi_write_png(path.c_str(), int(w), int(h), 4, rgba.data(), int(w * 4))) {
        std::fprintf(stderr, "failed to write %s\n", path.c_str());
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <out_dir> [--size WxH] [--frames N] [--fps F] [--bench N] [--backend B]\n",
                     argv[0]);
        return 2;
    }
    const std::filesystem::path out = argv[1];
    uint32_t w = 1280, h = 720;
    int frames = 0, bench = 0;
    float fps = 20.0f;
    std::string backend = "auto";
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--size" && i + 1 < argc)
            std::sscanf(argv[++i], "%ux%u", &w, &h);
        else if (a == "--frames" && i + 1 < argc)
            frames = std::atoi(argv[++i]);
        else if (a == "--fps" && i + 1 < argc)
            fps = std::strtof(argv[++i], nullptr);
        else if (a == "--bench" && i + 1 < argc)
            bench = std::atoi(argv[++i]);
        else if (a == "--backend" && i + 1 < argc)
            backend = argv[++i];
    }
    try {
        std::filesystem::create_directories(out);
        NativeDevice dev(backend);
        std::fprintf(stderr, "adapter: %s (%s)\n", dev.info().device.c_str(), dev.info().backend.c_str());
        Renderer r(dev.device(), OffscreenTarget::kFormat);
        r.set_background(0.13f, 0.14f, 0.16f);
        Scene s = build(r);
        const float eye[3] = {1.55f, -1.75f, 1.05f}, target[3] = {0.0f, 0.0f, 0.12f};
        const Camera fit = frame_bounds(r.bounds(), {1, -1, 0.7f}, {0, 0, 1}, 45.0f, float(w) / float(h));
        const Camera cam = look_at(eye, target, {0, 0, 1}, 45.0f, float(w) / float(h), fit.proj);
        r.set_camera(cam.view.data(), cam.proj.data(), false);
        OffscreenTarget t(dev, w, h);

        // The same scene with every effect off: the renderer as it was.
        effects_off(r);
        r.remove_point_light(s.light);
        r.remove_emitter(s.sparks);
        r.remove_emitter(s.smoke);
        r.remove_emissive_ramp(s.ramp);
        r.set_time(1.5);
        r.render(t.view(), w, h);
        if (!save((out / "plain.png").string(), t.read_rgba(), w, h))
            return 1;
        auto time_frames = [&](int n) {
            r.render(t.view(), w, h);
            dev.wait_idle();
            const auto t0 = std::chrono::steady_clock::now();
            for (int f = 0; f < n; ++f) {
                r.set_time(1.5 + f / 60.0);
                r.render(t.view(), w, h);
                dev.wait_idle();
            }
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / n;
        };
        double ms_plain = 0, ms_hdr = 0, ms_bloom = 0, ms_env = 0, ms_all = 0;
        if (bench > 0) {
            ms_plain = time_frames(bench);
            PostSettings p;
            p.hdr = true;
            r.set_post(p);
            ms_hdr = time_frames(bench);
            p.bloom = true;
            r.set_post(p);
            ms_bloom = time_frames(bench);
            effects_on(r, true);
            ms_env = time_frames(bench);
        }

        // Every effect on.
        r.clear();
        s = build(r);
        r.set_camera(cam.view.data(), cam.proj.data(), false);
        effects_on(r, true);
        r.set_time(1.5);
        r.render(t.view(), w, h);
        if (!save((out / "effects.png").string(), t.read_rgba(), w, h))
            return 1;
        if (bench > 0)
            ms_all = time_frames(bench);
        for (int f = 0; f < frames; ++f) {
            r.set_time(f / double(fps));
            r.render(t.view(), w, h);
            char name[64];
            std::snprintf(name, sizeof name, "frame_%03d.png", f);
            if (!save((out / name).string(), t.read_rgba(), w, h))
                return 1;
        }
        if (bench > 0)
            std::printf(
                "{\"size\":\"%ux%u\",\"adapter\":\"%s\",\"backend\":\"%s\",\"frames\":%d,\"ms\":{\"plain\":%.3f,"
                "\"hdr\":%.3f,\"hdr_bloom\":%.3f,\"hdr_bloom_ibl\":%.3f,\"all_effects\":%.3f}}\n",
                w, h, dev.info().device.c_str(), dev.info().backend.c_str(), bench, ms_plain, ms_hdr, ms_bloom, ms_env,
                ms_all);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "adacpp_render_effects_demo: %s\n", e.what());
        return 1;
    }
    return 0;
}
