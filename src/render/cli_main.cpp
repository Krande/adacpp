// adacpp_render: headless GLB -> PNG through the same WebGPU renderer the browser uses.
//
//   adacpp_render <in.glb> <out.png> [--size WxH] [--view iso|front|back|left|right|top|bottom]
//                 [--backend auto|vulkan|gl] [--y-up] [--no-cull] [--cpu-cull] [--stats]
//                 [--pick X,Y] [--bench N] [--camera ex,ey,ez,tx,ty,tz]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "glb_scene.h"
#include "native_device.h"
#include "renderer.h"

using namespace adacpp::render;

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <in.glb> <out.png> [--size WxH] [--view iso|front|back|left|right|top|bottom] "
                     "[--backend auto|vulkan|gl] [--y-up] [--no-cull] [--cpu-cull] [--stats] [--pick X,Y] [--bench N] "
                     "[--camera ex,ey,ez,tx,ty,tz]\n",
                     argv[0]);
        return 2;
    }
    std::string in = argv[1], out = argv[2], view = "iso", backend = "auto";
    uint32_t w = 1280, h = 960;
    bool cull = true, gpu_cull = true, print_stats = false, y_up = false;
    int pick_x = -1, pick_y = -1, bench = 0;
    float cam_eye[3], cam_target[3];
    bool cam_set = false;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--size" && i + 1 < argc)
            std::sscanf(argv[++i], "%ux%u", &w, &h);
        else if (a == "--view" && i + 1 < argc)
            view = argv[++i];
        else if (a == "--backend" && i + 1 < argc)
            backend = argv[++i];
        else if (a == "--pick" && i + 1 < argc)
            std::sscanf(argv[++i], "%d,%d", &pick_x, &pick_y);
        else if (a == "--camera" && i + 1 < argc)
            cam_set = std::sscanf(argv[++i], "%f,%f,%f,%f,%f,%f", &cam_eye[0], &cam_eye[1], &cam_eye[2], &cam_target[0],
                                  &cam_target[1], &cam_target[2]) == 6;
        else if (a == "--bench" && i + 1 < argc)
            bench = std::atoi(argv[++i]);
        else if (a == "--y-up")
            y_up = true;
        else if (a == "--no-cull")
            cull = false;
        else if (a == "--cpu-cull")
            gpu_cull = false;
        else if (a == "--stats")
            print_stats = true;
    }
    // z-up world, matching the viewer's camera presets.
    const std::map<std::string, std::array<float, 3>> views = {
        {"iso", {-1, -1, 1}}, {"front", {0, -1, 0}},   {"back", {0, 1, 0}},        {"left", {-1, 0, 0}},
        {"right", {1, 0, 0}}, {"top", {0, 0.001f, 1}}, {"bottom", {0, 0.001f, -1}}};
    auto vit = views.find(view);
    if (vit == views.end()) {
        std::fprintf(stderr, "unknown view '%s'\n", view.c_str());
        return 2;
    }
    try {
        NativeDevice dev(backend);
        const auto &info = dev.info();
        std::fprintf(stderr, "adapter: %s (%s, %s)\n", info.device.c_str(), info.backend.c_str(),
                     info.description.c_str());
        Renderer renderer(dev.device(), OffscreenTarget::kFormat);
        renderer.set_culling(cull, cull, cull && gpu_cull);
        const auto load_t0 = std::chrono::steady_clock::now();
        const auto res = load_glb(renderer, in, y_up);
        const double load_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_t0).count();
        Camera cam = frame_bounds(renderer.bounds(), vit->second, {0, 0, 1}, 60.0f, float(w) / float(h));
        if (cam_set)
            cam = look_at(cam_eye, cam_target, {0, 0, 1}, 60.0f, float(w) / float(h), cam.proj);
        renderer.set_camera(cam.view.data(), cam.proj.data(), false);
        OffscreenTarget target(dev, w, h);
        renderer.render(target.view(), w, h);
        // --bench N: N more frames, each waited for, so the time includes the GPU.
        double bench_ms = 0.0;
        if (bench > 0) {
            dev.wait_idle();
            const auto t0 = std::chrono::steady_clock::now();
            for (int f = 0; f < bench; ++f) {
                renderer.render(target.view(), w, h);
                dev.wait_idle();
            }
            bench_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / bench;
        }
        const auto rgba = target.read_rgba();
        if (!stbi_write_png(out.c_str(), int(w), int(h), 4, rgba.data(), int(w * 4))) {
            std::fprintf(stderr, "failed to write %s\n", out.c_str());
            return 1;
        }
        if (pick_x >= 0 && pick_y >= 0) {
            bool done = false;
            PickResult hit;
            renderer.pick(uint32_t(pick_x), uint32_t(pick_y), w, h, [&](const PickResult &r) {
                hit = r;
                done = true;
            });
            while (!done)
                dev.process_events();
            std::printf("{\"pick\":{\"hit\":%s,\"instance\":%u,\"segment\":%u,\"triangle\":%u,"
                        "\"world\":[%.4f,%.4f,%.4f]}}\n",
                        hit.hit ? "true" : "false", hit.instance, hit.segment, hit.triangle, hit.world[0], hit.world[1],
                        hit.world[2]);
        }
        if (print_stats) {
            RenderStats st;
            bool have = false;
            renderer.read_cull_stats([&](const RenderStats &s) {
                st = s;
                have = true;
            });
            while (!have)
                dev.process_events();
            std::printf("{\"geometries\":%u,\"instances\":%u,\"vertices\":%llu,\"triangles\":%llu,"
                        "\"clusters_total\":%llu,\"clusters_drawn\":%llu,\"triangles_drawn\":%llu,"
                        "\"clusters_occluded\":%llu,\"gpu_cull\":%s,\"draw_calls\":%u,\"cull_ms\":%.3f,\"segments\":%"
                        "llu,\"frame_ms\":%.3f,\"load_ms\":%.0f,"
                        "\"backend\":\"%s\"}\n",
                        res.geometries, res.instances, (unsigned long long) res.vertices,
                        (unsigned long long) res.triangles, (unsigned long long) st.clusters_total,
                        (unsigned long long) st.clusters_drawn, (unsigned long long) st.triangles_drawn,
                        (unsigned long long) st.clusters_occluded, st.gpu_culled ? "true" : "false", st.draw_calls,
                        st.cull_ms, (unsigned long long) res.segments, bench_ms, load_ms, info.backend.c_str());
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "adacpp_render: %s\n", e.what());
        return 1;
    }
    return 0;
}
