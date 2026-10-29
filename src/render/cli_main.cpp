// adacpp_render: headless GLB -> PNG through the same WebGPU renderer the browser uses.
//
//   adacpp_render <in.glb> <out.png> [--size WxH] [--view iso|front|back|left|right|top|bottom]
//                 [--backend auto|vulkan|gl] [--y-up] [--no-cull] [--stats]
#include <cstdio>
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
                     "[--backend auto|vulkan|gl] [--y-up] [--no-cull] [--stats]\n",
                     argv[0]);
        return 2;
    }
    std::string in = argv[1], out = argv[2], view = "iso", backend = "auto";
    uint32_t w = 1280, h = 960;
    bool cull = true, print_stats = false, y_up = false;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--size" && i + 1 < argc)
            std::sscanf(argv[++i], "%ux%u", &w, &h);
        else if (a == "--view" && i + 1 < argc)
            view = argv[++i];
        else if (a == "--backend" && i + 1 < argc)
            backend = argv[++i];
        else if (a == "--y-up")
            y_up = true;
        else if (a == "--no-cull")
            cull = false;
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
        renderer.set_culling(cull, cull);
        const auto res = load_glb(renderer, in, y_up);
        const Camera cam = frame_bounds(renderer.bounds(), vit->second, {0, 0, 1}, 60.0f, float(w) / float(h));
        renderer.set_camera(cam.view.data(), cam.proj.data(), false);
        OffscreenTarget target(dev, w, h);
        renderer.render(target.view(), w, h);
        const auto rgba = target.read_rgba();
        if (!stbi_write_png(out.c_str(), int(w), int(h), 4, rgba.data(), int(w * 4))) {
            std::fprintf(stderr, "failed to write %s\n", out.c_str());
            return 1;
        }
        if (print_stats) {
            const auto &st = renderer.stats();
            std::printf("{\"geometries\":%u,\"instances\":%u,\"vertices\":%llu,\"triangles\":%llu,"
                        "\"clusters_total\":%llu,\"clusters_drawn\":%llu,\"triangles_drawn\":%llu,"
                        "\"draw_calls\":%u,\"cull_ms\":%.3f,\"backend\":\"%s\"}\n",
                        res.geometries, res.instances, (unsigned long long) res.vertices,
                        (unsigned long long) res.triangles, (unsigned long long) st.clusters_total,
                        (unsigned long long) st.clusters_drawn, (unsigned long long) st.triangles_drawn, st.draw_calls,
                        st.cull_ms, info.backend.c_str());
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "adacpp_render: %s\n", e.what());
        return 1;
    }
    return 0;
}
