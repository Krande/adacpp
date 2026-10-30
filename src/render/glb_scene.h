// Native GLB -> Renderer scene loading (tinygltf), and a camera that frames it the way the viewer does.
#pragma once

#ifndef __EMSCRIPTEN__

#include <array>
#include <string>

#include "renderer.h"

namespace adacpp::render {

struct GlbLoadResult {
    uint32_t geometries = 0;
    uint32_t instances = 0;
    uint64_t vertices = 0;
    uint64_t triangles = 0;
    uint64_t segments = 0; // per-object index ranges (adapy draw ranges), 0 = one per primitive
};

// Adds every mesh primitive of the GLB's default scene as one geometry + instance, with node transforms
// applied. adapy writes its z-up coordinates into the GLB as they are (the viewer only turns the camera's
// up axis), so nothing is rotated by default; `y_up_to_z_up` rotates a conventional y-up glTF instead.
struct GlbOptions {
    bool y_up_to_z_up = false;
    // Pixels (see InstanceDesc::line_width). Lines without vertex colours are wireframes / feature edges;
    // lines with them are FEA members (beam elements) drawn in their field colour.
    float line_width = 1.0f;
    float member_width = 1.0f;
    float point_size = 1.0f;
    // Linear RGBA for every line without vertex colours (wireframes, edges); alpha 0 = its material's colour.
    std::array<float, 4> line_color{0, 0, 0, 0};
    // Draw every surface double-sided, whatever its material says (thin shells seen from behind).
    bool double_sided = false;
};
GlbLoadResult load_glb(Renderer &renderer, const std::string &path, const GlbOptions &options = {});

struct Camera {
    std::array<float, 16> view;
    std::array<float, 16> proj; // OpenGL convention, like three.js
};

// Perspective camera looking at `bounds` (min xyz, max xyz) from `direction` (towards the camera, world
// space), fitted so the bounding sphere fills the view. `up` is the world up axis.
// Perspective camera at `eye` looking at `target`; near/far are taken from `fit` (a frame_bounds()
// projection) so depth precision matches the fitted view.
Camera look_at(const float eye[3], const float target[3], std::array<float, 3> up, float fov_y_deg, float aspect,
               const std::array<float, 16> &fit);

Camera frame_bounds(const std::array<float, 6> &bounds, std::array<float, 3> direction, std::array<float, 3> up,
                    float fov_y_deg, float aspect, float margin = 1.1f);

} // namespace adacpp::render

#endif
