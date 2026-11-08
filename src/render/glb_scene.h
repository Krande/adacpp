// GLB -> Renderer scene loading (tinygltf), and a camera that frames it the way the viewer does. The one
// GLB reader for every host: the Python offscreen renderer (posters) and the browser (wasm_bindings), so
// a model draws the same in both.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "renderer.h"

namespace adacpp::render {

// One geometry the load added (a batch of primitives of one material and topology; or a feature-edge
// overlay), with its instance, so a caller can drive it afterwards (a morph, per-vertex colours).
struct GlbGeometry {
    uint32_t geometry = 0;
    uint32_t instance = 0;
    Topology topology = Topology::Triangles;
    uint32_t vertex_count = 0;
    bool vertex_colors = false;
    bool feature_edges = false; // the overlay GlbOptions::feature_edge_angle_deg added
    // World-space positions (3 per vertex), in the geometry's vertex order, when GlbOptions::keep_positions.
    std::vector<float> positions;
};

struct GlbLoadResult {
    uint32_t geometries = 0;
    uint32_t instances = 0;
    uint64_t vertices = 0;
    uint64_t triangles = 0;
    uint64_t segments = 0; // per-object index ranges (adapy draw ranges), 0 = one per primitive
    uint64_t feature_edges = 0;
    // World-space bounds of what was loaded: min xyz, max xyz (inverted when nothing was).
    std::array<float, 6> bounds{3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
    std::vector<GlbGeometry> loaded;
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
    // Above 0: also draw the surfaces' feature edges as Ada Studio's viewer does with tessellation edges
    // hidden (its default) -- vertices welded by position, then every boundary edge and every edge whose
    // two faces meet at more than this angle (degrees; the viewer's is 30), in `line_color` at
    // `line_width`. 0 draws only the lines the GLB carries.
    float feature_edge_angle_deg = 0.0f;
    // Keep each added geometry's world-space positions in GlbLoadResult::loaded (for a caller that builds
    // more geometry on the same vertices, e.g. an FEA mesh's element edges).
    bool keep_positions = false;
};
GlbLoadResult load_glb(Renderer &renderer, const std::string &path, const GlbOptions &options = {});
// The same from the GLB's bytes (the browser has no file to name). `name` labels errors.
GlbLoadResult load_glb_bytes(Renderer &renderer, std::vector<unsigned char> bytes, const GlbOptions &options = {},
                             const std::string &name = "GLB");

// Feature edges of a triangle mesh (positions: 3 per vertex; indices: 3 per triangle), as segment endpoint
// positions (6 floats per edge): see GlbOptions::feature_edge_angle_deg.
std::vector<float> feature_edges(const std::vector<float> &positions, const std::vector<uint32_t> &indices,
                                 float angle_deg);

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
