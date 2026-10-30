// adacpp WebGPU renderer: a retained-mode, cluster ("meshlet") based renderer for CAD and FEA models.
//
// One implementation for every host: the browser (wasm, emdawnwebgpu), the headless CLI and Python
// (native, wgpu-native). The host owns the device and the colour target; this class owns the scene and
// draws it.
//
// Scene model
//   geometry   vertex data + primitives, uploaded once. Triangle geometries are split into spatially
//              compact clusters of at most 255 vertices / 124 triangles (meshoptimizer), which may span
//              several small segments; hidden state and colour are resolved per triangle.
//   segment    a contiguous index range of a geometry, i.e. one CAD object / FEM part inside a batched
//              buffer (adapy's draw ranges). Segments carry the per-object state: hidden, colour override.
//   instance   a geometry placed in the world: model matrix, base colour, flags, morph influence.
//
// Drawing is GPU "vertex pulling": clusters are culled (hidden segment, view frustum, back-facing normal
// cone, occlusion) into a visible-cluster list, and one non-indexed draw per instance expands
// visible_clusters x 124 x 3 vertices in the vertex shader, which looks every vertex up through the
// cluster's local index list. Unused triangle slots of a partial cluster collapse to a degenerate point.
//
// Large scenes (2M+ triangles) are culled on the GPU, in two passes against the previous frame's visible
// set (see kCullWGSL): what was visible last frame is drawn first, a depth pyramid is built from it, and
// every other cluster is tested against that pyramid before the second pass draws the ones still visible.
// Smaller scenes, scenes with occlusion culling switched off, and picking are culled on the CPU (frustum +
// cone).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "gpu.h"

namespace adacpp::render {

enum class Topology : uint32_t { Triangles = 0, Lines = 1, Points = 2 };

// Instance flags. Keep in step with the WGSL constants in shaders.h and with the TypeScript bridge.
enum InstanceFlags : uint32_t {
    kVertexColors = 1u << 0,  // multiply the base colour by the geometry's per-vertex colours
    kUnlit = 1u << 1,         // no lighting (MeshBasicMaterial, lines, points)
    kDoubleSided = 1u << 2,   // no back-face culling (and no cone culling)
    kTransparent = 1u << 3,   // alpha-blended, drawn after the opaque pass without depth writes
    kHidden = 1u << 4,        // skip the whole instance
    kMorphRelative = 1u << 5, // morph target holds displacements (else absolute target positions)
    kNoPick = 1u << 8,        // drawn, but never returned by pick() (helpers, overlays, edges)
    kClipped = 1u << 11,      // cut by the section planes (set_clip_planes), as three.js' clippingPlanes
    kDepthOffset = 1u << 12,  // triangles pulled in front of coplanar faces and lines (negative polygonOffset)
};

// Segment state flags (second u32 of a segment's state pair).
enum SegmentFlags : uint32_t {
    kSegmentHidden = 1u << 0,
    // The colour override is a matte surface (metalness 0, roughness 1: three.js' default
    // MeshStandardMaterial, e.g. the selection highlight), not the instance's own material.
    kSegmentMatte = 1u << 1,
};

struct GeometryDesc {
    Topology topology = Topology::Triangles;
    const float *positions = nullptr; // 3 per vertex
    size_t vertex_count = 0;
    const float *normals = nullptr;    // optional, 3 per vertex
    const uint32_t *colors = nullptr;  // optional, packed RGBA8 per vertex, linear
    const float *morph = nullptr;      // optional single morph target, 3 per vertex
    const uint32_t *indices = nullptr; // optional; non-indexed geometry draws 0..vertex_count-1
    size_t index_count = 0;
    const uint32_t *segments = nullptr; // optional [start, count] pairs in index units; default one segment
    size_t segment_count = 0;
    const float *distances = nullptr; // optional, lines: distance along the line at each vertex (for dashes)
};

struct InstanceDesc {
    std::array<float, 16> model{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // column-major
    std::array<float, 4> color{1, 1, 1, 1};                                      // linear RGBA
    uint32_t flags = 0;
    float morph_influence = 0.0f;
    // three.js MeshStandardMaterial parameters; `specular` false = Lambert (diffuse only).
    float metalness = 0.0f;
    float roughness = 1.0f;
    bool specular = true;
    // Lines: width in pixels; points: side in pixels (squares, like three.js). Above 1 they are drawn as
    // screen-space quads.
    float line_width = 1.0f;
    // Lines: dash and gap lengths, in the geometry's `distances` units (three.js' dashSize / gapSize); a
    // zero dash draws solid.
    float dash = 0.0f;
    float gap = 0.0f;
};

// What is under a pixel: the nearest pickable triangle there.
struct PickResult {
    bool hit = false;
    uint32_t instance = 0; // instance id, as returned by add_instance()
    uint32_t segment = 0;  // segment index within the instance's geometry
    uint32_t triangle = 0; // triangle index in the geometry's original index list (index position / 3)
    float world[3] = {0, 0, 0};
};

struct RenderStats {
    uint32_t geometries = 0;
    uint32_t instances = 0;
    uint64_t clusters_total = 0;
    uint64_t clusters_drawn = 0;
    uint64_t triangles_total = 0;
    uint64_t triangles_drawn = 0;
    uint64_t clusters_occluded = 0;
    uint32_t draw_calls = 0;
    double cull_ms = 0.0;
    // GPU culling: the drawn / occluded counts live on the GPU and are only filled in by read_cull_stats().
    bool gpu_culled = false;
};

class Renderer {
public:
    // 255 = the most a u8 local index addresses. No mesh-shader limit applies to vertex pulling, and flat-shaded
    // CAD (1.65 vertices per triangle) filled 64-vertex clusters at ~38 of 124 triangles.
    static constexpr uint32_t kClusterMaxVertices = 255;
    static constexpr uint32_t kClusterMaxTriangles = 124;
    static constexpr uint32_t kSampleCount = 4;

    // `color_format` is the format of every target passed to render(). The renderer writes sRGB-encoded
    // values itself, so pass a non-sRGB (unorm) format.
    Renderer(WGPUDevice device, WGPUTextureFormat color_format);
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    uint32_t add_geometry(const GeometryDesc &desc);
    void update_colors(uint32_t geometry, const uint32_t *colors, size_t count);
    void update_morph(uint32_t geometry, const float *morph, size_t vertex_count);
    void remove_geometry(uint32_t geometry);
    uint32_t segment_count(uint32_t geometry) const;

    uint32_t add_instance(uint32_t geometry);
    void set_instance(uint32_t instance, const InstanceDesc &desc);
    // Two u32 per segment: RGBA8 colour override (alpha 0 = no override) and SegmentFlags.
    void set_segment_states(uint32_t instance, const uint32_t *packed, size_t segment_count);
    void remove_instance(uint32_t instance);
    void clear();

    // `proj_gl` is an OpenGL-convention projection (clip z in [-1, 1], e.g. three.js); it is remapped to
    // WebGPU's [0, 1] here. Matrices are column-major.
    void set_camera(const float view[16], const float proj_gl[16], bool orthographic);
    void set_background(float r, float g, float b, float a = 1.0f); // sRGB-encoded, as written to the target
    // Ambient irradiance and headlight intensity, in three.js units (AmbientLight / DirectionalLight).
    void set_lighting(float ambient, float headlight);
    // `occlusion` allows GPU culling with two-pass occlusion culling (used from 2M triangles); off, the CPU
    // culls (frustum, cone).
    void set_culling(bool frustum, bool cone, bool occlusion = true);
    // Section planes: `count` (at most kMaxClipPlanes) planes as (nx, ny, nz, constant), three.js' Plane. A
    // point p of a kClipped instance is cut away where dot(n, p) + constant < 0 for any plane.
    static constexpr size_t kMaxClipPlanes = 6;
    void set_clip_planes(const float *planes, size_t count);

    void render(WGPUTextureView target, uint32_t width, uint32_t height);

    // GPU picking at pixel (x, y) of a width x height target (origin top-left), with the camera of the
    // last set_camera(). Renders only that pixel (a pick matrix narrows the frustum, so cluster culling
    // drops nearly everything), reads it back asynchronously and calls `done` -- from the browser's
    // event loop on the web; natively once the device is polled.
    void pick(uint32_t x, uint32_t y, uint32_t width, uint32_t height, std::function<void(const PickResult &)> done);
    const RenderStats &stats() const {
        return stats_;
    }
    // The last frame's stats with the GPU culling counts read back (drawn clusters / triangles, occluded
    // clusters). Asynchronous like pick(); with CPU culling it calls back with stats() as they are.
    void read_cull_stats(std::function<void(const RenderStats &)> done);

    // World-space bounds of all visible instances: min xyz, max xyz. Empty scene -> all zeros.
    std::array<float, 6> bounds() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    RenderStats stats_;
};

} // namespace adacpp::render
