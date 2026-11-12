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
//
// After the opaque model: section caps (the cut faces of kCapped solids, by stencil counting). After the
// transparent one: textured quads lying in the world, kOnTop instances (and quads), which ignore depth, and
// last of all sprites, camera-facing textured quads sorted back to front.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "camera_controller.h"
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
    kDepthBack = 1u << 14,    // triangles pushed behind coplanar lines, so edges on them win (positive polygonOffset)
    kUnderlay = 1u << 15,     // lines / points drawn before the faces (a ground grid): a coplanar face hides them
    kCapped = 1u << 16,       // a closed solid whose cut by the section planes is filled (set_section_caps)
    // Drawn over the model: after the opaque and transparent passes and the section caps, before the sprites,
    // in instance-id order, with no depth test or write (a transform gizmo: three.js' depthTest false).
    kOnTop = 1u << 17,
    // A transparent instance that still culls its back faces (three.js' single-sided transparent material).
    // Without it a transparent instance draws both sides. No effect on opaque instances.
    kCullBack = 1u << 18,
    // Quads (set_quad) only: the texture is sampled at its nearest texel, so the cells of a grid keep hard
    // edges. Clear, it is filtered (trilinear when the texture has a mip chain).
    kNearest = 1u << 19,
};

// Segment state flags (second u32 of a segment's state pair).
enum SegmentFlags : uint32_t {
    kSegmentHidden = 1u << 0,
    // The colour override is a matte surface (metalness 0, roughness 1: three.js' default
    // MeshStandardMaterial, e.g. the selection highlight), not the instance's own material.
    kSegmentMatte = 1u << 1,
};

// Sprite flags. Keep in step with the WGSL constants in shaders.h and with the TypeScript bridge.
enum SpriteFlags : uint32_t {
    kSpriteSizeAttenuation = 1u << 0, // three.js' sizeAttenuation; clear, a perspective camera scales it by its depth
    kSpriteDepthTest = 1u << 1,       // hidden behind the model (sprites never write depth)
    kSpriteHidden = 1u << 2,
    kSpriteClipped = 1u << 3, // cut by the section planes
};

// SpriteDesc::texture for an untextured sprite: a quad of the tint colour.
inline constexpr uint32_t kNoTexture = 0xFFFFFFFFu;

// A camera-facing textured quad: three.js' Sprite, with SpriteMaterial's parameters.
struct SpriteDesc {
    uint32_t texture = kNoTexture;          // as returned by add_texture()
    std::array<float, 3> position{0, 0, 0}; // world anchor
    // three.js' sprite.scale: width and height in world units. Without kSpriteSizeAttenuation a perspective
    // camera multiplies them by the anchor's view depth (a constant size on screen: the size at depth 1); an
    // orthographic one leaves them as they are. Exactly three.js' sprite vertex shader.
    std::array<float, 2> scale{1, 1};
    std::array<float, 2> center{0.5f,
                                0.5f}; // three.js' sprite.center: where the anchor is in the quad, (0, 0) bottom-left
    float rotation = 0.0f;             // radians, counter-clockwise on screen (SpriteMaterial.rotation)
    std::array<float, 4> color{1, 1, 1, 1}; // tint multiplied into the texture: linear RGB, alpha = opacity
    uint32_t flags = kSpriteSizeAttenuation | kSpriteDepthTest;
    // Draw order follows three.js: by render_order (ascending), then back to front, then by id.
    float render_order = 0.0f;
};

// A textured quad lying in the world: three.js' Mesh of a PlaneGeometry(1, 1) with a MeshBasicMaterial that
// has a map. Unlit, alpha-blended, double-sided, never written to depth and never returned by pick().
struct QuadDesc {
    uint32_t texture = kNoTexture; // as returned by add_texture(); kNoTexture draws the tint alone
    // Places the unit square (x and y in [-0.5, 0.5], z = 0) in the world; column-major. The texture's top
    // row lies along the square's y = +0.5 edge and its left column along x = -0.5 (an image stands upright
    // seen from +z with y up): for a grid whose first row is at the low y edge, give y a negative scale.
    std::array<float, 16> model{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    std::array<float, 4> color{1, 1, 1, 1}; // tint multiplied into the texture: linear RGB, alpha = opacity
    // InstanceFlags, of which a quad reads: kHidden, kClipped (cut by the section planes), kOnTop (drawn
    // over the model with no depth test; clear, the model hides it where it is in front), kDepthOffset
    // (pulled in front of a face it lies on) and kNearest.
    uint32_t flags = 0;
    // Draw order among quads, as three.js: by render_order (ascending), then back to front (by the square's
    // centre), then by id. Depth-tested quads are drawn after the transparent instances, kOnTop quads after
    // the kOnTop instances; sprites come after both.
    float render_order = 0.0f;
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

// What is under a pixel: the nearest pickable triangle or point there.
struct PickResult {
    bool hit = false;
    uint32_t instance = 0; // instance id, as returned by add_instance()
    uint32_t segment = 0;  // segment index within the instance's geometry
    // Triangles: triangle index in the geometry's original index list (index position / 3). Points: the
    // point's position in the index list (its vertex index when the geometry has no indices).
    uint32_t triangle = 0;
    uint32_t topology = 0; // what was hit, as Topology: 0 a triangle, 2 a point
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
    //
    // Depth is reversed-Z in a 32-bit float buffer (near 1, far 0), for a precision that is near-constant
    // relative to the distance. Section caps also need a stencil: the device should be created with the
    // optional depth32float-stencil8 feature where the adapter has it (WGPUFeatureName_Depth32FloatStencil8,
    // "depth32float-stencil8" in the browser); without it caps fall back to 24-bit depth + stencil.
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
    // The instance's model matrix alone (column-major), everything else as it was last set: what an
    // animation moves each frame.
    void set_instance_model(uint32_t instance, const float model[16]);
    const InstanceDesc &instance_desc(uint32_t instance) const;
    // Two u32 per segment: RGBA8 colour override (alpha 0 = no override) and SegmentFlags.
    void set_segment_states(uint32_t instance, const uint32_t *packed, size_t segment_count);
    void remove_instance(uint32_t instance);
    void clear(); // every geometry, instance, sprite, quad and texture

    // `proj_gl` is an OpenGL-convention projection (clip z in [-1, 1], e.g. three.js); it is remapped to
    // WebGPU's [0, 1] here, reversed (near 1, far 0). Matrices are column-major.
    void set_camera(const float view[16], const float proj_gl[16], bool orthographic);
    // The renderer's own camera (camera_controller.h): the host maps its input to orbit / pan / zoom / fit
    // and the matrices follow. Off by default, when set_camera() is the camera. Switched on, render() and
    // pick() take the view and the projection from the controller instead, for the aspect of the target
    // they are given, with near and far around bounds(); set_camera() is then ignored until it is switched
    // off again. The controller can be driven either way, e.g. framed before it is switched on.
    CameraController &camera() {
        return camera_;
    }
    const CameraController &camera() const {
        return camera_;
    }
    void set_camera_controlled(bool controlled) {
        camera_controlled_ = controlled;
    }
    bool camera_controlled() const {
        return camera_controlled_;
    }
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
    // Section caps (default off): the cut through every kCapped | kClipped, single-sided instance is filled
    // with a lit face on each section plane, as three.js' stencil capping does. The instance must be a closed
    // solid: back faces minus front faces on the kept side of a plane count the solid's interior there.
    void set_section_caps(bool enabled);
    // One colour for every cap (linear RGB); alpha 0 (the default) gives each cap its instance's base colour.
    void set_section_cap_color(float r, float g, float b, float a = 1.0f);

    // Textures for sprites and quads: width x height RGBA8 texels, top row first, sRGB-encoded with straight
    // alpha (canvas pixels). `mipmaps` builds the mip chain a label needs to stay sharp when drawn small; off,
    // the texture is the one level, which is what an image replaced often wants (nothing is filtered on the
    // CPU). update_texture() with the size the texture has writes into it in place -- no allocation, and for
    // a texture without mipmaps a single upload, cheap enough for every frame -- and with another size makes
    // the texture again; the id, and whether it has mipmaps, stay.
    uint32_t add_texture(const uint8_t *rgba, uint32_t width, uint32_t height, bool mipmaps = true);
    void update_texture(uint32_t texture, const uint8_t *rgba, uint32_t width, uint32_t height);
    void remove_texture(uint32_t texture); // sprites and quads still using it are not drawn

    // Sprites are drawn after everything else, alpha-blended and sorted; pick() never returns them.
    uint32_t add_sprite();
    void set_sprite(uint32_t sprite, const SpriteDesc &desc);
    void remove_sprite(uint32_t sprite);

    // Textured quads in the world (QuadDesc): a map, a grid of values or an image laid on a plane.
    uint32_t add_quad();
    void set_quad(uint32_t quad, const QuadDesc &desc);
    void remove_quad(uint32_t quad);

    void render(WGPUTextureView target, uint32_t width, uint32_t height);

    // GPU picking at pixel (x, y) of a width x height target (origin top-left), with the camera of the
    // last set_camera() (or the controller's, see camera()). Renders only that pixel (a pick matrix narrows the
    // frustum, so cluster culling drops nearly everything), reads it back asynchronously and calls `done` -- from the
    // browser's event loop on the web; natively once the device is polled. Points are found as squares of their drawn
    // size, at least kPickPointPixels across, unless a triangle is in front; lines are not pickable.
    static constexpr uint32_t kPickPointPixels = 6;
    void pick(uint32_t x, uint32_t y, uint32_t width, uint32_t height, std::function<void(const PickResult &)> done);
    const RenderStats &stats() const {
        return stats_;
    }
    // The last frame's stats with the GPU culling counts read back (drawn clusters / triangles, occluded
    // clusters). Asynchronous like pick(); with CPU culling it calls back with stats() as they are.
    void read_cull_stats(std::function<void(const RenderStats &)> done);

    // World-space bounds of all visible instances and quads: min xyz, max xyz. Empty scene -> all zeros.
    std::array<float, 6> bounds() const;

private:
    struct Impl;
    void apply_controlled_camera(uint32_t width, uint32_t height);

    std::unique_ptr<Impl> impl_;
    RenderStats stats_;
    CameraController camera_;
    bool camera_controlled_ = false;
};

} // namespace adacpp::render
