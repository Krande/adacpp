// GLB -> Renderer scene loading (tinygltf), and a camera that frames it the way the viewer does. The one
// GLB reader for every host: the Python offscreen renderer (posters) and the browser (wasm_bindings), so
// a model draws the same in both.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
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
    // The glTF node whose transform places the geometry, when it was kept apart to be animated
    // (GlbOptions::animations): its vertices are then in that node's space and its instance's model matrix
    // is the node's world transform, which GlbAnimations::apply moves. -1: merged in world space, the
    // instance's matrix the identity.
    int32_t node = -1;
    // World-space positions (3 per vertex; the node's space for an animated geometry), in the geometry's
    // vertex order, and its index buffer (into them), when GlbOptions::keep_positions.
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    // Per segment of the geometry (what PickResult::segment indexes): the GlbLoadResult::objects entry it
    // draws, or -1 for a gap between draw ranges and for a primitive that is no object. Each segment of a
    // feature-edge overlay outlines one segment of the faces, and names its object (not in
    // GlbObject::segments, which are the faces a pick reports).
    std::vector<int32_t> segment_objects;
};

// One model object of the GLB: something a user selects, hides or reads the name of.
struct GlbObject {
    std::string id;      // as the GLB names it: see GlbLoadResult::objects
    std::string name;    // for display; not unique
    int32_t parent = -1; // index into GlbLoadResult::objects; -1 = a root
    // World-space bounds (min xyz, max xyz) and triangle count of the object's own segments and of every
    // object below it, so a grouping object has the union of its children. Inverted (min > max) when
    // nothing under it was drawn.
    std::array<float, 6> bounds{3.4e38f, 3.4e38f, 3.4e38f, -3.4e38f, -3.4e38f, -3.4e38f};
    uint32_t triangles = 0;
    // The object's own segments as (instance id, segment index), the pair a pick reports. Empty for an
    // object that only groups others.
    std::vector<std::array<uint32_t, 2>> segments;
};

// The animations of a GLB loaded with GlbOptions::animations, and the instances they move. Sampled here, as
// three.js' AnimationMixer samples a clip: translation, rotation and scale channels (morph weights are not
// read) with LINEAR (rotations by spherical interpolation), STEP and CUBICSPLINE interpolation; a node under
// an animated node follows it. Empty (count() 0) for a GLB without animations and when the option is off.
// A copy shares the data. The instances are the renderer's: once it is cleared, or they are removed, drop
// this too.
class GlbAnimations {
public:
    struct Data;
    GlbAnimations() = default;
    explicit GlbAnimations(std::shared_ptr<Data> data) : data_(std::move(data)) {}

    size_t count() const;
    const std::string &name(size_t animation) const; // as the GLB names it; may be empty
    // Seconds: the time of its last keyframe.
    float duration(size_t animation) const;
    // Puts every animated instance where `animation` has it at `time` seconds: before a channel's first
    // keyframe its first value holds, after its last the last (the caller wraps the time to loop). Nodes
    // the animation does not move are at rest; `animation` -1 puts everything back in the rest pose (the
    // node transforms the GLB holds, which is how the model is loaded). One animation at a time: they are
    // not blended.
    void apply(Renderer &renderer, int animation, float time) const;
    // The glTF nodes kept apart (each animated node and every node below one), parents first, and the
    // world matrix (column-major) apply() last gave node i of them -- the rest pose before any apply --
    // for a host that hangs something of its own on a moving node.
    const std::vector<int32_t> &nodes() const;
    std::array<float, 16> node_matrix(size_t i) const;

private:
    std::shared_ptr<Data> data_;
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
    // The model's objects, parents before their children (depth first, so a subtree is one contiguous run).
    //  - A GLB with adapy's scene extras ("id_hierarchy": {id: [name, parent id]} and/or
    //    "draw_ranges_<node>": {id: [start, count]}): one object per id of the hierarchy, plus any id only
    //    the draw ranges name (a root, named by its id). `id` is that id. Siblings are in id order. Only
    //    draw ranges make segments objects: a primitive without them (edge lines, points) is none.
    //  - Any other GLB (`node_objects`): one object per node of the scene, `id` its node index in decimal,
    //    named as the node (else its mesh, else empty), under its parent node. Every primitive of a node's
    //    mesh is a segment of it.
    std::vector<GlbObject> objects;
    bool node_objects = false;
    // The default scene's "extras" and the root "ADA_EXT_data" extension, as the bytes of JSON text the GLB
    // holds them in (empty when absent), for a host that reads design or FEA metadata from them.
    std::string extras;
    std::string ada_ext_data;
    // With GlbOptions::animations: the GLB's animations (see GlbAnimations). `bounds` and the objects'
    // bounds then cover the motion: the rest pose and every animation sampled at 33 evenly spaced times,
    // each animated segment's box carried as a box (so a little generous, and a fast swing between two
    // samples can reach past it).
    GlbAnimations animations;
};

// Adds every mesh primitive of the GLB's default scene as one geometry + instance, with node transforms
// applied. Reads EXT_/KHR_meshopt_compression and, where the build has Draco (ADACPP_RENDER_DRACO, on by
// default), KHR_draco_mesh_compression. adapy writes its z-up coordinates into the GLB as they are (the viewer only
// turns the camera's up axis), so nothing is rotated by default; `y_up_to_z_up` rotates a conventional y-up glTF
// instead.
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
    // Mark every single-sided surface as a closed solid (kCapped), so its cut by the section planes is
    // filled once Renderer::set_section_caps is on. Surfaces drawn double-sided (by their material or by
    // `double_sided`) are thin shells and are left hollow.
    bool section_caps = false;
    // Above 0: also draw the surfaces' feature edges as Ada Studio's viewer does with tessellation edges
    // hidden (its default) -- vertices welded by position, then every boundary edge and every edge whose
    // two faces meet at more than this angle (degrees; the viewer's is 30), in `line_color` at
    // `line_width`. Worked out per segment, as the viewer does per draw range, so an object's edges can be
    // hidden with it. 0 draws only the lines the GLB carries.
    float feature_edge_angle_deg = 0.0f;
    // Keep each added geometry's world-space positions and indices in GlbLoadResult::loaded (for a caller
    // that builds more geometry on the same vertices, e.g. an FEA mesh's element edges).
    bool keep_positions = false;
    // Keep what the GLB's animations move apart, so it can be moved: every node an animation channel
    // targets, and every node below one, gets its own geometries (batched per node, in the node's space)
    // and instances, placed by their model matrix, and GlbLoadResult::animations plays them. The rest of
    // the model is merged as always. Off (the default), and for a GLB without animations, everything is
    // merged in its rest pose. Objects, segments and picking are the same either way: an animated
    // instance's segments name objects as any other's do. A model of many small animated nodes becomes as
    // many draws.
    bool animations = false;
    // Added to the world position of everything loaded (after `y_up_to_z_up`, so in the renderer's z-up
    // world), in double before any vertex or matrix becomes a float: a model far from the origin, offset by
    // minus a point on it, arrives near the origin with the precision float coordinates out there would
    // lose. Every position, matrix and bounds the load reports includes it.
    std::array<double, 3> offset{0, 0, 0};
};
GlbLoadResult load_glb(Renderer &renderer, const std::string &path, const GlbOptions &options = {});
// The same from the GLB's bytes (the browser has no file to name). `name` labels errors.
GlbLoadResult load_glb_bytes(Renderer &renderer, std::vector<unsigned char> bytes, const GlbOptions &options = {},
                             const std::string &name = "GLB");
// The world box (min xyz, max xyz) of what load_glb_bytes would draw with GlbOptions::offset 0, in double and
// without loading anything: each primitive's POSITION min / max (or its data where they are absent) through
// its node's world matrix, in the rest pose. Inverted (min > max) when nothing is drawable.
std::array<double, 6> glb_bounds_bytes(const std::vector<unsigned char> &bytes, bool y_up_to_z_up = false);

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
