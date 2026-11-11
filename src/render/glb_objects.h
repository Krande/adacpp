// The model's objects while the GLB is read (internal): GlbLoadResult::objects in the making, and the
// segments adapy's draw ranges cut a primitive into.
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "glb_gltf.h"
#include "glb_scene.h"

namespace adacpp::render::detail {

// The model's objects while the scene is read (GlbLoadResult::objects): filled in the order the GLB gives
// them, then put parents first by finish().
class ObjectTable {
public:
    // adapy's "id_hierarchy": {id: [name, parent id]}, the parent of a root being "*".
    void add_hierarchy(const tinygltf::Value &hierarchy);
    // The object with this id; one the hierarchy does not list becomes a root named by its id.
    int32_t by_id(const std::string &id);
    // The object of a glTF node, under its parent node's.
    int32_t for_node(const tinygltf::Model &model, int node_index, int32_t parent);
    void add_segment(int32_t object, uint32_t instance, uint32_t segment, const float bounds[6], uint32_t triangles);
    // Moves the objects into `res` parents first, and points every geometry's segments at their new places.
    void finish(GlbLoadResult &res);

private:
    static std::string text(const tinygltf::Value &v);
    // Numbers by value ("9" before "10"), anything else as text after them.
    static bool id_less(const std::string &a, const std::string &b);

    std::vector<GlbObject> objects_;
    std::unordered_map<std::string, int32_t> index_;
    bool sort_by_id_ = false;
};

struct RangeSegment {
    uint32_t start, count;
    int32_t object; // ObjectTable index; -1 = a gap no range covers
};

// adapy writes each merged node's per-object index ranges into the scene extras as
// "draw_ranges_<node name>": {object id: [start, count]}. They become the geometry's segments, as in the
// viewer, so clusters and culling here behave as they do in the browser.
std::vector<RangeSegment> segments_from_extras(const tinygltf::Value *ranges, size_t index_count, ObjectTable &objects);

} // namespace adacpp::render::detail
