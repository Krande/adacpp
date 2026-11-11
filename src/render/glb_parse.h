// Reading the GLB (internal): what tinygltf leaves to the caller -- accessors as floats and indices, Draco
// primitives, meshopt-compressed views, and the JSON a host wants passed through as written.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "glb_gltf.h"
#include "glb_scene.h"

namespace adacpp::render::detail {

// Read an accessor as floats (`comps` per element), honouring stride, component type and normalisation.
std::vector<float> read_floats(const tinygltf::Model &model, int accessor_index, int comps);
std::vector<uint32_t> read_indices(const tinygltf::Model &model, int accessor_index);
// A primitive compressed with KHR_draco_mesh_compression: its vertices and triangles from the Draco data
// (the accessors of such a primitive only describe them). False for a primitive without the extension.
bool read_draco(const tinygltf::Model &model, const tinygltf::Primitive &prim, std::vector<float> &positions,
                std::vector<float> &normals, std::vector<float> &colors, std::vector<uint32_t> &indices);
// GlbLoadResult::extras and ::ada_ext_data, from the GLB as it was handed in.
void read_passthrough(const std::vector<unsigned char> &glb, GlbLoadResult &res);
// The GLB with every EXT_/KHR_meshopt_compression view decoded, so tinygltf can read it; a GLB without
// the extension comes back unchanged.
std::vector<unsigned char> decode_meshopt_glb(std::vector<unsigned char> glb);
// The GLB's document, meshopt views decoded, images skipped. Throws, naming the GLB `name`, when it cannot
// be read or needs Draco and the build has none.
tinygltf::Model parse_glb(std::vector<unsigned char> bytes, const std::string &name);

} // namespace adacpp::render::detail
