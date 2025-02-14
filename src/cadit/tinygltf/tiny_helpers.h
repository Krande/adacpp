#ifndef TINY_HELPERS_H
#define TINY_HELPERS_H

#include "../../geom/Mesh.h"
#include "tiny_gltf.h"

void AddMesh(tinygltf::Model& model, const std::string& name, const Mesh& my_mesh);

int write_boxes_to_gltf(const std::string& filename, const std::vector<std::vector<float>>& box_origins,
                        const std::vector<std::vector<float>>& box_dims);

int write_to_gltf(const std::string& filename, const Mesh& mesh);

#endif //TINY_HELPERS_H
