#ifndef TINY_HELPERS_H
#define TINY_HELPERS_H

#include "../../geom/Mesh.h"

void AddMesh(tinygltf::Model &model, const std::string &name, const Mesh& my_mesh);

#endif //TINY_HELPERS_H
