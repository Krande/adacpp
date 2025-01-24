//
// Created by ofskrand on 24.01.2025.
//
#include <iostream>

#include "../../src/geom/Mesh.h"
#include "../../src/visit/tess_helpers.h"
#include "../../src/helpers/helpers.h"
#include "../../src/cadit/tinygltf/tiny_helpers.h"
#include <tiny_gltf.h>

int main(int argc, char *argv[]) {
    std::vector<std::vector<float>> box_origins = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},
        {2.0, 2.0, 2.0}
    };

    std::vector<std::vector<float>> box_dims = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0}
    };
    std::string filename = "output.glb";

    tinygltf::Model model;
    std::cout << "Exporting to " << filename <<"\n";

    // Create a scene
    tinygltf::Scene scene;
    model.scenes.push_back(scene);
    model.defaultScene = 0;

    for (int i = 0; i < box_origins.size(); i++) {
        const auto& origin = box_origins[i];
        const auto& dim = box_dims[i];
        TopoDS_Solid box = create_box(origin, dim);
        Mesh mesh = tessellate_shape(0, box, true, 1.0, false);
        mesh.color = random_color();

        std::cout << "Adding mesh" << i << "\n";
        AddMesh(model, "mesh", mesh);
    }
    // If filename contains .glb set variable "glb" to true
    bool glb = filename.find(".glb") != std::string::npos;

    // Save to file
    tinygltf::TinyGLTF gltf;
    if (!gltf.WriteGltfSceneToFile(&model, filename, true, true, true, glb)) {
        std::cerr << "Failed to write glTF file" << std::endl;
        return -1;
    }

    return 0;
}